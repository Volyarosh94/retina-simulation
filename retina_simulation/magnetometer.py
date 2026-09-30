"""Simulated magnetometer nodes: an RM3100 beside some of the radar receivers.

Each magnetometer sits at a radar node's receive site and watches the same
simulated world the radar nodes do. What it reads, per axis, in nanotesla,
north/east/down:

    B = main field     WMM2025 at the site (~48,600 nT near Greenville, SC)
      + crust          a fixed offset per site (tens to ~100 nT)
      + daily (Sq)     the solar-quiet variation, the same shape at every site
      + storm          when switched on, common to the whole network
      + UAPs           every anomalous object as a 1e9 A·m² dipole, 1/r³
      + noise          the RM3100's datasheet noise for its cycle count,
                       then quantised to the chip's counts

Aircraft and drones contribute nothing: their moments are too small to matter
at these distances, which is the task's model. A "UAP" is an object the world
spawned as ``object_type == "anomalous"``; an airliner that turns anomalous
mid-flight (hijack, spoof...) is still an airliner.

Nothing here draws from the world's shared ``random`` module. Placement,
crust, dipole directions and noise are pure functions of the fleet seed, so
adding magnetometers leaves the radar scene of a given seed exactly as it was.

The simulator only produces readings (and, for scoring, the truth behind
them). Detection runs on the server, over the whole network at once: the
daily variation and storms are the same everywhere, and only a network can
subtract them before looking for something local. See the retina-server
design note on simulated magnetometers.
"""

from __future__ import annotations

import hashlib
import math
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from functools import lru_cache

Vec = tuple[float, float, float]

# μ0/4π in T·m/A, and nT per tesla.
MU0_OVER_4PI = 1e-7
TESLA_TO_NT = 1e9
# The task's UAP model: a magnetic dipole of this moment.
UAP_MOMENT_AM2 = 1e9
# Beyond this slant range a 1e9 A·m² dipole is under 0.002 nT on its axis:
# nothing a 13 nT-per-count sensor could register.
UAP_CULL_KM = 50.0
# The RM3100's linear range.
SENSOR_RANGE_NT = 800_000.0

R_EARTH_KM = 6371.0

PLACEMENTS = ("spread", "core", "random")


# ── RM3100 characteristics (PNI UM16 Table 3-1) ──────────────────────────────
# The same figures the retina-magnetometer node app uses: noise 30/20/15 nT at
# 50/100/200 cycles, interpolated log-log and extended by the nearest segment's
# slope (11.3 nT at 400, 8.4 at 800); gain from PNI's formula.

_NOISE_POINTS = ((50, 30.0), (100, 20.0), (200, 15.0))


def rm3100_noise_nt(cycle_count: int) -> float:
    if cycle_count <= _NOISE_POINTS[1][0]:
        (c0, n0), (c1, n1) = _NOISE_POINTS[0], _NOISE_POINTS[1]
    else:
        (c0, n0), (c1, n1) = _NOISE_POINTS[1], _NOISE_POINTS[2]
    slope = math.log(n1 / n0) / math.log(c1 / c0)
    return n0 * (cycle_count / c0) ** slope


def rm3100_gain_counts_per_ut(cycle_count: int) -> float:
    return 0.3671 * cycle_count + 1.5


# ── Deterministic randomness ─────────────────────────────────────────────────


def _digest(seed: int, stream: str, index) -> bytes:
    return hashlib.blake2b(f"{seed}|{stream}|{index}".encode(), digest_size=16).digest()


def uniform(seed: int, stream: str, index=0) -> float:
    return int.from_bytes(_digest(seed, stream, index)[:8], "big") / 2**64


def gauss(seed: int, stream: str, index=0) -> float:
    d = _digest(seed, stream, index)
    u1 = (int.from_bytes(d[:8], "big") + 1) / (2**64 + 1)
    u2 = int.from_bytes(d[8:], "big") / 2**64
    return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


def unit_vector(seed: int, stream: str, index=0) -> Vec:
    z = 2.0 * uniform(seed, stream + ":z", index) - 1.0
    phi = 2.0 * math.pi * uniform(seed, stream + ":phi", index)
    r = math.sqrt(max(0.0, 1.0 - z * z))
    return (r * math.cos(phi), r * math.sin(phi), z)


# ── Physics ──────────────────────────────────────────────────────────────────


def dipole_field_nt(moment_am2: Vec, displacement_m: Vec) -> Vec:
    """B = (μ0/4π)[3(m·r̂)r̂ − m]/r³ at ``displacement_m`` from the dipole, in nT.

    200 nT at 1 km on the axis of a 1e9 A·m² dipole, 1.6 nT at 5 km, half that
    on its equator. Floored at 1 m, where a point dipole means nothing anyway.
    """
    rx, ry, rz = displacement_m
    r2 = max(rx * rx + ry * ry + rz * rz, 1.0)
    r = math.sqrt(r2)
    m_dot_r = moment_am2[0] * rx + moment_am2[1] * ry + moment_am2[2] * rz
    k = MU0_OVER_4PI * TESLA_TO_NT / (r2 * r2 * r)
    return (
        k * (3.0 * m_dot_r * rx - moment_am2[0] * r2),
        k * (3.0 * m_dot_r * ry - moment_am2[1] * r2),
        k * (3.0 * m_dot_r * rz - moment_am2[2] * r2),
    )


@lru_cache(maxsize=1)
def _wmm():
    # pygeomag ships NOAA's WMM_2025.COF unchanged and reproduces the 100
    # official WMM2025 test values; zero dependencies.
    from pygeomag import GeoMag

    return GeoMag(coefficients_file="wmm/WMM_2025.COF")


def _decimal_year(t: float) -> float:
    moment = datetime.fromtimestamp(t, tz=timezone.utc)
    start = datetime(moment.year, 1, 1, tzinfo=timezone.utc).timestamp()
    end = datetime(moment.year + 1, 1, 1, tzinfo=timezone.utc).timestamp()
    return moment.year + (t - start) / (end - start)


def main_field_nt(lat: float, lon: float, alt_m: float, t: float) -> Vec:
    """WMM2025 at a site: (north, east, down) in nT. Valid 2025.0–2030.0."""
    r = _wmm().calculate(
        glat=lat, glon=lon, alt=alt_m / 1000.0, time=_decimal_year(t), allow_date_outside_lifespan=True
    )
    return (r.x, r.y, r.z)


# Solar-quiet variation: a0 plus 24/12/8/6 h harmonics as (amplitude nT, local
# time of that harmonic's maximum), fitted to quiet days at FRD and BSL, the
# INTERMAGNET observatories either side of Greenville (315 station-days, the
# GFZ international quiet days, 97.6–99.9 % of variance explained). Seasons:
# J = May–Aug, E = equinox months, D = Nov–Feb; "min" is 2019 (mean F10.7
# 69.7), "max" 2024 (191.2). Identical to the retina-magnetometer simulator.
_SQ_F107 = (69.7, 191.2)
_SQ = {
    ("min", "J"): {
        "x": (-4.0, ((7.0, 22.06), (5.2, 3.83), (3.4, 5.74), (1.5, 0.51))),
        "y": (0.0, ((15.4, 4.44), (15.0, 7.35), (7.2, 0.34), (2.0, 2.22))),
        "z": (-1.9, ((5.3, 22.71), (5.9, 4.93), (3.3, 6.83), (0.9, 1.06))),
    },
    ("min", "E"): {
        "x": (-3.0, ((6.6, 23.31), (5.2, 5.11), (1.5, 6.92), (0.2, 1.33))),
        "y": (0.3, ((9.7, 4.89), (9.7, 7.92), (5.0, 1.01), (1.7, 3.49))),
        "z": (-1.9, ((4.3, 23.43), (4.3, 5.37), (2.5, 7.53), (0.7, 2.52))),
    },
    ("min", "D"): {
        "x": (0.7, ((2.2, 0.28), (4.0, 5.56), (1.7, 7.52), (0.8, 2.73))),
        "y": (-0.5, ((4.4, 3.89), (6.6, 8.12), (3.7, 1.23), (2.1, 3.70))),
        "z": (-1.2, ((2.9, 23.21), (2.8, 5.27), (1.8, 7.44), (0.9, 2.59))),
    },
    ("max", "J"): {
        "x": (-3.2, ((7.0, 21.12), (7.4, 3.52), (4.5, 5.91), (1.2, 5.89))),
        "y": (-0.9, ((23.6, 4.74), (19.7, 7.65), (7.7, 0.46), (2.0, 1.64))),
        "z": (-2.8, ((6.8, 23.23), (6.8, 5.21), (3.2, 6.96), (1.0, 0.46))),
    },
    ("max", "E"): {
        "x": (-5.8, ((12.0, 22.78), (8.9, 4.68), (3.5, 6.51), (0.2, 1.04))),
        "y": (-1.0, ((20.9, 5.05), (17.3, 8.09), (8.5, 1.03), (2.4, 3.30))),
        "z": (-4.1, ((7.6, 23.60), (6.2, 5.33), (3.4, 7.40), (0.9, 2.18))),
    },
    ("max", "D"): {
        "x": (-4.2, ((10.3, 0.96), (8.8, 6.13), (4.6, 7.78), (2.0, 2.76))),
        "y": (-0.9, ((11.4, 5.60), (12.5, 8.91), (6.6, 1.74), (3.4, 4.01))),
        "z": (-2.1, ((5.1, 23.44), (4.8, 5.59), (2.8, 7.71), (1.4, 2.74))),
    },
}


def _phasors(entry):
    a0, harmonics = entry
    return a0, [
        a * complex(math.cos(2 * math.pi * m * t / 24.0), -math.sin(2 * math.pi * m * t / 24.0))
        for m, (a, t) in enumerate(harmonics, start=1)
    ]


def _blend(p, q, w):
    return p[0] + (q[0] - p[0]) * w, [a + (b - a) * w for a, b in zip(p[1], q[1])]


def sq_variation_nt(
    t: float, longitude: float, *, f107: float = 120.0, seed: int = 0, variability: float = 0.25
) -> Vec:
    """The quiet-day daily variation at a longitude, (north, east, down) nT.

    Near zero at night, X lowest and Y swinging from positive to negative
    around local noon, tens of nT peak to peak. Each local day has its own
    amplitude factor in [1 − v, 1 + v] from the seed, blended between local
    noons so there is never a step.
    """
    moment = datetime.fromtimestamp(t, tz=timezone.utc)
    doy = moment.timetuple().tm_yday - 1 + (moment.hour * 3600 + moment.minute * 60 + moment.second) / 86400.0
    w_solar = min(1.0, max(0.0, (f107 - _SQ_F107[0]) / (_SQ_F107[1] - _SQ_F107[0])))
    s = math.cos(2 * math.pi * (doy - 182.5) / 365.25)
    season, w_season = ("J", s) if s >= 0 else ("D", -s)
    local_hours = t / 3600.0 + longitude / 15.0
    lt = local_hours % 24.0
    position = (local_hours - 12.0) / 24.0
    day = math.floor(position)
    ease = 0.5 - 0.5 * math.cos(math.pi * (position - day))
    f0 = 1.0 + variability * (2.0 * uniform(seed, "sq-amplitude", day) - 1.0)
    f1 = 1.0 + variability * (2.0 * uniform(seed, "sq-amplitude", day + 1) - 1.0)
    amplitude = f0 + (f1 - f0) * ease
    out = []
    for comp in ("x", "y", "z"):
        by_solar = [
            _blend(_phasors(_SQ[(sol, "E")][comp]), _phasors(_SQ[(sol, season)][comp]), w_season)
            for sol in ("min", "max")
        ]
        a0, phasors = _blend(by_solar[0], by_solar[1], w_solar)
        value = a0
        for m, c in enumerate(phasors, start=1):
            angle = 2 * math.pi * m * lt / 24.0
            value += (c * complex(math.cos(angle), math.sin(angle))).real
        out.append(amplitude * value)
    return (out[0], out[1], out[2])


@dataclass(frozen=True)
class Storm:
    """A coarse geomagnetic storm, common to the whole network: a sudden
    commencement, a main phase pulling X towards 0.72·Dst over six hours,
    exponential recovery, and Pc4–Pc5 pulsations (45–600 s). Here to show
    that network-wide disturbances do not look like UAPs to a network."""

    start: float
    dst_min_nt: float = -150.0
    main_phase_h: float = 6.0
    recovery_h: float = 12.0
    commencement_nt: float = 25.0
    pulsation_nt: float = 6.0
    seed: int = 0

    def __call__(self, t: float) -> Vec:
        age = t - self.start
        if age < 0:
            return (0.0, 0.0, 0.0)
        main = self.main_phase_h * 3600.0
        envelope = (1.0 - math.exp(-age / (main / 3.0))) * math.exp(-max(0.0, age - main) / (self.recovery_h * 3600.0))
        dst = self.dst_min_nt * envelope
        x = 0.72 * dst + self.commencement_nt * (1.0 - math.exp(-age / 60.0)) * math.exp(-age / 5400.0)
        y, z = 0.08 * dst, -0.15 * dst
        for k in range(6):
            period = 45.0 + 555.0 * uniform(self.seed, "storm-period", k)
            amp = self.pulsation_nt * (0.3 + 0.7 * uniform(self.seed, "storm-amplitude", k)) * envelope
            value = amp * math.sin(2 * math.pi * age / period + 2 * math.pi * uniform(self.seed, "storm-phase", k))
            d = unit_vector(self.seed, "storm-direction", k)
            x, y, z = x + value * d[0], y + value * d[1], z + value * d[2]
        return (x, y, z)


# ── Magnetometer nodes ───────────────────────────────────────────────────────


@dataclass
class MagnetometerNode:
    node_id: str
    host_node_id: str  # the radar node it sits beside
    lat: float
    lon: float
    alt_m: float
    crust_nt: Vec  # static local anomaly, per site
    background_nt: Vec = (0.0, 0.0, 0.0)  # WMM2025 + crust, refreshed daily
    background_day: int = -1


def _haversine_km(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    return R_EARTH_KM * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def radar_sites(node_configs: list[dict]) -> list[dict]:
    """One entry per receive site: nodes configured at the same coordinates
    share a roof, and a roof gets one magnetometer. The host is the site's
    lowest node id, so the choice is stable across restarts."""
    sites: dict[tuple, dict] = {}
    for cfg in node_configs:
        try:
            key = (round(float(cfg["rx_lat"]), 5), round(float(cfg["rx_lon"]), 5))
        except (KeyError, TypeError, ValueError):
            continue
        site = sites.get(key)
        if site is None or str(cfg["node_id"]) < site["node_id"]:
            sites[key] = {
                "node_id": str(cfg["node_id"]),
                "lat": float(cfg["rx_lat"]),
                "lon": float(cfg["rx_lon"]),
                "alt_m": float(cfg.get("rx_alt_ft", 0.0)) * 0.3048,
            }
    return sorted(sites.values(), key=lambda s: s["node_id"])


def choose_sites(
    sites: list[dict], count: int, placement: str, *, center: tuple[float, float], seed: int
) -> list[dict]:
    """Which ``count`` sites get a magnetometer.

    - ``spread``: farthest-point sampling from the site nearest the centre, so
      the magnetometers cover the fleet's area as evenly as the sites allow.
    - ``core``: the sites nearest the centre, a dense cluster where passes are
      likely to be seen by more than one magnetometer.
    - ``random``: a seeded shuffle.
    """
    count = max(0, min(int(count), len(sites)))
    if count == 0:
        return []
    by_centre = sorted(sites, key=lambda s: (_haversine_km(center[0], center[1], s["lat"], s["lon"]), s["node_id"]))
    if placement == "core":
        return by_centre[:count]
    if placement == "random":
        return sorted(sites, key=lambda s: uniform(seed, "mag-placement", s["node_id"]))[:count]
    chosen = [by_centre[0]]
    remaining = [s for s in sites if s is not by_centre[0]]
    while len(chosen) < count and remaining:
        best = max(
            remaining,
            key=lambda s: (min(_haversine_km(s["lat"], s["lon"], c["lat"], c["lon"]) for c in chosen), s["node_id"]),
        )
        chosen.append(best)
        remaining.remove(best)
    return chosen


@dataclass
class MagnetometerSettings:
    """What the runtime config controls on the simulator side. (The detection
    threshold is the server's: detection runs there.)"""

    enabled: bool = True
    n_nodes: int = 8
    placement: str = "spread"
    cycle_count: int = 200
    storm: bool = False

    @classmethod
    def from_config(cls, cfg: dict, current: MagnetometerSettings | None = None) -> MagnetometerSettings:
        base = current or cls()
        enabled = cfg.get("mag_enabled", base.enabled)
        n_nodes = cfg.get("mag_n_nodes", base.n_nodes)
        placement = cfg.get("mag_placement", base.placement)
        cycle_count = cfg.get("mag_cycle_count", base.cycle_count)
        storm = cfg.get("mag_storm", base.storm)
        return cls(
            enabled=bool(enabled),
            n_nodes=int(n_nodes) if isinstance(n_nodes, int) and not isinstance(n_nodes, bool) else base.n_nodes,
            placement=placement if placement in PLACEMENTS else base.placement,
            cycle_count=int(cycle_count)
            if isinstance(cycle_count, int) and 30 <= cycle_count <= 1000
            else base.cycle_count,
            storm=bool(storm),
        )


@dataclass
class MagnetometerArray:
    """The fleet's magnetometers: placement, and one reading per node per tick."""

    node_configs: list[dict]
    center: tuple[float, float]
    seed: int = 0
    settings: MagnetometerSettings = field(default_factory=MagnetometerSettings)
    moment_am2: float = UAP_MOMENT_AM2
    f107: float = 120.0
    nodes: list[MagnetometerNode] = field(default_factory=list)
    storm: Storm | None = None
    _tick: int = 0

    def __post_init__(self) -> None:
        self.place()

    # ── Configuration ────────────────────────────────────────────────────────

    def place(self) -> None:
        if not self.settings.enabled:
            self.nodes = []
            return
        sites = choose_sites(
            radar_sites(self.node_configs),
            self.settings.n_nodes,
            self.settings.placement,
            center=self.center,
            seed=self.seed,
        )
        self.nodes = [
            MagnetometerNode(
                node_id=f"mag-{site['node_id']}",
                host_node_id=site["node_id"],
                lat=site["lat"],
                lon=site["lon"],
                alt_m=site["alt_m"],
                # The WMM's long-wavelength field misses the local crust,
                # which WMMHR alone shifts ~50-100 nT per component here; an
                # offset per site at about that size keeps the nodes' static
                # levels as different as real ones would be.
                crust_nt=(
                    60.0 * gauss(self.seed, "mag-crust-x", site["node_id"]),
                    60.0 * gauss(self.seed, "mag-crust-y", site["node_id"]),
                    60.0 * gauss(self.seed, "mag-crust-z", site["node_id"]),
                ),
            )
            for site in sites
        ]

    def apply(self, settings: MagnetometerSettings, now: float) -> bool:
        """Adopt new settings; returns True if the set of nodes changed."""
        replace = (settings.enabled, settings.n_nodes, settings.placement) != (
            self.settings.enabled,
            self.settings.n_nodes,
            self.settings.placement,
        )
        if settings.storm and self.storm is None:
            self.storm = Storm(start=now, seed=self.seed + 7919)
        elif not settings.storm:
            self.storm = None
        self.settings = settings
        if replace:
            self.place()
        return replace

    # ── Sampling ─────────────────────────────────────────────────────────────

    def _background(self, node: MagnetometerNode, t: float) -> Vec:
        day = int(t // 86400)
        if node.background_day != day:
            main = main_field_nt(node.lat, node.lon, node.alt_m, t)
            node.background_nt = (main[0] + node.crust_nt[0], main[1] + node.crust_nt[1], main[2] + node.crust_nt[2])
            node.background_day = day
        return node.background_nt

    def uap_moment(self, object_id: str) -> Vec:
        """Each UAP keeps one dipole direction for its whole life, drawn from
        the seed and its id: fixed in the local frame, not turning with it."""
        u = unit_vector(self.seed, "uap-moment", object_id)
        return (u[0] * self.moment_am2, u[1] * self.moment_am2, u[2] * self.moment_am2)

    def sample(self, anomalous: list, t: float) -> list[dict]:
        """One reading per node at epoch ``t`` from the world's anomalous
        objects (anything with lat, lon, alt_km and object_id).

        Returns, per node: the reading in nT (x north, y east, z down) as the
        chip would report it, and the truth behind it: the UAP field's
        magnitude and the object nearest the node, with its slant range.
        """
        self._tick += 1
        sigma = rm3100_noise_nt(self.settings.cycle_count)
        gain = rm3100_gain_counts_per_ut(self.settings.cycle_count)
        storm = self.storm(t) if self.storm is not None else (0.0, 0.0, 0.0)
        sq_centre = sq_variation_nt(t, self.center[1], f107=self.f107, seed=self.seed)
        out = []
        for node in self.nodes:
            base = self._background(node, t)
            # The daily variation depends on local time, which across a 100 km
            # fleet differs by minutes: worth recomputing only for a node well
            # east or west of the centre.
            if abs(node.lon - self.center[1]) > 0.25:
                sq = sq_variation_nt(t, node.lon, f107=self.f107, seed=self.seed)
            else:
                sq = sq_centre
            local = (sq[0] + storm[0], sq[1] + storm[1], sq[2] + storm[2])
            uap = [0.0, 0.0, 0.0]
            nearest_id, nearest_km = None, math.inf
            for obj in anomalous:
                north = math.radians(obj.lat - node.lat) * R_EARTH_KM
                east = math.radians(obj.lon - node.lon) * R_EARTH_KM * math.cos(math.radians(node.lat))
                down = node.alt_m / 1000.0 - obj.alt_km
                slant_km = math.sqrt(north * north + east * east + down * down)
                if slant_km < nearest_km:
                    nearest_id, nearest_km = obj.object_id, slant_km
                if slant_km > UAP_CULL_KM:
                    continue
                # Displacement from the dipole to the sensor, in metres.
                b = dipole_field_nt(self.uap_moment(obj.object_id), (-north * 1000.0, -east * 1000.0, -down * 1000.0))
                uap[0] += b[0]
                uap[1] += b[1]
                uap[2] += b[2]
            reading = []
            for i, axis in enumerate(("x", "y", "z")):
                value = (
                    base[i]
                    + local[i]
                    + uap[i]
                    + sigma * gauss(self.seed, f"mag-noise-{axis}-{node.node_id}", self._tick)
                )
                value = max(-SENSOR_RANGE_NT, min(SENSOR_RANGE_NT, value))
                counts = round(value * gain / 1000.0)
                reading.append(counts * 1000.0 / gain)
            uap_nt = math.sqrt(uap[0] ** 2 + uap[1] ** 2 + uap[2] ** 2)
            out.append(
                {
                    "node": node,
                    "reading": (reading[0], reading[1], reading[2]),
                    "uap_nt": uap_nt,
                    "nearest_uap": nearest_id,
                    "nearest_uap_km": None if nearest_id is None else nearest_km,
                }
            )
        return out


def build_magnetometer_push_payload(array: MagnetometerArray, batches: list[tuple[int, list[dict]]]) -> dict:
    """The body of POST /api/sim/magnetometer/push.

    ``batches`` is [(t_ms, sample(...) output), ...], oldest first: normally
    one tick, more if a push failed and the samples were held for the next.
    Nodes are listed even with no samples in this push, so the server knows
    the current set; truth rides beside the readings, marked as such, for
    scoring detections, and never feeds detection itself.
    """
    per_node: dict[str, dict] = {}
    for node in array.nodes:
        per_node[node.node_id] = {
            "node_id": node.node_id,
            "host_node_id": node.host_node_id,
            "lat": round(node.lat, 6),
            "lon": round(node.lon, 6),
            "alt_m": round(node.alt_m, 1),
            "samples": [],
            "truth": [],
        }
    for t_ms, readings in batches:
        for r in readings:
            entry = per_node.get(r["node"].node_id)
            if entry is None:  # re-placed since this tick was sampled
                continue
            x, y, z = r["reading"]
            entry["samples"].append([t_ms, round(x, 2), round(y, 2), round(z, 2)])
            entry["truth"].append(
                [
                    t_ms,
                    round(r["uap_nt"], 3),
                    r["nearest_uap"],
                    None if r["nearest_uap_km"] is None else round(r["nearest_uap_km"], 3),
                ]
            )
    return {
        "ts_ms": int(time.time() * 1000),
        "sensor": {
            "model": "RM3100",
            "cycle_count": array.settings.cycle_count,
            "noise_nt": round(rm3100_noise_nt(array.settings.cycle_count), 2),
            "lsb_nt": round(1000.0 / rm3100_gain_counts_per_ut(array.settings.cycle_count), 3),
        },
        "uap_moment_am2": array.moment_am2,
        "storm": array.storm is not None,
        "nodes": list(per_node.values()),
    }
