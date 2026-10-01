"""
Shared Simulation World for Retina Passive Radar Network.

Maintains a single "world" of simulated aircraft objects, and multiple
synthetic nodes that observe them. Each node generates detections only
for aircraft within its detection cone (Yagi beamwidth + range limit).

Architecture:
  SimulationWorld  ─ holds SimulatedAircraft objects
       │
       ├── SyntheticNodeView(node1) ─ observes aircraft through node1's geometry
       ├── SyntheticNodeView(node2) ─ observes aircraft through node2's geometry
       └── ...

Usage:
    world = SimulationWorld()
    world.add_node(node_config_1)
    world.add_node(node_config_2)
    world.step(dt)  # advance simulation
    frames = world.generate_all_frames(timestamp_ms)  # {node_id: frame_dict}
"""

import hashlib
import json
import math
import random
import time
from dataclasses import asdict, dataclass, field

C_KM_US = 0.299792458  # speed of light km/μs
C_KM_S = 299792.458  # speed of light km/s
R_EARTH = 6371.0  # Earth radius km

# ── US flight corridor waypoints (major airports / airways) ──────────────────
_US_WAYPOINTS = [
    # East coast
    (33.6407, -84.4277),  # ATL Atlanta
    (35.2144, -80.9473),  # CLT Charlotte
    (35.8776, -78.7875),  # RDU Raleigh
    (36.0984, -79.9372),  # GSO Greensboro
    (37.5054, -77.3197),  # RIC Richmond
    (38.8512, -77.0402),  # DCA Washington
    (39.1776, -76.6683),  # BWI Baltimore
    (39.8744, -75.2424),  # PHL Philadelphia
    (40.6413, -73.7781),  # JFK New York
    (42.3656, -71.0096),  # BOS Boston
    # Southeast
    (32.1271, -81.2020),  # SAV Savannah
    (28.4312, -81.3081),  # MCO Orlando
    (25.7959, -80.2870),  # MIA Miami
    (30.4941, -81.6879),  # JAX Jacksonville
    (27.9755, -82.5332),  # TPA Tampa
    # Central
    (36.1245, -86.6782),  # BNA Nashville
    (38.1744, -85.7360),  # SDF Louisville
    (39.0489, -84.6678),  # CVG Cincinnati
    (41.4117, -81.8498),  # CLE Cleveland
    (42.2125, -83.3534),  # DTW Detroit
    (41.9742, -87.9073),  # ORD Chicago
    (44.8848, -93.2223),  # MSP Minneapolis
    (38.7487, -90.3700),  # STL St Louis
    (39.2976, -94.7139),  # MCI Kansas City
    (29.9934, -90.2580),  # MSY New Orleans
    (29.6454, -95.2789),  # IAH Houston
    (32.8968, -97.0380),  # DFW Dallas
    (35.3926, -97.6007),  # OKC Oklahoma City
    (39.8561, -104.6737),  # DEN Denver
    # West
    (33.4373, -112.0078),  # PHX Phoenix
    (36.0840, -115.1537),  # LAS Las Vegas
    (34.0522, -118.2437),  # LAX Los Angeles
    (37.6213, -122.3790),  # SFO San Francisco
    (47.4502, -122.3088),  # SEA Seattle
    (45.5898, -122.5951),  # PDX Portland
]

# ── Regional waypoint nets ────────────────────────────────────────────────────
# A metro-scoped fleet has no receivers outside its own metro, so cross-country
# en-route traffic is pure waste: it burns simulation budget on aircraft no node
# can ever see, and puts ground-truth tracks on the map thousands of km from the
# only coverage that exists. Selecting a regional net keeps the background
# traffic inside the region the fleet actually covers.
#
# Keyed by the same metro codes as generator._KNOWN_METROS.
_REGIONAL_WAYPOINTS: dict[str, list[tuple[float, float]]] = {
    "gvl": [
        (34.8957, -82.2189),  # GSP Greenville-Spartanburg
        (34.8479, -82.3499),  # GMU Greenville Downtown
        (34.9157, -81.9565),  # SPA Spartanburg Downtown Memorial
        (34.4946, -82.7093),  # AND Anderson Regional
        (35.4362, -82.5418),  # AVL Asheville
        (35.2144, -80.9473),  # CLT Charlotte
        (34.8964, -81.0572),  # RKH Rock Hill
        (33.9388, -81.1195),  # CAE Columbia
        (34.4984, -81.9573),  # GRD Greenwood
        (35.7565, -81.6790),  # HKY Hickory
    ],
}


def waypoints_for_metro(metro: str | None) -> list[tuple[float, float]]:
    """Waypoint net for a metro code, falling back to the nationwide net.

    An unknown or absent code returns _US_WAYPOINTS, so callers that don't know
    about regional scoping keep the original coast-to-coast behaviour.
    """
    if not metro:
        return _US_WAYPOINTS
    return _REGIONAL_WAYPOINTS.get(metro.strip().lower(), _US_WAYPOINTS)


# Transponder-outage timing.  An outage has to outlast the backend's 45 s
# fix-age cap and the 60 s map expiry to be observable as "ADS-B went away"
# rather than as a dropped update, and has to be short enough that a 20-minute
# capture sees several start and end — hence a minute to four minutes, opening
# 20 s to 2 min after spawn (long enough that the track is established first).
_ADSB_OUTAGE_START_S = (20.0, 120.0)
_ADSB_OUTAGE_DURATION_S = (60.0, 240.0)
# Re-roll of already-flying aircraft when the knob is raised at runtime opens
# sooner: verification should not have to wait for a whole fleet turnover.
_ADSB_OUTAGE_RUNTIME_START_S = (5.0, 60.0)

# Unit conversions for the feed's tar1090 fields.
_KNOTS_TO_KM_S = 1.852 / 3600.0
_FT_TO_KM = 0.0003048

# Altitude sanity for live feed rows.  adsb.retina.fm sometimes serves ANOTHER
# aircraft's cruise altitude in a low GA row's alt_baro (+23k…+33k ft over truth,
# flipping between polls, persisting up to ~50 s), and every row carries
# type "other" with no alt_geom — there is no feed field to filter on.  Scale
# separates the two cases: a real aircraft moves under 150 m in a 5 s poll even
# at 6,000 ft/min, while the observed corruption is 7–10 km.
_LIVE_ALT_JUMP_KM = 1.0
# A SELF-CONSISTENT run of rejected readings — every one within
# _LIVE_ALT_JUMP_KM of the last — that lasts a full minute is a wrong HELD
# altitude (a corrupt spawn, or a corrupt value adopted earlier), so adopt it
# and bound the wrongness to a minute instead of pinning the aircraft forever.
# Consistency is what makes that safe: replaying N6389R (2026-09-10) the feed
# disagreed with the coasted altitude for 97 s straight across FOUR different
# corrupt values (+10.0 km for 22 s, +2.1 km for 7 s, +9.9 km for 48 s, +8.0 km
# for 13 s, then truth).  Adopting on elapsed time alone would have taken
# +9.9 km at the 60 s mark and then rejected the RETURNING TRUTH as a fresh
# >1 km jump for another minute — worse than no guard at all.  A genuinely
# re-levelling aircraft never jumps _LIVE_ALT_JUMP_KM between polls, so its
# readings track the coasted altitude and never open a run at all.
_LIVE_ALT_ADOPT_S = 60.0


@dataclass
class SimulatedAircraft:
    """A simulated aircraft in the world with lat/lon/alt position."""

    object_id: str
    # Position LLA
    lat: float
    lon: float
    alt_km: float
    # Velocity (km/s) in ENU-like local frame
    vel_east: float  # km/s east
    vel_north: float  # km/s north
    vel_up: float  # km/s vertical
    # Heading (degrees from north, clockwise)
    heading_deg: float
    speed_km_s: float
    # Cruise speed the in-trail separation modulation recovers toward once a
    # conflict clears (speed_km_s is the CURRENT speed, which separation and
    # anomaly events both mutate).  0.0 = "not set" for pre-existing callers
    # that construct aircraft directly; separation then treats the current
    # speed as cruise.
    base_speed_km_s: float = 0.0
    # Type and classification
    has_adsb: bool = False
    is_anomalous: bool = False
    object_type: str = "aircraft"  # "aircraft", "drone", "anomalous"
    adsb_hex: str | None = None
    adsb_callsign: str | None = None
    # Lifecycle
    created_at: float = 0.0
    lifetime_s: float = 600.0
    # Expired and rerouted toward the region edge for retirement (see
    # SimulationWorld._route_out); guards the reroute from re-firing.
    departing: bool = False
    # Waypoint navigation
    waypoints: list = field(default_factory=list)
    waypoint_idx: int = 0
    # Mid-flight anomaly injection (scheduled event)
    anomaly_event: str | None = None  # None | "hijack" | "spoof" | "orbit" | "altitude_jump" | "id_swap"
    anomaly_trigger_at: float = 0.0  # world time when event fires
    anomaly_fired: bool = False  # True once the event has been applied
    _pre_spoof_lat: float = 0.0  # real position before GPS spoof
    _pre_spoof_lon: float = 0.0
    # Transponder outage (scheduled window in world-clock seconds).  The
    # aircraft still HAS a transponder — has_adsb and adsb_hex are untouched —
    # it just stops broadcasting for the window, which is what a real failed
    # or switched-off transponder looks like to every downstream consumer.
    adsb_outage_start_s: float | None = None
    adsb_outage_end_s: float | None = None
    # World clock as of the last step(), so adsb_silent can be a plain
    # property: the aircraft has no back-reference to its world.
    sim_now_s: float = 0.0
    # Provenance.  "sim" = spawned by the world's own planner; "live" = a real
    # aircraft mirrored from an ADS-B feed (SimulationWorld.ingest_live_aircraft).
    # A live aircraft flies its real track: no waypoints, no separation
    # slowing, no lifetime — it leaves when the feed stops reporting it.
    source: str = "sim"
    # The real transponder hex of a live aircraft, kept even while the world
    # casts it as dark (adsb_hex is None then), so the cast can flip back
    # without a re-ingest and the feed can find the aircraft again by key.
    live_hex: str | None = None
    # World clock of the last feed FIX (the row's capture time mapped onto the
    # world clock, not the time the poll carrying it arrived); a live aircraft
    # whose newest fix is older than SimulationWorld.live_stale_s is dropped.
    # Keying on the fix rather than the poll matters because the feed keeps
    # echoing a stale row long after an aircraft stops transmitting.
    live_seen_s: float = 0.0
    # World clock at the FIRST of the current run of rejected feed altitudes
    # (SimulationWorld.ingest_live_aircraft's jump guard); None whenever the
    # last reading was accepted.  A SELF-CONSISTENT run lasting
    # _LIVE_ALT_ADOPT_S is adopted.
    live_alt_reject_s: float | None = None
    # The last REJECTED altitude (km), so the guard can tell a run that agrees
    # with itself — one wrong held altitude, worth adopting once it has stood
    # for _LIVE_ALT_ADOPT_S — from the feed flipping between several other
    # aircraft's levels, which is never a re-level and must never be adopted.
    live_alt_reject_km: float = 0.0
    # A scripted object flies a fixed straight line at constant speed: no
    # waypoints, no erratic manoeuvres, no separation slowing. The UAP flyby
    # (SimulationWorld.inject_uap_flyby) is one, so that its closest approach
    # to a magnetometer happens where and when it was asked to. The line lies
    # on the flat east/north plane around track_origin (lat, lon): the object
    # is at track_start_km (east, north) at created_at and moves from there at
    # its vel_east/vel_north. It retires at retire_at_s on the world clock
    # instead of by lifetime.
    track_origin: tuple[float, float] | None = None
    track_start_km: tuple[float, float] = (0.0, 0.0)
    retire_at_s: float | None = None

    @property
    def scripted(self) -> bool:
        """True for an object on a fixed line (track_origin) rather than a route."""
        return self.track_origin is not None

    @property
    def adsb_silent(self) -> bool:
        """True while the world clock sits inside this aircraft's outage."""
        if self.adsb_outage_start_s is None or self.adsb_outage_end_s is None:
            return False
        return self.adsb_outage_start_s <= self.sim_now_s < self.adsb_outage_end_s


@dataclass
class NodeConfig:
    """Configuration for a synthetic radar node."""

    node_id: str = "synth-node-01"
    rx_lat: float = 33.939182
    rx_lon: float = -84.651910
    rx_alt_ft: float = 950.0
    tx_lat: float = 33.75667
    tx_lon: float = -84.331844
    tx_alt_ft: float = 1600.0
    fc_hz: float = 195_000_000.0
    fs_hz: float = 2_000_000.0
    doppler_min: float = -300.0
    doppler_max: float = 300.0
    min_doppler: float = 15.0
    # Detection geometry
    beam_azimuth_deg: float | None = None  # None → auto broadside in add_node
    beam_width_deg: float = 42.0  # Yagi half-power beamwidth (fleet spec)
    max_range_km: float = 50.0  # maximum RX→target range (monostatic)
    # Maximum *bistatic* range: (RX→target) + (target→TX) − baseline, i.e. the
    # differential range the delay measurement actually represents, and what
    # sets received power via the bistatic radar equation.  Physically the
    # correct limit — it makes the footprint an ellipse with foci at RX and TX
    # rather than a circle around the RX.
    # None keeps the older monostatic rule, so real hardware nodes carrying
    # only max_range_km are unaffected.
    max_bistatic_range_km: float | None = None


def config_hash(config: NodeConfig) -> str:
    """Compute a short hash of the node configuration."""
    cfg_str = json.dumps(asdict(config), sort_keys=True)
    return hashlib.sha256(cfg_str.encode()).hexdigest()[:16]


@dataclass
class MetroCell:
    """A metro coverage cell that hub-radial traffic is routed through.

    Aircraft assigned to a cell are arrivals (fly the edge→core radial),
    departures (core→edge) or overflights (a chord through the core) — the same
    radial pattern as real STAR/SID procedures, which is where the receiver
    ring's coverage spokes lie.
    """

    core_lat: float
    core_lon: float
    radius_km: float = 70.0
    ops_weight: float = 1.0
    arrival_bearings_deg: list = field(default_factory=list)  # empty → uniform
    departure_bearings_deg: list = field(default_factory=list)


# ── Coordinate helpers ────────────────────────────────────────────────────────


def _lla_to_enu(lat, lon, alt_km, ref_lat, ref_lon, ref_alt_km):
    """Convert LLA to ENU (km) relative to reference point."""
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    north = dlat * R_EARTH
    east = dlon * R_EARTH * math.cos(math.radians(ref_lat))
    up = alt_km - ref_alt_km
    return east, north, up


def _enu_to_lla(east_km, north_km, up_km, ref_lat, ref_lon, ref_alt_km):
    """Convert ENU (km) back to LLA."""
    lat = ref_lat + math.degrees(north_km / R_EARTH)
    lon = ref_lon + math.degrees(east_km / (R_EARTH * math.cos(math.radians(ref_lat))))
    alt_km_out = ref_alt_km + up_km
    return lat, lon, alt_km_out


def _haversine_km(lat1, lon1, lat2, lon2):
    """Great-circle distance in km."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    return R_EARTH * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _bearing_deg(lat1, lon1, lat2, lon2):
    """Bearing from point1 to point2, in degrees from north clockwise."""
    dlon = math.radians(lon2 - lon1)
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    x = math.sin(dlon) * math.cos(lat2r)
    y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    return math.degrees(math.atan2(x, y)) % 360


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _bistatic_delay(target_enu, tx_enu, rx_enu=(0, 0, 0)):
    """Compute bistatic differential delay in microseconds."""
    d_tx_tgt = _norm([target_enu[i] - tx_enu[i] for i in range(3)])
    d_tgt_rx = _norm([rx_enu[i] - target_enu[i] for i in range(3)])
    d_tx_rx = _norm([rx_enu[i] - tx_enu[i] for i in range(3)])
    diff_range = (d_tx_tgt + d_tgt_rx) - d_tx_rx
    return diff_range / C_KM_US


def _bistatic_doppler(target_enu, vel_enu, tx_enu, rx_enu, freq_hz):
    """Compute bistatic Doppler shift in Hz."""
    to_tx = [tx_enu[i] - target_enu[i] for i in range(3)]
    to_rx = [rx_enu[i] - target_enu[i] for i in range(3)]
    d_tx = _norm(to_tx)
    d_rx = _norm(to_rx)
    if d_tx < 1e-9 or d_rx < 1e-9:
        return 0.0
    u_tx = [to_tx[i] / d_tx for i in range(3)]
    u_rx = [to_rx[i] / d_rx for i in range(3)]
    v_rad_tx = sum(vel_enu[i] * u_tx[i] for i in range(3))
    v_rad_rx = sum(vel_enu[i] * u_rx[i] for i in range(3))
    return (freq_hz / C_KM_S) * (v_rad_tx + v_rad_rx)


# ── Flight corridor route generation ─────────────────────────────────────────

_NATIONWIDE_MIN_LEG_KM = 400.0


def _min_leg_km(waypoints: list[tuple[float, float]]) -> float:
    """A "this is a real en-route leg" threshold scaled to a waypoint net.

    Scaled from the net's own extent, capped at the nationwide 400 km so the
    continent-wide net keeps its original behaviour exactly. A regional net has
    no pair 400 km apart, so without the scaling every destination would be
    rejected and the fallback would hand back coast-to-coast routes — the exact
    thing regional scoping exists to prevent.
    """
    if len(waypoints) < 2:
        return 0.0
    lats = [wp[0] for wp in waypoints]
    lons = [wp[1] for wp in waypoints]
    span_km = _haversine_km(min(lats), min(lons), max(lats), max(lons))
    return min(_NATIONWIDE_MIN_LEG_KM, 0.35 * span_km)


def _pick_route(
    center_lat: float,
    center_lon: float,
    max_dist_km: float = 300,
    waypoints: list[tuple[float, float]] | None = None,
) -> list[tuple[float, float]]:
    """Pick a sequence of 2-4 waypoints near center forming a realistic route."""
    waypoints = waypoints if waypoints is not None else _US_WAYPOINTS
    nearby = [wp for wp in waypoints if _haversine_km(center_lat, center_lon, wp[0], wp[1]) < max_dist_km]
    if len(nearby) < 2:
        nearby = sorted(waypoints, key=lambda wp: _haversine_km(center_lat, center_lon, wp[0], wp[1]))[:6]

    n_waypoints = random.randint(2, min(4, len(nearby)))
    start = random.choice(nearby)
    route = [start]
    remaining = [wp for wp in nearby if wp != start]
    for _ in range(n_waypoints - 1):
        if not remaining:
            break
        # Pick next waypoint somewhat in the forward direction
        last = route[-1]
        remaining.sort(key=lambda wp: _haversine_km(last[0], last[1], wp[0], wp[1]))
        # Choose from closest 3, weighted toward closer ones
        candidates = remaining[: min(3, len(remaining))]
        nxt = random.choice(candidates)
        route.append(nxt)
        remaining = [wp for wp in remaining if wp != nxt]
    return route


# ── SimulationWorld ───────────────────────────────────────────────────────────


class SimulationWorld:
    """Shared simulation world with aircraft and multiple observer nodes."""

    def __init__(
        self,
        center_lat: float = 34.85,
        center_lon: float = -82.39,
        waypoints: list[tuple[float, float]] | None = None,
    ):
        self.center_lat = center_lat
        self.center_lon = center_lon
        # En-route waypoint net for background traffic. Defaults to the
        # nationwide list; set a regional net (waypoints_for_metro) to keep
        # background aircraft inside a metro-scoped fleet's coverage.
        self.waypoints = waypoints if waypoints is not None else _US_WAYPOINTS
        self.aircraft: list[SimulatedAircraft] = []
        self.nodes: dict[str, NodeConfig] = {}
        self._next_id = 1
        self._time = 0.0
        # Target count range
        self.min_aircraft = 5
        self.max_aircraft = 15
        # Object type spawn fractions (adjustable at runtime).
        # frac_anomalous doubles as the master gate for ALL anomaly generation:
        # at 0 it also suppresses the mid-flight anomaly scheduler
        # (_maybe_schedule_anomaly), which would otherwise keep turning normal
        # commercial aircraft anomalous at its own hardcoded rate. One switch,
        # so "anomalies off" means off — and raising it turns everything back on.
        self.frac_anomalous: float = 0.0
        # Drones off by default, matching the backend's simulation_config
        # default (user call, 2026-08: fixed-wing scene only).  The old 0.10
        # here spawned a handful of drones in the window between world
        # construction and the first config poll on EVERY fleet restart —
        # visible as "a few drones exist even with the slider at 0" until
        # they aged out.  The orchestrator poll's absent-key fallback must
        # stay matched to this value.
        self.frac_drone: float = 0.0
        self.frac_dark: float = 0.15
        # Fraction of ADS-B-equipped aircraft that suffer a transponder
        # outage mid-flight.  Orthogonal to the frac_* type roll above (it is
        # a fraction OF the ADS-B population, not of all spawns), and OFF by
        # default so nothing changes for anyone who does not set it.
        self.frac_adsb_outage: float = 0.0
        # remaining fraction = commercial aircraft with ADS-B
        #
        # Live-derived traffic (ingest_live_aircraft): the share of feed
        # aircraft the world casts as DARK — mirrored without their
        # transponder, so the synthetic nodes echo a real trajectory the
        # server holds no ADS-B tag for.  Orthogonal to frac_dark, which only
        # rolls the world's own synthetic spawns, and to min/max_aircraft,
        # which only count them: the feed decides how many live aircraft
        # there are.  The cast is a stable per-hex hash (live_role_is_dark),
        # so moving the knob re-partitions the aircraft already in the air
        # instead of re-rolling them.
        self.frac_live_dark: float = 0.0
        # Feed aircraft not refreshed for this long (world seconds) are dropped.
        self.live_stale_s: float = 60.0
        self.live_aircraft: dict[str, SimulatedAircraft] = {}
        # Hub-radial flight planning: when metro_cells is non-empty, this
        # fraction of spawns is routed through a metro cell; the rest are
        # en-route traffic on the inter-metro waypoint net. Empty cells →
        # pure fallback (the original random-node-anchored spawn).
        self.metro_cells: list[MetroCell] = []
        self.frac_metro_traffic: float = 0.6
        self.arrival_departure_overflight_weights = (0.45, 0.35, 0.20)
        # Traffic separation.  Real traffic never stacks: terminal in-trail
        # minima are ~3 NM and crossing flows are altitude-split, but the
        # hub-radial planner converges every metro spawn on the same ~2 km
        # core with no deconfliction at all, so the fleet routinely flew
        # pairs inside the solver's association gates (delay gate ≈ 1-3 km)
        # — making association ambiguity a property of the SIMULATOR, not of
        # realistic traffic.  Two mechanisms, both gated on these knobs:
        # spawn poses resample away from live traffic, and in-flight
        # conflicts resolve by slowing the later-created aircraft
        # (_enforce_separation).  A pair needs BOTH horizontal and vertical
        # proximity to count as a conflict.
        self.min_separation_km: float = 5.0
        self.min_vertical_sep_km: float = 0.6  # ~2000 ft

    @property
    def clock_s(self) -> float:
        """Seconds of simulated time since the world was built."""
        return self._time

    def add_node(self, config: NodeConfig):
        """Register a synthetic node in the simulation.

        An explicit beam_azimuth_deg (aimed Yagi) is kept as-is. When unset
        (None) the beam is auto-aimed broadside to the RX→TX baseline, which
        maximises cross-coverage of aircraft transiting the bistatic zone.
        """
        if config.beam_azimuth_deg is None:
            config.beam_azimuth_deg = (
                _bearing_deg(
                    config.rx_lat,
                    config.rx_lon,
                    config.tx_lat,
                    config.tx_lon,
                )
                + 90.0
            ) % 360.0
        self.nodes[config.node_id] = config

    def _choose_spawn_pose(self) -> tuple[float, float, list]:
        """Pick spawn (lat, lon) and route. With metro cells: frac_metro_traffic
        is hub-radial through a metro (denser, feeds the rings), the rest is
        nationwide en-route background so the whole map stays alive (every plane a
        blue dot, not every blue dot a plane). Without cells: the legacy
        node-anchored spawn (keeps training export and solo arcs intact)."""
        if not self.metro_cells:
            return self._fallback_pose()
        if random.random() < self.frac_metro_traffic:
            cell = self._pick_metro_cell()
            kind = random.choices(
                ("arrival", "departure", "overflight"),
                weights=self.arrival_departure_overflight_weights,
            )[0]
            return self._radial_pose(cell, kind)
        return self._nationwide_pose()

    def _nationwide_pose(self) -> tuple[float, float, list]:
        """En-route traffic on self.waypoints, spawned anywhere along the leg (not
        just at airports) and independent of node placement — the background that
        keeps the map alive. Nationwide by default; regional under metro scoping."""
        net = self.waypoints
        start = random.choice(net)
        min_leg = _min_leg_km(net)
        far = [wp for wp in net if _haversine_km(start[0], start[1], wp[0], wp[1]) > min_leg]
        dest = random.choice(far or net)
        t = random.uniform(0.0, 1.0)
        lat = start[0] + t * (dest[0] - start[0]) + random.gauss(0, 0.3)
        lon = start[1] + t * (dest[1] - start[1]) + random.gauss(0, 0.3)
        return lat, lon, [(lat, lon), dest]

    def _pick_metro_cell(self) -> MetroCell:
        weights = [c.ops_weight for c in self.metro_cells]
        if sum(weights) <= 0:
            # All-zero ops_weight (e.g. a hand-authored config): random.choices
            # rejects a zero total, so fall back to a uniform pick.
            return random.choice(self.metro_cells)
        return random.choices(self.metro_cells, weights=weights)[0]

    def _radial_pose(self, cell: MetroCell, kind: str) -> tuple[float, float, list]:
        """Edge↔core radial (arrival/departure) or a chord near the core
        (overflight) — the radial geometry of real STAR/SID procedures, which is
        where the receiver ring's coverage spokes lie."""

        def at(bearing_deg: float, dist_km: float) -> tuple[float, float]:
            br = math.radians(bearing_deg)
            lat = cell.core_lat + math.degrees((dist_km * math.cos(br)) / R_EARTH)
            lon = cell.core_lon + math.degrees(
                (dist_km * math.sin(br)) / (R_EARTH * math.cos(math.radians(cell.core_lat)))
            )
            return lat, lon

        core = (cell.core_lat + random.gauss(0, 0.02), cell.core_lon + random.gauss(0, 0.02))

        if kind == "arrival":
            bearing = random.choice(cell.arrival_bearings_deg) if cell.arrival_bearings_deg else random.uniform(0, 360)
            lat, lon = at(bearing, cell.radius_km)
            return lat, lon, [(lat, lon), core]

        if kind == "departure":
            bearing = (
                random.choice(cell.departure_bearings_deg) if cell.departure_bearings_deg else random.uniform(0, 360)
            )
            return core[0], core[1], [core, at(bearing, cell.radius_km)]

        # overflight: enter one edge, exit near the opposite edge, passing near the core
        entry = random.uniform(0, 360)
        exit_b = (entry + 180 + random.uniform(-35, 35)) % 360
        lat, lon = at(entry, cell.radius_km)
        return lat, lon, [(lat, lon), at(exit_b, cell.radius_km)]

    def _fallback_pose(self) -> tuple[float, float, list]:
        """Original spawn: broadside to a random node's baseline, heading to a
        nearby waypoint. Used for en-route traffic and whenever no metro cells
        are configured (keeps training export and solo single-node arcs intact)."""
        if self.nodes:
            anchor = random.choice(list(self.nodes.values()))
            baseline_bearing = _bearing_deg(anchor.rx_lat, anchor.rx_lon, anchor.tx_lat, anchor.tx_lon)
            perp_rad = math.radians((baseline_bearing + 90.0) % 360.0)
            dist_km = random.uniform(5.0, anchor.max_range_km * 0.7)
            # R_EARTH-derived like every other conversion in this file — these
            # were the last 111.32 literals, 0.11% off the rest of the sim.
            anchor_lat, anchor_lon, _ = _enu_to_lla(
                dist_km * math.sin(perp_rad),
                dist_km * math.cos(perp_rad),
                0.0,
                anchor.rx_lat,
                anchor.rx_lon,
                0.0,
            )
        else:
            anchor_lat, anchor_lon = self.center_lat, self.center_lon

        lat = anchor_lat + random.gauss(0, 0.03)
        lon = anchor_lon + random.gauss(0, 0.03)
        route = [(lat, lon)] + _pick_route(anchor_lat, anchor_lon, max_dist_km=50, waypoints=self.waypoints)
        return lat, lon, route

    def _spawn_aircraft(self, mode: str = "detection") -> SimulatedAircraft:
        """Spawn a new aircraft along a realistic flight corridor.

        Object types are selected probabilistically from the frac_* instance
        attributes: dark aircraft (no transponder), drones (low/slow), anomalous
        objects (erratic), and commercial aircraft with ADS-B as the remainder.
        frac_anomalous defaults to 0 — see __init__.

        mode="anomalous" is an explicit opt-in that injects anomalies regardless
        of frac_anomalous; it is a testing mode and is not used by any deployment
        (every compose profile sets FLEET_MODE=adsb).
        """
        oid = f"obj-{self._next_id:05d}"
        self._next_id += 1

        is_anomalous = False
        has_adsb = False
        adsb_hex = None
        adsb_callsign = None
        object_type = "aircraft"

        # Roll object type using instance fractions (adjustable at runtime)
        roll = random.random()
        if roll < self.frac_anomalous:
            object_type = "anomalous"
            is_anomalous = True
        elif roll < self.frac_anomalous + self.frac_drone:
            object_type = "drone"
        elif roll < self.frac_anomalous + self.frac_drone + self.frac_dark:
            object_type = "aircraft"  # dark aircraft — no ADS-B
        else:
            object_type = "aircraft"  # commercial — will get ADS-B in adsb modes

        # Resample the spawn pose away from live traffic — best-effort, never
        # fails: after the attempt budget the farthest candidate wins, so a
        # saturated core degrades to "as separated as the planner could get"
        # rather than blocking spawns (step() spawns in a while-loop up to
        # min_aircraft; a hard reject there would spin forever).
        lat, lon, route = self._choose_spawn_pose()
        best = (self._nearest_traffic_km(lat, lon), lat, lon, route)
        for _ in range(9):
            if best[0] >= self.min_separation_km:
                break
            lat, lon, route = self._choose_spawn_pose()
            d = self._nearest_traffic_km(lat, lon)
            if d > best[0]:
                best = (d, lat, lon, route)
        _, lat, lon, route = best

        if mode == "anomalous" and random.random() < 0.2:
            is_anomalous = True
            object_type = "anomalous"
            speed_km_s = random.uniform(0.01, 0.5)  # 10-500 m/s
            alt_km = random.uniform(0.3, 15.0)
        elif object_type == "anomalous":
            speed_km_s = random.uniform(0.01, 0.5)
            alt_km = random.uniform(0.3, 15.0)
        elif object_type == "drone":
            speed_km_s = random.uniform(0.01, 0.06)  # 10-60 m/s — small UAS
            alt_km = random.uniform(0.05, 0.5)  # 50-500m AGL
        else:
            speed_km_s = random.uniform(0.12, 0.27)  # 120-270 m/s → typical jet
            alt_km = random.uniform(5.0, 12.0)  # 16k-40k ft

        # Anomalous objects also get ADS-B — anomalous means unusual flight
        # behaviour (speed/altitude/heading changes), NOT transponder absence.
        #
        # The floor must be the SAME cumulative boundary the type roll used for
        # "commercial" above. It was previously hardcoded to 0.30, which silently
        # assumed the original defaults (0.05 + 0.10 + 0.15). Any other fractions
        # — including anything set through the Physics Settings slider at
        # runtime — pushed part of the commercial band below the literal, so
        # those aircraft were spawned with no transponder and showed up as dark.
        adsb_roll_floor = self.frac_anomalous + self.frac_drone + self.frac_dark
        if (mode in ("adsb", "anomalous") and object_type != "drone" and roll >= adsb_roll_floor) or is_anomalous:
            has_adsb = True
            adsb_hex = f"{random.randint(0x100000, 0xFFFFFF):06x}"
            letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            adsb_callsign = f"{''.join(random.choices(letters, k=3))}{random.randint(100, 9999)}"

        outage_start, outage_end = self._roll_adsb_outage(has_adsb, _ADSB_OUTAGE_START_S)

        # Initial heading toward next waypoint
        next_wp = route[1] if len(route) > 1 else route[0]
        heading = _bearing_deg(lat, lon, next_wp[0], next_wp[1])
        heading_rad = math.radians(heading)

        vel_east = speed_km_s * math.sin(heading_rad)
        vel_north = speed_km_s * math.cos(heading_rad)
        vel_up = random.uniform(-0.003, 0.003)

        return SimulatedAircraft(
            object_id=oid,
            lat=lat,
            lon=lon,
            alt_km=alt_km,
            vel_east=vel_east,
            vel_north=vel_north,
            vel_up=vel_up,
            heading_deg=heading,
            speed_km_s=speed_km_s,
            base_speed_km_s=speed_km_s,
            has_adsb=has_adsb,
            is_anomalous=is_anomalous,
            object_type=object_type,
            adsb_hex=adsb_hex,
            adsb_callsign=adsb_callsign,
            adsb_outage_start_s=outage_start,
            adsb_outage_end_s=outage_end,
            sim_now_s=self._time,
            created_at=self._time,
            lifetime_s=random.uniform(180, 900) if object_type != "drone" else random.uniform(60, 300),
            waypoints=route,
            waypoint_idx=1,
            **self._maybe_schedule_anomaly(is_anomalous, object_type),
        )

    def _roll_adsb_outage(self, has_adsb: bool, start_range: tuple[float, float]) -> tuple[float | None, float | None]:
        """Roll a transponder-outage window, or (None, None) for no outage.

        Dark aircraft are never candidates: they have no transponder to lose.
        """
        if not has_adsb or self.frac_adsb_outage <= 0.0:
            return None, None
        if random.random() >= self.frac_adsb_outage:
            return None, None
        start = self._time + random.uniform(*start_range)
        return start, start + random.uniform(*_ADSB_OUTAGE_DURATION_S)

    def schedule_adsb_outages(self) -> int:
        """Roll outages for live ADS-B aircraft that have none scheduled yet.

        Called when frac_adsb_outage is raised at runtime: without it the knob
        only reaches aircraft spawned after the change, so verification would
        have to wait for a full fleet turnover.  Aircraft that already carry a
        window (including one that has already ended) are left alone, so
        repeated config polls cannot re-roll the same aircraft every 5 s.
        Returns the number newly scheduled.
        """
        scheduled = 0
        for ac in self.aircraft:
            if ac.adsb_outage_start_s is not None:
                continue
            start, end = self._roll_adsb_outage(ac.has_adsb, _ADSB_OUTAGE_RUNTIME_START_S)
            if start is None:
                continue
            ac.adsb_outage_start_s = start
            ac.adsb_outage_end_s = end
            ac.sim_now_s = self._time
            scheduled += 1
        return scheduled

    def inject_uap_flyby(
        self,
        lat: float,
        lon: float,
        *,
        ground_alt_km: float = 0.0,
        closest_approach_m: float = 600.0,
        altitude_m: float = 300.0,
        speed_mps: float = 120.0,
        heading_deg: float = 90.0,
        lead_s: float = 60.0,
        name: str = "uap",
    ) -> SimulatedAircraft:
        """Fly a UAP past a point: an anomalous object on a straight, level
        track whose horizontal closest approach to (lat, lon) is
        ``closest_approach_m`` to the right of track, ``altitude_m`` above the
        ground there, ``lead_s`` seconds from now.

        The track is a straight line on the flat east/north plane around
        (lat, lon), the plane a magnetometer there measures distance in, so
        the pass comes exactly as close as asked, exactly when asked, at any
        speed and lead.

        It is an ordinary world object from then on: the radar nodes echo it
        if it crosses a beam, the ground-truth push carries it as anomalous,
        and magnetometers see it as a dipole. It has no transponder (a UAP is
        unidentified by definition) and retires once it is as far past the
        point as it started before it.
        """
        speed_km_s = speed_mps / 1000.0
        heading_rad = math.radians(heading_deg)
        along = (math.sin(heading_rad), math.cos(heading_rad))  # east, north
        right = (math.cos(heading_rad), -math.sin(heading_rad))
        run_km = speed_km_s * lead_s
        cpa_km = closest_approach_m / 1000.0
        east = right[0] * cpa_km - along[0] * run_km
        north = right[1] * cpa_km - along[1] * run_km
        start_lat, start_lon, _ = _enu_to_lla(east, north, 0.0, lat, lon, 0.0)
        oid = f"{name}-{self._next_id:05d}"
        self._next_id += 1
        ac = SimulatedAircraft(
            object_id=oid,
            lat=start_lat,
            lon=start_lon,
            alt_km=ground_alt_km + altitude_m / 1000.0,
            vel_east=along[0] * speed_km_s,
            vel_north=along[1] * speed_km_s,
            vel_up=0.0,
            heading_deg=heading_deg % 360.0,
            speed_km_s=speed_km_s,
            base_speed_km_s=speed_km_s,
            has_adsb=False,
            is_anomalous=True,
            object_type="anomalous",
            created_at=self._time,
            lifetime_s=float("inf"),
            sim_now_s=self._time,
            track_origin=(lat, lon),
            track_start_km=(east, north),
            retire_at_s=self._time + 2.0 * lead_s,
        )
        self.aircraft.append(ac)
        return ac

    def _fly_scripted(self, ac: SimulatedAircraft) -> None:
        """Put a scripted object where its line has it at this world time.

        Placed from the time elapsed since it was created, not stepped like
        other traffic: stepping longitude by the cosine of the latitude
        reached so far bends a diagonal track off its line, by 100 m over a
        ten-minute lead at 120 m/s and by kilometres at higher speeds, and
        the closest approach moves with it. No steering, no perturbation, and
        level at the altitude it was given.
        """
        elapsed = self._time - ac.created_at
        ac.lat, ac.lon, _ = _enu_to_lla(
            ac.track_start_km[0] + ac.vel_east * elapsed,
            ac.track_start_km[1] + ac.vel_north * elapsed,
            0.0,
            ac.track_origin[0],
            ac.track_origin[1],
            0.0,
        )

    # Expired aircraft are retired only once they are at least this far from
    # the world center — a target vanishing overhead reads as a tracking bug
    # on the map, so retirement happens off at the edges, beyond the ~60 km
    # node coverage of a metro-scoped fleet.  (A nationwide fleet spawns most
    # aircraft beyond this radius, which degrades to the old expire-anywhere
    # behaviour — acceptable, nothing deployed runs unscoped.)
    retire_edge_km: float = 70.0
    # Extra flight time an expired aircraft gets to actually REACH the edge.
    # Metro-routed traffic (85% of spawns) circulates inside the metro and
    # rarely crosses retire_edge_km on its own, so under the old 2x-lifetime
    # hard cap most of it still vanished mid-view — the edge gate only ever
    # covered planes that happened to be leaving anyway.  On expiry the
    # aircraft is now rerouted outward (_route_out) and 70 km at the slowest
    # commercial speed (~0.1 km/s) is ~700 s, so 900 s means the backstop
    # below fires only for genuinely stuck aircraft.
    exit_grace_s: float = 900.0

    def _route_out(self, ac: "SimulatedAircraft") -> None:
        """Point an expired aircraft at the region edge and let it fly out.

        Replaces the remaining route with one waypoint past retire_edge_km on
        the bearing away from the world center (current heading when the
        aircraft sits at the center itself), so retirement is something the
        viewer watches happen at the edge instead of a dot blinking out.
        """
        if _haversine_km(self.center_lat, self.center_lon, ac.lat, ac.lon) > 1.0:
            brg = _bearing_deg(self.center_lat, self.center_lon, ac.lat, ac.lon)
        else:
            brg = ac.heading_deg
        dist_km = self.retire_edge_km + 15.0
        brg_rad = math.radians(brg)
        wp_lat = self.center_lat + dist_km * math.cos(brg_rad) / 111.32
        wp_lon = self.center_lon + dist_km * math.sin(brg_rad) / (111.32 * math.cos(math.radians(self.center_lat)))
        ac.waypoints = [(wp_lat, wp_lon)]
        ac.waypoint_idx = 0
        ac.departing = True

    def _should_retire(self, ac: "SimulatedAircraft") -> bool:
        if ac.retire_at_s is not None:
            return self._time >= ac.retire_at_s
        age = self._time - ac.created_at
        if age < ac.lifetime_s:
            return False
        if ac.object_type == "drone":
            # Drones loop low and slow and are expected to churn; an amber
            # X-frame vanishing reads as turnover, not a tracking bug.
            if age > ac.lifetime_s * 2:
                return True
        elif age > ac.lifetime_s + self.exit_grace_s:
            # Leak backstop only — a departing aircraft normally crosses the
            # edge well inside the grace window.
            return True
        return _haversine_km(self.center_lat, self.center_lon, ac.lat, ac.lon) > self.retire_edge_km

    def step(self, dt: float, mode: str = "detection"):
        """Advance simulation by dt seconds."""
        self._time += dt

        # Expired non-drones head for the edge before the retire filter sees
        # them past the edge (see _route_out / _should_retire).
        for ac in self.aircraft:
            if ac.source == "live":
                continue
            if not ac.departing and ac.object_type != "drone" and self._time - ac.created_at >= ac.lifetime_s:
                self._route_out(ac)

        # Retire expired aircraft (edge-gated — see _should_retire).  Live
        # aircraft leave when the feed stops reporting them instead.
        self._expire_live_aircraft()
        self.aircraft = [ac for ac in self.aircraft if ac.source == "live" or not self._should_retire(ac)]

        # Spawn to maintain target count — synthetic aircraft only.  The feed
        # sets the live population; min/max_aircraft stay the operator's
        # knobs for the synthetic traffic layered on top of it, so raising
        # live traffic never starves synthetic spawns and vice versa.
        n_synth = self.synthetic_count()
        while n_synth < self.min_aircraft:
            self.aircraft.append(self._spawn_aircraft(mode))
            n_synth += 1
        if n_synth < self.max_aircraft and random.random() < 0.01:
            self.aircraft.append(self._spawn_aircraft(mode))

        # Update each aircraft
        for ac in self.aircraft:
            ac.sim_now_s = self._time
            self._update_aircraft(ac, dt)

        self._enforce_separation(dt)

    def _nearest_traffic_km(self, lat: float, lon: float) -> float:
        """Horizontal distance to the closest live aircraft (inf when none).

        Horizontal-only on purpose: spawn altitude is rolled after the pose,
        so a vertical allowance here would be checked against a value that
        does not exist yet — and a stricter horizontal-only bubble at spawn
        costs nothing (the resample budget absorbs it)."""
        if not self.aircraft:
            return float("inf")
        return min(_haversine_km(lat, lon, ac.lat, ac.lon) for ac in self.aircraft)

    def _enforce_separation(self, dt: float):
        """In-trail speed modulation: the later-created aircraft of any
        conflicting pair slows toward 70% of cruise until the conflict clears,
        then recovers toward cruise.

        Speed-only, never heading: heading belongs to the waypoint router, and
        a lateral dodge here would fight it every tick.  Slowing the trailer
        resolves both in-trail stacking (the leader pulls away) and crossing
        conflicts (the leader crosses first) — the same sequencing terminal
        control actually applies.  Exemptions are the point, not an
        optimisation: anomalous aircraft keep their erratic close approaches
        (that proximity IS the signature the radar network exists to catch),
        and drones live below min_vertical_sep_km of the jet flow anyway.
        The blend is dt-based (~3 s time constant) so speed changes read as
        throttle on the Doppler channel, not steps."""
        flow = [ac for ac in self.aircraft if not ac.is_anomalous and ac.object_type == "aircraft"]
        flow.sort(key=lambda ac: ac.created_at)
        slowed: set[str] = set()
        for i, trail in enumerate(flow):
            for lead in flow[:i]:
                if abs(lead.alt_km - trail.alt_km) >= self.min_vertical_sep_km:
                    continue
                if _haversine_km(lead.lat, lead.lon, trail.lat, trail.lon) < self.min_separation_km:
                    slowed.add(trail.object_id)
                    break
        blend = 1.0 - math.exp(-dt / 3.0)
        for ac in flow:
            if ac.source == "live":
                # Real traffic is a leader for synthetic trailers to yield
                # to, never a trailer: its speed is the feed's to set.
                continue
            base = ac.base_speed_km_s or ac.speed_km_s
            target = base * 0.7 if ac.object_id in slowed else base
            ac.speed_km_s += (target - ac.speed_km_s) * blend

    # ── Mid-flight anomaly scheduling ────────────────────────────────────────

    _ANOMALY_EVENTS = ["hijack", "spoof", "orbit", "altitude_jump", "id_swap"]

    def _maybe_schedule_anomaly(self, is_anomalous: bool, object_type: str) -> dict:
        """Return kwargs to schedule a mid-flight anomaly event on ~8% of
        normal commercial aircraft.  Already-anomalous or drones are skipped.

        Gated on frac_anomalous: this rate is larger than the spawn-time
        fraction, so without the gate, zeroing frac_anomalous would still leave
        the majority of anomalies running (they just appear 30-120s late).
        """
        if self.frac_anomalous <= 0:
            return {}
        if is_anomalous or object_type == "drone":
            return {}
        if random.random() > 0.08:
            return {}
        event = random.choice(self._ANOMALY_EVENTS)
        # Fire 30-120s after spawn so the aircraft establishes a normal track first
        trigger = self._time + random.uniform(30, 120)
        return {"anomaly_event": event, "anomaly_trigger_at": trigger}

    def _update_aircraft(self, ac: SimulatedAircraft, dt: float):
        """Update aircraft position and navigate toward waypoints."""
        if ac.source == "live":
            # The feed owns the trajectory; between polls the aircraft
            # coasts on its last reported velocity.  No waypoints, no
            # perturbation, no anomaly events — those would put the echo
            # somewhere the real aircraft is not.
            self._dead_reckon(ac, dt)
            return
        if ac.scripted:
            self._fly_scripted(ac)
            return

        # ── Fire scheduled mid-flight anomaly event ──────────────────────────
        if ac.anomaly_event and not ac.anomaly_fired and self._time >= ac.anomaly_trigger_at:
            self._fire_anomaly_event(ac)

        # ── GPS spoof: real position diverges from ADS-B reported location ───
        # After spoof fires, the aircraft keeps flying normally but ADS-B
        # reports a frozen/drifting false position.  The radar sees the REAL
        # position (via delay/Doppler), creating a mismatch.
        # (ADS-B override happens in generate_detections_for_node)

        # Move in ENU then convert back to LLA
        dlat = (ac.vel_north / R_EARTH) * (180 / math.pi) * dt
        dlon = (ac.vel_east / (R_EARTH * math.cos(math.radians(ac.lat)))) * (180 / math.pi) * dt
        ac.lat += dlat
        ac.lon += dlon
        ac.alt_km += ac.vel_up * dt
        # Level off: vel_up was set once at spawn and integrated forever, so
        # every aircraft eventually saturated against the altitude clamps —
        # the whole fleet ended up pinned to the floor or the ceiling.  Unlike
        # vel_east/north (recomputed from heading each tick), there is no
        # vertical navigation, so decay toward level flight with a ~5 min
        # time constant and stop climbing at the clamp.
        ac.vel_up *= math.exp(-dt / 300.0)

        # Clamp altitude
        if ac.alt_km <= 0.1 or ac.alt_km >= 15.0:
            ac.vel_up = 0.0
        ac.alt_km = max(0.1, min(ac.alt_km, 15.0))

        # ── Orbit anomaly: circle in place instead of following waypoints ────
        if ac.anomaly_event == "orbit" and ac.anomaly_fired:
            # Constant 6°/s turn → tight circle (~1 km radius at 200 m/s)
            ac.heading_deg = (ac.heading_deg + 6.0 * dt) % 360
        elif ac.waypoint_idx < len(ac.waypoints):
            # Navigate toward next waypoint
            target_wp = ac.waypoints[ac.waypoint_idx]
            dist_to_wp = _haversine_km(ac.lat, ac.lon, target_wp[0], target_wp[1])

            if dist_to_wp < 5.0:
                # Arrived at waypoint, move to next
                ac.waypoint_idx += 1
            else:
                # Steer toward waypoint (gradual turn)
                desired_heading = _bearing_deg(ac.lat, ac.lon, target_wp[0], target_wp[1])
                heading_diff = (desired_heading - ac.heading_deg + 180) % 360 - 180
                # Max turn rate ~3°/s (standard rate turn)
                max_turn = 3.0 * dt
                turn = max(-max_turn, min(max_turn, heading_diff))
                ac.heading_deg = (ac.heading_deg + turn) % 360

        # Update velocity from heading and speed
        heading_rad = math.radians(ac.heading_deg)
        ac.vel_east = ac.speed_km_s * math.sin(heading_rad)
        ac.vel_north = ac.speed_km_s * math.cos(heading_rad)

        # Small random perturbations
        if not ac.is_anomalous:
            ac.vel_east += random.gauss(0, 0.0005) * dt
            ac.vel_north += random.gauss(0, 0.0005) * dt
        else:
            # Anomalous: erratic behaviour including instant stop/start
            r = random.random()
            if r < 0.005:
                # Instant stop: velocity drops to near zero
                ac.speed_km_s = random.uniform(0.0, 0.005)
                ac.vel_east = 0.0
                ac.vel_north = 0.0
                ac.vel_up = 0.0
            elif r < 0.015:
                # Instant start / speed jump: sudden acceleration
                ac.speed_km_s = random.uniform(0.15, 0.5)
                ac.heading_deg = random.uniform(0, 360)
            elif r < 0.025:
                # Sharp turn: heading change much larger than standard rate
                ac.heading_deg = (ac.heading_deg + random.uniform(30, 120)) % 360
            elif r < 0.03:
                # Random speed/heading change
                ac.speed_km_s = random.uniform(0.01, 0.5)
                ac.heading_deg = random.uniform(0, 360)

    def _fire_anomaly_event(self, ac: SimulatedAircraft):
        """Execute a scheduled mid-flight anomaly event."""
        ac.anomaly_fired = True
        ac.is_anomalous = True
        ev = ac.anomaly_event

        if ev == "hijack":
            # Sudden supersonic acceleration + 180° heading reversal.
            # Triggers: supersonic, instant_acceleration, instant_direction_change
            ac.speed_km_s = random.uniform(0.36, 0.55)  # 360-550 m/s (Mach 1.05-1.6)
            ac.heading_deg = (ac.heading_deg + random.uniform(140, 220)) % 360
            ac.vel_up = random.uniform(-0.02, 0.02)  # erratic climb/descent

        elif ev == "spoof":
            # GPS spoofing: freeze the ADS-B reported position at current location.
            # The aircraft continues flying normally, but ADS-B data will report
            # the frozen position.  Radar delay/Doppler will diverge from ADS-B.
            ac._pre_spoof_lat = ac.lat
            ac._pre_spoof_lon = ac.lon
            # Don't change flight dynamics — the spoof is in ADS-B only

        elif ev == "orbit":
            # Sudden orbit/loiter: aircraft stops following waypoints and
            # enters a tight circle.  Speed stays the same.
            # Triggers: instant_direction_change (continuous high turn rate)
            pass  # Orbit logic is in _update_aircraft above

        elif ev == "altitude_jump":
            # Impossible altitude change: ±3-8 km in one step.
            # Triggers: could be detected by alt_baro anomaly checks if added
            jump = random.choice([-1, 1]) * random.uniform(3.0, 8.0)
            ac.alt_km = max(0.3, min(15.0, ac.alt_km + jump))
            # Also change speed to make it detectable via existing checks
            ac.speed_km_s = random.uniform(0.30, 0.45)  # 300-450 m/s

        elif ev == "id_swap":
            # Transponder identity swap: ADS-B hex changes mid-flight.
            # Same aircraft, new identity — suspicious behavior.
            ac.adsb_hex = f"{random.randint(0x100000, 0xFFFFFF):06x}"
            letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            ac.adsb_callsign = f"{''.join(random.choices(letters, k=3))}{random.randint(100, 9999)}"
            # Also do a speed bump to trigger existing anomaly detectors
            ac.speed_km_s = random.uniform(0.35, 0.50)

    def _aircraft_in_detection_cone(self, ac: SimulatedAircraft, node: NodeConfig) -> bool:
        """Check if aircraft is within the node's detection cone.

        Range is limited on *bistatic* range when the node declares one — the
        sum of both legs minus the baseline, which is what the delay actually
        measures and what sets received power.  A monostatic RX-distance limit
        ignores the TX leg entirely, so it accepts targets far behind the
        transmitter and rejects near ones on a long baseline.  Nodes without
        max_bistatic_range_km keep the monostatic rule so real hardware is
        unaffected.
        """
        if node.max_bistatic_range_km is not None:
            rx_alt_km = node.rx_alt_ft * 0.3048 / 1000.0
            tx_alt_km = node.tx_alt_ft * 0.3048 / 1000.0
            target_enu = _lla_to_enu(
                ac.lat,
                ac.lon,
                ac.alt_km,
                node.rx_lat,
                node.rx_lon,
                rx_alt_km,
            )
            tx_enu = _lla_to_enu(
                node.tx_lat,
                node.tx_lon,
                tx_alt_km,
                node.rx_lat,
                node.rx_lon,
                rx_alt_km,
            )
            # _bistatic_delay returns the differential range in µs; multiply
            # back by c to compare in km.  Reused rather than recomputing the
            # two legs so the gate and the emitted delay can never disagree.
            diff_range_km = _bistatic_delay(target_enu, tx_enu, (0.0, 0.0, 0.0)) * C_KM_US
            if diff_range_km > node.max_bistatic_range_km:
                return False
        else:
            dist = _haversine_km(node.rx_lat, node.rx_lon, ac.lat, ac.lon)
            if dist > node.max_range_km:
                return False

        bearing = _bearing_deg(node.rx_lat, node.rx_lon, ac.lat, ac.lon)
        angle_diff = abs((bearing - node.beam_azimuth_deg + 180) % 360 - 180)
        return angle_diff <= node.beam_width_deg / 2

    def generate_detections_for_node(self, node_id: str, timestamp_ms: int) -> dict:
        """Generate a detection frame for a specific node.

        Returns a frame dict: {timestamp, delay[], doppler[], snr[], adsb?[]}
        Only includes aircraft within the node's detection cone.
        """
        node = self.nodes.get(node_id)
        if node is None:
            return {"timestamp": timestamp_ms, "delay": [], "doppler": [], "snr": []}

        rx_alt_km = node.rx_alt_ft * 0.3048 / 1000.0
        tx_alt_km = node.tx_alt_ft * 0.3048 / 1000.0
        rx_enu = (0.0, 0.0, 0.0)
        tx_enu = _lla_to_enu(
            node.tx_lat,
            node.tx_lon,
            tx_alt_km,
            node.rx_lat,
            node.rx_lon,
            rx_alt_km,
        )

        delays = []
        dopplers = []
        snrs = []
        adsb_list = []
        has_any_adsb = False

        for ac in self.aircraft:
            if not self._aircraft_in_detection_cone(ac, node):
                continue

            # Convert aircraft to ENU relative to this node's RX
            target_enu = _lla_to_enu(
                ac.lat,
                ac.lon,
                ac.alt_km,
                node.rx_lat,
                node.rx_lon,
                rx_alt_km,
            )
            # Velocity already in km/s ENU
            vel_enu = (ac.vel_east, ac.vel_north, ac.vel_up)

            delay = _bistatic_delay(target_enu, tx_enu, rx_enu)
            doppler = _bistatic_doppler(target_enu, vel_enu, tx_enu, rx_enu, node.fc_hz)

            # SNR depends on distance — compute before noise/miss decisions
            dist = _norm(target_enu)
            base_snr = 25 - 10 * math.log10(max(dist, 1))
            snr = max(base_snr + random.gauss(0, 2), 4.0)

            # SNR-dependent missed detection: weaker signals are harder to detect.
            # P(miss) rises from ~5% at SNR 20+ to ~40% near detection threshold SNR 5.
            p_miss = max(0.0, min(0.5, 0.05 + 0.35 * (1 - (snr - 4) / 18)))
            if random.random() < p_miss:
                continue

            # Add measurement noise — scales inversely with SNR.
            # 0.1 µs base = GPS-disciplined SDR with proper FM cross-correlation
            # (achievable at SNR > 15 dB with 150 kHz FM bandwidth).
            noise_scale = max(0.5, 2.0 - (snr - 4) / 18)
            delay += random.gauss(0, 0.1 * noise_scale)
            doppler += random.gauss(0, 2.0 * noise_scale)

            delays.append(round(delay, 2))
            dopplers.append(round(doppler, 2))
            snrs.append(round(snr, 2))

            # ADS-B entry.  A silent transponder gets exactly what a dark
            # aircraft gets — a None slot — so the outage is indistinguishable
            # downstream from "this echo carries no ADS-B tag".
            if ac.has_adsb and not ac.adsb_silent:
                has_any_adsb = True
                speed_ms = ac.speed_km_s * 1000
                # GPS spoof: ADS-B reports frozen pre-spoof position while
                # radar delay/Doppler sees the real (moving) aircraft.
                if ac.anomaly_event == "spoof" and ac.anomaly_fired:
                    report_lat = ac._pre_spoof_lat
                    report_lon = ac._pre_spoof_lon
                else:
                    report_lat = ac.lat
                    report_lon = ac.lon
                adsb_list.append(
                    {
                        "hex": ac.adsb_hex,
                        "flight": ac.adsb_callsign,
                        "lat": round(report_lat, 5),
                        "lon": round(report_lon, 5),
                        "alt_baro": round(ac.alt_km * 1000 / 0.3048),
                        "gs": round(speed_ms * 1.94384, 1),
                        "track": round(ac.heading_deg, 1),
                    }
                )
            else:
                adsb_list.append(None)

        # Clutter/false alarm detections — Poisson-distributed count (mean ~2)
        # with range-biased delays (more clutter at shorter ranges).
        n_clutter = min(int(random.expovariate(0.5)), 6)
        for _ in range(n_clutter):
            # Bias toward shorter delays (near-range ground clutter)
            clutter_delay = random.expovariate(0.05)
            clutter_delay = min(clutter_delay, 60.0)
            delays.append(round(clutter_delay, 2))
            dopplers.append(round(random.uniform(node.doppler_min, node.doppler_max), 2))
            snrs.append(round(random.uniform(4, 8), 2))
            adsb_list.append(None)

        frame = {
            "timestamp": timestamp_ms,
            "delay": delays,
            "doppler": dopplers,
            "snr": snrs,
        }

        if has_any_adsb:
            frame["adsb"] = adsb_list

        return frame

    def generate_all_frames(self, timestamp_ms: int) -> dict[str, dict]:
        """Generate detection frames for all registered nodes."""
        return {node_id: self.generate_detections_for_node(node_id, timestamp_ms) for node_id in self.nodes}

    def get_aircraft_summary(self) -> list[dict]:
        """Return summary of all current aircraft (for debugging/monitoring)."""
        return [
            {
                "id": ac.object_id,
                "lat": round(ac.lat, 5),
                "lon": round(ac.lon, 5),
                "alt_km": round(ac.alt_km, 2),
                "heading": round(ac.heading_deg, 1),
                "speed_ms": round(ac.speed_km_s * 1000, 1),
                "has_adsb": ac.has_adsb,
                "is_anomalous": ac.is_anomalous,
                "object_type": ac.object_type,
                "adsb_hex": ac.adsb_hex,
                "adsb_callsign": ac.adsb_callsign,
                "adsb_silent": ac.adsb_silent,
                "anomaly_event": ac.anomaly_event,
                "source": ac.source,
            }
            for ac in self.aircraft
        ]

    # ── Live ADS-B seeding ───────────────────────────────────────────────────

    def synthetic_count(self) -> int:
        """Aircraft spawned by this world's own planner (excludes live)."""
        return sum(1 for ac in self.aircraft if ac.source != "live")

    def live_role_is_dark(self, hex_code: str) -> bool:
        """Whether a feed aircraft is cast as dark at the current frac_live_dark.

        A stable hash of the hex against the fraction rather than a random
        roll: the same aircraft gets the same answer on every ingest, so a
        knob change flips exactly the aircraft the new fraction implies
        (raising it only ever turns more aircraft dark, lowering it only
        ever restores transponders) and nothing flickers between polls.
        """
        if self.frac_live_dark <= 0.0:
            return False
        if self.frac_live_dark >= 1.0:
            return True
        u = int(hashlib.sha256(hex_code.encode()).hexdigest()[:8], 16) / 0x100000000
        return u < self.frac_live_dark

    def _cast_live_role(self, ac: SimulatedAircraft) -> None:
        """Apply the dark/ADS-B cast to one live aircraft.

        Dark means the world mirrors the aircraft WITHOUT its transponder:
        has_adsb off and adsb_hex None, so the node frames tag no echo with
        it, the ADS-B push omits it, and the ground truth keys it by its
        ``live-<hex>`` object id — exactly the shape of a synthetic dark
        spawn, with a real trajectory underneath.
        """
        dark = self.live_role_is_dark(ac.live_hex or "")
        ac.has_adsb = not dark
        ac.adsb_hex = None if dark else ac.live_hex

    def set_frac_live_dark(self, value: float) -> int:
        """Set the live dark share and re-cast every live aircraft in the air.

        Returns how many live aircraft are dark afterwards.
        """
        self.frac_live_dark = min(1.0, max(0.0, float(value)))
        for ac in self.live_aircraft.values():
            self._cast_live_role(ac)
        return sum(1 for ac in self.live_aircraft.values() if not ac.has_adsb)

    def ingest_live_aircraft(self, rows: list[dict], now_wall: float | None = None) -> dict:
        """Merge one feed poll (live_adsb.parse_point_response rows) into the world.

        A row's position is advanced from its capture time to ``now_wall``
        along its reported velocity before it is applied, so a 10 s old fix
        does not drag the echo 2 km behind the real aircraft.  Known hexes
        are updated in place (their ids, and so their ground-truth keys, are
        stable); new ones join as ``live-<hex>`` aircraft.  A missed poll is a
        coast and not a blink: absence never removes an aircraft, that is
        _expire_live_aircraft's job on the world clock.

        The feed is not trusted blindly, because it demonstrably lies:
          * an ``on_ground`` row RETIRES a live aircraft immediately — the
            only positive "it landed" signal there is, and the aircraft is
            otherwise flown on into a 146 s ghost track;
          * a row whose fix is already older than ``live_stale_s`` is skipped
            outright — a fix that old is no evidence the aircraft is where
            extrapolating it would put it;
          * an altitude that disagrees with the coasted one by more than
            _LIVE_ALT_JUMP_KM is held off (see the constant), while the rest
            of the row is still applied.
        """
        now_wall = time.time() if now_wall is None else now_wall
        created = updated = landed = alt_rejected = 0
        for row in rows:
            hex_code = str(row.get("hex") or "").strip().lower()
            lat = row.get("lat")
            lon = row.get("lon")
            if not hex_code or not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
                continue
            captured_at = row.get("captured_at")
            raw_age = now_wall - captured_at if isinstance(captured_at, (int, float)) else 0.0
            if raw_age > self.live_stale_s:
                continue
            if row.get("on_ground"):
                # Landed: drop it the way _expire_live_aircraft would, now,
                # instead of dead-reckoning a parked aircraft across the map
                # for another staleness window.  An unknown hex is never
                # created from a ground row — traffic on a taxiway is not a
                # radar target the fleet should be echoing.
                ac = self.live_aircraft.pop(hex_code, None)
                if ac is not None:
                    self.aircraft = [a for a in self.aircraft if a is not ac]
                    landed += 1
                continue
            speed_km_s = max(0.0, float(row.get("gs") or 0.0)) * _KNOTS_TO_KM_S
            heading = float(row.get("track") or 0.0) % 360.0
            alt_km = max(0.05, float(row.get("alt_baro") or 0.0) * _FT_TO_KM)
            vel_up = float(row.get("baro_rate") or 0.0) * _FT_TO_KM / 60.0
            heading_rad = math.radians(heading)
            vel_east = speed_km_s * math.sin(heading_rad)
            vel_north = speed_km_s * math.cos(heading_rad)
            age = min(max(raw_age, 0.0), self.live_stale_s)
            lat = float(lat) + (vel_north / R_EARTH) * (180 / math.pi) * age
            lon = float(lon) + (vel_east / (R_EARTH * math.cos(math.radians(float(lat))))) * (180 / math.pi) * age
            alt_km = max(0.05, alt_km + vel_up * age)
            flight = str(row.get("flight") or "").strip() or None

            ac = self.live_aircraft.get(hex_code)
            if ac is None:
                ac = SimulatedAircraft(
                    object_id=f"live-{hex_code}",
                    lat=lat,
                    lon=lon,
                    alt_km=alt_km,
                    vel_east=vel_east,
                    vel_north=vel_north,
                    vel_up=vel_up,
                    heading_deg=heading,
                    speed_km_s=speed_km_s,
                    base_speed_km_s=speed_km_s,
                    object_type="aircraft",
                    adsb_callsign=flight,
                    created_at=self._time,
                    lifetime_s=float("inf"),
                    waypoints=[],
                    waypoint_idx=0,
                    sim_now_s=self._time,
                    source="live",
                    live_hex=hex_code,
                )
                self._cast_live_role(ac)
                # Same outage roll a synthetic transponder aircraft gets at
                # spawn: frac_adsb_outage is a fraction of the ADS-B
                # population, and live ADS-B aircraft are part of it.
                ac.adsb_outage_start_s, ac.adsb_outage_end_s = self._roll_adsb_outage(ac.has_adsb, _ADSB_OUTAGE_START_S)
                self.live_aircraft[hex_code] = ac
                self.aircraft.append(ac)
                created += 1
            else:
                ac.lat = lat
                ac.lon = lon
                # Altitude jump guard.  ac.alt_km has been coasted to the world
                # clock by _dead_reckon and the row's alt_km extrapolated to
                # now_wall by vel_up * age, so the two are directly comparable.
                # A disagreement past _LIVE_ALT_JUMP_KM is the feed serving
                # someone else's cruise level: keep the coasted altitude but
                # apply the rest of the row — position, velocities and
                # baro_rate were right in every corrupt row observed.  The one
                # escape is a SELF-CONSISTENT run of rejections (every reading
                # within _LIVE_ALT_JUMP_KM of the one before) lasting
                # _LIVE_ALT_ADOPT_S: that is the altitude we HOLD being wrong,
                # not the feed.  A reading that disagrees with the previous
                # REJECTED one restarts the run instead, because a feed
                # flipping between values — N6389R, 97 s over four corrupt
                # altitudes — is never an aircraft changing level.
                if abs(alt_km - ac.alt_km) > _LIVE_ALT_JUMP_KM:
                    if ac.live_alt_reject_s is None or abs(alt_km - ac.live_alt_reject_km) > _LIVE_ALT_JUMP_KM:
                        ac.live_alt_reject_s = self._time
                        ac.live_alt_reject_km = alt_km
                        alt_rejected += 1
                    elif self._time - ac.live_alt_reject_s >= _LIVE_ALT_ADOPT_S:
                        ac.alt_km = alt_km
                        ac.live_alt_reject_s = None
                        ac.live_alt_reject_km = 0.0
                    else:
                        ac.live_alt_reject_km = alt_km
                        alt_rejected += 1
                else:
                    ac.alt_km = alt_km
                    ac.live_alt_reject_s = None
                    ac.live_alt_reject_km = 0.0
                ac.vel_east = vel_east
                ac.vel_north = vel_north
                ac.vel_up = vel_up
                ac.heading_deg = heading
                ac.speed_km_s = speed_km_s
                ac.base_speed_km_s = speed_km_s
                if flight:
                    ac.adsb_callsign = flight
                self._cast_live_role(ac)
                updated += 1
            # The FIX time on the world clock, not "now": expiry then measures
            # the age of the aircraft's newest fix rather than the age of the
            # last poll that happened to mention it.
            ac.live_seen_s = self._time - age
            ac.sim_now_s = self._time
        return {
            "created": created,
            "updated": updated,
            "landed": landed,
            "alt_rejected": alt_rejected,
            "live": len(self.live_aircraft),
        }

    def _expire_live_aircraft(self) -> int:
        """Drop live aircraft whose newest feed FIX is older than live_stale_s.

        live_seen_s is the fix's capture time on the world clock (set by
        ingest_live_aircraft), so an aircraft that stops transmitting leaves
        live_stale_s after its LAST FIX rather than live_stale_s after the feed
        stops echoing that fix — the difference between ~115 s and ≤ 60 s of
        ghost track for an aircraft the feed keeps repeating.
        """
        stale = [h for h, ac in self.live_aircraft.items() if self._time - ac.live_seen_s > self.live_stale_s]
        if not stale:
            return 0
        for h in stale:
            del self.live_aircraft[h]
        gone = {f"live-{h}" for h in stale}
        self.aircraft = [ac for ac in self.aircraft if ac.object_id not in gone]
        return len(stale)

    def clear_live_aircraft(self) -> int:
        """Remove every live aircraft (the feed was switched off)."""
        n = len(self.live_aircraft)
        if n:
            self.live_aircraft.clear()
            self.aircraft = [ac for ac in self.aircraft if ac.source != "live"]
        return n

    def _dead_reckon(self, ac: SimulatedAircraft, dt: float) -> None:
        """Coast a live aircraft on its last reported velocity."""
        ac.lat += (ac.vel_north / R_EARTH) * (180 / math.pi) * dt
        ac.lon += (ac.vel_east / (R_EARTH * math.cos(math.radians(ac.lat)))) * (180 / math.pi) * dt
        ac.alt_km = max(0.05, ac.alt_km + ac.vel_up * dt)

    # ── ML Training Data Batch Export ────────────────────────────────────────

    def generate_training_batch(self, n_frames: int, dt: float = 0.5, mode: str = "adsb") -> list[dict]:
        """Generate a batch of labeled training frames for ML pipelines.

        Each output record contains:
          - node_id, timestamp
          - delay[], doppler[], snr[] (measurements)
          - ground_truth[]: per-detection truth with object_id, lat, lon, alt,
            heading, speed, has_adsb, is_anomalous
          - adsb[] (if mode includes ADS-B)

        This runs the simulation fast (no sleep) to produce data quickly.

        Args:
            n_frames: Total number of frames to generate (across all nodes).
            dt: Simulation time step in seconds.
            mode: "detection", "adsb", or "anomalous".

        Returns:
            List of labeled frame dicts.
        """
        records = []
        frames_per_step = max(len(self.nodes), 1)
        n_steps = max(1, n_frames // frames_per_step)

        for _step_i in range(n_steps):
            self.step(dt, mode=mode)
            timestamp_ms = int((self._time) * 1000)

            for node_id, node in self.nodes.items():
                rx_alt_km = node.rx_alt_ft * 0.3048 / 1000.0
                tx_alt_km = node.tx_alt_ft * 0.3048 / 1000.0
                rx_enu = (0.0, 0.0, 0.0)
                tx_enu = _lla_to_enu(
                    node.tx_lat,
                    node.tx_lon,
                    tx_alt_km,
                    node.rx_lat,
                    node.rx_lon,
                    rx_alt_km,
                )

                delays = []
                dopplers = []
                snrs = []
                ground_truth = []
                adsb_list = []

                for ac in self.aircraft:
                    if not self._aircraft_in_detection_cone(ac, node):
                        continue

                    target_enu = _lla_to_enu(
                        ac.lat,
                        ac.lon,
                        ac.alt_km,
                        node.rx_lat,
                        node.rx_lon,
                        rx_alt_km,
                    )
                    vel_enu = (ac.vel_east, ac.vel_north, ac.vel_up)

                    delay = _bistatic_delay(target_enu, tx_enu, rx_enu)
                    doppler = _bistatic_doppler(target_enu, vel_enu, tx_enu, rx_enu, node.fc_hz)

                    delay_noisy = delay + random.gauss(0, 0.3)
                    doppler_noisy = doppler + random.gauss(0, 2.0)

                    dist = _norm(target_enu)
                    base_snr = 25 - 10 * math.log10(max(dist, 1))
                    snr = max(base_snr + random.gauss(0, 2), 4.0)

                    delays.append(round(delay_noisy, 2))
                    dopplers.append(round(doppler_noisy, 2))
                    snrs.append(round(snr, 2))

                    ground_truth.append(
                        {
                            "object_id": ac.object_id,
                            "lat": round(ac.lat, 5),
                            "lon": round(ac.lon, 5),
                            "alt_km": round(ac.alt_km, 2),
                            "heading_deg": round(ac.heading_deg, 1),
                            "speed_ms": round(ac.speed_km_s * 1000, 1),
                            "has_adsb": ac.has_adsb,
                            "is_anomalous": ac.is_anomalous,
                            "anomaly_event": ac.anomaly_event if ac.anomaly_fired else None,
                            "delay_true": round(delay, 4),
                            "doppler_true": round(doppler, 4),
                            "is_clutter": False,
                        }
                    )

                    if ac.has_adsb:
                        if ac.anomaly_event == "spoof" and ac.anomaly_fired:
                            report_lat = ac._pre_spoof_lat
                            report_lon = ac._pre_spoof_lon
                        else:
                            report_lat = ac.lat
                            report_lon = ac.lon
                        adsb_list.append(
                            {
                                "hex": ac.adsb_hex,
                                "flight": ac.adsb_callsign,
                                "lat": round(report_lat, 5),
                                "lon": round(report_lon, 5),
                                "alt_baro": round(ac.alt_km * 1000 / 0.3048),
                                "gs": round(ac.speed_km_s * 1000 * 1.94384, 1),
                                "track": round(ac.heading_deg, 1),
                            }
                        )
                    else:
                        adsb_list.append(None)

                # Add clutter with ground truth label
                n_clutter = random.randint(0, 3)
                for _ in range(n_clutter):
                    delays.append(round(random.uniform(0, 60), 2))
                    dopplers.append(round(random.uniform(node.doppler_min, node.doppler_max), 2))
                    snrs.append(round(random.uniform(4, 7), 2))
                    ground_truth.append(
                        {
                            "object_id": None,
                            "is_clutter": True,
                            "is_anomalous": False,
                            "has_adsb": False,
                        }
                    )
                    adsb_list.append(None)

                record = {
                    "node_id": node_id,
                    "timestamp": timestamp_ms,
                    "delay": delays,
                    "doppler": dopplers,
                    "snr": snrs,
                    "ground_truth": ground_truth,
                }
                if mode in ("adsb", "anomalous"):
                    record["adsb"] = adsb_list

                records.append(record)

        return records

    def export_training_ndjson(self, path: str, n_frames: int = 10000, dt: float = 0.5, mode: str = "adsb"):
        """Export training data as newline-delimited JSON file.

        Fast bulk export for ML training pipelines.
        """
        records = self.generate_training_batch(n_frames, dt, mode)
        with open(path, "w") as f:
            for rec in records:
                f.write(json.dumps(rec) + "\n")
        return len(records)
