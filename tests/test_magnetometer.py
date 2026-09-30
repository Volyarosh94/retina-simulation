"""Simulated magnetometers: physics, placement, readings, truth, and config."""

import math
import random
import statistics
from dataclasses import dataclass

import pytest

from retina_simulation import magnetometer as mag
from retina_simulation.magnetometer import MagnetometerArray, MagnetometerSettings, build_magnetometer_push_payload
from retina_simulation.world import SimulationWorld

T0 = 1_790_769_600.0  # 2026-09-30T12:00:00Z


def radar(node_id, lat, lon, alt_ft=900.0):
    return {"node_id": node_id, "rx_lat": lat, "rx_lon": lon, "rx_alt_ft": alt_ft}


FLEET = [
    radar("synth-A", 34.85, -82.39),
    radar("synth-A2", 34.85, -82.39),  # same roof as synth-A (a dual site)
    radar("synth-B", 35.10, -82.10),
    radar("synth-C", 34.60, -82.70),
    radar("synth-D", 34.95, -82.30),
    radar("synth-E", 34.70, -82.00),
]


def array(n=3, placement="spread", seed=1, **settings):
    return MagnetometerArray(
        node_configs=FLEET,
        center=(34.85, -82.39),
        seed=seed,
        settings=MagnetometerSettings(n_nodes=n, placement=placement, **settings),
    )


@dataclass
class Obj:
    object_id: str
    lat: float
    lon: float
    alt_km: float


class TestPhysics:
    def test_dipole_task_numbers(self):
        m = (1e9, 0.0, 0.0)
        assert math.dist(mag.dipole_field_nt(m, (1000, 0, 0)), (0, 0, 0)) == pytest.approx(200.0)
        assert math.dist(mag.dipole_field_nt(m, (5000, 0, 0)), (0, 0, 0)) == pytest.approx(1.6)
        assert math.dist(mag.dipole_field_nt(m, (0, 1000, 0)), (0, 0, 0)) == pytest.approx(100.0)

    def test_background_is_wmm2025(self):
        x, y, z = mag.main_field_nt(34.85, -82.39, 300.0, datetime_2026_09_30())
        # NOAA's calculator, WMM-2025, the same point and day.
        assert (x, y, z) == pytest.approx((22397.7, -2760.5, 43002.2), abs=0.5)

    def test_rm3100_tables(self):
        assert mag.rm3100_noise_nt(200) == pytest.approx(15.0)
        assert mag.rm3100_noise_nt(50) == pytest.approx(30.0)
        assert mag.rm3100_gain_counts_per_ut(200) == pytest.approx(74.92)

    def test_daily_variation_is_quiet_at_night_and_tens_of_nt_by_day(self):
        local_midnight = T0 - 12 * 3600 + 82.39 / 15 * 3600
        night = mag.sq_variation_nt(local_midnight, -82.39, variability=0)
        noon = mag.sq_variation_nt(local_midnight + 12 * 3600, -82.39, variability=0)
        assert max(abs(v) for v in night) < 6
        assert noon[0] < -10  # X dips near local noon

    def test_storm_is_zero_before_it_starts(self):
        storm = mag.Storm(start=T0)
        assert storm(T0 - 1) == (0.0, 0.0, 0.0)
        assert storm(T0 + 6 * 3600)[0] < -80


def datetime_2026_09_30():
    return 1_790_726_400.0


class TestPlacement:
    def test_one_magnetometer_per_roof(self):
        sites = mag.radar_sites(FLEET)
        assert len(sites) == 5
        assert {s["node_id"] for s in sites} == {"synth-A", "synth-B", "synth-C", "synth-D", "synth-E"}

    def test_every_magnetometer_sits_on_a_radar_site(self):
        a = array(n=5)
        positions = {(round(c["rx_lat"], 5), round(c["rx_lon"], 5)): c["node_id"] for c in FLEET}
        for node in a.nodes:
            assert (round(node.lat, 5), round(node.lon, 5)) in positions
            assert node.node_id == f"mag-{node.host_node_id}"

    def test_count_is_capped_by_sites(self):
        assert len(array(n=50).nodes) == 5
        assert array(n=0).nodes == []

    def test_spread_starts_central_and_reaches_out(self):
        chosen = [n.host_node_id for n in array(n=2, placement="spread").nodes]
        assert chosen[0] == "synth-A"  # nearest the centre
        assert chosen[1] in ("synth-B", "synth-C", "synth-E")  # then the farthest

    def test_core_is_nearest_the_centre(self):
        assert [n.host_node_id for n in array(n=2, placement="core").nodes] == ["synth-A", "synth-D"]

    def test_random_is_seeded(self):
        a = [n.host_node_id for n in array(n=3, placement="random", seed=4).nodes]
        b = [n.host_node_id for n in array(n=3, placement="random", seed=4).nodes]
        assert a == b

    def test_placement_does_not_touch_the_worlds_random_state(self):
        state = random.getstate()
        a = array(n=5, placement="random")
        a.sample([], T0)
        assert random.getstate() == state


class TestReadings:
    def test_quiet_reading_is_background_plus_noise(self):
        a = array(n=1)
        node = a.nodes[0]
        readings = [a.sample([], T0 + k)[0]["reading"] for k in range(1500)]
        background = mag.main_field_nt(node.lat, node.lon, node.alt_m, T0)
        for axis in range(3):
            values = [r[axis] for r in readings]
            expected = background[axis] + node.crust_nt[axis] + mag.sq_variation_nt(T0 + 750, -82.39, seed=1)[axis]
            assert statistics.fmean(values) == pytest.approx(expected, abs=8)
            # Datasheet noise plus 13.35 nT quantisation steps.
            assert statistics.pstdev(values) == pytest.approx(math.hypot(15.0, 13.35 / math.sqrt(12)), rel=0.08)

    def test_cycle_count_changes_the_noise(self):
        a = array(n=1, cycle_count=800)
        values = [a.sample([], T0 + k)[0]["reading"][0] for k in range(1500)]
        assert statistics.pstdev(values) == pytest.approx(
            math.hypot(mag.rm3100_noise_nt(800), 3.39 / math.sqrt(12)), rel=0.1
        )

    def test_uap_overhead_adds_the_dipole_field_and_truth(self):
        a = array(n=1)
        node = a.nodes[0]
        overhead = Obj("uap-1", node.lat, node.lon, node.alt_m / 1000.0 + 0.5)
        quiet = a.sample([], T0)[0]
        with_uap = a.sample([overhead], T0)[0]
        m = a.uap_moment("uap-1")
        expected = mag.dipole_field_nt(m, (0.0, 0.0, 500.0))
        assert with_uap["uap_nt"] == pytest.approx(math.dist(expected, (0, 0, 0)), rel=1e-6)
        # 1e9 A·m² at 500 m: 800 nT on the dipole's equator, 1600 nT on its axis.
        assert 800 - 1e-6 <= with_uap["uap_nt"] <= 1600 + 1e-6
        assert with_uap["nearest_uap"] == "uap-1" and with_uap["nearest_uap_km"] == pytest.approx(0.5, abs=1e-6)
        assert quiet["uap_nt"] == 0.0 and quiet["nearest_uap"] is None

    def test_far_uaps_are_culled_but_still_reported_as_nearest(self):
        a = array(n=1)
        far = Obj("uap-far", 36.0, -82.39, 1.0)  # ~130 km away
        r = a.sample([far], T0)[0]
        assert r["uap_nt"] == 0.0
        assert r["nearest_uap"] == "uap-far" and r["nearest_uap_km"] > 100

    def test_each_uap_keeps_its_own_dipole(self):
        a = array(n=1)
        assert a.uap_moment("uap-1") == a.uap_moment("uap-1")
        assert a.uap_moment("uap-1") != a.uap_moment("uap-2")
        assert math.dist(a.uap_moment("uap-1"), (0, 0, 0)) == pytest.approx(1e9)

    def test_same_seed_same_readings(self):
        first = [r["reading"] for k in range(5) for r in array(n=3, seed=9).sample([], T0 + k)]
        again = [r["reading"] for k in range(5) for r in array(n=3, seed=9).sample([], T0 + k)]
        other = [r["reading"] for k in range(5) for r in array(n=3, seed=10).sample([], T0 + k)]
        assert first == again and first != other

    def test_storm_is_common_to_every_node(self):
        a = array(n=3)
        a.apply(MagnetometerSettings(n_nodes=3, storm=True), T0 - 3 * 3600)
        quiet = array(n=3)
        with_storm = a.sample([], T0)
        without = quiet.sample([], T0)
        shifts = [w["reading"][0] - q["reading"][0] for w, q in zip(with_storm, without)]
        assert all(s < -40 for s in shifts)
        assert max(shifts) - min(shifts) < 60  # the same storm, give or take noise


class TestSettings:
    def test_from_config_takes_valid_keys_and_ignores_bad_ones(self):
        s = MagnetometerSettings.from_config(
            {"mag_enabled": True, "mag_n_nodes": 12, "mag_placement": "core", "mag_cycle_count": 400, "mag_storm": True}
        )
        assert (s.n_nodes, s.placement, s.cycle_count, s.storm) == (12, "core", 400, True)
        kept = MagnetometerSettings.from_config(
            {"mag_n_nodes": "many", "mag_placement": "moon", "mag_cycle_count": 5}, s
        )
        assert (kept.n_nodes, kept.placement, kept.cycle_count) == (12, "core", 400)

    def test_apply_replaces_only_when_the_node_set_changes(self):
        a = array(n=2)
        assert not a.apply(MagnetometerSettings(n_nodes=2, cycle_count=400), T0)
        assert a.apply(MagnetometerSettings(n_nodes=4), T0)
        assert len(a.nodes) == 4
        assert a.apply(MagnetometerSettings(enabled=False, n_nodes=4), T0)
        assert a.nodes == []


class TestPayload:
    def test_shape(self):
        a = array(n=2)
        batches = [(int(T0 * 1000), a.sample([], T0)), (int(T0 * 1000) + 1000, a.sample([], T0 + 1))]
        payload = build_magnetometer_push_payload(a, batches)
        assert payload["sensor"] == {"model": "RM3100", "cycle_count": 200, "noise_nt": 15.0, "lsb_nt": 13.348}
        assert payload["uap_moment_am2"] == 1e9 and payload["storm"] is False
        node = payload["nodes"][0]
        assert node["node_id"].startswith("mag-synth-") and node["host_node_id"].startswith("synth-")
        assert len(node["samples"]) == 2 and len(node["samples"][0]) == 4
        assert node["truth"][0][:3] == [int(T0 * 1000), 0.0, None]

    def test_every_current_node_is_listed_even_without_samples(self):
        a = array(n=3)
        assert len(build_magnetometer_push_payload(a, [])["nodes"]) == 3


class TestFlyby:
    def test_straight_pass_at_the_requested_closest_approach(self):
        world = SimulationWorld(center_lat=34.85, center_lon=-82.39)
        world.min_aircraft = world.max_aircraft = 0
        target = (34.85, -82.39)
        ac = world.inject_uap_flyby(
            *target, ground_alt_km=0.3, closest_approach_m=700, altitude_m=400, speed_mps=100, heading_deg=45, lead_s=60
        )
        assert ac.object_type == "anomalous" and ac.is_anomalous and not ac.has_adsb and ac.scripted
        state = random.getstate()
        closest = (math.inf, None)
        for _ in range(120):
            world.step(1.0)
            north = math.radians(ac.lat - target[0]) * 6371.0
            east = math.radians(ac.lon - target[1]) * 6371.0 * math.cos(math.radians(target[0]))
            d = math.hypot(north, east)
            if d < closest[0]:
                closest = (d, world.clock_s)
        assert closest[0] == pytest.approx(0.7, abs=0.01)
        assert closest[1] == pytest.approx(60.0, abs=1.0)
        assert ac.alt_km == pytest.approx(0.7)
        assert ac.heading_deg == 45.0  # never steered
        assert random.getstate() == state  # scripted flight draws nothing from the shared RNG
        world.step(1.0)
        assert ac not in world.aircraft  # retired a lead time after the pass

    def test_orchestrator_applies_a_fresh_request_once(self):
        from retina_simulation.orchestrator import FleetOrchestrator, apply_magnetometer_config

        orch = FleetOrchestrator(
            node_configs=[{**c, "tx_lat": 34.0, "tx_lon": -82.0, "tx_alt_ft": 1000, "fc_hz": 1e8} for c in FLEET],
            seed=1,
        )
        orch._build_world()
        request = {
            "seq": 3,
            "requested_at": T0,
            "node_id": orch.magnetometers.nodes[1].node_id,
            "closest_approach_m": 500,
        }
        ac = apply_magnetometer_config(orch, {"mag_flyby": request}, T0 + 1)
        assert ac is not None and ac in orch.world.aircraft
        assert apply_magnetometer_config(orch, {"mag_flyby": request}, T0 + 2) is None  # the same request
        stale = {**request, "seq": 4, "requested_at": T0 - 1000}
        assert apply_magnetometer_config(orch, {"mag_flyby": stale}, T0 + 3) is None

    def test_a_request_numbered_afresh_by_a_restarted_server_is_still_flown(self):
        from retina_simulation.orchestrator import FleetOrchestrator, apply_magnetometer_config

        orch = FleetOrchestrator(
            node_configs=[{**c, "tx_lat": 34.0, "tx_lon": -82.0, "tx_alt_ft": 1000, "fc_hz": 1e8} for c in FLEET],
            seed=1,
        )
        orch._build_world()
        first = {"seq": 5, "requested_at": T0, "closest_approach_m": 500}
        assert apply_magnetometer_config(orch, {"mag_flyby": first}, T0 + 1) is not None
        # The server restarted and counts from one again; the fleet did not.
        after_restart = {"seq": 1, "requested_at": T0 + 30, "closest_approach_m": 500}
        assert apply_magnetometer_config(orch, {"mag_flyby": after_restart}, T0 + 31) is not None

    def test_orchestrator_replaces_magnetometers_from_config(self):
        from retina_simulation.orchestrator import FleetOrchestrator, apply_magnetometer_config

        orch = FleetOrchestrator(
            node_configs=[{**c, "tx_lat": 34.0, "tx_lon": -82.0, "tx_alt_ft": 1000, "fc_hz": 1e8} for c in FLEET],
            seed=1,
            magnetometers=2,
        )
        orch._build_world()
        assert len(orch.magnetometers.nodes) == 2
        apply_magnetometer_config(orch, {"mag_n_nodes": 4, "mag_placement": "core"}, T0)
        assert len(orch.magnetometers.nodes) == 4
        apply_magnetometer_config(orch, {"mag_enabled": False}, T0)
        assert orch.magnetometers.nodes == []
