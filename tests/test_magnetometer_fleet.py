"""The fleet's side of the magnetometers: one reading per world step, the push
to the server, the config poll, and the seed the draws follow.

Nothing here is timed. Runs last a counted number of world steps, and the
fake server is slow or down for a counted number of steps, so a loaded
machine slows these tests down without changing what they see.
"""

import asyncio
import itertools
import json
import logging
import random
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import pytest

from retina_simulation import orchestrator as orchestrator_module
from retina_simulation.orchestrator import (
    FLYBY_MAX_AGE_S,
    MAGNETOMETER_ERROR_LOG_S,
    MAGNETOMETER_HOLD_STEPS,
    FleetOrchestrator,
    _poll_simulation_config,
    _push_magnetometers_live,
)

T0 = 1_790_769_600.0  # 2026-09-30T12:00:00Z

# The world steps every 10 ms here: the fleet's one second, scaled down.
TICK_S = 0.01
# A run that has not got what it waits for by now never will.
STEP_LIMIT = 2000

OK = (200, {"status": "ok"})

RADARS = [
    {
        "node_id": f"synth-{name}",
        "rx_lat": lat,
        "rx_lon": lon,
        "rx_alt_ft": 900.0,
        "tx_lat": 34.0,
        "tx_lon": -82.0,
        "tx_alt_ft": 1000.0,
        "fc_hz": 1e8,
        "region": "us",
        "tx_callsign": "WTST",
    }
    for name, lat, lon in (("A", 34.85, -82.39), ("B", 35.10, -82.10), ("C", 34.60, -82.70), ("D", 34.95, -82.30))
]


def fleet(magnetometers=3, seed=1, hold_steps=None):
    orch = FleetOrchestrator(node_configs=[dict(r) for r in RADARS], seed=seed, magnetometers=magnetometers)
    orch._build_world()
    # No traffic: these tests are about the magnetometers.
    orch.world.min_aircraft = orch.world.max_aircraft = 0
    if hold_steps is not None:
        # For tests of delivery rather than of the hold: room enough that a
        # starved machine cannot turn them into tests of the hold.
        orch.magnetometer_readings = deque(maxlen=hold_steps)
    return orch


def run(orch, steps, *alongside, until=lambda: True):
    """The fleet's own simulation loop, with the coroutines ``alongside`` it
    gathered as main_async gathers them, for ``steps`` world steps and then
    until ``until()`` holds. Returns the moment of each step."""
    stepped_at = []
    step = orch.world.step

    def counted_step(*args, **kwargs):
        step(*args, **kwargs)
        stepped_at.append(time.monotonic())
        if len(stepped_at) >= steps and (until() or len(stepped_at) >= STEP_LIMIT):
            orch._running = False

    orch.world.step = counted_step

    async def main():
        await asyncio.gather(orch.run_simulation_loop(tick_dt=TICK_S), *alongside)

    asyncio.run(main())
    return stepped_at


def push(orch, fake):
    return _push_magnetometers_live(orch, fake.url, interval_s=TICK_S)


def step_stamps(orch):
    return [truth["timestamp_ms"] for truth in orch.ground_truth]


def waiting(orch):
    return [t_ms for t_ms, _ in orch.magnetometer_readings]


def stamps(push_record, node_id):
    """The sample times one push carried for one magnetometer."""
    return [row[0] for node in push_record["body"]["nodes"] if node["node_id"] == node_id for row in node["samples"]]


def broken_sample(anomalous, t):
    raise ModuleNotFoundError("No module named 'pygeomag'")


class FakeServer:
    """The server's push route, on 127.0.0.1 and an ephemeral port.

    It keeps every push: its body, its status, and the moments it arrived
    and was answered. It answers once the world has taken
    ``steps_per_answer`` more steps (or stopped), with ``reply(i)`` for the
    i-th push: a status and a JSON body.
    """

    def __init__(self, orch, steps_per_answer=0, reply=lambda i: OK):
        self.pushes = []
        lock = threading.Lock()
        arrivals = itertools.count()
        fake = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                arrived = time.monotonic()
                with lock:
                    index = next(arrivals)
                steps = len(orch.ground_truth)
                # Counted in world steps: a loaded machine slows the steps and
                # this wait down together. Capped, so that a world that has
                # stopped stepping cannot hang the test.
                while (
                    len(orch.ground_truth) < steps + steps_per_answer
                    and orch._running
                    and time.monotonic() < arrived + 1.0
                ):
                    time.sleep(0.001)
                status, answer = reply(index)
                payload = json.dumps(answer).encode()
                with lock:
                    fake.pushes.append(
                        {
                            "index": index,
                            "status": status,
                            "body": body,
                            "arrived": arrived,
                            "answered": time.monotonic(),
                        }
                    )
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def log_message(self, *args):
                pass

        self._httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        self._thread.start()
        self.url = f"http://127.0.0.1:{self._httpd.server_address[1]}"

    def accepted(self):
        return [p for p in self.pushes if p["status"] == 200]

    def delivered(self, node_id):
        """Sample times the server took for one magnetometer, in order."""
        return [t_ms for p in self.accepted() for t_ms in stamps(p, node_id)]

    def close(self):
        self._httpd.shutdown()
        self._httpd.server_close()
        self._thread.join()


@pytest.fixture
def server():
    started = []

    def start(orch, **kwargs):
        fake = FakeServer(orch, **kwargs)
        started.append(fake)
        return fake

    yield start
    for fake in started:
        fake.close()


class TestReadingsPerStep:
    def test_one_reading_per_world_step_stamped_with_the_step(self, monkeypatch):
        orch = fleet()
        taken = []
        sample = orch.magnetometers.sample

        def spy(anomalous, t):
            taken.append(round(t * 1000))
            return sample(anomalous, t)

        monkeypatch.setattr(orch.magnetometers, "sample", spy)
        run(orch, MAGNETOMETER_HOLD_STEPS + 5)
        steps = step_stamps(orch)
        assert len(steps) == MAGNETOMETER_HOLD_STEPS + 5
        assert taken == steps
        # Nothing drained them here: the newest steps wait, and no more.
        assert waiting(orch) == steps[-MAGNETOMETER_HOLD_STEPS:]

    def test_readings_leave_the_radar_scene_of_a_seed_as_it_was(self):
        def one_step(magnetometers):
            random.seed(11)
            orch = FleetOrchestrator(
                node_configs=[dict(r) for r in RADARS],
                seed=1,
                magnetometers=magnetometers,
                min_aircraft=6,
                max_aircraft=6,
            )
            orch._build_world()
            orch.world.frac_anomalous = 0.5
            run(orch, 1)
            return orch, random.getstate()

        with_magnetometers, state = one_step(4)
        without, state_without = one_step(0)
        # Something to read, and it was read.
        assert any(ac.object_type == "anomalous" for ac in with_magnetometers.world.aircraft)
        assert len(with_magnetometers.magnetometer_readings) == 1 and not without.magnetometer_readings
        assert state == state_without
        assert with_magnetometers.world.get_aircraft_summary() == without.world.get_aircraft_summary()


class TestIsolation:
    def test_a_failing_reading_never_stops_the_radar_fleet(self, server, monkeypatch, caplog):
        orch = fleet()
        up = server(orch)
        monkeypatch.setattr(orch.magnetometers, "sample", broken_sample)
        with caplog.at_level(logging.WARNING, logger="fleet"):
            run(orch, 20, push(orch, up), until=lambda: up.accepted())
        steps = step_stamps(orch)
        assert len(steps) >= 20
        assert orch.get_stats()["magnetometer_errors"] == len(steps)
        failures = [r for r in caplog.records if r.getMessage().startswith("Magnetometer reading failed")]
        assert len(failures) == 1 and failures[0].exc_info
        # The pushes went on, listing the magnetometers with nothing to send.
        assert all(node["samples"] == [] for p in up.accepted() for node in p["body"]["nodes"])

    def test_a_failing_reading_is_logged_at_once_then_once_a_minute(self, monkeypatch, caplog):
        orch = fleet()
        monkeypatch.setattr(orch.magnetometers, "sample", broken_sample)
        with caplog.at_level(logging.WARNING, logger="fleet"):
            for k in range(150):
                orch._sample_magnetometers(int(T0 * 1000) + 1000 * k)
            # A minute on, the next failure is heard, with the count so far.
            orch._magnetometer_error_logged_at -= MAGNETOMETER_ERROR_LOG_S
            orch._sample_magnetometers(int(T0 * 1000) + 150_000)
        failures = [r for r in caplog.records if r.getMessage().startswith("Magnetometer reading failed")]
        assert [r.args[0] for r in failures] == [1, 151]
        assert [bool(r.exc_info) for r in failures] == [True, False]
        assert orch.get_stats()["magnetometer_errors"] == 151
        assert not orch.magnetometer_readings

    def test_the_stats_line_counts_failed_readings(self, monkeypatch, caplog):
        orch = fleet()
        monkeypatch.setattr(orch.magnetometers, "sample", broken_sample)
        # The STATS line comes every 10 s of the loop's clock: give the loop
        # one that moves 11 s whenever it is read.
        clock = itertools.count(start=0.0, step=11.0)
        monkeypatch.setattr(orchestrator_module, "time", SimpleNamespace(monotonic=lambda: next(clock), time=time.time))
        with caplog.at_level(logging.INFO, logger="fleet"):
            run(orch, 3)
        stats = [r.getMessage() for r in caplog.records if r.getMessage().startswith("STATS:")]
        assert "| 3 magnetometer errors |" in stats[-1]

    def test_a_main_field_model_that_cannot_load_is_reported_at_startup(self, monkeypatch, caplog):
        def missing(*args):
            raise ModuleNotFoundError("No module named 'pygeomag'")

        monkeypatch.setattr(orchestrator_module, "main_field_nt", missing)
        with caplog.at_level(logging.WARNING, logger="fleet"):
            orch = fleet()
        assert orch.world is not None and len(orch.magnetometers.nodes) == 3
        assert any("No module named 'pygeomag'" in r.getMessage() for r in caplog.records)


class TestPush:
    def test_a_slow_server_costs_no_reading(self, server):
        # The server answers each push only once the world has taken two more
        # steps. When the push loop took the readings itself, it missed every
        # step it spent waiting, and the server's detector skips any window
        # with a gap of over 1.5 steps.
        orch = fleet(hold_steps=1000)
        slow = server(orch, steps_per_answer=2)
        run(orch, 10, push(orch, slow), until=lambda: len(slow.accepted()) >= 3)
        node_id = orch.magnetometers.nodes[0].node_id
        # Every step exactly once, in order: taken by the server, or still
        # waiting for the next push when the run stopped.
        assert slow.delivered(node_id) + waiting(orch) == step_stamps(orch)
        # The pushes did fall behind the steps: each carried several.
        assert max(len(stamps(p, node_id)) for p in slow.accepted()) > 1

    def test_the_world_steps_while_a_push_waits_for_its_answer(self, server):
        # A push sent from the event loop itself, rather than from a thread,
        # would hold up every world step until the server answered.
        orch = fleet()
        slow = server(orch, steps_per_answer=2)
        stepped_at = run(orch, 5, push(orch, slow), until=lambda: len(slow.accepted()) >= 3)
        windows = [(p["arrived"], p["answered"]) for p in slow.accepted()]
        assert any(arrived < t < answered for arrived, answered in windows for t in stepped_at)

    @pytest.mark.parametrize("status", [408, 429, 500, 503])
    def test_a_push_that_failed_goes_again_with_the_next(self, server, status):
        # Timed out, told to slow down, or a server error: worth trying again.
        orch = fleet(hold_steps=1000)
        flaky = server(orch, reply=lambda i: (status, {"detail": "try again"}) if i < 2 else OK)
        run(orch, 3, push(orch, flaky), until=lambda: flaky.accepted())
        node_id = orch.magnetometers.nodes[0].node_id
        assert [p["status"] for p in flaky.pushes[:3]] == [status, status, 200]
        failed = [t_ms for p in flaky.pushes[:2] for t_ms in stamps(p, node_id)]
        assert failed and set(failed) <= set(flaky.delivered(node_id))
        assert flaky.delivered(node_id) + waiting(orch) == step_stamps(orch)

    def test_a_push_the_server_refuses_is_dropped_not_sent_again(self, server, caplog):
        orch = fleet(hold_steps=1000)
        node_id = orch.magnetometers.nodes[0].node_id
        # Reasons as the server words them: the field by index, then its
        # bounds. The first two are one reason at different indices.
        reasons = [
            "nodes[0].truth[2] uap_nt must be a number 0–1,000,000",
            "nodes[1].truth[0] uap_nt must be a number 0–1,000,000",
            "nodes[0].samples[0] time is 6.2 s ahead of the server clock (at most 5 s)",
        ]
        refusing = server(orch, reply=lambda i: (400, {"detail": reasons[i]}) if i < len(reasons) else OK)
        with caplog.at_level(logging.INFO, logger="fleet"):
            run(orch, 3, push(orch, refusing), until=lambda: len(refusing.accepted()) >= 2)
        refused = [t_ms for p in refusing.pushes if p["status"] == 400 for t_ms in stamps(p, node_id)]
        sent_after = [t_ms for p in refusing.pushes[len(reasons) :] for t_ms in stamps(p, node_id)]
        assert refused and not set(refused) & set(sent_after)
        # Every step exactly once: refused, taken, or still waiting.
        assert sorted(refused + refusing.delivered(node_id) + waiting(orch)) == step_stamps(orch)
        warnings = [r.getMessage() for r in caplog.records if r.levelno >= logging.WARNING]
        assert len(warnings) == 2
        assert warnings[0].startswith("Magnetometer push refused (HTTP 400)") and reasons[0] in warnings[0]
        assert reasons[2] in warnings[1]

    def test_an_outage_longer_than_the_hold_leaves_a_gap_not_a_backlog(self, server, caplog):
        orch = fleet()
        outage_steps = 3 * MAGNETOMETER_HOLD_STEPS
        down = server(orch, reply=lambda i: (503, {"detail": "down"}) if len(orch.ground_truth) < outage_steps else OK)
        with caplog.at_level(logging.INFO, logger="fleet"):
            run(orch, outage_steps, push(orch, down), until=lambda: down.accepted())
        steps = step_stamps(orch)
        node_id = orch.magnetometers.nodes[0].node_id
        # No backlog: no push ever carried more than the hold.
        assert max(len(stamps(p, node_id)) for p in down.pushes) == MAGNETOMETER_HOLD_STEPS
        # The first push through carried the steps the hold kept, one after
        # another, and the steps before them are a gap that is never filled.
        recovered = stamps(down.accepted()[0], node_id)
        first = steps.index(recovered[0])
        assert recovered == steps[first : first + MAGNETOMETER_HOLD_STEPS]
        assert first > 0 and not set(steps[:first]) & set(down.delivered(node_id))
        messages = [r.getMessage() for r in caplog.records]
        assert sum(m.startswith("Magnetometer push failing") for m in messages) == 1
        assert sum(m.startswith("Magnetometer push recovered") for m in messages) == 1


class FakeConfigResponse:
    def __init__(self, cfg):
        self._body = json.dumps(cfg).encode()

    def read(self):
        return self._body

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def poll(monkeypatch, orch, *configs, scene=None, observe=lambda: None):
    """Cycles of the fleet's config poll, serving ``configs`` one per cycle.
    Returns ``observe()`` as it stood before each fetch, which is what the
    cycle before it left."""
    import urllib.request

    served = list(configs)
    observed = []

    def fake_urlopen(url, context=None, timeout=None):
        observed.append(observe())
        cfg = served.pop(0)
        if not served:
            orch._running = False  # this cycle is the last
        return FakeConfigResponse(cfg)

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    orch._running = True
    asyncio.run(_poll_simulation_config(orch, "http://validation.example", interval_s=0.0, scene=scene))
    return observed


def flyby_request(requested_at, **request):
    return {"seq": 1, "requested_at": requested_at, **request}


def passes(orch):
    return [ac for ac in orch.world.aircraft if ac.scripted]


class TestConfigPoll:
    def test_a_flyby_is_flown_though_the_config_stamp_never_moves(self, monkeypatch):
        # A flyby is a command, not a setting: the server leaves the config's
        # stamp alone for one, so the fleet has to see it without it.
        orch = fleet()
        settings = {"_updated_at": 1.0, "mag_n_nodes": 3}
        request = flyby_request(time.time(), node_id=orch.magnetometers.nodes[0].node_id)
        poll(monkeypatch, orch, settings, *[{**settings, "mag_flyby": request}] * 4)
        assert len(passes(orch)) == 1  # flown, and once, over the four polls that served it

    def test_the_settings_keys_still_apply_only_when_the_stamp_moves(self, monkeypatch):
        orch = fleet()
        counts = poll(
            monkeypatch,
            orch,
            {"_updated_at": 1.0, "mag_n_nodes": 2},
            {"_updated_at": 1.0, "mag_n_nodes": 4},  # the same stamp: not a change
            {"_updated_at": 2.0, "mag_n_nodes": 4},
            observe=lambda: len(orch.magnetometers.nodes),
        )
        assert counts == [3, 2, 2] and len(orch.magnetometers.nodes) == 4

    def test_settings_apply_before_the_flyby_that_comes_with_them(self, monkeypatch):
        # A pass asked of a magnetometer that the same poll places.
        orch = fleet()
        placed = {n.host_node_id for n in orch.magnetometers.nodes}
        newcomer = next(r for r in RADARS if r["node_id"] not in placed)
        cfg = {
            "_updated_at": 1.0,
            "mag_n_nodes": 4,
            "mag_placement": "core",
            "mag_cycle_count": 400,
            "mag_flyby": flyby_request(time.time(), node_id=f"mag-{newcomer['node_id']}", lead_s=30),
        }
        poll(monkeypatch, orch, cfg)
        settings = orch.magnetometers.settings
        assert (len(orch.magnetometers.nodes), settings.placement, settings.cycle_count) == (4, "core", 400)
        assert [ac.track_origin for ac in passes(orch)] == [(newcomer["rx_lat"], newcomer["rx_lon"])]
        assert passes(orch)[0].object_type == "anomalous"

    def test_a_stale_request_is_never_flown_however_often_it_is_served(self, monkeypatch):
        orch = fleet()
        cfg = {"_updated_at": 1.0, "mag_flyby": flyby_request(time.time() - FLYBY_MAX_AGE_S - 1)}
        poll(monkeypatch, orch, cfg, cfg, cfg)
        assert passes(orch) == []

    def test_a_restarted_fleet_flies_the_last_request_again_while_it_is_fresh(self, monkeypatch):
        # The server goes on serving the last request, and a restarted fleet
        # remembers nothing: back within two minutes, it flies it again.
        cfg = {"_updated_at": 1.0, "mag_flyby": flyby_request(time.time())}
        before_restart, after_restart = fleet(), fleet()
        poll(monkeypatch, before_restart, cfg, cfg)
        poll(monkeypatch, after_restart, cfg, cfg)
        assert len(passes(before_restart)) == 1 and len(passes(after_restart)) == 1

    def test_the_servers_detector_keys_reach_the_fleet_and_change_nothing(self, monkeypatch):
        orch = fleet()
        before = (orch.magnetometers.settings, [n.node_id for n in orch.magnetometers.nodes])
        poll(monkeypatch, orch, {"_updated_at": 1.0, "mag_threshold_sigma": 4.0, "mag_common_mode": False})
        assert (orch.magnetometers.settings, [n.node_id for n in orch.magnetometers.nodes]) == before

    def test_a_magnetometer_settings_failure_leaves_the_rest_of_the_poll_alone(self, monkeypatch, caplog):
        orch = fleet()
        stops = []

        async def stop():
            stops.append(True)

        def broken(*args):
            raise RuntimeError("bad mag_placement")

        monkeypatch.setattr(orch, "stop", stop)
        monkeypatch.setattr(orchestrator_module, "apply_magnetometer_settings", broken)
        with caplog.at_level(logging.WARNING, logger="fleet"):
            poll(monkeypatch, orch, {"_updated_at": 1.0, "frac_anomalous": 0.3, "max_range_km": 75.0})
        assert orch.world.frac_anomalous == 0.3
        assert stops == [True]  # the scene change still restarts the fleet
        assert any(r.getMessage() == "Magnetometer settings not applied: bad mag_placement" for r in caplog.records)

    def test_a_flyby_that_fails_leaves_the_poll_running(self, monkeypatch, caplog):
        orch = fleet()

        def broken(*args):
            raise RuntimeError("no such magnetometer")

        monkeypatch.setattr(orchestrator_module, "fly_requested_pass", broken)
        with caplog.at_level(logging.WARNING, logger="fleet"):
            poll(
                monkeypatch,
                orch,
                {"_updated_at": 1.0, "mag_flyby": flyby_request(time.time())},
                {"_updated_at": 2.0, "frac_anomalous": 0.3, "mag_n_nodes": 4},
            )
        # The next poll still came, and applied its settings.
        assert orch.world.frac_anomalous == 0.3 and len(orch.magnetometers.nodes) == 4
        assert any(r.getMessage() == "UAP flyby not flown: no such magnetometer" for r in caplog.records)


class TestSeed:
    @staticmethod
    def built_from(monkeypatch, tmp_path, config, *argv):
        """The orchestrator main() builds from ``config``; main_async stops
        there, as it does when no node connects."""
        path = tmp_path / "fleet_config.json"
        path.write_text(json.dumps(config))
        built = []

        async def connect_nothing(self):
            built.append(self)

        monkeypatch.setattr(FleetOrchestrator, "connect_all", connect_nothing)
        monkeypatch.setattr(sys, "argv", ["orchestrator", "--config", str(path), *argv])
        orchestrator_module.main()
        return built[0]

    def test_the_magnetometers_follow_the_seed_the_config_was_generated_with(self, monkeypatch, tmp_path):
        # fleet-entrypoint.sh gives the generator --seed and the orchestrator
        # none, so the stamp is the only place the fleet's seed is found.
        scene = {"n_nodes": 4, "dual_fraction": 0.0, "layout": "ring", "seed": 7}
        orch = self.built_from(monkeypatch, tmp_path, {"fleet": {"scene": scene}, "nodes": RADARS}, "--seed", "3")
        assert orch.seed == 7 and orch.magnetometers.seed == 7

    def test_without_a_stamp_they_follow_the_seed_flag(self, monkeypatch, tmp_path):
        orch = self.built_from(monkeypatch, tmp_path, {"nodes": RADARS}, "--seed", "9")
        assert orch.seed == 9 and orch.magnetometers.seed == 9
