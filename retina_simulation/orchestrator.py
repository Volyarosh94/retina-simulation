"""
Fleet Orchestrator — Runs a large-scale test network of 100-1000 synthetic
nodes against a RETINA server using a shared SimulationWorld.

All nodes observe the same simulated aircraft from their own geometry.
Detection frames are streamed over TCP to the server, exercising:
  - TCP protocol (HELLO → CONFIG → DETECTION/HEARTBEAT)
  - Passive radar pipeline (tracker + solver)
  - Inter-node association and multi-node solver
  - Node analytics (trust scoring, reputation)
  - Data archival
  - Live map feed (aircraft.json / WebSocket)

Usage:
    # 200 nodes against local server
    python fleet_orchestrator.py --config fleet_config.json --host localhost --port 3012

    # 500 nodes against test server
    python fleet_orchestrator.py --config fleet_config.json --host testapi.retina.fm --port 3012 --nodes 500

    # With validation (stores ground truth for comparison)
    python fleet_orchestrator.py --config fleet_config.json --validate --validation-url http://localhost:8000
"""

import argparse
import asyncio
import json
import logging
import os
import signal
import ssl
import time
from datetime import datetime, timezone

# Add parent dir so we can import simulation packages
from retina_simulation.generator import (
    _KNOWN_METROS,
    coverage_cells,
    fleet_summary,
    generate_fleet,
)
from retina_simulation.live_adsb import LiveAdsbClient
from retina_simulation.magnetometer import (
    MagnetometerArray,
    MagnetometerSettings,
    build_magnetometer_push_payload,
    uniform,
)
from retina_simulation.tower_resolver import apply_tower_assignments, resolve_towers
from retina_simulation.world import (
    MetroCell,
    NodeConfig,
    SimulationWorld,
    waypoints_for_metro,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("fleet")


def _cells_to_metrocells(cell_dicts: list[dict]) -> list[MetroCell]:
    """Build MetroCell objects from first-class cell descriptors (fleet config
    'cells' or generator.coverage_cells). Cores come straight from the ring spec,
    so water-displaced receivers can't drift the hub-radial aim point. Cells
    lacking a core_lat/core_lon are skipped (a cell can't be placed without a
    core) rather than aborting the whole run on a malformed config."""
    cells = []
    for c in cell_dicts:
        if c.get("core_lat") is None or c.get("core_lon") is None:
            log.warning("Skipping coverage cell without core_lat/core_lon: %r", c.get("ring_id", c))
            continue
        cells.append(
            MetroCell(
                core_lat=c["core_lat"],
                core_lon=c["core_lon"],
                radius_km=c.get("radius_km", 70.0),
                ops_weight=c.get("ops_weight", 1.0),
            )
        )
    return cells


RETINA_VERSION = "1.0"
HEARTBEAT_INTERVAL_S = 60
CONFIG_ACK_TIMEOUT_S = 10


def _config_hash(cfg: dict) -> str:
    import hashlib

    cfg_str = json.dumps(cfg, sort_keys=True)
    return hashlib.sha256(cfg_str.encode()).hexdigest()[:16]


class NodeConnection:
    """Manages a single async TCP connection for one synthetic node."""

    def __init__(self, node_cfg: dict, host: str, port: int):
        self.cfg = node_cfg
        self.node_id = node_cfg["node_id"]
        self.host = host
        self.port = port
        self.reader: asyncio.StreamReader | None = None
        self.writer: asyncio.StreamWriter | None = None
        self.connected = False
        self.handshake_ok = False
        self.frames_sent = 0
        self.last_heartbeat = 0.0
        self._cfg_hash = _config_hash(node_cfg)

    async def connect(self, max_retries: int = 3) -> bool:
        """Establish TCP connection with retry."""
        for attempt in range(max_retries):
            try:
                self.reader, self.writer = await asyncio.wait_for(
                    asyncio.open_connection(self.host, self.port),
                    timeout=10.0,
                )
                self.connected = True
                return True
            except (ConnectionRefusedError, asyncio.TimeoutError, OSError) as e:
                wait = min(2 ** (attempt + 1), 15)
                log.debug("%s: connect failed (%s), retry in %ds", self.node_id, e, wait)
                await asyncio.sleep(wait)
        return False

    async def _send(self, msg: dict):
        """Send a newline-delimited JSON message."""
        if not self.writer:
            return
        data = json.dumps(msg).encode("utf-8") + b"\n"
        self.writer.write(data)
        await self.writer.drain()

    async def _recv(self, timeout: float = CONFIG_ACK_TIMEOUT_S) -> dict | None:
        """Receive a single newline-delimited JSON message."""
        if not self.reader:
            return None
        try:
            line = await asyncio.wait_for(self.reader.readline(), timeout=timeout)
            if not line:
                return None
            return json.loads(line.strip())
        except (asyncio.TimeoutError, json.JSONDecodeError):
            return None

    async def handshake(self) -> bool:
        """Perform RETINA protocol handshake."""
        # HELLO
        await self._send(
            {
                "type": "HELLO",
                "node_id": self.node_id,
                "version": RETINA_VERSION,
                "is_synthetic": True,
                "capabilities": {
                    "detection": True,
                    "adsb_correlation": True,
                    "doppler": True,
                    "config_hash": True,
                    "heartbeat": True,
                },
            }
        )

        # CONFIG
        await self._send(
            {
                "type": "CONFIG",
                "node_id": self.node_id,
                "config_hash": self._cfg_hash,
                "config": self.cfg,
            }
        )

        # Wait for CONFIG_ACK
        for _ in range(3):
            ack = await self._recv(timeout=CONFIG_ACK_TIMEOUT_S)
            if ack and ack.get("type") == "CONFIG_ACK":
                if ack.get("config_hash") == self._cfg_hash:
                    self.handshake_ok = True
                    return True
            # Re-send CONFIG on timeout
            await self._send(
                {
                    "type": "CONFIG",
                    "node_id": self.node_id,
                    "config_hash": self._cfg_hash,
                    "config": self.cfg,
                }
            )
        return False

    async def send_detection(self, frame: dict):
        """Send a detection frame."""
        await self._send(
            {
                "type": "DETECTION",
                "node_id": self.node_id,
                "data": frame,
            }
        )
        self.frames_sent += 1

    async def send_heartbeat(self):
        """Send a heartbeat if interval has elapsed."""
        now = time.monotonic()
        if now - self.last_heartbeat < HEARTBEAT_INTERVAL_S:
            return
        await self._send(
            {
                "type": "HEARTBEAT",
                "node_id": self.node_id,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "config_hash": self._cfg_hash,
                "status": "active",
            }
        )
        self.last_heartbeat = now

    async def close(self):
        """Close the connection."""
        if self.writer:
            self.writer.close()
            try:
                await self.writer.wait_closed()
            except Exception:
                pass
        self.connected = False


class FleetOrchestrator:
    """Manages a fleet of 100-1000 synthetic nodes sending to a RETINA server."""

    def __init__(
        self,
        node_configs: list[dict],
        host: str = "localhost",
        port: int = 3012,
        mode: str = "adsb",
        frame_interval: float = 0.5,
        max_concurrent_connects: int = 50,
        connect_retries: int = 3,
        time_scale: float = 1.0,
        min_aircraft: int = 0,
        max_aircraft: int = 0,
        beam_width_deg: float = 0.0,
        max_range_km: float = 0.0,
        hub_radial: bool = True,
        metro_traffic_frac: float = 0.6,
        cells: list[dict] | None = None,
        metro: str | None = None,
        seed: int = 0,
        magnetometers: int = 8,
        mag_placement: str = "spread",
    ):
        self.node_configs = node_configs
        self.seed = seed
        # Magnetometers beside some radar receivers (magnetometer.py). The
        # runtime config's mag_* keys take over from these at the first poll.
        self.mag_settings = MagnetometerSettings(
            enabled=magnetometers > 0, n_nodes=max(magnetometers, 0), placement=mag_placement
        )
        self.magnetometers: MagnetometerArray | None = None
        self._last_flyby_at = 0.0
        self.cells = cells or []
        self.metro = metro
        self.host = host
        self.port = port
        self.mode = mode
        self.frame_interval = frame_interval
        self.max_concurrent_connects = max_concurrent_connects
        self.connect_retries = max(1, connect_retries)
        self.time_scale = max(0.1, time_scale)
        self.min_aircraft = max(0, min_aircraft)
        self.max_aircraft = max(0, max_aircraft)
        self.beam_width_deg = max(0.0, beam_width_deg)
        self.max_range_km = max(0.0, max_range_km)
        self.hub_radial = hub_radial
        self.metro_traffic_frac = min(1.0, max(0.0, metro_traffic_frac))
        self.connections: dict[str, NodeConnection] = {}
        self.world: SimulationWorld | None = None
        # Runtime switch for the live ADS-B seed task (_seed_live_adsb):
        # the task only runs when main_async wires it, but the backend's
        # simulation config can pause it without a restart.
        self.live_adsb_enabled = True
        self._running = False
        self._stats = {
            "total_frames": 0,
            "total_detections": 0,
            "connected_nodes": 0,
            "handshake_ok": 0,
            "start_time": 0,
            "errors": 0,
        }
        # Ground truth storage for validation
        self.ground_truth: list[dict] = []
        # Auto-reconnect state: per-node earliest-retry timestamp and attempt count
        self._reconnect_next: dict[str, float] = {}
        self._reconnect_attempts: dict[str, int] = {}

    def _build_world(self):
        """Build the SimulationWorld from node configurations."""
        # Determine center from all node positions
        lats = [c["rx_lat"] for c in self.node_configs]
        lons = [c["rx_lon"] for c in self.node_configs]
        center_lat = sum(lats) / len(lats)
        center_lon = sum(lons) / len(lons)

        self.world = SimulationWorld(
            center_lat=center_lat,
            center_lon=center_lon,
            waypoints=waypoints_for_metro(self.metro),
        )
        # Aircraft population is a field, not a per-sensor count. Nationwide it is
        # floored high so the whole US stays populated; a metro-scoped fleet covers
        # ~1/40th of that area, so the same floor would pack hundreds of aircraft
        # into one terminal area. --min-aircraft/--max-aircraft override either way.
        n = len(self.node_configs)
        if self.metro:
            auto_min_aircraft = max(15, n // 2)
            auto_max_aircraft = max(30, n)
        else:
            auto_min_aircraft = max(150, n // 2)
            auto_max_aircraft = max(300, n)
        self.world.min_aircraft = self.min_aircraft or auto_min_aircraft
        self.world.max_aircraft = self.max_aircraft or auto_max_aircraft
        if self.world.max_aircraft < self.world.min_aircraft:
            self.world.max_aircraft = self.world.min_aircraft

        for cfg in self.node_configs:
            # Propagate CLI overrides into the raw config dict so the
            # handshake sends correct beam/range to the server's
            # InterNodeAssociator (which builds overlap grids from these).
            if self.beam_width_deg:
                cfg["beam_width_deg"] = self.beam_width_deg
            if self.max_range_km:
                cfg["max_range_km"] = self.max_range_km

            node = NodeConfig(
                node_id=cfg["node_id"],
                rx_lat=cfg["rx_lat"],
                rx_lon=cfg["rx_lon"],
                rx_alt_ft=cfg["rx_alt_ft"],
                tx_lat=cfg["tx_lat"],
                tx_lon=cfg["tx_lon"],
                tx_alt_ft=cfg["tx_alt_ft"],
                fc_hz=cfg["fc_hz"],
                fs_hz=cfg.get("fs_hz", 2_000_000),
                beam_width_deg=self.beam_width_deg or cfg.get("beam_width_deg", 42),
                max_range_km=self.max_range_km or cfg.get("max_range_km", 50),
                beam_azimuth_deg=cfg.get("beam_azimuth_deg"),  # None → broadside in add_node
                # Without this the world falls back to the monostatic RX-radius
                # rule while the handshake tells the server the bistatic limit —
                # nodes then "detect" 1.6x beyond what the server will accept.
                max_bistatic_range_km=cfg.get("max_bistatic_range_km"),
            )
            self.world.add_node(node)

        if self.hub_radial and self.cells:
            self.world.metro_cells = _cells_to_metrocells(self.cells)
            self.world.frac_metro_traffic = self.metro_traffic_frac

        self.magnetometers = MagnetometerArray(
            node_configs=self.node_configs,
            center=(center_lat, center_lon),
            seed=self.seed,
            settings=self.mag_settings,
        )

        log.info(
            "SimulationWorld: center=(%.2f, %.2f), %d nodes, %d-%d aircraft, %d metro cells, waypoint net=%s (%d)",
            center_lat,
            center_lon,
            len(self.node_configs),
            self.world.min_aircraft,
            self.world.max_aircraft,
            len(self.world.metro_cells),
            self.metro or "nationwide",
            len(self.world.waypoints),
        )

    async def _connect_batch(self, configs: list[dict]) -> list[dict]:
        """Connect a batch of nodes concurrently and return failed configs."""
        sem = asyncio.Semaphore(self.max_concurrent_connects)
        failed: list[dict] = []

        async def _connect_one(cfg):
            async with sem:
                if cfg["node_id"] in self.connections:
                    return
                conn = NodeConnection(cfg, self.host, self.port)
                if await conn.connect():
                    if await conn.handshake():
                        self.connections[cfg["node_id"]] = conn
                        return
                    else:
                        await conn.close()
                        log.warning("%s: handshake failed", cfg["node_id"])
                failed.append(cfg)

        await asyncio.gather(*[_connect_one(c) for c in configs])
        return failed

    async def connect_all(self):
        """Connect all nodes in batches and retry failed handshakes."""
        log.info("Connecting %d nodes to %s:%d ...", len(self.node_configs), self.host, self.port)

        batch_size = max(1, min(self.max_concurrent_connects, len(self.node_configs)))
        pending = list(self.node_configs)

        for attempt in range(1, self.connect_retries + 1):
            if not pending:
                break

            log.info(
                "Connection round %d/%d: %d nodes pending",
                attempt,
                self.connect_retries,
                len(pending),
            )

            next_pending: list[dict] = []
            round_start_connected = len(self.connections)

            for i in range(0, len(pending), batch_size):
                batch = pending[i : i + batch_size]
                failed = await self._connect_batch(batch)
                next_pending.extend(failed)
                len(self.connections) - round_start_connected
                log.info(
                    "  batch %d-%d: %d/%d connected this round (total: %d/%d)",
                    i + 1,
                    i + len(batch),
                    len(batch) - len(failed),
                    len(batch),
                    len(self.connections),
                    len(self.node_configs),
                )
                await asyncio.sleep(1.0)

            pending = next_pending
            if pending and attempt < self.connect_retries:
                log.info("Retrying %d failed nodes after cooldown", len(pending))
                await asyncio.sleep(min(2 * attempt, 5))

        self._stats["connected_nodes"] = len(self.connections)
        self._stats["handshake_ok"] = len(self.connections)
        log.info("Fleet connected: %d/%d nodes", len(self.connections), len(self.node_configs))
        if pending:
            log.warning("%d nodes still failed after retries", len(pending))

    def _record_ground_truth(self, timestamp_ms: int):
        """Snapshot the simulation ground truth for validation."""
        if self.world is None:
            return
        truth = {
            "timestamp_ms": timestamp_ms,
            "aircraft": self.world.get_aircraft_summary(),
        }
        self.ground_truth.append(truth)
        # Keep rolling window to bound memory
        if len(self.ground_truth) > 1000:
            self.ground_truth = self.ground_truth[-500:]

    async def _send_frame_to_node(self, node_id: str, frame: dict):
        """Send a detection frame to a single node connection."""
        conn = self.connections.get(node_id)
        if not conn or not conn.connected:
            return
        try:
            await conn.send_detection(frame)
            await conn.send_heartbeat()
            n_det = len(frame.get("delay", []))
            self._stats["total_frames"] += 1
            self._stats["total_detections"] += n_det
        except (ConnectionResetError, BrokenPipeError, OSError):
            conn.connected = False
            self._stats["errors"] += 1

    async def _reconnect_one(self, conn: NodeConnection) -> bool:
        """Attempt to reconnect and re-handshake a single node. Returns True on success."""
        await conn.close()
        ok = await conn.connect(max_retries=1)
        if ok:
            ok = await conn.handshake()
        if ok:
            self._reconnect_next.pop(conn.node_id, None)
            self._reconnect_attempts.pop(conn.node_id, None)
            log.info("Reconnected: %s", conn.node_id)
            return True
        # Exponential backoff: 30 s → 60 → 120 → 240 → 300 s (cap)
        attempt = self._reconnect_attempts.get(conn.node_id, 0) + 1
        self._reconnect_attempts[conn.node_id] = attempt
        delay = min(30 * (2 ** (attempt - 1)), 300)
        self._reconnect_next[conn.node_id] = time.monotonic() + delay
        return False

    async def _reconnect_loop(self, check_interval_s: float = 15.0):
        """Background task: periodically reconnect nodes whose TCP connection dropped.

        After a Docker rebuild the server closes all sockets.  The simulation
        loop marks those nodes ``connected=False`` and skips them.  This loop
        finds them and re-establishes the TCP + handshake so frames resume
        automatically without restarting the process.
        """
        # Give the initial connect_all() a head-start before we start checking.
        await asyncio.sleep(check_interval_s)

        while self._running:
            now = time.monotonic()
            due = [
                conn
                for conn in self.connections.values()
                if not conn.connected and now >= self._reconnect_next.get(conn.node_id, 0.0)
            ]

            if due:
                log.info(
                    "Auto-reconnect: %d disconnected node(s) eligible for retry",
                    len(due),
                )
                batch_size = self.max_concurrent_connects
                reconnected = 0
                for i in range(0, len(due), batch_size):
                    batch = due[i : i + batch_size]
                    results = await asyncio.gather(
                        *[self._reconnect_one(c) for c in batch],
                        return_exceptions=True,
                    )
                    reconnected += sum(1 for r in results if r is True)
                    if i + batch_size < len(due):
                        await asyncio.sleep(1.0)

                still_down = sum(1 for c in self.connections.values() if not c.connected)
                log.info(
                    "Auto-reconnect done: %d recovered, %d still down",
                    reconnected,
                    still_down,
                )

            await asyncio.sleep(check_interval_s)

    async def run_simulation_loop(self, duration_s: float = 0):
        """Main simulation loop — step world continuously, send frames staggered.

        The world now advances every ``tick_dt=1s`` (multiplied by time_scale)
        regardless of ``frame_interval``.  Node frames are staggered evenly
        across ``frame_interval`` so the total frame rate stays the same
        (n_nodes / frame_interval per second) but aircraft positions update
        every real second instead of every ``frame_interval`` seconds.
        This prevents the "stationary then teleport" issue when frame_interval
        is large (e.g. 40 s).
        """
        self._running = True
        self._stats["start_time"] = time.monotonic()
        tick_dt = 1.0  # world advances this many real-seconds per loop iteration
        report_interval = 10  # log stats every N seconds
        last_report = time.monotonic()

        # Stagger per-node send times evenly across frame_interval so we never
        # send to all nodes at once.
        # Sort nodes by geographic position (rx_lat, rx_lon) so that nodes in
        # the same metro area receive adjacent stagger slots.  Without sorting,
        # connection order is random → two nearby nodes can be 20-40 s apart in
        # stagger → frame-sync gate rejects them → zero metro associations.
        # With geo-sort, same-metro nodes are 0.2–2 s apart → all pass the 3 s
        # gate and timing-induced position error stays < 500 m.
        now_init = time.monotonic()
        raw_node_ids = list(self.connections.keys())

        def _geo_sort_key(nid: str) -> tuple:
            if self.world and nid in self.world.nodes:
                n = self.world.nodes[nid]
                return (round(n.rx_lat, 0), round(n.rx_lon, 0), n.rx_lat, n.rx_lon)
            return (0.0, 0.0, 0.0, 0.0)

        node_ids = sorted(raw_node_ids, key=_geo_sort_key)
        n_nodes = max(len(node_ids), 1)
        next_send: dict[str, float] = {}
        for i, nid in enumerate(node_ids):
            next_send[nid] = now_init + i * (self.frame_interval / n_nodes)

        log.info(
            "Starting simulation loop (tick=%.1fs, frame_interval=%.1fs, "
            "time_scale=%.1fx, mode=%s, duration=%s, ~%.1f frames/s)",
            tick_dt,
            self.frame_interval,
            self.time_scale,
            self.mode,
            f"{duration_s}s" if duration_s else "infinite",
            n_nodes / self.frame_interval,
        )

        try:
            while self._running:
                loop_start = time.monotonic()

                # Advance the simulation world by one tick
                self.world.step(tick_dt * self.time_scale, mode=self.mode)
                timestamp_ms = int(time.time() * 1000)

                # Record ground truth on every world step
                self._record_ground_truth(timestamp_ms)

                # Send frames only to nodes whose per-node cooldown has elapsed
                now_t = time.monotonic()
                send_tasks = []
                for node_id, conn in self.connections.items():
                    if not conn.connected:
                        continue
                    if now_t < next_send.get(node_id, 0.0):
                        continue
                    frame = self.world.generate_detections_for_node(node_id, timestamp_ms)
                    if frame.get("delay"):  # only send non-empty frames
                        send_tasks.append(self._send_frame_to_node(node_id, frame))
                    # Schedule next send regardless of whether frame was non-empty
                    next_send[node_id] = now_t + self.frame_interval

                if send_tasks:
                    await asyncio.gather(*send_tasks)

                # Periodic stats report
                now_t = time.monotonic()
                if now_t - last_report >= report_interval:
                    elapsed = now_t - self._stats["start_time"]
                    active = sum(1 for c in self.connections.values() if c.connected)
                    fps = self._stats["total_frames"] / max(elapsed, 1)
                    dps = self._stats["total_detections"] / max(elapsed, 1)
                    log.info(
                        "STATS: %.0fs elapsed | %d active nodes | %d frames (%.0f/s) | "
                        "%d detections (%.0f/s) | %d errors | %d aircraft",
                        elapsed,
                        active,
                        self._stats["total_frames"],
                        fps,
                        self._stats["total_detections"],
                        dps,
                        self._stats["errors"],
                        len(self.world.aircraft) if self.world else 0,
                    )
                    last_report = now_t

                # Check duration limit
                if duration_s > 0 and (now_t - self._stats["start_time"]) >= duration_s:
                    log.info("Duration limit reached (%.0fs), stopping", duration_s)
                    break

                # Pace the loop to tick_dt
                elapsed_loop = time.monotonic() - loop_start
                sleep_time = max(0, tick_dt - elapsed_loop)
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)

        except asyncio.CancelledError:
            log.info("Simulation loop cancelled")
        finally:
            self._running = False

    async def stop(self):
        """Gracefully disconnect all nodes."""
        self._running = False
        log.info("Disconnecting %d nodes...", len(self.connections))
        close_tasks = [conn.close() for conn in self.connections.values()]
        await asyncio.gather(*close_tasks, return_exceptions=True)
        log.info("All nodes disconnected")

    def get_stats(self) -> dict:
        """Return current fleet statistics."""
        elapsed = time.monotonic() - self._stats.get("start_time", time.monotonic())
        active = sum(1 for c in self.connections.values() if c.connected)
        return {
            **self._stats,
            "elapsed_s": round(elapsed, 1),
            "active_nodes": active,
            "frames_per_sec": round(self._stats["total_frames"] / max(elapsed, 1), 1),
            "detections_per_sec": round(self._stats["total_detections"] / max(elapsed, 1), 1),
            "aircraft_count": len(self.world.aircraft) if self.world else 0,
            "live_aircraft_count": len(self.world.live_aircraft) if self.world else 0,
            "ground_truth_snapshots": len(self.ground_truth),
        }

    def save_ground_truth(self, path: str):
        """Save ground truth data for offline validation."""
        with open(path, "w") as f:
            json.dump(
                {
                    "fleet_stats": self.get_stats(),
                    "ground_truth": self.ground_truth[-200:],  # last 200 snapshots
                },
                f,
                indent=2,
            )
        log.info("Ground truth saved: %s (%d snapshots)", path, len(self.ground_truth))


def build_ground_truth_payload(aircraft_summaries: list[dict]) -> list[dict]:
    """Remap world aircraft summaries to the server ground-truth push schema.

    Dark objects have no ADS-B hex, so their stable object id doubles as the
    ground-truth key; entries with neither are unidentifiable and dropped.
    """
    payload_aircraft = []
    for ac in aircraft_summaries:
        hex_code = ac.get("adsb_hex") or ac.get("id", "")
        if not hex_code:
            continue
        payload_aircraft.append(
            {
                "hex": hex_code,
                "lat": ac["lat"],
                "lon": ac["lon"],
                "alt_m": ac["alt_km"] * 1000,
                "heading": ac.get("heading", 0),
                "speed_ms": ac.get("speed_ms", 0),
                "object_type": ac.get("object_type", "aircraft"),
                "is_anomalous": ac.get("is_anomalous", False),
                "has_adsb": ac.get("has_adsb", False),
                # Transponder present but currently silent.  has_adsb stays
                # True — the truth is "this aircraft has a transponder and it
                # is off right now", which is what verifies a known-track hold.
                "adsb_silent": ac.get("adsb_silent", False),
                "adsb_callsign": ac.get("adsb_callsign") or None,
                "anomaly_event": ac.get("anomaly_event") or None,
                # "live" for an aircraft mirrored from the ADS-B feed, "sim"
                # for the world's own spawns — so the server can count and
                # colour the two populations apart.
                "source": ac.get("source") or "sim",
            }
        )
    return payload_aircraft


def build_real_adsb_body(payload: list[dict]) -> dict:
    """Wrap relayed adsb.lol aircraft for /api/sim/adsb/push.

    source=real declares the world these positions belong to.  The server
    stores it on each cache entry and claiming keys on it: without the tag,
    every relayed real aircraft is a decoy a synthetic node's echo can bind
    to on a delay/Doppler coincidence — a plane icon at a position no radar
    in either world measured.  The fleet's own push (build_adsb_push_payload)
    stays untagged and the server defaults it to the simulated world.
    """
    return {
        "ts_ms": int(time.time() * 1000),
        "source": "real",
        "aircraft": payload,
    }


def build_adsb_push_payload(aircraft_summaries: list[dict]) -> list[dict]:
    """Remap world aircraft summaries to the server ADS-B push schema.

    Transponder-equipped aircraft that are actually broadcasting only.  This push IS the simulated ADS-B
    broadcast, and a dark target by definition emits none — pushing it with
    its object id standing in for the hex minted a fake transponder per dark
    aircraft on the server, so every dark solve keyed mn-adsb-* and the dark
    lane stayed permanently empty.  Dark aircraft still reach the server
    through the ground-truth push, where the object id is the intended key
    (build_ground_truth_payload above).

    An aircraft inside a transponder outage is skipped for the same reason:
    this push IS the broadcast, so a silent aircraft must stop appearing in
    it — otherwise the outage is invisible to the server and the known-track
    hold it exists to exercise is never entered.
    """
    payload_aircraft = []
    for ac in aircraft_summaries:
        hex_code = ac.get("adsb_hex") or ""
        if not hex_code:
            continue
        if ac.get("adsb_silent"):
            continue
        speed_ms = ac.get("speed_ms", 0)
        payload_aircraft.append(
            {
                "hex": hex_code,
                "flight": ac.get("adsb_callsign") or "",
                "lat": round(ac["lat"], 5),
                "lon": round(ac["lon"], 5),
                "alt_baro": round(ac["alt_km"] * 1000 / 0.3048),
                "gs": round(speed_ms * 1.94384, 1),
                "track": round(ac.get("heading", 0), 1),
            }
        )
    return payload_aircraft


async def _push_ground_truth_live(
    orchestrator: FleetOrchestrator,
    base_url: str,
    interval_s: float = 2.0,
):
    """Push live ground truth positions to the server every interval_s seconds.

    The server stores these in _ground_truth_trails so the frontend map can
    render side-by-side solved track (yellow) vs ground truth (cyan dashed).
    """
    import urllib.request

    log.info("Ground truth live push started (url=%s, interval=%.1fs)", base_url, interval_s)
    url = f"{base_url}/api/test/ground-truth/push"
    loop = asyncio.get_event_loop()
    ssl_context = None
    if "localhost" in base_url or "127.0.0.1" in base_url:
        ssl_context = ssl._create_unverified_context()  # noqa: S323 — unverified context only for the localhost/127.0.0.1 dev path above

    while orchestrator._running:
        await asyncio.sleep(interval_s)

        if orchestrator.world is None:
            continue

        try:
            aircraft = orchestrator.world.get_aircraft_summary()
            payload_aircraft = build_ground_truth_payload(aircraft)

            if payload_aircraft:
                body = json.dumps(
                    {
                        "ts_ms": int(time.time() * 1000),
                        "aircraft": payload_aircraft,
                    }
                ).encode()
                req = urllib.request.Request(
                    url,
                    data=body,
                    headers={
                        "Content-Type": "application/json",
                        **({"X-API-Key": _k} if (_k := os.environ.get("RADAR_API_KEY", "")) else {}),
                    },
                    method="POST",
                )
                await loop.run_in_executor(
                    None,
                    lambda r=req, ctx=ssl_context: urllib.request.urlopen(r, timeout=4, context=ctx),
                )
        except Exception as e:
            log.warning("Ground truth push failed: %s", e)


async def _push_real_adsb(
    orchestrator: FleetOrchestrator,
    base_url: str,
    areas: list[dict],
    interval_s: float = 10.0,
):
    """Poll adsb.lol for real aircraft and push them to the server as ADS-B positions.

    This injects real air traffic into the simulation alongside synthetic aircraft
    so the map shows a realistic mix.
    """
    import urllib.request

    try:
        try:
            from clients.adsb_lol import AdsbLolClient
        except ImportError:
            import importlib.util

            _p = os.path.join(os.path.dirname(os.path.dirname(__file__)), "clients", "adsb_lol.py")
            spec = importlib.util.spec_from_file_location("adsb_lol", _p)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            AdsbLolClient = mod.AdsbLolClient
    except Exception as e:
        log.warning("Cannot import AdsbLolClient: %s — real ADSB disabled", e)
        return

    client = AdsbLolClient(areas)
    url = f"{base_url}/api/sim/adsb/push"
    loop = asyncio.get_event_loop()
    ssl_context = None
    if "localhost" in base_url or "127.0.0.1" in base_url:
        ssl_context = ssl._create_unverified_context()  # noqa: S323 — unverified context only for the localhost/127.0.0.1 dev path above

    log.info("Real ADS-B feed started (%d areas, interval=%.0fs)", len(areas), interval_s)

    while orchestrator._running:
        await asyncio.sleep(interval_s)
        try:
            aircraft = await loop.run_in_executor(None, client.fetch_all)
            if not aircraft:
                continue

            payload = []
            for ac in aircraft:
                h = ac.get("hex", "")
                if not h or not ac.get("lat") or not ac.get("lon"):
                    continue
                payload.append(
                    {
                        "hex": h,
                        "flight": ac.get("flight", ""),
                        "lat": ac["lat"],
                        "lon": ac["lon"],
                        "alt_baro": ac.get("alt_baro", 0),
                        "gs": ac.get("gs", 0),
                        "track": ac.get("track", 0),
                    }
                )

            if payload:
                body = json.dumps(build_real_adsb_body(payload)).encode()
                req = urllib.request.Request(
                    url,
                    data=body,
                    headers={
                        "Content-Type": "application/json",
                        **({"X-API-Key": _k} if (_k := os.environ.get("RADAR_API_KEY", "")) else {}),
                    },
                    method="POST",
                )
                await loop.run_in_executor(
                    None,
                    lambda r=req, ctx=ssl_context: urllib.request.urlopen(r, timeout=5, context=ctx),
                )
                log.debug("Real ADS-B push: %d aircraft", len(payload))
        except Exception as e:
            log.debug("Real ADSB push failed: %s", e)


async def _seed_live_adsb(
    orchestrator: FleetOrchestrator,
    feed_url: str,
    areas: list[dict],
    interval_s: float = 5.0,
):
    """Pull real aircraft from the live feed into the world every interval_s.

    This is what makes the synthetic nodes fly real traffic: every aircraft
    the feed reports over the metro becomes a world aircraft the nodes echo
    (delay/Doppler from its real position and velocity), cast as ADS-B or
    dark by frac_live_dark.  Unlike the opt-in adsb.lol relay
    (_push_real_adsb), which only decorates the map with a second world's
    positions, these aircraft ARE in the simulated world — so their ADS-B
    tags reach the server through the ordinary node frames and 1 Hz push,
    untagged, as simulated-world traffic.

    The backend's live_adsb_enabled flag (via _poll_simulation_config)
    pauses the pull and clears the live aircraft; re-enabling resumes on the
    next poll.  A failed fetch leaves the world alone: aircraft coast on
    their last velocity and expire only after the world's staleness window.
    """
    client = LiveAdsbClient(areas, base_url=feed_url)
    loop = asyncio.get_event_loop()
    log.info(
        "Live ADS-B seeding started (feed=%s, %d area(s), interval=%.0fs)",
        feed_url,
        len(client.areas),
        interval_s,
    )
    last_live = -1
    was_enabled = True
    while orchestrator._running:
        await asyncio.sleep(interval_s)
        if orchestrator.world is None:
            continue
        if not orchestrator.live_adsb_enabled:
            if was_enabled:
                n = orchestrator.world.clear_live_aircraft()
                log.info("Live ADS-B seeding paused by simulation config (%d live aircraft removed)", n)
                was_enabled = False
            continue
        was_enabled = True
        try:
            rows = await loop.run_in_executor(None, client.fetch_all)
        except Exception as e:  # defensive: the client swallows its own errors
            log.debug("Live ADS-B fetch failed: %s", e)
            continue
        if not any(client.last_status.values()):
            log.warning("Live ADS-B feed unreachable (%s) — live aircraft coasting", client.last_error)
            continue
        stats = orchestrator.world.ingest_live_aircraft(rows)
        if stats["live"] != last_live:
            n_dark = sum(1 for ac in orchestrator.world.live_aircraft.values() if not ac.has_adsb)
            log.info(
                "Live ADS-B: %d aircraft in world (%d dark, frac_live_dark=%.2f; +%d new this poll)",
                stats["live"],
                n_dark,
                orchestrator.world.frac_live_dark,
                stats["created"],
            )
            last_live = stats["live"]


async def _poll_simulation_config(
    orchestrator: FleetOrchestrator,
    base_url: str,
    interval_s: float = 5.0,
    scene: dict | None = None,
):
    """Poll /api/simulation/config every interval_s and apply updated spawn fractions to world.

    Also watches for a scene change (n_nodes / dual_fraction): those are
    baked into the fleet at container boot (see generator.py), so unlike the
    spawn fractions above they cannot be applied in-process. When ``scene``
    (this container's own stamped {n_nodes, dual_fraction, layout, seed}) is
    present and differs from what the backend now reports, this logs a loud
    WARN and calls ``orchestrator.stop()`` — every loop then exits, the
    process exits 0, and `restart: unless-stopped` relaunches into
    fleet-entrypoint.sh, which fetches the desired scene before regenerating.
    """
    import urllib.request

    log.info("Simulation config polling started (url=%s, interval=%.1fs)", base_url, interval_s)
    url = f"{base_url}/api/simulation/config"
    loop = asyncio.get_event_loop()
    ssl_context = None
    if "localhost" in base_url or "127.0.0.1" in base_url:
        ssl_context = ssl._create_unverified_context()  # noqa: S323 — unverified context only for the localhost/127.0.0.1 dev path above
    last_updated_at = 0.0

    while orchestrator._running:
        await asyncio.sleep(interval_s)

        if orchestrator.world is None:
            continue

        try:

            def _fetch():
                import json as _json

                with urllib.request.urlopen(url, context=ssl_context, timeout=5) as resp:
                    return _json.loads(resp.read())

            cfg = await loop.run_in_executor(None, _fetch)
            updated_at = cfg.get("_updated_at", 0.0)
            if updated_at > last_updated_at:
                # Fallback matches SimulationWorld's default (anomalies off), so a
                # payload missing the key cannot silently switch them back on.
                orchestrator.world.frac_anomalous = float(cfg.get("frac_anomalous", 0.0))
                orchestrator.world.frac_drone = float(cfg.get("frac_drone", 0.0))
                orchestrator.world.frac_dark = float(cfg.get("frac_dark", 0.15))
                # Default 0.0 (matching SimulationWorld) so a payload missing
                # the key cannot switch outages on.  Raising it also re-rolls
                # aircraft already in the air — see schedule_adsb_outages.
                orchestrator.world.frac_adsb_outage = float(cfg.get("frac_adsb_outage", 0.0))
                if orchestrator.world.frac_adsb_outage > 0.0:
                    n_sched = orchestrator.world.schedule_adsb_outages()
                    if n_sched:
                        log.info(
                            "ADS-B outages scheduled for %d in-flight aircraft (frac %.2f)",
                            n_sched,
                            orchestrator.world.frac_adsb_outage,
                        )
                if "min_aircraft" in cfg:
                    orchestrator.world.min_aircraft = int(cfg["min_aircraft"])
                if "max_aircraft" in cfg:
                    orchestrator.world.max_aircraft = int(cfg["max_aircraft"])
                # Live feed knobs.  Fallbacks match the world/orchestrator
                # defaults (0.0 dark, seeding on) so an older backend that
                # ships neither key changes nothing.  set_frac_live_dark
                # re-casts the aircraft already in the air, so the slider
                # takes effect at the next poll rather than the next fleet
                # turnover.
                n_live_dark = orchestrator.world.set_frac_live_dark(float(cfg.get("frac_live_dark", 0.0)))
                orchestrator.live_adsb_enabled = bool(cfg.get("live_adsb_enabled", True))
                # Isolated: nothing about the magnetometers may stop the rest
                # of this poll, the scene-change restart below included.
                try:
                    apply_magnetometer_config(orchestrator, cfg, time.time())
                except Exception as e:
                    log.warning("Magnetometer config not applied: %s", e)
                last_updated_at = updated_at
                log.info(
                    "Simulation config updated: anomalous=%.2f drone=%.2f dark=%.2f adsb_outage=%.2f "
                    "aircraft=%d–%d live_adsb=%s live_dark=%.2f (%d/%d live aircraft dark)",
                    orchestrator.world.frac_anomalous,
                    orchestrator.world.frac_drone,
                    orchestrator.world.frac_dark,
                    orchestrator.world.frac_adsb_outage,
                    orchestrator.world.min_aircraft,
                    orchestrator.world.max_aircraft,
                    "on" if orchestrator.live_adsb_enabled else "off",
                    orchestrator.world.frac_live_dark,
                    n_live_dark,
                    len(orchestrator.world.live_aircraft),
                )

                # Scene-change detection. Absent scene stamp (stale volume,
                # in-process generation) or absent config keys (never PUT) →
                # no comparison, no restart.
                scene_n_nodes = scene.get("n_nodes") if scene else None
                scene_dual_fraction = scene.get("dual_fraction") if scene else None
                scene_diff = False
                if scene:
                    if "n_nodes" in cfg and scene_n_nodes is not None and int(cfg["n_nodes"]) != int(scene_n_nodes):
                        scene_diff = True
                    if (
                        "dual_fraction" in cfg
                        and scene_dual_fraction is not None
                        and abs(float(cfg["dual_fraction"]) - float(scene_dual_fraction)) > 1e-6
                    ):
                        scene_diff = True
                # max_range_km needs no stamp: the orchestrator itself holds
                # the running value, so this runs even when `scene` is None.
                # Applying a range change requires regenerating node configs
                # (every node's cfg is built from it at construction), which
                # is exactly the restart path — the poll loop deliberately
                # does NOT live-apply it.
                if "max_range_km" in cfg and abs(float(cfg["max_range_km"]) - float(orchestrator.max_range_km)) > 1e-6:
                    scene_diff = True
                if scene_diff:
                    log.warning(
                        "Scene change requested (n_nodes=%s dual_fraction=%s "
                        "max_range_km=%s, running n_nodes=%s dual_fraction=%s "
                        "max_range_km=%s) — shutting down for regeneration",
                        cfg.get("n_nodes"),
                        cfg.get("dual_fraction"),
                        cfg.get("max_range_km"),
                        scene_n_nodes,
                        scene_dual_fraction,
                        orchestrator.max_range_km,
                    )
                    await orchestrator.stop()
                    return
        except Exception as e:
            log.debug("Config poll failed: %s", e)


# A flyby request older than this when the fleet first sees it is not acted on:
# a UAP appearing minutes after the click that asked for it reads as a bug. It
# is also all that keeps a restarted fleet, which remembers nothing, from
# flying the last request again: the server goes on serving it, so a fleet
# back within this time of it (a scene change right after the click) does fly
# it again, in a new world where the first pass no longer exists.
FLYBY_MAX_AGE_S = 120.0


def apply_magnetometer_config(orchestrator: FleetOrchestrator, cfg: dict, now: float):
    """Apply the runtime config's mag_* keys and any new UAP flyby request.

    Node count and placement re-place the magnetometers in-process: unlike the
    radar nodes they hold no TCP session, so nothing needs a fleet restart.
    Returns the injected flyby object, if any.
    """
    array = getattr(orchestrator, "magnetometers", None)
    if array is None or orchestrator.world is None:
        return None
    settings = MagnetometerSettings.from_config(cfg, array.settings)
    if array.apply(settings, now):
        log.info(
            "Magnetometers: %d node(s), %s placement%s",
            len(array.nodes),
            settings.placement,
            "" if settings.enabled else " (disabled)",
        )
    request = cfg.get("mag_flyby")
    if not isinstance(request, dict):
        return None
    seq = request.get("seq")
    requested_at = request.get("requested_at")
    if isinstance(requested_at, bool) or not isinstance(requested_at, (int, float)):
        return None
    # A running fleet flies each request once (a restarted one: see
    # FLYBY_MAX_AGE_S). Its time says whether it is new, not its number: the
    # server numbers requests afresh when it restarts, and a fleet that kept
    # running would take every request after that for an old one.
    if requested_at <= orchestrator._last_flyby_at:
        return None
    orchestrator._last_flyby_at = requested_at
    if now - requested_at > FLYBY_MAX_AGE_S:
        log.info("Ignoring stale UAP flyby request #%s", seq)
        return None
    if not array.nodes:
        log.warning("UAP flyby #%s requested with no magnetometers placed", seq)
        return None
    target = next((n for n in array.nodes if request.get("node_id") in (n.node_id, n.host_node_id)), None)
    if target is None:
        target = array.nodes[int(uniform(orchestrator.seed, "flyby-target", requested_at) * len(array.nodes))]

    def number(key, default, low, high):
        value = request.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not low <= value <= high:
            return default
        return float(value)

    heading = request.get("heading_deg")
    if isinstance(heading, bool) or not isinstance(heading, (int, float)):
        heading = 360.0 * uniform(orchestrator.seed, "flyby-heading", requested_at)
    ac = orchestrator.world.inject_uap_flyby(
        target.lat,
        target.lon,
        ground_alt_km=target.alt_m / 1000.0,
        closest_approach_m=number("closest_approach_m", 600.0, 0.0, 20_000.0),
        altitude_m=number("altitude_m", 300.0, 50.0, 15_000.0),
        speed_mps=number("speed_mps", 120.0, 5.0, 1000.0),
        heading_deg=float(heading),
        lead_s=number("lead_s", 60.0, 10.0, 600.0),
    )
    log.info(
        "UAP flyby #%s: %s passes %s (%s) in %.0f s",
        seq,
        ac.object_id,
        target.node_id,
        target.host_node_id,
        number("lead_s", 60.0, 10.0, 600.0),
    )
    return ac


async def _push_magnetometers_live(
    orchestrator: FleetOrchestrator,
    base_url: str,
    poll_s: float = 0.25,
    push_interval_s: float = 1.0,
    hold_s: float = 10.0,
):
    """Sample the magnetometers once per world tick and push the readings.

    Sampling follows the world clock — one reading per step, which is when
    objects move — rather than this loop's timer, so a slow push can neither
    duplicate nor skip a tick. If a push fails its readings go with the next
    one, up to hold_s old; older ones are dropped, and the server sees a gap.
    """
    import urllib.request

    url = f"{base_url}/api/sim/magnetometer/push"
    loop = asyncio.get_event_loop()
    ssl_context = None
    if "localhost" in base_url or "127.0.0.1" in base_url:
        ssl_context = ssl._create_unverified_context()  # noqa: S323 — unverified context only for the localhost/127.0.0.1 dev path above
    log.info("Magnetometer push started (url=%s)", url)
    last_clock = None
    last_push = 0.0
    pending: list[tuple[int, list[dict]]] = []
    while orchestrator._running:
        await asyncio.sleep(poll_s)
        world, array = orchestrator.world, orchestrator.magnetometers
        if world is None or array is None:
            continue
        if world.clock_s != last_clock:
            last_clock = world.clock_s
            if array.nodes:
                now = time.time()
                anomalous = [ac for ac in world.aircraft if ac.object_type == "anomalous"]
                pending.append((int(now * 1000), array.sample(anomalous, now)))
        if time.monotonic() - last_push < push_interval_s:
            continue
        last_push = time.monotonic()
        cutoff = int((time.time() - hold_s) * 1000)
        pending = [batch for batch in pending if batch[0] >= cutoff]
        try:
            body = json.dumps(build_magnetometer_push_payload(array, pending)).encode()
            req = urllib.request.Request(
                url,
                data=body,
                headers={
                    "Content-Type": "application/json",
                    **({"X-API-Key": _k} if (_k := os.environ.get("RADAR_API_KEY", "")) else {}),
                },
                method="POST",
            )
            await loop.run_in_executor(
                None,
                lambda r=req, ctx=ssl_context: urllib.request.urlopen(r, timeout=3, context=ctx),
            )
            pending = []
        except Exception as e:
            log.debug("Magnetometer push failed: %s", e)


async def _push_adsb_live(
    orchestrator: FleetOrchestrator,
    base_url: str,
    interval_s: float = 1.0,
):
    """Push every aircraft's current ADS-B position to the server every interval_s.

    This runs independently of the frame pipeline so each aircraft gets a
    fresh position every second regardless of how many nodes observe it.
    The server endpoint updates state.adsb_aircraft directly and marks it
    dirty so the next 1-Hz WS broadcast includes the latest positions.
    """
    import urllib.request

    log.info("ADS-B live push started (url=%s, interval=%.1fs)", base_url, interval_s)
    url = f"{base_url}/api/sim/adsb/push"
    loop = asyncio.get_event_loop()
    ssl_context = None
    if "localhost" in base_url or "127.0.0.1" in base_url:
        import ssl as _ssl

        ssl_context = _ssl._create_unverified_context()  # noqa: S323 — unverified context only for the localhost/127.0.0.1 dev path above

    while orchestrator._running:
        await asyncio.sleep(interval_s)

        if orchestrator.world is None:
            continue

        try:
            aircraft_raw = orchestrator.world.get_aircraft_summary()
            payload_aircraft = build_adsb_push_payload(aircraft_raw)

            if not payload_aircraft:
                continue

            body = json.dumps(
                {
                    "ts_ms": int(time.time() * 1000),
                    "aircraft": payload_aircraft,
                }
            ).encode()
            req = urllib.request.Request(
                url,
                data=body,
                headers={
                    "Content-Type": "application/json",
                    **({"X-API-Key": _k} if (_k := os.environ.get("RADAR_API_KEY", "")) else {}),
                },
                method="POST",
            )
            await loop.run_in_executor(
                None,
                lambda r=req, ctx=ssl_context: urllib.request.urlopen(r, timeout=3, context=ctx),
            )
        except Exception as e:
            log.debug("ADS-B push failed: %s", e)


async def _validate_against_server(
    orchestrator: FleetOrchestrator,
    base_url: str,
    interval_s: float = 30.0,
):
    """Periodically compare server output against simulation ground truth."""
    import urllib.request

    log.info("Validation loop started (interval=%.0fs, url=%s)", interval_s, base_url)
    loop = asyncio.get_event_loop()
    ssl_context = None
    if "localhost" in base_url or "127.0.0.1" in base_url:
        ssl_context = ssl._create_unverified_context()  # noqa: S323 — unverified context only for the localhost/127.0.0.1 dev path above

    def _get_json(endpoint_url):
        with urllib.request.urlopen(endpoint_url, timeout=10, context=ssl_context) as r:
            return json.loads(r.read())

    while orchestrator._running:
        await asyncio.sleep(interval_s)

        if not orchestrator.ground_truth:
            continue

        try:
            server_aircraft_data = await loop.run_in_executor(
                None, _get_json, f"{base_url}/api/radar/data/aircraft.json"
            )
            server_aircraft = server_aircraft_data.get("aircraft", [])

            try:
                analytics = await loop.run_in_executor(None, _get_json, f"{base_url}/api/radar/analytics")
            except Exception:
                analytics = {}

            try:
                nodes_status = await loop.run_in_executor(None, _get_json, f"{base_url}/api/radar/nodes")
            except Exception:
                nodes_status = {}

            # Compare against latest ground truth
            truth = orchestrator.ground_truth[-1]
            truth_aircraft = truth["aircraft"]

            log.info(
                "VALIDATION: server=%d aircraft, truth=%d aircraft, server_nodes=%d connected, analytics_nodes=%d",
                len(server_aircraft),
                len(truth_aircraft),
                nodes_status.get("connected", 0),
                len(analytics.get("nodes", {})),
            )

        except Exception as e:
            log.debug("Validation check failed: %s", e)


def _parse_metro_areas(metros_str: str) -> list[dict]:
    """Parse comma-separated metro codes into area dicts for AdsbLolClient."""
    result = []
    for code in metros_str.split(","):
        code = code.strip().lower()
        if code in _KNOWN_METROS:
            result.append(_KNOWN_METROS[code])
        else:
            log.warning("Unknown metro code: %s (available: %s)", code, ",".join(_KNOWN_METROS))
    return result


async def main_async(args):
    """Main async entry point."""
    # Load or generate fleet config
    # Scene stamp (n_nodes/dual_fraction/layout/seed) the generator wrote into
    # the config it produced — read here so _poll_simulation_config can detect
    # a backend-requested scene change and self-restart for regeneration.
    # Only ever populated via the loaded-config-file path: fleet-entrypoint.sh
    # always runs the generator CLI (which stamps it) before starting us with
    # --config. None here means "no comparison" — a stale volume with no
    # stamp, or the in-process generation fallback below, never triggers a
    # restart loop.
    scene = None
    if args.config and os.path.exists(args.config):
        with open(args.config) as f:
            data = json.load(f)
        all_nodes = data.get("nodes", data.get("fleet", {}).get("nodes", []))
        if not all_nodes:
            # Fallback: maybe it's the old nodes_config.json format
            all_nodes = data.get("nodes", [])
        cells = data.get("cells", [])
        scene = data.get("fleet", {}).get("scene")
    else:
        log.info("No config file, generating %d nodes...", args.nodes)
        regions = [r.strip() for r in args.regions.split(",")]
        all_nodes = generate_fleet(
            n_nodes=args.nodes,
            regions=regions,
            seed=args.seed,
            n_cluster=args.n_cluster,
            n_clusters=args.n_clusters,
            metro=getattr(args, "metro", None),
        )
        cells = coverage_cells(
            n_cluster=args.n_cluster,
            n_clusters=args.n_clusters,
            metro=getattr(args, "metro", None),
            layout=getattr(args, "layout", "ring"),
        )

    # When --metros is specified, filter nodes to only those near selected metros
    if getattr(args, "metros", "") and args.metros:
        metro_areas = _parse_metro_areas(args.metros)
        if metro_areas:

            def _near_any_metro(node):
                for m in metro_areas:
                    dlat = abs(node["rx_lat"] - m["lat"])
                    dlon = abs(node["rx_lon"] - m["lon"])
                    if dlat < 2.0 and dlon < 2.0:  # ~200km box
                        return True
                return False

            before = len(all_nodes)
            all_nodes = [n for n in all_nodes if _near_any_metro(n)]
            log.info("Metro filter (%s): %d → %d nodes", args.metros, before, len(all_nodes))

    # Resolve real TX towers for each node (skip for non-US regions — FCC-only).
    # Coverage-ring receivers keep their shared illuminator: resolving per-RX
    # would give each ring node a different nearest tower, breaking the shared-TX
    # association invariant and the metro-cell grouping.
    if getattr(args, "use_real_towers", False):
        log.info("Resolving real TX towers via FCC API (cached)…")
        resolvable = [n for n in all_nodes if not str(n.get("node_id", "")).startswith("synth-RING")]
        assignments = resolve_towers(resolvable)
        updated = apply_tower_assignments(resolvable, assignments)
        log.info("Real tower assignments applied to %d / %d nodes.", updated, len(resolvable))

    # Limit to requested number
    if args.nodes and args.nodes < len(all_nodes):
        all_nodes = all_nodes[: args.nodes]

    # Keep only cells whose ring survived node filtering (e.g. --metros / --nodes)
    if cells:
        node_ids = [n["node_id"] for n in all_nodes]
        cells = [c for c in cells if any(nid.startswith(c["ring_id"]) for nid in node_ids)]

    log.info("Fleet: %d nodes, %d metro cells", len(all_nodes), len(cells))
    summary = fleet_summary(all_nodes)
    log.info("Summary: %s", json.dumps(summary, indent=2))

    orchestrator = FleetOrchestrator(
        node_configs=all_nodes,
        host=args.host,
        port=args.port,
        mode=args.mode,
        frame_interval=args.interval,
        max_concurrent_connects=args.concurrency,
        connect_retries=args.connect_retries,
        time_scale=args.time_scale,
        min_aircraft=args.min_aircraft,
        max_aircraft=args.max_aircraft,
        beam_width_deg=args.beam_width_deg,
        max_range_km=args.max_range_km,
        hub_radial=not args.no_hub_radial,
        metro_traffic_frac=args.metro_traffic_frac,
        cells=cells,
        metro=getattr(args, "metro", None),
        seed=args.seed,
        magnetometers=getattr(args, "magnetometers", 8),
        mag_placement=getattr(args, "mag_placement", "spread"),
    )

    # Build shared simulation world
    orchestrator._build_world()

    # Handle graceful shutdown
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(orchestrator.stop()))

    # Connect all nodes
    await orchestrator.connect_all()

    if orchestrator._stats["connected_nodes"] == 0:
        log.error("No nodes connected, exiting")
        return

    # Start validation loop if requested
    tasks = [
        orchestrator.run_simulation_loop(duration_s=args.duration),
        orchestrator._reconnect_loop(check_interval_s=15.0),
    ]

    # Always push per-aircraft ADS-B positions every second so every aircraft
    # has a fresh position in state.adsb_aircraft regardless of node visibility.
    if args.validation_url:
        tasks.append(
            _push_adsb_live(
                orchestrator,
                args.validation_url,
                interval_s=1.0,
            )
        )

    # Always push ground truth when we have a validation_url — needed for
    # anomaly detection and the frontend map overlay, not just validation.
    if args.validation_url:
        tasks.append(
            _push_ground_truth_live(
                orchestrator,
                args.validation_url,
                interval_s=2.0,
            )
        )

    # Magnetometer readings, one per world tick, for the server's detector and
    # the /sim map. Nothing to send without a server to send it to.
    if args.validation_url:
        tasks.append(_push_magnetometers_live(orchestrator, args.validation_url))

    # Poll server for updated simulation physics fractions (set from frontend UI).
    if args.validation_url:
        tasks.append(
            _poll_simulation_config(
                orchestrator,
                args.validation_url,
                interval_s=5.0,
                scene=scene,
            )
        )

    # Real ADS-B from adsb.lol — opt-in via --real-adsb, scoped to the configured
    # metro areas.  --metro (generation-time scoping) implies the same area for
    # real traffic, so a Greenville-only fleet gets Greenville ADS-B without also
    # passing --metros.  Opt-in rather than implied by metro config: relaying
    # real traffic over the same footprint the synthetic fleet flies in hands
    # the server's claiming stage a pool of decoy transponders, and for a long
    # time metro config alone switched this on — the "realistic mix" arrived
    # with ghost planes attached.
    adsb_metros = getattr(args, "metros", "") or getattr(args, "metro", "") or ""
    # Live ADS-B seeding: real aircraft over the metro join the simulated
    # world and are echoed by the synthetic nodes (see _seed_live_adsb).
    # Needs an area to query, so it is scoped like the relay below.
    live_url = (getattr(args, "live_adsb_url", "") or "").strip()
    live_seeding = False
    if live_url and adsb_metros:
        live_areas = _parse_metro_areas(adsb_metros)
        if live_areas:
            live_seeding = True
            tasks.append(
                _seed_live_adsb(
                    orchestrator,
                    live_url,
                    areas=live_areas,
                    interval_s=max(1.0, float(getattr(args, "live_adsb_interval", 5.0) or 5.0)),
                )
            )
    elif live_url:
        log.warning("Live ADS-B seeding needs --metro/--metros to know where to look — disabled")
    if getattr(args, "real_adsb", False) and args.validation_url and adsb_metros:
        if live_seeding:
            # Both would push the same real hexes, one tagged source=real and
            # one as simulated-world traffic, and the server keys its cache
            # by hex — the two writers would take turns owning each entry.
            log.warning("--real-adsb ignored: live ADS-B seeding already carries real traffic in the simulated world")
        else:
            metro_areas = _parse_metro_areas(adsb_metros)
            if metro_areas:
                tasks.append(
                    _push_real_adsb(
                        orchestrator,
                        args.validation_url,
                        areas=metro_areas,
                        interval_s=10.0,
                    )
                )
    elif adsb_metros and args.validation_url and not live_seeding:
        log.info("Real ADS-B relay disabled (pass --real-adsb to inject adsb.lol traffic)")

    if args.validate and args.validation_url:
        tasks.append(
            _validate_against_server(
                orchestrator,
                args.validation_url,
                interval_s=30.0,
            )
        )

    try:
        await asyncio.gather(*tasks)
    finally:
        # Save ground truth
        if args.ground_truth_path:
            orchestrator.save_ground_truth(args.ground_truth_path)

        await orchestrator.stop()

        # Final report
        stats = orchestrator.get_stats()
        log.info("=" * 60)
        log.info("FINAL REPORT")
        log.info("=" * 60)
        for k, v in stats.items():
            log.info("  %s: %s", k, v)


def main():
    parser = argparse.ArgumentParser(description="Fleet Orchestrator — run 100-1000 synthetic nodes")
    parser.add_argument("--config", type=str, default="fleet_config.json", help="Path to fleet_config.json")
    parser.add_argument("--nodes", type=int, default=0, help="Number of nodes to use (0 = all from config)")
    parser.add_argument("--regions", type=str, default="us", help="Regions for auto-generation: us,eu,au")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for fleet generation")
    parser.add_argument(
        "--n-cluster",
        "--n-ring",
        dest="n_cluster",
        type=int,
        default=30,
        help="Total metro-ring receiver budget, split across --n-clusters "
        "rings (only used when auto-generating, i.e. no --config). "
        "Matches the generator default.",
    )
    parser.add_argument(
        "--n-clusters",
        "--n-rings",
        dest="n_clusters",
        type=int,
        default=5,
        help="Number of distinct metro rings to fan the --n-cluster budget "
        "across (5 = Dallas, Chicago, Atlanta, Denver, Kansas City; "
        "1 = single Dallas ring). Capped at the number of rings that "
        "survive --metro. Matches the generator default.",
    )
    parser.add_argument("--host", type=str, default="localhost", help="Server hostname")
    parser.add_argument("--port", type=int, default=3012, help="Server TCP port")
    parser.add_argument(
        "--mode", type=str, default="adsb", choices=["detection", "adsb", "anomalous"], help="Detection mode"
    )
    parser.add_argument("--interval", type=float, default=0.5, help="Frame interval in seconds")
    parser.add_argument(
        "--time-scale", type=float, default=1.0, help="Simulation speed multiplier (for demo visibility)"
    )
    parser.add_argument("--duration", type=float, default=0, help="Run duration in seconds (0 = infinite)")
    parser.add_argument(
        "--min-aircraft", type=int, default=0, help="Minimum aircraft to keep alive (0 = auto demo default)"
    )
    parser.add_argument("--max-aircraft", type=int, default=0, help="Maximum aircraft in world (0 = auto demo default)")
    parser.add_argument(
        "--beam-width-deg", type=float, default=0, help="Override node beam width for demo visibility (0 = use config)"
    )
    parser.add_argument(
        "--max-range-km", type=float, default=0, help="Override node max range for demo visibility (0 = use config)"
    )
    parser.add_argument("--concurrency", type=int, default=50, help="Max concurrent TCP connections during setup")
    parser.add_argument(
        "--connect-retries", type=int, default=3, help="How many retry rounds to use for failed handshakes"
    )
    parser.add_argument(
        "--use-real-towers", action="store_true", help="Resolve real TX towers via FCC API (persistent cache; US only)"
    )
    parser.add_argument("--validate", action="store_true", help="Enable validation against server API")
    parser.add_argument(
        "--validation-url", type=str, default="http://localhost:8000", help="Base URL for validation API calls"
    )
    parser.add_argument(
        "--ground-truth-path", type=str, default="ground_truth.json", help="Path to save ground truth data"
    )
    parser.add_argument(
        "--metro",
        type=str,
        default=None,
        choices=sorted(_KNOWN_METROS),
        help="Generate the whole fleet inside one metro area (drops "
        "solo/rural nodes and non-local rings). Applies at "
        "generation time, so it needs no --config.",
    )
    parser.add_argument(
        "--metros",
        type=str,
        default="",
        help="Comma-separated metro codes to focus on (e.g. atl,gvl). "
        "Filters an already-generated fleet to these metros. "
        f"Available: {','.join(_KNOWN_METROS.keys())}",
    )
    parser.add_argument(
        "--real-adsb",
        action="store_true",
        help="Relay real adsb.lol traffic over the configured metro areas "
        "into the server (tagged source=real so it is display-only for "
        "claiming). Off by default: the relay used to switch on with metro "
        "config alone, and the decoy transponders it added produced ghost "
        "planes on the map.",
    )
    parser.add_argument(
        "--live-adsb-url",
        type=str,
        default="",
        help="Base URL of an adsb.lol-shaped live feed (e.g. https://adsb.retina.fm). "
        "Real aircraft over the --metro/--metros areas then join the simulated "
        "world and are echoed by the synthetic nodes; the backend's "
        "frac_live_dark casts a share of them as dark. Empty = off.",
    )
    parser.add_argument(
        "--live-adsb-interval",
        type=float,
        default=5.0,
        help="Seconds between live feed polls (min 1)",
    )
    parser.add_argument(
        "--magnetometers",
        type=int,
        default=8,
        help="Magnetometers to place beside radar receive sites (0 = none). "
        "The backend's mag_n_nodes takes over at the first config poll.",
    )
    parser.add_argument(
        "--mag-placement",
        choices=("spread", "core", "random"),
        default="spread",
        help="How to choose the sites that get a magnetometer (see magnetometer.choose_sites)",
    )
    parser.add_argument(
        "--no-hub-radial",
        action="store_true",
        help="Disable hub-radial flight planning (use legacy random-anchor spawn)",
    )
    parser.add_argument(
        "--metro-traffic-frac",
        type=float,
        default=0.6,
        help="Fraction of spawns routed through metro coverage rings (rest en-route)",
    )
    args = parser.parse_args()

    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
