# retina-simulation

The synthetic fleet behind the RETINA server's `/sim` map. Radar nodes watch
one simulated world of aircraft, drones and anomalous objects, and speak the
node protocol to the server over TCP. Magnetometers sit beside some of those
nodes and read the same world.

retina-server vendors this library as its `libs/retina-simulation` submodule.
Its [`docs/simulation.md`](https://github.com/offworldlabs/retina-server/blob/main/docs/simulation.md)
is the reference for the fleet, for the runtime configuration it polls and
for the protocol. This file covers what that page does not: working on the
library itself, and the magnetometers.

## Working on it

```bash
# retina-custody is on no package index; CI installs it the same way.
uv pip install "retina-custody @ git+https://github.com/offworldlabs/retina-custody@v0.1.0"
uv pip install -e '.[dev]' pre-commit==4.6.2 vulture==2.14
pre-commit run --all-files
pytest
```

The fleet runs against a local server as `python -m retina_simulation.orchestrator`.
The flags are covered in retina-server's `docs/simulation.md`, under
"Running the Fleet Locally".

## Magnetometer nodes

A PNI RM3100 magnetometer can sit at some of the fleet's radar receive sites.
There is at most one per roof, so two radar nodes that share a site share its
magnetometer. Each one reads the field once per world step, in nanotesla, on
north, east and down axes:

| Part | Model | Size near Greenville, SC |
| --- | --- | --- |
| Main field | WMM2025 at the site through `pygeomag`, evaluated once a day per site | ~48,600 nT (F), 62° inclination |
| Crust | A fixed offset per site, drawn from the seed | 60 nT (σ) per component |
| Daily variation (Sq) | 4-harmonic solar-quiet model fitted to the FRD and BSL observatories on either side of Greenville, blended by season and solar cycle, with day-to-day variability | 30–90 nT peak to peak in its largest component (east) |
| Storm | Optional (`mag_storm`). A sudden commencement, a main phase pulling north towards 0.72 · Dst (Dst −150 nT), exponential recovery, and Pc4–Pc5 pulsations. It is the same at every site. | ~110 nT at its deepest |
| UAPs | Every anomalous object is a magnetic dipole of 1e9 A·m², with the field falling as 1/r³ | 200 nT at 1 km; 1,600 nT at 500 m |
| Noise | Datasheet noise for the cycle count, then quantised to the chip's counts | 15 nT at 200 cycles, 13.3 nT per count |

Aircraft and drones contribute nothing. Their moments are far too small at
these distances, which is the task's model.

A UAP is any object the world spawned as `object_type == "anomalous"`. Its
dipole points in a fixed direction drawn from the seed. The field is summed
over every UAP within 50 km, the distance beyond which a 1e9 A·m² dipole is
under 0.002 nT.

The noise figures (30, 20 and 15 nT at 50, 100 and 200 cycles) come from PNI's
datasheet, table 3-1. They are interpolated log-log and extended by the
nearest segment's slope, which gives ~11.3 nT at 400 cycles and ~8.4 nT at 800.
The gain is PNI's 0.3671·CC + 1.5 counts per µT. These are the same figures,
and the same physics, as the standalone
[retina-magnetometer](https://github.com/Volyarosh94/retina-magnetometer)
simulator. That simulator feeds a real node app over a simulated I2C bus;
this one feeds the server.

### Determinism

The seed alone fixes:

- where the magnetometers sit;
- each site's crust;
- each UAP's dipole direction;
- every noise sample.

These come from hashing the seed with a stream name and an index, never from
the world's shared `random` module. Adding magnetometers therefore leaves the
radar scene of a given seed exactly as it was. `tests/test_magnetometer.py`
holds both properties.

### Placement

`choose_sites` takes the fleet's receive sites (one per roof, named for the
lowest node id there) and picks `mag_n_nodes` of them:

| `mag_placement` | Chooses |
| --- | --- |
| `spread` (default) | Farthest-point sampling from the site nearest the scene's centre: the network covers the fleet |
| `core` | The sites nearest the centre: a dense cluster, where one UAP is seen by several magnetometers |
| `random` | A seeded shuffle |

### What goes to the server

Once a second the orchestrator posts the readings from each world step since
the last post to `/api/sim/magnetometer/push`, with the simulator's key. This
sits beside the ground-truth and ADS-B pushes. The radar TCP frames and
`/v1/nodes` are untouched.

```jsonc
{
  "ts_ms": 1790726400000,
  "sensor": {"model": "RM3100", "cycle_count": 200, "noise_nt": 15.0, "lsb_nt": 13.348},
  "uap_moment_am2": 1e9,
  "storm": false,
  "nodes": [{
    "node_id": "mag-synth-GVL-SCAT-0007",   // "mag-" + the radar node it sits beside
    "host_node_id": "synth-GVL-SCAT-0007",
    "lat": 34.91, "lon": -82.33, "alt_m": 290.0,
    "samples": [[1790726400000, 22371.4, -2745.1, 43012.9]],   // t_ms, north, east, down (nT)
    "truth": [[1790726400000, 0.0, null, null]]                  // t_ms, UAP field (nT), nearest UAP, its slant range (km)
  }]
}
```

The `truth` rows are there to score detections, never to make them. The
server's detector reads only `samples`.

If a post fails, the orchestrator keeps the last 10 s of readings and sends
them with the next post. An outage longer than that leaves a gap in the data;
it does not build a backlog.

### Runtime settings

The orchestrator applies the runtime configuration's `mag_*` keys at each
config poll, in-process. No fleet restart is needed, because the
magnetometers hold no connection of their own.

| Key | Default | Effect |
| --- | --- | --- |
| `mag_enabled` | true | Place magnetometers at all |
| `mag_n_nodes` | 8 | How many, never more than the fleet has sites |
| `mag_placement` | `spread` | See above |
| `mag_cycle_count` | 200 | The sensor's cycle count: its noise and its resolution |
| `mag_storm` | false | Run the storm, starting at the moment it is switched on |

Changing the count or the placement re-places the magnetometers at once. The
cycle count changes the noise from the next reading on. The server also owns
`mag_threshold_sigma` and `mag_common_mode`, which set how it detects and
never reach the simulator.

The `--magnetometers N` and `--mag-placement` flags set the starting values.
The server's keys take over at the first poll.

### UAP flybys

`mag_flyby` in the runtime configuration asks the fleet to fly one UAP past one
magnetometer: a straight, level pass with a given closest approach, altitude,
speed and heading (random if not given). It reaches its closest point `lead_s`
seconds after the fleet picks the request up, and the fleet polls every 5 s.
The server's `POST /api/sim/magnetometers/flyby` writes the request. The
usual way in is the Physics Layer page (`/sim/physics`), or the button in a
magnetometer's panel on the `/sim` map.

- **Once each.** The fleet flies each request once, and tells a new request
  from one it has flown by the time it was made. Requests also carry a
  number, but the server starts counting again when it restarts, and a fleet
  that kept running must not take every later request for an old one. A
  request replaced by a newer one before the next poll is not flown.
- **Staleness.** A request more than two minutes old when the fleet first
  sees it is ignored. This stops a fleet that restarts from replaying the
  last pass.
- **What the pass is.** An ordinary anomalous world object on a scripted
  track, with no transponder. The radar nodes echo it when it crosses a beam,
  and the ground-truth push carries it. It retires once it is as far past the
  magnetometer as it started before it.

### Where detection happens

Not here. The simulator produces readings and the truth behind them; the
server detects, over the whole network at once. The daily variation and a
storm are the same at every site, and only a network can take them away
before it looks for something local. retina-server's design note on simulated
magnetometers explains the detector, its calibration and its measured false
alarm rate.
