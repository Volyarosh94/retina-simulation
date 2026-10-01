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

From a fresh clone, with uv:

```bash
uv venv --python 3.12 && source .venv/bin/activate
# retina-custody is on no package index, so it comes from its repository.
uv pip install "retina-custody @ git+https://github.com/offworldlabs/retina-custody@v0.1.0"
uv pip install -e '.[dev]' pre-commit==4.6.2 vulture==2.14
pre-commit run --all-files
pytest
```

CI runs the same steps on Python 3.12, installing into the runner's own
Python (`uv pip install --system`) rather than a virtual environment.

The fleet runs against a local server as
`python -m retina_simulation.orchestrator`. The flags are covered in
retina-server's `docs/simulation.md`, under "Running the Fleet Locally".

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
| Noise | Gaussian, then quantised to the chip's counts, so that the readings show the datasheet's noise for the cycle count | 15 nT at 200 cycles, 13.3 nT per count |

Aircraft and drones contribute nothing: in the model these nodes use, their
moments are far too small to register at these distances.

A UAP is any object the world spawned as `object_type == "anomalous"`. Its
dipole points in a fixed direction drawn from the seed. The field is summed
over every UAP within 50 km, the distance beyond which a 1e9 A·m² dipole is
under 0.002 nT.

The noise figures (30, 20 and 15 nT at 50, 100 and 200 cycles) come from PNI's
datasheet, table 3-1. They are interpolated log-log and extended by the
nearest segment's slope, which gives ~11.3 nT at 400 cycles and ~8.4 nT at 800.
The gain is PNI's 0.3671·CC + 1.5 counts per µT. The table's figures are the
noise of the chip's output, so they already hold its rounding to counts, a
uniform error worth LSB²/12 of variance. The simulator therefore draws
√(σ² − LSB²/12) before rounding (14.5 nT for 15 at 200 cycles), and its
readings come out at the table's figure, which is also the noise the push
reports for the sensor. These are the same figures, and the same physics, as
the RM3100 simulator in retina-magnetometer, a separate repository with a
node app for a real RM3100 and a register-level simulator of the chip. That
simulator feeds the node app over a simulated I2C bus; this one feeds the
server.

### Determinism

The fleet's seed alone fixes:

- where the magnetometers sit;
- each site's crust;
- each UAP's dipole direction;
- every noise sample;
- which magnetometer a flyby passes, and on what heading, when its request
  does not say.

These come from hashing the seed with a stream name and an index, never from
the world's shared `random` module. Adding magnetometers therefore leaves the
radar scene of a given seed exactly as it was. `tests/test_magnetometer.py`
and `tests/test_magnetometer_fleet.py` hold both properties.

The fleet's seed is the one the fleet was generated with. The generator
stamps it into the config it writes, and the orchestrator takes it from
there; `--seed` stands in only when the orchestrator generates the fleet
itself or the config has no stamp. retina-server's fleet container passes
its `FLEET_SEED` to the generator alone, so the stamp is where it is found.

### Placement

`choose_sites` takes the fleet's receive sites (one per roof, named for the
lowest node id there) and picks `mag_n_nodes` of them:

| `mag_placement` | Chooses |
| --- | --- |
| `spread` (default) | Farthest-point sampling from the site nearest the scene's centre: the network covers the fleet |
| `core` | The sites nearest the centre: a dense cluster, where one UAP is seen by several magnetometers |
| `random` | A seeded shuffle |

### What goes to the server

The simulation loop reads every magnetometer once per world step, stamped
with that step's time, as it records the step's ground truth. Once a second a
push loop of its own posts the readings taken since the server last answered
to `/api/sim/magnetometer/push`, with the simulator's key. It posts one at a
time and from a thread, so waiting on the server never holds up the world.
A slow server delays readings but costs none, as long as it answers within a
post's 3 s timeout; a post it has not answered by then has failed (below).
This sits beside the ground-truth and ADS-B pushes. The radar TCP frames and
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

A post fails when the server does not answer in time, or answers with a 5xx,
a 408 or a 429. Its readings then wait for the next post, up to the last ten
world steps (10 s). An outage longer than that leaves a gap in the data; it
does not build a backlog. The orchestrator logs a warning when posts start
failing and a line when they recover.

A post the server refuses, with any other 4xx, is not sent again: the same
readings would only be refused again, and would hold back every reading after
them until the hold dropped them all. Its readings are dropped, and the
server's reason (the `detail` of its answer) is logged as a warning, once for
each kind of reason.

A reading that fails, in an environment missing `pygeomag` for instance, is
skipped and counted as `magnetometer_errors`, in the fleet's stats and its
STATS log line. It is logged at once, then at most once a minute. A
main-field model that cannot load is also reported at startup. Nothing about
the magnetometers stops the radar fleet.

### Runtime settings

The orchestrator applies the runtime configuration's `mag_*` keys in-process,
at the first config poll after the server stamps a change (the config's
`_updated_at`), as it does every other setting. No fleet restart is needed,
because the magnetometers hold no connection of their own.

| Key | Default | Effect |
| --- | --- | --- |
| `mag_enabled` | true | Place magnetometers at all |
| `mag_n_nodes` | 8 | How many, never more than the fleet has sites |
| `mag_placement` | `spread` | See above |
| `mag_cycle_count` | 200 | The sensor's cycle count: its noise and its resolution |
| `mag_storm` | false | Run the storm, starting at the moment it is switched on |

Changing the count or the placement re-places the magnetometers at once. The
cycle count changes the noise from the next reading on. The server also owns
`mag_threshold_sigma` and `mag_common_mode`, which set how it detects. They
reach the simulator with the rest of the polled config, and it ignores them.

The `--magnetometers N` and `--mag-placement` flags set the starting values.
The server's keys take over at the first poll.

### UAP flybys

`mag_flyby` in the runtime configuration asks the fleet to fly one UAP past one
magnetometer: a straight, level pass with a given closest approach, altitude,
speed and heading (random if not given). The line is straight on the flat
plane around the magnetometer, the plane the magnetometer measures distance
in, so the pass comes as close as asked at any speed and lead.

The server's `POST /api/sim/magnetometers/flyby` writes the request into the
config the fleet polls every 5 s, and leaves the config's `_updated_at` stamp
alone: a flyby is a command, not a setting, and a moved stamp would read on
the Physics Layer page as someone else's change. So the fleet looks for a
request at every poll, not only when the stamp moves, and the pass reaches
its closest point `lead_s` seconds after the poll that picks it up. A poll
that also changes settings applies them first, so a request can name a
magnetometer the same poll places. The usual way in is the Physics Layer
page (`/sim/physics`), or the button in a magnetometer's panel on the `/sim`
map.

- **Once each.** A running fleet flies each request once, however many polls
  serve it, and tells a new request from one it has flown by the time it was
  made. Requests also carry a number, but the server starts counting again
  when it restarts, and a fleet that kept running must not take every later
  request for an old one. A request replaced by a newer one before the next
  poll is not flown.
- **Staleness and restarts.** A request more than two minutes old when the
  fleet first sees it is ignored. That age is also all that keeps a restarted
  fleet from flying the last request again: the fleet remembers nothing
  across a restart, and the server goes on serving the last request. A
  restarted fleet whose first poll comes within two minutes of a request (a
  scene change right after the click, say) flies it again, in a new world
  where the first pass, finished or not, no longer exists. One back later
  than that does not.
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
