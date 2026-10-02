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

A PNI RM3100 magnetometer can sit at some of the fleet's radar receive sites,
at most one per roof, so radar nodes that share a site share its
magnetometer. Each reads the field once per world step, in nanotesla, on
north, east and down axes:

| Part | Model | Size near Greenville, SC |
| --- | --- | --- |
| Main field | WMM2025 at the site through `pygeomag`, evaluated once a day per site | ~48,600 nT (F), 62° inclination |
| Crust | A fixed offset per site, drawn from the seed | 60 nT (σ) per component |
| Daily variation (Sq) | 4-harmonic solar-quiet model fitted to the FRD and BSL observatories on either side of Greenville, blended by season and solar cycle, with day-to-day variability | 30–90 nT peak to peak in its largest component (east) |
| Storm | Optional (`mag_storm`). A sudden commencement, a main phase pulling north towards 0.72 · Dst (Dst −150 nT), exponential recovery, and Pc4–Pc5 pulsations. It is the same at every site. | ~110 nT at its deepest |
| UAPs | Every anomalous object is a magnetic dipole of 1e9 A·m², with the field falling as 1/r³ | 200 nT at 1 km; 1,600 nT at 500 m |
| Noise | Gaussian, then quantised to the chip's counts, so that the readings show the datasheet's noise for the cycle count | 15 nT at 200 cycles, 13.3 nT per count |

Aircraft and drones contribute nothing: their moments are far too small to
register at these distances. A UAP is any object spawned as
`object_type == "anomalous"`, its dipole in a fixed direction drawn from the
seed, and the field is summed over every UAP within 50 km.

The noise figures (30, 20 and 15 nT at 50, 100 and 200 cycles) are from PNI's
datasheet, table 3-1, interpolated log-log (~11.3 nT at 400 cycles, ~8.4 nT
at 800). They already include the rounding to counts, so the simulator draws
√(σ² − LSB²/12) before rounding and its readings come out at the datasheet
figure. These are the same figures as the RM3100 simulator in
retina-magnetometer, which feeds a node app over a simulated I2C bus.

The fleet's seed alone fixes where the magnetometers sit, the crust, each
UAP's dipole direction, every noise sample, and a flyby's magnetometer and
heading when the request does not say. They come from hashing the seed,
never from the world's shared `random` module, so adding magnetometers
leaves a seed's radar scene as it was (`tests/test_magnetometer.py`,
`tests/test_magnetometer_fleet.py`).

### Placement

`choose_sites` takes the fleet's receive sites (one per roof, named for the
lowest node id there) and picks `mag_n_nodes` of them:

| `mag_placement` | Chooses |
| --- | --- |
| `spread` (default) | Farthest-point sampling from the site nearest the scene's centre: the network covers the fleet |
| `core` | The sites nearest the centre: a dense cluster, where one UAP is seen by several magnetometers |
| `random` | A seeded shuffle |

### What goes to the server

A push loop posts the readings taken since the server last answered to
`/api/sim/magnetometer/push` once a second, with the simulator's key, one
post at a time from a thread so the world never waits on the server. The
radar TCP frames and `/v1/nodes` are untouched.

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

The server's detector reads only `samples`; the `truth` rows are for scoring.
A post that times out (3 s) or gets a 5xx, 408 or 429 has failed, and its
readings wait for the next post, up to the last ten world steps. A post
refused with any other 4xx is dropped, not sent again, and the server's
reason is logged. A reading that fails is counted as `magnetometer_errors`.
Nothing about the magnetometers stops the radar fleet.

### Runtime settings

The orchestrator applies the `mag_*` keys in-process at the first config
poll after the server stamps a change, without a restart:

| Key | Default | Effect |
| --- | --- | --- |
| `mag_enabled` | true | Place magnetometers at all |
| `mag_n_nodes` | 8 | How many, never more than the fleet has sites |
| `mag_placement` | `spread` | See above |
| `mag_cycle_count` | 200 | The sensor's cycle count: its noise and its resolution |
| `mag_storm` | false | Run the storm, starting at the moment it is switched on |

The server's own `mag_threshold_sigma` and `mag_common_mode` reach the
simulator with the rest of the config and are ignored. `--magnetometers N`
and `--mag-placement` set the starting values until the first poll.

### UAP flybys

`mag_flyby` in the runtime config asks the fleet to fly one UAP past one
magnetometer on a straight, level pass with a given closest approach,
altitude, speed and heading (random if not given), drawn on the flat plane
around the magnetometer so it comes as close as asked. The server's
`POST /api/sim/magnetometers/flyby` writes it and leaves `_updated_at`
alone, so the fleet looks for a request at every poll. The pass reaches its
closest point `lead_s` seconds after the poll that picks it up. The fleet
flies each request once, telling new from old by the time it was made, and
ignores a request more than two minutes old when it first sees it. The UAP
is an ordinary anomalous world object with no transponder, so the radar
nodes echo it too.

Detection happens on the server, over the whole network, since only a
network can remove the daily variation and storms before looking for
something local. retina-server's design note on simulated magnetometers
covers the detector and its measured false-alarm rate.
