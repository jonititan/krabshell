# krabshell

An operator server for [krABMaga](https://github.com/krABMaga/krABMaga)
simulations on the A5 geospatial grid.

One process runs the simulation on a dedicated thread at full speed, and an
Axum edge beside it takes intents in, serves state out, and hosts a dashboard
for loading scenarios, driving the clock, inspecting cells and downloading
checkpoints. The simulation stays the authority: HTTP never steps the
schedule.

The example scenarios are drones. krabshell presents each simulated drone to
ground stations as a real MAVLink vehicle over UDP, and reports what the drones
see to TAK as Cursor-on-Target. It does the real-time half of both protocols
itself (heartbeats, telemetry, handshakes and timeouts, all on the wall clock),
so the simulation underneath can run as fast as it likes.

The drones model is a krABMaga model: every drone, person, vehicle, intruder
and point of interest is a krABMaga agent with its own `step()`, on a krABMaga
schedule, against a krABMaga `State` whose occupancy is a `SparseA5Grid`. It
lives in its own crate, `sim-drones`, and runs without krabshell too; see
[Without krabshell](#without-krabshell).

The design is in [`krabshell_spec.md`](krabshell_spec.md). How the code
implements it, and what was learned doing so, is in
[`docs/architecture.md`](docs/architecture.md). Speed numbers are in
[`docs/baseline.md`](docs/baseline.md).

## Quick start

```sh
cargo run --release -p sim-axum --bin krabshell -- --scenario london-patrol --run
```

Open <http://127.0.0.1:8080/>. With no `DASHBOARD_TOKEN` set the server accepts
any token and refuses to bind anything but loopback. The MAVLink gateway and the
CoT emitter start with it; see [Drones: MAVLink and CoT](#drones-mavlink-and-cot).

Every build links krABMaga with its `gis` feature, which pulls in Bevy, so the
first build is slow and needs Bevy's system libraries on Linux (`libasound2-dev
libudev-dev libwayland-dev libfontconfig1-dev libxkbcommon-dev pkg-config`).

The random-walker packs used for benchmarks live in `bench/scenarios`. They run
on the built-in grid (`walkers`) or on krABMaga's (`walkers-krabmaga`):

```sh
cargo run --release -p sim-axum --bin krabshell -- \
  --scenarios bench/scenarios --scenario london-walkers --model walkers-krabmaga
```

## Layout

```
crates/
  sim-core/       types, A5 cell maths, pack reading — no Bevy, Axum or krABMaga
  sim-drones/     the drones model as krABMaga agents and State; krab-drones
  sim-engine/     driver thread, Model seam, walkers, scenario store, checkpoints, simctl
  sim-mavlink/    MAVLink gateway: drones as UDP vehicles for ground stations
  sim-cot/        Cursor-on-Target emitter: tracks and sightings for TAK
  sim-axum/       REST + WebSocket edge, auth, the krabshell binary
  sim-dashboard/  Askama templates and static assets (htmx, canvas map)
scenarios/        drone packs: london-patrol (gold), kaunas-deliveries (gold),
                  swarm-survey (fast)
bench/scenarios/  random-walker packs for benchmarks and the krABMaga replay
docs/             architecture notes and baseline numbers
```

## Features

| Cargo feature | Effect |
|---|---|
| *(none)* | krABMaga with `gis`: the drones model and `walkers-krabmaga`. |
| `parallel` | krABMaga's `parallel` feature: its thread-safe grid and schedule. The drones schedule still runs on one thread, so gold hashes match the default build. |

Enable on the binary with `-p sim-axum --features parallel`, on `simctl` with
`-p sim-engine --features parallel`, or on `krab-drones` with
`-p sim-drones --features parallel`.

## Operating

### Server

```
krabshell [--bind 127.0.0.1:8080] [--scenarios scenarios] [--scenario ID]
          [--model KIND] [--run] [--checkpoint-on-exit PATH]
          [--mavlink-bind 127.0.0.1:14551] [--mavlink-gcs 127.0.0.1:14550,...]
          [--mavlink-hz 5] [--no-mavlink]
          [--cot-dest 239.2.3.1:6969] [--cot-ttl 1] [--cot-iface IP] [--no-cot]
          [--log-dir var/logs] [--log-every 60] [--no-logs]
```

| Env | Meaning |
|---|---|
| `DASHBOARD_TOKEN` | Shared secret for the API and dashboard. Required off loopback. |
| `KRABSHELL_BIND`, `KRABSHELL_SCENARIOS`, `KRABSHELL_SCENARIO` | Same as the flags. |
| `KRABSHELL_MAVLINK_BIND`, `KRABSHELL_MAVLINK_GCS`, `KRABSHELL_COT_DEST`, `KRABSHELL_COT_IFACE`, `KRABSHELL_LOG_DIR` | Same as the flags. |

Ctrl-C pauses the driver, optionally writes a checkpoint, then joins the
thread.

### Deliveries and run logs

A drones pack can carry an address book (see below). Every address then
orders at random, rarely and independently (`order_rate_per_address_day`), and
each order joins a first-come first-served queue at the depot that serves the
address. Drones with a `depot` are its couriers: the next one that is on its
home pad, disarmed, loaded (`dispatch_load_s` after landing) and charged to
`launch_battery_pct` is given the oldest order as a fresh mission: take off
(`dispatch_takeoff_m` above the pad), fly to the address at its `cruise_agl_m`
above the terrain, lower the parcel with `DO_WINCH` / `WINCH_DELIVER`, fly home,
descend to `dispatch_approach_m` and land. The order is delivered when the
winch completes, and a `delivery` report says when it was placed, dispatched
and delivered. A courier that comes home without delivering (low battery, a
ground station took it) puts its order back at the front of the queue. Orders
come from the run's seed, so `gold` runs place and deliver the same orders.

A drone whose battery reaches 0 % in the air crashes where it is: it leaves
the world, a `crash` report is logged, and the order it carried goes back to
the front of the queue. A crashed courier's duty goes to its hub's spare (a
drone with `"spare": "<depot>"`), and once the spares are used, to a new drone
on the wreck's pad. New drones get an agent id and MAVLink system id nothing
has had before. Every touchdown after a flight is a `landing` report with the
battery left.

### Geofences

Geofences are keep-out zones, at every altitude: circles and polygons. A
pack lists the ones it starts with in `world/geofences.json`, each with an
optional `name` and `minutes` (until it closes by itself):

```json
[
  { "name": "Kaunas Castle and Old Town", "circle": { "lat": 54.8985, "lon": 23.8860, "radius_m": 350 } },
  { "name": "Ąžuolynas park", "polygon": [[23.9330, 54.9035], [23.9500, 54.9045], [23.9525, 54.8975], [23.9355, 54.8962]] }
]
```

A fence over a drone's pad, or outside the world, stops the pack loading. A
reset opens the pack's fences again. While the run goes on, the dashboard's
Geofences tab, the API and intents open and close fences: `Custom` 110 with
`{"circle": {"lat", "lon", "radius_m"}}` or `{"polygon": [[lon, lat],
...]}` and optionally `"name"` and `"minutes"`; `Custom` 111 with `{"id"}`.
A ground station's fence plan (see below) arrives as `Custom` 112 and
replaces the fences ground stations set before.

The autopilot knows nothing of fences. The operator, who flies the drones the
simulation flies itself (couriers, and drones a pack starts on a mission),
keeps them `geofence_margin_m` clear the way a ground station would, by
MAVLink: it re-uploads the mission with detour waypoints round each fence
(`MISSION_ITEM_INT`, `DO_JUMP`s renumbered, `MISSION_SET_CURRENT`), turns a
return home into a mission round the fences, moves a drone a fence opened
round out of it with `DO_REPOSITION` and then resumes, and sends a courier
whose address is fenced home (its order waits for the fence to close). When a
fence closes, its detours come out again. Drones flown by a ground station
are left to it. A drone inside a fence in the air is a `breach` report, once
per fence it enters; opens, closes and reroutes are reports too
(`geofence`, `reroute`). Ground stations can download the fences as a fence
plan (see below).

A pack that sets `heatmap_resolution` or `order_rate_per_address_day` gets a
run log: krabshell writes a folder `var/logs/<scenario>-<UTC start>/` every
`--log-every` seconds and on shutdown, and a reset, a restored checkpoint or
another scenario starts a new one. It holds:

| File | What |
|---|---|
| `heatmap.geojson` | Drone-seconds of flight per A5 cell of `heatmap_resolution`, as a styled choropleth (log-scale YlOrRd classes in simplestyle `fill`/`stroke`, legend in the collection's `properties`). geojson.io shows it as it is; QGIS can style on `class`. Time parked or disarmed is not flight. |
| `deliveries.csv` | Every delivered order: depot, drone, address, ticks, and seconds waiting for a drone, flying, and in total. |
| `delivery-times.csv`, `delivery-times.svg` | Histogram of time from order to delivery, in whole-minute bins, with mean, median, 90th percentile and slowest. |
| `landings.csv` | The end of every flight: drone, landed or crashed, and the battery percentage it touched down with (-1 for a crash), with what replaced a crashed drone. |
| `landing-battery.csv`, `landing-battery.svg` | Histogram of battery at touchdown in 5 % bins, one sample per flight, crashes in a bin of their own at -1. |
| `geofences.geojson` | Every geofence that opened, with when it opened and closed and how often it was breached, and every breach as a point. Styled for geojson.io. |
| `breaches.csv` | Every breach: drone, fence, whether it flew in or the fence opened round it, and who was flying it. |
| `reroutes.csv` | Every set of MAVLink messages the operator sent to take a drone round a fence, and why. |
| `summary.json` | Totals (flights and crashes among them, geofences, breaches, reroutes), and any reports the logger missed. |

For a long run without the server, `simctl fly` flies the pack for some
simulated hours as fast as it can and writes the same folder
(`var/logs/<scenario>-<hours>h/`):

```sh
cargo run --release -p sim-engine --bin simctl -- fly kaunas-deliveries --hours 8
```

The dashboard's **Geofences** tab shows them: every open fence on the map
(and in a list with its shape, area, who opened it, when it closes and how
often it has been breached), and the latest opens, closes, breaches and
reroutes. Draw a circle (centre, then edge) or a polygon (corners, then click
the first again or press Enter) to open one, optionally for a set number of
minutes, and close any fence from the list. The State page's map draws the
fences too. Both maps have layers to show or hide: drones; delivery locations
(hubs and landmarks, open orders' addresses, recent drop-offs); people and
vehicles (where they are, and camera sightings); geofences; and the A5
occupancy grid. The choice is remembered in the browser.

### Drones: MAVLink and CoT

Step by step, with network layouts and troubleshooting:
[`docs/connecting.md`](docs/connecting.md) (QGroundControl and ATAK/WinTAK).

**MAVLink.** Every drone in the world is a separate MAVLink vehicle (its
`sysid`, component 1) on one UDP socket. krabshell listens on
`--mavlink-bind` and sends heartbeats and telemetry to each `--mavlink-gcs`
address, and to any ground station that has sent it a packet. What works:

- HEARTBEAT, SYS_STATUS, GLOBAL_POSITION_INT, ATTITUDE, VFR_HUD, HOME_POSITION,
  MISSION_CURRENT and MISSION_ITEM_REACHED;
- the mission protocol: upload (`MISSION_COUNT` → `MISSION_REQUEST_INT` →
  `MISSION_ITEM_INT` → `MISSION_ACK`), download and clear;
- the geofences as a fence plan (`MAV_MISSION_TYPE_FENCE`), advertised in
  AUTOPILOT_VERSION: download all of them as `NAV_FENCE_CIRCLE_EXCLUSION` and
  `NAV_FENCE_POLYGON_VERTEX_EXCLUSION` items; upload a plan to set the
  ground stations' own fences (the scenario's and the dashboard's stay as they
  are; inclusion fences are refused); clear it to close them. `MISSION_CURRENT.fence_id` changes
  whenever a fence opens or closes;
- commands (`COMMAND_LONG`/`COMMAND_INT`): arm/disarm, takeoff, mission start,
  RTL, land, DO_SET_MODE, DO_REPOSITION, DO_PAUSE_CONTINUE, DO_SET_MISSION_CURRENT;
- the capability handshake QGroundControl needs for a generic autopilot
  (AUTOPILOT_VERSION with MISSION_INT).

Mission items run in the simulation: NAV_WAYPOINT, NAV_LOITER_UNLIM,
NAV_LOITER_TIME, NAV_RETURN_TO_LAUNCH, NAV_LAND, NAV_TAKEOFF, CONDITION_DELAY,
DO_JUMP, DO_CHANGE_SPEED and DO_WINCH (the drone hovers while the winch runs).
Altitudes are above mean sea level, over the pack's terrain if it has one;
mission items may be relative to home, absolute, or terrain-following.

The simulation runs at full speed unless you choose otherwise under Speed on
the dashboard (Max, Real time, or a Custom rate in ticks per second), so drones
fly faster than real time; telemetry shows wherever the sim has got to. Real
time is one tick per `tick_seconds`: 10 ticks/s in the example packs. When the simulation is paused,
commands are refused with `TEMPORARILY_REJECTED`, and an upload is acknowledged
but applies on the next tick (a STATUSTEXT says so).

MAVLink has no authentication: anything that can reach the port can fly the
drones. krabshell warns when it is bound off loopback.

**CoT.** Drones go out once a second as friendly rotary-wing UAVs
(`a-f-A-M-H-Q`); sightings as unknown people (`a-u-G-U-C-I`), vehicles
(`a-u-G-E-V`) and drones (`a-u-A-M-H-Q`), each linked to the drone that saw it;
points of interest as spot markers (`b-m-p-s-m`) every 15 seconds. The default
destination is ATAK's SA multicast group, `239.2.3.1:6969`, with legacy XML,
which every TAK client accepts. `GET /api/v1/reports?since=N` serves the same
sightings as JSON.

**From Windows, when krabshell runs in WSL2.** WSL2's default NAT networking
does not pass UDP between Windows' localhost and WSL's, and multicast does not
leave it. Either:

- switch WSL to mirrored networking (`[wsl2]` / `networkingMode=mirrored` in
  `%UserProfile%\.wslconfig`, then `wsl --shutdown`), and the defaults reach
  QGroundControl and WinTAK on the same machine; or
- point krabshell at Windows explicitly:

  ```sh
  WIN=$(ip route show default | awk '{print $3}')
  krabshell --scenario london-patrol --run \
    --mavlink-bind 0.0.0.0:14551 --mavlink-gcs $WIN:14550 \
    --cot-dest $WIN:4242   # a UDP input added in WinTAK's network settings
  ```

  Windows Firewall may ask to let QGroundControl and WinTAK receive UDP.

For a phone running ATAK, send CoT unicast to the phone's address and a UDP
input configured on it (`--cot-dest <phone-ip>:<port>`).

### Map basemaps

The State page can draw OpenStreetMap or Esri tiles under the cells. They are
off by default. Tiles are fetched by the operator's browser, not the server, so
they need internet access from the browser and fall under each provider's
usage policy. The list lives in `BASEMAPS` in
`crates/sim-axum/src/routes/pages.rs`.

### Headless

```sh
cargo run --release -p sim-engine --bin simctl -- list
cargo run --release -p sim-engine --bin simctl -- --scenarios bench/scenarios bench global-drift
cargo run --release -p sim-engine --bin simctl -- --scenarios bench/scenarios replay london-walkers --ticks 500
cargo run --release -p sim-engine --bin simctl -- cell --lon -0.1276 --lat 51.5072 --res 15
cargo run --release -p sim-engine --bin simctl -- fly kaunas-deliveries --hours 8
```

### Without krabshell

`krab-drones` runs a drones pack on krABMaga alone: no driver, server, gateways
or dashboard. It reads the pack, builds the `World` (the krABMaga `State`),
and steps it on a krABMaga `Schedule` the way `simulate!` does:

```sh
cargo run --release -p sim-drones --bin krab-drones -- scenarios/kaunas-deliveries --steps 36000
cargo run --release -p sim-drones --bin krab-drones -- scenarios/london-patrol --reports reports.jsonl
cargo run --release -p sim-drones --bin krab-drones -- scenarios/london-patrol --tui
```

It prints agents, occupied cells, steps per second, reports by kind, orders
and the gold hash. `--reports` writes every detection, waypoint and delivery as
JSON lines. `--tui` hands the world to krABMaga's `simulate!` with its terminal
monitor instead, and prints no summary.

To embed the model in your own krABMaga program, depend on `sim-drones`:

```rust
use sim_drones::krabmaga::engine::state::State;

let blob = sim_core::read_pack("scenarios/london-patrol".as_ref())?;
let mut world = sim_drones::World::new(&blob)?;
let mut schedule = sim_drones::schedule();
world.init(&mut schedule);
for _ in 0..1_000 {
    schedule.step(&mut world);
}
```

### API

Everything is under `/api/v1`. Send `Authorization: Bearer $DASHBOARD_TOKEN`.

```sh
T="Authorization: Bearer $DASHBOARD_TOKEN"
curl -s -H "$T" -X POST localhost:8080/api/v1/run/start -d '{}' -H 'content-type: application/json'
curl -s -H "$T" localhost:8080/api/v1/run
curl -s -H "$T" -X POST localhost:8080/api/v1/run/step -d '{"n":100}' -H 'content-type: application/json'
curl -s -H "$T" "localhost:8080/api/v1/cells/lookup?lon=-0.1276&lat=51.5072"
curl -s -H "$T" localhost:8080/api/v1/state/download -o world.ckpt.json
```

| Method | Path | |
|---|---|---|
| GET | `/health` | No token needed. 503 when the driver is dead or faulted. |
| GET | `/run` | Mode, tick, rates, counters, recent intents. |
| POST | `/run/start` `/run/pause` `/run/step` `/run/reset` `/run/rate` | Clock control. |
| GET POST | `/scenarios`, `/scenarios/{id}/load` | List, upload (multipart zip), load. |
| GET PUT | `/config`, `/config/download` | Effective config; changes that alter the address space reset the world. |
| GET | `/state?cells=…` or `?center=…&k=…` | Bags for a cell set. |
| GET | `/state/map?res=N` | Occupancy aggregated to parent cells, with polygons. |
| GET | `/state/cells/{cell}` | One bag. |
| GET POST | `/state/download`, `/state/upload` | Checkpoint out and in (restore requires pause). |
| POST | `/intents` | Submit one intent. |
| GET | `/intents/log` | JSON Lines replay log (gold profile). |
| GET | `/metrics` | Prometheus text. |
| GET | `/reports?since=N` | Model reports (sightings, waypoints reached) newer than `seq` N. |
| GET | `/cells/lookup?lon=&lat=&res=` | Coordinate to cell. |
| GET | `/state/overlays` | What the model publishes beside its agents: geofences, open orders' addresses. |
| GET POST | `/geofences` | Open geofences, with breach counts and recent fence activity; open one (`{"circle": {"lat", "lon", "radius_m"}}` or `{"polygon": [[lon, lat], …]}`, optional `"minutes"`). |
| DELETE | `/geofences/{id}` | Close a geofence, whoever opened it. |
| GET | `/ws` | Live protocol: subscribe, AOI, intents, deltas. |

Errors are `application/problem+json` with a stable `code`.

### WebSocket

Connect to `/api/v1/ws?token=…`. Messages are JSON with a `type` tag.

```json
{"type": "set_aoi", "center": "636119e9a0000000", "k": 2}
{"type": "intent", "intent": {"id": "…uuid…", "cell": "636119e9a0000000",
  "actor": "me", "verb": "spawn", "payload": {"count": 3}}}
```

The server sends `hello`, then `snapshot` when your area changes, `delta` as
it changes, `accepted`/`rejected` for each intent, and a `tick` heartbeat at
`publish_hz`.

## Scenario packs

```
scenarios/<id>/
  manifest.toml       id, name, model, resolution, root, profile
  config.toml         [run] and [grid] defaults (optional)
  params.toml         model parameters (optional)
  world/agents.json   seeded agents by cell or lon/lat (optional)
  world/fields.json   non-grid model state (optional)
```

The manifest owns the address space and wins over `config.toml`. Use
`simctl cell` to find a `root` cell id. See `scenarios/london-patrol` for a
commented example.

A `drones` pack seeds `world/agents.json` with entities of five kinds, each by
`lon`/`lat`:

| Kind | `state` |
|---|---|
| `drone` | `sysid` (1–250), `callsign`, optional `elevation_m` (what its pad stands on, above mean sea level, e.g. a roof; the terrain if left out), optional `mission` (a list of `MISSION_ITEM_INT` fields: `seq`, `frame`, `command`, `param1`–`param4`, `x`, `y`, `z`) and `autostart`, or optional `depot` (a depot in the address book, making it a courier) and `cruise_agl_m` |
| `intruder` | `route`: `[[lon, lat, alt_m], ...]` flown in a loop, altitudes above the ground at each point, optional `speed_ms` |
| `person`, `vehicle` | optional `speed_ms`, `radius_m` |
| `poi` | `name` |

`params.toml` sets `tick_seconds`, flight (`cruise_ms`, `climb_ms`,
`takeoff_alt_m`, `rtl_alt_m`), the battery (`battery_drain_pct_s`,
`battery_reserve_pct`, `charge_pct_s` on the home pad, `launch_battery_pct`
below which a takeoff from the ground waits or is refused), the winch
(`winch_drop_ms`, `winch_slow_ms`, `winch_slow_m`, `winch_release_s`,
`winch_reel_ms`, `winch_lock_s`), the camera (`hfov_deg`, `max_range_m`,
`p_detect`, `sensor_period_s`, `report_interval_s`) and generated targets
(`persons`, `vehicles`, `wander_radius_m`, `origin_lon`, `origin_lat`),
deliveries (`order_rate_per_address_day`, `dispatch_takeoff_m`,
`dispatch_approach_m`, `dispatch_load_s`) and `heatmap_resolution` (see
[Deliveries and run logs](#deliveries-and-run-logs)).

Terrain goes in `world/fields.json` as a regular latitude/longitude grid of
elevations above mean sea level; the format is documented in
`crates/sim-drones/src/terrain.rs`. Without it the ground is flat
at 0 m. People, vehicles and points of interest sit on the terrain, and the
camera's footprint grows with height above it. The address book goes in the
same file under `"addresses"`: `{"depots": ["MEGA", ...], "items": [[lat_e7,
lon_e7, depot_index, "label"], ...]}`. `scenarios/kaunas-deliveries` has a
Copernicus GLO-30 grid and every OpenStreetMap address within 5 km of its hubs (34502), built by
`tools/kaunas_deliveries.py`.

## Development

```sh
cargo test --workspace
cargo test --workspace --features sim-axum/parallel,sim-drones/parallel
cargo clippy --workspace --all-targets
```

Every test binary links Bevy; `CARGO_PROFILE_DEV_DEBUG=0
CARGO_PROFILE_TEST_DEBUG=0` keeps them small. CI runs formatting, clippy, tests
and gold replays (drones through `simctl` and standalone through
`krab-drones`, walkers on krABMaga's grid) on the default build and again with
`parallel`.
