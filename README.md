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

The random-walker packs used for benchmarks live in `bench/scenarios`. To run
one on the krABMaga grid instead of the built-in one (links Bevy; the first
build is slow):

```sh
cargo run --release -p sim-axum --features krabmaga --bin krabshell -- \
  --scenarios bench/scenarios --scenario london-walkers --model walkers-krabmaga
```

## Layout

```
crates/
  sim-core/       types and A5 cell maths — no Bevy, Axum or krABMaga
  sim-engine/     driver thread, models, scenario packs, checkpoints, simctl
  sim-mavlink/    MAVLink gateway: drones as UDP vehicles for ground stations
  sim-cot/        Cursor-on-Target emitter: tracks and sightings for TAK
  sim-axum/       REST + WebSocket edge, auth, the krabshell binary
  sim-dashboard/  Askama templates and static assets (htmx, canvas map)
scenarios/        drone packs: london-patrol (gold), swarm-survey (fast)
bench/scenarios/  random-walker packs for benchmarks and the krABMaga replay
docs/             architecture notes and baseline numbers
```

## Features

| Cargo feature | Effect |
|---|---|
| *(none)* | Built-in sparse A5 grid. Fast to build, no Bevy. |
| `krabmaga` | Adds the `walkers-krabmaga` model on krABMaga's `SparseA5Grid` (`gis`). |
| `parallel` | As `krabmaga`, with krABMaga's `parallel` feature. |

Enable on the binary with `-p sim-axum --features krabmaga` or on the CLI with
`-p sim-engine --features krabmaga`.

## Operating

### Server

```
krabshell [--bind 127.0.0.1:8080] [--scenarios scenarios] [--scenario ID]
          [--model KIND] [--run] [--checkpoint-on-exit PATH]
          [--mavlink-bind 127.0.0.1:14551] [--mavlink-gcs 127.0.0.1:14550,...]
          [--mavlink-hz 5] [--no-mavlink]
          [--cot-dest 239.2.3.1:6969] [--cot-ttl 1] [--cot-iface IP] [--no-cot]
```

| Env | Meaning |
|---|---|
| `DASHBOARD_TOKEN` | Shared secret for the API and dashboard. Required off loopback. |
| `KRABSHELL_BIND`, `KRABSHELL_SCENARIOS`, `KRABSHELL_SCENARIO` | Same as the flags. |
| `KRABSHELL_MAVLINK_BIND`, `KRABSHELL_MAVLINK_GCS`, `KRABSHELL_COT_DEST`, `KRABSHELL_COT_IFACE` | Same as the flags. |

Ctrl-C pauses the driver, optionally writes a checkpoint, then joins the
thread.

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
- commands (`COMMAND_LONG`/`COMMAND_INT`): arm/disarm, takeoff, mission start,
  RTL, land, DO_SET_MODE, DO_REPOSITION, DO_PAUSE_CONTINUE, DO_SET_MISSION_CURRENT;
- the capability handshake QGroundControl needs for a generic autopilot
  (AUTOPILOT_VERSION with MISSION_INT).

Mission items run in the simulation: NAV_WAYPOINT, NAV_LOITER_UNLIM,
NAV_LOITER_TIME, NAV_RETURN_TO_LAUNCH, NAV_LAND, NAV_TAKEOFF, CONDITION_DELAY,
DO_JUMP and DO_CHANGE_SPEED. Altitudes are relative to home.

The simulation runs at full speed unless you set a rate on the dashboard, so
drones fly faster than real time; telemetry shows wherever the sim has got to.
10 ticks/s is real time in the example packs. When the simulation is paused,
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
| `drone` | `sysid` (1–250), `callsign`, optional `mission` (a list of `MISSION_ITEM_INT` fields: `seq`, `frame`, `command`, `param1`–`param4`, `x`, `y`, `z`) and `autostart` |
| `intruder` | `route`: `[[lon, lat, alt_m], ...]` flown in a loop, optional `speed_ms` |
| `person`, `vehicle` | optional `speed_ms`, `radius_m` |
| `poi` | `name` |

`params.toml` sets `tick_seconds`, flight (`cruise_ms`, `climb_ms`,
`takeoff_alt_m`, `rtl_alt_m`, `battery_drain_pct_s`, `battery_reserve_pct`),
the camera (`hfov_deg`, `max_range_m`, `p_detect`, `sensor_period_s`,
`report_interval_s`) and generated targets (`persons`, `vehicles`,
`wander_radius_m`, `origin_lon`, `origin_lat`).

## Development

```sh
cargo test --workspace                          # fast, no Bevy
cargo test --workspace --features sim-axum/krabmaga
cargo clippy --workspace --all-targets
```

CI runs formatting, clippy and tests without features, then the `krabmaga`
and `parallel` matrix with a gold replay on the krABMaga backend.
