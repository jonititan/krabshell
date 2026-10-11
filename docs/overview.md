# krabshell

krabshell is an operator server for [krABMaga](https://github.com/krABMaga/krABMaga) simulations on the A5 geospatial grid. One process runs the simulation on a thread of its own, as fast as the model allows, and beside it an Axum edge takes intents in, serves state out and hosts a dashboard for loading scenarios, driving the clock, inspecting cells and downloading checkpoints. The simulation stays the authority: nothing outside it steps the schedule or touches model state. The example scenarios are delivery, patrol and exercise drones, some sharing the sky with manned aircraft. krabshell presents the drones to ground stations as MAVLink vehicles and everything in the air to TAK as Cursor-on-Target, doing the real-time half of both protocols itself so the simulation underneath never has to slow down.

## The pieces

Everything below runs in the one `krabshell` process. Each piece reads the simulation's published view and, if it wants to change the world, submits an intent. None of them can make a tick wait.

| Piece | Crate | What it does |
|---|---|---|
| Simulation thread | `sim-engine` | One OS thread, `sim-driver`, owns the world. It ticks, applies queued intents at the start of each tick, and publishes a read-only view up to `publish_hz` times a second (20 by default). It never awaits, opens a socket or reads a file during a tick. |
| Models | `sim-drones`, `sim-engine` | `drones` (the krABMaga drones model: drones, intruder drones, manned aircraft on published routes, people, vehicles and points of interest), and `walkers` and `walkers-krabmaga` (random walkers used for benchmarks, on the built-in grid and on krABMaga's `SparseA5Grid`). |
| Rewind | `sim-engine` | In the `gold` profile, a `sim-history` thread keeps a keyframe of the drones world every `run.history_every_ticks` ticks (100 by default) and rebuilds any recent tick on request, for the dashboard's timeline and `?at=` on the state endpoints. |
| HTTP edge | `sim-axum` | REST under `/api/v1`, the live WebSocket at `/api/v1/ws`, token auth and per-actor rate limiting. |
| Dashboard | `sim-axum`, `sim-dashboard` | Server-rendered pages (Askama templates, htmx) for the run, scenarios, the map, geofences, stations, intents, import and export, weather, conflicts and these docs. Its assets are compiled into the binary, so it works with no internet. |
| MAVLink gateway | `sim-mavlink` | Presents every drone as its own MAVLink vehicle on one UDP socket: heartbeats, telemetry, the mission protocol, the fence plan and commands. Listens on `127.0.0.1:14551` and sends to QGroundControl on `127.0.0.1:14550` by default. |
| CoT emitter | `sim-cot` | Sends drone and manned aircraft tracks, camera sightings and points of interest to TAK as Cursor-on-Target XML over UDP, to ATAK's multicast group `239.2.3.1:6969` by default. |
| Weather | `sim-weather`, `sim-axum` | Fetches wind per A5 cell from Open-Meteo, recorded or live, for packs that ask for it, and caches it on disk. |
| Conflicts | `sim-conflict`, `sim-drones`, `sim-axum` | Predicts where the aircraft in the air could meet over the next minute and advises how to keep them apart. The drones model assesses inside the step every few simulated seconds, and its drones and manned aircraft fly the advisories. The Conflicts screen shows the same assessment, made off the tick thread and only while someone is looking. |
| Run logger | `sim-axum` | For packs that ask for one, writes a flight heatmap, deliveries, landings and geofence activity to `var/logs/`. |

How these fit together, and why, is in [Architecture notes](architecture.md).

## Quick start

### Prerequisites

- Rust 1.95 or later (the workspace's `rust-version`; `rust-toolchain.toml` picks stable).
- Every build links krABMaga with its `gis` feature, which pulls in Bevy. On Linux that needs Bevy's system libraries:

  ```sh
  sudo apt-get install libasound2-dev libudev-dev libwayland-dev libfontconfig1-dev libxkbcommon-dev pkg-config
  ```

  The first build is slow for the same reason.

### Run

```sh
cargo run --release -p sim-axum --bin krabshell -- --scenario london-patrol --run
```

This loads the `london-patrol` pack from `scenarios/` and starts it at maximum speed. Without `--run` the world loads paused; without `--scenario` the server starts with an empty world. The MAVLink gateway and the CoT emitter start with it. Stop it with Ctrl-C.

### Open the dashboard

Open <http://127.0.0.1:8080/>. It redirects to the Run page, where the clock, speed and scenario controls are.

### The token

The server has one shared secret, `DASHBOARD_TOKEN`, read from the environment at start-up.

- **Unset** (or blank): every request is accepted, and a warning is logged. The server then refuses to start on any address but loopback.
- **Set**: the dashboard asks for it once at `/login` and keeps it in a cookie; API clients send it as `Authorization: Bearer <token>`; WebSocket clients add `?token=<token>`.

```sh
DASHBOARD_TOKEN=change-me cargo run --release -p sim-axum --bin krabshell -- \
  --bind 0.0.0.0:8080 --scenario london-patrol
```

MAVLink and CoT have no authentication of their own. Details, and every flag, are in [Operating the server](operating.md).

## Where to go next

- [Operating the server](operating.md): every flag and environment variable, auth, rate limits, shutdown, run logs, `simctl`.
- [The dashboard](dashboard.md): each page and what its controls do.
- [The map](map.md): layers, symbols, basemaps and drawing on the map.
- [Scenario packs](scenarios.md): the pack layout, manifests, config and model parameters.
- [Drones](drones.md): what the drones model simulates: flight, missions, batteries, deliveries, sensors and the manned aircraft beside them.
- [Drone behaviours](drone-behaviours.md): authoring and uploading the behaviour tree every drone runs.
- [Geofences](geofences.md): keep-out zones, altitude bands, and how drones are routed round them.
- [Stations and regions](stations.md): launch and recovery stations, regions of interest, surveys and delivery range.
- [Weather](weather.md): recorded and live wind from Open-Meteo, caching and licensing.
- [Conflicts](conflicts.md): conflict prediction, resolution advisories and how they are flown.
- [Connecting ground stations and TAK](connecting.md): QGroundControl, ATAK and WinTAK, step by step.
- [Protocols](protocols.md): what the MAVLink gateway and the CoT emitter speak.
- [API guide](api-guide.md): using the REST API and the WebSocket.
- [API reference](/docs/api): every endpoint, generated from the OpenAPI document.
- [Architecture notes](architecture.md): how the code implements the design, and what was learned.
- [Speed baseline](baseline.md): measured tick rates and the cost of the edge.
- [Development](development.md): the workspace, building, tests and keeping the API document in sync.
