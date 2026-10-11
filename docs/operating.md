# Operating the server

The `krabshell` binary is the simulation driver, the HTTP edge, the dashboard, the MAVLink gateway, the CoT emitter, the weather refresher, the conflict service and the run logger in one process. This page covers how to start it, every flag and environment variable it reads, who it lets in, what it limits, what it writes to disk and how it stops. For running scenarios with no server at all, see [Headless: simctl](#headless-simctl). Everything here is taken from [`main.rs`](../crates/sim-axum/src/main.rs), [`auth.rs`](../crates/sim-axum/src/auth.rs) and [`runlog.rs`](../crates/sim-axum/src/runlog.rs).

## Starting it

```sh
cargo run --release -p sim-axum --bin krabshell -- --scenario london-patrol --run
```

or, once built:

```sh
cargo build --release -p sim-axum --bin krabshell
target/release/krabshell --scenario london-patrol --run
```

Paths such as `--scenarios`, `--log-dir` and `--weather-cache` are relative to the working directory, so run it from the repository root unless you pass absolute paths.

Start-up runs in a fixed order, and any step that fails stops the process with a message saying why:

1. Read `DASHBOARD_TOKEN`. With no token and a non-loopback `--bind`, refuse to start.
2. Load the `--scenario` pack from `--scenarios`, or an empty world (the `walkers` model, no agents) if none is given. Apply `--model` if given.
3. Fetch the pack's weather, if it asks for any. A `recorded` pack whose weather cannot be fetched stops start-up here.
4. Spawn the driver thread and build the world. With `--run`, start the clock.
5. Bind the MAVLink gateway and start the CoT emitter, unless switched off.
6. Start the weather refresher, the conflict service and, unless `--no-logs`, the run logger.
7. Bind the HTTP listener and log `dashboard on http://<address>/`.

The world is built before the listener binds, so the server never answers with a half-loaded scenario.

## Flags and environment variables

Every flag can be given on the command line. Some can also come from the environment; a flag on the command line wins over its variable. `krabshell --help` prints the same list.

### Server and scenario

| Flag | Env | Default | Meaning |
|---|---|---|---|
| `--bind ADDR` | `KRABSHELL_BIND` | `127.0.0.1:8080` | Address the HTTP server listens on. Anything but loopback needs `DASHBOARD_TOKEN`. |
| `--scenarios DIR` | `KRABSHELL_SCENARIOS` | `scenarios` | Directory of scenario packs. Uploaded packs go here too. |
| `--scenario ID` | `KRABSHELL_SCENARIO` | none | Pack to load at start-up, by the name of its directory in `--scenarios`. Without it the server starts with an empty world. |
| `--model KIND` | | none | Run the start-up pack on a different model than its manifest names: `drones`, `walkers` or `walkers-krabmaga`. |
| `--run` | | off | Start running at once, at maximum speed, instead of paused. |
| `--checkpoint-on-exit PATH` | | none | Write a checkpoint to `PATH` on a clean shutdown. See [Shutdown](#shutdown). |
| `--allowed-origin URL` | `KRABSHELL_ALLOWED_ORIGINS` | none | Another origin (`scheme://host[:port]`) that may change things, such as the public address behind a reverse proxy. Repeatable, or comma-separated in the variable. See [Requests from other sites](#requests-from-other-sites). |

### MAVLink gateway

| Flag | Env | Default | Meaning |
|---|---|---|---|
| `--mavlink-bind ADDR` | `KRABSHELL_MAVLINK_BIND` | `127.0.0.1:14551` | UDP address the gateway listens on. |
| `--mavlink-gcs ADDR,...` | `KRABSHELL_MAVLINK_GCS` | `127.0.0.1:14550` | Ground stations to send telemetry to without waiting to hear from them. Comma-separated, or repeat the flag. QGroundControl listens on 14550. Any ground station that sends a packet is sent telemetry as well. |
| `--mavlink-hz HZ` | | `5` | Position and attitude telemetry rate per drone. |
| `--no-mavlink` | | off | Do not start the gateway. |

MAVLink has no authentication: anything that can reach the port can command the drones. krabshell logs a warning when `--mavlink-bind` is not loopback. If the port is taken, start-up fails and says to use `--mavlink-bind` or `--no-mavlink`.

### Cursor-on-Target emitter

| Flag | Env | Default | Meaning |
|---|---|---|---|
| `--cot-dest ADDR` | `KRABSHELL_COT_DEST` | `239.2.3.1:6969` | Where CoT events go: ATAK's SA multicast group, or any unicast `host:port`. |
| `--cot-ttl N` | | `1` | Multicast TTL. 1 keeps it on the local network. |
| `--cot-iface IPV4` | `KRABSHELL_COT_IFACE` | none | Local IPv4 address of the interface to send from. |
| `--no-cot` | | off | Do not start the emitter. |

How to reach QGroundControl and TAK clients from WSL2, another machine or a phone is in [Connecting ground stations and TAK](connecting.md).

### Run logs

| Flag | Env | Default | Meaning |
|---|---|---|---|
| `--log-dir DIR` | `KRABSHELL_LOG_DIR` | `var/logs` | Where run logs go, one folder per run. |
| `--log-every SECONDS` | | `60` | How often the run log is rewritten, in wall-clock seconds. Values below 1 are treated as 1. |
| `--no-logs` | | off | Do not write run logs. |

### Weather

| Flag | Env | Default | Meaning |
|---|---|---|---|
| `--weather-cache DIR` | `KRABSHELL_WEATHER_CACHE` | `var/weather-cache` | Where fetched weather is cached, so a scenario run again does not fetch it again. |
| `--no-weather` | | off | Never fetch weather. Packs asking for `recorded` weather then refuse to load, `live` ones fly in still air, and a pack's own `world/weather.json` still works. |
| `--open-meteo-key KEY` | `OPEN_METEO_API_KEY` | none | Open-Meteo API key. The free API is for non-commercial use only. The value is hidden from `--help`. |
| `--open-meteo-url URL` | `KRABSHELL_OPEN_METEO_URL` | Open-Meteo's | Root URL of a forecast API to use instead of the public one, such as a self-hosted instance. |
| `--open-meteo-historical-url URL` | `KRABSHELL_OPEN_METEO_HISTORICAL_URL` | Open-Meteo's | Root URL of a historical forecast API, likewise. |

What the weather does to the drones, and how packs ask for it, is in [Weather](weather.md).

### Other environment variables

| Env | Meaning |
|---|---|
| `DASHBOARD_TOKEN` | The shared secret for the API and dashboard. See [Authentication](#authentication). |
| `RUST_LOG` | Log filter, in `tracing-subscriber`'s syntax. Default `info,tower_http=warn`. `RUST_LOG=debug,tower_http=debug` logs every request. |

## Authentication

krabshell has one shared token for the whole server, read from `DASHBOARD_TOKEN` when it starts. There are no user accounts. Leading and trailing whitespace is trimmed, and a blank value counts as unset.

### Without a token

Every request is accepted and a warning is logged at start-up. The server will only bind a loopback address (`127.0.0.0/8` or `::1`). Anything else stops start-up:

```text
refusing to bind 0.0.0.0:8080 without a token: set DASHBOARD_TOKEN, or bind 127.0.0.1
```

This is a hard rule because the dashboard can reset the world, restore checkpoints and upload scenario packs.

### With a token

A request is let in if it carries the token in any of three places:

| Where | For |
|---|---|
| `Authorization: Bearer <token>` header | Scripts and tools. |
| `krabshell_token` cookie | The dashboard. The login form sets it: `HttpOnly`, `SameSite=Strict`, `Path=/`, kept for seven days. It has no `Secure` flag, since the default bind is plain HTTP. |
| `?token=<token>` query parameter | The WebSocket, where browsers cannot set headers. |

A request without a valid token to `/api/v1/*` gets `401` with an `application/problem+json` body whose `code` is `unauthorized`. A dashboard page redirects to `/login`. Posting the right token there sets the cookie and goes to the Run page; `POST /logout` clears it.

These need no token at all: `GET /api/v1/health`, `GET /api/v1/openapi.json`, `/login`, `/logout`, everything under `/docs`, and the dashboard's static files under `/assets/`.

### Requests from other sites

A token says who may ask, not from where. A browser sends the dashboard's
cookie with a form that another site posts to krabshell, and in open mode
there is no token to send at all. So every request that changes something
(anything but `GET`, `HEAD` and `OPTIONS`), and every WebSocket, must come
from the dashboard's own origin or from a tool that is not a browser:

- one whose `Origin` header is not this server's own (the scheme and the
  `Host` it was sent to) is refused, as is one the browser marks
  `Sec-Fetch-Site: cross-site` or `same-site`. Another port on `localhost` is
  another origin;
- one with neither header, as `curl` and scripts send, passes.

Without a token the server also answers only to a loopback host name
(`localhost`, `127.x.x.x`, `[::1]`). That stops DNS rebinding, where a name an
attacker controls is pointed at `127.0.0.1` to make their page same-origin
with the dashboard.

A refused request gets `403` with the problem code `forbidden`. Behind a
reverse proxy that rewrites `Host`, list the public origin with
`--allowed-origin https://ops.example`.

The server speaks plain HTTP. To expose it beyond a trusted network, put it behind something that terminates TLS.

MAVLink and CoT are not covered by the token. Bind the gateway to loopback unless the network is trusted.

## Limits

### Intents per actor

Every intent, from REST, the WebSocket, the dashboard or MAVLink, passes a per-actor token bucket before it takes a place in the driver's queue. The sustained rate is the run config's `run.intent_rate_per_actor` (200 a second by default), with bursts of twice that. The actor is:

| Source | Actor |
|---|---|
| `POST /api/v1/intents` and the WebSocket | The intent's own `actor` field, whatever the client claims. |
| The dashboard, and the geofence, station and region routes of the API | `operator` |
| MAVLink | `mavlink:<ip>:<port>` of the ground station's socket |

An intent over the limit is refused: `429` with code `rate_limited` over REST, a `rejected` message with reason `rate_limited` over the WebSocket, a flash message on the dashboard. Since the actor id is whatever a client says it is, this limits accidents and runaway clients, not someone who holds the token.

### Queue and body sizes

| Limit | Value | What happens |
|---|---|---|
| `run.max_pending_intents` | 10 000 | A full queue refuses new intents at once: `429` with code `backpressured`. Nothing waits. |
| `run.max_applied_per_tick` | 1 000 | Intents beyond this in one tick wait for the next. |
| `run.intent_horizon` | 8 ticks | How far ahead a client may schedule an intent. |
| Intent body | 16 KiB | Larger intents are refused. |
| Any request body | 64 MiB | The cap on scenario and checkpoint uploads. |
| Uploaded pack | 256 MiB unpacked, 4 096 files | Guards against zip bombs. |

The `run.*` values come from a pack's `config.toml` and can be changed on a running world with `PUT /api/v1/config` (the dashboard's I/O page only downloads the config); see [Scenario packs](scenarios.md).

## The clock

The driver is always in one of these modes:

| Mode | Meaning |
|---|---|
| `paused` | Not ticking. The default after start-up without `--run`, and after a load or reset. |
| `running` | Ticking at `run.target_tps`, or as fast as the model goes when it is unset. |
| `stepping` | Working through a fixed number of steps, then back to `paused`. |
| `loading` | Swapping in a scenario or checkpoint. |
| `faulted` | A tick panicked. The world is frozen at the last good tick, the thread is still alive, and `GET /api/v1/health` answers `503`. A reset recovers it; with `run.auto_recover = true` the driver reloads the scenario by itself. |

`--run` starts the clock at maximum speed whatever the pack's `target_tps` says. `tick_seconds` must be between 0.001 and 10; a pack or config outside that is refused. On the dashboard, Speed offers Max, Real time (one tick per `tick_seconds`, for packs that set it) and a Custom rate in ticks per second. The drones packs ship with no `target_tps`, so drones fly faster than real time unless you choose otherwise; the MAVLink gateway and CoT emitter keep their own real-time cadence regardless.

## Profiles: gold and fast

Every run has a determinism profile, set by the pack's manifest (`profile = "gold"` or `"fast"`) and shown as `run.profile` in the config. Changing it needs a reset.

| | `gold` | `fast` (the default) |
|---|---|---|
| Purpose | Science and replay | Games, visualisation, load |
| State hash | Each published view and checkpoint carries a stable hash of the world, unless the view was truncated at `run.max_published_cells` | None |
| Intent log | Every applied intent is logged with its tick; `GET /api/v1/intents/log` downloads it as JSON Lines | Not kept; that route returns `400` |
| Replay | `simctl replay` runs the pack twice and checks the hashes match | Refused |
| Rewind | The recent past can be shown again, tick by tick (drones packs); see [Rewind](#rewind) | None |

The drones model always steps agents in id order and draws every random number from a (seed, entity, tick) stream, so its runs are repeatable in either profile; gold adds the hash and the log that prove it. The walker models only fix their iteration order in gold.

The shipped packs: `london-patrol`, `kaunas-deliveries` and `iron-wolf` are gold, `swarm-survey` is fast. The benchmark packs in `bench/scenarios`: `london-walkers` is gold, `global-drift` is fast.

The run config also has `run.parallel` and `run.threads`. They are shown on the dashboard and in `GET /api/v1/run`, but nothing acts on them yet: whether krABMaga's parallel code is compiled in is the `parallel` Cargo feature, and even then the drones schedule runs on one thread.

### Rewind

In `gold`, a drones world keeps its recent past. Every `run.history_every_ticks` ticks (100 by default) the driver hands a keyframe to a `sim-history` thread, which keeps the last `run.history_keyframes` (60 by default; 0 turns rewind off) and every intent applied since the oldest. With the defaults and 0.1 s ticks that is the last ten minutes. To show a past tick it restores the keyframe before it and replays the intents and steps, so the view is the world exactly as it was. The live run never waits for it.

The dashboard's timeline steps back through it (see [The dashboard](dashboard.md#rewind)), and `GET /api/v1/history` and `?at=<tick>` on the state endpoints read it (see [API guide](api-guide.md#rewinding)). A load, reset or restored checkpoint starts it afresh. In `fast`, or for the walker models, `GET /api/v1/history` answers `available: false` and says why.

## Cargo features

| Feature | Effect |
|---|---|
| *(none)* | krABMaga with `gis` (and so Bevy): the drones model and `walkers-krabmaga`. |
| `parallel` | Also krABMaga's `parallel` feature: its thread-safe grid and schedule. The drones schedule still runs on one thread, so gold hashes match the default build. `krab-drones --tui` is not available in this build. |

Enable it per binary:

```sh
cargo run --release -p sim-axum --features parallel --bin krabshell -- ...
cargo run --release -p sim-engine --features parallel --bin simctl -- ...
cargo run --release -p sim-drones --features parallel --bin krab-drones -- ...
```

The health endpoint and start-up log say which build is running: `krABMaga (gis)` or `krABMaga (gis, parallel)`.

## Shutdown

Ctrl-C (SIGINT) or SIGTERM starts a clean shutdown, in this order:

1. The HTTP server stops accepting connections and finishes the requests in flight. The MAVLink gateway, the CoT emitter, the weather refresher, the conflict service and the run logger stop with it; the run logger writes its files one last time.
2. The driver is paused, so a checkpoint describes one tick rather than a world still moving.
3. With `--checkpoint-on-exit PATH`, a checkpoint is written to `PATH`. A failure here is logged, not fatal.
4. The driver thread is joined and the process exits.

A checkpoint is pretty-printed JSON. There is no flag to start from one: start the server, pause it, and restore through the dashboard's I/O page or `POST /api/v1/state/upload`. Restoring needs the driver paused. In gold, a restored checkpoint has the same tick and the same hash.

```sh
target/release/krabshell --scenario kaunas-deliveries --run --checkpoint-on-exit var/last.ckpt.json
```

`var/` and `*.ckpt.json` are in `.gitignore`.

## Run logs

A drones pack that sets `heatmap_resolution` or a positive `order_rate_per_address_day` in its `params.toml` gets a run log. While such a pack is loaded, the run logger samples the published view about every 50 ms (it never touches the simulation) and writes its files to:

```text
<log-dir>/<scenario>-<UTC start>/      e.g. var/logs/kaunas-deliveries-20261003T142233Z/
```

The files are rewritten every `--log-every` seconds when something has changed, and once more on shutdown. Each file is written to a temporary name and renamed, so a reader never sees half a file. A reset, a restored checkpoint (any jump back in ticks) or a different scenario finishes the current folder and starts a new one, so earlier runs are never overwritten. A file appears only once it has something in it.

| File | Written once there is | What |
|---|---|---|
| `heatmap.geojson` | Any flight (with `heatmap_resolution` set) | Drone-seconds of flight per A5 cell at `heatmap_resolution`, as a styled choropleth. Time parked or disarmed is not flight. |
| `deliveries.csv` | A delivery | Every delivered order: depot, drone, address, ticks, and seconds waiting, flying and in total. |
| `delivery-times.csv`, `delivery-times.svg` | A delivery | Histogram of time from order to delivery in whole-minute bins, with mean, median, 90th percentile and slowest. |
| `rejected-orders.csv` | A rejected order | Every order turned away for a geofence over its address, with the stage and reason. |
| `landings.csv` | A landing or crash | The end of every flight: drone, landed or crashed, battery left (-1 for a crash), and what replaced a crashed drone. |
| `landing-battery.csv`, `landing-battery.svg` | A landing or crash | Histogram of battery at touchdown in 5 % bins, crashes at -1. |
| `geofences.geojson` | A geofence | Every geofence that opened, with when it opened and closed and how often it was breached, and every breach as a point. |
| `breaches.csv`, `reroutes.csv` | A geofence | Every breach, and every set of MAVLink messages the operator sent to take a drone round a fence. |
| `summary.json` | Any of the above | Totals, and how many reports the logger missed. |

If the logger falls so far behind that the driver has already dropped reports it had not read, it logs a warning and counts them in `summary.json`; deliveries may then be undercounted.

## Headless: simctl

`simctl` runs the same driver with no HTTP, no gateways and no subscribers. It is how the speed baseline is measured, and the quickest way to fly a pack for hours. Source: [`simctl.rs`](../crates/sim-engine/src/bin/simctl.rs).

```sh
cargo run --release -p sim-engine --bin simctl -- [--scenarios DIR] [--model KIND] <command>
```

`--scenarios` (default `scenarios`) and `--model` go before or after the command.

| Command | What it does | Options |
|---|---|---|
| `list` | Lists the packs: id, model, resolution and name. | |
| `run ID` | Runs the pack for a number of ticks and prints the tick, agents, occupied cells, rate, last step time and, in gold, the hash. | `--ticks` (10 000), `--tps` (cap the rate; maximum speed if left out), `--checkpoint PATH` (write one at the end), `--profile gold\|fast` (override the pack) |
| `bench ID` | Measures ticks per second after a warm-up. | `--warmup` (1 000), `--ticks` (20 000), or `--seconds S` to run flat out for a fixed wall-clock time instead |
| `replay ID` | Runs a gold pack twice from scratch and checks the hashes agree. Refuses a fast pack. | `--ticks` (500) |
| `fly ID` (alias `heatmap`) | Flies a drones pack for some simulated hours as fast as it goes and writes its run log, then prints deliveries, flights, crashes and geofence activity. Refuses a pack that asks for no run log. | `--hours` (8), `--sample-s` (5; simulated seconds between samples), `--out DIR` (default `var/logs/<scenario>-<hours>h`) |
| `cell` | Prints the A5 cell at a coordinate, its centroid, parents, neighbour count and edge length. Use it to fill in a manifest's `root`. | `--lon`, `--lat`, `--res` (9) |

```sh
cargo run --release -p sim-engine --bin simctl -- list
cargo run --release -p sim-engine --bin simctl -- run london-patrol --ticks 6000 --checkpoint var/patrol.ckpt.json
cargo run --release -p sim-engine --bin simctl -- --scenarios bench/scenarios bench london-walkers --warmup 200 --ticks 1500
cargo run --release -p sim-engine --bin simctl -- replay london-patrol --ticks 600
cargo run --release -p sim-engine --bin simctl -- fly kaunas-deliveries --hours 8
cargo run --release -p sim-engine --bin simctl -- cell --lon -0.1276 --lat 51.5072 --res 15
```

To run the drones model with no krabshell at all, on krABMaga alone, see [Development](development.md#running-the-drones-model-without-krabshell).
