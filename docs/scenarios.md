# Scenario packs

A scenario pack is a directory of plain files that says what to simulate: which model, where on the A5 grid, how the run loop behaves, the model's parameters and any agents or data the world starts with. The server lists the packs in its scenarios directory, loads one at a time, and accepts new ones as zip uploads. Anything that runs a model reads the same files: the `krabshell` server, `simctl`, and the standalone `krab-drones` runner. This page covers the pack format, the run configuration, the three models, every parameter the drones model reads, and the packs that ship with the repository.

## Pack layout

```
<id>/
  manifest.toml         identity, model and address space (required)
  config.toml           [run], [grid] and [model] defaults (optional)
  params.toml           model parameters (optional)
  world/agents.json     seed agents (optional)
  world/fields.json     non-grid model data (optional)
  world/<name>.json     more model data, joined to the fields as <name> (optional)
```

Only `manifest.toml` is required. A missing `config.toml` means the defaults below; a missing `params.toml` means no parameters, so the model's defaults. The reader is [`../crates/sim-core/src/pack.rs`](../crates/sim-core/src/pack.rs).

Every `world/*.json` file other than `agents.json` and `fields.json` is read and added to the fields under its file name without the extension. A drones pack uses this to keep hand-written data apart from a large generated `fields.json`: `world/geofences.json` becomes the `geofences` field. If `fields.json` already has a key of the same name, the pack is refused rather than one silently replacing the other.

### What the drones model reads from `world/`

| File | Field | What it holds | See |
|---|---|---|---|
| `agents.json` | — | Drones, intruders, people, vehicles and points of interest. | [Seed agents](#seed-agents) |
| `fields.json` | `terrain` | Ground elevation grid. Without it the ground is flat at 0 m. | [Terrain](#terrain) |
| `fields.json` | `addresses` | The address book: depots and the addresses that order deliveries. | [The address book](#the-address-book) |
| `geofences.json` | `geofences` | Fences open from the first tick. | [Geofences](geofences.md) |
| `stations.json` | `stations` | Launch and recovery stations. | [Stations](stations.md) |
| `regions.json` | `regions` | Regions of interest to survey. | [Stations](stations.md) |
| `behaviour.json` | `behaviour` | The behaviour tree every drone uses, in place of the default. | [Drone behaviours](drone-behaviours.md) |
| `weather.json` | `weather` | A wind field shipped with the pack, so no fetch is needed. | [Weather](weather.md) |

## The manifest

`manifest.toml` is the pack's identity. Unknown keys are refused.

| Key | Required | Default | Meaning |
|---|---|---|---|
| `id` | yes | — | Load key. 1 to 64 characters of `A-Z`, `a-z`, `0-9`, `-` and `_`. Keep it the same as the directory name: the store loads a pack by its directory name. |
| `name` | yes | — | Display name. |
| `description` | no | empty | One or two sentences for the Scenarios page. |
| `model` | yes | — | The model to build: `drones`, `walkers` or `walkers-krabmaga`. |
| `resolution` | yes | — | Occupancy resolution, `R_work`. 15 is about 180 m per cell. |
| `root` | no | none | Island root cell, as a hex cell id string. Anything outside it is refused at the edge. Absent means a global grid. Must not be finer than `resolution`. |
| `profile` | no | `fast` | Determinism profile, `gold` or `fast`. See [Profiles](#profiles). |
| `engine_rev` | no | none | The krABMaga revision the pack was written against. Recorded only; nothing reads it yet. |

The manifest owns the address space. When the pack loads, its `resolution`, `root` and `profile` replace whatever `config.toml` says for `grid.resolution`, `grid.root` and `run.profile`. If `config.toml` asks for a `view_resolution` finer than the manifest's resolution, it is pulled back to the resolution less 4 (and no lower than 0).

To find a `root` cell id, use `simctl cell`:

```sh
cargo run --release -p sim-engine --bin simctl -- cell --lon -0.1276 --lat 51.5072 --res 6
```

## Run configuration

The run configuration is everything about how the loop runs, as opposed to what it simulates. It has three tables: `[run]`, `[grid]` and `[model]`. A pack's `config.toml` sets its defaults; unknown keys are refused, so a misspelt key is an error rather than silently ignored. The type is `Config` in [`../crates/sim-core/src/config.rs`](../crates/sim-core/src/config.rs).

### `[run]`

| Key | Default | Meaning |
|---|---|---|
| `profile` | `fast` | `gold` or `fast`. The manifest's `profile` overrides it on load. |
| `parallel` | `false` | Ask for krABMaga's parallel schedule. At present it is shown on the Run page and in `GET /run`, and no model reads it: the drones model always steps on one thread. |
| `threads` | the machine's CPU count | Threads for the parallel schedule. Shown, not used, as for `parallel`. |
| `target_tps` | absent | Ticks per second to aim for. Absent means as fast as the model goes. The dashboard's Speed control and `POST /run/rate` change it. |
| `publish_hz` | 20 | Most views published per second, whatever the tick rate. More than 0, at most 1000. |
| `intent_horizon` | 8 | How many ticks ahead a client may schedule an intent. |
| `max_pending_intents` | 10000 | Intents queued for the driver before new ones are refused as backpressured. At least 1. |
| `max_applied_per_tick` | 1000 | Intents applied in one tick. At least 1. |
| `max_published_cells` | 50000 | Most cells in one published view. At least 1. |
| `intent_rate_per_actor` | 200 | Intents per second one actor may submit, sustained; bursts of twice this are allowed. Positive. |
| `auto_recover` | `false` | Restart the world automatically after a tick panics. |
| `history_every_ticks` | 100 | Ticks between rewind keyframes, in the `gold` profile. Stepping back replays at most this many ticks. |
| `history_keyframes` | 60 | Rewind keyframes kept; 0 turns rewind off. 60 × 100 ticks is ten minutes at 0.1 s a tick. |

### `[grid]`

| Key | Default | Meaning |
|---|---|---|
| `resolution` | 15 | Occupancy resolution. The manifest overrides it on load. |
| `root` | absent | Island root cell. The manifest overrides it on load. |
| `view_resolution` | 11 | The coarser resolution the dashboard's heat map aggregates to, about 2.8 km cells at 11. Must not be finer than `resolution`. |

### `[model]` and `params.toml`

Model parameters are free-form: a name and a boolean, integer, float, string or list. `sim-core` never reads them; the model does, on reset. They can live in `config.toml` under `[model]`, in `params.toml`, or both. `params.toml` is merged over `[model]`, key by key, so `params.toml` wins where both set a key.

Unlike the manifest and the `[run]` and `[grid]` tables, parameters are not checked against a list. A parameter the model does not know is ignored, and so is one of the wrong type (a string where a number is expected, say), which leaves the default in force. Integers are accepted wherever a float is expected.

### Downloading, editing and applying a config

The **Effective configuration** link on the dashboard's I/O page (`GET /api/v1/config/download`) downloads the configuration the driver is using now, as `<scenario>-config.toml`. It is the effective config: the pack's `config.toml` with the manifest folded in, `params.toml` merged into `[model]`, and any change made since. It is lossless, so it can be edited and used again in two ways:

- **Applied to the running server** with `PUT /api/v1/config`, as JSON of the same shape (`{"run": {...}, "grid": {...}, "model": {...}}`). `GET /api/v1/config` returns the current one as JSON to start from. The dashboard has no form for this; use the API. The config is checked before it is accepted.
- **Saved into a pack** as its `config.toml`. The manifest still overrides `grid.resolution`, `grid.root` and `run.profile` when that pack loads.

Some changes cannot be made under a running world: `run.profile`, `run.parallel`, `run.threads`, `grid.resolution`, `grid.root` and anything in `model`. A config that changes any of them rebuilds the world from the loaded pack: back to tick 0, paused. The `PUT` response lists them in `reset_fields`. Every other change (the tick rate, `publish_hz`, the intent limits) takes effect at once and the run carries on.

A rebuild started this way does not fetch weather again. Changing the `weather` parameters with `PUT /api/v1/config` leaves the wind the world was loaded with; load the pack again to fetch for new parameters, which also discards the config change.

### Profiles

| Profile | What it means |
|---|---|
| `gold` | For science and replay. Models step in a fixed order with seeded randomness, the driver hashes every published view and keeps the log of applied intents, and two runs of the same pack and intents agree tick for tick. `simctl replay` and `GET /api/v1/intents/log` need it. |
| `fast` | For games and visualisation. Iteration order may be undefined and no hash or intent log is kept. |

The drones model behaves the same in both: its agents always step in id order and every random choice comes from a seeded stream. What `gold` adds for a drones run is the hash and the intent log, which is what makes a run with ground stations and dashboard actions in it replayable. See [Drones: determinism](drones.md#determinism).

### The seed

Each model reads an integer `seed` parameter. The drones model defaults to `0x0d0e5eed00000001` and the walker models to `0x5eed123456789abc`. Change the seed and a `gold` run is a different, equally repeatable, run.

## Discovering, uploading and loading packs

The server reads packs from one directory, `--scenarios` (default `scenarios`, environment `KRABSHELL_SCENARIOS`). Every subdirectory that reads as a pack is listed on the **Scenarios** page and by `GET /api/v1/scenarios`, sorted by id. A subdirectory that fails to read is skipped with a warning in the server log, not shown. `simctl` and `krab-drones` read the same directories.

**Uploading.** The upload form on the Scenarios page, or `POST /api/v1/scenarios` as multipart form data, takes a zip in a part named `pack` (or `file`) and an `id` part. Without an `id`, the zip's file name without `.zip` is used. The rules:

- The files go at the root of the zip: `manifest.toml`, not `my-pack/manifest.toml`.
- The manifest's `id` must match the upload id.
- A path that would land outside the pack directory (absolute, or with `..`) is refused.
- At most 4096 entries, and at most 256 MiB unpacked.
- The pack must read as a whole: manifest, config, params and every `world/*.json`.

An upload is unpacked to a staging directory and checked before it is moved into place, so a bad upload leaves nothing behind. A pack with the same id is replaced. Uploading does not load it.

**Loading.** The Load button on the Scenarios page, or `POST /api/v1/scenarios/{id}/load`, reads the pack from disk, fetches its weather if it asks for any, and hands the decoded pack to the driver. The old world is dropped and the new one starts at tick 0, paused. Reports, metrics and the intent log start afresh; report sequence numbers carry on rising. If the pack does not build (a geofence over a drone's pad, a seed agent outside the root, a bad behaviour tree), the load is refused and the error says why. `krabshell --scenario <id>` loads one at start-up, and `--run` starts it running.

`--model` (on `krabshell` and on `simctl`) runs a pack on a different model than its manifest names, for example `walkers-krabmaga` against a `walkers` pack.

## Models

The `model` key picks one of three models registered in [`../crates/sim-engine/src/model.rs`](../crates/sim-engine/src/model.rs).

| Model | What it is |
|---|---|
| `drones` | The krABMaga drones model in `sim-drones`: drones flying MAVLink missions and deliveries over terrain, with people, vehicles, intruders and points of interest to find. Described in [Drones](drones.md). |
| `walkers` | The reference model: agents that random-walk from cell to neighbouring cell on the built-in sparse grid. Simple enough that a wrong answer is obvious; used for benchmarks and to exercise the server. |
| `walkers-krabmaga` | The same walkers with krABMaga's `SparseA5Grid` as occupancy of record, to compare the two grids on one pack. |

### Walker parameters

Both walker models read the same parameters.

| Parameter | Default | Meaning |
|---|---|---|
| `walkers` | 0 | Walkers to create at reset, besides the seed agents. |
| `spread_k` | 3 | They start spread round the origin over a disk this many cells across in radius. Clamped to 0–24. |
| `stay_probability` | 0.0 | Chance a walker holds its cell on a tick. Clamped to 0–1. |
| `seed` | `0x5eed123456789abc` | Seed for every random choice. |
| `origin_lon`, `origin_lat` | the root's centre, or 0, 0 | Where the crowd starts. Both must be given. |

A walker pack's seed agents can be of any `kind`; `"state": {"anchored": true}` keeps one from moving. Walkers accept `Spawn` (up to 10000 at once, with `kind` and `anchored`), `Despawn` and `Move` intents.

## Seed agents

`world/agents.json` is a list of agents the world starts with:

```json
[
  { "id": 1, "kind": "drone", "lon": 23.89266, "lat": 54.9401304,
    "state": { "sysid": 1, "callsign": "MEGA-1", "elevation_m": 81.2,
               "depot": "MEGA", "cruise_agl_m": 60.0 } }
]
```

| Key | Meaning |
|---|---|
| `id` | Optional. Agent ids must be unique and from 1 to 2147483647. Without one, the agent takes the next free id. |
| `kind` | What it is. The drones model knows `drone`, `intruder`, `person`, `vehicle` and `poi`. |
| `lon`, `lat` | Where it starts, in degrees. |
| `cell` | Or a cell id instead of `lon` and `lat`; the agent starts at its centre. |
| `state` | Kind-specific settings. |

The `state` each drones kind takes is listed in [Drones: agent types](drones.md#agent-types). An agent outside the root, or a `state` that does not make sense (a `depot` not in the address book, a `sysid` used twice), stops the pack loading.

## Terrain

A drones pack can carry ground elevation in `world/fields.json` under `terrain`: a regular latitude and longitude grid, in decimetres above mean sea level.

```json
{ "terrain": {
    "lat0_e7": 548330556, "lon0_e7": 237908333,
    "dlat_e7": 2778, "dlon_e7": 4167,
    "rows": 596, "cols": 554,
    "elev_dm": [ ... ],
    "source": "attribution text"
} }
```

Sample `(r, c)` is at `lat0 + r·dlat`, `lon0 + c·dlon` (both × 10⁷), row 0 southmost, each row west to east. Between samples the ground is bilinear; beyond the edge it is the nearest edge sample. All altitudes in the drones model are above mean sea level, and without terrain the ground is flat at 0 m. The format is documented in [`../crates/sim-drones/src/terrain.rs`](../crates/sim-drones/src/terrain.rs).

## The address book

Deliveries need an address book in `world/fields.json` under `addresses`: the depots, and every address that can order.

```json
{ "addresses": {
    "source": "OpenStreetMap contributors, ODbL 1.0",
    "depots": ["MEGA", "AKRO"],
    "items": [
      [548660916, 238743222, 1, "Kalvarijos g. 35"],
      [548655305, 238995177, 1, "Bitininkų g. 25"]
    ]
} }
```

| Key | Meaning |
|---|---|
| `depots` | Depot names. A courier drone names one as its `depot`, and a spare as its `spare`. |
| `items` | One per address: `[lat_e7, lon_e7, depot_index, "label"]`. Latitude comes first. `depot_index` is the position in `depots` of the depot that serves the address, counting from 0. |
| `source` | Free text, for attribution. Not read. |

An item naming a depot that is not in the list stops the pack loading. With an address book and `order_rate_per_address_day` above 0, every address orders at random; how orders become flights is in [Drones: deliveries and dispatch](drones.md#deliveries-and-dispatch). The weather fetch also uses the addresses to decide what area to cover when the pack has no terrain or `weather_bbox`.

## Drones model parameters

Every parameter the drones model reads, from [`../crates/sim-drones/src/settings.rs`](../crates/sim-drones/src/settings.rs) and [`../crates/sim-drones/src/world.rs`](../crates/sim-drones/src/world.rs). Clamped values are pulled into range silently. Durations given in seconds are rounded to whole ticks where noted.

### Time and randomness

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `tick_seconds` | 0.1 | s | Simulated time per tick, 0.001–10; a config outside that range is refused. The dashboard's clock, the heatmap and the weather clock read the same value. Without it the dashboard offers no real-time speed. |
| `seed` | `0x0d0e5eed00000001` | — | Seed for every random choice: where people and vehicles start and wander, vehicle types, orders, and what the cameras see. |

### Flight

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `cruise_ms` | 10.0 | m/s | Cruise airspeed of every drone that does not set its own `cruise_ms`. Also the airspeed at which `battery_drain_pct_s` applies. |
| `climb_ms` | 3.0 | m/s | Climb and descent rate. At least 0.1. |
| `takeoff_alt_m` | 20.0 | m | Height above the pad or ground for a `NAV_TAKEOFF` command that gives none. The launch wind is checked at this height. |
| `rtl_alt_m` | 30.0 | m | Return-to-launch height above home. |
| `max_mission_items` | 128 | items | Longest mission an upload may carry. Clamped to 1–4096. |
| `max_wind_ms` | 12.0 | m/s | Mean wind at takeoff height above which drones stay on the ground. 0 is no limit. |
| `max_gust_ms` | 15.0 | m/s | Gust at takeoff height above which drones stay on the ground. 0 is no limit. |

### Battery

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `battery_drain_pct_s` | 0.0667 (100/1500) | %/s | Drain in the air at `cruise_ms` airspeed: a 25-minute endurance. It scales with the square of airspeed. 0 flies for ever. |
| `hover_drain_pct` | 0 | % | Share of that drain drawn at zero airspeed. 0 makes the drain purely proportional to airspeed squared. Clamped to 0–100. |
| `battery_reserve_pct` | 15 | % | At or below this in the air, the low-battery failsafe turns the drone for home. A drone at or below it cannot arm. Clamped to 0–100. |
| `charge_pct_s` | 0 | %/s | Charge gained on the ground on the home pad or at a station. 0 is no charger. |
| `launch_battery_pct` | 0 | % | A takeoff from the ground waits (in a mission) or is refused (as a command) below this. Couriers take no order below it. Clamped to 0–100. |

### Winch

A `DO_WINCH` item holds the drone in a hover for as long as the real sequence would take. There is no line or parcel in the world.

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `winch_drop_ms` | 2.0 | m/s | Lowering rate until `winch_slow_m` above the ground. At least 0.01. |
| `winch_slow_ms` | 0.5 | m/s | Lowering rate over the last `winch_slow_m`. At least 0.01. |
| `winch_slow_m` | 3.0 | m | Length of the slow final stretch. |
| `winch_release_s` | 5.0 | s | Time on the ground for the parcel to release. |
| `winch_reel_ms` | 2.5 | m/s | Reeling-in rate. At least 0.01. |
| `winch_lock_s` | 3.0 | s | Time to lock the reel once it is in. |

### Camera

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `hfov_deg` | 70 | degrees | Horizontal field of view. The footprint radius is height above the ground × tan(`hfov_deg` / 2). Clamped to 1–170. |
| `max_range_m` | 250 | m | Cap on the footprint radius, and the slant range at which intruders can be seen. At least 1. |
| `p_detect` | 0.85 | — | Chance of a detection at the centre of the footprint, per scan. Clamped to 0–1. |
| `edge_falloff` | 0.6 | — | How much of that chance is lost at the edge: there it is `p_detect` × (1 − `edge_falloff`). Clamped to 0–1. |
| `sensor_period_s` | 1.0 | s | Time between scans. Rounded to whole ticks, at least one. |
| `report_interval_s` | 10.0 | s | The same target is reported to the same drone at most this often. Rounded to whole ticks, at least one. |

### People, vehicles and intruders

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `persons` | 0 | — | People scattered at reset, besides any seed agents. |
| `vehicles` | 0 | — | Vehicles scattered at reset. |
| `origin_lon`, `origin_lat` | the root's centre, or 0, 0 | degrees | Centre of the scatter. Both must be given. |
| `wander_radius_m` | 600 | m | Radius of the scatter, and how far each person or vehicle wanders from where it started. At least 1. |
| `person_speed_ms` | 1.4 | m/s | Walking speed. |
| `vehicle_speed_ms` | 8.0 | m/s | Driving speed. |
| `intruder_speed_ms` | 12.0 | m/s | Speed of intruders that do not set their own `speed_ms`. |

### Deliveries

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `order_rate_per_address_day` | 0 | orders per address per day | How often each address in the address book orders. 0 places no orders. Above 0 it also starts a run log. |
| `dispatch_takeoff_m` | 30.0 | m | Takeoff height above the pad on a delivery or survey trip. |
| `dispatch_approach_m` | 15.0 | m | Height above the pad a returning courier descends to before it lands. |
| `dispatch_load_s` | 120.0 | s | Loading time after a courier lands before it takes the next order. Rounded to whole ticks. |
| `reject_orders_in_geofences` | `false` | boolean | Turn away an order whose address is under a geofence rather than hold it in the queue. Must be the boolean `true`. |

### Geofences, stations and regions

These are described on their own pages.

| Parameter | Default | Unit | Meaning | See |
|---|---|---|---|---|
| `geofence_margin_m` | 30 | m | Distance the drones the simulation flies keep from every fence. Clamped to 2–1000. | [Geofences](geofences.md) |
| `geofence_vertical_margin_m` | 10 | m | Distance kept above or below a fence's altitude band. Clamped to 0–500. | [Geofences](geofences.md) |
| `beneath_min_agl_m` | 20 | m | Lowest height above the ground drones fly to pass beneath a fence. Clamped to 1–1000. | [Geofences](geofences.md) |
| `roi_revisit_min` | 30 | min | How long after a survey a region of interest is due another. | [Stations](stations.md) |
| `survey_agl_m` | 60 | m | Height above the ground a survey is flown at. Clamped to 5–500. | [Stations](stations.md) |
| `crash_roi_radius_m` | 150 | m | Radius of the region of interest left where a drone crashes. 0 leaves none. Clamped to 0–5000. | [Stations](stations.md) |

### Read by the server around the model

| Parameter | Default | Meaning | See |
|---|---|---|---|
| `heatmap_resolution` | absent | A5 resolution (0–30) of the flight heatmap. Setting it starts a run log. | [Operating](operating.md) |
| `conflict_resolution` | absent | Default A5 resolution (8–18) for conflict prediction. The conflicts service also reads `cruise_ms`, `climb_ms` and `intruder_speed_ms`. | [Conflicts](conflicts.md) |
| `atc` | true | The drones model assesses conflicts itself and its drones and manned aircraft fly their advisories. `false` turns it off. | [Conflicts](conflicts.md#advisories-are-flown) |
| `atc_every_s` | 5 | Seconds between those assessments. | [Conflicts](conflicts.md#advisories-are-flown) |
| `advisory_hold_s` | 10 | Seconds an advisory is flown for, renewed while what it avoids is still coming. | [Conflicts](conflicts.md#advisories-are-flown) |
| `manned_lateral_m` | 300 | Lateral separation drones and intruders must keep from a manned aircraft, m. | [Conflicts](conflicts.md#manned-aircraft-protected-volume-and-giving-way) |
| `manned_vertical_m` | 150 | Vertical separation, above and below, m. | [Conflicts](conflicts.md#manned-aircraft-protected-volume-and-giving-way) |
| `weather` | `"off"` | `"off"`, `"recorded"` or `"live"`. | [Weather](weather.md) |
| `weather_start_utc` | none | UTC time of tick 0 for recorded weather, e.g. `"2026-03-14T06:00:00Z"`. Required when `weather = "recorded"`. | [Weather](weather.md) |
| `weather_hours` | 24 | Hours of weather to fetch, 1–384. | [Weather](weather.md) |
| `weather_resolution` | 12, or the world's resolution if coarser | A5 resolution of the weather cells. No finer than the world's. | [Weather](weather.md) |
| `weather_model` | `"best_match"` | Open-Meteo model token. | [Weather](weather.md) |
| `weather_refresh_min` | 60 | Minutes between live fetches. Clamped to 5–1440. | [Weather](weather.md) |
| `weather_bbox` | none | `[min_lon, min_lat, max_lon, max_lat]` to fetch over. | [Weather](weather.md) |
| `weather_margin_km` | 3 | Margin round the agents, sites and addresses when there is no terrain or box. Clamped to 0–200. | [Weather](weather.md) |

Weather is fetched by the server when a pack loads. `simctl` and `krab-drones` do not fetch it, so under them a pack that asks for `recorded` or `live` weather flies in still air unless it ships `world/weather.json`.

## Shipped packs

### `scenarios/`

| Pack | Model | Profile | What it demonstrates |
|---|---|---|---|
| `london-patrol` | `drones` | `gold` | The smallest complete drones world, and the commented example to copy. |
| `kaunas-deliveries` | `drones` | `gold` | Deliveries over real terrain and real addresses, with geofences, crashes, spares, recorded weather and a run log. |
| `swarm-survey` | `drones` | `fast` | Load on the MAVLink and CoT gateways. |

**`london-patrol`.** Four drones over central London, resolution 15 inside root `6361800000000000`. HAWK-1, HAWK-2 and HAWK-3 start on looping patrols (`autostart`, with a `DO_JUMP` back to the first waypoint) over Westminster, the City and the South Bank. UAV-4 waits on the ground for a ground station to take it. There are 1200 people and 150 vehicles within 1800 m of the origin, ten named landmarks as points of interest, and one intruder, UNKNOWN-1, flying a four-point circuit at 14 m/s. `battery_drain_pct_s = 0`, so the patrols fly for ever; set a drain to see low-battery returns. Because the patrols are flown by the simulation, they are kept clear of any geofence opened from the dashboard.

**`kaunas-deliveries`.** Parcel delivery from the roofs of two shopping centres in Kaunas, Mega (81.2 m above sea level) and Akropolis (45.9 m). Each hub has twenty couriers at cruise heights of 60, 70 or 80 m above the terrain and one spare (MEGA-SPARE, AKRO-SPARE) waiting on its pad. The address book holds 34502 OpenStreetMap addresses within 5 km, each ordering 0.06 times a day, about 86 orders an hour. The terrain is a Copernicus GLO-30 grid. `world/geofences.json` keeps the drones off the Old Town, the hospital, Žalgiris Arena and Ąžuolynas park, and the UAS geographical zones Oro navigacija publishes over the city that bind at cruise height. It shows:

- dispatch: orders queued per hub, couriers flying terrain-following missions, `DO_WINCH` deliveries, loading (120 s) and charging (0.0333 %/s, back to 80 % before the next order);
- the battery: 30 minutes at 15 m/s with a 20 % reserve, so crashes only happen if you lower `battery_reserve_pct` or `launch_battery_pct`, after which the spare and then new drones take over;
- `reject_orders_in_geofences = true`, and `order_rejected` reports;
- recorded weather for 2026-03-14 from 09:00 UTC, six hours;
- a run log, from `heatmap_resolution = 15`.

The recorded weather is fetched from Open-Meteo the first time the pack loads and cached in `var/weather-cache/`. With `--no-weather`, and nothing cached, the pack refuses to load. `world/agents.json` and `world/fields.json` are generated by `tools/kaunas_deliveries.py`; the zones in `world/geofences.json` by `tools/lt_geozones.py`.

**`swarm-survey`.** Thirty drones on the ground in central London, all left to ground stations, with ten points of interest, two intruders (one at 20 m/s), 4000 people and 1000 vehicles within 4 km. The cameras reach 300 m and scan every 2 s, reporting each target at most every 30 s. It does not set `battery_drain_pct_s` or `charge_pct_s`, so drones a ground station flies have the default 25-minute endurance and no charger. Fast profile: use it for gateway load, not replay.

### `bench/scenarios/`

The walker packs used for benchmarks and replay checks. Point the server or `simctl` at them with `--scenarios bench/scenarios`.

| Pack | Model | Profile | What it demonstrates |
|---|---|---|---|
| `london-walkers` | `walkers` | `gold` | A small, reproducible island run: 2000 walkers on 180 m cells round central London inside root `6361800000000000`, with two anchored landmarks. Two runs of equal length hash the same. |
| `global-drift` | `walkers` | `fast` | Throughput on an unrooted global grid: 20000 walkers at resolution 9 (about 11 km cells) round Zürich, never holding still. |

```sh
cargo run --release -p sim-engine --bin simctl -- --scenarios bench/scenarios replay london-walkers --ticks 500
cargo run --release -p sim-engine --bin simctl -- --scenarios bench/scenarios bench global-drift
```

`global-drift` sets `spread_k = 30`, which is clamped to 24.

## Writing a pack

1. Copy `scenarios/london-patrol` to a new directory and change `id` to the directory's name.
2. Set `resolution`, and a `root` from `simctl cell` if the world should be an island.
3. Put the agents in `world/agents.json` and the parameters in `params.toml`.
4. Check it reads: `cargo run --release -p sim-engine --bin simctl -- list` lists only packs that read, silently leaving out the rest, so if yours is missing, `simctl run <id> --ticks 1` prints why.
5. Run a drones pack without the server: `cargo run --release -p sim-drones --bin krab-drones -- scenarios/<id> --steps 10000`.
6. For a `gold` pack, check it replays: `cargo run --release -p sim-engine --bin simctl -- replay <id>`.
