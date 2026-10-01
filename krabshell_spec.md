# Project Plan and Specification

## krABMaga + Axum Simulation Server

**Version:** 0.1  
**Status:** Draft for implementation  
**Date:** 10 September 2026  
**Related stack:** krABMaga (`gis` / `SparseA5Grid`), Axum, Bevy client (out of band), web dashboard

---

## 1. Purpose

Build a process that runs a krABMaga discrete-event ABM at full engine speed (sequential first, parallel when the model allows it) while an Axum edge:

- accepts **intentions** (player or operator actions) into the sim,
- **serves state** as cell-addressed snapshots and deltas,
- exposes a **dashboard** to load scenarios, start/stop/step the clock, inspect live state, and download configuration and checkpoints.

The simulation remains the authority. HTTP/WebSocket never call `Schedule::step`. Bevy (or any other viewer) is a client of the same protocol the dashboard uses.

Spatial addressing uses the A5 sparse grid already in krABMaga: `GeoCell` is the unit of load, lookup, subscription, persistence, and (later) shard/island bounds.

---

## 2. Goals

1. **Max-speed tick loop.** Dedicated OS thread(s). No Tokio work inside `Agent::step` or `Field::lazy_update`. Target: tick budget dominated by krABMaga, not by serialization or accept loops.
2. **Intent ingress.** Authenticated clients submit time-stamped actions bound to a `GeoCell` (and optional agent id). Intents are queued and applied at tick boundaries.
3. **State egress.** REST for snapshots and files; WebSocket for live cell subscriptions and control events.
4. **Operator dashboard.** Load scenario packs, edit run config, start / pause / single-step / reset, watch counters and a coarse map, download state and config.
5. **A5 as the address space.** Every load, AOI, and download path is expressed as cells or parent cells, not as ad-hoc bounding boxes (boxes are allowed only as a query that compiles to cells).
6. **Feature path to `parallel`.** `gis` + `parallel` must compile, test, and run a reference model. Intra-step parallelism is phase 2; spatial parent-cell partitioning is phase 3.

---

## 3. Non-goals (v1)

- Putting the world inside WASM-4 or any fantasy-console runtime.
- Running `Schedule` on Tokio worker threads.
- Optimistic Time-Warp rollback of the whole planet.
- MPI / multi-process domain decomposition (design cells so this can be added later).
- Bit-identical parallel runs (document as non-deterministic unless a “gold sequential” flag is on).
- Embedding Bevy `DefaultPlugins` in the server process.
- A full GIS editor or authoring suite.

---

## 4. Architectural principles

### 4.1 Two runtimes, one process

| Runtime | Owns |
|---|---|
| OS tick thread(s) | `Schedule`, `State`, `SparseA5Grid`, intent drain, `lazy_update` |
| Tokio (Axum) | HTTP, WebSocket, dashboard static files, disk I/O for scenario packs, auth |

Communication is bounded channels, never shared `RefCell` state.

```
                    +------------------+
   Browser / Bevy   |   Axum (Tokio)   |
   REST + WS   <--> |  control + AOI   |
                    +--------+---------+
                             | mpsc / watch
                    +--------v---------+
                    |  SimDriver thread |
                    |  krABMaga step()  |
                    |  SparseA5Grid     |
                    +------------------+
```

### 4.2 Double-buffer contract

Agents read the **read** bags, write the **write** bags. Intents are applied to the write side after the previous `lazy_update` and before or at the start of the next `step`, never mid-iteration. Published snapshots are taken from the **committed** read side only.

### 4.3 Cell is the I/O quantum

- Subscribe: set of `GeoCell` (usually `grid_disk(camera, k)` or a parent at coarser resolution).
- Snapshot file: list of occupied cells + bags + header (tick, scenario id, resolution, root).
- Intent: `{ tick?, cell, agent?, verb, payload }`.
- Dashboard map: parent cells at a display resolution, not every occupancy cell.

### 4.4 Isolation of visualization

`bevy_a5` math may be used server-side for `lonlat_to_cell` and neighbor queries. No window, no `CellEntityIndex` as occupancy of record. Occupancy of record is `SparseA5Grid`.

---

## 5. Repository layout

Monorepo workspace:

```
krabmaga-server/
  Cargo.toml                 workspace
  crates/
    sim-core/                types only: GeoCell newtypes, Intent, Snapshot
                             header, ScenarioManifest, errors. No Bevy, no Axum.
    sim-engine/              SimDriver, State adapter, scenario loaders,
                             checkpoint encode/decode. Depends on krABMaga
                             with features gis (+ optional parallel).
    sim-axum/                HTTP/WS routes, auth, dashboard assets.
    sim-dashboard/           frontend (see §11). Built and embedded or served.
  scenarios/                 pack files for tests and demos
  docs/                      this spec
```

Optional later: `sim-client` (Bevy). v1 dashboard is enough to operate the sim.

**krABMaga dependency.** Pin a git rev. Upstream work that this project may need to carry as patches (or PRs):

- `gis` + `parallel` test matrix (`cargo test --features "gis parallel"`).
- `apply_to_all_values` GridOption parity on `SparseA5Grid`.
- Headless `gis` (A5 math without `DefaultPlugins`).
- `move_object` helper that updates `obj2loc` and `loc2objs` together.

Until those land upstream, vendor via `[patch]` or a fork branch named `server-gis-parallel`.

---

## 6. Simulation engine specification

### 6.1 SimDriver

A single owner thread (or a small pool used only by krABMaga `parallel`) that loops:

```
loop {
  apply control (Start | Pause | Step | Reset | LoadScenario | SetRate)
  drain intent queue up to cap N per tick (see §8)
  if running or one-shot step:
      state.apply_intents(batch)
      schedule.step(&mut state)
      committed_tick += 1
      publish snapshot delta / metrics
  else:
      wait on control or intent (or sleep to honor rate limit)
}
```

**Rate control.** Default: unconstrained (“maximum speed”). Optional `target_tps` for dashboard viewing and client interpolation. When unconstrained, do not publish a WS frame per tick if ticks exceed a publish budget (see §9.3).

**Pause.** Leaves `Schedule` and fields intact. Intents may still enqueue; they apply on the next running tick unless tagged `tick = T` and T is already past (rejected).

**Single-step.** One `schedule.step`, then pause.

**Reset.** Drop state, load last scenario (or empty world), `committed_tick = 0`.

### 6.2 State adapter

Define a crate-local `ServerState: State` that owns:

- `SparseA5Grid<AgentHandle>` or typed occupancy objects,
- any extra fields (`Network`, scalars, RNG),
- an **intent inbox** drained in `update` / before agent steps,
- a **metrics** struct (agent count, occupied cells, last step ns, dropped intents).

Agents remain krABMaga `Agent` implementations. Do not introduce async in `step`.

### 6.3 Maximum speed

Rules the engine crate must enforce:

1. Tick thread is `std::thread`, named `sim-driver`, optionally pinned.
2. No `block_on`, no `reqwest`, no file I/O inside the step. Scenario load and checkpoint write happen on Tokio or a blocking pool; the driver receives already-decoded `ScenarioBlob`.
3. Snapshot publish is zero-copy where possible: driver writes into a preallocated `watch` / triple buffer of `WorldView`; Axum serializes off-thread.
4. Logging in the hot path is counters, not per-agent strings.
5. When `parallel` is on, do not wrap the entire `State` in a single `Mutex` for the duration of every agent (that is the current krABMaga bottleneck). v1 may still use upstream `Schedule` as-is; v2 changes the step protocol (see §14).

### 6.4 Parallel feature

**v1 (must):** workspace builds with `--features gis,parallel`. A reference model (e.g. flockers or random walkers on `SparseA5Grid`) runs 1k ticks without panic. Document that results may differ from sequential.

**v2:** read-only neighbor access without `Mutex<State>`; writes only to write-bags / `DBDashMap` shards.

**v3:** partition occupied parent cells across workers; migrant queue at parent borders.

Dashboard exposes `parallel: bool` and `threads: u32` in run config. Changing them requires reset.

### 6.5 Determinism profile

| Profile | Use | Rules |
|---|---|---|
| `gold` | science / replay | sequential schedule, sorted bags, per-cell seeded RNG, no parallel |
| `fast` | games / viz | parallel allowed, iteration order undefined |

Checkpoints store the profile. `gold` replays must match tick-for-tick given the same intent log.

---

## 7. A5 addressing

### 7.1 Identifiers

- `GeoCell` (u64 from `bevy_a5` / A5) is the wire id. Encode as decimal string or hex in JSON; binary protocols use `u64`.
- `resolution: u32` — occupancy resolution `R_work`.
- `root: Option<GeoCell>` — if `Some`, the grid is an island (`new_with_root`). If `None`, global.
- Display / shard resolution `R_view` or `R_s` where `R_s < R_work`.

### 7.2 Queries the server must implement

Used by dashboard and clients; implemented in `sim-engine` with the same functions as the field:

- `lonlat_to_cell(lon, lat, res) -> GeoCell`
- `cell_to_lonlat(cell) -> (lon, lat)` (cell centroid)
- `parent(cell)`, `children(cell)`
- `neighbors(cell)`, `grid_disk(cell, k)`
- `cells_in_bbox(min_lon, min_lat, max_lon, max_lat, res) -> Vec<GeoCell>`

BBox is a convenience; storage and subscriptions remain cell sets.

### 7.3 Occupancy vs display

Occupancy lives at `R_work`. Dashboard heatmaps aggregate to `R_view` (count of agents per parent). Downloads may request either.

---

## 8. Intention protocol

### 8.1 Schema (`sim-core`)

```
Intent {
  id: Uuid,                  // client-generated, idempotency key
  tick: Option<u64>,         // None = apply on next committed tick + 1
  cell: GeoCell,
  agent: Option<AgentId>,
  actor: ActorId,            // user / system
  verb: IntentVerb,          // Spawn | Despawn | Move | SetField | Custom(u16)
  payload: Bytes,            // JSON or postcard, verb-defined
  issued_at: u64,            // client ms, diagnostic only
}
```

`IntentVerb::Custom` is the extension point for game-specific actions. The engine registers handlers: `fn apply(&mut ServerState, Intent) -> Result<()>`.

### 8.2 Admission

Axum validates auth, size, and that `cell` is inside the current root/resolution. It does **not** interpret game rules.

Driver admission (per tick, after control, before `step`):

1. Dedup by `id` (ring of last K ids).
2. If `tick` is `Some` and `tick <= committed_tick`: reject `Stale`.
3. If `tick` is `Some` and `tick > committed_tick + horizon`: reject `TooFar` (horizon default 8).
4. If queue length > `max_pending`: reject `Backpressured` (do not stall the tick).
5. Cap `max_applied_per_tick`.

Rejected intents are acked on the same WS connection with the reason. They are not applied later.

### 8.3 Semantics

- Apply on the write side so the following `step` sees them, or as a pre-step mutation of bags (spawn/despawn).
- A `Move` that crosses cells updates both bags in one helper.
- System intents (`LoadScenario` is control, not an intent) do not share this queue.

### 8.4 Intent log

Optional append-only log `{ tick, intent }` for `gold` replay. Stored beside checkpoints. Dashboard can download it.

---

## 9. State serving

### 9.1 Snapshot header

```
SnapshotMeta {
  scenario_id: String,
  tick: u64,
  profile: gold | fast,
  root: Option<GeoCell>,
  resolution: u32,
  agent_count: u64,
  occupied_cells: u64,
  hash: Option<u64>,        // gold profile: stable hash of bags
}
```

### 9.2 REST (operator + tools)

All JSON unless noted. Prefix `/api/v1`.

| Method | Path | Purpose |
|---|---|---|
| GET | `/health` | process up, driver alive, last tick age |
| GET | `/run` | status: mode, tick, tps, parallel, scenario |
| POST | `/run/start` | run unconstrained or with `{ target_tps }` |
| POST | `/run/pause` | pause |
| POST | `/run/step` | `{ n?: u32 }` single or n steps |
| POST | `/run/reset` | reload current scenario |
| POST | `/run/rate` | `{ target_tps: Option<f64> }` null = max speed |
| GET | `/scenarios` | list packs |
| POST | `/scenarios/{id}/load` | load + reset |
| POST | `/scenarios` | upload pack (multipart or JSON) |
| GET | `/config` | current run + engine config |
| PUT | `/config` | replace config (allowed keys only); may require pause |
| GET | `/config/download` | file attachment (TOML or JSON) |
| GET | `/state` | `{ cells?: [...], res?: u32 }` snapshot JSON |
| GET | `/state/download` | checkpoint file (postcard+zstd or JSON) |
| POST | `/state/upload` | restore checkpoint (paused) |
| GET | `/state/cells/{cell}` | bag at cell |
| GET | `/metrics` | Prometheus text or JSON |
| POST | `/intents` | submit one intent (REST fallback) |

Errors: problem+json, stable `code` (`stale_intent`, `unknown_scenario`, `sim_panic`, …).

### 9.3 WebSocket `/api/v1/ws`

Client → server:

```
Subscribe { cells: [GeoCell], res?: u32 }
Unsubscribe { cells }
SetAoi { center: GeoCell, k: u32 }     // server expands via grid_disk
Intent { ... }
Ping
```

Server → client:

```
Hello { tick, meta }
Snapshot { tick, cells: [{ cell, objects }] }   // initial fill for AOI
Delta { tick, upsert: [...], remove: [cell or object] }
Rejected { intent_id, reason }
Tick { tick, tps, agents }                      // heartbeat, rate-limited
Error { code, message }
```

**Publish budget.** If the sim runs faster than the wire:

- coalesce deltas until `publish_hz` (default 20 Hz) or `publish_every_n_ticks`,
- always include the latest `tick`,
- never block the driver on a slow socket (drop the client or skip frames; count `dropped_views`).

### 9.4 Checkpoint format

v1: JSON for interoperability + optional `application/zstd` postcard.

File sections: `meta`, `config`, `cells[]` of `{ cell, objects[] }`, `fields` (non-grid), `rng`, `intent_log?`.

Round-trip: upload checkpoint → same `tick` and, in `gold` profile, same `hash`.

---

## 10. Scenarios and configuration

### 10.1 Scenario pack

Directory or zip:

```
manifest.toml          # id, name, engine_rev, resolution, root, profile
config.toml            # run defaults: parallel, threads, publish_hz, target_tps
world/                 # optional pre-seeding
  agents.json          # or geojson points that compile to cells
  fields.json
params.toml            # model parameters (infection rate, etc.)
```

`manifest.id` is the load key. Loading is a control message: decode on Tokio, send `ScenarioBlob` to the driver, driver swaps state on the tick boundary (or immediately if paused).

### 10.2 Run configuration (downloadable)

```
[run]
profile = "fast" | "gold"
parallel = false
threads = 8
target_tps = null          # null = max speed
publish_hz = 20
intent_horizon = 8
max_pending_intents = 10000
max_applied_per_tick = 1000

[grid]
resolution = 15
root = null                # or hex GeoCell

[model]
# free-form, passed to State::reset
```

`GET /config/download` returns this file as it is *currently effective*, not the file on disk.

---

## 11. Dashboard specification

A browser UI served by Axum (`/` static + `/api`). v1: server-rendered pages with light JS, or a small SPA (htmx or Leptos). Not Bevy.

### 11.1 Screens

1. **Run control**  
   Status pill (running / paused / loading / fault). Buttons: Start (max speed), Start at N tps, Pause, Step, Reset. Live: tick, tps (sim and publish), agent count, occupied cells, intent queue depth, last step microseconds, dropped views.

2. **Scenarios**  
   Table of packs. Load, upload zip, show manifest. Confirm before replacing a running world.

3. **State view**  
   - Table: inspect a cell (paste hex / click map) → bag list.  
   - Coarse map: parent cells coloured by occupancy (Canvas or simple SVG). Click to set inspect cell and optional AOI.  
   - Optional live WS updates at `publish_hz`.

4. **Intents**  
   Form to submit a system intent (spawn at cell, despawn id). Recent accepts/rejects.

5. **Import / export**  
   Download config, download checkpoint (current tick), upload checkpoint, download intent log.

6. **Logs / fault**  
   If the driver panics, dashboard shows the catch: sim is paused, last good tick, button to reset. Axum stays up.

### 11.2 Auth (v1)

Single shared bearer token or basic auth from env (`DASHBOARD_TOKEN`). Bind `127.0.0.1` by default. No public anonymous write.

### 11.3 Accessibility of control

Start/stop must work if the map fails to load. Control API is independent of WS.

---

## 12. Faults, backpressure, lifecycle

- Driver panic: thread join reports to a `watch::Receiver<DriverHealth>`. Axum returns 503 on `/run` and `/state` writes. Restart driver with last scenario if `auto_recover=true`, else wait for operator reset.
- Slow dashboard clients never `await` inside the driver.
- Checkpoint write: snapshot the committed view into an owned buffer on the driver (copy bags for requested cells), write the file on a blocking thread.
- Graceful shutdown: Axum shutdown signal → control `Pause` → optional auto-checkpoint → join driver → exit.

---

## 13. Security notes

- Intents are untrusted payloads; cap size (e.g. 16 KiB) and rate per actor.
- Scenario upload is trusted-operator only (zip slip: reject `..`, cap uncompressed size).
- Do not expose raw `std::process` or path reads.
- CORS allowlist for the dashboard origin only.

---

## 14. Implementation plan

### Phase 0 — Fork / patch gate (3–5 days)

- Branch krABMaga: `gis` + `parallel` builds.
- `cargo test --features "gis parallel"` on field unit tests.
- Confirm `GeoCell` / `A5Cell3D` are `Copy + Eq + Hash + Send + Sync`.
- Spike: 10k agents sequential vs parallel on `SparseA5Grid`, record ticks/s. This is the speed baseline the server must not destroy.

**Exit:** numbers in `docs/baseline.md`, fork pin in workspace.

### Phase 1 — Headless driver (1 week)

- `sim-core` + `sim-engine` with one reference model (random walkers or flockers on A5).
- `SimDriver` in a thread, channels for `Start|Pause|Step|Reset`.
- In-memory scenario (no HTTP yet).
- Metrics: tick, ns/step, agents.
- CLI: `sim-engine --scenario demo --ticks 10000 --max-speed`.

**Exit:** CLI max-speed tps within ~10% of the Phase 0 baseline (serialization off).

### Phase 2 — Axum control plane (1 week)

- `/health`, `/run/*`, `/config`, `/metrics`.
- Embed driver in `sim-axum`.
- Token auth.
- Integration test: start → 1000 steps via API → pause → read tick.

**Exit:** operator can start/stop without a dashboard.

### Phase 3 — State and intents (1–1.5 weeks)

- REST snapshot by cell list; download/upload checkpoint JSON.
- WS subscribe + delta + intent + reject.
- Intent admission rules.
- Test: two clients, disjoint cells, intents do not leak across bags.

**Exit:** curl and a small WS script can drive and observe the world.

### Phase 4 — Dashboard (1–1.5 weeks)

- Screens in §11.
- Coarse parent-cell map.
- Load scenario zip from `scenarios/`.
- Download config + checkpoint from the UI.

**Exit:** a new operator can load a pack, run max speed, pause, inspect a cell, download state, without CLI.

### Phase 5 — Speed under I/O (3–5 days)

- Publish coalescing; verify max-speed tps with 1, 10, 50 WS subscribers on a small AOI.
- Checkpoint of 10⁵ agents without stalling ticks more than one step (copy-then-write).
- Feature flag `parallel` on the running binary; dashboard toggle requires reset.

**Exit:** written perf table; no unbounded memory from queued deltas.

### Phase 6 — Parallel v2 / cell partition (stretch)

- Remove full-`State` mutex from the hot path if still present upstream.
- Optional parent-cell work split inside one process.
- Dashboard shows worker counts and migrant stats.

Not required to call v1 done.

### Phase 7 — Bevy client (separate milestone)

Same protocol as the dashboard WS. Out of scope for the operator release; specified only so the wire format stays client-agnostic.

---

## 15. Testing strategy

| Layer | What |
|---|---|
| Unit | intent admission, cell query helpers, snapshot round-trip |
| Engine | reference model 1k ticks, bag conservation, pause/step |
| Feature matrix | `gis`; `gis,parallel` |
| API | axum `oneshot` tests for every route |
| WS | subscribe → spawn intent → delta contains object |
| Gold replay | intent log + checkpoint → matching hash |
| Perf | baseline vs server, with and without publishers |
| Chaos | kill WS clients, upload huge scenario (must reject), driver panic recovery |

CI: `fmt`, `clippy`, `test`, `test --features parallel`, build dashboard.

---

## 16. Risks

| Risk | Mitigation |
|---|---|
| `gis` pulls Bevy windowing into the server | fork / feature-split math-only A5; server uses headless |
| `parallel` + `gis` untested upstream | Phase 0 gate; do not advertise parallel until green |
| Snapshot clone kills max speed | cell-filtered views; triple buffer; publish_hz |
| HashMap bag order breaks gold | sort by agent id before hash and before gold step |
| Intent flood | caps + per-actor rate + drop, never block tick |
| Dashboard as a second sim | UI is view-only except control/intents |
| Scope creep into rollback netcode | v1 is conservative lockstep on the server; client prediction later |

---

## 17. Success criteria (v1)

1. Reference A5 model runs from the dashboard at max speed.
2. Measured tps on the driver is within 10% of the headless baseline with dashboard open and one WS map subscription.
3. Operator can load a scenario pack, start, pause, step, reset.
4. Operator can download effective config and a checkpoint that reloads to the same tick.
5. Intents submitted from the dashboard appear in the target cell on the next tick and are visible in `/state/cells/{cell}`.
6. Driver panic does not take down Axum; UI shows fault and allows reset.
7. `gis` + `parallel` builds; parallel run is optional and documented as `fast` profile.

---

## 18. Open decisions (resolve in Phase 0–1)

1. Checkpoint binary: postcard+zstd vs JSON-only in v1.  
   **Recommendation:** JSON download in UI; postcard on the wire flag later.
2. Dashboard stack: htmx+Askama vs Leptos vs static SPA.  
   **Recommendation:** Askama + htmx + a single Canvas module for the map (least JS, same Axum process).
3. Agent wire type: opaque JSON per model vs typed enum in `sim-core`.  
   **Recommendation:** `serde_json::Value` plus `kind: string` in v1; typed when the second model lands.
4. Multi-island (`new_with_root` many times) in one process.  
   **Recommendation:** one root per run in v1; islands are a v2 control-plane object.

---

## 19. Document history

| Ver | Date | Notes |
|---|---|---|
| 0.1 | 2026-09-10 | Initial plan: driver isolation, A5 I/O, Axum API, dashboard, phased delivery |
