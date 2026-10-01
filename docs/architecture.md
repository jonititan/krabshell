# Architecture notes

The spec (`krabshell_spec.md`) is the design. This file records how the
scaffold implements it and the things discovered along the way that the spec
did not know.

## Crates

| Crate | Owns | Must not |
|---|---|---|
| `sim-core` | `GeoCell` and A5 queries, intents, snapshots, checkpoints, config, scenario manifests, WebSocket messages, problem codes, the deterministic RNG | depend on Bevy, Axum, Tokio or krABMaga |
| `sim-engine` | the driver thread, the `Model` trait, the built-in grid, reference models, scenario loading, checkpoint encoding, `simctl` | await, open sockets, or do file I/O inside a tick |
| `sim-mavlink` | the MAVLink gateway: one UDP socket presenting every drone as a vehicle, the mission handshake, command acks | step the schedule or touch model state; block on the driver |
| `sim-cot` | the Cursor-on-Target emitter: drone tracks, sightings and POIs as XML over UDP | step the schedule or touch model state |
| `sim-axum` | REST, WebSocket, auth, the dashboard, the `krabshell` binary | step the schedule or touch model state directly |
| `sim-dashboard` | Askama templates and static assets, compiled into `sim-axum` | contain a build step |

## The driver

One `std::thread` named `sim-driver` owns the `Model`. Three channels:

- **Control in** (bounded): start, pause, step, reset, load, set config,
  restore, checkpoint. Each carries an optional ack. The driver publishes
  *before* it acks, so a caller that reads the view after a successful command
  sees the state it asked for.
- **Intents in** (bounded at `max_pending_intents`): a full channel is a
  `Backpressured` rejection at submit time. Nothing waits.
- **Snapshot out**: an `ArcSwap<Published>`. The driver swaps in a new view at
  most `publish_hz` times a second, and immediately after operator actions.
  Readers clone an `Arc`. A slow reader never delays a tick; a fast loop never
  queues frames. Frames nobody read are counted as `dropped_views`.

A step burst is acked when accepted, not when finished. `POST /run/step` and
the dashboard's Step button wait (up to 10 s) for the burst to end so their
response describes the world after the step.

The tick body runs under `catch_unwind`. A panic puts the driver in `Faulted`
at the last good tick; the thread survives and `Reset` recovers. When the
thread itself exits, the final published view has `alive: false` and
`/health` answers 503.

**Reports out.** Besides state, a model can raise events through
`Model::drain_reports`: a sensor saw something, a vehicle reached a waypoint.
The driver drains them after each successful step, stamps each with a `seq`
that never goes backwards (not even across a reload), and keeps the last 512 in
the published view. Consumers remember the last `seq` they handled and read
`Published::reports_since`. A gap in `seq` means they fell behind.

## Edges

Every way into the driver shares `sim_engine::edge`: the cell check (right
resolution, inside the root) and the per-actor rate limiter. REST, WebSocket,
the dashboard form and the MAVLink gateway all use it, so they cannot drift
apart. (Before this they were three copies, and the dashboard's had lost the
root check.)

The drone gateways follow the same rules as HTTP (operator guide:
[`connecting.md`](connecting.md)). They read `DriverHandle::view()`
on their own timers and submit intents; they never step the schedule.

- **MAVLink** (`sim-mavlink`) runs the real-time half of the protocol itself.
  Heartbeats at 1 Hz and telemetry at `--mavlink-hz` go out on the wall clock
  whatever the tick rate. The upload handshake, with its 1.5 s item timeout
  and retries, completes in the gateway. Only a finished mission reaches the
  simulation, as one intent, so a drone never flies half a plan. Acks come
  back on a crossbeam channel polled every 20 ms. A `MISSION_ACK` waits until a
  published view carries the new mission (its `mission_rev` moves), so a
  download straight after an upload returns the new mission. Commands are
  refused with `TEMPORARILY_REJECTED` while the driver is paused, so nothing
  queued fires unexpectedly when it resumes.
- **CoT** (`sim-cot`) sends one XML event per datagram. Event times are wall
  clock, because TAK clients age markers against their own clocks; the sim
  tick goes in `<remarks>`.

The drone model publishes each drone's full state (pose, mode, mission, battery)
in `WorldObject.state`, and `drones::drones_in` parses it back. That parser is
the contract both gateways read through; neither sees model internals.

## Models

`Model` is the seam. The driver knows how to reset one, hand it admitted
intents, step it, and read a sorted, capped snapshot out of it.

- `walkers` — built-in double-buffered grid. Builds without Bevy.
- `walkers-krabmaga` — krABMaga's `SparseA5Grid` as occupancy of record.
  Needs `--features krabmaga` (or `parallel`).

Any walker pack can run on either with `--model`. Both follow the same intent
rule: an agent placed by an intent (spawn or move) holds that cell for the tick
that applies it instead of walking off it.

- `drones`: drones, intruder drones, people, vehicles and points of
  interest. Positions are continuous; occupancy is the cell each entity is in.
  Each tick is `tick_seconds` of simulated time. Nothing paces it to the wall
  clock; that is the gateways' job.
  - **Missions.** They execute MAVLink commands (see `drones/mission.rs`) and
    arrive as `Custom` intents 100–103 carrying `MISSION_ITEM_INT` /
    `COMMAND_LONG` fields. An intent addressed to a drone's old cell is still
    accepted: the drone moves between publish and apply, and only the world
    bounds are checked.
  - **Gold determinism.** Poses are integers in MAVLink's `_INT` units, trig
    goes through `libm`, collections are `BTreeMap`s, and every random choice
    comes from a (seed, entity, tick) stream.
  - **Occupancy refresh.** Occupancy is re-resolved when a pose crosses a
    fixed ~4.5 m square, not after "distance since the last lookup". That
    keeps it a pure function of state, so a restored checkpoint follows the
    same path as an uninterrupted run (a test checks this).

## Things the spec did not know

- **krABMaga 0.6.2 already has** `apply_to_all_values`, grid-disk,
  spherical-cap and lon/lat helpers on `SparseA5Grid`. Two items on the
  spec §5 patch list are done upstream.
- **`gis` pulls in all of Bevy**, not just `bevy_a5`. The grid only needs
  `bevy_a5`, which itself needs only Bevy's ECS/maths sub-crates. A
  feature split upstream would remove ≈600 crates from a server build.
- **`SparseA5Grid::lazy_update` clears the write side.** An agent not
  re-placed during a tick disappears at commit. Correct for krABMaga's
  "every agent writes itself every step" convention; a trap for anything
  that mutates the grid between steps (like intents). The krABMaga-backed
  model buffers intents and applies them at the start of the next step.
- **The single-threaded `SparseA5Grid` has no object→cell index.**
  `get_location` scans every bag. Models must track agent locations
  themselves. The `parallel` variant does have `obj2loc`.
- **`a5::grid_disk` returns a compacted cover.** Runs of siblings are merged
  into parents, so the centre cell may not appear. `GeoCell::grid_disk`
  expands back to the centre's resolution; `grid_disk_compact` keeps the raw
  form.
- **A5 resolutions are coarse.** Resolution 9 is ≈11 km per cell, 12 is
  ≈1.4 km, 15 is ≈180 m, 18 is ≈22 m. The defaults are 15 for occupancy and
  11 for the dashboard map.
- **`a5` must stay on the major version `bevy_a5` uses** (0.7 today), or cell
  ids computed by the server and by krABMaga could diverge.

## Protocol notes

- Cells travel as 16-digit zero-padded hex strings, never JSON numbers:
  browsers would silently round ids above 2⁵³.
- Errors are `application/problem+json` with a stable `code`. Clients branch
  on `code` only.
- WebSocket deltas are computed per connection, off the tick thread, by
  diffing the connection's area of interest against the last frame it sent.
- `/health` is deliberately unauthenticated so a process monitor needs no
  secret. Everything else needs the token.
- Intents are rate-limited per claimed actor id at the edge (token bucket,
  `run.intent_rate_per_actor`, bursts of twice the rate), before they can take
  a slot in the driver's queue. With one shared token the actor id is
  self-declared, so this stops runaway clients, not a hostile token holder.

## Known gaps

- Spec Phase 5 measurements (many subscribers, large checkpoints under load).
- Intra-step parallelism and parent-cell partitioning (Phases 6+). `run.parallel`
  and `run.threads` are accepted and shown but nothing reads them yet.
- MAVLink is unsigned and CoT is plain UDP; neither is authenticated.
- The dashboard map shows occupancy, not individual drones or sightings.
- Tested against pymavlink and a CoT listener, not yet against QGroundControl
  or ATAK themselves.
- CORS allowlist (§13): the dashboard is same-origin, so none is configured.
- Checkpoints are JSON only; postcard+zstd is a later wire flag (§18.1).
