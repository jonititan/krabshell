# Using the HTTP API

Everything the dashboard does, a script can do over HTTP. This page explains
how the API is put together: how to authenticate, what errors look like, what
the limits are, how intents reach the world, how to read state, how
checkpoints move in and out, and how the WebSocket streams the world live. It
ends with a worked `curl` session. The reference for each route, generated
from the OpenAPI document, is at [`/docs/api`](/docs/api); this page does not
repeat it.

The API never steps the simulation. Every route either reads the view the
driver last published or posts a message to the driver, so no request, however
slow, costs the simulation a tick (see [Architecture](architecture.md#the-driver)).

## Base path and versioning

Every route is under `/api/v1`. The prefix is the version: when a client would
have to change, the version is bumped, not the shape of `v1`. New fields and new
error codes may appear in `v1` responses; clients should ignore fields they do
not know.

The OpenAPI 3.1 description is served at `/api/v1/openapi.json`. Point Swagger
UI, Redoc, Postman or a client generator at it. It is kept by hand in
[`crates/sim-axum/openapi.json`](../crates/sim-axum/openapi.json), and the tests
in [`crates/sim-axum/tests/openapi.rs`](../crates/sim-axum/tests/openapi.rs)
fail if a route is added without it or a response grows a field it does not
describe.

The WebSocket has its own version number, sent in its first message (see
[WebSocket](#websocket)). It is `1`.

## Authentication

There is one shared secret for the whole server, read from the
`DASHBOARD_TOKEN` environment variable. There are no accounts. A request can
present the token in any of three ways:

| Where | Form | For |
|---|---|---|
| Header | `Authorization: Bearer <token>` | Scripts and tools. |
| Cookie | `krabshell_token=<token>` | The dashboard. The login form at `/login` sets it (`HttpOnly`, `SameSite=Strict`, seven days). |
| Query | `?token=<token>` | The WebSocket, since a browser cannot set headers on one. It works on every API route. |

Two routes need no token: `GET /api/v1/health`, so a process monitor can watch
the driver without holding a secret, and `GET /api/v1/openapi.json`, since the
description of the API is not a secret. The documentation pages under `/docs`
are public for the same reason. A request without a valid token gets a `401`
with the code `unauthorized`.

With `DASHBOARD_TOKEN` unset or blank the server accepts every request, and
refuses to bind anything but loopback. Set it before exposing the server. See
[Operating](operating.md) for the flags and environment.

Writes and WebSocket upgrades from a browser must come from the server's own
origin: one whose `Origin` names another site (or another port) gets `403`
with the code `forbidden`. Scripts that send no `Origin` are unaffected. See
[Requests from other sites](operating.md#requests-from-other-sites).

Treat it like a password: anyone who
holds it can drive the world, and the `actor` an intent claims is not checked
against it (see [Rate limits](#rate-limits)).

## Errors

Failures come back as `application/problem+json`, shaped after RFC 7807:

```json
{
  "code": "stale_intent",
  "title": "stale intent",
  "detail": "intent targets tick 5, already committed through 101",
  "status": 400,
  "meta": {"reason": "stale", "target": 5, "committed": 101}
}
```

Switch on `code`, never on `detail`, which is for people and may change. `meta`
is present only when there is machine-readable context; a rejected intent puts
its rejection reason and fields there. The codes and their statuses:

| Status | Codes |
|---|---|
| 400 | `bad_request`, `bad_cell`, `bad_config`, `malformed_scenario`, `stale_intent`, `intent_too_far`, `duplicate_intent`, `cell_out_of_bounds`, `intent_rejected` |
| 401 | `unauthorized` |
| 403 | `forbidden` |
| 404 | `not_found`, `unknown_scenario` |
| 409 | `requires_pause`, `requires_reset`, `intent_discarded` |
| 413 | `payload_too_large` |
| 429 | `rate_limited`, `backpressured` |
| 500 | `internal`, `sim_panic` |
| 502 | `weather_unavailable` |
| 503 | `driver_unavailable` |

The list is closed and only grows: adding a code is compatible, renaming one is
not. It lives in [`crates/sim-core/src/error.rs`](../crates/sim-core/src/error.rs).

`driver_unavailable` means the process is fine but the simulation thread is
gone or did not answer in time. `GET /health` answers `503` with
`"status": "degraded"` in the same case, and when the model has panicked
(`mode` is `faulted`).

Some requests are refused before a handler sees them, and those answers are
plain text, not problem bodies: a missing or malformed query parameter (`400`),
a JSON body without `Content-Type: application/json` (`415`), a body that is
not the JSON the route takes (`400` or `422`), and a body over the size limit
(`413`). A client should treat any non-JSON error body as a request it got
wrong.

## Limits

### Body size

| What | Limit | Answer when over |
|---|---|---|
| Any request body | 64 MiB | `413`, plain text: `length limit exceeded`. It comes before any handler, so it is not a problem body. |
| One intent on `POST /intents` | 16 KiB | `413`, `payload_too_large` |
| A scenario pack, once unzipped | 256 MiB and 4 096 files | `400`, `malformed_scenario` |

The overall cap is `MAX_BODY_BYTES` in
[`crates/sim-axum/src/lib.rs`](../crates/sim-axum/src/lib.rs). Checkpoints fit
with room to spare: `london-patrol` is about 1 MB, `kaunas-deliveries` about
1.6 MB at the start of a run and `swarm-survey` about 3.6 MB.

### Rate limits

Intents are rate-limited per `actor`, with a token bucket: `run.intent_rate_per_actor`
intents a second sustained (200 by default), in bursts of up to twice that. The
limit applies wherever an intent enters: `POST /intents`, the WebSocket, the
routes that build intents for you (geofences, stations, regions of interest,
all as the actor `operator`), the dashboard and the MAVLink gateway. Over the
limit an intent is refused with `rate_limited` (`429`) before it takes a place
in the driver's queue.

The actor is whatever the client says it is, so this stops runaway scripts, not
a hostile holder of the token. Nothing else is rate-limited.

The driver's own queue holds at most `run.max_pending_intents` (10 000 by
default). When it is full an intent is refused with `backpressured` (`429`)
rather than waiting: the tick loop never stalls to make room.

### Cells

One query may name or return at most 20 000 cells. `GET /state` without a
selection is refused when the world has more occupied cells than that (ask for
cells, or use `/state/map`); `/state/map` is refused when the resolution asked
for would give more. A WebSocket area of interest is capped at the same 20 000.

The driver publishes at most `run.max_published_cells` occupied cells (50 000 by
default). A larger world is cut at the cap, and every view of it says so with
`truncated: true`.

## Routes

The groups, in the order of the [reference](/docs/api). All paths are under
`/api/v1`.

| Group | Routes | What for |
|---|---|---|
| Health | `GET /health`, `GET /openapi.json` | Liveness and the API description. No token. |
| Run | `GET /run`, `POST /run/start`, `/run/pause`, `/run/step`, `/run/reset`, `/run/rate` | The clock: mode, tick, rates, counters and recent intents; start (optionally at `target_tps`), pause, step `n` ticks, reset, change rate. |
| Scenarios | `GET /scenarios`, `POST /scenarios`, `POST /scenarios/{id}/load` | List packs, install one from a zip (multipart field `pack` or `file`, and `id`), load one paused at tick 0. See [Scenarios](scenarios.md). |
| Config | `GET /config`, `PUT /config`, `GET /config/download` | The effective config, as JSON or TOML. A change to `run.profile`, `run.parallel`, `run.threads`, `grid.resolution`, `grid.root` or `model` rebuilds the world; the answer lists them in `reset_fields`. |
| State | `GET /state`, `/state/map`, `/state/cells/{cell}`, `/state/entities`, `/state/overlays`, `/state/addresses`, `/state/download`, `POST /state/upload` | The published world: bags by cell, an aggregated map, one cell, positioned objects, model overlays, and checkpoints. See [State](#state-and-the-cell-model) and [Checkpoints](#checkpoints). |
| Intents | `POST /intents`, `GET /intents/log` | Submit one intent; download the gold replay log. See [Intents](#intents). |
| Metrics and reports | `GET /metrics`, `GET /reports?since=` | Prometheus gauges; model events (sightings, waypoints, deliveries) newer than a sequence number. |
| Cells | `GET /cells/lookup?lon=&lat=&res=` | A coordinate to its cell, with outline, parent, edge length and whether it is in this world. |
| Geofences | `GET POST /geofences`, `DELETE PATCH /geofences/{id}`, `POST /geofences/{id}/extend` | List, open, close, rename and extend fences. See [Geofences](geofences.md). |
| Stations | `GET POST /stations`, `DELETE /stations/{id}` | Launch and recovery stations. See [Stations](stations.md). |
| Regions of interest | `GET POST /rois`, `DELETE /rois/{id}` | Areas for the drones to survey. See [Stations](stations.md). |
| Conflicts | `GET /conflicts`, `GET PUT /conflicts/settings` | Where aircraft could meet in the next minute, and what each is advised to do; the cell resolution it is worked out at (8 to 18). See [Conflicts](conflicts.md). |
| Weather | `GET /weather`, `POST /weather/refresh`, `GET /weather/cells`, `GET /weather/cells/{cell}` | The wind the run flies in, by cell, height and time. See [Weather](weather.md). |
| WebSocket | `GET /ws` | Live subscriptions and intents. See [WebSocket](#websocket). |

Every control route (`/run/*`, `/scenarios/{id}/load`, `PUT /config`,
`/state/upload`) answers once the driver has published the result, so the body
it returns already describes the world after the change. `POST /run/step` waits
up to ten seconds for its burst to finish; a longer burst answers with
`mode: "stepping"`, which is the truth. The geofence, station and region routes
are conveniences that build an intent and submit it as `operator`; they answer
like `POST /intents`.

## Intents

An intent is a request to change the world, addressed to a cell. It is not a
command. The driver may refuse it, and a refusal is final: nothing is retried
on the client's behalf. The types are in
[`crates/sim-core/src/intent.rs`](../crates/sim-core/src/intent.rs).

### Shape

```json
{
  "id": "6f1a1e2c-0000-4000-8000-000000000001",
  "tick": 250,
  "cell": "636119e9a0000000",
  "agent": 4,
  "actor": "planner-1",
  "verb": {"custom": 103},
  "payload": {"command": 400, "params": [1]},
  "issued_at": 1791634670582
}
```

| Field | Required | Meaning |
|---|---|---|
| `id` | yes | A UUID the client makes up. It is the idempotency key: a second intent with the same id is refused as `duplicate_intent`, never applied twice. The driver remembers the last 16 384 ids. |
| `cell` | yes | The cell the intent is about, at the world's resolution and inside its root (see [Cells](#cell-ids)). |
| `actor` | yes | Who is asking. Used for rate limiting and shown in the activity list and the intent log. |
| `verb` | yes | `"spawn"`, `"despawn"`, `"move"`, `"set_field"`, or `{"custom": n}`. |
| `tick` | no | The tick to apply it on. Absent means the next tick the driver commits. |
| `agent` | no | The agent it is about, for verbs that need one. |
| `payload` | no | Whatever the verb needs. Opaque to the server; the model reads it. |
| `issued_at` | no | The client's clock in milliseconds. Diagnostic only, never used for ordering. |

### Verbs

What a verb does is up to the model. The built-in models handle these:

| Model | Verb | Payload and effect |
|---|---|---|
| `walkers`, `walkers-krabmaga` | `spawn` | `{"count": n, "kind": "walker", "anchored": false}`, all optional. Places `n` (1 to 10 000) agents in the cell. |
| | `despawn` | With `agent`, removes that agent; without, empties the cell. |
| | `move` | Needs `agent`. Puts it in the cell. |
| `drones` | `spawn` | `{"kind": "person" \| "vehicle" \| "poi", "count": n}`. Places up to 1 000 at the cell's centre, on the ground. Drones are not spawned this way. |
| | `despawn` | As for walkers. |
| | `{"custom": 100}` to `103` | Mission upload, clear, set current item, and `COMMAND_LONG`. Need `agent`, the drone's id. See [Protocols](protocols.md). |
| | `{"custom": 110}` to `114` | Open, close, set (a ground station's whole plan), extend and rename geofences. See [Geofences](geofences.md). |
| | `{"custom": 120}`, `121` | Add and remove a station. See [Stations](stations.md). |
| | `{"custom": 130}`, `131` | Add and remove a region of interest. |
| | `{"custom": 140}` | Replace the wind field. The weather service sends these. See [Weather](weather.md). |

The drones refuse `move` and `set_field`: drones move by mission, and targets
move on their own. Every model refuses verbs it has no handler for, with
`intent_rejected`. The codes and their payloads are documented in
[`crates/sim-drones/src/world.rs`](../crates/sim-drones/src/world.rs) (module
`codes`).

### Admission

An intent passes these checks in order. The first it fails is its answer.

1. **At the edge** (REST or WebSocket, before the driver sees it). On REST the
   body must be at most 16 KiB and parse as an intent. The cell must be at the
   world's resolution and inside its root, or the intent is refused as
   `cell_out_of_bounds`. Then the actor's rate limit: `rate_limited`.
2. **Into the queue.** If the driver's queue is full: `backpressured`.
3. **On the tick thread, as it is taken off the queue.** This happens whether
   the world is running or paused. A repeated `id` is `duplicate_intent`. A
   `tick` already committed is `stale_intent`. A `tick` more than
   `run.intent_horizon` ticks ahead (8 by default) is `intent_too_far`.
4. **When its tick comes.** At the start of each tick the driver hands the
   model every queued intent that is due, up to `run.max_applied_per_tick`
   (1 000 by default); the rest wait for the next tick. The model may still
   refuse one, as `intent_rejected`, with its reason in `detail`.

An intent that passes step 4 is applied on that tick. For the drones,
"applied" means checked against the drone's committed state and handed to it;
the drone acts on it as it steps.

The cell is only checked against the world's bounds, not against where an agent
is. An intent addressed to a drone's old cell is still accepted: a drone can
move between the view a client read and the tick that applies the intent.

### What the answer means

`POST /intents` submits the intent and waits up to half a second for the
outcome:

- `200` with `"status": "applied"` and `applied_tick` when it was applied.
- `200` with `"status": "queued"` and `applied_tick: null` when it was admitted
  but not yet applied. A paused world never reaches it until it is started or
  stepped; an intent for a later tick waits for that tick.
- A problem body when it was refused at any step.

A queued intent that the model later refuses cannot be answered on the same
request. Its fate shows in `recent_intents` on `GET /run` (the last 32
intents, applied or refused), and the geofence, station and region lists carry
a `refused` list for the same reason.

On the WebSocket every intent gets exactly one answer: `accepted` once it has
been applied, or `rejected` with its reason. See
[Server messages](#server-messages).

### The intent log

In the `gold` profile the driver records every applied intent with the tick it
was applied on. `GET /intents/log` downloads that record as JSON Lines
(`application/jsonl`), one `{"tick": n, "intent": {...}}` per line, named
`<scenario>-tick<N>.intents.jsonl`. In the `fast` profile it answers `400`.

The log is what makes a gold run replayable: the same pack, seed and log give
the same world, hash for hash. It is also carried in gold checkpoints (see
[Checkpoints](#checkpoints)). It holds at most 100 000 entries; past that the
server logs a warning and records no more, and a replay of that run is
incomplete. Refused intents are not logged. Weather updates are intents too, so
a gold replay flies the same air.

## State and the cell model

### Cell ids

Every place in the world is an A5 cell: a pentagon on a
hierarchical, equal-area grid. A cell id is a 64-bit integer that encodes the
cell and its resolution. On the wire it is always a string of 16 lowercase hex
digits, zero-padded, such as `"636119e9a0000000"`, never a JSON number:
browsers would silently round ids above 2⁵³.

On input the server also accepts a `0x` prefix, and a decimal `u64`. Without
a prefix, a string is hex when it is 16 characters long or contains a letter
a–f, and decimal otherwise. So a decimal id of exactly 16 digits is read as
hex; send hex.

A world has two resolutions:

- `resolution`, the occupancy resolution: which cell each agent is in. Intents
  must be addressed at this resolution. It is 15 (cells about 180 m across)
  unless the pack says otherwise.
- `view_resolution`, the coarser one the map aggregates to by default (11).

A world may also have a `root`, a coarse cell it lives inside (an island run).
Cells outside it are out of bounds. Without a root the world is global.

Rough cell sizes: resolution 9 is about 11 km, 12 about 1.4 km, 15 about 180 m,
18 about 22 m. `GET /cells/lookup` gives the exact nominal edge for any
resolution.

### Reading state

Every state route reads the view the driver last published. It is rebuilt at
most `run.publish_hz` times a second (20 by default), and straight after every
control command. A read may therefore be up to one publish interval behind the
tick the driver has reached; the `tick` in each response says which tick it
describes.

The view has a header, `meta`:

| Field | Meaning |
|---|---|
| `scenario_id` | The loaded pack. |
| `tick` | The committed tick this view describes. |
| `profile` | `gold` or `fast`. |
| `root` | The island root, absent for a global world. |
| `resolution` | The occupancy resolution. |
| `agent_count`, `occupied_cells` | Totals. |
| `hash` | In `gold` only: an order-independent hash of every bag, the replay equality check. |

The world is a list of bags, one per occupied cell:

```json
{
  "cell": "636119e8a0000000",
  "objects": [
    {"id": 580, "kind": "person", "state": {"lat_e7": 515055379, "lon_e7": -1291234, "alt_mm": 0, "hdg_cdeg": 8354, "spd_cms": 140}}
  ]
}
```

`state` has no fixed schema; each model writes what it needs. The drones use
MAVLink's integer units: `lat_e7` and `lon_e7` (degrees × 10⁷), `alt_mm`,
`hdg_cdeg` (centidegrees), `spd_cms`. See [Drones](drones.md) for the rest.

| Route | Returns |
|---|---|
| `GET /state?cells=a,b,c` | The bags for those cells (empty cells are left out). |
| `GET /state?center=c&k=n` | The bags in a disk of `n` rings (1 by default, at most 64) round `c`. |
| `GET /state` | Every published bag, if there are at most 20 000. |
| `GET /state/cells/{cell}` | One bag; an empty bag for an empty cell, or `404` if the view was truncated and the cell may have been cut. |
| `GET /state/map?res=n` | Counts aggregated to resolution `n` (the view resolution by default), each with its outline as `[lon, lat]` pairs, and the largest count. This is what the dashboard map draws. |
| `GET /state/entities?kind=drone,poi` | Every object with an exact position, flattened to degrees, metres and metres a second. The headers `x-sim-tick`, `x-sim-mode` and `x-sim-tick-seconds` let a client predict movement between polls. |
| `GET /state/overlays` | What the model publishes beside its agents: for the drones, geofences, open orders, stations, delivery range and regions of interest. The tick is in `x-sim-tick`. |
| `GET /state/addresses` | Every address in the loaded pack's address book, `[lat_e7, lon_e7, depot, label]` with `depot` indexing `depots`. The ETag is the scenario: send it in `If-None-Match` for a `304` until another is loaded. |

`GET /state` also accepts `res`, and ignores it; use `/state/map` to aggregate.

The `hash` is an unsigned 64-bit integer sent as a JSON number. JavaScript
cannot hold it exactly; compare hashes in a language with 64-bit integers, or
as the raw text.

### Rewinding

In the `gold` profile the driver keeps the recent past and rebuilds any tick
in it on request. `GET /history` says what it covers:

```json
{"available": true, "first_tick": 1800, "live_tick": 7800, "every_ticks": 100, "tick_seconds": 0.1}
```

Add `at=<tick>` to `GET /state/map`, `/state/entities`, `/state/overlays`,
`/state/cells/{cell}`, `/reports` or `/conflicts` to read the world as it was
at that tick, anywhere from `first_tick` to `live_tick`. A tick outside that is
`404`. The answer is rebuilt from a keyframe (one every `run.history_every_ticks`
ticks, the last `run.history_keyframes` kept) by replaying the logged intents
and steps, so it is the world exactly as it was; it costs up to
`history_every_ticks` steps the first time, and nothing for a tick asked for
again. In the `fast` profile `available` is false and `reason` says why.

### Looking up a cell

`GET /cells/lookup?lon=-0.1276&lat=51.5072` compiles a coordinate to its cell
at the world's resolution (or `res`):

```json
{
  "cell": "636119e9a0000000",
  "resolution": 15,
  "centroid": [-0.12756817991351, 51.50815132960366],
  "boundary": [[-0.12742227613000523, 51.506722983848306], "..."],
  "parent": "636119e980000000",
  "in_world": true,
  "edge_meters": 177.95797835302577
}
```

`in_world` is true when the cell is at the world's resolution and inside its
root: an intent addressed to it will pass the cell check. This is the usual
way to find the cell for an intent from a map position. `simctl cell --lon
--lat --res` does the same offline.

## Checkpoints

A checkpoint is the whole committed world as one JSON file: what an operator
downloads to keep a run, and uploads to go back to it. The format is in
[`crates/sim-core/src/snapshot.rs`](../crates/sim-core/src/snapshot.rs)
(`Checkpoint`) and
[`crates/sim-engine/src/checkpoint.rs`](../crates/sim-engine/src/checkpoint.rs).

`GET /state/download` asks the driver for a copy of the world and serves it,
pretty-printed, as `application/json` with a file name of
`<scenario>-tick<N>.ckpt.json`. The copy is taken between ticks; the run does
not have to be paused.

| Field | Contents |
|---|---|
| `meta` | The same header as a state view, with the gold `hash`. |
| `config` | The effective config: `run`, `grid` and the model's parameters. |
| `cells` | Every occupied cell's bag, uncapped. |
| `fields` | Model state that does not live in the grid. For the drones: airspace and fences, stations, regions, dispatch, spares, the seed, the weather field and more. |
| `rng` | The driver's random number state, so a gold run continues the same sequence. |
| `intent_log` | In `gold` only: the applied intents so far. |

`POST /state/upload` restores one. The body is the file itself, not a form:

```sh
curl -s -H "$T" -X POST --data-binary @london-patrol-tick102.ckpt.json \
  localhost:8080/api/v1/state/upload
```

- The run must not be running or stepping; otherwise the answer is `409`
  `requires_pause`.
- A gold checkpoint is checked against its own hash; one whose cells do not
  match is refused with `400` and `checkpoint hash mismatch`. A fast checkpoint
  has no hash to check.
- The world is replaced, paused at the checkpoint's tick, and the answer is
  `{"tick", "agent_count", "hash"}`.
- The checkpoint's wind field, if it has one, becomes the run's weather.
- The body limit applies (see [Body size](#body-size)).

`krabshell --checkpoint-on-exit PATH` writes one on shutdown.

## WebSocket

`GET /api/v1/ws` upgrades to a WebSocket that streams the cells a client cares
about and takes intents on the same connection. Messages are JSON text frames
with a `type` tag. The types are in
[`crates/sim-core/src/wire.rs`](../crates/sim-core/src/wire.rs) and the
connection handling in
[`crates/sim-axum/src/routes/ws.rs`](../crates/sim-axum/src/routes/ws.rs).

Connect with the token in the query string:

```
ws://127.0.0.1:8080/api/v1/ws?token=<token>
```

### Client messages

| `type` | Fields | Effect |
|---|---|---|
| `set_aoi` | `center`, `k`, optional `res` (ignored) | Replace the area of interest with a disk of `k` rings round `center` (`k` is capped at 64), at `center`'s resolution. |
| `subscribe` | `cells`, optional `res` | Add cells to the area of interest. When `res` is given, a cell coarser than the world's resolution is expanded to all its children at the world's resolution, so a client can subscribe to a whole district by naming its parent. Without `res`, cells are taken as given, and a cell at the wrong resolution matches nothing. |
| `unsubscribe` | `cells` | Remove exactly these cells. |
| `intent` | `intent` | Submit an intent (the same shape as for REST). |
| `ping` | | The server answers `pong`. |

An area of interest is capped at 20 000 cells. A message that would pass the
cap, or names a bad cell, is answered with an `error` and leaves the area as it
was. A message that does not parse is answered with an `error` with code
`bad_request`; the connection stays open.

```json
{"type": "set_aoi", "center": "636119e9a0000000", "k": 2}
{"type": "subscribe", "cells": ["636119e980000000"], "res": 15}
{"type": "intent", "intent": {"id": "6f1a1e2c-0000-4000-8000-0000000000a1",
  "cell": "636119e9a0000000", "actor": "ws-client", "verb": "spawn",
  "payload": {"kind": "poi"}}}
```

### Server messages

| `type` | Fields | When |
|---|---|---|
| `hello` | `tick`, `meta`, `protocol` | Once, first, on connect. `protocol` is `1`. |
| `snapshot` | `tick`, `cells` | On the next frame after the area of interest changes: every occupied cell in it. |
| `delta` | `tick`, `upsert`, `remove` | On each frame after that, if anything in the area changed. |
| `tick` | `tick` and the metrics | Every frame, whether or not anything changed. |
| `accepted` | `intent_id` | An intent sent on this connection was applied. |
| `rejected` | `intent_id`, `reason` and its fields | An intent sent on this connection was refused. |
| `error` | `code`, `message` | A message from the client could not be acted on. |
| `pong` | | The answer to `ping`. |

A `delta` carries, in `upsert`, the whole bag of every cell whose contents
changed or that became occupied, and in `remove`, one entry per cell that lost
agents: `{"cell", "objects": [ids]}` for the agents that left, or `{"cell"}`
alone when the cell emptied. Applying `upsert` (replace the bag) then `remove`
to the last state reproduces the current one. Either list is left out when
empty, and no `delta` is sent when nothing changed.

`rejected` carries the reason flattened beside the id, with the reason's own
fields:

```json
{"type": "rejected", "intent_id": "…", "reason": "stale", "target": 5, "committed": 101}
```

The reasons are `stale`, `too_far`, `backpressured`, `duplicate`,
`out_of_bounds`, `rejected` (with `message`), `rate_limited` and `discarded`
(with `why`). They are the same checks as for REST (see
[Admission](#admission)), named after the reason rather than the problem code.
`discarded` means the intent was queued but the world was replaced (a reset,
a scenario load, a checkpoint restore, or a config change that rebuilds it)
before its tick came. `accepted` comes once the intent has been applied, not
when it is queued, so on a paused world it waits for the next step.

One connection may have at most 1 000 intents waiting for an answer. More are
refused as `backpressured` until some are answered. An intent is capped at
16 KiB as on REST (an `error` with `payload_too_large`), a message at 1 MiB
(the server closes the connection), and a subscription's cells are counted
before a coarse cell is expanded, so asking for too much is refused at once.

`tick` carries the same counters as `GET /run`: `agent_count`,
`occupied_cells`, `sim_tps`, `publish_hz`, `last_step_ns`,
`intent_queue_depth`, `intents_applied`, `intents_rejected`, `dropped_views`
and `truncated_cells`.

### Frames and backpressure

Each connection is served by its own task, which wakes on a timer at the run's
`publish_hz` (read when the connection opens) and sends whatever the driver has
published by then. It never sees every tick: a world running at thousands of
ticks a second and a client reading at 20 Hz simply see different frames, and
consecutive frames may be several ticks apart. Neither waits for the other.

Deltas are worked out per connection, against the last frame that connection
was sent, off the tick thread. The driver does not know how many sockets are
open. A slow client holds up only its own task: its sends take longer, its
frames come further apart, and its next delta covers everything since the last
one, so nothing is lost and nothing queues. If a send fails the connection
closes. A client that falls behind and wants a clean slate can send its
`set_aoi` again, which brings a fresh `snapshot`.

With an empty area of interest a connection gets only `tick` heartbeats.
Intent answers are sent on the next frame after the driver decides.

## A worked session

This drives the `london-patrol` pack from the shell. It assumes a server on
`localhost:8080` started with `DASHBOARD_TOKEN` set; responses are trimmed.

```sh
T="Authorization: Bearer $DASHBOARD_TOKEN"
J="Content-Type: application/json"
API=localhost:8080/api/v1
```

Load the pack. It comes up paused at tick 0:

```sh
curl -s -H "$T" -X POST $API/scenarios/london-patrol/load
```

```json
{"mode": "paused", "tick": 0, "scenario_id": "london-patrol", "model": "drones",
 "profile": "gold", "meta": {"resolution": 15, "root": "6361800000000000",
 "agent_count": 1365, "occupied_cells": 334, "...": "..."}, "...": "..."}
```

Step 100 ticks. The answer comes once the burst has finished:

```sh
curl -s -H "$T" -H "$J" -X POST $API/run/step -d '{"n": 100}'
```

```json
{"mode": "paused", "tick": 100, "...": "..."}
```

Find the cell under a point, and read the cells round it:

```sh
curl -s -H "$T" "$API/cells/lookup?lon=-0.1276&lat=51.5072"
curl -s -H "$T" "$API/state?center=636119e9a0000000&k=1"
curl -s -H "$T" "$API/state/entities?kind=drone"
```

```json
{"cell": "636119e9a0000000", "resolution": 15, "in_world": true, "...": "..."}
{"meta": {"tick": 100, "...": "..."}, "cells": [{"cell": "636119e8a0000000", "objects": ["..."]}], "truncated": false}
[{"id": 1, "kind": "drone", "callsign": "HAWK-1", "lat": 51.504, "lon": -0.128, "alt_m": 30.0, "mode": "auto", "activity": "flying", "...": "..."}]
```

Place a point of interest there. The world is paused, so the intent is
admitted and waits:

```sh
curl -s -H "$T" -H "$J" -X POST $API/intents -d '{
  "id": "6f1a1e2c-0000-4000-8000-000000000001",
  "cell": "636119e9a0000000", "actor": "me",
  "verb": "spawn", "payload": {"kind": "poi"}}'
```

```json
{"id": "6f1a1e2c-0000-4000-8000-000000000001", "applied_tick": null, "status": "queued"}
```

Step once, and the run status shows it applied:

```sh
curl -s -H "$T" -H "$J" -X POST $API/run/step -d '{}'
```

```json
{"tick": 101, "intents_applied": 1, "recent_intents": [{"id": "6f1a1e2c-…", "actor": "me",
 "verb": "Spawn", "cell": "636119e9a0000000", "tick": 101, "rejected": null, "detail": "applied"}]}
```

Sending the same id again is refused straight away, even while paused:

```json
{"code": "duplicate_intent", "title": "duplicate intent", "detail": "duplicate intent id",
 "status": 400, "meta": {"reason": "duplicate"}}
```

Run at 50 ticks a second for a while, then pause:

```sh
curl -s -H "$T" -H "$J" -X POST $API/run/start -d '{"target_tps": 50}'
curl -s -H "$T" -X POST $API/run/pause
```

Download a checkpoint and the intent log (the pack is gold):

```sh
curl -s -H "$T" -OJ $API/state/download      # london-patrol-tick<N>.ckpt.json
curl -s -H "$T" -OJ $API/intents/log         # london-patrol-tick<N>.intents.jsonl
```

Restore the checkpoint later. Pause first, or the answer is `409
requires_pause`:

```sh
curl -s -H "$T" -X POST --data-binary @london-patrol-tick102.ckpt.json $API/state/upload
```

```
{"tick": 102, "agent_count": 1366, "hash": …}
```

For something live, open `ws://localhost:8080/api/v1/ws?token=…`, send a
`set_aoi`, and read the `snapshot`, `delta` and `tick` frames as they come.
