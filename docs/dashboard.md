# Dashboard

The dashboard is the operator console krabshell serves beside its API. It
loads scenarios, drives the clock, shows the world on a map, edits geofences,
stations and regions of interest, and takes checkpoints in and out. It is
server-rendered HTML ([Askama](https://github.com/askama-rs/askama) templates
in [`crates/sim-dashboard/templates`](../crates/sim-dashboard/templates)) with
[htmx](https://htmx.org/) for the forms and polling, and a little JavaScript
for the maps. Every button calls the same driver the REST API does; nothing in
the browser steps the simulation.

## Signing in

Every page except `/login`, `/docs` and the static assets sits behind the
shared token (`DASHBOARD_TOKEN`). A request without it is redirected to
`/login`.

The login form posts the token to `/login`. If it matches, the server sets a
`krabshell_token` cookie (`HttpOnly`, `SameSite=Strict`, kept for seven days)
and sends you to `/run`. The same cookie authorises the dashboard's own calls
to `/api/v1/...` and its WebSocket, so the browser never handles the token in
script. A wrong token returns the form with "That token was not accepted."

With no `DASHBOARD_TOKEN` set, the login page says so and accepts any token.
The server then refuses to bind anything but loopback; see
[operating.md](operating.md).

**Sign out**, at the right of the top bar, posts to `/logout`, which clears the
cookie and returns you to the login page.

## The top bar

The top bar holds the tabs, the page width setting and **Sign out**. `/`
redirects to **Run**.

| Tab | Path | What it is for |
|---|---|---|
| Run | `/run` | Clock control and what the numbers mean |
| Scenarios | `/scenarios` | List, load and upload scenario packs |
| State | `/state` | Occupancy map and cell inspector |
| Geofences | `/geofences` | Draw, close, extend and reopen keep-out zones |
| Stations & regions | `/stations` | Add and remove launch and recovery stations and regions of interest |
| Weather | `/weather` | Wind status, wind map and point queries |
| Conflicts | `/conflicts` | Conflict heatmap and resolution advisories |
| Intents | `/intents` | Submit an intent by hand and watch outcomes |
| Import / export | `/io` | Download config, checkpoint and intent log; restore a checkpoint |
| Docs | `/docs` | These guides, and the generated endpoint reference at `/docs/api` |

**Width** picks how wide the page may grow: Normal, Wide (up to 1800 px) or
Full (the whole window). The choice is stored in the browser
(`localStorage`, key `krabshell.width`) and applied before the page draws, so
it does not jump. The maps redraw to fit when it changes.

## The control panel

Every page except the docs carries the same control panel at the
top. It is one htmx fragment (`/fragments/control`) that refreshes itself
once a second. Each button swaps the same fragment with the result, so a click
and the poll never fight over the page. A green, amber or red banner just
above the panel reports the outcome of the last action. The poll leaves it
alone; a green one fades after six seconds, and an amber or red one stays
until the next action.

| Control | Dashboard action | API equivalent |
|---|---|---|
| **Start** | `POST /actions/start` | `POST /api/v1/run/start` |
| **Pause** | `POST /actions/pause` | `POST /api/v1/run/pause` |
| **Step** (count box, blank is 1) | `POST /actions/step` | `POST /api/v1/run/step` |
| **Reset** (asks first) | `POST /actions/reset` | `POST /api/v1/run/reset` |
| **Speed**: Max, Real time, Custom | `POST /actions/speed` | `POST /api/v1/run/rate` |

**Start** keeps the speed already chosen. **Step** shows the world after the
step, not the frame that accepted it. **Reset** drops the world and reloads
the scenario from tick 0.

**Speed** applies at once, running or paused. *Max* runs as fast as the
driver can. *Real time* is one tick per `tick_seconds` of the scenario, and is
greyed out for a scenario without `tick_seconds`. *Custom* takes a rate in
ticks per second; typing in the box selects it, and what you type survives the
once-a-second refresh.

Below the buttons the panel shows tick, simulated time, agents, occupied
cells and the measured sim rate, then a smaller row: target rate, sim time per
tick, publishing rate, last step time, intent queue depth, intents applied and
rejected, dropped views, whether the schedule runs in parallel, and the
occupancy and view resolutions and root cell. The Run page explains the main
ones.

The panel also raises banners for three faults:

- the simulation thread has stopped (restart the server);
- the model panicked (the world is frozen at the last good tick; Reset to
  continue);
- the published view hit `run.max_published_cells`, so some cells are missing
  from the map and from lookups.

## Rewind

Above the control panel, **⏪ Rewind** pauses the run and lets you step back
through what just happened, then replay it and carry on.

- **⏪ Rewind** pauses the run (so the past stops growing) and puts you in
  review at the current tick. The strip turns amber and reads
  `REVIEW · tick 2 470 · 00:04:07 · 30.1 s behind now`.
- **◀ / ▶** step one tick; **−10 s / +10 s** ten simulated seconds; the slider
  drags to any tick in the history.
- **▶ Replay** plays forward from the cursor at the chosen speed (×1 is real
  time, up to ×64). When it reaches now, review ends and the run starts again
  at the speed it had: what already happened plays first, then the simulation
  picks up where it left off. Press it again to pause the replay.
- **⏭ Now** leaves review without starting the run. Pressing Start or Step on
  the control panel also ends review.

While reviewing, the maps (drones, aircraft and their rings, people and
vehicles, occupancy, overlays), the cell inspector and the Conflicts screen's
heatmap and tables show the cursor's tick. The control panel, the geofence and
station lists and anything sent out (MAVLink, CoT, run logs) stay live.
Sightings, which fade in as they are reported, are not replayed. Each browser
tab has its own cursor, kept while you move between pages.

The past is rebuilt, not recorded. Every `run.history_every_ticks` ticks (100)
the driver hands a keyframe of the world to a history thread, which keeps the
last `run.history_keyframes` (60: ten minutes at 0.1 s a tick), and every intent
applied since the oldest. To show a tick it restores the keyframe before and
replays the intents and steps up to it, so what you see is the world exactly
as it was, conflicts included. That needs the `gold` profile, which replays to
the bit; with `fast` the strip says rewind is unavailable. A load, reset or
checkpoint restore starts the history again.

## Run

The control panel and a glossary: sim time, sim time per tick, sim rate,
publishing, dropped views and intent queue. A large dropped-views count at
maximum speed is normal: it counts snapshots replaced before anything read
them. The driver itself is described in [architecture.md](architecture.md).

## Scenarios

The **Packs** table lists every pack in the scenarios directory: id, name and
description, model, resolution, root cell, profile and number of seeded
agents. The current pack is highlighted. It refreshes every ten seconds
(`/fragments/scenarios`).

**Load** (or **Reload** on the current pack) asks for confirmation, then posts
to `/actions/load/{id}`; the API equivalent is
`POST /api/v1/scenarios/{id}/load`. Loading replaces the running world and
returns to tick 0. A pack whose model this build cannot run is tagged "not in
this build" and its button is disabled.

**Upload a pack** takes an id and a zip of `manifest.toml`, `config.toml`,
`params.toml` and an optional `world/`. The form posts straight to
`POST /api/v1/scenarios` as multipart, so the browser shows the API's JSON
reply rather than a dashboard page; go back to see the pack in the list. The
page also lists the model kinds this build can run. Pack contents are in
[scenarios.md](scenarios.md).

## State

The **Occupancy map** draws agents per A5 cell, with drones, stations,
geofences and the rest over it. Its controls:

| Control | Effect |
|---|---|
| Display resolution | The A5 resolution cells are aggregated to, from 0 to the occupancy resolution. Starts at the scenario's `grid.view_resolution`. |
| Basemap | None, or map tiles under the cells. |
| Live | When unticked the map stops polling and holds what it shows. |
| Fit | Zoom to the occupied cells. |
| Layers | Show or hide each layer. |

The map, its layers, symbols and interaction are in [map.md](map.md).

**Inspect a cell** shows one cell: id, resolution, centroid, whether it lies
in this world, the agent count, and a table of every agent in it with its
kind and state. A coarse cell (coarser than occupancy) also gives the number
of occupied cells inside it and each agent's occupancy cell, and caps the
list. Clicking the map inspects the clicked cell at the display resolution;
pasting an id into the box and pressing **Look up** inspects any cell at any
resolution. Both fetch `/fragments/cell?cell=...`; the API equivalent is
`GET /api/v1/state/cells/{cell}`.

`/state?cell=<id>` opens the page with that cell already inspected.

## Geofences

A map with drawing tools (**Select**, **Draw circle**, **Draw polygon**), a
draft form for name, radius, duration and altitude band, and three tables:
current fences (with rename, extend and close), previous fences (with reopen),
and recent breaches and reroutes. The page drives `/api/v1/geofences` and
needs JavaScript throughout. A banner says when the scenario's model has no
geofences. See [geofences.md](geofences.md).

## Stations & regions

A map with **Select**, **Add station** and **Add region** tools, a draft
form, and tables of launch and recovery stations (slots in use, drones
waiting, remove), the couriers' delivery range per hub, regions of interest
(surveys, last surveyed, tasked drones, remove) and recent activity. It drives
`/api/v1/stations` and `/api/v1/rois` and needs JavaScript. See
[stations.md](stations.md).

## Weather

The **Status** panel is server-rendered and refreshes every five seconds
(`/fragments/weather`): mode, simulation time in UTC, tick 0, source, window,
weather cells, heights, area, last fetch, cost, cache and launch limits. In
live mode **Refresh now** posts to `/actions/weather/refresh` (API:
`POST /api/v1/weather/refresh`). A run without weather says so and shows the
`params.toml` lines that turn it on.

The **Wind map** shades each weather cell by wind speed at a chosen height,
either now (following the simulation) or at any hour of the window. Clicking
the map, or the **Query** form (latitude and longitude, or an A5 cell, and a
height), shows the wind there: by height now, and through the window at that
height. These use `/api/v1/weather/cells` and need JavaScript. See
[weather.md](weather.md).

## Conflicts

The **Conflict heatmap** shades every cell an aircraft could be in over the
next minute, by aircraft expected or by the chance of two or more. **Cells**
picks the A5 resolution conflicts are counted at; it posts to
`/actions/conflicts/resolution` (API: `PUT /api/v1/conflicts/settings`) and
applies at once.

Below the map, two server-rendered tables refresh every two seconds
(`/fragments/conflicts`): **Requests for conflict resolution** (the advisory
for each aircraft, who it conflicts with, how soon, the total risk before and
after, and the outcome) and the **Top 10 A5 cells by conflict probability**.
Clicking a row picks its cell out on the map and centres on it. The tables
work without JavaScript; the map does not. See [conflicts.md](conflicts.md).

## Intents

A form for one intent: cell (at occupancy resolution), verb (`spawn`,
`despawn`, `move`), and optionally agent id, count and kind. It posts to
`/actions/intent`; the API equivalent is `POST /api/v1/intents`. The driver
admits it to the queue and the model applies or refuses it on the next tick.
Despawn with no agent clears the whole cell; move needs an agent id.

The **Activity** table lists recent intents (tick, actor, verb, cell,
outcome), refreshed every two seconds. The result of your own submission
appears in a banner under the form, and stays until you submit again. Intents from the dashboard are subject to the same
per-actor rate limit as any other.

## Import / export

| Link | API | What you get |
|---|---|---|
| Effective configuration | `GET /api/v1/config/download` | TOML, as the driver is running it, not the file on disk |
| Checkpoint at the current tick | `GET /api/v1/state/download` | JSON: header, config, every occupied cell, model fields and generator state |
| Intent log | `GET /api/v1/intents/log` | JSON Lines for gold replay; only offered in the gold profile |

**Restore a checkpoint** is only offered while the run is paused (reload the
page after pausing). It uploads the file to `POST /api/v1/state/upload` from
script and reports the tick and agent count it restored to. The world and the
tick counter jump to whatever the file says.

## Docs

The last tab is this documentation: the guides from the repository's `docs/`
directory, compiled into the binary so they always match the build serving
them, with a contents list per page. The endpoint reference is at `/docs/api`,
rendered from the OpenAPI 3.1 document at `/api/v1/openapi.json`; it lists
every operation by tag with its parameters, body and responses, then the
schemas. None of these need a token. [api-guide.md](api-guide.md) shows how to
use the API from scripts.

## What needs JavaScript

The console is built so its core works with htmx alone, and the parts that
need more say so with a `<noscript>` note.

| Works with htmx form posts and polling | Needs JavaScript beyond htmx |
|---|---|
| Control panel: start, pause, step, reset, speed | Every map ([map.js](../crates/sim-dashboard/assets/map.js)) |
| Loading a scenario | Geofence, station and region editing and their tables |
| Cell lookup by id | Clicking the map to inspect a cell |
| Intents form and activity | Live cell view over the WebSocket |
| Weather status and refresh | Wind map and wind query |
| Conflict resolution choice and tables | Conflict heatmap |
| | Rewind ([timeline.js](../crates/sim-dashboard/assets/timeline.js)) |
| Downloads | Checkpoint restore |

htmx itself is a script, so "without JavaScript" here means without anything
but htmx. Scenario upload and the downloads are plain HTML and work with no
script at all.

On the State page, [live.js](../crates/sim-dashboard/assets/live.js) keeps the
inspected cell current. It opens the WebSocket at `/api/v1/ws`, sets its area
of interest to the inspected cell (`set_aoi`, `k` = 1), and re-fetches the cell
fragment on each `snapshot` or `delta`. A cell coarser than occupancy gets no
deltas of its own, so it is re-fetched on the `tick` heartbeat instead, at most
once a second. If the socket drops it reconnects with back-off up to a minute,
and the map status line notes "live updates unavailable". **Look up** still
works without it. The protocol is in [protocols.md](protocols.md).
