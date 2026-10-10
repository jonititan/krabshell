# Geofences

Geofences are keep-out zones in a drones scenario. They are circles and polygons
that close airspace to drones, from the surface up or only within an altitude
band. A scenario pack can start with its own fences, and you can open, close,
extend and rename fences while a run goes on, from the dashboard, the REST API,
raw intents or a ground station's fence plan. The simulation flies its own drones
around the fences, and under a fence whose floor is high enough. Every drone found
inside a fence is reported as a breach. This page covers what a fence is, how
drones avoid one, and how to manage fences. The code is in
[`airspace.rs`](../crates/sim-drones/src/airspace.rs) (geometry and routing),
[`operator.rs`](../crates/sim-drones/src/operator.rs) (the MAVLink rerouting) and
[`world.rs`](../crates/sim-drones/src/world.rs) (intents, scheduling and reports).

## What a fence is

A fence has a shape, an optional altitude band, an optional name and an id. Ids
start at 1, count up and are never reused within a run, so a fence that reopens
gets a new number.

| Field | Meaning |
|---|---|
| `circle` | `{"lat", "lon", "radius_m"}` in degrees and metres. The radius must be from 1 to 50000 m. |
| `polygon` | `[[lon, lat], ...]` in degrees, corners in order, not closed. It needs at least three corners that enclose more than 1 m². |
| `floor_m` | Where the fence starts. Absent, null or 0 means the surface. |
| `ceiling_m` | Where the fence ends. Absent or null means unlimited. It must be above the floor (or above the ground, with no floor). |
| `altitude_ref` | `"agl"` (the default) or `"amsl"`. See [Altitude bands](#altitude-bands). |
| `name` | Free text. It is trimmed and cut to 80 characters. It may be empty. |
| `minutes` | How long the fence stays open, in simulated minutes. Absent, null or -1 means until it is closed. Any other value must be more than 0. |
| `after_minutes` | How many simulated minutes it waits before opening. Absent, null or 0 means it opens on the next step. |

Every corner and centre must be inside the world. If a fence would keep drones off
any drone's home pad, it is refused, and this check includes the
[horizontal margin](#how-drones-keep-clear). It is made against the fence's
outline whatever its altitude band, so a raised fence over a pad is refused too.

Each fence records who opened it, as `source`:

| `source` | Opened by | Dashboard shows |
|---|---|---|
| `scenario` | The pack's `world/geofences.json`. | Scenario |
| `dashboard` | The dashboard, or the `/api/v1/geofences` endpoints (the API always sets this). | Dashboard |
| `gcs` | A ground station's fence plan. | Ground station |
| `intent` | A raw `Custom` 110 intent that names no source of its own. | API |

A raw intent can give its own `source`: up to 24 letters, digits, `-` or `_`,
but not `scenario` or `gcs`.

### Altitude bands

Heights are metres above ground level (AGL) by default. A drone is inside a fence
when it is within the shape, its height above the ground is at least the floor,
and it is below the ceiling. A drone on the ground is never inside one.

With `"altitude_ref": "amsl"` the floor and ceiling are read as metres above mean
sea level. They are turned into heights above the ground once, when the request
is checked, using the terrain at the fence's centre: a circle's centre, or the mean
of a polygon's corners. After that the fence is stored and published in AGL. A
floor that ends up at or below the ground becomes the surface. A ceiling at or
below the ground is refused.

The dashboard only takes AGL heights. Pack fences and raw intents can use AMSL.
The REST endpoint passes `altitude_ref` on to the model, but it refuses negative
heights before the model sees them.

## How drones keep clear

The autopilot knows nothing of fences. Clearance is the job of the **operator**:
the part of the simulation that flies the drones the simulation flies itself
(couriers, and drones a pack starts on a mission). It keeps those drones clear the
way a ground station would, by MAVLink. Drones flown by an outside ground station
are left to that station (see [Ground stations](#ground-stations-and-mavlink)).

### The horizontal margin

Each fence is grown by `geofence_margin_m` into a convex outline. The default is
30 m, and the value is clamped to between 2 and 1000 m. A circle's outline is a
16-sided polygon whose edges touch the circle. A polygon is flown around as its
convex hull, so drones also keep out of the notch of an L-shaped fence. Whether a
drone is *inside* a polygon is still judged by the polygon itself.

A leg that would cross an outline is replaced by the shortest way round the
outlines' corners. When the operator replans a drone, it sends:

- the mission again, with detour waypoints inserted wherever a leg crosses a fence
  (`MISSION_COUNT`/`MISSION_ITEM_INT`), `DO_JUMP` targets renumbered to match, and
  `MISSION_SET_CURRENT` to carry on along it;
- if the drone cannot see a clear way from where it is to its place on the new
  mission, `DO_REPOSITION` to each corner of that way in turn, then
  `MISSION_START`;
- for a drone returning home, a mission of detour waypoints at the return height
  that ends in `NAV_LAND` on its pad, in place of RTL;
- for a drone that a fence has opened round, `DO_REPOSITION` to just outside the
  nearest side, then the rerouted mission (or `NAV_RETURN_TO_LAUNCH`) once it is
  out;
- for a courier whose delivery address is now under a fence it cannot get beneath,
  `NAV_RETURN_TO_LAUNCH`. The order goes back in the queue when the courier lands,
  unless `reject_orders_in_geofences` turns it away (see
  [Effect on orders](#effect-on-orders)).

A mission item whose destination is inside a fence is moved to the fence's edge,
so the drone holds there. The operator replans a drone whenever the airspace
changes (any fence opening or closing), the drone's mission changes, or its
flight mode changes. When a fence closes, its detours come out again. If a
rerouted mission would be longer than `max_mission_items` (default 128), the
operator sends nothing for that drone.

### Flying beneath a raised fence

A fence matters to a leg only if the leg is flown within
`geofence_vertical_margin_m` of the fence's band. The default margin is 10 m,
clamped to between 0 and 500 m. A fence that is in the way can also be flown
beneath when its floor less the vertical margin is at least `beneath_min_agl_m`.
The default minimum is 20 m, clamped to between 1 and 1000 m. With the defaults,
the floor must therefore be 30 m AGL or higher.

The operator plans a blocked leg both ways: around the fences at the leg's height,
and beneath them at the lowest floor in the way less the vertical margin. It flies
the low way when that is shorter, counting the drop and the climb back
(`2 × (cruise height − low height)`), or when going round has no good path. On
the low way, the drone drops to the low height where the leg starts, follows the
terrain under the fence, and climbs again where the leg ends.

If a delivery address lies under such a fence, the delivery item is lowered to
the low height, and the drone winches the parcel down from there instead of
climbing into the fence.

A fence with a ceiling does not get in the way of a leg flown at least the
vertical margin above that ceiling. However, the operator never climbs a drone to
pass over a fence. It only goes round or beneath.

### What a fence covers

A fence that reaches too low to fly beneath (one whose floor less the vertical
margin is below `beneath_min_agl_m`, including every fence from the surface)
*covers* the ground under it:

- no order to an address under it is dispatched;
- a courier already on its way there is sent home;
- no station under it can be added, and stations already under it are left out
  of trip planning and the [delivery range](stations.md#delivery-range).

## Effect on orders

What happens to orders for a covered address depends on the pack parameter
`reject_orders_in_geofences`. It defaults to false. The `kaunas-deliveries` pack
sets it to true.

- **false**: the order stays in its depot's queue. Couriers skip it and take the
  oldest order they can deliver. It is dispatched once the fence closes.
- **true**: the order is turned away. This happens when the order is placed, when
  a fence opens over its address while it waits in the queue, or when its drone
  comes back (or crashes) without delivering it because a fence opened on the
  way. Each one is an `order_rejected` report naming the fence and the stage
  (`placed`, `queued` or `returned`). If the pack keeps a run log, the order also
  appears in `rejected-orders.csv`.

## Breaches

If a drone is in the air inside a fence's shape and band, that is a `breach`. One
report is made per fence entered, on the way in. The report gives:

- the fence;
- `how`: `flew in`, or `fence opened round it`;
- `flown_by`: `operator`, or `ground station`;
- the drone's flight mode, `sysid` and `callsign`, and its depot if it is a
  courier.

Drones flown by a ground station are not rerouted, so they breach any fence they
fly into. A drone flown by the simulation breaches a fence only if the fence opens
round it, or if it is walled in with no way round. In that case it flies straight
and the breach is logged.

## Scheduled and timed fences

A fence with `minutes` closes by itself that many simulated minutes after it opens.
Its `closes_tick` shows when. A fence with `after_minutes` waits as *scheduled*
until its time comes, then opens for its `minutes`. You can cancel a scheduled
fence, but you cannot extend one until it is open. Minutes are simulated time, so
they follow `tick_seconds` and not the wall clock. The shortest timed span is one
tick.

**Extend** gives an open fence a new closing time, `minutes` from now, or with -1
keeps it open until it is closed. It works on any open fence, including one that
was open until closed. The dashboard offers Extend only for fences that already
have a closing time.

**Rename** works on open, scheduled and previously closed fences. An empty name
clears it. A previous fence keeps its new name if it is reopened.

Closed fences are kept, newest first, as *previous* fences. Each has a
`closed_tick` and a `closed` reason: `expired`, `closed` or `cancelled`. A fence
removed by a ground station's plan counts as `closed`. At most 100 are kept. The
dashboard can reopen any of them.

All changes are made at the start of the next simulation step, before any drone
moves, so every drone sees the same airspace. A reset closes every fence,
forgets the scheduled and previous ones, starts the ids at 1 again and opens the
pack's fences again.

## Pack-defined fences

A pack lists the fences it starts with in `world/geofences.json`. Each entry takes
the shape, band and `name` fields above, plus `minutes` (counted from tick 0).
Pack fences open on the first tick, with source `scenario`. `after_minutes` is
not read for pack fences.

```json
[
  { "name": "Kaunas Castle and Old Town", "circle": { "lat": 54.8985, "lon": 23.8860, "radius_m": 350 } },
  { "name": "Ąžuolynas park", "polygon": [[23.9330, 54.9035], [23.9500, 54.9045], [23.9525, 54.8975], [23.9355, 54.8962]] },
  { "name": "Stadium, low level only", "circle": { "lat": 54.8905, "lon": 23.9142, "radius_m": 150 }, "ceiling_m": 60, "minutes": 120 }
]
```

The pack does not load if a fence is malformed, outside the world, or would keep
drones off a pad. The error names the entry by its index.

`kaunas-deliveries` also carries the UAS geographical zones that Oro navigacija
publishes over Kaunas. Each has a `"zone"` id, which the simulation ignores.
`uv run tools/lt_geozones.py` fetches the current zones and rewrites them, and
leaves fences without a `"zone"` alone. The script keeps only zones whose band
overlaps the couriers' cruise heights. It writes them as full-height fences,
without their band.

See [Scenarios](scenarios.md) for the rest of a pack's layout.

## The dashboard's Geofences tab

The **Geofences** tab draws the airspace on the [shared map](map.md). A fence is
drawn with its number and, if it has one, its altitude band in the form
`SFC–120 m` or `60 m–UNL`. A fence with a raised floor is lighter and dashed.

To open a fence:

1. Choose **Draw circle** and click the centre, then click the edge, or type the
   radius. Or choose **Draw polygon**, click each corner, and finish by clicking
   the first corner again, pressing Enter, or pressing **Open fence**.
   Backspace or Delete removes the last corner. Esc cancels.
2. Optionally give it a name, a number of minutes until it closes, and a floor and
   ceiling in metres AGL. Leave these blank for the surface and unlimited.
3. Press **Open fence**.

**Current fences** lists every fence that is open or waiting to open. Each row
shows:

- its number and name, with ✎ to rename it;
- its shape and area, and its altitude band;
- who opened it, when it opened (or when it will), and when it closes;
- its breaches.

Each row has **Close** (or **Cancel** for a scheduled fence). A fence due to
close also has **Extend**, which takes a number of minutes from now, or -1 to keep
it open until closed. Click a row to pick the fence out on the map. A fence of the
scenario's can be closed too. A reset opens it again.

**Previous fences** lists fences that have closed. Each can be reopened as it was
(shape, name and altitude band), either now or after some minutes (0 for now),
and either for some minutes or until closed (-1). A reopened fence gets a new
number.

**Recent activity** shows breaches and the operator's reroutes, newest first.

If the run is paused, a request is queued, and the model checks it when the
simulation next steps. If the model then refuses it, the page says why. See
[Dashboard](dashboard.md) for the rest of the dashboard.

## The REST API

Every endpoint is under `/api/v1` and needs the token. Changes take effect on the
next step. A successful change returns `{"id", "applied_tick", "status"}`. The
`status` is `applied`, or `queued` if the world is paused. A request the
endpoint can check for itself (a bad shape or band, an unknown id) is refused
straight away with an `application/problem+json` error. Anything the model
refuses later, such as a fence over a pad, appears in the `refused` list of
`GET /geofences`. See the [API guide](api-guide.md) and the endpoint reference at
`/docs/api`.

```sh
T="Authorization: Bearer $DASHBOARD_TOKEN"
J='content-type: application/json'

# List: open fences (with breach counts), scheduled, previous, recent breaches and reroutes
curl -s -H "$T" localhost:8080/api/v1/geofences

# Open a 300 m circle for 45 simulated minutes
curl -s -H "$T" -H "$J" -X POST localhost:8080/api/v1/geofences \
  -d '{"name": "Fire at the depot", "circle": {"lat": 54.9, "lon": 23.91, "radius_m": 300}, "minutes": 45}'

# Open a polygon from 60 m AGL up, in 10 minutes' time, until closed
curl -s -H "$T" -H "$J" -X POST localhost:8080/api/v1/geofences \
  -d '{"polygon": [[23.93, 54.903], [23.95, 54.904], [23.952, 54.897]], "floor_m": 60, "after_minutes": 10}'

# Keep fence 7 open for another 30 minutes from now (-1: until closed)
curl -s -H "$T" -H "$J" -X POST localhost:8080/api/v1/geofences/7/extend -d '{"minutes": 30}'

# Rename fence 7 (an empty name clears it)
curl -s -H "$T" -H "$J" -X PATCH localhost:8080/api/v1/geofences/7 -d '{"name": "Stadium"}'

# Close fence 7, or cancel it if it is still waiting to open
curl -s -H "$T" -X DELETE localhost:8080/api/v1/geofences/7
```

`GET /geofences` returns:

| Field | Meaning |
|---|---|
| `tick`, `tick_seconds`, `mode` | The run's tick, the simulated seconds per tick, and whether it is running. |
| `rev` | The airspace revision. It is bumped whenever a fence opens or closes. |
| `margin_m` | The horizontal margin. It is 0 for a model without geofences. |
| `fences` | The open fences, each with `breaches` (the number of breach reports among the recent reports, so it can undercount on a long run). |
| `scheduled` | Fences waiting to open, soonest first. Here `opened_tick` is when they will open. |
| `previous` | Closed fences, newest first, with `closed_tick` and `closed`. |
| `recent` | Up to 100 of the latest `breach` and `reroute` reports, newest first. |
| `refused` | Intents the model refused lately, with the reason. |

A published fence is the same fence serialised. It has `id`, `shape` (`circle`
or `polygon`), `lat_e7`/`lon_e7`/`radius_m` for a circle or `points` for a
polygon, `floor_m`, `ceiling_m`, `name`, `source`, `opened_tick` and
`closes_tick`. Absent fields mean "none". The published `points` are
`[lat_e7, lon_e7]` pairs in degrees × 10⁷. That is the other way round from the
`[lon, lat]` degrees a request takes. `GET /state/overlays` carries the same data
under `geofences`.

### Raw intents

The endpoints above are a front to `Custom` intents, which you can also send
through `POST /api/v1/intents` or the [WebSocket](protocols.md):

| Code | Payload | Does |
|---|---|---|
| 110 | The fence fields above, and optionally `source` | Opens a fence, or schedules one with `after_minutes`. |
| 111 | `{"id"}` | Closes an open fence or cancels a scheduled one. |
| 112 | `{"fences": [shape, ...]}` | Replaces the fences ground stations have set (the MAVLink gateway sends this). |
| 113 | `{"id", "minutes"}` | Changes how long an open fence stays open, from now (-1: until closed). |
| 114 | `{"id", "name"}` | Renames an open, scheduled or previous fence. |

## Ground stations and MAVLink

The [MAVLink gateway](protocols.md) serves the open fences as a fence plan
(`MAV_MISSION_TYPE_FENCE`): a `NAV_FENCE_CIRCLE_EXCLUSION` for each circle, and a
`NAV_FENCE_POLYGON_VERTEX_EXCLUSION` for each polygon corner. Scheduled fences are
not included, and neither are altitude bands. `MISSION_CURRENT.fence_id` is the
airspace revision, so it changes whenever a fence opens or closes.

A fence plan uploaded by a ground station becomes one `Custom` 112 intent, and
the fences ground stations set become exactly that plan:

- uploaded fences open with source `gcs`, from the surface, without a name, until
  closed;
- `gcs` fences missing from the plan close;
- fences in the plan that match one opened by the scenario, the dashboard or an
  intent are left alone (a station sends back what it downloaded);
- clearing the fence plan closes every `gcs` fence and nothing else;
- inclusion fences are not simulated, so a plan containing one is refused, as is
  a plan with a fence over a pad. One bad fence refuses the whole plan.

See [Connecting](connecting.md#3-fly-uav-4-on-a-mission) for how this looks in
QGroundControl. No geofences are sent over Cursor-on-Target. TAK clients see the
drones' tracks, not the fences.

## Reports and run logs

Everything about fences is a report, which you can read at `GET /api/v1/reports`:

| Kind | When | Detail |
|---|---|---|
| `geofence` | A fence opens, is scheduled, closes, is cancelled, extended or renamed. | The fence, with `action` (`open`, `schedule`, `close`, `cancel`, `extend`, `rename`) and `reason` (`scenario`, `requested`, `scheduled`, `expired`). Renaming a previous fence makes no report. |
| `breach` | A drone in the air enters a fence. | See [Breaches](#breaches). |
| `reroute` | The operator sends a drone messages to keep it clear. | `reason`, the `messages` sent, the number of `detours`, `airspace_rev`. |
| `order_rejected` | An order is turned away for a fenced address. | `order`, `address`, `label`, `depot`, `fence`, `fence_name`, `stage`, `reason`. |

A reroute's `reason` is one of:

- `route round a geofence`
- `way home round a geofence`
- `inside a geofence`
- `on the way round a geofence`
- `round the geofence, mission`
- `round the geofence, home`
- `delivery address geofenced`

A pack that keeps a run log (one that sets `heatmap_resolution` or
`order_rate_per_address_day`) writes these files in `var/logs/<scenario>-<UTC start>/`
once any fence has opened:

- `geofences.geojson`: every fence that opened, with when it opened and closed and
  how often it was breached, and every breach as a point, styled for geojson.io;
- `breaches.csv`: every breach;
- `reroutes.csv`: every set of messages the operator sent;
- `summary.json`, which counts them.

`simctl fly` writes the same folder. See [Operating](operating.md).
