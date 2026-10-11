# Stations and regions of interest

Launch and recovery stations are pads with chargers. Any drone the simulation
flies may land on one, charge there and take off again. Each station has a fixed
number of slots. Couriers use stations to reach addresses too far away for one
charge. A drone low on battery uses one when home is out of reach. Regions of
interest are circles that the same drones fly out to survey with their cameras
when they have nothing else to do. The model adds a region by itself wherever a
drone crashes. Stations change how far each depot can deliver, and the map draws
that reach. You add and remove stations and regions from the dashboard, the REST
API or intents, and a pack can start with its own. The code is in
[`stations.rs`](../crates/sim-drones/src/stations.rs) (stations, range and trip
planning), [`regions.rs`](../crates/sim-drones/src/regions.rs) (regions and
surveys), [`agents.rs`](../crates/sim-drones/src/agents.rs) (what drones do with
them) and [`world.rs`](../crates/sim-drones/src/world.rs) (intents, the overlays
and the range).

## Stations

A station has:

- an `id`;
- a `name` (optional for intents, required by the API and the dashboard; trimmed
  and cut to 80 characters);
- a position;
- a number of launch and recovery `slots`, from 1 to 100. This is required.
- a `source` (`scenario`, `dashboard` or `intent`);
- `added_tick`, the tick it was added.

Ids start at 1 and are never reused within a run.

A new station must be inside the world. It cannot go under a geofence that
[covers the ground](geofences.md#what-a-fence-covers), meaning one too low to fly
beneath. It cannot be within 20 m of another station. Each pad has a 10 m radius,
and two may not overlap.

**Stations only help when there is a charger.** The pack parameter
`charge_pct_s` (percent per second on the ground) defaults to 0, which means no
charging. With no charger, no trip is planned by way of a station, and the
delivery range is one charge from home. `kaunas-deliveries` sets `charge_pct_s =
0.0333`, which is about 50 minutes from empty to full.

### Slots

Slots limit how many drones can be down on a station, or coming down onto it, at
once. A drone the simulation flies takes a slot when it comes within 50 m of a
station it is landing on, and holds the slot until it has taken off and left. If
every slot is taken when it arrives, it holds off in Loiter there until one comes
free. The overlays mark such a drone as `waiting`.

One exception: if a drone's low-battery failsafe has fired, it has no charge left
to wait with, so it lands regardless of slots. Drones flown by a ground station
do not take slots or wait. Any drone on the ground on the pad still counts
against the station's slots.

### Trips by way of stations

Before a courier sets off, or a drone sets off on a survey, it works out whether
one charge will do the trip. The estimate covers:

- level flight at cruise airspeed along the straight line out and back;
- the climb to cruise height plus `dispatch_takeoff_m` and the descent, for each
  hop;
- the work at the far end: the winch cycle for a delivery, or the circuit for a
  survey.

The total is multiplied by 1.25 as a margin for detours. It must fit within the
charge the drone has, keeping `battery_reserve_pct` (default 15). Drain goes with
the square of airspeed, so a hop into the wind costs more than the same hop with
it. Deliveries, surveys, diversions and trips home plan for the wind in the
drone's cell at its cruise height; the published delivery range plans for the
wind at home, blowing the worst way.

If one charge will not do, and there is a usable station and a charger, the drone
plans the trip as hops between stations. It takes the shortest chain of hops out
to a station from which it can reach the target and get back to a station, then
the shortest chain home from there. A hop from home starts with the charge the
drone has. A hop from a station assumes a full charge. Each stop is three mission
items:

- a waypoint at cruise height;
- `NAV_LAND` on the station;
- `NAV_TAKEOFF`, with the charge to leave with in `param2`.

That charge is enough for the next hop plus the reserve, in whole percent. Putting
it in `param2` is a krabshell extension: the takeoff waits on the charger until
the battery holds that much.

What happens when no chain works depends on the trip. A courier with no station
that helps flies straight, as it would without stations. A survey that cannot be
reached is skipped. Stations under a fence that covers the ground are left out of
all of this.

### Low battery and coming home

When a drone the simulation flies drops to `battery_reserve_pct`, its failsafe
fires and sends it home. If it cannot reach home on what is left, it lands at the
nearest station it can reach. It prefers a station with a slot free when one is
in reach. This is a `station` report with `action` `divert`, the drone's
`battery_pct`, and the `station` and its `name`. If no station is in reach either,
it carries on home and may crash.

When a drone is on the ground at a station away from home, with no order or
survey in hand, it flies home. If home is more than one charge away, it goes by
way of other stations. Each takeoff waits on the charger for enough for the next
hop.

## Delivery range

The model works out how far each depot's couriers can deliver and get back,
using the same rules:

- the couriers leave home at `launch_battery_pct` (a full charge if that is 0);
- they winch at their cruise height;
- they plan for the wind at home at cruise height, blowing the worst way;
- they can stop at any usable station.

For each depot it takes the courier that reaches furthest. For a pair of stops
(stations or home) that a courier can get out to and back from, the places it can
deliver between them form an ellipse, with the two stops as its foci. A depot's
reach is the union of those ellipses.

The model works the range out again whenever:

- a station is added or removed;
- a geofence changes, since that changes which stations can be used;
- the wind changes.

Geofences are not subtracted from the area. An address inside the range may still
be fenced off.

The range is published in the overlays and in `GET /api/v1/stations` as `range`:

```json
{ "stations_rev": 3, "airspace_rev": 12,
  "hubs": [ { "depot": "north", "home": [549300000, 239100000],
              "direct_m": 5400, "reach_m": 11800,
              "areas": [ { "a": [549300000, 239100000], "b": [549500000, 238700000], "sum_m": 10800 } ] } ] }
```

| Field | Meaning |
|---|---|
| `direct_m` | The furthest from home on one charge, there and back. |
| `reach_m` | The furthest from home by way of stations. Null means anywhere: the battery never runs down (`battery_drain_pct_s = 0`). |
| `areas` | The ellipses: foci `a` and `b` as `[lat_e7, lon_e7]`, and `sum_m`, the most the distances from a point to the two foci may add up to. |

The maps draw the range as the **Delivery range** layer. The **Stations &
regions** tab lists each depot's reach. [`krab-drones`](development.md) prints it in
its summary.

## Regions of interest

A region of interest is a circle with:

- an `id`;
- a `name`;
- a centre and a `radius_m`, from 10 to 5000 m;
- a `source` (`scenario`, `dashboard`, `intent`, or `crash`);
- `added_tick`, the tick it was added.

The model keeps a count of `surveys` done, `last_surveyed_tick`, and the drones
`assigned` to survey it now.

### Surveys

A drone the simulation flies takes up a survey when it is free for work. That
means it is on its home pad, charged to `launch_battery_pct`, and the wind is
within its launch limits. A courier must also have no order it can take, so
deliveries come first.

The drone picks the region most overdue a look that it can reach:

- regions never surveyed come first, oldest first;
- then the region surveyed longest ago, once `roi_revisit_min` simulated minutes
  have passed since its last survey (default 30).

Only one drone at a time goes to each region.

The drone flies out at its cruise height (a courier's `cruise_agl_m`, otherwise
`survey_agl_m`), by way of stations if it must. It flies a circle of eight legs
inside the region at `survey_agl_m` (default 60 m, clamped to between 5 and
500 m), with the camera looking. The circle's radius is 0.6 of the region's,
and at least 10 m. When the circle closes, the drone makes a `survey` report and
the region's count goes up. Then it flies home. A drone that comes back without
closing the circle frees the region for another drone.

Detections made on the way are ordinary `detection` reports. See
[Drones](drones.md) for the camera.

### Crash sites

When any drone crashes, the model adds a region of `crash_roi_radius_m` where it
fell. The default is 150 m, clamped to 5000 m. Set it to 0 for none. The region
is named "Crash of" and the drone's callsign, has source `crash`, and is reported
as a `roi` report with `action` `crash`. The crash report itself carries the
region's id as `roi`. A crash region is surveyed like any other, and is removed
like any other.

## Pack-defined stations and regions

A pack lists the stations and regions it starts with in `world/stations.json` and
`world/regions.json`. They are added on the first tick with source `scenario`, and
a reset adds them again after clearing everything added since. `slots` is required
for each station, as it is for any other.

```json
[ { "name": "Šilainiai", "lat": 54.93, "lon": 23.85, "slots": 4 } ]
```

```json
[ { "name": "Nemunas island", "lat": 54.895, "lon": 23.905, "radius_m": 400 } ]
```

An entry that is malformed or outside the world stops the pack loading. See
[Scenarios](scenarios.md) for the rest of a pack's layout. `iron-wolf` ships
`world/stations.json`. No pack in `scenarios/` ships `world/regions.json`; the
`world/regions.js` in `iron-wolf` is not read, as only `.json` files are.

## The dashboard's Stations & regions tab

The tab draws everything on the [shared map](map.md). A station is drawn as an
APP-6 ground installation. When you zoom in, or pick it out, it is labelled with
its name and its slots in use, for example `Šilainiai · 1/4 slots`. A region is drawn
as a named area of interest (NAI). It is heavier and blue while drones are tasked
to survey it, and red if a drone crashed there.

- **Add station**: click where it goes, then give its name and its slots (1 to
  100), and press **Add**.
- **Add region**: click the centre, then click the edge (or type the radius, 10 to
  5000 m), then name it and press **Add**.
- Esc cancels either.

**Launch and recovery stations** lists each station with:

- its position;
- who added it, and when;
- its slots in use, out of its slots;
- the drones holding off it.

Each row has **Remove**. Under the list, **Delivery range** gives each depot's
reach, and how far one charge goes on its own.

**Regions of interest** lists each region with:

- its radius;
- who added it (crash sites are tagged), and when;
- the surveys done, and the last one;
- the drones tasked to survey it.

Each row has **Remove**.

**Recent activity** shows, newest first:

- stations added and removed;
- drones diverting to a station on low battery, with the charge left;
- regions designated, removed or added at a crash;
- surveys done.

Click a row to pick out its station or region on the map. You can remove any
station or region, the scenario's included. A reset puts the scenario's back.
Changes made while the run is paused are queued, and the page says if the model
then refuses one.

## The REST API

Every endpoint is under `/api/v1` and needs the token. Additions and removals take
effect on the next step. A successful request returns `{"id", "applied_tick",
"status"}`, where `status` is `applied`, or `queued` if the world is paused. A
request the endpoint can check for itself is refused straight away with an
`application/problem+json` error. That covers a missing name or slots, a radius
out of range, or an id the model does not list (404). A request the model refuses
later, such as a station under a fence or too near another, appears in the
`refused` list. See the [API guide](api-guide.md) and the endpoint reference at
`/docs/api`.

```sh
T="Authorization: Bearer $DASHBOARD_TOKEN"
J='content-type: application/json'

# Stations, with slots in use, the delivery range and recent station reports
curl -s -H "$T" localhost:8080/api/v1/stations

# Add a station with four slots
curl -s -H "$T" -H "$J" -X POST localhost:8080/api/v1/stations \
  -d '{"name": "Šilainiai", "lat": 54.93, "lon": 23.85, "slots": 4}'

# Remove station 2
curl -s -H "$T" -X DELETE localhost:8080/api/v1/stations/2

# Regions of interest, with recent region and survey reports
curl -s -H "$T" localhost:8080/api/v1/rois

# Designate a 400 m region
curl -s -H "$T" -H "$J" -X POST localhost:8080/api/v1/rois \
  -d '{"name": "Nemunas island", "lat": 54.895, "lon": 23.905, "radius_m": 400}'

# Remove region 5 (a crash site's included)
curl -s -H "$T" -X DELETE localhost:8080/api/v1/rois/5
```

`GET /stations` returns:

| Field | Meaning |
|---|---|
| `tick`, `tick_seconds`, `mode` | Where the run is. |
| `available` | False for a model without stations. |
| `stations` | Each station, with `drones` (on the ground at it now), `occupied` (slots taken) and `waiting` (drones holding off for one). |
| `range` | The [delivery range](#delivery-range). |
| `recent` | Up to 100 of the latest `station` reports. |
| `refused` | Intents the model refused lately. |

`GET /rois` returns the same, with `rois` in place of `stations` and no `range`.
Its `recent` holds `roi` and `survey` reports.

Positions in the lists are `lat_e7`/`lon_e7` (degrees × 10⁷). Requests take `lat`
and `lon` in degrees. `GET /state/overlays` carries the same lists as `stations`,
`rois` and `range`.

### Raw intents

The endpoints are a front to `Custom` intents, which you can also send through
`POST /api/v1/intents` or the [WebSocket](protocols.md):

| Code | Payload | Does |
|---|---|---|
| 120 | `{"lat", "lon", "slots"}`, and optionally `name` and `source` | Adds a station. |
| 121 | `{"id"}` | Removes a station. |
| 130 | `{"lat", "lon", "radius_m"}`, and optionally `name` and `source` | Designates a region of interest. |
| 131 | `{"id"}` | Removes a region of interest. |

The REST endpoints always send `source` as `dashboard`. A raw intent can name its
own source: up to 24 letters, digits, `-` or `_`, but not `scenario`, `gcs` or
`crash`. Without one, the source is `intent`.

## Reports

| Kind | When | Detail |
|---|---|---|
| `station` | A station is added or removed. | The station, with `action` `add` or `remove`. |
| `station` | A drone low on battery diverts to a station. | `action` `divert`, `battery_pct`, `station`, `name`, and the drone's `sysid`, `callsign` and, for a courier, `depot`. |
| `roi` | A region is designated, removed or left at a crash. | The region, with `action` `add`, `remove` or `crash`. |
| `survey` | A drone closes its circle over a region. | `roi`, `name`, `duration_s` (from setting off), and the drone's `sysid`, `callsign` and depot. |

Read them at `GET /api/v1/reports`. The run log has no files for stations or
surveys. Landings and crashes at stations appear in `landings.csv` like any
other. Neither stations nor regions are sent over MAVLink or Cursor-on-Target.
