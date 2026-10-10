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

The drones model is a krABMaga model: every drone, person, vehicle, intruder
and point of interest is a krABMaga agent with its own `step()`, on a krABMaga
schedule, against a krABMaga `State` whose occupancy is a `SparseA5Grid`. It
lives in its own crate, `sim-drones`, and runs without krabshell too; see
[Without krabshell](#without-krabshell).

The design is in [`krabshell_spec.md`](krabshell_spec.md). How the code
implements it, and what was learned doing so, is in
[`docs/architecture.md`](docs/architecture.md). Speed numbers are in
[`docs/baseline.md`](docs/baseline.md).
How to author and upload drone behaviour trees is in
[`docs/drone-behaviours.md`](docs/drone-behaviours.md).

## Quick start

```sh
cargo run --release -p sim-axum --bin krabshell -- --scenario london-patrol --run
```

Open <http://127.0.0.1:8080/>. With no `DASHBOARD_TOKEN` set the server accepts
any token and refuses to bind anything but loopback. The MAVLink gateway and the
CoT emitter start with it; see [Drones: MAVLink and CoT](#drones-mavlink-and-cot).

Every build links krABMaga with its `gis` feature, which pulls in Bevy, so the
first build is slow and needs Bevy's system libraries on Linux (`libasound2-dev
libudev-dev libwayland-dev libfontconfig1-dev libxkbcommon-dev pkg-config`).

The random-walker packs used for benchmarks live in `bench/scenarios`. They run
on the built-in grid (`walkers`) or on krABMaga's (`walkers-krabmaga`):

```sh
cargo run --release -p sim-axum --bin krabshell -- \
  --scenarios bench/scenarios --scenario london-walkers --model walkers-krabmaga
```

## Layout

```
crates/
  sim-core/       types, A5 cell maths, pack reading — no Bevy, Axum or krABMaga
  sim-drones/     the drones model as krABMaga agents and State; krab-drones
  sim-engine/     driver thread, Model seam, walkers, scenario store, checkpoints, simctl
  sim-mavlink/    MAVLink gateway: drones as UDP vehicles for ground stations
  sim-cot/        Cursor-on-Target emitter: tracks and sightings for TAK
  sim-weather/    wind per A5 cell from Open-Meteo, recorded or live, cached on disk
  sim-conflict/   conflict detection and resolution advisories on A5 cells
  sim-axum/       REST + WebSocket edge, auth, the krabshell binary
  sim-dashboard/  Askama templates and static assets (htmx, milsymbol, canvas map)
scenarios/        drone packs: london-patrol (gold), kaunas-deliveries (gold),
                  swarm-survey (fast)
bench/scenarios/  random-walker packs for benchmarks and the krABMaga replay
docs/             architecture notes, behaviour-tree guide and baseline numbers
```

## Features

| Cargo feature | Effect |
|---|---|
| *(none)* | krABMaga with `gis`: the drones model and `walkers-krabmaga`. |
| `parallel` | krABMaga's `parallel` feature: its thread-safe grid and schedule. The drones schedule still runs on one thread, so gold hashes match the default build. |

Enable on the binary with `-p sim-axum --features parallel`, on `simctl` with
`-p sim-engine --features parallel`, or on `krab-drones` with
`-p sim-drones --features parallel`.

## Operating

### Server

```
krabshell [--bind 127.0.0.1:8080] [--scenarios scenarios] [--scenario ID]
          [--model KIND] [--run] [--checkpoint-on-exit PATH]
          [--mavlink-bind 127.0.0.1:14551] [--mavlink-gcs 127.0.0.1:14550,...]
          [--mavlink-hz 5] [--no-mavlink]
          [--cot-dest 239.2.3.1:6969] [--cot-ttl 1] [--cot-iface IP] [--no-cot]
          [--log-dir var/logs] [--log-every 60] [--no-logs]
```

| Env | Meaning |
|---|---|
| `DASHBOARD_TOKEN` | Shared secret for the API and dashboard. Required off loopback. |
| `KRABSHELL_BIND`, `KRABSHELL_SCENARIOS`, `KRABSHELL_SCENARIO` | Same as the flags. |
| `KRABSHELL_MAVLINK_BIND`, `KRABSHELL_MAVLINK_GCS`, `KRABSHELL_COT_DEST`, `KRABSHELL_COT_IFACE`, `KRABSHELL_LOG_DIR` | Same as the flags. |

Ctrl-C pauses the driver, optionally writes a checkpoint, then joins the
thread.

### Deliveries and run logs

A drones pack can carry an address book (see below). Every address then
orders at random, rarely and independently (`order_rate_per_address_day`), and
each order joins a first-come first-served queue at the depot that serves the
address. Drones with a `depot` are its couriers: the next one that is on its
home pad, disarmed, loaded (`dispatch_load_s` after landing) and charged to
`launch_battery_pct` is given the oldest order as a fresh mission: take off
(`dispatch_takeoff_m` above the pad), fly to the address at its `cruise_agl_m`
above the terrain, lower the parcel with `DO_WINCH` / `WINCH_DELIVER`, fly home,
descend to `dispatch_approach_m` and land. The order is delivered when the
winch completes, and a `delivery` report says when it was placed, dispatched
and delivered. A courier that comes home without delivering (low battery, a
ground station took it) puts its order back at the front of the queue. Orders
come from the run's seed, so `gold` runs place and deliver the same orders.

An order whose address is inside a geofence waits in the queue until the
fence closes. A pack that sets `reject_orders_in_geofences = true` turns such
orders away instead: as they are placed, the moment a new fence opens over
an order still waiting, and, for an order a drone was carrying when a fence
opened over its address, when the drone comes back without it. An
`order_rejected` report gives the order, the address, the `stage` (placed,
queued, returned) and the reason ("the delivery address is inside geofence
#3 (Žalgiris Arena), which opened while the order waited"), and the run log
lists them in `rejected-orders.csv`. `kaunas-deliveries` does.

A drone whose battery reaches 0 % in the air crashes where it is: it leaves
the world, a `crash` report is logged, and the order it carried goes back to
the front of the queue. A crashed courier's duty goes to its hub's spare (a
drone with `"spare": "<depot>"`), and once the spares are used, to a new drone
on the wreck's pad. New drones get an agent id and MAVLink system id nothing
has had before. Every touchdown after a flight is a `landing` report with the
battery left.

### Geofences

Geofences are keep-out zones: circles and polygons, from the surface up
without limit unless they have an altitude band (`floor_m`, `ceiling_m`,
metres above ground level, or above sea level with `"altitude_ref":
"amsl"`, taken at the fence's centre). A drone is inside a fence only
within its band. A leg a fence blocks is also tried beneath it, at the
floor less `geofence_vertical_margin_m` (default 10) and no lower than
`beneath_min_agl_m` (default 20), following the terrain, and flown that way
when it is shorter counting the drop and climb back; a delivery address
under such a fence is winched to from down there. A fence that reaches too
low to fly beneath keeps orders from its addresses as before. A
pack lists the ones it starts with in `world/geofences.json`, each with an
optional `name` and `minutes` (until it closes by itself):

```json
[
  { "name": "Kaunas Castle and Old Town", "circle": { "lat": 54.8985, "lon": 23.8860, "radius_m": 350 } },
  { "name": "Ąžuolynas park", "polygon": [[23.9330, 54.9035], [23.9500, 54.9045], [23.9525, 54.8975], [23.9355, 54.8962]] }
]
```

`kaunas-deliveries` also carries the UAS geographical zones Oro navigacija
publishes over Kaunas (each with a `"zone"` id, which the simulation
ignores); `uv run tools/lt_geozones.py` fetches the current zones and
rewrites them, leaving fences without a `"zone"` alone.

A fence over a drone's pad, or outside the world, stops the pack loading. A
reset opens the pack's fences again. While the run goes on, the dashboard's
Geofences tab, the API and intents open and close fences: `Custom` 110 with
`{"circle": {"lat", "lon", "radius_m"}}` or `{"polygon": [[lon, lat],
...]}` and optionally `"name"`, `"minutes"` (-1: until closed),
`"after_minutes"` (it waits to open until then), `"floor_m"` and
`"ceiling_m"` (numbers of 0 or more with the ceiling above the floor; null
or absent: the surface, unlimited); `Custom` 111 with `{"id"}`
closes one, or cancels one still waiting; `Custom` 113 with `{"id",
"minutes"}` changes how long an open one stays open (from now; -1: until
closed); `Custom` 114 with `{"id", "name"}` renames one.
A ground station's fence plan (see below) arrives as `Custom` 112 and
replaces the fences ground stations set before.

The autopilot knows nothing of fences. The operator, who flies the drones the
simulation flies itself (couriers, and drones a pack starts on a mission),
keeps them `geofence_margin_m` clear the way a ground station would, by
MAVLink: it re-uploads the mission with detour waypoints round each fence
(`MISSION_ITEM_INT`, `DO_JUMP`s renumbered, `MISSION_SET_CURRENT`), turns a
return home into a mission round the fences, moves a drone a fence opened
round out of it with `DO_REPOSITION` and then resumes, and sends a courier
whose address is fenced home (its order waits for the fence to close). When a
fence closes, its detours come out again. Drones flown by a ground station
are left to it. A drone inside a fence in the air is a `breach` report, once
per fence it enters; opens, closes and reroutes are reports too
(`geofence`, `reroute`). Ground stations can download the fences as a fence
plan (see below).

A pack that sets `heatmap_resolution` or `order_rate_per_address_day` gets a
run log: krabshell writes a folder `var/logs/<scenario>-<UTC start>/` every
`--log-every` seconds and on shutdown, and a reset, a restored checkpoint or
another scenario starts a new one. It holds:

| File | What |
|---|---|
| `heatmap.geojson` | Drone-seconds of flight per A5 cell of `heatmap_resolution`, as a styled choropleth (log-scale YlOrRd classes in simplestyle `fill`/`stroke`, legend in the collection's `properties`). geojson.io shows it as it is; QGIS can style on `class`. Time parked or disarmed is not flight. |
| `deliveries.csv` | Every delivered order: depot, drone, address, ticks, and seconds waiting for a drone, flying, and in total. |
| `rejected-orders.csv` | Every order turned away when it was placed, with the geofence its address was inside and the reason. |
| `delivery-times.csv`, `delivery-times.svg` | Histogram of time from order to delivery, in whole-minute bins, with mean, median, 90th percentile and slowest. |
| `landings.csv` | The end of every flight: drone, landed or crashed, and the battery percentage it touched down with (-1 for a crash), with what replaced a crashed drone. |
| `landing-battery.csv`, `landing-battery.svg` | Histogram of battery at touchdown in 5 % bins, one sample per flight, crashes in a bin of their own at -1. |
| `geofences.geojson` | Every geofence that opened, with when it opened and closed and how often it was breached, and every breach as a point. Styled for geojson.io. |
| `breaches.csv` | Every breach: drone, fence, whether it flew in or the fence opened round it, and who was flying it. |
| `reroutes.csv` | Every set of MAVLink messages the operator sent to take a drone round a fence, and why. |
| `summary.json` | Totals (flights and crashes among them, geofences, breaches, reroutes), and any reports the logger missed. |

For a long run without the server, `simctl fly` flies the pack for some
simulated hours as fast as it can and writes the same folder
(`var/logs/<scenario>-<hours>h/`):

```sh
cargo run --release -p sim-engine --bin simctl -- fly kaunas-deliveries --hours 8
```

The dashboard's **Geofences** tab manages them. **Current fences** lists
every fence open or waiting to open (shape and area, altitude band, who
opened it, when it opened or will, when it closes, breaches), each with
Close (or Cancel), and
Extend for one due to close: a number of minutes from now, or -1 to keep it
open until closed. The ✎ beside a fence's name renames it, current or
previous. Draw
a circle (centre, then edge) or a polygon (corners, then click the first
again or press Enter) on the map to open one, with a name and optionally a
number of minutes and a floor and ceiling in metres above ground level
(blank: the surface, unlimited). **Previous fences** lists those that have
closed: reopen any as it was (altitude band included), now or after some
minutes (0 for now), for some minutes or
until closed (-1). It opens with a new number. **Recent activity** shows
breaches and the operator's reroutes. The State page's map draws the fences
too. Both maps have layers to show or hide: drones; delivery locations
(hubs and landmarks, open orders' addresses, recent drop-offs); people and
vehicles (where they are, and camera sightings); geofences; launch and
recovery stations; regions of interest; and the A5 occupancy grid. The
choice is remembered in the browser.

### Behaviour trees

What every drone does each tick is decided by one behaviour tree
([bonsai-bt](https://docs.rs/bonsai-bt)), the same for every drone, read
from `crates/sim-drones/behaviour/drone.json`, or from a pack's own
`world/behaviour.json`. The tree is JSON in bonsai's own format, with a
`description` beside the `root`:

```json
{ "description": "...",
  "root": { "Sequence": [
    { "Action": "ObeyCommands" },
    { "Action": "Fly" },
    { "AlwaysSucceed": { "Sequence": [ { "Action": "ReadyForWork" },
      { "Select": [ { "Action": "TakeOrder" }, { "Action": "Survey" } ] } ] } }
  ] } }
```

Like a [TinyBrains](https://tinybrains.dev) brain, it is small, shared, and
evaluated afresh every tick: its leaves are the drone's senses (`Airborne`,
`IsCourier`, `FlownBySimulation`, `BackFromTrip`, `ReadyForWork`,
`SensorDue`, `AtStation`, `LowBattery`), which succeed or fail, and its acts
(`AvoidGeofences`, `ObeyCommands`, `Fly`, `FinishTrip`, `TakeOrder`,
`Survey`, `DivertToStation`, `ReturnToBase`, `LogBreaches`, `Scan`); the
branches are bonsai's `Sequence`, `Select`, `If`, `Invert`,
`AlwaysSucceed`, `WhenAll`, `WhenAny`, `Race` and `After`. What a drone is
in the middle of lives in its state, not the tree, so checkpoints restore
exactly; nodes that wait across ticks (`Wait`, `WaitForever`, `Timeout`,
`While`, `WhileAll`) are refused, as is a tree without exactly one `Fly`.
Each node is documented in `crates/sim-drones/src/behaviour.rs`. The
default tree is the drones' behaviour before trees (gold runs replayed
unchanged), with stations and surveys added. Each drone publishes what it
is doing as `activity` (`delivering`, `surveying`, `returning`,
`charging`, `flying`, `idle`).

### Stations and regions of interest

Launch and recovery stations are where drones take off, land and charge;
regions of interest are circles for the drones to survey, and the model
adds one by itself where a drone crashes. A model that has them publishes
both in its overlays (`stations`, with the drones on the ground at each;
`rois`, with the surveys done, when last, and the drones `assigned` to
survey each), and reports what happens to them (`station`, `roi` and
`survey` reports). Intents add and remove them: `Custom` 120 with `{"lat",
"lon", "name", "source"}` adds a station and `Custom` 121 with `{"id"}`
removes one; `Custom` 130 with `{"lat", "lon", "radius_m", "name",
"source"}` (10 to 5000 m) designates a region and `Custom` 131 with
`{"id"}` removes one. A station has a number of launch and recovery
`slots` (1 to 100, required): a drone coming in to land takes one as it
comes within 50 m and keeps it until it takes off again, and one that finds
them all taken holds off there until one is free (`waiting` in the
overlays) — unless its low-battery failsafe has fired, when it lands
regardless. A drone diverting on a low battery picks a station with a slot
free if it can reach one. A pack lists those it starts with in
`world/stations.json` (`[{"name", "lat", "lon", "slots"}]`) and
`world/regions.json` (`[{"name", "lat", "lon", "radius_m"}]`). A station
cannot go under a geofence too low to fly beneath.

A courier whose trip will not go on one charge (straight-line flight at
cruise speed, its climbs, and the winch, with a 25 % margin, from what it
has above `battery_reserve_pct`) plans it as hops between stations: the
shortest chain out to one from which the address and back to a station is
in reach, and home from there. At each stop it lands, charges until it has
enough for the next hop and the reserve (its `NAV_TAKEOFF` carries that
charge in `param2`, a krabshell extension: the takeoff waits on a charger
until the battery holds it), and takes off. With no station that helps it
flies straight, as before. A drone the simulation flies whose low-battery
failsafe fires out of reach of home diverts to the nearest station it can
reach (a `station` report with `action` `divert`), and once down there
flies home, by way of other stations if need be, when charged. Stations
only help when there is a charger (`charge_pct_s`).

The model works out how far each depot's couriers can deliver, with the
same rules: leaving home at `launch_battery_pct` (full if 0), winching at
their cruise height, by way of whatever stations they could use. For a
pair of stations (or home) a courier can get out to and back from, the
places it can deliver between them form an ellipse with the two as its
foci, so a depot's reach is a union of ellipses. It is worked out again
whenever a station is added or removed, or a geofence changes which
stations can be used, and published in the overlays as `range` (each
depot's `areas`, its furthest `reach_m` and its one-charge `direct_m`).
Geofences are not taken out of it. The maps draw it as the **Delivery
range** layer, the Stations & regions tab lists each depot's reach, and
`krab-drones` prints it.

A drone the simulation flies that is free for work (on its pad, charged to
the launch level, and for a courier, with no order it can take) surveys
the region of interest most overdue a look that it can reach: never
surveyed first, oldest first, then the one surveyed longest ago, once
`roi_revisit_min` (default 30) have passed, one drone at a time. It flies
there (by way of stations if it must), circles inside it at
`survey_agl_m` (default 60) with the camera on, and comes home; a `survey`
report marks the circle closed. A drone that crashes leaves a region of
`crash_roi_radius_m` (default 150; 0 for none) where it fell, named "Crash
of" its callsign.

The dashboard's **Stations & regions** tab manages them. Add a station
(click where it goes, then name it and give its slots) or a region (centre,
then edge, or type the radius, then name it) on the map. **Launch and
recovery stations** lists each with its position, who added it and when,
its slots in use, and the drones holding off it, with Remove, and each
depot's delivery reach. **Regions of interest** lists each with its radius, who
added it (crash sites are marked), when, the surveys done and the last, and
the drones on their way to survey it, with Remove. **Recent activity** shows
stations added and removed, regions designated, removed or added at a
crash, and surveys done. A model that publishes neither leaves the page
empty.

### Weather

The drones fly in wind, one series per A5 cell. A drone looks up its wind
from the cell it is in, through that cell's ancestor at the *weather
resolution* (12 by default, about 1.4 km, near the grid of the weather
models), at its height above the ground. In wind:

- **Flight.** It holds a straight track by crabbing into the crosswind; its
  ground speed is what the wind leaves of its airspeed, slower into a
  headwind and faster with one. A wind stronger than it can fly blows it
  back, nose into the wind. Hovering costs airspeed but not position. People,
  vehicles and scripted intruders are not affected.
- **Battery.** Drain goes with the square of airspeed (through the air,
  climb included), scaled so `battery_drain_pct_s` is the drain at
  `cruise_ms`. Hovering in still air costs nothing unless `hover_drain_pct`
  gives it a floor; hovering into a wind costs what flying at that speed
  does. This applies with or without weather.
- **Planning.** Couriers and surveys plan each leg for the wind in their cell
  at cruise height: a headwind leg costs more than the same leg downwind, and
  a trip that would not make it with the wind as it is goes by way of
  stations instead. The delivery range shown on the map shrinks to what holds
  in the worst direction.
- **Launching.** Drones the simulation flies stay on the ground while the
  wind at takeoff height is over `max_wind_ms` (default 12) or gusts over
  `max_gust_ms` (default 15); a ground station's takeoff command is refused
  likewise. 0 is no limit.
- **Telemetry.** MAVLink `VFR_HUD` reports airspeed and ground speed apart,
  and the heading is where the nose points, not the track.

A pack asks for weather in `params.toml`:

```toml
weather = "recorded"                        # "live", or "off" (the default)
weather_start_utc = "2026-03-14T09:00:00Z"  # recorded: the UTC time of tick 0
weather_hours = 24                          # how much to fetch, 1 to 384
weather_resolution = 12                     # A5 resolution of the weather cells
weather_model = "best_match"                # an Open-Meteo model token
# weather_bbox = [23.79, 54.83, 24.02, 55.0]  # min lon, min lat, max lon, max lat
# weather_margin_km = 3                     # round the agents, without terrain or a box
# weather_refresh_min = 60                  # live: how often to fetch again
```

- **`recorded`** fixes tick 0 at `weather_start_utc` and fetches the wind for
  that window once, from Open-Meteo's forecast API (which holds about three
  months back and 16 days ahead) or, for older starts, its historical
  forecast archive. A recorded run is reproducible: the same scenario flies
  in the same air every time, and if the weather cannot be fetched the load
  is refused (`weather_unavailable`) rather than flown in still air.
- **`live`** fixes tick 0 at the moment of loading and fetches the latest
  forecast from the hour before. The server fetches again every
  `weather_refresh_min` and hands the new field to the world as a
  `WEATHER_SET` intent, which the gold intent log records, so a live run
  replays too. If a fetch fails, the run flies in still air and tries again
  every minute.

Weather is fetched for every weather cell over the pack's terrain, or else
round its agents, sites and addresses with `weather_margin_km`, or
`weather_bbox`; at 10, 80, 120 and 180 m, with gusts at 10 m, hourly. Between
hours the wind is linear in time, between heights linear in height, and below
10 m it falls off toward the ground. Kaunas is 169 cells: two requests of
up to 100 locations, which Open-Meteo counts as 169 calls.

**Caching.** Fetched fields go to `var/weather-cache/` (`--weather-cache`),
keyed by what was asked. Running a scenario again does not call Open-Meteo
again; a window wholly in the past is kept for good, and one reaching into
the future (a forecast) is fetched again once the hour has turned.

**Checkpoints** carry the wind the world was flying in, so a restore flies in
the same air. A pack can also ship its own field as `world/weather.json`
(the `WindField` shape in `sim-core`) and need no fetch at all.

| Flag | Default | Meaning |
|---|---|---|
| `--weather-cache` | `var/weather-cache` | Where fetched weather is kept |
| `--no-weather` | off | Never fetch: recorded packs refuse to load, live ones fly in still air |
| `--open-meteo-key` | none | A commercial Open-Meteo key (`OPEN_METEO_API_KEY`) |
| `--open-meteo-url`, `--open-meteo-historical-url` | Open-Meteo's | A self-hosted Open-Meteo |

**Licence.** Open-Meteo's free API is for non-commercial use, at up to
10 000 calls a day (each location counts). Commercial use needs a key. The
data is CC BY 4.0: the Weather screen carries the credit, "Weather data by
Open-Meteo.com".

The dashboard's **Weather** screen shows the status, a map of the wind by
cell at any height and hour, and answers what the wind is at a point or a
cell: now at each height, and through the window.

### Conflict advisories

`sim-conflict` predicts where aircraft could be and advises how to keep them
apart. The dashboard's **Conflicts** screen and `GET /api/v1/conflicts` run
it on the drones and intruders in the air (drones on the ground or charging
are left out), with the run's wind, for the minute ahead: a heatmap of each
A5 cell at its worst moment (by the chance of two or more aircraft there
together, or the summed probability), the requests for conflict resolution,
and the ten cells most aircraft are expected in, with the aircraft
contributing to each. It is worked out at most once a tick, off the tick
thread, and only while someone is looking. The advisories are shown, not
flown.

For each aircraft state (position, altitude, true airspeed, heading,
vertical speed, turn rate, mass and a performance model), it flies a seeded
cloud of particles forward under an energy model. Fixed-wing aircraft use
BADA 3's Total Energy Model with energy share factors for constant-CAS and
constant-Mach climbs; multirotors use an excess-power envelope. The model is
scaled to the climb or descent the aircraft is seen flying. At each step
(5 s for a minute for drones, 15 s for five minutes for airliners) every
particle lands in an A5 cell and an altitude layer, with a protected zone of
one ring of neighbouring cells. The share of particles covering a cell is the
probability the aircraft is there.

Each cell's probabilities are summed over all aircraft at each step. Where
two or more aircraft reach the threshold (1 by default, the expected number
of aircraft), the cell is in conflict. Advisories come from trying climb,
descend, level off, turn and speed changes for the aircraft involved,
predicting again, and advising whatever removes the most risk; vertical
manoeuvres are preferred. A second aircraft is advised knowing the first's
advisory, so the two come out complementary.

Altitude layers are the airspace's: 1 000 ft deep below FL290 and 2 000 ft
from FL290 up, placed so no flight level sits on a boundary. Below FL290 each
layer is its level ±500 ft (FL100's is 9 500 to 10 500 ft); from FL290 each
runs from 500 ft below its level to 1 500 ft above (FL330's is 32 500 to
34 500 ft), since centring FL290's would overlap FL280's. Aircraft at
properly separated levels are therefore in different layers. The drone
preset uses 30 m layers instead.

No BADA or OpenAP aircraft data is included (BADA's is licensed by
EUROCONTROL; OpenAP's is LGPL). `FixedWing`'s fields are named as OpenAP
names them, to be filled from either source; the generic jet and quadcopter
are illustrative.

```sh
cargo run --release -p sim-conflict --example head_on
```

### Map symbols

The maps draw NATO joint military symbology (drones and intruders as the
2525 letter SIDCs of the CoT types TAK gets for them, so the dashboard and
TAK agree; the
rest as APP-6 D / MIL-STD-2525D 20-digit SIDCs, which have the civilian
individuals and vehicles the letter codes lack), with
[milsymbol](https://github.com/spatialillusions/milsymbol) 3.0.4 (MIT),
which is compiled into the binary so the dashboard needs no internet. Symbols
stay upright, as the standard has them; a moving drone's heading is its
direction-of-movement arrow.

| What | Symbol | SIDC |
|---|---|---|
| Delivery drone | friendly air, military rotary-wing UAV (CoT `a-f-A-M-H-Q`), with its direction of movement | `SFAPMHQ--------` |
| Intruder drone, and camera sightings of one | unknown air, military rotary-wing UAV (CoT `a-u-A-M-H-Q`) | `SUAPMHQ--------` |
| Camera sighting of a person | unknown land civilian unit, individual | `10011100001103000000` |
| Camera sighting of a car / van / pickup / bus / lorry | unknown land equipment, civilian automobile / utility vehicle / open-bed truck / multiple passenger vehicle / tractor-trailer | `100115000016` + `0100` / `0400` / `0200` / `0300` / `0600` + `0000` |
| Launch and recovery station | friendly ground installation, air base | `SFGPIBA---H----` |

Civilians are drawn in the unknown identity's yellow rather than APP-6's
optional civilian purple, since who they are is not known. Every vehicle has
a type (`car`, `van`, `pickup`, `bus`, `lorry`), drawn from the seed unless
`world/agents.json` gives one as `"type"`, and its sightings carry it as
`vehicle_type`. The CoT emitter sends TAK the types it always has: drones
`a-f-A-M-H-Q` and intruders `a-u-A-M-H-Q`, the same symbols as the
dashboard's; person and vehicle sightings `a-u-G-U-C-I`
and `a-u-G-E-V`, which CoT has no civilian individual for.

Sightings fade over 30 seconds. Where people and vehicles really are (not
what the cameras saw) stays as faint dots, and points of interest as
diamonds. A region of interest is a named area of interest: a dashed circle
labelled `NAI` and its name, heavier while drones are on their way to survey
it, red when a crash put it there. A geofence's label carries its altitude
band when it has one (`#4 · 60 m–UNL`, `#4 · SFC–120 m`; SFC is the surface,
UNL unlimited), and a fence whose floor is raised is drawn lighter and
dashed, since drones may pass beneath it. A drone's label says what it is
doing (`MEGA-3 · delivering`) when the model says, and its flight mode
otherwise.

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
- the geofences as a fence plan (`MAV_MISSION_TYPE_FENCE`), advertised in
  AUTOPILOT_VERSION: download all of them as `NAV_FENCE_CIRCLE_EXCLUSION` and
  `NAV_FENCE_POLYGON_VERTEX_EXCLUSION` items; upload a plan to set the
  ground stations' own fences (the scenario's and the dashboard's stay as they
  are; inclusion fences are refused); clear it to close them. `MISSION_CURRENT.fence_id` changes
  whenever a fence opens or closes;
- commands (`COMMAND_LONG`/`COMMAND_INT`): arm/disarm, takeoff, mission start,
  RTL, land, DO_SET_MODE, DO_REPOSITION, DO_PAUSE_CONTINUE, DO_SET_MISSION_CURRENT;
- the capability handshake QGroundControl needs for a generic autopilot
  (AUTOPILOT_VERSION with MISSION_INT).

Mission items run in the simulation: NAV_WAYPOINT, NAV_LOITER_UNLIM,
NAV_LOITER_TIME, NAV_RETURN_TO_LAUNCH, NAV_LAND, NAV_TAKEOFF, CONDITION_DELAY,
DO_JUMP, DO_CHANGE_SPEED and DO_WINCH (the drone hovers while the winch runs).
Altitudes are above mean sea level, over the pack's terrain if it has one;
mission items may be relative to home, absolute, or terrain-following.

The simulation runs at full speed unless you choose otherwise under Speed on
the dashboard (Max, Real time, or a Custom rate in ticks per second), so drones
fly faster than real time; telemetry shows wherever the sim has got to. Real
time is one tick per `tick_seconds`: 10 ticks/s in the example packs. When the simulation is paused,
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
cargo run --release -p sim-engine --bin simctl -- fly kaunas-deliveries --hours 8
```

### Without krabshell

`krab-drones` runs a drones pack on krABMaga alone: no driver, server, gateways
or dashboard. It reads the pack, builds the `World` (the krABMaga `State`),
and steps it on a krABMaga `Schedule` the way `simulate!` does:

```sh
cargo run --release -p sim-drones --bin krab-drones -- scenarios/kaunas-deliveries --steps 36000
cargo run --release -p sim-drones --bin krab-drones -- scenarios/london-patrol --reports reports.jsonl
cargo run --release -p sim-drones --bin krab-drones -- scenarios/london-patrol --tui
```

It prints the brain's size, agents, occupied cells, steps per second,
reports by kind, orders, stations, regions and surveys, and the gold hash.
Everything the drones do (behaviour trees, geofence bands, stations,
regions of interest) is in the model; the dashboard only draws it and sends
intents, so it all runs here the same. `--reports` writes every detection, waypoint and delivery as
JSON lines. `--tui` hands the world to krABMaga's `simulate!` with its terminal
monitor instead, and prints no summary.

To embed the model in your own krABMaga program, depend on `sim-drones`:

```rust
use sim_drones::krabmaga::engine::state::State;

let blob = sim_core::read_pack("scenarios/london-patrol".as_ref())?;
let mut world = sim_drones::World::new(&blob)?;
let mut schedule = sim_drones::schedule();
world.init(&mut schedule);
for _ in 0..1_000 {
    schedule.step(&mut world);
}
```

### API

Everything is under `/api/v1`. Send `Authorization: Bearer $DASHBOARD_TOKEN`.

The full reference is at **`/docs`** on a running server (the dashboard's **API**
tab), rendered from the OpenAPI 3.1 document served at `/api/v1/openapi.json`.
Neither needs a token. Point Swagger UI, Redoc, Postman or a client generator at
the JSON. The document is kept by hand in
[`crates/sim-axum/openapi.json`](crates/sim-axum/openapi.json); the tests in
`crates/sim-axum/tests/openapi.rs` fail if a route is added without it, or if a
response grows a field it does not describe.

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
| GET | `/openapi.json` | No token needed. This API as OpenAPI 3.1. |
| GET | `/run` | Mode, tick, rates, counters, recent intents. |
| POST | `/run/start` `/run/pause` `/run/step` `/run/reset` `/run/rate` | Clock control. |
| GET POST | `/scenarios`, `/scenarios/{id}/load` | List, upload (multipart zip), load. |
| GET PUT | `/config`, `/config/download` | Effective config; changes that alter the address space reset the world. |
| GET | `/state?cells=…` or `?center=…&k=…` | Bags for a cell set. |
| GET | `/state/map?res=N` | Occupancy aggregated to parent cells, with polygons. |
| GET | `/state/cells/{cell}` | One bag. |
| GET | `/state/entities?kind=…` | Every object with an exact position; `x-sim-tick`, `x-sim-mode` and `x-sim-tick-seconds` headers. |
| GET POST | `/state/download`, `/state/upload` | Checkpoint out and in (restore requires pause). |
| POST | `/intents` | Submit one intent. |
| GET | `/intents/log` | JSON Lines replay log (gold profile). |
| GET | `/metrics` | Prometheus text. |
| GET | `/reports?since=N` | Model reports (sightings, waypoints reached) newer than `seq` N. |
| GET | `/cells/lookup?lon=&lat=&res=` | Coordinate to cell. |
| GET | `/state/overlays` | What the model publishes beside its agents: geofences, open orders' addresses, stations and regions of interest (each key absent for a model without it). |
| GET POST | `/geofences` | Fences open, waiting to open and previously closed, with breach counts, recent breaches and reroutes; open one (`{"circle": {"lat", "lon", "radius_m"}}` or `{"polygon": [[lon, lat], …]}`, optional `"name"`, `"minutes"` (-1: until closed), `"after_minutes"`, and an altitude band in metres above ground level, `"floor_m"` (null: the surface) and `"ceiling_m"` (null: unlimited), with the ceiling above the floor). |
| DELETE | `/geofences/{id}` | Close a geofence, whoever opened it, or cancel one waiting to open. |
| PATCH | `/geofences/{id}` | `{"name": "..."}`: rename a geofence, open, waiting or previously closed (empty clears the name). |
| POST | `/geofences/{id}/extend` | `{"minutes": m}`: keep an open geofence open `m` minutes from now, or with -1 until it is closed. |
| GET POST | `/stations` | Launch and recovery stations as the model publishes them (`available` is false for a model without them), with their slots in use and drones waiting, each depot's delivery `range`, and recent station reports; add one (`{"lat", "lon", "name", "slots"}`, degrees, 1 to 100 slots). |
| DELETE | `/stations/{id}` | Remove a station, whoever added it. 404 if the model does not list it. |
| GET POST | `/rois` | Regions of interest as the model publishes them, with recent region and survey reports; designate one (`{"lat", "lon", "radius_m", "name"}`, degrees, 10 to 5000 m). |
| DELETE | `/rois/{id}` | Remove a region of interest, a crash site's included. 404 if the model does not list it. |
| GET | `/conflicts?outlines=` | Where the aircraft in the air could meet in the next minute: cells two or more could share (worst first, with summed probability, chance of two or more and contributors), resolution advisories, and conflicts left unresolved. |
| GET | `/weather` | The run's wind: mode, source, tick 0 and the simulation's UTC time, window, cells, last fetch and cache use, launch limits. |
| POST | `/weather/refresh` | Live weather only: fetch again now and hand it to the world. |
| GET | `/weather/cells?alt_m=&utc=` | The wind in every weather cell at a height and time, with outlines. |
| GET | `/weather/cells/{cell}?alt_m=&utc=` | The wind a cell takes (any resolution at least as fine as the weather's): now at the height asked and at each fetched height, and through the window. |
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
| `drone` | `sysid` (1–250), `callsign`, optional `elevation_m` (what its pad stands on, above mean sea level, e.g. a roof; the terrain if left out), optional `mission` (a list of `MISSION_ITEM_INT` fields: `seq`, `frame`, `command`, `param1`–`param4`, `x`, `y`, `z`) and `autostart`, or optional `depot` (a depot in the address book, making it a courier) and `cruise_agl_m` |
| `intruder` | `route`: `[[lon, lat, alt_m], ...]` flown in a loop, altitudes above the ground at each point, optional `speed_ms` |
| `person`, `vehicle` | optional `speed_ms`, `radius_m`; a vehicle's `type` (`car`, `van`, `pickup`, `bus` or `lorry`; drawn from the seed if left out) |
| `poi` | `name` |

`params.toml` sets `tick_seconds`, flight (`cruise_ms`, `climb_ms`,
`takeoff_alt_m`, `rtl_alt_m`), the battery (`battery_drain_pct_s`, the drain
at `cruise_ms` airspeed; `hover_drain_pct`, the share of it drawn at zero
airspeed, default 0, so drain is purely proportional to the square of
airspeed; `battery_reserve_pct`, `charge_pct_s` on the home pad,
`launch_battery_pct` below which a takeoff from the ground waits or is
refused), the launch wind limits (`max_wind_ms`, `max_gust_ms`; see
[Weather](#weather)), the winch
(`winch_drop_ms`, `winch_slow_ms`, `winch_slow_m`, `winch_release_s`,
`winch_reel_ms`, `winch_lock_s`), the camera (`hfov_deg`, `max_range_m`,
`p_detect`, `sensor_period_s`, `report_interval_s`) and generated targets
(`persons`, `vehicles`, `wander_radius_m`, `origin_lon`, `origin_lat`),
deliveries (`order_rate_per_address_day`, `dispatch_takeoff_m`,
`dispatch_approach_m`, `dispatch_load_s`, `reject_orders_in_geofences`),
`heatmap_resolution` (see
[Deliveries and run logs](#deliveries-and-run-logs)), and
`geofence_margin_m`: the buffer, in metres, the drones the simulation flies
keep from every geofence (default 30, from 2 to 1000; see
[Geofences](#geofences)), `geofence_vertical_margin_m` and
`beneath_min_agl_m` (flying beneath fences), and `roi_revisit_min`,
`survey_agl_m` and `crash_roi_radius_m` (see
[Stations and regions of interest](#stations-and-regions-of-interest)).
A pack's `world/behaviour.json`, `world/stations.json` and
`world/regions.json` are optional (see [Behaviour trees](#behaviour-trees)).

Terrain goes in `world/fields.json` as a regular latitude/longitude grid of
elevations above mean sea level; the format is documented in
`crates/sim-drones/src/terrain.rs`. Without it the ground is flat
at 0 m. People, vehicles and points of interest sit on the terrain, and the
camera's footprint grows with height above it. The address book goes in the
same file under `"addresses"`: `{"depots": ["MEGA", ...], "items": [[lat_e7,
lon_e7, depot_index, "label"], ...]}`. `scenarios/kaunas-deliveries` has a
Copernicus GLO-30 grid and every OpenStreetMap address within 5 km of its hubs (34502), built by
`tools/kaunas_deliveries.py`.

## Development

```sh
cargo test --workspace
cargo test --workspace --features sim-axum/parallel,sim-drones/parallel
cargo clippy --workspace --all-targets
```

Every test binary links Bevy; `CARGO_PROFILE_DEV_DEBUG=0
CARGO_PROFILE_TEST_DEBUG=0` keeps them small. CI runs formatting, clippy, tests
and gold replays (drones through `simctl` and standalone through
`krab-drones`, walkers on krABMaga's grid) on the default build and again with
`parallel`.
