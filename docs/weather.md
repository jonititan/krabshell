# Weather

The drones can fly in real wind. krabshell fetches it from Open-Meteo before a
scenario loads and holds one series per A5 *weather cell*. A drone takes the
wind of the weather cell above its own cell, at its height, at the
simulation's UTC time. Weather is off unless a pack asks for it. When it is
on, wind slows drones into a headwind, makes them crab in a crosswind, drains
their batteries faster and can keep them on the ground. A run with recorded
weather flies in the same air every time it is loaded.

## Modes

A pack chooses a mode with the `weather` param in `params.toml`.

| Mode | Tick 0 is | Fetched | If the fetch fails |
|---|---|---|---|
| `off` (the default) | Nothing: the run has no UTC clock | Never | — |
| `recorded` | `weather_start_utc` | Once, at load | The load is refused (`weather_unavailable`) |
| `live` | The wall-clock time of loading | At load, then every `weather_refresh_min` | The run flies in still air and tries again every minute |

**Recorded** weather keeps a run reproducible. The point is that the same
scenario flies in the same air every time, so krabshell refuses the load
rather than fly it quietly in still air.

**Live** weather follows the forecast while the run goes on. A refresh fetches
a new window starting an hour before the current wall-clock time and gives it
to the world as a `WEATHER_SET` intent (custom verb 140). The run's tick 0
does not move. The gold intent log records the intent like any other, so a
live run replays in the air it actually flew in.

A pack can also ship its own field as `world/weather.json`, in the `WindField`
shape from [`crates/sim-core/src/weather.rs`](../crates/sim-core/src/weather.rs).
It is used when `weather` is `off` (or absent), needs no fetch, and works with
`--no-weather`. If the pack also asks for `recorded` or `live` weather, the
fetched field replaces it.

## Pack params

```toml
weather = "recorded"                        # "live", or "off" (the default)
weather_start_utc = "2026-03-14T09:00:00Z"  # recorded only: the UTC time of tick 0
weather_hours = 24                          # how much to fetch
weather_resolution = 12                     # A5 resolution of the weather cells
weather_model = "best_match"                # an Open-Meteo model token
# weather_bbox = [23.79, 54.83, 24.02, 55.0]  # min lon, min lat, max lon, max lat
# weather_margin_km = 3                     # margin round the agents and sites
# weather_refresh_min = 60                  # live: how often to fetch again
```

| Param | Default | Limits | Meaning |
|---|---|---|---|
| `weather` | `off` | `off`, `recorded`, `live` | The mode. Anything else refuses the load. |
| `weather_start_utc` | none | RFC 3339 | Required for `recorded`; ignored for `live`. |
| `weather_hours` | 24 | 1 to 384 (16 days) | Length of the window fetched, from tick 0. |
| `weather_resolution` | 12, or the world's resolution if that is coarser | 0 to the world's resolution | Resolution of the weather cells. 12 is about 1.4 km, close to a weather model's grid. |
| `weather_model` | `best_match` | letters, digits and `_` | An Open-Meteo model token. `best_match` lets Open-Meteo choose. |
| `weather_bbox` | none | a valid box | Fetch exactly this area. |
| `weather_margin_km` | 3 | 0 to 200 (clamped) | Margin round the pack's points when there is no box and no terrain. |
| `weather_refresh_min` | 60 | 5 to 1440 (clamped) | Live only: minutes between fetches. |
| `max_wind_ms` | 12 | 0 or more; 0 is no limit | Launch limit on wind at takeoff height. |
| `max_gust_ms` | 15 | 0 or more; 0 is no limit | Launch limit on gusts. |

The bundled `kaunas-deliveries` pack uses `recorded` weather from
2026-03-14 09:00 UTC for 6 hours. `iron-wolf` uses `recorded` weather from
2026-05-12 08:00 UTC for 12 hours, over a `weather_bbox` that takes in Kaunas
Airport and its traffic, at `weather_resolution` 11 (about 2.8 km cells) to
keep the number of weather cells down.

## What is fetched

For each weather cell, krabshell asks Open-Meteo for the hourly wind speed and
direction at 10, 80, 120 and 180 m above the ground, and the gusts at 10 m,
at the cell's centroid. Levels that Open-Meteo has no data for are dropped. A
missing value at one level is filled from the nearest level that has one. A
missing hour is filled from the hour before (or, at the start, from the first
hour with data).

180 m is the highest level. Wind above it is the 180 m wind (see
[How the wind is read](#how-the-wind-is-read)), so a manned aircraft
hundreds of metres up flies in the 180 m wind.

The window runs from the hour at or before tick 0 to the hour at or after
tick 0 plus `weather_hours`. Six hours from 09:00 is therefore seven frames,
09:00 to 15:00.

**Sources.** A window that starts within the last 85 days, or in the future,
comes from Open-Meteo's forecast API. Older windows come from its historical
forecast archive, which has the same variables. The status shows which was
used, for example `Open-Meteo best_match historical forecast`.

**Requests and cost.** Locations go 100 to a request. Open-Meteo counts each
location as one call. The Kaunas pack has 169 weather cells, so it is two
requests and 169 calls. One area may hold at most 1 000 weather cells. Beyond
that the load is refused with a message to lower `weather_resolution` or
shrink `weather_bbox`. An area too big to list at the world's own resolution
(more than 2 million world cells) is refused too, asking for a smaller
`weather_bbox`.

## The area

The area to fetch is chosen in this order:

1. `weather_bbox`, if the pack gives one.
2. Otherwise the extent of the pack's terrain (`world/terrain`), if it has any.
3. Otherwise a box round every seeded agent, station, region of interest and
   delivery address, widened by `weather_margin_km` on every side.

If none of these gives a point, the load is refused and asks for
`weather_bbox`.

The weather cells are not simply the A5 cells of the area at the weather
resolution. krabshell takes every world cell in the area and collects their
ancestors at the weather resolution. A drone finds its wind through its own
cell's ancestor, and A5 ancestors do not always cover their descendants
exactly, so this is the set that drones will actually ask for.

## How the wind is read

Values are stored as whole centimetres per second, eastward and northward, so
a field read from a pack, the cache or a checkpoint gives the same bits on
every machine.

- **In time**, the wind is linear between hourly frames. Before the first
  frame and after the last, the end frame holds.
- **In height**, it is linear between the fetched levels. Above the top level
  the top holds. Below the lowest level (10 m) it falls off toward the ground
  as a one-seventh power law, reaching zero at ground level.
- **Gusts** are the 10 m gusts, linear in time, at every height.
- **Outside the field** (a cell with no weather cell, or no weather at all)
  the air is calm.

Heights are above the ground. A drone's height is its altitude less the
terrain under it.

## Simulation time

With weather, the run has a UTC clock: tick 0 is the field's start time, and
the time now is that start plus tick × `tick_seconds`. The clock moves with
simulated time, not wall-clock time. A run stepped faster than real time
moves through the forecast faster too. If it outruns the window, the last
hour's wind holds.

The **Weather** screen and `GET /api/v1/weather` show the time of tick 0 and
the simulation's time now. The conflict assessment uses the same clock for
the wind it predicts in (see [Conflicts](conflicts.md)).

## How wind affects the drones

Wind applies to the drones the simulation flies and to drones flown by a
ground station. Manned aircraft (the drones model's `aircraft` kind) fly
their route's airspeed through the wind and hold the track, so their ground
speed changes with it. People, vehicles and scripted intruders are not
affected: intruders fly their routes at a set ground speed.

- **Flight.** A drone holds a straight track by crabbing into the crosswind.
  Its ground speed is what the wind leaves of its airspeed: slower into a
  headwind, faster downwind. A wind stronger than it can fly blows it back,
  nose into the wind. Hovering in wind costs airspeed but not position, until
  the wind is stronger than it can fly, when it drifts downwind.
- **Battery.** Drain goes with the square of airspeed through the air,
  climbing included, scaled so `battery_drain_pct_s` is the drain at
  `cruise_ms`. Hovering in still air costs nothing unless `hover_drain_pct`
  sets a floor. Hovering into a wind costs what flying at that speed does.
  This is the same model with or without weather.
- **Planning.** Couriers and surveys plan each trip for the wind in their cell
  at cruise height. A headwind leg costs more than the same leg downwind, and
  a trip that would not make it goes by way of stations instead. The
  delivery range drawn on the map shrinks to what holds in the worst
  direction, and is worked out again when the hourly frame changes. See
  [Drones](drones.md) and [Stations](stations.md).
- **Launching.** A drone on the ground looks up the wind in its cell at its
  takeoff height (`takeoff_alt_m`, default 20 m). If the wind is over
  `max_wind_ms` or the gusts are over `max_gust_ms`, the behaviour tree's
  `WindWithinLimits` condition fails and it stays on the ground (see
  [Drone behaviours](drone-behaviours.md)). A ground station's takeoff
  command is refused with the wind and gust speeds in the message. A drone
  already in the air is never grounded by wind.
- **Telemetry.** MAVLink `VFR_HUD` reports airspeed and ground speed
  separately, and the heading is where the nose points, not the track. See
  [Protocols](protocols.md).

## Caching

Fetched fields go to `var/weather-cache/`, or the directory given by
`--weather-cache` (`KRABSHELL_WEATHER_CACHE`). Each fetch is one JSON file,
named by a fingerprint of everything that decides the answer: the host, the
model, the weather resolution, the cells and the window.

- A window wholly in the past never changes, so its file is kept for good.
  Running the scenario again reads it and does not call Open-Meteo.
- A window that reaches into the future is a forecast. Its fingerprint also
  includes the hour it was fetched in, so in the next hour the cache misses
  and the newer forecast is fetched.

Files are written under a temporary name and renamed, so a crash, or two
servers sharing the directory, never leave half a file. An unreadable file is
ignored and fetched again. Nothing is ever deleted: clear the directory by
hand if it grows. The status shows how many entries it holds and their size.

## Server flags

| Flag | Environment | Default | Meaning |
|---|---|---|---|
| `--weather-cache` | `KRABSHELL_WEATHER_CACHE` | `var/weather-cache` | Where fetched weather is kept. |
| `--no-weather` | | off | Never fetch. Recorded packs refuse to load, live ones fly in still air, and a pack's own `world/weather.json` still works. |
| `--open-meteo-key` | `OPEN_METEO_API_KEY` | none | A commercial Open-Meteo key. |
| `--open-meteo-url` | `KRABSHELL_OPEN_METEO_URL` | Open-Meteo's | Root of a forecast API to use instead, such as a self-hosted Open-Meteo. |
| `--open-meteo-historical-url` | `KRABSHELL_OPEN_METEO_HISTORICAL_URL` | Open-Meteo's | Root of a historical forecast API, likewise. |

Requests time out after 30 seconds.

## Offline

Without network access, or with `--no-weather`:

- A `recorded` pack loads only if its window is already in the cache (and
  `--no-weather` is not set). Otherwise the load is refused with
  `weather_unavailable`.
- A `live` pack loads and flies in still air. The status shows the error, and
  the server tries again every minute until a fetch succeeds.
- A pack with `world/weather.json` and no `weather` param always works.

## Checkpoints and resets

A checkpoint carries the wind the world was flying in, so a restore flies in
the same air, whatever the pack now asks for. After a restore the status
shows the origin as `checkpoint`.

A reset rebuilds the world from the pack as it was loaded, which for a live
run may hold an older field than the server last fetched. For a live run the
server checks every 20 seconds whether the world flies in the field it holds,
by comparing fingerprints, and sends its field again if not. A recorded run
is not resent: the status shows a banner instead.

## Licence

Open-Meteo's free API is for non-commercial use, at up to 10 000 calls a day,
and each location counts as a call. It also limits calls a minute and an
hour (600 and 5 000 at the time of writing), and counts a location as more
than one call when the window is longer than two weeks. Over a limit,
Open-Meteo answers HTTP 429 and the fetch fails with `rate limited`, like any
other failed fetch (see [Modes](#modes)): krabshell does not wait and try
again within a load. Nothing is cached from a failed fetch, so a recorded
pack is fetched whole once the limit has passed. Commercial use needs a key
(`--open-meteo-key`). The data is CC BY 4.0. The Weather screen and the API
carry the credit: "Weather data by Open-Meteo.com (CC BY 4.0)".

## The Weather screen

The dashboard's **Weather** screen ([Dashboard](dashboard.md#weather)) has
three parts.

- **Status.** The mode, the simulation's UTC time, tick 0, the origin
  (`open_meteo`, `pack` or `checkpoint`), the source, the window, the number
  of weather cells and heights, the area, when it was last fetched, what the
  fetch cost (or that it came from the cache), the cache's size and the launch
  limits. It updates every five seconds. A live run has a **Refresh now**
  button. A banner says when the last fetch failed, or when the world is not
  flying in the field shown (after a reset or a restore). With no weather the
  panel shows the params to add.
- **Wind map.** Each weather cell shaded by wind speed with an arrow
  downwind, at a chosen height (10 to 180 m, 50 m by default). With **Now**
  ticked it follows the simulation's time; untick it and drag the **Hour**
  slider to look through the window. The colour scale tops out at the
  strongest wind shown, rounded up to 5 m/s and at least 10. Drones,
  stations, the delivery range, geofences and the other map layers can be
  shown on top (see [Map](map.md)).
- **Query.** Click the map, or give a latitude and longitude or an A5 cell,
  and a height. It shows the weather cell the point takes its wind from, the
  wind and gusts now, the wind at each fetched height, and a chart of wind and
  gusts through the whole window at that height.

## API

Every weather endpoint is under `/api/v1` and needs the dashboard token (see
[API guide](api-guide.md)). The full reference is at [`/docs/api`](/docs/api).

| Method | Path | |
|---|---|---|
| GET | `/weather` | The status: mode, origin, source, clock, window, area, fetch cost, cache, sync and launch limits. |
| POST | `/weather/refresh` | Fetch a live run's weather again now. |
| GET | `/weather/cells` | The wind in every weather cell, at a height and time. |
| GET | `/weather/cells/{cell}` | The wind one cell takes: now, at each height, and through the window. |

```sh
T="Authorization: Bearer $DASHBOARD_TOKEN"

# Status. "available" is true when the drones fly in a field.
curl -s -H "$T" localhost:8080/api/v1/weather

# Every weather cell at 80 m, at the simulation's time now.
curl -s -H "$T" "localhost:8080/api/v1/weather/cells?alt_m=80"

# The same at a given hour.
curl -s -H "$T" "localhost:8080/api/v1/weather/cells?alt_m=80&utc=2026-03-14T11:00:00Z"

# What a cell of any resolution at least as fine as the weather's takes.
CELL=$(curl -s -H "$T" "localhost:8080/api/v1/cells/lookup?lon=23.9036&lat=54.8985" | jq -r .cell)
curl -s -H "$T" "localhost:8080/api/v1/weather/cells/$CELL?alt_m=50"

# Fetch a live run's weather again.
curl -s -H "$T" -X POST localhost:8080/api/v1/weather/refresh
```

`alt_m` is metres above the ground, 0 to 10 000, and defaults to 50 (about
where couriers cruise). `utc` is an RFC 3339 time and defaults to the
simulation's time now. Each wind gives `east_ms` and `north_ms` (the direction
it blows toward), `speed_ms`, `from_deg` (the direction it blows from,
clockwise from north, as forecasts give it) and `gust_ms`.

Errors:

- `/weather/cells` and `/weather/cells/{cell}` answer 404 when the run has no
  weather, or when no weather was fetched for that cell.
- `/weather/cells/{cell}` answers 400 for a cell coarser than the weather
  resolution, and for a bad `alt_m` or `utc`.
- `/weather/refresh` answers 400 when the run has no weather, when its
  weather is recorded (recorded weather is fixed), or with `--no-weather`. It
  answers `weather_unavailable` when the fetch itself fails.

The status's `rev` and `world_rev` are fingerprints of the field the server
holds and the one the world flies in. `in_sync` is true when they agree.
