# The drones model

The `drones` model simulates multirotor drones flying MAVLink missions over a world of things to find: people and vehicles wandering the streets, intruder drones on fixed circuits, and named points of interest. Drones can be flown by ground stations through krabshell's MAVLink gateway, or by the simulation itself as delivery couriers and patrols. Every entity is a krABMaga agent stepping against a krABMaga `State`, with krABMaga's `SparseA5Grid` as the occupancy of record. The code is in [`../crates/sim-drones/src/`](../crates/sim-drones/src/lib.rs); the seam to the driver is [`../crates/sim-engine/src/models/drones/mod.rs`](../crates/sim-engine/src/models/drones/mod.rs). Every parameter named on this page is listed with its default in [Scenario packs](scenarios.md#drones-model-parameters).

## Agent types

A pack seeds agents in `world/agents.json` (see [Seed agents](scenarios.md#seed-agents)), each with a `kind` and a `state`. People and vehicles can also be scattered by count. All altitudes are above mean sea level, over the pack's terrain; without terrain the ground is at 0 m.

### Drones

A drone starts disarmed on its home pad, with a full battery. Its home is where it starts: the terrain there, or `elevation_m` above sea level for a rooftop pad. Within 10 m of home it lands on the pad; anywhere else, on the terrain.

| `state` key | Default | Meaning |
|---|---|---|
| `sysid` | the lowest free | MAVLink system id, 1–250, unique in the world. |
| `callsign` | `UAV-<sysid>` | Name on the map, in CoT and in reports. |
| `elevation_m` | the terrain | What its pad stands on, metres above sea level. |
| `cruise_ms` | the `cruise_ms` parameter | Its own cruise airspeed. |
| `mission` | none | Mission items to start with, each with the fields of `MISSION_ITEM_INT`: `seq`, `frame`, `command`, `autocontinue`, `param1`–`param4`, `x` (latitude × 10⁷), `y` (longitude × 10⁷), `z` (metres). |
| `autostart` | `false` | Start flying the mission at tick 0. Needs a `mission`. The simulation flies it. |
| `depot` | none | A depot in the address book. Makes the drone a courier for that depot, flown by the simulation. |
| `cruise_agl_m` | 60 | A courier's cruise height above the terrain. |
| `spare` | none | A depot in the address book this drone stands by for. It must not have a `depot` of its own. |

A drone with neither `autostart`, `depot`, nor a call-up as a spare is left to ground stations. See [Who flies a drone](#who-flies-a-drone).

### Intruders

An intruder is an uncontrolled drone flying a closed circuit. It starts at its seed position and flies to each point of its route in turn, then back to the first, for ever.

| `state` key | Default | Meaning |
|---|---|---|
| `route` | required | At least two `[lon, lat, alt_m]` points, each altitude above the ground at that point. |
| `speed_ms` | the `intruder_speed_ms` parameter | Ground speed. |
| `callsign` | empty | Name in reports and CoT. |

Intruders fly at a fixed ground speed and climb at 3 m/s whatever the wind. They cannot be commanded.

### People and vehicles

A person or vehicle walks or drives to a random point within its wander radius of where it started, waits up to 20 s, and picks another. It stays on the terrain and is not affected by wind. A destination outside the world is skipped.

| `state` key | Default | Meaning |
|---|---|---|
| `speed_ms` | `person_speed_ms` or `vehicle_speed_ms` | Speed. |
| `radius_m` | `wander_radius_m` | How far from its start it wanders. |
| `type` | drawn from the seed | A vehicle's type: `car`, `van`, `pickup`, `bus` or `lorry`. |
| `callsign` | empty | Name in reports. |

`persons` and `vehicles` scatter that many more at reset, uniformly over a disc of `wander_radius_m` round `origin_lon`, `origin_lat` (or the root's centre). Drawn vehicle types follow city traffic: 70 % cars, 14 % vans, 8 % pickups, 5 % lorries and 3 % buses.

### Points of interest

A point of interest is a named place that never moves. Its name is its `state`'s `name`, or else `callsign`, or else `note`, or else `POI <id>`. Drones' cameras can see it, and the CoT emitter sends it to TAK.

### Spawning while running

A `Spawn` intent adds people, vehicles or points of interest at the centre of the intent's cell: payload `kind` (`person`, the default, `vehicle` or `poi`), `count` (1 to 1000), and optionally `type` for a vehicle and `name` for a point of interest. Drones and intruders cannot be spawned. A `Despawn` intent removes one agent, or everything in a cell. `Move` and `SetField` are refused: drones move by mission, and everything else moves on its own.

## How a tick works

A tick is `tick_seconds` of simulated time, 0.1 s by default. Nothing paces it to the wall clock; the gateways keep MAVLink and CoT on real time however fast the simulation runs. A step goes:

1. The tick advances, and agents added by intents or as replacements join the schedule.
2. The dispatcher steps first. It closes geofences whose time is up, makes the geofence, station and region changes asked for since the last tick, opens scheduled fences that are due, turns away queued orders a new fence covers (with `reject_orders_in_geofences`), works out the delivery range again if anything it depends on changed, and places the orders due.
3. Every agent steps in id order. Each reads the committed world (everything as it was at the end of the last step) and writes itself to the uncommitted side. A drone runs its behaviour tree; a person, vehicle or intruder moves; a point of interest stays put.
4. The grid and the published board are committed together.

Intents are checked against the committed world as they arrive, and applied by the agent they address at the start of its next step. Nothing an intent does is visible until the step that applies it commits.

### A drone's step

Each drone first takes up any courier duty it has been called up for, looks up the wind in its cell, and then ticks its behaviour tree from the root. The default tree, [`../crates/sim-drones/behaviour/drone.json`](../crates/sim-drones/behaviour/drone.json), does this, in order:

1. If the simulation flies it, plan round the geofences and queue what a ground station would send.
2. Obey the commands that arrived since the last tick: from ground stations, the API, and step 1.
3. If the simulation flies it, hold off a station whose landing slots are all taken.
4. Fly one tick: battery and autopilot. A crash ends the step.
5. If it is a courier or surveyor down and disarmed after a trip, hand back any order it did not deliver and start loading.
6. If the simulation flies it: low on battery and out of reach of home, divert to a station; or, charged at a station away from home, fly home.
7. If it is free for work and the wind is within launch limits, take the oldest order it can, or else survey the region of interest most overdue a look.
8. Report any geofence it has entered.
9. If its camera is due and it is in the air, scan.

A pack can replace the tree with its own `world/behaviour.json`. The leaves and the rules for writing one are in [Drone behaviours](drone-behaviours.md).

Each drone also publishes an `activity`, derived each step for people to read: `surveying`, `delivering`, `returning`, `charging`, `idle` or `flying`.

## Flight

Positions and motion are integers in the units MAVLink's `_INT` messages use: degrees × 10⁷, millimetres, centidegrees and centimetres per second. Each step converts to floating point, moves, and rounds back. Distances use a local flat-Earth approximation, good to well under a metre over a scenario's few kilometres. The flight code is [`../crates/sim-drones/src/flight.rs`](../crates/sim-drones/src/flight.rs) and the autopilot [`../crates/sim-drones/src/mission.rs`](../crates/sim-drones/src/mission.rs).

A drone flies straight at its target, horizontally at its cruise airspeed and vertically at `climb_ms`, independently, as a multirotor does: it turns at once and never overshoots. In still air its nose points along the track. In wind it holds the straight track by crabbing into the crosswind, so its ground speed is what the wind leaves of its airspeed; a wind stronger than it can fly blows it backwards, nose into the wind. Hovering in wind costs airspeed but not position. On the ground it is not blown about. The wind it flies in is that of its cell at its height above the ground; without weather it is calm. See [Weather](weather.md).

### Modes

| Mode | What the drone does |
|---|---|
| `disarmed` | On the ground, motors off. |
| `idle` | On the ground, armed, waiting for a command. |
| `takeoff` | Climbing straight up to an altitude, then `loiter`. |
| `auto` | Flying the uploaded mission. Past its end, `loiter` in the air or `idle` on the ground. |
| `loiter` | Holding position in the air. |
| `goto` | Flying to a point given by `DO_REPOSITION`, then `loiter`. |
| `rtl` | Climbing to `rtl_alt_m` above home if lower, flying home level, then `land`. |
| `land` | Descending onto whatever is below, then `disarmed`. |

### Mission items

A mission item has exactly the fields of `MISSION_ITEM_INT`, so a mission uploaded by a ground station comes back out unchanged on download. The frame decides what `z` means:

| Frame | `z` is |
|---|---|
| 0, 5 | metres above mean sea level |
| 3, 6 | metres above home (the pad) |
| 10, 11 | metres above the terrain under the drone, worked out afresh each tick, so the leg follows the ground |
| 2 | not used; allowed only for items that do not fly anywhere |

Any other frame is refused at upload, as is an item whose `seq` is not its index, a destination outside the world, or a mission longer than `max_mission_items`. A zero latitude and longitude means "here".

| Command | Number | What it does |
|---|---|---|
| `NAV_WAYPOINT` | 16 | Fly to the point. `param1` is a hold time in seconds; `param2` an acceptance radius in metres (1 m if not given). |
| `NAV_LOITER_UNLIM` | 17 | Fly to the point and hold there until told otherwise. |
| `NAV_LOITER_TIME` | 19 | Fly to the point and hold `param1` seconds. |
| `NAV_RETURN_TO_LAUNCH` | 20 | Switch to `rtl`. |
| `NAV_LAND` | 21 | Fly level to the point, descend and land. The drone stays armed if items follow, and disarms if it was the last. |
| `NAV_TAKEOFF` | 22 | Climb to `z`. On the ground it first waits until the battery holds `launch_battery_pct`; on a charger, `param2` asks for more (a krabshell extension, used by trips through stations). |
| `CONDITION_DELAY` | 112 | Hover for `param1` seconds. |
| `DO_JUMP` | 177 | Go to item `param1`, `param2` more times; -1 for ever. |
| `DO_CHANGE_SPEED` | 178 | Set the cruise airspeed to `param2` m/s. |
| `DO_WINCH` | 42600 | Hover while a winch runs: `param2` 4 (`WINCH_DELIVER`) lowers a parcel the whole height, releases it and reels in; `param2` 1 (`RELATIVE_LENGTH_CONTROL`) pays out `param3` metres at `param4` m/s. See [the winch](#the-winch). |

Any other command (a camera trigger, a servo) completes at once. A mission runs at most 16 items that take no time in one tick, so a `DO_JUMP` loop cannot hang the simulation. Each completed item is a `waypoint` report.

### Commands

Commands arrive as `COMMAND_LONG` parameters, through the MAVLink gateway or as intent code 103.

| Command | Number | Accepted when |
|---|---|---|
| `COMPONENT_ARM_DISARM` | 400 | `param1` 1 arms, refused at or below `battery_reserve_pct`. `param1` 0 disarms, refused in the air unless `param2` is 21196. |
| `NAV_TAKEOFF` | 22 | Armed; on the ground, the battery at `launch_battery_pct` and the wind at takeoff height within `max_wind_ms` and `max_gust_ms`. Climbs to `param7` metres above the pad or ground, or `takeoff_alt_m`. |
| `MISSION_START` | 300 | Armed with a mission loaded. Starts at item `param1` if given. |
| `NAV_RETURN_TO_LAUNCH` | 20 | In the air. |
| `NAV_LAND` | 21 | In the air. |
| `DO_SET_MODE` | 176 | The auto flag (4) in `param1` starts the mission; anything else holds position in the air. |
| `DO_PAUSE_CONTINUE` | 193 | `param1` 1 continues the mission; 0 holds position in the air. |
| `DO_REPOSITION` | 192 | In the air, to a point inside the world. `param5` latitude, `param6` longitude, `param7` altitude above home (the current altitude if not given). |
| `DO_SET_MISSION_CURRENT` | 224 | `param1` within the mission. |

Anything else is refused with "command N is not supported". Mission uploads (code 100), clears (101) and set-current (102) are intents too. Every drone intent is tried on a copy of the drone as earlier intents this tick will leave it, so one that is accepted cannot fail when it is applied. The MAVLink side of all this is in [Protocols](protocols.md).

## Battery

The battery is held in parts per million of a full charge, fine enough to drain smoothly at a 0.1 s tick.

- **Drain.** Only in the air. Each tick it loses `battery_drain_pct_s` scaled by the square of its airspeed (horizontal through the air, and vertical) over `cruise_ms`: `drain × (h + (1 − h) × v² / cruise_ms²)`, where `h` is `hover_drain_pct` / 100. With the default `hover_drain_pct` of 0, hovering in still air costs nothing; hovering into a wind costs what flying at that speed does. A drone flying faster than `cruise_ms` drains faster than the nominal rate.
- **Failsafe.** When the charge reaches `battery_reserve_pct` in the air, the drone switches to `rtl` by itself, unless it is already returning or landing. This happens once per charge, as an autopilot's does: whoever flies the drone may then send it elsewhere. It resets once the charge is back above the reserve.
- **Crash.** If the charge reaches 0 in the air, the drone falls to the ground where it is and is wrecked. See [Crashes and spares](#crashes-and-spares). With `battery_drain_pct_s = 0` nothing drains and nothing crashes.
- **Charging.** On the ground on its home pad, or on a station's pad, it gains `charge_pct_s` a tick's worth at a time, up to full. The default is 0: no charger.
- **Launching.** A takeoff from the ground waits (in a mission) or is refused (as a command) below `launch_battery_pct`.

## Camera and sightings

Each drone carries a downward-looking camera ([`../crates/sim-drones/src/sensor.rs`](../crates/sim-drones/src/sensor.rs)). It scans every `sensor_period_s`, on ticks that are a whole multiple of the period, and only in the air.

- Its footprint on the ground is a circle of radius height-above-ground × tan(`hfov_deg` / 2), capped at `max_range_m`. Below 1 m, it sees nothing.
- People, vehicles and points of interest inside the footprint can be seen. Intruders are searched for by slant range out to `max_range_m` instead, since they are not on the ground. Other drones are not reported.
- The chance of seeing a target on a scan falls linearly from `p_detect` at the centre to `p_detect` × (1 − `edge_falloff`) at the edge.
- The same target is reported to the same drone at most once every `report_interval_s`.

Each sighting is a `detection` report with the target's position. The CoT emitter turns them into TAK markers; see [Protocols](protocols.md).

## Missions

A drone gets a mission in one of four ways:

- **From the pack.** A seed drone's `mission`, flown from tick 0 if `autostart` is set.
- **From a ground station** over MAVLink, or from anything that submits intent 100.
- **From dispatch.** A courier is given a fresh mission for each order. See below.
- **From a survey.** A drone the simulation flies that is free for work flies out to the region of interest most overdue a look, circles it at `survey_agl_m` with its camera on, and comes home, by way of stations if it must. See [Stations](stations.md).

Uploading a mission replaces the old one and starts it from item 0, but does not start it flying. Clearing a mission while in `auto` puts the drone in `loiter`.

## Deliveries and dispatch

Deliveries need an address book in the pack ([The address book](scenarios.md#the-address-book)) and `order_rate_per_address_day` above 0. The code is [`../crates/sim-drones/src/dispatch.rs`](../crates/sim-drones/src/dispatch.rs).

### Orders and queues

Every address orders at random, independently, at `order_rate_per_address_day`. Together that is one Poisson stream at the summed rate, with each order going to an address chosen uniformly; the gaps are drawn from the seed and rounded to whole ticks. A `gold` run places the same orders at the same ticks.

Each order joins a first-come first-served queue at the depot that serves its address. Each depot has its own queue; the published overlays list up to 2000 open orders, waiting ones first, for the map to draw.

### Couriers and the launch rules

A drone with a `depot` is a courier for it. A courier takes the next order when it is all of:

- on the ground on its home pad, disarmed or idle, with no order and no survey;
- loaded: `dispatch_load_s` since it last landed (a courier is loaded at the start);
- charged to `launch_battery_pct`;
- inside the launch wind limits: the wind at `takeoff_alt_m` in its cell no more than `max_wind_ms`, the gusts no more than `max_gust_ms`.

It takes the oldest order in its depot's queue whose address is not under a geofence that cannot be flown beneath. Orders under such a fence are skipped, and wait. A courier with no order it can take surveys instead, if there are regions of interest due.

### The delivery mission

The courier is given a fresh mission for one order:

| Item | Command | Frame | Where |
|---|---|---|---|
| 0 | `NAV_TAKEOFF` | relative to home | `dispatch_takeoff_m` above the pad |
| 1 | `NAV_WAYPOINT` | terrain | the address, at the courier's `cruise_agl_m` |
| 2 | `DO_WINCH` | — | `WINCH_DELIVER`: lower the parcel and reel in |
| 3 | `NAV_WAYPOINT` | terrain | home, at `cruise_agl_m`, 5 m acceptance radius |
| 4 | `NAV_WAYPOINT` | relative to home | home, `dispatch_approach_m` above the pad |
| 5 | `NAV_LAND` | relative to home | the pad |

It starts at once, in `auto`; the takeoff waits on the pad for `launch_battery_pct` if it must. If the address is too far for one charge and there are stations, the trip goes by way of them, landing and charging at each. The simulation keeps the flight clear of geofences as it goes; see [Geofences](geofences.md).

The order counts as delivered when the `DO_WINCH` item completes: a `delivery` report at the address, with how long the order waited for a drone, how long the flight took and the total. When the courier lands home and disarms, loading starts and the order is forgotten.

### The winch

There is no line or parcel in the world. A `DO_WINCH` item holds the drone in a hover for as long as the real sequence would take. For `WINCH_DELIVER` from height `h` above the ground:

```
(h − winch_slow_m) / winch_drop_ms + winch_slow_m / winch_slow_ms
  + winch_release_s + h / winch_reel_ms + winch_lock_s
```

With the defaults, from 70 m that is about 75 s.

### Orders that do not get delivered

A courier that comes down and disarms still holding an undelivered order (its low-battery failsafe sent it home, a ground station landed it, or the simulation sent it home because a fence opened over the address) puts the order back at the front of its depot's queue, and starts loading.

By default an order whose address is under a geofence waits in the queue until the fence closes. With `reject_orders_in_geofences = true` it is turned away instead, at whichever of three moments comes first:

| `stage` | When |
|---|---|
| `placed` | The address is under a fence when the order is placed. |
| `queued` | A fence opens over the address while the order waits. |
| `returned` | The courier comes back without it, or crashes with it, and a fence now covers the address. |

Each is an `order_rejected` report naming the fence and the reason, for example "the delivery address is inside geofence #3 (Žalgiris Arena), which opened while the order waited". Turning orders away changes no other order: the order stream draws the same numbers either way.

## Crashes and spares

A drone whose battery reaches 0 in the air falls where it is and is wrecked. It leaves the board at once, and off the schedule after that step. Then:

1. Any order it was carrying and had not delivered goes back to the front of its depot's queue (or is turned away; see above).
2. If `crash_roi_radius_m` is above 0, a region of interest named "Crash of `<callsign>`" is left where it fell, so another drone goes to look.
3. Something takes its place:
   - A courier's duty goes to its depot's first spare still standing by. The spare is called up: it becomes a courier for that depot, flown by the simulation, from its own pad.
   - Otherwise (a courier once the spares are used, a spare, a patrol, a ground station's drone) a new drone arrives on the wreck's home pad, in the same role: courier for the same depot, spare for the same depot, flown by the simulation if the wreck was.

A new drone gets an agent id and a MAVLink system id nothing has had before; ids are never reused. Its callsign is the wreck's with the trailing number swapped for the new system id: `MEGA-7` is followed by `MEGA-43`, `MEGA-SPARE` by `MEGA-SPARE-44`. Once system id 250 has been used there is nothing to replace a wreck with. The crash is a `crash` report saying what replaced it.

## Who flies a drone

A drone is flown by the simulation if it is a courier, a drone the pack starts with `autostart`, a spare once it is called up, or a replacement for one of those. The simulation acts as its operator ([`../crates/sim-drones/src/operator.rs`](../crates/sim-drones/src/operator.rs)): it keeps the drone `geofence_margin_m` clear of every fence by sending what a ground station would (missions with detour waypoints, `DO_REPOSITION`, `NAV_RETURN_TO_LAUNCH`), holds it off a full station, diverts it to a station when the failsafe fires out of reach of home, flies it home from a station, and gives it work. Each set of messages it sends to take a drone round a fence is a `reroute` report; a divert is a `station` report.

Every other drone (UAV-4 in `london-patrol`, all of `swarm-survey`, a spare not yet called up) is left to ground stations. The simulation never takes work on for it and does not steer it round fences; it only flies what it is told.

A ground station can command any drone, including one the simulation flies. Its commands are obeyed at the start of the drone's next step, followed by anything the operator sends that tick. The simulation does not hand the drone over. It goes on keeping it clear of fences (a mission a ground station uploads is rerouted like any other), and a courier goes back to work once it is down on its home pad again: a courier landed with an undelivered order hands it back to the queue, and takes the next one when it is loaded and charged. To keep a drone wholly in a ground station's hands, seed it without `depot` or `autostart`.

## Reports

Besides its state, the model raises events. The driver stamps each with a `seq` that only grows, keeps the last 512 in the published view, and serves them at `GET /api/v1/reports?since=<seq>`. The run logger, the CoT emitter and the MAVLink gateway read them. Every report has `tick`, `kind`, a stable `uid`, optionally `observer` (the agent it is about) and `target`, a position (`lat_e7`, `lon_e7`, `alt_mm` above sea level) and a `detail` object. The kinds are in [`../crates/sim-core/src/report.rs`](../crates/sim-core/src/report.rs).

| `kind` | When | Position | `detail` |
|---|---|---|---|
| `detection` | A camera sees a target. `target` and `target_kind` (`person`, `vehicle`, `intruder`, `poi`) name it. | the target | `sysid`, `range_m`; the target's `callsign` and `vehicle_type` if it has them; `course_cdeg` and `speed_cms` if it is moving |
| `waypoint` | A drone completes a mission item. | the drone | `seq`, `sysid` |
| `delivery` | A courier's `DO_WINCH` completes. | the address | `order`, `address`, `label`, `depot`, `sysid`, `callsign`, `ordered_tick`, `dispatched_tick`, `delivered_tick`, `wait_s`, `flight_s`, `total_s` |
| `order_rejected` | An order is turned away under a geofence. | the address | `order`, `address`, `label`, `depot`, `ordered_tick`, `fence`, `fence_name`, `stage`, `reason` |
| `landing` | A drone touches down after flying. | the drone | `sysid`, `callsign`, `depot` (a courier's), `battery_pct` |
| `crash` | A drone's battery runs flat in the air. | where it fell | `sysid`, `callsign`, `depot`, `cause`, `requeued_order`, `roi`, `replaced_by` (`id`, `sysid`, `callsign`, `spare`, or null) |
| `geofence` | A fence opens, is scheduled, renamed, extended, closed or cancelled. | its centre | the fence, `action` (`open`, `schedule`, `rename`, `extend`, `close`, `cancel`), `reason` (`scenario`, `requested`, `scheduled`, `expired`) |
| `breach` | A drone in the air is inside a fence, once per fence it enters. | the drone | `sysid`, `callsign`, `depot`, `fence`, `how` (`flew in`, `fence opened round it`), `flown_by` (`operator`, `ground station`), `mode` |
| `reroute` | The operator sends a drone messages to take it round a fence. | the drone | `sysid`, `callsign`, `reason`, `messages`, `detours`, `airspace_rev` |
| `station` | A station is added or removed, or a drone diverts to one. | the station, or the drone | the station and `action` (`add`, `remove`); for a divert, `action` `divert`, `sysid`, `callsign`, `battery_pct`, `station`, `name` |
| `roi` | A region of interest is added, removed, or left where a drone crashed. | its centre | the region and `action` (`add`, `remove`, `crash`) |
| `survey` | A drone finishes circling a region of interest. | the drone | `sysid`, `callsign`, `depot`, `roi`, `name`, `duration_s` |

`krab-drones --reports <file>` writes every report to a file, one JSON object per line.

## Intents

The drones model handles these `Custom` intent codes. They are listed here for completeness; payloads and examples are in the [API guide](api-guide.md) and the pages linked.

| Code | Intent | See |
|---|---|---|
| 100 | Mission upload, `{"items": [...]}` | [Missions](#missions) |
| 101 | Mission clear | |
| 102 | Mission set current, `{"seq": n}` | |
| 103 | Command, `{"command": n, "params": [p1, ..., p7]}` (null for "not given") | [Commands](#commands) |
| 110–114 | Geofence open, close, set (a ground station's plan), extend, rename | [Geofences](geofences.md) |
| 120, 121 | Station add, remove | [Stations](stations.md) |
| 130, 131 | Region of interest add, remove | [Stations](stations.md) |
| 140 | Replace the wind field | [Weather](weather.md) |

Drone intents (100–103) need an `agent` id. Every intent's cell must be inside the world.

## Determinism

The drones model is built to replay bit for bit:

- Poses are integers, and all trigonometry goes through `libm`, so the same inputs give the same bits on every machine.
- Every random choice (where people and vehicles start, where they wander, vehicle types, orders, camera detections) comes from its own stream keyed by the seed, the entity and the tick. Nothing depends on the order in which things happen within a tick.
- Agents step in id order, on a single-threaded schedule, even in a build with krABMaga's `parallel` feature.
- Intents are applied at the start of a step and logged with the tick they were applied on. Live weather arrives as intent 140, so it is logged too.
- A behaviour tree starts afresh every tick and keeps nothing between ticks; what a drone is in the middle of lives in its state, which checkpoints keep.

This holds in both profiles. Run the pack in the `gold` profile to make use of it: the driver then hashes every published view and keeps the intent log, so `simctl replay` can check two runs agree and a run with ground stations and dashboard actions in it can be replayed. Checkpoints carry every agent's full state, the order queues, spares, fences, stations, regions and the wind, so a restored run carries on exactly as an uninterrupted one would.

## Running without the server

`krab-drones` runs a drones pack on krABMaga alone: no driver, server or gateways. It prints a summary, including the delivery range, at the end.

```sh
cargo run --release -p sim-drones --bin krab-drones -- scenarios/kaunas-deliveries --steps 36000
cargo run --release -p sim-drones --bin krab-drones -- scenarios/london-patrol --reports reports.jsonl
cargo run --release -p sim-drones --bin krab-drones -- scenarios/london-patrol --tui
```

`--steps` (default 10000) is per repetition, `--reps` repeats from scratch, and `--tui` runs under krABMaga's `simulate!` with its terminal monitor. `simctl fly <id> --hours 8` flies a pack through the driver for some simulated hours and writes its run log. Neither fetches weather.
