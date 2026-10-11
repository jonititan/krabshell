# MAVLink and Cursor-on-Target reference

krabshell presents the simulated drones to outside software over two UDP
links. The MAVLink gateway makes each drone look like a real vehicle to a
ground station such as QGroundControl, MAVSDK or pymavlink. The
Cursor-on-Target (CoT) emitter puts the drones and what they see on a TAK map.
This page lists exactly what each link sends, what it accepts and how it
answers. For a step-by-step setup, see [connecting.md](connecting.md).

Both run inside the `krabshell` process, beside the web server. Like every
other edge, they only read the published view of the simulation and submit
intents to it; neither touches model state or steps the schedule. The source is
[`crates/sim-mavlink`](../crates/sim-mavlink/src/lib.rs) and
[`crates/sim-cot`](../crates/sim-cot/src/lib.rs).

## Configuration

| Flag | Environment variable | Default | Meaning |
|---|---|---|---|
| `--mavlink-bind` | `KRABSHELL_MAVLINK_BIND` | `127.0.0.1:14551` | UDP address the gateway listens and sends from |
| `--mavlink-gcs` | `KRABSHELL_MAVLINK_GCS` | `127.0.0.1:14550` | Ground stations to send telemetry to before they have spoken. Comma-separated or repeated for several |
| `--mavlink-hz` | none | `5` | Position, attitude and HUD rate per drone, Hz. Clamped to 0.1–50 |
| `--no-mavlink` | none | off | Do not start the gateway |
| `--cot-dest` | `KRABSHELL_COT_DEST` | `239.2.3.1:6969` | Where CoT goes: a multicast group or a unicast `host:port` |
| `--cot-ttl` | none | `1` | Multicast TTL. Only applies when `--cot-dest` is multicast |
| `--cot-iface` | `KRABSHELL_COT_IFACE` | OS default | Local IPv4 address the CoT socket binds to and sends from |
| `--no-cot` | none | off | Do not start the emitter |

The flags are defined in
[`crates/sim-axum/src/main.rs`](../crates/sim-axum/src/main.rs). The CoT drone
rate (1 Hz) has no flag.

Neither protocol has authentication. Anything that can reach the MAVLink port
can arm and fly the drones, so krabshell logs a warning when `--mavlink-bind`
is not a loopback address. Only bind beyond loopback on a network you trust.

## MAVLink

### Vehicles and system ids

One UDP socket carries every drone. Each drone is its own MAVLink system: its
`sysid` from the scenario, component 1 (`MAV_COMP_ID_AUTOPILOT1`).
QGroundControl only creates a vehicle for heartbeats from component 1, and
uses the system id to tell vehicles apart on a shared link.

- A pack may set a drone's `sysid` from 1 to 250. A drone without one gets the
  lowest free id. Two drones with the same id stop the pack loading.
- A spare or replacement that takes over from a crashed drone gets the next
  unused id, and a callsign with the old number swapped for the new one
  (`MEGA-7` is followed by `MEGA-43`). Ids above 250 are never handed out.
- Messages addressed to system 0 (broadcast), or to a system id that is not one
  of the drones, are ignored. Another vehicle can share the link.

Every message goes out as MAVLink 2, with a separate sequence counter per
drone.

### Who receives what

Periodic messages (heartbeats and telemetry) go to every address in
`--mavlink-gcs`, and to every peer that has sent the gateway a packet in the
last 60 seconds. A ground station that speaks first is therefore served even
if it is not in `--mavlink-gcs`.

Replies (acks, mission items, parameter values, `STATUSTEXT` about a request)
go only to the peer that asked.

### Timing

The simulation runs as fast as it can and knows nothing of real time. The
gateway keeps the real-time half of the protocol itself, on the wall clock:

| Timer | Period | What happens |
|---|---|---|
| Heartbeat | 1 s | `HEARTBEAT`, `SYS_STATUS` and `MISSION_CURRENT` for every drone |
| Telemetry | 1 / `--mavlink-hz` (200 ms by default) | `GLOBAL_POSITION_INT`, `ATTITUDE`, `VFR_HUD` for every drone; `MISSION_CURRENT` when the current item, mission or fence plan changed; `MISSION_ITEM_REACHED` for items reached since the last pass |
| Poll | 20 ms | Collect answers from the simulation; re-request overdue upload items |
| Mission item timeout | 1.5 s | Ask again for the item an upload is waiting for |
| Upload retries | 5 | After five unanswered requests, give up with `MAV_MISSION_OPERATION_CANCELLED` |
| Ack wait | 1 s | How long to wait for the simulation to apply a mission, fence plan or command before answering anyway (below QGroundControl's 1.5 s retry) |
| Peer expiry | 60 s | Stop sending periodic messages to a silent peer not in `--mavlink-gcs` |

Each message shows the drone's state in the most recent published view. When
the simulation runs faster than real time, the drones move faster than real
time; nothing is queued or replayed. Reloading the scenario, resetting the run
or restoring an earlier state drops any half-finished upload, since the drone
it was for may no longer exist.

### Messages by direction

Sent by krabshell:

| Message | When | Notes |
|---|---|---|
| `HEARTBEAT` | 1 Hz per drone | `MAV_TYPE_QUADROTOR`, `MAV_AUTOPILOT_GENERIC`; mode flags and `custom_mode` as in [Modes](#modes) |
| `SYS_STATUS` | 1 Hz per drone | Battery remaining from the model; voltage a 4S pack scaled between 14.0 and 16.8 V; current unknown (-1); GPS, gyro, accelerometer and compass present and healthy |
| `MISSION_CURRENT` | 1 Hz, and at once on a change | `seq`, `total`, `mission_state`, `mission_id` (the drone's mission revision) and `fence_id` (the airspace revision) |
| `GLOBAL_POSITION_INT` | `--mavlink-hz` | `alt` above mean sea level, `relative_alt` above home; `vx`/`vy` along the track; `hdg` where the nose points, which in wind is not the track |
| `ATTITUDE` | `--mavlink-hz` | Yaw only; roll, pitch and rates are 0 |
| `VFR_HUD` | `--mavlink-hz` | Airspeed (through the wind), ground speed, altitude above sea level, climb rate, heading of the nose; throttle 50 more than 0.2 m above home, otherwise 0 |
| `MISSION_ITEM_REACHED` | When the simulation reports an item done | Also sent for a `DO_JUMP` taken |
| `HOME_POSITION` | On request | Answer to `MAV_CMD_REQUEST_MESSAGE` for message 242 |
| `AUTOPILOT_VERSION` | On request | Advertises `MISSION_INT`, `COMMAND_INT`, `MISSION_FENCE` and `MAVLINK2`. Without these QGroundControl treats a generic autopilot as MAVLink 1 only and never asks for the fence plan |
| `MISSION_COUNT`, `MISSION_ITEM_INT` | During a download | See [Mission protocol](#mission-protocol) |
| `MISSION_REQUEST_INT`, `MISSION_ACK` | During an upload | |
| `COMMAND_ACK` | Answer to a command | See [Commands](#commands) |
| `PARAM_VALUE` | Answer to a parameter request | See [Parameters](#parameters) |
| `STATUSTEXT` | With a refusal or warning | The reason, cut to 50 bytes |

Accepted from a ground station:

| Message | Handling |
|---|---|
| `HEARTBEAT` | Marks the sender as a peer. Nothing else |
| `MISSION_COUNT`, `MISSION_ITEM_INT` | Mission or fence plan upload |
| `MISSION_REQUEST_LIST`, `MISSION_REQUEST_INT`, `MISSION_REQUEST` | Mission or fence plan download |
| `MISSION_CLEAR_ALL` | Clear the mission, or the ground stations' fences |
| `MISSION_SET_CURRENT` | Set the current item. No ack; the next `MISSION_CURRENT` shows it |
| `MISSION_ACK` | The end of a download. Ignored |
| `COMMAND_LONG`, `COMMAND_INT` | See [Commands](#commands) |
| `SET_MODE` | Passed on as `DO_SET_MODE`, with `base_mode` as param1 and `custom_mode` as param2. No ack |
| `PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`, `PARAM_SET` | See [Parameters](#parameters) |

Any other message only marks its sender as a peer. A frame the MAVLink library
cannot parse is skipped, and the rest of the datagram is still read.

### Modes

A drone is always in one of eight modes. The heartbeat reports the mode as
`custom_mode`, and sets `base_mode` flags to match:

| `custom_mode` | Mode | `base_mode` flags beyond `CUSTOM_MODE_ENABLED` | What the drone does |
|---:|---|---|---|
| 0 | Disarmed | none | On the ground, motors off |
| 1 | Idle | `SAFETY_ARMED`, `STABILIZE_ENABLED` | On the ground, armed, waiting for a command |
| 2 | Takeoff | armed, `GUIDED_ENABLED` | Climbing straight up; holds (Loiter) on reaching the height |
| 3 | Auto | armed, `AUTO_ENABLED` | Flying the uploaded mission |
| 4 | Loiter | armed, `GUIDED_ENABLED` | Holding position in the air |
| 5 | Goto | armed, `GUIDED_ENABLED` | Flying to a `DO_REPOSITION` point; holds on arrival |
| 6 | RTL | armed, `AUTO_ENABLED` | Climbing to the return height, flying home, landing |
| 7 | Land | armed, `AUTO_ENABLED` | Descending where it is, then disarming |

`HEARTBEAT.system_status` is `MAV_STATE_ACTIVE` while armed and
`MAV_STATE_STANDBY` while disarmed. It is `MAV_STATE_CRITICAL` for every drone
if the simulation has faulted or its tick thread has stopped. A paused
simulation is not critical.

The custom mode numbers are for display. A ground station cannot select a
mode by its number: `DO_SET_MODE` and `SET_MODE` only look at the
`MAV_MODE_FLAG_AUTO_ENABLED` bit (4) of the base mode. With it set, the drone
starts its mission, as `MISSION_START` would. Without it, a drone in the air
holds position (Loiter), and one on the ground is left as it is. To land or
return home, use `NAV_LAND` or `NAV_RETURN_TO_LAUNCH`.

### Commands

`COMMAND_LONG` and `COMMAND_INT` are handled alike. For `COMMAND_INT`, `x` and
`y` become param5 and param6 in degrees and `z` becomes param7; the frame is
not read.

The gateway answers three commands itself:

| Command | Id | Answer |
|---|---:|---|
| `REQUEST_MESSAGE` | 512 | For message 148 (`AUTOPILOT_VERSION`) or 242 (`HOME_POSITION`): `ACCEPTED`, then the message. Anything else: `UNSUPPORTED` |
| `REQUEST_AUTOPILOT_CAPABILITIES` | 520 | `ACCEPTED`, then `AUTOPILOT_VERSION` |
| `SET_MESSAGE_INTERVAL` | 511 | `ACCEPTED`, but the rates do not change. They are the gateway's own |

These go to the simulation, as a `COMMAND` intent (code 103) for the drone:

| Command | Id | Parameters | Refused when |
|---|---:|---|---|
| `COMPONENT_ARM_DISARM` | 400 | param1 ≥ 0.5 arms, otherwise disarms; param2 21196 forces a disarm in flight | Arming below the battery reserve; disarming in the air without the force code |
| `NAV_TAKEOFF` | 22 | param7 height above where it stands (pad or ground), metres; 0 or absent uses the pack's `takeoff_alt_m` | Not armed; on the ground below the launch charge; launch wind or gusts above the pack's limits |
| `MISSION_START` | 300 | param1 first item (from 1; otherwise it carries on from the current item, or restarts a finished mission) | Not armed; no mission |
| `NAV_RETURN_TO_LAUNCH` | 20 | none | On the ground |
| `NAV_LAND` | 21 | none; lands where it is | On the ground |
| `DO_SET_MODE` | 176 | param1 base mode flags; see [Modes](#modes) | As `MISSION_START`, when it starts the mission |
| `DO_PAUSE_CONTINUE` | 193 | param1 1 continues (as `MISSION_START`), 0 pauses (Loiter) | Pausing on the ground |
| `DO_REPOSITION` | 192 | param5 latitude, param6 longitude (degrees), param7 altitude above home (absent: keep the current height) | On the ground; no position; a position outside the world |
| `DO_SET_MISSION_CURRENT` | 224 | param1 item | Past the end of the mission |

Any other command is answered `MAV_RESULT_UNSUPPORTED`.

The answer to a simulation command:

| Outcome | `COMMAND_ACK.result` |
|---|---|
| Applied within 1 s | `ACCEPTED` |
| Refused by the model | `DENIED`, with a `STATUSTEXT` warning giving the reason |
| Not applied within 1 s | `TEMPORARILY_REJECTED` |
| Simulation paused | `TEMPORARILY_REJECTED`, at once, with the `STATUSTEXT` "Simulation is paused" |
| Not submitted (no such drone, the drone's cell outside the world, or the sender over its rate limit) | `FAILED`, with a `STATUSTEXT` giving the reason |

Every intent the gateway submits is made by the actor `mavlink:<ip>:<port>`
of the sender, and counts against that actor's limit
(`run.intent_rate_per_actor` in the pack's `config.toml`). The intents show in
the dashboard's Intents log under that name.

### Mission protocol

Missions use the `_INT` messages with `mission_type`
`MAV_MISSION_TYPE_MISSION`. The legacy `MISSION_REQUEST` is answered as
`MISSION_REQUEST_INT` for downloads.

**Upload.** The ground station sends `MISSION_COUNT`. The gateway asks for each
item in turn with `MISSION_REQUEST_INT` and holds them until the last arrives.
A duplicate or out-of-order item is not taken; the reply is a request for the
item still needed. The whole mission then goes to the simulation as one
`MISSION_UPLOAD` intent (code 100), so a drone never flies half a plan.
`MISSION_ACK` follows once a published view carries the new mission, so a
download straight after the ack returns it.

| Upload outcome | `MISSION_ACK` type |
|---|---|
| Applied | `MAV_MISSION_ACCEPTED` |
| Not applied within 1 s (simulation paused or busy) | `MAV_MISSION_ACCEPTED`, with the `STATUSTEXT` "Mission queued: simulation paused". It applies on the next tick. If the world is reset, reloaded or restored before then, it never applies, and an error `STATUSTEXT` "Mission not applied: world reset" says so (a fence plan, "Fence plan not applied: …") |
| `count` above 4096 | `MAV_MISSION_NO_SPACE`, before any item is requested |
| More items than the pack's `max_mission_items` (default 128, at most 4096) | `MAV_MISSION_NO_SPACE` |
| An unsupported frame | `MAV_MISSION_UNSUPPORTED_FRAME` |
| A position outside the world | `MAV_MISSION_DENIED` |
| Any other refusal, or the intent could not be submitted | `MAV_MISSION_ERROR` |
| Five requests for one item unanswered | `MAV_MISSION_OPERATION_CANCELLED` |

Every refusal comes with a `STATUSTEXT` error giving the reason, such as "item
3 is outside the world".

**Download.** `MISSION_REQUEST_LIST` gets `MISSION_COUNT` with the mission's
revision as `opaque_id`. Each `MISSION_REQUEST_INT` (or `MISSION_REQUEST`) gets
that `MISSION_ITEM_INT`, with `current` set on the item the drone is at. A
request past the end gets `MISSION_ACK` `MAV_MISSION_INVALID_SEQUENCE`. Items
come back exactly as uploaded, including NaN parameters.

**Clear.** `MISSION_CLEAR_ALL`, or an upload of zero items, empties the mission
(`MISSION_CLEAR`, code 101). A drone flying it holds position.

**Current item.** `MISSION_SET_CURRENT` and `DO_SET_MISSION_CURRENT` move the
drone to an item (`MISSION_SET_CURRENT`, code 102). A drone in Auto flies to
it.

Each upload or clear bumps the drone's mission revision, which appears as
`MISSION_CURRENT.mission_id` and as the `opaque_id` of the count and ack.

#### Frames

| Frame | Ids | Altitude means |
|---|---|---|
| `GLOBAL`, `GLOBAL_INT` | 0, 5 | Above mean sea level |
| `GLOBAL_RELATIVE_ALT`, `GLOBAL_RELATIVE_ALT_INT` | 3, 6 | Above home (the drone's launch pad, which may be a rooftop). QGroundControl's default |
| `GLOBAL_TERRAIN_ALT`, `GLOBAL_TERRAIN_ALT_INT` | 10, 11 | Above the terrain under the drone, so the leg follows the ground |
| `MISSION` | 2 | Only for items that do not fly somewhere |

Any other frame refuses the upload. A position of 0, 0 means "here", as
MAVLink allows for take-off and landing. A NaN altitude keeps the current
height. `autocontinue` is stored and returned, but every item continues.

#### Items

| Item | Id | What the drone does |
|---|---:|---|
| `NAV_TAKEOFF` | 22 | Climbs straight up to the item's altitude. On the ground it first waits for the pack's `launch_battery_pct`, or param2 percent if that is higher and it is on a charger (a krabshell extension) |
| `NAV_WAYPOINT` | 16 | Flies to the point. param1 hold time, seconds; param2 acceptance radius, metres (default 1) |
| `NAV_LOITER_UNLIM` | 17 | Flies to the point and holds until the item or mode is changed |
| `NAV_LOITER_TIME` | 19 | Flies to the point and holds for param1 seconds |
| `NAV_RETURN_TO_LAUNCH` | 20 | Switches to RTL: climbs to at least `rtl_alt_m` above home, flies home, lands, disarms |
| `NAV_LAND` | 21 | Flies level to the point (0, 0: where it is), then descends onto the home pad within 10 m of home, or onto the terrain elsewhere. Stays armed if more items follow; disarms if it is the last |
| `CONDITION_DELAY` | 112 | Holds position for param1 seconds |
| `DO_JUMP` | 177 | Jumps to item param1, param2 times (−1: forever) |
| `DO_CHANGE_SPEED` | 178 | Sets cruise speed to param2 metres per second, if above 0 |
| `DO_WINCH` | 42600 | Holds position while the winch runs. param2 `WINCH_DELIVER` (4) lowers from the drone's height, releases, reels up and locks, timed by the pack's `winch_*` params; `WINCH_RELATIVE_LENGTH_CONTROL` (1) takes \|param3\| metres at \|param4\| m/s. Other actions complete at once |

Any other command (camera, gimbal, servo) completes at once, so a mission
containing one still runs. At most 16 items that take no time run in one tick,
so a loop of `DO_JUMP`s cannot stall the simulation.

#### Fence plan

The geofences are served as a fence plan (`MAV_MISSION_TYPE_FENCE`), the way
ArduPilot and PX4 hold one. Every drone serves the same plan, and a plan
uploaded to any drone applies to all of them. See [geofences.md](geofences.md)
for what fences do in the simulation.

- **Download.** `MISSION_REQUEST_LIST` with the fence type returns every open
  geofence, whoever opened it: a `NAV_FENCE_CIRCLE_EXCLUSION` (5004) per
  circle, with the radius in param1, and a
  `NAV_FENCE_POLYGON_VERTEX_EXCLUSION` (5002) per polygon corner, each with the
  corner count in param1. The frame is `GLOBAL`. Altitude bands are not
  carried. The plan is fixed when the download starts, so a fence opening
  partway through does not mix two plans. The count's `opaque_id`, and
  `MISSION_CURRENT.fence_id`, are the airspace revision, which changes
  whenever a fence opens or closes.
- **Upload.** A fence plan uploads like a mission and becomes one
  `GEOFENCE_SET` intent (code 112). It replaces the fences ground stations set
  before. Fences in the plan that match one the scenario, the dashboard or
  another intent opened are left as they are, so a plan downloaded and sent
  back unchanged changes nothing, and deleting one of those in the ground
  station does not close it. Uploaded fences run from the surface up, with no
  ceiling. A return point (5000) is ignored. Inclusion fences (5001, 5003) are
  not simulated, and a plan with one is refused with
  `MAV_MISSION_UNSUPPORTED`. A plan the model refuses, such as one with a
  fence over a drone's pad, gets `MAV_MISSION_DENIED` and a `STATUSTEXT`.
- **Clear.** `MISSION_CLEAR_ALL` with the fence type, or an empty upload,
  closes every fence ground stations set.

#### Rally points

Rally points are not simulated. A rally download has zero items. An empty
rally upload, or clearing rally points, is accepted; a non-empty rally upload
gets `MAV_MISSION_UNSUPPORTED`.

### Parameters

Each drone exposes one read-only parameter, `SYSID_THISMAV` (a `UINT8` set to
its system id), so a ground station's parameter download finishes rather than
timing out. `PARAM_REQUEST_LIST` and `PARAM_REQUEST_READ` return it.
`PARAM_SET` changes nothing and echoes it back.

### Ground stations and the simulation's own flying

There is no takeover or handback. Whether the simulation flies a drone is
fixed by the pack: couriers, spares, and drones that start with
`"autostart": true` are flown by the simulation's operator; every other drone
is left to ground stations. A ground station's commands are obeyed whichever
it is. The operator does not stop flying a drone because a ground station has
sent it something, and carries on routing it round geofences (see
[drones.md](drones.md) and [drone-behaviours.md](drone-behaviours.md)). The
operator's own route changes are logged as reroute reports, not sent over
MAVLink. A drone a ground station flies into a fence is not rerouted; the
breach is reported.

Conflict advisories apply to every drone in the air, a ground station's too
(see [Advisories are flown](conflicts.md#advisories-are-flown)). A drone
advised flies its climb, descent, turn or speed change for the pack's
`advisory_hold_s`, longer while the aircraft it avoids is still closing, and
then carries on in its mode from where it is. The heartbeat's mode does not
change while it does, and nothing about the advisory is sent over MAVLink: a
ground station sees only the drone leave its track or height. Commands sent
meanwhile are still taken, and the drone acts on them once the advisory is
over. A pack turns advisories off with `atc = false`.

## Cursor-on-Target

### What is sent

Each event is one CoT XML document (`<event version="2.0">`, legacy XML, TAK
protocol version 0) in its own UDP datagram, which every TAK client accepts.
CoT is one-way: the emitter sends and never listens.

| Event | `type` | `uid` | `how` | Rate | Stale after |
|---|---|---|---|---|---|
| A drone | `a-f-A-M-H-Q` (friendly rotary-wing UAV) | `krabshell-uav-<sysid>` | `m-g` | Once a second | 10 s |
| A manned aircraft on its route | `a-n-A-C-F` (neutral civil fixed-wing) | `krabshell-ac-<id>` | `m-g` | Once a second | 10 s |
| A person a drone saw | `a-u-G-U-C-I` | `krabshell-det-<id>` | `m-s` | Each sighting report | 30 s |
| A vehicle a drone saw | `a-u-G-E-V` | `krabshell-det-<id>` | `m-s` | Each sighting report | 30 s |
| An intruder drone a drone saw | `a-u-A-M-H-Q` | `krabshell-det-<id>` | `m-s` | Each sighting report | 30 s |
| Any other kind of target | `a-u-G` | `krabshell-det-<id>` | `m-s` | Each sighting report | 30 s |
| A point of interest | `b-m-p-s-m` (spot marker) | `krabshell-poi-<id>` | `h-g-i-g-o` | Every 15 s | 10 minutes |

`<id>` is the simulation's agent id of the thing seen. A sighting of a point of
interest refreshes its spot marker (same uid and type, 10-minute stale).
Intruders appear only when a drone sees them. The model decides how often a
sighting is reported: at most once per `report_interval_s` for the same target
and drone (15 s in `london-patrol`). New reports are picked up every 200 ms.
Only sighting (detection) reports go out as CoT; landings, breaches, crashes and
the rest are in the dashboard and `GET /api/v1/reports`.

Manned aircraft (the drones model's `aircraft` kind) are sent with the drones,
at the same rate and stale time, while they fly their route. One parked
between flights sends nothing, and its last marker goes stale.

Every uid starts with `krabshell-`, so the markers never collide with real
ones. Two krabshell processes sending to the same place use the same uids,
and a TAK client shows both worlds as one.

### Event contents

- **Times.** `time`, `start` and `stale` are wall-clock UTC with milliseconds.
  TAK clients age markers against their own clock, so events never arrive
  already stale, even when the simulation runs far faster than real time. The
  simulation tick goes in the remarks.
- **Point.** `lat` and `lon` to seven decimal places. `hae` is the simulation's
  altitude above mean sea level, standing in for height above the ellipsoid
  (no geoid separation is applied). `ce`/`le` are 5 m for drones, 10 m for
  manned aircraft and sightings, and 1 m for points of interest.
- **`<contact callsign>`.** The drone's callsign; a manned aircraft's
  callsign, or "Aircraft 42"; a sighting's callsign, or
  "Person 42", "Vehicle 42", "Unknown UAV 42"; a point of interest's name.
- **`<track course speed>`.** Degrees true and metres per second. Drones send
  their ground track and speed, and so do manned aircraft. Sightings send
  them when the report has them.
- **`<link relation="p-p">`.** On a sighting, the uid and type of the drone that
  made it.
- **`<remarks>`.** For a drone: callsign, mode, battery, mission item and
  simulation tick, as in `HAWK-1 · auto · battery 100% · mission item 2/6 ·
  sim tick 5310`. For a manned aircraft: `ARR26 · manned aircraft · sim tick
  5310`. For a sighting: `Seen by HAWK-1 at 38 m · sim tick 5310`.

A sighting as sent (wrapped here for reading):

```xml
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<event version="2.0" uid="krabshell-det-42" type="a-u-G-U-C-I" how="m-s"
  time="2026-09-24T10:34:56.000Z" start="2026-09-24T10:34:56.000Z"
  stale="2026-09-24T10:35:26.000Z">
  <point lat="51.5072000" lon="-0.1276000" hae="0.0" ce="10.0" le="10.0"/>
  <detail>
    <contact callsign="Person 42"/>
    <track course="90.0" speed="1.40"/>
    <link uid="krabshell-uav-1" type="a-f-A-M-H-Q" relation="p-p"/>
    <remarks>Seen by HAWK-1 at 38 m · sim tick 5310</remarks>
  </detail>
</event>
```

### Destination

By default events go to `239.2.3.1:6969`, the situational-awareness multicast
group ATAK and WinTAK listen on. With a multicast destination the TTL is
`--cot-ttl` (1 keeps it on the local network) and multicast loopback is on, so
a listener on the same machine receives it. Any unicast `host:port` works too:
a UDP input added in a TAK client, or a TAK Server's plain-CoT UDP input.
krabshell does not speak TAK Server's TLS streaming protocol. The socket is
IPv4.

`--cot-iface` binds the socket to one local IPv4 address, to choose which
network the events leave on when the machine has several.

## Checking the links

[`tools/mavlink_check.py`](../tools/mavlink_check.py) lists the drones, flies
one round a square and lands it. [`tools/cot_listen.py`](../tools/cot_listen.py)
prints each CoT event as it arrives. Their use is described in
[connecting.md](connecting.md#checking-without-qgroundcontrol-or-atak). The
gateway's own end-to-end tests, which double as worked examples of each
exchange, are in
[`crates/sim-mavlink/tests/gateway.rs`](../crates/sim-mavlink/tests/gateway.rs)
and [`crates/sim-cot/tests/emitter.rs`](../crates/sim-cot/tests/emitter.rs).
