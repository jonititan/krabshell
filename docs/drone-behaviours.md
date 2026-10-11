# Drone behaviour trees

The drones model uses [`bonsai-bt`](https://docs.rs/bonsai-bt) to decide what
each drone does. A behaviour is JSON: its branches control the order and
conditions of its leaves, and its leaves are the senses and actions listed
below. The default tree is
[`crates/sim-drones/behaviour/drone.json`](../crates/sim-drones/behaviour/drone.json).

## How a tree runs

Every drone in a world shares the same tree. At each simulation step, each
drone starts at the tree's root and evaluates it synchronously, top to bottom.
The tree's position is reset at the end of the step; it does not resume a
running branch on the next one. A drone's mission, order, survey, battery and
other work-in-progress live in its simulation state instead, so they survive
checkpoints.

Each leaf returns success or failure. Senses only report whether their
condition is true. Actions do work and can also fail when there is no
applicable work to do. A sequence stops at its first failure; a selector tries
the next child after a failure. Use `AlwaysSucceed` around optional sequences
so a false sense or unavailable action does not prevent later per-tick work
from running.

The tree must contain exactly one `Fly` action. Arrange the control flow so
that it is reached on every tick: a branch that fails before it can skip the
autopilot update for that drone. The validator checks that there is one `Fly`
in the JSON, but cannot guarantee that every possible branch reaches it.

Do not use `Wait`, `WaitForever`, `Timeout`, `While` or `WhileAll`: these
branches depend on state across ticks and are rejected when the drones world
is constructed. Every branch list must have at least one child. The tree is
reset even when a bonsai node would normally preserve its running child.

## JSON format and branch nodes

A behaviour file has a required `root` and an optional human-readable
`description`:

```json
{
  "description": "Obey the operator and advance the autopilot every tick.",
  "root": {
    "Sequence": [
      { "Action": "ObeyCommands" },
      { "Action": "Fly" }
    ]
  }
}
```

`Action` takes one action name as a string. Bonsai branch nodes use these JSON
forms:

| Node | JSON form | Behaviour |
|---|---|---|
| `Sequence` | `{"Sequence": [child, ...]}` | Runs children in order. Stops and fails on the first failure; succeeds when all succeed. |
| `Select` | `{"Select": [child, ...]}` | Tries children in order. Stops and succeeds on the first success; fails when all fail. |
| `If` | `{"If": [condition, then, else]}` | Runs `condition`, then runs `then` on success or `else` on failure. |
| `Invert` | `{"Invert": child}` | Swaps success and failure. |
| `AlwaysSucceed` | `{"AlwaysSucceed": child}` | Runs the child but reports success even if it fails. |
| `WhenAll` | `{"WhenAll": [child, ...]}` | Succeeds when all children succeed; fails when any child fails. |
| `WhenAny` | `{"WhenAny": [child, ...]}` | Succeeds when any child succeeds; fails when all children fail. |
| `After` | `{"After": [child, ...]}` | Runs children until all succeed in sequence; fails if any child fails. |
| `Race` | `{"Race": [child, ...]}` | Returns the result of the first child to finish. |

The drone leaves are synchronous, so prefer `Sequence`, `Select`, `If`,
`Invert`, and `AlwaysSucceed` for ordinary drone control. `WhenAll`,
`WhenAny`, `After`, and `Race` are accepted bonsai nodes, but do not make
actions run concurrently with each other in simulation time.

## Drone leaves

The strings below are case-sensitive. A sense answers a question without
changing the world. An action performs work; actions described as conditional
can fail when their prerequisite is not met.

### Senses

| Action name | Succeeds when |
|---|---|
| `Airborne` | The drone is off the ground: not on its pad, a station or the terrain. |
| `IsCourier` | It has courier state and the world has at least one depot. |
| `FlownBySimulation` | The simulation flies it, rather than a ground station: couriers, drones the pack starts with `"autostart": true`, spares once they are called up, and drones that replace a crashed one of these. It does not change when a ground station sends the drone commands (see [protocols.md](protocols.md#ground-stations-and-the-simulations-own-flying)). |
| `BackFromTrip` | It has a delivery order or survey in progress and is disarmed. |
| `ReadyForWork` | It is idle or disarmed on its home pad, charged to the configured launch level, and has no survey in progress. A courier must also have a depot, have finished its loading delay and have no order; any other drone must be one the simulation flies. |
| `SensorDue` | The configured sensor schedule calls for a scan on this tick. |
| `AtStation` | It is grounded at a launch/recovery station other than its home pad. |
| `LowBattery` | The low-battery failsafe has fired for the current charge. |
| `WindWithinLimits` | Wind and gusts at the takeoff height are within `max_wind_ms` and `max_gust_ms`. Without weather data, wind is calm. |

### Actions

| Action name | Effect and result |
|---|---|
| `AvoidGeofences` | Applies the simulation's keep-out response to a simulation-controlled drone: reroutes or repositions it, or sends a courier home if its destination is fenced. It queues work for `ObeyCommands`; succeeds. |
| `WaitForSlot` | Manages a station landing slot. Coming in to land at a station whose slots are all taken, the drone holds position until one is free; a drone on its low-battery failsafe lands regardless. Succeeds. |
| `ObeyCommands` | Applies the instructions queued since the last tick, from ground stations (over [MAVLink](protocols.md#mavlink) or the API) and from `AvoidGeofences`, including mission and flight commands; succeeds. |
| `Fly` | Advances the autopilot and battery by one simulation tick, and handles reached waypoints and landing events. A [conflict advisory](drones.md#conflict-advisories) being flown takes the place of the autopilot until it runs out. Fails if the drone runs out of battery in the air and crashes. |
| `FinishTrip` | Completes post-trip bookkeeping. It releases an unfinished survey, requeues an undelivered order (unless a geofence now rejects it), and starts courier loading; succeeds. |
| `TakeOrder` | Assigns the oldest eligible order at the drone's depot and starts its delivery mission. It may plan charging stops at stations. Fails if no order is available or the drone is not a courier. |
| `Survey` | Starts a mission to the most overdue reachable region of interest, circles it with the camera on, and returns. Fails when no region is due and reachable. |
| `DivertToStation` | When the low-battery failsafe is heading home but home is out of reach, sets a landing mission to the nearest reachable station. Fails if home is reachable, the drone is grounded, or no station is reachable. |
| `ReturnToBase` | At a station away from home, starts a mission home, planning charging stops when needed. Fails if the drone is busy or cannot plan a route home. |
| `LogBreaches` | Reports geofences the airborne drone has entered, including a fence that opens around it; succeeds. |
| `Scan` | Scans nearby objects with the camera and emits detection reports for sightings; succeeds. |

`TakeOrder` and `Survey` are alternatives in the default tree. A tree that
changes this choice should retain `ReadyForWork` and, when weather is enabled,
`WindWithinLimits` before dispatching work. Keep `AvoidGeofences`,
`ObeyCommands`, `WaitForSlot`, `Fly`, `LogBreaches`, and the scheduled camera
scan if the corresponding built-in handling should continue to run.

## A practical tree

This smaller tree keeps operator commands, geofence handling, station-slot
management, flight, and camera scans, but omits automatic delivery and survey
dispatch. Optional phases are wrapped so their failed senses do not block
`Fly` or later work:

```json
{
  "description": "Keep the operator-controlled flight loop and scheduled scans.",
  "root": {
    "Sequence": [
      {
        "AlwaysSucceed": {
          "Sequence": [
            { "Action": "FlownBySimulation" },
            { "Action": "AvoidGeofences" }
          ]
        }
      },
      { "Action": "ObeyCommands" },
      {
        "AlwaysSucceed": {
          "Sequence": [
            { "Action": "FlownBySimulation" },
            { "Action": "WaitForSlot" }
          ]
        }
      },
      { "Action": "Fly" },
      { "Action": "LogBreaches" },
      {
        "AlwaysSucceed": {
          "Sequence": [
            { "Action": "SensorDue" },
            { "Action": "Airborne" },
            { "Action": "Scan" }
          ]
        }
      }
    ]
  }
}
```

For a full delivery, failsafe, survey and sensor policy, start from the default
tree rather than this reduced example. A custom tree replaces the default for
every drone in the world; it does not add extra actions around it.

## Put a behaviour in a scenario pack

Add `world/behaviour.json` to a drones scenario pack. A pack without this file
uses the default tree. The JSON file contents are the behaviour object itself
(`description` and `root`); do not wrap them in a second `behaviour` property.
Scenario loading joins `world/behaviour.json` into the world's `behaviour`
field.

For example, a pack can have this layout:

```text
my-pack/
  manifest.toml
  config.toml
  params.toml
  world/
    agents.json
    behaviour.json
    fields.json
```

Ensure the `id` in `manifest.toml` is the same as the pack's directory/upload
id. Zip the *contents* of the pack directory so `manifest.toml` and `world/`
are at the archive root, not nested under `my-pack/`:

```sh
cd my-pack
zip -r ../my-pack.zip manifest.toml config.toml params.toml world/
```

### Upload in the dashboard

Open the server's **Scenarios** tab, enter the manifest id, select the zip and
upload. The new pack appears in the pack list; load it to replace the current
world and start the run from tick 0.

### Upload with the API

`POST /api/v1/scenarios` takes a multipart form with an `id` field and a `pack`
file field. When the server uses `DASHBOARD_TOKEN`, include its bearer token:

```sh
curl -fS -H "Authorization: Bearer $DASHBOARD_TOKEN" \
  -F 'id=my-pack' \
  -F 'pack=@my-pack.zip' \
  http://127.0.0.1:8080/api/v1/scenarios
```

Then load the uploaded pack with `POST /api/v1/scenarios/my-pack/load` (the
server's Scenarios tab also has a load control). Loading replaces the current
world. The archive is unpacked and the pack structure is checked on upload; the
behaviour tree itself is parsed and validated when the drones model is
constructed on load. A failed tree load reports the JSON, action-name,
unsupported-node, empty-branch, or `Fly`-count error that prevented it.

To check the pack without the server, put it in a scenario directory and run a
short local simulation:

```sh
cargo run --release -p sim-engine --bin simctl -- \
  --scenarios /path/to/packs run my-pack --ticks 10
```

The command constructs the model and therefore catches behaviour-tree errors.
It also starts the simulation, so use a low tick count for a quick load check.
