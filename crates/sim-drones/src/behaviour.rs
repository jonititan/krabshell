//! What a drone decides to do each tick, as a behaviour tree.
//!
//! Every drone has the same brain: one [bonsai](https://docs.rs/bonsai-bt)
//! behaviour tree, read from a JSON file. Its leaves are the drone's senses
//! and acts ([`Node`]): a sense succeeds or fails on what the drone can tell
//! about itself and the world (is it in the air, is its sensor due); an act
//! changes something (fly one tick, take an order, scan). The branches are
//! bonsai's own `Sequence`, `Select`, `If`, `Invert`, `AlwaysSucceed`,
//! `WhenAll`, `WhenAny`, `Race` and `After`, so changing what drones do is a
//! matter of rearranging the file, not the code.
//!
//! The tree is ticked from its root once per simulation step, for each drone
//! in turn, and starts afresh every step: nothing about where it got to is
//! carried from one step to the next. What a drone is in the middle of (a
//! delivery, a survey) lives in its own state, which checkpoints keep, so a
//! restored run decides exactly as an uninterrupted one. That also rules out
//! nodes that wait across steps (`Wait`, `WaitForever`, `Timeout`, `While`,
//! `WhileAll`): a brain using them is refused when the pack loads.
//!
//! The default brain is `behaviour/drone.json` in this crate. A pack can give
//! its own as `world/behaviour.json`:
//!
//! ```json
//! { "description": "What this brain does, for people.",
//!   "root": { "Sequence": [ { "Action": "ObeyCommands" }, { "Action": "Fly" } ] } }
//! ```

use bonsai_bt::{Behavior, Event, Float, Status, UpdateArgs, BT};
use serde::{Deserialize, Serialize};

/// The default brain, used when a pack has no `world/behaviour.json`.
pub const DEFAULT: &str = include_str!("../behaviour/drone.json");

/// A leaf of the tree: a sense (succeeds or fails, changes nothing) or an
/// act. Names are as they appear in the file, as `{"Action": "Fly"}`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Node {
    // ------------------------------------------------------------ senses
    /// Off the ground.
    Airborne,
    /// A courier: it delivers from a depot.
    IsCourier,
    /// The simulation flies it (couriers, spares, drones a pack starts on a
    /// mission), rather than a ground station.
    FlownBySimulation,
    /// A courier down and disarmed with the order it set out with.
    BackFromTrip,
    /// On its home pad with nothing to do, charged to the launch level and,
    /// for a courier, loaded: free to take on work.
    ReadyForWork,
    /// The camera takes a look this tick.
    SensorDue,
    /// On the ground at a launch and recovery station away from home.
    AtStation,
    /// The low-battery failsafe has fired on this charge.
    LowBattery,

    // -------------------------------------------------------------- acts
    /// Keep clear of the geofences, as a ground station would: reroute the
    /// mission, reposition out of a fence, send a courier whose address is
    /// fenced home. Queued with the commands it obeys this tick.
    AvoidGeofences,
    /// Coming in to land at a station whose slots are all taken: hold off
    /// until one is free, then carry on down; on its low-battery failsafe,
    /// land regardless. Always succeeds.
    WaitForSlot,
    /// Carry out the commands sent since the last tick, by ground stations
    /// and by its own `AvoidGeofences`.
    ObeyCommands,
    /// One tick of flight and battery: the autopilot flies the mission or
    /// mode it is in. Fails if the battery runs flat in the air (a crash).
    Fly,
    /// Back from a trip: hand an undelivered order back to the depot's
    /// queue (or turn it away, if a fence now covers it) and start loading.
    FinishTrip,
    /// Take the oldest order at its depot that it can reach, by way of
    /// stations to recharge at if it is too far for one charge. Fails if
    /// there is none.
    TakeOrder,
    /// Fly to the region of interest most overdue a look, circle it with
    /// the camera on and come home. Fails if none is due and in reach.
    Survey,
    /// Low on battery and too far from home: land at the nearest station in
    /// reach instead. Fails if home is nearer or no station is in reach.
    DivertToStation,
    /// Charged enough at a station away from home: fly home, by way of
    /// other stations if it must. Fails while it still needs to charge.
    ReturnToBase,
    /// Report each geofence it has flown into (or that opened round it).
    LogBreaches,
    /// Look at everything within sensor range, and report what it sees.
    Scan,
}

/// The file a brain is read from.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct BrainFile {
    #[serde(default)]
    pub description: String,
    pub root: Behavior<Node>,
}

/// One behaviour tree, shared by every drone in a world.
#[derive(Debug, Clone)]
pub struct Brain {
    pub description: String,
    pub root: Behavior<Node>,
    tree: BT<Node, ()>,
}

impl Brain {
    /// Read and check a brain from its JSON.
    pub fn from_value(value: &serde_json::Value) -> anyhow::Result<Brain> {
        let file = BrainFile::deserialize(value)
            .map_err(|e| anyhow::anyhow!("behaviour tree did not parse: {e}"))?;
        check(&file.root)?;
        let flies = count(&file.root, &|n| n == Some(Node::Fly));
        if flies != 1 {
            anyhow::bail!(
                "behaviour tree must fly the drone once a tick (one Fly node), not {flies} times"
            );
        }
        Ok(Brain {
            description: file.description,
            tree: BT::new(file.root.clone(), ()),
            root: file.root,
        })
    }

    /// The pack's `world/behaviour.json`, or the default.
    pub fn from_fields(fields: &serde_json::Value) -> anyhow::Result<Brain> {
        match fields.get("behaviour") {
            Some(value) => {
                Brain::from_value(value).map_err(|e| anyhow::anyhow!("world/behaviour.json: {e}"))
            }
            None => Brain::default_brain(),
        }
    }

    pub fn default_brain() -> anyhow::Result<Brain> {
        let value: serde_json::Value = serde_json::from_str(DEFAULT)?;
        Brain::from_value(&value).map_err(|e| anyhow::anyhow!("the default behaviour tree: {e}"))
    }

    /// How many nodes the tree has, branches and leaves.
    pub fn size(&self) -> usize {
        count(&self.root, &|_| true)
    }

    /// Tick the tree once from its root, `dt` seconds of simulated time.
    /// `leaf` answers each sense and carries out each act: true for success,
    /// false for failure, `None` for still running.
    pub fn tick(&mut self, dt: f64, mut leaf: impl FnMut(Node) -> Option<bool>) -> Status {
        self.tree.reset_bt();
        let event: Event = UpdateArgs { dt: dt as Float }.into();
        let outcome = self
            .tree
            .tick(&event, &mut |args, _: &mut ()| match leaf(*args.action) {
                Some(true) => (Status::Success, args.dt),
                Some(false) => (Status::Failure, args.dt),
                None => (Status::Running, 0.0),
            });
        outcome.map_or(Status::Failure, |(status, _)| status)
    }
}

/// Refuse nodes that wait across ticks: the tree starts afresh every tick.
fn check(b: &Behavior<Node>) -> anyhow::Result<()> {
    use Behavior::*;
    match b {
        Action(_) => Ok(()),
        Wait(_) | WaitForever | Timeout(..) | While(..) | WhileAll(..) => anyhow::bail!(
            "{} waits across ticks, but the tree is ticked afresh from its root every tick; \
             keep what a drone is in the middle of in its state, and test it with a sense",
            name(b)
        ),
        Invert(c) | AlwaysSucceed(c) => check(c),
        If(c, a, b) => {
            check(c)?;
            check(a)?;
            check(b)
        }
        Select(list)
        | Sequence(list)
        | MemorylessSequence(list)
        | MemorylessSelector(list)
        | WhenAll(list)
        | WhenAny(list)
        | After(list)
        | Race(list) => {
            if list.is_empty() {
                anyhow::bail!("{} has no children", name(b));
            }
            list.iter().try_for_each(check)
        }
    }
}

fn name(b: &Behavior<Node>) -> &'static str {
    use Behavior::*;
    match b {
        Wait(_) => "Wait",
        WaitForever => "WaitForever",
        Action(_) => "Action",
        Invert(_) => "Invert",
        AlwaysSucceed(_) => "AlwaysSucceed",
        Timeout(..) => "Timeout",
        Select(_) | MemorylessSelector(_) => "Select",
        If(..) => "If",
        Sequence(_) | MemorylessSequence(_) => "Sequence",
        While(..) => "While",
        WhileAll(..) => "WhileAll",
        WhenAll(_) => "WhenAll",
        WhenAny(_) => "WhenAny",
        After(_) => "After",
        Race(_) => "Race",
    }
}

/// Nodes for which `counts` holds: it sees each leaf's node, and `None`
/// for each branch.
fn count(b: &Behavior<Node>, counts: &impl Fn(Option<Node>) -> bool) -> usize {
    use Behavior::*;
    let here = usize::from(counts(match b {
        Action(n) => Some(*n),
        _ => None,
    }));
    let below = |list: &[Behavior<Node>]| list.iter().map(|x| count(x, counts)).sum::<usize>();
    here + match b {
        Action(_) | Wait(_) | WaitForever => 0,
        Invert(c) | AlwaysSucceed(c) | Timeout(_, c) => count(c, counts),
        If(c, a, e) => count(c, counts) + count(a, counts) + count(e, counts),
        While(c, list) | WhileAll(c, list) => count(c, counts) + below(list),
        Select(list)
        | Sequence(list)
        | MemorylessSequence(list)
        | MemorylessSelector(list)
        | WhenAll(list)
        | WhenAny(list)
        | After(list)
        | Race(list) => below(list),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn brain(root: serde_json::Value) -> anyhow::Result<Brain> {
        Brain::from_value(&serde_json::json!({ "root": root }))
    }

    #[test]
    fn the_default_brain_loads_and_flies_once() {
        let b = Brain::default_brain().unwrap();
        assert!(b.size() > 10);
        assert!(!b.description.is_empty());
    }

    #[test]
    fn the_whole_tree_runs_in_one_tick_in_order_and_again_from_the_root_the_next() {
        let mut b = brain(serde_json::json!({ "Sequence": [
            { "AlwaysSucceed": { "Sequence": [ { "Action": "Airborne" }, { "Action": "Scan" } ] } },
            { "Action": "Fly" },
            { "Select": [ { "Action": "TakeOrder" }, { "Action": "Survey" } ] },
        ]}))
        .unwrap();
        for _ in 0..2 {
            let mut seen = Vec::new();
            let status = b.tick(0.1, |n| {
                seen.push(n);
                Some(!matches!(n, Node::Airborne | Node::TakeOrder))
            });
            assert_eq!(status, Status::Success);
            assert_eq!(
                seen,
                [Node::Airborne, Node::Fly, Node::TakeOrder, Node::Survey]
            );
        }
    }

    #[test]
    fn brains_that_wait_across_ticks_or_do_not_fly_once_are_refused() {
        let fly = serde_json::json!({ "Action": "Fly" });
        assert!(brain(serde_json::json!({ "Sequence": [ { "Wait": 1.0 }, fly ] })).is_err());
        assert!(brain(serde_json::json!({ "Sequence": [ "WaitForever", fly ] })).is_err());
        assert!(brain(serde_json::json!({ "Action": "Scan" })).is_err());
        assert!(brain(serde_json::json!({ "Sequence": [ fly, fly ] })).is_err());
        assert!(brain(serde_json::json!({ "Action": "Teleport" })).is_err());
        assert!(brain(serde_json::json!({ "Sequence": [] })).is_err());
        assert!(brain(fly).is_ok());
    }
}
