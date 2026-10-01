//! End-to-end behaviour of the tick thread.
//!
//! These drive a real `Driver` over its real channels, because the properties
//! that matter — a paused world stays put, a panicking model does not take the
//! process with it, an intent lands on the next tick — only exist once the
//! thread, the queues and the published view are all in play.

use std::time::{Duration, Instant};

use sim_core::config::Param;
use sim_core::{
    ActorId, CellBag, Config, GeoCell, Intent, IntentVerb, Profile, RejectReason, Report,
    ReportKind, RunMode, ScenarioBlob, ScenarioManifest,
};
use sim_engine::model::{Model, Truncation};
use sim_engine::{Control, Driver, DriverHandle, ModelRegistry, Submission};
use uuid::Uuid;

const RESOLUTION: i32 = 6;

fn scenario(profile: Profile, walkers: i64) -> ScenarioBlob {
    let manifest = ScenarioManifest {
        id: "driver-test".into(),
        name: "Driver test".into(),
        description: String::new(),
        model: "walkers".into(),
        engine_rev: None,
        resolution: RESOLUTION,
        root: None,
        profile,
    };
    let mut config = Config::default();
    manifest.apply_to(&mut config);
    config.run.publish_hz = 200.0;
    config.model.insert("walkers".into(), Param::Int(walkers));
    config.model.insert("spread_k".into(), Param::Int(2));
    config
        .model
        .insert("origin_lon".into(), Param::Float(-0.1276));
    config
        .model
        .insert("origin_lat".into(), Param::Float(51.5072));
    ScenarioBlob {
        manifest,
        config,
        agents: Vec::new(),
        fields: serde_json::Value::Null,
    }
}

fn london() -> GeoCell {
    GeoCell::from_lonlat(-0.1276, 51.5072, RESOLUTION).unwrap()
}

/// Poll the published view until `predicate` holds, or give up.
fn wait_for(handle: &DriverHandle, what: &str, predicate: impl Fn(&sim_engine::Published) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        if predicate(&handle.view()) {
            return;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    panic!(
        "timed out waiting for {what}; view = {:?}",
        handle.view().meta
    );
}

#[test]
fn a_fresh_driver_is_paused_at_tick_zero() {
    let driver =
        Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 50)).unwrap();
    let handle = driver.handle();

    let view = handle.view();
    assert_eq!(view.meta.tick, 0);
    assert_eq!(view.health.mode, RunMode::Paused);
    assert!(view.health.alive);
    assert_eq!(view.meta.agent_count, 50);

    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(
        handle.view().meta.tick,
        0,
        "a paused world must not advance"
    );
}

#[test]
fn stepping_advances_exactly_the_requested_number_of_ticks() {
    let driver =
        Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 20)).unwrap();
    let handle = driver.handle();

    handle.command(Control::Step { n: 25 }).unwrap();
    wait_for(&handle, "25 ticks", |v| v.meta.tick == 25);

    std::thread::sleep(Duration::from_millis(50));
    let view = handle.view();
    assert_eq!(view.meta.tick, 25, "stepping stops where it was told to");
    assert_eq!(view.health.mode, RunMode::Paused);
}

#[test]
fn start_runs_and_pause_stops_it_where_it_is() {
    let driver =
        Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 20)).unwrap();
    let handle = driver.handle();

    handle.command(Control::Start { target_tps: None }).unwrap();
    wait_for(&handle, "the world to advance", |v| v.meta.tick > 10);

    handle.command(Control::Pause).unwrap();
    let stopped_at = handle.view().meta.tick;
    std::thread::sleep(Duration::from_millis(60));
    assert_eq!(handle.view().meta.tick, stopped_at);
    assert!(handle.view().metrics.sim_tps > 0.0);
}

#[test]
fn a_rate_limited_run_is_slower_than_the_engine_can_go() {
    let driver =
        Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 10)).unwrap();
    let handle = driver.handle();

    handle
        .command(Control::Start {
            target_tps: Some(50.0),
        })
        .unwrap();
    std::thread::sleep(Duration::from_millis(400));
    handle.command(Control::Pause).unwrap();

    let ticks = handle.view().meta.tick;
    assert!(
        (1..=60).contains(&ticks),
        "50 tps over ~0.4s should be tens of ticks, got {ticks}"
    );
}

#[test]
fn an_intent_lands_on_the_next_tick_and_is_visible_in_the_view() {
    let driver = Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 0)).unwrap();
    let handle = driver.handle();
    let cell = london();

    let (ack_tx, ack_rx) = crossbeam_channel::bounded(1);
    handle
        .submit(Submission {
            intent: Intent::operator(cell, IntentVerb::Spawn, serde_json::json!({"count": 4})),
            ack: Some(ack_tx),
        })
        .unwrap();

    handle.command(Control::Step { n: 1 }).unwrap();
    wait_for(&handle, "the spawn to commit", |v| v.meta.agent_count == 4);

    let ack = ack_rx.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(ack, sim_engine::IntentAck::Applied { tick: 1 });

    let view = handle.view();
    let bag = view.bag(cell).expect("the target cell is occupied");
    assert_eq!(bag.objects.len(), 4);
    assert_eq!(view.metrics.intents_applied, 1);
    assert!(view.recent_intents.iter().any(|r| r.rejected.is_none()));
}

#[test]
fn a_stale_intent_is_rejected_with_a_reason() {
    let driver = Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 0)).unwrap();
    let handle = driver.handle();

    handle.command(Control::Step { n: 5 }).unwrap();
    wait_for(&handle, "5 ticks", |v| v.meta.tick == 5);

    let (ack_tx, ack_rx) = crossbeam_channel::bounded(1);
    handle
        .submit(Submission {
            intent: Intent {
                tick: Some(2),
                ..Intent::operator(london(), IntentVerb::Spawn, serde_json::Value::Null)
            },
            ack: Some(ack_tx),
        })
        .unwrap();

    let ack = ack_rx.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(
        ack,
        sim_engine::IntentAck::Rejected(RejectReason::Stale {
            target: 2,
            committed: 5
        })
    );
    assert_eq!(handle.view().meta.agent_count, 0);
}

#[test]
fn an_intent_the_model_refuses_comes_back_rejected() {
    let driver = Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 0)).unwrap();
    let handle = driver.handle();

    let (ack_tx, ack_rx) = crossbeam_channel::bounded(1);
    handle
        .submit(Submission {
            intent: Intent::operator(london(), IntentVerb::Custom(3), serde_json::Value::Null),
            ack: Some(ack_tx),
        })
        .unwrap();
    handle.command(Control::Step { n: 1 }).unwrap();

    let ack = ack_rx.recv_timeout(Duration::from_secs(5)).unwrap();
    assert!(matches!(
        ack,
        sim_engine::IntentAck::Rejected(RejectReason::Rejected { .. })
    ));
    // The ack is sent mid-tick; the counter reaches readers with the next publish.
    wait_for(&handle, "the rejection to be counted", |v| {
        v.metrics.intents_rejected == 1
    });
}

#[test]
fn a_checkpoint_restores_to_the_same_tick_and_hash() {
    let driver =
        Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Gold, 80)).unwrap();
    let handle = driver.handle();

    handle.command(Control::Step { n: 40 }).unwrap();
    wait_for(&handle, "40 ticks", |v| v.meta.tick == 40);

    let checkpoint = handle.checkpoint().unwrap();
    assert_eq!(checkpoint.meta.tick, 40);
    assert!(checkpoint.meta.hash.is_some());
    sim_engine::checkpoint::verify(&checkpoint).unwrap();

    // A separate driver, wound forward differently, must land on the same world.
    let other = Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Gold, 5)).unwrap();
    let other_handle = other.handle();
    other_handle.command(Control::Step { n: 3 }).unwrap();
    wait_for(&other_handle, "3 ticks", |v| v.meta.tick == 3);

    other_handle
        .command(Control::Restore(checkpoint.clone()))
        .unwrap();
    wait_for(&other_handle, "the restore", |v| v.meta.tick == 40);

    let restored = other_handle.view();
    assert_eq!(restored.meta.agent_count, checkpoint.meta.agent_count);
    assert_eq!(restored.meta.hash, checkpoint.meta.hash);
}

#[test]
fn resetting_returns_the_world_to_tick_zero() {
    let driver =
        Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 30)).unwrap();
    let handle = driver.handle();

    handle.command(Control::Step { n: 10 }).unwrap();
    wait_for(&handle, "10 ticks", |v| v.meta.tick == 10);

    handle.command(Control::Reset).unwrap();
    let view = handle.view();
    assert_eq!(view.meta.tick, 0);
    assert_eq!(view.meta.agent_count, 30);
    assert_eq!(view.health.mode, RunMode::Paused);
}

#[test]
fn a_config_change_that_needs_a_reset_rebuilds_the_world() {
    let driver =
        Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 30)).unwrap();
    let handle = driver.handle();
    handle.command(Control::Step { n: 5 }).unwrap();
    wait_for(&handle, "5 ticks", |v| v.meta.tick == 5);

    // Publish rate is cosmetic: the world keeps running.
    let mut cosmetic = handle.view().config.clone();
    cosmetic.run.publish_hz = 5.0;
    handle
        .command(Control::SetConfig(Box::new(cosmetic)))
        .unwrap();
    assert_eq!(handle.view().meta.tick, 5);

    // Resolution changes what a cell id means, so the world must be rebuilt.
    let mut structural = handle.view().config.clone();
    structural.grid.resolution = 7;
    handle
        .command(Control::SetConfig(Box::new(structural)))
        .unwrap();
    let view = handle.view();
    assert_eq!(view.meta.tick, 0);
    assert_eq!(view.meta.resolution, 7);
}

#[test]
fn a_bad_config_is_refused_without_disturbing_the_run() {
    let driver =
        Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 10)).unwrap();
    let handle = driver.handle();
    handle.command(Control::Step { n: 3 }).unwrap();
    wait_for(&handle, "3 ticks", |v| v.meta.tick == 3);

    let mut invalid = handle.view().config.clone();
    invalid.run.publish_hz = 0.0;
    let err = handle
        .command(Control::SetConfig(Box::new(invalid)))
        .unwrap_err();
    assert!(matches!(err, sim_engine::DriverError::Refused(_)));
    assert_eq!(handle.view().meta.tick, 3);
    assert_eq!(handle.view().config.run.publish_hz, 200.0);
}

/// A model whose step always panics, to exercise the fault path.
struct Exploding;

impl Model for Exploding {
    fn kind(&self) -> &'static str {
        "exploding"
    }
    fn reset(&mut self, _blob: &ScenarioBlob) -> anyhow::Result<()> {
        Ok(())
    }
    fn apply_intent(&mut self, _intent: &Intent) -> Result<(), RejectReason> {
        Ok(())
    }
    fn step(&mut self, tick: u64) {
        panic!("model exploded on tick {tick}");
    }
    fn snapshot(&self, out: &mut Vec<CellBag>, _limit: usize) -> Truncation {
        out.clear();
        Truncation::default()
    }
    fn agent_count(&self) -> u64 {
        0
    }
    fn occupied_cells(&self) -> u64 {
        0
    }
    fn rng_state(&self) -> u64 {
        0
    }
    fn set_rng_state(&mut self, _state: u64) {}
    fn load_cells(&mut self, _cells: Vec<CellBag>) -> anyhow::Result<()> {
        Ok(())
    }
}

#[test]
fn a_panicking_model_faults_the_driver_without_killing_the_thread() {
    let mut registry = ModelRegistry::with_builtins();
    registry.register("exploding", |_| Ok(Box::new(Exploding)));

    let mut blob = scenario(Profile::Fast, 0);
    blob.manifest.model = "exploding".into();

    let driver = Driver::spawn(registry, blob).unwrap();
    let handle = driver.handle();

    handle.command(Control::Start { target_tps: None }).unwrap();
    wait_for(&handle, "the fault", |v| v.health.mode == RunMode::Faulted);

    let view = handle.view();
    assert!(view.health.alive, "the thread survives a model panic");
    assert_eq!(view.health.last_good_tick, 0);
    assert!(view
        .health
        .fault
        .as_deref()
        .unwrap_or_default()
        .contains("exploded"));

    // The driver refuses to run again until an operator resets it.
    assert!(handle.command(Control::Start { target_tps: None }).is_err());

    // A reset clears the fault and the world is usable again.
    handle.command(Control::Reset).unwrap();
    assert_eq!(handle.view().health.mode, RunMode::Paused);
    assert!(handle.view().health.fault.is_none());
}

#[test]
fn the_view_reports_itself_dead_once_the_driver_stops() {
    let driver = Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 5)).unwrap();
    let handle = driver.handle();
    assert!(handle.health().alive);

    driver.shutdown();
    assert!(
        !handle.health().alive,
        "the edge must be able to answer 503"
    );
}

#[test]
fn intents_beyond_the_queue_are_rejected_rather_than_blocking_the_caller() {
    let mut blob = scenario(Profile::Fast, 0);
    blob.config.run.max_pending_intents = 4;
    let driver = Driver::spawn(ModelRegistry::with_builtins(), blob).unwrap();
    let handle = driver.handle();

    // The driver is paused, so nothing drains: the channel fills and then says no.
    let mut rejections = 0;
    for _ in 0..200 {
        let intent = Intent {
            id: Uuid::new_v4(),
            ..Intent::operator(london(), IntentVerb::Spawn, serde_json::Value::Null)
        };
        if let Err(RejectReason::Backpressured { .. }) =
            handle.submit(Submission::fire_and_forget(intent))
        {
            rejections += 1;
        }
    }
    assert!(rejections > 0, "a full queue must push back");
}

#[test]
fn an_operator_intent_from_a_different_actor_is_recorded_for_the_dashboard() {
    let driver = Driver::spawn(ModelRegistry::with_builtins(), scenario(Profile::Fast, 0)).unwrap();
    let handle = driver.handle();

    let mut intent = Intent::operator(london(), IntentVerb::Spawn, serde_json::json!({"count": 1}));
    intent.actor = ActorId("player-9".into());
    handle.submit(Submission::fire_and_forget(intent)).unwrap();
    handle.command(Control::Step { n: 1 }).unwrap();
    wait_for(&handle, "the spawn", |v| v.meta.agent_count == 1);

    let view = handle.view();
    let record = view
        .recent_intents
        .iter()
        .find(|r| r.actor == "player-9")
        .expect("the dashboard needs to see who did what");
    assert_eq!(record.tick, 1);
    assert!(record.rejected.is_none());
}

/// A model that raises two reports every tick, to exercise the report ring.
struct Chatty {
    pending: Vec<Report>,
}

impl Model for Chatty {
    fn kind(&self) -> &'static str {
        "chatty"
    }
    fn reset(&mut self, _blob: &ScenarioBlob) -> anyhow::Result<()> {
        Ok(())
    }
    fn apply_intent(&mut self, _intent: &Intent) -> Result<(), RejectReason> {
        Ok(())
    }
    fn step(&mut self, tick: u64) {
        for n in 0..2 {
            self.pending.push(Report {
                seq: 0,
                tick,
                kind: ReportKind::Detection,
                uid: format!("t{tick}-{n}"),
                observer: Some(1),
                target: Some(n),
                target_kind: "person".into(),
                lat_e7: 0,
                lon_e7: 0,
                alt_mm: 0,
                detail: serde_json::Value::Null,
            });
        }
    }
    fn snapshot(&self, out: &mut Vec<CellBag>, _limit: usize) -> Truncation {
        out.clear();
        Truncation::default()
    }
    fn agent_count(&self) -> u64 {
        0
    }
    fn occupied_cells(&self) -> u64 {
        0
    }
    fn rng_state(&self) -> u64 {
        0
    }
    fn set_rng_state(&mut self, _state: u64) {}
    fn load_cells(&mut self, _cells: Vec<CellBag>) -> anyhow::Result<()> {
        Ok(())
    }
    fn drain_reports(&mut self, out: &mut Vec<Report>) {
        out.append(&mut self.pending);
    }
}

#[test]
fn reports_are_numbered_in_order_and_survive_a_reload_without_reusing_numbers() {
    let mut registry = ModelRegistry::with_builtins();
    registry.register("chatty", |_| {
        Ok(Box::new(Chatty {
            pending: Vec::new(),
        }))
    });
    let mut blob = scenario(Profile::Fast, 0);
    blob.manifest.model = "chatty".into();

    let driver = Driver::spawn(registry, blob).unwrap();
    let handle = driver.handle();
    handle.command(Control::Step { n: 3 }).unwrap();
    wait_for(&handle, "3 ticks", |v| v.meta.tick == 3);

    let view = handle.view();
    let seqs: Vec<u64> = view.reports.iter().map(|r| r.seq).collect();
    assert_eq!(seqs, vec![1, 2, 3, 4, 5, 6]);
    assert_eq!(view.reports[4].uid, "t3-0");
    assert_eq!(view.reports_since(4).len(), 2);
    assert!(view.reports_since(6).is_empty());

    // A reset clears the old world's reports but keeps counting.
    handle.command(Control::Reset).unwrap();
    assert!(handle.view().reports.is_empty());
    handle.command(Control::Step { n: 1 }).unwrap();
    wait_for(&handle, "1 tick", |v| v.meta.tick == 1);
    let seqs: Vec<u64> = handle.view().reports.iter().map(|r| r.seq).collect();
    assert_eq!(seqs, vec![7, 8]);
    driver.shutdown();
}
