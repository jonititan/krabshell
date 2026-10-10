//! Rewind: a tick rebuilt from the history is the world as it was then.

use std::time::Duration;

use sim_core::{GeoCell, Intent, IntentVerb, Profile};
use sim_engine::{read_pack, Control, Driver, DriverHandle, ModelRegistry, Submission};

fn london_patrol(every: u64, keep: usize) -> sim_core::ScenarioBlob {
    let dir =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/london-patrol");
    let mut blob = read_pack(&dir).unwrap();
    assert_eq!(blob.config.run.profile, Profile::Gold);
    blob.config.run.history_every_ticks = every;
    blob.config.run.history_keyframes = keep;
    blob
}

/// Step `n` ticks and wait until the published view is the paused world at
/// the end of them.
fn step(handle: &DriverHandle, n: u64) {
    let target = handle.view().meta.tick + n;
    handle.command(Control::Step { n }).unwrap();
    for _ in 0..1_000 {
        let view = handle.view();
        if view.meta.tick == target && !view.health.mode.is_advancing() {
            return;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("never reached tick {target}");
}

/// Wait until the history has caught up with the keyframes sent to it.
fn settle(handle: &DriverHandle, first: u64) {
    for _ in 0..200 {
        if handle.history_range().first_tick == Some(first) {
            return;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!(
        "history never started at {first}: {:?}",
        handle.history_range()
    );
}

#[test]
fn a_rewound_tick_is_the_world_as_it_was_intents_and_all() {
    let driver = Driver::spawn(ModelRegistry::with_builtins(), london_patrol(100, 60)).unwrap();
    let handle = driver.handle();

    let mut seen = Vec::new();
    step(&handle, 137);
    seen.push((137, handle.view().meta.hash.unwrap()));
    // Vehicles draw their type from the model's generator, so this also
    // checks the replay carries the generator on.
    let cell = GeoCell::from_lonlat(-0.1276, 51.5072, 15).unwrap();
    handle
        .submit(Submission::fire_and_forget(Intent::operator(
            cell,
            IntentVerb::Spawn,
            serde_json::json!({ "kind": "vehicle", "count": 3 }),
        )))
        .unwrap();
    step(&handle, 1);
    seen.push((138, handle.view().meta.hash.unwrap()));
    step(&handle, 62);
    seen.push((200, handle.view().meta.hash.unwrap()));
    step(&handle, 99);
    seen.push((299, handle.view().meta.hash.unwrap()));
    step(&handle, 151);
    assert_eq!(handle.view().meta.tick, 450);
    settle(&handle, 0);

    // Backwards, forwards, and the same tick twice.
    for &(tick, hash) in seen.iter().rev().chain(seen.iter()) {
        let view = handle.view_at(tick).unwrap();
        assert_eq!(view.meta.tick, tick);
        assert_eq!(view.meta.hash, Some(hash), "tick {tick}");
    }
    // The live tick is the live view.
    assert_eq!(
        handle.view_at(450).unwrap().meta.hash,
        handle.view().meta.hash
    );
    assert!(handle.view_at(451).is_err(), "the future cannot be shown");
}

#[test]
fn the_history_keeps_its_window_and_starts_again_with_a_new_world() {
    let driver = Driver::spawn(ModelRegistry::with_builtins(), london_patrol(50, 3)).unwrap();
    let handle = driver.handle();
    step(&handle, 260);
    // Keyframes at 100, 150, 200, 250 were taken; three are kept.
    settle(&handle, 150);
    assert!(handle.view_at(149).is_err(), "older than the window");
    assert!(handle.view_at(150).is_ok());

    handle.command(Control::Reset).unwrap();
    settle(&handle, 0);
    assert!(handle.view_at(0).is_ok());
}

#[test]
fn the_fast_profile_has_no_rewind() {
    let mut blob = london_patrol(100, 60);
    blob.config.run.profile = Profile::Fast;
    let driver = Driver::spawn(ModelRegistry::with_builtins(), blob).unwrap();
    let handle = driver.handle();
    step(&handle, 120);
    let range = handle.history_range();
    assert_eq!(range.first_tick, None);
    assert!(range.off.unwrap().contains("gold"));
    assert!(handle.view_at(10).is_err());
}
