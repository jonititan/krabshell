use std::collections::BTreeSet;
use std::path::Path;

use krabmaga::engine::schedule::Schedule;
use krabmaga::engine::state::State;
use sim_core::{AgentId, Intent, IntentVerb, ScenarioBlob};

use crate::agents::{Dispatcher, DroneAgent, GroundAgent, IntruderAgent, PoiAgent};
use crate::{flight, schedule, Kind, World};

fn pack(name: &str) -> ScenarioBlob {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(name);
    sim_core::read_pack(&dir).unwrap()
}

fn started(name: &str) -> (World, Schedule) {
    let mut world = World::new(&pack(name)).unwrap();
    let mut schedule = schedule();
    world.init(&mut schedule);
    (world, schedule)
}

fn hash(world: &World) -> u64 {
    let mut cells = Vec::new();
    world.snapshot(&mut cells, usize::MAX);
    sim_core::snapshot::stable_hash(&cells)
}

fn count(world: &World, kind: Kind) -> usize {
    world.records().iter().filter(|r| r.kind == kind).count()
}

#[test]
fn every_entity_is_an_agent_on_the_schedule() {
    let (world, schedule) = started("london-patrol");
    let events = schedule.get_all_events();
    let of = |is: fn(&dyn krabmaga::engine::agent::Agent) -> bool| {
        events.iter().filter(|a| is(a.as_ref())).count()
    };
    assert_eq!(of(|a| a.downcast_ref::<Dispatcher>().is_some()), 1);
    assert_eq!(
        of(|a| a.downcast_ref::<DroneAgent>().is_some()),
        count(&world, Kind::Drone)
    );
    assert_eq!(
        of(|a| a.downcast_ref::<GroundAgent>().is_some()),
        count(&world, Kind::Person) + count(&world, Kind::Vehicle)
    );
    assert_eq!(
        of(|a| a.downcast_ref::<IntruderAgent>().is_some()),
        count(&world, Kind::Intruder)
    );
    assert_eq!(
        of(|a| a.downcast_ref::<PoiAgent>().is_some()),
        count(&world, Kind::Poi)
    );
    assert_eq!(events.len() as u64, world.agent_count() + 1);
    assert!(count(&world, Kind::Person) > 0 && count(&world, Kind::Vehicle) > 0);
}

#[test]
fn a_drone_sees_into_every_cell_holding_something_in_sensor_range() {
    let (mut world, mut schedule) = started("london-patrol");
    for _ in 0..50 {
        schedule.step(&mut world);
    }
    let range = world.settings.sensor.max_range_m;
    let records = world.records().to_vec();
    let mut checked = 0;
    // Put an observer where every seventh agent is, in the cell it occupies.
    for observer in records.iter().step_by(7) {
        let seen: BTreeSet<AgentId> = world.nearby(observer.cell).into_iter().collect();
        let from = observer.saved.pose.point();
        for target in &records {
            if flight::ground_distance_m(&from, &target.saved.pose.point()) <= range {
                assert!(
                    seen.contains(&target.id),
                    "{} {} is in range of {} but not in its neighbourhood",
                    target.kind.name(),
                    target.id,
                    observer.id
                );
                checked += 1;
            }
        }
    }
    assert!(checked > 1_000, "only {checked} pairs in range");
}

#[test]
fn the_standalone_loop_and_an_embedder_with_its_own_clock_agree() {
    let (mut free, mut a) = started("kaunas-deliveries");
    let (mut pinned, mut b) = started("kaunas-deliveries");
    for tick in 1..=600 {
        a.step(&mut free);
        pinned.step_at(&mut b, tick);
    }
    assert_eq!(free.tick(), 600);
    assert_eq!(pinned.tick(), 600);
    assert_eq!(hash(&free), hash(&pinned));
    assert_eq!(free.report_counts(), pinned.report_counts());
}

#[test]
fn a_gold_run_repeats_and_init_starts_it_over() {
    let (mut world, mut schedule) = started("london-patrol");
    for _ in 0..400 {
        schedule.step(&mut world);
    }
    let (first, first_reports) = (hash(&world), world.report_counts().clone());
    assert!(!first_reports.is_empty());

    // As `simulate!` does between repetitions.
    let mut schedule = crate::schedule();
    world.init(&mut schedule);
    assert_eq!(world.tick(), 0);
    for _ in 0..400 {
        schedule.step(&mut world);
    }
    assert_eq!(hash(&world), first);
    assert_eq!(world.report_counts(), &first_reports);
}

#[test]
fn spawned_agents_join_the_schedule_and_despawned_ones_leave_it() {
    let (mut world, mut schedule) = started("london-patrol");
    for _ in 0..5 {
        schedule.step(&mut world);
    }
    let before = world.agent_count();
    let cell = world.records()[0].cell;
    world
        .apply_intent(&Intent::operator(
            cell,
            IntentVerb::Spawn,
            serde_json::json!({ "kind": "vehicle", "count": 3 }),
        ))
        .unwrap();
    assert_eq!(world.agent_count(), before, "nothing changes until a step");
    schedule.step(&mut world);
    assert_eq!(world.agent_count(), before + 3);
    assert_eq!(schedule.get_all_events().len() as u64, before + 3 + 1);
    let newest = world.records().last().unwrap().clone();
    assert_eq!(newest.kind, Kind::Vehicle);

    let mut despawn = Intent::operator(cell, IntentVerb::Despawn, serde_json::Value::Null);
    despawn.agent = Some(newest.id);
    world.apply_intent(&despawn).unwrap();
    assert!(world.apply_intent(&despawn).is_err(), "already going");
    schedule.step(&mut world);
    assert_eq!(world.agent_count(), before + 2);
    assert!(world.record(newest.id).is_none());
    assert_eq!(schedule.get_all_events().len() as u64, before + 2 + 1);
}

#[test]
fn the_board_holds_exactly_what_the_grid_committed() {
    let (mut world, mut schedule) = started("london-patrol");
    for _ in 0..300 {
        schedule.step(&mut world);
        let from_grid = std::cell::RefCell::new(Vec::new());
        world
            .grid
            .iter_objects(|cell, id| from_grid.borrow_mut().push((cell.raw(), *id)));
        let mut from_grid = from_grid.into_inner();
        from_grid.sort_unstable();
        let mut from_board: Vec<(u64, AgentId)> = world
            .records()
            .iter()
            .map(|r| (r.cell.raw(), r.id))
            .collect();
        from_board.sort_unstable();
        assert_eq!(from_grid, from_board, "tick {}", world.tick());
    }
}

#[test]
fn a_committed_view_stays_put_while_the_world_steps_on() {
    let (mut world, mut schedule) = started("kaunas-deliveries");
    for _ in 0..50 {
        schedule.step(&mut world);
    }
    let held = world.committed();
    let before = hash(&world);
    for _ in 0..50 {
        schedule.step(&mut world);
    }
    let mut cells = Vec::new();
    held.snapshot(&mut cells, usize::MAX);
    assert_eq!(sim_core::snapshot::stable_hash(&cells), before);
    assert_ne!(hash(&world), before, "the world moved on");
    assert_eq!(held.records().len() as u64, world.agent_count());
}

#[test]
fn every_vehicle_has_a_type_drawn_from_the_seed_and_sightings_of_it_say_which() {
    let (mut world, mut schedule) = started("london-patrol");
    let types: Vec<_> = world
        .records()
        .iter()
        .filter(|r| r.kind == Kind::Vehicle)
        .map(|r| r.saved.vehicle_type.expect("every vehicle has a type"))
        .collect();
    let distinct: BTreeSet<&str> = types.iter().map(|t| t.name()).collect();
    assert!(distinct.len() >= 3, "{distinct:?}");
    assert!(world
        .records()
        .iter()
        .filter(|r| r.kind != Kind::Vehicle)
        .all(|r| r.saved.vehicle_type.is_none()));
    // The same pack draws the same types.
    let (again, _) = started("london-patrol");
    let again: Vec<_> = again
        .records()
        .iter()
        .filter_map(|r| r.saved.vehicle_type)
        .collect();
    assert_eq!(types, again);

    let mut reports = Vec::new();
    for _ in 0..3_000 {
        schedule.step(&mut world);
        world.drain_reports(&mut reports);
    }
    let vehicles: Vec<_> = reports
        .iter()
        .filter(|r| r.target_kind == "vehicle")
        .collect();
    assert!(!vehicles.is_empty());
    for r in vehicles {
        let named = r.detail["vehicle_type"].as_str().unwrap();
        let target = world.record(r.target.unwrap()).unwrap();
        assert_eq!(Some(named), target.saved.vehicle_type.map(|t| t.name()));
    }
}
