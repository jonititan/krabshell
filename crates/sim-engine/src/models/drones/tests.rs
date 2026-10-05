use std::collections::BTreeSet;

use super::*;
use sim_core::config::Param;
use sim_core::{ActorId, Config, IntentVerb, Profile, ReportKind, ScenarioManifest, SeedAgent};
use sim_drones::Record;
use uuid::Uuid;

/// Resolution-6 parent over southern England, the same root as the London packs.
const ROOT: &str = "6361800000000000";
const LON: f64 = -0.1276;
const LAT: f64 = 51.5072;

fn square_patrol() -> serde_json::Value {
    let corner = |east: f64, north: f64| {
        let (lat, lon) = flight::displace(flight::to_e7(LAT), flight::to_e7(LON), east, north);
        (lat, lon)
    };
    let mut items = vec![serde_json::json!({
        "seq": 0, "frame": 3, "command": 22, "z": 40.0
    })];
    for (i, (e, n)) in [(150.0, 0.0), (150.0, 150.0), (0.0, 150.0)]
        .iter()
        .enumerate()
    {
        let (x, y) = corner(*e, *n);
        items.push(serde_json::json!({
            "seq": i + 1, "frame": 3, "command": 16,
            "param2": 3.0, "param4": null, "x": x, "y": y, "z": 40.0
        }));
    }
    // Back to the first waypoint, forever.
    items.push(serde_json::json!({
        "seq": 4, "frame": 2, "command": 177, "param1": 1.0, "param2": -1.0
    }));
    serde_json::Value::Array(items)
}

fn blob(profile: Profile) -> ScenarioBlob {
    let manifest = ScenarioManifest {
        id: "drones-test".into(),
        name: "drones test".into(),
        description: String::new(),
        model: "drones".into(),
        engine_rev: None,
        resolution: 15,
        root: Some(GeoCell::parse(ROOT).unwrap()),
        profile,
    };
    let mut config = Config::default();
    manifest.apply_to(&mut config);
    config.model.insert("persons".into(), Param::Int(80));
    config.model.insert("vehicles".into(), Param::Int(20));
    config
        .model
        .insert("wander_radius_m".into(), Param::Float(250.0));
    config.model.insert("origin_lon".into(), Param::Float(LON));
    config.model.insert("origin_lat".into(), Param::Float(LAT));
    config.model.insert("seed".into(), Param::Int(11));

    let agent = |id: u64, kind: &str, lon: f64, lat: f64, state: serde_json::Value| SeedAgent {
        id: Some(id),
        kind: kind.into(),
        cell: None,
        lon: Some(lon),
        lat: Some(lat),
        state,
    };
    ScenarioBlob {
        manifest,
        config,
        agents: vec![
            agent(
                1,
                "drone",
                LON,
                LAT,
                serde_json::json!({ "sysid": 1, "callsign": "HAWK-1", "mission": square_patrol(), "autostart": true }),
            ),
            agent(
                2,
                "drone",
                LON + 0.001,
                LAT,
                serde_json::json!({ "sysid": 2 }),
            ),
            agent(
                3,
                "poi",
                LON + 0.0005,
                LAT + 0.0005,
                serde_json::json!({ "name": "Trafalgar Square" }),
            ),
            agent(
                4,
                "intruder",
                LON,
                LAT + 0.001,
                serde_json::json!({ "route": [[LON, LAT + 0.001, 45.0], [LON + 0.002, LAT + 0.001, 45.0]] }),
            ),
        ],
        fields: serde_json::Value::Null,
    }
}

fn drones(profile: Profile) -> Drones {
    Drones::new(&blob(profile)).unwrap()
}

fn record(model: &Drones, id: AgentId) -> &Record {
    model.world().record(id).unwrap()
}

fn run(model: &mut Drones, from: u64, ticks: u64) -> Vec<Report> {
    let mut reports = Vec::new();
    for tick in from..from + ticks {
        model.step(tick);
        model.drain_reports(&mut reports);
    }
    reports
}

fn hash(model: &Drones) -> u64 {
    let mut cells = Vec::new();
    model.snapshot(&mut cells, usize::MAX);
    sim_core::snapshot::stable_hash(&cells)
}

fn intent(model: &Drones, agent: AgentId, code: u16, payload: serde_json::Value) -> Intent {
    let pose = record(model, agent).saved.pose;
    Intent {
        id: Uuid::new_v4(),
        tick: None,
        cell: model.world().cell_of(pose.lat_e7, pose.lon_e7).unwrap(),
        agent: Some(agent),
        actor: ActorId::operator(),
        verb: IntentVerb::Custom(code),
        payload,
        issued_at: 0,
    }
}

#[test]
fn a_seeded_world_holds_every_kind_of_entity() {
    let model = drones(Profile::Gold);
    assert_eq!(model.agent_count(), 4 + 80 + 20);
    let kinds: BTreeSet<&str> = model
        .world()
        .records()
        .iter()
        .map(|r| r.kind.name())
        .collect();
    assert_eq!(kinds.len(), 5);
    assert_eq!(record(&model, 1).saved.callsign, "HAWK-1");
    assert_eq!(record(&model, 2).saved.callsign, "UAV-2");
}

#[test]
fn the_patrolling_drone_reports_waypoints_and_sightings() {
    let mut model = drones(Profile::Gold);
    let reports = run(&mut model, 1, 1_500); // 150 simulated seconds
    let waypoints: Vec<u64> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Waypoint)
        .map(|r| r.detail["seq"].as_u64().unwrap())
        .collect();
    assert!(waypoints.starts_with(&[0, 1, 2, 3, 4, 1]), "{waypoints:?}");

    let sightings: BTreeSet<&str> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Detection)
        .map(|r| r.target_kind.as_str())
        .collect();
    assert!(sightings.contains("person"), "{sightings:?}");
    assert!(
        reports
            .iter()
            .all(|r| r.kind != ReportKind::Detection || r.observer == Some(1)),
        "only the airborne drone can see anything"
    );

    let hawk = record(&model, 1);
    assert_eq!(hawk.saved.drone.as_ref().unwrap().mode, Mode::Auto);
    assert!(hawk.saved.pose.alt_mm > 39_000);
}

#[test]
fn gold_runs_are_identical_down_to_the_reports() {
    let mut a = drones(Profile::Gold);
    let mut b = drones(Profile::Gold);
    let ra = run(&mut a, 1, 800);
    let rb = run(&mut b, 1, 800);
    assert_eq!(hash(&a), hash(&b));
    assert_eq!(ra, rb);
    assert!(!ra.is_empty());
}

#[test]
fn a_checkpoint_resumes_on_the_same_path() {
    let mut straight = drones(Profile::Gold);
    run(&mut straight, 1, 400);
    let mut cells = Vec::new();
    straight.snapshot(&mut cells, usize::MAX);
    let fields = straight.fields();
    let rest_straight = run(&mut straight, 401, 400);

    let mut restored = drones(Profile::Gold);
    restored.load_cells(cells).unwrap();
    restored.restore_fields(&fields).unwrap();
    let rest_restored = run(&mut restored, 401, 400);

    assert_eq!(hash(&straight), hash(&restored));
    assert_eq!(rest_straight, rest_restored);
}

#[test]
fn a_mission_uploaded_by_intent_is_flown() {
    let mut model = drones(Profile::Gold);
    let (x, y) = flight::displace(flight::to_e7(LAT), flight::to_e7(LON + 0.001), 0.0, 80.0);
    let items = serde_json::json!({ "items": [
        { "seq": 0, "frame": 3, "command": 22, "z": 25.0 },
        { "seq": 1, "frame": 3, "command": 16, "x": x, "y": y, "z": 25.0 },
        { "seq": 2, "frame": 3, "command": 21 }
    ]});
    model
        .apply_intent(&intent(&model, 2, codes::MISSION_UPLOAD, items))
        .unwrap();
    let arm = serde_json::json!({ "command": 400, "params": [1, 0, 0, 0, 0, 0, 0] });
    model
        .apply_intent(&intent(&model, 2, codes::COMMAND, arm))
        .unwrap();
    let start = serde_json::json!({ "command": 300, "params": [0, 0] });
    model
        .apply_intent(&intent(&model, 2, codes::COMMAND, start))
        .unwrap();

    let reports = run(&mut model, 1, 900);
    let reached: Vec<u64> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Waypoint && r.observer == Some(2))
        .map(|r| r.detail["seq"].as_u64().unwrap())
        .collect();
    assert_eq!(reached, vec![0, 1, 2]);
    let saved = &record(&model, 2).saved;
    let drone = saved.drone.as_ref().unwrap();
    assert_eq!(drone.mode, Mode::Disarmed);
    assert!(drone.grounded(&saved.pose, model.world().terrain()));
}

#[test]
fn bad_intents_are_refused_with_a_reason() {
    let mut model = drones(Profile::Gold);
    let far = serde_json::json!({ "items": [
        { "seq": 0, "frame": 3, "command": 16, "x": 400_000_000, "y": 100_000_000, "z": 30.0 }
    ]});
    let err = model
        .apply_intent(&intent(&model, 2, codes::MISSION_UPLOAD, far))
        .unwrap_err();
    assert!(err.to_string().contains("outside the world"), "{err}");

    let takeoff = serde_json::json!({ "command": 22, "params": [0, 0, 0, 0, 0, 0, 30] });
    let err = model
        .apply_intent(&intent(&model, 2, codes::COMMAND, takeoff))
        .unwrap_err();
    assert!(err.to_string().contains("arm"), "{err}");

    // A POI is not a drone.
    let err = model
        .apply_intent(&intent(
            &model,
            3,
            codes::MISSION_CLEAR,
            serde_json::Value::Null,
        ))
        .unwrap_err();
    assert!(err.to_string().contains("not a drone"), "{err}");
}

#[test]
fn published_drone_state_carries_what_a_gateway_needs() {
    let mut model = drones(Profile::Gold);
    run(&mut model, 1, 50);
    let mut cells = Vec::new();
    model.snapshot(&mut cells, usize::MAX);
    let drone = cells
        .iter()
        .flat_map(|b| &b.objects)
        .find(|o| o.id == 1)
        .unwrap();
    let state = &drone.state;
    for key in [
        "lat_e7", "lon_e7", "alt_mm", "hdg_cdeg", "spd_cms", "callsign",
    ] {
        assert!(state.get(key).is_some(), "missing {key}: {state}");
    }
    let d = &state["drone"];
    for key in [
        "sysid",
        "mode",
        "home",
        "mission",
        "mission_seq",
        "mission_rev",
        "battery_ppm",
    ] {
        assert!(d.get(key).is_some(), "missing drone.{key}: {d}");
    }
    assert_eq!(d["mission"].as_array().unwrap().len(), 5);
    assert!(d["mission"][1]["param4"].is_null(), "NaN travels as null");
}

#[test]
fn a_gateway_can_read_every_drone_back_out_of_a_snapshot() {
    let mut model = drones(Profile::Gold);
    run(&mut model, 1, 30);
    let mut cells = Vec::new();
    model.snapshot(&mut cells, usize::MAX);
    let seen = drones_in(&cells);
    assert_eq!(seen.keys().copied().collect::<Vec<_>>(), vec![1, 2]);
    let hawk = &seen[&1];
    assert_eq!(hawk.callsign, "HAWK-1");
    assert_eq!(hawk.id, 1);
    assert_eq!(hawk.mission.len(), 5);
    assert!(hawk.armed());
    assert!(!seen[&2].armed());
    assert!(hawk.battery_pct > 99.0);
}

fn courier_blob() -> ScenarioBlob {
    let mut blob = blob(Profile::Gold);
    let near = |east_m: f64, north_m: f64| {
        let (lat, lon) = flight::displace(flight::to_e7(LAT), flight::to_e7(LON), east_m, north_m);
        serde_json::json!([lat, lon, 0, format!("{east_m:.0} E {north_m:.0} N")])
    };
    blob.fields = serde_json::json!({ "addresses": {
        "depots": ["HUB"],
        "items": [near(600.0, 0.0), near(-400.0, 300.0), near(0.0, -500.0)],
    }});
    blob.agents.push(SeedAgent {
        id: Some(5),
        kind: "drone".into(),
        cell: None,
        lon: Some(LON - 0.001),
        lat: Some(LAT),
        state: serde_json::json!({ "sysid": 5, "callsign": "COURIER", "depot": "HUB", "cruise_agl_m": 50 }),
    });
    // About one order every 30 s across the three addresses.
    blob.config
        .model
        .insert("order_rate_per_address_day".into(), Param::Float(1_000.0));
    blob.config
        .model
        .insert("dispatch_load_s".into(), Param::Float(10.0));
    blob
}

#[test]
fn orders_are_dispatched_delivered_reported_and_the_courier_comes_home() {
    let mut model = drones(Profile::Gold);
    model.reset(&courier_blob()).unwrap();

    let reports = run(&mut model, 1, 9_000);
    let deliveries: Vec<&Report> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Delivery)
        .collect();
    assert!(deliveries.len() >= 3, "{} deliveries", deliveries.len());
    let addresses: Vec<(i32, i32)> = model
        .world()
        .book()
        .addresses
        .iter()
        .map(|a| (a.lat_e7, a.lon_e7))
        .collect();
    for (i, d) in deliveries.iter().enumerate() {
        assert_eq!(d.observer, Some(5));
        assert!(addresses.contains(&(d.lat_e7, d.lon_e7)));
        let detail = &d.detail;
        let f = |k: &str| detail[k].as_f64().unwrap();
        assert!((f("total_s") - f("wait_s") - f("flight_s")).abs() < 1e-6);
        assert!(
            f("flight_s") > 30.0,
            "a 400–600 m trip with a winch cycle takes a while"
        );
        // One courier serves the queue oldest first.
        if i > 0 {
            assert!(detail["order"].as_u64() > deliveries[i - 1].detail["order"].as_u64());
        }
    }
    // Orders keep coming faster than one drone delivers, so some wait.
    let dispatch = model.world().dispatch();
    assert!(dispatch.placed > dispatch.delivered);
    assert_eq!(dispatch.delivered as usize, deliveries.len());
    let courier = record(&model, 5).saved.courier.as_ref().unwrap();
    assert!(
        courier.landed_tick.is_some(),
        "it has been home at least once"
    );
}

#[test]
fn a_checkpoint_mid_delivery_restores_the_queue_and_the_courier() {
    let blob = courier_blob();
    let mut original = drones(Profile::Gold);
    original.reset(&blob).unwrap();
    run(&mut original, 1, 2_500);
    assert!(
        record(&original, 5)
            .saved
            .courier
            .as_ref()
            .unwrap()
            .order
            .is_some()
            || original.world().dispatch().queued() > 0
    );

    let mut cells = Vec::new();
    original.snapshot(&mut cells, usize::MAX);
    let mut restored = drones(Profile::Gold);
    restored.reset(&blob).unwrap();
    restored.load_cells(cells).unwrap();
    restored.restore_fields(&original.fields()).unwrap();
    assert_eq!(restored.world().dispatch(), original.world().dispatch());

    let a = run(&mut original, 2_501, 3_000);
    let b = run(&mut restored, 2_501, 3_000);
    assert_eq!(hash(&original), hash(&restored));
    assert_eq!(a, b);
}
