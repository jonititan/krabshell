use super::*;
use sim_core::{ActorId, Config, ScenarioManifest};
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
    let blob = blob(profile);
    let mut model = Drones {
        grid: SparseCellGrid::new(blob.config.grid.resolution, blob.config.grid.root),
        entities: BTreeMap::new(),
        settings: settings(&blob.config.model),
        seed: 0,
        gold: false,
        next_id: 1,
        last_seen: BTreeMap::new(),
        reports: Vec::new(),
        events: Vec::new(),
        sightings: Vec::new(),
    };
    model.reset(&blob).unwrap();
    model
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
    let pose = model.entities[&agent].pose;
    Intent {
        id: Uuid::new_v4(),
        tick: None,
        cell: model.cell_of(pose.lat_e7, pose.lon_e7).unwrap(),
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
    let kinds: BTreeSet<&str> = model.entities.values().map(|e| e.kind.name()).collect();
    assert_eq!(kinds.len(), 5);
    assert_eq!(model.entities[&1].callsign, "HAWK-1");
    assert_eq!(model.entities[&2].callsign, "UAV-2");
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

    let drone = model.entities[&1].drone.as_ref().unwrap();
    assert_eq!(drone.mode, Mode::Auto);
    assert!(model.entities[&1].pose.alt_mm > 39_000);
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
    let drone = &model.entities[&2];
    assert_eq!(drone.drone.as_ref().unwrap().mode, Mode::Disarmed);
    assert!(drone.pose.grounded());
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
