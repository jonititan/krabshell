use std::collections::BTreeSet;

use super::*;
use sim_core::config::Param;
use sim_core::RejectReason;
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

/// One courier and a spare at a hub whose batteries cannot finish a trip:
/// 40 s of flight, no reserve, the nearest address a minute away.
fn crashing_blob() -> ScenarioBlob {
    let mut blob = courier_blob();
    blob.agents.push(SeedAgent {
        id: Some(6),
        kind: "drone".into(),
        cell: None,
        lon: Some(LON - 0.0012),
        lat: Some(LAT),
        state: serde_json::json!({ "sysid": 6, "callsign": "HUB-SPARE", "spare": "HUB" }),
    });
    for (key, value) in [
        ("battery_drain_pct_s", 2.5),
        ("battery_reserve_pct", 0.0),
        ("launch_battery_pct", 0.0),
    ] {
        blob.config.model.insert(key.into(), Param::Float(value));
    }
    blob
}

#[test]
fn a_courier_that_runs_flat_crashes_and_the_spare_then_new_drones_replace_it() {
    let blob = crashing_blob();
    let mut model = drones(Profile::Gold);
    model.reset(&blob).unwrap();
    let drones_before = model
        .world()
        .records()
        .iter()
        .filter(|r| r.saved.drone.is_some())
        .count();

    let reports = run(&mut model, 1, 6_000);
    let all_crashes: Vec<&Report> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Crash)
        .collect();
    // HAWK-1's patrol runs flat too; follow the hub's couriers.
    let crashes: Vec<&Report> = all_crashes
        .iter()
        .copied()
        .filter(|r| r.detail["depot"] == "HUB")
        .collect();
    assert!(crashes.len() >= 3, "{} crashes", crashes.len());
    assert!(
        !reports.iter().any(|r| r.kind == ReportKind::Delivery),
        "nothing gets that far"
    );

    // The courier first, its spare next, then a new drone for each crash.
    assert_eq!(crashes[0].observer, Some(5));
    assert_eq!(crashes[0].detail["replaced_by"]["id"], 6);
    assert_eq!(crashes[0].detail["replaced_by"]["spare"], true);
    assert_eq!(crashes[1].observer, Some(6));
    // Never an id or a system id anything has had before.
    let mut ids: BTreeSet<u64> = model.world().records().iter().map(|r| r.id).collect();
    ids.extend([1, 5, 6]);
    let mut sysids = BTreeSet::from([1, 2, 5, 6]);
    for crash in all_crashes
        .iter()
        .filter(|c| c.detail["replaced_by"]["spare"] == false)
    {
        let by = &crash.detail["replaced_by"];
        assert!(
            sysids.insert(by["sysid"].as_u64().unwrap()),
            "{by} reuses a sysid"
        );
    }
    let new_ids: Vec<u64> = all_crashes
        .iter()
        .filter(|c| c.detail["replaced_by"]["spare"] == false)
        .map(|c| c.detail["replaced_by"]["id"].as_u64().unwrap())
        .collect();
    assert!(new_ids.windows(2).all(|w| w[0] < w[1]), "{new_ids:?}");
    ids.extend(new_ids.iter().copied());
    for (i, crash) in crashes.iter().enumerate().skip(1) {
        let by = &crash.detail["replaced_by"];
        assert_eq!(by["spare"], false);
        let id = by["id"].as_u64().unwrap();
        assert!(by["callsign"].as_str().unwrap().starts_with("HUB-SPARE-"));
        if let Some(next) = crashes.get(i + 1) {
            assert_eq!(
                next.observer,
                Some(id),
                "the new drone flies the next order"
            );
        }
        // Every crash put the order it carried back in the queue.
        assert!(crash.detail["requeued_order"].as_u64().is_some());
    }
    // Wrecks leave. The spare was already in the fleet, so calling it up
    // leaves one drone fewer; every crash after that brings a new one.
    let live: Vec<&Record> = model
        .world()
        .records()
        .iter()
        .filter(|r| r.saved.drone.is_some())
        .collect();
    assert_eq!(live.len(), drones_before - 1);
    assert!(model.world().record(5).is_none() && model.world().record(6).is_none());
    // Each wreck lies where it fell, short of the address and away from home.
    for crash in &crashes {
        assert!(crash.alt_mm <= 1_000, "{} mm", crash.alt_mm);
    }
    // No order is lost: every one placed is queued or on board.
    let dispatch = model.world().dispatch();
    let carried = live
        .iter()
        .filter(|r| r.saved.courier.as_ref().is_some_and(|c| c.order.is_some()))
        .count();
    assert_eq!(dispatch.placed as usize, dispatch.queued() + carried);
}

#[test]
fn every_flight_ends_in_a_landing_report_with_its_battery() {
    let mut blob = courier_blob();
    blob.config
        .model
        .insert("battery_drain_pct_s".into(), Param::Float(0.1));
    let mut model = drones(Profile::Gold);
    model.reset(&blob).unwrap();
    let reports = run(&mut model, 1, 9_000);
    let landings: Vec<&Report> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Landing && r.observer == Some(5))
        .collect();
    let deliveries = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Delivery)
        .count();
    // One flight per delivery, give or take the one in the air at the end.
    assert!(
        landings.len() + 1 >= deliveries && landings.len() <= deliveries + 1,
        "{} landings, {deliveries} deliveries",
        landings.len()
    );
    // No charger here, so each flight lands with less than the last.
    let mut last = 100.0;
    for l in &landings {
        let pct = l.detail["battery_pct"].as_f64().unwrap();
        assert!(pct > 0.0 && pct < last, "{pct} % after {last} %");
        assert_eq!(l.detail["depot"], "HUB");
        last = pct;
    }
    assert!(!reports.iter().any(|r| r.kind == ReportKind::Crash));
}

#[test]
fn a_checkpoint_after_a_crash_replays_the_replacements_exactly() {
    let blob = crashing_blob();
    let mut original = drones(Profile::Gold);
    original.reset(&blob).unwrap();
    let before = run(&mut original, 1, 2_500);
    assert!(before.iter().any(|r| r.kind == ReportKind::Crash));

    let mut cells = Vec::new();
    original.snapshot(&mut cells, usize::MAX);
    let mut restored = drones(Profile::Gold);
    restored.reset(&blob).unwrap();
    restored.load_cells(cells).unwrap();
    restored.restore_fields(&original.fields()).unwrap();

    let a = run(&mut original, 2_501, 4_000);
    let b = run(&mut restored, 2_501, 4_000);
    assert!(a.iter().any(|r| r.kind == ReportKind::Crash));
    assert_eq!(a, b);
    assert_eq!(hash(&original), hash(&restored));
}

/// A point `east`, `north` metres from the test origin.
fn at(east: f64, north: f64) -> (f64, f64) {
    let (lat, lon) = flight::displace(flight::to_e7(LAT), flight::to_e7(LON), east, north);
    (f64::from(lon) * 1e-7, f64::from(lat) * 1e-7)
}

fn open_circle(
    model: &mut Drones,
    east: f64,
    north: f64,
    radius_m: f64,
) -> Result<(), RejectReason> {
    let (lon, lat) = at(east, north);
    let payload = serde_json::json!({ "circle": { "lat": lat, "lon": lon, "radius_m": radius_m } });
    let fence = intent(model, 1, codes::GEOFENCE_OPEN, payload);
    model.apply_intent(&Intent {
        agent: None,
        ..fence
    })
}

fn distance_to(record: &Record, east: f64, north: f64) -> f64 {
    let (lon, lat) = at(east, north);
    flight::ground_distance_m(
        &record.saved.pose.point(),
        &flight::Point::at(lon, lat, 0.0),
    )
}

#[test]
fn the_operator_routes_a_patrol_round_a_new_geofence_by_mission_upload_and_back_when_it_closes() {
    let mut model = drones(Profile::Gold);
    run(&mut model, 1, 200);
    // Across the patrol's east side, (150, 0) to (150, 150).
    open_circle(&mut model, 150.0, 75.0, 20.0).unwrap();
    let mut reports = Vec::new();
    let mut closest = f64::INFINITY;
    for tick in 201..4_000 {
        model.step(tick);
        model.drain_reports(&mut reports);
        closest = closest.min(distance_to(record(&model, 1), 150.0, 75.0));
    }
    let fences = model.overlays();
    assert_eq!(geofences_in(&fences).1.len(), 1);
    let reroutes: Vec<&Report> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Reroute && r.observer == Some(1))
        .collect();
    assert!(!reroutes.is_empty());
    let messages = reroutes[0].detail["messages"].to_string();
    assert!(messages.contains("MISSION_ITEM_INT"), "{messages}");
    assert!(
        !reports.iter().any(|r| r.kind == ReportKind::Breach),
        "the patrol never enters the fence"
    );
    // The margin is 30 m: it keeps about 50 m from the centre.
    assert!(closest > 45.0, "came within {closest:.1} m");
    let drone = record(&model, 1).saved.drone.clone().unwrap();
    assert!(drone.mission.len() > 5, "detours were inserted");
    // The loop still loops: the jump lands on the first patrol waypoint.
    let jump = drone
        .mission
        .iter()
        .find(|i| i.command == mission::DO_JUMP)
        .unwrap();
    assert_eq!(
        drone.mission[jump.param1 as usize].x,
        square_patrol()[1]["x"].as_i64().unwrap() as i32
    );
    let laps = reports
        .iter()
        .filter(|r| {
            r.kind == ReportKind::Waypoint && r.observer == Some(1) && r.detail["seq"] == jump.seq
        })
        .count();
    assert!(laps >= 2, "{laps} laps");

    // Closed again, the operator takes its detours back out.
    let id = geofences_in(&model.overlays()).1[0].id;
    let close = intent(
        &model,
        1,
        codes::GEOFENCE_CLOSE,
        serde_json::json!({ "id": id }),
    );
    model.apply_intent(&close).unwrap();
    run(&mut model, 4_000, 400);
    let drone = record(&model, 1).saved.drone.clone().unwrap();
    assert_eq!(drone.mission.len(), 5);
    let base: Vec<MissionItem> = serde_json::from_value(square_patrol()).unwrap();
    // MAVLink's NaN "leave as is" parameters never compare equal with ==.
    assert!(
        sim_drones::operator::same(&drone.mission, &base),
        "{:?}",
        drone.mission
    );
}

#[test]
fn a_fence_opened_round_a_drone_is_a_breach_and_the_operator_flies_it_out_and_carries_on() {
    let mut model = drones(Profile::Gold);
    // On to the far corner, (150, 150), well clear of both pads.
    let mut tick = 1;
    while distance_to(record(&model, 1), 150.0, 150.0) > 20.0 {
        model.step(tick);
        tick += 1;
        assert!(tick < 3_000);
    }
    let pose = record(&model, 1).saved.pose;
    let (lon, lat) = (pose.lon(), pose.lat());
    let payload = serde_json::json!({ "circle": { "lat": lat, "lon": lon, "radius_m": 25.0 } });
    let fence = Intent {
        agent: None,
        ..intent(&model, 1, codes::GEOFENCE_OPEN, payload)
    };
    model.apply_intent(&fence).unwrap();
    let reports = run(&mut model, tick, 2_000);

    let breaches: Vec<&Report> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Breach)
        .collect();
    assert_eq!(breaches.len(), 1, "{breaches:?}");
    assert_eq!(breaches[0].detail["how"], "fence opened round it");
    assert_eq!(breaches[0].detail["flown_by"], "operator");
    let reasons: Vec<&str> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Reroute)
        .map(|r| r.detail["reason"].as_str().unwrap())
        .collect();
    assert_eq!(reasons[0], "inside a geofence");
    assert!(
        reasons.contains(&"round the geofence, mission"),
        "{reasons:?}"
    );
    let drone = record(&model, 1).saved.drone.clone().unwrap();
    assert_eq!(drone.mode, mission::Mode::Auto, "back on its patrol");
    assert!(drone.fenced.is_empty());
    let after = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Waypoint && r.observer == Some(1) && r.tick > tick + 600)
        .count();
    assert!(after > 0, "it keeps patrolling");
}

#[test]
fn a_ground_station_flying_into_a_fence_is_logged_and_left_alone() {
    let mut model = drones(Profile::Gold);
    // UAV-2 sits on its pad about 69 m east of the origin; a fence 150 m north.
    open_circle(&mut model, 69.0, 150.0, 30.0).unwrap();
    for command in [
        serde_json::json!({ "command": 400, "params": [1, 0, 0, 0, 0, 0, 0] }),
        serde_json::json!({ "command": 22, "params": [0, 0, 0, 0, 0, 0, 30] }),
    ] {
        model
            .apply_intent(&intent(&model, 2, codes::COMMAND, command))
            .unwrap();
    }
    run(&mut model, 1, 200);
    let (lon, lat) = at(69.0, 300.0);
    let go = serde_json::json!({ "command": 192, "params": [-1, 0, 0, 0, lat, lon, 30] });
    model
        .apply_intent(&intent(&model, 2, codes::COMMAND, go))
        .unwrap();
    let reports = run(&mut model, 201, 600);
    let breaches: Vec<&Report> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Breach)
        .collect();
    assert_eq!(breaches.len(), 1);
    assert_eq!(breaches[0].observer, Some(2));
    assert_eq!(breaches[0].detail["how"], "flew in");
    assert_eq!(breaches[0].detail["flown_by"], "ground station");
    assert!(!reports
        .iter()
        .any(|r| r.kind == ReportKind::Reroute && r.observer == Some(2)));
}

#[test]
fn a_geofence_may_not_keep_a_drone_off_its_pad_or_enclose_nothing() {
    let mut model = drones(Profile::Gold);
    let err = open_circle(&mut model, 10.0, 0.0, 20.0).unwrap_err();
    assert!(err.to_string().contains("pad"), "{err}");
    let flat = serde_json::json!({ "polygon": [[LON, LAT + 0.002], [LON + 0.001, LAT + 0.002]] });
    let fence = Intent {
        agent: None,
        ..intent(&model, 1, codes::GEOFENCE_OPEN, flat)
    };
    assert!(model.apply_intent(&fence).is_err());
    let close = intent(
        &model,
        1,
        codes::GEOFENCE_CLOSE,
        serde_json::json!({ "id": 99 }),
    );
    assert!(model.apply_intent(&close).is_err());
}

/// The test world with a fence of its own across the patrol's east side,
/// and the battery that would otherwise send the patrol home.
fn fenced_blob() -> ScenarioBlob {
    let mut blob = blob(Profile::Gold);
    let (lon, lat) = at(150.0, 75.0);
    blob.fields = serde_json::json!({ "geofences": [
        { "name": "East side", "circle": { "lat": lat, "lon": lon, "radius_m": 20.0 } },
    ]});
    blob.config
        .model
        .insert("battery_drain_pct_s".into(), Param::Float(0.0));
    blob
}

#[test]
fn a_packs_fences_open_from_the_start_the_patrol_keeps_out_and_a_reset_reopens_them() {
    let mut model = drones(Profile::Gold);
    model.reset(&fenced_blob()).unwrap();
    let (_, fences) = geofences_in(&model.overlays());
    assert_eq!(fences.len(), 1);
    assert_eq!(fences[0].source, "scenario");
    assert_eq!(fences[0].name, "East side");
    assert_eq!(fences[0].opened_tick, 0);

    let mut reports = Vec::new();
    let mut closest = f64::INFINITY;
    for tick in 1..4_000 {
        model.step(tick);
        model.drain_reports(&mut reports);
        closest = closest.min(distance_to(record(&model, 1), 150.0, 75.0));
    }
    assert!(closest > 45.0, "came within {closest:.1} m");
    assert!(!reports.iter().any(|r| r.kind == ReportKind::Breach));
    assert!(reports.iter().any(|r| r.kind == ReportKind::Reroute));
    // Opened when the run began, and reported then.
    assert!(reports.iter().any(|r| r.kind == ReportKind::Geofence
        && r.detail["reason"] == "scenario"
        && r.detail["name"] == "East side"));

    // Closed during the run; a reset brings it back.
    let close = intent(
        &model,
        1,
        codes::GEOFENCE_CLOSE,
        serde_json::json!({ "id": fences[0].id }),
    );
    model.apply_intent(&close).unwrap();
    run(&mut model, 4_000, 1);
    assert!(geofences_in(&model.overlays()).1.is_empty());
    model.reset(&fenced_blob()).unwrap();
    assert_eq!(geofences_in(&model.overlays()).1, fences);
}

#[test]
fn a_pack_fence_over_a_pad_or_outside_the_world_stops_the_pack_loading() {
    let mut blob = blob(Profile::Gold);
    let (lon, lat) = at(0.0, 0.0);
    blob.fields = serde_json::json!({ "geofences": [
        { "circle": { "lat": lat, "lon": lon, "radius_m": 50.0 } },
    ]});
    let err = Drones::new(&blob).err().expect("refused").to_string();
    assert!(
        err.contains("geofences.json fence 0") && err.contains("pad"),
        "{err}"
    );
    blob.fields = serde_json::json!({ "geofences": { "not": "a list" } });
    assert!(Drones::new(&blob).is_err());
}

#[test]
fn a_checkpoint_with_fences_open_replays_exactly() {
    let blob = fenced_blob();
    let mut original = drones(Profile::Gold);
    original.reset(&blob).unwrap();
    run(&mut original, 1, 1_500);
    // One more by hand, on the far side, for a while.
    let (lon, lat) = at(0.0, 150.0);
    let payload = serde_json::json!({ "circle": { "lat": lat, "lon": lon, "radius_m": 15.0 }, "minutes": 4, "name": "North" });
    let fence = Intent {
        agent: None,
        ..intent(&original, 1, codes::GEOFENCE_OPEN, payload)
    };
    original.apply_intent(&fence).unwrap();
    run(&mut original, 1_501, 1_500);
    assert_eq!(geofences_in(&original.overlays()).1.len(), 2);

    let mut cells = Vec::new();
    original.snapshot(&mut cells, usize::MAX);
    let mut restored = drones(Profile::Gold);
    restored.reset(&blob).unwrap();
    restored.load_cells(cells).unwrap();
    restored.restore_fields(&original.fields()).unwrap();
    assert_eq!(restored.overlays(), original.overlays());

    // The hand-made fence closes by itself in this stretch.
    let a = run(&mut original, 3_001, 2_000);
    let b = run(&mut restored, 3_001, 2_000);
    assert!(a
        .iter()
        .any(|r| r.kind == ReportKind::Geofence && r.detail["action"] == "close"));
    assert_eq!(a, b);
    assert_eq!(hash(&original), hash(&restored));
}

#[test]
fn a_courier_skips_orders_to_fenced_addresses() {
    let mut model = drones(Profile::Gold);
    model.reset(&courier_blob()).unwrap();
    // Over the address 500 m south: nothing goes there while it is open.
    open_circle(&mut model, 0.0, -500.0, 60.0).unwrap();
    let reports = run(&mut model, 1, 9_000);
    let south = model.world().book().addresses[2].clone();
    let deliveries: Vec<&Report> = reports
        .iter()
        .filter(|r| r.kind == ReportKind::Delivery)
        .collect();
    assert!(deliveries.len() >= 2);
    assert!(deliveries
        .iter()
        .all(|d| (d.lat_e7, d.lon_e7) != (south.lat_e7, south.lon_e7)));
    let queued = &model.world().dispatch().queues[0];
    assert!(queued.iter().any(|o| o.address == 2), "its orders wait");
}

#[test]
fn a_ground_station_fence_plan_replaces_only_its_own_fences_and_a_repeat_changes_nothing() {
    let mut model = drones(Profile::Gold);
    open_circle(&mut model, 300.0, 300.0, 30.0).unwrap();
    run(&mut model, 1, 1);
    let circle = |e: f64, n: f64| {
        let (lon, lat) = at(e, n);
        serde_json::json!({ "circle": { "lat": lat, "lon": lon, "radius_m": 40.0 } })
    };
    let set = |model: &mut Drones, fences: Vec<serde_json::Value>| {
        let mut plan = intent(
            model,
            1,
            codes::GEOFENCE_SET,
            serde_json::json!({ "fences": fences }),
        );
        plan.agent = None;
        model.apply_intent(&plan).unwrap();
    };
    let ids = |model: &Drones| -> Vec<(u32, String)> {
        geofences_in(&model.overlays())
            .1
            .into_iter()
            .map(|f| (f.id, f.source))
            .collect()
    };
    set(
        &mut model,
        vec![circle(-300.0, 300.0), circle(-300.0, -300.0)],
    );
    run(&mut model, 2, 1);
    let first = ids(&model);
    assert_eq!(first.len(), 3);
    assert_eq!(first.iter().filter(|f| f.1 == "gcs").count(), 2);
    let rev = geofences_in(&model.overlays()).0;

    // The same plan again: nothing opens or closes.
    set(
        &mut model,
        vec![circle(-300.0, 300.0), circle(-300.0, -300.0)],
    );
    run(&mut model, 3, 1);
    assert_eq!(ids(&model), first);
    assert_eq!(geofences_in(&model.overlays()).0, rev);

    // A smaller plan closes the one left out, and never the intent's fence.
    set(&mut model, vec![circle(-300.0, 300.0)]);
    run(&mut model, 4, 1);
    let now = ids(&model);
    assert_eq!(now.len(), 2);
    assert!(now.contains(&first[0]) && now.contains(&first[1]));

    // One bad fence refuses the whole plan.
    let mut bad = intent(
        &model,
        1,
        codes::GEOFENCE_SET,
        serde_json::json!({ "fences": [circle(-300.0, -300.0), circle(10.0, 0.0)] }),
    );
    bad.agent = None;
    let err = model.apply_intent(&bad).unwrap_err();
    assert!(err.to_string().contains("fence 1"), "{err}");
}

#[test]
fn a_fence_can_wait_to_open_close_when_its_time_is_up_and_be_remembered() {
    let mut model = drones(Profile::Gold);
    let (lon, lat) = at(-300.0, 300.0);
    let circle = serde_json::json!({ "lat": lat, "lon": lon, "radius_m": 30.0 });
    let open = |model: &Drones, extra: serde_json::Value| {
        let mut payload = serde_json::json!({ "circle": circle, "name": "Later" });
        payload
            .as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        Intent {
            agent: None,
            ..intent(model, 1, codes::GEOFENCE_OPEN, payload)
        }
    };
    // Nought minutes open, or a nonsense wait, is refused.
    assert!(model
        .apply_intent(&open(&model, serde_json::json!({ "minutes": 0 })))
        .is_err());
    assert!(model
        .apply_intent(&open(&model, serde_json::json!({ "after_minutes": -2 })))
        .is_err());

    // In a minute (600 ticks), for a minute.
    model
        .apply_intent(&open(
            &model,
            serde_json::json!({ "after_minutes": 1, "minutes": 1 }),
        ))
        .unwrap();
    run(&mut model, 1, 1);
    let g = &model.overlays()["geofences"];
    assert_eq!(g["fences"], serde_json::json!([]));
    assert_eq!(g["scheduled"][0]["opened_tick"], 601);
    assert_eq!(g["scheduled"][0]["closes_tick"], 1201);

    // A checkpoint while it waits keeps it waiting.
    let mut cells = Vec::new();
    model.snapshot(&mut cells, usize::MAX);
    let mut restored = drones(Profile::Gold);
    restored.load_cells(cells).unwrap();
    restored.restore_fields(&model.fields()).unwrap();
    assert_eq!(restored.overlays(), model.overlays());

    let reports = run(&mut model, 2, 600);
    assert!(reports.iter().any(|r| r.kind == ReportKind::Geofence
        && r.detail["action"] == "open"
        && r.detail["reason"] == "scheduled"
        && r.tick == 601));
    assert_eq!(geofences_in(&model.overlays()).1[0].name, "Later");
    run(&mut model, 602, 600);
    let g = &model.overlays()["geofences"];
    assert_eq!(g["fences"], serde_json::json!([]));
    assert_eq!(g["previous"][0]["closed"], "expired");
    assert_eq!(g["previous"][0]["closed_tick"], 1201);
    assert_eq!(g["previous"][0]["name"], "Later");

    // Cancelled before it opens, it is remembered too.
    model
        .apply_intent(&open(&model, serde_json::json!({ "after_minutes": 5 })))
        .unwrap();
    run(&mut model, 1202, 1);
    let id = model.overlays()["geofences"]["scheduled"][0]["id"]
        .as_u64()
        .unwrap();
    let cancel = intent(
        &model,
        1,
        codes::GEOFENCE_CLOSE,
        serde_json::json!({ "id": id }),
    );
    model.apply_intent(&cancel).unwrap();
    run(&mut model, 1203, 1);
    let g = &model.overlays()["geofences"];
    assert_eq!(g["scheduled"], serde_json::json!([]));
    assert_eq!(g["previous"][0]["closed"], "cancelled");
    assert_eq!(g["previous"].as_array().unwrap().len(), 2);
}

#[test]
fn an_open_fence_can_be_given_more_time_from_now_or_kept_open_until_closed() {
    let mut model = drones(Profile::Gold);
    let (lon, lat) = at(-300.0, 300.0);
    let payload =
        serde_json::json!({ "circle": { "lat": lat, "lon": lon, "radius_m": 30.0 }, "minutes": 1 });
    model
        .apply_intent(&Intent {
            agent: None,
            ..intent(&model, 1, codes::GEOFENCE_OPEN, payload)
        })
        .unwrap();
    run(&mut model, 1, 300);
    let fence = &geofences_in(&model.overlays()).1[0];
    assert_eq!(fence.closes_tick, Some(601));
    let extend = |model: &Drones, minutes: serde_json::Value| {
        intent(
            model,
            1,
            codes::GEOFENCE_EXTEND,
            serde_json::json!({ "id": fence.id, "minutes": minutes }),
        )
    };

    // Two minutes from now (tick 301): it closes at 1501, not 601 + 1200.
    model
        .apply_intent(&extend(&model, serde_json::json!(2)))
        .unwrap();
    let reports = run(&mut model, 301, 1);
    assert!(reports
        .iter()
        .any(|r| r.kind == ReportKind::Geofence && r.detail["action"] == "extend"));
    assert_eq!(
        geofences_in(&model.overlays()).1[0].closes_tick,
        Some(1_501)
    );

    // -1: until it is closed.
    model
        .apply_intent(&extend(&model, serde_json::json!(-1)))
        .unwrap();
    run(&mut model, 302, 2_000);
    let fences = geofences_in(&model.overlays()).1;
    assert_eq!(fences.len(), 1, "still open");
    assert_eq!(fences[0].closes_tick, None);

    // Nonsense, a missing number, or a fence that is not open, is refused.
    assert!(model
        .apply_intent(&extend(&model, serde_json::json!(0)))
        .is_err());
    assert!(model
        .apply_intent(&intent(
            &model,
            1,
            codes::GEOFENCE_EXTEND,
            serde_json::json!({ "id": fence.id })
        ))
        .is_err());
    assert!(model
        .apply_intent(&intent(
            &model,
            1,
            codes::GEOFENCE_EXTEND,
            serde_json::json!({ "id": 99, "minutes": 5 })
        ))
        .is_err());
}

#[test]
fn a_fence_can_be_renamed_open_waiting_or_closed() {
    let mut model = drones(Profile::Gold);
    let (lon, lat) = at(-300.0, 300.0);
    let circle = serde_json::json!({ "lat": lat, "lon": lon, "radius_m": 30.0 });
    for extra in [
        serde_json::json!({}),
        serde_json::json!({ "after_minutes": 10 }),
    ] {
        let mut payload = serde_json::json!({ "circle": circle, "name": "Old" });
        payload
            .as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        model
            .apply_intent(&Intent {
                agent: None,
                ..intent(&model, 1, codes::GEOFENCE_OPEN, payload)
            })
            .unwrap();
    }
    run(&mut model, 1, 1);
    let g = model.overlays()["geofences"].clone();
    let open = g["fences"][0]["id"].as_u64().unwrap();
    let waiting = g["scheduled"][0]["id"].as_u64().unwrap();
    let rename = |model: &Drones, id: u64, name: &str| {
        intent(
            model,
            1,
            codes::GEOFENCE_RENAME,
            serde_json::json!({ "id": id, "name": name }),
        )
    };
    model
        .apply_intent(&rename(&model, open, "  Stadium  "))
        .unwrap();
    model
        .apply_intent(&rename(&model, waiting, "Later"))
        .unwrap();
    let reports = run(&mut model, 2, 1);
    let g = model.overlays()["geofences"].clone();
    assert_eq!(g["fences"][0]["name"], "Stadium", "trimmed");
    assert_eq!(g["scheduled"][0]["name"], "Later");
    assert!(reports
        .iter()
        .any(|r| r.kind == ReportKind::Geofence && r.detail["action"] == "rename"));
    // Renaming changes no shape, so nothing replans.
    assert_eq!(geofences_in(&model.overlays()).0, 1);

    // Closed, then renamed: it reopens under the new name. Empty clears it.
    let close = intent(
        &model,
        1,
        codes::GEOFENCE_CLOSE,
        serde_json::json!({ "id": open }),
    );
    model.apply_intent(&close).unwrap();
    run(&mut model, 3, 1);
    model.apply_intent(&rename(&model, open, "")).unwrap();
    run(&mut model, 4, 1);
    let previous = &model.overlays()["geofences"]["previous"][0];
    assert_eq!(previous["id"], open);
    assert!(previous.get("name").is_none(), "{previous}");

    assert!(model.apply_intent(&rename(&model, 99, "x")).is_err());
    let nameless = intent(
        &model,
        1,
        codes::GEOFENCE_RENAME,
        serde_json::json!({ "id": open }),
    );
    assert!(model.apply_intent(&nameless).is_err());
}
