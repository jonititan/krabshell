//! Route-level tests (spec §15).
//!
//! Each one drives the real router against a real driver, so a handler that
//! forgets to wait for the driver, or answers from a stale view, fails here
//! rather than in a browser.

use axum::body::Body;
use axum::http::{header, Request, StatusCode};
use http_body_util::BodyExt;
use serde_json::{json, Value};
use sim_axum::auth::Auth;
use sim_axum::{router, AppState};
use sim_core::config::Param;
use sim_core::{Config, GeoCell, Profile, ScenarioBlob, ScenarioManifest};
use sim_engine::{Driver, ModelRegistry, ScenarioStore};
use tower::ServiceExt;

const RESOLUTION: i32 = 6;
const TOKEN: &str = "test-token";

struct Harness {
    router: axum::Router,
    _driver: Driver,
    scenarios: std::path::PathBuf,
}

impl Drop for Harness {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.scenarios);
    }
}

fn scenario(profile: Profile, walkers: i64) -> ScenarioBlob {
    let manifest = ScenarioManifest {
        id: "api-test".into(),
        name: "API test".into(),
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
    // Ignored by the walkers model; gives the dashboard a real-time rate (10 tps).
    config
        .model
        .insert("tick_seconds".into(), Param::Float(0.1));
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
        fields: Value::Null,
    }
}

fn harness(profile: Profile, walkers: i64) -> Harness {
    harness_for(scenario(profile, walkers))
}

/// A harness running a scenario pack from `scenarios/`.
fn pack_harness(name: &str) -> Harness {
    let pack = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(name);
    harness_for(sim_engine::read_pack(&pack).unwrap())
}

fn harness_for(blob: ScenarioBlob) -> Harness {
    let dir = std::env::temp_dir().join(format!(
        "krabshell-api-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();

    let driver = Driver::spawn(ModelRegistry::with_builtins(), blob).unwrap();
    let state = AppState::new(
        driver.handle(),
        ScenarioStore::new(&dir),
        Auth::with_token(Some(TOKEN.to_owned())),
    );
    Harness {
        router: router(state),
        _driver: driver,
        scenarios: dir,
    }
}

/// Status, body parsed as JSON (or null), and the raw body text.
async fn call(harness: &Harness, request: Request<Body>) -> (StatusCode, Value, String) {
    let response = harness.router.clone().oneshot(request).await.unwrap();
    let status = response.status();
    let bytes = response.into_body().collect().await.unwrap().to_bytes();
    let text = String::from_utf8_lossy(&bytes).into_owned();
    let json = serde_json::from_slice(&bytes).unwrap_or(Value::Null);
    (status, json, text)
}

fn get(path: &str) -> Request<Body> {
    Request::builder()
        .uri(path)
        .header(header::AUTHORIZATION, format!("Bearer {TOKEN}"))
        .body(Body::empty())
        .unwrap()
}

fn post(path: &str, body: Value) -> Request<Body> {
    Request::builder()
        .method("POST")
        .uri(path)
        .header(header::AUTHORIZATION, format!("Bearer {TOKEN}"))
        .header(header::CONTENT_TYPE, "application/json")
        .body(Body::from(body.to_string()))
        .unwrap()
}

fn delete(path: &str) -> Request<Body> {
    Request::builder()
        .method("DELETE")
        .uri(path)
        .header(header::AUTHORIZATION, format!("Bearer {TOKEN}"))
        .body(Body::empty())
        .unwrap()
}

fn put(path: &str, body: Value) -> Request<Body> {
    Request::builder()
        .method("PUT")
        .uri(path)
        .header(header::AUTHORIZATION, format!("Bearer {TOKEN}"))
        .header(header::CONTENT_TYPE, "application/json")
        .body(Body::from(body.to_string()))
        .unwrap()
}

fn form(path: &str, body: &str) -> Request<Body> {
    Request::builder()
        .method("POST")
        .uri(path)
        .header(header::COOKIE, format!("krabshell_token={TOKEN}"))
        .header(header::CONTENT_TYPE, "application/x-www-form-urlencoded")
        .body(Body::from(body.to_owned()))
        .unwrap()
}

fn page(path: &str) -> Request<Body> {
    Request::builder()
        .uri(path)
        .header(header::COOKIE, format!("krabshell_token={TOKEN}"))
        .body(Body::empty())
        .unwrap()
}

fn london() -> String {
    GeoCell::from_lonlat(-0.1276, 51.5072, RESOLUTION)
        .unwrap()
        .to_hex()
}

fn spawn(id: &str, cell: &str, count: u64) -> Value {
    json!({
        "id": id,
        "cell": cell,
        "actor": "test",
        "verb": "spawn",
        "payload": { "count": count, "kind": "probe" }
    })
}

// ---------------------------------------------------------------- auth

#[tokio::test]
async fn health_is_readable_without_a_token() {
    let h = harness(Profile::Fast, 10);
    let request = Request::builder()
        .uri("/api/v1/health")
        .body(Body::empty())
        .unwrap();
    let (status, body, _) = call(&h, request).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["status"], "ok");
    assert_eq!(body["alive"], true);
}

#[tokio::test]
async fn everything_else_needs_a_token() {
    let h = harness(Profile::Fast, 10);
    let anonymous = Request::builder()
        .uri("/api/v1/run")
        .body(Body::empty())
        .unwrap();
    let (status, body, _) = call(&h, anonymous).await;
    assert_eq!(status, StatusCode::UNAUTHORIZED);
    assert_eq!(body["code"], "unauthorized");

    let wrong = Request::builder()
        .uri("/api/v1/run")
        .header(header::AUTHORIZATION, "Bearer not-the-token")
        .body(Body::empty())
        .unwrap();
    assert_eq!(call(&h, wrong).await.0, StatusCode::UNAUTHORIZED);

    assert_eq!(call(&h, get("/api/v1/run")).await.0, StatusCode::OK);
}

// ----------------------------------------------------------------- run

#[tokio::test]
async fn run_status_describes_the_loaded_world() {
    let h = harness(Profile::Gold, 25);
    let (status, body, _) = call(&h, get("/api/v1/run")).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["mode"], "paused");
    assert_eq!(body["tick"], 0);
    assert_eq!(body["scenario_id"], "api-test");
    assert_eq!(body["profile"], "gold");
    assert_eq!(body["agent_count"], 25);
}

#[tokio::test]
async fn stepping_answers_with_the_world_after_the_step() {
    let h = harness(Profile::Fast, 10);
    let (status, body, _) = call(&h, post("/api/v1/run/step", json!({"n": 12}))).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(
        body["tick"], 12,
        "the response must not describe the frame before the step"
    );
    assert_eq!(body["mode"], "paused");
}

#[tokio::test]
async fn a_zero_step_is_refused() {
    let h = harness(Profile::Fast, 10);
    let (status, body, _) = call(&h, post("/api/v1/run/step", json!({"n": 0}))).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["code"], "bad_request");
}

#[tokio::test]
async fn start_and_pause_move_the_world_and_stop_it() {
    let h = harness(Profile::Fast, 10);
    let (status, body, _) = call(&h, post("/api/v1/run/start", json!({}))).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["mode"], "running");

    tokio::time::sleep(std::time::Duration::from_millis(120)).await;

    let (_, paused, _) = call(&h, post("/api/v1/run/pause", Value::Null)).await;
    assert_eq!(paused["mode"], "paused");
    let stopped_at = paused["tick"].as_u64().unwrap();
    assert!(stopped_at > 0);

    tokio::time::sleep(std::time::Duration::from_millis(50)).await;
    let (_, later, _) = call(&h, get("/api/v1/run")).await;
    assert_eq!(later["tick"].as_u64().unwrap(), stopped_at);
}

#[tokio::test]
async fn a_nonsense_rate_is_refused_with_a_problem_body() {
    let h = harness(Profile::Fast, 10);
    let response = h
        .router
        .clone()
        .oneshot(post("/api/v1/run/rate", json!({"target_tps": -4.0})))
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    assert_eq!(
        response.headers()[header::CONTENT_TYPE],
        "application/problem+json"
    );
}

#[tokio::test]
async fn resetting_returns_to_tick_zero() {
    let h = harness(Profile::Fast, 10);
    call(&h, post("/api/v1/run/step", json!({"n": 5}))).await;
    let (_, body, _) = call(&h, post("/api/v1/run/reset", Value::Null)).await;
    assert_eq!(body["tick"], 0);
    assert_eq!(body["agent_count"], 10);
}

// -------------------------------------------------------------- config

#[tokio::test]
async fn the_config_download_is_the_effective_config_not_a_file() {
    let h = harness(Profile::Fast, 10);
    call(&h, post("/api/v1/run/rate", json!({"target_tps": 42.0}))).await;

    let response = h
        .router
        .clone()
        .oneshot(get("/api/v1/config/download"))
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let disposition = response.headers()[header::CONTENT_DISPOSITION]
        .to_str()
        .unwrap()
        .to_owned();
    assert!(disposition.contains("attachment"));

    let bytes = response.into_body().collect().await.unwrap().to_bytes();
    let text = String::from_utf8(bytes.to_vec()).unwrap();
    assert!(text.contains("target_tps = 42.0"), "{text}");
    toml::from_str::<Config>(&text).expect("the download parses back into a Config");
}

#[tokio::test]
async fn a_config_change_that_needs_a_reset_says_so() {
    let h = harness(Profile::Fast, 10);
    call(&h, post("/api/v1/run/step", json!({"n": 4}))).await;
    let (_, mut next, _) = call(&h, get("/api/v1/config")).await;
    next["grid"]["resolution"] = json!(7);

    let (status, body, text) = call(&h, put("/api/v1/config", next)).await;
    assert_eq!(status, StatusCode::OK, "{text}");
    assert_eq!(body["reset_fields"], json!(["grid.resolution"]));
    assert_eq!(body["tick"], 0);
    assert_eq!(body["config"]["grid"]["resolution"], 7);
}

#[tokio::test]
async fn a_cosmetic_config_change_keeps_the_world() {
    let h = harness(Profile::Fast, 10);
    call(&h, post("/api/v1/run/step", json!({"n": 4}))).await;
    let (_, mut next, _) = call(&h, get("/api/v1/config")).await;
    next["run"]["publish_hz"] = json!(5.0);

    let (status, body, _) = call(&h, put("/api/v1/config", next)).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["reset_fields"], json!([]));
    assert_eq!(body["tick"], 4);
}

#[tokio::test]
async fn an_invalid_config_is_refused_before_it_reaches_the_driver() {
    let h = harness(Profile::Fast, 10);
    let (_, mut config, _) = call(&h, get("/api/v1/config")).await;
    config["run"]["publish_hz"] = json!(0.0);

    let (status, body, _) = call(&h, put("/api/v1/config", config)).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["code"], "bad_config");

    let (_, after, _) = call(&h, get("/api/v1/config")).await;
    assert_eq!(after["run"]["publish_hz"], 200.0);
}

// ------------------------------------------------------------- intents

#[tokio::test]
async fn an_intent_lands_in_the_cell_it_names() {
    let h = harness(Profile::Fast, 0);
    let cell = london();

    let (status, body, _) = call(
        &h,
        post(
            "/api/v1/intents",
            spawn("3f2504e0-4f89-41d3-9a0c-0305e82c3301", &cell, 3),
        ),
    )
    .await;
    assert_eq!(status, StatusCode::OK, "{body}");
    assert_eq!(
        body["status"], "queued",
        "a paused world has not applied it yet"
    );

    call(&h, post("/api/v1/run/step", json!({"n": 1}))).await;

    let (status, bag, _) = call(&h, get(&format!("/api/v1/state/cells/{cell}"))).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(bag["objects"].as_array().unwrap().len(), 3);
    assert_eq!(bag["objects"][0]["kind"], "probe");
}

#[tokio::test]
async fn an_intent_on_a_running_world_reports_the_tick_it_landed_on() {
    let h = harness(Profile::Fast, 0);
    call(&h, post("/api/v1/run/start", json!({"target_tps": 200.0}))).await;

    let (status, body, _) = call(
        &h,
        post(
            "/api/v1/intents",
            spawn("3f2504e0-4f89-41d3-9a0c-0305e82c3309", &london(), 1),
        ),
    )
    .await;
    assert_eq!(status, StatusCode::OK, "{body}");
    assert_eq!(body["status"], "applied");
    assert!(body["applied_tick"].as_u64().unwrap() > 0);
}

#[tokio::test]
async fn the_same_intent_id_is_only_applied_once() {
    let h = harness(Profile::Fast, 0);
    let body = spawn("3f2504e0-4f89-41d3-9a0c-0305e82c3302", &london(), 1);

    call(&h, post("/api/v1/intents", body.clone())).await;
    let (status, problem, _) = call(&h, post("/api/v1/intents", body)).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(problem["code"], "duplicate_intent");

    call(&h, post("/api/v1/run/step", json!({"n": 1}))).await;
    let (_, run, _) = call(&h, get("/api/v1/run")).await;
    assert_eq!(run["agent_count"], 1);
}

#[tokio::test]
async fn a_stale_intent_is_refused_with_its_reason() {
    let h = harness(Profile::Fast, 0);
    call(&h, post("/api/v1/run/step", json!({"n": 5}))).await;

    let mut body = spawn("3f2504e0-4f89-41d3-9a0c-0305e82c3304", &london(), 1);
    body["tick"] = json!(3);
    let (status, problem, _) = call(&h, post("/api/v1/intents", body)).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(problem["code"], "stale_intent");
    assert_eq!(problem["meta"]["committed"], 5);
}

#[tokio::test]
async fn an_intent_at_the_wrong_resolution_is_refused() {
    let h = harness(Profile::Fast, 0);
    let wrong = GeoCell::from_lonlat(-0.1276, 51.5072, RESOLUTION + 2)
        .unwrap()
        .to_hex();
    let (status, body, _) = call(
        &h,
        post(
            "/api/v1/intents",
            spawn("3f2504e0-4f89-41d3-9a0c-0305e82c3303", &wrong, 1),
        ),
    )
    .await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["code"], "cell_out_of_bounds");
}

#[tokio::test]
async fn an_oversized_intent_is_refused_before_parsing() {
    let h = harness(Profile::Fast, 0);
    let mut body = spawn("3f2504e0-4f89-41d3-9a0c-0305e82c3305", &london(), 1);
    body["payload"]["padding"] = json!("x".repeat(20_000));
    let (status, problem, _) = call(&h, post("/api/v1/intents", body)).await;
    assert_eq!(status, StatusCode::PAYLOAD_TOO_LARGE);
    assert_eq!(problem["code"], "payload_too_large");
}

#[tokio::test]
async fn a_malformed_intent_is_a_bad_request() {
    let h = harness(Profile::Fast, 0);
    let (status, problem, _) = call(&h, post("/api/v1/intents", json!({"verb": "spawn"}))).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(problem["code"], "bad_request");
}

#[tokio::test]
async fn a_flooding_actor_is_rate_limited_without_affecting_others() {
    let h = harness(Profile::Fast, 0);
    let (_, mut config, _) = call(&h, get("/api/v1/config")).await;
    config["run"]["intent_rate_per_actor"] = json!(2.0);
    assert_eq!(
        call(&h, put("/api/v1/config", config)).await.0,
        StatusCode::OK
    );
    // Run the world so each accepted intent is acked within a tick. On a paused
    // world every request waits out its ack timeout, which is long enough for
    // the bucket to refill between requests.
    call(&h, post("/api/v1/run/start", json!({}))).await;

    let submit = |actor: &str, n: u32| {
        post(
            "/api/v1/intents",
            json!({
                "id": uuid::Uuid::new_v4(),
                "cell": london(),
                "actor": actor,
                "verb": "spawn",
                "payload": { "count": 1, "n": n }
            }),
        )
    };

    let mut limited = 0;
    for n in 0..10 {
        let (status, body, _) = call(&h, submit("flood", n)).await;
        if status == StatusCode::TOO_MANY_REQUESTS {
            assert_eq!(body["code"], "rate_limited");
            limited += 1;
        }
    }
    assert_eq!(limited, 6, "a rate of 2/s allows a burst of 4");

    let (status, _, _) = call(&h, submit("polite", 0)).await;
    assert_eq!(status, StatusCode::OK, "another actor still gets through");
}

#[tokio::test]
async fn the_intent_log_is_only_offered_for_gold_runs() {
    let fast = harness(Profile::Fast, 5);
    let (status, body, _) = call(&fast, get("/api/v1/intents/log")).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["code"], "bad_request");

    let gold = harness(Profile::Gold, 0);
    call(
        &gold,
        post(
            "/api/v1/intents",
            spawn("3f2504e0-4f89-41d3-9a0c-0305e82c3306", &london(), 2),
        ),
    )
    .await;
    call(&gold, post("/api/v1/run/step", json!({"n": 1}))).await;

    let (status, _, text) = call(&gold, get("/api/v1/intents/log")).await;
    assert_eq!(status, StatusCode::OK);
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines.len(), 1, "{text}");
    let entry: Value = serde_json::from_str(lines[0]).unwrap();
    assert_eq!(entry["tick"], 1);
    assert_eq!(entry["intent"]["verb"], "spawn");
}

// --------------------------------------------------------------- state

#[tokio::test]
async fn a_malformed_cell_id_is_a_problem_not_a_panic() {
    let h = harness(Profile::Fast, 5);
    let (status, body, _) = call(&h, get("/api/v1/state/cells/not-a-cell")).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["code"], "bad_cell");
}

#[tokio::test]
async fn an_empty_cell_is_an_empty_bag() {
    let h = harness(Profile::Fast, 0);
    let (status, body, _) = call(&h, get(&format!("/api/v1/state/cells/{}", london()))).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["objects"], json!([]));
}

#[tokio::test]
async fn state_can_be_selected_by_cell_list_or_disk() {
    let h = harness(Profile::Fast, 60);
    let cell = london();

    let (status, body, _) = call(&h, get(&format!("/api/v1/state?cells={cell}"))).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["cells"].as_array().unwrap().len(), 1);
    assert_eq!(body["cells"][0]["cell"], cell);

    let (status, disk, _) = call(&h, get(&format!("/api/v1/state?center={cell}&k=2"))).await;
    assert_eq!(status, StatusCode::OK);
    let total: usize = disk["cells"]
        .as_array()
        .unwrap()
        .iter()
        .map(|b| b["objects"].as_array().unwrap().len())
        .sum();
    assert_eq!(total, 60, "every walker was seeded inside a k=2 disk");
}

#[tokio::test]
async fn the_map_aggregates_to_a_coarser_resolution_with_polygons() {
    let h = harness(Profile::Fast, 200);
    call(&h, post("/api/v1/run/step", json!({"n": 10}))).await;

    let (status, body, _) = call(&h, get("/api/v1/state/map?res=3")).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["resolution"], 3);

    let cells = body["cells"].as_array().unwrap();
    assert!(!cells.is_empty());
    assert!(cells[0]["boundary"].as_array().unwrap().len() >= 3);
    assert!(body["max_count"].as_u64().unwrap() > 0);

    let total: u64 = cells.iter().map(|c| c["count"].as_u64().unwrap()).sum();
    assert_eq!(total, 200, "aggregation must not lose or duplicate agents");
}

#[tokio::test]
async fn a_lookup_turns_a_coordinate_into_a_cell_in_this_world() {
    let h = harness(Profile::Fast, 5);
    let (status, body, _) = call(&h, get("/api/v1/cells/lookup?lon=-0.1276&lat=51.5072")).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["resolution"], RESOLUTION);
    assert_eq!(body["in_world"], true);
    assert_eq!(body["cell"], london());
    assert!(body["boundary"].as_array().unwrap().len() >= 3);
}

#[tokio::test]
async fn a_checkpoint_downloads_and_restores_to_the_same_tick_and_hash() {
    let h = harness(Profile::Gold, 50);
    call(&h, post("/api/v1/run/step", json!({"n": 20}))).await;

    let response = h
        .router
        .clone()
        .oneshot(get("/api/v1/state/download"))
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let bytes = response.into_body().collect().await.unwrap().to_bytes();
    let checkpoint: Value = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(checkpoint["meta"]["tick"], 20);
    assert!(checkpoint["meta"]["hash"].is_number());

    call(&h, post("/api/v1/run/step", json!({"n": 10}))).await;
    assert_eq!(call(&h, get("/api/v1/run")).await.1["tick"], 30);

    let upload = Request::builder()
        .method("POST")
        .uri("/api/v1/state/upload")
        .header(header::AUTHORIZATION, format!("Bearer {TOKEN}"))
        .body(Body::from(bytes.to_vec()))
        .unwrap();
    let (status, body, text) = call(&h, upload).await;
    assert_eq!(status, StatusCode::OK, "{text}");
    assert_eq!(body["tick"], 20);
    assert_eq!(body["hash"], checkpoint["meta"]["hash"]);
}

#[tokio::test]
async fn a_tampered_checkpoint_is_refused() {
    let h = harness(Profile::Gold, 20);
    call(&h, post("/api/v1/run/step", json!({"n": 3}))).await;
    let response = h
        .router
        .clone()
        .oneshot(get("/api/v1/state/download"))
        .await
        .unwrap();
    let bytes = response.into_body().collect().await.unwrap().to_bytes();
    let mut checkpoint: Value = serde_json::from_slice(&bytes).unwrap();
    checkpoint["cells"][0]["objects"][0]["id"] = json!(999_999);

    let upload = Request::builder()
        .method("POST")
        .uri("/api/v1/state/upload")
        .header(header::AUTHORIZATION, format!("Bearer {TOKEN}"))
        .body(Body::from(checkpoint.to_string()))
        .unwrap();
    let (status, body, _) = call(&h, upload).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert!(body["detail"].as_str().unwrap().contains("hash"), "{body}");
    assert_eq!(call(&h, get("/api/v1/run")).await.1["tick"], 3);
}

#[tokio::test]
async fn restoring_a_running_world_is_refused() {
    let h = harness(Profile::Fast, 5);
    call(&h, post("/api/v1/run/start", json!({}))).await;

    let upload = Request::builder()
        .method("POST")
        .uri("/api/v1/state/upload")
        .header(header::AUTHORIZATION, format!("Bearer {TOKEN}"))
        .body(Body::from("{}"))
        .unwrap();
    let (status, body, _) = call(&h, upload).await;
    assert_eq!(status, StatusCode::CONFLICT);
    assert_eq!(body["code"], "requires_pause");
}

// ------------------------------------------------------------- metrics

#[tokio::test]
async fn metrics_are_prometheus_text() {
    let h = harness(Profile::Fast, 7);
    let (status, _, text) = call(&h, get("/api/v1/metrics")).await;
    assert_eq!(status, StatusCode::OK);
    assert!(text.contains("# TYPE krabshell_tick gauge"));
    assert!(text.contains("krabshell_agents 7"));
    assert!(text.contains("krabshell_driver_alive 1"));
}

// ----------------------------------------------------------- scenarios

#[tokio::test]
async fn an_unknown_scenario_is_a_404_with_a_stable_code() {
    let h = harness(Profile::Fast, 5);
    let (status, body, _) = call(&h, post("/api/v1/scenarios/nope/load", Value::Null)).await;
    assert_eq!(status, StatusCode::NOT_FOUND);
    assert_eq!(body["code"], "unknown_scenario");
}

#[tokio::test]
async fn scenario_ids_cannot_walk_out_of_the_store() {
    let h = harness(Profile::Fast, 5);
    let (status, body, _) = call(
        &h,
        post("/api/v1/scenarios/..%2F..%2Fetc/load", Value::Null),
    )
    .await;
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "got {status}: {body}"
    );
    assert_ne!(body["code"], "internal");
}

#[tokio::test]
async fn a_pack_on_disk_is_listed_and_loadable() {
    let h = harness(Profile::Fast, 5);
    let pack = h.scenarios.join("tiny");
    std::fs::create_dir_all(&pack).unwrap();
    std::fs::write(
        pack.join("manifest.toml"),
        "id = \"tiny\"\nname = \"Tiny\"\nmodel = \"walkers\"\nresolution = 5\n",
    )
    .unwrap();
    std::fs::write(pack.join("params.toml"), "walkers = 9\n").unwrap();

    let (status, list, _) = call(&h, get("/api/v1/scenarios")).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(list[0]["id"], "tiny");

    let (status, run, text) = call(&h, post("/api/v1/scenarios/tiny/load", Value::Null)).await;
    assert_eq!(status, StatusCode::OK, "{text}");
    assert_eq!(run["scenario_id"], "tiny");
    assert_eq!(run["agent_count"], 9);
    assert_eq!(run["meta"]["resolution"], 5);
}

#[tokio::test]
async fn a_zip_pack_can_be_uploaded() {
    use std::io::Write;
    let h = harness(Profile::Fast, 5);

    let mut archive = Vec::new();
    {
        let mut zip = zip::ZipWriter::new(std::io::Cursor::new(&mut archive));
        let options: zip::write::FileOptions<'_, ()> = zip::write::FileOptions::default();
        zip.start_file("manifest.toml", options).unwrap();
        zip.write_all(b"id = \"zipped\"\nname = \"Zipped\"\nmodel = \"walkers\"\nresolution = 5\n")
            .unwrap();
        zip.finish().unwrap();
    }

    let boundary = "krabshellboundary";
    let mut body = Vec::new();
    body.extend_from_slice(
        format!("--{boundary}\r\nContent-Disposition: form-data; name=\"id\"\r\n\r\nzipped\r\n")
            .as_bytes(),
    );
    body.extend_from_slice(
        format!(
            "--{boundary}\r\nContent-Disposition: form-data; name=\"pack\"; filename=\"zipped.zip\"\r\n\
             Content-Type: application/zip\r\n\r\n"
        )
        .as_bytes(),
    );
    body.extend_from_slice(&archive);
    body.extend_from_slice(format!("\r\n--{boundary}--\r\n").as_bytes());

    let request = Request::builder()
        .method("POST")
        .uri("/api/v1/scenarios")
        .header(header::AUTHORIZATION, format!("Bearer {TOKEN}"))
        .header(
            header::CONTENT_TYPE,
            format!("multipart/form-data; boundary={boundary}"),
        )
        .body(Body::from(body))
        .unwrap();
    let (status, result, text) = call(&h, request).await;
    assert_eq!(status, StatusCode::OK, "{text}");
    assert_eq!(result["scenario"]["id"], "zipped");
    assert!(h.scenarios.join("zipped/manifest.toml").is_file());
}

// ----------------------------------------------------------- dashboard

#[tokio::test]
async fn an_unauthenticated_page_redirects_to_the_login_screen() {
    let h = harness(Profile::Fast, 5);
    let response = h
        .router
        .clone()
        .oneshot(Request::builder().uri("/run").body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert!(response.status().is_redirection());
    assert_eq!(response.headers()[header::LOCATION], "/login");
}

#[tokio::test]
async fn logging_in_sets_the_cookie_the_pages_accept() {
    let h = harness(Profile::Fast, 5);
    let bad = Request::builder()
        .method("POST")
        .uri("/login")
        .header(header::CONTENT_TYPE, "application/x-www-form-urlencoded")
        .body(Body::from("token=nope"))
        .unwrap();
    let (status, _, html) = call(&h, bad).await;
    assert_eq!(status, StatusCode::UNAUTHORIZED);
    assert!(html.contains("not accepted"));

    let good = Request::builder()
        .method("POST")
        .uri("/login")
        .header(header::CONTENT_TYPE, "application/x-www-form-urlencoded")
        .body(Body::from(format!("token={TOKEN}")))
        .unwrap();
    let response = h.router.clone().oneshot(good).await.unwrap();
    assert!(response.status().is_redirection());
    let cookie = response.headers()[header::SET_COOKIE].to_str().unwrap();
    assert!(cookie.starts_with(&format!("krabshell_token={TOKEN}")));
    assert!(cookie.contains("HttpOnly"));
}

#[tokio::test]
async fn every_dashboard_screen_renders() {
    let h = harness(Profile::Gold, 30);
    for path in ["/run", "/scenarios", "/state", "/intents", "/io"] {
        let (status, _, html) = call(&h, page(path)).await;
        assert_eq!(status, StatusCode::OK, "{path} did not render");
        assert!(html.contains("krab"), "{path} is missing the layout");
        assert!(
            html.contains("id=\"control\""),
            "{path} is missing the control panel"
        );
    }
}

#[tokio::test]
async fn fragments_render_for_htmx() {
    let h = harness(Profile::Fast, 30);
    let cell = london();
    for path in [
        "/fragments/control".to_owned(),
        "/fragments/scenarios".to_owned(),
        "/fragments/intents".to_owned(),
        format!("/fragments/cell?cell={cell}"),
    ] {
        let (status, _, html) = call(&h, page(&path)).await;
        assert_eq!(status, StatusCode::OK, "{path}");
        assert!(
            !html.contains("<html"),
            "{path} should be a fragment, not a page"
        );
    }
}

#[tokio::test]
async fn the_control_panel_can_step_start_and_pause_the_world() {
    let h = harness(Profile::Fast, 10);

    let (status, _, html) = call(&h, form("/actions/step", "n=7")).await;
    assert_eq!(status, StatusCode::OK);
    assert!(html.contains("Stepped 7"), "{html}");
    assert_eq!(call(&h, get("/api/v1/run")).await.1["tick"], 7);

    let (_, _, html) = call(&h, form("/actions/start", "tps=")).await;
    assert!(html.contains("maximum speed"), "{html}");

    let (_, _, html) = call(&h, form("/actions/pause", "")).await;
    assert!(html.contains("Paused."), "{html}");
}

#[tokio::test]
async fn the_speed_choice_sets_the_rate_without_starting_and_start_keeps_it() {
    let h = harness(Profile::Fast, 5);
    let rate = |v: &Value| v["target_tps"].clone();

    let (status, _, html) = call(&h, form("/actions/speed", "speed=realtime&tps=")).await;
    assert_eq!(status, StatusCode::OK);
    assert!(html.contains("real time (10 tps)"), "{html}");
    assert!(html.contains(r#"value="realtime" checked"#), "{html}");
    let run = call(&h, get("/api/v1/run")).await.1;
    assert_eq!(rate(&run), 10.0);
    assert_eq!(run["mode"], "paused");

    let (_, _, html) = call(&h, form("/actions/speed", "speed=custom&tps=25")).await;
    assert!(html.contains("25 tps (2.5× real time)"), "{html}");
    assert!(html.contains(r#"value="custom" checked"#), "{html}");
    assert_eq!(rate(&call(&h, get("/api/v1/run")).await.1), 25.0);

    // Start from the dashboard keeps the chosen speed.
    let (_, _, html) = call(&h, form("/actions/start", "")).await;
    assert!(html.contains("Running at 25 tps"), "{html}");
    let run = call(&h, get("/api/v1/run")).await.1;
    assert_eq!(rate(&run), 25.0);
    assert_eq!(run["mode"], "running");

    let (_, _, html) = call(&h, form("/actions/speed", "speed=max&tps=25")).await;
    assert!(html.contains("Speed set to maximum speed"), "{html}");
    let run = call(&h, get("/api/v1/run")).await.1;
    assert_eq!(rate(&run), Value::Null);
    assert_eq!(run["mode"], "running");
}

#[tokio::test]
async fn a_custom_speed_must_be_a_positive_rate() {
    let h = harness(Profile::Fast, 5);
    call(&h, form("/actions/speed", "speed=custom&tps=40")).await;
    let (_, _, html) = call(&h, form("/actions/speed", "speed=custom&tps=")).await;
    assert!(html.contains("Enter a custom rate"), "{html}");
    let (_, _, html) = call(&h, form("/actions/speed", "speed=custom&tps=-3")).await;
    assert!(html.contains("is not a tick rate"), "{html}");
    assert_eq!(call(&h, get("/api/v1/run")).await.1["target_tps"], 40.0);
}

#[tokio::test]
async fn a_bad_tick_rate_in_the_form_is_reported_not_applied() {
    let h = harness(Profile::Fast, 5);
    let (status, _, html) = call(&h, form("/actions/start", "tps=quickly")).await;
    assert_eq!(status, StatusCode::OK);
    assert!(html.contains("is not a tick rate"), "{html}");
    assert_eq!(call(&h, get("/api/v1/run")).await.1["mode"], "paused");
}

#[tokio::test]
async fn the_intent_form_queues_an_intent_that_lands() {
    let h = harness(Profile::Fast, 0);
    let (status, _, html) = call(
        &h,
        form(
            "/actions/intent",
            &format!("cell={}&verb=spawn&count=4&kind=scout", london()),
        ),
    )
    .await;
    assert_eq!(status, StatusCode::OK);
    assert!(html.contains("Queued"), "{html}");

    call(&h, post("/api/v1/run/step", json!({"n": 1}))).await;
    assert_eq!(call(&h, get("/api/v1/run")).await.1["agent_count"], 4);
}

#[tokio::test]
async fn dashboard_assets_are_served_from_the_binary() {
    let h = harness(Profile::Fast, 5);
    for (path, needle) in [
        ("/assets/app.css", "krabshell operator console"),
        ("/assets/map.js", "Occupancy map"),
        ("/assets/live.js", "WebSocket"),
        ("/assets/htmx.min.js", "htmx"),
    ] {
        let request = Request::builder().uri(path).body(Body::empty()).unwrap();
        let (status, _, text) = call(&h, request).await;
        assert_eq!(status, StatusCode::OK, "{path}");
        assert!(text.contains(needle), "{path} did not look right");
    }
}

#[tokio::test]
async fn inspecting_a_coarse_cell_rolls_up_every_agent_inside_it() {
    let h = harness(Profile::Fast, 0);
    let fine = GeoCell::from_lonlat(-0.1276, 51.5072, RESOLUTION).unwrap();
    let parent = fine.parent(Some(RESOLUTION - 4)).unwrap();
    // A sibling under the same parent, so the roll-up spans two occupancy cells.
    let sibling = parent
        .children(Some(RESOLUTION))
        .unwrap()
        .into_iter()
        .find(|c| *c != fine)
        .unwrap();

    for (id, cell, count) in [
        ("3f2504e0-4f89-41d3-9a0c-0305e82c3310", fine, 3),
        ("3f2504e0-4f89-41d3-9a0c-0305e82c3311", sibling, 2),
    ] {
        let (status, body, _) = call(
            &h,
            post("/api/v1/intents", spawn(id, &cell.to_hex(), count)),
        )
        .await;
        assert_eq!(status, StatusCode::OK, "{body}");
    }
    call(&h, post("/api/v1/run/step", json!({"n": 1}))).await;

    let (status, _, html) = call(
        &h,
        page(&format!("/fragments/cell?cell={}", parent.to_hex())),
    )
    .await;
    assert_eq!(status, StatusCode::OK);
    assert!(html.contains("data-coarse"), "{html}");
    assert!(html.contains("Occupancy cell"), "{html}");
    assert_eq!(
        html.matches("<span class=\"tag\">probe</span>").count(),
        5,
        "{html}"
    );
    assert!(html.contains(&fine.to_hex()) && html.contains(&sibling.to_hex()));

    // At occupancy resolution it is still just the one bag, with no roll-up.
    let (_, _, html) = call(&h, page(&format!("/fragments/cell?cell={}", fine.to_hex()))).await;
    assert!(!html.contains("data-coarse"));
    assert_eq!(
        html.matches("<span class=\"tag\">probe</span>").count(),
        3,
        "{html}"
    );
}

#[tokio::test]
async fn the_state_page_offers_basemaps_with_none_as_the_default() {
    let h = harness(Profile::Fast, 0);
    let (status, _, html) = call(&h, page("/state")).await;
    assert_eq!(status, StatusCode::OK);
    assert!(html.contains("id=\"map-basemap\""));
    assert!(html.contains("<option value=\"\">None</option>"));
    assert!(html.contains("tile.openstreetmap.org"));
}

// ------------------------------------------------------------ geofences

#[tokio::test]
async fn a_model_without_geofences_lists_none() {
    let h = harness(Profile::Fast, 10);
    let (status, body, _) = call(&h, get("/api/v1/geofences")).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["fences"], json!([]));
    assert_eq!(body["margin_m"], 0.0);
}

#[tokio::test]
async fn geofences_open_and_close_over_the_api_and_the_page_shows_them() {
    let h = pack_harness("london-patrol");
    let (status, page_body, _) = call(&h, page("/geofences")).await;
    assert_eq!(status, StatusCode::OK);
    let _ = page_body;
    let (_, _, html) = call(&h, page("/geofences")).await;
    assert!(html.contains("Current fences") && html.contains("Previous fences"));
    assert!(html.contains("/assets/geofences.js"));
    let (status, _, js) = call(&h, page("/assets/geofences.js")).await;
    assert_eq!(status, StatusCode::OK);
    assert!(js.contains("krabshell:geofences"));

    // Over Regent's Park, clear of every pad. The world is paused, so it
    // queues and opens on the next step.
    let circle =
        json!({ "circle": { "lat": 51.5313, "lon": -0.1527, "radius_m": 150 }, "minutes": 30 });
    let (status, body, text) = call(&h, post("/api/v1/geofences", circle)).await;
    assert_eq!(status, StatusCode::OK, "{text}");
    assert_eq!(body["status"], "queued");
    let (status, _, _) = call(&h, post("/api/v1/run/step", json!({ "n": 1 }))).await;
    assert_eq!(status, StatusCode::OK);

    let (_, body, _) = call(&h, get("/api/v1/geofences")).await;
    let fences = body["fences"].as_array().unwrap();
    assert_eq!(fences.len(), 1, "{body}");
    assert_eq!(fences[0]["source"], "dashboard");
    assert_eq!(fences[0]["shape"], "circle");
    assert_eq!(fences[0]["breaches"], 0);
    assert!(fences[0]["closes_tick"].as_u64().is_some());
    assert_eq!(body["margin_m"], 30.0);
    let id = fences[0]["id"].as_u64().unwrap();

    // One over HAWK-1's pad is refused, with the reason. Paused, it is only
    // queued; the refusal comes when the world steps, and the list says so.
    let pad = json!({ "polygon": [[-0.1285, 51.5035], [-0.1275, 51.5035], [-0.1275, 51.5045], [-0.1285, 51.5045]] });
    let (status, body, _) = call(&h, post("/api/v1/geofences", pad.clone())).await;
    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["status"], "queued");
    let queued = body["id"].clone();
    call(&h, post("/api/v1/run/step", json!({ "n": 1 }))).await;
    let (_, body, _) = call(&h, get("/api/v1/geofences")).await;
    let refused = body["refused"]
        .as_array()
        .unwrap()
        .iter()
        .find(|r| r["id"] == queued)
        .expect("the refusal is listed");
    assert!(
        refused["detail"].as_str().unwrap().contains("pad"),
        "{refused}"
    );
    assert_eq!(body["fences"].as_array().unwrap().len(), 1);
    // Running, the refusal comes straight back.
    call(&h, post("/api/v1/run/start", json!({ "target_tps": 50 }))).await;
    let (status, body, _) = call(&h, post("/api/v1/geofences", pad)).await;
    assert_eq!(status, StatusCode::BAD_REQUEST, "{body}");
    assert!(body["detail"].as_str().unwrap().contains("pad"), "{body}");
    call(&h, post("/api/v1/run/pause", json!({}))).await;
    let (status, _, _) = call(&h, post("/api/v1/geofences", json!({ "square": 1 }))).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);

    // Kept open until closed instead of 30 minutes.
    let (status, _, text) = call(
        &h,
        post(
            &format!("/api/v1/geofences/{id}/extend"),
            json!({ "minutes": -1 }),
        ),
    )
    .await;
    assert_eq!(status, StatusCode::OK, "{text}");
    call(&h, post("/api/v1/run/step", json!({ "n": 1 }))).await;
    let (_, body, _) = call(&h, get("/api/v1/geofences")).await;
    assert!(body["fences"][0]["closes_tick"].is_null(), "{body}");
    let (status, _, _) = call(
        &h,
        post("/api/v1/geofences/999/extend", json!({ "minutes": 5 })),
    )
    .await;
    assert_eq!(status, StatusCode::NOT_FOUND);

    let (status, _, text) = call(&h, delete(&format!("/api/v1/geofences/{id}"))).await;
    assert_eq!(status, StatusCode::OK, "{text}");
    call(&h, post("/api/v1/run/step", json!({ "n": 1 }))).await;
    let (_, body, _) = call(&h, get("/api/v1/geofences")).await;
    assert_eq!(body["fences"], json!([]));
    assert_eq!(body["previous"][0]["id"], id);
    assert_eq!(body["previous"][0]["closed"], "closed");

    // Reopened as it was, in a minute, until closed: it waits, scheduled,
    // with a new number, and can be cancelled before it opens.
    let previous = body["previous"][0].clone();
    let again = json!({
        "circle": { "lat": previous["lat_e7"].as_f64().unwrap() * 1e-7, "lon": previous["lon_e7"].as_f64().unwrap() * 1e-7, "radius_m": previous["radius_m"] },
        "after_minutes": 1, "minutes": -1,
    });
    let (status, _, text) = call(&h, post("/api/v1/geofences", again)).await;
    assert_eq!(status, StatusCode::OK, "{text}");
    call(&h, post("/api/v1/run/step", json!({ "n": 1 }))).await;
    let (_, body, _) = call(&h, get("/api/v1/geofences")).await;
    assert_eq!(body["fences"], json!([]));
    let waiting = &body["scheduled"][0];
    assert!(waiting["id"].as_u64().unwrap() > id);
    assert!(waiting["closes_tick"].is_null());
    let waiting = waiting["id"].as_u64().unwrap();
    let (status, _, _) = call(&h, delete(&format!("/api/v1/geofences/{waiting}"))).await;
    assert_eq!(status, StatusCode::OK);
    call(&h, post("/api/v1/run/step", json!({ "n": 1 }))).await;
    let (_, body, _) = call(&h, get("/api/v1/geofences")).await;
    assert_eq!(body["scheduled"], json!([]));
    assert_eq!(body["previous"][0]["closed"], "cancelled");

    let (status, _, _) = call(&h, delete("/api/v1/geofences/999")).await;
    assert_eq!(status, StatusCode::NOT_FOUND);
}
