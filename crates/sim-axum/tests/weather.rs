//! Loading scenarios that ask for weather, against a stand-in for Open-Meteo:
//! recorded weather is fetched once and then served from the cache, a fetch
//! that fails refuses a recorded load, and live weather refreshes into the
//! running world.

use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use axum::body::Body;
use axum::extract::{Query, State};
use axum::http::{header, Request, StatusCode};
use axum::routing::get;
use axum::{Json, Router};
use http_body_util::BodyExt;
use serde_json::{json, Value};
use sim_axum::auth::Auth;
use sim_axum::weather::WeatherService;
use sim_axum::{router, AppState};
use sim_engine::{Driver, ModelRegistry, ScenarioStore};
use sim_weather::fetch::Hosts;
use sim_weather::{Cache, Fetcher};
use tower::ServiceExt;

const TOKEN: &str = "test-token";

/// Locations asked for, across every request.
#[derive(Default)]
struct Seen(AtomicUsize);

/// 6 m/s from the north at every height, gusting 9, for whatever hours and
/// places are asked.
async fn forecast(
    State(seen): State<Arc<Seen>>,
    Query(q): Query<HashMap<String, String>>,
) -> Json<Value> {
    let lats: Vec<&str> = q["latitude"].split(',').collect();
    seen.0.fetch_add(lats.len(), Ordering::SeqCst);
    let hour = |key: &str| sim_weather::parse_utc(&format!("{}:00Z", q[key])).unwrap();
    let times: Vec<i64> = (hour("start_hour")..=hour("end_hour"))
        .step_by(3600)
        .collect();
    let n = times.len();
    let one = json!({
        "latitude": 0.0, "longitude": 0.0, "utc_offset_seconds": 0, "timezone": "GMT",
        "hourly": {
            "time": times,
            "wind_speed_10m": vec![6.0; n], "wind_direction_10m": vec![0.0; n],
            "wind_speed_80m": vec![6.0; n], "wind_direction_80m": vec![0.0; n],
            "wind_speed_120m": vec![6.0; n], "wind_direction_120m": vec![0.0; n],
            "wind_speed_180m": vec![6.0; n], "wind_direction_180m": vec![0.0; n],
            "wind_gusts_10m": vec![9.0; n]
        }
    });
    Json(Value::Array(vec![one; lats.len()]))
}

async fn stand_in() -> (String, Arc<Seen>) {
    let seen = Arc::new(Seen::default());
    let app = Router::new()
        .route("/v1/forecast", get(forecast))
        .with_state(Arc::clone(&seen));
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, app).await.unwrap() });
    (format!("http://{addr}/"), seen)
}

fn scratch(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "krabshell-weather-api-{name}-{}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn copy_dir(from: &std::path::Path, to: &std::path::Path) {
    std::fs::create_dir_all(to).unwrap();
    for entry in std::fs::read_dir(from).unwrap() {
        let entry = entry.unwrap();
        let target = to.join(entry.file_name());
        if entry.file_type().unwrap().is_dir() {
            copy_dir(&entry.path(), &target);
        } else {
            std::fs::copy(entry.path(), target).unwrap();
        }
    }
}

/// A scenario directory holding london-patrol as `id`, with weather params
/// appended to its `params.toml`.
fn store_with(dir: &std::path::Path, id: &str, params: &str) {
    let src =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/london-patrol");
    let pack = dir.join(id);
    copy_dir(&src, &pack);
    let manifest = std::fs::read_to_string(pack.join("manifest.toml")).unwrap();
    let manifest = manifest.replacen("id = \"london-patrol\"", &format!("id = \"{id}\""), 1);
    std::fs::write(pack.join("manifest.toml"), manifest).unwrap();
    let mut p = std::fs::read_to_string(pack.join("params.toml")).unwrap_or_default();
    p.push('\n');
    p.push_str(params);
    std::fs::write(pack.join("params.toml"), p).unwrap();
}

struct Harness {
    router: axum::Router,
    _driver: Driver,
}

fn harness(store: &std::path::Path, weather: Arc<WeatherService>) -> Harness {
    let driver = Driver::spawn(
        ModelRegistry::with_builtins(),
        sim_core::ScenarioBlob::empty(),
    )
    .unwrap();
    let state = AppState::new(
        driver.handle(),
        ScenarioStore::new(store),
        Auth::with_token(Some(TOKEN.to_owned())),
    )
    .with_weather(weather);
    Harness {
        router: router(state),
        _driver: driver,
    }
}

fn service(host: &str, cache: &std::path::Path) -> Arc<WeatherService> {
    let hosts = Hosts {
        forecast: Some(host.to_owned()),
        historical: Some(host.to_owned()),
        ..Hosts::default()
    };
    Arc::new(WeatherService::new(Some(
        Fetcher::new(Cache::new(cache), hosts).unwrap(),
    )))
}

async fn call(h: &Harness, method: &str, uri: &str, body: Option<Value>) -> (StatusCode, Value) {
    let mut request = Request::builder()
        .method(method)
        .uri(uri)
        .header(header::AUTHORIZATION, format!("Bearer {TOKEN}"));
    let body = match body {
        Some(b) => {
            request = request.header(header::CONTENT_TYPE, "application/json");
            Body::from(b.to_string())
        }
        None => Body::empty(),
    };
    let response = h
        .router
        .clone()
        .oneshot(request.body(body).unwrap())
        .await
        .unwrap();
    let status = response.status();
    let bytes = response.into_body().collect().await.unwrap().to_bytes();
    (
        status,
        serde_json::from_slice(&bytes).unwrap_or(Value::Null),
    )
}

const RECORDED: &str =
    "weather = \"recorded\"\nweather_start_utc = \"2025-11-02T09:15:00Z\"\nweather_hours = 2\n";

#[tokio::test]
async fn recorded_weather_is_fetched_once_and_then_served_from_the_cache() {
    let (host, seen) = stand_in().await;
    let store = scratch("store");
    let cache = scratch("cache");
    store_with(&store, "windy", RECORDED);
    let h = harness(&store, service(&host, &cache));

    let (status, body) = call(&h, "POST", "/api/v1/scenarios/windy/load", None).await;
    assert_eq!(status, StatusCode::OK, "{body}");
    let (_, w) = call(&h, "GET", "/api/v1/weather", None).await;
    assert_eq!(w["origin"], "open_meteo", "{w}");
    assert_eq!(w["mode"], "recorded");
    assert_eq!(w["cache_hit"], false);
    assert_eq!(w["start_utc"], "2025-11-02T09:15:00Z");
    assert_eq!(w["sim_utc"], "2025-11-02T09:15:00Z");
    assert_eq!(w["in_sync"], true, "{w}");
    let asked = seen.0.load(Ordering::SeqCst);
    assert!(asked > 0 && w["locations"] == asked, "{w}");

    // Run it again: no call.
    let (status, _) = call(&h, "POST", "/api/v1/scenarios/windy/load", None).await;
    assert_eq!(status, StatusCode::OK);
    let (_, w) = call(&h, "GET", "/api/v1/weather", None).await;
    assert_eq!(w["cache_hit"], true);
    assert_eq!(w["locations"], 0);
    assert_eq!(seen.0.load(Ordering::SeqCst), asked);
    assert_eq!(w["cache"]["entries"], 1);

    // The drones fly in it: a drone's cell has the north wind.
    call(&h, "POST", "/api/v1/run/step", Some(json!({"n": 50}))).await;
    let (_, drones) = call(&h, "GET", "/api/v1/state/entities?kind=drone", None).await;
    let cell = drones[0]["cell"].as_str().unwrap();
    let (status, detail) = call(
        &h,
        "GET",
        &format!("/api/v1/weather/cells/{cell}?alt_m=80"),
        None,
    )
    .await;
    assert_eq!(status, StatusCode::OK, "{detail}");
    assert_eq!(detail["wind"]["north_ms"], -6.0);
    assert_eq!(detail["wind"]["from_deg"], 0.0);
}

#[tokio::test]
async fn a_recorded_load_whose_weather_cannot_be_fetched_is_refused() {
    let store = scratch("refused");
    let cache = scratch("refused-cache");
    store_with(&store, "unreachable", RECORDED);
    // Nothing listens on the discard port.
    let h = harness(&store, service("http://127.0.0.1:9/", &cache));
    let (status, body) = call(&h, "POST", "/api/v1/scenarios/unreachable/load", None).await;
    assert_eq!(status, StatusCode::BAD_GATEWAY, "{body}");
    assert_eq!(body["code"], "weather_unavailable");
    // And a pack asking for weather nonsense is a config error.
    store_with(&store, "nonsense", "weather = \"sometimes\"\n");
    let (status, body) = call(&h, "POST", "/api/v1/scenarios/nonsense/load", None).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["code"], "bad_config");
}

#[tokio::test]
async fn live_weather_starts_now_and_a_refresh_reaches_the_world() {
    let (host, _seen) = stand_in().await;
    let store = scratch("live");
    let cache = scratch("live-cache");
    store_with(&store, "live", "weather = \"live\"\nweather_hours = 3\n");
    let h = harness(&store, service(&host, &cache));
    let (status, body) = call(&h, "POST", "/api/v1/scenarios/live/load", None).await;
    assert_eq!(status, StatusCode::OK, "{body}");
    let (_, w) = call(&h, "GET", "/api/v1/weather", None).await;
    assert_eq!(w["mode"], "live");
    assert_eq!(w["refresh_min"], 60);
    let start = sim_weather::parse_utc(w["start_utc"].as_str().unwrap()).unwrap();
    assert!((sim_weather::now_unix() - start).abs() < 60, "{w}");

    let (status, body) = call(&h, "POST", "/api/v1/weather/refresh", None).await;
    assert_eq!(status, StatusCode::OK, "{body}");
    assert_eq!(
        body["start_utc"], w["start_utc"],
        "the run's clock stays put"
    );
    // The world takes it at the next tick.
    call(&h, "POST", "/api/v1/run/step", Some(json!({"n": 2}))).await;
    let (_, w) = call(&h, "GET", "/api/v1/weather", None).await;
    assert_eq!(w["in_sync"], true, "{w}");
}
