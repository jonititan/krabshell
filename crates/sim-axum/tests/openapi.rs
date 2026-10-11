//! The OpenAPI document against the server it describes.
//!
//! The document is written by hand, so these are what keep it true: every
//! route the router declares is described, every description is routed, and
//! what the routes actually answer matches the schemas, for every scenario
//! pack. A new field in a response fails here until the document has it.

use axum::body::Body;
use axum::http::{header, Request, StatusCode};
use http_body_util::BodyExt;
use serde_json::{json, Value};
use sim_axum::auth::Auth;
use sim_axum::openapi::{operations, resolve, DOC};
use std::sync::Arc;

use sim_axum::weather::WeatherService;
use sim_axum::{router, AppState};
use sim_engine::{Driver, ModelRegistry, ScenarioStore};
use tower::ServiceExt;

const TOKEN: &str = "test-token";
const PACKS: [&str; 3] = ["london-patrol", "kaunas-deliveries", "swarm-survey"];

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

fn pack(name: &str) -> sim_core::ScenarioBlob {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(name);
    sim_engine::read_pack(&path).unwrap()
}

fn harness(name: &str) -> Harness {
    harness_for(pack(name), Arc::new(WeatherService::offline()))
}

fn harness_for(blob: sim_core::ScenarioBlob, weather: Arc<WeatherService>) -> Harness {
    let dir = std::env::temp_dir().join(format!(
        "krabshell-openapi-{}-{}",
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
    )
    .with_weather(weather);
    Harness {
        router: router(state),
        _driver: driver,
        scenarios: dir,
    }
}

/// A pack with its own wind: from the south-west, stronger with height,
/// over every weather cell its agents' area needs.
async fn weather_harness(name: &str) -> Harness {
    let mut blob = pack(name);
    // This field stands in for whatever the pack asks to fetch.
    blob.config.model.retain(|k, _| !k.starts_with("weather"));
    let (mut w, mut s, mut e, mut n) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for a in &blob.agents {
        let (lon, lat) = match (a.lon, a.lat, a.cell) {
            (Some(lon), Some(lat), _) => (lon, lat),
            (_, _, Some(cell)) => cell.centroid().unwrap(),
            _ => continue,
        };
        (w, s, e, n) = (w.min(lon), s.min(lat), e.max(lon), n.max(lat));
    }
    let spec = sim_weather::WeatherSpec {
        mode: sim_core::WeatherMode::Recorded,
        start_unix: 1_773_468_000,
        hours: 3,
        resolution: 11,
        world_resolution: blob.manifest.resolution,
        bbox: [w - 0.03, s - 0.03, e + 0.03, n + 0.03],
        model: "best_match".into(),
        refresh_min: 60,
    };
    let cells = sim_weather::Fetcher::cells(&spec).unwrap();
    let levels = [10u16, 80, 120];
    let frames = 4;
    let field = json!({
        "resolution": 11, "mode": "recorded", "start_unix": spec.start_unix,
        "t0_unix": spec.start_unix, "step_s": 3600, "frames": frames, "levels_m": levels,
        "cells": cells.iter().map(|c| json!({
            "cell": c,
            "u_cms": (0..frames).flat_map(|_| [300, 400, 450]).collect::<Vec<i16>>(),
            "v_cms": (0..frames).flat_map(|_| [300, 400, 450]).collect::<Vec<i16>>(),
            "gust_cms": vec![800; frames],
        })).collect::<Vec<_>>(),
        "source": "test", "attribution": "Weather data by Open-Meteo.com (CC BY 4.0)", "fetched_unix": 0,
    });
    if !blob.fields.is_object() {
        blob.fields = json!({});
    }
    blob.fields["weather"] = field;
    let weather = Arc::new(WeatherService::offline());
    weather.prepare(&mut blob).await.unwrap();
    harness_for(blob, weather)
}

/// Status, content type and body.
async fn call(
    h: &Harness,
    method: &str,
    uri: &str,
    body: Option<Value>,
) -> (StatusCode, String, Vec<u8>) {
    let mut request = Request::builder()
        .method(method.to_uppercase().as_str())
        .uri(uri)
        .header(header::AUTHORIZATION, format!("Bearer {TOKEN}"));
    let body = match body {
        Some(json) => {
            request = request.header(header::CONTENT_TYPE, "application/json");
            Body::from(json.to_string())
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
    let content_type = response
        .headers()
        .get(header::CONTENT_TYPE)
        .and_then(|v| v.to_str().ok())
        .unwrap_or_default()
        .to_owned();
    let bytes = response
        .into_body()
        .collect()
        .await
        .unwrap()
        .to_bytes()
        .to_vec();
    (status, content_type, bytes)
}

/// The `(method, path)` pairs `router()` declares, read from its source.
///
/// Axum cannot list its routes, so this reads `lib.rs`: each `.route("path",`
/// and the method routers after it, up to the next `.route`.
fn declared_routes() -> Vec<(String, String)> {
    let source = include_str!("../src/lib.rs");
    let mut out = Vec::new();
    let mut rest = source;
    while let Some(at) = rest.find(".route(") {
        rest = &rest[at + ".route(".len()..];
        let Some(open) = rest.find('"') else { break };
        let after = &rest[open + 1..];
        let close = after.find('"').unwrap();
        let path = after[..close].to_owned();
        let span = &after[close..];
        let span = &span[..span.find(".route").unwrap_or(span.len())];
        for method in sim_axum::openapi::METHODS {
            let needle = format!("{method}(");
            let mut search = span;
            while let Some(i) = search.find(&needle) {
                let before = search[..i].chars().last();
                if !before.is_some_and(|c| c.is_alphanumeric() || c == '_') {
                    out.push((method.to_owned(), path.clone()));
                    break;
                }
                search = &search[i + needle.len()..];
            }
        }
    }
    out.sort();
    out
}

#[test]
fn every_route_is_described_and_nothing_else() {
    let declared = declared_routes();
    assert!(
        declared.len() > 20,
        "read too few routes from lib.rs: {declared:?}"
    );
    let mut described: Vec<(String, String)> = operations(&DOC)
        .into_iter()
        .map(|(m, p)| (m.to_owned(), p))
        .collect();
    described.sort();
    let missing: Vec<_> = declared.iter().filter(|r| !described.contains(r)).collect();
    let extra: Vec<_> = described.iter().filter(|r| !declared.contains(r)).collect();
    assert!(
        missing.is_empty(),
        "routes openapi.json does not describe: {missing:?}"
    );
    assert!(
        extra.is_empty(),
        "operations openapi.json describes that are not routed: {extra:?}"
    );
}

#[tokio::test]
async fn every_described_operation_reaches_a_handler() {
    let h = harness("london-patrol");
    for (method, path) in operations(&DOC) {
        let uri = format!(
            "/api/v1{}",
            path.replace("{id}", "999999").replace("{cell}", "0")
        );
        let body = (method != "get" && method != "delete").then(|| json!({}));
        let (status, _, bytes) = call(&h, method, &uri, body).await;
        // Axum's own misses: 405, or a 404 with no problem body.
        assert_ne!(status, StatusCode::METHOD_NOT_ALLOWED, "{method} {uri}");
        assert!(
            !(status == StatusCode::NOT_FOUND && bytes.is_empty()),
            "{method} {uri} is not routed"
        );
    }
}

#[tokio::test]
async fn the_document_and_the_page_need_no_token() {
    let h = harness("london-patrol");
    for uri in [
        "/api/v1/openapi.json",
        "/docs",
        "/docs/api",
        "/docs/operating",
    ] {
        let response = h
            .router
            .clone()
            .oneshot(Request::get(uri).body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK, "{uri}");
    }
    let (_, content_type, bytes) = call(&h, "get", "/api/v1/openapi.json", None).await;
    assert_eq!(content_type, "application/json");
    let served: Value = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(served["openapi"], "3.1.0");

    let (_, _, page) = call(&h, "get", "/docs/api", None).await;
    let page = String::from_utf8(page).unwrap();
    for (_, path) in operations(&DOC) {
        assert!(
            page.contains(&format!("/api/v1{path}")),
            "/docs/api lacks {path}"
        );
    }
}

/// What each pack answers matches the documented schema, for every GET
/// without path parameters, after the world has moved a little.
#[tokio::test]
async fn responses_match_their_schemas() {
    for pack in PACKS {
        let h = harness(pack);
        // Step so there are reports, flights and stations in use.
        let (status, _, bytes) =
            call(&h, "post", "/api/v1/run/step", Some(json!({"n": 200}))).await;
        assert_eq!(status, StatusCode::OK);
        check(pack, "post", "/run/step", "200", &bytes);

        for (method, path) in operations(&DOC) {
            let op = &DOC["paths"][&path][method];
            let needs_parameters = op["parameters"]
                .as_array()
                .into_iter()
                .flatten()
                .any(|p| resolve(&DOC, p)["required"] == true);
            if method != "get" || needs_parameters || path == "/ws" {
                continue;
            }
            let (status, content_type, bytes) =
                call(&h, method, &format!("/api/v1{path}"), None).await;
            let code = status.as_u16().to_string();
            let documented = &DOC["paths"][&path][method]["responses"][&code];
            assert!(
                !documented.is_null(),
                "{pack}: GET {path} answered {code}, which is not documented"
            );
            let documented = resolve(&DOC, documented);
            let media = documented["content"]
                .as_object()
                .and_then(|c| c.keys().next().cloned())
                .unwrap_or_default();
            assert!(
                content_type.starts_with(&media),
                "{pack}: GET {path} answered {content_type}, documented as {media}"
            );
            if content_type.contains("json") && !content_type.contains("jsonl") {
                check(pack, method, &path, &code, &bytes);
            }
        }
    }
}

/// A cell inside the loaded world, for the per-cell routes.
#[tokio::test]
async fn a_cell_and_a_lookup_match_their_schemas() {
    let h = harness("london-patrol");
    let (_, _, bytes) = call(
        &h,
        "get",
        "/api/v1/cells/lookup?lon=-0.1276&lat=51.5072",
        None,
    )
    .await;
    check("london-patrol", "get", "/cells/lookup", "200", &bytes);
    let cell = serde_json::from_slice::<Value>(&bytes).unwrap()["cell"]
        .as_str()
        .unwrap()
        .to_owned();
    let (status, _, bytes) = call(&h, "get", &format!("/api/v1/state/cells/{cell}"), None).await;
    assert_eq!(status, StatusCode::OK);
    check("london-patrol", "get", "/state/cells/{cell}", "200", &bytes);
}

fn check(pack: &str, method: &str, path: &str, code: &str, bytes: &[u8]) {
    let body: Value = serde_json::from_slice(bytes).unwrap();
    let response = resolve(&DOC, &DOC["paths"][path][method]["responses"][code]);
    let media = response["content"]
        .as_object()
        .unwrap()
        .values()
        .next()
        .unwrap();
    let mut errors = Vec::new();
    validate(&media["schema"], &body, "$", &mut errors);
    assert!(
        errors.is_empty(),
        "{pack}: {method} {path} {code} does not match openapi.json:\n  {}",
        errors.join("\n  ")
    );
}

/// With wind loaded, the weather routes and the entities match their
/// schemas, and a drone's cell finds its wind.
#[tokio::test]
async fn weather_responses_match_their_schemas() {
    let h = weather_harness("kaunas-deliveries").await;
    let (status, _, bytes) = call(&h, "post", "/api/v1/run/step", Some(json!({"n": 300}))).await;
    assert_eq!(status, StatusCode::OK);

    let (status, _, bytes_status) = call(&h, "get", "/api/v1/weather", None).await;
    assert_eq!(status, StatusCode::OK);
    check("kaunas-deliveries", "get", "/weather", "200", &bytes_status);
    let body: Value = serde_json::from_slice(&bytes_status).unwrap();
    assert_eq!(body["available"], true);
    assert_eq!(body["origin"], "pack");
    assert_eq!(body["in_sync"], true, "{body}");
    assert!(
        body["sim_utc"]
            .as_str()
            .unwrap()
            .starts_with("2026-03-14T06:00:30"),
        "{body}"
    );

    let (status, _, cells) = call(&h, "get", "/api/v1/weather/cells?alt_m=80", None).await;
    assert_eq!(status, StatusCode::OK);
    check("kaunas-deliveries", "get", "/weather/cells", "200", &cells);
    let cells: Value = serde_json::from_slice(&cells).unwrap();
    // 4 m/s each way at 80 m: from the south-west at 5.66 m/s.
    assert_eq!(cells["cells"][0]["speed_ms"], 5.66);
    assert_eq!(cells["cells"][0]["from_deg"], 225.0);

    let (_, _, entities) = call(&h, "get", "/api/v1/state/entities?kind=drone", None).await;
    check(
        "kaunas-deliveries",
        "get",
        "/state/entities",
        "200",
        &entities,
    );
    let entities: Value = serde_json::from_slice(&entities).unwrap();
    let cell = entities[0]["cell"].as_str().unwrap().to_owned();
    let (status, _, detail) = call(
        &h,
        "get",
        &format!("/api/v1/weather/cells/{cell}?alt_m=10"),
        None,
    )
    .await;
    assert_eq!(
        status,
        StatusCode::OK,
        "{}",
        String::from_utf8_lossy(&detail)
    );
    check(
        "kaunas-deliveries",
        "get",
        "/weather/cells/{cell}",
        "200",
        &detail,
    );
    let detail: Value = serde_json::from_slice(&detail).unwrap();
    assert_eq!(detail["series"].as_array().unwrap().len(), 4);
    assert_eq!(detail["levels"].as_array().unwrap().len(), 3);
    assert_eq!(detail["wind"]["east_ms"], 3.0);
    let _ = bytes;
}

#[tokio::test]
async fn without_weather_the_cell_routes_say_so() {
    let h = harness("london-patrol");
    let (status, _, bytes) = call(&h, "get", "/api/v1/weather/cells", None).await;
    assert_eq!(status, StatusCode::NOT_FOUND);
    let body: Value = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(body["code"], "not_found");
    let (status, _, bytes) = call(&h, "post", "/api/v1/weather/refresh", None).await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    check("london-patrol", "post", "/weather/refresh", "400", &bytes);
}

/// london-patrol with two more drones flying at each other at 40 m: a
/// conflict to resolve. The world's own advisories are off, so the pair
/// carries on into it rather than getting out of the way first.
fn head_on_harness() -> Harness {
    let mut blob = pack("london-patrol");
    blob.config
        .model
        .insert("atc".into(), sim_core::config::Param::Bool(false));
    let (lat, lon0, lon1) = (51.51, -0.11, -0.0927);
    let mission = |to_lon: f64| {
        json!([
            { "seq": 0, "frame": 3, "command": 22, "z": 40.0 },
            { "seq": 1, "frame": 3, "command": 16, "x": (lat * 1e7) as i64, "y": (to_lon * 1e7) as i64, "z": 40.0 },
        ])
    };
    for (id, lon, to, call) in [(901u64, lon0, lon1, "WEST-1"), (902, lon1, lon0, "EAST-1")] {
        blob.agents.push(sim_core::SeedAgent {
            id: Some(id),
            kind: "drone".into(),
            cell: None,
            lon: Some(lon),
            lat: Some(lat),
            state: json!({ "sysid": id - 700, "callsign": call, "autostart": true, "mission": mission(to) }),
        });
    }
    harness_for(blob, Arc::new(WeatherService::offline()))
}

/// The conflict report finds the head-on pair, asks one of them to
/// manoeuvre, and its cells and advisories match their schemas.
#[tokio::test]
async fn a_head_on_pair_is_reported_and_advised() {
    let h = head_on_harness();
    // Up to 40 m, then about 30 s into the 1.2 km: 600 m apart, closing.
    let (status, _, _) = call(&h, "post", "/api/v1/run/step", Some(json!({"n": 450}))).await;
    assert_eq!(status, StatusCode::OK);
    let (status, _, bytes) = call(&h, "get", "/api/v1/conflicts", None).await;
    assert_eq!(status, StatusCode::OK);
    check("london-patrol", "get", "/conflicts", "200", &bytes);
    let r: Value = serde_json::from_slice(&bytes).unwrap();
    assert!(r["aircraft"].as_u64().unwrap() >= 2, "{}", r["aircraft"]);
    let cells = r["cells"].as_array().unwrap();
    let pair = |c: &Value| {
        let who: Vec<&str> = c["aircraft"]
            .as_array()
            .unwrap()
            .iter()
            .filter_map(|a| a["callsign"].as_str())
            .collect();
        who.contains(&"WEST-1") && who.contains(&"EAST-1")
    };
    let shared: Vec<&Value> = cells.iter().filter(|c| pair(c)).collect();
    assert!(!shared.is_empty(), "no cell has both");
    assert!(shared.iter().any(|c| c["conflict"] == true));
    // Worst first.
    let summed: Vec<f64> = cells
        .iter()
        .map(|c| c["summed"].as_f64().unwrap())
        .collect();
    assert!(summed.windows(2).all(|w| w[0] >= w[1]));
    let advised: Vec<&str> = r["advisories"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|a| a["callsign"].as_str())
        .collect();
    assert!(
        advised.contains(&"WEST-1") || advised.contains(&"EAST-1"),
        "{}",
        r["advisories"]
    );
    assert!(r["risk_after"].as_f64().unwrap() < r["risk_before"].as_f64().unwrap());

    // The heatmap has every cell either could be in, not only the shared
    // ones: some only one of them reaches.
    let heat = r["heat"].as_array().unwrap();
    assert!(
        heat.len() > cells.len(),
        "{} heat, {} shared",
        heat.len(),
        cells.len()
    );
    assert!(heat
        .iter()
        .any(|c| c["collision"] == 0.0 && c["summed"].as_f64().unwrap() > 0.0));

    // The same tick again is the same report, not worked out afresh.
    let (_, _, again) = call(&h, "get", "/api/v1/conflicts?outlines=false", None).await;
    let again: Value = serde_json::from_slice(&again).unwrap();
    assert_eq!(again["took_ms"], r["took_ms"]);
    assert!(again["cells"][0]["boundary"].as_array().unwrap().is_empty());
}

/// The conflict cells' resolution can be chosen, refused out of range, and
/// handed back to the world's; a new choice is a new report at once.
#[tokio::test]
async fn the_conflict_resolution_can_be_chosen() {
    let h = head_on_harness();
    call(&h, "post", "/api/v1/run/step", Some(json!({"n": 450}))).await;
    let (status, _, bytes) = call(&h, "get", "/api/v1/conflicts/settings", None).await;
    assert_eq!(status, StatusCode::OK);
    check("london-patrol", "get", "/conflicts/settings", "200", &bytes);
    let s: Value = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(
        (s["resolution"].as_i64(), s["from"].as_str()),
        (Some(15), Some("world"))
    );

    let (status, _, bytes) = call(
        &h,
        "put",
        "/api/v1/conflicts/settings",
        Some(json!({"resolution": 13})),
    )
    .await;
    assert_eq!(status, StatusCode::OK);
    check("london-patrol", "put", "/conflicts/settings", "200", &bytes);
    let (_, _, bytes) = call(&h, "get", "/api/v1/conflicts?outlines=false", None).await;
    let r: Value = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(
        (r["resolution"].as_i64(), r["resolution_from"].as_str()),
        (Some(13), Some("operator"))
    );
    let cell = sim_core::GeoCell::parse(r["heat"][0]["cell"].as_str().unwrap()).unwrap();
    assert_eq!(cell.resolution(), 13);

    let (status, _, bytes) = call(
        &h,
        "put",
        "/api/v1/conflicts/settings",
        Some(json!({"resolution": 30})),
    )
    .await;
    assert_eq!(status, StatusCode::BAD_REQUEST);
    check("london-patrol", "put", "/conflicts/settings", "400", &bytes);

    call(
        &h,
        "put",
        "/api/v1/conflicts/settings",
        Some(json!({"resolution": null})),
    )
    .await;
    let (_, _, bytes) = call(&h, "get", "/api/v1/conflicts?outlines=false", None).await;
    let r: Value = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(r["resolution"].as_i64(), Some(15));
}

// ------------------------------------------------- a small schema checker

/// Check a value against the subset of JSON Schema the document uses.
///
/// Stricter than JSON Schema in one way, on purpose: an object with
/// `properties` may hold no others unless `additionalProperties` says so, so
/// an undocumented field is an error rather than silently allowed.
fn validate(schema: &Value, value: &Value, at: &str, errors: &mut Vec<String>) {
    let schema = resolve(&DOC, schema);

    if let Some(branches) = schema["oneOf"].as_array().or(schema["anyOf"].as_array()) {
        let fits = branches.iter().any(|b| {
            let mut e = Vec::new();
            validate(b, value, at, &mut e);
            e.is_empty()
        });
        if !fits {
            errors.push(format!("{at}: {value} fits none of the alternatives"));
        }
        return;
    }

    if let Some(parts) = schema["allOf"].as_array() {
        for part in parts {
            validate_shallow(part, value, at, errors);
        }
        check_unknown(schema, value, at, errors);
        return;
    }

    validate_shallow(schema, value, at, errors);
    check_unknown(schema, value, at, errors);
}

/// Everything but the unknown-field check, which an `allOf` does once for
/// all its parts.
fn validate_shallow(schema: &Value, value: &Value, at: &str, errors: &mut Vec<String>) {
    let schema = resolve(&DOC, schema);
    if schema.get("allOf").is_some() || schema.get("oneOf").is_some() {
        let mut e = Vec::new();
        validate(schema, value, at, &mut e);
        // Unknown fields are judged by the outer schema.
        errors.extend(e.into_iter().filter(|e| !e.contains("undocumented field")));
        return;
    }

    if let Some(c) = schema.get("const") {
        if c != value {
            errors.push(format!("{at}: {value} is not {c}"));
        }
    }
    if let Some(options) = schema["enum"].as_array() {
        if !options.contains(value) {
            errors.push(format!("{at}: {value} is not one of {options:?}"));
        }
    }

    let types: Vec<&str> = match &schema["type"] {
        Value::String(t) => vec![t.as_str()],
        Value::Array(ts) => ts.iter().filter_map(Value::as_str).collect(),
        _ => Vec::new(),
    };
    if !types.is_empty() && !types.iter().any(|t| is_type(t, value)) {
        errors.push(format!("{at}: {value} is not {}", types.join(" or ")));
        return;
    }

    match value {
        Value::Object(map) => {
            for name in schema["required"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(Value::as_str)
            {
                if !map.contains_key(name) {
                    errors.push(format!("{at}: missing required field {name}"));
                }
            }
            for (name, child) in map {
                if let Some(property) = schema["properties"].get(name) {
                    validate(property, child, &format!("{at}.{name}"), errors);
                } else if let Some(extra @ Value::Object(_)) = schema.get("additionalProperties") {
                    validate(extra, child, &format!("{at}.{name}"), errors);
                }
            }
        }
        Value::Array(items) => {
            if let Some(prefix) = schema["prefixItems"].as_array() {
                for (i, (s, v)) in prefix.iter().zip(items).enumerate() {
                    validate(s, v, &format!("{at}[{i}]"), errors);
                }
            } else if let Some(item) = schema.get("items") {
                // Enough to catch a shape mistake without walking a city.
                for (i, v) in items.iter().take(200).enumerate() {
                    validate(item, v, &format!("{at}[{i}]"), errors);
                }
            }
        }
        _ => {}
    }
}

/// Fields the schema (all of an `allOf`'s parts together) does not name.
fn check_unknown(schema: &Value, value: &Value, at: &str, errors: &mut Vec<String>) {
    let Value::Object(map) = value else { return };
    let mut known = Vec::new();
    let mut open = false;
    collect_known(schema, &mut known, &mut open);
    if open || known.is_empty() {
        return;
    }
    for name in map.keys() {
        if !known.contains(name) {
            errors.push(format!("{at}: undocumented field {name}"));
        }
    }
}

fn collect_known(schema: &Value, known: &mut Vec<String>, open: &mut bool) {
    let schema = resolve(&DOC, schema);
    for part in schema["allOf"].as_array().into_iter().flatten() {
        collect_known(part, known, open);
    }
    if matches!(
        schema.get("additionalProperties"),
        Some(Value::Bool(true) | Value::Object(_))
    ) {
        *open = true;
    }
    if let Some(properties) = schema["properties"].as_object() {
        known.extend(properties.keys().cloned());
    }
}

fn is_type(t: &str, value: &Value) -> bool {
    match t {
        "null" => value.is_null(),
        "boolean" => value.is_boolean(),
        "integer" => value.is_i64() || value.is_u64(),
        "number" => value.is_number(),
        "string" => value.is_string(),
        "array" => value.is_array(),
        "object" => value.is_object(),
        _ => true,
    }
}
