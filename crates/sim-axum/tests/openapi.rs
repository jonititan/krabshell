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

fn harness(pack: &str) -> Harness {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(pack);
    let blob = sim_engine::read_pack(&path).unwrap();
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
    );
    Harness {
        router: router(state),
        _driver: driver,
        scenarios: dir,
    }
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
    for uri in ["/api/v1/openapi.json", "/docs"] {
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

    let (_, _, page) = call(&h, "get", "/docs", None).await;
    let page = String::from_utf8(page).unwrap();
    for (_, path) in operations(&DOC) {
        assert!(
            page.contains(&format!("/api/v1{path}")),
            "/docs lacks {path}"
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
