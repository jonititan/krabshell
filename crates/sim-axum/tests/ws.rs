//! The live protocol over a real socket (spec §15, "WS").
//!
//! Route tests use `oneshot`, which cannot upgrade a connection, so these bind
//! an ephemeral port and speak to the server the way the dashboard or a Bevy
//! client would.

use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use sim_axum::auth::Auth;
use sim_axum::{router, AppState};
use sim_core::config::Param;
use sim_core::{Config, GeoCell, Profile, ScenarioBlob, ScenarioManifest};
use sim_engine::{Control, Driver, DriverHandle, ModelRegistry, ScenarioStore};
use tokio_tungstenite::tungstenite::Message;

const RESOLUTION: i32 = 6;
const TOKEN: &str = "ws-token";

type Socket =
    tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

struct Server {
    addr: std::net::SocketAddr,
    handle: DriverHandle,
    _driver: Driver,
}

async fn serve(walkers: i64) -> Server {
    let manifest = ScenarioManifest {
        id: "ws-test".into(),
        name: "WS test".into(),
        description: String::new(),
        model: "walkers".into(),
        engine_rev: None,
        resolution: RESOLUTION,
        root: None,
        profile: Profile::Fast,
    };
    let mut config = Config::default();
    manifest.apply_to(&mut config);
    config.run.publish_hz = 100.0;
    config.model.insert("walkers".into(), Param::Int(walkers));
    config.model.insert("spread_k".into(), Param::Int(1));
    config
        .model
        .insert("origin_lon".into(), Param::Float(-0.1276));
    config
        .model
        .insert("origin_lat".into(), Param::Float(51.5072));
    let blob = ScenarioBlob {
        manifest,
        config,
        agents: Vec::new(),
        fields: Value::Null,
    };

    let driver = Driver::spawn(ModelRegistry::with_builtins(), blob).unwrap();
    let handle = driver.handle();
    let state = AppState::new(
        handle.clone(),
        ScenarioStore::new(std::env::temp_dir().join("krabshell-ws-none")),
        Auth::with_token(Some(TOKEN.to_owned())),
    );

    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router(state)).await.unwrap() });

    Server {
        addr,
        handle,
        _driver: driver,
    }
}

async fn connect(server: &Server) -> Socket {
    let url = format!("ws://{}/api/v1/ws?token={TOKEN}", server.addr);
    let (socket, _) = tokio_tungstenite::connect_async(url)
        .await
        .expect("websocket connects");
    socket
}

async fn send(socket: &mut Socket, msg: Value) {
    socket
        .send(Message::Text(msg.to_string().into()))
        .await
        .unwrap();
}

/// Read frames until one satisfies `predicate`, skipping heartbeats and the rest.
async fn expect(socket: &mut Socket, what: &str, predicate: impl Fn(&Value) -> bool) -> Value {
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        let frame = tokio::time::timeout(remaining, socket.next())
            .await
            .unwrap_or_else(|_| panic!("timed out waiting for {what}"))
            .expect("socket stayed open")
            .expect("frame is readable");
        if let Message::Text(text) = frame {
            let msg: Value = serde_json::from_str(&text).unwrap();
            if predicate(&msg) {
                return msg;
            }
        }
    }
}

fn london() -> GeoCell {
    GeoCell::from_lonlat(-0.1276, 51.5072, RESOLUTION).unwrap()
}

fn objects_in(frame: &Value, key: &str, cell: &str) -> usize {
    frame[key]
        .as_array()
        .into_iter()
        .flatten()
        .filter(|bag| bag["cell"] == cell)
        .map(|bag| bag["objects"].as_array().map_or(0, Vec::len))
        .sum()
}

#[tokio::test]
async fn a_connection_without_the_token_is_refused() {
    let server = serve(0).await;
    let url = format!("ws://{}/api/v1/ws", server.addr);
    assert!(tokio_tungstenite::connect_async(url).await.is_err());
}

#[tokio::test]
async fn the_first_frame_is_hello_with_the_world_header() {
    let server = serve(12).await;
    let mut socket = connect(&server).await;
    let hello = expect(&mut socket, "hello", |m| m["type"] != "tick").await;
    assert_eq!(hello["type"], "hello");
    assert_eq!(hello["protocol"], 1);
    assert_eq!(hello["meta"]["scenario_id"], "ws-test");
    assert_eq!(hello["meta"]["agent_count"], 12);
}

#[tokio::test]
async fn subscribing_then_spawning_delivers_the_new_objects_in_a_delta() {
    let server = serve(0).await;
    let mut socket = connect(&server).await;
    let cell = london().to_hex();

    send(&mut socket, json!({"type": "subscribe", "cells": [cell]})).await;
    let snapshot = expect(&mut socket, "the initial snapshot", |m| {
        m["type"] == "snapshot"
    })
    .await;
    assert_eq!(objects_in(&snapshot, "cells", &cell), 0);

    let intent_id = "7c9e6679-7425-40de-944b-e07fc1f90ae7";
    send(
        &mut socket,
        json!({
            "type": "intent",
            "intent": {
                "id": intent_id,
                "cell": cell,
                "actor": "ws-client",
                "verb": "spawn",
                "payload": { "count": 2, "kind": "probe" }
            }
        }),
    )
    .await;

    // The world is paused; step it from the side, as an operator would.
    tokio::time::sleep(Duration::from_millis(30)).await;
    let driver = server.handle.clone();
    tokio::task::spawn_blocking(move || driver.command(Control::Step { n: 1 }))
        .await
        .unwrap()
        .unwrap();

    let accepted = expect(&mut socket, "the acceptance", |m| m["type"] == "accepted").await;
    assert_eq!(accepted["intent_id"], intent_id);

    let delta = expect(&mut socket, "a delta with the spawn", |m| {
        m["type"] == "delta" && objects_in(m, "upsert", &cell) > 0
    })
    .await;
    assert_eq!(objects_in(&delta, "upsert", &cell), 2);
    assert_eq!(delta["upsert"][0]["objects"][0]["kind"], "probe");
}

#[tokio::test]
async fn an_intent_outside_the_world_is_rejected_on_the_same_connection() {
    let server = serve(0).await;
    let mut socket = connect(&server).await;
    let wrong = GeoCell::from_lonlat(-0.1276, 51.5072, RESOLUTION + 1)
        .unwrap()
        .to_hex();
    let intent_id = "7c9e6679-7425-40de-944b-e07fc1f90ae8";

    send(
        &mut socket,
        json!({
            "type": "intent",
            "intent": { "id": intent_id, "cell": wrong, "actor": "ws-client", "verb": "spawn" }
        }),
    )
    .await;

    let rejected = expect(&mut socket, "the rejection", |m| m["type"] == "rejected").await;
    assert_eq!(rejected["intent_id"], intent_id);
    assert_eq!(rejected["reason"], "out_of_bounds");
}

#[tokio::test]
async fn two_clients_with_disjoint_areas_only_see_their_own_cells() {
    let server = serve(0).await;
    let here = london();
    let there = here
        .grid_disk(6)
        .unwrap()
        .into_iter()
        .find(|c| !here.grid_disk(2).unwrap().contains(c))
        .unwrap();

    let mut a = connect(&server).await;
    let mut b = connect(&server).await;
    send(
        &mut a,
        json!({"type": "subscribe", "cells": [here.to_hex()]}),
    )
    .await;
    send(
        &mut b,
        json!({"type": "subscribe", "cells": [there.to_hex()]}),
    )
    .await;
    expect(&mut a, "a's snapshot", |m| m["type"] == "snapshot").await;
    expect(&mut b, "b's snapshot", |m| m["type"] == "snapshot").await;

    send(
        &mut a,
        json!({"type": "intent", "intent": {
            "id": "7c9e6679-7425-40de-944b-e07fc1f90ae9",
            "cell": here.to_hex(), "actor": "a", "verb": "spawn", "payload": {"count": 1}
        }}),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(30)).await;
    let driver = server.handle.clone();
    tokio::task::spawn_blocking(move || driver.command(Control::Step { n: 1 }))
        .await
        .unwrap()
        .unwrap();

    expect(&mut a, "a's delta", |m| m["type"] == "delta").await;

    // Give b several publish intervals; it must see heartbeats but no delta.
    let deadline = tokio::time::Instant::now() + Duration::from_millis(250);
    let mut heartbeats = 0;
    while let Ok(Some(Ok(frame))) = tokio::time::timeout_at(deadline, b.next()).await {
        if let Message::Text(text) = frame {
            let msg: Value = serde_json::from_str(&text).unwrap();
            assert_ne!(
                msg["type"], "delta",
                "b saw a change in a cell it never subscribed to: {msg}"
            );
            if msg["type"] == "tick" {
                heartbeats += 1;
            }
        }
    }
    assert!(heartbeats > 0, "b should still receive heartbeats");
}

#[tokio::test]
async fn set_aoi_expands_a_disk_and_snapshots_it() {
    let server = serve(30).await;
    let mut socket = connect(&server).await;
    send(
        &mut socket,
        json!({"type": "set_aoi", "center": london().to_hex(), "k": 1}),
    )
    .await;
    let snapshot = expect(&mut socket, "the aoi snapshot", |m| m["type"] == "snapshot").await;
    let total: usize = snapshot["cells"]
        .as_array()
        .unwrap()
        .iter()
        .map(|b| b["objects"].as_array().unwrap().len())
        .sum();
    assert_eq!(total, 30, "all walkers were seeded inside a k=1 disk");
}

#[tokio::test]
async fn ping_is_answered_and_garbage_is_an_error_not_a_disconnect() {
    let server = serve(0).await;
    let mut socket = connect(&server).await;

    send(&mut socket, json!({"type": "ping"})).await;
    expect(&mut socket, "pong", |m| m["type"] == "pong").await;

    socket.send(Message::Text("not json".into())).await.unwrap();
    let error = expect(&mut socket, "an error frame", |m| m["type"] == "error").await;
    assert_eq!(error["code"], "bad_request");

    send(&mut socket, json!({"type": "ping"})).await;
    expect(&mut socket, "a second pong", |m| m["type"] == "pong").await;
}

#[tokio::test]
async fn intents_waiting_on_a_paused_world_do_not_starve_the_controls() {
    // Each intent waits for a tick to be answered, and a paused world has
    // none. They must not each hold one of the blocking threads that every
    // control route needs (Tokio keeps 512).
    let server = serve(0).await;
    let mut socket = connect(&server).await;
    let cell = london().to_hex();
    let sent = 600;
    for i in 0..sent {
        send(
            &mut socket,
            json!({
                "type": "intent",
                "intent": {
                    "id": uuid::Uuid::new_v4(),
                    "cell": cell,
                    // Many actors, so the per-actor rate limit does not bite.
                    "actor": format!("flood-{i}"),
                    "verb": "spawn",
                    "payload": { "count": 1, "kind": "probe" }
                }
            }),
        )
        .await;
    }
    tokio::time::sleep(Duration::from_millis(100)).await;

    let driver = server.handle.clone();
    tokio::time::timeout(
        Duration::from_secs(5),
        tokio::task::spawn_blocking(move || driver.command(Control::Step { n: 1 })),
    )
    .await
    .expect("a control still gets a thread")
    .unwrap()
    .unwrap();

    // And every intent is answered once the tick runs.
    let mut accepted = 0;
    while accepted < sent {
        expect(&mut socket, "the acceptances", |m| m["type"] == "accepted").await;
        accepted += 1;
    }
}

#[tokio::test]
async fn an_intent_thrown_away_by_a_reset_is_answered_as_discarded() {
    let server = serve(0).await;
    let mut socket = connect(&server).await;
    let intent_id = "7c9e6679-7425-40de-944b-e07fc1f90ae9";
    send(
        &mut socket,
        json!({
            "type": "intent",
            "intent": { "id": intent_id, "cell": london().to_hex(), "actor": "ws-client",
                        "verb": "spawn", "payload": { "count": 1 } }
        }),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    let driver = server.handle.clone();
    tokio::task::spawn_blocking(move || driver.command(Control::Reset))
        .await
        .unwrap()
        .unwrap();
    let rejected = expect(&mut socket, "the discard", |m| m["type"] == "rejected").await;
    assert_eq!(rejected["intent_id"], intent_id);
    assert_eq!(rejected["reason"], "discarded");
}

#[tokio::test]
async fn subscribing_to_a_continent_is_refused_without_enumerating_it() {
    let server = serve(0).await;
    let mut socket = connect(&server).await;
    // Four of the twelve resolution-0 faces: 4 × 5 120 cells at the test
    // world's resolution 6, over the 20 000 cap. (At the packs' resolution 15
    // one face alone is about 1.3 billion.)
    let faces: std::collections::BTreeSet<String> = [
        (-0.13, 51.5),
        (151.2, -33.9),
        (-74.0, 40.7),
        (37.6, 55.8),
        (116.4, 39.9),
        (-46.6, -23.5),
    ]
    .iter()
    .map(|&(lon, lat)| GeoCell::from_lonlat(lon, lat, 0).unwrap().to_hex())
    .collect();
    assert!(faces.len() >= 4, "{faces:?}");
    send(
        &mut socket,
        json!({"type": "subscribe", "cells": faces.iter().take(4).collect::<Vec<_>>(), "res": 0}),
    )
    .await;
    let error = expect(&mut socket, "the refusal", |m| m["type"] == "error").await;
    assert_eq!(error["code"], "bad_request");
}

#[tokio::test]
async fn an_intent_over_the_rest_size_cap_is_refused_here_too() {
    let server = serve(0).await;
    let mut socket = connect(&server).await;
    send(
        &mut socket,
        json!({
            "type": "intent",
            "intent": { "id": "7c9e6679-7425-40de-944b-e07fc1f90aea", "cell": london().to_hex(),
                        "actor": "ws-client", "verb": "spawn",
                        "payload": { "blob": "x".repeat(20_000) } }
        }),
    )
    .await;
    let error = expect(&mut socket, "the refusal", |m| m["type"] == "error").await;
    assert_eq!(error["code"], "payload_too_large");
}
