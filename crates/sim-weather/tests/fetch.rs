//! Fetching against a stand-in for Open-Meteo: what is asked for, how the
//! answer becomes a field, and that the cache spares a second call.

use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use axum::extract::{Query, State};
use axum::routing::get;
use axum::{Json, Router};
use serde_json::{json, Value};
use sim_core::WeatherMode;
use sim_weather::fetch::Hosts;
use sim_weather::{Cache, Fetcher, WeatherSpec};

/// 2026-03-14T06:00:00Z.
const START: i64 = 1_773_468_000;

#[derive(Default)]
struct Seen {
    requests: AtomicUsize,
    locations: AtomicUsize,
}

/// Wind from the west at 5 m/s at 10 m and 8 m/s at 80 and 120 m, nothing
/// at 180 m (as a model without that level answers), gusts 9 m/s; frame
/// three's 10 m speed missing.
async fn forecast(
    State(seen): State<Arc<Seen>>,
    Query(q): Query<HashMap<String, String>>,
) -> Json<Value> {
    seen.requests.fetch_add(1, Ordering::SeqCst);
    let lats: Vec<&str> = q["latitude"].split(',').collect();
    let lons: Vec<&str> = q["longitude"].split(',').collect();
    assert_eq!(q["wind_speed_unit"], "ms");
    assert_eq!(q["timeformat"], "unixtime");
    assert_eq!(q["start_hour"], "2026-03-14T06:00");
    assert_eq!(q["end_hour"], "2026-03-14T09:00");
    assert!(q["hourly"].contains("wind_speed_120m"));
    seen.locations.fetch_add(lats.len(), Ordering::SeqCst);
    let times: Vec<i64> = (0..4).map(|i| START + i * 3600).collect();
    let answers: Vec<Value> = lats
        .iter()
        .zip(&lons)
        .map(|(lat, lon)| {
            json!({
                "latitude": lat.parse::<f64>().unwrap(), "longitude": lon.parse::<f64>().unwrap(),
                "generationtime_ms": 0.1, "utc_offset_seconds": 0, "timezone": "GMT",
                "timezone_abbreviation": "GMT", "elevation": 40.0,
                "hourly_units": { "time": "unixtime", "wind_speed_10m": "m/s" },
                "hourly": {
                    "time": times,
                    "wind_speed_10m": [5.0, 5.0, null, 5.0],
                    "wind_direction_10m": [270, 270, 270, 270],
                    "wind_speed_80m": [8.0, 8.0, 8.0, 8.0],
                    "wind_direction_80m": [270, 270, 270, 270],
                    "wind_speed_120m": [8.0, 8.0, 8.0, 8.0],
                    "wind_direction_120m": [270, 270, 270, 270],
                    "wind_speed_180m": [null, null, null, null],
                    "wind_direction_180m": [null, null, null, null],
                    "wind_gusts_10m": [9.0, 9.0, 9.0, 9.0]
                }
            })
        })
        .collect();
    Json(Value::Array(answers))
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

fn spec() -> WeatherSpec {
    WeatherSpec {
        mode: WeatherMode::Recorded,
        // Half past six for two hours: frames at 06, 07, 08 and 09.
        start_unix: START + 1_800,
        hours: 2,
        resolution: 11,
        world_resolution: 15,
        bbox: [23.85, 54.88, 23.95, 54.94],
        model: "best_match".into(),
        refresh_min: 60,
    }
}

fn scratch(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("krabshell-weather-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

#[tokio::test]
async fn a_fetch_becomes_a_field_per_cell_and_a_second_is_served_from_the_cache() {
    let (host, seen) = stand_in().await;
    let dir = scratch("cache");
    let hosts = Hosts {
        forecast: Some(host.clone()),
        historical: Some(host),
        ..Hosts::default()
    };
    let fetcher = Fetcher::new(Cache::new(&dir), hosts.clone()).unwrap();
    // "Now" is a year on, so the window is history and cached for good.
    let now = START + 365 * 86_400;

    let first = fetcher.fetch(&spec(), now).await.unwrap();
    assert!(!first.cache_hit);
    let cells = Fetcher::cells(&spec()).unwrap();
    assert_eq!(first.locations, cells.len());
    assert_eq!(seen.locations.load(Ordering::SeqCst), cells.len());

    let f = &first.field;
    assert_eq!(f.frames, 4);
    assert_eq!(f.t0_unix, START);
    assert_eq!(f.start_unix, START + 1_800);
    // 180 m had nothing anywhere, so it is dropped.
    assert_eq!(f.levels_m, vec![10, 80, 120]);
    assert_eq!(f.cells.len(), cells.len());
    assert!(f.attribution.contains("Open-Meteo"));
    // From the west: toward the east.
    let c = &f.cells[0];
    assert_eq!((c.u_cms[0], c.v_cms[0]), (500, 0));
    assert_eq!(c.u_cms[1], 800);
    // Frame two's missing 10 m came from the nearest level, 80 m.
    assert_eq!(c.u_cms[2 * 3], 800);
    assert_eq!(c.gust_cms, vec![900, 900, 900, 900]);
    // Sampled through a fine cell inside it.
    let (lon, lat) = c.cell.centroid().unwrap();
    let fine = sim_core::GeoCell::from_lonlat(lon, lat, 15).unwrap();
    // Every world cell in the area finds its wind.
    for world in sim_core::cell::cells_in_bbox(23.85, 54.88, 23.95, 54.94, 15, 100_000).unwrap() {
        let w = f
            .wind_at(world, 10.0, (START + 1_800) as f64)
            .expect("covered");
        assert!((w.east_ms - 5.0).abs() < 1e-9);
    }
    let _ = fine;

    // The same scenario again, another day: no call.
    let again = Fetcher::new(Cache::new(&dir), hosts).unwrap();
    let second = again.fetch(&spec(), now + 86_400).await.unwrap();
    assert!(second.cache_hit);
    assert_eq!(
        seen.requests.load(Ordering::SeqCst),
        cells.len().div_ceil(100)
    );
    assert_eq!(second.field.rev(), first.field.rev());
    let _ = std::fs::remove_dir_all(&dir);
}

#[tokio::test]
async fn a_forecast_is_fetched_again_once_the_hour_turns() {
    let (host, seen) = stand_in().await;
    let dir = scratch("forecast");
    let hosts = Hosts {
        forecast: Some(host.clone()),
        historical: Some(host),
        ..Hosts::default()
    };
    let fetcher = Fetcher::new(Cache::new(&dir), hosts).unwrap();
    // "Now" is the start: the window runs into the future.
    let now = START + 60;
    fetcher.fetch(&spec(), now).await.unwrap();
    let calls = seen.requests.load(Ordering::SeqCst);
    assert!(fetcher.fetch(&spec(), now + 600).await.unwrap().cache_hit);
    assert_eq!(seen.requests.load(Ordering::SeqCst), calls);
    assert!(!fetcher.fetch(&spec(), now + 3_600).await.unwrap().cache_hit);
    assert!(seen.requests.load(Ordering::SeqCst) > calls);
    let _ = std::fs::remove_dir_all(&dir);
}

#[tokio::test]
async fn too_many_cells_is_refused_before_anything_is_asked() {
    let mut s = spec();
    s.resolution = 14;
    s.bbox = [23.0, 54.0, 24.0, 54.5];
    let err = Fetcher::cells(&s).unwrap_err().to_string();
    assert!(err.contains("weather_resolution"), "{err}");
}
