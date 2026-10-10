//! The weather API: what wind the run flies in, cell by cell.
//!
//! Everything here reads the weather service's copy of the field (what the
//! world was last given) and the published view's clock. Nothing waits on
//! the tick thread.

use axum::extract::{Path, Query, State};
use axum::Json;
use serde::{Deserialize, Serialize};
use sim_core::config::Param;
use sim_core::weather::CellWind;
use sim_core::{ErrorCode, GeoCell, Wind, WindField};
use sim_weather::format_utc;

use crate::problem::{ApiError, ApiResult};
use crate::weather::{world_rev, Origin, WeatherState};
use crate::AppState;

/// Heights shown when none is asked for, metres above the ground: about
/// where couriers cruise.
const DEFAULT_ALT_M: f64 = 50.0;

#[derive(Serialize)]
pub struct Window {
    pub first_utc: String,
    pub last_utc: String,
    pub frames: usize,
    pub step_s: u32,
}

#[derive(Serialize)]
pub struct CacheUse {
    pub dir: String,
    pub entries: usize,
    pub bytes: u64,
}

#[derive(Serialize)]
pub struct Limits {
    pub max_wind_ms: f64,
    pub max_gust_ms: f64,
}

#[derive(Serialize)]
pub struct WeatherStatus {
    /// There is a wind field: the drones fly in it.
    pub available: bool,
    pub mode: Option<sim_core::WeatherMode>,
    pub origin: Option<Origin>,
    pub source: Option<String>,
    pub attribution: Option<String>,
    /// The UTC time of tick 0.
    pub start_utc: Option<String>,
    /// The UTC time now in the simulation.
    pub sim_utc: Option<String>,
    pub window: Option<Window>,
    pub resolution: Option<i32>,
    pub cells: usize,
    pub levels_m: Vec<u16>,
    /// `[min_lon, min_lat, max_lon, max_lat]` fetched.
    pub area: Option<[f64; 4]>,
    pub model: Option<String>,
    pub hours: Option<u32>,
    pub refresh_min: Option<u32>,
    pub fetched_utc: Option<String>,
    pub attempted_utc: Option<String>,
    pub cache_hit: bool,
    /// Open-Meteo locations the last fetch cost; 0 from the cache.
    pub locations: usize,
    pub cache: Option<CacheUse>,
    /// Fingerprints of the field here and the one the world flies in.
    pub rev: Option<String>,
    pub world_rev: Option<String>,
    pub in_sync: bool,
    pub error: Option<String>,
    pub limits: Limits,
}

fn limits(state: &AppState) -> Limits {
    let view = state.driver.view();
    let p = |k: &str, d: f64| {
        view.config
            .model
            .get(k)
            .and_then(Param::as_f64)
            .unwrap_or(d)
    };
    Limits {
        max_wind_ms: p("max_wind_ms", 12.0),
        max_gust_ms: p("max_gust_ms", 15.0),
    }
}

/// UTC Unix seconds now in the simulation: the world's own clock if it has
/// weather, else worked out from the field's start and the tick.
fn sim_unix(state: &AppState, field: &WindField) -> f64 {
    let view = state.driver.view();
    if let Some(utc) = view.overlays["weather"]["utc"].as_f64() {
        return utc;
    }
    let tick_s = view
        .config
        .tick_seconds()
        .unwrap_or(sim_core::config::DEFAULT_TICK_SECONDS);
    field.utc_at(view.meta.tick as f64 * tick_s)
}

pub fn status_of(state: &AppState) -> WeatherStatus {
    let current: Option<std::sync::Arc<WeatherState>> = state.weather.current();
    let field = current.as_ref().and_then(|c| c.field.clone());
    let world = world_rev(&state.driver);
    let spec = current.as_ref().and_then(|c| c.spec.clone());
    WeatherStatus {
        available: field.is_some(),
        mode: current.as_ref().and_then(|c| c.mode()),
        origin: current.as_ref().map(|c| c.origin),
        source: field.as_ref().map(|f| f.source.clone()),
        attribution: field.as_ref().map(|f| f.attribution.clone()),
        start_utc: field.as_ref().map(|f| format_utc(f.start_unix)),
        sim_utc: field
            .as_ref()
            .map(|f| format_utc(sim_unix(state, f) as i64)),
        window: field.as_ref().map(|f| Window {
            first_utc: format_utc(f.t0_unix),
            last_utc: format_utc(f.frame_times().last().unwrap_or(f.t0_unix)),
            frames: f.frames,
            step_s: f.step_s,
        }),
        resolution: field.as_ref().map(|f| f.resolution),
        cells: field.as_ref().map_or(0, |f| f.cells.len()),
        levels_m: field
            .as_ref()
            .map(|f| f.levels_m.clone())
            .unwrap_or_default(),
        area: spec.as_ref().map(|s| s.bbox),
        model: spec.as_ref().map(|s| s.model.clone()),
        hours: spec.as_ref().map(|s| s.hours),
        refresh_min: spec
            .as_ref()
            .filter(|s| s.mode == sim_core::WeatherMode::Live)
            .map(|s| s.refresh_min),
        fetched_utc: field
            .as_ref()
            .filter(|f| f.fetched_unix > 0)
            .map(|f| format_utc(f.fetched_unix)),
        attempted_utc: current.as_ref().map(|c| format_utc(c.attempted_unix)),
        cache_hit: current.as_ref().is_some_and(|c| c.cache_hit),
        locations: current.as_ref().map_or(0, |c| c.locations),
        cache: state.weather.fetcher().map(|f| {
            let (entries, bytes) = f.cache().usage();
            CacheUse {
                dir: f.cache().dir().display().to_string(),
                entries,
                bytes,
            }
        }),
        rev: current
            .as_ref()
            .filter(|c| c.field.is_some())
            .map(|c| c.rev.to_string()),
        world_rev: world.map(|r| r.to_string()),
        in_sync: match (&current, world) {
            (Some(c), Some(w)) => c.field.is_some() && c.rev == w,
            (Some(c), None) => c.field.is_none(),
            (None, w) => w.is_none(),
        },
        error: current.as_ref().and_then(|c| c.error.clone()),
        limits: limits(state),
    }
}

/// What wind the run flies in, where it came from, and how fresh it is.
pub async fn weather_status(State(state): State<AppState>) -> Json<WeatherStatus> {
    Json(status_of(&state))
}

/// Fetch a live run's weather again now.
pub async fn weather_refresh(State(state): State<AppState>) -> ApiResult<Json<WeatherStatus>> {
    state
        .weather
        .refresh(&state.driver)
        .await
        .map_err(|err| ApiError::bad_request(format!("{err:#}")))?;
    let status = status_of(&state);
    if let Some(err) = &status.error {
        return Err(ApiError::new(ErrorCode::WeatherUnavailable, err.clone()));
    }
    Ok(Json(status))
}

#[derive(Deserialize)]
pub struct CellsQuery {
    /// Metres above the ground.
    #[serde(default)]
    pub alt_m: Option<f64>,
    /// An RFC 3339 time; the simulation's now if absent.
    #[serde(default)]
    pub utc: Option<String>,
}

#[derive(Serialize, Clone, Copy)]
pub struct WindView {
    pub east_ms: f64,
    pub north_ms: f64,
    pub speed_ms: f64,
    /// Where it blows from, degrees clockwise from north.
    pub from_deg: f64,
    pub gust_ms: f64,
}

impl From<Wind> for WindView {
    fn from(w: Wind) -> Self {
        let r = |v: f64| (v * 100.0).round() / 100.0;
        WindView {
            east_ms: r(w.east_ms),
            north_ms: r(w.north_ms),
            speed_ms: r(w.speed_ms()),
            from_deg: (w.from_deg() * 10.0).round() / 10.0,
            gust_ms: r(w.gust_ms),
        }
    }
}

#[derive(Serialize)]
pub struct CellWindView {
    pub cell: GeoCell,
    #[serde(flatten)]
    pub wind: WindView,
    /// Boundary ring as `[lon, lat]` pairs.
    pub boundary: Vec<[f64; 2]>,
}

#[derive(Serialize)]
pub struct CellsBody {
    pub utc: String,
    pub alt_m: f64,
    pub resolution: i32,
    pub max_speed_ms: f64,
    pub cells: Vec<CellWindView>,
}

fn field_of(state: &AppState) -> ApiResult<std::sync::Arc<WindField>> {
    state
        .weather
        .current()
        .and_then(|c| c.field.clone())
        .ok_or_else(|| ApiError::not_found("this run has no weather"))
}

fn when(state: &AppState, field: &WindField, utc: Option<&str>) -> ApiResult<f64> {
    match utc {
        Some(t) => sim_weather::parse_utc(t)
            .map(|t| t as f64)
            .map_err(|err| ApiError::bad_request(err.to_string())),
        None => Ok(sim_unix(state, field)),
    }
}

fn height(alt_m: Option<f64>) -> ApiResult<f64> {
    let alt = alt_m.unwrap_or(DEFAULT_ALT_M);
    if !(0.0..=10_000.0).contains(&alt) {
        return Err(ApiError::bad_request(
            "alt_m is metres above the ground, 0 to 10000",
        ));
    }
    Ok(alt)
}

/// The wind in every weather cell, at a height and a time.
pub async fn weather_cells(
    State(state): State<AppState>,
    Query(q): Query<CellsQuery>,
) -> ApiResult<Json<CellsBody>> {
    let field = field_of(&state)?;
    let alt_m = height(q.alt_m)?;
    let utc = when(&state, &field, q.utc.as_deref())?;
    let cells: Vec<CellWindView> = field
        .cells
        .iter()
        .map(|c: &CellWind| CellWindView {
            cell: c.cell,
            wind: field.sample(c, alt_m, utc).into(),
            boundary: c
                .cell
                .boundary()
                .unwrap_or_default()
                .into_iter()
                .map(|(lon, lat)| [lon, lat])
                .collect(),
        })
        .collect();
    Ok(Json(CellsBody {
        utc: format_utc(utc as i64),
        alt_m,
        resolution: field.resolution,
        max_speed_ms: cells.iter().map(|c| c.wind.speed_ms).fold(0.0, f64::max),
        cells,
    }))
}

#[derive(Serialize)]
pub struct LevelView {
    pub alt_m: f64,
    #[serde(flatten)]
    pub wind: WindView,
}

#[derive(Serialize)]
pub struct FrameView {
    pub utc: String,
    #[serde(flatten)]
    pub wind: WindView,
}

#[derive(Serialize)]
pub struct CellDetail {
    /// The cell asked about.
    pub cell: GeoCell,
    /// The weather cell it takes its wind from: itself, or its ancestor at
    /// the weather resolution.
    pub weather_cell: GeoCell,
    pub resolution: i32,
    /// `[lon, lat]` of the weather cell's centre, where it was fetched.
    pub centroid: [f64; 2],
    pub boundary: Vec<[f64; 2]>,
    pub utc: String,
    pub alt_m: f64,
    /// Now, at the height asked.
    pub wind: WindView,
    /// Now, at each fetched level.
    pub levels: Vec<LevelView>,
    /// Every frame, at the height asked.
    pub series: Vec<FrameView>,
}

/// The wind a cell (of any resolution at least as fine as the weather's)
/// takes: now, at each height, and through the whole window.
pub async fn weather_cell(
    State(state): State<AppState>,
    Path(cell): Path<String>,
    Query(q): Query<CellsQuery>,
) -> ApiResult<Json<CellDetail>> {
    let field = field_of(&state)?;
    let cell = GeoCell::parse(&cell).map_err(ApiError::from)?;
    let alt_m = height(q.alt_m)?;
    let utc = when(&state, &field, q.utc.as_deref())?;
    let weather_cell = field.cell_for(cell).ok_or_else(|| {
        ApiError::bad_request(format!(
            "cell {cell} is coarser than the weather's resolution {}",
            field.resolution
        ))
    })?;
    let series = field
        .series(weather_cell)
        .ok_or_else(|| ApiError::not_found(format!("no weather was fetched for {weather_cell}")))?;
    let (lon, lat) = weather_cell.centroid().map_err(ApiError::from)?;
    Ok(Json(CellDetail {
        cell,
        weather_cell,
        resolution: field.resolution,
        centroid: [lon, lat],
        boundary: weather_cell
            .boundary()
            .unwrap_or_default()
            .into_iter()
            .map(|(lon, lat)| [lon, lat])
            .collect(),
        utc: format_utc(utc as i64),
        alt_m,
        wind: field.sample(series, alt_m, utc).into(),
        levels: field
            .levels_m
            .iter()
            .map(|&m| LevelView {
                alt_m: f64::from(m),
                wind: field.sample(series, f64::from(m), utc).into(),
            })
            .collect(),
        series: field
            .frame_times()
            .map(|t| FrameView {
                utc: format_utc(t),
                wind: field.sample(series, alt_m, t as f64).into(),
            })
            .collect(),
    }))
}
