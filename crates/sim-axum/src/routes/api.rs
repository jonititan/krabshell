//! REST surface (spec §9.2).
//!
//! Handlers do three kinds of thing and nothing else: read the published view,
//! post a message to the driver, or touch the scenario directory. Anything that
//! blocks — a driver round trip, a directory scan — goes through
//! `spawn_blocking` so a Tokio worker is never parked on the tick thread.

use std::sync::Arc;

use axum::body::Bytes;
use axum::extract::{Multipart, Path, Query, State};
use axum::http::header;
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::{Deserialize, Serialize};
use sim_core::cell::{self, CellError};
use sim_core::{
    CellBag, Config, DriverHealth, ErrorCode, GeoCell, Intent, Metrics, Profile, Report,
    ReportKind, RunMode, ScenarioSummary, SnapshotMeta,
};
use sim_engine::{checkpoint, Control, IntentAck, Published, Submission};

use crate::problem::{from_driver, ApiError, ApiResult};
use crate::{AppState, MAX_INTENT_BYTES};

/// Ceiling on how many cells one REST query may name or return.
const MAX_QUERY_CELLS: usize = 20_000;

// ---------------------------------------------------------------- health

#[derive(Serialize)]
pub struct HealthBody {
    pub status: &'static str,
    pub engine: &'static str,
    pub uptime_seconds: u64,
    pub tick: u64,
    #[serde(flatten)]
    pub driver: DriverHealth,
}

/// Liveness. Unauthenticated on purpose so a monitor can watch it.
pub async fn health(State(state): State<AppState>) -> Response {
    let view = state.driver.view();
    let driver = state.driver.health();
    let ok = driver.alive && driver.mode != RunMode::Faulted;
    let body = HealthBody {
        status: if ok { "ok" } else { "degraded" },
        engine: sim_engine::engine_description(),
        uptime_seconds: state.started_at.elapsed().as_secs(),
        tick: view.meta.tick,
        driver,
    };
    let code = if ok {
        axum::http::StatusCode::OK
    } else {
        axum::http::StatusCode::SERVICE_UNAVAILABLE
    };
    (code, Json(body)).into_response()
}

// ------------------------------------------------------------------- run

#[derive(Serialize)]
pub struct RunStatus {
    pub mode: RunMode,
    pub scenario_id: String,
    pub scenario_name: String,
    pub model: String,
    pub engine: &'static str,
    pub profile: Profile,
    pub parallel: bool,
    pub threads: u32,
    pub target_tps: Option<f64>,
    /// Flattened, so `tick` and the counters sit at the top level.
    #[serde(flatten)]
    pub metrics: Metrics,
    pub meta: SnapshotMeta,
    pub health: DriverHealth,
    /// True when the published view dropped cells at the cap.
    pub truncated: bool,
    pub recent_intents: Vec<sim_engine::IntentRecord>,
}

fn status_of(state: &AppState, view: &Published) -> RunStatus {
    RunStatus {
        mode: view.health.mode,
        scenario_id: view.scenario_id.clone(),
        scenario_name: view.scenario_name.clone(),
        model: view.model_kind.clone(),
        engine: sim_engine::engine_description(),
        profile: view.config.run.profile,
        parallel: view.config.run.parallel,
        threads: view.config.run.threads,
        target_tps: view.config.run.target_tps,
        metrics: view.metrics,
        meta: view.meta.clone(),
        health: state.driver.health(),
        truncated: view.truncated,
        recent_intents: view.recent_intents.clone(),
    }
}

pub async fn run_status(State(state): State<AppState>) -> Json<RunStatus> {
    let view = state.driver.view();
    Json(status_of(&state, &view))
}

#[derive(Deserialize, Default)]
pub struct StartBody {
    /// Absent or null means maximum speed.
    #[serde(default)]
    pub target_tps: Option<f64>,
}

pub async fn run_start(
    State(state): State<AppState>,
    body: Option<Json<StartBody>>,
) -> ApiResult<Json<RunStatus>> {
    let target_tps = body.and_then(|Json(b)| b.target_tps);
    if target_tps.is_some_and(|tps| !(tps > 0.0 && tps.is_finite())) {
        return Err(ApiError::bad_request("target_tps must be positive or null"));
    }
    command(&state, Control::Start { target_tps }).await
}

pub async fn run_pause(State(state): State<AppState>) -> ApiResult<Json<RunStatus>> {
    command(&state, Control::Pause).await
}

#[derive(Deserialize, Default)]
pub struct StepBody {
    #[serde(default)]
    pub n: Option<u64>,
}

pub async fn run_step(
    State(state): State<AppState>,
    body: Option<Json<StepBody>>,
) -> ApiResult<Json<RunStatus>> {
    let n = body.and_then(|Json(b)| b.n).unwrap_or(1);
    if n == 0 {
        return Err(ApiError::bad_request("n must be at least 1"));
    }
    let accepted = command(&state, Control::Step { n }).await?;
    // The driver acks a step burst when it accepts it, not when it finishes.
    // An operator asking for one step expects the answer to describe the world
    // after that step, so wait for the burst here rather than returning a tick
    // the caller would have to poll past.
    if settle(&state).await {
        let view = state.driver.view();
        return Ok(Json(status_of(&state, &view)));
    }
    Ok(accepted)
}

/// Wait for a step burst to finish. Returns false if it is still going.
///
/// Long bursts are legitimate, so this gives up rather than holding the
/// connection: the returned status then says `stepping`, which is the truth.
pub async fn settle(state: &AppState) -> bool {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    while std::time::Instant::now() < deadline {
        if state.driver.view().health.mode != RunMode::Stepping {
            return true;
        }
        tokio::time::sleep(std::time::Duration::from_millis(2)).await;
    }
    false
}

pub async fn run_reset(State(state): State<AppState>) -> ApiResult<Json<RunStatus>> {
    command(&state, Control::Reset).await
}

#[derive(Deserialize)]
pub struct RateBody {
    /// `null` means maximum speed.
    pub target_tps: Option<f64>,
}

pub async fn run_rate(
    State(state): State<AppState>,
    Json(body): Json<RateBody>,
) -> ApiResult<Json<RunStatus>> {
    if body
        .target_tps
        .is_some_and(|tps| !(tps > 0.0 && tps.is_finite()))
    {
        return Err(ApiError::bad_request("target_tps must be positive or null"));
    }
    command(&state, Control::SetRate(body.target_tps)).await
}

/// Send a control command off-thread and answer with the resulting status.
///
/// The driver publishes before it acks, so the status returned here already
/// reflects the command rather than the frame before it.
async fn command(state: &AppState, control: Control) -> ApiResult<Json<RunStatus>> {
    let driver = state.driver.clone();
    tokio::task::spawn_blocking(move || driver.command(control))
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?
        .map_err(from_driver)?;
    let view = state.driver.view();
    Ok(Json(status_of(state, &view)))
}

// ------------------------------------------------------------- scenarios

pub async fn scenarios_list(
    State(state): State<AppState>,
) -> ApiResult<Json<Vec<ScenarioSummary>>> {
    let store = Arc::clone(&state.scenarios);
    let packs = tokio::task::spawn_blocking(move || store.list())
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?;
    Ok(Json(packs))
}

pub async fn scenarios_load(
    State(state): State<AppState>,
    Path(id): Path<String>,
) -> ApiResult<Json<RunStatus>> {
    let store = Arc::clone(&state.scenarios);
    let mut blob = tokio::task::spawn_blocking(move || store.load(&id))
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))??;
    state.weather.prepare(&mut blob).await?;
    command(&state, Control::Load(Box::new(blob))).await
}

#[derive(Serialize)]
pub struct UploadResult {
    pub scenario: ScenarioSummary,
}

/// Install a pack from a multipart upload.
///
/// The zip reader enforces the limits in §13; this handler only decides which
/// part of the form is the archive and what to call it.
pub async fn scenarios_upload(
    State(state): State<AppState>,
    mut multipart: Multipart,
) -> ApiResult<Json<UploadResult>> {
    let mut id: Option<String> = None;
    let mut archive: Option<Bytes> = None;

    while let Some(field) = multipart
        .next_field()
        .await
        .map_err(|err| ApiError::bad_request(format!("malformed upload: {err}")))?
    {
        match field.name().unwrap_or_default() {
            "id" => {
                id = Some(
                    field
                        .text()
                        .await
                        .map_err(|err| ApiError::bad_request(err.to_string()))?,
                )
            }
            "pack" | "file" => {
                let name = field.file_name().map(str::to_owned);
                archive = Some(
                    field
                        .bytes()
                        .await
                        .map_err(|err| ApiError::bad_request(err.to_string()))?,
                );
                // Fall back to the filename stem when no explicit id was given.
                if id.is_none() {
                    id = name.and_then(|n| {
                        std::path::Path::new(&n)
                            .file_stem()
                            .map(|s| s.to_string_lossy().into_owned())
                    });
                }
            }
            _ => {}
        }
    }

    let (id, archive) = match (id, archive) {
        (Some(id), Some(archive)) => (id, archive),
        _ => {
            return Err(ApiError::bad_request(
                "expected a multipart form with a 'pack' zip and an 'id'",
            ))
        }
    };

    let store = Arc::clone(&state.scenarios);
    let scenario = tokio::task::spawn_blocking(move || store.import_zip(&id, &archive))
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))??;
    // A pack uploaded again under the same id may bring a new address book.
    state.addresses.forget(&scenario.id);
    Ok(Json(UploadResult { scenario }))
}

// ---------------------------------------------------------------- config

pub async fn config_get(State(state): State<AppState>) -> Json<Config> {
    Json(state.driver.view().config.clone())
}

#[derive(Serialize)]
pub struct ConfigApplied {
    pub config: Config,
    /// Fields whose change forced a world rebuild.
    pub reset_fields: Vec<String>,
    pub tick: u64,
}

pub async fn config_put(
    State(state): State<AppState>,
    Json(next): Json<Config>,
) -> ApiResult<Json<ConfigApplied>> {
    next.validate()?;
    let current = state.driver.view().config.clone();
    let reset_fields: Vec<String> = current
        .changes_requiring_reset(&next)
        .into_iter()
        .map(str::to_owned)
        .collect();

    let driver = state.driver.clone();
    let control = Control::SetConfig(Box::new(next));
    tokio::task::spawn_blocking(move || driver.command(control))
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?
        .map_err(from_driver)?;

    let view = state.driver.view();
    Ok(Json(ConfigApplied {
        config: view.config.clone(),
        reset_fields,
        tick: view.meta.tick,
    }))
}

/// The config as it is *currently effective*, not the file on disk (spec §10.2).
pub async fn config_download(State(state): State<AppState>) -> ApiResult<Response> {
    let view = state.driver.view();
    let text = toml::to_string_pretty(&view.config)
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?;
    let filename = format!("{}-config.toml", view.scenario_id);
    Ok((
        [
            (header::CONTENT_TYPE, "application/toml".to_owned()),
            (
                header::CONTENT_DISPOSITION,
                format!("attachment; filename=\"{filename}\""),
            ),
        ],
        text,
    )
        .into_response())
}

// ----------------------------------------------------------------- state

#[derive(Deserialize)]
pub struct StateQuery {
    /// Comma-separated cell ids. Absent means the whole published view.
    #[serde(default)]
    pub cells: Option<String>,
    /// Aggregate to this resolution instead of returning occupancy cells.
    #[serde(default)]
    pub res: Option<i32>,
    /// Expand an area of interest around a centre cell.
    #[serde(default)]
    pub center: Option<String>,
    #[serde(default)]
    pub k: Option<u32>,
}

#[derive(Serialize)]
pub struct StateBody {
    pub meta: SnapshotMeta,
    pub cells: Vec<CellBag>,
    /// True when the published view itself was capped, so cells may be missing.
    pub truncated: bool,
}

pub async fn state_get(
    State(state): State<AppState>,
    Query(query): Query<StateQuery>,
) -> ApiResult<Json<StateBody>> {
    let view = state.driver.view();
    let selection = resolve_selection(&query, &view)?;

    let cells = match selection {
        Some(wanted) => wanted
            .into_iter()
            .filter_map(|cell| view.bag(cell).cloned())
            .collect(),
        None => {
            if view.cells.len() > MAX_QUERY_CELLS {
                return Err(ApiError::new(
                    ErrorCode::BadRequest,
                    format!(
                        "the world has {} occupied cells; name the cells you want, or use /state/map",
                        view.cells.len()
                    ),
                ));
            }
            view.cells.clone()
        }
    };

    Ok(Json(StateBody {
        meta: view.meta.clone(),
        cells,
        truncated: view.truncated,
    }))
}

/// Which cells a query names, or `None` for "everything published".
fn resolve_selection(query: &StateQuery, view: &Published) -> ApiResult<Option<Vec<GeoCell>>> {
    if let Some(list) = &query.cells {
        let cells = parse_cell_list(list)?;
        return Ok(Some(cells));
    }
    if let Some(center) = &query.center {
        let center = GeoCell::parse(center).map_err(ApiError::from)?;
        let k = query.k.unwrap_or(1).min(64) as usize;
        let disk = center.grid_disk(k).map_err(ApiError::from)?;
        if disk.len() > MAX_QUERY_CELLS {
            return Err(ApiError::from(CellError::TooManyCells {
                limit: MAX_QUERY_CELLS,
            }));
        }
        return Ok(Some(disk));
    }
    let _ = view;
    Ok(None)
}

fn parse_cell_list(list: &str) -> ApiResult<Vec<GeoCell>> {
    let mut cells = Vec::new();
    for token in list.split(',').map(str::trim).filter(|t| !t.is_empty()) {
        if cells.len() >= MAX_QUERY_CELLS {
            return Err(ApiError::from(CellError::TooManyCells {
                limit: MAX_QUERY_CELLS,
            }));
        }
        cells.push(GeoCell::parse(token).map_err(ApiError::from)?);
    }
    Ok(cells)
}

#[derive(Serialize)]
pub struct MapCell {
    pub cell: GeoCell,
    pub count: u64,
    /// Boundary ring as `[lon, lat]` pairs, ready to draw.
    pub boundary: Vec<[f64; 2]>,
}

#[derive(Serialize)]
pub struct MapBody {
    pub tick: u64,
    pub resolution: i32,
    pub cells: Vec<MapCell>,
    pub max_count: u64,
    pub truncated: bool,
}

#[derive(Deserialize)]
pub struct MapQuery {
    #[serde(default)]
    pub res: Option<i32>,
}

/// Occupancy rolled up to a display resolution, with polygons (spec §7.3).
///
/// This is what makes the dashboard map cheap: a city of a hundred thousand
/// occupancy cells becomes a few hundred parent cells before it ever reaches
/// the browser.
pub async fn state_map(
    State(state): State<AppState>,
    Query(query): Query<MapQuery>,
) -> ApiResult<Json<MapBody>> {
    let view = state.driver.view();
    let resolution = query
        .res
        .unwrap_or(view.config.grid.view_resolution)
        .clamp(0, view.meta.resolution);

    let rolled = view.aggregate(resolution);
    if rolled.len() > MAX_QUERY_CELLS {
        return Err(ApiError::new(
            ErrorCode::BadRequest,
            format!(
                "aggregating to resolution {resolution} gives {} cells; ask for a coarser resolution",
                rolled.len()
            ),
        ));
    }

    let max_count = rolled.iter().map(|(_, count)| *count).max().unwrap_or(0);
    let cells = rolled
        .into_iter()
        .map(|(cell, count)| MapCell {
            cell,
            count,
            boundary: cell
                .boundary()
                .unwrap_or_default()
                .into_iter()
                .map(|(lon, lat)| [lon, lat])
                .collect(),
        })
        .collect();

    Ok(Json(MapBody {
        tick: view.meta.tick,
        resolution,
        cells,
        max_count,
        truncated: view.truncated,
    }))
}

pub async fn state_cell(
    State(state): State<AppState>,
    Path(cell): Path<String>,
) -> ApiResult<Json<CellBag>> {
    let cell = GeoCell::parse(&cell).map_err(ApiError::from)?;
    let view = state.driver.view();
    match view.bag(cell) {
        Some(bag) => Ok(Json(bag.clone())),
        None if view.truncated => Err(ApiError::new(
            ErrorCode::NotFound,
            "cell is empty, or was dropped from the published view at the cell cap",
        )),
        None => Ok(Json(CellBag {
            cell,
            objects: Vec::new(),
        })),
    }
}

/// Copy the committed world out of the driver and serve it as a file.
pub async fn state_download(State(state): State<AppState>) -> ApiResult<Response> {
    let driver = state.driver.clone();
    let snapshot = tokio::task::spawn_blocking(move || driver.checkpoint())
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?
        .map_err(from_driver)?;

    let filename = checkpoint::suggested_filename(&snapshot);
    let body = tokio::task::spawn_blocking(move || checkpoint::encode(&snapshot))
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?;

    Ok((
        [
            (header::CONTENT_TYPE, "application/json".to_owned()),
            (
                header::CONTENT_DISPOSITION,
                format!("attachment; filename=\"{filename}\""),
            ),
        ],
        body,
    )
        .into_response())
}

#[derive(Serialize)]
pub struct RestoreResult {
    pub tick: u64,
    pub agent_count: u64,
    pub hash: Option<u64>,
}

/// Restore a checkpoint. The driver must be paused (spec §9.2).
pub async fn state_upload(
    State(state): State<AppState>,
    body: Bytes,
) -> ApiResult<Json<RestoreResult>> {
    if state.driver.view().health.mode.is_advancing() {
        return Err(ApiError::new(
            ErrorCode::RequiresPause,
            "pause the run before restoring a checkpoint",
        ));
    }
    let snapshot = tokio::task::spawn_blocking(move || checkpoint::decode(&body))
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?
        .map_err(|err| ApiError::new(ErrorCode::BadRequest, err.to_string()))?;
    // The wind it was flying in, if it says; a checkpoint from before
    // weather leaves the world (and this) with the pack's.
    let weather = match snapshot.fields.get("weather") {
        None => None,
        Some(serde_json::Value::Null) => Some(None),
        Some(v) => Some(Some(sim_core::WindField::from_value(v).map_err(|err| {
            ApiError::new(ErrorCode::BadRequest, format!("checkpoint weather: {err}"))
        })?)),
    };
    let scenario_id = snapshot.meta.scenario_id.clone();

    let driver = state.driver.clone();
    let control = Control::Restore(Box::new(snapshot));
    tokio::task::spawn_blocking(move || driver.command(control))
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?
        .map_err(from_driver)?;

    if let Some(field) = weather {
        state.weather.adopt(&scenario_id, field);
    }
    let view = state.driver.view();
    Ok(Json(RestoreResult {
        tick: view.meta.tick,
        agent_count: view.meta.agent_count,
        hash: view.meta.hash,
    }))
}

// --------------------------------------------------------------- intents

#[derive(Serialize)]
pub struct IntentAccepted {
    pub id: uuid::Uuid,
    pub applied_tick: Option<u64>,
    pub status: &'static str,
}

/// Submit one intent over REST. The WebSocket is the better path for anything
/// interactive; this exists so `curl` can drive the world (spec §14, Phase 3).
pub async fn intents_post(
    State(state): State<AppState>,
    body: Bytes,
) -> ApiResult<Json<IntentAccepted>> {
    if body.len() > MAX_INTENT_BYTES {
        return Err(ApiError::new(
            ErrorCode::PayloadTooLarge,
            format!("intents are capped at {MAX_INTENT_BYTES} bytes"),
        ));
    }
    let intent: Intent = serde_json::from_slice(&body)
        .map_err(|err| ApiError::bad_request(format!("malformed intent: {err}")))?;
    submit_intent(&state, intent).await
}

/// Admit an intent, hand it to the driver, and wait briefly for its outcome.
async fn submit_intent(state: &AppState, intent: Intent) -> ApiResult<Json<IntentAccepted>> {
    // Bounds are checked here so an obviously misaddressed intent never costs
    // the driver a queue slot (spec §8.2).
    let view = state.driver.view();
    sim_engine::check_intent_cell(&view.meta, intent.cell)
        .map_err(|err| ApiError::new(ErrorCode::CellOutOfBounds, err.to_string()))?;

    if !state
        .limiter
        .allow(&intent.actor.0, view.config.run.intent_rate_per_actor)
    {
        return Err(ApiError::from(sim_core::RejectReason::RateLimited));
    }

    let id = intent.id;
    let (ack_tx, ack_rx) = crossbeam_channel::bounded(1);
    state
        .driver
        .submit(Submission {
            intent,
            ack: Some(ack_tx),
        })
        .map_err(ApiError::from)?;

    // Wait briefly for the driver to reach the intent. A paused world will not,
    // which is not an error: the intent stays queued for the next tick.
    let outcome = tokio::task::spawn_blocking(move || {
        ack_rx.recv_timeout(std::time::Duration::from_millis(500))
    })
    .await
    .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?;

    match outcome {
        Ok(IntentAck::Applied { tick }) => Ok(Json(IntentAccepted {
            id,
            applied_tick: Some(tick),
            status: "applied",
        })),
        Ok(IntentAck::Rejected(reason)) => Err(ApiError::from(reason)),
        // Dropped unanswered: it will never apply, so it is not "queued".
        Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
            Err(ApiError::from(sim_core::RejectReason::Discarded {
                why: "the driver dropped it unanswered".into(),
            }))
        }
        Err(crossbeam_channel::RecvTimeoutError::Timeout) => Ok(Json(IntentAccepted {
            id,
            applied_tick: None,
            status: "queued",
        })),
    }
}

// ------------------------------------------------------------- overlays

/// What the model publishes beside its agents, as it publishes it: for the
/// drones, the geofences and the addresses of open orders. Null for a model
/// with none. The tick is in `x-sim-tick`.
pub async fn state_overlays(
    State(state): State<AppState>,
) -> (axum::http::HeaderMap, Json<serde_json::Value>) {
    let view = state.driver.view();
    let mut headers = axum::http::HeaderMap::new();
    if let Ok(tick) = axum::http::HeaderValue::from_str(&view.meta.tick.to_string()) {
        headers.insert("x-sim-tick", tick);
    }
    (headers, Json(view.overlays.clone()))
}

/// Every delivery address in the loaded pack's address book, for the map's
/// layer of them: `{"scenario_id", "depots", "items"}`, each item
/// `[lat_e7, lon_e7, depot, label]` with `depot` an index into `depots`.
/// Empty for a pack without a book. The book never changes while a world
/// runs, so it is read from the pack once and the ETag is the scenario: a
/// client sends it back in `If-None-Match` and gets 304 until another
/// scenario is loaded.
pub async fn state_addresses(
    State(state): State<AppState>,
    headers: axum::http::HeaderMap,
) -> ApiResult<Response> {
    let scenario = state.driver.view().scenario_id.clone();
    let etag = format!("\"{}\"", scenario.replace(['"', '\\'], "_"));
    let fresh = headers
        .get(header::IF_NONE_MATCH)
        .and_then(|v| v.to_str().ok())
        .is_some_and(|v| v.split(',').any(|t| t.trim() == etag));
    if fresh {
        return Ok((axum::http::StatusCode::NOT_MODIFIED, [(header::ETAG, etag)]).into_response());
    }
    let book = Arc::clone(&state.addresses);
    let store = Arc::clone(&state.scenarios);
    let body = tokio::task::spawn_blocking(move || book.get(&store, &scenario))
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?;
    Ok((
        [
            (header::ETAG, etag),
            (header::CONTENT_TYPE, "application/json".to_owned()),
        ],
        body.to_vec(),
    )
        .into_response())
}

/// The loaded pack's address book, serialised once per scenario.
#[derive(Default)]
pub struct AddressCache {
    held: std::sync::Mutex<Option<(String, Bytes)>>,
}

impl AddressCache {
    fn get(&self, store: &sim_engine::ScenarioStore, scenario: &str) -> Bytes {
        if let Some((id, body)) = &*self.held.lock().unwrap() {
            if id == scenario {
                return body.clone();
            }
        }
        // A world with no pack on disk (reset to empty, or a pack since
        // removed) has no book to show.
        let book = store
            .load(scenario)
            .ok()
            .and_then(|blob| blob.fields.get("addresses").cloned());
        let depots = book.as_ref().and_then(|b| b.get("depots")).cloned();
        let items = book.as_ref().and_then(|b| b.get("items")).cloned();
        let body = serde_json::json!({
            "scenario_id": scenario,
            "depots": depots.unwrap_or_else(|| serde_json::json!([])),
            "items": items.unwrap_or_else(|| serde_json::json!([])),
        });
        let body = Bytes::from(serde_json::to_vec(&body).unwrap_or_default());
        *self.held.lock().unwrap() = Some((scenario.to_owned(), body.clone()));
        body
    }

    fn forget(&self, scenario: &str) {
        let mut held = self.held.lock().unwrap();
        if held.as_ref().is_some_and(|(id, _)| id == scenario) {
            *held = None;
        }
    }
}

// ------------------------------------------------------------- geofences

/// `Custom` intent codes for geofences; see `sim_drones::codes`.
const GEOFENCE_OPEN: u16 = 110;
const GEOFENCE_CLOSE: u16 = 111;
const GEOFENCE_EXTEND: u16 = 113;
const GEOFENCE_RENAME: u16 = 114;

#[derive(Serialize)]
pub struct GeofencesBody {
    pub tick: u64,
    /// Simulated seconds per tick, when the model says.
    pub tick_seconds: Option<f64>,
    pub mode: String,
    /// Bumped whenever a fence opens or closes.
    pub rev: u64,
    /// How far the drones the simulation flies keep from a fence, metres.
    pub margin_m: f64,
    /// The fences open now, as the model publishes them, each with
    /// `breaches`: how many the recent reports hold.
    pub fences: Vec<serde_json::Value>,
    /// Fences waiting to open, soonest first; `opened_tick` is when.
    pub scheduled: Vec<serde_json::Value>,
    /// Fences that have closed, newest first, with `closed_tick` and how
    /// they `closed`: to open again.
    pub previous: Vec<serde_json::Value>,
    /// The latest breach and reroute reports, newest first.
    pub recent: Vec<Report>,
    /// Intents the model refused lately, so a client that submitted one
    /// while the world was paused (and got `queued`) learns why it never
    /// took effect.
    pub refused: Vec<Refused>,
}

#[derive(Serialize)]
pub struct Refused {
    pub id: uuid::Uuid,
    pub detail: String,
}

fn tick_seconds(view: &Published) -> Option<f64> {
    view.config.tick_seconds()
}

/// The geofences open now, and what has happened to fences lately. Empty
/// for a model without geofences.
pub async fn geofences_list(State(state): State<AppState>) -> Json<GeofencesBody> {
    let view = state.driver.view();
    let g = &view.overlays["geofences"];
    let breaches = |id: u64| {
        view.reports
            .iter()
            .filter(|r| r.kind == ReportKind::Breach && r.detail["fence"].as_u64() == Some(id))
            .count()
    };
    let fences = g["fences"]
        .as_array()
        .map(|list| {
            list.iter()
                .map(|f| {
                    let mut f = f.clone();
                    let id = f["id"].as_u64().unwrap_or(0);
                    f["breaches"] = breaches(id).into();
                    f
                })
                .collect()
        })
        .unwrap_or_default();
    Json(GeofencesBody {
        tick: view.meta.tick,
        tick_seconds: tick_seconds(&view),
        mode: mode_of(&view),
        rev: g["rev"].as_u64().unwrap_or(0),
        margin_m: g["margin_m"].as_f64().unwrap_or(0.0),
        fences,
        scheduled: g["scheduled"].as_array().cloned().unwrap_or_default(),
        previous: g["previous"].as_array().cloned().unwrap_or_default(),
        recent: recent_reports(&view, &[ReportKind::Breach, ReportKind::Reroute]),
        refused: refused(&view),
    })
}

/// The latest reports of some kinds, newest first.
fn recent_reports(view: &Published, kinds: &[ReportKind]) -> Vec<Report> {
    view.reports
        .iter()
        .rev()
        .filter(|r| kinds.contains(&r.kind))
        .take(100)
        .cloned()
        .collect()
}

/// Intents the model refused lately.
fn refused(view: &Published) -> Vec<Refused> {
    view.recent_intents
        .iter()
        .filter(|r| r.rejected.is_some())
        .map(|r| Refused {
            id: r.id,
            detail: r.detail.clone(),
        })
        .collect()
}

fn mode_of(view: &Published) -> String {
    format!("{:?}", view.health.mode).to_lowercase()
}

/// A cell inside the world near a point, to address a geofence intent to.
fn cell_near(view: &Published, lon: f64, lat: f64) -> ApiResult<GeoCell> {
    let cell = GeoCell::from_lonlat(lon, lat, view.meta.resolution).map_err(ApiError::from)?;
    sim_engine::check_intent_cell(&view.meta, cell)
        .map_err(|err| ApiError::new(ErrorCode::CellOutOfBounds, err.to_string()))?;
    Ok(cell)
}

/// A fence's altitude band, `floor_m` and `ceiling_m` in metres above
/// ground level: each a number of 0 or more, or null or absent (the surface;
/// unlimited), with the ceiling above the floor. The model checks again;
/// this is for a clear answer before the intent takes a queue slot.
fn check_band(body: &serde_json::Value) -> ApiResult<()> {
    let metres = |key: &str| -> ApiResult<Option<f64>> {
        match body.get(key) {
            None | Some(serde_json::Value::Null) => Ok(None),
            Some(v) => match v.as_f64() {
                Some(m) if m.is_finite() && m >= 0.0 => Ok(Some(m)),
                _ => Err(ApiError::bad_request(format!(
                    "{key} is metres above ground level: a number of 0 or more, or null"
                ))),
            },
        }
    };
    let floor = metres("floor_m")?;
    if let Some(ceiling) = metres("ceiling_m")? {
        if ceiling <= floor.unwrap_or(0.0) {
            return Err(ApiError::bad_request(
                "ceiling_m must be above floor_m (or above the surface)",
            ));
        }
    }
    Ok(())
}

/// Open a geofence: `{"circle": {"lat", "lon", "radius_m"}}` or
/// `{"polygon": [[lon, lat], ...]}`, in degrees, with optionally a `"name"`,
/// `"minutes"` it stays open (-1 or absent: until closed),
/// `"after_minutes"` before it opens (0 or absent: on the next tick), and an
/// altitude band: `"floor_m"` (absent or null: from the surface) and
/// `"ceiling_m"` (absent or null: unlimited), metres above ground level.
pub async fn geofences_open(
    State(state): State<AppState>,
    Json(body): Json<serde_json::Value>,
) -> ApiResult<Json<IntentAccepted>> {
    let view = state.driver.view();
    check_band(&body)?;
    let num = |v: &serde_json::Value, k: &str| v.get(k).and_then(serde_json::Value::as_f64);
    let (lon, lat) = if let Some(c) = body.get("circle") {
        match (num(c, "lon"), num(c, "lat")) {
            (Some(lon), Some(lat)) => (lon, lat),
            _ => {
                return Err(ApiError::bad_request(
                    "a circle needs lat, lon and radius_m",
                ))
            }
        }
    } else if let Some(first) = body
        .get("polygon")
        .and_then(|p| p.as_array())
        .and_then(|p| p.first())
        .and_then(|c| c.as_array())
    {
        match (
            first.first().and_then(serde_json::Value::as_f64),
            first.get(1).and_then(serde_json::Value::as_f64),
        ) {
            (Some(lon), Some(lat)) => (lon, lat),
            _ => {
                return Err(ApiError::bad_request(
                    "polygon corners are [lon, lat] pairs",
                ))
            }
        }
    } else {
        return Err(ApiError::bad_request(
            "a geofence needs a circle or a polygon",
        ));
    };
    let cell = cell_near(&view, lon, lat)?;
    let mut payload = body;
    payload["source"] = "dashboard".into();
    let verb = sim_core::IntentVerb::Custom(GEOFENCE_OPEN);
    submit_intent(&state, Intent::operator(cell, verb, payload)).await
}

#[derive(Deserialize)]
pub struct ExtendBody {
    /// Minutes from now it stays open, or -1 until closed.
    pub minutes: f64,
}

/// Change how long an open geofence stays open: `{"minutes": m}`, `m`
/// minutes from now, or -1 until it is closed. From the next tick.
pub async fn geofences_extend(
    State(state): State<AppState>,
    Path(id): Path<u64>,
    Json(body): Json<ExtendBody>,
) -> ApiResult<Json<IntentAccepted>> {
    let view = state.driver.view();
    let fence = view.overlays["geofences"]["fences"]
        .as_array()
        .and_then(|list| list.iter().find(|f| f["id"].as_u64() == Some(id)))
        .ok_or_else(|| ApiError::not_found(format!("no open geofence {id}")))?;
    let cell = overlay_cell(&view, fence)?;
    let verb = sim_core::IntentVerb::Custom(GEOFENCE_EXTEND);
    let payload = serde_json::json!({ "id": id, "minutes": body.minutes });
    submit_intent(&state, Intent::operator(cell, verb, payload)).await
}

#[derive(Deserialize)]
pub struct RenameBody {
    /// The new name; empty clears it.
    pub name: String,
}

/// Rename a geofence, open, waiting to open or previously closed:
/// `{"name": "..."}`. From the next tick.
pub async fn geofences_rename(
    State(state): State<AppState>,
    Path(id): Path<u64>,
    Json(body): Json<RenameBody>,
) -> ApiResult<Json<IntentAccepted>> {
    let view = state.driver.view();
    let g = &view.overlays["geofences"];
    let fence = ["fences", "scheduled", "previous"]
        .iter()
        .filter_map(|k| g[*k].as_array())
        .flatten()
        .find(|f| f["id"].as_u64() == Some(id))
        .ok_or_else(|| ApiError::not_found(format!("no geofence {id}")))?;
    let cell = overlay_cell(&view, fence)?;
    let verb = sim_core::IntentVerb::Custom(GEOFENCE_RENAME);
    let payload = serde_json::json!({ "id": id, "name": body.name });
    submit_intent(&state, Intent::operator(cell, verb, payload)).await
}

/// A cell a published fence, station or region of interest touches (its
/// `lat_e7`/`lon_e7`, or a polygon's first corner), to address an intent
/// about it to.
fn overlay_cell(view: &Published, item: &serde_json::Value) -> ApiResult<GeoCell> {
    let (lat_e7, lon_e7) = match (item["lat_e7"].as_i64(), item["lon_e7"].as_i64()) {
        (Some(lat), Some(lon)) => (lat, lon),
        _ => {
            let first = &item["points"][0];
            (
                first[0].as_i64().unwrap_or(0),
                first[1].as_i64().unwrap_or(0),
            )
        }
    };
    cell_near(view, lon_e7 as f64 * 1e-7, lat_e7 as f64 * 1e-7)
}

/// Close a geofence, whoever opened it, or cancel one waiting to open. It
/// closes on the next tick.
pub async fn geofences_close(
    State(state): State<AppState>,
    Path(id): Path<u64>,
) -> ApiResult<Json<IntentAccepted>> {
    let view = state.driver.view();
    let g = &view.overlays["geofences"];
    let fence = ["fences", "scheduled"]
        .iter()
        .filter_map(|k| g[*k].as_array())
        .flatten()
        .find(|f| f["id"].as_u64() == Some(id))
        .ok_or_else(|| ApiError::not_found(format!("no open or scheduled geofence {id}")))?;
    let cell = overlay_cell(&view, fence)?;
    let verb = sim_core::IntentVerb::Custom(GEOFENCE_CLOSE);
    let payload = serde_json::json!({ "id": id });
    submit_intent(&state, Intent::operator(cell, verb, payload)).await
}

// ------------------------------------ stations and regions of interest

/// `Custom` intent codes for launch and recovery stations and regions of
/// interest; see `sim_drones::codes`.
const STATION_ADD: u16 = 120;
const STATION_REMOVE: u16 = 121;
const ROI_ADD: u16 = 130;
const ROI_REMOVE: u16 = 131;

/// How big a region of interest may be, as a radius in metres.
const ROI_RADIUS_M: std::ops::RangeInclusive<f64> = 10.0..=5000.0;

/// The longest name a station or region may have, in characters.
const MAX_NAME_CHARS: usize = 80;

#[derive(Serialize)]
pub struct StationsBody {
    pub tick: u64,
    /// Simulated seconds per tick, when the model says.
    pub tick_seconds: Option<f64>,
    pub mode: String,
    /// Whether the model publishes stations at all.
    pub available: bool,
    /// The launch and recovery stations, as the model publishes them, each
    /// with its `slots`, `occupied` (slots taken), `waiting` (drones holding
    /// off for one) and `drones` (on the ground at it now).
    pub stations: Vec<serde_json::Value>,
    /// How far each hub's couriers can deliver, by way of the stations, as
    /// the model publishes it (null without one).
    pub range: serde_json::Value,
    /// The latest station reports (added, removed), newest first.
    pub recent: Vec<Report>,
    /// Intents the model refused lately, as in [`GeofencesBody`].
    pub refused: Vec<Refused>,
}

#[derive(Serialize)]
pub struct RoisBody {
    pub tick: u64,
    /// Simulated seconds per tick, when the model says.
    pub tick_seconds: Option<f64>,
    pub mode: String,
    /// Whether the model publishes regions of interest at all.
    pub available: bool,
    /// The regions of interest, as the model publishes them, with the
    /// surveys done, when last, and the drones `assigned` to survey them.
    pub rois: Vec<serde_json::Value>,
    /// The latest region (designated, removed, added at a crash) and survey
    /// reports, newest first.
    pub recent: Vec<Report>,
    /// Intents the model refused lately, as in [`GeofencesBody`].
    pub refused: Vec<Refused>,
}

/// The launch and recovery stations, and what has happened to them lately.
/// Empty, and not `available`, for a model without them.
pub async fn stations_list(State(state): State<AppState>) -> Json<StationsBody> {
    let view = state.driver.view();
    let list = view.overlays["stations"].as_array();
    Json(StationsBody {
        tick: view.meta.tick,
        tick_seconds: tick_seconds(&view),
        mode: mode_of(&view),
        available: list.is_some(),
        stations: list.cloned().unwrap_or_default(),
        range: view.overlays["range"].clone(),
        recent: recent_reports(&view, &[ReportKind::Station]),
        refused: refused(&view),
    })
}

/// The regions of interest, and the latest region and survey reports.
/// Empty, and not `available`, for a model without them.
pub async fn rois_list(State(state): State<AppState>) -> Json<RoisBody> {
    let view = state.driver.view();
    let list = view.overlays["rois"].as_array();
    Json(RoisBody {
        tick: view.meta.tick,
        tick_seconds: tick_seconds(&view),
        mode: mode_of(&view),
        available: list.is_some(),
        rois: list.cloned().unwrap_or_default(),
        recent: recent_reports(&view, &[ReportKind::Roi, ReportKind::Survey]),
        refused: refused(&view),
    })
}

/// Where a new station or region goes and what it is called, from
/// `{"lat", "lon", "name"}` in degrees: a name is needed, of at most
/// `MAX_NAME_CHARS` once trimmed.
fn place(body: &serde_json::Value, what: &str) -> ApiResult<(f64, f64, String)> {
    let num = |k: &str| body.get(k).and_then(serde_json::Value::as_f64);
    let (Some(lat), Some(lon)) = (num("lat"), num("lon")) else {
        return Err(ApiError::bad_request(format!(
            "a {what} needs lat and lon, in degrees"
        )));
    };
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return Err(ApiError::bad_request(
            "lat is from -90 to 90 degrees and lon from -180 to 180",
        ));
    }
    let name = match body.get("name") {
        Some(serde_json::Value::String(name)) => name.trim().to_owned(),
        None | Some(serde_json::Value::Null) => String::new(),
        Some(_) => return Err(ApiError::bad_request("name is a string")),
    };
    if name.is_empty() {
        return Err(ApiError::bad_request(format!("a {what} needs a name")));
    }
    if name.chars().count() > MAX_NAME_CHARS {
        return Err(ApiError::bad_request(format!(
            "a name is at most {MAX_NAME_CHARS} characters"
        )));
    }
    Ok((lat, lon, name))
}

/// The `STATION_ADD` payload for a body of `{"lat", "lon", "name",
/// "slots"}` (1 to 100 launch and recovery slots), and where it is.
fn station_payload(body: &serde_json::Value) -> ApiResult<(f64, f64, serde_json::Value)> {
    let (lat, lon, name) = place(body, "station")?;
    let slots = body
        .get("slots")
        .and_then(serde_json::Value::as_u64)
        .filter(|n| (1..=100).contains(n))
        .ok_or_else(|| {
            ApiError::bad_request("a station needs slots: 1 to 100 launch and recovery slots")
        })?;
    let payload = serde_json::json!({
        "lat": lat, "lon": lon, "name": name, "slots": slots, "source": "dashboard",
    });
    Ok((lat, lon, payload))
}

/// The `ROI_ADD` payload for a body of `{"lat", "lon", "radius_m",
/// "name"}`, and where it is.
fn roi_payload(body: &serde_json::Value) -> ApiResult<(f64, f64, serde_json::Value)> {
    let (lat, lon, name) = place(body, "region of interest")?;
    let radius = body
        .get("radius_m")
        .and_then(serde_json::Value::as_f64)
        .filter(|r| ROI_RADIUS_M.contains(r))
        .ok_or_else(|| {
            ApiError::bad_request(format!(
                "a region of interest needs radius_m, from {} to {} metres",
                ROI_RADIUS_M.start(),
                ROI_RADIUS_M.end()
            ))
        })?;
    let payload = serde_json::json!({
        "lat": lat, "lon": lon, "radius_m": radius, "name": name, "source": "dashboard",
    });
    Ok((lat, lon, payload))
}

/// One item of a published list (`overlays[key]`) by its id.
fn listed<'a>(view: &'a Published, key: &str, id: u64) -> Option<&'a serde_json::Value> {
    view.overlays[key]
        .as_array()?
        .iter()
        .find(|item| item["id"].as_u64() == Some(id))
}

/// Add a launch and recovery station: `{"lat", "lon", "name"}`, in
/// degrees. From the next tick.
pub async fn stations_add(
    State(state): State<AppState>,
    Json(body): Json<serde_json::Value>,
) -> ApiResult<Json<IntentAccepted>> {
    let view = state.driver.view();
    let (lat, lon, payload) = station_payload(&body)?;
    let cell = cell_near(&view, lon, lat)?;
    let verb = sim_core::IntentVerb::Custom(STATION_ADD);
    submit_intent(&state, Intent::operator(cell, verb, payload)).await
}

/// Remove a launch and recovery station, whoever added it. From the next
/// tick.
pub async fn stations_remove(
    State(state): State<AppState>,
    Path(id): Path<u64>,
) -> ApiResult<Json<IntentAccepted>> {
    let view = state.driver.view();
    let station = listed(&view, "stations", id)
        .ok_or_else(|| ApiError::not_found(format!("no station {id}")))?;
    let cell = overlay_cell(&view, station)?;
    let verb = sim_core::IntentVerb::Custom(STATION_REMOVE);
    let payload = serde_json::json!({ "id": id });
    submit_intent(&state, Intent::operator(cell, verb, payload)).await
}

/// Designate a region of interest for the drones to survey: `{"lat",
/// "lon", "radius_m", "name"}`, degrees and metres (10 to 5000). From the
/// next tick.
pub async fn rois_add(
    State(state): State<AppState>,
    Json(body): Json<serde_json::Value>,
) -> ApiResult<Json<IntentAccepted>> {
    let view = state.driver.view();
    let (lat, lon, payload) = roi_payload(&body)?;
    let cell = cell_near(&view, lon, lat)?;
    let verb = sim_core::IntentVerb::Custom(ROI_ADD);
    submit_intent(&state, Intent::operator(cell, verb, payload)).await
}

/// Remove a region of interest, whoever designated it (a crash's
/// included). From the next tick.
pub async fn rois_remove(
    State(state): State<AppState>,
    Path(id): Path<u64>,
) -> ApiResult<Json<IntentAccepted>> {
    let view = state.driver.view();
    let roi = listed(&view, "rois", id)
        .ok_or_else(|| ApiError::not_found(format!("no region of interest {id}")))?;
    let cell = overlay_cell(&view, roi)?;
    let verb = sim_core::IntentVerb::Custom(ROI_REMOVE);
    let payload = serde_json::json!({ "id": id });
    submit_intent(&state, Intent::operator(cell, verb, payload)).await
}

#[derive(Deserialize)]
pub struct EntitiesQuery {
    /// Comma-separated kinds, e.g. `drone,poi`. Omit for every kind that has
    /// a position.
    #[serde(default)]
    pub kind: Option<String>,
}

#[derive(Serialize)]
pub struct Entity {
    pub id: u64,
    pub kind: String,
    pub cell: GeoCell,
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    pub heading_deg: f64,
    pub speed_ms: f64,
    /// Through the air, when it flies in wind; absent in still air, where it
    /// is `speed_ms`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub airspeed_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub callsign: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mode: Option<String>,
    /// What a drone's behaviour is doing ("delivering", "surveying",
    /// "charging", ...), when the model says.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub activity: Option<String>,
}

/// Every object that carries an exact position (`lat_e7`/`lon_e7` in its
/// state), for drawing individual things on the map rather than cell counts.
/// The body is the entities; headers say which tick they are from
/// (`x-sim-tick`), how long a tick is in simulated seconds
/// (`x-sim-tick-seconds`, when the model says) and whether the run is
/// advancing (`x-sim-mode`), so a client can predict motion between polls.
pub async fn state_entities(
    State(state): State<AppState>,
    Query(query): Query<EntitiesQuery>,
) -> (axum::http::HeaderMap, Json<Vec<Entity>>) {
    let view = state.driver.view();
    let mut headers = axum::http::HeaderMap::new();
    let mut header = |name: &'static str, value: String| {
        if let Ok(value) = axum::http::HeaderValue::from_str(&value) {
            headers.insert(name, value);
        }
    };
    header("x-sim-tick", view.meta.tick.to_string());
    header(
        "x-sim-mode",
        format!("{:?}", view.health.mode).to_lowercase(),
    );
    if let Some(seconds) = tick_seconds(&view) {
        header("x-sim-tick-seconds", seconds.to_string());
    }
    let kinds: Option<Vec<&str>> = query.kind.as_deref().map(|k| {
        k.split(',')
            .map(str::trim)
            .filter(|k| !k.is_empty())
            .collect()
    });
    let int = |v: &serde_json::Value, key: &str| v.get(key).and_then(|x| x.as_i64());
    let mut out = Vec::new();
    for bag in &view.cells {
        for object in &bag.objects {
            if kinds
                .as_ref()
                .is_some_and(|k| !k.contains(&object.kind.as_str()))
            {
                continue;
            }
            let s = &object.state;
            // A manned aircraft between flights is parked out of sight.
            if s.pointer("/flight/active") == Some(&serde_json::Value::Bool(false)) {
                continue;
            }
            let (Some(lat), Some(lon)) = (int(s, "lat_e7"), int(s, "lon_e7")) else {
                continue;
            };
            out.push(Entity {
                id: object.id,
                kind: object.kind.clone(),
                cell: bag.cell,
                lat: lat as f64 * 1e-7,
                lon: lon as f64 * 1e-7,
                alt_m: int(s, "alt_mm").unwrap_or(0) as f64 * 1e-3,
                heading_deg: int(s, "hdg_cdeg").unwrap_or(0) as f64 / 100.0,
                speed_ms: int(s, "spd_cms").unwrap_or(0) as f64 / 100.0,
                airspeed_ms: int(s, "air_cms").map(|v| v as f64 / 100.0),
                callsign: s
                    .get("callsign")
                    .and_then(|c| c.as_str())
                    .map(str::to_owned),
                mode: s
                    .get("drone")
                    .and_then(|d| d.get("mode"))
                    .and_then(|m| m.as_str())
                    .map(str::to_owned),
                activity: s
                    .get("activity")
                    .and_then(|a| a.as_str())
                    .map(str::to_owned),
            });
        }
    }
    (headers, Json(out))
}

#[derive(Deserialize)]
pub struct ReportsQuery {
    /// Only reports with a larger `seq`. Omit for everything still held.
    #[serde(default)]
    pub since: u64,
}

#[derive(Serialize)]
pub struct ReportsBody {
    pub tick: u64,
    /// The highest `seq` issued so far; pass it back as `since` next time.
    pub last_seq: u64,
    pub reports: Vec<Report>,
}

/// Model events (sightings, waypoints reached) newer than `since`.
///
/// The driver keeps the most recent few hundred. A caller that falls further
/// behind sees a jump between its `since` and the first `seq` returned.
pub async fn reports(
    State(state): State<AppState>,
    Query(query): Query<ReportsQuery>,
) -> Json<ReportsBody> {
    let view = state.driver.view();
    Json(ReportsBody {
        tick: view.meta.tick,
        last_seq: view.reports.last().map_or(query.since, |r| r.seq),
        reports: view.reports_since(query.since).to_vec(),
    })
}

/// The replay log, available when the run is in the `gold` profile (spec §8.4).
pub async fn intents_log(State(state): State<AppState>) -> ApiResult<Response> {
    if state.driver.view().config.run.profile != Profile::Gold {
        return Err(ApiError::new(
            ErrorCode::BadRequest,
            "the intent log is only kept in the gold profile",
        ));
    }
    let driver = state.driver.clone();
    let snapshot = tokio::task::spawn_blocking(move || driver.checkpoint())
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, err.to_string()))?
        .map_err(from_driver)?;

    let log = snapshot.intent_log.unwrap_or_default();
    // JSON Lines: an append-only log should stream and diff line by line.
    let mut body = String::new();
    for entry in &log {
        body.push_str(&serde_json::to_string(entry).unwrap_or_default());
        body.push('\n');
    }
    let filename = format!(
        "{}-tick{}.intents.jsonl",
        snapshot.meta.scenario_id, snapshot.meta.tick
    );
    Ok((
        [
            (header::CONTENT_TYPE, "application/jsonl".to_owned()),
            (
                header::CONTENT_DISPOSITION,
                format!("attachment; filename=\"{filename}\""),
            ),
        ],
        body,
    )
        .into_response())
}

// --------------------------------------------------------------- metrics

/// Prometheus text exposition (spec §9.2).
pub async fn metrics(State(state): State<AppState>) -> Response {
    let view = state.driver.view();
    let m = &view.metrics;
    let health = state.driver.health();

    let mut out = String::with_capacity(2048);
    let mut gauge = |name: &str, help: &str, value: f64| {
        out.push_str(&format!("# HELP krabshell_{name} {help}\n"));
        out.push_str(&format!("# TYPE krabshell_{name} gauge\n"));
        out.push_str(&format!("krabshell_{name} {value}\n"));
    };

    gauge("tick", "Committed tick.", m.tick as f64);
    gauge("agents", "Agents in the world.", m.agent_count as f64);
    gauge(
        "occupied_cells",
        "Occupied A5 cells.",
        m.occupied_cells as f64,
    );
    gauge("sim_tps", "Simulation ticks per second.", m.sim_tps);
    gauge(
        "publish_hz",
        "Snapshots published per second.",
        m.publish_hz,
    );
    gauge(
        "last_step_ns",
        "Duration of the last step in nanoseconds.",
        m.last_step_ns as f64,
    );
    gauge(
        "intent_queue_depth",
        "Intents waiting to be applied.",
        m.intent_queue_depth as f64,
    );
    gauge(
        "intents_applied_total",
        "Intents applied since load.",
        m.intents_applied as f64,
    );
    gauge(
        "intents_rejected_total",
        "Intents rejected since load.",
        m.intents_rejected as f64,
    );
    gauge(
        "dropped_views_total",
        "Published snapshots no reader took.",
        m.dropped_views as f64,
    );
    gauge(
        "truncated_cells",
        "Cells omitted from the published view.",
        m.truncated_cells as f64,
    );
    gauge(
        "driver_alive",
        "1 when the tick thread is running.",
        health.alive as u8 as f64,
    );
    gauge(
        "driver_faulted",
        "1 when the model has panicked.",
        (health.mode == RunMode::Faulted) as u8 as f64,
    );
    gauge(
        "uptime_seconds",
        "Process uptime.",
        state.started_at.elapsed().as_secs_f64(),
    );

    ([(header::CONTENT_TYPE, "text/plain; version=0.0.4")], out).into_response()
}

// ----------------------------------------------------------------- cells

#[derive(Deserialize)]
pub struct LookupQuery {
    pub lon: f64,
    pub lat: f64,
    #[serde(default)]
    pub res: Option<i32>,
}

#[derive(Serialize)]
pub struct LookupBody {
    pub cell: GeoCell,
    pub resolution: i32,
    pub centroid: [f64; 2],
    pub boundary: Vec<[f64; 2]>,
    pub parent: Option<GeoCell>,
    /// Whether this cell is addressable in the current run.
    pub in_world: bool,
    pub edge_meters: f64,
}

/// Compile a coordinate to a cell. The dashboard uses it when an operator
/// clicks the map (spec §7.2).
pub async fn cells_lookup(
    State(state): State<AppState>,
    Query(query): Query<LookupQuery>,
) -> ApiResult<Json<LookupBody>> {
    let view = state.driver.view();
    let resolution = query.res.unwrap_or(view.meta.resolution);
    let cell = GeoCell::from_lonlat(query.lon, query.lat, resolution).map_err(ApiError::from)?;
    let (lon, lat) = cell.centroid().map_err(ApiError::from)?;

    let in_world = resolution == view.meta.resolution
        && view.meta.root.is_none_or(|root| cell.is_within(root));

    Ok(Json(LookupBody {
        cell,
        resolution,
        centroid: [lon, lat],
        boundary: cell
            .boundary()
            .unwrap_or_default()
            .into_iter()
            .map(|(lon, lat)| [lon, lat])
            .collect(),
        parent: cell.parent(None).ok(),
        in_world,
        edge_meters: cell::nominal_cell_edge_meters(resolution),
    }))
}

// ------------------------------------------------------------- conflicts

#[derive(Deserialize)]
pub struct ConflictsQuery {
    /// Leave out the cells' outlines.
    #[serde(default)]
    pub outlines: Option<bool>,
}

/// Where the aircraft in the air could meet in the next minute, and what
/// each is asked to do about it. Worked out for the current tick on the
/// first ask, then reused until the tick moves.
pub async fn conflicts(
    State(state): State<AppState>,
    Query(q): Query<ConflictsQuery>,
) -> ApiResult<Json<crate::conflicts::ConflictReport>> {
    let report = state
        .conflicts
        .report(&state.driver, &state.weather)
        .await
        .map_err(|err| ApiError::new(ErrorCode::Internal, format!("{err:#}")))?;
    let mut report = (*report).clone();
    if q.outlines == Some(false) {
        for c in report.cells.iter_mut() {
            c.boundary.clear();
        }
        for c in report.heat.iter_mut() {
            c.boundary.clear();
        }
    }
    Ok(Json(report))
}

#[derive(Serialize)]
pub struct ConflictSettings {
    pub resolution: i32,
    /// `operator`, `pack` or `world`.
    pub from: &'static str,
    /// The world's occupancy resolution.
    pub world_resolution: i32,
    /// What may be chosen.
    pub min_resolution: i32,
    pub max_resolution: i32,
    /// Nominal edge of a cell at the resolution in use, metres.
    pub edge_m: f64,
}

pub fn conflict_settings_of(state: &AppState) -> ConflictSettings {
    let view = state.driver.view();
    let r = state.conflicts.resolution(&view);
    ConflictSettings {
        resolution: r.resolution,
        from: r.from,
        world_resolution: view.meta.resolution,
        min_resolution: *crate::conflicts::RESOLUTIONS.start(),
        max_resolution: *crate::conflicts::RESOLUTIONS.end(),
        edge_m: cell::nominal_cell_edge_meters(r.resolution).round(),
    }
}

/// The cell resolution conflicts are assessed at.
pub async fn conflict_settings(State(state): State<AppState>) -> Json<ConflictSettings> {
    Json(conflict_settings_of(&state))
}

#[derive(Deserialize)]
pub struct ConflictSettingsBody {
    /// A resolution, or null for the pack's (or else the world's).
    pub resolution: Option<i32>,
}

/// Choose the cell resolution conflicts are assessed at.
pub async fn conflict_settings_put(
    State(state): State<AppState>,
    Json(body): Json<ConflictSettingsBody>,
) -> ApiResult<Json<ConflictSettings>> {
    state
        .conflicts
        .choose_resolution(body.resolution)
        .map_err(ApiError::bad_request)?;
    Ok(Json(conflict_settings_of(&state)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn a_new_station_goes_to_the_model_as_the_dashboard_s() {
        let body =
            json!({ "lat": 54.93, "lon": 23.85, "name": "  Šilainiai ", "slots": 4, "x": 1 });
        let (lat, lon, payload) = station_payload(&body).unwrap();
        assert_eq!((lat, lon), (54.93, 23.85));
        assert_eq!(
            payload,
            json!({ "lat": 54.93, "lon": 23.85, "name": "Šilainiai", "slots": 4, "source": "dashboard" })
        );
    }

    #[test]
    fn a_new_region_carries_its_radius() {
        let body = json!({ "lat": 54.93, "lon": 23.85, "radius_m": 150, "name": "Park" });
        let (_, _, payload) = roi_payload(&body).unwrap();
        assert_eq!(payload["radius_m"], 150.0);
        assert_eq!(payload["name"], "Park");
        assert_eq!(payload["source"], "dashboard");
        for radius in [json!(10), json!(5000)] {
            let mut body = body.clone();
            body["radius_m"] = radius;
            assert!(roi_payload(&body).is_ok());
        }
    }

    #[test]
    fn places_without_a_position_name_or_sensible_radius_are_refused() {
        for body in [
            json!({ "lat": 54.93, "name": "x" }),
            json!({ "lat": "54.93", "lon": 23.85, "name": "x" }),
            json!({ "lat": 91, "lon": 23.85, "name": "x" }),
            json!({ "lat": 54.93, "lon": -181, "name": "x" }),
            json!({ "lat": 54.93, "lon": 23.85 }),
            json!({ "lat": 54.93, "lon": 23.85, "name": "   " }),
            json!({ "lat": 54.93, "lon": 23.85, "name": 7 }),
            json!({ "lat": 54.93, "lon": 23.85, "name": "n".repeat(81) }),
            json!({ "lat": 54.93, "lon": 23.85, "name": "x" }),
            json!({ "lat": 54.93, "lon": 23.85, "name": "x", "slots": 0 }),
            json!({ "lat": 54.93, "lon": 23.85, "name": "x", "slots": 101 }),
            json!({ "lat": 54.93, "lon": 23.85, "name": "x", "slots": 2.5 }),
        ] {
            assert!(station_payload(&body).is_err(), "{body}");
        }
        for radius in [
            json!(9.9),
            json!(5000.5),
            json!(-1),
            json!("big"),
            json!(null),
        ] {
            let body = json!({ "lat": 54.93, "lon": 23.85, "name": "x", "radius_m": radius });
            assert!(roi_payload(&body).is_err(), "{body}");
        }
    }

    #[test]
    fn a_fence_band_needs_its_ceiling_above_its_floor() {
        for ok in [
            json!({}),
            json!({ "floor_m": null, "ceiling_m": null }),
            json!({ "floor_m": 0 }),
            json!({ "floor_m": 60 }),
            json!({ "ceiling_m": 120 }),
            json!({ "floor_m": 30, "ceiling_m": 30.5 }),
        ] {
            assert!(check_band(&ok).is_ok(), "{ok}");
        }
        for bad in [
            json!({ "floor_m": -1 }),
            json!({ "floor_m": "60" }),
            json!({ "ceiling_m": 0 }),
            json!({ "floor_m": 60, "ceiling_m": 60 }),
            json!({ "floor_m": 60, "ceiling_m": 10 }),
        ] {
            assert!(check_band(&bad).is_err(), "{bad}");
        }
    }
}
