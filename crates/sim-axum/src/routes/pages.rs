//! The operator dashboard (spec §11).
//!
//! Askama templates plus htmx: the server renders every screen, and htmx swaps
//! small fragments in place. That keeps control working when JavaScript is
//! unhappy — start, pause, step and reset are plain form posts, so the buttons
//! do not depend on the map, the WebSocket, or anything else loading first
//! (spec §11.3).
//!
//! Two things do use JavaScript, and both degrade to nothing: the canvas map,
//! and the live cell view over the WebSocket.

use std::sync::Arc;

use askama::Template;
use axum::extract::{Path, Query, State};
use axum::http::{header, StatusCode};
use axum::response::{Html, IntoResponse, Redirect, Response};
use axum::routing::{get, post};
use axum::{Form, Router};
use serde::Deserialize;
use sim_core::config::{Config, Param};
use sim_core::{GeoCell, Intent, IntentVerb, Profile, RunMode};
use sim_engine::{Control, Published, Submission};

use crate::auth::Auth;
use crate::AppState;

/// Assets are compiled into the binary: one file to deploy, and the dashboard
/// cannot drift from the server that serves it.
const APP_CSS: &str = include_str!("../../../sim-dashboard/assets/app.css");
const MAP_JS: &str = include_str!("../../../sim-dashboard/assets/map.js");
const GEOFENCES_JS: &str = include_str!("../../../sim-dashboard/assets/geofences.js");
const STATIONS_JS: &str = include_str!("../../../sim-dashboard/assets/stations.js");
const LIVE_JS: &str = include_str!("../../../sim-dashboard/assets/live.js");
const WEATHER_JS: &str = include_str!("../../../sim-dashboard/assets/weather.js");
const CONFLICTS_JS: &str = include_str!("../../../sim-dashboard/assets/conflicts.js");
const HTMX_JS: &str = include_str!("../../../sim-dashboard/assets/htmx.min.js");
/// milsymbol 3.0.4 (MIT), which draws the maps' APP-6 / MIL-STD-2525 symbols.
/// Vendored so the dashboard works offline. Its path carries the version, so
/// browsers may keep it for good; a new version gets a new path.
const MILSYMBOL_JS: &str = include_str!("../../../sim-dashboard/assets/milsymbol.js");

pub fn router(auth: Arc<Auth>) -> Router<AppState> {
    let public = Router::new()
        .route("/login", get(login_form).post(login_submit))
        .route("/logout", post(logout))
        .route("/docs", get(docs_page))
        .route("/assets/app.css", get(|| asset(APP_CSS, "text/css")))
        .route("/assets/map.js", get(|| asset(MAP_JS, "text/javascript")))
        .route(
            "/assets/geofences.js",
            get(|| asset(GEOFENCES_JS, "text/javascript")),
        )
        .route(
            "/assets/stations.js",
            get(|| asset(STATIONS_JS, "text/javascript")),
        )
        .route("/assets/live.js", get(|| asset(LIVE_JS, "text/javascript")))
        .route(
            "/assets/weather.js",
            get(|| asset(WEATHER_JS, "text/javascript")),
        )
        .route(
            "/assets/conflicts.js",
            get(|| asset(CONFLICTS_JS, "text/javascript")),
        )
        .route(
            "/assets/htmx.min.js",
            get(|| asset(HTMX_JS, "text/javascript")),
        )
        .route(
            "/assets/milsymbol-3.0.4.js",
            get(|| async {
                (
                    [
                        (header::CONTENT_TYPE, "text/javascript"),
                        (header::CACHE_CONTROL, "public, max-age=31536000, immutable"),
                    ],
                    MILSYMBOL_JS,
                )
                    .into_response()
            }),
        );

    let guarded = Router::new()
        .route("/", get(|| async { Redirect::to("/run") }))
        .route("/run", get(run_page))
        .route("/scenarios", get(scenarios_page))
        .route("/state", get(state_page))
        .route("/geofences", get(geofences_page))
        .route("/stations", get(stations_page))
        .route("/intents", get(intents_page))
        .route("/io", get(io_page))
        .route("/weather", get(weather_page))
        .route("/conflicts", get(conflicts_page))
        .route("/fragments/conflicts", get(conflicts_fragment))
        .route("/fragments/weather", get(weather_fragment))
        .route("/actions/weather/refresh", post(action_weather_refresh))
        .route("/fragments/control", get(control_fragment))
        .route("/fragments/scenarios", get(scenarios_fragment))
        .route("/fragments/cell", get(cell_fragment))
        .route("/fragments/intents", get(intents_fragment))
        .route("/actions/start", post(action_start))
        .route("/actions/speed", post(action_speed))
        .route("/actions/pause", post(action_pause))
        .route("/actions/step", post(action_step))
        .route("/actions/reset", post(action_reset))
        .route("/actions/load/{id}", post(action_load))
        .route("/actions/intent", post(action_intent))
        .route_layer(axum::middleware::from_fn_with_state(
            auth,
            crate::auth::require_page_token,
        ));

    public.merge(guarded)
}

async fn asset(body: &'static str, content_type: &'static str) -> Response {
    (
        [
            (header::CONTENT_TYPE, content_type),
            // Assets change only when the binary does, but a stale dashboard is
            // confusing enough that a short cache is the right trade.
            (header::CACHE_CONTROL, "public, max-age=60"),
        ],
        body,
    )
        .into_response()
}

// ------------------------------------------------------------ view models

/// A short message shown after an action.
#[derive(Clone)]
pub struct Flash {
    pub kind: &'static str,
    pub message: String,
}

impl Flash {
    fn ok(message: impl Into<String>) -> Self {
        Flash {
            kind: "ok",
            message: message.into(),
        }
    }
    fn bad(message: impl Into<String>) -> Self {
        Flash {
            kind: "bad",
            message: message.into(),
        }
    }
    fn warn(message: impl Into<String>) -> Self {
        Flash {
            kind: "warn",
            message: message.into(),
        }
    }
}

/// Everything the control panel shows, pre-formatted so the template does no
/// arithmetic and no rounding decisions.
pub struct ControlView {
    pub mode: String,
    pub mode_class: &'static str,
    pub alive: bool,
    pub tick: String,
    /// Simulated time since tick 0, if the model says how long a tick is.
    pub sim_time: Option<String>,
    /// Simulated time per tick.
    pub tick_length: Option<String>,
    pub agents: String,
    pub cells: String,
    pub sim_tps: String,
    pub publish_hz: String,
    pub step_time: String,
    pub queue_depth: String,
    pub applied: String,
    pub rejected: String,
    pub dropped_views: String,
    pub scenario_id: String,
    pub scenario_name: String,
    pub model: String,
    pub engine: String,
    pub profile: String,
    pub parallel: String,
    pub rate: String,
    /// Which speed choice is selected: `max`, `realtime` or `custom`.
    pub speed: &'static str,
    /// The real-time rate, if the scenario says how long a tick is.
    pub realtime_tps: Option<String>,
    /// The custom rate box's value: the current rate when it is custom.
    pub custom_tps: String,
    pub resolution: String,
    pub view_resolution: String,
    pub root: String,
    pub truncated: bool,
    pub fault: Option<String>,
    pub last_good_tick: String,
    pub flash: Option<Flash>,
}

impl ControlView {
    fn build(state: &AppState, flash: Option<Flash>) -> Self {
        let view = state.driver.view();
        let health = state.driver.health();
        let m = &view.metrics;
        let mode = health.mode;

        ControlView {
            mode: format!("{mode:?}").to_lowercase(),
            mode_class: match mode {
                RunMode::Running => "pill running",
                RunMode::Stepping => "pill stepping",
                RunMode::Paused => "pill paused",
                RunMode::Loading => "pill loading",
                RunMode::Faulted => "pill faulted",
            },
            alive: health.alive,
            tick: thousands(view.meta.tick),
            sim_time: tick_seconds(&view.config)
                .map(|dt| format_sim_time(view.meta.tick as f64 * dt)),
            tick_length: tick_seconds(&view.config).map(|dt| format!("{} s", format_tps(dt))),
            agents: thousands(view.meta.agent_count),
            cells: thousands(view.meta.occupied_cells),
            sim_tps: format_rate(m.sim_tps),
            publish_hz: format!("{:.1}", m.publish_hz),
            step_time: format_nanos(m.last_step_ns),
            queue_depth: thousands(m.intent_queue_depth),
            applied: thousands(m.intents_applied),
            rejected: thousands(m.intents_rejected),
            dropped_views: thousands(m.dropped_views),
            scenario_id: view.scenario_id.clone(),
            scenario_name: view.scenario_name.clone(),
            model: view.model_kind.clone(),
            engine: sim_engine::engine_description().to_owned(),
            profile: match view.config.run.profile {
                Profile::Gold => "gold".to_owned(),
                Profile::Fast => "fast".to_owned(),
            },
            parallel: if view.config.run.parallel {
                format!("on, {} threads", view.config.run.threads)
            } else {
                "off".to_owned()
            },
            rate: describe_rate(&view.config),
            speed: speed_choice(&view.config),
            realtime_tps: realtime_tps(&view.config).map(format_tps),
            custom_tps: match (speed_choice(&view.config), view.config.run.target_tps) {
                ("custom", Some(tps)) => format_tps(tps),
                _ => String::new(),
            },
            resolution: view.meta.resolution.to_string(),
            view_resolution: view.config.grid.view_resolution.to_string(),
            root: view
                .meta
                .root
                .map(|c| c.to_hex())
                .unwrap_or_else(|| "global".to_owned()),
            truncated: view.truncated,
            fault: health.fault.clone(),
            last_good_tick: thousands(health.last_good_tick),
            flash,
        }
    }
}

pub struct ScenarioRow {
    pub id: String,
    pub name: String,
    pub description: String,
    pub model: String,
    pub resolution: String,
    pub root: String,
    pub profile: String,
    pub seed_agents: String,
    pub is_current: bool,
    pub known_model: bool,
}

pub struct ObjectRow {
    pub id: String,
    pub kind: String,
    pub state: String,
    /// The occupancy cell the agent sits in. Only shown when the inspected
    /// cell is coarser than occupancy, where it is not the same for every row.
    pub cell: String,
}

pub struct CellView {
    pub query: String,
    pub cell: Option<String>,
    pub centroid: String,
    pub resolution: String,
    pub in_world: bool,
    /// The inspected cell is coarser than occupancy, so it is a roll-up.
    pub coarse: bool,
    pub agents: String,
    pub occupied: String,
    pub objects: Vec<ObjectRow>,
    /// Agents beyond [`MAX_CELL_ROWS`] that the table leaves out.
    pub hidden: String,
    pub error: Option<String>,
}

/// Rows the cell inspector renders. A resolution-0 click on a big world could
/// otherwise put every agent on the page.
const MAX_CELL_ROWS: usize = 500;

pub struct IntentRow {
    pub id: String,
    pub actor: String,
    pub verb: String,
    pub cell: String,
    pub tick: String,
    pub rejected: bool,
    pub detail: String,
}

// -------------------------------------------------------------- templates

#[derive(Template)]
#[template(path = "login.html")]
struct LoginTemplate {
    error: Option<String>,
    open: bool,
}

/// The weather status, as the Weather screen shows it.
pub struct WeatherView {
    pub available: bool,
    pub mode: String,
    pub live: bool,
    pub origin: String,
    pub source: String,
    pub attribution: String,
    pub start_utc: String,
    pub sim_utc: String,
    pub window: String,
    pub frames: usize,
    pub cells: String,
    pub levels: String,
    pub area: String,
    pub fetched: String,
    pub fetch: String,
    pub cache: String,
    pub in_sync: bool,
    pub error: Option<String>,
    pub limits: String,
    pub flash: Option<Flash>,
}

impl WeatherView {
    fn build(state: &AppState, flash: Option<Flash>) -> WeatherView {
        let s = crate::routes::weather::status_of(state);
        let dash = || "—".to_owned();
        let pretty = |utc: &Option<String>| {
            utc.as_deref()
                .map_or_else(dash, |t| t.replacen('T', " ", 1).replace('Z', " UTC"))
        };
        let origin = match s.origin {
            Some(crate::weather::Origin::OpenMeteo) => "Open-Meteo",
            Some(crate::weather::Origin::Pack) => "the pack's world/weather.json",
            Some(crate::weather::Origin::Checkpoint) => "a restored checkpoint",
            None => "—",
        };
        WeatherView {
            available: s.available,
            mode: match s.mode {
                Some(sim_core::WeatherMode::Live) => "live".into(),
                Some(sim_core::WeatherMode::Recorded) => "recorded".into(),
                None => "off".into(),
            },
            live: s.mode == Some(sim_core::WeatherMode::Live),
            origin: origin.into(),
            source: s.source.clone().unwrap_or_else(dash),
            attribution: s.attribution.clone().unwrap_or_default(),
            start_utc: pretty(&s.start_utc),
            sim_utc: pretty(&s.sim_utc),
            window: s.window.as_ref().map_or_else(dash, |w| {
                format!(
                    "{} to {}, {} frames {} min apart",
                    pretty(&Some(w.first_utc.clone())),
                    pretty(&Some(w.last_utc.clone())),
                    w.frames,
                    w.step_s / 60
                )
            }),
            frames: s.window.as_ref().map_or(0, |w| w.frames),
            cells: match s.resolution {
                Some(r) => format!("{} at resolution {r}", s.cells),
                None => dash(),
            },
            levels: if s.levels_m.is_empty() {
                dash()
            } else {
                s.levels_m
                    .iter()
                    .map(|m| format!("{m} m"))
                    .collect::<Vec<_>>()
                    .join(", ")
            },
            area: s.area.map_or_else(dash, |[w, so, e, n]| {
                format!("{so:.3}°N {w:.3}°E to {n:.3}°N {e:.3}°E")
            }),
            fetched: pretty(&s.fetched_utc),
            fetch: if s.origin == Some(crate::weather::Origin::OpenMeteo) {
                if s.cache_hit {
                    "from the cache: no Open-Meteo calls".into()
                } else {
                    format!("{} Open-Meteo locations", s.locations)
                }
            } else {
                dash()
            },
            cache: s.cache.as_ref().map_or_else(
                || "fetching is off (--no-weather)".into(),
                |c| {
                    format!(
                        "{} entries, {:.1} MB in {}",
                        c.entries,
                        c.bytes as f64 / 1e6,
                        c.dir
                    )
                },
            ),
            in_sync: s.in_sync,
            error: s.error.clone(),
            limits: format!(
                "wind {} m/s, gusts {} m/s at takeoff height",
                s.limits.max_wind_ms, s.limits.max_gust_ms
            ),
            flash,
        }
    }
}

/// A row of the resolution requests table.
pub struct RequestRow {
    pub callsign: String,
    pub action: String,
    pub against: String,
    pub in_s: String,
    pub risk: String,
    pub status: &'static str,
    pub status_kind: &'static str,
}

/// A row of the top cells table.
pub struct CellRow {
    pub rank: usize,
    pub cell: String,
    pub lon: f64,
    pub lat: f64,
    pub altitude: String,
    pub in_s: String,
    pub summed: String,
    pub collision: String,
    pub conflict: bool,
    pub aircraft: Vec<(String, String)>,
}

/// The Conflicts screen's tables.
pub struct ConflictsView {
    pub ready: bool,
    pub error: Option<String>,
    pub summary: String,
    pub requests: Vec<RequestRow>,
    pub cells: Vec<CellRow>,
    pub more_cells: usize,
}

fn action_text(m: sim_conflict::Manoeuvre, t: &sim_conflict::Targets) -> String {
    use sim_conflict::Manoeuvre as M;
    let ms = |v: f64| format!("{:.1} m/s", v.abs());
    match m {
        M::Maintain => "Maintain".into(),
        M::Climb => format!("Climb at {}", t.vs_ms.map_or_else(String::new, ms)),
        M::Descend => format!("Descend at {}", t.vs_ms.map_or_else(String::new, ms)),
        M::LevelOff => "Level off".into(),
        M::TurnLeft | M::TurnRight => format!(
            "Turn {} to {:03.0}°",
            if m == M::TurnLeft { "left" } else { "right" },
            t.heading_deg.unwrap_or(0.0)
        ),
        M::SpeedUp | M::SlowDown => format!(
            "{} to {}",
            if m == M::SpeedUp {
                "Speed up"
            } else {
                "Slow down"
            },
            t.tas_ms.map_or_else(String::new, ms)
        ),
    }
}

impl ConflictsView {
    async fn build(state: &AppState) -> ConflictsView {
        let report = match state.conflicts.report(&state.driver, &state.weather).await {
            Ok(r) => r,
            Err(err) => {
                return ConflictsView {
                    ready: false,
                    error: Some(format!("{err:#}")),
                    summary: String::new(),
                    requests: Vec::new(),
                    cells: Vec::new(),
                    more_cells: 0,
                }
            }
        };
        let secs = |t: f64| {
            if t.is_finite() {
                format!("{t:.0} s")
            } else {
                "—".into()
            }
        };
        let mut requests: Vec<RequestRow> = report
            .advisories
            .iter()
            .map(|a| RequestRow {
                callsign: a.callsign.clone(),
                action: action_text(a.manoeuvre, &a.targets),
                against: a
                    .against
                    .iter()
                    .map(|c| c.callsign.clone())
                    .collect::<Vec<_>>()
                    .join(", "),
                in_s: secs(a.first_conflict_s),
                risk: format!("{:.1} → {:.1}", a.risk_before, a.risk_after),
                status: if a.clear {
                    "clears it"
                } else {
                    "still in conflict"
                },
                status_kind: if a.clear { "ok" } else { "warn" },
            })
            .collect();
        requests.extend(report.unresolved.iter().map(|u| RequestRow {
            callsign: format!("{} and {}", u.a.callsign, u.b.callsign),
            action: "No advisory resolves it".into(),
            against: String::new(),
            in_s: secs(u.first_s),
            risk: format!("peak {:.2} aircraft", u.peak_total),
            status: "unresolved",
            status_kind: "bad",
        }));
        let cells: Vec<CellRow> = report
            .cells
            .iter()
            .take(10)
            .enumerate()
            .map(|(i, c)| {
                let (lon, lat) = c.cell.centroid().unwrap_or((0.0, 0.0));
                CellRow {
                    rank: i + 1,
                    cell: c.cell.to_hex(),
                    lon,
                    lat,
                    altitude: format!("{:.0}–{:.0} m", c.alt_from_m, c.alt_to_m),
                    in_s: secs(c.t_s),
                    summed: format!("{:.2}", c.summed),
                    collision: format!("{:.0}%", c.collision * 100.0),
                    conflict: c.conflict,
                    aircraft: c
                        .aircraft
                        .iter()
                        .map(|a| (a.callsign.clone(), format!("{:.0}%", a.probability * 100.0)))
                        .collect(),
                }
            })
            .collect();
        ConflictsView {
            ready: true,
            error: None,
            summary: format!(
                "Tick {} · {} aircraft in the air · {:.0} s ahead in {:.0} s steps · conflict at {:.1} aircraft expected in a cell · assessed in {:.0} ms",
                thousands(report.tick),
                report.aircraft,
                report.horizon_s,
                report.step_s,
                report.threshold,
                report.took_ms
            ),
            requests,
            more_cells: report.cells.len().saturating_sub(cells.len()),
            cells,
        }
    }
}

#[derive(Template)]
#[template(path = "conflicts.html")]
struct ConflictsTemplate {
    nav: &'static str,
    control: ControlView,
    conflicts: ConflictsView,
    basemaps: &'static [Basemap],
}

#[derive(Template)]
#[template(path = "fragments/conflicts.html")]
struct ConflictsFragment {
    conflicts: ConflictsView,
}

#[derive(Template)]
#[template(path = "weather.html")]
struct WeatherTemplate {
    nav: &'static str,
    control: ControlView,
    weather: WeatherView,
    basemaps: &'static [Basemap],
}

#[derive(Template)]
#[template(path = "fragments/weather.html")]
struct WeatherFragment {
    weather: WeatherView,
}

#[derive(Template)]
#[template(path = "docs.html")]
struct DocsTemplate {
    nav: &'static str,
    docs: &'static crate::openapi::DocsView,
}

#[derive(Template)]
#[template(path = "run.html")]
struct RunTemplate {
    nav: &'static str,
    control: ControlView,
}

#[derive(Template)]
#[template(path = "scenarios.html")]
struct ScenariosTemplate {
    nav: &'static str,
    control: ControlView,
    scenarios: Vec<ScenarioRow>,
    model_kinds: String,
}

#[derive(Template)]
#[template(path = "state.html")]
struct StateTemplate {
    nav: &'static str,
    control: ControlView,
    cell: CellView,
    view_resolution: String,
    resolution: String,
    basemaps: &'static [Basemap],
}

#[derive(Template)]
#[template(path = "geofences.html")]
struct GeofencesTemplate {
    nav: &'static str,
    control: ControlView,
    basemaps: &'static [Basemap],
}

#[derive(Template)]
#[template(path = "stations.html")]
struct StationsTemplate {
    nav: &'static str,
    control: ControlView,
    basemaps: &'static [Basemap],
}

/// A raster tile layer the map can draw under the cells.
///
/// Tiles are fetched by the operator's browser straight from the provider, so
/// a basemap needs internet access from the browser (not from the server) and
/// falls under that provider's usage policy. "None" stays the default: an
/// air-gapped console must not sit waiting on tiles.
pub struct Basemap {
    pub id: &'static str,
    pub name: &'static str,
    /// XYZ template with `{z}`, `{x}` and `{y}`. Only keyless providers belong
    /// here: CARTO's, for one, now watermarks every tile without a key.
    pub url: &'static str,
    pub max_zoom: u8,
    pub attribution: &'static str,
    pub attribution_url: &'static str,
}

pub const BASEMAPS: &[Basemap] = &[
    Basemap {
        id: "osm",
        name: "OpenStreetMap",
        url: "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        max_zoom: 19,
        attribution: "© OpenStreetMap contributors",
        attribution_url: "https://www.openstreetmap.org/copyright",
    },
    Basemap {
        id: "esri-light-gray",
        name: "Light gray (Esri)",
        url: "https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Light_Gray_Base/MapServer/tile/{z}/{y}/{x}",
        max_zoom: 16,
        attribution: "Esri, HERE, Garmin, © OpenStreetMap contributors",
        attribution_url: "https://www.esri.com/en-us/legal/terms/data-attributions",
    },
    Basemap {
        id: "esri-dark-gray",
        name: "Dark gray (Esri)",
        url: "https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Dark_Gray_Base/MapServer/tile/{z}/{y}/{x}",
        max_zoom: 16,
        attribution: "Esri, HERE, Garmin, © OpenStreetMap contributors",
        attribution_url: "https://www.esri.com/en-us/legal/terms/data-attributions",
    },
    Basemap {
        id: "esri-imagery",
        name: "Satellite (Esri)",
        url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        max_zoom: 19,
        attribution: "Esri, Maxar, Earthstar Geographics, and the GIS User Community",
        attribution_url: "https://www.esri.com/en-us/legal/terms/data-attributions",
    },
];

#[derive(Template)]
#[template(path = "intents.html")]
struct IntentsTemplate {
    nav: &'static str,
    control: ControlView,
    recent: Vec<IntentRow>,
    resolution: String,
}

#[derive(Template)]
#[template(path = "io.html")]
struct IoTemplate {
    nav: &'static str,
    control: ControlView,
    gold: bool,
    paused: bool,
}

#[derive(Template)]
#[template(path = "fragments/control.html")]
struct ControlFragment {
    control: ControlView,
}

#[derive(Template)]
#[template(path = "fragments/scenarios.html")]
struct ScenariosFragment {
    scenarios: Vec<ScenarioRow>,
}

#[derive(Template)]
#[template(path = "fragments/cell.html")]
struct CellFragment {
    cell: CellView,
}

#[derive(Template)]
#[template(path = "fragments/intents.html")]
struct IntentsFragment {
    recent: Vec<IntentRow>,
}

fn render(template: impl Template) -> Response {
    match template.render() {
        Ok(body) => Html(body).into_response(),
        Err(err) => {
            tracing::error!(%err, "dashboard template failed to render");
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                "the dashboard template failed to render; the API is unaffected",
            )
                .into_response()
        }
    }
}

// ------------------------------------------------------------------ login

#[derive(Deserialize)]
pub struct LoginForm {
    token: String,
}

async fn login_form(State(state): State<AppState>) -> Response {
    render(LoginTemplate {
        error: None,
        open: state.auth.is_open(),
    })
}

async fn login_submit(State(state): State<AppState>, Form(form): Form<LoginForm>) -> Response {
    if state.auth.check_login(form.token.trim()) {
        return (
            [(header::SET_COOKIE, Auth::cookie_header(form.token.trim()))],
            Redirect::to("/run"),
        )
            .into_response();
    }
    (
        crate::auth::LOGIN_REJECTED,
        render(LoginTemplate {
            error: Some("That token was not accepted.".to_owned()),
            open: false,
        }),
    )
        .into_response()
}

async fn logout() -> Response {
    (
        [(header::SET_COOKIE, Auth::clear_cookie_header())],
        Redirect::to("/login"),
    )
        .into_response()
}

// ------------------------------------------------------------------ pages

async fn conflicts_page(State(state): State<AppState>) -> Response {
    render(ConflictsTemplate {
        nav: "conflicts",
        control: ControlView::build(&state, None),
        conflicts: ConflictsView::build(&state).await,
        basemaps: BASEMAPS,
    })
}

async fn conflicts_fragment(State(state): State<AppState>) -> Response {
    render(ConflictsFragment {
        conflicts: ConflictsView::build(&state).await,
    })
}

async fn weather_page(State(state): State<AppState>) -> Response {
    render(WeatherTemplate {
        nav: "weather",
        control: ControlView::build(&state, None),
        weather: WeatherView::build(&state, None),
        basemaps: BASEMAPS,
    })
}

async fn weather_fragment(State(state): State<AppState>) -> Response {
    render(WeatherFragment {
        weather: WeatherView::build(&state, None),
    })
}

async fn action_weather_refresh(State(state): State<AppState>) -> Response {
    let flash = match state.weather.refresh(&state.driver).await {
        Ok(s) => match &s.error {
            Some(err) => Flash::bad(format!("Refresh failed: {err}")),
            None if s.cache_hit => {
                Flash::ok("Refreshed from the cache: the forecast has not moved this hour.")
            }
            None => Flash::ok(format!("Refreshed: {} Open-Meteo locations.", s.locations)),
        },
        Err(err) => Flash::warn(format!("{err:#}")),
    };
    render(WeatherFragment {
        weather: WeatherView::build(&state, Some(flash)),
    })
}

/// The API reference, rendered from the OpenAPI document. Outside the token
/// wall, like the document itself: it says how to use the API, not what is in
/// the world.
async fn docs_page() -> Response {
    render(DocsTemplate {
        nav: "docs",
        docs: &crate::openapi::VIEW,
    })
}

async fn run_page(State(state): State<AppState>) -> Response {
    render(RunTemplate {
        nav: "run",
        control: ControlView::build(&state, None),
    })
}

async fn scenarios_page(State(state): State<AppState>) -> Response {
    let scenarios = scenario_rows(&state).await;
    render(ScenariosTemplate {
        nav: "scenarios",
        control: ControlView::build(&state, None),
        scenarios,
        model_kinds: state.model_kinds.join(", "),
    })
}

async fn state_page(State(state): State<AppState>, Query(query): Query<CellQuery>) -> Response {
    let view = state.driver.view();
    render(StateTemplate {
        nav: "state",
        control: ControlView::build(&state, None),
        cell: cell_view(&view, query.cell.as_deref()),
        view_resolution: view.config.grid.view_resolution.to_string(),
        resolution: view.meta.resolution.to_string(),
        basemaps: BASEMAPS,
    })
}

async fn geofences_page(State(state): State<AppState>) -> Response {
    render(GeofencesTemplate {
        nav: "geofences",
        control: ControlView::build(&state, None),
        basemaps: BASEMAPS,
    })
}

async fn stations_page(State(state): State<AppState>) -> Response {
    render(StationsTemplate {
        nav: "stations",
        control: ControlView::build(&state, None),
        basemaps: BASEMAPS,
    })
}

async fn intents_page(State(state): State<AppState>) -> Response {
    let view = state.driver.view();
    render(IntentsTemplate {
        nav: "intents",
        control: ControlView::build(&state, None),
        recent: intent_rows(&view),
        resolution: view.meta.resolution.to_string(),
    })
}

async fn io_page(State(state): State<AppState>) -> Response {
    let view = state.driver.view();
    render(IoTemplate {
        nav: "io",
        control: ControlView::build(&state, None),
        gold: view.config.run.profile == Profile::Gold,
        paused: !view.health.mode.is_advancing(),
    })
}

// -------------------------------------------------------------- fragments

async fn control_fragment(State(state): State<AppState>) -> Response {
    render(ControlFragment {
        control: ControlView::build(&state, None),
    })
}

async fn scenarios_fragment(State(state): State<AppState>) -> Response {
    render(ScenariosFragment {
        scenarios: scenario_rows(&state).await,
    })
}

#[derive(Deserialize)]
pub struct CellQuery {
    #[serde(default)]
    cell: Option<String>,
}

async fn cell_fragment(State(state): State<AppState>, Query(query): Query<CellQuery>) -> Response {
    let view = state.driver.view();
    render(CellFragment {
        cell: cell_view(&view, query.cell.as_deref()),
    })
}

async fn intents_fragment(State(state): State<AppState>) -> Response {
    render(IntentsFragment {
        recent: intent_rows(&state.driver.view()),
    })
}

// ---------------------------------------------------------------- actions

#[derive(Deserialize)]
pub struct StartForm {
    /// Blank means maximum speed. Absent (the dashboard's Start button) keeps
    /// the speed already chosen.
    tps: Option<String>,
}

async fn action_start(State(state): State<AppState>, Form(form): Form<StartForm>) -> Response {
    let Some(tps) = form.tps else {
        let target_tps = state.driver.view().config.run.target_tps;
        let message = format!("Running at {}.", describe_rate_tps(&state, target_tps));
        return act(&state, Control::Start { target_tps }, message).await;
    };
    let trimmed = tps.trim();
    let target_tps = if trimmed.is_empty() {
        None
    } else {
        match trimmed.parse::<f64>() {
            Ok(tps) if tps > 0.0 && tps.is_finite() => Some(tps),
            _ => {
                return control_with(
                    &state,
                    Flash::bad(format!(
                        "'{trimmed}' is not a tick rate. Leave it blank for maximum speed."
                    )),
                )
            }
        }
    };
    let message = match target_tps {
        Some(tps) => format!("Running at {tps:.0} ticks per second."),
        None => "Running at maximum speed.".to_owned(),
    };
    act(&state, Control::Start { target_tps }, message).await
}

#[derive(Deserialize)]
pub struct SpeedForm {
    /// `max`, `realtime` or `custom`.
    #[serde(default)]
    speed: String,
    /// The custom rate, ticks per second. Only read for `custom`.
    #[serde(default)]
    tps: String,
}

/// Change the target rate without starting or pausing the run.
async fn action_speed(State(state): State<AppState>, Form(form): Form<SpeedForm>) -> Response {
    let config = state.driver.view().config.clone();
    let target_tps = match form.speed.as_str() {
        "max" => None,
        "realtime" => match realtime_tps(&config) {
            Some(tps) => Some(tps),
            None => {
                return control_with(
                    &state,
                    Flash::bad("This scenario has no tick_seconds, so it has no real-time rate."),
                )
            }
        },
        "custom" => {
            let trimmed = form.tps.trim();
            match trimmed.parse::<f64>() {
                Ok(tps) if tps > 0.0 && tps.is_finite() => Some(tps),
                _ if trimmed.is_empty() => {
                    return control_with(
                        &state,
                        Flash::warn("Enter a custom rate in ticks per second."),
                    )
                }
                _ => {
                    return control_with(
                        &state,
                        Flash::bad(format!("'{trimmed}' is not a tick rate.")),
                    )
                }
            }
        }
        other => return control_with(&state, Flash::bad(format!("Unknown speed '{other}'."))),
    };
    let message = format!("Speed set to {}.", describe_rate_tps(&state, target_tps));
    act(&state, Control::SetRate(target_tps), message).await
}

async fn action_pause(State(state): State<AppState>) -> Response {
    act(&state, Control::Pause, "Paused.").await
}

#[derive(Deserialize)]
pub struct StepForm {
    #[serde(default)]
    n: String,
}

async fn action_step(State(state): State<AppState>, Form(form): Form<StepForm>) -> Response {
    let n = match form.n.trim() {
        "" => 1,
        text => match text.parse::<u64>() {
            Ok(n) if n > 0 => n,
            _ => return control_with(&state, Flash::bad(format!("'{text}' is not a step count."))),
        },
    };
    let response = act(&state, Control::Step { n }, format!("Stepped {n}.")).await;
    // Show the world after the step, not the frame that accepted it.
    if crate::routes::api::settle(&state).await {
        return control_with(&state, Flash::ok(format!("Stepped {n}.")));
    }
    response
}

async fn action_reset(State(state): State<AppState>) -> Response {
    act(&state, Control::Reset, "Reset to tick 0.").await
}

async fn action_load(State(state): State<AppState>, Path(id): Path<String>) -> Response {
    let store = Arc::clone(&state.scenarios);
    let wanted = id.clone();
    let blob = tokio::task::spawn_blocking(move || store.load(&wanted)).await;
    match blob {
        Ok(Ok(mut blob)) => {
            if let Err(err) = state.weather.prepare(&mut blob).await {
                return control_with(&state, Flash::bad(err.to_string()));
            }
            act(
                &state,
                Control::Load(Box::new(blob)),
                format!("Loaded '{id}'."),
            )
            .await
        }
        Ok(Err(err)) => control_with(&state, Flash::bad(err.to_string())),
        Err(err) => control_with(&state, Flash::bad(err.to_string())),
    }
}

#[derive(Deserialize)]
pub struct IntentForm {
    cell: String,
    verb: String,
    #[serde(default)]
    agent: String,
    #[serde(default)]
    count: String,
    #[serde(default)]
    kind: String,
}

async fn action_intent(State(state): State<AppState>, Form(form): Form<IntentForm>) -> Response {
    let cell = match GeoCell::parse(form.cell.trim()) {
        Ok(cell) => cell,
        Err(err) => return intents_with(&state, Flash::bad(err.to_string())),
    };
    let verb = match form.verb.as_str() {
        "spawn" => IntentVerb::Spawn,
        "despawn" => IntentVerb::Despawn,
        "move" => IntentVerb::Move,
        other => return intents_with(&state, Flash::bad(format!("unknown verb '{other}'"))),
    };

    let view = state.driver.view();
    if let Err(err) = sim_engine::check_intent_cell(&view.meta, cell) {
        return intents_with(&state, Flash::bad(format!("That {err}.")));
    }

    let mut payload = serde_json::Map::new();
    if let Ok(count) = form.count.trim().parse::<u64>() {
        payload.insert("count".into(), count.into());
    }
    if !form.kind.trim().is_empty() {
        payload.insert("kind".into(), form.kind.trim().into());
    }

    let mut intent = Intent::operator(cell, verb, serde_json::Value::Object(payload));
    intent.agent = form.agent.trim().parse::<u64>().ok();

    if !state
        .limiter
        .allow(&intent.actor.0, view.config.run.intent_rate_per_actor)
    {
        return intents_with(
            &state,
            Flash::bad(sim_core::RejectReason::RateLimited.to_string()),
        );
    }

    match state.driver.submit(Submission::fire_and_forget(intent)) {
        Ok(()) => intents_with(
            &state,
            Flash::ok("Queued. It applies on the next tick the driver commits."),
        ),
        Err(reason) => intents_with(&state, Flash::bad(reason.to_string())),
    }
}

/// Run a control command and re-render the control panel with the outcome.
async fn act(state: &AppState, control: Control, message: impl Into<String>) -> Response {
    let driver = state.driver.clone();
    let outcome = tokio::task::spawn_blocking(move || driver.command(control)).await;
    let flash = match outcome {
        Ok(Ok(())) => Flash::ok(message),
        Ok(Err(err)) => Flash::bad(err.to_string()),
        Err(err) => Flash::bad(err.to_string()),
    };
    control_with(state, flash)
}

fn control_with(state: &AppState, flash: Flash) -> Response {
    render(ControlFragment {
        control: ControlView::build(state, Some(flash)),
    })
}

fn intents_with(state: &AppState, flash: Flash) -> Response {
    // The intent form swaps the whole panel, so the flash rides along with the
    // refreshed activity list.
    let view = state.driver.view();
    let _ = flash.kind;
    render(IntentsFragment {
        recent: std::iter::once(IntentRow {
            id: "—".to_owned(),
            actor: "dashboard".to_owned(),
            verb: "submit".to_owned(),
            cell: "—".to_owned(),
            tick: thousands(view.meta.tick),
            rejected: flash.kind == "bad",
            detail: flash.message,
        })
        .chain(intent_rows(&view))
        .collect(),
    })
}

// ------------------------------------------------------------------ shared

async fn scenario_rows(state: &AppState) -> Vec<ScenarioRow> {
    let store = Arc::clone(&state.scenarios);
    let packs = tokio::task::spawn_blocking(move || store.list())
        .await
        .unwrap_or_default();
    let current = state.driver.view().scenario_id.clone();
    let kinds = Arc::clone(&state.model_kinds);

    packs
        .into_iter()
        .map(|pack| ScenarioRow {
            is_current: pack.id == current,
            known_model: kinds.contains(&pack.model),
            id: pack.id,
            name: pack.name,
            description: pack.description,
            model: pack.model,
            resolution: pack.resolution.to_string(),
            root: pack
                .root
                .map(|c| c.to_hex())
                .unwrap_or_else(|| "global".to_owned()),
            profile: match pack.profile {
                Profile::Gold => "gold".to_owned(),
                Profile::Fast => "fast".to_owned(),
            },
            seed_agents: thousands(pack.seed_agents),
        })
        .collect()
}

fn cell_view(view: &Published, query: Option<&str>) -> CellView {
    let query = query.unwrap_or("").trim().to_owned();
    let blank = |query: String, error: Option<String>| CellView {
        query,
        cell: None,
        centroid: String::new(),
        resolution: String::new(),
        in_world: error.is_none(),
        coarse: false,
        agents: String::new(),
        occupied: String::new(),
        objects: Vec::new(),
        hidden: String::new(),
        error,
    };
    if query.is_empty() {
        return blank(query, None);
    }

    let cell = match GeoCell::parse(&query) {
        Ok(cell) => cell,
        Err(err) => return blank(query, Some(err.to_string())),
    };

    let centroid = cell
        .centroid()
        .map(|(lon, lat)| format!("{lon:.5}, {lat:.5}"))
        .unwrap_or_else(|_| "unknown".to_owned());

    let occupancy = view.meta.resolution;
    let coarse = cell.resolution() < occupancy;
    let bags = view.bags_within(cell);
    let agents: usize = bags.iter().map(|bag| bag.objects.len()).sum();
    let objects: Vec<ObjectRow> = bags
        .iter()
        .flat_map(|bag| bag.objects.iter().map(move |o| (bag.cell, o)))
        .take(MAX_CELL_ROWS)
        .map(|(at, o)| ObjectRow {
            id: o.id.to_string(),
            kind: o.kind.clone(),
            state: if o.state.is_null() {
                String::new()
            } else {
                o.state.to_string()
            },
            cell: at.to_hex(),
        })
        .collect();
    let hidden = agents - objects.len();

    // A coarse cell is in the world if any of it overlaps the root: it may
    // contain the root, or lie inside it.
    let in_world = cell.resolution() <= occupancy
        && view
            .meta
            .root
            .is_none_or(|root| cell.is_within(root) || root.is_within(cell));

    CellView {
        query,
        cell: Some(cell.to_hex()),
        centroid,
        resolution: cell.resolution().to_string(),
        in_world,
        coarse,
        agents: thousands(agents as u64),
        occupied: thousands(bags.len() as u64),
        objects,
        hidden: if hidden > 0 {
            thousands(hidden as u64)
        } else {
            String::new()
        },
        error: None,
    }
}

fn intent_rows(view: &Published) -> Vec<IntentRow> {
    view.recent_intents
        .iter()
        .rev()
        .map(|record| IntentRow {
            id: record.id.to_string(),
            actor: record.actor.clone(),
            verb: record.verb.clone(),
            cell: record.cell.to_hex(),
            tick: thousands(record.tick),
            rejected: record.rejected.is_some(),
            detail: record.detail.clone(),
        })
        .collect()
}

/// Group a count with thin separators. Operator screens are full of numbers
/// that differ by orders of magnitude; ungrouped they are unreadable at a glance.
fn thousands(value: u64) -> String {
    let digits = value.to_string();
    let mut out = String::with_capacity(digits.len() + digits.len() / 3);
    for (index, ch) in digits.chars().enumerate() {
        if index > 0 && (digits.len() - index) % 3 == 0 {
            out.push('\u{202f}');
        }
        out.push(ch);
    }
    out
}

/// Ticks per second that keep simulated time in step with the wall clock:
/// one over the model's `tick_seconds`, if it has one.
fn realtime_tps(config: &Config) -> Option<f64> {
    tick_seconds(config).map(|s| 1.0 / s)
}

/// Simulated seconds per tick, from the model's `tick_seconds`.
fn tick_seconds(config: &Config) -> Option<f64> {
    config
        .model
        .get("tick_seconds")
        .and_then(Param::as_f64)
        .filter(|s| *s > 0.0 && s.is_finite())
}

/// `4 d 03:12:44` (days only once there are any), whole seconds.
fn format_sim_time(seconds: f64) -> String {
    let total = seconds.max(0.0).floor() as u64;
    let (days, rest) = (total / 86_400, total % 86_400);
    let clock = format!(
        "{:02}:{:02}:{:02}",
        rest / 3600,
        rest % 3600 / 60,
        rest % 60
    );
    if days > 0 {
        format!("{days} d {clock}")
    } else {
        clock
    }
}

fn is_realtime(config: &Config, tps: f64) -> bool {
    realtime_tps(config).is_some_and(|rt| (tps - rt).abs() <= rt * 1e-9)
}

fn speed_choice(config: &Config) -> &'static str {
    match config.run.target_tps {
        None => "max",
        Some(tps) if is_realtime(config, tps) => "realtime",
        Some(_) => "custom",
    }
}

/// Up to three significant decimals, without trailing zeros: 10, 2.5, 0.125.
fn format_tps(tps: f64) -> String {
    let text = format!("{tps:.3}");
    text.trim_end_matches('0').trim_end_matches('.').to_owned()
}

fn describe_rate(config: &Config) -> String {
    describe(config, config.run.target_tps)
}

fn describe_rate_tps(state: &AppState, target_tps: Option<f64>) -> String {
    describe(&state.driver.view().config, target_tps)
}

fn describe(config: &Config, target_tps: Option<f64>) -> String {
    match target_tps {
        None => "maximum speed".to_owned(),
        Some(tps) if is_realtime(config, tps) => format!("real time ({} tps)", format_tps(tps)),
        Some(tps) => match realtime_tps(config) {
            Some(rt) => format!(
                "{} tps ({}× real time)",
                format_tps(tps),
                format_tps(tps / rt)
            ),
            None => format!("{} tps", format_tps(tps)),
        },
    }
}

fn format_rate(tps: f64) -> String {
    if tps >= 1_000_000.0 {
        format!("{:.2} M", tps / 1_000_000.0)
    } else if tps >= 1_000.0 {
        format!("{:.1} k", tps / 1_000.0)
    } else {
        format!("{tps:.0}")
    }
}

fn format_nanos(ns: u64) -> String {
    if ns >= 1_000_000 {
        format!("{:.2} ms", ns as f64 / 1e6)
    } else if ns >= 1_000 {
        format!("{:.1} \u{b5}s", ns as f64 / 1e3)
    } else {
        format!("{ns} ns")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_are_grouped_for_reading() {
        assert_eq!(thousands(0), "0");
        assert_eq!(thousands(999), "999");
        assert_eq!(thousands(1_000), "1\u{202f}000");
        assert_eq!(thousands(20_000_000), "20\u{202f}000\u{202f}000");
    }

    #[test]
    fn rates_and_durations_change_unit_rather_than_digits() {
        assert_eq!(format_rate(12.0), "12");
        assert_eq!(format_rate(2_450.0), "2.5 k");
        assert_eq!(format_rate(1_500_000.0), "1.50 M");
        assert_eq!(format_nanos(500), "500 ns");
        assert_eq!(format_nanos(4_500), "4.5 \u{b5}s");
        assert_eq!(format_nanos(2_000_000), "2.00 ms");
    }

    #[test]
    fn sim_time_reads_as_a_clock_with_days() {
        assert_eq!(format_sim_time(0.0), "00:00:00");
        assert_eq!(format_sim_time(4_260.9), "01:11:00");
        assert_eq!(format_sim_time(3.0 * 86_400.0 + 3_725.0), "3 d 01:02:05");
    }
}
