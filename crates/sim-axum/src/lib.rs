//! The HTTP edge (spec §9) and the operator dashboard (§11).
//!
//! Nothing here owns the world. Every route reads the driver's published
//! snapshot or posts a message to it, so a slow client, a large download or a
//! wedged browser cannot cost the simulation a tick.

pub mod auth;
pub mod problem;
pub mod ratelimit;
pub mod routes;

use std::sync::Arc;
use std::time::Instant;

use axum::routing::{get, post, put};
use axum::Router;
use sim_engine::{DriverHandle, ScenarioStore};
use tower_http::limit::RequestBodyLimitLayer;
use tower_http::trace::TraceLayer;

use crate::auth::Auth;

/// API prefix. Bump the version, never the shape, when clients must change.
pub const API_PREFIX: &str = "/api/v1";

/// Largest body any route will read. Scenario uploads are the big one; intents
/// are capped far lower in their own handler (spec §13).
pub const MAX_BODY_BYTES: usize = 64 * 1024 * 1024;

/// Largest single intent body.
pub const MAX_INTENT_BYTES: usize = 16 * 1024;

#[derive(Clone)]
pub struct AppState {
    pub driver: DriverHandle,
    pub scenarios: Arc<ScenarioStore>,
    pub auth: Arc<Auth>,
    pub started_at: Instant,
    /// Models this build can instantiate, for the scenario screen.
    pub model_kinds: Arc<Vec<String>>,
    pub limiter: Arc<ratelimit::ActorLimiter>,
}

impl AppState {
    pub fn new(driver: DriverHandle, scenarios: ScenarioStore, auth: Auth) -> Self {
        AppState {
            driver,
            scenarios: Arc::new(scenarios),
            auth: Arc::new(auth),
            started_at: Instant::now(),
            model_kinds: Arc::new(
                sim_engine::ModelRegistry::with_builtins()
                    .kinds()
                    .into_iter()
                    .map(str::to_owned)
                    .collect(),
            ),
            limiter: Arc::new(ratelimit::ActorLimiter::new()),
        }
    }
}

/// The whole server: API, WebSocket, dashboard, static assets.
pub fn router(state: AppState) -> Router {
    let auth = Arc::clone(&state.auth);

    // `/health` sits outside the token wall on purpose: a process monitor
    // should be able to see that the driver is alive without holding a secret.
    let public_api = Router::new().route("/health", get(routes::api::health));

    let guarded_api = Router::new()
        .route("/run", get(routes::api::run_status))
        .route("/run/start", post(routes::api::run_start))
        .route("/run/pause", post(routes::api::run_pause))
        .route("/run/step", post(routes::api::run_step))
        .route("/run/reset", post(routes::api::run_reset))
        .route("/run/rate", post(routes::api::run_rate))
        .route("/scenarios", get(routes::api::scenarios_list))
        .route("/scenarios", post(routes::api::scenarios_upload))
        .route("/scenarios/{id}/load", post(routes::api::scenarios_load))
        .route("/config", get(routes::api::config_get))
        .route("/config", put(routes::api::config_put))
        .route("/config/download", get(routes::api::config_download))
        .route("/state", get(routes::api::state_get))
        .route("/state/map", get(routes::api::state_map))
        .route("/state/entities", get(routes::api::state_entities))
        .route("/state/download", get(routes::api::state_download))
        .route("/state/upload", post(routes::api::state_upload))
        .route("/state/cells/{cell}", get(routes::api::state_cell))
        .route("/intents", post(routes::api::intents_post))
        .route("/intents/log", get(routes::api::intents_log))
        .route("/metrics", get(routes::api::metrics))
        .route("/reports", get(routes::api::reports))
        .route("/cells/lookup", get(routes::api::cells_lookup))
        .route("/ws", get(routes::ws::upgrade))
        .route_layer(axum::middleware::from_fn_with_state(
            Arc::clone(&auth),
            auth::require_api_token,
        ));

    let pages = routes::pages::router(Arc::clone(&auth));

    Router::new()
        .nest(API_PREFIX, public_api.merge(guarded_api))
        .merge(pages)
        .layer(RequestBodyLimitLayer::new(MAX_BODY_BYTES))
        .layer(TraceLayer::new_for_http())
        .with_state(state)
}
