//! Turning failures into `application/problem+json` (spec §9.2).
//!
//! Every fallible handler returns [`ApiResult`], so a route can use `?` and
//! still produce a body with a stable `code` a client can switch on.

use axum::http::{header, StatusCode};
use axum::response::{IntoResponse, Response};
use sim_core::{ErrorCode, Problem};
use sim_engine::DriverError;

/// What every JSON handler returns.
pub type ApiResult<T> = Result<T, ApiError>;

/// A [`Problem`] that knows how to become an HTTP response.
#[derive(Debug, Clone)]
pub struct ApiError(pub Problem);

impl ApiError {
    pub fn new(code: ErrorCode, detail: impl Into<String>) -> Self {
        ApiError(Problem::new(code, detail))
    }

    pub fn bad_request(detail: impl Into<String>) -> Self {
        ApiError::new(ErrorCode::BadRequest, detail)
    }

    pub fn not_found(detail: impl Into<String>) -> Self {
        ApiError::new(ErrorCode::NotFound, detail)
    }

    pub fn with_meta(mut self, meta: serde_json::Value) -> Self {
        self.0 = self.0.with_meta(meta);
        self
    }

    pub fn code(&self) -> ErrorCode {
        self.0.code
    }
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let status =
            StatusCode::from_u16(self.0.status).unwrap_or(StatusCode::INTERNAL_SERVER_ERROR);
        let body = serde_json::to_vec(&self.0).unwrap_or_else(|_| {
            br#"{"code":"internal","title":"internal","detail":"error body could not be encoded","status":500}"#
                .to_vec()
        });
        (
            status,
            [(header::CONTENT_TYPE, "application/problem+json")],
            body,
        )
            .into_response()
    }
}

impl<E: Into<Problem>> From<E> for ApiError {
    fn from(err: E) -> Self {
        ApiError(err.into())
    }
}

/// The driver being unreachable is a 503, not a 500: the process is fine, the
/// simulation is not (spec §12).
pub fn from_driver(err: DriverError) -> ApiError {
    match err {
        DriverError::Gone => ApiError::new(
            ErrorCode::DriverUnavailable,
            "the simulation thread is not running",
        ),
        DriverError::Timeout => ApiError::new(
            ErrorCode::DriverUnavailable,
            "the simulation thread did not answer in time",
        ),
        DriverError::Refused(message) => ApiError::new(ErrorCode::BadRequest, message),
    }
}
