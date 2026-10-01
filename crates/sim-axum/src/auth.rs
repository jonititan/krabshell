//! Shared-secret auth (spec §11.2).
//!
//! One token, from the environment, for the whole server. That is enough for a
//! single-operator tool bound to loopback, and it is deliberately not a user
//! system: there are no accounts to get wrong.
//!
//! Three ways to present it, because three kinds of client need it:
//!
//! - `Authorization: Bearer <token>` for tools and scripts,
//! - a `krabshell_token` cookie for the dashboard, which cannot set headers on
//!   a plain navigation,
//! - `?token=` for the WebSocket, since browsers cannot set headers there
//!   either.

use std::sync::Arc;

use axum::extract::{Request, State};
use axum::http::{header, StatusCode};
use axum::middleware::Next;
use axum::response::{IntoResponse, Redirect, Response};
use serde::Deserialize;
use sim_core::ErrorCode;

use crate::problem::ApiError;

/// Cookie the dashboard stores its token in.
pub const COOKIE: &str = "krabshell_token";

#[derive(Debug, Clone)]
pub struct Auth {
    token: Option<String>,
}

impl Auth {
    /// Read the token from `DASHBOARD_TOKEN`, or run open.
    ///
    /// An absent token is legitimate for a loopback dev run, so it is a warning
    /// rather than a hard failure — but the bind address check in `main` will
    /// refuse to expose an open server beyond loopback.
    pub fn from_env() -> Self {
        match std::env::var("DASHBOARD_TOKEN") {
            Ok(token) if !token.trim().is_empty() => Auth {
                token: Some(token.trim().to_owned()),
            },
            _ => {
                tracing::warn!(
                    "DASHBOARD_TOKEN is not set: every request will be accepted. \
                     Set it before binding anything but loopback."
                );
                Auth { token: None }
            }
        }
    }

    pub fn with_token(token: Option<String>) -> Self {
        Auth { token }
    }

    pub fn is_open(&self) -> bool {
        self.token.is_none()
    }

    /// Constant-time-ish comparison. The token is short and the attacker is on
    /// loopback by default, but there is no reason to leak the prefix length.
    fn matches(&self, offered: &str) -> bool {
        let Some(expected) = &self.token else {
            return true;
        };
        if expected.len() != offered.len() {
            return false;
        }
        expected
            .bytes()
            .zip(offered.bytes())
            .fold(0u8, |acc, (a, b)| acc | (a ^ b))
            == 0
    }

    /// Whether a request carries a usable token.
    pub fn accepts(&self, request: &Request) -> bool {
        if self.is_open() {
            return true;
        }
        if let Some(offered) = bearer(request) {
            if self.matches(&offered) {
                return true;
            }
        }
        if let Some(offered) = cookie_token(request) {
            if self.matches(&offered) {
                return true;
            }
        }
        if let Some(offered) = query_token(request) {
            if self.matches(&offered) {
                return true;
            }
        }
        false
    }

    /// Build the `Set-Cookie` value the login form returns.
    ///
    /// `HttpOnly` so page scripts cannot read it, `SameSite=Strict` so another
    /// site cannot ride it, and no `Secure` flag because the default bind is
    /// plain HTTP on loopback.
    pub fn cookie_header(token: &str) -> String {
        format!("{COOKIE}={token}; Path=/; HttpOnly; SameSite=Strict; Max-Age=604800")
    }

    pub fn clear_cookie_header() -> String {
        format!("{COOKIE}=; Path=/; HttpOnly; SameSite=Strict; Max-Age=0")
    }

    /// Check a token offered by the login form.
    pub fn check_login(&self, offered: &str) -> bool {
        self.matches(offered)
    }
}

/// Guard for `/api/*`: unauthenticated requests get a problem body.
pub async fn require_api_token(
    State(auth): State<Arc<Auth>>,
    request: Request,
    next: Next,
) -> Response {
    if auth.accepts(&request) {
        return next.run(request).await;
    }
    ApiError::new(
        ErrorCode::Unauthorized,
        "present the dashboard token as a bearer header, a krabshell_token cookie, or ?token=",
    )
    .into_response()
}

/// Guard for dashboard pages: unauthenticated requests get the login screen.
pub async fn require_page_token(
    State(auth): State<Arc<Auth>>,
    request: Request,
    next: Next,
) -> Response {
    if auth.accepts(&request) {
        return next.run(request).await;
    }
    Redirect::to("/login").into_response()
}

fn bearer(request: &Request) -> Option<String> {
    let value = request
        .headers()
        .get(header::AUTHORIZATION)?
        .to_str()
        .ok()?;
    let token = value
        .strip_prefix("Bearer ")
        .or_else(|| value.strip_prefix("bearer "))?;
    Some(token.trim().to_owned())
}

fn cookie_token(request: &Request) -> Option<String> {
    let header = request.headers().get(header::COOKIE)?.to_str().ok()?;
    header.split(';').find_map(|pair| {
        let (name, value) = pair.split_once('=')?;
        (name.trim() == COOKIE).then(|| value.trim().to_owned())
    })
}

#[derive(Deserialize)]
struct TokenQuery {
    token: Option<String>,
}

fn query_token(request: &Request) -> Option<String> {
    let query = request.uri().query()?;
    let parsed: TokenQuery = serde_urlencoded::from_str(query).ok()?;
    parsed.token
}

/// Status code used when the login form is given the wrong token.
pub const LOGIN_REJECTED: StatusCode = StatusCode::UNAUTHORIZED;

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;

    fn request(
        build: impl FnOnce(axum::http::request::Builder) -> axum::http::request::Builder,
    ) -> Request {
        build(Request::builder().uri("/api/v1/run"))
            .body(Body::empty())
            .unwrap()
    }

    #[test]
    fn an_open_server_accepts_anything() {
        let auth = Auth::with_token(None);
        assert!(auth.is_open());
        assert!(auth.accepts(&request(|b| b)));
    }

    #[test]
    fn a_bearer_header_is_accepted() {
        let auth = Auth::with_token(Some("s3cret".into()));
        assert!(auth.accepts(&request(|b| b.header("authorization", "Bearer s3cret"))));
        assert!(!auth.accepts(&request(|b| b.header("authorization", "Bearer wrong!"))));
        assert!(!auth.accepts(&request(|b| b)));
    }

    #[test]
    fn the_dashboard_cookie_is_accepted_among_other_cookies() {
        let auth = Auth::with_token(Some("s3cret".into()));
        assert!(auth.accepts(&request(|b| {
            b.header("cookie", "theme=dark; krabshell_token=s3cret; other=1")
        })));
        assert!(!auth.accepts(&request(|b| b.header("cookie", "krabshell_token=nope"))));
    }

    #[test]
    fn a_query_token_is_accepted_so_websockets_can_connect() {
        let auth = Auth::with_token(Some("s3cret".into()));
        let ws = Request::builder()
            .uri("/api/v1/ws?token=s3cret")
            .body(Body::empty())
            .unwrap();
        assert!(auth.accepts(&ws));
    }

    #[test]
    fn a_token_of_a_different_length_is_refused_without_comparing_bytes() {
        let auth = Auth::with_token(Some("s3cret".into()));
        assert!(!auth.check_login("s3"));
        assert!(!auth.check_login("s3cretlonger"));
        assert!(auth.check_login("s3cret"));
    }
}
