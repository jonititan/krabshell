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
//!
//! A token says who may ask; [`same_site_only`] says from where. A browser
//! sends the cookie with a form another site posts to us, and in open mode
//! there is no token at all, so any page the operator has open could reset
//! the world. Every request that changes something, and every WebSocket, must
//! therefore come from the dashboard's own origin or from a tool that is not a
//! browser.

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
    /// Origins besides the server's own that may send writes, as
    /// `scheme://host[:port]`: the public address behind a reverse proxy.
    origins: Vec<String>,
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
                origins: Vec::new(),
            },
            _ => {
                tracing::warn!(
                    "DASHBOARD_TOKEN is not set: every request will be accepted. \
                     Set it before binding anything but loopback."
                );
                Auth {
                    token: None,
                    origins: Vec::new(),
                }
            }
        }
    }

    pub fn with_token(token: Option<String>) -> Self {
        Auth {
            token,
            origins: Vec::new(),
        }
    }

    /// Also accept writes from these origins (`scheme://host[:port]`).
    pub fn with_origins(self, origins: Vec<String>) -> Self {
        let origins = origins
            .into_iter()
            .map(|o| o.trim().trim_end_matches('/').to_ascii_lowercase())
            .filter(|o| !o.is_empty())
            .collect();
        Auth { origins, ..self }
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

/// Refuse what a browser sends on another site's behalf.
///
/// - A write (anything but GET, HEAD and OPTIONS) or a WebSocket upgrade
///   whose `Origin` is not the server's own, or that the browser marks
///   `Sec-Fetch-Site: cross-site` or `same-site` (another port on localhost
///   is "same-site"), is refused. Requests with neither header come from
///   tools, not browsers, and pass.
/// - With no token, the server is only safe because only loopback can reach
///   it, and DNS rebinding gets round that: an attacker's name that resolves
///   to 127.0.0.1 makes their page same-origin with us. So in open mode the
///   `Host` must be a loopback name too.
pub async fn same_site_only(
    State(auth): State<Arc<Auth>>,
    request: Request,
    next: Next,
) -> Response {
    match auth.refusal(&request) {
        None => next.run(request).await,
        Some(why) => ApiError::new(ErrorCode::Forbidden, why).into_response(),
    }
}

impl Auth {
    fn refusal(&self, request: &Request) -> Option<&'static str> {
        let headers = request.headers();
        let text = |name: &str| headers.get(name).and_then(|v| v.to_str().ok());
        let host = text("host").map(str::to_ascii_lowercase);
        if self.is_open() {
            if let Some(host) = &host {
                if !is_loopback_host(host) {
                    return Some(
                        "this server has no token, so it only answers to a loopback host name",
                    );
                }
            }
        }
        let method = request.method();
        let upgrade = text("upgrade").is_some_and(|u| u.eq_ignore_ascii_case("websocket"));
        let reads = matches!(method.as_str(), "GET" | "HEAD" | "OPTIONS");
        if reads && !upgrade {
            return None;
        }
        if matches!(text("sec-fetch-site"), Some("cross-site" | "same-site")) {
            return Some("refused: sent from another site");
        }
        let origin = text("origin")?.trim().to_ascii_lowercase();
        let own = host
            .as_deref()
            .is_some_and(|host| origin.split_once("://").is_some_and(|(_, a)| a == host));
        if own || self.origins.contains(&origin) {
            None
        } else {
            Some("refused: sent from another origin (allow it with --allowed-origin)")
        }
    }
}

/// `localhost`, `127.x.x.x` or `[::1]`, with or without a port.
fn is_loopback_host(host: &str) -> bool {
    let name = if let Some(rest) = host.strip_prefix('[') {
        rest.split(']').next().unwrap_or("")
    } else {
        host.rsplit_once(':').map_or(host, |(name, _)| name)
    };
    name == "localhost"
        || name.ends_with(".localhost")
        || name
            .parse::<std::net::IpAddr>()
            .is_ok_and(|ip| ip.is_loopback())
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

    fn refused(auth: &Auth, method: &str, headers: &[(&str, &str)]) -> bool {
        let mut b = Request::builder().method(method).uri("/actions/reset");
        for (k, v) in headers {
            b = b.header(*k, *v);
        }
        auth.refusal(&b.body(Body::empty()).unwrap()).is_some()
    }

    #[test]
    fn writes_from_another_site_are_refused() {
        let auth = Auth::with_token(None);
        let host = ("host", "127.0.0.1:8080");
        // The dashboard itself, and tools that are not browsers.
        assert!(!refused(
            &auth,
            "POST",
            &[host, ("origin", "http://127.0.0.1:8080")]
        ));
        assert!(!refused(&auth, "POST", &[host]));
        assert!(!refused(&auth, "POST", &[]));
        // Another site, another port, a sandboxed page.
        assert!(refused(
            &auth,
            "POST",
            &[host, ("origin", "https://evil.example")]
        ));
        assert!(refused(
            &auth,
            "POST",
            &[host, ("origin", "http://127.0.0.1:3000")]
        ));
        assert!(refused(&auth, "POST", &[host, ("origin", "null")]));
        assert!(refused(
            &auth,
            "DELETE",
            &[host, ("sec-fetch-site", "same-site")]
        ));
        // Reads pass; a WebSocket upgrade is a write.
        assert!(!refused(
            &auth,
            "GET",
            &[host, ("origin", "https://evil.example")]
        ));
        assert!(refused(
            &auth,
            "GET",
            &[
                host,
                ("upgrade", "websocket"),
                ("origin", "https://evil.example")
            ]
        ));
    }

    #[test]
    fn an_open_server_answers_only_to_loopback_names() {
        let open = Auth::with_token(None);
        for host in [
            "127.0.0.1:8080",
            "localhost:8080",
            "[::1]:8080",
            "localhost",
        ] {
            assert!(!refused(&open, "GET", &[("host", host)]), "{host}");
        }
        // DNS rebinding: an attacker's name pointed at 127.0.0.1.
        assert!(refused(
            &open,
            "GET",
            &[("host", "rebind.evil.example:8080")]
        ));
        // With a token the name does not matter.
        let closed = Auth::with_token(Some("s3cret".into()));
        assert!(!refused(&closed, "GET", &[("host", "krabshell.lan:8080")]));
    }

    #[test]
    fn an_allowed_origin_may_write() {
        let auth = Auth::with_token(Some("s3cret".into()))
            .with_origins(vec!["https://Ops.Example/".into()]);
        let host = ("host", "10.0.0.5:8080");
        assert!(!refused(
            &auth,
            "POST",
            &[host, ("origin", "https://ops.example")]
        ));
        assert!(refused(
            &auth,
            "POST",
            &[host, ("origin", "https://other.example")]
        ));
    }

    #[test]
    fn a_token_of_a_different_length_is_refused_without_comparing_bytes() {
        let auth = Auth::with_token(Some("s3cret".into()));
        assert!(!auth.check_login("s3"));
        assert!(!auth.check_login("s3cretlonger"));
        assert!(auth.check_login("s3cret"));
    }
}
