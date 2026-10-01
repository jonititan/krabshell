//! Route handlers, split by audience: `api` for tools and the dashboard's
//! fetches, `ws` for live subscriptions, `pages` for the operator screens.

pub mod api;
pub mod pages;
pub mod ws;
