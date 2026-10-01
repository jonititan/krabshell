//! Per-actor intent rate limiting (spec §13). Lives in `sim_engine::edge` so
//! the MAVLink gateway shares it; re-exported here for the routes.

pub use sim_engine::edge::ActorLimiter;
