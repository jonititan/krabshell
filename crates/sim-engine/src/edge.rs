//! Checks every way into the driver shares, whatever the transport.
//!
//! REST, the WebSocket, the dashboard and the MAVLink gateway all admit intents
//! from outside. The address check and the per-actor rate limit live here so
//! they cannot drift apart between them (spec §8.2, §13). Neither needs the
//! tick thread: both run at the edge, before an intent costs a queue slot.

use sim_core::{GeoCell, RejectReason, SnapshotMeta};

/// Why an intent's cell does not belong to the running world.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CellCheck {
    #[error("cell is at resolution {got}, but this world runs at {want}")]
    WrongResolution { got: i32, want: i32 },
    #[error("cell is outside the run's root {root}")]
    OutsideRoot { root: GeoCell },
}

impl From<CellCheck> for RejectReason {
    fn from(_: CellCheck) -> Self {
        RejectReason::OutOfBounds
    }
}

/// Whether `cell` is addressable in the world `meta` describes.
pub fn check_intent_cell(meta: &SnapshotMeta, cell: GeoCell) -> Result<(), CellCheck> {
    if cell.resolution() != meta.resolution {
        return Err(CellCheck::WrongResolution {
            got: cell.resolution(),
            want: meta.resolution,
        });
    }
    match meta.root {
        Some(root) if !cell.is_within(root) => Err(CellCheck::OutsideRoot { root }),
        _ => Ok(()),
    }
}

// ----------------------------------------------------------- rate limiting
//
// A token bucket per actor id, checked before an intent takes a slot in the
// driver's queue. The queue cap protects the tick loop; this protects other
// actors from one noisy one.
//
// The actor id is whatever the client claims. With a single shared token that
// is all there is to go on, so this limits accidents and runaway clients, not
// a determined adversary holding the token.

use std::collections::HashMap;
use std::sync::Mutex;
use std::time::Instant;

/// Above this many tracked actors, full buckets are forgotten. A full bucket
/// carries no information, so dropping it changes nothing for that actor.
const PRUNE_ABOVE: usize = 10_000;

#[derive(Debug, Clone, Copy)]
struct Bucket {
    tokens: f64,
    refilled: Instant,
}

#[derive(Debug, Default)]
pub struct ActorLimiter {
    buckets: Mutex<HashMap<String, Bucket>>,
}

impl ActorLimiter {
    pub fn new() -> Self {
        Self::default()
    }

    /// Take one token for `actor` at `rate` per second, bursting to `2 * rate`.
    pub fn allow(&self, actor: &str, rate: f64) -> bool {
        self.allow_at(actor, rate, Instant::now())
    }

    fn allow_at(&self, actor: &str, rate: f64, now: Instant) -> bool {
        let burst = (rate * 2.0).max(1.0);
        let mut buckets = self.buckets.lock().unwrap_or_else(|e| e.into_inner());

        if buckets.len() > PRUNE_ABOVE {
            buckets.retain(|_, b| {
                b.tokens + now.duration_since(b.refilled).as_secs_f64() * rate < burst
            });
        }

        let bucket = buckets.entry(actor.to_owned()).or_insert(Bucket {
            tokens: burst,
            refilled: now,
        });
        let elapsed = now.duration_since(bucket.refilled).as_secs_f64();
        bucket.tokens = (bucket.tokens + elapsed * rate).min(burst);
        bucket.refilled = now;

        if bucket.tokens >= 1.0 {
            bucket.tokens -= 1.0;
            true
        } else {
            false
        }
    }

    #[cfg(test)]
    fn tracked(&self) -> usize {
        self.buckets.lock().unwrap().len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn a_burst_of_twice_the_rate_is_allowed_then_refused() {
        let limiter = ActorLimiter::new();
        let now = Instant::now();
        let allowed = (0..50).filter(|_| limiter.allow_at("a", 10.0, now)).count();
        assert_eq!(allowed, 20);
    }

    #[test]
    fn tokens_come_back_with_time() {
        let limiter = ActorLimiter::new();
        let start = Instant::now();
        while limiter.allow_at("a", 10.0, start) {}
        assert!(!limiter.allow_at("a", 10.0, start));
        assert!(limiter.allow_at("a", 10.0, start + Duration::from_millis(150)));
    }

    #[test]
    fn one_noisy_actor_does_not_starve_another() {
        let limiter = ActorLimiter::new();
        let now = Instant::now();
        while limiter.allow_at("noisy", 5.0, now) {}
        assert!(limiter.allow_at("quiet", 5.0, now));
    }

    #[test]
    fn idle_actors_are_forgotten_rather_than_accumulating() {
        let limiter = ActorLimiter::new();
        let start = Instant::now();
        for i in 0..=PRUNE_ABOVE {
            limiter.allow_at(&format!("actor-{i}"), 100.0, start);
        }
        // A second later every bucket has refilled, so pruning drops them all.
        limiter.allow_at("late", 100.0, start + Duration::from_secs(1));
        assert!(limiter.tracked() < 10);
    }
}
