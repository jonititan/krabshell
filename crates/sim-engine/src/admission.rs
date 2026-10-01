//! Intent admission (spec §8.2).
//!
//! These are the checks the driver can make without asking the model: identity,
//! timing, and queue pressure. Game rules get their veto later, in
//! [`crate::model::Model::apply_intent`].
//!
//! Every rejection here is final. Nothing is retried on a client's behalf, so
//! the client always learns the outcome of what it submitted.

use std::collections::{HashSet, VecDeque};

use sim_core::{Config, Intent, RejectReason};
use uuid::Uuid;

/// How many recent ids the duplicate ring remembers. Sized so a client
/// retrying a burst of intents cannot smuggle a duplicate past the window.
const DEDUP_RING: usize = 16_384;

#[derive(Debug, Clone, PartialEq)]
pub enum AdmissionOutcome {
    Accept,
    Reject(RejectReason),
}

/// Stateful admission checks. Lives on the tick thread.
pub struct Admission {
    seen: HashSet<Uuid>,
    order: VecDeque<Uuid>,
    horizon: u64,
    max_pending: usize,
}

impl Admission {
    pub fn new(config: &Config) -> Self {
        Admission {
            seen: HashSet::with_capacity(DEDUP_RING),
            order: VecDeque::with_capacity(DEDUP_RING),
            horizon: config.run.intent_horizon,
            max_pending: config.run.max_pending_intents,
        }
    }

    /// Decide the fate of one intent against the currently committed tick.
    pub fn admit(&mut self, intent: &Intent, committed_tick: u64) -> AdmissionOutcome {
        if self.seen.contains(&intent.id) {
            return AdmissionOutcome::Reject(RejectReason::Duplicate);
        }
        if let Some(target) = intent.tick {
            if target <= committed_tick {
                return AdmissionOutcome::Reject(RejectReason::Stale {
                    target,
                    committed: committed_tick,
                });
            }
            if target > committed_tick + self.horizon {
                return AdmissionOutcome::Reject(RejectReason::TooFar {
                    target,
                    horizon: self.horizon,
                });
            }
        }
        self.remember(intent.id);
        AdmissionOutcome::Accept
    }

    /// The rejection to hand back when the pending queue is full.
    pub fn backpressure(&self, pending: usize) -> RejectReason {
        RejectReason::Backpressured { pending }
    }

    pub fn max_pending(&self) -> usize {
        self.max_pending
    }

    fn remember(&mut self, id: Uuid) {
        if self.order.len() >= DEDUP_RING {
            if let Some(evicted) = self.order.pop_front() {
                self.seen.remove(&evicted);
            }
        }
        self.order.push_back(id);
        self.seen.insert(id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{ActorId, GeoCell, IntentVerb};

    fn admission(horizon: u64) -> Admission {
        let mut config = Config::default();
        config.run.intent_horizon = horizon;
        Admission::new(&config)
    }

    fn intent(tick: Option<u64>) -> Intent {
        Intent {
            id: Uuid::new_v4(),
            tick,
            cell: GeoCell::from_raw(1234),
            agent: None,
            actor: ActorId::operator(),
            verb: IntentVerb::Spawn,
            payload: serde_json::Value::Null,
            issued_at: 0,
        }
    }

    #[test]
    fn an_untimed_intent_is_always_admitted() {
        let mut admission = admission(8);
        assert_eq!(
            admission.admit(&intent(None), 500),
            AdmissionOutcome::Accept
        );
    }

    #[test]
    fn an_intent_for_a_committed_tick_is_stale() {
        let mut admission = admission(8);
        assert_eq!(
            admission.admit(&intent(Some(10)), 10),
            AdmissionOutcome::Reject(RejectReason::Stale {
                target: 10,
                committed: 10
            })
        );
        assert_eq!(
            admission.admit(&intent(Some(11)), 10),
            AdmissionOutcome::Accept,
            "the next tick is the earliest a client can target"
        );
    }

    #[test]
    fn an_intent_beyond_the_horizon_is_refused() {
        let mut admission = admission(8);
        assert_eq!(
            admission.admit(&intent(Some(18)), 10),
            AdmissionOutcome::Accept
        );
        assert_eq!(
            admission.admit(&intent(Some(19)), 10),
            AdmissionOutcome::Reject(RejectReason::TooFar {
                target: 19,
                horizon: 8
            })
        );
    }

    #[test]
    fn the_same_id_is_only_admitted_once() {
        let mut admission = admission(8);
        let once = intent(None);
        assert_eq!(admission.admit(&once, 0), AdmissionOutcome::Accept);
        assert_eq!(
            admission.admit(&once, 0),
            AdmissionOutcome::Reject(RejectReason::Duplicate)
        );
    }

    #[test]
    fn the_dedup_ring_forgets_the_oldest_ids_rather_than_growing() {
        let mut admission = admission(8);
        let first = intent(None);
        assert_eq!(admission.admit(&first, 0), AdmissionOutcome::Accept);
        for _ in 0..DEDUP_RING {
            admission.admit(&intent(None), 0);
        }
        assert_eq!(admission.order.len(), DEDUP_RING);
        assert_eq!(admission.seen.len(), DEDUP_RING);
        assert_eq!(
            admission.admit(&first, 0),
            AdmissionOutcome::Accept,
            "the id aged out of the window"
        );
    }
}
