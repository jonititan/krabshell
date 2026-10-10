//! Resolution advisories: what each aircraft in conflict should do.
//!
//! The search is greedy and the comparison is the prediction itself. With
//! everyone carrying on as they are, find the conflicts. Then, for every
//! aircraft in one that has no advisory yet, and every manoeuvre it could
//! fly, predict it again flying that manoeuvre, find the conflicts again, and
//! score what is left. Give the advisory that removes the most risk, net of
//! the manoeuvre's cost (vertical manoeuvres cost least, as in airborne
//! collision avoidance), and go round again with that aircraft committed,
//! until nothing is left in conflict or no manoeuvre helps. Because the
//! second aircraft's choice is made knowing the first's, the two come out
//! complementary (one climbs, the other descends or holds) without a
//! separate coordination rule.
//!
//! An aircraft with the right of way is never advised: its conflicts are
//! resolved by everyone else, or left over if no one else can.

use std::collections::{BTreeMap, HashMap};

use serde::Serialize;

use crate::conflict::{can_meet, detect, encounters, risk, risk_for, urgency, Conflict, Encounter};
use crate::predict::{
    targets, AircraftState, Key, Manoeuvre, Plane, Prediction, Predictor, Targets, WindSource,
};
use crate::{Config, Error, Fleet};

/// What one aircraft is advised to do.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Advisory {
    pub aircraft: u64,
    pub manoeuvre: Manoeuvre,
    pub targets: Targets,
    /// The aircraft it was in conflict with.
    pub against: Vec<u64>,
    /// When its first conflict was, seconds from now.
    pub first_conflict_s: f64,
    /// Risk over everyone, before this advisory and after it.
    pub risk_before: f64,
    pub risk_after: f64,
    /// It is in no conflict once every advisory is followed.
    pub clear: bool,
}

/// Everything an assessment found.
#[derive(Debug, Clone, Serialize)]
pub struct Assessment {
    /// Conflicts with everyone carrying on as they are.
    pub conflicts: Vec<Conflict>,
    pub encounters: Vec<Encounter>,
    pub advisories: Vec<Advisory>,
    /// Conflicts left once every advisory is followed.
    pub residual: Vec<Conflict>,
    pub risk_before: f64,
    pub risk_after: f64,
    /// Each aircraft's prediction under what it is advised (or maintain).
    #[serde(skip)]
    pub predictions: Vec<Prediction>,
    /// Each aircraft's prediction carrying on as it is: where the conflicts
    /// the advisories answer come from.
    #[serde(skip)]
    pub unadvised: Vec<Prediction>,
}

/// What is in one key: the summed probability, how many aircraft, and how
/// many of them protected (counted on the protected plane only).
#[derive(Debug, Clone, Copy, Default)]
struct Tally {
    total: f64,
    aircraft: u32,
    protected: u32,
}

/// Summed probabilities and contributor counts, per step and key, kept up to
/// date as predictions are swapped, so a candidate manoeuvre is scored by
/// the cells it touches alone.
struct Board<'c> {
    cfg: &'c Config,
    // Ordered, so the risk is summed in the same order every time and an
    // assessment repeats to the bit.
    steps: Vec<BTreeMap<Key, Tally>>,
    times: Vec<f64>,
}

/// Whether `p` counts as protected in `key`.
fn guards(p: &Prediction, key: &Key) -> u32 {
    u32::from(p.protected && key.plane == Plane::Protected)
}

impl<'c> Board<'c> {
    fn new<'p>(cfg: &'c Config, predictions: impl Iterator<Item = &'p Prediction>) -> Board<'c> {
        let mut board = Board {
            cfg,
            steps: vec![BTreeMap::new(); cfg.steps],
            times: vec![0.0; cfg.steps],
        };
        for p in predictions {
            board.add(p, true);
        }
        board
    }

    fn add(&mut self, p: &Prediction, adding: bool) {
        for (s, cells) in p.cells.iter().enumerate() {
            self.times[s] = p.times_s[s];
            for (k, v) in cells {
                let e = self.steps[s].entry(*k).or_default();
                if adding {
                    e.total += v;
                    e.aircraft += 1;
                    e.protected += guards(p, k);
                } else {
                    e.total -= v;
                    e.aircraft -= 1;
                    e.protected -= guards(p, k);
                    if e.aircraft == 0 {
                        self.steps[s].remove(k);
                    }
                }
            }
        }
    }

    /// What a key adds to the risk with this tally.
    fn contrib(&self, key: &Key, t: Tally, step: usize) -> f64 {
        if can_meet(key.plane, t.aircraft, t.protected) && t.total >= self.cfg.threshold - 1e-9 {
            t.total * urgency(self.times[step], self.cfg)
        } else {
            0.0
        }
    }

    fn risk(&self) -> f64 {
        let mut sum = 0.0;
        for (s, cells) in self.steps.iter().enumerate() {
            for (k, t) in cells {
                sum += self.contrib(k, *t, s);
            }
        }
        sum
    }

    /// The risk in the keys an aircraft's prediction covers.
    fn risk_for(&self, p: &Prediction) -> f64 {
        let mut sum = 0.0;
        for (s, cells) in p.cells.iter().enumerate() {
            for k in cells.keys() {
                if let Some(t) = self.steps[s].get(k) {
                    sum += self.contrib(k, *t, s);
                }
            }
        }
        sum
    }

    /// How the risk changes if `old` (on the board) is swapped for `new`.
    fn delta(&self, old: &Prediction, new: &Prediction) -> f64 {
        let mut d = 0.0;
        for s in 0..self.steps.len() {
            let (o, n) = (&old.cells[s], &new.cells[s]);
            for (k, p_old) in o {
                let t = self.steps[s].get(k).copied().unwrap_or_default();
                let p_new = n.get(k).copied();
                let stays = u32::from(p_new.is_some());
                let after = Tally {
                    total: t.total - p_old + p_new.unwrap_or(0.0),
                    aircraft: t.aircraft - 1 + stays,
                    protected: t.protected - guards(old, k) + stays * guards(new, k),
                };
                d += self.contrib(k, after, s) - self.contrib(k, t, s);
            }
            for (k, p_new) in n {
                if o.contains_key(k) {
                    continue;
                }
                let t = self.steps[s].get(k).copied().unwrap_or_default();
                let after = Tally {
                    total: t.total + p_new,
                    aircraft: t.aircraft + 1,
                    protected: t.protected + guards(new, k),
                };
                d += self.contrib(k, after, s) - self.contrib(k, t, s);
            }
        }
        d
    }
}

/// Predict every aircraft, find the conflicts, and advise.
pub fn assess(
    states: &[AircraftState],
    fleet: &Fleet,
    wind: &dyn WindSource,
    cfg: &Config,
) -> Result<Assessment, Error> {
    cfg.check()?;
    let mut ids = std::collections::BTreeSet::new();
    for s in states {
        s.check()?;
        if !ids.insert(s.id) {
            return Err(Error::DuplicateId(s.id));
        }
        if !fleet.contains_key(&s.performance) {
            return Err(Error::UnknownPerformance(s.performance.clone()));
        }
    }
    let perf = |s: &AircraftState| &fleet[&s.performance];
    let mut predictor = Predictor::new(cfg, wind);
    let mut current: BTreeMap<u64, Prediction> = states
        .iter()
        .map(|s| (s.id, predictor.predict(s, perf(s), Manoeuvre::Maintain)))
        .collect();
    let all = |m: &BTreeMap<u64, Prediction>| -> Vec<Conflict> {
        let refs: Vec<&Prediction> = m.values().collect();
        detect(&refs, cfg)
    };

    let first = all(&current);
    let unadvised: Vec<Prediction> = current.values().cloned().collect();
    let mut conflicts = first.clone();
    let mut board = Board::new(cfg, current.values());
    let risk_before = board.risk();
    let mut advisories: Vec<Advisory> = Vec::new();
    // A manoeuvre's prediction does not depend on anyone else's, so each is
    // flown once however many rounds consider it.
    let mut tried: HashMap<(u64, usize), Prediction> = HashMap::new();
    let most = states.len().min(cfg.max_advisories);

    while advisories.len() < most {
        let base = board.risk();
        if base <= 0.0 {
            break;
        }
        let mut best: Option<(f64, &AircraftState, usize)> = None;
        for s in states {
            if s.right_of_way
                || advisories.iter().any(|a| a.aircraft == s.id)
                || board.risk_for(&current[&s.id]) <= 0.0
            {
                continue;
            }
            for (i, m) in Manoeuvre::CANDIDATES.iter().enumerate() {
                if targets(s, perf(s), *m, cfg).is_none() {
                    continue;
                }
                let trial = tried
                    .entry((s.id, i))
                    .or_insert_with(|| predictor.predict(s, perf(s), *m));
                let score = base + board.delta(&current[&s.id], trial) + m.cost() * base;
                if best.as_ref().is_none_or(|b| score < b.0) {
                    best = Some((score, s, i));
                }
            }
        }
        let Some((score, s, i)) = best else { break };
        // Only an advisory that helps beyond what it costs.
        if score >= base * 0.98 {
            break;
        }
        let trial = tried.remove(&(s.id, i)).expect("tried above");
        let mine: Vec<&Conflict> = conflicts
            .iter()
            .filter(|c| c.aircraft.iter().any(|(a, _)| *a == s.id))
            .collect();
        let mut against: Vec<u64> = mine.iter().flat_map(|c| c.against(s.id)).collect();
        against.sort_unstable();
        against.dedup();
        let first_conflict_s = mine.iter().map(|c| c.t_s).fold(f64::INFINITY, f64::min);
        board.add(&current[&s.id], false);
        board.add(&trial, true);
        advisories.push(Advisory {
            aircraft: s.id,
            manoeuvre: trial.manoeuvre,
            targets: trial.targets,
            against,
            first_conflict_s,
            risk_before: base,
            risk_after: board.risk(),
            clear: false,
        });
        current.insert(s.id, trial);
        conflicts = all(&current);
    }

    for a in advisories.iter_mut() {
        a.clear = risk_for(&conflicts, a.aircraft, cfg) == 0.0;
    }
    Ok(Assessment {
        encounters: encounters(&first),
        conflicts: first,
        advisories,
        risk_before,
        risk_after: risk(&conflicts, cfg),
        residual: conflicts,
        predictions: current.into_values().collect(),
        unadvised,
    })
}
