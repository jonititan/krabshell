//! Conflicts: cells where the aircraft predicted to be there add up to too
//! many.
//!
//! At each step, each cell and layer's probabilities are summed over every
//! aircraft. That sum is the number of aircraft expected there. A cell is in
//! conflict when at least two aircraft put something into it and the sum
//! reaches the threshold. Since one aircraft puts at most 1 into a cell, a
//! threshold of 1 or more never flags an aircraft against itself.

use std::collections::BTreeMap;

use serde::Serialize;
use sim_core::GeoCell;

use crate::predict::{Key, Prediction};
use crate::Config;

/// A cell and layer, at a step, with more aircraft expected than allowed.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Conflict {
    pub step: usize,
    /// Seconds from now.
    pub t_s: f64,
    pub cell: GeoCell,
    pub layer: i32,
    /// The layer's altitudes above mean sea level, m.
    pub alt_from_m: f64,
    pub alt_to_m: f64,
    /// The summed probability: aircraft expected here.
    pub total: f64,
    /// Each aircraft's probability, highest first.
    pub aircraft: Vec<(u64, f64)>,
}

/// Every conflict among the predictions, soonest first.
pub fn detect(predictions: &[&Prediction], cfg: &Config) -> Vec<Conflict> {
    let steps = predictions.iter().map(|p| p.cells.len()).max().unwrap_or(0);
    let mut out = Vec::new();
    for step in 0..steps {
        let mut sums: BTreeMap<Key, (f64, Vec<(u64, f64)>)> = BTreeMap::new();
        for p in predictions {
            let Some(cells) = p.cells.get(step) else {
                continue;
            };
            for (key, prob) in cells {
                let e = sums.entry(*key).or_default();
                e.0 += prob;
                e.1.push((p.id, *prob));
            }
        }
        let t_s = predictions
            .iter()
            .find_map(|p| p.times_s.get(step))
            .copied()
            .unwrap_or(0.0);
        for (key, (total, mut aircraft)) in sums {
            if aircraft.len() < 2 || total < cfg.threshold {
                continue;
            }
            aircraft.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
            out.push(Conflict {
                step,
                t_s,
                cell: key.cell,
                layer: key.layer,
                alt_from_m: cfg.layers.bounds(key.layer).0,
                alt_to_m: cfg.layers.bounds(key.layer).1,
                total,
                aircraft,
            });
        }
    }
    out
}

/// One A5 cell's worst moment: the altitude layer and step where the most
/// aircraft could be together.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Hotspot {
    pub cell: GeoCell,
    pub layer: i32,
    pub alt_from_m: f64,
    pub alt_to_m: f64,
    pub step: usize,
    pub t_s: f64,
    /// The summed probability there: aircraft expected.
    pub summed: f64,
    /// The chance two or more are there together, taking the aircraft as
    /// independent: `1 − Π(1 − pᵢ) − Σ pᵢ Π_{j≠i}(1 − pⱼ)`.
    pub collision: f64,
    /// It reaches the conflict threshold.
    pub conflict: bool,
    /// Each aircraft's probability, highest first.
    pub aircraft: Vec<(u64, f64)>,
}

/// The chance at least two of several independent events happen.
pub fn two_or_more(probabilities: impl IntoIterator<Item = f64>) -> f64 {
    // P(none) and P(exactly one), built up one aircraft at a time.
    let (mut none, mut one) = (1.0, 0.0);
    for p in probabilities {
        let p = p.clamp(0.0, 1.0);
        one = one * (1.0 - p) + none * p;
        none *= 1.0 - p;
    }
    (1.0 - none - one).max(0.0)
}

/// Every A5 cell two or more aircraft could share, at its worst layer and
/// step (by summed probability), worst first.
pub fn hotspots(predictions: &[&Prediction], cfg: &Config) -> Vec<Hotspot> {
    let steps = predictions.iter().map(|p| p.cells.len()).max().unwrap_or(0);
    let mut worst: BTreeMap<GeoCell, Hotspot> = BTreeMap::new();
    for step in 0..steps {
        let mut sums: BTreeMap<Key, (f64, Vec<(u64, f64)>)> = BTreeMap::new();
        for p in predictions {
            let Some(cells) = p.cells.get(step) else {
                continue;
            };
            for (key, prob) in cells {
                let e = sums.entry(*key).or_default();
                e.0 += prob;
                e.1.push((p.id, *prob));
            }
        }
        let t_s = predictions
            .iter()
            .find_map(|p| p.times_s.get(step))
            .copied()
            .unwrap_or(0.0);
        for (key, (summed, mut aircraft)) in sums {
            if aircraft.len() < 2 {
                continue;
            }
            if worst.get(&key.cell).is_some_and(|h| h.summed >= summed) {
                continue;
            }
            aircraft.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
            let (alt_from_m, alt_to_m) = cfg.layers.bounds(key.layer);
            worst.insert(
                key.cell,
                Hotspot {
                    cell: key.cell,
                    layer: key.layer,
                    alt_from_m,
                    alt_to_m,
                    step,
                    t_s,
                    summed,
                    collision: two_or_more(aircraft.iter().map(|(_, p)| *p)),
                    conflict: summed >= cfg.threshold,
                    aircraft,
                },
            );
        }
    }
    let mut out: Vec<Hotspot> = worst.into_values().collect();
    out.sort_by(|a, b| b.summed.total_cmp(&a.summed).then(a.cell.cmp(&b.cell)));
    out
}

/// How bad a set of conflicts is: each one's excess over a single aircraft,
/// weighted toward the near ones, which leave less time to act.
pub fn risk(conflicts: &[Conflict], cfg: &Config) -> f64 {
    conflicts
        .iter()
        .map(|c| c.total * urgency(c.t_s, cfg))
        .fold(0.0, |sum, r| sum + r)
}

/// The part of [`risk`] an aircraft takes part in.
pub fn risk_for(conflicts: &[Conflict], id: u64, cfg: &Config) -> f64 {
    conflicts
        .iter()
        .filter(|c| c.aircraft.iter().any(|(a, _)| *a == id))
        .map(|c| c.total * urgency(c.t_s, cfg))
        .fold(0.0, |sum, r| sum + r)
}

pub(crate) fn urgency(t_s: f64, cfg: &Config) -> f64 {
    1.0 / (1.0 + t_s / cfg.urgency_s.max(1.0))
}

/// Two aircraft in conflict, summed over the cells they share.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Encounter {
    pub a: u64,
    pub b: u64,
    /// When they first conflict, seconds from now.
    pub first_s: f64,
    /// The highest summed probability of any cell they share.
    pub peak_total: f64,
    /// Conflicting cells and steps they share.
    pub cells: usize,
}

/// The pairs in conflict, soonest first.
pub fn encounters(conflicts: &[Conflict]) -> Vec<Encounter> {
    let mut pairs: BTreeMap<(u64, u64), Encounter> = BTreeMap::new();
    for c in conflicts {
        for (i, (a, _)) in c.aircraft.iter().enumerate() {
            for (b, _) in &c.aircraft[i + 1..] {
                let (a, b) = if a < b { (*a, *b) } else { (*b, *a) };
                let e = pairs.entry((a, b)).or_insert(Encounter {
                    a,
                    b,
                    first_s: c.t_s,
                    peak_total: 0.0,
                    cells: 0,
                });
                e.first_s = e.first_s.min(c.t_s);
                e.peak_total = e.peak_total.max(c.total);
                e.cells += 1;
            }
        }
    }
    let mut out: Vec<Encounter> = pairs.into_values().collect();
    out.sort_by(|x, y| {
        x.first_s
            .total_cmp(&y.first_s)
            .then((x.a, x.b).cmp(&(y.a, y.b)))
    });
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::predict::{Manoeuvre, Targets};

    fn pred(id: u64, cell: GeoCell, p: f64) -> Prediction {
        Prediction {
            id,
            manoeuvre: Manoeuvre::Maintain,
            targets: Targets::default(),
            times_s: vec![5.0],
            cells: vec![BTreeMap::from([(Key { cell, layer: 2 }, p)])],
            mean: vec![[0.0; 3]],
        }
    }

    #[test]
    fn probabilities_add_up_and_one_aircraft_never_conflicts_alone() {
        let cell = GeoCell::from_lonlat(23.9, 54.9, 15).unwrap();
        let cfg = crate::Config::drones();
        let (a, b) = (pred(1, cell, 0.6), pred(2, cell, 0.5));
        let c = detect(&[&a, &b], &cfg);
        assert_eq!(c.len(), 1);
        assert!((c[0].total - 1.1).abs() < 1e-12);
        assert_eq!(c[0].aircraft, vec![(1, 0.6), (2, 0.5)]);
        // Below the threshold.
        let hot = hotspots(&[&a, &b], &cfg);
        assert_eq!(hot.len(), 1);
        assert!((hot[0].collision - 0.3).abs() < 1e-12);
        assert!(hot[0].conflict);
        let low = pred(2, cell, 0.3);
        assert!(detect(&[&a, &low], &cfg).is_empty());
        // Alone, even with certainty.
        let sure = pred(3, cell, 1.0);
        assert!(detect(&[&sure], &cfg).is_empty());
        let e = encounters(&detect(&[&a, &b], &cfg));
        assert_eq!((e[0].a, e[0].b, e[0].cells), (1, 2, 1));
    }

    #[test]
    fn two_or_more_is_the_chance_of_company() {
        assert_eq!(two_or_more([1.0, 1.0]), 1.0);
        assert_eq!(two_or_more([1.0, 0.0]), 0.0);
        assert!((two_or_more([0.5, 0.5]) - 0.25).abs() < 1e-12);
        // Three at a half: 1 − 1/8 − 3/8.
        assert!((two_or_more([0.5, 0.5, 0.5]) - 0.5).abs() < 1e-12);
    }
}
