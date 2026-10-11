//! The data behind the figures in `docs/figures/conflict-*.svg`, as JSON on
//! stdout. Everything comes from the crate itself, so the figures show what
//! it does, not a sketch of it.
//!
//! ```sh
//! cargo run --release -p sim-conflict --example figures > target/conflict-figures.json
//! python3 tools/conflict_figures.py target/conflict-figures.json docs/figures
//! ```

use serde_json::{json, Value};
use sim_conflict::conflict::{detect, heat, risk};
use sim_conflict::predict::targets;
use sim_conflict::{
    assess, AircraftState, Calm, Config, Fleet, Layers, Manoeuvre, Performance, Prediction,
    Predictor, FT,
};
use sim_core::GeoCell;

fn quad(id: u64, lat: f64, lon: f64, hdg: f64, turn: f64) -> AircraftState {
    AircraftState {
        id,
        lat,
        lon,
        alt_m: 60.0,
        tas_ms: 10.0,
        heading_deg: hdg,
        vs_ms: 0.0,
        turn_rate_dps: turn,
        mass_kg: None,
        performance: "quad".into(),
        protection: None,
        give_way: sim_conflict::GiveWay::First,
    }
}

fn outline(cell: GeoCell) -> Value {
    let ring: Vec<[f64; 2]> = cell
        .boundary()
        .unwrap_or_default()
        .into_iter()
        .map(|(lon, lat)| [lon, lat])
        .collect();
    json!(ring)
}

/// One step of a prediction: each cell (any layer) with the most any layer
/// of it holds, and its outline.
fn cells_at(p: &Prediction, step: usize) -> Value {
    let mut best: std::collections::BTreeMap<GeoCell, f64> = Default::default();
    for (k, v) in &p.cells[step] {
        let e = best.entry(k.cell).or_insert(0.0);
        *e = e.max(*v);
    }
    best.into_iter()
        .map(|(c, v)| json!({ "p": v, "ring": outline(c) }))
        .collect()
}

fn percentile(sorted: &[f64], q: f64) -> f64 {
    let i = ((sorted.len() - 1) as f64 * q).round() as usize;
    sorted[i]
}

/// Altitude band of a traced cloud at every step: 5th, 50th, 95th
/// percentile, with t = 0 at the reported state.
fn altitude_band(state: &AircraftState, p: &Prediction, clouds: &[Vec<[f64; 3]>]) -> Value {
    let mut rows = vec![json!([0.0, state.alt_m, state.alt_m, state.alt_m])];
    for (t, cloud) in p.times_s.iter().zip(clouds) {
        let mut alts: Vec<f64> = cloud.iter().map(|q| q[2]).collect();
        alts.sort_by(f64::total_cmp);
        rows.push(json!([
            t,
            percentile(&alts, 0.05),
            percentile(&alts, 0.5),
            percentile(&alts, 0.95)
        ]));
    }
    json!(rows)
}

fn main() {
    let quad_perf = Performance::generic_quadcopter();
    let jet_perf = Performance::generic_narrowbody();
    let fleet = Fleet::from([
        ("quad".to_owned(), quad_perf.clone()),
        ("jet".to_owned(), jet_perf.clone()),
    ]);
    let cfg = Config::drones();

    // 1. One drone in a gentle turn: its cloud, and that cloud in cells at
    //    two resolutions.
    let lone = quad(1, 54.9, 23.9, 60.0, 1.0);
    let (pred, clouds) = Predictor::new(&cfg, &Calm).trace(&lone, &quad_perf, Manoeuvre::Maintain);
    let shown = [0usize, 3, 7, 11];
    let cloud = json!({
        "start": [lone.lat, lone.lon],
        "mean": pred.mean.iter().map(|m| [m[0], m[1]]).collect::<Vec<_>>(),
        "times": pred.times_s,
        "steps": shown.iter().map(|&s| json!({
            "t": pred.times_s[s],
            "particles": clouds[s].iter().map(|q| [q[0], q[1]]).collect::<Vec<_>>(),
        })).collect::<Vec<_>>(),
    });
    let at = 5; // 30 s ahead
    let mut cells = Vec::new();
    for res in [15, 17] {
        let c = Config {
            resolution: res,
            ..Config::drones()
        };
        let (p, cl) = Predictor::new(&c, &Calm).trace(&lone, &quad_perf, Manoeuvre::Maintain);
        cells.push(json!({
            "resolution": res,
            "edge_m": sim_core::cell::nominal_cell_edge_meters(res),
            "t": p.times_s[at],
            "cells": cells_at(&p, at),
            "particles": cl[at].iter().map(|q| [q[0], q[1]]).collect::<Vec<_>>(),
            "mean": [p.mean[at][0], p.mean[at][1]],
        }));
    }

    // 2. The head-on encounter from the `head_on` example: detection,
    //    the first round of the advisory search, and the result.
    let pair = [
        quad(1, 54.9, 23.9, 90.0, 0.0),
        quad(2, 54.9, 23.9 + 0.018_76, 270.0, 0.0),
    ];
    let mut predictor = Predictor::new(&cfg, &Calm);
    let maintain: Vec<Prediction> = pair
        .iter()
        .map(|s| predictor.predict(s, &quad_perf, Manoeuvre::Maintain))
        .collect();
    let refs: Vec<&Prediction> = maintain.iter().collect();
    let conflicts = detect(&refs, &cfg);
    let base = risk(&conflicts, &cfg);
    let heat_steps: Vec<Value> = [3usize, 7, 11]
        .iter()
        .map(|&s| {
            let one: Vec<Prediction> = maintain
                .iter()
                .map(|p| Prediction {
                    times_s: vec![p.times_s[s]],
                    cells: vec![p.cells[s].clone()],
                    mean: vec![p.mean[s]],
                    ..p.clone()
                })
                .collect();
            let hs: Vec<&Prediction> = one.iter().collect();
            let in_conflict: std::collections::BTreeSet<GeoCell> = conflicts
                .iter()
                .filter(|c| c.step == s)
                .map(|c| c.cell)
                .collect();
            json!({
                "t": maintain[0].times_s[s],
                "means": maintain.iter().map(|p| [p.mean[s][0], p.mean[s][1]]).collect::<Vec<_>>(),
                "cells": heat(&hs).into_iter().map(|h| json!({
                    "summed": h.summed,
                    "conflict": in_conflict.contains(&h.cell),
                    "ring": outline(h.cell),
                })).collect::<Vec<_>>(),
            })
        })
        .collect();

    // Round one of the search: every manoeuvre of every aircraft in
    // conflict, scored as `assess` scores it.
    let mut candidates = Vec::new();
    for (i, s) in pair.iter().enumerate() {
        for m in Manoeuvre::CANDIDATES {
            if targets(s, &quad_perf, m, &cfg).is_none() {
                candidates.push(json!({ "aircraft": s.id, "manoeuvre": m, "offered": false }));
                continue;
            }
            let trial = predictor.predict(s, &quad_perf, m);
            let mut set: Vec<&Prediction> = refs.clone();
            set[i] = &trial;
            let left = risk(&detect(&set, &cfg), &cfg);
            candidates.push(json!({
                "aircraft": s.id, "manoeuvre": m, "offered": true,
                "risk_left": left, "cost": m.cost() * base, "score": left + m.cost() * base,
            }));
        }
    }

    let assessment = assess(&pair, &fleet, &Calm, &cfg).unwrap();
    let mut profiles = Vec::new();
    for s in &pair {
        let advised = assessment
            .advisories
            .iter()
            .find(|a| a.aircraft == s.id)
            .map_or(Manoeuvre::Maintain, |a| a.manoeuvre);
        let (pm, cm) = predictor.trace(s, &quad_perf, Manoeuvre::Maintain);
        let (pa, ca) = predictor.trace(s, &quad_perf, advised);
        profiles.push(json!({
            "aircraft": s.id,
            "advised": advised,
            "maintain": altitude_band(s, &pm, &cm),
            "after": altitude_band(s, &pa, &ca),
        }));
    }
    let layer_edges: Vec<f64> = (0..8).map(|l| cfg.layers.bounds(l).0).collect();
    let encounter = json!({
        "separation_m": 1200.0,
        "starts": pair.iter().map(|s| [s.lat, s.lon]).collect::<Vec<_>>(),
        "conflicts": conflicts.iter().map(|c| json!({
            "t": c.t_s, "total": c.total, "layer": c.layer,
        })).collect::<Vec<_>>(),
        "heat": heat_steps,
        "risk_before": base,
        "threshold": base * 0.98,
        "candidates": candidates,
        "advisories": assessment.advisories,
        "risk_after": assessment.risk_after,
        "residual": assessment.residual.len(),
        "profiles": profiles,
        "layer_edges_m": layer_edges,
        "vertical_buffer_m": cfg.vertical_buffer_m,
        "urgency_s": cfg.urgency_s,
    });

    // 3. Altitude layers: the airspace's standard ones round FL290, and the
    //    drones' 30 m ones.
    let standard = Layers::standard();
    let around: Vec<Value> = (20..=40)
        .filter_map(|l| {
            let (lo, hi) = standard.bounds(l);
            (lo >= 24_000.0 * FT && hi <= 38_000.0 * FT).then(|| json!([lo / FT, hi / FT]))
        })
        .collect();
    let drone_layers: Vec<Value> = (0..6)
        .map(|l| {
            let (lo, hi) = cfg.layers.bounds(l);
            json!([lo, hi])
        })
        .collect();

    // 4. The energy models' climb envelopes.
    let quad_mass = quad_perf.default_mass_kg();
    let max_speed = quad_perf.speed_envelope(60.0).1;
    let quad_curve: Vec<[f64; 3]> = (0..=40)
        .map(|i| {
            let v = max_speed * i as f64 / 40.0;
            [
                v,
                quad_perf.max_climb_ms(60.0, v, quad_mass),
                quad_perf.max_descent_ms(60.0, v, quad_mass),
            ]
        })
        .collect();
    let Performance::FixedWing(jet) = &jet_perf else {
        unreachable!()
    };
    let jet_mass = jet_perf.default_mass_kg();
    let jet_curve: Vec<[f64; 3]> = (0..=48)
        .map(|i| {
            let alt = jet.ceiling_m * i as f64 / 48.0;
            let tas = jet_perf.scheduled_tas(jet.schedule_cas_ms, jet.schedule_mach, alt);
            [
                alt / FT,
                jet_perf.max_climb_ms(alt, tas, jet_mass),
                jet_perf.max_descent_ms(alt, tas, jet_mass),
            ]
        })
        .collect();

    // Where the schedule changes from constant CAS to constant Mach: the
    // first height at which the Mach speed is the lower.
    let crossover_m = (0..=4000)
        .map(|i| i as f64 * 10.0)
        .find(|&alt| {
            jet.schedule_mach * sim_conflict::atmosphere::isa(alt).sound_ms
                < sim_conflict::atmosphere::cas_to_tas(jet.schedule_cas_ms, alt)
        })
        .unwrap_or(f64::NAN);

    // 7. A manned aircraft's protected volume on the grid: which cells and
    //    layers it puts itself in, beside a drone's own zone, and how far
    //    out the grid actually counts a drone as inside it.
    let protection = {
        let (lat, lon, alt) = (54.9, 23.9, 1_000.0);
        let (lateral, vertical) = (300.0, 150.0);
        let res = cfg.resolution;
        let offset = |east: f64, north: f64| {
            let r = 6_371_000.0;
            (
                lat + (north / r).to_degrees(),
                lon + (east / (r * lat.to_radians().cos())).to_degrees(),
            )
        };
        let cell_at = |la: f64, lo: f64| GeoCell::from_lonlat(lo, la, res).unwrap();
        let centre = cell_at(lat, lon);
        let volume = sim_conflict::protected_cells(centre, lat, lon, lateral);
        let around: Vec<Value> = centre
            .spherical_cap(lateral + 450.0)
            .unwrap()
            .into_iter()
            .map(|c| json!({ "ring": outline(c), "in": volume.contains(&c) }))
            .collect();
        // A drone inside the volume and one just outside it, the outside one
        // with the zone it keeps from other drones (its cell and one ring).
        let drone = |east: f64, north: f64, dz: f64, zone: bool| {
            let (la, lo) = offset(east, north);
            let c = cell_at(la, lo);
            json!({
                "at": [la, lo], "alt_m": alt + dz, "east": east, "north": north,
                "distance_m": (east * east + north * north).sqrt(),
                "cell": outline(c), "in": volume.contains(&c),
                "zone": if zone { c.grid_disk(cfg.buffer_rings).unwrap().into_iter().map(outline).collect::<Vec<_>>() } else { Vec::new() },
            })
        };
        // How far out a drone's cell is counted, every 5° round and every
        // 2 m out: the nearest it is not, and the farthest it still is.
        let (mut nearest_out, mut farthest_in) = (f64::INFINITY, 0.0_f64);
        for b in (0..72).map(|i| (i as f64 * 5.0).to_radians()) {
            for d in (50..300).map(|i| i as f64 * 2.0) {
                let (la, lo) = offset(d * b.sin(), d * b.cos());
                if volume.contains(&cell_at(la, lo)) {
                    farthest_in = farthest_in.max(d);
                } else {
                    nearest_out = nearest_out.min(d);
                }
            }
        }
        let layers = &cfg.layers;
        let (low, high) = (layers.index(alt - vertical), layers.index(alt + vertical));
        json!({
            "at": [lat, lon], "alt_m": alt, "resolution": res,
            "edge_m": sim_core::cell::nominal_cell_edge_meters(res),
            "lateral_m": lateral, "vertical_m": vertical,
            "cells": around, "volume_cells": volume.len(),
            "drones": [drone(170.0, -150.0, -80.0, false), drone(420.0, 160.0, -200.0, true)],
            "nearest_out_m": nearest_out, "farthest_in_m": farthest_in,
            "layers": (layers.index(alt - 260.0)..=layers.index(alt + 260.0))
                .map(|l| { let (a, b) = layers.bounds(l); json!({ "from": a, "to": b, "in": (low..=high).contains(&l) }) })
                .collect::<Vec<_>>(),
            "covered_m": [layers.bounds(low).0, layers.bounds(high).1],
        })
    };

    let out = json!({
        "protection": protection,
        "cloud": cloud,
        "cells": cells,
        "encounter": encounter,
        "layers": { "standard_ft": around, "drones_m": drone_layers },
        "energy": {
            "quad": { "name": quad_perf.name(), "alt_m": 60.0, "rows": quad_curve },
            "jet": { "name": jet_perf.name(), "rows": jet_curve,
                     "schedule_cas_ms": jet.schedule_cas_ms, "schedule_mach": jet.schedule_mach,
                     "crossover_ft": crossover_m / FT, "tropopause_ft": sim_conflict::atmosphere::H_TROP / FT },
        },
        "config": {
            "particles": cfg.particles, "step_s": cfg.step_s, "steps": cfg.steps,
            "buffer_rings": cfg.buffer_rings, "threshold": cfg.threshold,
        },
    });
    println!("{}", serde_json::to_string(&out).unwrap());
}
