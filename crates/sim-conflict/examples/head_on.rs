//! Two encounters, assessed and advised: drones nose to nose at the same
//! height, and airliners 1 000 ft apart in the 2 000 ft layers above FL290.
//!
//! ```sh
//! cargo run --release -p sim-conflict --example head_on
//! ```

use sim_conflict::{assess, AircraftState, Assessment, Calm, Config, Fleet, Performance, FT};

fn state(id: u64, lon: f64, alt_m: f64, tas: f64, hdg: f64, perf: &str) -> AircraftState {
    AircraftState {
        id,
        lat: 54.9,
        lon,
        alt_m,
        tas_ms: tas,
        heading_deg: hdg,
        vs_ms: 0.0,
        turn_rate_dps: 0.0,
        mass_kg: None,
        performance: perf.into(),
    }
}

fn report(title: &str, a: &Assessment) {
    println!("{title}");
    for e in &a.encounters {
        println!(
            "  {} and {}: first conflict in {:.0} s, peak {:.2} aircraft in a cell, {} cells",
            e.a, e.b, e.first_s, e.peak_total, e.cells
        );
    }
    for adv in &a.advisories {
        println!(
            "  advise {}: {:?} {} (risk {:.2} → {:.2}{})",
            adv.aircraft,
            adv.manoeuvre,
            serde_json::to_string(&adv.targets).unwrap(),
            adv.risk_before,
            adv.risk_after,
            if adv.clear { ", clear" } else { "" }
        );
    }
    println!(
        "  risk {:.2} → {:.2}, {} conflicts left\n",
        a.risk_before,
        a.risk_after,
        a.residual.len()
    );
}

fn main() {
    let fleet = Fleet::from([
        ("quad".to_owned(), Performance::generic_quadcopter()),
        ("jet".to_owned(), Performance::generic_narrowbody()),
    ]);

    // 1.2 km apart at 60 m, 10 m/s each way.
    let drones = [
        state(1, 23.9, 60.0, 10.0, 90.0, "quad"),
        state(2, 23.9 + 0.018_76, 60.0, 10.0, 270.0, "quad"),
    ];
    let t = std::time::Instant::now();
    let a = assess(&drones, &fleet, &Calm, &Config::drones()).unwrap();
    report(
        &format!(
            "Drones head-on at 60 m ({:.0} ms)",
            t.elapsed().as_secs_f64() * 1e3
        ),
        &a,
    );

    // 60 km apart at FL330 and FL340, 450 kt each way.
    let jets = [
        state(10, 23.9, 33_000.0 * FT, 231.5, 90.0, "jet"),
        state(11, 23.9 + 0.938, 34_000.0 * FT, 231.5, 270.0, "jet"),
    ];
    let t = std::time::Instant::now();
    let a = assess(&jets, &fleet, &Calm, &Config::airliners()).unwrap();
    report(
        &format!(
            "Airliners head-on at FL330 and FL340 ({:.0} ms)",
            t.elapsed().as_secs_f64() * 1e3
        ),
        &a,
    );
}
