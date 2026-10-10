//! Whole encounters through `assess`: what is a conflict, and what is
//! advised.

use sim_conflict::{
    assess, AircraftState, Calm, Config, Fleet, GiveWay, Manoeuvre, Performance, Protection, FT,
};

const LAT: f64 = 54.9;
const LON: f64 = 23.9;
const KT: f64 = 0.514_444;

fn fleet() -> Fleet {
    Fleet::from([
        ("quad".to_owned(), Performance::generic_quadcopter()),
        ("jet".to_owned(), Performance::generic_narrowbody()),
    ])
}

/// `east_m` east of the reference point.
fn east(east_m: f64) -> f64 {
    LON + (east_m / 6_371_000.0).to_degrees() / LAT.to_radians().cos()
}

#[allow(clippy::too_many_arguments)]
fn aircraft(
    id: u64,
    east_m: f64,
    alt_m: f64,
    tas: f64,
    hdg: f64,
    vs: f64,
    perf: &str,
) -> AircraftState {
    AircraftState {
        id,
        lat: LAT,
        lon: east(east_m),
        alt_m,
        tas_ms: tas,
        heading_deg: hdg,
        vs_ms: vs,
        turn_rate_dps: 0.0,
        mass_kg: None,
        performance: perf.into(),
        protection: None,
        give_way: sim_conflict::GiveWay::First,
    }
}

/// Two quadcopters 1.2 km apart flying at each other at 10 m/s: they meet
/// in a minute.
fn head_on_drones(separation_m: f64) -> Vec<AircraftState> {
    vec![
        aircraft(1, 0.0, 60.0, 10.0, 90.0, 0.0, "quad"),
        aircraft(2, 1_200.0, 60.0 + separation_m, 10.0, 270.0, 0.0, "quad"),
    ]
}

#[test]
fn head_on_drones_conflict_and_are_told_to_part_vertically() {
    let cfg = Config::drones();
    let a = assess(&head_on_drones(0.0), &fleet(), &Calm, &cfg).unwrap();
    assert!(!a.conflicts.is_empty());
    let e = &a.encounters[0];
    assert_eq!((e.a, e.b), (1, 2));
    // The protected zones meet well before the aircraft do.
    assert!(e.first_s >= 25.0 && e.first_s <= 55.0, "{e:?}");
    assert!(e.peak_total >= 1.5, "{e:?}");

    assert!(!a.advisories.is_empty());
    for adv in &a.advisories {
        assert!(
            matches!(adv.manoeuvre, Manoeuvre::Climb | Manoeuvre::Descend),
            "vertical first: {adv:?}"
        );
        assert!(adv.risk_after < adv.risk_before);
    }
    // If both are advised, they go opposite ways.
    if let [x, y] = &a.advisories[..] {
        assert_ne!(x.manoeuvre, y.manoeuvre, "{:?}", a.advisories);
    }
    assert!(
        a.risk_after < 0.1 * a.risk_before,
        "{} → {}",
        a.risk_before,
        a.risk_after
    );
    assert!(a.advisories.iter().all(|x| x.clear), "{:?}", a.advisories);
}

#[test]
fn the_same_drones_three_layers_apart_do_not_conflict() {
    let a = assess(&head_on_drones(90.0), &fleet(), &Calm, &Config::drones()).unwrap();
    assert!(a.conflicts.is_empty(), "{:?}", a.encounters);
    assert!(a.advisories.is_empty());
}

#[test]
fn drones_flying_apart_do_not_conflict() {
    let states = vec![
        aircraft(1, 0.0, 60.0, 10.0, 270.0, 0.0, "quad"),
        aircraft(2, 1_200.0, 60.0, 10.0, 90.0, 0.0, "quad"),
    ];
    assert!(assess(&states, &fleet(), &Calm, &Config::drones())
        .unwrap()
        .conflicts
        .is_empty());
}

/// Two airliners 60 km apart, nose to nose at 450 kt TAS: they meet in a
/// little over two minutes.
fn head_on_jets(fl_a: f64, fl_b: f64) -> Vec<AircraftState> {
    vec![
        aircraft(10, 0.0, fl_a * 100.0 * FT, 450.0 * KT, 90.0, 0.0, "jet"),
        aircraft(
            11,
            60_000.0,
            fl_b * 100.0 * FT,
            450.0 * KT,
            270.0,
            0.0,
            "jet",
        ),
    ]
}

#[test]
fn jets_at_separated_levels_do_not_conflict_but_closer_ones_do() {
    let cfg = Config::airliners();
    let fleet = fleet();
    let conflicts = |a: f64, b: f64| {
        !assess(&head_on_jets(a, b), &fleet, &Calm, &cfg)
            .unwrap()
            .conflicts
            .is_empty()
    };
    // Below FL290, 1 000 ft separates; 500 ft does not.
    assert!(!conflicts(100.0, 110.0));
    assert!(conflicts(100.0, 105.0));
    assert!(conflicts(200.0, 200.0));
    // From FL290, 2 000 ft separates; 1 000 ft does not.
    assert!(!conflicts(330.0, 350.0));
    assert!(conflicts(330.0, 340.0));
    // Across the joint: FL280 and FL290 are 1 000 ft apart below and at
    // the bottom of the 2 000 ft band.
    assert!(!conflicts(280.0, 290.0));
}

#[test]
fn a_jet_conflict_gets_an_advisory_that_clears_it() {
    let cfg = Config::airliners();
    let a = assess(&head_on_jets(330.0, 340.0), &fleet(), &Calm, &cfg).unwrap();
    assert!(!a.advisories.is_empty());
    let first = &a.advisories[0];
    assert!(
        first.first_conflict_s > 60.0 && first.first_conflict_s < 150.0,
        "{first:?}"
    );
    assert!(
        a.risk_after < 0.1 * a.risk_before,
        "{} → {}",
        a.risk_before,
        a.risk_after
    );
    // A climb or descent asks for the advisory rate, 1 500 ft/min.
    if let Some(vs) = first.targets.vs_ms {
        assert!((vs.abs() - 7.62).abs() < 0.5, "{first:?}");
    }
}

#[test]
fn an_assessment_repeats_exactly() {
    let cfg = Config::drones();
    let a = assess(&head_on_drones(0.0), &fleet(), &Calm, &cfg).unwrap();
    let b = assess(&head_on_drones(0.0), &fleet(), &Calm, &cfg).unwrap();
    assert_eq!(a.advisories, b.advisories);
    assert_eq!(a.conflicts, b.conflicts);
}

#[test]
fn unknown_types_and_repeated_ids_are_refused() {
    let mut states = head_on_drones(0.0);
    states[1].performance = "glider".into();
    assert!(assess(&states, &fleet(), &Calm, &Config::drones()).is_err());
    let mut states = head_on_drones(0.0);
    states[1].id = 1;
    assert!(assess(&states, &fleet(), &Calm, &Config::drones()).is_err());
}

#[test]
fn the_running_risk_matches_detecting_from_scratch() {
    let cfg = Config::drones();
    let a = assess(&head_on_drones(0.0), &fleet(), &Calm, &cfg).unwrap();
    let scratch = sim_conflict::conflict::risk(&a.conflicts, &cfg);
    assert!(
        (a.risk_before - scratch).abs() < 1e-9,
        "{} vs {scratch}",
        a.risk_before
    );
    let after = sim_conflict::conflict::risk(&a.residual, &cfg);
    assert!((a.risk_after - after).abs() < 1e-9);
}

/// Eight drones converging on one point from all round, at one height.
#[test]
fn a_crowd_converging_is_advised_apart() {
    let cfg = Config::drones();
    let states: Vec<AircraftState> = (0..8)
        .map(|i| {
            let bearing = f64::from(i) * 45.0;
            let (e, n) = (
                600.0 * bearing.to_radians().sin(),
                600.0 * bearing.to_radians().cos(),
            );
            AircraftState {
                lat: LAT + (n / 6_371_000.0).to_degrees(),
                ..aircraft(
                    100 + u64::from(i as u8),
                    e,
                    60.0,
                    10.0,
                    (bearing + 180.0) % 360.0,
                    0.0,
                    "quad",
                )
            }
        })
        .collect();
    let t = std::time::Instant::now();
    let a = assess(&states, &fleet(), &Calm, &cfg).unwrap();
    let took = t.elapsed();
    assert!(a.encounters.len() >= 8, "{}", a.encounters.len());
    assert!(a.advisories.len() >= 4, "{:?}", a.advisories);
    assert!(
        a.risk_after < 0.5 * a.risk_before,
        "{} → {}",
        a.risk_before,
        a.risk_after
    );
    // Each advisory lowered the risk it was given against.
    for w in a.advisories.windows(2) {
        assert!(w[1].risk_before <= w[0].risk_before + 1e-9);
    }
    eprintln!(
        "8 converging: {took:?}, {} advisories, risk {:.1} → {:.1}",
        a.advisories.len(),
        a.risk_before,
        a.risk_after
    );
}

/// An airliner level at 1 000 m, 80 m/s eastbound, with 300 m and ±150 m
/// of protection, giving way last, and a drone hovering `ahead_m` up
/// its track, `north_m` off it and `dz_m` above it.
fn airliner_and_drone(ahead_m: f64, north_m: f64, dz_m: f64) -> Vec<AircraftState> {
    let mut jet = aircraft(10, 0.0, 1_000.0, 80.0, 90.0, 0.0, "jet");
    jet.protection = Some(Protection {
        lateral_m: 300.0,
        vertical_m: 150.0,
    });
    jet.give_way = GiveWay::Last;
    let mut drone = aircraft(1, ahead_m, 1_000.0 + dz_m, 0.0, 0.0, 0.0, "quad");
    drone.lat += (north_m / 6_371_000.0).to_degrees();
    vec![jet, drone]
}

#[test]
fn a_drone_inside_an_airliners_protected_volume_is_advised_before_the_airliner() {
    let cfg = Config::drones();
    // 30 s up the track, 80 m below: inside ±150 m, and too close for the
    // drone alone (2.5 m/s down) to get out in time.
    let a = assess(
        &airliner_and_drone(2_400.0, 0.0, -80.0),
        &fleet(),
        &Calm,
        &cfg,
    )
    .unwrap();
    let e = &a.encounters[0];
    assert_eq!((e.a, e.b), (1, 10), "{:?}", a.encounters);
    assert_eq!(
        a.advisories[0].aircraft, 1,
        "the drone first: {:?}",
        a.advisories
    );
    assert_eq!(a.advisories[0].against, vec![10]);
    assert!(a.risk_after < a.risk_before);
}

#[test]
fn an_airliner_is_left_alone_when_the_drone_can_get_out_of_the_way() {
    let cfg = Config::drones();
    // 45 s up the track, 120 m below: a short descent clears the volume.
    let a = assess(
        &airliner_and_drone(3_600.0, 0.0, -120.0),
        &fleet(),
        &Calm,
        &cfg,
    )
    .unwrap();
    assert!(!a.conflicts.is_empty());
    assert!(
        a.advisories.iter().any(|adv| adv.aircraft == 1),
        "{:?}",
        a.advisories
    );
    assert!(
        a.advisories.iter().all(|adv| adv.aircraft != 10),
        "{:?}",
        a.advisories
    );
    assert!(a.residual.is_empty(), "{:?}", a.residual);
}

#[test]
fn a_drone_outside_the_volume_is_no_conflict_with_the_airliner() {
    let cfg = Config::drones();
    for (what, north_m, dz_m) in [
        ("200 m below", 0.0, -200.0),
        ("200 m above", 0.0, 200.0),
        ("600 m off the track", 600.0, 0.0),
    ] {
        let a = assess(
            &airliner_and_drone(2_400.0, north_m, dz_m),
            &fleet(),
            &Calm,
            &cfg,
        )
        .unwrap();
        assert!(a.conflicts.is_empty(), "{what}: {:?}", a.encounters);
    }
    // Inside the lateral minimum, though, it is.
    let a = assess(
        &airliner_and_drone(2_400.0, 150.0, 0.0),
        &fleet(),
        &Calm,
        &cfg,
    )
    .unwrap();
    assert!(!a.conflicts.is_empty(), "150 m off the track");
}

#[test]
fn the_protected_volume_is_the_minima_not_the_minima_plus_the_drones_zone() {
    // Without protection the drone's own zone (a ring of ~200 m cells and
    // 15 m) is what it keeps from anything; 120 m below the jet's track it
    // is clear of that, but inside the 150 m the jet asks for.
    let cfg = Config::drones();
    let mut states = airliner_and_drone(2_400.0, 0.0, -120.0);
    assert!(!assess(&states, &fleet(), &Calm, &cfg)
        .unwrap()
        .conflicts
        .is_empty());
    states[0].protection = None;
    assert!(assess(&states, &fleet(), &Calm, &cfg)
        .unwrap()
        .conflicts
        .is_empty());
}

#[test]
fn an_airliner_is_advised_when_nothing_that_gives_way_first_can_help() {
    let cfg = Config::drones();
    // An uncontrolled intruder in its way: it never gives way, so the
    // airliner has to.
    let mut states = airliner_and_drone(2_400.0, 0.0, -80.0);
    states[1].give_way = GiveWay::Never;
    let a = assess(&states, &fleet(), &Calm, &cfg).unwrap();
    assert!(
        a.advisories.iter().all(|adv| adv.aircraft != 1),
        "{:?}",
        a.advisories
    );
    let adv = a
        .advisories
        .iter()
        .find(|adv| adv.aircraft == 10)
        .expect("the airliner is advised");
    assert!(adv.risk_after < adv.risk_before, "{adv:?}");
}

#[test]
fn nobody_who_never_gives_way_is_advised_and_their_conflict_is_left_over() {
    let cfg = Config::drones();
    let mut states = airliner_and_drone(2_400.0, 0.0, -80.0);
    states[0].give_way = GiveWay::Never;
    states[1].give_way = GiveWay::Never;
    let a = assess(&states, &fleet(), &Calm, &cfg).unwrap();
    assert!(a.advisories.is_empty(), "{:?}", a.advisories);
    assert!(!a.residual.is_empty());
}
