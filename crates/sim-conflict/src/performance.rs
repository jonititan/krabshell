//! What an aircraft can do: the energy model that says how fast it can climb
//! or descend, and the limits on turning, accelerating and responding.
//!
//! Fixed-wing aircraft use the Total Energy Model of EUROCONTROL's BADA 3:
//!
//! ```text
//! (T − D) · V = m · g · dh/dt + m · V · dV/dt
//! ```
//!
//! split between climbing and accelerating by the *energy share factor* for
//! the speed schedule being flown (constant CAS or constant Mach, below or
//! above the tropopause). Thrust is a simple maximum-climb rating that lapses
//! with density and Mach; drag is a parabolic polar on the lift a level wing
//! needs. Only the published equations are used here: BADA's aircraft data
//! is licensed by EUROCONTROL and OpenAP's is LGPL, so neither is shipped.
//! [`FixedWing`]'s fields are named as OpenAP names them (`wing_area_m2` is
//! `wing.area`, `cd0` and `k` the clean drag polar, `max_thrust_n` an engine's
//! `max_thrust` times the engine count), so either source can fill them.
//!
//! Multirotors use the same idea in its simplest form: the power spare for
//! climbing falls as forward speed takes more of it.
//!
//! The energy model fixes the *shape* of a trajectory (a climb slows as the
//! air thins; a fast multirotor cannot also climb hard); the prediction
//! scales it to the rate the aircraft is actually seen flying.

use serde::{Deserialize, Serialize};

use crate::atmosphere::{self, cas_to_tas, isa, BETA_T, G0, H_TROP, KAPPA, R, RHO0};

/// An aircraft's performance.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Performance {
    FixedWing(FixedWing),
    Multirotor(Multirotor),
}

/// Power setting for the energy model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Thrust {
    /// Maximum climb.
    Climb,
    /// Flight idle.
    Idle,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FixedWing {
    pub name: String,
    /// Mass assumed when a state gives none, kg.
    pub mass_kg: f64,
    pub wing_area_m2: f64,
    /// Clean zero-lift drag coefficient.
    pub cd0: f64,
    /// Induced drag factor: `CD = cd0 + k·CL²`.
    pub k: f64,
    /// Total sea-level static thrust, N.
    pub max_thrust_n: f64,
    /// Maximum climb thrust as a share of static thrust.
    pub climb_thrust_factor: f64,
    /// Thrust lapse with density: `T ∝ (ρ/ρ₀)^thrust_lapse`.
    pub thrust_lapse: f64,
    /// Thrust lapse with Mach: `T ∝ 1 − mach_lapse·M`.
    pub mach_lapse: f64,
    /// Flight idle thrust as a share of static thrust.
    pub idle_thrust_factor: f64,
    /// Climb and descent speed schedule: this CAS, then this Mach above the
    /// crossover altitude.
    pub schedule_cas_ms: f64,
    pub schedule_mach: f64,
    pub min_cas_ms: f64,
    pub max_cas_ms: f64,
    pub max_mach: f64,
    pub ceiling_m: f64,
    /// Bank angle for turns.
    pub bank_deg: f64,
    pub max_rocd_ms: f64,
    /// Vertical speed a climb or descend advisory asks for.
    pub advisory_vs_ms: f64,
    /// Vertical acceleration onto an advisory, in g.
    pub vertical_accel_g: f64,
    /// Along-track acceleration onto a new speed, m/s².
    pub speed_accel_ms2: f64,
    /// Seconds between an advisory and the aircraft starting to follow it.
    pub response_delay_s: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Multirotor {
    pub name: String,
    pub mass_kg: f64,
    pub cruise_ms: f64,
    pub max_speed_ms: f64,
    /// Climb rate hovering; less in forward flight.
    pub max_climb_ms: f64,
    pub max_descent_ms: f64,
    pub accel_ms2: f64,
    pub yaw_rate_dps: f64,
    pub vertical_accel_ms2: f64,
    pub response_delay_s: f64,
    pub ceiling_m: f64,
    pub advisory_vs_ms: f64,
}

impl Performance {
    /// An A320-class twin jet. Round figures from open literature for
    /// illustration and tests; fill [`FixedWing`] from OpenAP or BADA for
    /// real types.
    pub fn generic_narrowbody() -> Performance {
        Performance::FixedWing(FixedWing {
            name: "generic narrow-body twin jet".into(),
            mass_kg: 65_000.0,
            wing_area_m2: 122.6,
            cd0: 0.022,
            k: 0.040,
            max_thrust_n: 240_000.0,
            climb_thrust_factor: 0.8,
            thrust_lapse: 0.75,
            mach_lapse: 0.5,
            idle_thrust_factor: 0.06,
            schedule_cas_ms: 144.0,
            schedule_mach: 0.78,
            min_cas_ms: 65.0,
            max_cas_ms: 180.0,
            max_mach: 0.82,
            ceiling_m: 12_000.0,
            bank_deg: 25.0,
            max_rocd_ms: 25.0,
            advisory_vs_ms: 7.62,
            vertical_accel_g: 0.25,
            speed_accel_ms2: 0.6,
            response_delay_s: 5.0,
        })
    }

    /// A small delivery quadcopter, with the krabshell drones' defaults.
    pub fn generic_quadcopter() -> Performance {
        Performance::Multirotor(Multirotor {
            name: "generic quadcopter".into(),
            mass_kg: 6.0,
            cruise_ms: 10.0,
            max_speed_ms: 18.0,
            max_climb_ms: 3.0,
            max_descent_ms: 2.5,
            accel_ms2: 3.0,
            yaw_rate_dps: 60.0,
            vertical_accel_ms2: 2.0,
            response_delay_s: 1.0,
            ceiling_m: 500.0,
            advisory_vs_ms: 3.0,
        })
    }

    pub fn name(&self) -> &str {
        match self {
            Performance::FixedWing(p) => &p.name,
            Performance::Multirotor(p) => &p.name,
        }
    }

    pub fn default_mass_kg(&self) -> f64 {
        match self {
            Performance::FixedWing(p) => p.mass_kg,
            Performance::Multirotor(p) => p.mass_kg,
        }
    }

    /// The energy model's vertical rate at a power setting, m/s (positive
    /// up), clamped to what the aircraft may do.
    pub fn energy_rate_ms(&self, alt_m: f64, tas_ms: f64, mass_kg: f64, thrust: Thrust) -> f64 {
        match self {
            Performance::FixedWing(p) => p.tem_rate(alt_m, tas_ms, mass_kg, thrust),
            Performance::Multirotor(p) => match thrust {
                Thrust::Climb => p.max_climb_at(tas_ms),
                Thrust::Idle => -p.max_descent_ms,
            },
        }
    }

    /// Steepest climb it can make here, m/s.
    pub fn max_climb_ms(&self, alt_m: f64, tas_ms: f64, mass_kg: f64) -> f64 {
        if alt_m >= self.ceiling_m() {
            return 0.0;
        }
        self.energy_rate_ms(alt_m, tas_ms, mass_kg, Thrust::Climb)
            .max(0.0)
    }

    /// Steepest descent it would normally fly here, m/s (positive).
    pub fn max_descent_ms(&self, alt_m: f64, tas_ms: f64, mass_kg: f64) -> f64 {
        match self {
            // Idle descent, or the advisory rate if the energy model says it
            // would hardly descend (slow and draggy is not how airliners
            // descend: they use speed brakes).
            Performance::FixedWing(p) => {
                (-p.tem_rate(alt_m, tas_ms, mass_kg, Thrust::Idle)).max(p.advisory_vs_ms)
            }
            Performance::Multirotor(p) => p.max_descent_ms,
        }
    }

    /// Turn rate, degrees per second.
    pub fn turn_rate_dps(&self, tas_ms: f64) -> f64 {
        match self {
            Performance::FixedWing(p) => {
                let v = tas_ms.max(30.0);
                (G0 * libm::tan(p.bank_deg.to_radians()) / v).to_degrees()
            }
            Performance::Multirotor(p) => p.yaw_rate_dps,
        }
    }

    pub fn vertical_accel_ms2(&self) -> f64 {
        match self {
            Performance::FixedWing(p) => p.vertical_accel_g * G0,
            Performance::Multirotor(p) => p.vertical_accel_ms2,
        }
    }

    pub fn speed_accel_ms2(&self) -> f64 {
        match self {
            Performance::FixedWing(p) => p.speed_accel_ms2,
            Performance::Multirotor(p) => p.accel_ms2,
        }
    }

    pub fn response_delay_s(&self) -> f64 {
        match self {
            Performance::FixedWing(p) => p.response_delay_s,
            Performance::Multirotor(p) => p.response_delay_s,
        }
    }

    pub fn advisory_vs_ms(&self) -> f64 {
        match self {
            Performance::FixedWing(p) => p.advisory_vs_ms,
            Performance::Multirotor(p) => p.advisory_vs_ms,
        }
    }

    pub fn ceiling_m(&self) -> f64 {
        match self {
            Performance::FixedWing(p) => p.ceiling_m,
            Performance::Multirotor(p) => p.ceiling_m,
        }
    }

    /// The true airspeeds it may fly at an altitude, m/s.
    pub fn speed_envelope(&self, alt_m: f64) -> (f64, f64) {
        match self {
            Performance::FixedWing(p) => {
                let low = cas_to_tas(p.min_cas_ms, alt_m);
                let high = cas_to_tas(p.max_cas_ms, alt_m).min(p.max_mach * isa(alt_m).sound_ms);
                (low, high.max(low))
            }
            Performance::Multirotor(p) => (0.0, p.max_speed_ms),
        }
    }

    /// The true airspeed a climb or descent holds at an altitude, given the
    /// calibrated airspeed and Mach it is flying: constant CAS up to the
    /// crossover, constant Mach above. A multirotor holds its airspeed.
    pub fn scheduled_tas(&self, cas_ms: f64, mach: f64, alt_m: f64) -> f64 {
        match self {
            Performance::FixedWing(_) => {
                let by_cas = cas_to_tas(cas_ms, alt_m);
                let by_mach = mach * isa(alt_m).sound_ms;
                by_cas.min(by_mach)
            }
            Performance::Multirotor(_) => cas_ms,
        }
    }
}

impl FixedWing {
    fn thrust_n(&self, alt_m: f64, mach: f64, thrust: Thrust) -> f64 {
        let sigma = isa(alt_m).density / RHO0;
        let lapse = libm::pow(sigma, self.thrust_lapse);
        match thrust {
            Thrust::Climb => {
                self.max_thrust_n
                    * self.climb_thrust_factor
                    * lapse
                    * (1.0 - self.mach_lapse * mach).max(0.1)
            }
            Thrust::Idle => self.max_thrust_n * self.idle_thrust_factor * lapse,
        }
    }

    /// Drag in level flight (lift equals weight), N.
    pub fn drag_n(&self, alt_m: f64, tas_ms: f64, mass_kg: f64) -> f64 {
        let air = isa(alt_m);
        let q = 0.5 * air.density * tas_ms * tas_ms;
        let cl = mass_kg * G0 / (q * self.wing_area_m2).max(1e-6);
        q * self.wing_area_m2 * (self.cd0 + self.k * cl * cl)
    }

    /// Rate of climb or descent from the Total Energy Model.
    fn tem_rate(&self, alt_m: f64, tas_ms: f64, mass_kg: f64, thrust: Thrust) -> f64 {
        let tas = tas_ms.max(20.0);
        let mach = atmosphere::mach(tas, alt_m);
        let excess = self.thrust_n(alt_m, mach, thrust) - self.drag_n(alt_m, tas, mass_kg);
        // On the schedule: constant CAS below the crossover, constant Mach
        // above it.
        let constant_mach =
            cas_to_tas(self.schedule_cas_ms, alt_m) >= self.schedule_mach * isa(alt_m).sound_ms;
        let rate = excess * tas / (mass_kg * G0) * energy_share(alt_m, mach, constant_mach);
        rate.clamp(-self.max_rocd_ms, self.max_rocd_ms)
    }
}

impl Multirotor {
    /// Excess power for climbing falls as forward flight takes more of it.
    fn max_climb_at(&self, tas_ms: f64) -> f64 {
        let f = (tas_ms / self.max_speed_ms.max(0.1)).clamp(0.0, 1.0);
        self.max_climb_ms * (1.0 - 0.5 * f * f)
    }
}

/// BADA 3's energy share factor: the share of excess power that goes into
/// climbing, flying a constant CAS or a constant Mach, below or above the
/// tropopause.
pub fn energy_share(alt_m: f64, mach: f64, constant_mach: bool) -> f64 {
    let m2 = mach * mach;
    let below = alt_m < H_TROP;
    let f = match (constant_mach, below) {
        (true, true) => 1.0 / (1.0 + KAPPA * R * BETA_T * m2 / (2.0 * G0)),
        (true, false) => 1.0,
        (false, below) => {
            let base = 1.0 + (KAPPA - 1.0) / 2.0 * m2;
            let compress = libm::pow(base, -1.0 / (KAPPA - 1.0))
                * (libm::pow(base, KAPPA / (KAPPA - 1.0)) - 1.0);
            let temp = if below {
                KAPPA * R * BETA_T * m2 / (2.0 * G0)
            } else {
                0.0
            };
            1.0 / (1.0 + temp + compress)
        }
    };
    f.clamp(0.1, 1.5)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_jet_climbs_more_slowly_as_the_air_thins() {
        let p = Performance::generic_narrowbody();
        let Performance::FixedWing(fw) = &p else {
            unreachable!()
        };
        let rate = |alt: f64| {
            let tas = p.scheduled_tas(fw.schedule_cas_ms, fw.schedule_mach, alt);
            p.max_climb_ms(alt, tas, 65_000.0)
        };
        let (low, mid, high) = (rate(1_500.0), rate(6_000.0), rate(11_000.0));
        assert!(low > mid && mid > high, "{low} {mid} {high}");
        // Initial climb in the 2 000–4 000 ft/min range; a few hundred near
        // the ceiling.
        assert!((10.0..21.0).contains(&low), "{low}");
        assert!((0.5..6.0).contains(&high), "{high}");
        // Heavier climbs less.
        let tas = p.scheduled_tas(fw.schedule_cas_ms, fw.schedule_mach, 6_000.0);
        assert!(p.max_climb_ms(6_000.0, tas, 75_000.0) < mid);
    }

    #[test]
    fn at_idle_a_jet_descends() {
        let p = Performance::generic_narrowbody();
        let tas = cas_to_tas(144.0, 8_000.0);
        assert!(p.energy_rate_ms(8_000.0, tas, 60_000.0, Thrust::Idle) < -5.0);
    }

    #[test]
    fn the_energy_share_is_the_bada_values() {
        // Constant Mach above the tropopause: all of it.
        assert_eq!(energy_share(12_000.0, 0.78, true), 1.0);
        // Constant CAS below: speed grows as it climbs, so less than all.
        let f = energy_share(5_000.0, 0.6, false);
        assert!((0.7..0.95).contains(&f), "{f}");
        // Constant Mach below: temperature falls, TAS falls, so more.
        assert!(energy_share(9_000.0, 0.78, true) > 1.0);
    }

    #[test]
    fn a_fast_multirotor_climbs_less_than_a_hovering_one() {
        let p = Performance::generic_quadcopter();
        assert_eq!(p.max_climb_ms(50.0, 0.0, 6.0), 3.0);
        assert!(p.max_climb_ms(50.0, 18.0, 6.0) < 2.0);
        assert_eq!(p.max_climb_ms(600.0, 0.0, 6.0), 0.0);
    }
}
