//! The ICAO standard atmosphere, and airspeed conversions.
//!
//! Troposphere and the lower stratosphere (to 20 km), which is all any
//! aircraft this crate predicts flies in. `libm` throughout, so a prediction
//! gives the same bits on every machine.

/// Ratio of specific heats for air.
pub const KAPPA: f64 = 1.4;
/// Specific gas constant for dry air, J/(kg·K).
pub const R: f64 = 287.052_87;
/// Standard gravity, m/s².
pub const G0: f64 = 9.806_65;
/// Sea-level standard temperature, K.
pub const T0: f64 = 288.15;
/// Sea-level standard pressure, Pa.
pub const P0: f64 = 101_325.0;
/// Sea-level standard density, kg/m³.
pub const RHO0: f64 = 1.225;
/// Temperature gradient below the tropopause, K/m.
pub const BETA_T: f64 = -0.0065;
/// Tropopause altitude, m.
pub const H_TROP: f64 = 11_000.0;

/// Temperature, pressure and density at a geopotential altitude.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Air {
    pub temperature_k: f64,
    pub pressure_pa: f64,
    pub density: f64,
    /// Speed of sound, m/s.
    pub sound_ms: f64,
}

pub fn isa(alt_m: f64) -> Air {
    let h = alt_m.clamp(-500.0, 20_000.0);
    let (t, p) = if h <= H_TROP {
        let t = T0 + BETA_T * h;
        (t, P0 * libm::pow(t / T0, -G0 / (BETA_T * R)))
    } else {
        let t_trop = T0 + BETA_T * H_TROP;
        let p_trop = P0 * libm::pow(t_trop / T0, -G0 / (BETA_T * R));
        (
            t_trop,
            p_trop * libm::exp(-G0 / (R * t_trop) * (h - H_TROP)),
        )
    };
    Air {
        temperature_k: t,
        pressure_pa: p,
        density: p / (R * t),
        sound_ms: libm::sqrt(KAPPA * R * t),
    }
}

const MU: f64 = (KAPPA - 1.0) / KAPPA;

/// True airspeed from calibrated airspeed (compressible), m/s.
pub fn cas_to_tas(cas_ms: f64, alt_m: f64) -> f64 {
    let a = isa(alt_m);
    let inner = libm::pow(1.0 + MU / 2.0 * RHO0 / P0 * cas_ms * cas_ms, 1.0 / MU) - 1.0;
    let outer = libm::pow(1.0 + P0 / a.pressure_pa * inner, MU) - 1.0;
    libm::sqrt(2.0 / MU * a.pressure_pa / a.density * outer)
}

/// Calibrated airspeed from true airspeed (compressible), m/s.
pub fn tas_to_cas(tas_ms: f64, alt_m: f64) -> f64 {
    let a = isa(alt_m);
    let inner = libm::pow(
        1.0 + MU / 2.0 * a.density / a.pressure_pa * tas_ms * tas_ms,
        1.0 / MU,
    ) - 1.0;
    let outer = libm::pow(1.0 + a.pressure_pa / P0 * inner, MU) - 1.0;
    libm::sqrt(2.0 / MU * P0 / RHO0 * outer)
}

pub fn mach(tas_ms: f64, alt_m: f64) -> f64 {
    tas_ms / isa(alt_m).sound_ms
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_standard_atmosphere_matches_its_tables() {
        let sl = isa(0.0);
        assert!((sl.density - 1.225).abs() < 1e-3);
        assert!((sl.sound_ms - 340.29).abs() < 0.05);
        // 11 km: 216.65 K, 22 632 Pa, 0.3639 kg/m³.
        let trop = isa(11_000.0);
        assert!((trop.temperature_k - 216.65).abs() < 1e-9);
        assert!((trop.pressure_pa - 22_632.0).abs() < 5.0);
        assert!((trop.density - 0.3639).abs() < 1e-3);
    }

    #[test]
    fn cas_and_tas_round_trip_and_tas_grows_with_height() {
        for alt in [0.0, 3_000.0, 9_000.0, 12_000.0] {
            let tas = cas_to_tas(140.0, alt);
            assert!((tas_to_cas(tas, alt) - 140.0).abs() < 1e-6, "{alt}");
        }
        assert!((cas_to_tas(140.0, 0.0) - 140.0).abs() < 1e-3);
        // 280 kt CAS at FL300: about 266 kt EAS after compressibility, and
        // ×1.635 (1/√σ, σ = 0.374) to TAS, so about 435 kt.
        let tas_kt = cas_to_tas(280.0 * 0.514_444, 9_144.0) / 0.514_444;
        assert!((tas_kt - 436.0).abs() < 4.0, "{tas_kt}");
    }
}
