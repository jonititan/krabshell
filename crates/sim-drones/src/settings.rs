//! Settings read from a pack's `params.toml`.

use std::collections::BTreeMap;

use sim_core::config::Param;

use crate::airspace::Atm;
use crate::dispatch::DispatchSettings;
use crate::flight;
use crate::mission::{Tuning, Winch};
use crate::sensor::Sensor;

/// Settings read from `params.toml`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Settings {
    pub(crate) tuning: Tuning,
    pub(crate) sensor: Sensor,
    pub(crate) max_mission_items: usize,
    /// Orders per second from each address.
    pub(crate) order_rate_per_address_s: f64,
    pub(crate) dispatch: DispatchSettings,
    pub(crate) cruise_cms: i32,
    pub(crate) person_speed_cms: i32,
    pub(crate) vehicle_speed_cms: i32,
    pub(crate) intruder_speed_cms: i32,
    pub(crate) wander_radius_m: u32,
    /// How far drones keep from a geofence, metres.
    pub(crate) geofence_margin_m: f64,
    pub(crate) atm: Atm,
}

pub(crate) fn param_f64(params: &BTreeMap<String, Param>, key: &str, default: f64) -> f64 {
    params
        .get(key)
        .and_then(Param::as_f64)
        .filter(|v| v.is_finite())
        .unwrap_or(default)
}

pub(crate) fn cms(metres_per_second: f64) -> i32 {
    libm::round(metres_per_second * 100.0) as i32
}

pub(crate) fn settings(params: &BTreeMap<String, Param>) -> Settings {
    let dt = param_f64(params, "tick_seconds", 0.1).clamp(0.001, 10.0);
    let ticks = |seconds: f64| (libm::round(seconds / dt) as u64).max(1);
    let hfov = param_f64(params, "hfov_deg", 70.0).clamp(1.0, 170.0);
    Settings {
        tuning: Tuning {
            dt,
            dt_ms: libm::round(dt * 1000.0) as u32,
            climb_ms: param_f64(params, "climb_ms", 3.0).max(0.1),
            takeoff_alt_mm: flight::to_mm(param_f64(params, "takeoff_alt_m", 20.0)),
            rtl_alt_mm: flight::to_mm(param_f64(params, "rtl_alt_m", 30.0)),
            // A 25-minute endurance by default. 0 flies forever.
            drain_ppm_tick: libm::round(
                param_f64(params, "battery_drain_pct_s", 100.0 / 1500.0).max(0.0) * 10_000.0 * dt,
            ) as u32,
            reserve_ppm: (param_f64(params, "battery_reserve_pct", 15.0).clamp(0.0, 100.0)
                * 10_000.0) as u32,
            // No charger by default.
            charge_ppm_tick: libm::round(
                param_f64(params, "charge_pct_s", 0.0).max(0.0) * 10_000.0 * dt,
            ) as u32,
            launch_ppm: (param_f64(params, "launch_battery_pct", 0.0).clamp(0.0, 100.0) * 10_000.0)
                as u32,
            winch: Winch {
                drop_ms: param_f64(params, "winch_drop_ms", 2.0).max(0.01),
                slow_ms: param_f64(params, "winch_slow_ms", 0.5).max(0.01),
                slow_m: param_f64(params, "winch_slow_m", 3.0).max(0.0),
                release_s: param_f64(params, "winch_release_s", 5.0).max(0.0),
                reel_ms: param_f64(params, "winch_reel_ms", 2.5).max(0.01),
                lock_s: param_f64(params, "winch_lock_s", 3.0).max(0.0),
            },
        },
        sensor: Sensor {
            half_fov_tan: libm::tan(hfov.to_radians() / 2.0),
            max_range_m: param_f64(params, "max_range_m", 250.0).max(1.0),
            p_max: param_f64(params, "p_detect", 0.85).clamp(0.0, 1.0),
            edge_falloff: param_f64(params, "edge_falloff", 0.6).clamp(0.0, 1.0),
            every_ticks: ticks(param_f64(params, "sensor_period_s", 1.0)),
            report_every_ticks: ticks(param_f64(params, "report_interval_s", 10.0)),
        },
        max_mission_items: param_f64(params, "max_mission_items", 128.0).clamp(1.0, 4096.0)
            as usize,
        order_rate_per_address_s: param_f64(params, "order_rate_per_address_day", 0.0).max(0.0)
            / 86_400.0,
        dispatch: DispatchSettings {
            // Set at reset, once the address book's size is known.
            orders_per_tick: 0.0,
            takeoff_m: param_f64(params, "dispatch_takeoff_m", 30.0) as f32,
            approach_m: param_f64(params, "dispatch_approach_m", 15.0) as f32,
            load_ticks: ticks(param_f64(params, "dispatch_load_s", 120.0)),
        },
        cruise_cms: cms(param_f64(params, "cruise_ms", 10.0)),
        person_speed_cms: cms(param_f64(params, "person_speed_ms", 1.4)),
        vehicle_speed_cms: cms(param_f64(params, "vehicle_speed_ms", 8.0)),
        intruder_speed_cms: cms(param_f64(params, "intruder_speed_ms", 12.0)),
        wander_radius_m: param_f64(params, "wander_radius_m", 600.0).max(1.0) as u32,
        geofence_margin_m: param_f64(params, "geofence_margin_m", 30.0).clamp(2.0, 1_000.0),
        atm: {
            let min_radius_m = param_f64(params, "atm_fence_min_radius_m", 150.0).max(10.0);
            Atm {
                per_tick: param_f64(params, "atm_fences_per_hour", 0.0).max(0.0) / 3_600.0 * dt,
                mean_open_ticks: param_f64(params, "atm_fence_mean_minutes", 20.0).max(0.1) * 60.0
                    / dt,
                min_radius_m,
                max_radius_m: param_f64(params, "atm_fence_max_radius_m", 500.0).max(min_radius_m),
                max_open: param_f64(params, "atm_max_fences", 6.0).clamp(0.0, 1_000.0) as usize,
                area_radius_m: param_f64(
                    params,
                    "atm_area_radius_m",
                    param_f64(params, "wander_radius_m", 600.0),
                )
                .max(1.0),
            }
        },
    }
}
