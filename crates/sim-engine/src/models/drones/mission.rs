//! MAVLink missions and commands, executed by a simulated multirotor.
//!
//! A [`MissionItem`] carries exactly the fields of `MISSION_ITEM_INT`, so a
//! mission uploaded by a ground station round-trips through the simulation and
//! back out on download unchanged. Command numbers and parameter meanings follow
//! MAVLink `common.xml`; see <https://mavlink.io/en/services/mission.html>.
//!
//! Altitudes are always taken as relative to the drone's home. The global
//! frames that carry an absolute altitude are accepted but read the same way:
//! a scenario has no terrain model to be absolute against.

use serde::{Deserialize, Serialize};

use super::flight::{self, Limits, Point, Pose};

// MAV_CMD values this executor understands.
pub const NAV_WAYPOINT: u16 = 16;
pub const NAV_LOITER_UNLIM: u16 = 17;
pub const NAV_LOITER_TIME: u16 = 19;
pub const NAV_RETURN_TO_LAUNCH: u16 = 20;
pub const NAV_LAND: u16 = 21;
pub const NAV_TAKEOFF: u16 = 22;
pub const CONDITION_DELAY: u16 = 112;
pub const DO_SET_MODE: u16 = 176;
pub const DO_JUMP: u16 = 177;
pub const DO_CHANGE_SPEED: u16 = 178;
pub const DO_REPOSITION: u16 = 192;
pub const DO_PAUSE_CONTINUE: u16 = 193;
pub const DO_SET_MISSION_CURRENT: u16 = 224;
pub const MISSION_START: u16 = 300;
pub const COMPONENT_ARM_DISARM: u16 = 400;

// MAV_FRAME values accepted in a mission.
const FRAME_GLOBAL: u8 = 0;
const FRAME_MISSION: u8 = 2;
const FRAME_GLOBAL_RELATIVE_ALT: u8 = 3;
const FRAME_GLOBAL_INT: u8 = 5;
const FRAME_GLOBAL_RELATIVE_ALT_INT: u8 = 6;

/// `MAV_MODE_FLAG_AUTO_ENABLED`, as DO_SET_MODE's first parameter carries it.
const MODE_FLAG_AUTO: u32 = 4;

/// `COMPONENT_ARM_DISARM` param2 that overrides the in-flight disarm check.
const FORCE_DISARM: f32 = 21_196.0;

/// One `MISSION_ITEM_INT`, field for field.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MissionItem {
    pub seq: u16,
    pub frame: u8,
    pub command: u16,
    #[serde(default = "yes")]
    pub autocontinue: u8,
    #[serde(default, with = "nan_as_null")]
    pub param1: f32,
    #[serde(default, with = "nan_as_null")]
    pub param2: f32,
    #[serde(default, with = "nan_as_null")]
    pub param3: f32,
    #[serde(default, with = "nan_as_null")]
    pub param4: f32,
    /// Latitude, degrees × 10⁷.
    #[serde(default)]
    pub x: i32,
    /// Longitude, degrees × 10⁷.
    #[serde(default)]
    pub y: i32,
    /// Altitude, metres.
    #[serde(default, with = "nan_as_null")]
    pub z: f32,
}

fn yes() -> u8 {
    1
}

/// MAVLink uses NaN to mean "leave unchanged" (yaw, most often). JSON has no
/// NaN, so it travels as `null`; without this a NaN would serialise as `null`
/// and fail to read back.
mod nan_as_null {
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<S: Serializer>(value: &f32, s: S) -> Result<S::Ok, S::Error> {
        if value.is_finite() {
            s.serialize_f32(*value)
        } else {
            s.serialize_none()
        }
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<f32, D::Error> {
        Ok(Option::<f32>::deserialize(d)?.unwrap_or(f32::NAN))
    }
}

impl MissionItem {
    /// Commands that fly somewhere carry a position in `x`/`y`/`z`.
    fn is_nav(&self) -> bool {
        matches!(
            self.command,
            NAV_WAYPOINT | NAV_LOITER_UNLIM | NAV_LOITER_TIME | NAV_LAND | NAV_TAKEOFF
        )
    }

    fn target(&self, from: &Pose) -> Point {
        // A zero position means "here", as MAVLink allows for TAKEOFF and LAND.
        let (lat_e7, lon_e7) = if self.x == 0 && self.y == 0 {
            (from.lat_e7, from.lon_e7)
        } else {
            (self.x, self.y)
        };
        let alt_mm = if self.z.is_finite() {
            flight::to_mm(f64::from(self.z))
        } else {
            from.alt_mm
        };
        Point {
            lat_e7,
            lon_e7,
            alt_mm,
        }
    }

    /// Where this item would take a drone, for checking against the world's
    /// bounds before accepting an upload. `None` for items that stay put.
    pub fn destination(&self) -> Option<Point> {
        (self.is_nav() && !(self.x == 0 && self.y == 0)).then_some(Point {
            lat_e7: self.x,
            lon_e7: self.y,
            alt_mm: 0,
        })
    }

    pub fn check(&self, index: usize) -> Result<(), String> {
        if usize::from(self.seq) != index {
            return Err(format!("item {index} has seq {}", self.seq));
        }
        let frame_ok = match self.frame {
            FRAME_GLOBAL
            | FRAME_GLOBAL_RELATIVE_ALT
            | FRAME_GLOBAL_INT
            | FRAME_GLOBAL_RELATIVE_ALT_INT => true,
            FRAME_MISSION => !self.is_nav(),
            _ => false,
        };
        if !frame_ok {
            return Err(format!(
                "item {index}: frame {} is not supported (use 3 or 6, or 2 for non-navigation items)",
                self.frame
            ));
        }
        Ok(())
    }
}

/// What the drone is doing. Serialised in the drone's state and mapped to
/// MAVLink heartbeat flags by the gateway.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "mode")]
pub enum Mode {
    /// On the ground, motors off.
    Disarmed,
    /// On the ground, armed, waiting for a command.
    Idle,
    /// Climbing straight up to an altitude, then holding there.
    Takeoff {
        #[serde(rename = "takeoff_alt_mm")]
        alt_mm: i32,
    },
    /// Flying the uploaded mission.
    Auto,
    /// Holding position in the air.
    Loiter,
    /// Flying to a point commanded by DO_REPOSITION, then holding.
    Goto { to: Point },
    /// Returning home at a safe height, then landing.
    Rtl,
    /// Descending where it is, then disarming.
    Land,
}

impl Mode {
    pub fn name(&self) -> &'static str {
        match self {
            Mode::Disarmed => "disarmed",
            Mode::Idle => "idle",
            Mode::Takeoff { .. } => "takeoff",
            Mode::Auto => "auto",
            Mode::Loiter => "loiter",
            Mode::Goto { .. } => "goto",
            Mode::Rtl => "rtl",
            Mode::Land => "land",
        }
    }
}

/// Everything about a drone beyond its pose.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Drone {
    pub sysid: u8,
    #[serde(flatten)]
    pub mode: Mode,
    pub home: Point,
    #[serde(default)]
    pub mission: Vec<MissionItem>,
    /// Index of the current mission item.
    #[serde(default)]
    pub mission_seq: u16,
    /// Bumped on every upload or clear, so a gateway can tell a new mission
    /// from the same one.
    #[serde(default)]
    pub mission_rev: u32,
    /// Whether the current item's position has been reached.
    #[serde(default)]
    pub arrived: bool,
    /// Milliseconds left on the current hold or delay.
    #[serde(default)]
    pub wait_ms: u32,
    /// Remaining repeats for each DO_JUMP, by the jump's seq. Missing means
    /// "not started yet".
    #[serde(default)]
    pub jumps: std::collections::BTreeMap<u16, i32>,
    /// Cruise speed, cm/s. DO_CHANGE_SPEED updates it.
    pub cruise_cms: i32,
    /// Battery remaining, parts per million of a full charge. Fine enough that
    /// a long endurance drains smoothly at a 0.1 s tick, rather than rounding
    /// to a whole step per tick.
    pub battery_ppm: u32,
}

/// A mission step's outcome, for the model to turn into a report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// Mission item `seq` completed.
    Reached(u16),
    /// The battery hit the reserve and the drone started home on its own.
    LowBattery,
}

/// Per-run flight settings from the pack's params.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tuning {
    pub dt: f64,
    pub dt_ms: u32,
    pub climb_ms: f64,
    pub takeoff_alt_mm: i32,
    pub rtl_alt_mm: i32,
    /// Battery drain while airborne, ppm per tick. Zero means endless.
    pub drain_ppm_tick: u32,
    /// Below this the drone returns home by itself.
    pub reserve_ppm: u32,
}

impl Drone {
    pub fn new(sysid: u8, home: Point, cruise_cms: i32) -> Self {
        Drone {
            sysid,
            mode: Mode::Disarmed,
            home,
            mission: Vec::new(),
            mission_seq: 0,
            mission_rev: 0,
            arrived: false,
            wait_ms: 0,
            jumps: Default::default(),
            cruise_cms,
            battery_ppm: 1_000_000,
        }
    }

    pub fn armed(&self) -> bool {
        !matches!(self.mode, Mode::Disarmed)
    }

    fn limits(&self, tuning: &Tuning) -> Limits {
        Limits {
            speed_ms: f64::from(self.cruise_cms) * 0.01,
            climb_ms: tuning.climb_ms,
        }
    }

    fn set_seq(&mut self, seq: u16) {
        self.mission_seq = seq;
        self.arrived = false;
        self.wait_ms = 0;
    }

    // ------------------------------------------------------------- uploads

    pub fn upload(&mut self, items: Vec<MissionItem>) {
        self.mission = items;
        self.mission_rev = self.mission_rev.wrapping_add(1);
        self.jumps.clear();
        self.set_seq(0);
    }

    pub fn clear(&mut self) {
        self.upload(Vec::new());
        if self.mode == Mode::Auto {
            self.mode = Mode::Loiter;
        }
    }

    pub fn set_current(&mut self, seq: u16) -> Result<(), String> {
        if usize::from(seq) >= self.mission.len() {
            return Err(format!(
                "seq {seq} is past the end of a {}-item mission",
                self.mission.len()
            ));
        }
        self.set_seq(seq);
        Ok(())
    }

    // ------------------------------------------------------------ commands

    /// Apply a `COMMAND_LONG`. `params` are param1..param7 in order.
    pub fn command(
        &mut self,
        cmd: u16,
        params: [f32; 7],
        pose: &Pose,
        tuning: &Tuning,
    ) -> Result<(), String> {
        let airborne = !pose.grounded();
        match cmd {
            COMPONENT_ARM_DISARM => {
                if params[0] >= 0.5 {
                    if self.armed() {
                        return Ok(());
                    }
                    if self.battery_ppm <= tuning.reserve_ppm {
                        return Err("battery below reserve".into());
                    }
                    self.mode = Mode::Idle;
                } else {
                    if airborne && params[1] != FORCE_DISARM {
                        return Err("refusing to disarm in flight".into());
                    }
                    self.mode = Mode::Disarmed;
                }
                Ok(())
            }
            NAV_TAKEOFF => {
                if !self.armed() {
                    return Err("arm before taking off".into());
                }
                let alt = if params[6].is_finite() && params[6] > 0.0 {
                    flight::to_mm(f64::from(params[6]))
                } else {
                    tuning.takeoff_alt_mm
                };
                self.mode = Mode::Takeoff { alt_mm: alt };
                Ok(())
            }
            MISSION_START => {
                if !self.armed() {
                    return Err("arm before starting the mission".into());
                }
                if self.mission.is_empty() {
                    return Err("no mission loaded".into());
                }
                let first = params[0];
                if first.is_finite()
                    && first >= 1.0
                    && usize::from(first as u16) < self.mission.len()
                {
                    self.set_seq(first as u16);
                } else if usize::from(self.mission_seq) >= self.mission.len() {
                    self.set_seq(0);
                }
                self.mode = Mode::Auto;
                Ok(())
            }
            NAV_RETURN_TO_LAUNCH => {
                if !airborne {
                    return Err("already on the ground".into());
                }
                self.mode = Mode::Rtl;
                Ok(())
            }
            NAV_LAND => {
                if !airborne {
                    return Err("already on the ground".into());
                }
                self.mode = Mode::Land;
                Ok(())
            }
            DO_SET_MODE => {
                let flags = params[0] as u32;
                if flags & MODE_FLAG_AUTO != 0 {
                    self.command(MISSION_START, [0.0; 7], pose, tuning)
                } else if airborne {
                    self.mode = Mode::Loiter;
                    Ok(())
                } else {
                    Ok(())
                }
            }
            DO_PAUSE_CONTINUE => {
                if params[0] >= 0.5 {
                    self.command(MISSION_START, [0.0; 7], pose, tuning)
                } else if airborne {
                    self.mode = Mode::Loiter;
                    Ok(())
                } else {
                    Err("not flying".into())
                }
            }
            DO_REPOSITION => {
                if !airborne {
                    return Err("take off before repositioning".into());
                }
                let lat = params[4];
                let lon = params[5];
                if !(lat.is_finite() && lon.is_finite()) {
                    return Err("reposition needs a latitude and longitude".into());
                }
                let alt = if params[6].is_finite() {
                    flight::to_mm(f64::from(params[6]))
                } else {
                    pose.alt_mm
                };
                self.mode = Mode::Goto {
                    to: Point {
                        lat_e7: flight::to_e7(f64::from(lat)),
                        lon_e7: flight::to_e7(f64::from(lon)),
                        alt_mm: alt,
                    },
                };
                Ok(())
            }
            DO_SET_MISSION_CURRENT => self.set_current(params[0].max(0.0) as u16),
            other => Err(format!("command {other} is not supported")),
        }
    }

    // ---------------------------------------------------------------- step

    /// Advance one tick. Returns what happened worth reporting.
    pub fn step(&mut self, pose: &mut Pose, tuning: &Tuning, events: &mut Vec<Event>) {
        if !pose.grounded() {
            self.battery_ppm = self.battery_ppm.saturating_sub(tuning.drain_ppm_tick);
            if tuning.drain_ppm_tick > 0
                && self.battery_ppm <= tuning.reserve_ppm
                && !matches!(self.mode, Mode::Rtl | Mode::Land)
            {
                self.mode = Mode::Rtl;
                events.push(Event::LowBattery);
            }
        }

        let limits = self.limits(tuning);
        match self.mode {
            Mode::Disarmed | Mode::Idle => flight::hold(pose),
            Mode::Takeoff { alt_mm } => {
                let target = pose.point().with_alt_mm(alt_mm);
                let left = flight::step_toward(pose, &target, limits, tuning.dt);
                if left.within(0.1, 0.3) {
                    self.mode = Mode::Loiter;
                }
            }
            Mode::Loiter => flight::hold(pose),
            Mode::Goto { to } => {
                let left = flight::step_toward(pose, &to, limits, tuning.dt);
                if left.within(0.5, 0.5) {
                    self.mode = Mode::Loiter;
                }
            }
            Mode::Rtl => {
                // Climb to the return height first, then fly home level, then land.
                let safe = pose.alt_mm.max(tuning.rtl_alt_mm);
                if pose.alt_mm < safe - 300 {
                    let target = pose.point().with_alt_mm(safe);
                    flight::step_toward(pose, &target, limits, tuning.dt);
                } else {
                    let target = self.home.with_alt_mm(safe);
                    let left = flight::step_toward(pose, &target, limits, tuning.dt);
                    if left.ground_m <= 1.0 {
                        self.mode = Mode::Land;
                    }
                }
            }
            Mode::Land => self.descend(pose, limits, tuning),
            Mode::Auto => self.run_mission(pose, limits, tuning, events),
        }
    }

    fn descend(&mut self, pose: &mut Pose, limits: Limits, tuning: &Tuning) {
        let target = pose.point().with_alt_mm(0);
        flight::step_toward(pose, &target, limits, tuning.dt);
        if pose.grounded() {
            pose.alt_mm = 0;
            flight::hold(pose);
            self.mode = Mode::Disarmed;
        }
    }

    fn finish_item(&mut self, events: &mut Vec<Event>) {
        events.push(Event::Reached(self.mission_seq));
        let next = self.mission_seq.saturating_add(1);
        self.set_seq(next);
    }

    /// Wait out `wait_ms`, returning true once it has elapsed.
    fn waited(&mut self, tuning: &Tuning) -> bool {
        if self.wait_ms == 0 {
            return true;
        }
        self.wait_ms = self.wait_ms.saturating_sub(tuning.dt_ms);
        self.wait_ms == 0
    }

    fn run_mission(
        &mut self,
        pose: &mut Pose,
        limits: Limits,
        tuning: &Tuning,
        events: &mut Vec<Event>,
    ) {
        // Non-navigation items take no time, but a bad DO_JUMP chain must not
        // spin the tick forever: cap how many run per tick.
        for _ in 0..16 {
            let Some(item) = self.mission.get(usize::from(self.mission_seq)).copied() else {
                // Past the end: hover where the mission left us.
                self.mode = if pose.grounded() {
                    Mode::Idle
                } else {
                    Mode::Loiter
                };
                flight::hold(pose);
                return;
            };

            match item.command {
                NAV_TAKEOFF => {
                    let target = pose.point().with_alt_mm(item.target(pose).alt_mm);
                    let left = flight::step_toward(pose, &target, limits, tuning.dt);
                    if left.within(0.1, 0.3) {
                        self.finish_item(events);
                    }
                    return;
                }
                NAV_WAYPOINT | NAV_LOITER_TIME | NAV_LOITER_UNLIM => {
                    if !self.arrived {
                        let target = item.target(pose);
                        let accept = if item.command == NAV_WAYPOINT && item.param2 > 0.0 {
                            f64::from(item.param2)
                        } else {
                            1.0
                        };
                        let left = flight::step_toward(pose, &target, limits, tuning.dt);
                        if !left.within(accept, 1.0) {
                            return;
                        }
                        self.arrived = true;
                        let hold_s = match item.command {
                            NAV_WAYPOINT | NAV_LOITER_TIME => item.param1.max(0.0),
                            _ => 0.0,
                        };
                        self.wait_ms = (hold_s * 1000.0) as u32;
                    } else {
                        flight::hold(pose);
                    }
                    if item.command == NAV_LOITER_UNLIM {
                        return;
                    }
                    if self.waited(tuning) {
                        self.finish_item(events);
                    }
                    return;
                }
                NAV_RETURN_TO_LAUNCH => {
                    events.push(Event::Reached(self.mission_seq));
                    self.mode = Mode::Rtl;
                    return;
                }
                NAV_LAND => {
                    if !self.arrived {
                        let target = item.target(pose).with_alt_mm(pose.alt_mm);
                        let left = flight::step_toward(pose, &target, limits, tuning.dt);
                        if left.ground_m > 1.0 {
                            return;
                        }
                        self.arrived = true;
                    }
                    self.descend(pose, limits, tuning);
                    if pose.grounded() {
                        events.push(Event::Reached(self.mission_seq));
                        self.set_seq(self.mission_seq.saturating_add(1));
                    }
                    return;
                }
                CONDITION_DELAY => {
                    flight::hold(pose);
                    if !self.arrived {
                        self.arrived = true;
                        self.wait_ms = (item.param1.max(0.0) * 1000.0) as u32;
                    }
                    if self.waited(tuning) {
                        self.finish_item(events);
                    }
                    return;
                }
                DO_JUMP => {
                    let seq = self.mission_seq;
                    let repeats = item.param2 as i32;
                    let left = *self.jumps.entry(seq).or_insert(repeats);
                    let target = item.param1.max(0.0) as u16;
                    if left != 0 && usize::from(target) < self.mission.len() {
                        if left > 0 {
                            self.jumps.insert(seq, left - 1);
                        }
                        events.push(Event::Reached(seq));
                        self.set_seq(target);
                    } else {
                        self.finish_item(events);
                    }
                }
                DO_CHANGE_SPEED => {
                    if item.param2 > 0.0 {
                        self.cruise_cms = libm::roundf(item.param2 * 100.0) as i32;
                    }
                    self.finish_item(events);
                }
                _ => {
                    // Anything else (camera triggers, servos) completes at once.
                    self.finish_item(events);
                }
            }
        }
        flight::hold(pose);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TUNING: Tuning = Tuning {
        dt: 0.1,
        dt_ms: 100,
        climb_ms: 3.0,
        takeoff_alt_mm: 20_000,
        rtl_alt_mm: 30_000,
        drain_ppm_tick: 10,
        reserve_ppm: 150_000,
    };

    fn item(seq: u16, command: u16, params: [f32; 4], x: i32, y: i32, z: f32) -> MissionItem {
        MissionItem {
            seq,
            frame: if matches!(command, DO_JUMP | DO_CHANGE_SPEED | CONDITION_DELAY) {
                2
            } else {
                6
            },
            command,
            autocontinue: 1,
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            x,
            y,
            z,
        }
    }

    fn home() -> Pose {
        Pose::at(-0.1276, 51.5072, 0.0)
    }

    fn north_of(pose: &Pose, metres: f64) -> (i32, i32) {
        flight::displace(pose.lat_e7, pose.lon_e7, 0.0, metres)
    }

    fn fly(drone: &mut Drone, pose: &mut Pose, ticks: usize) -> Vec<Event> {
        let mut events = Vec::new();
        for _ in 0..ticks {
            drone.step(pose, &TUNING, &mut events);
        }
        events
    }

    fn armed_drone(pose: &Pose) -> Drone {
        let mut drone = Drone::new(1, pose.point(), 1_000);
        drone
            .command(
                COMPONENT_ARM_DISARM,
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                pose,
                &TUNING,
            )
            .unwrap();
        drone
    }

    #[test]
    fn takeoff_waypoint_land_flies_the_whole_mission_and_disarms() {
        let mut pose = home();
        let (lat, lon) = north_of(&pose, 100.0);
        let mut drone = armed_drone(&pose);
        drone.upload(vec![
            item(0, NAV_TAKEOFF, [0.0; 4], 0, 0, 20.0),
            item(1, NAV_WAYPOINT, [0.0, 2.0, 0.0, f32::NAN], lat, lon, 20.0),
            item(2, NAV_LAND, [0.0; 4], 0, 0, 0.0),
        ]);
        drone
            .command(MISSION_START, [0.0; 7], &pose, &TUNING)
            .unwrap();

        let events = fly(&mut drone, &mut pose, 600);
        assert_eq!(
            events,
            vec![Event::Reached(0), Event::Reached(1), Event::Reached(2)]
        );
        assert_eq!(drone.mode, Mode::Disarmed);
        assert!(pose.grounded());
        let d = flight::ground_distance_m(
            &pose.point(),
            &Point {
                lat_e7: lat,
                lon_e7: lon,
                alt_mm: 0,
            },
        );
        assert!(d <= 2.0, "landed {d} m from the waypoint");
    }

    #[test]
    fn do_jump_repeats_the_requested_number_of_times() {
        let mut pose = home();
        let mut drone = armed_drone(&pose);
        drone.upload(vec![
            item(0, NAV_TAKEOFF, [0.0; 4], 0, 0, 5.0),
            item(1, CONDITION_DELAY, [0.2, 0.0, 0.0, 0.0], 0, 0, 0.0),
            item(2, DO_JUMP, [1.0, 2.0, 0.0, 0.0], 0, 0, 0.0),
        ]);
        drone
            .command(MISSION_START, [0.0; 7], &pose, &TUNING)
            .unwrap();
        let events = fly(&mut drone, &mut pose, 200);
        let delays = events.iter().filter(|e| **e == Event::Reached(1)).count();
        // Once through, then two repeats.
        assert_eq!(delays, 3);
        assert_eq!(drone.mode, Mode::Loiter);
    }

    #[test]
    fn condition_delay_holds_for_its_duration() {
        let mut pose = home();
        let mut drone = armed_drone(&pose);
        drone.upload(vec![
            item(0, NAV_TAKEOFF, [0.0; 4], 0, 0, 3.0),
            item(1, CONDITION_DELAY, [2.0, 0.0, 0.0, 0.0], 0, 0, 0.0),
        ]);
        drone
            .command(MISSION_START, [0.0; 7], &pose, &TUNING)
            .unwrap();
        let mut events = Vec::new();
        let mut ticks_in_delay = 0;
        for _ in 0..100 {
            drone.step(&mut pose, &TUNING, &mut events);
            if drone.mission_seq == 1 {
                ticks_in_delay += 1;
            }
        }
        assert_eq!(ticks_in_delay, 20);
    }

    #[test]
    fn rtl_climbs_flies_home_lands_and_disarms() {
        let mut pose = home();
        let start = pose.point();
        let (lat, lon) = north_of(&pose, 60.0);
        pose.lat_e7 = lat;
        pose.lon_e7 = lon;
        pose.alt_mm = 10_000;
        let mut drone = Drone::new(1, start, 1_000);
        drone.mode = Mode::Loiter;
        drone
            .command(NAV_RETURN_TO_LAUNCH, [0.0; 7], &pose, &TUNING)
            .unwrap();

        let mut peak = 0;
        let mut events = Vec::new();
        for _ in 0..800 {
            drone.step(&mut pose, &TUNING, &mut events);
            peak = peak.max(pose.alt_mm);
        }
        assert_eq!(drone.mode, Mode::Disarmed);
        assert!(peak >= 29_700, "climbed only to {peak} mm");
        assert!(flight::ground_distance_m(&pose.point(), &start) <= 1.0);
    }

    #[test]
    fn commands_that_do_not_fit_the_state_are_refused() {
        let pose = home();
        let mut drone = Drone::new(1, pose.point(), 1_000);
        assert!(drone
            .command(NAV_TAKEOFF, [0.0; 7], &pose, &TUNING)
            .is_err());
        assert!(drone
            .command(MISSION_START, [0.0; 7], &pose, &TUNING)
            .is_err());
        assert!(drone.command(NAV_LAND, [0.0; 7], &pose, &TUNING).is_err());
        assert!(drone.command(9_999, [0.0; 7], &pose, &TUNING).is_err());

        let mut flying = home();
        flying.alt_mm = 20_000;
        let mut drone = Drone::new(1, pose.point(), 1_000);
        drone.mode = Mode::Loiter;
        assert!(drone
            .command(COMPONENT_ARM_DISARM, [0.0; 7], &flying, &TUNING)
            .is_err());
    }

    #[test]
    fn a_draining_battery_sends_the_drone_home_at_the_reserve_and_zero_drain_never_does() {
        let mut pose = home();
        pose.alt_mm = 20_000;
        let tuning = Tuning {
            drain_ppm_tick: 1_000,
            ..TUNING
        };
        let mut drone = Drone::new(1, home().point(), 1_000);
        drone.mode = Mode::Loiter;
        let mut events = Vec::new();
        let mut ticks = 0;
        while !events.contains(&Event::LowBattery) {
            drone.step(&mut pose, &tuning, &mut events);
            ticks += 1;
            assert!(ticks < 2_000);
        }
        // (100 % − 15 % reserve) at 0.1 % a tick.
        assert_eq!(ticks, 850);
        assert_eq!(drone.mode, Mode::Rtl);

        let endless = Tuning {
            drain_ppm_tick: 0,
            ..TUNING
        };
        let mut drone = Drone::new(1, home().point(), 1_000);
        drone.mode = Mode::Loiter;
        for _ in 0..20_000 {
            drone.step(&mut pose, &endless, &mut events);
        }
        assert_eq!(drone.battery_ppm, 1_000_000);
        assert_eq!(drone.mode, Mode::Loiter);
    }

    #[test]
    fn nan_parameters_survive_a_json_round_trip() {
        let original = item(0, NAV_WAYPOINT, [0.0, 2.0, 0.0, f32::NAN], 1, 2, 30.0);
        let json = serde_json::to_value(original).unwrap();
        assert!(json["param4"].is_null());
        let back: MissionItem = serde_json::from_value(json).unwrap();
        assert!(back.param4.is_nan());
        assert_eq!(back.z, 30.0);
    }

    #[test]
    fn items_are_checked_for_sequence_and_frame() {
        assert!(item(0, NAV_WAYPOINT, [0.0; 4], 1, 1, 10.0).check(0).is_ok());
        assert!(item(1, NAV_WAYPOINT, [0.0; 4], 1, 1, 10.0)
            .check(0)
            .is_err());
        let mut local = item(0, NAV_WAYPOINT, [0.0; 4], 1, 1, 10.0);
        local.frame = 1;
        assert!(local.check(0).is_err());
        let mut nav_in_mission_frame = local;
        nav_in_mission_frame.frame = 2;
        assert!(nav_in_mission_frame.check(0).is_err());
    }
}
