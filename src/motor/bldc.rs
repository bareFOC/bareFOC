//! # motor::bldc — three-phase BLDC motor
//!
//! Ports
//! [`BLDCMotor.h`](source/Arduino-FOC/src/BLDCMotor.h) and
//! [`BLDCMotor.cpp`](source/Arduino-FOC/src/BLDCMotor.cpp)
//! from Arduino-FOC.
//!
//! The [`BLDCMotor`] struct embeds a
//! [`base::MotorConfig<C>`] and the three optional
//! peripherals (`sensor: Option<S>`, `driver: Option<D>`,
//! `current_sense: Option<CS>`) and implements the
//! [`base::FOCMotor`] trait. The four `pure-virtual` methods
//! on the trait are supplied here:
//!
//! - [`BLDCMotor::init`] — sanity-check the driver, the
//!   voltage limit, and the sensor-align voltage; reset the
//!   PIDs; enable the driver.
//! - [`BLDCMotor::enable`] — enable the driver's output
//!   stage, the current sense, and reset the four PIDs.
//! - [`BLDCMotor::disable`] — disable the current sense,
//!   set the driver's PWM to zero, disable the driver.
//! - [`BLDCMotor::set_phase_voltage`] — the FOC heart
//!   function. Dispatches on `foc_modulation` to one of the
//!   four PWM strategies (SinePWM, SpaceVectorPWM,
//!   Trapezoid_120, Trapezoid_150). The two trapezoidal
//!   strategies consult the [`TRAP_120_MAP`] /
//!   [`TRAP_150_MAP`] sector tables and call
//!   [`BLDCDriver::set_phase_state`] to disable the
//!   floating phase each electrical sector.

#![allow(non_camel_case_types)]
#![allow(clippy::approx_constant, clippy::excessive_precision)]

use crate::common::time::Clock;
use crate::math::{self, _2PI, _PI_6, _SQRT3, _SQRT3_2, _HIGH_IMPEDANCE, DQCurrent};
use crate::motor::base::{FOCMotor, MotorConfig};
use crate::motor::enums::{
    FOCMotorStatus, FOCModulationType, MotionControlType, TorqueControlType,
};
use crate::traits::current_sense::CurrentSense;
use crate::traits::driver::{BLDCDriver, PhaseState};
use crate::traits::sensor::{Direction, Sensor};

// ---------------------------------------------------------------------------
// Trapezoidal commutation tables
// ---------------------------------------------------------------------------
//
// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
// each is 60 degrees with values for 3 phases of
// 1 = positive, -1 = negative, 0 = high-z
const TRAP_120_MAP: [[i32; 3]; 6] = [
    [_HIGH_IMPEDANCE, 1, -1],
    [-1, 1, _HIGH_IMPEDANCE],
    [-1, _HIGH_IMPEDANCE, 1],
    [_HIGH_IMPEDANCE, -1, 1],
    [1, -1, _HIGH_IMPEDANCE],
    [1, _HIGH_IMPEDANCE, -1],
];

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
// each is 30 degrees with values for 3 phases of
// 1 = positive, -1 = negative, 0 = high-z
const TRAP_150_MAP: [[i32; 3]; 12] = [
    [_HIGH_IMPEDANCE, 1, -1],
    [-1, 1, -1],
    [-1, 1, _HIGH_IMPEDANCE],
    [-1, 1, 1],
    [-1, _HIGH_IMPEDANCE, 1],
    [-1, -1, 1],
    [_HIGH_IMPEDANCE, -1, 1],
    [1, -1, 1],
    [1, -1, _HIGH_IMPEDANCE],
    [1, -1, -1],
    [1, _HIGH_IMPEDANCE, -1],
    [1, 1, -1],
];

// ---------------------------------------------------------------------------
// BLDCMotor
// ---------------------------------------------------------------------------

/// Three-phase brushless DC motor.
///
/// Ports the upstream
/// [`BLDCMotor`](source/Arduino-FOC/src/BLDCMotor.h) class.
/// The struct is generic over four type parameters so the
/// compiler can monomorphise the FOC hot loop — see
/// `PORTING_GUIDELINES.md` §4 Pattern 1.
///
/// # Generic parameters
///
/// - `S: Sensor<C>` — the shaft sensor (encoder, hall,
///   magnetic, …). Required by [`FOCMotor::shaft_angle`]
///   and [`FOCMotor::electrical_angle`].
/// - `D: BLDCDriver` — the three-phase gate driver
///   (`BLDCDriver3PWM`, `BLDCDriver6PWM`, …). The
///   `set_phase_voltage` method calls
///   [`BLDCDriver::set_pwm`] (3-arg) and
///   [`BLDCDriver::set_phase_state`] (3-arg).
/// - `CS: CurrentSense` — the inline / lowside current
///   sense. Optional in [`FOCMotor::init_foc`]; required
///   for `foc_current` torque control.
/// - `C: Clock` — the microsecond clock used by the four
///   PIDs, the four LPFs, and the calibration routines.
///
/// # Lifetime parameter
///
/// The `'clk` lifetime tracks the borrowed clock reference
/// the constructor receives. Storing the reference rather
/// than an owned clock keeps the struct `no_std`-friendly
/// (no allocation, no `Default` requirement on the clock)
/// and lets a single clock be shared across many motors.
pub struct BLDCMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: BLDCDriver,
    CS: CurrentSense,
    C: Clock,
{
    cfg: MotorConfig<C>,
    pub driver: Option<D>,
    sensor: Option<S>,
    current_sense: Option<CS>,
    clock: &'clk C,
}

impl<'clk, S, D, CS, C> BLDCMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: BLDCDriver,
    CS: CurrentSense,
    C: Clock,
{
    /// Construct a new BLDC motor with the five motor
    /// parameters and a clock reference.
    ///
    /// Ports the upstream
    /// `BLDCMotor(int pp, float R, float KV, float Lq, float Ld)`
    /// constructor. The five motor parameters seed
    /// `pole_pairs`, `phase_resistance`, `KV_rating`, and
    /// `axis_inductance` / `phase_inductance` on the embedded
    /// `MotorConfig`. `bemf_coefficient` is set to
    /// [`_SQRT3`](crate::math::_SQRT3) per the upstream
    /// `BLDCMotor::estimateBEMF` formula.
    pub fn new(pp: i32, r: f32, kv: f32, lq: f32, ld: f32, clock: &'clk C) -> Self {
        let mut cfg = MotorConfig::new(clock);
        cfg.pole_pairs = pp;
        cfg.phase_resistance = r;
        cfg.KV_rating = kv;
        cfg.axis_inductance = DQCurrent { d: ld, q: lq };
        cfg.phase_inductance = lq;
        cfg.torque_controller = TorqueControlType::voltage;
        cfg.bemf_coefficient = _SQRT3;
        Self {
            cfg,
            driver: None,
            sensor: None,
            current_sense: None,
            clock,
        }
    }
}

// ---------------------------------------------------------------------------
// FOCMotor trait impl
// ---------------------------------------------------------------------------

impl<'clk, S, D, CS, C> FOCMotor<S, D, CS, C> for BLDCMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: BLDCDriver,
    CS: CurrentSense,
    C: Clock,
{
    fn cfg(&self) -> &MotorConfig<C> {
        &self.cfg
    }
    fn cfg_mut(&mut self) -> &mut MotorConfig<C> {
        &mut self.cfg
    }
    fn sensor(&self) -> Option<&S> {
        self.sensor.as_ref()
    }
    fn sensor_mut(&mut self) -> &mut Option<S> {
        &mut self.sensor
    }
    fn driver(&self) -> Option<&D> {
        self.driver.as_ref()
    }
    fn driver_mut(&mut self) -> &mut Option<D> {
        &mut self.driver
    }
    fn current_sense(&self) -> Option<&CS> {
        self.current_sense.as_ref()
    }
    fn current_sense_mut(&mut self) -> &mut Option<CS> {
        &mut self.current_sense
    }

    // -----------------------------------------------------------------
    // init — port of BLDCMotor.cpp L66-107
    // -----------------------------------------------------------------
    fn init(&mut self) -> i32 {
        // Driver not linked or not initialised → cannot init.
        if self.driver.is_none()
            || !self
                .driver
                .as_ref()
                .unwrap()
                .params()
                .initialized
        {
            self.cfg_mut().motor_status = FOCMotorStatus::motor_init_failed;
            return 0;
        }
        self.cfg_mut().motor_status = FOCMotorStatus::motor_initializing;

        // Sanity check the voltage limit configuration:
        // motor voltage_limit must not exceed the driver's.
        let driver_voltage_limit = self.driver().unwrap().params().voltage_limit;
        if self.cfg().voltage_limit > driver_voltage_limit {
            self.cfg_mut().voltage_limit = driver_voltage_limit;
        }
        // And the sensor-align voltage must not exceed the
        // (possibly tightened) motor voltage limit.
        if self.cfg().voltage_sensor_align > self.cfg().voltage_limit {
            self.cfg_mut().voltage_sensor_align = self.cfg().voltage_limit;
        }

        // Propagate the new limits into the controllers.
        let voltage_limit = self.cfg().voltage_limit;
        let current_limit = self.cfg().current_limit;
        let velocity_limit = self.cfg().velocity_limit;
        self.update_voltage_limit(voltage_limit);
        self.update_current_limit(current_limit);
        self.update_velocity_limit(velocity_limit);

        // If only a single inductance was set, mirror it to both
        // d and q axes.
        if math::isset(self.cfg().phase_inductance)
            && !math::isset(self.cfg().axis_inductance.q)
        {
            let l = self.cfg().phase_inductance;
            self.cfg_mut().axis_inductance = DQCurrent { d: l, q: l };
        }

        // Open-loop direction default.
        if self.sensor().is_none() {
            let c = self.cfg().controller;
            let d = self.cfg().sensor_direction;
            if (c == MotionControlType::angle_openloop
                || c == MotionControlType::velocity_openloop)
                && d == Direction::UNKNOWN
            {
                self.cfg_mut().sensor_direction = Direction::CW;
            }
        }

        self.clock.delay_ms(500);
        self.enable();
        self.clock.delay_ms(500);
        self.cfg_mut().motor_status = FOCMotorStatus::motor_uncalibrated;
        1
    }

    // -----------------------------------------------------------------
    // disable — port of BLDCMotor.cpp L111-121
    // -----------------------------------------------------------------
    fn disable(&mut self) {
        if let Some(cs) = self.current_sense_mut().as_mut() {
            cs.disable();
        }
        if let Some(d) = self.driver_mut().as_mut() {
            d.set_pwm(0.0, 0.0, 0.0);
            d.disable();
        }
        self.cfg_mut().enabled = 0;
    }

    // -----------------------------------------------------------------
    // enable — port of BLDCMotor.cpp L123-138
    // -----------------------------------------------------------------
    fn enable(&mut self) {
        if let Some(d) = self.driver_mut().as_mut() {
            d.enable();
            d.set_pwm(0.0, 0.0, 0.0);
        }
        if let Some(cs) = self.current_sense_mut().as_mut() {
            cs.enable();
        }
        // Reset the four PIDs.
        self.cfg_mut().PID_velocity.reset();
        self.cfg_mut().P_angle.reset();
        self.cfg_mut().PID_current_q.reset();
        self.cfg_mut().PID_current_d.reset();
        self.cfg_mut().enabled = 1;
    }

    // -----------------------------------------------------------------
    // set_phase_voltage — port of BLDCMotor.cpp L153-269
    // -----------------------------------------------------------------
    fn set_phase_voltage(&mut self, uq: f32, ud: f32, angle_el: f32) {
        // Compute Ua / Ub / Uc into locals first, then commit
        // to the driver at the end. Local scope lets us early
        // return from the trapezoid branches without leaving
        // half-set state.
        let mut ua: f32;
        let mut ub: f32;
        let mut uc: f32;

        match self.cfg().foc_modulation {
            FOCModulationType::Trapezoid_120 => {
                // Determine the sector (0..=5) and centre the
                // voltages around either voltage_limit/2
                // (centred modulation) or Uq (adaptable
                // centering, all phases drawn to 0 when Uq=0).
                let normalized =
                    math::normalize_angle(angle_el + _PI_6);
                let sector: usize =
                    (6.0 * (normalized / _2PI)) as usize;
                let center = if self.cfg().modulation_centered != 0
                {
                    self.driver().unwrap().params().voltage_limit
                        / 2.0
                } else {
                    uq
                };

                if TRAP_120_MAP[sector][0] == _HIGH_IMPEDANCE {
                    ua = center;
                    ub = TRAP_120_MAP[sector][1] as f32 * uq
                        + center;
                    uc = TRAP_120_MAP[sector][2] as f32 * uq
                        + center;
                    self.driver_mut().as_mut().unwrap().set_phase_state(
                        PhaseState::PHASE_OFF,
                        PhaseState::PHASE_ON,
                        PhaseState::PHASE_ON,
                    );
                } else if TRAP_120_MAP[sector][1]
                    == _HIGH_IMPEDANCE
                {
                    ua = TRAP_120_MAP[sector][0] as f32 * uq
                        + center;
                    ub = center;
                    uc = TRAP_120_MAP[sector][2] as f32 * uq
                        + center;
                    self.driver_mut().as_mut().unwrap().set_phase_state(
                        PhaseState::PHASE_ON,
                        PhaseState::PHASE_OFF,
                        PhaseState::PHASE_ON,
                    );
                } else {
                    ua = TRAP_120_MAP[sector][0] as f32 * uq
                        + center;
                    ub = TRAP_120_MAP[sector][1] as f32 * uq
                        + center;
                    uc = center;
                    self.driver_mut().as_mut().unwrap().set_phase_state(
                        PhaseState::PHASE_ON,
                        PhaseState::PHASE_ON,
                        PhaseState::PHASE_OFF,
                    );
                }
            }
            FOCModulationType::Trapezoid_150 => {
                let normalized =
                    math::normalize_angle(angle_el + _PI_6);
                let sector: usize =
                    (12.0 * (normalized / _2PI)) as usize;
                let center = if self.cfg().modulation_centered != 0
                {
                    self.driver().unwrap().params().voltage_limit
                        / 2.0
                } else {
                    uq
                };

                if TRAP_150_MAP[sector][0] == _HIGH_IMPEDANCE {
                    ua = center;
                    ub = TRAP_150_MAP[sector][1] as f32 * uq
                        + center;
                    uc = TRAP_150_MAP[sector][2] as f32 * uq
                        + center;
                    self.driver_mut().as_mut().unwrap().set_phase_state(
                        PhaseState::PHASE_OFF,
                        PhaseState::PHASE_ON,
                        PhaseState::PHASE_ON,
                    );
                } else if TRAP_150_MAP[sector][1]
                    == _HIGH_IMPEDANCE
                {
                    ua = TRAP_150_MAP[sector][0] as f32 * uq
                        + center;
                    ub = center;
                    uc = TRAP_150_MAP[sector][2] as f32 * uq
                        + center;
                    self.driver_mut().as_mut().unwrap().set_phase_state(
                        PhaseState::PHASE_ON,
                        PhaseState::PHASE_OFF,
                        PhaseState::PHASE_ON,
                    );
                } else if TRAP_150_MAP[sector][2]
                    == _HIGH_IMPEDANCE
                {
                    ua = TRAP_150_MAP[sector][0] as f32 * uq
                        + center;
                    ub = TRAP_150_MAP[sector][1] as f32 * uq
                        + center;
                    uc = center;
                    self.driver_mut().as_mut().unwrap().set_phase_state(
                        PhaseState::PHASE_ON,
                        PhaseState::PHASE_ON,
                        PhaseState::PHASE_OFF,
                    );
                } else {
                    ua = TRAP_150_MAP[sector][0] as f32 * uq
                        + center;
                    ub = TRAP_150_MAP[sector][1] as f32 * uq
                        + center;
                    uc = TRAP_150_MAP[sector][2] as f32 * uq
                        + center;
                    self.driver_mut().as_mut().unwrap().set_phase_state(
                        PhaseState::PHASE_ON,
                        PhaseState::PHASE_ON,
                        PhaseState::PHASE_ON,
                    );
                }
            }
            FOCModulationType::SinePWM
            | FOCModulationType::SpaceVectorPWM => {
                // Inverse Park + Clarke transforms.
                let (_sa, ca) = math::sincos(angle_el);
                let ualpha = ca * ud - _sa * uq;
                let ubeta = _sa * ud + ca * uq;
                // Persist the αβ voltages on the config so the
                // user can read them for telemetry. Upstream
                // sets these in the same branch (only the
                // sine/SVPWM path uses them).
                self.cfg_mut().Ualpha = ualpha;
                self.cfg_mut().Ubeta = ubeta;
                ua = ualpha;
                ub = -0.5 * ualpha + _SQRT3_2 * ubeta;
                uc = -0.5 * ualpha - _SQRT3_2 * ubeta;

                if self.cfg().modulation_centered != 0 {
                    let mut center = self
                        .driver()
                        .unwrap()
                        .params()
                        .voltage_limit
                        / 2.0;
                    if self.cfg().foc_modulation
                        == FOCModulationType::SpaceVectorPWM
                    {
                        // Midpoint clamp.
                        let umin = ua.min(ub).min(uc);
                        let umax = ua.max(ub).max(uc);
                        center -= (umax + umin) / 2.0;
                    }
                    ua += center;
                    ub += center;
                    uc += center;
                } else {
                    // Pull Umin to 0 (low-side current-sense
                    // support).
                    let umin = ua.min(ub).min(uc);
                    ua -= umin;
                    ub -= umin;
                    uc -= umin;
                }
            }
        }

        if let Some(d) = self.driver_mut().as_mut() {
            d.set_pwm(ua, ub, uc);
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::time::mock::MockClock;
    use crate::motor::enums::{FOCModulationType, FOCMotorStatus};
    use crate::traits::current_sense::mock::MockCurrentSense;
    use crate::traits::driver::mock::MockBldcDriver;
    use crate::traits::driver::{PhaseState, FOCDriver};
    use crate::traits::sensor::mock::MockSensor;

    type Bldc<'c> = BLDCMotor<
        'c,
        MockSensor,
        MockBldcDriver,
        MockCurrentSense,
        MockClock,
    >;

    // -----------------------------------------------------------------
    // ctor — config seeded correctly
    // -----------------------------------------------------------------
    #[test]
    fn bldc_new_seeds_config() {
        let clock = MockClock::new();
        let m: Bldc = BLDCMotor::new(7, 1.5, 100.0, 0.01, 0.01, &clock);
        assert_eq!(m.cfg().pole_pairs, 7);
        assert_eq!(m.cfg().phase_resistance, 1.5);
        assert_eq!(m.cfg().KV_rating, 100.0);
        assert_eq!(m.cfg().axis_inductance.d, 0.01);
        assert_eq!(m.cfg().axis_inductance.q, 0.01);
        assert_eq!(m.cfg().phase_inductance, 0.01);
        assert_eq!(m.cfg().bemf_coefficient, _SQRT3);
        assert!(matches!(m.cfg().foc_modulation, FOCModulationType::SinePWM));
        assert!(m.driver().is_none());
        assert!(m.sensor().is_none());
        assert!(m.current_sense().is_none());
    }

    // -----------------------------------------------------------------
    // TRAP_120_MAP — table shape and content
    // -----------------------------------------------------------------
    #[test]
    fn trap_120_map_has_six_sectors_and_one_high_z_per_row() {
        assert_eq!(TRAP_120_MAP.len(), 6);
        for row in TRAP_120_MAP.iter() {
            assert_eq!(row.len(), 3);
            // Exactly one _HIGH_IMPEDANCE per row.
            let hiz_count =
                row.iter().filter(|v| **v == _HIGH_IMPEDANCE).count();
            assert_eq!(hiz_count, 1);
            // Remaining entries must be ±1.
            for v in row.iter() {
                assert!(*v == 1 || *v == -1 || *v == _HIGH_IMPEDANCE);
            }
        }
    }

    #[test]
    fn trap_150_map_has_twelve_sectors_and_one_high_z_per_row() {
        assert_eq!(TRAP_150_MAP.len(), 12);
        for row in TRAP_150_MAP.iter() {
            let hiz_count =
                row.iter().filter(|v| **v == _HIGH_IMPEDANCE).count();
            assert!(hiz_count <= 1);
        }
    }

    // -----------------------------------------------------------------
    // set_phase_voltage — SinePWM branch with centred modulation
    // -----------------------------------------------------------------
    #[test]
    fn set_phase_voltage_sine_pwm_centre_modulation() {
        let clock = MockClock::new();
        let mut m: Bldc = BLDCMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        let vlim = d.params.voltage_limit;
        m.link_driver(d);
        m.cfg_mut().foc_modulation = FOCModulationType::SinePWM;
        m.cfg_mut().modulation_centered = 1;
        m.set_phase_voltage(1.0, 0.0, 0.0);
        let d = m.driver().unwrap();
        // Ualpha = 1.0, Ubeta = 0 → centred around voltage_limit/2.
        // All three phases should be within [0, voltage_limit].
        assert!(d.dc_a > 0.0 && d.dc_a < vlim);
        assert!(d.dc_b > 0.0 && d.dc_b < vlim);
        assert!(d.dc_c > 0.0 && d.dc_c < vlim);
    }

    #[test]
    fn set_phase_voltage_sine_pwm_non_centre_modulation() {
        let clock = MockClock::new();
        let mut m: Bldc = BLDCMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        m.cfg_mut().foc_modulation = FOCModulationType::SinePWM;
        m.cfg_mut().modulation_centered = 0;
        m.set_phase_voltage(1.0, 0.0, 0.0);
        let d = m.driver().unwrap();
        // Pulled to 0: min(ua,ub,uc) subtracted, so one phase is 0.
        let min_v = d.dc_a.min(d.dc_b).min(d.dc_c);
        assert!(min_v.abs() < 1e-5);
    }

    // -----------------------------------------------------------------
    // set_phase_voltage — trapezoid branches set phase_state
    // -----------------------------------------------------------------
    #[test]
    fn set_phase_voltage_trap_120_records_phase_state() {
        let clock = MockClock::new();
        let mut m: Bldc = BLDCMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        m.cfg_mut().foc_modulation = FOCModulationType::Trapezoid_120;
        m.set_phase_voltage(1.0, 0.0, 0.0);
        let d = m.driver().unwrap();
        // Exactly one phase must be OFF and the other two ON.
        let (sa, sb, sc) = d.last_phase.expect("set_phase_state called");
        let off_count = [sa, sb, sc]
            .iter()
            .filter(|p| **p == PhaseState::PHASE_OFF)
            .count();
        assert_eq!(off_count, 1);
    }

    #[test]
    fn set_phase_voltage_trap_150_records_phase_state() {
        let clock = MockClock::new();
        let mut m: Bldc = BLDCMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        m.cfg_mut().foc_modulation = FOCModulationType::Trapezoid_150;
        m.set_phase_voltage(1.0, 0.0, 0.0);
        let d = m.driver().unwrap();
        // 150° has 0 or 1 OFF phases depending on sector; the
        // important thing is `set_phase_state` was called.
        assert!(d.last_phase.is_some());
    }

    // -----------------------------------------------------------------
    // init — driver uninitialised → returns 0
    // -----------------------------------------------------------------
    #[test]
    fn init_returns_0_when_driver_not_initialised() {
        let clock = MockClock::new();
        let mut m: Bldc = BLDCMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        // Link a driver but don't init it.
        m.link_driver(MockBldcDriver::new());
        let r = m.init();
        assert_eq!(r, 0);
        assert!(matches!(m.cfg().motor_status, FOCMotorStatus::motor_init_failed));
    }

    // -----------------------------------------------------------------
    // init — driver initialised, no sensor → uncalibrated
    // -----------------------------------------------------------------
    #[test]
    fn init_returns_1_and_uncalibrated_without_sensor() {
        let clock = MockClock::new();
        let mut m: Bldc = BLDCMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        let r = m.init();
        assert_eq!(r, 1);
        assert!(matches!(m.cfg().motor_status, FOCMotorStatus::motor_uncalibrated));
    }

    // -----------------------------------------------------------------
    // init — voltage_limit is clamped to the driver's
    // -----------------------------------------------------------------
    #[test]
    fn init_clamps_voltage_limit_to_driver() {
        let clock = MockClock::new();
        let mut m: Bldc = BLDCMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        m.cfg_mut().voltage_limit = 100.0; // driver default is 12
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        let vlim = d.params.voltage_limit;
        m.link_driver(d);
        m.init();
        assert!(m.cfg().voltage_limit <= vlim);
    }
}
