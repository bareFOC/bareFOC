//! # motor::stepper — two-phase stepper motor
//!
//! Ports
//! [`StepperMotor.h`](source/Arduino-FOC/src/StepperMotor.h)
//! and
//! [`StepperMotor.cpp`](source/Arduino-FOC/src/StepperMotor.cpp)
//! from Arduino-FOC.
//!
//! The [`StepperMotor`] struct embeds a
//! [`base::MotorConfig<C>`] and the three optional
//! peripherals and implements the [`base::FOCMotor`] trait.
//! The four `pure-virtual` methods are:
//!
//! - [`StepperMotor::init`] — same shape as the BLDC version
//!   but with 2-phase driver semantics.
//! - [`StepperMotor::enable`] / [`StepperMotor::disable`] —
//!   same shape as the BLDC version, with 2-arg
//!   [`StepperDriver::set_pwm`].
//! - [`StepperMotor::set_phase_voltage`] — the FOC heart
//!   function. **SinePWM only** (no trapezoidal commutation
//!   for steppers). Calls the 2-arg
//!   [`StepperDriver::set_pwm`].

#![allow(non_camel_case_types)]

use crate::common::time::Clock;
use crate::math::{self, _SQRT2, DQCurrent};
use crate::motor::base::{FOCMotor, MotorConfig};
use crate::motor::enums::{
    FOCMotorStatus, MotionControlType, TorqueControlType,
};
use crate::traits::current_sense::CurrentSense;
use crate::traits::driver::StepperDriver;
use crate::traits::sensor::{Direction, Sensor};

// ---------------------------------------------------------------------------
// StepperMotor
// ---------------------------------------------------------------------------

/// Two-phase stepper motor.
///
/// Ports the upstream
/// [`StepperMotor`](source/Arduino-FOC/src/StepperMotor.h)
/// class. The struct is generic over four type parameters
/// (see `PORTING_GUIDELINES.md` §4 Pattern 1):
///
/// - `S: Sensor<C>` — the shaft sensor.
/// - `D: StepperDriver` — the two-phase gate driver
///   (`StepperDriver2PWM`, `StepperDriver4PWM`, …). The
///   `set_phase_voltage` method calls
///   [`StepperDriver::set_pwm`] (2-arg).
/// - `CS: CurrentSense` — the current sense (rarely used on
///   stepper builds, but the trait still carries it for
///   symmetry with the BLDC and hybrid builds).
/// - `C: Clock` — the microsecond clock.
///
/// The BEMF coefficient is [`_SQRT2`](crate::math::_SQRT2)
/// per the upstream `StepperMotor::estimateBEMF` formula
/// (RMS convention).
pub struct StepperMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: StepperDriver,
    CS: CurrentSense,
    C: Clock,
{
    cfg: MotorConfig<C>,
    pub driver: Option<D>,
    sensor: Option<S>,
    current_sense: Option<CS>,
    clock: &'clk C,
}

impl<'clk, S, D, CS, C> StepperMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: StepperDriver,
    CS: CurrentSense,
    C: Clock,
{
    /// Construct a new stepper motor.
    ///
    /// Ports the upstream
    /// `StepperMotor(int pp, float R, float KV, float Lq, float Ld)`
    /// constructor. `bemf_coefficient` is set to
    /// [`_SQRT2`](crate::math::_SQRT2).
    pub fn new(pp: i32, r: f32, kv: f32, lq: f32, ld: f32, clock: &'clk C) -> Self {
        let mut cfg = MotorConfig::new(clock);
        cfg.pole_pairs = pp;
        cfg.phase_resistance = r;
        cfg.KV_rating = kv;
        cfg.axis_inductance = DQCurrent { d: ld, q: lq };
        cfg.phase_inductance = lq;
        cfg.torque_controller = TorqueControlType::voltage;
        cfg.bemf_coefficient = _SQRT2;
        Self {
            cfg,
            driver: None,
            sensor: None,
            current_sense: None,
            clock,
        }
    }
}

impl<'clk, S, D, CS, C> FOCMotor<S, D, CS, C> for StepperMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: StepperDriver,
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
    // init — port of StepperMotor.cpp L38-80
    // -----------------------------------------------------------------
    fn init(&mut self) -> i32 {
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

        let driver_voltage_limit = self.driver().unwrap().params().voltage_limit;
        if self.cfg().voltage_limit > driver_voltage_limit {
            self.cfg_mut().voltage_limit = driver_voltage_limit;
        }
        if self.cfg().voltage_sensor_align > self.cfg().voltage_limit {
            self.cfg_mut().voltage_sensor_align = self.cfg().voltage_limit;
        }

        let voltage_limit = self.cfg().voltage_limit;
        let current_limit = self.cfg().current_limit;
        let velocity_limit = self.cfg().velocity_limit;
        self.update_voltage_limit(voltage_limit);
        self.update_current_limit(current_limit);
        self.update_velocity_limit(velocity_limit);

        if math::isset(self.cfg().phase_inductance)
            && !math::isset(self.cfg().axis_inductance.q)
        {
            let l = self.cfg().phase_inductance;
            self.cfg_mut().axis_inductance = DQCurrent { d: l, q: l };
        }

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
    // disable — port of StepperMotor.cpp L84-94
    // -----------------------------------------------------------------
    fn disable(&mut self) {
        if let Some(cs) = self.current_sense_mut().as_mut() {
            cs.disable();
        }
        if let Some(d) = self.driver_mut().as_mut() {
            d.set_pwm(0.0, 0.0);
            d.disable();
        }
        self.cfg_mut().enabled = 0;
    }

    // -----------------------------------------------------------------
    // enable — port of StepperMotor.cpp L96-111
    // -----------------------------------------------------------------
    fn enable(&mut self) {
        if let Some(d) = self.driver_mut().as_mut() {
            d.enable();
            d.set_pwm(0.0, 0.0);
        }
        if let Some(cs) = self.current_sense_mut().as_mut() {
            cs.enable();
        }
        self.cfg_mut().PID_velocity.reset();
        self.cfg_mut().P_angle.reset();
        self.cfg_mut().PID_current_q.reset();
        self.cfg_mut().PID_current_d.reset();
        self.cfg_mut().enabled = 1;
    }

    // -----------------------------------------------------------------
    // set_phase_voltage — port of StepperMotor.cpp L122-133
    // -----------------------------------------------------------------
    fn set_phase_voltage(&mut self, uq: f32, ud: f32, angle_el: f32) {
        // Inverse Park transform: Ualpha and Ubeta map
        // directly to the two phase voltages. No
        // centering, no modulation strategy — pure
        // sinusoidal 2-phase drive.
        let (_sa, ca) = math::sincos(angle_el);
        let ualpha = ca * ud - _sa * uq;
        let ubeta = _sa * ud + ca * uq;
        self.cfg_mut().Ualpha = ualpha;
        self.cfg_mut().Ubeta = ubeta;
        if let Some(d) = self.driver_mut().as_mut() {
            d.set_pwm(ualpha, ubeta);
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
    use crate::motor::enums::{FOCMotorStatus, MotionControlType};
    use crate::traits::current_sense::mock::MockCurrentSense;
    use crate::traits::driver::mock::MockStepperDriver;
    use crate::traits::driver::FOCDriver;
    use crate::traits::sensor::mock::MockSensor;

    type Stepper<'c> = StepperMotor<
        'c,
        MockSensor,
        MockStepperDriver,
        MockCurrentSense,
        MockClock,
    >;

    // -----------------------------------------------------------------
    // ctor — config seeded correctly with _SQRT2 BEMF coefficient
    // -----------------------------------------------------------------
    #[test]
    fn stepper_new_seeds_config() {
        let clock = MockClock::new();
        let m: Stepper = StepperMotor::new(3, 2.0, 50.0, 0.005, 0.005, &clock);
        assert_eq!(m.cfg().pole_pairs, 3);
        assert_eq!(m.cfg().phase_resistance, 2.0);
        assert_eq!(m.cfg().KV_rating, 50.0);
        assert_eq!(m.cfg().axis_inductance.d, 0.005);
        assert_eq!(m.cfg().axis_inductance.q, 0.005);
        assert_eq!(m.cfg().phase_inductance, 0.005);
        assert_eq!(m.cfg().bemf_coefficient, _SQRT2);
        assert!(m.driver().is_none());
        assert!(m.sensor().is_none());
    }

    // -----------------------------------------------------------------
    // set_phase_voltage — 2-arg call to stepper driver
    // -----------------------------------------------------------------
    #[test]
    fn set_phase_voltage_writes_two_phases() {
        let clock = MockClock::new();
        let mut m: Stepper = StepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockStepperDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        m.set_phase_voltage(1.0, 0.0, 0.0);
        let d = m.driver().unwrap();
        // angle_el = 0 → sa=0, ca=1 → ualpha = ud, ubeta = uq.
        // With uq=1, ud=0: ualpha = 0, ubeta = 1.
        assert!((d.dc_a - 0.0).abs() < 1e-5);
        assert!((d.dc_b - 1.0).abs() < 1e-5);
    }

    #[test]
    fn set_phase_voltage_persists_alpha_beta_on_config() {
        let clock = MockClock::new();
        let mut m: Stepper = StepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockStepperDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        m.set_phase_voltage(0.5, 0.3, 1.0);
        // αβ values are computed and persisted.
        let (_sa, ca) = libm::sincosf(1.0);
        let ualpha = ca * 0.3 - libm::sinf(1.0) * 0.5;
        let ubeta = libm::sinf(1.0) * 0.3 + ca * 0.5;
        assert!((m.cfg().Ualpha - ualpha).abs() < 1e-5);
        assert!((m.cfg().Ubeta - ubeta).abs() < 1e-5);
    }

    // -----------------------------------------------------------------
    // init — driver uninitialised → returns 0
    // -----------------------------------------------------------------
    #[test]
    fn init_returns_0_when_driver_not_initialised() {
        let clock = MockClock::new();
        let mut m: Stepper = StepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        m.link_driver(MockStepperDriver::new());
        let r = m.init();
        assert_eq!(r, 0);
        assert!(matches!(m.cfg().motor_status, FOCMotorStatus::motor_init_failed));
    }

    // -----------------------------------------------------------------
    // init — open-loop direction defaults to CW when no sensor
    // -----------------------------------------------------------------
    #[test]
    fn init_openloop_default_direction_is_cw() {
        let clock = MockClock::new();
        let mut m: Stepper = StepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        m.cfg_mut().controller = MotionControlType::angle_openloop;
        let mut d = MockStepperDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        m.init();
        // Open-loop without a sensor → direction defaults to CW.
        assert!(matches!(m.cfg().sensor_direction, Direction::CW));
    }

    // -----------------------------------------------------------------
    // enable / disable
    // -----------------------------------------------------------------
    #[test]
    fn enable_disabled_then_enabled_flag() {
        let clock = MockClock::new();
        let mut m: Stepper = StepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockStepperDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        m.enable();
        assert_eq!(m.cfg().enabled, 1);
        m.disable();
        assert_eq!(m.cfg().enabled, 0);
    }
}
