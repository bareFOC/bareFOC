//! # motor::hybrid_stepper — hybrid 2-phase-on-3-driver motor
//!
//! Ports
//! [`HybridStepperMotor.h`](source/Arduino-FOC/src/HybridStepperMotor.h)
//! and
//! [`HybridStepperMotor.cpp`](source/Arduino-FOC/src/HybridStepperMotor.cpp)
//! from Arduino-FOC.
//!
//! Hybrid stepper motors use a 3-phase BLDC driver (the
//! "C" phase is mid-rail biased) to drive a 2-phase stepper
//! coil pair. The C phase is held at the rail midpoint
//! during SinePWM, modulated to extend the linear range
//! during SVPWM, and forced to zero in the (unsupported)
//! trapezoidal commutation modes.
//!
//! The [`HybridStepperMotor`] struct implements the
//! [`base::FOCMotor`] trait. The four `pure-virtual`
//! methods are:
//!
//! - [`HybridStepperMotor::init`] /
//!   [`HybridStepperMotor::enable`] /
//!   [`HybridStepperMotor::disable`] — same shape as the
//!   BLDC version (3-arg
//!   [`BLDCDriver::set_pwm`](crate::traits::driver::BLDCDriver::set_pwm)).
//! - [`HybridStepperMotor::set_phase_voltage`] — the FOC
//!   heart function with three modulation branches:
//!   `SinePWM` (Uc fixed at voltage_limit/2),
//!   `SpaceVectorPWM` (Uc = Vo where Vo extends the linear
//!   range), and `Trapezoid_*` (forced to `set_pwm(0, 0, 0)`).
//!
//! The override of
//! [`base::FOCMotor::link_current_sense`] sets
//! [`CurrentSenseBase::driver_type`](crate::traits::current_sense::CurrentSenseBase::driver_type)
//! to
//! [`DriverType::Hybrid`](crate::traits::driver::DriverType::Hybrid)
//! so the Clarke-transform branch picks the "missing-phase
//! reconstruction" path.

#![allow(non_camel_case_types)]

use crate::common::time::Clock;
use crate::math::{self, _SQRT2, DQCurrent};
use crate::motor::base::{FOCMotor, MotorConfig};
use crate::motor::enums::{
    FOCMotorStatus, FOCModulationType, MotionControlType, TorqueControlType,
};
use crate::traits::current_sense::CurrentSense;
use crate::traits::driver::{BLDCDriver, DriverType, PhaseState};
use crate::traits::sensor::{Direction, Sensor};

// ---------------------------------------------------------------------------
// HybridStepperMotor
// ---------------------------------------------------------------------------

/// Hybrid stepper motor (2-phase coils, 3-phase driver).
///
/// Ports the upstream
/// [`HybridStepperMotor`](source/Arduino-FOC/src/HybridStepperMotor.h)
/// class. The struct is generic over four type parameters
/// (see `PORTING_GUIDELINES.md` §4 Pattern 1):
///
/// - `S: Sensor<C>` — the shaft sensor.
/// - `D: BLDCDriver` — the **3-phase** gate driver (the C
///   phase is the mid-rail bias). The `set_phase_voltage`
///   method calls [`BLDCDriver::set_pwm`] (3-arg) and
///   [`BLDCDriver::set_phase_state`] (3-arg).
/// - `CS: CurrentSense` — the current sense. Note that
///   the [`link_current_sense`](FOCMotor::link_current_sense)
///   override sets `cs.base_mut().driver_type` to
///   [`DriverType::Hybrid`] so the Clarke branch
///   reconstructs the missing phase.
/// - `C: Clock` — the microsecond clock.
pub struct HybridStepperMotor<'clk, S, D, CS, C>
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

impl<'clk, S, D, CS, C> HybridStepperMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: BLDCDriver,
    CS: CurrentSense,
    C: Clock,
{
    /// Construct a new hybrid stepper motor.
    ///
    /// Ports the upstream
    /// `HybridStepperMotor(int pp, float R, float KV, float Lq, float Ld)`
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

impl<'clk, S, D, CS, C> FOCMotor<S, D, CS, C>
    for HybridStepperMotor<'clk, S, D, CS, C>
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
    // link_current_sense — override of FOCMotor's default.
    //
    // The default just stores the CS. The hybrid motor must
    // also set `cs.base_mut().driver_type = DriverType::Hybrid`
    // so the Clarke transform's Hybrid branch (reconstruct the
    // missing phase from `a + b + c = 0`) is selected.
    // -----------------------------------------------------------------
    fn link_current_sense(&mut self, mut cs: CS) {
        cs.base_mut().driver_type = DriverType::Hybrid;
        *self.current_sense_mut() = Some(cs);
    }

    // -----------------------------------------------------------------
    // init — port of HybridStepperMotor.cpp L47-92
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
    // disable — port of HybridStepperMotor.cpp L95-105
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
    // enable — port of HybridStepperMotor.cpp L108-123
    // -----------------------------------------------------------------
    fn enable(&mut self) {
        if let Some(d) = self.driver_mut().as_mut() {
            d.enable();
            d.set_pwm(0.0, 0.0, 0.0);
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
    // set_phase_voltage — port of HybridStepperMotor.cpp L135-176
    // -----------------------------------------------------------------
    fn set_phase_voltage(&mut self, uq: f32, ud: f32, angle_el: f32) {
        let (_sa, ca) = math::sincos(angle_el);
        let mut ua = ca * ud - _sa * uq;
        let mut ub = _sa * ud + ca * uq;
        // The 2-phase compute (above) is identical to the
        // stepper path. The C phase is what makes this a
        // hybrid: it carries the mid-rail bias (SinePWM)
        // or the extended linear-range offset (SVPWM).
        let uc: f32;
        self.cfg_mut().Ualpha = ua;
        self.cfg_mut().Ubeta = ub;

        match self.cfg().foc_modulation {
            FOCModulationType::Trapezoid_120
            | FOCModulationType::Trapezoid_150 => {
                // Trapezoidal commutation is not supported for
                // hybrid steppers. Force all three PWMs to 0.
                ua = 0.0;
                ub = 0.0;
                uc = 0.0;
            }
            FOCModulationType::SinePWM => {
                // C phase fixed at half-rail to provide the
                // bias point for the A, B legs.
                let center =
                    self.driver().unwrap().params().voltage_limit
                        / 2.0;
                ua += center;
                ub += center;
                uc = center;
            }
            FOCModulationType::SpaceVectorPWM => {
                // C phase moves to extend the linear range.
                let umin = ua.min(ub).min(0.0);
                let umax = ua.max(ub).max(0.0);
                let vo = -(umin + umax) / 2.0
                    + self.driver().unwrap().params().voltage_limit
                        / 2.0;
                ua += vo;
                ub += vo;
                uc = vo;
            }
        }
        let _ = PhaseState::PHASE_ON; // keep import used
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
    use crate::traits::current_sense::CurrentSense;
    use crate::traits::driver::mock::MockBldcDriver;
    use crate::traits::driver::{DriverType, FOCDriver};
    use crate::traits::sensor::mock::MockSensor;

    type Hybrid<'c> = HybridStepperMotor<
        'c,
        MockSensor,
        MockBldcDriver,
        MockCurrentSense,
        MockClock,
    >;

    // -----------------------------------------------------------------
    // ctor — uses _SQRT2 BEMF coefficient (RMS convention)
    // -----------------------------------------------------------------
    #[test]
    fn hybrid_new_seeds_config() {
        let clock = MockClock::new();
        let m: Hybrid = HybridStepperMotor::new(1, 1.0, 100.0, 0.01, 0.01, &clock);
        assert_eq!(m.cfg().bemf_coefficient, _SQRT2);
        assert!(m.driver().is_none());
        assert!(m.sensor().is_none());
    }

    // -----------------------------------------------------------------
    // link_current_sense — sets driver_type to Hybrid
    // -----------------------------------------------------------------
    #[test]
    fn link_current_sense_overrides_driver_type_to_hybrid() {
        let clock = MockClock::new();
        let mut m: Hybrid =
            HybridStepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let cs = MockCurrentSense::new();
        m.link_current_sense(cs);
        let stored = m.current_sense().unwrap();
        assert_eq!(stored.base().driver_type, DriverType::Hybrid);
    }

    // -----------------------------------------------------------------
    // init — driver uninitialised → returns 0
    // -----------------------------------------------------------------
    #[test]
    fn init_returns_0_when_driver_not_initialised() {
        let clock = MockClock::new();
        let mut m: Hybrid =
            HybridStepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
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
        let mut m: Hybrid =
            HybridStepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        let r = m.init();
        assert_eq!(r, 1);
        assert!(matches!(m.cfg().motor_status, FOCMotorStatus::motor_uncalibrated));
    }

    // -----------------------------------------------------------------
    // set_phase_voltage — SinePWM forces C to mid-rail
    // -----------------------------------------------------------------
    #[test]
    fn set_phase_voltage_sine_pwm_c_is_midrail() {
        let clock = MockClock::new();
        let mut m: Hybrid =
            HybridStepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        let vlim = d.params.voltage_limit;
        m.link_driver(d);
        m.cfg_mut().foc_modulation = FOCModulationType::SinePWM;
        m.set_phase_voltage(0.0, 0.0, 0.0);
        let d = m.driver().unwrap();
        // C phase is fixed at voltage_limit / 2.
        assert!((d.dc_c - vlim / 2.0).abs() < 1e-5);
    }

    // -----------------------------------------------------------------
    // set_phase_voltage — SVPWM extends linear range
    // -----------------------------------------------------------------
    #[test]
    fn set_phase_voltage_svpwm_c_is_offset() {
        let clock = MockClock::new();
        let mut m: Hybrid =
            HybridStepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        let vlim = d.params.voltage_limit;
        m.link_driver(d);
        m.cfg_mut().foc_modulation = FOCModulationType::SpaceVectorPWM;
        m.set_phase_voltage(0.0, 0.0, 0.0);
        let d = m.driver().unwrap();
        // SVPWM offset: vo = -(0 + 0)/2 + vlim/2 = vlim/2.
        assert!((d.dc_c - vlim / 2.0).abs() < 1e-5);
    }

    // -----------------------------------------------------------------
    // set_phase_voltage — Trapezoid forces all phases to 0
    // -----------------------------------------------------------------
    #[test]
    fn set_phase_voltage_trap_forces_all_to_zero() {
        let clock = MockClock::new();
        let mut m: Hybrid =
            HybridStepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        m.cfg_mut().foc_modulation = FOCModulationType::Trapezoid_120;
        m.set_phase_voltage(1.0, 0.0, 0.0);
        let d = m.driver().unwrap();
        assert_eq!(d.dc_a, 0.0);
        assert_eq!(d.dc_b, 0.0);
        assert_eq!(d.dc_c, 0.0);
    }

    // -----------------------------------------------------------------
    // set_phase_voltage — persists Ualpha and Ubeta
    // -----------------------------------------------------------------
    #[test]
    fn set_phase_voltage_persists_alpha_beta() {
        let clock = MockClock::new();
        let mut m: Hybrid =
            HybridStepperMotor::new(1, 1.0, 100.0, 0.0, 0.0, &clock);
        let mut d = MockBldcDriver::new();
        d.init().unwrap();
        m.link_driver(d);
        m.set_phase_voltage(0.5, 0.3, 1.0);
        let (_sa, ca) = libm::sincosf(1.0);
        let ualpha = ca * 0.3 - libm::sinf(1.0) * 0.5;
        let ubeta = libm::sinf(1.0) * 0.3 + ca * 0.5;
        assert!((m.cfg().Ualpha - ualpha).abs() < 1e-5);
        assert!((m.cfg().Ubeta - ubeta).abs() < 1e-5);
    }
}
