//! # actuator::motor — unified motor control trait
//!
//! Abstracts the core methods required by cascaded actuators and calibration routines
//! ([`MotorControl`]), with out-of-the-box implementations for [`BLDCMotor`] and [`StepperMotor`].

use crate::common::time::Clock;
use crate::motor::base::FOCMotor;
use crate::motor::bldc::BLDCMotor;
use crate::motor::stepper::StepperMotor;
use crate::traits::current_sense::CurrentSense;
use crate::traits::driver::{BLDCDriver, StepperDriver};
use crate::traits::sensor::Sensor;

/// Unified motor control trait for actuator loops and calibration routines.
pub trait MotorControl<C: Clock> {
    /// Return current motor rotor angle in radians.
    fn shaft_angle(&self) -> f32;

    /// Return filtered motor rotor velocity in rad/s.
    fn shaft_velocity(&mut self) -> f32;

    /// Return configured motor velocity limit in rad/s.
    fn velocity_limit(&self) -> f32;

    /// Command target velocity or torque to inner motor controller.
    fn move_motor(&mut self, target: f32, clock: &C);

    /// Step high-speed inner FOC commutation loop.
    fn loop_foc(&mut self, clock: &C);
}

impl<'clk, S, D, CS, C> MotorControl<C> for BLDCMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: BLDCDriver,
    CS: CurrentSense,
    C: Clock,
{
    #[inline]
    fn shaft_angle(&self) -> f32 {
        FOCMotor::shaft_angle(self)
    }

    #[inline]
    fn shaft_velocity(&mut self) -> f32 {
        FOCMotor::shaft_velocity(self)
    }

    #[inline]
    fn velocity_limit(&self) -> f32 {
        self.cfg().velocity_limit
    }

    #[inline]
    fn move_motor(&mut self, target: f32, clock: &C) {
        FOCMotor::move_motor(self, target, clock);
    }

    #[inline]
    fn loop_foc(&mut self, clock: &C) {
        FOCMotor::loop_foc(self, clock);
    }
}

impl<'clk, S, D, CS, C> MotorControl<C> for StepperMotor<'clk, S, D, CS, C>
where
    S: Sensor<C>,
    D: StepperDriver,
    CS: CurrentSense,
    C: Clock,
{
    #[inline]
    fn shaft_angle(&self) -> f32 {
        FOCMotor::shaft_angle(self)
    }

    #[inline]
    fn shaft_velocity(&mut self) -> f32 {
        FOCMotor::shaft_velocity(self)
    }

    #[inline]
    fn velocity_limit(&self) -> f32 {
        self.cfg().velocity_limit
    }

    #[inline]
    fn move_motor(&mut self, target: f32, clock: &C) {
        FOCMotor::move_motor(self, target, clock);
    }

    #[inline]
    fn loop_foc(&mut self, clock: &C) {
        FOCMotor::loop_foc(self, clock);
    }
}
