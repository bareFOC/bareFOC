//! # motor — motor control abstractions
//!
//! This module collects the shared motor control code: a
//! [`base::MotorConfig`] struct that owns the configuration
//! block (pole pairs, voltage limits, PID controllers, …),
//! the [`base::FOCMotor`] trait that carries the
//! sensor-/driver-aware algorithms (`init_foc`, `loop_foc`,
//! `move_motor`, `characterise_motor`, …), the four upstream
//! enums ([`enums::MotionControlType`],
//! [`enums::TorqueControlType`],
//! [`enums::FOCModulationType`],
//! [`enums::FOCMotorStatus`]), and the three concrete motor
//! structs ([`bldc::BLDCMotor`], [`stepper::StepperMotor`],
//! [`hybrid_stepper::HybridStepperMotor`]).
//!
//! See `PORTING_STATUS.md` Core Library table for the
//! per-file status of this module.

pub mod base;
pub mod bldc;
pub mod enums;
pub mod hybrid_stepper;
pub mod stepper;
