//! # drivers — concrete motor driver implementations
//!
//! Ports the four driver classes from Arduino-FOC's
//! `drivers/` directory to `embedded-hal` v1.0:
//!
//! - [`bldc_3pwm::BLDCDriver3PWM`] — 3-phase BLDC with 3
//!   PWM channels and (optionally) 3 enable pins.
//! - [`bldc_6pwm::BLDCDriver6PWM`] — 3-phase BLDC with
//!   complementary high/low PWM (6 channels) and one
//!   enable pin. Supports a configurable dead-time and
//!   per-phase `PHASE_HI` / `PHASE_LO` / `PHASE_ON` /
//!   `PHASE_OFF` states.
//! - [`stepper_2pwm::StepperDriver2PWM`] — 2-phase stepper
//!   with 2 PWM channels and 2 direction pins.
//! - [`stepper_4pwm::StepperDriver4PWM`] — 2-phase stepper
//!   with 4 PWM channels (2 H-bridges).
//!
//! All four structs implement the abstract
//! [`FOCDriver`](crate::traits::driver::FOCDriver) trait
//! plus either [`BLDCDriver`](crate::traits::driver::BLDCDriver)
//! or
//! [`StepperDriver`](crate::traits::driver::StepperDriver),
//! so the motor can call `set_pwm` / `set_phase_state` /
//! `driver_type` polymorphically without `dyn` — see
//! `PORTING_GUIDELINES.md` §4 Pattern 1.
//!
//! Each module is a self-contained translation unit; the
//! `mod.rs` is intentionally minimal (just `pub mod`).

#![allow(non_camel_case_types)]

pub mod bldc_3pwm;
pub mod bldc_6pwm;
pub mod stepper_2pwm;
pub mod stepper_4pwm;
