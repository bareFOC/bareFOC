//! # current_sense — concrete current-sense implementations
//!
//! Ports the current-sense classes from Arduino-FOC's `current_sense/`
//! directory to `embedded-hal` v1.0 and a custom `AdcPin` trait:
//!
//! - [`inline::InlineCurrentSense`] — reads current through shunt resistors
//!   in series with the motor phases. Supports 2-shunt and 3-shunt.
//! - [`lowside::LowsideCurrentSense`] — reads current through shunt resistors
//!   on the low-side MOSFETs. Requires PWM-synchronized ADC reads (handled
//!   by the user's `AdcPin` implementation or platform-specific HAL).
//! - [`generic::GenericCurrentSense`] — a flexible wrapper that allows users
//!   to provide a custom closure for reading phase currents, useful for
//!   complex or non-standard hardware setups.
//!
//! All three structs implement the abstract
//! [`CurrentSense`](crate::traits::current_sense::CurrentSense) trait,
//! so the motor can call `get_phase_currents()` polymorphically without `dyn`
//! — see `PORTING_GUIDELINES.md` §4 Pattern 1.

#![allow(non_camel_case_types)]

pub mod generic;
pub mod inline;
pub mod lowside;
