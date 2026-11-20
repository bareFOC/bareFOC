//! # communication — Serial communication and debugging utilities
//!
//! Ports the communication classes from Arduino-FOC's `communication/`
//! directory to `embedded-hal` v1.0 and `no_std` Rust:
//!
//! - [`commander::Commander`] — String-based command protocol parser with
//!   callback registration.
//! - [`step_dir::StepDirListener`] — Step/direction pin listener for
//!   external motion control.
//! - [`debug`] — Documentation on using `defmt` for debug logging.
//!
//! These modules provide the building blocks for user-facing serial
//! interfaces without relying on `std::io` or heap allocation.

#![allow(non_camel_case_types)]

pub mod can;
pub mod commander;
pub mod debug;
pub mod step_dir;

