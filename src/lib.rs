//! # barefoc
//!
//! Pure-Rust, `no_std` port of the [Arduino SimpleFOC](https://simplefoc.com) library
//! ([GitHub: Arduino-FOC](https://github.com/simplefoc/Arduino-FOC)) targeting the
//! `embedded-hal` v1.0 trait ecosystem.
//!
//! ## Upstream Attribution
//!
//! BareFOC is directly ported from and inspired by the SimpleFOC project created by
//! Antun Skuric and the SimpleFOC community. All core Field Oriented Control (FOC)
//! algorithms, Clarke/Park transforms, space-vector PWM calculations, PID controllers,
//! sensor abstractions, and motor state machines are faithful translations of the
//! battle-tested C++ implementations in Arduino-FOC.
//!
//! - Website: <https://simplefoc.com>
//! - Upstream Repository: <https://github.com/simplefoc/Arduino-FOC>
//!
//! See [`PORTING_GUIDELINES.md`] for the full set of porting rules and
//! [`PORTING_STATUS.md`] for the per-module status and progress log.
//!
//! [`PORTING_GUIDELINES.md`]: https://github.com/simplefoc/Rust-FOC/blob/main/PORTING_GUIDELINES.md
//! [`PORTING_STATUS.md`]: https://github.com/simplefoc/Rust-FOC/blob/main/PORTING_STATUS.md

#![cfg_attr(docsrs, feature(doc_auto_cfg))]
#![no_std]

// ---------------------------------------------------------------------------
// Module placeholders. The crate layout is fixed by §2 of PORTING_GUIDELINES.md;
// each module is ported in its own commit and is intentionally empty here.
// ---------------------------------------------------------------------------

pub mod math;

pub mod common;


pub mod traits;

#[cfg(feature = "boot")]
pub mod boot;

// BareBoot application-side entry abstraction (optional `boot` feature).
#[cfg(feature = "boot")]
pub use traits::BootTargetProvider;

pub mod sensors;

pub mod drivers;

pub mod current_sense;

pub mod motor;

pub mod communication;

#[cfg(feature = "kinematics")]
pub mod kinematics;
