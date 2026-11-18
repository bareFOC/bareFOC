//! # hal — Platform-specific hardware abstractions
//!
//! This module contains hardware-specific implementations of the traits
//! defined in `barefoc`, gated behind Cargo feature flags.
//!
//! - `stm32f4`: Cortex-M DWT-based `Clock` and STM32F4-specific helper traits.
//! - `stm32g4`: Same as above, plus markers for hardware CORDIC acceleration.

#![allow(non_camel_case_types)]

#[cfg(feature = "stm32f4")]
pub mod stm32f4;

#[cfg(feature = "stm32g4")]
pub mod stm32g4;
