//! # communication::debug — Debug printing utilities
//!
//! In Arduino-FOC, debug printing is handled by `SimpleFOCDebug`.
//! In BareFOC, debug facilities are represented by [`BareFocDebug`].
//! We recommend using the `defmt` crate behind the `defmt`
//! feature flag for zero-cost, `no_std` compatible debug printing.
//!
//! # Example
//!
//! ```ignore
//! // In your Cargo.toml:
//! // barefoc = { version = "0.1", features = ["defmt"] }
//!
//! #[cfg(feature = "defmt")]
//! defmt::info!("Motor initialized");
//!
//! #[cfg(feature = "defmt")]
//! defmt::debug!("Target velocity: {:.2}", target);
//! ```
//!
//! For host-side testing, you can use standard `println!` behind the `std`
//! feature flag, but this should never be used in production `no_std` code.

/// Debugging namespace and utility struct corresponding to upstream `SimpleFOCDebug`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BareFocDebug;
