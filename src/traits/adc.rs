//! # traits::adc — ADC pin abstraction
//!
//! Provides a minimal `AdcPin` trait for reading raw ADC values.
//! This trait is used by the `current_sense` module to abstract
//! hardware-specific ADC reads while remaining `#![no_std]` compliant.
//!
//! Per `PORTING_GUIDELINES.md`, this follows the same monomorphization
//! pattern as `embedded-hal` traits (Pattern 1), ensuring zero-cost
//! abstraction and allowing the compiler to inline the `read` call.

/// A trait for reading raw ADC values from a pin/channel.
///
/// Implementors should return the raw ADC count (e.g., 12-bit or 16-bit).
/// The conversion to voltage or current is handled by the current sense
/// implementation via configured gains and offsets.
pub trait AdcPin {
    /// Reads the raw ADC value.
    ///
    /// Requires `&mut self` because most hardware ADC peripherals
    /// require mutable access to initiate a conversion and read the result.
    fn read(&mut self) -> u16;
}
