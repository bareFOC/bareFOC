//! # current_sense::generic — generic current sensing
//!
//! Ports [`GenericCurrentSense.h`](source/Arduino-FOC/src/current_sense/GenericCurrentSense.h)
//! and [`GenericCurrentSense.cpp`](source/Arduino-FOC/src/current_sense/GenericCurrentSense.cpp)
//! from Arduino-FOC.
//!
//! The `GenericCurrentSense` class is a flexible wrapper that allows users to
//! provide custom ADC read functions. This is useful for complex or non-standard
//! hardware setups, such as shared ADC DMA buffers or custom driver integrations
//! that don't fit the simple `AdcPin` trait model.

#![allow(non_camel_case_types)]

use crate::math::PhaseCurrent;
use crate::traits::current_sense::{CurrentSense, CurrentSenseBase};

/// Generic current-sense implementation.
///
/// Ports the upstream `GenericCurrentSense` class.
/// Uses a closure to read phase currents, bypassing the `AdcPin` trait
/// for maximum flexibility.
pub struct GenericCurrentSense<F, C>
where
    F: Fn() -> PhaseCurrent,
    C: crate::common::time::Clock,
{
    base: CurrentSenseBase,
    read_fn: F,
    _clock: core::marker::PhantomData<C>,
}

impl<F, C> GenericCurrentSense<F, C>
where
    F: Fn() -> PhaseCurrent,
    C: crate::common::time::Clock,
{
    /// Construct a new generic current sense.
    ///
    /// # Arguments
    /// * `read_fn` - A closure that returns the calibrated `PhaseCurrent`.
    pub fn new(read_fn: F) -> Self {
        Self {
            base: CurrentSenseBase::new(),
            read_fn,
            _clock: core::marker::PhantomData,
        }
    }
}

impl<F, C> CurrentSense for GenericCurrentSense<F, C>
where
    F: Fn() -> PhaseCurrent,
    C: crate::common::time::Clock,
{
    type Error = core::convert::Infallible;

    fn base(&self) -> &CurrentSenseBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut CurrentSenseBase {
        &mut self.base
    }

    fn init(&mut self) -> Result<(), Self::Error> {
        self.base.initialized = true;
        Ok(())
    }

    fn get_phase_currents(&mut self) -> PhaseCurrent {
        (self.read_fn)()
    }

    fn driver_align(
        &mut self,
        _voltage: f32,
        _modulation_centered: bool,
    ) -> i32 {
        if self.base.skip_align {
            return 1;
        }
        if !self.base.initialized {
            return 0;
        }
        1
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::time::mock::MockClock;

    #[test]
    fn generic_current_sense_new() {
        let read_fn = || PhaseCurrent { a: 1.0, b: 2.0, c: 3.0 };
        let cs = GenericCurrentSense::<_, MockClock>::new(read_fn);
        assert!(!cs.base.initialized);
    }

    #[test]
    fn generic_current_sense_init() {
        let read_fn = || PhaseCurrent { a: 1.0, b: 2.0, c: 3.0 };
        let mut cs = GenericCurrentSense::<_, MockClock>::new(read_fn);
        cs.init().unwrap();
        assert!(cs.base.initialized);
    }
}
