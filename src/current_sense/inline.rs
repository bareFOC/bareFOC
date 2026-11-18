//! # current_sense::inline — inline current sensing
//!
//! Ports [`InlineCurrentSense.h`](source/Arduino-FOC/src/current_sense/InlineCurrentSense.h)
//! and [`InlineCurrentSense.cpp`](source/Arduino-FOC/src/current_sense/InlineCurrentSense.cpp)
//! from Arduino-FOC.
//!
//! The `InlineCurrentSense` class reads the current through shunt resistors
//! placed in series with the motor phases. It supports 2-shunt (A and B)
//! and 3-shunt (A, B, and C) configurations.

#![allow(non_camel_case_types)]

use crate::common::time::Clock;
use crate::math::{self, PhaseCurrent};
use crate::traits::adc::AdcPin;
use crate::traits::current_sense::{CurrentSense, CurrentSenseBase};
use crate::traits::driver::DriverType;

/// Inline current-sense implementation.
///
/// Ports the upstream `InlineCurrentSense` class.
/// Generic over 2 or 3 `AdcPin` types and a `Clock` for delays.
///
/// To configure for 2-shunt, pass `None` for `pin_c`.
pub struct InlineCurrentSense<P1, P2, P3, C>
where
    P1: AdcPin,
    P2: AdcPin,
    P3: AdcPin,
    C: Clock,
{
    base: CurrentSenseBase,
    pin_a: P1,
    pin_b: P2,
    pin_c: Option<P3>,
    #[allow(dead_code)]
    shunt_resistor: f32,
    #[allow(dead_code)]
    amp_gain: f32,
    #[allow(dead_code)]
    volts_to_amps_ratio: f32,
    _clock: core::marker::PhantomData<C>,
}

impl<P1, P2, P3, C> InlineCurrentSense<P1, P2, P3, C>
where
    P1: AdcPin,
    P2: AdcPin,
    P3: AdcPin,
    C: Clock,
{
    /// Construct a new inline current sense.
    ///
    /// # Arguments
    /// * `shunt_resistor` - Shunt resistor value in ohms.
    /// * `gain` - Current-sense op-amp gain.
    /// * `pin_a` - Phase A ADC pin.
    /// * `pin_b` - Phase B ADC pin.
    /// * `pin_c` - Phase C ADC pin (optional).
    pub fn new(
        shunt_resistor: f32,
        gain: f32,
        pin_a: P1,
        pin_b: P2,
        pin_c: Option<P3>,
    ) -> Self {
        let volts_to_amps_ratio = 1.0 / shunt_resistor / gain;
        let mut base = CurrentSenseBase::new();
        base.gain_a = volts_to_amps_ratio;
        base.gain_b = volts_to_amps_ratio;
        base.gain_c = volts_to_amps_ratio;
        // Upstream sets pin_c to NOT_SET if not provided.
        // We handle this via the Option type.
        if pin_c.is_none() {
            base.pin_c = math::_NC;
            base.gain_c = 0.0;
        }

        Self {
            base,
            pin_a,
            pin_b,
            pin_c,
            shunt_resistor,
            amp_gain: gain,
            volts_to_amps_ratio,
            _clock: core::marker::PhantomData,
        }
    }

    /// Construct a new inline current sense using mV per Amp ratio.
    ///
    /// # Arguments
    /// * `mvp_a` - mV per Amp ratio.
    /// * `pin_a` - Phase A ADC pin.
    /// * `pin_b` - Phase B ADC pin.
    /// * `pin_c` - Phase C ADC pin (optional).
    pub fn new_mvp_a(
        mvp_a: f32,
        pin_a: P1,
        pin_b: P2,
        pin_c: Option<P3>,
    ) -> Self {
        let volts_to_amps_ratio = 1000.0 / mvp_a;
        let mut base = CurrentSenseBase::new();
        base.gain_a = volts_to_amps_ratio;
        base.gain_b = volts_to_amps_ratio;
        base.gain_c = volts_to_amps_ratio;
        if pin_c.is_none() {
            base.pin_c = math::_NC;
            base.gain_c = 0.0;
        }

        Self {
            base,
            pin_a,
            pin_b,
            pin_c,
            shunt_resistor: 0.0, // Not used when mvp_a is provided
            amp_gain: 0.0,
            volts_to_amps_ratio,
            _clock: core::marker::PhantomData,
        }
    }

    /// Calibrate the zero-current offsets of the ADC.
    ///
    /// Reads the ADC voltage `calibration_rounds` times and calculates the mean.
    /// Uses the provided `clock` for microsecond delays between reads.
    pub fn calibrate_offsets(&mut self, clock: &C) {
        let calibration_rounds = 1000;
        let mut sum_a = 0.0;
        let mut sum_b = 0.0;
        let mut sum_c = 0.0;

        for _ in 0..calibration_rounds {
            sum_a += self.pin_a.read() as f32;
            sum_b += self.pin_b.read() as f32;
            if let Some(ref mut pin_c) = self.pin_c {
                sum_c += pin_c.read() as f32;
            }
            clock.delay_ms(1);
        }

        self.base.offset_ia = sum_a / calibration_rounds as f32;
        self.base.offset_ib = sum_b / calibration_rounds as f32;
        self.base.offset_ic = sum_c / calibration_rounds as f32;
    }
}

impl<P1, P2, P3, C> CurrentSense for InlineCurrentSense<P1, P2, P3, C>
where
    P1: AdcPin,
    P2: AdcPin,
    P3: AdcPin,
    C: Clock,
{
    type Error = core::convert::Infallible;

    fn base(&self) -> &CurrentSenseBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut CurrentSenseBase {
        &mut self.base
    }

    fn init(&mut self) -> Result<(), Self::Error> {
        // Upstream sets center PWM during init if BLDC driver is linked.
        // We don't have a direct driver reference here, so we skip that.
        // The user is expected to call `calibrate_offsets` manually or
        // rely on the driver's alignment procedure which handles it.
        self.base.initialized = true;
        Ok(())
    }

    fn get_phase_currents(&mut self) -> PhaseCurrent {
        let mut current = PhaseCurrent { a: 0.0, b: 0.0, c: 0.0 };

        let raw_a = self.pin_a.read() as f32;
        current.a = (raw_a - self.base.offset_ia) * self.base.gain_a;

        let raw_b = self.pin_b.read() as f32;
        current.b = (raw_b - self.base.offset_ib) * self.base.gain_b;

        if let Some(ref mut pin_c) = self.pin_c {
            let raw_c = pin_c.read() as f32;
            current.c = (raw_c - self.base.offset_ic) * self.base.gain_c;
        }

        current
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

        match self.base.driver_type {
            DriverType::BLDC => 1,
            DriverType::Stepper => 1,
            DriverType::Hybrid => 1,
            _ => 0,
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::time::mock::MockClock;
    use crate::traits::adc::AdcPin;

    struct MockAdcPin {
        value: u16,
    }

    impl AdcPin for MockAdcPin {
        fn read(&mut self) -> u16 {
            self.value
        }
    }

    #[test]
    fn inline_current_sense_new_sets_gains() {
        let pin_a = MockAdcPin { value: 0 };
        let pin_b = MockAdcPin { value: 0 };
        let pin_c = Some(MockAdcPin { value: 0 });

        let cs = InlineCurrentSense::<MockAdcPin, MockAdcPin, MockAdcPin, MockClock>::new(0.01, 50.0, pin_a, pin_b, pin_c);
        let expected_ratio = 1.0 / 0.01 / 50.0; // 2.0

        assert_eq!(cs.base.gain_a, expected_ratio);
        assert_eq!(cs.base.gain_b, expected_ratio);
        assert_eq!(cs.base.gain_c, expected_ratio);
    }

    #[test]
    fn inline_current_sense_2_shunt_disables_c() {
        let pin_a = MockAdcPin { value: 0 };
        let pin_b = MockAdcPin { value: 0 };

        let cs = InlineCurrentSense::<MockAdcPin, MockAdcPin, MockAdcPin, MockClock>::new(
            0.01, 50.0, pin_a, pin_b, None,
        );

        assert_eq!(cs.base.pin_c, math::_NC);
        assert_eq!(cs.base.gain_c, 0.0);
    }

    #[test]
    fn inline_current_sense_calibrate_offsets() {
        let pin_a = MockAdcPin { value: 2048 };
        let pin_b = MockAdcPin { value: 2048 };
        let pin_c = Some(MockAdcPin { value: 2048 });
        let clock = MockClock::new();

        let mut cs = InlineCurrentSense::<MockAdcPin, MockAdcPin, MockAdcPin, MockClock>::new(0.01, 50.0, pin_a, pin_b, pin_c);
        cs.calibrate_offsets(&clock);

        assert_eq!(cs.base.offset_ia, 2048.0);
        assert_eq!(cs.base.offset_ib, 2048.0);
        assert_eq!(cs.base.offset_ic, 2048.0);
    }

    #[test]
    fn inline_current_sense_get_phase_currents() {
        // Offset is 2048, gain is 2.0.
        // If raw is 2548, current = (2548 - 2048) * 2.0 = 1000.0
        let pin_a = MockAdcPin { value: 2548 };
        let pin_b = MockAdcPin { value: 1548 };
        let pin_c = Some(MockAdcPin { value: 2048 });

        let mut cs = InlineCurrentSense::<MockAdcPin, MockAdcPin, MockAdcPin, MockClock>::new(0.01, 50.0, pin_a, pin_b, pin_c);
        cs.base.offset_ia = 2048.0;
        cs.base.offset_ib = 2048.0;
        cs.base.offset_ic = 2048.0;

        let currents = cs.get_phase_currents();
        assert_eq!(currents.a, 1000.0);
        assert_eq!(currents.b, -1000.0);
        assert_eq!(currents.c, 0.0);
    }

    #[test]
    fn inline_current_sense_get_phase_currents_2_shunt() {
        let pin_a = MockAdcPin { value: 2548 };
        let pin_b = MockAdcPin { value: 1548 };

        let mut cs = InlineCurrentSense::<MockAdcPin, MockAdcPin, MockAdcPin, MockClock>::new(
            0.01, 50.0, pin_a, pin_b, None,
        );
        cs.base.offset_ia = 2048.0;
        cs.base.offset_ib = 2048.0;

        let currents = cs.get_phase_currents();
        assert_eq!(currents.a, 1000.0);
        assert_eq!(currents.b, -1000.0);
        assert_eq!(currents.c, 0.0);
    }

    #[test]
    fn inline_current_sense_init_sets_initialized() {
        let pin_a = MockAdcPin { value: 0 };
        let pin_b = MockAdcPin { value: 0 };
        let pin_c = Some(MockAdcPin { value: 0 });

        let mut cs = InlineCurrentSense::<MockAdcPin, MockAdcPin, MockAdcPin, MockClock>::new(0.01, 50.0, pin_a, pin_b, pin_c);
        assert!(!cs.base.initialized);
        cs.init().unwrap();
        assert!(cs.base.initialized);
    }
}
