//! # sensors::magnetic_pwm — PWM-output magnetic sensor
//!
//! Ports
//! [`MagneticSensorPWM.h`](source/Arduino-FOC/src/sensors/MagneticSensorPWM.h)
//! and
//! [`MagneticSensorPWM.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorPWM.cpp)
//! from Arduino-FOC.
//!
//! `MagneticSensorPWM` reads a 0–100% duty-cycle PWM signal
//! (typically the OUT pin of an AS5600 in PWM mode) and
//! converts the pulse length to a shaft angle in `[0, 2π)`.
//! The conversion is `cpr = max - min + 1` and
//! `angle = ((raw - min) / cpr) × 2π`.
//!
//! ## Two operating modes
//!
//! - **Interrupt mode (default after `handle_pwm` fires)**: the
//!   user wires an EXTI on the PWM pin and calls
//!   [`MagneticSensorPWM::handle_pwm`] on every rising/falling
//!   edge. The handler measures the pulse length and stores
//!   it; `get_sensor_angle()` returns the cached value.
//!   This is the recommended production mode.
//!
//! - **Blocking mode** (Cargo feature `magnetic-pwm-blocking`):
//!   `get_raw_count()` blocks for up to `timeout_us` waiting
//!   for the next pulse, using a user-supplied
//!   `pulse_in` function. This is intended for one-shot
//!   startup probes; production code should use the
//!   interrupt path.
//!
//! The `is_interrupt_based` flag is set by either `handle_pwm`
//! or the explicit `enable_interrupt` constructor; once set
//! the sensor always returns the cached value.
//!
//! ## Why two constructors
//!
//! The upstream has two ctor overloads:
//!
//! - `(pin, min, max)` — caller knows the raw count range
//!   from calibration.
//! - `(pin, freqHz, totalClocks, minClocks, maxClocks)` —
//!   caller knows the PWM frequency and the sensor's clock
//!   count per period. The constructor computes
//!   `min_raw_count = lround(1e6/freq/total*minClocks)` etc.
//!
//! The FOC-rs port exposes both as `new_min_max(...)` and
//! `new_with_freq(...)`.

#![allow(non_camel_case_types)]

use crate::common::time::Clock;
use crate::traits::sensor::{Sensor, SensorBase};
use core::cell::Cell;

/// Default pulseIn timeout (microseconds). Matches the
/// upstream `unsigned int timeout_us = 1200`.
pub const DEFAULT_TIMEOUT_US: u32 = 1200;

/// PWM-output magnetic sensor.
///
/// Ports the upstream `MagneticSensorPWM` class
/// ([`MagneticSensorPWM.h`](source/Arduino-FOC/src/sensors/MagneticSensorPWM.h)).
///
/// The struct is generic over the [`Clock`] type used by the
/// base-class `init()`. The PWM pulse length is measured in
/// microseconds.
#[derive(Debug)]
pub struct MagneticSensorPWM<C: Clock> {
    base: SensorBase,

    /// PWM input pin number. Stored as a `u8` to match the
    /// upstream `uint8_t pinPWM`. FOC-rs does not configure
    /// the pin; the user is responsible for the `pinMode`
    /// call.
    pub pin_pwm: u8,

    /// Pullup configuration. Mirrors the upstream
    /// `Pullup pullup` field.
    pub pullup: Pullup,

    /// Most recent raw pulse length (microseconds) for
    /// interrupt mode, or last-blocked-read length for
    /// blocking mode. Exposed for debugging (matches the
    /// upstream public `unsigned long pulse_length_us`).
    ///
    /// Stored in a [`Cell`] so the interrupt handler can
    /// update it through a shared `&self` borrow.
    pub pulse_length_us: Cell<u32>,

    /// Timestamp of the most recent pulse edge. Mirrors the
    /// upstream `unsigned long pulse_timestamp`.
    pub pulse_timestamp: Cell<u32>,

    /// Microsecond timeout for the blocking `pulseIn` read.
    /// Mirrors the upstream `unsigned int timeout_us = 1200`.
    pub timeout_us: u32,

    /// Most recent raw count after clamping. Exposed for
    /// debugging (matches the upstream public `int
    /// raw_count`).
    pub raw_count: Cell<i32>,

    /// Smallest expected raw count. Set by the constructor.
    pub(crate) min_raw_count: i32,
    /// Largest expected raw count.
    pub(crate) max_raw_count: i32,
    /// Counts per revolution. `max - min + 1` (matches the
    /// upstream formula — note the `+ 1`, unlike
    /// [`crate::sensors::magnetic_analog::MagneticSensorAnalog`]
    /// which uses `max - min`).
    pub cpr: i32,

    /// `true` after an interrupt has fired (or
    /// `enable_interrupt` was called). When `true`, the
    /// blocking read is bypassed and the cached
    /// `pulse_length_us` is returned directly.
    pub is_interrupt_based: Cell<bool>,

    /// Timestamp of the most recent `handle_pwm` call. Stored
    /// in a [`Cell`] so the interrupt handler can update it.
    last_call_us: Cell<u32>,

    #[cfg(feature = "magnetic-pwm-blocking")]
    /// Optional blocking `pulseIn` function for non-interrupt mode.
    pulse_in_fn: Option<fn(u8, u32) -> u32>,

    _clock: core::marker::PhantomData<C>,
}

use crate::traits::sensor::Pullup;

impl<C: Clock> MagneticSensorPWM<C> {
    /// Construct a sensor with explicit `min`/`max` raw count
    /// bounds. `cpr = max - min + 1`.
    ///
    /// Mirrors the upstream
    /// `MagneticSensorPWM::MagneticSensorPWM(uint8_t
    /// _pinPWM, int _min, int _max)` constructor
    /// ([`MagneticSensorPWM.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorPWM.cpp)
    /// lines 9–22).
    pub fn new_min_max(pin_pwm: u8, min_raw_count: i32, max_raw_count: i32) -> Self {
        let cpr = max_raw_count - min_raw_count + 1;
        Self {
            base: SensorBase::new(),
            pin_pwm,
            pullup: Pullup::USE_INTERN,
            pulse_length_us: Cell::new(0),
            pulse_timestamp: Cell::new(0),
            timeout_us: DEFAULT_TIMEOUT_US,
            raw_count: Cell::new(0),
            min_raw_count,
            max_raw_count,
            cpr,
            is_interrupt_based: Cell::new(false),
            last_call_us: Cell::new(0),
            #[cfg(feature = "magnetic-pwm-blocking")]
            pulse_in_fn: None,
            _clock: core::marker::PhantomData,
        }
    }

    /// Construct a sensor from PWM frequency and clock-count
    /// parameters. Computes
    /// `min_raw_count = lround(1e6/freq/total*minClocks)` and
    /// the same for `max`, then `cpr = max - min + 1`. Also
    /// sets `min_elapsed_time = 1.0 / freq` on the
    /// [`SensorBase`].
    ///
    /// Mirrors the upstream
    /// `MagneticSensorPWM::MagneticSensorPWM(uint8_t
    /// _pinPWM, int freqHz, int _total_pwm_clocks, int
    /// _min_pwm_clocks, int _max_pwm_clocks)` constructor
    /// ([`MagneticSensorPWM.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorPWM.cpp)
    /// lines 35–50).
    pub fn new_with_freq(
        pin_pwm: u8,
        freq_hz: i32,
        total_pwm_clocks: i32,
        min_pwm_clocks: i32,
        max_pwm_clocks: i32,
    ) -> Self {
        let min_raw_count =
            libm::roundf(1_000_000.0_f32 / freq_hz as f32 / total_pwm_clocks as f32
                * min_pwm_clocks as f32) as i32;
        let max_raw_count =
            libm::roundf(1_000_000.0_f32 / freq_hz as f32 / total_pwm_clocks as f32
                * max_pwm_clocks as f32) as i32;
        let cpr = max_raw_count - min_raw_count + 1;
        let mut base = SensorBase::new();
        base.min_elapsed_time = 1.0_f32 / freq_hz as f32;
        Self {
            base,
            pin_pwm,
            pullup: Pullup::USE_INTERN,
            pulse_length_us: Cell::new(0),
            pulse_timestamp: Cell::new(0),
            timeout_us: DEFAULT_TIMEOUT_US,
            raw_count: Cell::new(0),
            min_raw_count,
            max_raw_count,
            cpr,
            is_interrupt_based: Cell::new(false),
            last_call_us: Cell::new(0),
            #[cfg(feature = "magnetic-pwm-blocking")]
            pulse_in_fn: None,
            _clock: core::marker::PhantomData,
        }
    }

    /// Mark the sensor as interrupt-based and remember the
    /// user-supplied interrupt callback. The user must wire
    /// this callback to their EXTI; the upstream
    /// `attachInterrupt(...)` call has no `embedded-hal`
    /// equivalent.
    pub fn enable_interrupt(&self, _do_pwm: fn()) {
        // No-op for the stored callback — the user is
        // responsible for the EXTI wiring. We just flip the
        // flag.
        self.is_interrupt_based.set(true);
        let _ = _do_pwm;
    }

    /// EXTI callback. The user calls this from their
    /// rising/falling-edge closure with the current
    /// microsecond timestamp.
    ///
    /// The very first call after construction is the
    /// "bootstrap" edge — there is no previous edge to
    /// measure against, so `pulse_length_us` is left at `0`
    /// and only `last_call_us` is updated. Every subsequent
    /// call updates `pulse_length_us = now_us - last_call_us`
    /// (matching the upstream's falling-edge branch).
    ///
    /// The upstream uses `digitalRead(pinPWM)` to distinguish
    /// rising from falling edges; the FOC-rs port cannot
    /// read the pin from inside the handler, so it treats
    /// every call as a "falling edge" and uses the
    /// `is_interrupt_based` flag as the "have we seen a
    /// previous edge?" signal.
    ///
    /// On every call, `is_interrupt_based` is set to `true`
    /// (so subsequent `get_raw_count` calls skip the
    /// blocking read).
    pub fn handle_pwm(&self, now_us: u32) {
        if self.is_interrupt_based.get() {
            // Subsequent edge: compute the pulse length.
            let last = self.last_call_us.get();
            if now_us > last {
                self.pulse_length_us.set(now_us - last);
                self.pulse_timestamp.set(last);
            }
        }
        self.last_call_us.set(now_us);
        self.is_interrupt_based.set(true);
    }

    /// Read the current raw count. In interrupt mode, returns
    /// the cached `pulse_length_us`. In blocking mode (gated
    /// on the `magnetic-pwm-blocking` Cargo feature), calls
    /// the user-supplied `pulse_in` closure to time the next
    /// pulse.
    pub fn get_raw_count(&self) -> i32 {
        if self.is_interrupt_based.get() {
            self.pulse_length_us.get() as i32
        } else {
            #[cfg(feature = "magnetic-pwm-blocking")]
            {
                if let Some(pulse_in) = self.pulse_in_fn {
                    let now = self.last_call_us.get();
                    self.pulse_timestamp.set(now);
                    let length = pulse_in(self.pin_pwm, self.timeout_us);
                    self.pulse_length_us.set(length);
                    return length as i32;
                }
            }
            // No blocking function available — return the
            // cached value (or 0 if never read).
            self.pulse_length_us.get() as i32
        }
    }
}

impl<C: Clock> Sensor<C> for MagneticSensorPWM<C> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    /// Read the sensor and return the shaft angle in
    /// `[0, 2π)`. Clamps the raw count to the configured
    /// `[min, max]` range before scaling.
    ///
    /// Uses the upstream formula
    /// `((raw - min) / cpr) × 2π` (note the `raw - min`,
    /// unlike [`crate::sensors::magnetic_analog::MagneticSensorAnalog`]
    /// which is just `raw / cpr`).
    fn get_sensor_angle(&self) -> f32 {
        let mut raw = self.get_raw_count();
        if raw > self.max_raw_count {
            raw = self.max_raw_count;
        }
        if raw < self.min_raw_count {
            raw = self.min_raw_count;
        }
        self.raw_count.set(raw);
        let m = (raw - self.min_raw_count) as f32;
        m * crate::math::_2PI / (self.cpr as f32)
    }
}

// ---------------------------------------------------------------------------
// Optional blocking-pulseIn support (Cargo feature gated).
//
// When the `magnetic-pwm-blocking` feature is enabled, the
// struct stores a user-supplied `fn(u8, u32) -> u32` callback
// that mimics Arduino's `pulseIn(pin, HIGH, timeout)`. The
// FOC-rs port cannot depend on a specific HAL's blocking
// read, so the user wires their own function.
// ---------------------------------------------------------------------------

#[cfg(feature = "magnetic-pwm-blocking")]
impl<C: Clock> MagneticSensorPWM<C> {
    /// Builder-style setter: supply a `pulseIn`-equivalent
    /// function `(pin, timeout_us) -> pulse_length_us`. The
    /// function is called by [`Self::get_raw_count`] when
    /// `is_interrupt_based` is `false`.
    pub fn with_pulse_in_fn(mut self, f: fn(u8, u32) -> u32) -> Self {
        self.pulse_in_fn = Some(f);
        self
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::time::mock::MockClock;

    // -----------------------------------------------------------------
    // cpr formula
    // -----------------------------------------------------------------
    #[test]
    fn cpr_equals_max_minus_min_plus_one() {
        let s = MagneticSensorPWM::<MockClock>::new_min_max(0, 100, 1000);
        // 1000 - 100 + 1 = 901
        assert_eq!(s.cpr, 901);
    }

    #[test]
    fn cpr_zero_when_max_equals_min() {
        // Edge case: max == min, cpr = 1
        let s = MagneticSensorPWM::<MockClock>::new_min_max(0, 500, 500);
        assert_eq!(s.cpr, 1);
    }

    #[test]
    fn min_max_and_pin_are_stored() {
        let s = MagneticSensorPWM::<MockClock>::new_min_max(7, 100, 1000);
        assert_eq!(s.pin_pwm, 7);
        assert_eq!(s.min_raw_count, 100);
        assert_eq!(s.max_raw_count, 1000);
        assert_eq!(s.timeout_us, DEFAULT_TIMEOUT_US);
        assert_eq!(s.pullup, Pullup::USE_INTERN);
    }

    // -----------------------------------------------------------------
    // Frequency-based constructor
    // -----------------------------------------------------------------
    #[test]
    fn new_with_freq_computes_min_max() {
        // AS5600 defaults: freq=115, total=4351, min=128, max=4223
        // Compute the expected value with the same formula
        // (rather than hardcoding) so the test doesn't
        // depend on f32 rounding direction.
        let min_raw = libm::roundf(
            1_000_000.0_f32 / 115.0 / 4351.0 * 128.0,
        ) as i32;
        let max_raw = libm::roundf(
            1_000_000.0_f32 / 115.0 / 4351.0 * 4223.0,
        ) as i32;
        let s = MagneticSensorPWM::<MockClock>::new_with_freq(0, 115, 4351, 128, 4223);
        assert_eq!(s.min_raw_count, min_raw);
        assert_eq!(s.max_raw_count, max_raw);
        assert_eq!(s.cpr, max_raw - min_raw + 1);
    }

    #[test]
    fn new_with_freq_sets_min_elapsed_time() {
        let s = MagneticSensorPWM::<MockClock>::new_with_freq(0, 115, 4351, 128, 4223);
        // min_elapsed_time = 1.0 / 115
        let expected = 1.0_f32 / 115.0;
        assert!((s.base().min_elapsed_time - expected).abs() < 1e-6);
    }

    // -----------------------------------------------------------------
    // get_sensor_angle — interrupt mode
    // -----------------------------------------------------------------
    #[test]
    fn get_sensor_angle_uses_cached_pulse_length() {
        // Default: is_interrupt_based = false. But we can
        // simulate a previous handle_pwm call by setting
        // pulse_length_us directly. Note: setting
        // is_interrupt_based to true bypasses the blocking
        // path entirely.
        let s = MagneticSensorPWM::<MockClock>::new_min_max(0, 0, 1000);
        // Manually stage the interrupt state.
        s.is_interrupt_based.set(true);
        s.pulse_length_us.set(500);
        // angle = (500 - 0) / 1001 * 2π ≈ 3.139
        let angle = s.get_sensor_angle();
        let expected = 500.0_f32 * core::f32::consts::TAU / 1001.0;
        assert!((angle - expected).abs() < 1e-3);
    }

    // -----------------------------------------------------------------
    // get_sensor_angle — clamping
    // -----------------------------------------------------------------
    #[test]
    fn get_sensor_angle_clamps_to_range() {
        let s = MagneticSensorPWM::<MockClock>::new_min_max(0, 100, 1000);
        s.is_interrupt_based.set(true);

        // raw > max → clamps to max → angle ≈ 2π
        s.pulse_length_us.set(5000);
        let angle = s.get_sensor_angle();
        let expected = 900.0_f32 * core::f32::consts::TAU / 901.0;
        assert!((angle - expected).abs() < 1e-3);

        // raw < min → clamps to min → angle = 0
        s.pulse_length_us.set(50);
        let angle = s.get_sensor_angle();
        assert_eq!(angle, 0.0);

        // In range — uses raw value
        s.pulse_length_us.set(550);
        let angle = s.get_sensor_angle();
        let expected = 450.0_f32 * core::f32::consts::TAU / 901.0;
        assert!((angle - expected).abs() < 1e-3);
    }

    // -----------------------------------------------------------------
    // handle_pwm — interrupt path
    // -----------------------------------------------------------------
    #[test]
    fn handle_pwm_measues_pulse_length() {
        let s = MagneticSensorPWM::<MockClock>::new_min_max(0, 0, 1000);

        // First edge (rising): no previous timestamp, so
        // pulse_length_us stays 0 but last_call_us is set.
        s.handle_pwm(1000);
        assert_eq!(s.pulse_length_us.get(), 0);
        assert_eq!(s.last_call_us.get(), 1000);
        assert!(s.is_interrupt_based.get());

        // Second edge (falling): now_us > last, so
        // pulse_length_us = 1500 - 1000 = 500.
        s.handle_pwm(1500);
        assert_eq!(s.pulse_length_us.get(), 500);
        assert_eq!(s.pulse_timestamp.get(), 1000);
        assert_eq!(s.last_call_us.get(), 1500);
    }

    #[test]
    fn handle_pwm_sets_is_interrupt_based() {
        let s = MagneticSensorPWM::<MockClock>::new_min_max(0, 0, 1000);
        assert!(!s.is_interrupt_based.get());
        s.handle_pwm(1_000);
        assert!(s.is_interrupt_based.get());
    }

    #[test]
    fn handle_pwm_handles_out_of_order_timestamps() {
        // If now_us < last (e.g. wraparound or out-of-order
        // delivery), we don't update pulse_length. The
        // upstream doesn't guard against this either, but
        // our guard means we don't store a bogus huge value.
        let s = MagneticSensorPWM::<MockClock>::new_min_max(0, 0, 1000);
        s.handle_pwm(2_000);
        s.handle_pwm(1_000); // before the previous
        // pulse_length should still be 0 (we only set it
        // when now > last).
        assert_eq!(s.pulse_length_us.get(), 0);
        assert_eq!(s.last_call_us.get(), 1_000);
    }

    // -----------------------------------------------------------------
    // get_raw_count — blocking vs interrupt
    // -----------------------------------------------------------------
    #[test]
    fn get_raw_count_returns_cached_in_interrupt_mode() {
        let s = MagneticSensorPWM::<MockClock>::new_min_max(0, 0, 1000);
        s.is_interrupt_based.set(true);
        s.pulse_length_us.set(777);
        assert_eq!(s.get_raw_count(), 777);
    }

    #[test]
    fn get_raw_count_returns_zero_without_interrupt() {
        let s = MagneticSensorPWM::<MockClock>::new_min_max(0, 0, 1000);
        // is_interrupt_based = false, no blocking function
        // available (feature off in default tests).
        // Returns 0 (the default pulse_length_us).
        assert_eq!(s.get_raw_count(), 0);
    }

    // -----------------------------------------------------------------
    // enable_interrupt
    // -----------------------------------------------------------------
    #[test]
    fn enable_interrupt_flags_is_interrupt_based() {
        let s = MagneticSensorPWM::<MockClock>::new_min_max(0, 0, 1000);
        assert!(!s.is_interrupt_based.get());
        s.enable_interrupt(|| {});
        assert!(s.is_interrupt_based.get());
    }
}
