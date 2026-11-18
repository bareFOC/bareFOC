//! # sensors::magnetic_analog — analog-output magnetic sensor
//!
//! Ports
//! [`MagneticSensorAnalog.h`](source/Arduino-FOC/src/sensors/MagneticSensorAnalog.h)
//! and
//! [`MagneticSensorAnalog.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorAnalog.cpp)
//! from Arduino-FOC.
//!
//! `MagneticSensorAnalog` reads a single analog pin (typically
//! the OUT pin of an AS5600 configured in analog mode) and
//! converts the raw ADC reading to a shaft angle in `[0, 2π)`.
//! The conversion is just a linear map: `cpr = max - min` and
//! `angle = (raw / cpr) × 2π`, identical to the upstream
//! formula.
//!
//! This is the simplest of the magnetic sensor variants but
//! suffers the precision loss of a DAC→ADC round trip; the
//! I²C/SPI variants skip that conversion and are
//! recommended for high-precision applications.
//!
//! ## Why a function-pointer `read`
//!
//! The upstream reads the ADC via the global Arduino
//! `analogRead(pin)` function. The FOC-rs port uses a raw
//! `fn() -> i32` callback supplied at construction time —
//! matching the [`GenericSensor`](super::generic::GenericSensor)
//! pattern. The user wires their HAL's ADC read into a
//! `static` and passes the function pointer. This keeps the
//! library free of any `embedded-hal` ADC dependency and
//! works in `no_std` without singletons.
//!
//! If `read` is `None` (the default), `get_sensor_angle()`
//! returns `0.0` without panicking — the same safety
//! behaviour as [`GenericSensor`](super::generic::GenericSensor).

#![allow(non_camel_case_types)]

use crate::common::time::Clock;
use crate::traits::sensor::{Pullup, Sensor, SensorBase};
use core::cell::Cell;

/// Analog-output magnetic sensor.
///
/// Ports the upstream `MagneticSensorAnalog` class
/// ([`MagneticSensorAnalog.h`](source/Arduino-FOC/src/sensors/MagneticSensorAnalog.h)).
///
/// The struct is generic over the [`Clock`] type used by the
/// base-class `init()`. The ADC read function is supplied as
/// a raw `fn() -> i32` pointer in the constructor (mirroring
/// the Arduino `analogRead()` global). The struct does not
/// own any GPIO peripheral; `pin_analog` is stored as
/// documentation for the user's HAL setup.
#[derive(Clone, Debug)]
pub struct MagneticSensorAnalog<C: Clock> {
    base: SensorBase,

    /// Analog input pin number. Stored as a `u8` to match the
    /// upstream `uint8_t pinAnalog`. FOC-rs does not
    /// configure the pin; the user is responsible for the
    /// `pinMode` call (see the upstream constructor).
    pub pin_analog: u8,

    /// Pullup configuration. Mirrors the upstream
    /// `Pullup pullup` field. `USE_INTERN` selects the MCU's
    /// internal pull-up resistor; `USE_EXTERN` selects an
    /// external pull-up.
    pub pullup: Pullup,

    /// Most recent raw ADC reading. Updated on every call to
    /// `get_sensor_angle()`. Exposed for debugging resolution
    /// issues (matches the upstream public `int raw_count`).
    ///
    /// Wrapped in [`Cell`] so the read can update it through a
    /// shared `&self` borrow — the `Sensor::get_sensor_angle`
    /// trait method is `&self` to fit the FOC hot loop, but
    /// the upstream C++ version is non-const and writes the
    /// raw value every time it's called.
    pub raw_count: Cell<i32>,

    /// Smallest expected raw ADC reading. The constructor
    /// stores this verbatim. Stored for inspection by the
    /// FOC engine (mirrors the upstream private field
    /// exposed via `int MagneticSensorAnalog::min_raw_count`
    /// in the same translation unit).
    #[allow(dead_code)]
    pub(crate) min_raw_count: i32,
    /// Largest expected raw ADC reading. Stored for
    /// inspection by the FOC engine (same as `min_raw_count`).
    #[allow(dead_code)]
    pub(crate) max_raw_count: i32,
    /// Counts per revolution. `max - min` (matches upstream).
    pub cpr: i32,

    /// ADC read function. `None` means "no read" —
    /// `get_sensor_angle()` returns `0.0` in that case.
    read: Option<fn() -> i32>,

    _clock: core::marker::PhantomData<C>,
}

impl<C: Clock> MagneticSensorAnalog<C> {
    /// Construct a new `MagneticSensorAnalog`.
    ///
    /// `min_raw_count` is the smallest expected ADC reading
    /// (often ~15 on an AS5600 even at the true zero). The
    /// FOC engine uses this to skip the dead-zone — getting
    /// it wrong produces a small click once per revolution.
    /// `max_raw_count` is the largest reading (often ~1020
    /// for a 10-bit ADC, ~4090 for the ESP32 12-bit ADC).
    ///
    /// `read` is the optional ADC function. Pass `None` to
    /// construct a stub (the angle will always read 0).
    ///
    /// Mirrors the upstream
    /// `MagneticSensorAnalog::MagneticSensorAnalog(uint8_t
    /// _pinAnalog, int _min, int _max)` constructor
    /// ([`MagneticSensorAnalog.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorAnalog.cpp)
    /// lines 8–22).
    pub const fn new(
        pin_analog: u8,
        min_raw_count: i32,
        max_raw_count: i32,
        read: Option<fn() -> i32>,
    ) -> Self {
        let cpr = max_raw_count - min_raw_count;
        Self {
            base: SensorBase::new(),
            pin_analog,
            pullup: Pullup::USE_INTERN,
            raw_count: Cell::new(0),
            min_raw_count,
            max_raw_count,
            cpr,
            read,
            _clock: core::marker::PhantomData,
        }
    }
}

impl<C: Clock> Sensor<C> for MagneticSensorAnalog<C> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    /// Read the ADC and return the scaled angle in `[0, 2π)`.
    ///
    /// Uses the upstream formula
    /// `angle = (raw_count / cpr) × 2π`. Note that this is
    /// **not** exactly `[0, 2π)` at the endpoints unless
    /// `min_raw_count == 0` — the upstream uses the simpler
    /// formula and relies on the user to pick `min`/`max`
    /// accordingly. The FOC-rs port preserves the behaviour
    /// 1:1.
    ///
    /// If `read` is `None`, returns `0.0` without panicking.
    ///
    /// Mirrors upstream `MagneticSensorAnalog::getSensorAngle`
    /// ([`MagneticSensorAnalog.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorAnalog.cpp)
    /// lines 33–37).
    fn get_sensor_angle(&self) -> f32 {
        let raw = self.read.map(|f| f()).unwrap_or(0);
        self.raw_count.set(raw);
        (raw as f32) * crate::math::_2PI / (self.cpr as f32)
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::time::mock::MockClock;
    use crate::math::_2PI;
    use core::sync::atomic::{AtomicI32, Ordering};

    // The `read` callback is a raw `fn() -> i32`, so it cannot
    // close over state. The test module uses an atomic static
    // to stage the next raw reading — `no_std`-compatible.
    static NEXT_RAW: AtomicI32 = AtomicI32::new(0);

    fn fake_analog_read() -> i32 {
        NEXT_RAW.load(Ordering::SeqCst)
    }

    // -----------------------------------------------------------------
    // cpr formula
    // -----------------------------------------------------------------
    #[test]
    fn cpr_equals_max_minus_min() {
        let s = MagneticSensorAnalog::<MockClock>::new(0, 100, 1000, None);
        assert_eq!(s.cpr, 900);

        let s = MagneticSensorAnalog::<MockClock>::new(0, 0, 1023, None);
        assert_eq!(s.cpr, 1023);

        // 12-bit ESP32 ADC range.
        let s = MagneticSensorAnalog::<MockClock>::new(0, 0, 4095, None);
        assert_eq!(s.cpr, 4095);
    }

    #[test]
    fn cpr_and_limits_are_stored() {
        let s = MagneticSensorAnalog::<MockClock>::new(7, 50, 200, None);
        assert_eq!(s.pin_analog, 7);
        assert_eq!(s.min_raw_count, 50);
        assert_eq!(s.max_raw_count, 200);
        assert_eq!(s.cpr, 150);
        assert_eq!(s.raw_count.get(), 0);
        assert_eq!(s.pullup, Pullup::USE_INTERN);
    }

    // -----------------------------------------------------------------
    // get_sensor_angle — scales raw to 2π
    // -----------------------------------------------------------------
    #[test]
    fn get_sensor_angle_scales_raw_to_2pi() {
        let s = MagneticSensorAnalog::<MockClock>::new(0, 0, 1023, Some(fake_analog_read));
        NEXT_RAW.store(0, Ordering::SeqCst);
        assert_eq!(s.get_sensor_angle(), 0.0);
        NEXT_RAW.store(1023, Ordering::SeqCst);
        assert!((s.get_sensor_angle() - _2PI).abs() < 1e-4);
        // Mid-range is 2π × 512/1023 ≈ π.
        NEXT_RAW.store(512, Ordering::SeqCst);
        // 512/1023 * 2π ≈ 3.1447 — off from π by 0.003. The
        // upstream formula is approximate, the tolerance
        // reflects that.
        assert!((s.get_sensor_angle() - core::f32::consts::PI).abs() < 5e-3);
    }

    #[test]
    fn get_sensor_angle_updates_raw_count() {
        let s = MagneticSensorAnalog::<MockClock>::new(0, 0, 1023, Some(fake_analog_read));
        NEXT_RAW.store(123, Ordering::SeqCst);
        let _ = s.get_sensor_angle();
        assert_eq!(s.raw_count.get(), 123);
        NEXT_RAW.store(456, Ordering::SeqCst);
        let _ = s.get_sensor_angle();
        assert_eq!(s.raw_count.get(), 456);
    }

    // -----------------------------------------------------------------
    // get_sensor_angle — missing callback
    // -----------------------------------------------------------------
    #[test]
    fn get_sensor_angle_without_read_callback_returns_zero() {
        // No read callback: must not panic, must return 0.
        let s = MagneticSensorAnalog::<MockClock>::new(0, 0, 1023, None);
        assert_eq!(s.get_sensor_angle(), 0.0);
    }

    // -----------------------------------------------------------------
    // Pullup field
    // -----------------------------------------------------------------
    #[test]
    fn pullup_defaults_to_use_intern() {
        let s = MagneticSensorAnalog::<MockClock>::new(0, 0, 1023, None);
        assert_eq!(s.pullup, Pullup::USE_INTERN);
    }

    // -----------------------------------------------------------------
    // SensorBase wiring
    // -----------------------------------------------------------------
    #[test]
    fn base_is_wired_through_sensor_trait() {
        let s = MagneticSensorAnalog::<MockClock>::new(0, 0, 1023, None);
        assert_eq!(s.base().min_elapsed_time, 0.000_100_f32);
        assert_eq!(s.base().full_rotations, 0);
    }
}
