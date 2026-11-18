//! # sensors::generic — user-callback sensor
//!
//! Ports [`GenericSensor.h`](source/Arduino-FOC/src/sensors/GenericSensor.h)
//! and
//! [`GenericSensor.cpp`](source/Arduino-FOC/src/sensors/GenericSensor.cpp)
//! from Arduino-FOC.
//!
//! `GenericSensor` is the simplest of the FOC-rs sensor wrappers:
//! it does not own any peripherals. Instead the user supplies two
//! raw function pointers (matching the upstream C++ API 1:1):
//!
//! - `read_callback: fn() -> f32` — invoked every time
//!   `get_sensor_angle()` is called; must return a value in
//!   `[0, 2π)`.
//! - `init_callback: fn()` — invoked exactly once by `Sensor::init`
//!   before the base-class init runs.
//!
//! Both callbacks are `None` by default (mirroring the upstream
//! `nullptr` defaults). When `read_callback` is `None`,
//! `get_sensor_angle()` returns `0.0` instead of panicking — the
//! upstream C++ would dereference a null function pointer in that
//! case, which is undefined behaviour; the FOC-rs port is slightly
//! safer.
//!
//! This is the right sensor to use when the user already has a
//! custom angle-reading function (e.g. one that interpolates
//! between I2C samples, or reads from a protocol the FOC-rs
//! library does not yet support natively).
//!
//! ## Why `fn()` and not `dyn FnMut()`
//!
//! The upstream C++ uses raw function pointers
//! (`float (*readCallback)()`, `void (*initCallback)()`) and the
//! Rust port preserves the signature so existing SimpleFOC code
//! ports directly. The trade-off is that the callbacks cannot
//! close over user state — for a stateful wrapper the user is
//! expected to thread state through a `static` (in `no_std`) or
//! through the surrounding scope with care about re-entrancy.
//!
//! ## Why no `enable_interrupts` method
//!
//! `GenericSensor` has no hardware of its own. Interrupt wiring
//! (if any) lives in the user-supplied read callback.

#![allow(non_camel_case_types)]

use crate::common::time::Clock;
use crate::traits::sensor::{Sensor, SensorBase};

/// User-callback sensor.
///
/// Ports the upstream `GenericSensor` class
/// ([`GenericSensor.h`](source/Arduino-FOC/src/sensors/GenericSensor.h)).
///
/// The struct is generic over the [`Clock`] type used for the
/// base-class `init()` timestamp, exactly like [`Encoder`] and
/// [`HallSensor`]. It carries no peripheral state — both
/// `read_callback` and `init_callback` are raw function pointers
/// (`fn()`) supplied at construction time, matching the upstream
/// C++ 1:1.
#[derive(Clone, Debug)]
pub struct GenericSensor<C: Clock> {
    base: SensorBase,

    /// Function pointer called by `get_sensor_angle()`. Must
    /// return the current shaft angle in radians in `[0, 2π)`.
    /// `None` means "no callback" — `get_sensor_angle()` returns
    /// `0.0` in that case.
    pub read_callback: Option<fn() -> f32>,

    /// Function pointer called by `Sensor::init()` exactly once,
    /// before the base-class state is seeded. Use it to bring up
    /// any hardware the read callback depends on. `None` means
    /// "no callback" — the init step is simply skipped.
    pub init_callback: Option<fn()>,

    _clock: core::marker::PhantomData<C>,
}

impl<C: Clock> GenericSensor<C> {
    /// Construct a new `GenericSensor`.
    ///
    /// Both callbacks are optional. `None` is the C++-equivalent
    /// of the upstream `nullptr` default.
    ///
    /// Mirrors the upstream
    /// `GenericSensor::GenericSensor(float (*readCallback)(),
    /// void (*initCallback)())` constructor
    /// ([`GenericSensor.cpp`](source/Arduino-FOC/src/sensors/GenericSensor.cpp)
    /// lines 9–13).
    pub const fn new(
        read_callback: Option<fn() -> f32>,
        init_callback: Option<fn()>,
    ) -> Self {
        Self {
            base: SensorBase::new(),
            read_callback,
            init_callback,
            _clock: core::marker::PhantomData,
        }
    }
}

impl<C: Clock> Sensor<C> for GenericSensor<C> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    /// Run the user init callback (if any), then seed the
    /// [`SensorBase`] state with two `get_sensor_angle()` reads
    /// (mirroring the upstream `Sensor::init` body).
    ///
    /// The body of the upstream base-class `Sensor::init` is
    /// inlined here: the default trait method cannot be
    /// invoked through `Self` (that would recurse into this
    /// override) and Rust does not yet have a clean "call the
    /// default method of a super-trait" syntax for this case.
    /// The inlined body is small and matches the upstream
    /// C++ exactly.
    ///
    /// Mirrors upstream `GenericSensor::init`
    /// ([`GenericSensor.cpp`](source/Arduino-FOC/src/sensors/GenericSensor.cpp)
    /// lines 15–19): the user init callback runs first, the
    /// base-class seeding runs second.
    fn init(&mut self, clock: &C) {
        if let Some(cb) = self.init_callback {
            cb();
        }
        // Inlined body of `Sensor::init` default method (see
        // `barefoc/src/traits/sensor.rs`).
        let _ = self.get_sensor_angle();
        let a = self.get_sensor_angle();
        let now = clock.now_us();
        {
            let base = self.base_mut();
            base.vel_angle_prev = a;
            base.vel_angle_prev_ts = now;
        }
        let _ = self.get_sensor_angle();
        let a2 = self.get_sensor_angle();
        let now2 = clock.now_us();
        {
            let base = self.base_mut();
            base.angle_prev = a2;
            base.angle_prev_ts = now2;
        }
    }

    /// Invoke the user read callback and return its value.
    ///
    /// If `read_callback` is `None`, returns `0.0` rather than
    /// panicking — the FOC-rs port is slightly safer than the
    /// upstream C++, which would dereference a null function
    /// pointer (undefined behaviour).
    ///
    /// Mirrors upstream `GenericSensor::getSensorAngle`
    /// ([`GenericSensor.cpp`](source/Arduino-FOC/src/sensors/GenericSensor.cpp)
    /// line 25).
    fn get_sensor_angle(&self) -> f32 {
        match self.read_callback {
            Some(cb) => cb(),
            None => 0.0,
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
    use core::sync::atomic::{AtomicU32, Ordering};

    // Counters used to verify the init callback fires (raw `fn`
    // pointers cannot close over state, so we use atomic
    // statics — `no_std`-compatible).
    static INIT_COUNT: AtomicU32 = AtomicU32::new(0);
    static READ_COUNT: AtomicU32 = AtomicU32::new(0);

    fn counting_init_callback() {
        INIT_COUNT.fetch_add(1, Ordering::SeqCst);
    }

    fn counting_read_callback() -> f32 {
        READ_COUNT.fetch_add(1, Ordering::SeqCst);
        1.23
    }

    fn constant_read_callback() -> f32 {
        core::f32::consts::FRAC_PI_2
    }

    // -----------------------------------------------------------------
    // get_sensor_angle — invokes the read callback
    // -----------------------------------------------------------------
    #[test]
    fn generic_sensor_invokes_read_callback() {
        READ_COUNT.store(0, Ordering::SeqCst);
        let s = GenericSensor::<MockClock>::new(Some(counting_read_callback), None);
        // Each call increments the static counter and returns 1.23.
        assert_eq!(s.get_sensor_angle(), 1.23);
        assert_eq!(s.get_sensor_angle(), 1.23);
        assert_eq!(s.get_sensor_angle(), 1.23);
        assert_eq!(READ_COUNT.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn generic_sensor_read_callback_can_return_any_value() {
        // A different callback returning π/2.
        let s = GenericSensor::<MockClock>::new(Some(constant_read_callback), None);
        assert!((s.get_sensor_angle() - core::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    // -----------------------------------------------------------------
    // init — invokes the init callback exactly once
    // -----------------------------------------------------------------
    #[test]
    fn generic_sensor_invokes_init_callback() {
        INIT_COUNT.store(0, Ordering::SeqCst);
        let clock = MockClock::new();
        clock.set_us(1_000);

        let mut s = GenericSensor::<MockClock>::new(None, Some(counting_init_callback));
        s.init(&clock);
        assert_eq!(INIT_COUNT.load(Ordering::SeqCst), 1);

        // A second init() call invokes it a second time.
        clock.advance_us(1_000);
        s.init(&clock);
        assert_eq!(INIT_COUNT.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn generic_sensor_init_seeds_base_state() {
        // The init callback runs first; the default base init
        // then seeds angle_prev/vel_angle_prev from the read
        // callback.
        let clock = MockClock::new();
        clock.set_us(5_000);

        let mut s = GenericSensor::<MockClock>::new(Some(constant_read_callback), None);
        s.init(&clock);
        // angle_prev was seeded from get_sensor_angle() = π/2.
        assert_eq!(s.base().angle_prev, core::f32::consts::FRAC_PI_2);
        assert_eq!(s.base().vel_angle_prev, core::f32::consts::FRAC_PI_2);
        assert_eq!(s.base().angle_prev_ts, 5_000);
    }

    // -----------------------------------------------------------------
    // None — safe default
    // -----------------------------------------------------------------
    #[test]
    fn generic_sensor_default_callbacks_return_zero() {
        // No callbacks: get_sensor_angle() must return 0.0
        // without panicking.
        let s = GenericSensor::<MockClock>::new(None, None);
        assert_eq!(s.get_sensor_angle(), 0.0);
    }

    #[test]
    fn generic_sensor_init_without_callback_does_not_panic() {
        // No init callback: init() must not panic, and the base
        // state must still be seeded (with 0.0 from the missing
        // read callback).
        let clock = MockClock::new();
        clock.set_us(2_000);

        let mut s = GenericSensor::<MockClock>::new(None, None);
        s.init(&clock);
        assert_eq!(s.base().angle_prev, 0.0);
        assert_eq!(s.base().angle_prev_ts, 2_000);
    }

    // -----------------------------------------------------------------
    // Construction — only the user-supplied callbacks are kept
    // -----------------------------------------------------------------
    #[test]
    fn generic_sensor_constructor_stores_callbacks() {
        let s = GenericSensor::<MockClock>::new(
            Some(counting_read_callback),
            Some(counting_init_callback),
        );
        assert!(s.read_callback.is_some());
        assert!(s.init_callback.is_some());

        let s_none = GenericSensor::<MockClock>::new(None, None);
        assert!(s_none.read_callback.is_none());
        assert!(s_none.init_callback.is_none());
    }
}
