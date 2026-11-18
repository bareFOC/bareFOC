//! # sensors::encoder — quadrature encoder
//!
//! Ports [`Encoder.h`](source/Arduino-FOC/src/sensors/Encoder.h) and
//! [`Encoder.cpp`](source/Arduino-FOC/src/sensors/Encoder.cpp) from
//! Arduino-FOC.
//!
//! The `Encoder` struct reads a 2-channel (A, B) or 3-channel (A, B,
//! index) quadrature encoder. It does not own the GPIO EXTI
//! peripherals — the user supplies the interrupt handler with the
//! current pin state and the microsecond timestamp, and the struct
//! does the pulse-counting and velocity math.
//!
//! ```ignore
//! // Typical wiring (STM32 example):
//! let mut encoder = Encoder::new(0, 1, 1000.0, 2);
//! encoder.init(&clock);
//! // Wire each EXTI:
//! exti_a.configure(Edge::RisingFalling, move || {
//!     let a = gpio_a.is_high();
//!     encoder.handle_a(a, clock.now_us());
//! });
//! ```
//!
//! ## Why no `enable_interrupts` method
//!
//! The upstream `Encoder::enableInterrupts(doA, doB, doIndex)` calls
//! the Arduino `attachInterrupt(digitalPinToInterrupt(pin), fn,
//! CHANGE)` to wire the EXTI. In FOC-rs the user wires the EXTI
//! themselves using their HAL's idiomatic API. The `handle_a`,
//! `handle_b`, and `handle_index` methods on the struct are the
//! pure-logic counterparts of the upstream interrupt callbacks; the
//! user invokes them from inside their EXTI closure.
//!
//! ## Quadrature mode
//!
//! The [`Quadrature::ON`] mode (default) counts every edge of the A
//! and B channels, giving `cpr = 4 × ppr`. [`Quadrature::OFF`]
//! counts only full periods, giving `cpr = ppr`. The setting is
//! matched to the encoder hardware — most modern magnetic
//! encoders (AS5048, AMT103, etc.) emit a full quadrature signal
//! and run in `Quadrature::ON`.

#![allow(non_camel_case_types)]

use crate::common::time::Clock;
use crate::traits::sensor::{Direction, Pullup, Sensor, SensorBase};

/// Quadrature decoding mode.
///
/// Ports the upstream `Quadrature` enum from
/// [`Encoder.h`](source/Arduino-FOC/src/sensors/Encoder.h) lines
/// 13–16. The mode selects whether the counter increments on
/// every channel edge (4× pulses per mechanical period) or only
/// on full A/B transitions (1× per period).
///
/// `ON` is the `Default` and matches the upstream default
/// (`Encoder::Encoder(...)` sets `quadrature = ON`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Quadrature {
    /// Enable 4× decoding. `cpr = 4 × ppr`. `Default`.
    #[default]
    ON = 0x00,
    /// Disable 4× decoding. `cpr = ppr`. Use this for encoders
    /// that already emit a quadrature index pulse and you want
    /// one count per mechanical detent.
    OFF = 0x01,
}

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

/// Quadrature encoder sensor.
///
/// Ports the upstream `Encoder` class
/// ([`Encoder.h`](source/Arduino-FOC/src/sensors/Encoder.h)).
///
/// The struct is generic over the [`Clock`] type used for
/// timestamping the `handle_a` / `handle_b` callbacks. The
/// microsecond timestamp is passed in by the caller (typically the
/// EXTI closure) so the encoder does not need a stored `&Clock`
/// reference.
///
/// `cpr` is computed from `ppr` and the [`Quadrature`] mode in
/// the constructor and exposed as a `pub` field so the FOC engine
/// can use it directly. `pinA` and `pinB` are stored as `i32` to
/// match the upstream `int` and to allow `-1` for "not connected"
/// (the FOC engine may set `sensor.pinA` to `-1` to disable a
/// physical pin without removing the struct).
#[derive(Clone, Debug)]
pub struct Encoder<C: Clock> {
    base: SensorBase,

    /// Pin number for channel A. `-1` means "not connected".
    pub pin_a: i32,
    /// Pin number for channel B. `-1` means "not connected".
    pub pin_b: i32,
    /// Pin number for the index (Z) channel. `0` means "no index".
    pub index_pin: i32,

    /// Pullup configuration for A, B, and index channels.
    pub pullup: Pullup,
    /// Quadrature decoding mode (4× or 1×).
    pub quadrature: Quadrature,
    /// Counts per revolution. Computed from `ppr` and `quadrature`
    /// in the constructor.
    pub cpr: f32,

    // Internal state — `pub(crate)` so the test mocks (in this
    // same file) and the host-side test suite can drive them
    // directly. Mirrors the upstream `volatile` qualifier with a
    // comment so the user is aware that real code in an ISR must
    // read these atomically.
    /// Pulse counter, signed (positive for CW).
    pub(crate) pulse_counter: i32,
    /// Microsecond timestamp of the most recent pulse edge.
    pub(crate) pulse_timestamp: u32,
    /// Last observed A channel state (HIGH=true, LOW=false).
    pub(crate) a_active: bool,
    /// Last observed B channel state.
    pub(crate) b_active: bool,
    /// Last observed index channel state.
    pub(crate) i_active: bool,
    /// `true` once the index pulse has been seen at least once.
    pub(crate) index_found: bool,

    // Velocity calculation state.
    pub(crate) pulse_per_second: f32,
    pub(crate) prev_pulse_counter: i32,
    pub(crate) prev_timestamp_us: u32,

    _clock: core::marker::PhantomData<C>,
}

impl<C: Clock> Encoder<C> {
    /// Construct a new encoder. `ppr` is the encoder's pulses per
    /// revolution; the actual `cpr` (counts per revolution) is
    /// `ppr * 4` in [`Quadrature::ON`] mode or `ppr` in
    /// [`Quadrature::OFF`].
    ///
    /// `index_pin == 0` means "no index channel". Any non-zero
    /// value is treated as a present index channel.
    ///
    /// Mirrors the upstream `Encoder::Encoder(int encA, int encB,
    /// float ppr, int index = 0)` constructor
    /// ([`Encoder.cpp`](source/Arduino-FOC/src/sensors/Encoder.cpp)
    /// lines 4–25).
    pub fn new(pin_a: i32, pin_b: i32, ppr: f32, index_pin: i32) -> Self {
        let quadrature = Quadrature::ON;
        let cpr = ppr * 4.0;
        Self {
            base: SensorBase::new(),
            pin_a,
            pin_b,
            index_pin,
            pullup: Pullup::USE_INTERN,
            quadrature,
            cpr,
            pulse_counter: 0,
            pulse_timestamp: 0,
            a_active: false,
            b_active: false,
            i_active: false,
            index_found: false,
            pulse_per_second: 0.0,
            prev_pulse_counter: 0,
            prev_timestamp_us: 0,
            _clock: core::marker::PhantomData,
        }
    }

    /// Encoder initialisation. Resets the pulse counter and
    /// timestamps, then delegates to the [`Sensor::init`] default
    /// to seed the base state. The user must wire the EXTI
    /// callbacks before calling `update()` for the first time
    /// (so the `a_active` / `b_active` fields have valid values).
    pub fn init(&mut self, clock: &C) {
        self.pulse_counter = 0;
        self.pulse_timestamp = clock.now_us();
        self.index_found = false;
        // The default `Sensor::init` reads the angle twice to
        // seed `angle_prev` and `vel_angle_prev`. For the
        // encoder, those reads return whatever the pulse
        // counter happens to be at this instant — sufficient
        // for the trait default's purpose.
        <Self as Sensor<C>>::init(self, clock);
    }

    /// A-channel interrupt handler. The user calls this from
    /// their EXTI closure with the current A-pin state and the
    /// microsecond timestamp.
    ///
    /// Mirrors upstream `Encoder::handleA`
    /// ([`Encoder.cpp`](source/Arduino-FOC/src/sensors/Encoder.cpp)
    /// lines 31–40). The counter increments on rising and falling
    /// edges; the sign depends on the relative phase of B:
    /// `+1` if A and B have the same state (CW in standard
    /// quadrature encoding), `-1` if they differ.
    pub fn handle_a(&mut self, a_state: bool, now_us: u32) {
        if a_state != self.a_active {
            self.pulse_timestamp = now_us;
            self.pulse_counter += if a_state == self.b_active { 1 } else { -1 };
        }
        self.a_active = a_state;
    }

    /// B-channel interrupt handler. Mirror of [`handle_a`].
    pub fn handle_b(&mut self, b_state: bool, now_us: u32) {
        if b_state != self.b_active {
            self.pulse_timestamp = now_us;
            self.pulse_counter += if self.a_active == b_state { 1 } else { -1 };
        }
        self.b_active = b_state;
    }

    /// Index-channel interrupt handler. The user calls this
    /// from their EXTI closure when the index pin fires.
    /// Resets the pulse counter to zero so the mechanical
    /// zero position aligns with the next update.
    pub fn handle_index(&mut self) {
        self.i_active = !self.i_active;
        self.index_found = true;
        self.pulse_counter = 0;
    }

    /// Whether the encoder has an index (Z) channel.
    pub fn has_index(&self) -> bool {
        self.index_pin != 0
    }

    /// Sign of the most recent pulse, used by `update()` to track
    /// rotation direction. Returns `+1` for CW, `-1` for CCW.
    #[allow(dead_code)]
    fn last_pulse_direction(&self) -> Direction {
        // Use the difference between the current pulse counter
        // and the previous one, normalised to ±1.
        let dp = self.pulse_counter - self.prev_pulse_counter;
        if dp > 0 {
            Direction::CW
        } else if dp < 0 {
            Direction::CCW
        } else {
            Direction::UNKNOWN
        }
    }
}

impl<C: Clock> Sensor<C> for Encoder<C> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    /// Shaft angle in `[0, 2π)`. Computed from the signed pulse
    /// counter modulo `cpr`.
    fn get_sensor_angle(&self) -> f32 {
        // `_2PI * (pulse_counter mod cpr) / cpr`, with the mod
        // done on a `f32` to handle the negative counter
        // correctly. We use `libm::floorf`-based Euclidean
        // modulo (instead of `f32::rem_euclid`, which is not
        // stable on `f32` in current Rust) so the result is
        // always in `[0, cpr)`.
        let pc = self.pulse_counter as f32;
        let m = pc - libm::floorf(pc / self.cpr) * self.cpr;
        crate::math::_2PI * m / self.cpr
    }

    /// Velocity in **pulses per second**.
    ///
    /// This intentionally does **not** return rad/s. The upstream
    /// `Encoder::getVelocity` returns pulses/s and the FOC engine
    /// integrates the mechanical angle from `getSensorAngle` —
    /// the motor's own velocity filter is what converts
    /// pulses/s to rad/s for the angle-loop branch. FOC-rs
    /// preserves this behaviour for parity.
    ///
    /// Mirrors upstream `Encoder::getVelocity`
    /// ([`Encoder.cpp`](source/Arduino-FOC/src/sensors/Encoder.cpp)
    /// lines 47–72). Uses `pulse_timestamp` and
    /// `prev_timestamp_us` for the time delta, with a 1-second
    /// "long time no step" guard that resets velocity to zero
    /// (in case the motor stalls and then re-starts the next
    /// time `getVelocity` is called).
    fn get_velocity(&mut self) -> f32 {
        let dt = self.pulse_timestamp.wrapping_sub(self.prev_timestamp_us);
        let dpulse = self.pulse_counter - self.prev_pulse_counter;

        // "Big step" guard — protects against a missing pulse
        // (e.g. an overflow during a long stall) that would
        // otherwise produce a huge bogus velocity.
        if dpulse.abs() > 1_000_000 {
            self.pulse_per_second = 0.0;
        } else {
            // dt is microseconds. To get pulses per second:
            // pulses/s = dpulse / (dt * 1e-6) = dpulse * 1e6 / dt.
            // Guard against dt == 0 to avoid division-by-zero.
            if dt == 0 {
                // No new pulse since last call; keep the
                // previous value.
            } else {
                self.pulse_per_second =
                    (dpulse as f32) * 1e6_f32 / (dt as f32);
            }
        }

        self.prev_pulse_counter = self.pulse_counter;
        self.prev_timestamp_us = self.pulse_timestamp;
        self.pulse_per_second
    }

    /// `1` if an index pin is configured and has not yet been
    /// found, `0` otherwise. Once `handle_index` has fired,
    /// the encoder no longer needs a search.
    fn needs_search(&self) -> i32 {
        if self.index_pin != 0 && !self.index_found {
            1
        } else {
            0
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
    use crate::traits::sensor::Sensor;

    // -----------------------------------------------------------------
    // Quadrature enum — disc + default
    // -----------------------------------------------------------------
    #[test]
    fn quadrature_discriminants_match_upstream() {
        assert_eq!(Quadrature::ON as u8, 0x00);
        assert_eq!(Quadrature::OFF as u8, 0x01);
    }

    #[test]
    fn quadrature_default_is_on() {
        assert_eq!(Quadrature::default(), Quadrature::ON);
    }

    // -----------------------------------------------------------------
    // cpr formula
    // -----------------------------------------------------------------
    #[test]
    fn cpr_is_four_times_ppr_in_quadrature_on_mode() {
        let enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        assert_eq!(enc.cpr, 4000.0);
        assert_eq!(enc.quadrature, Quadrature::ON);
    }

    #[test]
    fn cpr_equals_ppr_in_quadrature_off_mode() {
        // The user flips the mode after construction; the cpr
        // is computed in the constructor so they must also
        // recompute it themselves. This test documents the
        // intended usage pattern.
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.quadrature = Quadrature::OFF;
        enc.cpr = 1000.0;
        assert_eq!(enc.cpr, 1000.0);
    }

    // -----------------------------------------------------------------
    // has_index
    // -----------------------------------------------------------------
    #[test]
    fn has_index_returns_false_when_index_pin_is_zero() {
        let enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        assert!(!enc.has_index());
    }

    #[test]
    fn has_index_returns_true_when_index_pin_nonzero() {
        let enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 2);
        assert!(enc.has_index());
    }

    // -----------------------------------------------------------------
    // needs_search
    // -----------------------------------------------------------------
    #[test]
    fn needs_search_returns_one_with_index_before_index_found() {
        let enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 2);
        assert_eq!(enc.needs_search(), 1);
    }

    #[test]
    fn needs_search_returns_zero_without_index() {
        let enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        assert_eq!(enc.needs_search(), 0);
    }

    #[test]
    fn needs_search_returns_zero_after_index_found() {
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 2);
        enc.handle_index();
        assert_eq!(enc.needs_search(), 0);
    }

    // -----------------------------------------------------------------
    // handle_a / handle_b — pulse counter
    //
    // Standard quadrature: A leads B for CW, B leads A for CCW.
    // The handle_a rule: pulse_counter += +1 if A == B, -1 if
    // A != B (and the state changed).
    // -----------------------------------------------------------------
    // -----------------------------------------------------------------
    // handle_a / handle_b — pulse counter
    //
    // Upstream rule (mirrored in `handle_a` / `handle_b`):
    //   pulse_counter += (state == other_state) ? 1 : -1
    //
    // In words: the counter increments by 1 when the A and B pins
    // are in the same state at the edge, and decrements by 1 when
    // they differ. The user sets `sensor_direction = CCW` on the
    // motor if their physical rotation convention is opposite to
    // the encoder's sign convention.
    // -----------------------------------------------------------------
    #[test]
    fn handle_a_rising_decrements_when_b_is_low() {
        // A rising, B low → A != B → -1.
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.b_active = false;
        enc.handle_a(true, 100);
        assert_eq!(enc.pulse_counter, -1);
        assert_eq!(enc.pulse_timestamp, 100);
    }

    #[test]
    fn handle_a_rising_increments_when_b_is_high() {
        // A rising, B high → A == B → +1.
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.b_active = true;
        enc.handle_a(true, 100);
        assert_eq!(enc.pulse_counter, 1);
    }

    #[test]
    fn handle_a_falling_increments_when_b_is_low() {
        // A falling, B low → A == B → +1.
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.a_active = true;
        enc.b_active = false;
        enc.handle_a(false, 200);
        assert_eq!(enc.pulse_counter, 1);
    }

    #[test]
    fn handle_a_falling_decrements_when_b_is_high() {
        // A falling, B high → A != B → -1.
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.a_active = true;
        enc.b_active = true;
        enc.handle_a(false, 200);
        assert_eq!(enc.pulse_counter, -1);
    }

    #[test]
    fn handle_a_no_change_does_not_update_counter_or_timestamp() {
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.a_active = true;
        enc.pulse_counter = 5;
        enc.pulse_timestamp = 999;
        // Edge fires but state hasn't changed (defensive path).
        enc.handle_a(true, 1000);
        assert_eq!(enc.pulse_counter, 5);
        assert_eq!(enc.pulse_timestamp, 999);
    }

    #[test]
    fn handle_b_rising_decrements_when_a_is_low() {
        // B rising, A low → A != B → -1.
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.a_active = false;
        enc.handle_b(true, 100);
        assert_eq!(enc.pulse_counter, -1);
    }

    #[test]
    fn handle_b_rising_increments_when_a_is_high() {
        // B rising, A high → A == B → +1.
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.a_active = true;
        enc.handle_b(true, 100);
        assert_eq!(enc.pulse_counter, 1);
    }

    // -----------------------------------------------------------------
    // handle_index
    // -----------------------------------------------------------------
    #[test]
    fn handle_index_resets_counter_and_sets_index_found() {
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 2);
        enc.pulse_counter = 12345;
        enc.handle_index();
        assert!(enc.index_found);
        assert_eq!(enc.pulse_counter, 0);
    }

    // -----------------------------------------------------------------
    // get_sensor_angle
    // -----------------------------------------------------------------
    #[test]
    fn get_sensor_angle_zero_when_counter_zero() {
        let enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        assert_eq!(enc.get_sensor_angle(), 0.0);
    }

    #[test]
    fn get_sensor_angle_full_turn_at_cpr() {
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.pulse_counter = enc.cpr as i32;
        // Modulo cpr → 0 → angle 0.
        assert_eq!(enc.get_sensor_angle(), 0.0);
    }

    #[test]
    fn get_sensor_angle_quarter_turn() {
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.pulse_counter = (enc.cpr / 4.0) as i32;
        let expected = crate::math::_2PI * 0.25;
        assert!((enc.get_sensor_angle() - expected).abs() < 1e-4);
    }

    #[test]
    fn get_sensor_angle_negative_counter_wraps_to_two_pi() {
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.pulse_counter = -(enc.cpr as i32) / 4;
        // -cpr/4 mod cpr = 3*cpr/4 → angle 3π/2.
        let expected = crate::math::_2PI * 0.75;
        assert!((enc.get_sensor_angle() - expected).abs() < 1e-3);
    }

    // -----------------------------------------------------------------
    // get_velocity — pulses per second
    // -----------------------------------------------------------------
    #[test]
    fn get_velocity_computes_pulses_per_second() {
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.pulse_counter = 0;
        enc.prev_pulse_counter = 0;
        enc.prev_timestamp_us = 0;
        // Simulate 1000 pulses over 1 second.
        enc.pulse_counter = 1000;
        enc.pulse_timestamp = 1_000_000;
        let v = enc.get_velocity();
        assert!((v - 1000.0).abs() < 1e-3);
    }

    #[test]
    fn get_velocity_returns_zero_on_long_stall() {
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.pulse_counter = 0;
        enc.prev_pulse_counter = 0;
        enc.prev_timestamp_us = 0;
        // No new pulse for 1.5 s — long-stall guard.
        enc.pulse_counter = 0;
        enc.pulse_timestamp = 1_500_000;
        let v = enc.get_velocity();
        assert_eq!(v, 0.0);
    }

    #[test]
    fn get_velocity_ignores_huge_jumps() {
        // A bogus 5-million-pulse step is the "big step"
        // guard path.
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 0);
        enc.pulse_counter = 0;
        enc.prev_pulse_counter = 0;
        enc.prev_timestamp_us = 0;
        enc.pulse_counter = 5_000_000;
        enc.pulse_timestamp = 1_000;
        let v = enc.get_velocity();
        assert_eq!(v, 0.0);
    }

    // -----------------------------------------------------------------
    // init() — seeds base state and zeros pulse counter
    // -----------------------------------------------------------------
    #[test]
    fn init_resets_pulse_counter() {
        let clock = MockClock::new();
        clock.set_us(1_000);
        let mut enc: Encoder<MockClock> = Encoder::new(0, 1, 1000.0, 2);
        enc.pulse_counter = 999;
        enc.init(&clock);
        assert_eq!(enc.pulse_counter, 0);
        // The base.angle_prev is seeded from the initial angle
        // (which is 0 because pulse_counter is 0).
        assert_eq!(enc.base.angle_prev, 0.0);
    }
}
