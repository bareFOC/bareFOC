//! # time — microsecond clock abstraction
//!
//! Replaces the Arduino global `micros()` (and `_micros()` in
//! [`time_utils.h`](source/Arduino-FOC/src/common/time_utils.h)) with a
//! trait. The trait lives in `barefoc::common::time::Clock`; the
//! implementations are feature-gated per platform
//! (`PORTING_GUIDELINES.md` §9) and are out of scope for this crate
//! until phase 8.
//!
//! ## Deliberate deviations from upstream
//!
//! Upstream [`time_utils.cpp`](source/Arduino-FOC/src/common/time_utils.cpp)
//! exposes two functions:
//!
//! - `_micros()` → returns the hardware microsecond counter
//! - `_delay(ms)` → blocking millisecond delay (with an AVR-specific
//!   busy-loop workaround)
//!
//! `_micros()` becomes [`Clock::now_us`]. `_delay(ms)` becomes
//! [`Clock::delay_ms`] — a method on the same trait, with a default
//! no-op body. The default is empty so:
//!
//! - The `MockClock` test helper stays fast (motor tests run thousands
//!   of "delay" iterations in milliseconds-of-clock-time; a busy-wait
//!   fallback would slow host tests to a crawl).
//! - Production `Clock` impls (STM32, ESP32, …) override it with the
//!   right primitive for the platform (`cortex_m::asm::delay`,
//!   `esp_hal::delay::Delay`, RTIC `spawn`, etc.). A busy-wait on
//!   `now_us` was considered and rejected: it would be a footgun on
//!   FPU targets that have proper timer infrastructure.
//!
//! The upstream `unsigned long` return type (32-bit on AVR) becomes
//! `u32` here. The 71-minute wraparound behaviour is intentional and
//! matches upstream — callers using `now_us` for delta-time subtraction
//! must rely on modular arithmetic, which the upstream `pid.cpp` and
//! `lowpass_filter.cpp` already do (`(timestamp_now - timestamp_prev)
//! * 1e-6f`).
//!
//! ## Test helper
//!
//! `MockClock` lives behind `#[cfg(test)]` and is `pub(crate)` — it is
//! only used by the unit tests in this crate (filter + pid + motor). It
//! provides a settable "now" so tests can drive deterministic time
//! deltas without a real timer. `delay_ms` is inherited from the
//! trait's default no-op, so tests that call `clock.delay_ms(N)` do
//! nothing in zero time.

/// Microsecond monotonic clock. Replaces Arduino's global `micros()`.
///
/// Ports [`_micros`](source/Arduino-FOC/src/common/time_utils.h) from
/// Arduino-FOC. The trait replaces the global function so the FOC
/// engine can be tested without a real hardware timer.
///
/// # Contract
///
/// `now_us` returns a monotonically non-decreasing microsecond
/// counter. The 32-bit wraparound at ~71.6 minutes is intentional and
/// matches the upstream `unsigned long` semantics — callers should use
/// modular subtraction when computing elapsed time.
///
/// The trait takes `&self` (not `&mut self`) so it can be shared with
/// `Copy` semantics across the FOC loop without a `Cell`/`RefCell`.
/// Hardware implementations are expected to be stateless with respect
/// to the trait method (read a peripheral counter).
///
/// # Example
///
/// ```ignore
/// use barefoc::common::time::Clock;
///
/// struct MyClock;
/// impl Clock for MyClock {
///     fn now_us(&self) -> u32 { 0 } // hardware-specific
/// }
/// ```
pub trait Clock {
    /// Returns the current microsecond counter.
    fn now_us(&self) -> u32;

    /// Blocking millisecond delay. Default body is a no-op.
    ///
    /// Ports the upstream `_delay(ms)` helper
    /// ([`time_utils.cpp`](source/Arduino-FOC/src/common/time_utils.cpp)).
    /// The default is empty so test `Clock` impls (e.g. `MockClock`)
    /// stay fast — motor unit tests run thousands of `delay_ms`
    /// iterations inside the calibration routines, and a busy-wait on
    /// `now_us` would make the host test suite crawl.
    ///
    /// Production `Clock` impls in `phase 8` override this with the
    /// appropriate primitive for the target (`cortex_m::asm::delay` on
    /// STM32, `esp_hal::delay::Delay` on ESP32, an RTIC `spawn` on
    /// RTIC targets).
    fn delay_ms(&self, _ms: u32) {}
}

// ---------------------------------------------------------------------------
// Mock clock — used by unit tests in `filter.rs` and `pid.rs`.
// ---------------------------------------------------------------------------

/// Test-only mock clock with a settable time.
///
/// Lives under `#[cfg(test)]` and is `pub(crate)` so the host-side
/// unit tests for [`crate::common::filter`], [`crate::common::pid`],
/// and [`crate::motor`] can drive deterministic elapsed-time deltas
/// without a real timer. `delay_ms` is inherited from the
/// [`Clock::delay_ms`] default no-op, so `clock.delay_ms(N)` in
/// tests takes zero wall time.
///
/// In a typical test:
///
/// ```ignore
/// let clock = MockClock::new();
/// clock.set_us(1_000);
/// let mut lpf = LowPassFilter::new(0.01, NOT_SET, &clock);
/// clock.set_us(2_000);
/// assert!((lpf.update(10.0, &clock) - /* expected */).abs() < 1e-4);
/// ```
#[cfg(test)]
pub(crate) mod mock {
    use super::Clock;
    use core::cell::Cell;

    /// Deterministic, settable microsecond clock for unit tests.
    #[derive(Debug, Default)]
    pub struct MockClock {
        now: Cell<u32>,
    }

    impl MockClock {
        /// New mock clock initialised to `0 µs`.
        pub const fn new() -> Self {
            Self {
                now: Cell::new(0),
            }
        }

        /// Set the current time. Tests call this between `update`
        /// calls to simulate elapsed time.
        pub fn set_us(&self, t: u32) {
            self.now.set(t);
        }

        /// Advance the current time by `delta_us` microseconds.
        pub fn advance_us(&self, delta_us: u32) {
            self.now.set(self.now.get().wrapping_add(delta_us));
        }
    }

    impl Clock for MockClock {
        fn now_us(&self) -> u32 {
            self.now.get()
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::mock::MockClock;
    use super::Clock;

    #[test]
    fn mock_clock_starts_at_zero() {
        let c = MockClock::new();
        assert_eq!(c.now_us(), 0);
    }

    #[test]
    fn mock_clock_set_us() {
        let c = MockClock::new();
        c.set_us(1_234_567);
        assert_eq!(c.now_us(), 1_234_567);
    }

    #[test]
    fn mock_clock_advance() {
        let c = MockClock::new();
        c.set_us(500);
        c.advance_us(750);
        assert_eq!(c.now_us(), 1_250);
    }

    #[test]
    fn mock_clock_wraps_at_u32_boundary() {
        // Matches upstream _micros() behaviour: 32-bit modular wrap.
        let c = MockClock::new();
        c.set_us(u32::MAX);
        c.advance_us(5);
        // The clock value itself wraps; callers that subtract use
        // modular arithmetic.
        assert_eq!(c.now_us(), 4);
    }

    #[test]
    fn clock_is_object_safe() {
        // Trait-object usage is a deliberate supported pattern (e.g.,
        // the FOC motor could optionally be constructed with a
        // `&dyn Clock` if a single shared clock is desired). The hot
        // FOC loop will still be generic over `C: Clock` for zero-cost
        // dispatch, but `dyn` is a valid fallback.
        let c = MockClock::new();
        let dyn_clock: &dyn Clock = &c;
        c.set_us(42);
        assert_eq!(dyn_clock.now_us(), 42);
    }

    #[test]
    fn delay_ms_is_default_noop() {
        // The default `delay_ms` body must be a no-op: it must not
        // touch the clock state and must not consume any wall time.
        // This is the contract motor unit tests rely on (they call
        // `clock.delay_ms(N)` inside the calibration routines; the
        // mock must no-op so the test suite stays fast).
        let c = MockClock::new();
        c.set_us(1_000);
        c.delay_ms(500);
        // Time is unchanged — the default body is empty.
        assert_eq!(c.now_us(), 1_000);
    }
}
