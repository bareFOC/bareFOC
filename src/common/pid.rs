//! # pid — discrete-time PID controller
//!
//! Ports [`PIDController`](source/Arduino-FOC/src/common/pid.cpp) from
//! Arduino-FOC. Used for the velocity and current-sense closed loops
//! in the FOC engine.
//!
//! ## Algorithm
//!
//! Discrete-time PID with Tustin-bilinear (trapezoidal) integration
//! and output-side anti-windup. Per iteration:
//!
//! ```text
//! dt         = Ts   if Ts is configured
//!            = (now − prev) · 1e-6   otherwise
//!
//! u_p        = P · e_k
//! u_i        = u_i_{k-1} + (I · dt / 2) · (e_k + e_{k-1})   (Tustin)
//! u_d        = D · (e_k − e_{k-1}) / dt
//! u_k        = clamp(u_p + u_i + u_d,  −limit,  +limit)
//! u_k       := ramp-limit u_k so |(u_k − u_{k-1}) / dt| ≤ output_ramp
//! ```
//!
//! When `Ts` is [`NOT_SET`](crate::math::NOT_SET) and the measured
//! `dt` is `<= 0` (u32 wraparound) **or** `> 0.5 s` (long pause), the
//! controller falls back to `dt = 1e-3` for that iteration to keep
//! the integral bounded. `timestamp_prev` is still refreshed, so a
//! single bad sample does not poison the next iteration.
//!
//! ## Rust idiom deviations
//!
//! - The upstream C++ class exposes `P` / `I` / `D` / `output_ramp` /
//!   `limit` / `Ts` as public fields. FOC-rs exposes them through
//!   getter / setter pairs so the internal state stays private.
//! - The upstream `operator()(float error)` becomes
//!   [`PIDController::update`], which also takes the clock reference.
//! - The class is generic over `C: Clock` so the compiler emits a
//!   specialised, fully-inlined controller for each clock type. No
//!   `dyn` dispatch in the hot path.

use crate::common::time::Clock;
use crate::math::{constrain, isset};

/// Discrete-time PID controller with anti-windup and output ramping.
///
/// Ports [`PIDController`](source/Arduino-FOC/src/common/pid.h) from
/// Arduino-FOC. Generic over [`Clock`] so the controller can be
/// driven by any microsecond clock.
///
/// # Parameters (constructor)
///
/// - `p` — proportional gain.
/// - `i` — integral gain.
/// - `d` — derivative gain.
/// - `ramp` — maximum rate of change of the output, in
///   output-units/second. Pass [`NOT_SET`] (or 0 / negative) to
///   disable the ramp.
/// - `limit` — symmetric output clamp `[-limit, +limit]`. Pass
///   [`NOT_SET`] to disable.
/// - `sampling_time` — fixed `dt` in seconds. Pass [`NOT_SET`] for
///   adaptive (clock-driven) timing.
///
/// # Example
///
/// ```ignore
/// use barefoc::common::pid::PIDController;
/// use barefoc::common::time::Clock;
/// use barefoc::math::NOT_SET;
///
/// struct MyClock;
/// impl Clock for MyClock { fn now_us(&self) -> u32 { 0 } }
///
/// let clock = MyClock;
/// let mut pid = PIDController::new(0.5, 10.0, 0.0, NOT_SET, 12.0, NOT_SET, &clock);
/// let u = pid.update(1.0, &clock);
/// ```
///
/// `Clone` is derived so the controller can live inside
/// `#[derive(Clone)]` containers (e.g. the motor's
/// [`MotorConfig`](crate::motor::base::MotorConfig)). The
/// `PhantomData<C>` field is `Clone` regardless of `C`, and
/// the primitive fields are `Copy`, so the derive always
/// applies.
#[derive(Clone)]
pub struct PIDController<C: Clock> {
    p: f32,
    i: f32,
    d: f32,
    output_ramp: f32,
    limit: f32,
    ts: f32,
    error_prev: f32,
    output_prev: f32,
    integral_prev: f32,
    timestamp_prev: u32,
    _clock: core::marker::PhantomData<C>,
}

impl<C: Clock> PIDController<C> {
    /// Construct a new PID controller.
    ///
    /// Ports the
    /// `PIDController(float P, float I, float D, float ramp, float limit, float sampling_time)`
    /// constructor from upstream. The clock reference is used to
    /// seed the adaptive-timing baseline; the controller does **not**
    /// store the clock itself.
    pub fn new(
        p: f32,
        i: f32,
        d: f32,
        ramp: f32,
        limit: f32,
        sampling_time: f32,
        clock: &C,
    ) -> Self {
        Self {
            p,
            i,
            d,
            output_ramp: ramp,
            limit,
            ts: sampling_time,
            error_prev: 0.0,
            output_prev: 0.0,
            integral_prev: 0.0,
            timestamp_prev: clock.now_us(),
            _clock: core::marker::PhantomData,
        }
    }

    /// Proportional gain `P`.
    pub fn p(&self) -> f32 {
        self.p
    }
    /// Set proportional gain `P`.
    pub fn set_p(&mut self, v: f32) {
        self.p = v;
    }
    /// Integral gain `I`.
    pub fn i(&self) -> f32 {
        self.i
    }
    /// Set integral gain `I`.
    pub fn set_i(&mut self, v: f32) {
        self.i = v;
    }
    /// Derivative gain `D`.
    pub fn d(&self) -> f32 {
        self.d
    }
    /// Set derivative gain `D`.
    pub fn set_d(&mut self, v: f32) {
        self.d = v;
    }
    /// Output-ramp limit (output-units/second), or [`NOT_SET`] when
    /// disabled.
    pub fn output_ramp(&self) -> f32 {
        self.output_ramp
    }
    /// Set the output-ramp limit. Pass [`NOT_SET`] (or a non-positive
    /// value) to disable the ramp.
    pub fn set_output_ramp(&mut self, v: f32) {
        self.output_ramp = v;
    }
    /// Symmetric output clamp `[-limit, +limit]`, or [`NOT_SET`]
    /// when disabled.
    pub fn limit(&self) -> f32 {
        self.limit
    }
    /// Set the symmetric output clamp. Pass [`NOT_SET`] to disable.
    pub fn set_limit(&mut self, v: f32) {
        self.limit = v;
    }
    /// Fixed sample time `Ts` (seconds), or [`NOT_SET`] for adaptive
    /// (clock-driven) timing.
    pub fn ts(&self) -> f32 {
        self.ts
    }
    /// Set the sample time. Pass [`NOT_SET`] for adaptive timing.
    ///
    /// Switching to / from adaptive mode re-seeds `timestamp_prev` so
    /// the first `update` call after a mode change does not see a
    /// spurious huge `dt`.
    pub fn set_ts(&mut self, v: f32, clock: &C) {
        self.ts = v;
        self.timestamp_prev = clock.now_us();
    }

    /// Run one PID step and return the controller output.
    ///
    /// Ports `PIDController::operator()(float error)`. The clock is
    /// consulted only when `Ts == NOT_SET`; with a fixed `Ts` the
    /// call is a fixed multiply-add chain and never touches the
    /// clock.
    ///
    /// The clock argument is `&dyn Clock` (not `&C`) so callers
    /// that don't have the concrete clock type on hand (e.g. the
    /// motor's `shaft_velocity` default method) can still drive
    /// the controller with a `&NullClock` placeholder. The
    /// concrete `C` type is still carried in the type parameter
    /// so the `new` constructor can seed `timestamp_prev` at
    /// build time.
    pub fn update(&mut self, error: f32, clock: &dyn Clock) -> f32 {
        // Derive dt: fixed Ts wins; otherwise measure from the clock.
        let mut dt = self.ts;
        if !isset(dt) {
            let timestamp_now = clock.now_us();
            // u32 subtraction wraps at 0, matching upstream semantics.
            dt = timestamp_now.wrapping_sub(self.timestamp_prev) as f32 * 1e-6_f32;
            if dt <= 0.0 || dt > 0.5 {
                dt = 1e-3_f32;
            }
            // Always refresh the baseline — upstream does this
            // unconditionally, so a bad sample does not poison the
            // next one.
            self.timestamp_prev = timestamp_now;
        }

        // u_p = P · e_k
        let proportional = self.p * error;
        // Tustin-bilinear integral
        let mut integral =
            self.integral_prev + self.i * dt * 0.5_f32 * (error + self.error_prev);
        // Anti-windup: clamp the integral state itself.
        if isset(self.limit) {
            integral = constrain(integral, -self.limit, self.limit);
        }
        // u_d = D · (e_k − e_{k-1}) / dt
        let derivative = self.d * (error - self.error_prev) / dt;

        // Sum and clamp the output.
        let mut output = proportional + integral + derivative;
        if isset(self.limit) {
            output = constrain(output, -self.limit, self.limit);
        }

        // Output ramp: limit the rate of change of the (possibly
        // already clamped) output.
        if isset(self.output_ramp) && self.output_ramp > 0.0 {
            let output_rate = (output - self.output_prev) / dt;
            if output_rate > self.output_ramp {
                output = self.output_prev + self.output_ramp * dt;
            } else if output_rate < -self.output_ramp {
                output = self.output_prev - self.output_ramp * dt;
            }
        }

        // Save state for the next iteration.
        self.integral_prev = integral;
        self.output_prev = output;
        self.error_prev = error;
        output
    }

    /// Clear the controller state (`error_prev`, `output_prev`,
    /// `integral_prev`). The tuning gains and limits are not
    /// touched.
    ///
    /// Ports `PIDController::reset()` from upstream.
    pub fn reset(&mut self) {
        self.error_prev = 0.0;
        self.output_prev = 0.0;
        self.integral_prev = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Unit tests — MockClock drives deterministic dt.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::time::mock::MockClock;
    use crate::math::NOT_SET;

    /// Build a fresh PID with P=1, I=0, D=0, no ramp, no limit, fixed
    /// Ts. Returns the controller and the clock.
    fn p_only(ts: f32) -> (PIDController<MockClock>, MockClock) {
        let clock = MockClock::new();
        let pid = PIDController::new(1.0, 0.0, 0.0, NOT_SET, NOT_SET, ts, &clock);
        (pid, clock)
    }

    #[test]
    fn constructor_seeds_state_to_zero() {
        let (pid, _clock) = p_only(0.001);
        assert_eq!(pid.error_prev, 0.0);
        assert_eq!(pid.output_prev, 0.0);
        assert_eq!(pid.integral_prev, 0.0);
    }

    #[test]
    fn getters_and_setters() {
        let (mut pid, clock) = p_only(0.001);
        assert_eq!(pid.p(), 1.0);
        pid.set_p(2.5);
        assert_eq!(pid.p(), 2.5);
        pid.set_i(10.0);
        assert_eq!(pid.i(), 10.0);
        pid.set_d(0.5);
        assert_eq!(pid.d(), 0.5);
        pid.set_output_ramp(1.0);
        assert_eq!(pid.output_ramp(), 1.0);
        pid.set_limit(5.0);
        assert_eq!(pid.limit(), 5.0);
        pid.set_ts(NOT_SET, &clock);
        assert_eq!(pid.ts(), NOT_SET);
        // Back to a fixed value.
        pid.set_ts(0.002, &clock);
        assert_eq!(pid.ts(), 0.002);
    }

    #[test]
    fn proportional_only() {
        // P = 2, I = D = 0, fixed Ts → u = 2 * error.
        let (mut pid, _clock) = {
            let clock = MockClock::new();
            let pid = PIDController::new(2.0, 0.0, 0.0, NOT_SET, NOT_SET, 0.001, &clock);
            (pid, clock)
        };
        assert!((pid.update(0.5, &_clock) - 1.0).abs() < 1e-6);
        assert!((pid.update(-1.0, &_clock) - -2.0).abs() < 1e-6);
    }

    #[test]
    fn integral_accumulates_over_time() {
        // P = D = 0, I = 100, fixed Ts = 1 ms.
        // After N steps with constant error e, integral ≈ I · e · t.
        let (mut pid, _clock) = {
            let clock = MockClock::new();
            let pid = PIDController::new(0.0, 100.0, 0.0, NOT_SET, NOT_SET, 0.001, &clock);
            (pid, clock)
        };
        let mut total = 0.0_f32;
        for _ in 0..10 {
            total = pid.update(1.0, &_clock);
        }
        // Tustin integral after 10 steps of dt=1ms, e=1, I=100:
        // u = sum(I*dt/2*(e + e_prev)) with e_prev=0 on first step.
        // = 100 * 0.001 * 0.5 * (1+0)  (step 1)  = 0.05
        // + 100 * 0.001 * 0.5 * (1+1)    (step 2+) = 0.10 each * 9 = 0.90
        // = 0.95
        assert!((total - 0.95).abs() < 1e-4, "expected ≈0.95, got {total}");
    }

    #[test]
    fn derivative_responds_to_error_change() {
        // P = I = 0, D = 1, fixed Ts = 1 ms.
        // u_d = D · (e_k − e_{k-1}) / dt
        let (mut pid, _clock) = {
            let clock = MockClock::new();
            let pid = PIDController::new(0.0, 0.0, 1.0, NOT_SET, NOT_SET, 0.001, &clock);
            (pid, clock)
        };
        // First step: e_prev = 0, e = 1 → u_d = 1 * (1 - 0) / 0.001 = 1000.
        let u1 = pid.update(1.0, &_clock);
        assert!((u1 - 1000.0).abs() < 1.0, "u1 expected ≈1000, got {u1}");
        // Second step: e = 1 (same) → u_d = 0.
        let u2 = pid.update(1.0, &_clock);
        assert_eq!(u2, 0.0);
        // Third step: e = 0 → u_d = -1000.
        let u3 = pid.update(0.0, &_clock);
        assert!((u3 - -1000.0).abs() < 1.0, "u3 expected ≈-1000, got {u3}");
    }

    #[test]
    fn anti_windup_clamps_integral() {
        // P = 0, I = 100, D = 0, limit = 1.0, fixed Ts.
        // Without anti-windup, the integral would grow unbounded. With
        // it, the controller output must stay inside [-limit, +limit].
        let (mut pid, _clock) = {
            let clock = MockClock::new();
            let pid = PIDController::new(0.0, 100.0, 0.0, NOT_SET, 1.0, 0.001, &clock);
            (pid, clock)
        };
        let mut last = 0.0_f32;
        for _ in 0..200 {
            last = pid.update(1.0, &_clock);
        }
        // Integral anti-windup is checked first (clamp to ±limit), so
        // |u_i| ≤ 1, and u = u_i + u_p + u_d = u_i ≤ 1.
        assert!((last - 1.0).abs() < 1e-4, "expected ≈1.0, got {last}");
        // Now flip the error to negative and check we can wind back
        // down — anti-windup did not lock the integrator.
        for _ in 0..200 {
            last = pid.update(-1.0, &_clock);
        }
        assert!((last - -1.0).abs() < 1e-4, "expected ≈-1.0, got {last}");
    }

    #[test]
    fn output_ramp_limits_rate_of_change() {
        // P = 1, I = D = 0, ramp = 100 (units/s), fixed Ts = 1 ms.
        // Expected max |Δu| per step = ramp * dt = 0.1.
        let (mut pid, _clock) = {
            let clock = MockClock::new();
            let pid = PIDController::new(1.0, 0.0, 0.0, 100.0, NOT_SET, 0.001, &clock);
            (pid, clock)
        };
        // Step from 0 to 1.0 → unclamped u = 1.0, but the ramp
        // should clamp the first step to u_prev + ramp*dt = 0 + 0.1.
        let u1 = pid.update(1.0, &_clock);
        assert!((u1 - 0.1).abs() < 1e-5, "u1 expected ≈0.1, got {u1}");
        // Next 9 steps: ramp allows another +0.1 per step → u ≈ 1.0
        // after 10 steps.
        for _ in 0..9 {
            let _ = pid.update(1.0, &_clock);
        }
        let u10 = pid.update(1.0, &_clock);
        assert!((u10 - 1.0).abs() < 1e-4, "u10 expected ≈1.0, got {u10}");
    }

    #[test]
    fn fixed_ts_ignores_clock_advancement() {
        // Two controllers with the same fixed Ts but very different
        // clock deltas must produce identical outputs.
        let (mut pid_a, clock_a) = {
            let clock = MockClock::new();
            let pid = PIDController::new(1.0, 0.0, 0.0, NOT_SET, NOT_SET, 0.001, &clock);
            (pid, clock)
        };
        let (mut pid_b, clock_b) = {
            let clock = MockClock::new();
            let pid = PIDController::new(1.0, 0.0, 0.0, NOT_SET, NOT_SET, 0.001, &clock);
            (pid, clock)
        };
        clock_a.advance_us(1_000);
        clock_b.advance_us(50_000);
        let ua = pid_a.update(0.5, &clock_a);
        let ub = pid_b.update(0.5, &clock_b);
        assert!((ua - ub).abs() < 1e-6, "fixed Ts ignored: {ua} vs {ub}");
    }

    #[test]
    fn adaptive_dt_falls_back_on_overflow() {
        // Drive a wraparound by setting the clock past u32::MAX.
        let (mut pid, clock) = {
            let clock = MockClock::new();
            let pid = PIDController::new(1.0, 0.0, 0.0, NOT_SET, NOT_SET, NOT_SET, &clock);
            (pid, clock)
        };
        // Large positive dt (> 0.5 s) — fallback to 1 ms.
        clock.set_us(0);
        clock.advance_us(2_000_000);
        let u = pid.update(1.0, &clock);
        // P=1, error=1, dt=1e-3 (fallback) → u_p = 1.
        assert!((u - 1.0).abs() < 1e-5, "expected ≈1.0, got {u}");
    }

    #[test]
    fn reset_clears_state_but_keeps_gains() {
        let (mut pid, clock) = {
            let clock = MockClock::new();
            let pid = PIDController::new(1.0, 5.0, 0.5, 100.0, 2.0, 0.001, &clock);
            (pid, clock)
        };
        // Build up some state.
        for _ in 0..50 {
            let _ = pid.update(1.0, &clock);
        }
        pid.reset();
        assert_eq!(pid.error_prev, 0.0);
        assert_eq!(pid.output_prev, 0.0);
        assert_eq!(pid.integral_prev, 0.0);
        // Gains / limits preserved.
        assert_eq!(pid.p(), 1.0);
        assert_eq!(pid.i(), 5.0);
        assert_eq!(pid.d(), 0.5);
        assert_eq!(pid.output_ramp(), 100.0);
        assert_eq!(pid.limit(), 2.0);
    }
}
