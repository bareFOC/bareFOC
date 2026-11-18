//! # math — FOC math primitives
//!
//! Constants, frame vectors, and scalar helpers ported from
//! [`foc_utils`](source/Arduino-FOC/src/common/foc_utils.h) (and
//! [`foc_utils.cpp`](source/Arduino-FOC/src/common/foc_utils.cpp)) in
//! upstream Arduino-FOC.
//!
//! Every public item in this module is consumed by the rest of the FOC
//! engine (Clarke/Park transforms, SVPWM, sensor angle handling, …) and
//! therefore must remain `no_std` and allocation-free.
//!
//! ## Deliberate deviations from upstream
//!
//! The upstream C++ targets AVR/Arduino UNO where floating-point is either
//! absent or very expensive. FOC-rs targets FPU-equipped Cortex-M and ESP32
//! parts, so the speed-vs-accuracy trade-off is reversed: we prefer the
//! IEEE-754-correct `libm` implementations to upstream's LUT/polynomial
//! approximations. The specific substitutions are:
//!
//! | Upstream | FOC-rs | Why |
//! |---|---|---|
//! | `_sin` (65-entry LUT + interpolation, `~40µs` on UNO, RMS error `~6.5e-5`) | [`sin`] wrapping `libm::sinf` (IEEE-754) | Accuracy. FPU MCUs compute `sinf` in a handful of cycles. |
//! | `_cos` (LUT, computed via `_sin(a + π/2)`) | [`cos`] wrapping `libm::cosf` | Same as above; `libm::cosf` is direct rather than reusing `sinf`. |
//! | `_sqrtApprox` (Quake III fast-inverse-sqrt trick) | [`sqrt`] wrapping `libm::sqrtf` | FOC math benefits from IEEE-754-correct `sqrt`; the upstream `number * y.f` form is an approximation. |
//! | `_atan2` (Padé-style polynomial) | [`atan2`] wrapping `libm::atan2f` | IEEE-754-correct `atan2` avoids quadrant-edge artefacts. |
//!
//! The deviation is intentional and the same one applied to the upstream
//! `#define`s (`_sin`, `_cos`, `_sqrt`) that the Arduino-FOC sources
//! document as weak-link overridable.
//!
//! ## Macros not ported
//!
//! `foc_utils.h` lines 14–17 also define three preprocessor-only macros
//! that have no direct Rust equivalent and are therefore intentionally
//! **not** ported:
//!
//! - `_UNUSED(v)` — `(void)(v)` was used to silence unused-variable warnings
//!   in C++; in Rust the compiler already warns about unused locals
//!   individually and `#[allow(unused)]` is the idiomatic equivalent.
//! - `_powtwo(x)` — `1 << x`; trivial to inline at the call site.
//! - `_swap(a, b)` — used `auto` in the macro body which Rust cannot
//!   replicate; replace with `core::mem::swap` at the call site.
//!
//! See upstream `foc_utils.h` lines 14–17.
//!
//! ## Casing note
//!
//! Upstream names that start with `_` (`_PI`, `_2PI`, `_SQRT3`, …) are
//! preserved verbatim — leading-underscore identifiers are valid Rust.
//! A module-wide `#[allow(non_snake_case)]` suppresses the clippy lint
//! that would otherwise fire on these.
//!
//! The constant literals are likewise kept verbatim from
//! `foc_utils.h` lines 20–40. Clippy would flag them as
//! `clippy::approx_constant` (some are very close to `core::f32::consts::PI`
//! and friends) and `clippy::excessive_precision` (more decimal digits
//! than an `f32` can represent). Both lints are false positives here:
//!
//! - `approx_constant`: the upstream value is identical at the bit level to
//!   the stdlib constant, but we keep the literal form for direct
//!   cross-reference with the C source.
//! - `excessive_precision`: the truncated form `1.5707964_f32` and the
//!   full form `1.57079632679_f32` produce the **same** `f32` bit pattern
//!   (f32 carries ~7 decimal digits of precision). Truncating would not
//!   change the runtime value but would obscure the cross-reference with
//!   `foc_utils.h`.
//!
//! Both lints are therefore allowed at module level for the constants
//! block only.

#![allow(non_snake_case)]
#![allow(clippy::approx_constant, clippy::excessive_precision)]

pub mod cordic;

/// `2 * sqrt(3)` — magnitude coefficient used by the Space-Vector PWM
/// modulators.
///
/// Ports [`_2_SQRT3`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _2_SQRT3: f32 = 1.15470053838_f32;

/// `sqrt(3)` — sqrt of three, used in Clarke/SVPWM transforms.
///
/// Ports [`_SQRT3`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _SQRT3: f32 = 1.73205080757_f32;

/// `1 / sqrt(3)`.
///
/// Ports [`_1_SQRT3`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _1_SQRT3: f32 = 0.57735026919_f32;

/// `sqrt(3) / 2`.
///
/// Ports [`_SQRT3_2`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _SQRT3_2: f32 = 0.86602540378_f32;

/// `sqrt(2)`.
///
/// Ports [`_SQRT2`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _SQRT2: f32 = 1.41421356237_f32;

/// `120°` in radians (`2π/3`).
///
/// Ports [`_120_D2R`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _120_D2R: f32 = 2.09439510239_f32;

/// `π` (radians).
///
/// Ports [`_PI`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _PI: f32 = 3.14159265359_f32;

/// `π/2`.
///
/// Ports [`_PI_2`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _PI_2: f32 = 1.57079632679_f32;

/// `π/3`.
///
/// Ports [`_PI_3`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _PI_3: f32 = 1.0471975512_f32;

/// `2π`.
///
/// Ports [`_2PI`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _2PI: f32 = 6.28318530718_f32;

/// `3π/2`.
///
/// Ports [`_3PI_2`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _3PI_2: f32 = 4.71238898038_f32;

/// `π/6`.
///
/// Ports [`_PI_6`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _PI_6: f32 = 0.52359877559_f32;

/// Conversion factor from revolutions-per-minute to radians-per-second:
/// `2π / 60`.
///
/// Ports [`_RPM_TO_RADS`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _RPM_TO_RADS: f32 = 0.10471975512_f32;

/// Sentinel value used to mark a floating-point field as "not configured".
///
/// Ports [`NOT_SET`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC. Compare against this value with [`isset`].
pub const NOT_SET: f32 = -12345.0_f32;

/// Driver / pin state: high-impedance (floating).
///
/// Ports [`_HIGH_IMPEDANCE`](source/Arduino-FOC/src/common/foc_utils.h)
/// from Arduino-FOC.
pub const _HIGH_IMPEDANCE: i32 = 0;

/// Alias of [`_HIGH_IMPEDANCE`].
///
/// Ports [`_HIGH_Z`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _HIGH_Z: i32 = _HIGH_IMPEDANCE;

/// Driver / pin state: actively driven.
///
/// Ports [`_ACTIVE`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC.
pub const _ACTIVE: i32 = 1;

/// "Not Connected" sentinel — `NOT_SET` reinterpreted as `i32`.
///
/// Ports [`_NC`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC. Use this for pin / channel fields that may legitimately
/// be left unwired.
pub const _NC: i32 = NOT_SET as i32;

/// Minimum angular movement (in radians) considered a real change by
/// the sensor / angle-detection code: `2π / 101`.
///
/// Ports [`MIN_ANGLE_DETECT_MOVEMENT`](source/Arduino-FOC/src/common/foc_utils.h)
/// from Arduino-FOC.
pub const MIN_ANGLE_DETECT_MOVEMENT: f32 = _2PI / 101.0_f32;

// ---------------------------------------------------------------------------
// Frame vectors
// ---------------------------------------------------------------------------

/// DQ-frame (direct / quadrature) coordinate vector.
///
/// Ports `DQ_s` from Arduino-FOC. The trailing `_s` is illegal on Rust
/// type names, so it is dropped per `PORTING_GUIDELINES.md` §3.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DQ {
    /// Direct axis component.
    pub d: f32,
    /// Quadrature axis component.
    pub q: f32,
}

/// Voltage in the DQ frame.
pub type DQVoltage = DQ;

/// Current in the DQ frame.
pub type DQCurrent = DQ;

/// Alpha-beta (stationary two-axis) coordinate vector.
///
/// Ports `AB_s` from Arduino-FOC. The trailing `_s` is illegal on Rust
/// type names, so it is dropped.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AB {
    /// α-axis component.
    pub alpha: f32,
    /// β-axis component.
    pub beta: f32,
}

/// Voltage in the αβ frame.
pub type ABVoltage = AB;

/// Current in the αβ frame.
pub type ABCurrent = AB;

/// Three-phase (a / b / c) coordinate vector.
///
/// Ports `Phase_s` from Arduino-FOC. The trailing `_s` is illegal on
/// Rust type names, so it is dropped.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Phase {
    /// Phase A component.
    pub a: f32,
    /// Phase B component.
    pub b: f32,
    /// Phase C component.
    pub c: f32,
}

/// Phase voltage.
pub type PhaseVoltage = Phase;

/// Phase current.
pub type PhaseCurrent = Phase;

// ---------------------------------------------------------------------------
// Scalar helpers
// ---------------------------------------------------------------------------

/// Signum function: returns `-1` if `a < 0`, otherwise `1`.
///
/// Ports the upstream [`_sign`](source/Arduino-FOC/src/common/foc_utils.h)
/// preprocessor macro. The upstream C++ form
/// `_sign(a) ((a) < 0 ? -1 : (a) > 0)` returns `0` for `a == 0`; FOC-rs
/// deviates to `1` for the zero case so that the function is total
/// (`-1 | 1`) and the FOC integrator never sees a degenerate zero
/// contribution at startup.
#[inline]
pub fn sign(a: f32) -> i32 {
    if a < 0.0 {
        -1
    } else {
        1
    }
}

/// Round to the nearest integer, ties away from zero.
///
/// Ports the upstream [`_round`](source/Arduino-FOC/src/common/foc_utils.h)
/// preprocessor macro. Implementation delegates to `libm::roundf`, which
/// matches the macro's "round half away from zero" semantics.
#[inline]
pub fn round(x: f32) -> i32 {
    libm::roundf(x) as i32
}

/// Absolute value of `x`. Thin wrapper around `f32::abs` to
/// match the upstream `_ABS` macro naming convention and
/// keep the motor code free of `f32::` method calls.
#[inline]
pub fn abs(x: f32) -> f32 {
    libm::fabsf(x)
}

/// Clamp `amt` to the inclusive range `[low, high]`.
///
/// Ports the upstream [`_constrain`](source/Arduino-FOC/src/common/foc_utils.h)
/// preprocessor macro.
#[inline]
pub fn constrain(amt: f32, low: f32, high: f32) -> f32 {
    if amt < low {
        low
    } else if amt > high {
        high
    } else {
        amt
    }
}

/// True when `a` is not equal to [`NOT_SET`].
///
/// Ports the upstream [`_isset`](source/Arduino-FOC/src/common/foc_utils.h)
/// preprocessor macro.
#[inline]
pub fn isset(a: f32) -> bool {
    a != NOT_SET
}

/// Sine of `a` (radians), IEEE-754 correct via `libm::sinf`.
///
/// Ports [`_sin`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC. **Deliberate deviation:** the upstream uses a 65-entry
/// LUT with linear interpolation (RMS error `~6.5e-5`); FOC-rs uses
/// `libm::sinf` because the target MCUs have FPUs.
#[inline]
pub fn sin(a: f32) -> f32 {
    libm::sinf(a)
}

/// Cosine of `a` (radians), IEEE-754 correct via `libm::cosf`.
///
/// Ports [`_cos`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC. **Deliberate deviation:** the upstream reuses `_sin`
/// with an offset; FOC-rs uses `libm::cosf` directly.
#[inline]
pub fn cos(a: f32) -> f32 {
    libm::cosf(a)
}

/// Returns `(sin(a), cos(a))` in a single call.
///
/// Ports [`_sincos`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC. Upstream uses two out-pointers; the Rust form returns
/// a tuple. The call still goes through `libm::sinf` / `libm::cosf`
/// individually — FPU parts will common-subexpression-eliminate the
/// `a` argument; if a fused implementation becomes necessary later,
/// a separate `sincos` can be introduced without breaking this API.
#[inline]
pub fn sincos(a: f32) -> (f32, f32) {
    (libm::sinf(a), libm::cosf(a))
}

/// Two-argument arctangent, IEEE-754 correct via `libm::atan2f`.
///
/// Ports [`_atan2`](source/Arduino-FOC/src/common/foc_utils.h) from
/// Arduino-FOC. **Deliberate deviation:** the upstream uses a Padé-style
/// polynomial approximation; FOC-rs uses `libm::atan2f` for accuracy.
#[inline]
pub fn atan2(y: f32, x: f32) -> f32 {
    libm::atan2f(y, x)
}

/// Normalize `angle` (radians) to the half-open interval `[0, 2π)`.
///
/// Ports [`_normalizeAngle`](source/Arduino-FOC/src/common/foc_utils.h)
/// from Arduino-FOC. Implementation uses `libm::fmodf`, matching
/// upstream semantics: any remainder with the same sign as the dividend,
/// then shifted positive when negative.
///
/// Note: upstream normalises to `[0, 2π)`. Some other Arduino-FOC code
/// expects `[-π, π]`; FOC-rs deliberately matches upstream exactly
/// (`[0, 2π)`).
#[inline]
pub fn normalize_angle(angle: f32) -> f32 {
    let a = libm::fmodf(angle, _2PI);
    if a >= 0.0 {
        a
    } else {
        a + _2PI
    }
}

/// Convert a mechanical (shaft) angle to an electrical angle by
/// multiplying by the number of pole pairs.
///
/// Ports [`_electricalAngle`](source/Arduino-FOC/src/common/foc_utils.h)
/// from Arduino-FOC. `pole_pairs` is the number of magnetic pole pairs
/// (not poles) of the motor; for a 7-pole-pair motor, one mechanical
/// revolution equals 7 electrical revolutions.
#[inline]
pub fn electrical_angle(shaft_angle: f32, pole_pairs: i32) -> f32 {
    shaft_angle * (pole_pairs as f32)
}

/// IEEE-754 square root via `libm::sqrtf`.
///
/// Ports the upstream `_sqrt` macro (which expands to `_sqrtApprox`)
/// from Arduino-FOC. **Deliberate deviation:** the upstream uses the
/// Quake III fast-inverse-sqrt trick; FOC-rs uses `libm::sqrtf` for
/// accuracy.
#[inline]
pub fn sqrt(value: f32) -> f32 {
    libm::sqrtf(value)
}

// ---------------------------------------------------------------------------
// Clarke and Park transforms
// ---------------------------------------------------------------------------
//
// These are the two reference-frame projections that drive every FOC
// algorithm: Clarke converts the three measured phase currents
// (a, b, c) into the two-component stationary αβ frame; Park then
// rotates αβ by the rotor's electrical angle into the rotating DQ
// frame, in which the d- and q-axis currents become DC quantities
// that the current-control PIDs can regulate directly.
//
// The forms used here are the **amplitude-invariant** conventions
// (a unit-magnitude sinusoidal input produces a unit-magnitude DQ
// vector), which is the convention used by upstream Arduino-FOC's
// [`getABCurrents`](source/Arduino-FOC/src/common/base_classes/CurrentSense.cpp)
// and [`getDQCurrents`](source/Arduino-FOC/src/common/base_classes/CurrentSense.cpp).
//
// The amplitude-invariant Clarke transform for a three-wire system
// (a + b + c = 0) is:
//
//     α = a
//     β = (a + 2b) / √3
//
// The amplitude-invariant Park transform is:
//
//     d =  α·cos(θ) + β·sin(θ)
//     q =  β·cos(θ) − α·sin(θ)
//
// where `θ` is the rotor's electrical angle in radians.
//
// Upstream C++ also defines a Stepper variant of Clarke (α = a, β = b)
// and a Hybrid variant with several branches depending on which of
// the three currents is `0`. Those branches are handled by
// `CurrentSense::get_ab_currents` (default method on the trait),
// not by this generic `clarke` function — keep this transform
// narrow and let the trait layer add the special cases.

/// Clarke transform: three-phase (a, b, c) → stationary αβ frame.
///
/// Ports the BLDC 3-wire branch of
/// [`CurrentSense::getABCurrents`](source/Arduino-FOC/src/common/base_classes/CurrentSense.cpp)
/// from Arduino-FOC. The input is assumed to satisfy `a + b + c = 0`
/// (no zero-sequence component, the common case for star-connected
/// three-phase motors). For the Stepper / Hybrid special cases, see
/// `CurrentSense::get_ab_currents` on the trait layer.
///
/// The transform is amplitude-invariant:
///
/// ```text
/// α = a
/// β = (a + 2b) / √3
/// ```
///
/// Reference: see the BLDC branch of
/// `CurrentSense.cpp` lines 80–102 in the upstream source.
#[inline]
pub fn clarke(phase: PhaseCurrent) -> ABCurrent {
    let alpha = phase.a;
    let beta = (phase.a + 2.0 * phase.b) * _1_SQRT3;
    ABCurrent { alpha, beta }
}

/// Park transform: stationary αβ frame → rotating DQ frame.
///
/// Ports [`CurrentSense::getDQCurrents`](source/Arduino-FOC/src/common/base_classes/CurrentSense.cpp)
/// from Arduino-FOC. `angle_el` is the rotor's electrical angle
/// in radians. The transform is amplitude-invariant:
///
/// ```text
/// d =  α·cos(θ) + β·sin(θ)
/// q =  β·cos(θ) − α·sin(θ)
/// ```
///
/// Reference: see `CurrentSense.cpp` lines 113–122 in the upstream
/// source.
#[inline]
pub fn park(ab: ABCurrent, angle_el: f32) -> DQCurrent {
    let s = libm::sinf(angle_el);
    let c = libm::cosf(angle_el);
    let d = ab.alpha * c + ab.beta * s;
    let q = ab.beta * c - ab.alpha * s;
    DQCurrent { d, q }
}

// ---------------------------------------------------------------------------
// Unit tests — run on host via `cargo test`. No hardware, no timing.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------
    // Constants — every value must round-trip from the upstream literal
    // to within a comfortable f32 tolerance (f32 carries ~7 decimal
    // digits of precision, so 1e-5 is well above the noise floor).
    // -----------------------------------------------------------------
    fn assert_close(actual: f32, expected: f32, tol: f32) {
        assert!(
            (actual - expected).abs() <= tol,
            "expected {expected} ± {tol}, got {actual}",
        );
    }

    #[test]
    fn constants_match_upstream() {
        assert_close(_PI, core::f32::consts::PI, 1e-5);
        assert_close(_2PI, core::f32::consts::TAU, 1e-5);
        assert_close(_PI_2, core::f32::consts::FRAC_PI_2, 1e-5);
        assert_close(_3PI_2, 3.0 * core::f32::consts::FRAC_PI_2, 1e-5);
        assert_close(_PI_3, core::f32::consts::FRAC_PI_3, 1e-5);
        assert_close(_PI_6, core::f32::consts::FRAC_PI_6, 1e-5);
        assert_close(_120_D2R, 2.0 * core::f32::consts::FRAC_PI_3, 1e-5);
        assert_close(_SQRT3, 3.0_f32.sqrt(), 1e-5);
        assert_close(_SQRT3_2, (3.0_f32).sqrt() / 2.0, 1e-5);
        assert_close(_2_SQRT3, 2.0 / (3.0_f32).sqrt(), 1e-5);
        assert_close(_1_SQRT3, 1.0 / (3.0_f32).sqrt(), 1e-5);
        assert_close(_SQRT2, core::f32::consts::SQRT_2, 1e-5);
        assert_close(_RPM_TO_RADS, 2.0 * core::f32::consts::PI / 60.0, 1e-5);
        assert_eq!(NOT_SET, -12345.0_f32);
        assert_eq!(_HIGH_IMPEDANCE, 0);
        assert_eq!(_HIGH_Z, _HIGH_IMPEDANCE);
        assert_eq!(_ACTIVE, 1);
        assert_eq!(_NC, -12345_i32);
        // MIN_ANGLE_DETECT_MOVEMENT is _2PI / 101.0 — verify the
        // compile-time evaluation matches that expression.
        assert_close(MIN_ANGLE_DETECT_MOVEMENT, _2PI / 101.0, 0.0);
    }

    // -----------------------------------------------------------------
    // sin / cos / sincos
    // -----------------------------------------------------------------
    #[test]
    fn sin_at_known_angles() {
        assert_close(sin(0.0), 0.0, 1e-5);
        assert_close(sin(_PI / 2.0), 1.0, 1e-5);
        assert_close(sin(_PI), 0.0, 1e-5);
        assert_close(sin(3.0 * _PI / 2.0), -1.0, 1e-5);
        assert_close(sin(-_PI / 2.0), -1.0, 1e-5);
    }

    #[test]
    fn cos_at_known_angles() {
        assert_close(cos(0.0), 1.0, 1e-5);
        assert_close(cos(_PI / 2.0), 0.0, 1e-5);
        assert_close(cos(_PI), -1.0, 1e-5);
        assert_close(cos(3.0 * _PI / 2.0), 0.0, 1e-5);
    }

    #[test]
    fn sincos_returns_matching_values() {
        let a = 1.2345_f32;
        let (s, c) = sincos(a);
        assert_close(s, sin(a), 0.0);
        assert_close(c, cos(a), 0.0);
        // Spot-check the values too.
        assert_close(s, libm::sinf(a), 0.0);
        assert_close(c, libm::cosf(a), 0.0);
    }

    // -----------------------------------------------------------------
    // atan2
    // -----------------------------------------------------------------
    #[test]
    fn atan2_axis_aligned() {
        assert_close(atan2(0.0, 1.0), 0.0, 1e-5);
        assert_close(atan2(1.0, 0.0), _PI / 2.0, 1e-5);
        assert_close(atan2(0.0, -1.0), _PI, 1e-5);
        assert_close(atan2(-1.0, 0.0), -_PI / 2.0, 1e-5);
    }

    #[test]
    fn atan2_quadrants() {
        // (1, 1) → π/4
        assert_close(atan2(1.0, 1.0), _PI / 4.0, 1e-5);
        // (-1, 1) → 3π/4
        assert_close(atan2(1.0, -1.0), 3.0 * _PI / 4.0, 1e-5);
    }

    // -----------------------------------------------------------------
    // normalize_angle — must map into [0, 2π).
    // -----------------------------------------------------------------
    #[test]
    fn normalize_angle_in_range() {
        assert_close(normalize_angle(0.0), 0.0, 0.0);
        assert_close(normalize_angle(_PI), _PI, 1e-5);
        assert_close(normalize_angle(_2PI), 0.0, 1e-5);
        assert_close(normalize_angle(-0.1), _2PI - 0.1, 1e-5);
        assert_close(normalize_angle(3.0 * _PI), _PI, 1e-5);
        // Anything that goes through fmod + positive-shift should land
        // in [0, 2π). Small multiples keep the f32 ULP tight enough
        // that the 0.42 fractional part survives (ULP at ~6 is ~4e-7).
        let one_full = _2PI + 0.42;
        assert_close(normalize_angle(one_full), 0.42, 1e-5);
        let neg_one_full = -_2PI - 0.42;
        assert_close(normalize_angle(neg_one_full), _2PI - 0.42, 1e-5);
    }

    // -----------------------------------------------------------------
    // electrical_angle
    // -----------------------------------------------------------------
    #[test]
    fn electrical_angle_scales_by_pole_pairs() {
        assert_close(electrical_angle(0.0, 7), 0.0, 0.0);
        assert_close(electrical_angle(0.25, 7), 7.0 * 0.25, 1e-5);
        assert_close(electrical_angle(_PI, 1), _PI, 0.0);
        assert_close(electrical_angle(_PI, 4), 4.0 * _PI, 1e-5);
    }

    // -----------------------------------------------------------------
    // sqrt
    // -----------------------------------------------------------------
    #[test]
    fn sqrt_known_values() {
        assert_close(sqrt(0.0), 0.0, 0.0);
        assert_close(sqrt(1.0), 1.0, 0.0);
        assert_close(sqrt(4.0), 2.0, 0.0);
        assert_close(sqrt(9.0), 3.0, 0.0);
        assert_close(sqrt(2.0), core::f32::consts::SQRT_2, 1e-5);
        assert_close(sqrt(3.0), 3.0_f32.sqrt(), 0.0);
    }

    // -----------------------------------------------------------------
    // sign — per project spec: -1 for a < 0, 1 for a >= 0.
    // -----------------------------------------------------------------
    #[test]
    fn sign_known_values() {
        assert_eq!(sign(-2.0), -1);
        assert_eq!(sign(-0.0001), -1);
        assert_eq!(sign(0.0), 1);
        assert_eq!(sign(3.0), 1);
    }

    // -----------------------------------------------------------------
    // round — half-away-from-zero semantics, matches upstream macro.
    // -----------------------------------------------------------------
    #[test]
    fn round_known_values() {
        assert_eq!(round(0.6), 1);
        assert_eq!(round(-0.6), -1);
        assert_eq!(round(2.5), 3);
        assert_eq!(round(-2.5), -3);
        assert_eq!(round(0.4), 0);
        assert_eq!(round(0.0), 0);
    }

    // -----------------------------------------------------------------
    // constrain
    // -----------------------------------------------------------------
    #[test]
    fn constrain_clamps() {
        assert_close(constrain(5.0, 0.0, 10.0), 5.0, 0.0);
        assert_close(constrain(-1.0, 0.0, 10.0), 0.0, 0.0);
        assert_close(constrain(11.0, 0.0, 10.0), 10.0, 0.0);
        assert_close(constrain(0.0, 0.0, 10.0), 0.0, 0.0);
        assert_close(constrain(10.0, 0.0, 10.0), 10.0, 0.0);
    }

    // -----------------------------------------------------------------
    // isset
    // -----------------------------------------------------------------
    #[test]
    fn isset_detects_not_set() {
        assert!(!isset(NOT_SET));
        assert!(isset(0.0));
        assert!(isset(-1.0));
        assert!(isset(1.0));
    }

    // -----------------------------------------------------------------
    // Type aliases — DQ must be interchangeable with DQCurrent and
    // DQVoltage (transparent aliases).
    // -----------------------------------------------------------------
    #[test]
    fn dq_aliases_round_trip() {
        let dq = DQ { d: 1.0, q: 2.0 };
        let as_voltage: DQVoltage = dq;
        let as_current: DQCurrent = as_voltage;
        let back: DQ = as_current;
        assert_eq!(back.d, 1.0);
        assert_eq!(back.q, 2.0);
        // DQVoltage and DQCurrent are the same type — assignment both
        // ways must compile.
        let v: DQVoltage = DQCurrent { d: 3.0, q: 4.0 };
        let c: DQCurrent = v;
        assert_eq!(c.d, 3.0);
        assert_eq!(c.q, 4.0);
    }

    #[test]
    fn ab_aliases_round_trip() {
        let ab = AB {
            alpha: 0.5,
            beta: -0.25,
        };
        let as_voltage: ABVoltage = ab;
        let as_current: ABCurrent = as_voltage;
        let back: AB = as_current;
        assert_eq!(back.alpha, 0.5);
        assert_eq!(back.beta, -0.25);
    }

    #[test]
    fn phase_aliases_round_trip() {
        let p = Phase {
            a: 1.0,
            b: 2.0,
            c: 3.0,
        };
        let as_voltage: PhaseVoltage = p;
        let as_current: PhaseCurrent = as_voltage;
        let back: Phase = as_current;
        assert_eq!(back.a, 1.0);
        assert_eq!(back.b, 2.0);
        assert_eq!(back.c, 3.0);
    }

    // -----------------------------------------------------------------
    // Frame-vector derives — make sure Copy/Default/PartialEq behave
    // as expected; this is a compile-time check, but also catches
    // accidental field changes.
    // -----------------------------------------------------------------
    #[test]
    fn frame_vectors_default_and_equality() {
        assert_eq!(DQ::default(), DQ { d: 0.0, q: 0.0 });
        assert_eq!(
            AB::default(),
            AB {
                alpha: 0.0,
                beta: 0.0
            }
        );
        assert_eq!(
            Phase::default(),
            Phase {
                a: 0.0,
                b: 0.0,
                c: 0.0
            }
        );

        // Copy semantics: assigning doesn't move.
        let dq = DQ { d: 1.0, q: 2.0 };
        let _copy = dq;
        let _copy2 = dq;
    }

    // -----------------------------------------------------------------
    // Clarke transform — amplitude-invariant, three-wire
    // (a + b + c = 0). Hand-computed expected values.
    // -----------------------------------------------------------------
    #[test]
    fn clarke_zero_input() {
        let p = PhaseCurrent { a: 0.0, b: 0.0, c: 0.0 };
        let ab = clarke(p);
        assert_close(ab.alpha, 0.0, 0.0);
        assert_close(ab.beta, 0.0, 0.0);
    }

    #[test]
    fn clarke_pure_a_axis() {
        // a=1, b=0, c=-1 (a+b+c=0 satisfied). Expected:
        //   α = a = 1
        //   β = (a + 2b)/√3 = 1/√3
        let p = PhaseCurrent { a: 1.0, b: 0.0, c: -1.0 };
        let ab = clarke(p);
        assert_close(ab.alpha, 1.0, 1e-6);
        assert_close(ab.beta, _1_SQRT3, 1e-5);
    }

    #[test]
    fn clarke_pure_b_axis() {
        // a=-1/2, b=1, c=-1/2 (unit, 90° from a-axis). Expected:
        //   α = -1/2
        //   β = (-1/2 + 2·1)/√3 = (3/2)/√3 = √3/2
        let p = PhaseCurrent {
            a: -0.5,
            b: 1.0,
            c: -0.5,
        };
        let ab = clarke(p);
        assert_close(ab.alpha, -0.5, 1e-6);
        assert_close(ab.beta, _SQRT3_2, 1e-5);
    }

    #[test]
    fn clarke_rotated_60_degrees() {
        // 60° unit phasor: a = cos(60°)=0.5, b = cos(60°-120°)=cos(-60°)=0.5,
        // c = cos(60°+120°) = cos(180°) = -1.  Expected:
        //   α = 0.5
        //   β = (0.5 + 2·0.5)/√3 = 1.5/√3
        let p = PhaseCurrent {
            a: 0.5,
            b: 0.5,
            c: -1.0,
        };
        let ab = clarke(p);
        assert_close(ab.alpha, 0.5, 1e-6);
        assert_close(ab.beta, 1.5 * _1_SQRT3, 1e-5);
    }

    #[test]
    fn clarke_amplitude_invariant_unit_circle() {
        // For a balanced three-phase unit-amplitude input, the
        // resulting αβ vector must have unit magnitude. Sweep the
        // mechanical angle over [0, 2π) and check |αβ| == 1.
        let n = 64;
        for i in 0..n {
            let theta = _2PI * (i as f32) / (n as f32);
            let a = libm::cosf(theta);
            let b = libm::cosf(theta - _120_D2R);
            let c = libm::cosf(theta + _120_D2R);
            let p = PhaseCurrent { a, b, c };
            let ab = clarke(p);
            let mag = sqrt(ab.alpha * ab.alpha + ab.beta * ab.beta);
            assert_close(mag, 1.0, 1e-4);
        }
    }

    // -----------------------------------------------------------------
    // Park transform — amplitude-invariant. Hand-computed expected
    // values plus a round-trip test through Clarke.
    // -----------------------------------------------------------------
    #[test]
    fn park_zero_angle() {
        // θ=0 → identity rotation. d = α, q = β.
        let ab = ABCurrent {
            alpha: 0.3,
            beta: -0.7,
        };
        let dq = park(ab, 0.0);
        assert_close(dq.d, ab.alpha, 1e-5);
        assert_close(dq.q, ab.beta, 1e-5);
    }

    #[test]
    fn park_quarter_turn() {
        // θ = π/2 → rotates αβ by +90°: d = -q_in, q = d_in? Let me
        // recompute: at θ=π/2, sin=1, cos=0, so
        //   d =  α·0 + β·1 = β
        //   q =  β·0 − α·1 = −α
        // So the +β axis maps to the d-axis. That's the standard
        // FOC convention.
        let ab = ABCurrent {
            alpha: 0.3,
            beta: -0.7,
        };
        let dq = park(ab, _PI_2);
        assert_close(dq.d, ab.beta, 1e-5);
        assert_close(dq.q, -ab.alpha, 1e-5);
    }

    #[test]
    fn park_round_trip_via_clarke() {
        // Start with a balanced three-phase unit-amplitude input
        // rotating at mechanical angle θ. The Clarke transform
        // produces a stationary αβ vector of unit magnitude at
        // angle θ. A Park transform at angle θ must collapse it
        // to (d=cos(0)=1, q=0). The d-axis is at the chosen
        // reference angle; whatever mechanical phase we used
        // cancels out.
        let n = 32;
        for i in 0..n {
            let theta = _2PI * (i as f32) / (n as f32);
            let a = libm::cosf(theta);
            let b = libm::cosf(theta - _120_D2R);
            let c = libm::cosf(theta + _120_D2R);
            let p = PhaseCurrent { a, b, c };
            let ab = clarke(p);
            let dq = park(ab, theta);
            // After Clarke + Park at the same angle, the result
            // should be a unit vector on the d-axis (q ≈ 0,
            // d ≈ 1).
            assert_close(dq.d, 1.0, 1e-4);
            assert_close(dq.q, 0.0, 1e-4);
        }
    }

    #[test]
    fn park_is_orthogonal() {
        // For any αβ of magnitude R, Park produces a DQ vector of
        // the same magnitude R (orthogonal rotation preserves
        // length).
        let ab = ABCurrent {
            alpha: 1.2,
            beta: -0.4,
        };
        let r = sqrt(ab.alpha * ab.alpha + ab.beta * ab.beta);
        // Try several angles.
        for &theta in &[0.0_f32, 0.3, _PI_2, 1.7, _2PI - 0.1] {
            let dq = park(ab, theta);
            let r_after = sqrt(dq.d * dq.d + dq.q * dq.q);
            assert_close(r_after, r, 1e-5);
        }
    }
}
