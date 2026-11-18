//! # defaults — FOC engine default constants
//!
//! All `DEF_*` constants used to seed the FOC engine when a user has not
//! supplied their own value. Ports
//! [`defaults.h`](source/Arduino-FOC/src/common/defaults.h) from
//! Arduino-FOC.
//!
//! ## Deliberate deviation from upstream
//!
//! `defaults.h` lines 13–31 guard a block of current-sensing defaults
//! behind `#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//! || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)`.
//! That AVR-only branch ships weaker PID gains (`P = 2`, `I = 100`)
//! suitable for 16 MHz Arduino-class parts. FOC-rs targets Cortex-M /
//! ESP32 (per `AGENTS.md`), so the constants below are taken from the
//! **`#else` branch** verbatim (`P = 3`, `I = 300`, current bandwidth
//! `300 Hz`). If a future port wants to add AVR support, expose both
//! branches behind a Cargo feature flag.
//!
//! ## Type-fidelity caveat
//!
//! Upstream `defaults.h` is sloppy about `f` suffixes. The PID gains
//! for the velocity loop are written with `f` (`DEF_PID_VEL_P 0.5f`),
//! but the current-loop `P` and `RAMP` and the downsample counters
//! (`DEF_PID_CURR_P 3`, `DEF_PID_CURR_RAMP 0`, `DEF_MON_DOWNSMAPLE 100`)
//! are written as bare integer literals. To stay 1:1 with the upstream
//! source, FOC-rs preserves those types: `i32` for the int-shaped
//! constants and `f32` for the float-shaped ones. Consumers that pass
//! these into a `f32`-typed PID constructor will need an `as f32` cast
//! — that matches the upstream behaviour, where the implicit C
//! conversion would do the same.
//!
//! `DEF_PID_VEL_RAMP` uses the [`math::NOT_SET`](crate::math::NOT_SET)
//! sentinel to mean "no output-ramp limit configured"; callers detect
//! that with [`math::isset`](crate::math::isset).

#![allow(clippy::approx_constant, clippy::excessive_precision)]
// Upstream uses mixed-case suffixes (`DEF_CURR_FILTER_Tf`, `DEF_VEL_FILTER_Tf`,
// `DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf`) where the `f` of `Tf` is lowercase.
// Preserving the 1:1 upstream spelling is more useful for cross-referencing
// than submitting to Rust's `non_upper_case_globals` convention.
#![allow(non_upper_case_globals)]

use crate::math::{NOT_SET, _2PI};

/// Default power-supply voltage (volts) used as the upper bound for
/// voltage-mode PID limits.
///
/// Ports [`DEF_POWER_SUPPLY`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_POWER_SUPPLY: f32 = 12.0_f32;

// ---------------------------------------------------------------------------
// Velocity-loop PI defaults
// ---------------------------------------------------------------------------

/// Default velocity-loop PID proportional gain.
///
/// Ports [`DEF_PID_VEL_P`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_PID_VEL_P: f32 = 0.5_f32;

/// Default velocity-loop PID integral gain.
///
/// Ports [`DEF_PID_VEL_I`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_PID_VEL_I: f32 = 10.0_f32;

/// Default velocity-loop PID derivative gain.
///
/// Ports [`DEF_PID_VEL_D`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_PID_VEL_D: f32 = 0.0_f32;

/// Default velocity-loop output-ramp limit. The sentinel
/// [`NOT_SET`](crate::math::NOT_SET) means "no ramp configured".
///
/// Ports [`DEF_PID_VEL_RAMP`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_PID_VEL_RAMP: f32 = NOT_SET;

/// Default velocity-loop output limit. Set to
/// [`DEF_POWER_SUPPLY`].
///
/// Ports [`DEF_PID_VEL_LIMIT`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_PID_VEL_LIMIT: f32 = DEF_POWER_SUPPLY;

// ---------------------------------------------------------------------------
// Current-loop PID defaults — #else branch (stm32/due/teensy/esp32)
// ---------------------------------------------------------------------------

/// Default current-loop PID proportional gain. **Type is `i32`** to
/// match the upstream literal (bare `3`, no `f` suffix). See the module
/// docstring for why.
///
/// Ports [`DEF_PID_CURR_P`](source/Arduino-FOC/src/common/defaults.h)
/// (`#else` branch) from Arduino-FOC.
pub const DEF_PID_CURR_P: i32 = 3;

/// Default current-loop PID integral gain.
///
/// Ports [`DEF_PID_CURR_I`](source/Arduino-FOC/src/common/defaults.h)
/// (`#else` branch) from Arduino-FOC.
pub const DEF_PID_CURR_I: f32 = 300.0_f32;

/// Default current-loop PID derivative gain.
///
/// Ports [`DEF_PID_CURR_D`](source/Arduino-FOC/src/common/defaults.h)
/// (`#else` branch) from Arduino-FOC.
pub const DEF_PID_CURR_D: f32 = 0.0_f32;

/// Default current-loop output-ramp limit. **Type is `i32`** to match
/// the upstream literal (bare `0`, no `f` suffix). See the module
/// docstring for why.
///
/// Ports [`DEF_PID_CURR_RAMP`](source/Arduino-FOC/src/common/defaults.h)
/// (`#else` branch) from Arduino-FOC.
pub const DEF_PID_CURR_RAMP: i32 = 0;

/// Default current-loop output limit. Set to
/// [`DEF_POWER_SUPPLY`].
///
/// Ports [`DEF_PID_CURR_LIMIT`](source/Arduino-FOC/src/common/defaults.h)
/// (`#else` branch) from Arduino-FOC.
pub const DEF_PID_CURR_LIMIT: f32 = DEF_POWER_SUPPLY;

/// Current-loop target bandwidth (Hz) used to derive
/// [`DEF_CURR_FILTER_Tf`].
///
/// Ports [`DEF_CURR_BANDWIDTH`](source/Arduino-FOC/src/common/defaults.h)
/// (`#else` branch) from Arduino-FOC.
pub const DEF_CURR_BANDWIDTH: f32 = 300.0_f32;

/// Current-loop low-pass-filter time constant (seconds). Computed from
/// the target bandwidth as `1 / (2π · DEF_CURR_BANDWIDTH)` to match the
/// upstream `1/(_2PI*DEF_CURR_BANDWIDTH)` expression.
///
/// Ports [`DEF_CURR_FILTER_Tf`](source/Arduino-FOC/src/common/defaults.h)
/// (`#else` branch) from Arduino-FOC.
pub const DEF_CURR_FILTER_Tf: f32 = 1.0_f32 / (_2PI * DEF_CURR_BANDWIDTH);

// ---------------------------------------------------------------------------
// Current limit
// ---------------------------------------------------------------------------

/// Default per-motor current limit (amperes).
///
/// Ports [`DEF_CURRENT_LIM`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_CURRENT_LIM: f32 = 2.0_f32;

// ---------------------------------------------------------------------------
// Monitor / motion downsampling
// ---------------------------------------------------------------------------

/// Default monitor loop decimation factor. **Type is `i32`** to match
/// the upstream literal (bare `100`).
///
/// Ports [`DEF_MON_DOWNSMAPLE`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_MON_DOWNSMAPLE: i32 = 100;

/// Default motion-loop decimation factor. `0` disables motion-loop
/// downsampling. **Type is `i32`** to match the upstream literal (bare
/// `0`).
///
/// Ports [`DEF_MOTION_DOWNSMAPLE`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_MOTION_DOWNSMAPLE: i32 = 0;

// ---------------------------------------------------------------------------
// Angle-loop P defaults
// ---------------------------------------------------------------------------

/// Default angle-loop P-controller gain.
///
/// Ports [`DEF_P_ANGLE_P`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_P_ANGLE_P: f32 = 20.0_f32;

/// Default velocity limit (rad/s) used by the angle-loop clamp.
///
/// Ports [`DEF_VEL_LIM`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_VEL_LIM: f32 = 20.0_f32;

// ---------------------------------------------------------------------------
// Index search / sensor alignment
// ---------------------------------------------------------------------------

/// Default velocity (rad/s) used during the encoder-index search.
///
/// Ports [`DEF_INDEX_SEARCH_TARGET_VELOCITY`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_INDEX_SEARCH_TARGET_VELOCITY: f32 = 1.0_f32;

/// Default voltage (volts) applied during sensor and motor alignment.
///
/// Ports [`DEF_VOLTAGE_SENSOR_ALIGN`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_VOLTAGE_SENSOR_ALIGN: f32 = 3.0_f32;

// ---------------------------------------------------------------------------
// Velocity / current-sense low-pass filter defaults
// ---------------------------------------------------------------------------

/// Default velocity low-pass-filter time constant (seconds).
///
/// Ports [`DEF_VEL_FILTER_Tf`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_VEL_FILTER_Tf: f32 = 0.005_f32;

/// Default per-phase current-sense low-pass-filter time constant
/// (seconds). `0.0` disables the filter.
///
/// Ports [`DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf`](source/Arduino-FOC/src/common/defaults.h)
/// from Arduino-FOC.
pub const DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf: f32 = 0.0_f32;

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// All values below are within a comfortable f32 tolerance of the
    /// literal printed in the source — they are exact bits-level
    /// representations of the upstream C `#define`s, so the only
    /// rounding that can occur is in the `f32` cast itself.
    fn assert_close(actual: f32, expected: f32, tol: f32) {
        assert!(
            (actual - expected).abs() <= tol,
            "expected {expected} ± {tol}, got {actual}",
        );
    }

    #[test]
    fn power_supply_default() {
        assert_close(DEF_POWER_SUPPLY, 12.0_f32, 0.0);
    }

    #[test]
    fn velocity_pid_defaults() {
        assert_close(DEF_PID_VEL_P, 0.5_f32, 0.0);
        assert_close(DEF_PID_VEL_I, 10.0_f32, 0.0);
        assert_close(DEF_PID_VEL_D, 0.0_f32, 0.0);
        // RAMP is the NOT_SET sentinel — verify the bit pattern, not
        // the conceptual value.
        assert_eq!(DEF_PID_VEL_RAMP.to_bits(), NOT_SET.to_bits());
        // LIMIT aliases POWER_SUPPLY.
        assert_eq!(DEF_PID_VEL_LIMIT.to_bits(), DEF_POWER_SUPPLY.to_bits());
    }

    #[test]
    fn current_pid_defaults_use_else_branch() {
        // The #else branch is the stm32/esp32 branch. Verify we took
        // it (not the AVR branch).
        assert_eq!(DEF_PID_CURR_P, 3_i32);
        assert_ne!(DEF_PID_CURR_P, 2_i32, "AVR branch was selected — deviation docs are wrong");
        assert_close(DEF_PID_CURR_I, 300.0_f32, 0.0);
        assert_close(DEF_PID_CURR_D, 0.0_f32, 0.0);
        assert_eq!(DEF_PID_CURR_RAMP, 0_i32);
        assert_eq!(DEF_PID_CURR_LIMIT.to_bits(), DEF_POWER_SUPPLY.to_bits());
    }

    #[test]
    fn current_bandwidth_and_filter_derived() {
        assert_close(DEF_CURR_BANDWIDTH, 300.0_f32, 0.0);
        // DEF_CURR_FILTER_Tf = 1 / (2π · 300). Verify the derived
        // expression matches the constant (compile-time evaluated).
        let manual = 1.0_f32 / (_2PI * DEF_CURR_BANDWIDTH);
        assert_eq!(DEF_CURR_FILTER_Tf.to_bits(), manual.to_bits());
    }

    #[test]
    fn current_limit_default() {
        assert_close(DEF_CURRENT_LIM, 2.0_f32, 0.0);
    }

    #[test]
    fn monitor_downsamples() {
        assert_eq!(DEF_MON_DOWNSMAPLE, 100_i32);
        assert_eq!(DEF_MOTION_DOWNSMAPLE, 0_i32);
    }

    #[test]
    fn angle_loop_defaults() {
        assert_close(DEF_P_ANGLE_P, 20.0_f32, 0.0);
        assert_close(DEF_VEL_LIM, 20.0_f32, 0.0);
    }

    #[test]
    fn alignment_defaults() {
        assert_close(DEF_INDEX_SEARCH_TARGET_VELOCITY, 1.0_f32, 0.0);
        assert_close(DEF_VOLTAGE_SENSOR_ALIGN, 3.0_f32, 0.0);
    }

    #[test]
    fn velocity_and_current_sense_lpf_defaults() {
        assert_close(DEF_VEL_FILTER_Tf, 0.005_f32, 0.0);
        assert_close(DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf, 0.0_f32, 0.0);
    }
}
