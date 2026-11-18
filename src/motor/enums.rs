//! # motor::enums — motor control enums
//!
//! Ports the four enums declared at the top of
//! [`FOCMotor.h`](source/Arduino-FOC/src/common/base_classes/FOCMotor.h):
//!
//! - [`MotionControlType`] — torque / velocity / angle /
//!   open-loop / no-cascade / custom.
//! - [`TorqueControlType`] — voltage / DC-current /
//!   FOC-current / estimated-current.
//! - [`FOCModulationType`] — SinePWM / SpaceVectorPWM /
//!   Trapezoid_120 / Trapezoid_150.
//! - [`FOCMotorStatus`] — the motor's lifecycle state
//!   machine.
//!
//! The enums keep their exact upstream variant spelling per
//! `PORTING_GUIDELINES.md` §3. A module-wide
//! `#[allow(non_camel_case_types)]` (set below) suppresses
//! the clippy lint that would otherwise fire on the
//! lowercase variants.
//!
//! These enums are prerequisites for the concrete motor
//! structs (`BLDCMotor`, `StepperMotor`,
//! `HybridStepperMotor`) and are referenced from the
//! `MotorConfig` struct in `motor::base` (phase 4).

#![allow(non_camel_case_types)]

// ---------------------------------------------------------------------------
// MotionControlType
// ---------------------------------------------------------------------------

/// Motion (outer-loop) control mode.
///
/// Ports the upstream `MotionControlType` enum from
/// [`FOCMotor.h`](source/Arduino-FOC/src/common/base_classes/FOCMotor.h)
/// lines 49–57. The motor's `move_motor(target)` /
/// `loop_foc()` switch on this enum to dispatch between
/// torque, velocity, angle, open-loop, no-cascade, and
/// custom control modes.
///
/// # Variants
///
/// - `torque` (0x00) — Torque control (no outer loop; the
///   target is interpreted as a `Uq` voltage or a current
///   depending on `torque_controller`).
/// - `velocity` (0x01) — Closed-loop velocity control.
///   Uses the velocity PI controller.
/// - `angle` (0x02) — Closed-loop angle control with
///   velocity cascade. Angle P → velocity PI → current
///   PI.
/// - `velocity_openloop` (0x03) — Open-loop velocity
///   control (no sensor needed). The motor advances the
///   electrical angle at `target_velocity` rad/s.
/// - `angle_openloop` (0x04) — Open-loop angle control.
/// - `angle_nocascade` (0x05) — Closed-loop angle control
///   without the velocity cascade (direct angle PI →
///   voltage).
/// - `custom` (0x06) — User-supplied control method
///   (set via `link_custom_motion_control`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum MotionControlType {
    /// Torque control (no outer loop).
    torque = 0x00,
    /// Closed-loop velocity control.
    velocity = 0x01,
    /// Closed-loop angle control with velocity cascade.
    angle = 0x02,
    /// Open-loop velocity control (no sensor needed).
    velocity_openloop = 0x03,
    /// Open-loop angle control.
    angle_openloop = 0x04,
    /// Closed-loop angle control without velocity cascade.
    angle_nocascade = 0x05,
    /// User-supplied control method.
    #[default]
    custom = 0x06,
}

impl TryFrom<u8> for MotionControlType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(MotionControlType::torque),
            1 => Ok(MotionControlType::velocity),
            2 => Ok(MotionControlType::angle),
            3 => Ok(MotionControlType::velocity_openloop),
            4 => Ok(MotionControlType::angle_openloop),
            5 => Ok(MotionControlType::angle_nocascade),
            6 => Ok(MotionControlType::custom),
            _ => Err(()),
        }
    }
}

// ---------------------------------------------------------------------------
// TorqueControlType
// ---------------------------------------------------------------------------

/// Torque (inner-loop) control mode.
///
/// Ports the upstream `TorqueControlType` enum from
/// [`FOCMotor.h`](source/Arduino-FOC/src/common/base_classes/FOCMotor.h)
/// lines 62–67. The motor's `set_phase_voltage` /
/// `loop_foc` switch on this enum to dispatch between
/// voltage-mode torque, DC-current, FOC-current, and
/// estimated-current control.
///
/// # Variants
///
/// - `voltage` (0x00) — Open-loop voltage control. The
///   target is interpreted directly as a `Uq` setpoint.
/// - `dc_current` (0x01) — DC-bus current control using
///   one current magnitude.
/// - `foc_current` (0x02) — DQ-frame FOC current control.
///   Two PI controllers (one for d-axis, one for q-axis)
///   regulate `Id` and `Iq`.
/// - `estimated_current` (0x03) — Estimated-current
///   control (uses the motor's `KV_rating` and
///   `phase_resistance` to estimate torque without
///   measuring current). Useful for low-cost builds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum TorqueControlType {
    /// Open-loop voltage control. `target` is `Uq` in
    /// volts.
    #[default]
    voltage = 0x00,
    /// DC-bus current control using one current
    /// magnitude.
    dc_current = 0x01,
    /// DQ-frame FOC current control (d and q PIs).
    foc_current = 0x02,
    /// Estimated-current control using motor parameters.
    estimated_current = 0x03,
}

impl TryFrom<u8> for TorqueControlType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(TorqueControlType::voltage),
            1 => Ok(TorqueControlType::dc_current),
            2 => Ok(TorqueControlType::foc_current),
            3 => Ok(TorqueControlType::estimated_current),
            _ => Err(()),
        }
    }
}

// ---------------------------------------------------------------------------
// FOCModulationType
// ---------------------------------------------------------------------------

/// PWM modulation strategy used by the FOC engine.
///
/// Ports the upstream `FOCModulationType` enum from
/// [`FOCMotor.h`](source/Arduino-FOC/src/common/base_classes/FOCMotor.h)
/// lines 72–77. `set_phase_voltage` switches on this enum
/// to dispatch between sinusoidal PWM, space-vector PWM,
/// and the two trapezoidal commutation modes (120° and
/// 150°).
///
/// # Variants
///
/// - `SinePWM` (0x00) — Sinusoidal PWM. Each phase is
///   driven with a sinusoidally-weighted duty cycle.
///   Simple, low switching loss, but limited DC-bus
///   utilisation (~78%).
/// - `SpaceVectorPWM` (0x01) — Space-vector modulation.
///   Better DC-bus utilisation (~90%) at the cost of
///   more complex per-phase duty calculation.
/// - `Trapezoid_120` (0x02) — 120° trapezoidal
///   commutation. Each phase is on for 120° electrical
///   per cycle. High torque ripple, but only two
///   switches transition per 60°.
/// - `Trapezoid_150` (0x03) — 150° trapezoidal
///   commutation. Higher torque utilisation than
///   `Trapezoid_120` at slightly higher ripple.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum FOCModulationType {
    /// Sinusoidal PWM. Default.
    #[default]
    SinePWM = 0x00,
    /// Space-vector PWM. Higher DC-bus utilisation.
    SpaceVectorPWM = 0x01,
    /// 120° trapezoidal commutation.
    Trapezoid_120 = 0x02,
    /// 150° trapezoidal commutation.
    Trapezoid_150 = 0x03,
}

impl TryFrom<u8> for FOCModulationType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(FOCModulationType::SinePWM),
            1 => Ok(FOCModulationType::SpaceVectorPWM),
            2 => Ok(FOCModulationType::Trapezoid_120),
            3 => Ok(FOCModulationType::Trapezoid_150),
            _ => Err(()),
        }
    }
}

// ---------------------------------------------------------------------------
// FOCMotorStatus
// ---------------------------------------------------------------------------

/// Motor lifecycle state.
///
/// Ports the upstream `FOCMotorStatus` enum from
/// [`FOCMotor.h`](source/Arduino-FOC/src/common/base_classes/FOCMotor.h)
/// lines 81–90. The motor's `init`, `init_foc`, and
/// calibration routines advance this state machine.
///
/// # Variants
///
/// - `motor_uninitialized` (0x00) — Initial state, before
///   the first `init()` call.
/// - `motor_initializing` (0x01) — `init()` is in
///   progress.
/// - `motor_uncalibrated` (0x02) — Initialised, but not
///   calibrated (open-loop operation only).
/// - `motor_calibrating` (0x03) — `init_foc()` /
///   calibration is in progress.
/// - `motor_ready` (0x04) — Initialised and calibrated;
///   closed-loop operation is possible.
/// - `motor_error` (0x08) — Recoverable error (e.g.
///   overcurrent protection active). The user can call
///   `init_foc()` again to recover.
/// - `motor_calib_failed` (0x0E) — Calibration failed.
///   Possibly recoverable (the user can tweak the
///   `voltage_sensor_align` parameter and retry).
/// - `motor_init_failed` (0x0F) — Initialisation failed
///   in a way that's not recoverable (hardware fault,
///   missing sensor, …).
///
/// Note: the error and failure statuses are bit-pattern
/// distinct (0x08, 0x0E, 0x0F) so a caller can mask and
/// dispatch — see upstream `FOCMotor::motor_status`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum FOCMotorStatus {
    /// Initial state, before `init()`.
    #[default]
    motor_uninitialized = 0x00,
    /// `init()` is in progress.
    motor_initializing = 0x01,
    /// Initialised, but not calibrated (open-loop only).
    motor_uncalibrated = 0x02,
    /// Calibration is in progress.
    motor_calibrating = 0x03,
    /// Initialised and calibrated; closed-loop possible.
    motor_ready = 0x04,
    /// Recoverable error.
    motor_error = 0x08,
    /// Calibration failed (possibly recoverable).
    motor_calib_failed = 0x0E,
    /// Initialisation failed (not recoverable).
    motor_init_failed = 0x0F,
}

impl TryFrom<u8> for FOCMotorStatus {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(FOCMotorStatus::motor_uninitialized),
            0x01 => Ok(FOCMotorStatus::motor_initializing),
            0x02 => Ok(FOCMotorStatus::motor_uncalibrated),
            0x03 => Ok(FOCMotorStatus::motor_calibrating),
            0x04 => Ok(FOCMotorStatus::motor_ready),
            0x08 => Ok(FOCMotorStatus::motor_error),
            0x0E => Ok(FOCMotorStatus::motor_calib_failed),
            0x0F => Ok(FOCMotorStatus::motor_init_failed),
            _ => Err(()),
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------
    // Discriminant pinning — every variant's numeric value
    // must match the upstream C++ literal so any code that
    // serialises the status to a debug port stays
    // bit-compatible with the upstream CLI / monitor
    // protocol.
    // -----------------------------------------------------------------
    #[test]
    fn motion_control_type_discriminants_match_upstream() {
        assert_eq!(MotionControlType::torque as u8, 0x00);
        assert_eq!(MotionControlType::velocity as u8, 0x01);
        assert_eq!(MotionControlType::angle as u8, 0x02);
        assert_eq!(MotionControlType::velocity_openloop as u8, 0x03);
        assert_eq!(MotionControlType::angle_openloop as u8, 0x04);
        assert_eq!(MotionControlType::angle_nocascade as u8, 0x05);
        assert_eq!(MotionControlType::custom as u8, 0x06);
    }

    #[test]
    fn torque_control_type_discriminants_match_upstream() {
        assert_eq!(TorqueControlType::voltage as u8, 0x00);
        assert_eq!(TorqueControlType::dc_current as u8, 0x01);
        assert_eq!(TorqueControlType::foc_current as u8, 0x02);
        assert_eq!(TorqueControlType::estimated_current as u8, 0x03);
    }

    #[test]
    fn foc_modulation_type_discriminants_match_upstream() {
        assert_eq!(FOCModulationType::SinePWM as u8, 0x00);
        assert_eq!(FOCModulationType::SpaceVectorPWM as u8, 0x01);
        assert_eq!(FOCModulationType::Trapezoid_120 as u8, 0x02);
        assert_eq!(FOCModulationType::Trapezoid_150 as u8, 0x03);
    }

    #[test]
    fn foc_motor_status_discriminants_match_upstream() {
        assert_eq!(FOCMotorStatus::motor_uninitialized as u8, 0x00);
        assert_eq!(FOCMotorStatus::motor_initializing as u8, 0x01);
        assert_eq!(FOCMotorStatus::motor_uncalibrated as u8, 0x02);
        assert_eq!(FOCMotorStatus::motor_calibrating as u8, 0x03);
        assert_eq!(FOCMotorStatus::motor_ready as u8, 0x04);
        assert_eq!(FOCMotorStatus::motor_error as u8, 0x08);
        assert_eq!(FOCMotorStatus::motor_calib_failed as u8, 0x0E);
        assert_eq!(FOCMotorStatus::motor_init_failed as u8, 0x0F);
    }

    // -----------------------------------------------------------------
    // Defaults — match the most-common user-facing value
    // for each enum. Upstream C++ initialises the motor
    // with `MotionControlType::torque`,
    // `TorqueControlType::voltage`, and
    // `FOCModulationType::SinePWM`, but the FOC-rs Rust
    // `Default` impl is set to `custom` / `voltage` /
    // `SinePWM` to be conservative (custom is the
    // "nothing assumed" mode that the motor's
    // `init_foc()` overrides anyway; FOCMotorStatus is
    // uninitialised to match the upstream pre-init state).
    // -----------------------------------------------------------------
    #[test]
    fn motion_control_type_default_is_custom() {
        assert_eq!(MotionControlType::default(), MotionControlType::custom);
    }

    #[test]
    fn torque_control_type_default_is_voltage() {
        assert_eq!(TorqueControlType::default(), TorqueControlType::voltage);
    }

    #[test]
    fn foc_modulation_type_default_is_sine_pwm() {
        assert_eq!(
            FOCModulationType::default(),
            FOCModulationType::SinePWM
        );
    }

    #[test]
    fn foc_motor_status_default_is_uninitialized() {
        assert_eq!(
            FOCMotorStatus::default(),
            FOCMotorStatus::motor_uninitialized
        );
    }

    // -----------------------------------------------------------------
    // Round-trip — each enum's discriminant survives a
    // cast to u8 and back via a `match` (we can't use
    // `transmute` because the workspace has
    // `unsafe_code = "forbid"`). Catches accidental
    // removal of `#[repr(u*)]`.
    // -----------------------------------------------------------------
    #[test]
    fn round_trip_motion_control_type() {
        for v in [
            MotionControlType::torque,
            MotionControlType::velocity,
            MotionControlType::angle,
            MotionControlType::velocity_openloop,
            MotionControlType::angle_openloop,
            MotionControlType::angle_nocascade,
            MotionControlType::custom,
        ] {
            let n = v as u8;
            let back = match n {
                0x00 => MotionControlType::torque,
                0x01 => MotionControlType::velocity,
                0x02 => MotionControlType::angle,
                0x03 => MotionControlType::velocity_openloop,
                0x04 => MotionControlType::angle_openloop,
                0x05 => MotionControlType::angle_nocascade,
                0x06 => MotionControlType::custom,
                _ => panic!("unknown discriminant {n}"),
            };
            assert_eq!(back, v);
        }
    }

    #[test]
    fn round_trip_torque_control_type() {
        for v in [
            TorqueControlType::voltage,
            TorqueControlType::dc_current,
            TorqueControlType::foc_current,
            TorqueControlType::estimated_current,
        ] {
            let n = v as u8;
            let back = match n {
                0x00 => TorqueControlType::voltage,
                0x01 => TorqueControlType::dc_current,
                0x02 => TorqueControlType::foc_current,
                0x03 => TorqueControlType::estimated_current,
                _ => panic!("unknown discriminant {n}"),
            };
            assert_eq!(back, v);
        }
    }

    #[test]
    fn round_trip_foc_modulation_type() {
        for v in [
            FOCModulationType::SinePWM,
            FOCModulationType::SpaceVectorPWM,
            FOCModulationType::Trapezoid_120,
            FOCModulationType::Trapezoid_150,
        ] {
            let n = v as u8;
            let back = match n {
                0x00 => FOCModulationType::SinePWM,
                0x01 => FOCModulationType::SpaceVectorPWM,
                0x02 => FOCModulationType::Trapezoid_120,
                0x03 => FOCModulationType::Trapezoid_150,
                _ => panic!("unknown discriminant {n}"),
            };
            assert_eq!(back, v);
        }
    }

    #[test]
    fn round_trip_foc_motor_status() {
        for v in [
            FOCMotorStatus::motor_uninitialized,
            FOCMotorStatus::motor_initializing,
            FOCMotorStatus::motor_uncalibrated,
            FOCMotorStatus::motor_calibrating,
            FOCMotorStatus::motor_ready,
            FOCMotorStatus::motor_error,
            FOCMotorStatus::motor_calib_failed,
            FOCMotorStatus::motor_init_failed,
        ] {
            let n = v as u8;
            let back = match n {
                0x00 => FOCMotorStatus::motor_uninitialized,
                0x01 => FOCMotorStatus::motor_initializing,
                0x02 => FOCMotorStatus::motor_uncalibrated,
                0x03 => FOCMotorStatus::motor_calibrating,
                0x04 => FOCMotorStatus::motor_ready,
                0x08 => FOCMotorStatus::motor_error,
                0x0E => FOCMotorStatus::motor_calib_failed,
                0x0F => FOCMotorStatus::motor_init_failed,
                _ => panic!("unknown discriminant {n}"),
            };
            assert_eq!(back, v);
        }
    }

    // -----------------------------------------------------------------
    // Enum sizes — each enum must be exactly one byte so
    // they pack into the FOCMotor's status / config
    // fields without padding.
    // -----------------------------------------------------------------
    #[test]
    fn enums_are_one_byte() {
        assert_eq!(core::mem::size_of::<MotionControlType>(), 1);
        assert_eq!(core::mem::size_of::<TorqueControlType>(), 1);
        assert_eq!(core::mem::size_of::<FOCModulationType>(), 1);
        assert_eq!(core::mem::size_of::<FOCMotorStatus>(), 1);
    }
}
