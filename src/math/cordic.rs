//! Hardware CORDIC (Coordinate Rotation Digital Computer) trigonometric acceleration.
//!
//! Provides traits and helpers for hardware-accelerated sine and cosine computation,
//! notably on STM32G4 devices featuring the CORDIC peripheral.
//!
//! Upstream reference:
//! - STM32G4 Reference Manual RM0440 (CORDIC Co-processor)
//! - `source/Arduino-FOC/src/common/foc_utils.h`

use crate::math::{_2PI, _PI};

/// Common trait for trigonometric calculation backends (hardware CORDIC or software).
pub trait CordicTrig {
    /// Compute simultaneous sine and cosine of an electrical angle in radians.
    ///
    /// Returns `(sin, cos)`.
    fn sincos(&mut self, angle_rad: f32) -> (f32, f32);

    /// Compute sine of an electrical angle in radians.
    fn sin(&mut self, angle_rad: f32) -> f32 {
        self.sincos(angle_rad).0
    }

    /// Compute cosine of an electrical angle in radians.
    fn cos(&mut self, angle_rad: f32) -> f32 {
        self.sincos(angle_rad).1
    }
}

/// Software reference implementation of [`CordicTrig`] using `libm`.
#[derive(Debug, Default, Clone, Copy)]
pub struct SoftwareCordic;

impl CordicTrig for SoftwareCordic {
    fn sincos(&mut self, angle_rad: f32) -> (f32, f32) {
        crate::math::sincos(angle_rad)
    }
}

/// Convert an angle in radians in `[-π, π]` to Q1.31 fixed-point format.
///
/// The CORDIC angle input expects input scaled by `π`, such that `[-π, π]` maps to `[-1.0, 1.0)`.
#[inline]
pub fn angle_to_q1_31(angle_rad: f32) -> i32 {
    // Normalize to [-π, π]
    let mut a = angle_rad;
    while a > _PI {
        a -= _2PI;
    }
    while a < -_PI {
        a += _2PI;
    }

    // Scale to [-1.0, 1.0]
    let normalized = a / _PI;
    let clamped = normalized.clamp(-1.0, 0.9999999);
    (clamped * 2147483648.0) as i32
}

/// Convert a Q1.31 fixed-point value in `[-1.0, 1.0)` back to `f32`.
#[inline]
pub fn q1_31_to_f32(q: i32) -> f32 {
    q as f32 / 2147483648.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_software_cordic() {
        let mut cordic = SoftwareCordic;
        let (s0, c0) = cordic.sincos(0.0);
        assert!((s0 - 0.0).abs() < 1e-6);
        assert!((c0 - 1.0).abs() < 1e-6);

        let (s_pi2, c_pi2) = cordic.sincos(crate::math::_PI_2);
        assert!((s_pi2 - 1.0).abs() < 1e-6);
        assert!(c_pi2.abs() < 1e-6);

        assert!((cordic.sin(crate::math::_PI_2) - 1.0).abs() < 1e-6);
        assert!(cordic.cos(crate::math::_PI_2).abs() < 1e-6);
    }

    #[test]
    fn test_q1_31_conversion() {
        // 0 rad -> 0
        assert_eq!(angle_to_q1_31(0.0), 0);
        assert!((q1_31_to_f32(0) - 0.0).abs() < 1e-6);

        // π/2 rad -> 0.5 in Q1.31
        let q_pi2 = angle_to_q1_31(crate::math::_PI_2);
        let f_pi2 = q1_31_to_f32(q_pi2);
        assert!((f_pi2 - 0.5).abs() < 1e-4);

        // -π/2 rad -> -0.5 in Q1.31
        let q_neg_pi2 = angle_to_q1_31(-crate::math::_PI_2);
        let f_neg_pi2 = q1_31_to_f32(q_neg_pi2);
        assert!((f_neg_pi2 - (-0.5)).abs() < 1e-4);
    }
}
