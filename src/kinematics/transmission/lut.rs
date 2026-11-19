//! # transmission::lut — uniform Look-Up Table non-linear transmission
//!
//! Provides $\mathcal{O}(1)$ piecewise linear interpolation and discrete Jacobian
//! estimation for non-linear cams, linkages, and tendon profiles.

use super::Transmission;

/// Uniform Look-Up Table transmission with $N$ sample points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LutTransmission<const N: usize> {
    /// Minimum motor angle in radians.
    pub mot_min: f32,
    /// Maximum motor angle in radians.
    pub mot_max: f32,
    /// Joint output angles at uniform grid points from `mot_min` to `mot_max`.
    pub table: [f32; N],
    /// Current point index pointer for streaming writes (0..N).
    pub point_index: u16,
}

impl<const N: usize> LutTransmission<N> {
    /// Create a new LUT transmission over $[mot\_min, mot\_max]$ with sample table.
    ///
    /// Requires $N \ge 2$ and $mot\_max > mot\_min$.
    pub const fn new(mot_min: f32, mot_max: f32, table: [f32; N]) -> Self {
        Self {
            mot_min,
            mot_max,
            table,
            point_index: 0,
        }
    }

    /// Set initial point index.
    pub fn with_point_index(mut self, point_index: u16) -> Self {
        self.point_index = point_index;
        self
    }

    /// Step size between adjacent grid points in motor radians.
    #[inline]
    pub fn delta_theta(&self) -> f32 {
        if N < 2 || self.mot_max <= self.mot_min {
            1.0
        } else {
            (self.mot_max - self.mot_min) / ((N - 1) as f32)
        }
    }

    /// Verify whether the table is strictly monotonic (either monotonically increasing
    /// or monotonically decreasing).
    pub fn validate_monotonicity(&self) -> bool {
        if N < 2 {
            return false;
        }
        let diff0 = self.table[1] - self.table[0];
        if diff0.abs() < 1e-7 {
            return false;
        }
        let increasing = diff0 > 0.0;
        for i in 1..(N - 1) {
            let diff = self.table[i + 1] - self.table[i];
            if increasing && diff <= 1e-7 {
                return false;
            }
            if !increasing && diff >= -1e-7 {
                return false;
            }
        }
        true
    }

    /// Fast $\mathcal{O}(1)$ forward linear interpolation: joint angle from motor angle.
    pub fn forward_1d(&self, mot_angle: f32) -> f32 {
        if N == 0 {
            return 0.0;
        }
        if N == 1 {
            return self.table[0];
        }
        if mot_angle <= self.mot_min {
            return self.table[0];
        }
        if mot_angle >= self.mot_max {
            return self.table[N - 1];
        }

        let delta = self.delta_theta();
        let u = (mot_angle - self.mot_min) / delta;
        let mut idx = u as usize;
        if idx >= N - 1 {
            idx = N - 2;
        }
        let frac = u - (idx as f32);
        (1.0 - frac) * self.table[idx] + frac * self.table[idx + 1]
    }

    /// Discrete Jacobian $d\theta_{\text{joint}} / d\theta_{\text{mot}}$ evaluated at `mot_angle`.
    pub fn jacobian_1d(&self, mot_angle: f32) -> f32 {
        if N < 2 {
            return 0.0;
        }
        let delta = self.delta_theta();
        if delta <= 0.0 {
            return 0.0;
        }

        let u = (mot_angle - self.mot_min) / delta;
        let mut idx = if u < 0.0 {
            0
        } else {
            u as usize
        };
        if idx >= N - 1 {
            idx = N - 2;
        }
        (self.table[idx + 1] - self.table[idx]) / delta
    }

    /// Inverse kinematics: finds motor angle corresponding to `joint_angle`.
    ///
    /// Uses binary search on monotonic LUT followed by local linear interpolation.
    pub fn inverse_1d(&self, joint_angle: f32) -> f32 {
        if N < 2 {
            return self.mot_min;
        }
        let delta = self.delta_theta();
        let increasing = self.table[N - 1] > self.table[0];

        if increasing {
            if joint_angle <= self.table[0] {
                return self.mot_min;
            }
            if joint_angle >= self.table[N - 1] {
                return self.mot_max;
            }
            // Binary search interval
            let mut low = 0;
            let mut high = N - 1;
            while high - low > 1 {
                let mid = (low + high) / 2;
                if self.table[mid] <= joint_angle {
                    low = mid;
                } else {
                    high = mid;
                }
            }
            let denom = self.table[high] - self.table[low];
            let frac = if denom.abs() > 1e-7 {
                (joint_angle - self.table[low]) / denom
            } else {
                0.0
            };
            self.mot_min + (low as f32 + frac) * delta
        } else {
            if joint_angle >= self.table[0] {
                return self.mot_min;
            }
            if joint_angle <= self.table[N - 1] {
                return self.mot_max;
            }
            let mut low = 0;
            let mut high = N - 1;
            while high - low > 1 {
                let mid = (low + high) / 2;
                if self.table[mid] >= joint_angle {
                    low = mid;
                } else {
                    high = mid;
                }
            }
            let denom = self.table[high] - self.table[low];
            let frac = if denom.abs() > 1e-7 {
                (joint_angle - self.table[low]) / denom
            } else {
                0.0
            };
            self.mot_min + (low as f32 + frac) * delta
        }
    }
}

impl<const N: usize> Transmission<1, 1> for LutTransmission<N> {
    fn forward(&self, mot_angles: &[f32; 1]) -> [f32; 1] {
        [self.forward_1d(mot_angles[0])]
    }

    fn inverse(&self, joint_angles: &[f32; 1]) -> [f32; 1] {
        [self.inverse_1d(joint_angles[0])]
    }

    fn jacobian(&self, mot_angles: &[f32; 1]) -> [[f32; 1]; 1] {
        [[self.jacobian_1d(mot_angles[0])]]
    }
}
