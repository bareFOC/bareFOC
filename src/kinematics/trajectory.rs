//! Motion trajectory generation and multi-turn angle unwrapping.
//!
//! Provides zero-allocation, `#![no_std]` algorithms for:
//! - [`TrapezoidalTrajectory`]: Real-time trapezoidal profile generator producing
//!   smooth intermediate position and velocity setpoints from acceleration and
//!   velocity limits to eliminate mechanical jerk.
//! - [`MultiTurnAccumulator`]: Continuous angle unwrapping across $\pm\pi$ sensor
//!   boundaries with full revolution counting.

use core::f32::consts::PI;

/// Real-time trapezoidal trajectory profile generator.
///
/// Computes jerk-free intermediate position and velocity setpoints at every control
/// loop cycle based on `profile_velocity` ($v_{\max}$) and `profile_acceleration` ($a_{\max}$).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidalTrajectory {
    current_pos: f32,
    current_vel: f32,
    target_pos: f32,
    profile_vel: f32,
    profile_accel: f32,
}

impl TrapezoidalTrajectory {
    /// Create a new trapezoidal trajectory generator initialized to `initial_pos`.
    pub const fn new(initial_pos: f32) -> Self {
        Self {
            current_pos: initial_pos,
            current_vel: 0.0,
            target_pos: initial_pos,
            profile_vel: 10.0,
            profile_accel: 25.0,
        }
    }

    /// Reset the current position and velocity state.
    pub fn reset(&mut self, pos: f32) {
        self.current_pos = pos;
        self.current_vel = 0.0;
        self.target_pos = pos;
    }

    /// Update profile velocity and acceleration limits.
    pub fn set_profile(&mut self, max_vel: f32, max_accel: f32) {
        self.profile_vel = libm::fabsf(max_vel);
        self.profile_accel = libm::fabsf(max_accel);
    }

    /// Set the destination target position.
    pub fn set_target(&mut self, target_pos: f32) {
        self.target_pos = target_pos;
    }

    /// Current instantaneous position setpoint.
    pub const fn current_position(&self) -> f32 {
        self.current_pos
    }

    /// Current instantaneous velocity setpoint.
    pub const fn current_velocity(&self) -> f32 {
        self.current_vel
    }

    /// Destination target position.
    pub const fn target_position(&self) -> f32 {
        self.target_pos
    }

    /// Check if trajectory has converged to target within tolerances.
    pub fn is_finished(&self, pos_tolerance: f32, vel_tolerance: f32) -> bool {
        let dp = libm::fabsf(self.target_pos - self.current_pos);
        let dv = libm::fabsf(self.current_vel);
        dp <= pos_tolerance && dv <= vel_tolerance
    }

    /// Step the trajectory generator by `dt` seconds, returning `(position, velocity)`.
    pub fn update(&mut self, dt: f32) -> (f32, f32) {
        if dt <= 0.0 {
            return (self.current_pos, self.current_vel);
        }

        let dp = self.target_pos - self.current_pos;
        let v_max = self.profile_vel;
        let a_max = self.profile_accel;

        // If limits are unconstrained or zero, snap directly to target
        if v_max <= 0.0 || a_max <= 0.0 {
            self.current_pos = self.target_pos;
            self.current_vel = 0.0;
            return (self.current_pos, self.current_vel);
        }

        // Distance required to brake to full stop from velocity v: d_brake = v^2 / (2 * a_max)
        // Desired velocity based on stopping distance to target:
        let v_desired = if dp > 0.0 {
            let max_decel_v = libm::sqrtf(2.0 * a_max * dp);
            if v_max < max_decel_v {
                v_max
            } else {
                max_decel_v
            }
        } else if dp < 0.0 {
            let max_decel_v = -libm::sqrtf(2.0 * a_max * -dp);
            if -v_max > max_decel_v {
                -v_max
            } else {
                max_decel_v
            }
        } else {
            0.0
        };

        // Accelerate or decelerate towards v_desired with rate limited by a_max * dt
        let max_dv = a_max * dt;
        let dv = v_desired - self.current_vel;
        if dv > max_dv {
            self.current_vel += max_dv;
        } else if dv < -max_dv {
            self.current_vel -= max_dv;
        } else {
            self.current_vel = v_desired;
        }

        // Integrate position: p += v * dt
        self.current_pos += self.current_vel * dt;

        // If close to target and step crossed target or velocity is near stop, snap to exact target
        let new_dp = self.target_pos - self.current_pos;
        if (dp * new_dp <= 0.0 && libm::fabsf(self.current_vel) <= a_max * dt * 2.0)
            || (libm::fabsf(new_dp) < 1e-3 && libm::fabsf(self.current_vel) < 0.05)
        {
            self.current_pos = self.target_pos;
            self.current_vel = 0.0;
        }

        (self.current_pos, self.current_vel)
    }
}

/// Continuous multi-turn angle accumulator tracking full shaft revolutions across $\pm\pi$ wrap boundaries.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MultiTurnAccumulator {
    prev_raw_angle: f32,
    revolutions: i32,
    cumulative_angle: f32,
    initialized: bool,
}

impl Default for MultiTurnAccumulator {
    fn default() -> Self {
        Self {
            prev_raw_angle: 0.0,
            revolutions: 0,
            cumulative_angle: 0.0,
            initialized: false,
        }
    }
}

impl MultiTurnAccumulator {
    /// Create a new uninitialized multi-turn accumulator.
    pub const fn new() -> Self {
        Self {
            prev_raw_angle: 0.0,
            revolutions: 0,
            cumulative_angle: 0.0,
            initialized: false,
        }
    }

    /// Reset accumulator with a known shaft angle and revolution count.
    pub fn reset(&mut self, initial_angle: f32, initial_revolutions: i32) {
        self.prev_raw_angle = initial_angle;
        self.revolutions = initial_revolutions;
        self.cumulative_angle = (initial_revolutions as f32) * (2.0 * PI) + initial_angle;
        self.initialized = true;
    }

    /// Update with a new raw single-turn sensor angle (in radians, $[-\pi, +\pi]$ or $[0, 2\pi]$).
    ///
    /// Detects wrap-around transitions, updates the revolution counter, and returns the
    /// continuous accumulated angle in radians.
    pub fn update(&mut self, raw_angle: f32) -> f32 {
        if !self.initialized {
            self.prev_raw_angle = raw_angle;
            self.revolutions = 0;
            self.cumulative_angle = raw_angle;
            self.initialized = true;
            return self.cumulative_angle;
        }

        let mut d_theta = raw_angle - self.prev_raw_angle;

        // Wrap detection: standard threshold is PI radians between consecutive samples
        if d_theta < -PI {
            // Forward wrap (+pi -> -pi or 2pi -> 0)
            d_theta += 2.0 * PI;
            self.revolutions += 1;
        } else if d_theta > PI {
            // Reverse wrap (-pi -> +pi or 0 -> 2pi)
            d_theta -= 2.0 * PI;
            self.revolutions -= 1;
        }

        self.cumulative_angle += d_theta;
        self.prev_raw_angle = raw_angle;

        self.cumulative_angle
    }

    /// Return the number of full completed revolutions.
    pub const fn revolutions(&self) -> i32 {
        self.revolutions
    }

    /// Return the continuous unwrapped shaft angle in radians.
    pub const fn cumulative_angle(&self) -> f32 {
        self.cumulative_angle
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trapezoidal_trajectory_ramp() {
        let mut traj = TrapezoidalTrajectory::new(0.0);
        traj.set_profile(10.0, 20.0); // v_max = 10 rad/s, a_max = 20 rad/s^2
        traj.set_target(5.0);

        let dt = 0.01; // 10 ms step (100 Hz)
        let mut prev_v = 0.0;
        let mut prev_p = 0.0;

        for _ in 0..100 {
            let (p, v) = traj.update(dt);
            // Verify velocity does not exceed limit
            assert!(v <= 10.001);
            // Verify acceleration does not exceed limit (within numerical tolerance)
            let accel = (v - prev_v) / dt;
            assert!(libm::fabsf(accel) <= 20.05);
            // Verify position advances towards target
            assert!(p >= prev_p);

            prev_v = v;
            prev_p = p;
        }

        // Simulate until completion
        for _ in 0..500 {
            traj.update(dt);
        }

        assert!(traj.is_finished(0.001, 0.01));
        assert!((traj.current_position() - 5.0).abs() < 1e-3);
        assert_eq!(traj.current_velocity(), 0.0);
    }

    #[test]
    fn test_trapezoidal_trajectory_negative_step() {
        let mut traj = TrapezoidalTrajectory::new(2.0);
        traj.set_profile(5.0, 10.0);
        traj.set_target(-2.0);

        let dt = 0.01;
        for _ in 0..500 {
            traj.update(dt);
        }

        assert!(traj.is_finished(0.001, 0.01));
        assert!((traj.current_position() - (-2.0)).abs() < 1e-3);
    }

    #[test]
    fn test_multi_turn_accumulator_forward_and_reverse() {
        let mut acc = MultiTurnAccumulator::new();
        acc.update(3.0); // Initial near +pi

        // Cross +pi into -pi
        let angle1 = acc.update(-3.0);
        assert_eq!(acc.revolutions(), 1);
        assert!(angle1 > 3.0);

        // Cross another revolution
        acc.update(0.0);
        acc.update(3.0);
        let angle2 = acc.update(-3.0);
        assert_eq!(acc.revolutions(), 2);
        assert!(angle2 > angle1);

        // Reverse rotation: cross -pi into +pi
        acc.update(0.0);
        acc.update(-3.0);
        let angle3 = acc.update(3.0);
        assert_eq!(acc.revolutions(), 1);
        assert!(angle3 < angle2);
    }
}
