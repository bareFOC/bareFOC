//! # actuator::actuator_1dof — 1-DOF cascaded dual-loop joint actuator
//!
//! Combines a high-speed inner motor FOC loop (5–10 kHz) with a medium-speed outer
//! joint position loop (200–500 Hz), output sensor tracking, and transmission Jacobian
//! gain scheduling ($K_{p,\text{eff}} = K_p / J(\theta)$).

use crate::common::pid::PIDController;
use crate::common::time::Clock;
use crate::traits::sensor::Sensor;
use crate::kinematics::actuator::motor::MotorControl;
use crate::kinematics::transmission::Transmission;

/// 1-DOF cascaded dual-loop robotic joint actuator.
pub struct Actuator1DOF<M, T, SOut, C: Clock> {
    /// Inner motor controller implementing [`MotorControl`].
    pub motor: M,
    /// 1-DOF transmission mapping rotor to joint coordinates.
    pub transmission: T,
    /// Output joint sensor mounted on the joint axis.
    pub output_sensor: SOut,
    /// Outer joint position PID controller.
    pub pid_position: PIDController<C>,
    /// Downsample factor for outer position loop relative to inner FOC steps.
    pub outer_downsample: u32,
    /// Downsample counter.
    motion_cnt: u32,
    /// Target joint position in radians.
    pub target_joint_angle: f32,
    /// Last sampled joint angle in radians.
    pub current_joint_angle: f32,
    /// Last commanded motor velocity in rad/s.
    pub last_cmd_velocity: f32,
    /// Calibration state: 0=idle, 1=running, 2=done, 3=error.
    pub calib_state: u8,
    /// Calibration progress 0–100.
    pub calib_progress: u8,
}

impl<M, T, SOut, C: Clock> Actuator1DOF<M, T, SOut, C> {
    /// Create a new 1-DOF cascaded actuator.
    pub fn new(
        motor: M,
        transmission: T,
        output_sensor: SOut,
        pid_position: PIDController<C>,
    ) -> Self {
        Self {
            motor,
            transmission,
            output_sensor,
            pid_position,
            outer_downsample: 10,
            motion_cnt: 0,
            target_joint_angle: 0.0,
            current_joint_angle: 0.0,
            last_cmd_velocity: 0.0,
            calib_state: 0,
            calib_progress: 0,
        }
    }

    /// Set downsample ratio between inner FOC loop and outer position loop.
    pub fn with_downsample(mut self, downsample: u32) -> Self {
        self.outer_downsample = if downsample == 0 { 1 } else { downsample };
        self
    }
}

impl<M, T, SOut, C: Clock> Actuator1DOF<M, T, SOut, C>
where
    M: MotorControl<C>,
    T: Transmission<1, 1>,
    SOut: Sensor<C>,
{
    /// Execute the medium-speed outer position control loop.
    ///
    /// 1. Updates the joint output sensor.
    /// 2. Computes joint position error: $e = \theta_{\text{target}} - \theta_{\text{out}}$.
    /// 3. Reads motor shaft angle $\theta_{\text{mot}}$ and evaluates instantaneous Jacobian $J(\theta_{\text{mot}})$.
    /// 4. Computes gain-scheduled motor velocity command: $v_{\text{cmd}} = \frac{v_{\text{PID}}}{J(\theta_{\text{mot}})}$.
    /// 5. Feeds $v_{\text{cmd}}$ into the motor velocity controller.
    pub fn step_outer(&mut self, target_joint_angle: f32, clock: &C) -> f32 {
        self.target_joint_angle = target_joint_angle;
        self.output_sensor.update(clock);
        self.current_joint_angle = self.output_sensor.get_angle();

        let error = target_joint_angle - self.current_joint_angle;
        let v_joint_raw = self.pid_position.update(error, clock);

        let mot_angle = self.motor.shaft_angle();
        let j = self.transmission.jacobian(&[mot_angle])[0][0];

        // Jacobian gain scheduling: v_mot = v_joint / J
        let abs_j = if j.abs() < 1e-4 { 1e-4 } else { j.abs() };
        let sign_j = if j < 0.0 { -1.0 } else { 1.0 };
        let v_mot_cmd = (v_joint_raw / abs_j) * sign_j;

        let v_limit = self.motor.velocity_limit();
        let v_clamped = if v_mot_cmd > v_limit {
            v_limit
        } else if v_mot_cmd < -v_limit {
            -v_limit
        } else {
            v_mot_cmd
        };

        self.last_cmd_velocity = v_clamped;
        self.motor.move_motor(v_clamped, clock);
        v_clamped
    }

    /// Execute the high-speed inner FOC commutation loop (5–10 kHz).
    #[inline]
    pub fn step_foc(&mut self, clock: &C) {
        self.motor.loop_foc(clock);
    }

    /// Combined step helper: updates the outer loop when decimated, and always runs the inner FOC loop.
    pub fn step(&mut self, target_joint_angle: f32, clock: &C) {
        self.motion_cnt += 1;
        if self.motion_cnt >= self.outer_downsample {
            self.motion_cnt = 0;
            self.step_outer(target_joint_angle, clock);
        }
        self.step_foc(clock);
    }

    /// Get current joint position in radians from the output sensor.
    #[inline]
    pub fn get_joint_angle(&self) -> f32 {
        self.current_joint_angle
    }

    /// Get current joint velocity in rad/s from the output sensor.
    #[inline]
    pub fn get_joint_velocity(&mut self) -> f32 {
        self.output_sensor.get_velocity()
    }
}

