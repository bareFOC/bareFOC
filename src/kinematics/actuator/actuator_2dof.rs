//! # actuator::actuator_2dof — 2-DOF coupled differential joint actuator
//!
//! Coordinates two BLDC motors, a 2-DOF transmission (such as a differential wrist
//! or gimbal), and a synchronous 2-axis sensor ([`Sensor2D`]) with multivariable
//! inverse-Jacobian trajectory resolution.

use crate::common::pid::PIDController;
use crate::common::time::Clock;
use crate::kinematics::actuator::motor::MotorControl;
use crate::kinematics::sensor::Sensor2D;
use crate::kinematics::transmission::Transmission;

/// 2-DOF coupled differential robotic joint actuator.
pub struct Actuator2DOF<M1, M2, T, S2D, C: Clock> {
    /// Motor 1 (e.g. differential left / pitch motor).
    pub motor1: M1,
    /// Motor 2 (e.g. differential right / roll motor).
    pub motor2: M2,
    /// 2-DOF coupled transmission mechanism.
    pub transmission: T,
    /// Multi-axis synchronous 2D sensor.
    pub sensor_2d: S2D,
    /// Outer position PID for axis 0 (Pitch / $\alpha$).
    pub pid_alpha: PIDController<C>,
    /// Outer position PID for axis 1 (Roll / $\beta$).
    pub pid_beta: PIDController<C>,
    /// Downsample ratio between inner FOC step and outer position loop.
    pub outer_downsample: u32,
    motion_cnt: u32,
    /// Target joint angles $[\alpha, \beta]$ in radians.
    pub target_joint_angles: [f32; 2],
    /// Current measured joint angles $[\alpha, \beta]$ in radians.
    pub current_joint_angles: [f32; 2],
    /// Last commanded motor velocities $[v_{m1}, v_{m2}]$ in rad/s.
    pub last_cmd_velocities: [f32; 2],
}

impl<M1, M2, T, S2D, C: Clock> Actuator2DOF<M1, M2, T, S2D, C> {
    /// Create a new 2-DOF coupled differential actuator.
    pub fn new(
        motor1: M1,
        motor2: M2,
        transmission: T,
        sensor_2d: S2D,
        pid_alpha: PIDController<C>,
        pid_beta: PIDController<C>,
    ) -> Self {
        Self {
            motor1,
            motor2,
            transmission,
            sensor_2d,
            pid_alpha,
            pid_beta,
            outer_downsample: 10,
            motion_cnt: 0,
            target_joint_angles: [0.0, 0.0],
            current_joint_angles: [0.0, 0.0],
            last_cmd_velocities: [0.0, 0.0],
        }
    }

    /// Set outer loop decimation factor.
    pub fn with_downsample(mut self, downsample: u32) -> Self {
        self.outer_downsample = if downsample == 0 { 1 } else { downsample };
        self
    }
}

impl<M1, M2, T, S2D, C: Clock> Actuator2DOF<M1, M2, T, S2D, C>
where
    M1: MotorControl<C>,
    M2: MotorControl<C>,
    T: Transmission<2, 2>,
    S2D: Sensor2D<C>,
{
    /// Execute the medium-speed outer position control step for both axes simultaneously.
    ///
    /// 1. Synchronously reads joint angles $[\alpha, \beta]$.
    /// 2. Computes joint position tracking errors.
    /// 3. Computes joint velocity targets via separate axis PIDs.
    /// 4. Projects joint velocities into motor velocity targets using the inverse Jacobian $\mathbf{J}^{-1}$.
    /// 5. Clamps to motor velocity limits and dispatches commands.
    pub fn step_outer(&mut self, target_joint_angles: [f32; 2], clock: &C) -> [f32; 2] {
        self.target_joint_angles = target_joint_angles;
        self.sensor_2d.update_2d(clock);
        self.current_joint_angles = self.sensor_2d.get_angles();

        let err_alpha = target_joint_angles[0] - self.current_joint_angles[0];
        let err_beta = target_joint_angles[1] - self.current_joint_angles[1];

        let v_alpha = self.pid_alpha.update(err_alpha, clock);
        let v_beta = self.pid_beta.update(err_beta, clock);

        let mot_angles = [self.motor1.shaft_angle(), self.motor2.shaft_angle()];
        let jac = self.transmission.jacobian(&mot_angles);

        // 2x2 matrix inversion of Jacobian J:
        // [a b; c d]^-1 = 1/(ad - bc) * [d -b; -c a]
        let a = jac[0][0];
        let b = jac[0][1];
        let c = jac[1][0];
        let d = jac[1][1];

        let det = a * d - b * c;
        let (v_m1, v_m2) = if det.abs() > 1e-9 {
            let inv_det = 1.0 / det;
            (
                inv_det * (d * v_alpha - b * v_beta),
                inv_det * (-c * v_alpha + a * v_beta),
            )
        } else {
            (0.0, 0.0)
        };

        let lim1 = self.motor1.velocity_limit();
        let lim2 = self.motor2.velocity_limit();

        let v_m1_clamped = if v_m1 > lim1 {
            lim1
        } else if v_m1 < -lim1 {
            -lim1
        } else {
            v_m1
        };

        let v_m2_clamped = if v_m2 > lim2 {
            lim2
        } else if v_m2 < -lim2 {
            -lim2
        } else {
            v_m2
        };

        self.last_cmd_velocities = [v_m1_clamped, v_m2_clamped];
        self.motor1.move_motor(v_m1_clamped, clock);
        self.motor2.move_motor(v_m2_clamped, clock);

        [v_m1_clamped, v_m2_clamped]
    }

    /// Execute the inner FOC commutation loops for both motors.
    #[inline]
    pub fn step_foc(&mut self, clock: &C) {
        self.motor1.loop_foc(clock);
        self.motor2.loop_foc(clock);
    }

    /// Combined step helper: decimate outer loop and run both inner motor FOC loops.
    pub fn step(&mut self, target_joint_angles: [f32; 2], clock: &C) {
        self.motion_cnt += 1;
        if self.motion_cnt >= self.outer_downsample {
            self.motion_cnt = 0;
            self.step_outer(target_joint_angles, clock);
        }
        self.step_foc(clock);
    }

    /// Return measured joint angles $[\alpha, \beta]$.
    #[inline]
    pub fn get_joint_angles(&self) -> [f32; 2] {
        self.current_joint_angles
    }

    /// Return measured joint velocities from the 2D sensor.
    #[inline]
    pub fn get_joint_velocities(&self) -> [f32; 2] {
        self.sensor_2d.get_velocities()
    }
}
