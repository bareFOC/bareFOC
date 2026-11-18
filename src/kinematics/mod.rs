//! # barefoc-kinematics
//!
//! Robotic joint kinematics, non-linear transmissions, cascaded dual-loop actuators,
//! and on-device auto-calibration for the BareFOC ecosystem.
//!
//! # Architecture
//!
//! - **[`transmission`]**: Generic [`Transmission<M, J>`] trait mapping between $M$ motor
//!   axes and $J$ joint axes with forward kinematics, inverse kinematics, and instantaneous
//!   Jacobians. Includes [`LinearGear`], [`LutTransmission`], [`AnalyticalLinkage`]
//!   ([`FourBarLinkage`], [`EccentricCam`]), and [`CoupledDifferential`].
//! - **[`sensor`]**: Multi-axis synchronous [`Sensor2D`] trait and 1D adapter views.
//! - **[`actuator`]**: Cascaded dual-loop controllers ([`Actuator1DOF`], [`Actuator2DOF`])
//!   combining high-speed inner motor FOC with medium-speed outer joint position control
//!   and Jacobian gain scheduling ($K_{p,\text{eff}} = K_p / J(\theta)$).
//! - **[`calibration`]**: 1-call [`AutonomousCalibrator`] and step-by-step
//!   [`InteractiveCalibrator`] state machines with 64-byte quadword-aligned
//!   [`FlashCalibrationData`] serialization.

pub mod actuator;
pub mod calibration;
pub mod sensor;
pub mod transmission;

pub use actuator::{Actuator1DOF, Actuator2DOF, MotorControl};
pub use calibration::{
    AutonomousCalibrator, CalibrationConfig, CalibrationError, CalibrationResult,
    FlashCalibrationData, InteractiveCalibrator, InteractiveState, StorageError,
};
pub use sensor::{AxisView, DualSensorAdapter, MockSensor2D, Sensor2D};
pub use transmission::{
    AnalyticalLinkage, CoupledDifferential, EccentricCam, FourBarBranch, FourBarLinkage,
    LinearGear, LutTransmission, Transmission,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::pid::PIDController;
    use crate::common::time::Clock;
    use crate::traits::sensor::{Sensor, SensorBase};

    // -----------------------------------------------------------------------
    // Test mocks
    // -----------------------------------------------------------------------

    struct MockClock {
        us: core::cell::Cell<u32>,
    }

    impl MockClock {
        fn new() -> Self {
            Self {
                us: core::cell::Cell::new(1000),
            }
        }
        fn advance(&self, dt_us: u32) {
            self.us.set(self.us.get() + dt_us);
        }
    }

    impl Clock for MockClock {
        fn now_us(&self) -> u32 {
            self.us.get()
        }
    }

    struct SimpleMockSensor {
        base: SensorBase,
        angle: core::cell::Cell<f32>,
    }

    impl SimpleMockSensor {
        fn new(initial_angle: f32) -> Self {
            Self {
                base: SensorBase::new(),
                angle: core::cell::Cell::new(initial_angle),
            }
        }
    }

    impl<C: Clock> Sensor<C> for SimpleMockSensor {
        fn base(&self) -> &SensorBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut SensorBase {
            &mut self.base
        }
        fn get_sensor_angle(&self) -> f32 {
            self.angle.get()
        }
    }

    struct MockMotor {
        angle: f32,
        velocity: f32,
        velocity_limit: f32,
        loop_foc_count: usize,
    }

    impl MockMotor {
        fn new() -> Self {
            Self {
                angle: 0.0,
                velocity: 0.0,
                velocity_limit: 50.0,
                loop_foc_count: 0,
            }
        }
    }

    impl<C: Clock> MotorControl<C> for MockMotor {
        fn shaft_angle(&self) -> f32 {
            self.angle
        }
        fn shaft_velocity(&mut self) -> f32 {
            self.velocity
        }
        fn velocity_limit(&self) -> f32 {
            self.velocity_limit
        }
        fn move_motor(&mut self, target: f32, _clock: &C) {
            self.velocity = target;
        }
        fn loop_foc(&mut self, _clock: &C) {
            self.loop_foc_count += 1;
        }
    }

    // -----------------------------------------------------------------------
    // LinearGear Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_linear_gear_kinematics() {
        let gear = LinearGear::new(21.0)
            .with_zero_offset(1.0)
            .with_inverted(false);

        // Forward: (mot - offset) / ratio
        let mot = 43.0;
        let joint = gear.forward_1d(mot);
        assert!((joint - (43.0 - 1.0) / 21.0).abs() < 1e-6);

        // Inverse: joint * ratio + offset
        let mot_inv = gear.inverse_1d(joint);
        assert!((mot_inv - mot).abs() < 1e-6);

        // Jacobian: 1 / 21
        let jac = gear.jacobian_1d();
        assert!((jac - 1.0 / 21.0).abs() < 1e-6);

        // Transmission trait forward/inverse/jacobian
        let forward_trait = gear.forward(&[mot]);
        assert!((forward_trait[0] - joint).abs() < 1e-6);
        let inverse_trait = gear.inverse(&[joint]);
        assert!((inverse_trait[0] - mot).abs() < 1e-6);
        let jac_trait = gear.jacobian(&[mot]);
        assert!((jac_trait[0][0] - jac).abs() < 1e-6);
    }

    #[test]
    fn test_linear_gear_inversion() {
        let gear = LinearGear::new(10.0).with_inverted(true);
        assert_eq!(gear.sign(), -1.0);

        let joint = gear.forward_1d(20.0);
        assert!((joint - (-2.0)).abs() < 1e-6);
        let mot = gear.inverse_1d(-2.0);
        assert!((mot - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_linear_gear_backlash_compensation() {
        let gear = LinearGear::new(1.0).with_backlash(0.02, 0.05);

        // At zero velocity, backlash offset is 0
        let comp_zero = gear.backlash_compensation(0.0);
        assert!(comp_zero.abs() < 1e-6);

        // At high positive velocity, comp -> +delta
        let comp_pos = gear.backlash_compensation(5.0);
        assert!((comp_pos - 0.02).abs() < 1e-4);

        // At high negative velocity, comp -> -delta
        let comp_neg = gear.backlash_compensation(-5.0);
        assert!((comp_neg - (-0.02)).abs() < 1e-4);
    }

    #[test]
    fn test_linear_gear_elasticity_deflection() {
        let gear = LinearGear::new(1.0).with_torsional_stiffness(500.0); // 500 Nm/rad

        let torque = 5.0; // 5 Nm
        let deflection = gear.elasticity_deflection(torque);
        assert!((deflection - 0.01).abs() < 1e-6); // 5 / 500 = 0.01 rad

        let compensated = gear.compensate_forward(1.0, 0.0, torque);
        assert!((compensated - 1.01).abs() < 1e-6);
    }

    // -----------------------------------------------------------------------
    // LutTransmission Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_lut_piecewise_linear() {
        // Table of y = 2 * x + 1 for x in [0.0, 4.0] with 5 points: x = [0, 1, 2, 3, 4]
        let table = [1.0, 3.0, 5.0, 7.0, 9.0];
        let lut = LutTransmission::new(0.0, 4.0, table);

        assert!(lut.validate_monotonicity());

        // Exact grid points
        assert!((lut.forward_1d(0.0) - 1.0).abs() < 1e-6);
        assert!((lut.forward_1d(2.0) - 5.0).abs() < 1e-6);
        assert!((lut.forward_1d(4.0) - 9.0).abs() < 1e-6);

        // Midpoints
        assert!((lut.forward_1d(0.5) - 2.0).abs() < 1e-6);
        assert!((lut.forward_1d(2.5) - 6.0).abs() < 1e-6);

        // Bounds clamping
        assert!((lut.forward_1d(-1.0) - 1.0).abs() < 1e-6);
        assert!((lut.forward_1d(5.0) - 9.0).abs() < 1e-6);

        // Jacobian: dy/dx = 2.0 everywhere
        assert!((lut.jacobian_1d(0.5) - 2.0).abs() < 1e-5);
        assert!((lut.jacobian_1d(3.2) - 2.0).abs() < 1e-5);

        // Inverse kinematics
        assert!((lut.inverse_1d(1.0) - 0.0).abs() < 1e-6);
        assert!((lut.inverse_1d(5.0) - 2.0).abs() < 1e-6);
        assert!((lut.inverse_1d(6.0) - 2.5).abs() < 1e-6);
        assert!((lut.inverse_1d(9.0) - 4.0).abs() < 1e-6);
    }

    #[test]
    fn test_lut_monotonicity_validation() {
        let valid_table = [1.0, 2.0, 3.0, 4.0];
        let valid_lut = LutTransmission::new(0.0, 3.0, valid_table);
        assert!(valid_lut.validate_monotonicity());

        let non_monotonic_table = [1.0, 3.0, 2.0, 4.0];
        let invalid_lut = LutTransmission::new(0.0, 3.0, non_monotonic_table);
        assert!(!invalid_lut.validate_monotonicity());
    }

    // -----------------------------------------------------------------------
    // AnalyticalLinkage Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_four_bar_linkage() {
        // Equal link lengths L1 = L2 = L3 = L4 = 1.0 (parallelogram linkage)
        let fb = FourBarLinkage::new(1.0, 1.0, 1.0, 1.0, FourBarBranch::Open);

        let angle_in = 0.5; // rad
        let angle_out = fb.forward_1d(angle_in);
        // In open parallelogram branch, angle_out == angle_in
        assert!((angle_out - angle_in).abs() < 1e-3);

        let inv = fb.inverse_1d(angle_out);
        assert!((inv - angle_in).abs() < 1e-3);

        // Analytical Jacobian d(out)/d(in) should be 1.0
        let jac = fb.jacobian_1d(angle_in);
        assert!((jac - 1.0).abs() < 1e-2);
    }

    #[test]
    fn test_eccentric_cam() {
        let cam = EccentricCam::new(0.2, 1.0); // e = 0.2, L = 1.0

        let theta_in = 0.3;
        let theta_out = cam.forward_1d(theta_in);
        let theta_inv = cam.inverse_1d(theta_out);
        assert!((theta_inv - theta_in).abs() < 1e-5);

        // Analytical Jacobian vs numerical finite difference
        let jac = cam.jacobian_1d(theta_in);
        let eps = 1e-4;
        let num_jac = (cam.forward_1d(theta_in + eps) - cam.forward_1d(theta_in - eps)) / (2.0 * eps);
        assert!((jac - num_jac).abs() < 1e-4);
    }

    // -----------------------------------------------------------------------
    // CoupledDifferential Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_coupled_differential_kinematics_and_effort() {
        let diff = CoupledDifferential::symmetric(2.0); // R = 2.0

        let mot = [4.0, 2.0];
        // alpha = (m1 + m2)/(2R) = 6.0 / 4.0 = 1.5
        // beta = (m1 - m2)/(2R) = 2.0 / 4.0 = 0.5
        let joints = diff.forward_2d(mot);
        assert!((joints[0] - 1.5).abs() < 1e-6);
        assert!((joints[1] - 0.5).abs() < 1e-6);

        // Inverse
        let mot_inv = diff.inverse_2d(joints);
        assert!((mot_inv[0] - mot[0]).abs() < 1e-6);
        assert!((mot_inv[1] - mot[1]).abs() < 1e-6);

        // Effort reflection: tau_m = J^T * tau_j
        let joint_torques = [1.0, 2.0];
        let mot_torques = diff.reflect_torques(joint_torques);
        // tau_m1 = 0.25 * 1.0 + 0.25 * 2.0 = 0.75
        // tau_m2 = 0.25 * 1.0 - 0.25 * 2.0 = -0.25
        assert!((mot_torques[0] - 0.75).abs() < 1e-6);
        assert!((mot_torques[1] - (-0.25)).abs() < 1e-6);

        // Power balance verification: tau_m . v_m == tau_j . v_j
        let mot_vel = [10.0, 5.0];
        let joint_vel = diff.forward_velocity(&mot, &mot_vel);
        let power_joint = joint_torques[0] * joint_vel[0] + joint_torques[1] * joint_vel[1];
        let power_mot = mot_torques[0] * mot_vel[0] + mot_torques[1] * mot_vel[1];
        assert!((power_joint - power_mot).abs() < 1e-5);
    }

    // -----------------------------------------------------------------------
    // Sensor2D & Adapters Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_sensor_2d_and_adapters() {
        let clock = MockClock::new();
        let s1 = SimpleMockSensor::new(1.23);
        let s2 = SimpleMockSensor::new(4.56);
        let mut adapter = DualSensorAdapter::new(s1, s2);

        adapter.update_2d(&clock);
        let angles = adapter.get_angles();
        assert!((angles[0] - 1.23).abs() < 1e-6);
        assert!((angles[1] - 4.56).abs() < 1e-6);

        // 1D AxisView
        let axis0 = AxisView::new(&adapter, 0);
        assert!((axis0.get_sensor_angle() - 1.23).abs() < 1e-6);
        let axis1 = AxisView::new(&adapter, 1);
        assert!((axis1.get_sensor_angle() - 4.56).abs() < 1e-6);
    }

    // -----------------------------------------------------------------------
    // Actuator1DOF Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_actuator_1dof_cascaded_control_and_gain_scheduling() {
        let clock = MockClock::new();
        let motor = MockMotor::new();
        let gear = LinearGear::new(10.0); // J = 0.1
        let out_sensor = SimpleMockSensor::new(0.0);
        let pid_pos = PIDController::new(
            2.0,
            0.0,
            0.0,
            crate::math::NOT_SET,
            100.0,
            crate::math::NOT_SET,
            &clock,
        );

        let mut actuator = Actuator1DOF::new(motor, gear, out_sensor, pid_pos);

        // Command joint target = 0.5 rad (current joint = 0.0)
        // Error = 0.5
        // v_joint_raw = P * error = 2.0 * 0.5 = 1.0 rad/s
        // Gain scheduling: J = 0.1, v_mot_cmd = v_joint_raw / J = 1.0 / 0.1 = 10.0 rad/s!
        clock.advance(1000);
        let v_cmd = actuator.step_outer(0.5, &clock);
        assert!((v_cmd - 10.0).abs() < 1e-4);
        assert!((actuator.motor.velocity - 10.0).abs() < 1e-4);

        // Step FOC
        actuator.step_foc(&clock);
        assert_eq!(actuator.motor.loop_foc_count, 1);
    }

    // -----------------------------------------------------------------------
    // Actuator2DOF Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_actuator_2dof_decoupling() {
        let clock = MockClock::new();
        let m1 = MockMotor::new();
        let m2 = MockMotor::new();
        let diff = CoupledDifferential::symmetric(1.0); // 1:1 ratio
        let sensor2d = MockSensor2D::new([0.0, 0.0]);

        let pid_a = PIDController::new(
            1.0,
            0.0,
            0.0,
            crate::math::NOT_SET,
            100.0,
            crate::math::NOT_SET,
            &clock,
        );
        let pid_b = PIDController::new(
            1.0,
            0.0,
            0.0,
            crate::math::NOT_SET,
            100.0,
            crate::math::NOT_SET,
            &clock,
        );

        let mut act2 = Actuator2DOF::new(m1, m2, diff, sensor2d, pid_a, pid_b);

        // Command Pitch alpha = 2.0, Roll beta = 0.0
        // Joint target velocities: v_alpha = 2.0, v_beta = 0.0
        // Differential inverse: m1 = R*(alpha + beta) = 2.0, m2 = R*(alpha - beta) = 2.0
        clock.advance(1000);
        let v_cmds = act2.step_outer([2.0, 0.0], &clock);
        assert!((v_cmds[0] - 2.0).abs() < 1e-4);
        assert!((v_cmds[1] - 2.0).abs() < 1e-4);

        // Step FOC
        act2.step_foc(&clock);
        assert_eq!(act2.motor1.loop_foc_count, 1);
        assert_eq!(act2.motor2.loop_foc_count, 1);
    }

    // -----------------------------------------------------------------------
    // Calibration & Storage Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_flash_storage_serialization_and_checksum() {
        let mut data = FlashCalibrationData {
            gear_ratio: 21.0,
            zero_offset: 0.123,
            joint_min: -1.5,
            joint_max: 1.5,
            ..Default::default()
        };
        data.update_checksum();

        // 64-byte alignment and size check
        assert_eq!(core::mem::size_of::<FlashCalibrationData>(), 64);
        assert_eq!(core::mem::align_of::<FlashCalibrationData>(), 64);

        let bytes = data.to_bytes();
        assert_eq!(bytes.len(), 64);

        let restored = FlashCalibrationData::from_bytes(&bytes).expect("valid deserialization");
        assert_eq!(restored.gear_ratio, 21.0);
        assert_eq!(restored.zero_offset, 0.123);
        assert_eq!(restored.joint_min, -1.5);
        assert_eq!(restored.joint_max, 1.5);

        // Tamper test
        let mut corrupted = bytes;
        corrupted[10] ^= 0xFF;
        assert_eq!(
            FlashCalibrationData::from_bytes(&corrupted).unwrap_err(),
            StorageError::ChecksumMismatch
        );
    }

    #[test]
    fn test_autonomous_calibrator_fitting() {
        let config = CalibrationConfig::default();
        let calibrator = AutonomousCalibrator::new(config);

        // Sample a non-linear cam mechanism: y = sin(x / 4) + x * 0.5 for x in [0.0, 4.0]
        let res: CalibrationResult<16> = calibrator
            .fit_table(0.0, 4.0, |x| libm::sinf(x * 0.25) + x * 0.5)
            .expect("calibration fitting must succeed");

        assert_eq!(res.mot_min, 0.0);
        assert_eq!(res.mot_max, 4.0);
        assert!(res.lut.validate_monotonicity());
        assert_eq!(res.config.lut_points, 16);
    }

    #[test]
    fn test_interactive_calibrator_state_machine() {
        let mut calib: InteractiveCalibrator<4> = InteractiveCalibrator::new();
        assert_eq!(calib.state, InteractiveState::Idle);

        // 1. calib start
        let msg = calib.handle_command("calib start", 0.0, 0.0).unwrap();
        assert!(msg.contains("started"));
        assert_eq!(calib.state, InteractiveState::SettingMin);

        // 2. calib min at mot = -2.0, joint = -0.5
        let _ = calib.handle_command("calib min", -2.0, -0.5).unwrap();
        assert_eq!(calib.state, InteractiveState::SettingMax);
        assert_eq!(calib.mot_min, -2.0);

        // 3. calib max at mot = 2.0, joint = 0.5
        let _ = calib.handle_command("calib max", 2.0, 0.5).unwrap();
        assert_eq!(calib.state, InteractiveState::ReadyToSweep);
        assert_eq!(calib.mot_max, 2.0);

        // 4. calib sweep
        let _ = calib.handle_command("calib sweep", 0.0, 0.0).unwrap();
        assert_eq!(calib.state, InteractiveState::Sweeping);

        // 5. Feed 4 monotonic samples
        let n1 = calib.step_sample(-0.5).unwrap(); // sample 0
        assert!(n1.is_some());
        let n2 = calib.step_sample(-0.1).unwrap(); // sample 1
        assert!(n2.is_some());
        let n3 = calib.step_sample(0.2).unwrap(); // sample 2
        assert!(n3.is_some());
        let n4 = calib.step_sample(0.5).unwrap(); // sample 3 (final)
        assert!(n4.is_none());

        // State is now Completed
        assert_eq!(calib.state, InteractiveState::Completed);

        // 6. calib save
        let save_msg = calib.handle_command("calib save", 0.0, 0.0).unwrap();
        assert!(save_msg.contains("saved"));

        let result = calib.get_result().expect("result available");
        assert!(result.lut.validate_monotonicity());
    }
}
