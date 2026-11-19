//! # assembly — multi-axis actuator assembly and dynamic topology manager
//!
//! Provides [`ActuatorAssembly<M, S>`] managing $M$ motor channels and $S$ sensor channels
//! across runtime-reconfigurable kinematic topologies ([`TopologyPreset`]).

use crate::common::time::Clock;
use crate::kinematics::actuator::motor::MotorControl;
use crate::traits::sensor::Sensor;

/// Predefined multi-axis kinematic topology presets.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TopologyPreset {
    /// 4 independent 1-DOF joint axes.
    Four1Dof = 0,
    /// 2 independent 2-DOF coupled differential joint pairs ([0, 1] and [2, 3]).
    Two2Dof = 1,
    /// 1 1-DOF joint (motor 0) and 1 2-DOF differential joint pair (motors 1 and 2).
    One1DofOne2Dof = 2,
    /// 2 1-DOF joints (motors 0 and 1) and 1 2-DOF differential joint pair (motors 2 and 3).
    Two1DofOne2Dof = 3,
    /// Custom topology mapping driven by coupling bitmask.
    Custom = 0xFF,
}

impl TopologyPreset {
    /// Convert raw `u8` into `TopologyPreset`.
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => TopologyPreset::Four1Dof,
            1 => TopologyPreset::Two2Dof,
            2 => TopologyPreset::One1DofOne2Dof,
            3 => TopologyPreset::Two1DofOne2Dof,
            _ => TopologyPreset::Custom,
        }
    }
}

/// Flash-serializable configuration structure for topology and sensor mapping.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TopologyConfig {
    /// Active topology preset.
    pub preset: TopologyPreset,
    /// Routing map assigning sensor channel indices to motor/joint indices.
    pub sensor_map: [u8; 4],
    /// Coupling bitmask indicating coupled 2-DOF motor pairs.
    pub coupling_mask: u8,
}

impl Default for TopologyConfig {
    fn default() -> Self {
        Self {
            preset: TopologyPreset::Four1Dof,
            sensor_map: [0, 1, 2, 3],
            coupling_mask: 0x00,
        }
    }
}

impl TopologyConfig {
    /// Serialize configuration to a 6-byte raw array.
    pub fn to_bytes(&self) -> [u8; 6] {
        let mut b = [0u8; 6];
        b[0] = self.preset as u8;
        b[1..5].copy_from_slice(&self.sensor_map);
        b[5] = self.coupling_mask;
        b
    }

    /// Deserialize configuration from a 6-byte raw array.
    pub fn from_bytes(bytes: &[u8; 6]) -> Self {
        Self {
            preset: TopologyPreset::from_u8(bytes[0]),
            sensor_map: [bytes[1], bytes[2], bytes[3], bytes[4]],
            coupling_mask: bytes[5],
        }
    }
}

/// Telemetry state for an individual motor axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorAxisTelemetry {
    /// Target velocity or torque commanded to inner motor controller.
    pub target: f32,
    /// Measured or estimated motor current (e.g. q-axis or DC mA).
    pub current: f32,
    /// Current mechanical shaft angle in radians.
    pub shaft_angle: f32,
    /// Current mechanical shaft velocity in rad/s.
    pub shaft_velocity: f32,
    /// Assigned sensor channel index in the assembly's sensor pool.
    pub sensor_channel: u8,
    /// Configured maximum velocity limit in rad/s.
    pub velocity_limit: f32,
}

impl Default for MotorAxisTelemetry {
    fn default() -> Self {
        Self {
            target: 0.0,
            current: 0.0,
            shaft_angle: 0.0,
            shaft_velocity: 0.0,
            sensor_channel: 0,
            velocity_limit: 50.0,
        }
    }
}

/// Telemetry state for an individual joint axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointTelemetry {
    /// Target joint position setpoint in radians.
    pub target_pos: f32,
    /// Measured joint position in radians from assigned sensor channel.
    pub present_pos: f32,
    /// Measured joint velocity in rad/s from assigned sensor channel.
    pub present_vel: f32,
    /// Transmission gear reduction ratio (R).
    pub gear_ratio: f32,
    /// Backlash deadband compensation threshold in radians.
    pub backlash: f32,
    /// Outer position loop proportional gain.
    pub pid_p: f32,
    /// Outer position loop integral gain.
    pub pid_i: f32,
    /// Outer position loop derivative gain.
    pub pid_d: f32,
}

impl Default for JointTelemetry {
    fn default() -> Self {
        Self {
            target_pos: 0.0,
            present_pos: 0.0,
            present_vel: 0.0,
            gear_ratio: 1.0,
            backlash: 0.0,
            pid_p: 20.0,
            pid_i: 0.0,
            pid_d: 0.0,
        }
    }
}

/// Telemetry and control state for calibration operations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CalibTelemetry {
    /// Calibration command id (0=none, 1=start, 2=stop, 3=save).
    pub command: u8,
    /// Calibration state (0=idle, 1=running, 2=done, 3=error).
    pub state: u8,
    /// Progress indicator 0..100.
    pub progress: u8,
    /// Flash persistence status (0=clean, 1=dirty, 2=saved, 3=error).
    pub flash_status: u8,
}

/// Discrete-time PID state for an outer joint position loop.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct JointPidState {
    pub error_prev: f32,
    pub integral: f32,
    pub output_prev: f32,
    pub timestamp_prev: u32,
}

impl JointPidState {
    /// Reset internal integral and error history.
    pub fn reset(&mut self) {
        self.error_prev = 0.0;
        self.integral = 0.0;
        self.output_prev = 0.0;
        self.timestamp_prev = 0;
    }

    /// Update discrete PID and return velocity setpoint.
    pub fn update(&mut self, p: f32, i: f32, d: f32, error: f32, timestamp_us: u32) -> f32 {
        let dt = if self.timestamp_prev == 0 {
            1e-3
        } else {
            let diff = timestamp_us.wrapping_sub(self.timestamp_prev);
            let dt_s = diff as f32 * 1e-6;
            if dt_s <= 0.0 || dt_s > 0.5 {
                1e-3
            } else {
                dt_s
            }
        };
        self.timestamp_prev = timestamp_us;

        let u_p = p * error;
        // Tustin trapezoidal integration
        self.integral += (i * dt * 0.5) * (error + self.error_prev);
        let u_d = if dt > 1e-6 {
            d * (error - self.error_prev) / dt
        } else {
            0.0
        };
        self.error_prev = error;

        u_p + self.integral + u_d
    }
}

/// Multi-axis actuator assembly supervisor managing $M$ motor channels and $S$ sensor channels.
///
/// Coordinates inner high-speed motor commutation loops with outer cascaded joint position loops,
/// multiplexes sensor channels, and dynamically schedules 1-DOF and 2-DOF coupled differential
/// kinematics across configurable topology presets.
pub struct ActuatorAssembly<const M: usize = 4, const S: usize = 4> {
    /// Measured DC bus voltage in Volts.
    pub vbus_voltage: f32,
    /// Active topology preset mode (0=Four1Dof, 1=Two2Dof, 2=One1DofOne2Dof, 3=Two1DofOne2Dof, 0xFF=Custom).
    pub topology_mode: u8,
    /// Active topology configuration (preset, sensor map, coupling mask).
    pub topology: TopologyConfig,
    /// Per-motor axis telemetry and control targets.
    pub motors: [MotorAxisTelemetry; M],
    /// Per-joint axis telemetry, targets, and controller parameters.
    pub joints: [JointTelemetry; M],
    /// Calibration supervisor state and progress.
    pub calib: CalibTelemetry,
    /// Sampled sensor pool angles in radians.
    pub sensor_angles: [f32; S],
    /// Sampled sensor pool velocities in rad/s.
    pub sensor_velocities: [f32; S],
    /// Internal discrete PID controller states for joints.
    pub pid_states: [JointPidState; M],
    /// Decimation ratio of outer joint loops relative to inner FOC steps.
    pub outer_downsample: u32,
    /// Internal downsample counter.
    pub motion_cnt: u32,
}

impl<const M: usize, const S: usize> Default for ActuatorAssembly<M, S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const M: usize, const S: usize> ActuatorAssembly<M, S> {
    /// Construct a new actuator assembly with default settings.
    pub fn new() -> Self {
        let mut motors = [MotorAxisTelemetry::default(); M];
        for (i, m) in motors.iter_mut().enumerate() {
            m.sensor_channel = (i as u8).min((S.saturating_sub(1)) as u8);
        }

        Self {
            vbus_voltage: 0.0,
            topology_mode: TopologyPreset::Four1Dof as u8,
            topology: TopologyConfig::default(),
            motors,
            joints: [JointTelemetry::default(); M],
            calib: CalibTelemetry::default(),
            sensor_angles: [0.0; S],
            sensor_velocities: [0.0; S],
            pid_states: [JointPidState::default(); M],
            outer_downsample: 10,
            motion_cnt: 0,
        }
    }

    /// Set outer loop decimation factor.
    pub fn with_downsample(mut self, downsample: u32) -> Self {
        self.outer_downsample = if downsample == 0 { 1 } else { downsample };
        self
    }

    /// Switch active kinematics topology preset.
    pub fn set_topology(&mut self, preset: TopologyPreset) {
        self.topology.preset = preset;
        self.topology_mode = preset as u8;
        match preset {
            TopologyPreset::Four1Dof => {
                self.topology.coupling_mask = 0x00;
            }
            TopologyPreset::Two2Dof => {
                self.topology.coupling_mask = 0x03;
            }
            TopologyPreset::One1DofOne2Dof => {
                self.topology.coupling_mask = 0x04;
            }
            TopologyPreset::Two1DofOne2Dof => {
                self.topology.coupling_mask = 0x02;
            }
            TopologyPreset::Custom => {}
        }
    }

    /// Get active kinematics topology preset.
    #[inline]
    pub fn topology(&self) -> TopologyPreset {
        self.topology.preset
    }

    /// Set full topology configuration including sensor map and coupling bitmask.
    pub fn set_topology_config(&mut self, config: TopologyConfig) {
        self.topology = config;
        self.topology_mode = config.preset as u8;
        for (i, &s) in config.sensor_map.iter().enumerate().take(M) {
            self.motors[i].sensor_channel = s;
        }
    }

    /// Configure sensor channel multiplexing for a specific motor axis.
    pub fn set_sensor_mapping(&mut self, motor_idx: usize, sensor_channel: usize) {
        if motor_idx < 4 {
            self.topology.sensor_map[motor_idx] = sensor_channel as u8;
        }
        if motor_idx < M {
            self.motors[motor_idx].sensor_channel = sensor_channel as u8;
        }
    }

    /// Set target angle in radians for a joint axis.
    #[inline]
    pub fn set_joint_target(&mut self, joint_idx: usize, target_rad: f32) {
        if joint_idx < M {
            self.joints[joint_idx].target_pos = target_rad;
        }
    }

    /// Get measured angle in radians for a joint axis.
    #[inline]
    pub fn joint_angle(&self, joint_idx: usize) -> f32 {
        if joint_idx < M {
            self.joints[joint_idx].present_pos
        } else {
            0.0
        }
    }

    /// Get measured velocity in rad/s for a joint axis.
    #[inline]
    pub fn joint_velocity(&self, joint_idx: usize) -> f32 {
        if joint_idx < M {
            self.joints[joint_idx].present_vel
        } else {
            0.0
        }
    }

    /// Directly update cached angle and velocity for a sensor channel.
    pub fn set_sensor_state(&mut self, sensor_channel: usize, angle: f32, velocity: f32) {
        if sensor_channel < S {
            self.sensor_angles[sensor_channel] = angle;
            self.sensor_velocities[sensor_channel] = velocity;
        }
    }

    /// Restore topology configuration from flash calibration data block.
    pub fn load_from_flash(&mut self, data: &crate::kinematics::calibration::storage::FlashCalibrationData) {
        self.set_topology_config(data.topology);
    }

    /// Store active topology configuration into flash calibration data block.
    pub fn save_to_flash(&self, data: &mut crate::kinematics::calibration::storage::FlashCalibrationData) {
        data.topology = self.topology;
    }

    /// Step all sensors, outer cascaded joint loops, and inner motor loops.
    pub fn step_all<MOT, SEN, C>(
        &mut self,
        motors: &mut [MOT],
        sensors: &mut [SEN],
        clock: &C,
    )
    where
        MOT: MotorControl<C>,
        SEN: Sensor<C>,
        C: Clock,
    {
        // 1. Update all sensors in the sensor pool
        for (i, sensor) in sensors.iter_mut().enumerate().take(S) {
            sensor.update(clock);
            self.sensor_angles[i] = sensor.get_angle();
            self.sensor_velocities[i] = sensor.get_velocity();
        }

        // 2. Dispatch motors and outer joint loops
        self.step_motors(motors, clock);
    }

    /// Execute outer joint loops (when decimated) and inner motor loops using currently cached sensor states.
    pub fn step_motors<MOT, C>(
        &mut self,
        motors: &mut [MOT],
        clock: &C,
    )
    where
        MOT: MotorControl<C>,
        C: Clock,
    {
        self.motion_cnt += 1;
        if self.motion_cnt >= self.outer_downsample {
            self.motion_cnt = 0;
            self.step_outer(motors, clock);
        }

        // Always step high-speed inner FOC commutation loops
        for motor in motors.iter_mut().take(M) {
            motor.loop_foc(clock);
        }

        // Refresh telemetry
        for (i, motor) in motors.iter_mut().enumerate().take(M) {
            self.motors[i].shaft_angle = motor.shaft_angle();
            self.motors[i].shaft_velocity = motor.shaft_velocity();
        }
    }

    /// Execute medium-speed outer joint position loop based on active topology.
    pub fn step_outer<MOT, C>(
        &mut self,
        motors: &mut [MOT],
        clock: &C,
    )
    where
        MOT: MotorControl<C>,
        C: Clock,
    {
        match self.topology.preset {
            TopologyPreset::Four1Dof => {
                for i in 0..M.min(motors.len()) {
                    self.step_1dof_joint(i, i, motors, clock);
                }
            }
            TopologyPreset::Two2Dof => {
                if M >= 2 && motors.len() >= 2 {
                    self.step_2dof_pair(0, 1, 0, 1, motors, clock);
                }
                if M >= 4 && motors.len() >= 4 {
                    self.step_2dof_pair(2, 3, 2, 3, motors, clock);
                }
            }
            TopologyPreset::One1DofOne2Dof => {
                if M >= 1 && !motors.is_empty() {
                    self.step_1dof_joint(0, 0, motors, clock);
                }
                if M >= 3 && motors.len() >= 3 {
                    self.step_2dof_pair(1, 2, 1, 2, motors, clock);
                }
                if M >= 4 && motors.len() >= 4 {
                    self.step_1dof_joint(3, 3, motors, clock);
                }
            }
            TopologyPreset::Two1DofOne2Dof => {
                if M >= 1 && !motors.is_empty() {
                    self.step_1dof_joint(0, 0, motors, clock);
                }
                if M >= 2 && motors.len() >= 2 {
                    self.step_1dof_joint(1, 1, motors, clock);
                }
                if M >= 4 && motors.len() >= 4 {
                    self.step_2dof_pair(2, 3, 2, 3, motors, clock);
                }
            }
            TopologyPreset::Custom => {
                // Dispatch based on coupling bitmask:
                // Bit 0: pair (0, 1) coupled
                // Bit 1: pair (2, 3) coupled
                // Bit 2: pair (1, 2) coupled
                let mask = self.topology.coupling_mask;
                let mut handled = [false; M];

                if (mask & 0x01) != 0 && M >= 2 && motors.len() >= 2 {
                    self.step_2dof_pair(0, 1, 0, 1, motors, clock);
                    handled[0] = true;
                    handled[1] = true;
                }
                if (mask & 0x02) != 0 && M >= 4 && motors.len() >= 4 {
                    self.step_2dof_pair(2, 3, 2, 3, motors, clock);
                    handled[2] = true;
                    handled[3] = true;
                }
                if (mask & 0x04) != 0 && M >= 3 && motors.len() >= 3 && !handled[1] && !handled[2] {
                    self.step_2dof_pair(1, 2, 1, 2, motors, clock);
                    handled[1] = true;
                    handled[2] = true;
                }

                for i in 0..M.min(motors.len()) {
                    if !handled[i] {
                        self.step_1dof_joint(i, i, motors, clock);
                    }
                }
            }
        }
    }

    fn step_1dof_joint<MOT, C>(
        &mut self,
        motor_idx: usize,
        joint_idx: usize,
        motors: &mut [MOT],
        clock: &C,
    )
    where
        MOT: MotorControl<C>,
        C: Clock,
    {
        if motor_idx >= motors.len() || joint_idx >= M {
            return;
        }

        let s = if motor_idx < 4 {
            self.topology.sensor_map[motor_idx] as usize
        } else {
            motor_idx
        };

        if s < S {
            self.joints[joint_idx].present_pos = self.sensor_angles[s];
            self.joints[joint_idx].present_vel = self.sensor_velocities[s];
        }

        let err = self.joints[joint_idx].target_pos - self.joints[joint_idx].present_pos;
        let now = clock.now_us();
        let p = self.joints[joint_idx].pid_p;
        let i = self.joints[joint_idx].pid_i;
        let d = self.joints[joint_idx].pid_d;
        let v_joint = self.pid_states[joint_idx].update(p, i, d, err, now);

        let r = self.joints[joint_idx].gear_ratio;
        let gear_ratio = if libm::fabsf(r) < 1e-4 { 1.0 } else { r };
        let v_mot = v_joint * gear_ratio;

        let v_lim = motors[motor_idx].velocity_limit();
        let limit = if v_lim > 0.0 {
            v_lim
        } else {
            self.motors[motor_idx].velocity_limit
        };

        let v_cmd = if v_mot > limit {
            limit
        } else if v_mot < -limit {
            -limit
        } else {
            v_mot
        };

        self.motors[motor_idx].target = v_cmd;
        motors[motor_idx].move_motor(v_cmd, clock);
    }

    fn step_2dof_pair<MOT, C>(
        &mut self,
        m1_idx: usize,
        m2_idx: usize,
        j1_idx: usize,
        j2_idx: usize,
        motors: &mut [MOT],
        clock: &C,
    )
    where
        MOT: MotorControl<C>,
        C: Clock,
    {
        if m1_idx >= motors.len() || m2_idx >= motors.len() || j1_idx >= M || j2_idx >= M {
            return;
        }

        let s1 = if m1_idx < 4 {
            self.topology.sensor_map[m1_idx] as usize
        } else {
            m1_idx
        };
        let s2 = if m2_idx < 4 {
            self.topology.sensor_map[m2_idx] as usize
        } else {
            m2_idx
        };

        if s1 < S {
            self.joints[j1_idx].present_pos = self.sensor_angles[s1];
            self.joints[j1_idx].present_vel = self.sensor_velocities[s1];
        }
        if s2 < S {
            self.joints[j2_idx].present_pos = self.sensor_angles[s2];
            self.joints[j2_idx].present_vel = self.sensor_velocities[s2];
        }

        let err_alpha = self.joints[j1_idx].target_pos - self.joints[j1_idx].present_pos;
        let err_beta = self.joints[j2_idx].target_pos - self.joints[j2_idx].present_pos;

        let now = clock.now_us();
        let v_alpha = self.pid_states[j1_idx].update(
            self.joints[j1_idx].pid_p,
            self.joints[j1_idx].pid_i,
            self.joints[j1_idx].pid_d,
            err_alpha,
            now,
        );
        let v_beta = self.pid_states[j2_idx].update(
            self.joints[j2_idx].pid_p,
            self.joints[j2_idx].pid_i,
            self.joints[j2_idx].pid_d,
            err_beta,
            now,
        );

        let r1 = self.joints[j1_idx].gear_ratio;
        let r2 = self.joints[j2_idx].gear_ratio;
        let gear_r1 = if libm::fabsf(r1) < 1e-4 { 1.0 } else { r1 };
        let gear_r2 = if libm::fabsf(r2) < 1e-4 { 1.0 } else { r2 };

        let v_m1 = gear_r1 * v_alpha + gear_r2 * v_beta;
        let v_m2 = gear_r1 * v_alpha - gear_r2 * v_beta;

        let lim1 = {
            let l = motors[m1_idx].velocity_limit();
            if l > 0.0 {
                l
            } else {
                self.motors[m1_idx].velocity_limit
            }
        };
        let lim2 = {
            let l = motors[m2_idx].velocity_limit();
            if l > 0.0 {
                l
            } else {
                self.motors[m2_idx].velocity_limit
            }
        };

        let cmd1 = if v_m1 > lim1 {
            lim1
        } else if v_m1 < -lim1 {
            -lim1
        } else {
            v_m1
        };

        let cmd2 = if v_m2 > lim2 {
            lim2
        } else if v_m2 < -lim2 {
            -lim2
        } else {
            v_m2
        };

        self.motors[m1_idx].target = cmd1;
        self.motors[m2_idx].target = cmd2;
        motors[m1_idx].move_motor(cmd1, clock);
        motors[m2_idx].move_motor(cmd2, clock);
    }
}
