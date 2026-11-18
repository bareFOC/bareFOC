//! # calibration::autonomous — 1-call autonomous on-device calibration
//!
//! Automatically discovers mechanical limits via stall detection, samples transmission
//! transfer functions, validates monotonicity, constructs uniform [`LutTransmission`] models,
//! and packs 64-byte aligned [`FlashCalibrationData`] blocks.

use crate::common::time::Clock;
use crate::traits::sensor::Sensor;
use crate::kinematics::actuator::motor::MotorControl;
use crate::kinematics::calibration::storage::FlashCalibrationData;
use crate::kinematics::transmission::lut::LutTransmission;

/// Calibration errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationError {
    /// Actuator failed to reach stall or timed out.
    StallTimeout,
    /// Minimum limit is greater than or equal to maximum limit.
    InvalidLimits,
    /// Measured output trajectory is not monotonic.
    NonMonotonic,
    /// Mechanical endstop or sensor error during sweep.
    HardwareFault,
    /// Calibration not started or invalid sequence.
    InvalidState,
}

/// Configuration parameters for autonomous calibration routine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CalibrationConfig {
    /// Safe velocity for limit discovery sweep (rad/s).
    pub sweep_velocity: f32,
    /// Motor current or voltage limit during limit probing.
    pub probe_limit: f32,
    /// Velocity threshold below which stall is declared (rad/s).
    pub stall_vel_threshold: f32,
    /// Number of successive stall samples required to confirm endstop.
    pub stall_confirm_samples: usize,
    /// Maximum search timeout in microseconds.
    pub timeout_us: u32,
}

impl Default for CalibrationConfig {
    fn default() -> Self {
        Self {
            sweep_velocity: 1.0,
            probe_limit: 2.0,
            stall_vel_threshold: 0.05,
            stall_confirm_samples: 50,
            timeout_us: 10_000_000, // 10 seconds
        }
    }
}

/// Result of an autonomous calibration run.
#[derive(Clone, Debug, PartialEq)]
pub struct CalibrationResult<const N: usize> {
    /// Monotonic uniform LUT transmission.
    pub lut: LutTransmission<N>,
    /// Flash configuration header.
    pub config: FlashCalibrationData,
    /// Minimum discovered motor angle (rad).
    pub mot_min: f32,
    /// Maximum discovered motor angle (rad).
    pub mot_max: f32,
    /// Output joint angle at `mot_min` (rad).
    pub joint_min: f32,
    /// Output joint angle at `mot_max` (rad).
    pub joint_max: f32,
}

/// Autonomous 1-call calibrator.
pub struct AutonomousCalibrator {
    pub config: CalibrationConfig,
}

impl AutonomousCalibrator {
    /// Create a new autonomous calibrator with configuration.
    pub fn new(config: CalibrationConfig) -> Self {
        Self { config }
    }

    /// Fit a monotonic LUT transmission and pack flash configuration directly
    /// from known or measured mechanical limits and a sampling closure.
    pub fn fit_table<const N: usize, F>(
        &self,
        mot_min: f32,
        mot_max: f32,
        mut sample_fn: F,
    ) -> Result<CalibrationResult<N>, CalibrationError>
    where
        F: FnMut(f32) -> f32,
    {
        if N < 2 || mot_max <= mot_min {
            return Err(CalibrationError::InvalidLimits);
        }

        let delta = (mot_max - mot_min) / ((N - 1) as f32);
        let mut table = [0.0f32; N];

        for (i, slot) in table.iter_mut().enumerate() {
            let mot_angle = mot_min + (i as f32) * delta;
            *slot = sample_fn(mot_angle);
        }

        let lut = LutTransmission::new(mot_min, mot_max, table);
        if !lut.validate_monotonicity() {
            return Err(CalibrationError::NonMonotonic);
        }

        let joint_min = table[0];
        let joint_max = table[N - 1];

        let mut flash_cfg = FlashCalibrationData {
            joint_min,
            joint_max,
            lut_mot_min: mot_min,
            lut_mot_max: mot_max,
            lut_points: N as u16,
            ..Default::default()
        };
        flash_cfg.update_checksum();

        Ok(CalibrationResult {
            lut,
            config: flash_cfg,
            mot_min,
            mot_max,
            joint_min,
            joint_max,
        })
    }

    /// Execute a simulated or automated sweep given known motor limits and an output sensor.
    pub fn sweep_and_fit<const N: usize, S, C>(
        &self,
        mot_min: f32,
        mot_max: f32,
        sensor: &mut S,
        clock: &C,
        mut move_to: impl FnMut(f32),
    ) -> Result<CalibrationResult<N>, CalibrationError>
    where
        S: Sensor<C>,
        C: Clock,
    {
        if N < 2 || mot_max <= mot_min {
            return Err(CalibrationError::InvalidLimits);
        }

        let delta = (mot_max - mot_min) / ((N - 1) as f32);
        let mut table = [0.0f32; N];

        for (i, slot) in table.iter_mut().enumerate() {
            let mot_angle = mot_min + (i as f32) * delta;
            move_to(mot_angle);
            sensor.update(clock);
            *slot = sensor.get_angle();
        }

        let lut = LutTransmission::new(mot_min, mot_max, table);
        if !lut.validate_monotonicity() {
            return Err(CalibrationError::NonMonotonic);
        }

        let joint_min = table[0];
        let joint_max = table[N - 1];

        let mut flash_cfg = FlashCalibrationData {
            joint_min,
            joint_max,
            lut_mot_min: mot_min,
            lut_mot_max: mot_max,
            lut_points: N as u16,
            ..Default::default()
        };
        flash_cfg.update_checksum();

        Ok(CalibrationResult {
            lut,
            config: flash_cfg,
            mot_min,
            mot_max,
            joint_min,
            joint_max,
        })
    }

    /// Full end-to-end 1-call autonomous calibration routine on live motor and sensor.
    pub fn auto_calibrate<const N: usize, M, C, SOut>(
        &self,
        motor: &mut M,
        sensor: &mut SOut,
        clock: &C,
    ) -> Result<CalibrationResult<N>, CalibrationError>
    where
        M: MotorControl<C>,
        C: Clock,
        SOut: Sensor<C>,
    {
        // 1. Discover negative endstop (mot_min)
        let mot_min = self.find_endstop(motor, clock, -self.config.sweep_velocity)?;

        // 2. Discover positive endstop (mot_max)
        let mot_max = self.find_endstop(motor, clock, self.config.sweep_velocity)?;

        if mot_max <= mot_min {
            return Err(CalibrationError::InvalidLimits);
        }

        // 3. Sweep across discovered range and sample
        let delta = (mot_max - mot_min) / ((N - 1) as f32);
        let mut table = [0.0f32; N];

        for (i, slot) in table.iter_mut().enumerate() {
            let target_mot = mot_min + (i as f32) * delta;
            motor.move_motor(target_mot, clock);
            motor.loop_foc(clock);
            sensor.update(clock);
            *slot = sensor.get_angle();
        }

        let lut = LutTransmission::new(mot_min, mot_max, table);
        if !lut.validate_monotonicity() {
            return Err(CalibrationError::NonMonotonic);
        }

        let joint_min = table[0];
        let joint_max = table[N - 1];

        let mut flash_cfg = FlashCalibrationData {
            joint_min,
            joint_max,
            lut_mot_min: mot_min,
            lut_mot_max: mot_max,
            lut_points: N as u16,
            ..Default::default()
        };
        flash_cfg.update_checksum();

        Ok(CalibrationResult {
            lut,
            config: flash_cfg,
            mot_min,
            mot_max,
            joint_min,
            joint_max,
        })
    }

    fn find_endstop<M, C>(
        &self,
        motor: &mut M,
        clock: &C,
        velocity: f32,
    ) -> Result<f32, CalibrationError>
    where
        M: MotorControl<C>,
        C: Clock,
    {
        let start_time = clock.now_us();
        let mut stall_count = 0;

        loop {
            motor.move_motor(velocity, clock);
            motor.loop_foc(clock);

            let vel = motor.shaft_velocity().abs();
            if vel < self.config.stall_vel_threshold {
                stall_count += 1;
                if stall_count >= self.config.stall_confirm_samples {
                    // Confirmed stall limit
                    return Ok(motor.shaft_angle());
                }
            } else {
                stall_count = 0;
            }

            let elapsed = clock.now_us().wrapping_sub(start_time);
            if elapsed > self.config.timeout_us {
                return Err(CalibrationError::StallTimeout);
            }
        }
    }
}
