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
use super::{CalibCommand, CalibState};

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
    pub state: CalibState,
    pub progress: u8,
    pub aborted: bool,
}

impl AutonomousCalibrator {
    /// Create a new autonomous calibrator with configuration.
    pub fn new(config: CalibrationConfig) -> Self {
        Self {
            config,
            state: CalibState::Idle,
            progress: 0,
            aborted: false,
        }
    }

    /// Process a calibration command trigger.
    pub fn handle_command(&mut self, cmd: CalibCommand) -> Result<CalibState, CalibrationError> {
        match cmd {
            CalibCommand::StartAutonomous => {
                self.aborted = false;
                self.state = CalibState::Running;
                self.progress = 0;
                Ok(self.state)
            }
            CalibCommand::Abort => {
                self.abort();
                Ok(self.state)
            }
            CalibCommand::Reset => {
                self.reset();
                Ok(self.state)
            }
            CalibCommand::None => Ok(self.state),
            _ => Err(CalibrationError::InvalidState),
        }
    }

    /// Abort active calibration routine.
    pub fn abort(&mut self) {
        self.aborted = true;
        self.state = CalibState::Idle;
        self.progress = 0;
    }

    /// Reset calibrator state to Idle.
    pub fn reset(&mut self) {
        self.aborted = false;
        self.state = CalibState::Idle;
        self.progress = 0;
    }

    /// Returns true if abort was requested.
    pub fn is_aborted(&self) -> bool {
        self.aborted
    }

    /// Autonomous limit discovery lifecycle routine.
    /// Finds negative limit, positive limit, and verifies limits.
    pub fn discover_limits<M, C>(
        &mut self,
        motor: &mut M,
        clock: &C,
    ) -> Result<(f32, f32), CalibrationError>
    where
        M: MotorControl<C>,
        C: Clock,
    {
        if self.aborted {
            return Err(CalibrationError::InvalidState);
        }
        self.state = CalibState::Running;
        self.progress = 10;

        let mot_min = match self.find_endstop(motor, clock, -self.config.sweep_velocity) {
            Ok(min) => min,
            Err(e) => {
                self.state = CalibState::Error;
                return Err(e);
            }
        };

        if self.aborted {
            self.state = CalibState::Idle;
            return Err(CalibrationError::InvalidState);
        }
        self.progress = 50;

        let mot_max = match self.find_endstop(motor, clock, self.config.sweep_velocity) {
            Ok(max) => max,
            Err(e) => {
                self.state = CalibState::Error;
                return Err(e);
            }
        };

        if self.aborted {
            self.state = CalibState::Idle;
            return Err(CalibrationError::InvalidState);
        }

        if mot_max <= mot_min {
            self.state = CalibState::Error;
            return Err(CalibrationError::InvalidLimits);
        }

        self.progress = 100;
        self.state = CalibState::Done;
        Ok((mot_min, mot_max))
    }

    /// Autonomous limit homing lifecycle routine.
    /// Drives actuator in direction (+1.0 or -1.0) until stall endstop is confirmed.
    pub fn home_to_limit<M, C>(
        &mut self,
        motor: &mut M,
        clock: &C,
        direction: f32,
    ) -> Result<f32, CalibrationError>
    where
        M: MotorControl<C>,
        C: Clock,
    {
        if self.aborted {
            return Err(CalibrationError::InvalidState);
        }
        self.state = CalibState::Running;
        let vel = if direction < 0.0 {
            -self.config.sweep_velocity.abs()
        } else {
            self.config.sweep_velocity.abs()
        };
        let limit = match self.find_endstop(motor, clock, vel) {
            Ok(l) => l,
            Err(e) => {
                self.state = CalibState::Error;
                return Err(e);
            }
        };
        self.state = CalibState::Done;
        Ok(limit)
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
        &mut self,
        motor: &mut M,
        sensor: &mut SOut,
        clock: &C,
    ) -> Result<CalibrationResult<N>, CalibrationError>
    where
        M: MotorControl<C>,
        C: Clock,
        SOut: Sensor<C>,
    {
        if self.aborted {
            return Err(CalibrationError::InvalidState);
        }
        self.state = CalibState::Running;
        self.progress = 5;

        // 1. Discover negative endstop (mot_min)
        let mot_min = match self.find_endstop(motor, clock, -self.config.sweep_velocity) {
            Ok(min) => min,
            Err(e) => {
                self.state = CalibState::Error;
                return Err(e);
            }
        };

        if self.aborted {
            self.state = CalibState::Idle;
            return Err(CalibrationError::InvalidState);
        }
        self.progress = 35;

        // 2. Discover positive endstop (mot_max)
        let mot_max = match self.find_endstop(motor, clock, self.config.sweep_velocity) {
            Ok(max) => max,
            Err(e) => {
                self.state = CalibState::Error;
                return Err(e);
            }
        };

        if self.aborted {
            self.state = CalibState::Idle;
            return Err(CalibrationError::InvalidState);
        }
        self.progress = 65;

        if mot_max <= mot_min {
            self.state = CalibState::Error;
            return Err(CalibrationError::InvalidLimits);
        }

        // 3. Sweep across discovered range and sample
        let delta = (mot_max - mot_min) / ((N - 1) as f32);
        let mut table = [0.0f32; N];

        for (i, slot) in table.iter_mut().enumerate() {
            if self.aborted {
                self.state = CalibState::Idle;
                return Err(CalibrationError::InvalidState);
            }
            let target_mot = mot_min + (i as f32) * delta;
            motor.move_motor(target_mot, clock);
            motor.loop_foc(clock);
            sensor.update(clock);
            *slot = sensor.get_angle();
            self.progress = (65 + (i * 30 / N) as u8).min(95);
        }

        let lut = LutTransmission::new(mot_min, mot_max, table);
        if !lut.validate_monotonicity() {
            self.state = CalibState::Error;
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

        self.progress = 100;
        self.state = CalibState::Done;

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
