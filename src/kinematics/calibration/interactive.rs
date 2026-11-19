//! # calibration::interactive — guided interactive calibration state machine
//!
//! Provides a step-by-step state machine with serial command bindings
//! (`calib start`, `calib min`, `calib max`, `calib sweep`, `calib save`)
//! suitable for serial console or graphical user interfaces.

use super::autonomous::{CalibrationError, CalibrationResult};
use super::storage::FlashCalibrationData;
use super::CalibCommand;
use crate::kinematics::transmission::lut::LutTransmission;

/// Interactive calibration state machine phases.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InteractiveState {
    /// Idle, awaiting `calib start`.
    Idle,
    /// Prompting operator to position actuator at minimum limit (`calib min`).
    SettingMin,
    /// Prompting operator to position actuator at maximum limit (`calib max`).
    SettingMax,
    /// Limits acquired, ready to initiate sampling sweep (`calib sweep`).
    ReadyToSweep,
    /// Sampling sweep in progress.
    Sweeping,
    /// Calibration successfully completed and verified, awaiting `calib save`.
    Completed,
    /// Error state.
    Error(CalibrationError),
}

/// Interactive guided calibrator.
pub struct InteractiveCalibrator<const N: usize> {
    pub state: InteractiveState,
    pub mot_min: f32,
    pub mot_max: f32,
    pub joint_min: f32,
    pub joint_max: f32,
    pub samples: [f32; N],
    pub sweep_index: usize,
    pub flash_config: Option<FlashCalibrationData>,
}

impl<const N: usize> Default for InteractiveCalibrator<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> InteractiveCalibrator<N> {
    /// Create a new interactive calibrator.
    pub fn new() -> Self {
        Self {
            state: InteractiveState::Idle,
            mot_min: 0.0,
            mot_max: 0.0,
            joint_min: 0.0,
            joint_max: 0.0,
            samples: [0.0; N],
            sweep_index: 0,
            flash_config: None,
        }
    }

    /// Process an ASCII command string and update the calibration state machine.
    ///
    /// Supported commands:
    /// - `"calib start"`: Start calibration sequence (transitions to `SettingMin`).
    /// - `"calib min"`: Lock current position as minimum mechanical limit.
    /// - `"calib max"`: Lock current position as maximum mechanical limit.
    /// - `"calib sweep"`: Begin or finalize sampling sweep.
    /// - `"calib save"`: Finalize and pack flash configuration.
    /// - `"calib reset"`: Reset state machine back to `Idle`.
    /// - `"calib status"`: Return current state description.
    pub fn handle_command(
        &mut self,
        cmd: &str,
        current_mot: f32,
        current_joint: f32,
    ) -> Result<&'static str, CalibrationError> {
        let trimmed = cmd.trim();

        if trimmed.eq_ignore_ascii_case("calib reset") {
            self.state = InteractiveState::Idle;
            self.sweep_index = 0;
            self.flash_config = None;
            return Ok("Calibration reset to Idle");
        }

        if trimmed.eq_ignore_ascii_case("calib status") {
            return Ok(match self.state {
                InteractiveState::Idle => "Status: Idle",
                InteractiveState::SettingMin => "Status: SettingMin — move to min and send 'calib min'",
                InteractiveState::SettingMax => "Status: SettingMax — move to max and send 'calib max'",
                InteractiveState::ReadyToSweep => "Status: ReadyToSweep — send 'calib sweep'",
                InteractiveState::Sweeping => "Status: Sweeping in progress",
                InteractiveState::Completed => "Status: Completed — send 'calib save'",
                InteractiveState::Error(_) => "Status: Error",
            });
        }

        match self.state {
            InteractiveState::Idle => {
                if trimmed.eq_ignore_ascii_case("calib start") {
                    self.state = InteractiveState::SettingMin;
                    self.sweep_index = 0;
                    self.flash_config = None;
                    Ok("Calibration started. Move joint to MINIMUM limit and send 'calib min'")
                } else {
                    Err(CalibrationError::InvalidState)
                }
            }
            InteractiveState::SettingMin => {
                if trimmed.eq_ignore_ascii_case("calib min") {
                    self.mot_min = current_mot;
                    self.joint_min = current_joint;
                    self.state = InteractiveState::SettingMax;
                    Ok("Minimum limit locked. Move joint to MAXIMUM limit and send 'calib max'")
                } else {
                    Err(CalibrationError::InvalidState)
                }
            }
            InteractiveState::SettingMax => {
                if trimmed.eq_ignore_ascii_case("calib max") {
                    if current_mot <= self.mot_min {
                        self.state = InteractiveState::Error(CalibrationError::InvalidLimits);
                        return Err(CalibrationError::InvalidLimits);
                    }
                    self.mot_max = current_mot;
                    self.joint_max = current_joint;
                    self.state = InteractiveState::ReadyToSweep;
                    Ok("Maximum limit locked. Send 'calib sweep' to begin sampling")
                } else {
                    Err(CalibrationError::InvalidState)
                }
            }
            InteractiveState::ReadyToSweep => {
                if trimmed.eq_ignore_ascii_case("calib sweep") {
                    self.state = InteractiveState::Sweeping;
                    self.sweep_index = 0;
                    Ok("Sweep initiated. Feed samples via step_sample()")
                } else {
                    Err(CalibrationError::InvalidState)
                }
            }
            InteractiveState::Sweeping => {
                // If user issues "calib sweep" while in sweeping, check if complete
                if trimmed.eq_ignore_ascii_case("calib sweep") {
                    if self.sweep_index >= N {
                        self.finalize_sweep()
                    } else {
                        Ok("Sweeping in progress")
                    }
                } else {
                    Err(CalibrationError::InvalidState)
                }
            }
            InteractiveState::Completed => {
                if trimmed.eq_ignore_ascii_case("calib save") {
                    Ok("Calibration saved to flash configuration block")
                } else {
                    Err(CalibrationError::InvalidState)
                }
            }
            InteractiveState::Error(err) => Err(err),
        }
    }

    /// Feed a sampled joint angle during the sweep phase.
    /// Returns the target motor angle for the next step, or `None` when sweep is complete.
    pub fn step_sample(&mut self, sampled_joint: f32) -> Result<Option<f32>, CalibrationError> {
        if self.state != InteractiveState::Sweeping {
            return Err(CalibrationError::InvalidState);
        }

        if self.sweep_index < N {
            self.samples[self.sweep_index] = sampled_joint;
            self.sweep_index += 1;
        }

        if self.sweep_index >= N {
            let _ = self.finalize_sweep()?;
            Ok(None)
        } else {
            let delta = (self.mot_max - self.mot_min) / ((N - 1) as f32);
            let next_mot = self.mot_min + (self.sweep_index as f32) * delta;
            Ok(Some(next_mot))
        }
    }

    /// Finalize sweep, validate monotonicity, and build flash configuration.
    pub fn finalize_sweep(&mut self) -> Result<&'static str, CalibrationError> {
        let lut = LutTransmission::new(self.mot_min, self.mot_max, self.samples);
        if !lut.validate_monotonicity() {
            self.state = InteractiveState::Error(CalibrationError::NonMonotonic);
            return Err(CalibrationError::NonMonotonic);
        }

        let mut flash_cfg = FlashCalibrationData {
            joint_min: self.joint_min,
            joint_max: self.joint_max,
            lut_mot_min: self.mot_min,
            lut_mot_max: self.mot_max,
            lut_points: N as u16,
            ..Default::default()
        };
        flash_cfg.update_checksum();

        self.flash_config = Some(flash_cfg);
        self.state = InteractiveState::Completed;
        Ok("Sweep complete and monotonic. Ready to save ('calib save')")
    }

    /// Return the generated [`CalibrationResult`] if calibration is complete.
    pub fn get_result(&self) -> Option<CalibrationResult<N>> {
        if self.state == InteractiveState::Completed {
            let config = self.flash_config?;
            let lut = LutTransmission::new(self.mot_min, self.mot_max, self.samples);
            Some(CalibrationResult {
                lut,
                config,
                mot_min: self.mot_min,
                mot_max: self.mot_max,
                joint_min: self.joint_min,
                joint_max: self.joint_max,
            })
        } else {
            None
        }
    }

    /// Process a bus calibration command trigger (`CalibCommand`).
    pub fn handle_calib_command(
        &mut self,
        cmd: CalibCommand,
        current_mot: f32,
        current_joint: f32,
    ) -> Result<InteractiveState, CalibrationError> {
        match cmd {
            CalibCommand::StartInteractive | CalibCommand::StartAutonomous => {
                self.state = InteractiveState::SettingMin;
                self.sweep_index = 0;
                self.flash_config = None;
                Ok(self.state)
            }
            CalibCommand::SetMin => {
                if self.state != InteractiveState::SettingMin {
                    return Err(CalibrationError::InvalidState);
                }
                self.mot_min = current_mot;
                self.joint_min = current_joint;
                self.state = InteractiveState::SettingMax;
                Ok(self.state)
            }
            CalibCommand::SetMax => {
                if self.state != InteractiveState::SettingMax {
                    return Err(CalibrationError::InvalidState);
                }
                if current_mot <= self.mot_min {
                    self.state = InteractiveState::Error(CalibrationError::InvalidLimits);
                    return Err(CalibrationError::InvalidLimits);
                }
                self.mot_max = current_mot;
                self.joint_max = current_joint;
                self.state = InteractiveState::ReadyToSweep;
                Ok(self.state)
            }
            CalibCommand::Sweep => {
                match self.state {
                    InteractiveState::ReadyToSweep => {
                        self.state = InteractiveState::Sweeping;
                        self.sweep_index = 0;
                        Ok(self.state)
                    }
                    InteractiveState::Sweeping => {
                        if self.sweep_index >= N {
                            self.finalize_sweep()?;
                        }
                        Ok(self.state)
                    }
                    _ => Err(CalibrationError::InvalidState),
                }
            }
            CalibCommand::SaveFlash => {
                if self.state != InteractiveState::Completed {
                    return Err(CalibrationError::InvalidState);
                }
                Ok(self.state)
            }
            CalibCommand::Reset | CalibCommand::Abort => {
                self.state = InteractiveState::Idle;
                self.sweep_index = 0;
                self.flash_config = None;
                Ok(self.state)
            }
            CalibCommand::None => Ok(self.state),
        }
    }

    /// Interactive homing lifecycle step: sets zero offset / home angle in motor radians.
    pub fn step_homing(&mut self, home_mot_angle: f32) -> f32 {
        self.mot_min = home_mot_angle;
        if let Some(ref mut cfg) = self.flash_config {
            cfg.zero_offset = home_mot_angle;
            cfg.update_checksum();
        }
        home_mot_angle
    }

    /// Interactive backlash measurement step: computes backlash half-deadband
    /// $\delta = 0.5 \cdot |\theta_{\text{fwd}} - \theta_{\text{rev}}|$ and updates flash configuration.
    pub fn step_backlash(&mut self, forward_joint: f32, reverse_joint: f32) -> f32 {
        let delta = libm::fabsf(forward_joint - reverse_joint) * 0.5;
        if let Some(ref mut cfg) = self.flash_config {
            cfg.backlash_delta = delta;
            cfg.update_checksum();
        }
        delta
    }
}
