//! # calibration — on-device calibration and flash configuration
//!
//! Provides autonomous (1-call) and interactive guided calibration state machines,
//! monotonicity verification, and 64-byte aligned non-volatile flash storage packing.

pub mod autonomous;
pub mod interactive;
pub mod storage;

/// Calibration command trigger codes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CalibCommand {
    /// No operation / idle.
    None = 0,
    /// Trigger autonomous limit discovery and calibration sweep.
    StartAutonomous = 1,
    /// Abort active calibration routine immediately.
    Abort = 2,
    /// Begin interactive guided calibration.
    StartInteractive = 3,
    /// Lock minimum joint limit in interactive calibration.
    SetMin = 4,
    /// Lock maximum joint limit in interactive calibration.
    SetMax = 5,
    /// Initiate or step sampling sweep.
    Sweep = 6,
    /// Finalize and save calibration configuration to flash.
    SaveFlash = 7,
    /// Reset calibration state to Idle.
    Reset = 8,
}

impl TryFrom<u8> for CalibCommand {
    type Error = ();

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::None),
            1 => Ok(Self::StartAutonomous),
            2 => Ok(Self::Abort),
            3 => Ok(Self::StartInteractive),
            4 => Ok(Self::SetMin),
            5 => Ok(Self::SetMax),
            6 => Ok(Self::Sweep),
            7 => Ok(Self::SaveFlash),
            8 => Ok(Self::Reset),
            _ => Err(()),
        }
    }
}

/// Calibration state reported in telemetry and registers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CalibState {
    /// Actuator is idle, no calibration running.
    Idle = 0,
    /// Calibration routine in progress.
    Running = 1,
    /// Calibration successfully completed.
    Done = 2,
    /// Calibration halted with error.
    Error = 3,
}

impl From<CalibState> for u8 {
    fn from(s: CalibState) -> Self {
        s as u8
    }
}

impl TryFrom<u8> for CalibState {
    type Error = ();

    fn try_from(val: u8) -> Result<Self, ()> {
        match val {
            0 => Ok(Self::Idle),
            1 => Ok(Self::Running),
            2 => Ok(Self::Done),
            3 => Ok(Self::Error),
            _ => Err(()),
        }
    }
}

pub use autonomous::{AutonomousCalibrator, CalibrationConfig, CalibrationError, CalibrationResult};
pub use interactive::{InteractiveCalibrator, InteractiveState};
pub use storage::{
    load_from_flash, save_to_flash, FlashCalibrationData, StorageError, FLASH_MAGIC, FLASH_VERSION,
};
