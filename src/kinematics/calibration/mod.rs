//! # calibration — on-device calibration and flash configuration
//!
//! Provides autonomous (1-call) and interactive guided calibration state machines,
//! monotonicity verification, and 64-byte aligned non-volatile flash storage packing.

pub mod autonomous;
pub mod interactive;
pub mod storage;

pub use autonomous::{AutonomousCalibrator, CalibrationConfig, CalibrationError, CalibrationResult};
pub use interactive::{InteractiveCalibrator, InteractiveState};
pub use storage::{FlashCalibrationData, StorageError, FLASH_MAGIC, FLASH_VERSION};
