//! # traits — abstract sensor / driver / current-sense interfaces
//!
//! This module collects the three abstract base classes from upstream
//! Arduino-FOC's `common/base_classes/` directory. Each port replaces
//! a C++ virtual base class with a Rust trait and a `*Base` struct
//! that owns the protected state — see `PORTING_GUIDELINES.md` §1
//! "Pattern 1: C++ Virtual Dispatch → Rust Generics".
//!
//! Concrete sensor / driver / current-sense implementations (in the
//! `sensors/`, `drivers/`, and `current_sense/` modules) embed the
//! matching `*Base` struct and implement the trait.
//!
//! See `PORTING_STATUS.md` Core Library table for the per-file status
//! of this module.

pub mod adc;
#[cfg(feature = "boot")]
pub mod bootloader;
pub mod current_sense;
pub mod driver;
pub mod sensor;
pub mod storage;
pub mod uid;

pub use adc::AdcPin;
#[cfg(feature = "boot")]
pub use bootloader::{BootTargetProvider, BootloaderHandoff};
pub use current_sense::{CurrentSense, CurrentSenseBase};
pub use driver::{DriverType, FOCDriver, FocDriverParams, PhaseState};
pub use sensor::{Direction, Sensor, SensorBase};
pub use storage::ConfigStorage;
pub use uid::{CANONICAL_HEX_LEN, HardwareUid, MAX_UID_BYTES, StaticUid};
pub use crate::common::time::Clock;

