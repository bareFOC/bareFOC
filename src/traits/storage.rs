//! Non-volatile configuration persistence traits.
//!
//! Provides the [`ConfigStorage`] interface to load and commit the EEPROM
//! portion (`0x00..0x60`) of the [`ControlTable`] to on-chip Flash memory
//! across power cycles.

use crate::control_table::ControlTable;

/// Abstract interface for persisting Control Table parameters to non-volatile storage.
pub trait ConfigStorage {
    /// Storage error type returned on failure.
    type Error;

    /// Load persisted configuration from non-volatile storage into `table`.
    ///
    /// If valid configuration data is found, populates the EEPROM area (`0x00..0x60`).
    /// If no valid configuration exists or CRC verification fails, restores factory
    /// defaults and sets bit 7 (`FLASH_CORRUPT`) of `HardwareErrorStatus`.
    fn load_config(&mut self, table: &mut ControlTable) -> Result<(), Self::Error>;

    /// Commit the EEPROM area of `table` to non-volatile storage.
    fn save_config(&mut self, table: &ControlTable) -> Result<(), Self::Error>;

    /// Erase stored configuration and restore `table` to factory defaults.
    fn factory_reset(&mut self, table: &mut ControlTable) -> Result<(), Self::Error>;
}
