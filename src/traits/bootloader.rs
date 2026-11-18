//! BareBoot bootloader-presence abstraction for application firmware.
//!
//! The trait is intentionally minimal: it carries no MMIO and no bus logic.
//! The platform (HAL) supplies a [`crate::boot::BootTarget`] describing its
//! bootloader flash slot, application slot and SRAM bounds; the default method
//! delegates the vector-table check to the dependency-free `boot` core.
//!
//! Source contract: `bareBoot/docs/handsfree-app-updates.md` section 4.1.
//!
//! This module is gated behind the `boot` Cargo feature.

/// Supplies the BareBoot memory contract for the current MCU / board.
///
/// Implementors only report a layout. Trigger decoding, cookie writing and
/// system reset stay in the HAL / application so this crate remains portable.
pub trait BootTargetProvider {
    /// Flash / SRAM layout used to locate the bootloader and its mailbox.
    fn boot_target(&self) -> crate::boot::BootTarget;

    /// Whether a Katapult/BareBoot-compatible bootloader appears to be present.
    ///
    /// Delegates to [`crate::boot::BootTarget::is_present`], which checks the
    /// initial SP and reset vector as specified in doc section 4.1.
    fn is_bareboot_present(&self) -> bool {
        self.boot_target().is_present()
    }
}
