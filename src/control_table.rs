//! Control-table descriptor set for the `barefoc-multibus` register map.
#![allow(dead_code, unused_imports)]
//!
//! Mirrors `ref/Miniata/firmware/src/control_table.h` (offsets / widths / access
//! modes) and adds the `calibrate` command (7) to the system-command hook table
//! as an extension of upstream semantics (design D5).
//!
//! ## Register classification
//!
//! - Board-level (selector-independent): `0x00`, `0x01`, `0x03`, `0x04`, `0x05`,
//!   `0x50`, `0x54`, `0x58`, `0x64`.
//! - Motor-window (selector-gated): `0x02`, `0x08`–`0x53`.
//! - Selector: `REG_MOTOR_SELECT` at `0x70` (u8, RW, default 0).
//!
//! Upstream Reference: `ref/Miniata/firmware/src/control_table.h`,
//! `ref/Miniata/firmware/src/control_table.cpp`,
//! `ref/Miniata/firmware/src/bootloader_trap.cpp`.

use barefoc_multibus::table::{Error as TableError, HookResult, RegEntry, RegMap, WriteHookFn};
use barefoc_multibus::transport::RegServer;

// ---------------------------------------------------------------------------
// Defaults (moved here from the app constants)
// ---------------------------------------------------------------------------
pub const DEFAULT_DEVICE_ID: u8 = 0x4D;
pub const DEFAULT_I2C_ADDR: u8 = 0x30;
pub const DEFAULT_FW_VERSION: u32 = 0x0000_0100;
pub const DEFAULT_JOINT_SLOT: u8 = 0;
pub const DEFAULT_VEL_MAX: f32 = 30.0;
pub const DEFAULT_ACC_MAX: f32 = 60.0;
pub const DEFAULT_POS_KP: f32 = 3.0;
pub const DEFAULT_POS_KI: f32 = 0.0;
pub const DEFAULT_POS_KD: f32 = 0.0;
pub const DEFAULT_LIMIT_VOLTAGE: f32 = 1.5;
pub const DEFAULT_ZERO_OFFSET: f32 = 0.0;

// ---------------------------------------------------------------------------
// Application context carried by the register map (context C).
// ---------------------------------------------------------------------------
#[derive(Clone, Copy)]
pub struct MotorState {
    pub save_requested: bool,
    pub load_requested: bool,
    pub erase_requested: bool,
    pub zero_pos_requested: bool,
    pub reboot_bootloader_requested: bool,
    pub reboot_system_requested: bool,
    pub calibrate_requested: bool,
    /// Raw 96-bit STM32 UID cached for the RO `REG_UID` register (0x58..0x64).
    pub uid: [u8; 12],
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            save_requested: false,
            load_requested: false,
            erase_requested: false,
            zero_pos_requested: false,
            reboot_bootloader_requested: false,
            reboot_system_requested: false,
            calibrate_requested: false,
            uid: [0u8; 12],
        }
    }
}

// ---------------------------------------------------------------------------
// System-command hook (REG_SYSTEM_CMD, 0x50) — self-clearing.
// ---------------------------------------------------------------------------
fn system_cmd_hook(ctx: &mut MotorState, _offset: u16, value: &[u8]) -> HookResult {
    let cmd = value[0];
    match cmd {
        0 => {} // CMD_NONE
        1 => {
            // CMD_SAVE_CONFIG (stub)
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: SAVE_CONFIG (stub)");
            ctx.save_requested = true;
        }
        2 => {
            // CMD_LOAD_CONFIG (stub)
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: LOAD_CONFIG (stub)");
            ctx.load_requested = true;
        }
        3 => {
            // CMD_ERASE_CONFIG (stub)
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: ERASE_CONFIG (stub)");
            ctx.erase_requested = true;
        }
        4 => {
            // CMD_ZERO_POS — resets encoder unwrapping (handled by binding layer).
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: ZERO_POS");
            ctx.zero_pos_requested = true;
        }
        5 => {
            // CMD_REBOOT_BOOTLOADER
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: REBOOT_BOOTLOADER");
            ctx.reboot_bootloader_requested = true;
        }
        6 => {
            // CMD_REBOOT_SYSTEM
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: REBOOT_SYSTEM");
            ctx.reboot_system_requested = true;
        }
        7 => {
            // CMD_CALIBRATE (extension)
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: CALIBRATE");
            ctx.calibrate_requested = true;
        }
        _ => {
            #[cfg(feature = "defmt")]
            defmt::warn!("system_cmd: unknown command {}", cmd);
        }
    }
    HookResult::SelfClear
}

/// Write hook for `REG_JOINT_SLOT` (0x64): the value is retained (NOT
/// self-clearing) and a save is requested so the binding layer persists the
/// new slot via the flash config block (design D7). The actual blocking flash
/// write happens in the main loop's `do_save` path, not inside the hook, so
/// the bus phase stays non-blocking (design D4).
fn joint_slot_hook(ctx: &mut MotorState, _offset: u16, _value: &[u8]) -> HookResult {
    ctx.save_requested = true;
    HookResult::Applied
}

/// Write hook for `REG_MOTOR_SELECT` (0x70) in the single-motor build: only
/// selector value 0 is valid (N == 1). Out-of-range writes self-clear.
fn motor_select_hook_single(_ctx: &mut MotorState, _offset: u16, value: &[u8]) -> HookResult {
    if value[0] == 0 {
        HookResult::Applied
    } else {
        HookResult::SelfClear
    }
}

// Expose hook type for callers (not strictly needed).
pub type SystemCmdHook = WriteHookFn<MotorState>;

// ---------------------------------------------------------------------------
// build_table
// ---------------------------------------------------------------------------
pub fn build_table(ctx: MotorState) -> RegMap<MotorState, 128, 40> {
    let mut map = RegMap::new(ctx);

    // byte registers 0x00..0x05
    map.add_register(RegEntry::ro(0x00, 1)).unwrap(); // REG_DEVICE_ID
    map.add_register(RegEntry::rw(0x01, 1)).unwrap(); // REG_I2C_ADDR
    map.add_register(RegEntry::rw(0x02, 1)).unwrap(); // REG_CONTROL_MODE
    map.add_register(RegEntry::ro(0x03, 1)).unwrap(); // REG_ERROR_CODE
    map.add_register(RegEntry::ro(0x04, 1)).unwrap(); // REG_ARM_STATE
    map.add_register(RegEntry::ro(0x05, 1)).unwrap(); // REG_CALIBRATED
    // reserved 0x06..0x07
    map.add_register(RegEntry::rw(0x06, 2)).unwrap();

    // f32 targets 0x08..0x1C
    map.add_register(RegEntry::rw(0x08, 4)).unwrap(); // REG_TARGET_POS
    map.add_register(RegEntry::rw(0x0C, 4)).unwrap(); // REG_TARGET_VEL
    map.add_register(RegEntry::rw(0x10, 4)).unwrap(); // REG_TARGET_EFFORT
    map.add_register(RegEntry::rw(0x14, 4)).unwrap(); // REG_VEL_MAX
    map.add_register(RegEntry::rw(0x18, 4)).unwrap(); // REG_ACC_MAX
    map.add_register(RegEntry::rw(0x1C, 4)).unwrap(); // REG_RESERVED_1C

    // burst telemetry 0x20..0x38 (RO)
    map.add_register(RegEntry::ro(0x20, 4)).unwrap(); // REG_PRESENT_OUT_POS
    map.add_register(RegEntry::ro(0x24, 4)).unwrap(); // REG_PRESENT_OUT_VEL
    map.add_register(RegEntry::ro(0x28, 4)).unwrap(); // REG_PRESENT_TORQUE_IQ
    map.add_register(RegEntry::ro(0x2C, 4)).unwrap(); // REG_PRESENT_MOT_POS
    map.add_register(RegEntry::ro(0x30, 4)).unwrap(); // REG_PRESENT_MOT_VEL
    map.add_register(RegEntry::ro(0x34, 4)).unwrap(); // REG_PRESENT_VBUS
    map.add_register(RegEntry::ro(0x38, 4)).unwrap(); // REG_PRESENT_TEMP

    // tuning 0x3C..0x4C
    map.add_register(RegEntry::rw(0x3C, 4)).unwrap(); // REG_POS_KP
    map.add_register(RegEntry::rw(0x40, 4)).unwrap(); // REG_POS_KI
    map.add_register(RegEntry::rw(0x44, 4)).unwrap(); // REG_POS_KD
    map.add_register(RegEntry::rw(0x48, 4)).unwrap(); // REG_LIMIT_VOLTAGE
    map.add_register(RegEntry::rw(0x4C, 4)).unwrap(); // REG_ZERO_OFFSET

    // system command
    map.add_register(RegEntry::rw_hooked(0x50, 1, system_cmd_hook)).unwrap();
    // reserved tail 0x51..0x53
    map.add_register(RegEntry::rw(0x51, 3)).unwrap();

    // identity registers appended beyond legacy 0x53 (design D8)
    map.add_register(RegEntry::ro(0x54, 4)).unwrap(); // REG_FW_VERSION (u32 RO)
    map.add_register(RegEntry::ro(0x58, 12)).unwrap(); // REG_UID (12 B RO)
    map.add_register(RegEntry::rw_hooked(0x64, 1, joint_slot_hook)).unwrap(); // REG_JOINT_SLOT (u8 Rw)

    // Selector register (always present, even in single-motor builds).
    map.add_register(RegEntry::rw_hooked(0x70, 1, motor_select_hook_single)).unwrap(); // REG_MOTOR_SELECT (u8 Rw)

    // ---- Defaults applied on init ----
    let _ = map.set(0x00, &[DEFAULT_DEVICE_ID]);
    let _ = map.set(0x01, &[DEFAULT_I2C_ADDR]);
    let _ = map.set(0x02, &[0u8]); // IDLE
    let _ = map.set(0x54, &DEFAULT_FW_VERSION.to_le_bytes());
    let _ = map.set(0x64, &[DEFAULT_JOINT_SLOT]);
    let _ = map.set(0x70, &[0u8]); // REG_MOTOR_SELECT defaults to 0

    let z = 0.0f32.to_le_bytes();
    for off in [0x08u16, 0x0C, 0x10, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30, 0x38, 0x4C] {
        let _ = map.set(off, &z);
    }
    let vbus = 0.0f32.to_le_bytes();
    let _ = map.set(0x34, &vbus); // filled each tick; default 0
    let _ = map.set(0x14, &DEFAULT_VEL_MAX.to_le_bytes());
    let _ = map.set(0x18, &DEFAULT_ACC_MAX.to_le_bytes());
    let _ = map.set(0x3C, &DEFAULT_POS_KP.to_le_bytes());
    let _ = map.set(0x40, &DEFAULT_POS_KI.to_le_bytes());
    let _ = map.set(0x44, &DEFAULT_POS_KD.to_le_bytes());
    let _ = map.set(0x48, &DEFAULT_LIMIT_VOLTAGE.to_le_bytes());

    let _ = TableError::Unmapped; // keep import used (no-op)
    map
}

// Helper re-exports for main.rs convenience.
pub fn read_f32<C>(map: &RegMap<C, 128, 40>, offset: u16) -> f32 {
    let mut buf = [0u8; 4];
    if map.get(offset, &mut buf).is_ok() {
        f32::from_le_bytes(buf)
    } else {
        0.0
    }
}

pub fn write_f32<C>(map: &mut RegMap<C, 128, 40>, offset: u16, v: f32) {
    let _ = map.set(offset, &v.to_le_bytes());
}

// Keep RegServer import used.
pub fn _use_server<C>(_s: &RegServer<C, 128, 40>) {}

// ===========================================================================
// Multi-motor profile (gated behind `feature = "multi-motor"`)
// ===========================================================================
#[cfg(feature = "multi-motor")]
#[derive(Clone, Copy)]
pub struct MultiMotorState<const N: usize> {
    pub motors: [MotorState; N],
    pub selected: u8,
    pub uid: [u8; 12],
    pub slot: u8,
    pub fw_version: u32,
    pub system_cmd_pending: u8,
    pub save_requested: bool,
    pub load_requested: bool,
    pub erase_requested: bool,
    pub zero_pos_requested: bool,
    pub reboot_bootloader_requested: bool,
    pub reboot_system_requested: bool,
    pub calibrate_requested: bool,
}

#[cfg(feature = "multi-motor")]
impl<const N: usize> Default for MultiMotorState<N> {
    fn default() -> Self {
        Self {
            motors: [MotorState::default(); N],
            selected: 0,
            uid: [0u8; 12],
            slot: DEFAULT_JOINT_SLOT,
            fw_version: DEFAULT_FW_VERSION,
            system_cmd_pending: 0,
            save_requested: false,
            load_requested: false,
            erase_requested: false,
            zero_pos_requested: false,
            reboot_bootloader_requested: false,
            reboot_system_requested: false,
            calibrate_requested: false,
        }
    }
}

#[cfg(feature = "multi-motor")]
impl<const N: usize> MultiMotorState<N> {
    /// Construct a multi-motor state seeded with a board UID.
    pub fn new(uid: [u8; 12]) -> Self {
        let mut s = Self::default();
        s.uid = uid;
        s
    }
}

#[cfg(feature = "multi-motor")]
fn system_cmd_hook_multi<const N: usize>(
    ctx: &mut MultiMotorState<N>,
    _offset: u8,
    value: &[u8],
) -> HookResult {
    let cmd = value[0];
    match cmd {
        0 => {}
        1 => {
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: SAVE_CONFIG (stub)");
            ctx.save_requested = true;
        }
        2 => {
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: LOAD_CONFIG (stub)");
            ctx.load_requested = true;
        }
        3 => {
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: ERASE_CONFIG (stub)");
            ctx.erase_requested = true;
        }
        4 => {
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: ZERO_POS");
            ctx.zero_pos_requested = true;
        }
        5 => {
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: REBOOT_BOOTLOADER");
            ctx.reboot_bootloader_requested = true;
        }
        6 => {
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: REBOOT_SYSTEM");
            ctx.reboot_system_requested = true;
        }
        7 => {
            #[cfg(feature = "defmt")]
            defmt::info!("system_cmd: CALIBRATE");
            ctx.calibrate_requested = true;
        }
        _ => {
            #[cfg(feature = "defmt")]
            defmt::warn!("system_cmd: unknown command {}", cmd);
        }
    }
    ctx.system_cmd_pending = cmd;
    HookResult::SelfClear
}

#[cfg(feature = "multi-motor")]
fn joint_slot_hook_multi<const N: usize>(
    ctx: &mut MultiMotorState<N>,
    _offset: u8,
    _value: &[u8],
) -> HookResult {
    ctx.save_requested = true;
    HookResult::Applied
}

#[cfg(feature = "multi-motor")]
fn motor_select_hook_multi<const N: usize>(
    ctx: &mut MultiMotorState<N>,
    _offset: u8,
    value: &[u8],
) -> HookResult {
    if (value[0] as usize) < N {
        ctx.selected = value[0];
        HookResult::Applied
    } else {
        HookResult::SelfClear
    }
}

#[cfg(feature = "multi-motor")]
pub fn build_multi_motor_table<const N: usize>() -> RegMap<MultiMotorState<N>, 128, 40> {
    let mut map = RegMap::new(MultiMotorState::<N>::default());

    // byte registers 0x00..0x05
    map.add_register(RegEntry::ro(0x00, 1)).unwrap(); // REG_DEVICE_ID
    map.add_register(RegEntry::rw(0x01, 1)).unwrap(); // REG_I2C_ADDR
    map.add_register(RegEntry::rw(0x02, 1)).unwrap(); // REG_CONTROL_MODE
    map.add_register(RegEntry::ro(0x03, 1)).unwrap(); // REG_ERROR_CODE
    map.add_register(RegEntry::ro(0x04, 1)).unwrap(); // REG_ARM_STATE
    map.add_register(RegEntry::ro(0x05, 1)).unwrap(); // REG_CALIBRATED
    // reserved 0x06..0x07
    map.add_register(RegEntry::rw(0x06, 2)).unwrap();

    // f32 targets 0x08..0x1C
    map.add_register(RegEntry::rw(0x08, 4)).unwrap(); // REG_TARGET_POS
    map.add_register(RegEntry::rw(0x0C, 4)).unwrap(); // REG_TARGET_VEL
    map.add_register(RegEntry::rw(0x10, 4)).unwrap(); // REG_TARGET_EFFORT
    map.add_register(RegEntry::rw(0x14, 4)).unwrap(); // REG_VEL_MAX
    map.add_register(RegEntry::rw(0x18, 4)).unwrap(); // REG_ACC_MAX
    map.add_register(RegEntry::rw(0x1C, 4)).unwrap(); // REG_RESERVED_1C

    // burst telemetry 0x20..0x38 (RO)
    map.add_register(RegEntry::ro(0x20, 4)).unwrap(); // REG_PRESENT_OUT_POS
    map.add_register(RegEntry::ro(0x24, 4)).unwrap(); // REG_PRESENT_OUT_VEL
    map.add_register(RegEntry::ro(0x28, 4)).unwrap(); // REG_PRESENT_TORQUE_IQ
    map.add_register(RegEntry::ro(0x2C, 4)).unwrap(); // REG_PRESENT_MOT_POS
    map.add_register(RegEntry::ro(0x30, 4)).unwrap(); // REG_PRESENT_MOT_VEL
    map.add_register(RegEntry::ro(0x34, 4)).unwrap(); // REG_PRESENT_VBUS
    map.add_register(RegEntry::ro(0x38, 4)).unwrap(); // REG_PRESENT_TEMP

    // tuning 0x3C..0x4C
    map.add_register(RegEntry::rw(0x3C, 4)).unwrap(); // REG_POS_KP
    map.add_register(RegEntry::rw(0x40, 4)).unwrap(); // REG_POS_KI
    map.add_register(RegEntry::rw(0x44, 4)).unwrap(); // REG_POS_KD
    map.add_register(RegEntry::rw(0x48, 4)).unwrap(); // REG_LIMIT_VOLTAGE
    map.add_register(RegEntry::rw(0x4C, 4)).unwrap(); // REG_ZERO_OFFSET

    // system command
    map.add_register(RegEntry::rw_hooked(0x50, 1, system_cmd_hook_multi::<N>)).unwrap();
    // reserved tail 0x51..0x53
    map.add_register(RegEntry::rw(0x51, 3)).unwrap();

    // identity registers appended beyond legacy 0x53 (design D8)
    map.add_register(RegEntry::ro(0x54, 4)).unwrap(); // REG_FW_VERSION (u32 RO)
    map.add_register(RegEntry::ro(0x58, 12)).unwrap(); // REG_UID (12 B RO)
    map.add_register(RegEntry::rw_hooked(0x64, 1, joint_slot_hook_multi::<N>)).unwrap(); // REG_JOINT_SLOT (u8 Rw)

    // Selector register (gates the motor window).
    map.add_register(RegEntry::rw_hooked(0x70, 1, motor_select_hook_multi::<N>)).unwrap(); // REG_MOTOR_SELECT (u8 Rw)

    // ---- Defaults applied on init ----
    let _ = map.set(0x00, &[DEFAULT_DEVICE_ID]);
    let _ = map.set(0x01, &[DEFAULT_I2C_ADDR]);
    let _ = map.set(0x02, &[0u8]); // IDLE
    let _ = map.set(0x54, &DEFAULT_FW_VERSION.to_le_bytes());
    let _ = map.set(0x64, &[DEFAULT_JOINT_SLOT]);
    let _ = map.set(0x70, &[0u8]);

    let z = 0.0f32.to_le_bytes();
    for off in [0x08u8, 0x0C, 0x10, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30, 0x38, 0x4C] {
        let _ = map.set(off, &z);
    }
    let vbus = 0.0f32.to_le_bytes();
    let _ = map.set(0x34, &vbus);
    let _ = map.set(0x14, &DEFAULT_VEL_MAX.to_le_bytes());
    let _ = map.set(0x18, &DEFAULT_ACC_MAX.to_le_bytes());
    let _ = map.set(0x3C, &DEFAULT_POS_KP.to_le_bytes());
    let _ = map.set(0x40, &DEFAULT_POS_KI.to_le_bytes());
    let _ = map.set(0x44, &DEFAULT_POS_KD.to_le_bytes());
    let _ = map.set(0x48, &DEFAULT_LIMIT_VOLTAGE.to_le_bytes());

    map
}
