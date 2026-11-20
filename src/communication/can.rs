//! Standardized CAN / CAN-FD frame mappings for Smart Servo Control Table.
//!
//! Maps CAN 11-bit arbitration IDs using the standard node-command schema:
//! `arb_id = ((node_id as u32) << 5) | (cmd_id & 0x1F)`
//! with special handling for admin commands (`CmdID = 0x7F`).

use crate::control_table::{
    ControlTable, OperatingMode, CMD_REBOOT_BOOTLOADER, CMD_SAVE_CONFIG,
};

// Command IDs (0x00..0x1F, plus 0x7F admin)
pub const CAN_CMD_GET_VERSION: u8 = 0x00;
pub const CAN_CMD_SET_OPERATING_MODE: u8 = 0x01;
pub const CAN_CMD_SET_GOAL_POSITION: u8 = 0x02;
pub const CAN_CMD_SET_GOAL_VELOCITY: u8 = 0x03;
pub const CAN_CMD_SET_GOAL_TORQUE: u8 = 0x04;
pub const CAN_CMD_SET_LIMITS: u8 = 0x05;
pub const CAN_CMD_SAVE_CONFIG: u8 = 0x06;
pub const CAN_CMD_ADMIN_REBOOT: u8 = 0x7F;

/// Build an 11-bit standard arbitration ID for a node and command.
pub const fn make_can_id(node_id: u8, cmd_id: u8) -> u32 {
    ((node_id as u32) << 5) | (cmd_id as u32 & 0x1F)
}

/// Build an admin arbitration ID for Katapult/BareBoot reboot.
pub const fn make_admin_reboot_id(node_id: u8) -> u32 {
    ((node_id as u32) << 7) | (CAN_CMD_ADMIN_REBOOT as u32)
}

/// Encode a target position setpoint into an 8-byte CAN frame.
pub fn encode_set_position(node_id: u8, pos: f32) -> (u32, [u8; 8], usize) {
    let id = make_can_id(node_id, CAN_CMD_SET_GOAL_POSITION);
    let mut payload = [0u8; 8];
    payload[0..4].copy_from_slice(&pos.to_le_bytes());
    (id, payload, 4)
}

/// Encode a bootloader reboot trigger into an 8-byte CAN frame.
pub fn encode_reboot_bootloader(node_id: u8) -> (u32, [u8; 8], usize) {
    let id = make_admin_reboot_id(node_id);
    let mut payload = [0u8; 8];
    payload[0] = CMD_REBOOT_BOOTLOADER;
    (id, payload, 1)
}

/// Parse and decode an incoming CAN frame into the [`ControlTable`].
///
/// Returns `true` if the frame requested an immediate Katapult bootloader reboot.
pub fn process_can_frame(id: u32, payload: &[u8], table: &mut ControlTable) -> bool {
    // Check for admin reboot command (CmdID = 0x7F)
    if (id & 0x7F) == CAN_CMD_ADMIN_REBOOT as u32 {
        if !payload.is_empty() && payload[0] == CMD_REBOOT_BOOTLOADER {
            return true;
        }
    }

    let cmd_id = (id & 0x1F) as u8;
    match cmd_id {
        CAN_CMD_SET_OPERATING_MODE => {
            if !payload.is_empty() {
                if let Ok(mode) = OperatingMode::try_from(payload[0]) {
                    table.set_operating_mode(mode);
                }
            }
        }
        CAN_CMD_SET_GOAL_POSITION => {
            if payload.len() >= 4 {
                let pos = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                table.set_goal_position(pos);
            }
        }
        CAN_CMD_SET_GOAL_VELOCITY => {
            if payload.len() >= 4 {
                let vel = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                table.set_goal_velocity(vel);
            }
        }
        CAN_CMD_SET_GOAL_TORQUE => {
            if payload.len() >= 4 {
                let cur = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                table.set_goal_current(cur);
            }
        }
        CAN_CMD_SAVE_CONFIG => {
            if !payload.is_empty() && payload[0] == CMD_SAVE_CONFIG {
                // SaveConfig trigger
                table.write_u8(crate::control_table::REG_SAVE_CONFIG_CMD, CMD_SAVE_CONFIG);
            }
        }
        _ => {}
    }

    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_can_frame_position_encoding_and_processing() {
        let mut table = ControlTable::new();
        let (id, payload, len) = encode_set_position(0x01, 2.5);

        assert_eq!(len, 4);
        assert_eq!(id, make_can_id(1, CAN_CMD_SET_GOAL_POSITION));

        let is_reboot = process_can_frame(id, &payload[0..len], &mut table);
        assert!(!is_reboot);
        assert_eq!(table.goal_position(), 2.5);
    }

    #[test]
    fn test_can_reboot_bootloader_frame() {
        let mut table = ControlTable::new();
        let (id, payload, len) = encode_reboot_bootloader(0x01);

        let is_reboot = process_can_frame(id, &payload[0..len], &mut table);
        assert!(is_reboot);
    }
}
