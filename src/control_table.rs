//! Universal Dynamixel-grade Smart Servo Control Table.
//!
//! Standardized, hardware-agnostic 256-byte Control Table partitioned into:
//! - EEPROM Area (`0x00..0xA0`, 160 bytes): non-volatile configuration committed to Flash.
//! - RAM Area (`0xA0..0x100`, 96 bytes): volatile operational targets, setpoints, and telemetry.
//!
//! # Memory Map
//!
//! | Address | Name | Type | Access | Description |
//! |---|---|---|---|---|
//! | 0x00 | ModelNumber | u16 | RO | Device model identifier |
//! | 0x02 | FirmwareVersion | u8 | RO | Firmware revision |
//! | 0x03 | ProtocolVersion | u8 | RO | Control Table protocol version |
//! | 0x04 | HardwareUid | [u8; 12] | RO | Factory silicon 96-bit unique identifier |
//! | 0x10 | BoardNodeId | u8 | RW | Device bus target address (default 0x40) |
//! | 0x11 | BoardName | str (15B) | RW | User-configurable board node name (null-padded) |
//! | 0x20 | Joint0Addr | u8 | RW | Joint 0 servo address (0..127) |
//! | 0x21 | Joint0Name | str (15B) | RW | Joint 0 name (null-padded UTF-8) |
//! | 0x30 | Joint1Addr | u8 | RW | Joint 1 servo address (0..127) |
//! | 0x31 | Joint1Name | str (15B) | RW | Joint 1 name (null-padded UTF-8) |
//! | 0x40 | Joint2Addr | u8 | RW | Joint 2 servo address (0..127) |
//! | 0x41 | Joint2Name | str (15B) | RW | Joint 2 name (null-padded UTF-8) |
//! | 0x50 | Joint3Addr | u8 | RW | Joint 3 servo address (0..127) |
//! | 0x51 | Joint3Name | str (15B) | RW | Joint 3 name (null-padded UTF-8) |
//! | 0x60 | OperatingMode | u8 | RW | Motion control mode (0..7) |
//! | 0x61 | SecondaryId | u8 | RW | Secondary / group ID |
//! | 0x62 | TopologyPreset | u8 | RW | Multi-motor kinematics preset (0..3, 0xFF) |
//! | 0x63 | CouplingMask | u8 | RW | Kinematics coupling bitmask |
//! | 0x64 | HomingOffset | f32 | RW | Homing position zero offset (rad) |
//! | 0x68 | CurrentLimit | f32 | RW | Maximum allowable motor current (A) |
//! | 0x6C | AccelerationLimit | f32 | RW | Maximum acceleration limit (rad/s^2) |
//! | 0x70 | VelocityLimit | f32 | RW | Maximum velocity limit (rad/s) |
//! | 0x74 | MinPositionLimit | f32 | RW | Minimum software travel limit (rad) |
//! | 0x78 | MaxPositionLimit | f32 | RW | Maximum software travel limit (rad) |
//! | 0x7C | VoltageLimit | f32 | RW | Maximum motor phase/drive voltage (V) |
//! | 0x80 | TemperatureLimit | f32 | RW | Thermal shutdown temperature threshold (deg C) |
//! | 0x84 | MinBusVoltage | f32 | RW | Minimum bus voltage cutoff (V) |
//! | 0x88 | MaxBusVoltage | f32 | RW | Maximum bus voltage cutoff (V) |
//! | 0x8C | PosPGain | f32 | RW | Position PID proportional gain Kp |
//! | 0x90 | PosIGain | f32 | RW | Position PID integral gain Ki |
//! | 0x94 | PosDGain | f32 | RW | Position PID derivative gain Kd |
//! | 0x98 | VelPGain | f32 | RW | Velocity PID proportional gain Kp |
//! | 0x9C | VelIGain | f32 | RW | Velocity PID integral gain Ki |
//! | 0xA0 | TorqueEnable | u8 | RW | Motor bridge drive enable (0=Coast, 1=Enable) |
//! | 0xA1 | Led | u8 | RW | Status LED output |
//! | 0xA2 | StatusReturnLevel | u8 | RW | Comms response verbosity |
//! | 0xA3 | RebootCommand | u8 | RW | Software reboot trigger (0xF0 = Katapult bootloader) |
//! | 0xA4 | SaveConfigCommand | u8 | RW | Flash persistence trigger (0x55 = Save, 0xAA = Factory) |
//! | 0xA8 | ProfileVelocity | f32 | RW | Trapezoidal trajectory maximum velocity (rad/s) |
//! | 0xAC | ProfileAcceleration | f32 | RW | Trapezoidal trajectory acceleration (rad/s^2) |
//! | 0xB0 | GoalPosition | f32 | RW | Target position setpoint (rad) |
//! | 0xB4 | GoalVelocity | f32 | RW | Target velocity setpoint (rad/s) |
//! | 0xB8 | GoalCurrent | f32 | RW | Target current / torque setpoint (A) |
//! | 0xBC | GoalVoltage | f32 | RW | Target drive voltage setpoint (V) |
//! | 0xC0 | HardwareErrorStatus | u16 | RO | Active hardware fault bitfield |
//! | 0xC2 | PresentTemperature | f32 | RO | Measured internal MCU / stage temperature (deg C) |
//! | 0xC4 | PresentPosition | f32 | RO | Measured shaft / joint position (rad) |
//! | 0xC8 | PresentVelocity | f32 | RO | Measured shaft velocity (rad/s) |
//! | 0xCC | PresentCurrent | f32 | RO | Measured motor current (A) |
//! | 0xD0 | PresentVoltage | f32 | RO | Commanded / applied phase voltage (V) |
//! | 0xD4 | PresentBusVoltage | f32 | RO | Measured DC supply / bus voltage (V) |
//! | 0xD8 | FilteredCurrent | f32 | RO | Low-pass filtered current telemetry (A) |
//! | 0xDC | TrajectoryPosition | f32 | RO | Instantaneous trajectory generator position setpoint (rad) |
//! | 0xE0 | Moving | u8 | RO | Motion profile active flag (1=in motion, 0=stopped) |
//! | 0xE2 | MultiTurnRevolutions | i32 | RO | Cumulative completed shaft revolutions |
//! | 0xE6 | MultiTurnAngle | f32 | RO | Cumulative unwrapped multi-turn angle (rad) |

/// Total byte capacity of the standardized Control Table.
pub const CONTROL_TABLE_SIZE: usize = 256;

/// Byte offset marking the boundary between EEPROM and RAM areas.
pub const EEPROM_AREA_SIZE: usize = 160;

/// Byte size of the volatile RAM register area.
pub const RAM_AREA_SIZE: usize = 96;

// Register byte offsets
pub const REG_MODEL_NUMBER: usize = 0x00;
pub const REG_FIRMWARE_VERSION: usize = 0x02;
pub const REG_PROTOCOL_VERSION: usize = 0x03;

// Board Identity
pub const REG_HARDWARE_UID: usize = 0x04;
pub const REG_BOARD_NODE_ID: usize = 0x10;
pub const REG_NODE_ID: usize = REG_BOARD_NODE_ID;
pub const REG_BOARD_NAME: usize = 0x11;

// Joint Identity Slots (4x)
pub const REG_JOINT0_ADDR: usize = 0x20;
pub const REG_JOINT0_NAME: usize = 0x21;
pub const REG_JOINT1_ADDR: usize = 0x30;
pub const REG_JOINT1_NAME: usize = 0x31;
pub const REG_JOINT2_ADDR: usize = 0x40;
pub const REG_JOINT2_NAME: usize = 0x41;
pub const REG_JOINT3_ADDR: usize = 0x50;
pub const REG_JOINT3_NAME: usize = 0x51;

// Configuration & Limits in EEPROM (0x60..0xA0)
pub const REG_OPERATING_MODE: usize = 0x60;
pub const REG_SECONDARY_ID: usize = 0x61;
pub const REG_TOPOLOGY_PRESET: usize = 0x62;
pub const REG_COUPLING_MASK: usize = 0x63;
pub const REG_HOMING_OFFSET: usize = 0x64;
pub const REG_CURRENT_LIMIT: usize = 0x68;
pub const REG_ACCELERATION_LIMIT: usize = 0x6C;
pub const REG_VELOCITY_LIMIT: usize = 0x70;
pub const REG_MIN_POSITION_LIMIT: usize = 0x74;
pub const REG_MAX_POSITION_LIMIT: usize = 0x78;
pub const REG_VOLTAGE_LIMIT: usize = 0x7C;
pub const REG_TEMPERATURE_LIMIT: usize = 0x80;
pub const REG_MIN_BUS_VOLTAGE: usize = 0x84;
pub const REG_MAX_BUS_VOLTAGE: usize = 0x88;
pub const REG_POS_P_GAIN: usize = 0x8C;
pub const REG_POS_I_GAIN: usize = 0x90;
pub const REG_POS_D_GAIN: usize = 0x94;
pub const REG_VEL_P_GAIN: usize = 0x98;
pub const REG_VEL_I_GAIN: usize = 0x9C;
pub const REG_VEL_D_GAIN: usize = 0x9C;
pub const REG_CURRENT_P_GAIN: usize = 0x9C;
pub const REG_CURRENT_I_GAIN: usize = 0x9C;
pub const REG_CURRENT_D_GAIN: usize = 0x9C;

// Volatile RAM area (0xA0..0x100)
pub const REG_TORQUE_ENABLE: usize = 0xA0;
pub const REG_LED: usize = 0xA1;
pub const REG_STATUS_RETURN_LEVEL: usize = 0xA2;
pub const REG_REBOOT_CMD: usize = 0xA3;
pub const REG_SAVE_CONFIG_CMD: usize = 0xA4;
pub const REG_PROFILE_VELOCITY: usize = 0xA8;
pub const REG_PROFILE_ACCELERATION: usize = 0xAC;
pub const REG_GOAL_POSITION: usize = 0xB0;
pub const REG_GOAL_VELOCITY: usize = 0xB4;
pub const REG_GOAL_CURRENT: usize = 0xB8;
pub const REG_GOAL_VOLTAGE: usize = 0xBC;
pub const REG_HARDWARE_ERROR_STATUS: usize = 0xC0;
pub const REG_PRESENT_TEMPERATURE: usize = 0xC2;
pub const REG_PRESENT_POSITION: usize = 0xC4;
pub const REG_PRESENT_VELOCITY: usize = 0xC8;
pub const REG_PRESENT_CURRENT: usize = 0xCC;
pub const REG_PRESENT_VOLTAGE: usize = 0xD0;
pub const REG_PRESENT_BUS_VOLTAGE: usize = 0xD4;
pub const REG_FILTERED_CURRENT: usize = 0xD8;
pub const REG_TRAJECTORY_POSITION: usize = 0xDC;
pub const REG_MOVING: usize = 0xE0;
pub const REG_MULTI_TURN_REVOLUTIONS: usize = 0xE2;
pub const REG_MULTI_TURN_ANGLE: usize = 0xE6;

/// Trigger value written to `RebootCommand` (0x63) to request BareBoot Katapult bootloader.
pub const CMD_REBOOT_BOOTLOADER: u8 = 0xF0;

/// Trigger value written to `SaveConfigCommand` (0x64) to commit EEPROM area to Flash.
pub const CMD_SAVE_CONFIG: u8 = 0x55;

/// Trigger value written to `SaveConfigCommand` (0x64) to restore hardcoded factory defaults.
pub const CMD_FACTORY_RESET: u8 = 0xAA;

/// Standardized smart servo operating modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OperatingMode {
    /// Closed-loop torque / current control.
    Torque = 0,
    /// Closed-loop velocity control.
    Velocity = 1,
    /// Closed-loop single-turn position control [-pi, +pi].
    Position = 2,
    /// Closed-loop continuous multi-turn extended position control.
    MultiTurnPosition = 3,
    /// Closed-loop position control with active current limiting.
    CurrentBasedPosition = 4,
    /// Open-loop direct phase voltage control.
    OpenLoopVoltage = 5,
    /// Open-loop velocity control without sensor feedback.
    OpenLoopVelocity = 6,
    /// Open-loop angle control without sensor feedback.
    OpenLoopAngle = 7,
}

impl TryFrom<u8> for OperatingMode {
    type Error = ();

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::Torque),
            1 => Ok(Self::Velocity),
            2 => Ok(Self::Position),
            3 => Ok(Self::MultiTurnPosition),
            4 => Ok(Self::CurrentBasedPosition),
            5 => Ok(Self::OpenLoopVoltage),
            6 => Ok(Self::OpenLoopVelocity),
            7 => Ok(Self::OpenLoopAngle),
            _ => Err(()),
        }
    }
}

/// Runtime kinematics topology presets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TopologyPreset {
    /// 4 independent 1-DOF actuators.
    Independent4x1Dof = 0,
    /// 2 independent 2-DOF differential gimbals.
    Differential2x2Dof = 1,
    /// 1x 1-DOF joint + 1x 2-DOF differential gimbal.
    Hybrid1Dof2Dof = 2,
    /// 2x 1-DOF joints + 1x 2-DOF differential gimbal.
    Hybrid2x1Dof2Dof = 3,
    /// Custom kinematics matrix specified via coupling bitmask.
    Custom = 0xFF,
}

impl TryFrom<u8> for TopologyPreset {
    type Error = ();

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::Independent4x1Dof),
            1 => Ok(Self::Differential2x2Dof),
            2 => Ok(Self::Hybrid1Dof2Dof),
            3 => Ok(Self::Hybrid2x1Dof2Dof),
            0xFF => Ok(Self::Custom),
            _ => Err(()),
        }
    }
}

/// Bitfield flags for the 16-bit `HardwareErrorStatus` register (0x80).
pub mod error_flags {
    /// Motor current exceeded CurrentLimit; outputs disarmed.
    pub const OVERCURRENT: u16 = 1 << 0;
    /// Temperature exceeded TemperatureLimit; thermal shutdown tripped.
    pub const OVERHEAT: u16 = 1 << 1;
    /// DC supply voltage exceeded MaxBusVoltage.
    pub const OVERVOLTAGE: u16 = 1 << 2;
    /// DC supply voltage dropped below MinBusVoltage.
    pub const UNDERVOLTAGE: u16 = 1 << 3;
    /// Target position requested beyond soft Min/MaxPositionLimit.
    pub const POSITION_LIMIT: u16 = 1 << 4;
    /// Velocity exceeded VelocityLimit.
    pub const VELOCITY_LIMIT: u16 = 1 << 5;
    /// Hardware driver bridge fault.
    pub const DRIVER_FAULT: u16 = 1 << 6;
    /// Flash configuration header CRC32 check failed; factory defaults loaded.
    pub const FLASH_CORRUPT: u16 = 1 << 7;
}

/// Standardized 176-byte Control Table data structure.
#[derive(Clone, Copy)]
pub struct ControlTable {
    bytes: [u8; CONTROL_TABLE_SIZE],
}

impl Default for ControlTable {
    fn default() -> Self {
        let mut table = Self {
            bytes: [0u8; CONTROL_TABLE_SIZE],
        };
        table.load_factory_defaults();
        table
    }
}

impl ControlTable {
    /// Create a zero-initialized Control Table.
    pub const fn empty() -> Self {
        Self {
            bytes: [0u8; CONTROL_TABLE_SIZE],
        }
    }

    /// Create a new Control Table initialized with factory defaults.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset all EEPROM and RAM parameters to factory defaults.
    pub fn load_factory_defaults(&mut self) {
        self.bytes.fill(0);

        // EEPROM Identification
        self.write_u16(REG_MODEL_NUMBER, 0x0503);
        self.write_u8(REG_FIRMWARE_VERSION, 0x01);
        self.write_u8(REG_PROTOCOL_VERSION, 0x02);
        self.write_u8(REG_BOARD_NODE_ID, 0x40); // Default address 0x40
        self.write_u8(REG_JOINT0_ADDR, 0);
        self.write_u8(REG_JOINT1_ADDR, 1);
        self.write_u8(REG_JOINT2_ADDR, 2);
        self.write_u8(REG_JOINT3_ADDR, 3);
        self.write_u8(REG_OPERATING_MODE, OperatingMode::Position as u8);
        self.write_u8(REG_SECONDARY_ID, 0xFD);
        self.write_u8(REG_TOPOLOGY_PRESET, TopologyPreset::Independent4x1Dof as u8);
        self.write_u8(REG_COUPLING_MASK, 0x00);

        // Soft Limits & Protection
        self.write_f32(REG_HOMING_OFFSET, 0.0);
        self.write_f32(REG_CURRENT_LIMIT, 1.5); // 1.5 A for DRV8411
        self.write_f32(REG_ACCELERATION_LIMIT, 50.0); // 50 rad/s^2
        self.write_f32(REG_VELOCITY_LIMIT, 25.0); // 25 rad/s
        self.write_f32(REG_MIN_POSITION_LIMIT, -3.14159265);
        self.write_f32(REG_MAX_POSITION_LIMIT, 3.14159265);
        self.write_f32(REG_VOLTAGE_LIMIT, 5.0); // 5.0 V
        self.write_f32(REG_TEMPERATURE_LIMIT, 75.0); // 75 deg C thermal trip
        self.write_f32(REG_MIN_BUS_VOLTAGE, 3.0); // 3.0 V undervoltage
        self.write_f32(REG_MAX_BUS_VOLTAGE, 7.0); // 7.0 V overvoltage

        // Default Control Gains
        self.write_f32(REG_POS_P_GAIN, 20.0);
        self.write_f32(REG_POS_I_GAIN, 0.0);
        self.write_f32(REG_POS_D_GAIN, 0.1);

        self.write_f32(REG_VEL_P_GAIN, 0.5);
        self.write_f32(REG_VEL_I_GAIN, 10.0);
        self.write_f32(REG_VEL_D_GAIN, 0.0);

        self.write_f32(REG_CURRENT_P_GAIN, 3.0);
        self.write_f32(REG_CURRENT_I_GAIN, 300.0);
        self.write_f32(REG_CURRENT_D_GAIN, 0.0);

        // Volatile defaults
        self.write_u8(REG_TORQUE_ENABLE, 0); // Disabled by default
        self.write_f32(REG_PROFILE_VELOCITY, 10.0);
        self.write_f32(REG_PROFILE_ACCELERATION, 25.0);
    }

    /// Read an immutable byte slice of the entire 256-byte table.
    pub fn as_bytes(&self) -> &[u8; CONTROL_TABLE_SIZE] {
        &self.bytes
    }

    /// Access the raw byte buffer mutably.
    pub fn as_bytes_mut(&mut self) -> &mut [u8; CONTROL_TABLE_SIZE] {
        &mut self.bytes
    }

    /// Read an immutable view of the 160-byte non-volatile EEPROM section.
    pub fn eeprom_bytes(&self) -> &[u8; EEPROM_AREA_SIZE] {
        let slice: &[u8] = &self.bytes[0..EEPROM_AREA_SIZE];
        slice.try_into().unwrap()
    }

    /// Read a mutable view of the 160-byte non-volatile EEPROM section.
    pub fn eeprom_bytes_mut(&mut self) -> &mut [u8; EEPROM_AREA_SIZE] {
        let slice: &mut [u8] = &mut self.bytes[0..EEPROM_AREA_SIZE];
        slice.try_into().unwrap()
    }

    /// Read an immutable view of the 96-byte volatile RAM section.
    pub fn ram_bytes(&self) -> &[u8; RAM_AREA_SIZE] {
        let slice: &[u8] = &self.bytes[EEPROM_AREA_SIZE..CONTROL_TABLE_SIZE];
        slice.try_into().unwrap()
    }

    /// Read a mutable view of the 96-byte volatile RAM section.
    pub fn ram_bytes_mut(&mut self) -> &mut [u8; RAM_AREA_SIZE] {
        let slice: &mut [u8] = &mut self.bytes[EEPROM_AREA_SIZE..CONTROL_TABLE_SIZE];
        slice.try_into().unwrap()
    }

    // Direct offset helpers
    pub fn read_u8(&self, offset: usize) -> u8 {
        if offset < CONTROL_TABLE_SIZE {
            self.bytes[offset]
        } else {
            0
        }
    }

    pub fn write_u8(&mut self, offset: usize, val: u8) {
        if offset < CONTROL_TABLE_SIZE {
            self.bytes[offset] = val;
        }
    }

    pub fn read_u16(&self, offset: usize) -> u16 {
        if offset + 2 <= CONTROL_TABLE_SIZE {
            u16::from_le_bytes([self.bytes[offset], self.bytes[offset + 1]])
        } else {
            0
        }
    }

    pub fn write_u16(&mut self, offset: usize, val: u16) {
        if offset + 2 <= CONTROL_TABLE_SIZE {
            let b = val.to_le_bytes();
            self.bytes[offset..offset + 2].copy_from_slice(&b);
        }
    }

    pub fn read_i32(&self, offset: usize) -> i32 {
        if offset + 4 <= CONTROL_TABLE_SIZE {
            i32::from_le_bytes([
                self.bytes[offset],
                self.bytes[offset + 1],
                self.bytes[offset + 2],
                self.bytes[offset + 3],
            ])
        } else {
            0
        }
    }

    pub fn write_i32(&mut self, offset: usize, val: i32) {
        if offset + 4 <= CONTROL_TABLE_SIZE {
            let b = val.to_le_bytes();
            self.bytes[offset..offset + 4].copy_from_slice(&b);
        }
    }

    pub fn read_f32(&self, offset: usize) -> f32 {
        if offset + 4 <= CONTROL_TABLE_SIZE {
            f32::from_le_bytes([
                self.bytes[offset],
                self.bytes[offset + 1],
                self.bytes[offset + 2],
                self.bytes[offset + 3],
            ])
        } else {
            0.0
        }
    }

    pub fn write_f32(&mut self, offset: usize, val: f32) {
        if offset + 4 <= CONTROL_TABLE_SIZE {
            let b = val.to_le_bytes();
            self.bytes[offset..offset + 4].copy_from_slice(&b);
        }
    }

    // Typed field getters and setters
    /// Read hardware silicon UID (12 bytes, RO).
    pub fn hardware_uid(&self) -> &[u8; 12] {
        let slice = &self.bytes[REG_HARDWARE_UID..REG_HARDWARE_UID + 12];
        slice.try_into().unwrap()
    }

    /// Set hardware silicon UID (12 bytes).
    pub fn set_hardware_uid(&mut self, uid: &[u8; 12]) {
        self.bytes[REG_HARDWARE_UID..REG_HARDWARE_UID + 12].copy_from_slice(uid);
    }

    /// Read user-configurable board name (up to 15 bytes null-padded UTF-8).
    pub fn board_name(&self) -> &str {
        let slice = &self.bytes[REG_BOARD_NAME..REG_BOARD_NAME + 15];
        let len = slice.iter().position(|&b| b == 0).unwrap_or(slice.len());
        core::str::from_utf8(&slice[..len]).unwrap_or("")
    }

    /// Set user-configurable board name (up to 15 bytes null-padded UTF-8).
    pub fn set_board_name(&mut self, name: &str) {
        let bytes = name.as_bytes();
        let copy_len = bytes.len().min(15);
        self.bytes[REG_BOARD_NAME..REG_BOARD_NAME + copy_len].copy_from_slice(&bytes[..copy_len]);
        self.bytes[REG_BOARD_NAME + copy_len..REG_BOARD_NAME + 15].fill(0);
    }

    /// Read board node ID (device address, 0..127).
    pub fn board_node_id(&self) -> u8 {
        self.read_u8(REG_BOARD_NODE_ID)
    }

    /// Set board node ID (device address, 0..127).
    pub fn set_board_node_id(&mut self, id: u8) {
        self.write_u8(REG_BOARD_NODE_ID, id);
    }

    /// Read joint servo address (0..127) for joint `index` (0..3).
    pub fn joint_addr(&self, index: usize) -> u8 {
        match index {
            0 => self.read_u8(REG_JOINT0_ADDR),
            1 => self.read_u8(REG_JOINT1_ADDR),
            2 => self.read_u8(REG_JOINT2_ADDR),
            3 => self.read_u8(REG_JOINT3_ADDR),
            _ => 0,
        }
    }

    /// Set joint servo address (0..127) for joint `index` (0..3).
    pub fn set_joint_addr(&mut self, index: usize, addr: u8) {
        let clamped = addr.min(127);
        let offset = match index {
            0 => REG_JOINT0_ADDR,
            1 => REG_JOINT1_ADDR,
            2 => REG_JOINT2_ADDR,
            3 => REG_JOINT3_ADDR,
            _ => return,
        };
        self.write_u8(offset, clamped);
    }

    /// Read joint name (up to 15 bytes null-padded UTF-8) for joint `index` (0..3).
    pub fn joint_name(&self, index: usize) -> &str {
        let offset = match index {
            0 => REG_JOINT0_NAME,
            1 => REG_JOINT1_NAME,
            2 => REG_JOINT2_NAME,
            3 => REG_JOINT3_NAME,
            _ => return "",
        };
        let slice = &self.bytes[offset..offset + 15];
        let len = slice.iter().position(|&b| b == 0).unwrap_or(slice.len());
        core::str::from_utf8(&slice[..len]).unwrap_or("")
    }

    /// Set joint name (up to 15 bytes null-padded UTF-8) for joint `index` (0..3).
    pub fn set_joint_name(&mut self, index: usize, name: &str) {
        let offset = match index {
            0 => REG_JOINT0_NAME,
            1 => REG_JOINT1_NAME,
            2 => REG_JOINT2_NAME,
            3 => REG_JOINT3_NAME,
            _ => return,
        };
        let bytes = name.as_bytes();
        let copy_len = bytes.len().min(15);
        self.bytes[offset..offset + copy_len].copy_from_slice(&bytes[..copy_len]);
        self.bytes[offset + copy_len..offset + 15].fill(0);
    }

    pub fn node_id(&self) -> u8 {
        self.read_u8(REG_NODE_ID)
    }

    pub fn set_node_id(&mut self, id: u8) {
        self.write_u8(REG_NODE_ID, id);
    }

    pub fn operating_mode(&self) -> OperatingMode {
        OperatingMode::try_from(self.read_u8(REG_OPERATING_MODE)).unwrap_or(OperatingMode::Position)
    }

    pub fn set_operating_mode(&mut self, mode: OperatingMode) {
        self.write_u8(REG_OPERATING_MODE, mode as u8);
    }

    pub fn topology_preset(&self) -> TopologyPreset {
        TopologyPreset::try_from(self.read_u8(REG_TOPOLOGY_PRESET))
            .unwrap_or(TopologyPreset::Independent4x1Dof)
    }

    pub fn set_topology_preset(&mut self, preset: TopologyPreset) {
        self.write_u8(REG_TOPOLOGY_PRESET, preset as u8);
    }

    pub fn coupling_mask(&self) -> u8 {
        self.read_u8(REG_COUPLING_MASK)
    }

    pub fn set_coupling_mask(&mut self, mask: u8) {
        self.write_u8(REG_COUPLING_MASK, mask);
    }

    pub fn torque_enable(&self) -> bool {
        self.read_u8(REG_TORQUE_ENABLE) != 0
    }

    pub fn set_torque_enable(&mut self, enable: bool) {
        self.write_u8(REG_TORQUE_ENABLE, if enable { 1 } else { 0 });
    }

    pub fn goal_position(&self) -> f32 {
        self.read_f32(REG_GOAL_POSITION)
    }

    pub fn set_goal_position(&mut self, pos: f32) {
        self.write_f32(REG_GOAL_POSITION, pos);
    }

    pub fn goal_velocity(&self) -> f32 {
        self.read_f32(REG_GOAL_VELOCITY)
    }

    pub fn set_goal_velocity(&mut self, vel: f32) {
        self.write_f32(REG_GOAL_VELOCITY, vel);
    }

    pub fn goal_current(&self) -> f32 {
        self.read_f32(REG_GOAL_CURRENT)
    }

    pub fn set_goal_current(&mut self, current: f32) {
        self.write_f32(REG_GOAL_CURRENT, current);
    }

    pub fn goal_voltage(&self) -> f32 {
        self.read_f32(REG_GOAL_VOLTAGE)
    }

    pub fn set_goal_voltage(&mut self, volt: f32) {
        self.write_f32(REG_GOAL_VOLTAGE, volt);
    }

    pub fn current_limit(&self) -> f32 {
        self.read_f32(REG_CURRENT_LIMIT)
    }

    pub fn velocity_limit(&self) -> f32 {
        self.read_f32(REG_VELOCITY_LIMIT)
    }

    pub fn acceleration_limit(&self) -> f32 {
        self.read_f32(REG_ACCELERATION_LIMIT)
    }

    pub fn min_position_limit(&self) -> f32 {
        self.read_f32(REG_MIN_POSITION_LIMIT)
    }

    pub fn max_position_limit(&self) -> f32 {
        self.read_f32(REG_MAX_POSITION_LIMIT)
    }

    pub fn voltage_limit(&self) -> f32 {
        self.read_f32(REG_VOLTAGE_LIMIT)
    }

    pub fn temperature_limit(&self) -> f32 {
        self.read_f32(REG_TEMPERATURE_LIMIT)
    }

    pub fn min_bus_voltage(&self) -> f32 {
        self.read_f32(REG_MIN_BUS_VOLTAGE)
    }

    pub fn max_bus_voltage(&self) -> f32 {
        self.read_f32(REG_MAX_BUS_VOLTAGE)
    }

    pub fn hardware_error_status(&self) -> u16 {
        self.read_u16(REG_HARDWARE_ERROR_STATUS)
    }

    pub fn set_hardware_error_status(&mut self, status: u16) {
        self.write_u16(REG_HARDWARE_ERROR_STATUS, status);
    }

    pub fn set_error_flag(&mut self, flag: u16) {
        let cur = self.hardware_error_status();
        self.set_hardware_error_status(cur | flag);
    }

    pub fn clear_error_flag(&mut self, flag: u16) {
        let cur = self.hardware_error_status();
        self.set_hardware_error_status(cur & !flag);
    }

    /// Enforce soft travel and rate limits on setpoints based on configured limits.
    pub fn enforce_limits(&mut self) {
        let min_pos = self.min_position_limit();
        let max_pos = self.max_position_limit();
        if min_pos < max_pos {
            let goal_p = self.goal_position();
            if goal_p < min_pos {
                self.set_goal_position(min_pos);
                self.set_error_flag(error_flags::POSITION_LIMIT);
            } else if goal_p > max_pos {
                self.set_goal_position(max_pos);
                self.set_error_flag(error_flags::POSITION_LIMIT);
            }
        }

        let vel_lim = self.velocity_limit();
        if vel_lim > 0.0 {
            let goal_v = self.goal_velocity();
            if libm::fabsf(goal_v) > vel_lim {
                let clamped = if goal_v > 0.0 { vel_lim } else { -vel_lim };
                self.set_goal_velocity(clamped);
                self.set_error_flag(error_flags::VELOCITY_LIMIT);
            }
        }

        let curr_lim = self.current_limit();
        if curr_lim > 0.0 {
            let goal_i = self.goal_current();
            if libm::fabsf(goal_i) > curr_lim {
                let clamped = if goal_i > 0.0 { curr_lim } else { -curr_lim };
                self.set_goal_current(clamped);
            }
        }

        let volt_lim = self.voltage_limit();
        if volt_lim > 0.0 {
            let goal_u = self.goal_voltage();
            if libm::fabsf(goal_u) > volt_lim {
                let clamped = if goal_u > 0.0 { volt_lim } else { -volt_lim };
                self.set_goal_voltage(clamped);
            }
        }
    }

    /// Evaluate hardware telemetry against safety cutoffs and update `HardwareErrorStatus`.
    ///
    /// If an overcurrent or thermal shutdown trip occurs, automatically disarms motor torque.
    pub fn evaluate_errors(&mut self, current: f32, bus_voltage: f32, temp: f32) {
        // Overcurrent evaluation
        let curr_lim = self.current_limit();
        if curr_lim > 0.0 && libm::fabsf(current) > curr_lim {
            self.set_error_flag(error_flags::OVERCURRENT);
            self.set_torque_enable(false); // Disarm bridge into Coast mode
        }

        // Thermal shutdown
        let temp_lim = self.temperature_limit();
        if temp_lim > 0.0 && temp > temp_lim {
            self.set_error_flag(error_flags::OVERHEAT);
            self.set_torque_enable(false);
        }

        // Bus voltage checks
        let min_v = self.min_bus_voltage();
        let max_v = self.max_bus_voltage();
        if min_v > 0.0 && bus_voltage < min_v {
            self.set_error_flag(error_flags::UNDERVOLTAGE);
        } else {
            self.clear_error_flag(error_flags::UNDERVOLTAGE);
        }

        if max_v > 0.0 && bus_voltage > max_v {
            self.set_error_flag(error_flags::OVERVOLTAGE);
        } else {
            self.clear_error_flag(error_flags::OVERVOLTAGE);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_control_table_size_and_partitions() {
        let table = ControlTable::new();
        assert_eq!(table.as_bytes().len(), 256);
        assert_eq!(table.eeprom_bytes().len(), 160);
        assert_eq!(table.ram_bytes().len(), 96);
    }

    #[test]
    fn test_board_identity_accessors() {
        let mut table = ControlTable::new();
        // Hardware UID
        let uid = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C];
        table.set_hardware_uid(&uid);
        assert_eq!(table.hardware_uid(), &uid);

        // Board node id
        assert_eq!(table.board_node_id(), 0x40);
        table.set_board_node_id(0x42);
        assert_eq!(table.board_node_id(), 0x42);
        assert_eq!(table.node_id(), 0x42);

        // Board name
        table.set_board_name("wrist_node");
        assert_eq!(table.board_name(), "wrist_node");
        // Test null padding and truncation beyond 15 bytes
        table.set_board_name("a_very_long_board_name_exceeding_15");
        assert_eq!(table.board_name().len(), 15);
        assert_eq!(table.board_name(), &"a_very_long_board_name_exceeding_15"[..15]);
    }

    #[test]
    fn test_joint_identity_accessors_and_bounds() {
        let mut table = ControlTable::new();
        // Defaults
        assert_eq!(table.joint_addr(0), 0);
        assert_eq!(table.joint_addr(1), 1);
        assert_eq!(table.joint_addr(2), 2);
        assert_eq!(table.joint_addr(3), 3);
        assert_eq!(table.joint_name(0), "");

        // Set joint addr and bounds clamping to 0-127
        table.set_joint_addr(0, 5);
        assert_eq!(table.joint_addr(0), 5);
        table.set_joint_addr(1, 200); // Exceeds 127
        assert_eq!(table.joint_addr(1), 127);

        // Joint naming and null padding
        table.set_joint_name(0, "wrist_pitch");
        assert_eq!(table.joint_name(0), "wrist_pitch");
        table.set_joint_name(1, "wrist_yaw");
        assert_eq!(table.joint_name(1), "wrist_yaw");

        // Out of bounds joint index
        assert_eq!(table.joint_addr(4), 0);
        assert_eq!(table.joint_name(4), "");
        table.set_joint_addr(4, 10); // No-op
        table.set_joint_name(4, "invalid"); // No-op
    }

    #[test]
    fn test_control_table_factory_defaults_identity() {
        let mut table = ControlTable::empty();
        table.set_board_name("custom");
        table.set_joint_addr(0, 50);
        table.set_joint_name(0, "test_joint");
        table.load_factory_defaults();
        assert_eq!(table.board_node_id(), 0x40);
        assert_eq!(table.board_name(), "");
        assert_eq!(table.joint_addr(0), 0);
        assert_eq!(table.joint_addr(1), 1);
        assert_eq!(table.joint_addr(2), 2);
        assert_eq!(table.joint_addr(3), 3);
        assert_eq!(table.joint_name(0), "");
    }

    #[test]
    fn test_flash_persistence_eeprom_roundtrip() {
        let mut table = ControlTable::new();
        let uid = [0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        table.set_hardware_uid(&uid);
        table.set_board_name("wrist_node");
        table.set_joint_addr(0, 5);
        table.set_joint_name(0, "wrist_pitch");
        table.set_joint_addr(1, 6);
        table.set_joint_name(1, "wrist_yaw");

        let saved_eeprom = *table.eeprom_bytes();

        let mut restored = ControlTable::empty();
        restored.eeprom_bytes_mut().copy_from_slice(&saved_eeprom);

        assert_eq!(restored.hardware_uid(), &uid);
        assert_eq!(restored.board_name(), "wrist_node");
        assert_eq!(restored.joint_addr(0), 5);
        assert_eq!(restored.joint_name(0), "wrist_pitch");
        assert_eq!(restored.joint_addr(1), 6);
        assert_eq!(restored.joint_name(1), "wrist_yaw");
    }

    #[test]
    fn test_default_values_and_mode_switching() {
        let mut table = ControlTable::new();
        assert_eq!(table.node_id(), 0x40);
        assert_eq!(table.operating_mode(), OperatingMode::Position);

        // Switch mode at runtime
        table.set_operating_mode(OperatingMode::MultiTurnPosition);
        assert_eq!(table.operating_mode(), OperatingMode::MultiTurnPosition);
        assert_eq!(table.read_u8(REG_OPERATING_MODE), 3);

        // Topology preset
        assert_eq!(table.topology_preset(), TopologyPreset::Independent4x1Dof);
        table.set_topology_preset(TopologyPreset::Differential2x2Dof);
        assert_eq!(table.topology_preset(), TopologyPreset::Differential2x2Dof);
    }

    #[test]
    fn test_serialization_roundtrip() {
        let mut table = ControlTable::new();
        table.write_f32(REG_GOAL_POSITION, 1.2345);
        table.write_f32(REG_GOAL_VELOCITY, -10.5);
        table.write_u16(REG_HARDWARE_ERROR_STATUS, 0x1234);

        assert_eq!(table.read_f32(REG_GOAL_POSITION), 1.2345);
        assert_eq!(table.read_f32(REG_GOAL_VELOCITY), -10.5);
        assert_eq!(table.hardware_error_status(), 0x1234);
    }

    #[test]
    fn test_limits_clamping() {
        let mut table = ControlTable::new();
        table.write_f32(REG_MIN_POSITION_LIMIT, -1.0);
        table.write_f32(REG_MAX_POSITION_LIMIT, 2.0);
        table.write_f32(REG_VELOCITY_LIMIT, 10.0);

        // Within limits
        table.set_goal_position(0.5);
        table.enforce_limits();
        assert_eq!(table.goal_position(), 0.5);

        // Exceed max pos
        table.set_goal_position(3.0);
        table.enforce_limits();
        assert_eq!(table.goal_position(), 2.0);
        assert_ne!(table.hardware_error_status() & error_flags::POSITION_LIMIT, 0);

        // Exceed min pos
        table.set_goal_position(-2.5);
        table.enforce_limits();
        assert_eq!(table.goal_position(), -1.0);

        // Velocity clamping
        table.set_goal_velocity(25.0);
        table.enforce_limits();
        assert_eq!(table.goal_velocity(), 10.0);
        assert_ne!(table.hardware_error_status() & error_flags::VELOCITY_LIMIT, 0);
    }

    #[test]
    fn test_error_evaluation_trips() {
        let mut table = ControlTable::new();
        table.set_torque_enable(true);
        assert!(table.torque_enable());

        // Nominal operating condition
        table.evaluate_errors(0.5, 5.0, 45.0);
        assert_eq!(table.hardware_error_status(), 0);
        assert!(table.torque_enable());

        // Overcurrent trip
        table.evaluate_errors(2.0, 5.0, 45.0); // Limit is 1.5A
        assert_ne!(table.hardware_error_status() & error_flags::OVERCURRENT, 0);
        assert!(!table.torque_enable()); // Must disarm into Coast mode

        // Overheating trip
        table.set_torque_enable(true);
        table.evaluate_errors(0.5, 5.0, 80.0); // Limit is 75C
        assert_ne!(table.hardware_error_status() & error_flags::OVERHEAT, 0);
        assert!(!table.torque_enable());

        // Undervoltage flag
        table.evaluate_errors(0.5, 2.5, 45.0); // Min bus is 3.0V
        assert_ne!(table.hardware_error_status() & error_flags::UNDERVOLTAGE, 0);
    }
}
