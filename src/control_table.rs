//! Universal Dynamixel-grade Smart Servo Control Table.
//!
//! Standardized, hardware-agnostic 176-byte Control Table partitioned into:
//! - EEPROM Area (`0x00..0x60`, 96 bytes): non-volatile configuration committed to Flash.
//! - RAM Area (`0x60..0xB0`, 80 bytes): volatile operational targets, setpoints, and telemetry.
//!
//! # Memory Map
//!
//! | Address | Name | Type | Access | Description |
//! |---|---|---|---|---|
//! | 0x00 | ModelNumber | u16 | RO | Device model identifier |
//! | 0x02 | FirmwareVersion | u8 | RO | Firmware revision |
//! | 0x03 | ProtocolVersion | u8 | RO | Control Table protocol version |
//! | 0x07 | NodeId | u8 | RW | Device bus target address (e.g. I2C/CAN node ID) |
//! | 0x08 | OperatingMode | u8 | RW | Motion control mode (0..7) |
//! | 0x09 | SecondaryId | u8 | RW | Secondary / group ID |
//! | 0x0A | TopologyPreset | u8 | RW | Multi-motor kinematics preset (0..3, 0xFF) |
//! | 0x0F | CouplingMask | u8 | RW | Kinematics coupling bitmask |
//! | 0x10 | HomingOffset | f32 | RW | Homing position zero offset (rad) |
//! | 0x14 | CurrentLimit | f32 | RW | Maximum allowable motor current (A) |
//! | 0x18 | AccelerationLimit | f32 | RW | Maximum acceleration limit (rad/s^2) |
//! | 0x1C | VelocityLimit | f32 | RW | Maximum velocity limit (rad/s) |
//! | 0x20 | MinPositionLimit | f32 | RW | Minimum software travel limit (rad) |
//! | 0x24 | MaxPositionLimit | f32 | RW | Maximum software travel limit (rad) |
//! | 0x28 | VoltageLimit | f32 | RW | Maximum motor phase/drive voltage (V) |
//! | 0x2C | TemperatureLimit | f32 | RW | Thermal shutdown temperature threshold (deg C) |
//! | 0x30 | MinBusVoltage | f32 | RW | Minimum bus voltage cutoff (V) |
//! | 0x34 | MaxBusVoltage | f32 | RW | Maximum bus voltage cutoff (V) |
//! | 0x38 | PosPGain | f32 | RW | Position PID proportional gain Kp |
//! | 0x3C | PosIGain | f32 | RW | Position PID integral gain Ki |
//! | 0x40 | PosDGain | f32 | RW | Position PID derivative gain Kd |
//! | 0x44 | VelPGain | f32 | RW | Velocity PID proportional gain Kp |
//! | 0x48 | VelIGain | f32 | RW | Velocity PID integral gain Ki |
//! | 0x4C | VelDGain | f32 | RW | Velocity PID derivative gain Kd |
//! | 0x50 | CurrentPGain | f32 | RW | Current loop proportional gain Kp |
//! | 0x54 | CurrentIGain | f32 | RW | Current loop integral gain Ki |
//! | 0x58 | CurrentDGain | f32 | RW | Current loop derivative gain Kd |
//! | 0x60 | TorqueEnable | u8 | RW | Motor bridge drive enable (0=Coast, 1=Enable) |
//! | 0x61 | Led | u8 | RW | Status LED output |
//! | 0x62 | StatusReturnLevel | u8 | RW | Comms response verbosity |
//! | 0x63 | RebootCommand | u8 | RW | Software reboot trigger (0xF0 = Katapult bootloader) |
//! | 0x64 | SaveConfigCommand | u8 | RW | Flash persistence trigger (0x55 = Save, 0xAA = Factory) |
//! | 0x68 | ProfileVelocity | f32 | RW | Trapezoidal trajectory maximum velocity (rad/s) |
//! | 0x6C | ProfileAcceleration | f32 | RW | Trapezoidal trajectory acceleration (rad/s^2) |
//! | 0x70 | GoalPosition | f32 | RW | Target position setpoint (rad) |
//! | 0x74 | GoalVelocity | f32 | RW | Target velocity setpoint (rad/s) |
//! | 0x78 | GoalCurrent | f32 | RW | Target current / torque setpoint (A) |
//! | 0x7C | GoalVoltage | f32 | RW | Target drive voltage setpoint (V) |
//! | 0x80 | HardwareErrorStatus | u16 | RO | Active hardware fault bitfield |
//! | 0x82 | PresentTemperature | f32 | RO | Measured internal MCU / stage temperature (deg C) |
//! | 0x84 | PresentPosition | f32 | RO | Measured shaft / joint position (rad) |
//! | 0x88 | PresentVelocity | f32 | RO | Measured shaft velocity (rad/s) |
//! | 0x8C | PresentCurrent | f32 | RO | Measured motor current (A) |
//! | 0x90 | PresentVoltage | f32 | RO | Commanded / applied phase voltage (V) |
//! | 0x94 | PresentBusVoltage | f32 | RO | Measured DC supply / bus voltage (V) |
//! | 0x98 | FilteredCurrent | f32 | RO | Low-pass filtered current telemetry (A) |
//! | 0x9C | TrajectoryPosition | f32 | RO | Instantaneous trajectory generator position setpoint (rad) |
//! | 0xA0 | Moving | u8 | RO | Motion profile active flag (1=in motion, 0=stopped) |
//! | 0xA2 | MultiTurnRevolutions | i32 | RO | Cumulative completed shaft revolutions |
//! | 0xA6 | MultiTurnAngle | f32 | RO | Cumulative unwrapped multi-turn angle (rad) |

/// Total byte capacity of the standardized Control Table.
pub const CONTROL_TABLE_SIZE: usize = 176;

/// Byte offset marking the boundary between EEPROM and RAM areas.
pub const EEPROM_AREA_SIZE: usize = 96;

/// Byte size of the volatile RAM register area.
pub const RAM_AREA_SIZE: usize = 80;

// Register byte offsets
pub const REG_MODEL_NUMBER: usize = 0x00;
pub const REG_FIRMWARE_VERSION: usize = 0x02;
pub const REG_PROTOCOL_VERSION: usize = 0x03;
pub const REG_NODE_ID: usize = 0x07;
pub const REG_OPERATING_MODE: usize = 0x08;
pub const REG_SECONDARY_ID: usize = 0x09;
pub const REG_TOPOLOGY_PRESET: usize = 0x0A;
pub const REG_COUPLING_MASK: usize = 0x0F;
pub const REG_HOMING_OFFSET: usize = 0x10;
pub const REG_CURRENT_LIMIT: usize = 0x14;
pub const REG_ACCELERATION_LIMIT: usize = 0x18;
pub const REG_VELOCITY_LIMIT: usize = 0x1C;
pub const REG_MIN_POSITION_LIMIT: usize = 0x20;
pub const REG_MAX_POSITION_LIMIT: usize = 0x24;
pub const REG_VOLTAGE_LIMIT: usize = 0x28;
pub const REG_TEMPERATURE_LIMIT: usize = 0x2C;
pub const REG_MIN_BUS_VOLTAGE: usize = 0x30;
pub const REG_MAX_BUS_VOLTAGE: usize = 0x34;
pub const REG_POS_P_GAIN: usize = 0x38;
pub const REG_POS_I_GAIN: usize = 0x3C;
pub const REG_POS_D_GAIN: usize = 0x40;
pub const REG_VEL_P_GAIN: usize = 0x44;
pub const REG_VEL_I_GAIN: usize = 0x48;
pub const REG_VEL_D_GAIN: usize = 0x4C;
pub const REG_CURRENT_P_GAIN: usize = 0x50;
pub const REG_CURRENT_I_GAIN: usize = 0x54;
pub const REG_CURRENT_D_GAIN: usize = 0x58;

pub const REG_TORQUE_ENABLE: usize = 0x60;
pub const REG_LED: usize = 0x61;
pub const REG_STATUS_RETURN_LEVEL: usize = 0x62;
pub const REG_REBOOT_CMD: usize = 0x63;
pub const REG_SAVE_CONFIG_CMD: usize = 0x64;
pub const REG_PROFILE_VELOCITY: usize = 0x68;
pub const REG_PROFILE_ACCELERATION: usize = 0x6C;
pub const REG_GOAL_POSITION: usize = 0x70;
pub const REG_GOAL_VELOCITY: usize = 0x74;
pub const REG_GOAL_CURRENT: usize = 0x78;
pub const REG_GOAL_VOLTAGE: usize = 0x7C;
pub const REG_HARDWARE_ERROR_STATUS: usize = 0x80;
pub const REG_PRESENT_TEMPERATURE: usize = 0x82;
pub const REG_PRESENT_POSITION: usize = 0x84;
pub const REG_PRESENT_VELOCITY: usize = 0x88;
pub const REG_PRESENT_CURRENT: usize = 0x8C;
pub const REG_PRESENT_VOLTAGE: usize = 0x90;
pub const REG_PRESENT_BUS_VOLTAGE: usize = 0x94;
pub const REG_FILTERED_CURRENT: usize = 0x98;
pub const REG_TRAJECTORY_POSITION: usize = 0x9C;
pub const REG_MOVING: usize = 0xA0;
pub const REG_MULTI_TURN_REVOLUTIONS: usize = 0xA2;
pub const REG_MULTI_TURN_ANGLE: usize = 0xA6;

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
        self.write_u8(REG_NODE_ID, 0x40); // Default I2C address 0x40
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

    /// Read an immutable byte slice of the entire 176-byte table.
    pub fn as_bytes(&self) -> &[u8; CONTROL_TABLE_SIZE] {
        &self.bytes
    }

    /// Access the raw byte buffer mutably.
    pub fn as_bytes_mut(&mut self) -> &mut [u8; CONTROL_TABLE_SIZE] {
        &mut self.bytes
    }

    /// Read an immutable view of the 96-byte non-volatile EEPROM section.
    pub fn eeprom_bytes(&self) -> &[u8; EEPROM_AREA_SIZE] {
        let slice: &[u8] = &self.bytes[0..EEPROM_AREA_SIZE];
        slice.try_into().unwrap()
    }

    /// Read a mutable view of the 96-byte non-volatile EEPROM section.
    pub fn eeprom_bytes_mut(&mut self) -> &mut [u8; EEPROM_AREA_SIZE] {
        let slice: &mut [u8] = &mut self.bytes[0..EEPROM_AREA_SIZE];
        slice.try_into().unwrap()
    }

    /// Read an immutable view of the 80-byte volatile RAM section.
    pub fn ram_bytes(&self) -> &[u8; RAM_AREA_SIZE] {
        let slice: &[u8] = &self.bytes[EEPROM_AREA_SIZE..CONTROL_TABLE_SIZE];
        slice.try_into().unwrap()
    }

    /// Read a mutable view of the 80-byte volatile RAM section.
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
        assert_eq!(table.as_bytes().len(), 176);
        assert_eq!(table.eeprom_bytes().len(), 96);
        assert_eq!(table.ram_bytes().len(), 80);
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
