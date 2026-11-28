
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use barefoc::common::filter::LowPassFilter;
use barefoc::common::time::Clock;
use barefoc::kinematics::calibration::{AutonomousCalibrator, CalibrationConfig};
use barefoc::kinematics::{
    ActuatorAssembly, CoupledDifferential, DynamicTransmission, EccentricCam, FourBarBranch,
    FourBarLinkage, LinearGear, LutTransmission, MotorControl, TopologyPreset,
};
use barefoc::motor::enums::{MotionControlType, TorqueControlType};
use barefoc::safety::{SafetyConfig, SafetyInput, SafetySupervisor};
use barefoc::traits::adc::AdcPin;
use barefoc::traits::sensor::{
    Direction, Sensor, SensorBase, SensorBusControl, SensorConfig, SensorRuntimeConfig, SensorStatus,
};
use barefoc_commander::kinematics::{Commander4Axis, KinematicsCommander};
use barefoc_drivers::comms::registers::{RegisterError, RegisterId, RegisterValue};
use barefoc_drivers::comms::RegisterBridge;
use barefoc_drivers::dc::traits::{DCDriver, DCDriverParams};
use barefoc_drivers::dc::{CurrentMirrorSense, DCDriver2PWM, DCMotor, DummySensor};
use barefoc::control_table::{
    error_flags, ControlTable, ThermalReadings,
    REG_PRESENT_BUS_VOLTAGE, REG_PRESENT_CURRENT,
    REG_PRESENT_POSITION, REG_PRESENT_TEMPERATURE, REG_PRESENT_VELOCITY,
    REG_REBOOT_CMD, REG_SAVE_CONFIG_CMD, CMD_SAVE_CONFIG, CMD_REBOOT_BOOTLOADER,
    REG_POS_P_GAIN, REG_VEL_P_GAIN, REG_VEL_I_GAIN, REG_VEL_D_GAIN,
    REG_VOLTAGE_LIMIT, REG_VELOCITY_LIMIT, REG_GOAL_POSITION, REG_TOPOLOGY_PRESET,
    REG_MIN_POSITION_LIMIT, REG_MAX_POSITION_LIMIT,
};
use barefoc_stm32::stm32h5::flash_storage::{KinematicsFlashConfig, MAX_FLASH_LUT_POINTS};
use barefoc_stm32::stm32h5::H5FlashStorage;
use barefoc_multibus::profile::{
    multi_motor_profile, refresh_telemetry_multi, reg_offset, register_save_config_hook,
    register_sensor_map_hook, register_topology_mode_hook, sync_rw_from_bridge_multi,
    MultiMotorState,
};
use barefoc_multibus::table::HookResult;
use barefoc_multibus::transport::RegServer;
use barefoc_stm32::bus_target_i3c::{ServiceOutcome, Stm32I3cTarget};
use barefoc_stm32::Stm32I2cBusControl;

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::mode::Blocking;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use rtt_target::{rprintln, DownChannel};
use tmag_rs::barefoc::Tmag5273Sensor;
use tmag_rs::{AngleEnable, DeviceVariant, MagneticChannel, OperatingMode, Range, Tmag5273};

static TMAG_DETECTED: AtomicBool = AtomicBool::new(false);
static SAVE_CONFIG_REQUESTED: AtomicBool = AtomicBool::new(false);
static I3C_TARGET_ALIVE: AtomicBool = AtomicBool::new(false);
static I3C2_BUS_HEALTHY: AtomicBool = AtomicBool::new(true);
static BOOT_GUARD_CLEARED: AtomicBool = AtomicBool::new(false);

static GLOBAL_STORAGE: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<H5FlashStorage<embassy_stm32::flash::Flash<'static, Blocking>>>>,
> = Mutex::new(RefCell::new(None));

static SHARED_CONTROL_TABLE: Mutex<CriticalSectionRawMutex, RefCell<ControlTable>> =
    Mutex::new(RefCell::new(ControlTable::empty()));

/// Custom register offset for flash configuration save trigger over MultiBus (0x55).
pub const REG_SAVE_CONFIG_MULTIBUS: u16 = barefoc_multibus::REG_SAVE_CONFIG;

/// Custom register offset for runtime topology switching over MultiBus.
pub const REG_TOPOLOGY_MODE: u16 = barefoc_multibus::REG_TOPOLOGY_MODE;

/// Custom register offset for 4-channel sensor map `[u8; 4]` over MultiBus.
pub const REG_SENSOR_MAP_MULTIBUS: u16 = barefoc_multibus::REG_SENSOR_MAP;

/// Read-only register offset for `SUPPORTED_SENSORS_MASK` (`u16` LE) over MultiBus.
pub const REG_SUPPORTED_SENSORS_MULTIBUS: u16 = barefoc_multibus::REG_SUPPORTED_SENSORS;

/// Validates a `REG_MOTORx_SENSOR_TYPE` write against `bsp::SUPPORTED_SENSORS_MASK`.
#[allow(dead_code)]
pub fn validate_and_set_motor_sensor_type(
    table: &mut ControlTable,
    index: usize,
    sensor_type: u8,
) -> bool {
    if sensor_type < 16 && ((crate::bsp::SUPPORTED_SENSORS_MASK >> sensor_type) & 1) != 0 {
        table.set_motor_sensor_type(index, sensor_type);
        true
    } else {
        false
    }
}

// ---------------------------------------------------------------------------
// Soft-Reset Mailbox (0x2000_7FE0..0x2000_7FFF) & Katapult Synchronization
// ---------------------------------------------------------------------------

pub const SOFT_RESET_MAILBOX_ADDR: usize = 0x2000_7FE0;

/// Magic token placed in SRAM mailbox to request staying in the bootloader indefinitely.
pub const REQUEST_CANBOOT: u64 = 0x5984_E3FA_6CA1_589B;

/// Magic token placed in SRAM mailbox for application handoff ("FISHAPP1" in ASCII).
pub const REQUEST_APP: u64 = 0x4649_5348_4150_5031;

/// Legacy BareBoot/Katapult start application token.
pub const REQUEST_START_APP: u64 = 0x7B06_EC45_A9A8_243D;

/// Hardware STM32H503 silicon unique identifier base address (96-bit / 12-byte).
pub const UID_BASE: *const u8 = 0x08FF_F800 as *const u8;

/// Soft-reset RAM mailbox preserved across warm reboots.
/// Total size: 32 bytes (0x20) matching the reserved section at 0x2000_7FE0.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SoftResetMailbox {
    pub magic: u64,
    pub dynamic_node_id: u8,
    pub bus_type: u8,
    pub flags: u16,
    pub can_assigned_id: u32,
    pub reserved: [u8; 12],
    pub crc32: u32,
}

/// Standard IEEE 802.3 CRC32 calculation.
pub fn calculate_crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Deterministically derive a unique 7-bit node ID from the 96-bit factory silicon UID.
/// Maps into standard user I2C address range 0x10..=0x70 (97 addresses).
pub fn derive_uid_node_id(uid: &[u8; 12]) -> u8 {
    let crc = calculate_crc32(uid);
    0x10 + (crc % 97) as u8
}

impl SoftResetMailbox {
    pub fn compute_crc(&self) -> u32 {
        let bytes: [u8; 32] = unsafe { core::mem::transmute_copy(self) };
        calculate_crc32(&bytes[..28])
    }

    pub fn is_valid(&self) -> bool {
        (self.magic == REQUEST_APP || self.magic == REQUEST_CANBOOT || self.magic == REQUEST_START_APP)
            && (0x08..=0x77).contains(&self.dynamic_node_id)
            && self.crc32 == self.compute_crc()
    }

    pub fn new(magic: u64, dynamic_node_id: u8, bus_type: u8) -> Self {
        let mut mb = Self {
            magic,
            dynamic_node_id,
            bus_type,
            flags: 0,
            can_assigned_id: 0,
            reserved: [0u8; 12],
            crc32: 0,
        };
        mb.crc32 = mb.compute_crc();
        mb
    }
}

pub unsafe fn read_soft_reset_mailbox() -> Option<SoftResetMailbox> {
    let ptr = SOFT_RESET_MAILBOX_ADDR as *const SoftResetMailbox;
    let mb = core::ptr::read_volatile(ptr);
    if mb.is_valid() {
        Some(mb)
    } else {
        None
    }
}

pub unsafe fn write_soft_reset_mailbox(mb: &SoftResetMailbox) {
    let ptr = SOFT_RESET_MAILBOX_ADDR as *mut SoftResetMailbox;
    core::ptr::write_volatile(ptr, *mb);
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
}

const DEFAULT_COMM_ADDRESS: u8 = match crate::bsp::COMM_ADDRESS {
    Some(addr) => addr,
    None => crate::bsp::DEFAULT_NODE_ID,
};
static ACTIVE_NODE_ID: AtomicU8 = AtomicU8::new(DEFAULT_COMM_ADDRESS);


// ---------------------------------------------------------------------------
// System Clock implementing barefoc::common::time::Clock via Embassy Instant
// ---------------------------------------------------------------------------
#[derive(Clone, Copy)]
pub struct EmbassyClock;

impl Clock for EmbassyClock {
    fn now_us(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    fn delay_ms(&self, ms: u32) {
        embassy_time::block_for(Duration::from_millis(ms as u64));
    }
}

static CLOCK: EmbassyClock = EmbassyClock;

// ---------------------------------------------------------------------------
// ADC1 Channel Multiplexer & AdcPin Adapter
// ---------------------------------------------------------------------------
static GLOBAL_ADC: Mutex<
    ThreadModeRawMutex,
    RefCell<Option<Adc<'static, crate::bsp::embassy_bsp::VbusAdcPeripheral, Blocking>>>,
> = Mutex::new(RefCell::new(None));

pub enum AdcInputPin {
    ChA(Peri<'static, crate::bsp::embassy_bsp::Ch3AdcPin>),
    ChB(Peri<'static, crate::bsp::embassy_bsp::Ch1AdcPin>),
    ChC(Peri<'static, crate::bsp::embassy_bsp::Ch2AdcPin>),
    ChD(Peri<'static, crate::bsp::embassy_bsp::Ch0AdcPin>),
    Vsens(Peri<'static, crate::bsp::embassy_bsp::VbusAdcPin>),
}

pub struct Stm32AdcPin {
    pin: AdcInputPin,
    sample_time: SampleTime,
}

impl Stm32AdcPin {
    pub fn new(pin: AdcInputPin, sample_time: SampleTime) -> Self {
        Self { pin, sample_time }
    }
}

impl AdcPin for Stm32AdcPin {
    fn read(&mut self) -> u16 {
        let mut sum: u32 = 0;
        const SAMPLES: u32 = 8;
        GLOBAL_ADC.lock(|cell| {
            if let Some(ref mut adc) = *cell.borrow_mut() {
                for _ in 0..SAMPLES {
                    let val = match &mut self.pin {
                        AdcInputPin::ChA(pin) => adc.blocking_read(pin, self.sample_time),
                        AdcInputPin::ChB(pin) => adc.blocking_read(pin, self.sample_time),
                        AdcInputPin::ChC(pin) => adc.blocking_read(pin, self.sample_time),
                        AdcInputPin::ChD(pin) => adc.blocking_read(pin, self.sample_time),
                        AdcInputPin::Vsens(pin) => adc.blocking_read(pin, self.sample_time),
                    };
                    sum += val as u32;
                }
            }
        });
        (sum / SAMPLES) as u16
    }
}

/// Read internal MCU core temperature sensor in degrees Celsius.
pub fn read_internal_temperature() -> f32 {
    let mut sum: u32 = 0;
    const SAMPLES: u32 = 4;
    GLOBAL_ADC.lock(|cell| {
        if let Some(ref mut adc) = *cell.borrow_mut() {
            let mut temp_ch = adc.enable_temperature();
            for _ in 0..SAMPLES {
                sum += adc.blocking_read(&mut temp_ch, SampleTime::Cycles65) as u32;
            }
        }
    });
    let avg = (sum / SAMPLES) as f32;
    // Conversion for STM32H5 internal temp sensor: ~0.70V at 25C, 2.5mV/C
    let v_sense = (avg * 3.3) / 4095.0;
    ((v_sense - 0.70) / 0.0025) + 25.0
}


// ---------------------------------------------------------------------------
// Bridge A Sign-Magnitude H-Bridge Driver (PA15 DIR, PB5 PWM on TIM1_CH3)
// ---------------------------------------------------------------------------
pub struct BridgeADriver {
    params: DCDriverParams,
    dir_pin: Output<'static>,
    pwm_ch: SimplePwmChannel<'static, crate::bsp::embassy_bsp::PwmTimer0>,
}

impl BridgeADriver {
    pub fn new(
        dir_pin: Output<'static>,
        pwm_ch: SimplePwmChannel<'static, crate::bsp::embassy_bsp::PwmTimer0>,
    ) -> Self {
        Self {
            params: DCDriverParams::new(),
            dir_pin,
            pwm_ch,
        }
    }
}

impl DCDriver for BridgeADriver {
    fn init(&mut self) -> Result<(), ()> {
        self.params.initialized = true;
        if self.params.voltage_power_supply > 0.0 && self.params.voltage_limit > self.params.voltage_power_supply {
            self.params.voltage_limit = self.params.voltage_power_supply;
        }
        self.dir_pin.set_low();
        self.pwm_ch.set_duty_cycle(0);
        Ok(())
    }

    fn enable(&mut self) {}

    fn disable(&mut self) {
        self.set_pwm(0.0);
    }

    fn set_pwm(&mut self, voltage: f32) {
        let v_limit = self.params.voltage_limit;
        let v_supply = if self.params.voltage_power_supply > 0.0 {
            self.params.voltage_power_supply
        } else {
            1.0
        };
        let max_duty = self.pwm_ch.max_duty_cycle() as f32;

        if voltage > 0.0 {
            let u = if voltage > v_limit { v_limit } else { voltage };
            let duty_fraction = (u / v_supply).clamp(0.0, 1.0);
            self.dir_pin.set_high();
            let duty = ((1.0 - duty_fraction) * max_duty) as u32;
            self.pwm_ch.set_duty_cycle(duty);
        } else if voltage < 0.0 {
            let abs_v = -voltage;
            let u = if abs_v > v_limit { v_limit } else { abs_v };
            let duty_fraction = (u / v_supply).clamp(0.0, 1.0);
            self.dir_pin.set_low();
            let duty = (duty_fraction * max_duty) as u32;
            self.pwm_ch.set_duty_cycle(duty);
        } else {
            self.dir_pin.set_low();
            self.pwm_ch.set_duty_cycle(0);
        }
    }

    fn params(&self) -> &DCDriverParams {
        &self.params
    }

    fn params_mut(&mut self) -> &mut DCDriverParams {
        &mut self.params
    }
}

// ---------------------------------------------------------------------------
// Quad H-Bridge Unified DCDriver Enum
// ---------------------------------------------------------------------------
pub enum QuadDCDriver {
    A(BridgeADriver),
    B(DCDriver2PWM<SimplePwmChannel<'static, crate::bsp::embassy_bsp::PwmTimer0>, SimplePwmChannel<'static, crate::bsp::embassy_bsp::PwmTimer1>>),
    C(DCDriver2PWM<SimplePwmChannel<'static, crate::bsp::embassy_bsp::PwmTimer1>, SimplePwmChannel<'static, crate::bsp::embassy_bsp::PwmTimer1>>),
    D(DCDriver2PWM<SimplePwmChannel<'static, crate::bsp::embassy_bsp::PwmTimer0>, SimplePwmChannel<'static, crate::bsp::embassy_bsp::PwmTimer1>>),
}

impl DCDriver for QuadDCDriver {
    fn init(&mut self) -> Result<(), ()> {
        match self {
            Self::A(d) => d.init(),
            Self::B(d) => d.init(),
            Self::C(d) => d.init(),
            Self::D(d) => d.init(),
        }
    }

    fn enable(&mut self) {
        match self {
            Self::A(d) => d.enable(),
            Self::B(d) => d.enable(),
            Self::C(d) => d.enable(),
            Self::D(d) => d.enable(),
        }
    }

    fn disable(&mut self) {
        match self {
            Self::A(d) => d.disable(),
            Self::B(d) => d.disable(),
            Self::C(d) => d.disable(),
            Self::D(d) => d.disable(),
        }
    }

    fn set_pwm(&mut self, voltage: f32) {
        match self {
            Self::A(d) => d.set_pwm(voltage),
            Self::B(d) => d.set_pwm(voltage),
            Self::C(d) => d.set_pwm(voltage),
            Self::D(d) => d.set_pwm(voltage),
        }
    }

    fn params(&self) -> &DCDriverParams {
        match self {
            Self::A(d) => d.params(),
            Self::B(d) => d.params(),
            Self::C(d) => d.params(),
            Self::D(d) => d.params(),
        }
    }

    fn params_mut(&mut self) -> &mut DCDriverParams {
        match self {
            Self::A(d) => d.params_mut(),
            Self::B(d) => d.params_mut(),
            Self::C(d) => d.params_mut(),
            Self::D(d) => d.params_mut(),
        }
    }
}

// ---------------------------------------------------------------------------
// Shared Motor State & MultiBus Register Bridge
// ---------------------------------------------------------------------------
#[derive(Clone, Copy)]
pub struct SharedServoState {
    pub target: f32,
    pub current_angle: f32,
    pub current_velocity: f32,
    pub current_ma: f32,
    pub p_angle: f32,
    pub pid_vel_p: f32,
    pub pid_vel_i: f32,
    pub pid_vel_d: f32,
    pub voltage_limit: f32,
    pub current_limit: f32,
    pub velocity_limit: f32,
    pub min_position_limit: f32,
    pub max_position_limit: f32,
    pub stall_timeout_ms: u32,
    pub hardware_error_status: u16,
    pub bus_voltage: f32,
    pub temperature_c: f32,
    pub calib_cmd: u8,
    pub enabled: bool,
    pub status: u8,
    pub controller_mode: u8, // 0 = angle, 1 = velocity_openloop, 2 = torque (voltage), 3 = velocity
    pub homing_requested: bool,
    pub homing_voltage: f32,
    pub stall_threshold_ma: f32,
    pub sensor_bus_freq_hz: u32,
    pub sensor_range: u8,
    pub sensor_averaging: u8,
    pub sensor_crc_enable: u8,
    pub sensor_fault_status: u8,
}

impl SharedServoState {
    pub const fn new() -> Self {
        Self {
            target: 0.0,
            current_angle: 0.0,
            current_velocity: 0.0,
            current_ma: 0.0,
            p_angle: 20.0,
            pid_vel_p: 0.2,
            pid_vel_i: 2.0,
            pid_vel_d: 0.001,
            voltage_limit: 5.0,
            current_limit: 0.6,
            velocity_limit: 50.0,
            min_position_limit: 0.0,
            max_position_limit: 0.0,
            stall_timeout_ms: 200,
            hardware_error_status: 0,
            bus_voltage: 5.0,
            temperature_c: 25.0,
            calib_cmd: 0,
            enabled: false,
            status: 1, // Ready
            controller_mode: 0,
            homing_requested: false,
            homing_voltage: 4.0,
            stall_threshold_ma: 100.0,
            sensor_bus_freq_hz: 1_000_000,
            sensor_range: 1,
            sensor_averaging: 0,
            sensor_crc_enable: 0,
            sensor_fault_status: 0,
        }
    }
}

impl RegisterBridge for SharedServoState {
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::Status => Ok(RegisterValue::U8(self.status)),
            RegisterId::Target => Ok(RegisterValue::Float(self.target)),
            RegisterId::Enable => Ok(RegisterValue::U8(if self.enabled { 1 } else { 0 })),
            RegisterId::ControlMode => Ok(RegisterValue::U8(self.controller_mode)),
            RegisterId::Angle | RegisterId::SensorAngle => Ok(RegisterValue::Float(self.current_angle)),
            RegisterId::Velocity | RegisterId::SensorVelocity => {
                Ok(RegisterValue::Float(self.current_velocity))
            }
            RegisterId::CurrentQ | RegisterId::CurrentDC | RegisterId::CurrentA => {
                Ok(RegisterValue::Float(self.current_ma))
            }
            RegisterId::AngPidP => Ok(RegisterValue::Float(self.p_angle)),
            RegisterId::VelPidP => Ok(RegisterValue::Float(self.pid_vel_p)),
            RegisterId::VelPidI => Ok(RegisterValue::Float(self.pid_vel_i)),
            RegisterId::VelPidD => Ok(RegisterValue::Float(self.pid_vel_d)),
            RegisterId::VoltageLimit => Ok(RegisterValue::Float(self.voltage_limit)),
            RegisterId::CurrentLimit => Ok(RegisterValue::Float(self.current_limit)),
            RegisterId::VelocityLimit => Ok(RegisterValue::Float(self.velocity_limit)),
            RegisterId::MinPositionLimit => Ok(RegisterValue::Float(self.min_position_limit)),
            RegisterId::MaxPositionLimit => Ok(RegisterValue::Float(self.max_position_limit)),
            RegisterId::StallTimeoutMs => Ok(RegisterValue::U32(self.stall_timeout_ms)),
            RegisterId::HardwareErrorStatus => Ok(RegisterValue::U16(self.hardware_error_status)),
            RegisterId::PresentBusVoltage => Ok(RegisterValue::Float(self.bus_voltage)),
            RegisterId::PresentTemperature => Ok(RegisterValue::Float(self.temperature_c)),
            RegisterId::CalibCommand => Ok(RegisterValue::U8(self.calib_cmd)),
            RegisterId::SensorBusFreqHz => Ok(RegisterValue::U32(self.sensor_bus_freq_hz)),
            RegisterId::SensorRange => Ok(RegisterValue::U8(self.sensor_range)),
            RegisterId::SensorAveraging => Ok(RegisterValue::U8(self.sensor_averaging)),
            RegisterId::SensorCrcEnable => Ok(RegisterValue::U8(self.sensor_crc_enable)),
            RegisterId::SensorFaultStatus => Ok(RegisterValue::U8(self.sensor_fault_status)),
            RegisterId::NumMotors => Ok(RegisterValue::U8(4)),
            RegisterId::MotorAddress => Ok(RegisterValue::U8(ACTIVE_NODE_ID.load(Ordering::Relaxed))),
            RegisterId::TransmissionType
            | RegisterId::GearRatio
            | RegisterId::BacklashDelta
            | RegisterId::OmegaThresh
            | RegisterId::TorsionalStiffness
            | RegisterId::JointOffsetAlpha
            | RegisterId::JointOffsetBeta
            | RegisterId::JointTargetAlpha
            | RegisterId::JointTargetBeta
            | RegisterId::JointAngleAlpha
            | RegisterId::JointAngleBeta
            | RegisterId::JointVelocityAlpha
            | RegisterId::JointVelocityBeta
            | RegisterId::OuterDownsample
            | RegisterId::OuterPidAlpha_P
            | RegisterId::OuterPidAlpha_I
            | RegisterId::OuterPidAlpha_D
            | RegisterId::OuterPidAlpha_Lim
            | RegisterId::OuterPidAlpha_Ramp
            | RegisterId::OuterPidBeta_P
            | RegisterId::OuterPidBeta_I
            | RegisterId::OuterPidBeta_D
            | RegisterId::OuterPidBeta_Lim
            | RegisterId::OuterPidBeta_Ramp
            | RegisterId::DiffMatrix_T11
            | RegisterId::DiffMatrix_T12
            | RegisterId::DiffMatrix_T21
            | RegisterId::DiffMatrix_T22
            | RegisterId::LinkageL1
            | RegisterId::LinkageL2_L4
            | RegisterId::LutMotMin
            | RegisterId::LutMotMax
            | RegisterId::LutPoints
            | RegisterId::LutPointIndex
            | RegisterId::LutPointValue
            | RegisterId::LutStatus => SHARED_QUAD.lock(|sq| {
                let s = sq.borrow();
                match id {
                    RegisterId::TransmissionType => Ok(RegisterValue::U8(s.kin.transmission_type)),
                    RegisterId::GearRatio => Ok(RegisterValue::Float(s.kin.gear_ratio)),
                    RegisterId::BacklashDelta => Ok(RegisterValue::Float(s.kin.backlash_delta)),
                    RegisterId::OmegaThresh => Ok(RegisterValue::Float(s.kin.omega_thresh)),
                    RegisterId::TorsionalStiffness => Ok(RegisterValue::Float(s.kin.torsional_stiffness)),
                    RegisterId::JointOffsetAlpha => Ok(RegisterValue::Float(s.kin.joint_offset_alpha)),
                    RegisterId::JointOffsetBeta => Ok(RegisterValue::Float(s.kin.joint_offset_beta)),
                    RegisterId::JointTargetAlpha => Ok(RegisterValue::Float(s.joint_targets[0] - s.kin.joint_offset_alpha)),
                    RegisterId::JointTargetBeta => Ok(RegisterValue::Float(s.joint_targets[1] - s.kin.joint_offset_beta)),
                    RegisterId::JointAngleAlpha => Ok(RegisterValue::Float(s.joint_angles[0] - s.kin.joint_offset_alpha)),
                    RegisterId::JointAngleBeta => Ok(RegisterValue::Float(s.joint_angles[1] - s.kin.joint_offset_beta)),
                    RegisterId::JointVelocityAlpha => Ok(RegisterValue::Float(s.joint_velocities[0])),
                    RegisterId::JointVelocityBeta => Ok(RegisterValue::Float(s.joint_velocities[1])),
                    RegisterId::OuterDownsample => Ok(RegisterValue::U8(s.kin.outer_downsample)),
                    RegisterId::OuterPidAlpha_P => Ok(RegisterValue::Float(s.kin.outer_pid_alpha[0])),
                    RegisterId::OuterPidAlpha_I => Ok(RegisterValue::Float(s.kin.outer_pid_alpha[1])),
                    RegisterId::OuterPidAlpha_D => Ok(RegisterValue::Float(s.kin.outer_pid_alpha[2])),
                    RegisterId::OuterPidAlpha_Lim => Ok(RegisterValue::Float(s.kin.outer_pid_alpha[3])),
                    RegisterId::OuterPidAlpha_Ramp => Ok(RegisterValue::Float(s.kin.outer_pid_alpha[4])),
                    RegisterId::OuterPidBeta_P => Ok(RegisterValue::Float(s.kin.outer_pid_beta[0])),
                    RegisterId::OuterPidBeta_I => Ok(RegisterValue::Float(s.kin.outer_pid_beta[1])),
                    RegisterId::OuterPidBeta_D => Ok(RegisterValue::Float(s.kin.outer_pid_beta[2])),
                    RegisterId::OuterPidBeta_Lim => Ok(RegisterValue::Float(s.kin.outer_pid_beta[3])),
                    RegisterId::OuterPidBeta_Ramp => Ok(RegisterValue::Float(s.kin.outer_pid_beta[4])),
                    RegisterId::DiffMatrix_T11 => Ok(RegisterValue::Float(s.kin.diff_matrix[0])),
                    RegisterId::DiffMatrix_T12 => Ok(RegisterValue::Float(s.kin.diff_matrix[1])),
                    RegisterId::DiffMatrix_T21 => Ok(RegisterValue::Float(s.kin.diff_matrix[2])),
                    RegisterId::DiffMatrix_T22 => Ok(RegisterValue::Float(s.kin.diff_matrix[3])),
                    RegisterId::LinkageL1 => Ok(RegisterValue::Float(s.kin.linkage_lengths[0])),
                    RegisterId::LinkageL2_L4 => Ok(RegisterValue::Float3([
                        s.kin.linkage_lengths[1],
                        s.kin.linkage_lengths[2],
                        s.kin.linkage_lengths[3],
                    ])),
                    RegisterId::LutMotMin => Ok(RegisterValue::Float(s.kin.lut_mot_min)),
                    RegisterId::LutMotMax => Ok(RegisterValue::Float(s.kin.lut_mot_max)),
                    RegisterId::LutPoints => Ok(RegisterValue::U16(s.kin.lut_points)),
                    RegisterId::LutPointIndex => Ok(RegisterValue::U16(s.lut_point_index)),
                    RegisterId::LutPointValue => {
                        let idx = (s.lut_point_index as usize).min(MAX_FLASH_LUT_POINTS - 1);
                        Ok(RegisterValue::Float(s.kin.lut_table[idx]))
                    }
                    RegisterId::LutStatus => Ok(RegisterValue::U8(0)),
                    _ => Err(RegisterError::InvalidPayload),
                }
            }),
            _ => Err(RegisterError::InvalidPayload),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match (id, value) {
            (RegisterId::Target, RegisterValue::Float(v)) => {
                self.target = v;
                Ok(())
            }
            (RegisterId::Enable, RegisterValue::U8(v)) => {
                self.enabled = v > 0;
                Ok(())
            }
            (RegisterId::ControlMode, RegisterValue::U8(v)) => {
                self.controller_mode = v;
                Ok(())
            }
            (RegisterId::AngPidP, RegisterValue::Float(v)) => {
                self.p_angle = v;
                Ok(())
            }
            (RegisterId::VelPidP, RegisterValue::Float(v)) => {
                self.pid_vel_p = v;
                Ok(())
            }
            (RegisterId::VelPidI, RegisterValue::Float(v)) => {
                self.pid_vel_i = v;
                Ok(())
            }
            (RegisterId::VelPidD, RegisterValue::Float(v)) => {
                self.pid_vel_d = v;
                Ok(())
            }
            (RegisterId::VoltageLimit, RegisterValue::Float(v)) => {
                self.voltage_limit = v;
                Ok(())
            }
            (RegisterId::CurrentLimit, RegisterValue::Float(v)) => {
                self.current_limit = v;
                Ok(())
            }
            (RegisterId::VelocityLimit, RegisterValue::Float(v)) => {
                self.velocity_limit = v;
                Ok(())
            }
            (RegisterId::MinPositionLimit, RegisterValue::Float(v)) => {
                self.min_position_limit = v;
                Ok(())
            }
            (RegisterId::MaxPositionLimit, RegisterValue::Float(v)) => {
                self.max_position_limit = v;
                Ok(())
            }
            (RegisterId::StallTimeoutMs, RegisterValue::U32(v)) => {
                self.stall_timeout_ms = v;
                Ok(())
            }
            (RegisterId::CalibCommand, RegisterValue::U8(v)) => {
                self.calib_cmd = v;
                Ok(())
            }
            (RegisterId::SensorBusFreqHz, RegisterValue::U32(v)) => {
                self.sensor_bus_freq_hz = v;
                Ok(())
            }
            (RegisterId::SensorRange, RegisterValue::U8(v)) => {
                self.sensor_range = v;
                Ok(())
            }
            (RegisterId::SensorAveraging, RegisterValue::U8(v)) => {
                self.sensor_averaging = v;
                Ok(())
            }
            (RegisterId::SensorCrcEnable, RegisterValue::U8(v)) => {
                self.sensor_crc_enable = v;
                Ok(())
            }
            (RegisterId::MotorAddress, RegisterValue::U8(v)) => {
                if (0x08..=0x77).contains(&v) {
                    ACTIVE_NODE_ID.store(v, Ordering::Relaxed);
                }
                Ok(())
            }
            (id, value) => {
                SHARED_QUAD.lock(|sq| {
                    let mut s = sq.borrow_mut();
                    match (id, value) {
                        (RegisterId::TransmissionType, RegisterValue::U8(v)) => s.kin.transmission_type = v,
                        (RegisterId::GearRatio, RegisterValue::Float(v)) => s.kin.gear_ratio = v,
                        (RegisterId::BacklashDelta, RegisterValue::Float(v)) => s.kin.backlash_delta = v,
                        (RegisterId::OmegaThresh, RegisterValue::Float(v)) => s.kin.omega_thresh = v,
                        (RegisterId::TorsionalStiffness, RegisterValue::Float(v)) => s.kin.torsional_stiffness = v,
                        (RegisterId::JointOffsetAlpha, RegisterValue::Float(v)) => s.kin.joint_offset_alpha = v,
                        (RegisterId::JointOffsetBeta, RegisterValue::Float(v)) => s.kin.joint_offset_beta = v,
                        (RegisterId::JointTargetAlpha, RegisterValue::Float(v)) => {
                            s.joint_targets[0] = v + s.kin.joint_offset_alpha;
                        }
                        (RegisterId::JointTargetBeta, RegisterValue::Float(v)) => {
                            s.joint_targets[1] = v + s.kin.joint_offset_beta;
                        }
                        (RegisterId::OuterDownsample, RegisterValue::U8(v)) => s.kin.outer_downsample = v.max(1),
                        (RegisterId::OuterPidAlpha_P, RegisterValue::Float(v)) => s.kin.outer_pid_alpha[0] = v,
                        (RegisterId::OuterPidAlpha_I, RegisterValue::Float(v)) => s.kin.outer_pid_alpha[1] = v,
                        (RegisterId::OuterPidAlpha_D, RegisterValue::Float(v)) => s.kin.outer_pid_alpha[2] = v,
                        (RegisterId::OuterPidAlpha_Lim, RegisterValue::Float(v)) => s.kin.outer_pid_alpha[3] = v,
                        (RegisterId::OuterPidAlpha_Ramp, RegisterValue::Float(v)) => s.kin.outer_pid_alpha[4] = v,
                        (RegisterId::OuterPidBeta_P, RegisterValue::Float(v)) => s.kin.outer_pid_beta[0] = v,
                        (RegisterId::OuterPidBeta_I, RegisterValue::Float(v)) => s.kin.outer_pid_beta[1] = v,
                        (RegisterId::OuterPidBeta_D, RegisterValue::Float(v)) => s.kin.outer_pid_beta[2] = v,
                        (RegisterId::OuterPidBeta_Lim, RegisterValue::Float(v)) => s.kin.outer_pid_beta[3] = v,
                        (RegisterId::OuterPidBeta_Ramp, RegisterValue::Float(v)) => s.kin.outer_pid_beta[4] = v,
                        (RegisterId::DiffMatrix_T11, RegisterValue::Float(v)) => s.kin.diff_matrix[0] = v,
                        (RegisterId::DiffMatrix_T12, RegisterValue::Float(v)) => s.kin.diff_matrix[1] = v,
                        (RegisterId::DiffMatrix_T21, RegisterValue::Float(v)) => s.kin.diff_matrix[2] = v,
                        (RegisterId::DiffMatrix_T22, RegisterValue::Float(v)) => s.kin.diff_matrix[3] = v,
                        (RegisterId::LinkageL1, RegisterValue::Float(v)) => s.kin.linkage_lengths[0] = v,
                        (RegisterId::LinkageL2_L4, RegisterValue::Float3(v)) => {
                            s.kin.linkage_lengths[1] = v[0];
                            s.kin.linkage_lengths[2] = v[1];
                            s.kin.linkage_lengths[3] = v[2];
                        }
                        (RegisterId::LutMotMin, RegisterValue::Float(v)) => s.kin.lut_mot_min = v,
                        (RegisterId::LutMotMax, RegisterValue::Float(v)) => s.kin.lut_mot_max = v,
                        (RegisterId::LutPointIndex, RegisterValue::U16(v)) => {
                            s.lut_point_index = v.min((MAX_FLASH_LUT_POINTS - 1) as u16);
                        }
                        (RegisterId::LutPointValue, RegisterValue::Float(v)) => {
                            let idx = (s.lut_point_index as usize).min(MAX_FLASH_LUT_POINTS - 1);
                            s.kin.lut_table[idx] = v;
                            if (idx as u16) + 1 > s.kin.lut_points {
                                s.kin.lut_points = (idx as u16) + 1;
                            }
                        }
                        _ => {}
                    }
                });
                Ok(())
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Quad Multi-Motor Shared State
// ---------------------------------------------------------------------------
#[derive(Clone, Copy)]
pub struct SharedQuadState {
    pub motors: [SharedServoState; 4],
    pub topology_mode: u8,
    pub sensor_map: [u8; 4],
    pub kin: KinematicsFlashConfig,
    pub lut_point_index: u16,
    pub joint_targets: [f32; 4],
    pub joint_angles: [f32; 4],
    pub joint_velocities: [f32; 4],
    pub selected_motor: u8,
}

impl SharedQuadState {
    pub const fn new() -> Self {
        Self {
            motors: [SharedServoState::new(); 4],
            topology_mode: 0, // Four1Dof
            sensor_map: [0, 1, 2, 3],
            kin: KinematicsFlashConfig::default_const(),
            lut_point_index: 0,
            joint_targets: [0.0; 4],
            joint_angles: [0.0; 4],
            joint_velocities: [0.0; 4],
            selected_motor: 0,
        }
    }
}

/// Construct a [`DynamicTransmission`] instance from persisted/runtime [`KinematicsFlashConfig`].
fn build_dynamic_transmission(kin: &KinematicsFlashConfig) -> DynamicTransmission<MAX_FLASH_LUT_POINTS> {
    match kin.transmission_type {
        1 => DynamicTransmission::Differential(CoupledDifferential::from_matrix([
            [kin.diff_matrix[0], kin.diff_matrix[1]],
            [kin.diff_matrix[2], kin.diff_matrix[3]],
        ])),
        2 => DynamicTransmission::Lut(LutTransmission::new(
            kin.lut_mot_min,
            kin.lut_mot_max,
            kin.lut_table,
        )),
        3 => DynamicTransmission::FourBar(FourBarLinkage::new(
            kin.linkage_lengths[0],
            kin.linkage_lengths[1],
            kin.linkage_lengths[2],
            kin.linkage_lengths[3],
            FourBarBranch::Open,
        )),
        4 => DynamicTransmission::Cam(EccentricCam::new(
            kin.linkage_lengths[0],
            kin.linkage_lengths[1],
        )),
        _ => DynamicTransmission::Linear(
            LinearGear::new(kin.gear_ratio)
                .with_zero_offset(kin.joint_offset_alpha)
                .with_backlash(kin.backlash_delta, kin.omega_thresh),
        ),
    }
}

static SHARED_QUAD: Mutex<CriticalSectionRawMutex, RefCell<SharedQuadState>> =
    Mutex::new(RefCell::new(SharedQuadState::new()));

// ---------------------------------------------------------------------------
// Concrete Type Aliases & Unified Sensor Enum
// ---------------------------------------------------------------------------
#[allow(dead_code)]
pub enum N20Sensor<I2C: embedded_hal::i2c::I2c> {
    Tmag(Tmag5273Sensor<I2C>),
    Fallback(DummySensor<EmbassyClock>),
}

impl<I2C: embedded_hal::i2c::I2c> Sensor<EmbassyClock> for N20Sensor<I2C> {
    fn base(&self) -> &SensorBase {
        match self {
            Self::Tmag(s) => <Tmag5273Sensor<I2C> as Sensor<EmbassyClock>>::base(s),
            Self::Fallback(s) => <DummySensor<EmbassyClock> as Sensor<EmbassyClock>>::base(s),
        }
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        match self {
            Self::Tmag(s) => <Tmag5273Sensor<I2C> as Sensor<EmbassyClock>>::base_mut(s),
            Self::Fallback(s) => <DummySensor<EmbassyClock> as Sensor<EmbassyClock>>::base_mut(s),
        }
    }

    fn get_sensor_angle(&self) -> f32 {
        match self {
            Self::Tmag(s) => <Tmag5273Sensor<I2C> as Sensor<EmbassyClock>>::get_sensor_angle(s),
            Self::Fallback(s) => <DummySensor<EmbassyClock> as Sensor<EmbassyClock>>::get_sensor_angle(s),
        }
    }

    fn status(&self) -> SensorStatus {
        match self {
            Self::Tmag(s) => <Tmag5273Sensor<I2C> as Sensor<EmbassyClock>>::status(s),
            Self::Fallback(s) => <DummySensor<EmbassyClock> as Sensor<EmbassyClock>>::status(s),
        }
    }
}

type SharedSensorI2c = crate::hal::SharedI2cDevice<'static, I2c<'static, Blocking, Master>>;
type MotorSensorType = N20Sensor<SharedSensorI2c>;
type MotorCurrentSenseType = CurrentMirrorSense<Stm32AdcPin, EmbassyClock>;
type DCMotorType = DCMotor<'static, MotorSensorType, QuadDCDriver, MotorCurrentSenseType, EmbassyClock>;
static mut SENSOR_I2C_BUS: core::mem::MaybeUninit<RefCell<I2c<'static, Blocking, Master>>> =
    core::mem::MaybeUninit::uninit();

// ---------------------------------------------------------------------------
// Assembly Motor Adapter implementing MotorControl
// ---------------------------------------------------------------------------
pub struct AssemblyDCMotor<'a>(pub &'a mut DCMotorType);

impl<'a> MotorControl<EmbassyClock> for AssemblyDCMotor<'a> {
    fn shaft_angle(&self) -> f32 {
        self.0.shaft_angle()
    }

    fn shaft_velocity(&mut self) -> f32 {
        self.0.shaft_velocity()
    }

    fn velocity_limit(&self) -> f32 {
        self.0.velocity_limit
    }

    fn move_motor(&mut self, target: f32, _clock: &EmbassyClock) {
        self.0.move_motor(target);
    }

    fn loop_foc(&mut self, _clock: &EmbassyClock) {}
}

// Dedicated static storage to prevent stack overflow on Cortex-M33
static mut MOTORS_STORE: core::mem::MaybeUninit<[DCMotorType; 4]> = core::mem::MaybeUninit::uninit();
static mut ASSEMBLY_STORE: core::mem::MaybeUninit<ActuatorAssembly<4, 4>> = core::mem::MaybeUninit::uninit();
static mut COMMANDER_STORE: core::mem::MaybeUninit<Commander4Axis<EmbassyClock>> = core::mem::MaybeUninit::uninit();

// ---------------------------------------------------------------------------
// 1 kHz Quad Motor Control & Kinematics Task
// ---------------------------------------------------------------------------
async fn motor_task(
    motors: &'static mut [DCMotorType; 4],
    mut i2c1_bus: Stm32I2cBusControl,
    mut led_b: Output<'static>,
    mut led_r: Output<'static>,
    has_sensor: bool,
    mut down: DownChannel,
    mut uart: Option<Uart<'static, Blocking>>,
    mut vsens_pin: Stm32AdcPin,
) {

    TMAG_DETECTED.store(has_sensor, Ordering::Relaxed);
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut loop_count: u32 = 0;

    let mut supervisors: [SafetySupervisor; 4] = SHARED_CONTROL_TABLE.lock(|ct| {
        let mut cfg = SafetyConfig::from_control_table(&ct.borrow());
        cfg.max_angle_jump_rad = 1.5;
        cfg.max_enable_step_rad = 0.5;
        cfg.runaway_timeout_ms = 150;
        [
            SafetySupervisor::new(cfg),
            SafetySupervisor::new(cfg),
            SafetySupervisor::new(cfg),
            SafetySupervisor::new(cfg),
        ]
    });
    let mut calibrators: [AutonomousCalibrator; 4] = [
        AutonomousCalibrator::new(CalibrationConfig::default()),
        AutonomousCalibrator::new(CalibrationConfig::default()),
        AutonomousCalibrator::new(CalibrationConfig::default()),
        AutonomousCalibrator::new(CalibrationConfig::default()),
    ];
    let mut last_bus_voltage: f32 = 5.0;
    let mut last_mcu_temp: f32 = 25.0;
    let mut last_currents_a: [f32; 4] = [0.0; 4];

    let mut assembly = unsafe {
        let ptr = (*core::ptr::addr_of_mut!(ASSEMBLY_STORE)).as_mut_ptr();
        ptr.write(ActuatorAssembly::<4, 4>::new().with_downsample(10));
        let asm_ref = &mut *ptr;
        asm_ref.set_topology(TopologyPreset::Four1Dof);
        asm_ref
    };

    // Cold boot angle hydration in actuator assembly
    for i in 0..4 {
        let ang = motors[i].shaft_angle();
        motors[i].target = ang;
        assembly.set_joint_target(i, ang);
        assembly.set_sensor_state(i, ang, 0.0);
    }

    let mut lpf_currents = [
        LowPassFilter::new(0.015, 0.001, &CLOCK),
        LowPassFilter::new(0.015, 0.001, &CLOCK),
        LowPassFilter::new(0.015, 0.001, &CLOCK),
        LowPassFilter::new(0.015, 0.001, &CLOCK),
    ];

    let mut commander4 = unsafe {
        let ptr = (*core::ptr::addr_of_mut!(COMMANDER_STORE)).as_mut_ptr();
        ptr.write(Commander4Axis::new_with_clock(&CLOCK));
        &mut *ptr
    };

    let mut rtt_buf = [0u8; 64];
    let mut rtt_line: heapless::Vec<u8, 64> = heapless::Vec::new();
    let mut last_sensor_bus_freq_hz: u32 = i2c1_bus.bus_frequency_hz();
    let mut last_sensor_cfg: SensorConfig = SHARED_QUAD.lock(|s| {
        let m0 = s.borrow().motors[0];
        SensorConfig {
            range: m0.sensor_range,
            averaging: m0.sensor_averaging,
            power_mode: 0,
            crc_enabled: m0.sensor_crc_enable != 0,
            glitch_filter: false,
        }
    });

    let mut last_kin: KinematicsFlashConfig = SHARED_QUAD.lock(|s| s.borrow().kin);
    let mut _dyn_tx: DynamicTransmission<MAX_FLASH_LUT_POINTS> = build_dynamic_transmission(&last_kin);

    rprintln!("Quad motor task started at 1 kHz with multi-topology kinematics.");

    loop {
        ticker.next().await;
        if I3C2_BUS_HEALTHY.load(Ordering::Relaxed) {
            barefoc_stm32::feed_iwdg();
        }
        loop_count = loop_count.wrapping_add(1);

        if !BOOT_GUARD_CLEARED.load(Ordering::Relaxed)
            && loop_count >= 50
            && I3C_TARGET_ALIVE.load(Ordering::Relaxed)
            && I3C2_BUS_HEALTHY.load(Ordering::Relaxed)
        {
            barefoc_stm32::clear_boot_loop_guard();
            BOOT_GUARD_CLEARED.store(true, Ordering::Relaxed);
            rprintln!("BootLoopGuard cleared after 50 ms healthy I3C2 + motor execution.");
        }

        // 1. Fetch updated state & topology from SHARED_QUAD
        let (topology_mode, sensor_map, kin, joint_targets, motor_states) = SHARED_QUAD.lock(|s| {
            let s = s.borrow();
            (s.topology_mode, s.sensor_map, s.kin, s.joint_targets, s.motors)
        });

        // 2. Check for dynamic topology & transmission changes
        let current_preset = assembly.topology();
        let target_preset = TopologyPreset::from_u8(topology_mode);
        if current_preset != target_preset {
            assembly.set_topology(target_preset);
            rprintln!("Dynamic topology switched to {:?} (mode={})", target_preset, topology_mode);
        }
        if kin != last_kin {
            last_kin = kin;
            _dyn_tx = build_dynamic_transmission(&kin);
            assembly.outer_downsample = (kin.outer_downsample as u32).max(1);
            assembly.joints[0].gear_ratio = kin.gear_ratio;
            assembly.joints[0].backlash = kin.backlash_delta;
            assembly.joints[0].pid_p = kin.outer_pid_alpha[0];
            assembly.joints[0].pid_i = kin.outer_pid_alpha[1];
            assembly.joints[0].pid_d = kin.outer_pid_alpha[2];
            assembly.joints[1].pid_p = kin.outer_pid_beta[0];
            assembly.joints[1].pid_i = kin.outer_pid_beta[1];
            assembly.joints[1].pid_d = kin.outer_pid_beta[2];
        }

        // 2b. Apply live sensor bus frequency & sensor configuration updates
        let target_freq_hz = motor_states[0].sensor_bus_freq_hz;
        if target_freq_hz != last_sensor_bus_freq_hz && target_freq_hz > 0 {
            if let Ok(applied) = i2c1_bus.set_bus_frequency_hz(target_freq_hz) {
                last_sensor_bus_freq_hz = applied;
                rprintln!("I2C1 bus frequency updated live to {} Hz", applied);
            }
        }
        let target_sensor_cfg = SensorConfig {
            range: motor_states[0].sensor_range,
            averaging: motor_states[0].sensor_averaging,
            power_mode: 0,
            crc_enabled: motor_states[0].sensor_crc_enable != 0,
            glitch_filter: false,
        };
        if target_sensor_cfg != last_sensor_cfg {
            if let Some(N20Sensor::Tmag(ref mut tmag)) = motors[0].sensor_mut() {
                if tmag.apply_sensor_config(target_sensor_cfg).is_ok() {
                    last_sensor_cfg = target_sensor_cfg;
                    rprintln!(
                        "TMAG5273 config updated live: range={} avg={} crc={}",
                        target_sensor_cfg.range,
                        target_sensor_cfg.averaging,
                        target_sensor_cfg.crc_enabled
                    );
                }
            } else {
                last_sensor_cfg = target_sensor_cfg;
            }
        }

        // 3. Apply motor parameters, evaluate SafetySupervisor & handle CalibCmd
        let mut clamped_targets = [0.0f32; 4];
        let mut safety_faults = [0u16; 4];
        for i in 0..4 {
            let s = &motor_states[i];

            // Handle AutonomousCalibrator trigger (CalibCommand == 1 -> discover_limits)
            if s.calib_cmd == 1 {
                SHARED_QUAD.lock(|sq| sq.borrow_mut().motors[i].calib_cmd = 0);
                let mut wrapped_mot = AssemblyDCMotor(&mut motors[i]);
                if let Ok((min_p, max_p)) = calibrators[i].discover_limits(&mut wrapped_mot, &CLOCK) {
                    SHARED_QUAD.lock(|sq| {
                        let mut sq = sq.borrow_mut();
                        sq.motors[i].min_position_limit = min_p;
                        sq.motors[i].max_position_limit = max_p;
                    });
                    SHARED_CONTROL_TABLE.lock(|ct| {
                        let mut table = ct.borrow_mut();
                        table.write_f32(REG_MIN_POSITION_LIMIT, min_p);
                        table.write_f32(REG_MAX_POSITION_LIMIT, max_p);
                    });
                    SAVE_CONFIG_REQUESTED.store(true, Ordering::Relaxed);
                }
            }

            let mot = &mut motors[i];

            if s.enabled && mot.enabled == 0 {
                supervisors[i].clear_faults();
                SHARED_CONTROL_TABLE.lock(|ct| {
                    let mut table = ct.borrow_mut();
                    let cur = table.hardware_error_status();
                    table.set_hardware_error_status(cur & error_flags::FLASH_CORRUPT);
                });
                mot.enable();
            } else if !s.enabled && mot.enabled != 0 {
                mot.disable();
            }

            mot.velocity_limit = s.velocity_limit;
            mot.P_angle.set_p(s.p_angle);
            mot.PID_velocity.set_p(s.pid_vel_p);
            mot.PID_velocity.set_i(s.pid_vel_i);
            mot.PID_velocity.set_d(s.pid_vel_d);

            match s.controller_mode {
                0 => mot.controller = MotionControlType::angle,
                1 => mot.controller = MotionControlType::velocity_openloop,
                2 => mot.controller = MotionControlType::torque,
                3 => mot.controller = MotionControlType::velocity,
                _ => {}
            }

            // Sync SafetyConfig from SharedServoState
            {
                let cfg = supervisors[i].config_mut();
                cfg.min_position = s.min_position_limit;
                cfg.max_position = s.max_position_limit;
                cfg.velocity_limit = s.velocity_limit;
                cfg.voltage_limit = s.voltage_limit;
                cfg.i_peak_max = s.current_limit;
                cfg.i_cont = s.current_limit * 0.6;
                cfg.stall_timeout_ms = s.stall_timeout_ms;
            }

            if i == 0 {
                if let Some(N20Sensor::Tmag(ref tmag)) = mot.sensor() {
                    if tmag.consecutive_errors() >= 3 {
                        let _ = i2c1_bus.recover_bus();
                    }
                }
            }
            let sensor_status = mot.sensor().map(|sens| sens.status()).unwrap_or(SensorStatus::Ok);
            let safety_out = supervisors[i].evaluate(&SafetyInput {
                dt_us: 1000,
                enabled: mot.enabled != 0,
                control_mode: mot.controller,
                target: s.target,
                angle: mot.shaft_angle(),
                velocity: mot.shaft_velocity(),
                applied_voltage: mot.voltage.q,
                current_a: last_currents_a[i],
                bus_voltage: last_bus_voltage,
                temps: ThermalReadings {
                    mcu: last_mcu_temp,
                    ..ThermalReadings::unequipped()
                },
                sensor_status,
                driver_fault: false,
                comm_age_ms: 0,
                stepper_error_rad: None,
            });

            mot.voltage_limit = safety_out.effective_voltage_limit;
            clamped_targets[i] = safety_out.clamped_target;
            safety_faults[i] = safety_out.fault_flags;

            if safety_out.disarm && mot.enabled != 0 {
                mot.disable();
                SHARED_QUAD.lock(|sq| {
                    sq.borrow_mut().motors[i].enabled = false;
                    sq.borrow_mut().motors[i].hardware_error_status = safety_out.fault_flags;
                });
                SHARED_CONTROL_TABLE.lock(|ct| {
                    ct.borrow_mut().set_error_flag(safety_out.fault_flags);
                });
            }

            assembly.set_joint_target(i, joint_targets[i]);
            assembly.set_sensor_mapping(i, sensor_map[i] as usize);
        }

        // 4. Update sensor channel feedback in assembly pool
        for i in 0..4 {
            let (angle, vel) = if let Some(s) = motors[i].sensor_mut() {
                (s.get_angle(), s.get_velocity())
            } else {
                (0.0, 0.0)
            };
            assembly.set_sensor_state(i, angle, vel);
        }

        // 5. Execute 1 kHz actuator assembly control loops
        {
            let (m0, rest) = motors.split_at_mut(1);
            let (m1, rest) = rest.split_at_mut(1);
            let (m2, m3) = rest.split_at_mut(1);

            let mut wrapped = [
                AssemblyDCMotor(&mut m0[0]),
                AssemblyDCMotor(&mut m1[0]),
                AssemblyDCMotor(&mut m2[0]),
                AssemblyDCMotor(&mut m3[0]),
            ];

            if assembly.topology() == TopologyPreset::Four1Dof {
                // In independent 1-DOF mode, drive motors with SafetySupervisor-clamped targets
                for i in 0..4 {
                    wrapped[i].move_motor(clamped_targets[i], &CLOCK);
                }
            } else {
                // In coupled 2-DOF or mixed topologies, execute outer joint kinematics
                assembly.step_motors(&mut wrapped, &CLOCK);
            }
        }

        // 6. Current sensing, emergency stall protection (< 2 ms) & telemetry publish
        let mut measured_currents = [0.0f32; 4];
        let mut shaft_angles = [0.0f32; 4];
        let mut shaft_vels = [0.0f32; 4];
        let mut statuses = [0u8; 4];

        for i in 0..4 {
            let shaft_ang = motors[i].shaft_angle();
            let shaft_vel = motors[i].shaft_velocity();
            let raw_ma = motors[i]
                .current_sense_mut()
                .map(|cs| cs.get_current_magnitude() * 1000.0)
                .unwrap_or(0.0);
            let cur_ma = lpf_currents[i].update(raw_ma, &CLOCK);
            let stat = motors[i].motor_status as u8;

            // Immediate emergency overcurrent stall protection (> 600 mA)
            if cur_ma > 600.0 && motors[i].enabled != 0 {
                rprintln!("EMERGENCY OVERCURRENT TRIP ON MOTOR {}! I={:.1} mA > 600 mA. Coasting.", i, cur_ma);
                motors[i].disable();
                SHARED_QUAD.lock(|sq| {
                    sq.borrow_mut().motors[i].enabled = false;
                });
                SHARED_CONTROL_TABLE.lock(|ct| {
                    ct.borrow_mut().set_error_flag(error_flags::OVERCURRENT);
                });
            }

            measured_currents[i] = cur_ma;
            shaft_angles[i] = shaft_ang;
            shaft_vels[i] = shaft_vel;
            statuses[i] = stat;
        }

        // 7. Measure hardware telemetry: VSENS (PB1, 10k/2.7k divider) & internal MCU temperature
        let raw_vsens = vsens_pin.read() as f32;
        let bus_voltage = (raw_vsens * 3.3 / 4095.0) * crate::bsp::VBUS_DIVIDER_SCALE;
        let mcu_temp = read_internal_temperature();
        last_bus_voltage = bus_voltage;
        last_mcu_temp = mcu_temp;
        for i in 0..4 {
            last_currents_a[i] = measured_currents[i] / 1000.0;
        }
        let max_current_a = measured_currents.iter().cloned().fold(0.0f32, f32::max) / 1000.0;

        // 8. Update unified Control Table with telemetry, sync MultiBus parameters, and evaluate error status
        SHARED_CONTROL_TABLE.lock(|ct| {
            let mut table = ct.borrow_mut();

            // Synchronize runtime MultiBus register writes (SHARED_QUAD) into SHARED_CONTROL_TABLE
            let (sq_motors, sq_topo, sq_selected) = SHARED_QUAD.lock(|sq| {
                let s = sq.borrow();
                (s.motors, s.topology_mode, s.selected_motor as usize)
            });

            let sel = if sq_selected < 4 { sq_selected } else { 0 };
            table.write_u8(REG_TOPOLOGY_PRESET, sq_topo);
            table.write_f32(REG_POS_P_GAIN, sq_motors[sel].p_angle);
            table.write_f32(REG_VEL_P_GAIN, sq_motors[sel].pid_vel_p);
            table.write_f32(REG_VEL_I_GAIN, sq_motors[sel].pid_vel_i);
            table.write_f32(REG_VEL_D_GAIN, sq_motors[sel].pid_vel_d);
            table.write_f32(REG_VOLTAGE_LIMIT, sq_motors[sel].voltage_limit);
            table.write_f32(REG_VELOCITY_LIMIT, sq_motors[sel].velocity_limit);
            table.set_motor_homing_offset(0, sq_motors[0].current_angle);
            table.set_sensor_bus_freq_hz(sq_motors[0].sensor_bus_freq_hz);
            table.set_sensor_range(sq_motors[0].sensor_range);
            table.set_sensor_averaging(sq_motors[0].sensor_averaging);
            table.set_sensor_crc_enable(sq_motors[0].sensor_crc_enable != 0);

            table.evaluate_errors(max_current_a, bus_voltage, mcu_temp);
            table.enforce_limits();

            table.write_f32(REG_PRESENT_BUS_VOLTAGE, bus_voltage);
            table.write_f32(REG_PRESENT_TEMPERATURE, mcu_temp);
            table.write_f32(REG_PRESENT_CURRENT, max_current_a);
            table.write_f32(REG_PRESENT_POSITION, shaft_angles[0]);
            table.write_f32(REG_PRESENT_VELOCITY, shaft_vels[0]);

            // SaveConfig trigger (0x55 written to 0x64 or ControlTable)
            let save_req = SAVE_CONFIG_REQUESTED.swap(false, Ordering::Relaxed);
            if save_req || table.read_u8(REG_SAVE_CONFIG_CMD) == CMD_SAVE_CONFIG {
                table.write_u8(REG_SAVE_CONFIG_CMD, 0);
                let mut kin_snap = SHARED_QUAD.lock(|sq| {
                    let s = sq.borrow();
                    let mut k = s.kin;
                    k.topology_mode = s.topology_mode;
                    k.sensor_map = s.sensor_map;
                    k
                });
                kin_snap.coupling_mask = assembly.topology.coupling_mask;
                table.set_node_id(ACTIVE_NODE_ID.load(Ordering::Relaxed));
                GLOBAL_STORAGE.lock(|gs| {
                    if let Some(ref mut st) = *gs.borrow_mut() {
                        let _ = st.save_config_v2(&table, &kin_snap);
                    }
                });
                rprintln!("Flash v2 configuration (ControlTable + Kinematics) saved to Bank 2 sectors.");
            }

            // RebootCommand trigger (0xF0 written to 0x63)
            if table.read_u8(REG_REBOOT_CMD) == CMD_REBOOT_BOOTLOADER {
                table.write_u8(REG_REBOOT_CMD, 0);
                // Disarm all drivers into Coast mode
                for m in motors.iter_mut() {
                    m.disable();
                }
                let active_node_id = table.node_id();
                let mb = SoftResetMailbox::new(REQUEST_CANBOOT, active_node_id, 1);
                unsafe { write_soft_reset_mailbox(&mb) };
                barefoc_stm32::clear_boot_loop_guard();
                barefoc_stm32::bootloader::enter_bareboot(barefoc_stm32::bootloader::default_target());
            }

            // Red status LED reflects hardware error status
            if table.hardware_error_status() != 0 {
                led_r.set_low(); // Active low: LED ON
            } else {
                led_r.set_high(); // LED OFF
            }
        });

        // Publish back to SHARED_QUAD
        let ct_err = SHARED_CONTROL_TABLE.lock(|ct| ct.borrow().hardware_error_status());
        SHARED_QUAD.lock(|sq| {
            let mut sq = sq.borrow_mut();
            for i in 0..4 {
                sq.motors[i].current_angle = shaft_angles[i];
                sq.motors[i].current_velocity = shaft_vels[i];
                sq.motors[i].current_ma = measured_currents[i];
                sq.motors[i].status = statuses[i];
                sq.motors[i].sensor_fault_status = statuses[i];
                sq.motors[i].bus_voltage = bus_voltage;
                sq.motors[i].temperature_c = mcu_temp;
                sq.motors[i].hardware_error_status = ct_err | safety_faults[i];
                sq.joint_angles[i] = assembly.joint_angle(i);
                sq.joint_velocities[i] = assembly.joint_velocity(i);
            }
        });


        // 7. Service RTT & USART1 Commander commands (every 10 ms = 100 Hz)
        if loop_count % 10 == 0 {
            // Read RTT DownChannel
            let count = down.read(&mut rtt_buf);
            for &b in &rtt_buf[..count] {
                if b == b'\r' || b == b'\n' {
                    if !rtt_line.is_empty() {
                        if let Ok(cmd_str) = core::str::from_utf8(&rtt_line) {
                            handle_unified_command(cmd_str.trim(), &mut commander4, &mut assembly);
                        }
                        rtt_line.clear();
                    }
                } else if rtt_line.push(b).is_err() {
                    rtt_line.clear();
                }
            }

            // Read USART1 non-blocking if present
            if let Some(ref mut _u) = uart {
                let regs = embassy_stm32::pac::USART1;
                while regs.isr().read().rxne() {
                    let b = (regs.rdr().read().dr() & 0xFF) as u8;
                    if b == b'\r' || b == b'\n' {
                        if !rtt_line.is_empty() {
                            if let Ok(cmd_str) = core::str::from_utf8(&rtt_line) {
                                handle_unified_command(cmd_str.trim(), &mut commander4, &mut assembly);
                            }
                            rtt_line.clear();
                        }
                    } else if rtt_line.push(b).is_err() {
                        rtt_line.clear();
                    }
                }
            }
        }

        // 8. Periodic telemetry & LED Status (2 Hz idle, 10 Hz active)
        let any_enabled = motors.iter().any(|m| m.enabled != 0);
        let log_interval = if any_enabled { 100 } else { 500 };
        if loop_count % log_interval == 0 {
            if TMAG_DETECTED.load(Ordering::Relaxed) {
                led_b.set_low();
                led_r.set_high();
            } else {
                led_r.toggle();
                led_b.set_high();
            }

            let sel = SHARED_QUAD.lock(|s| s.borrow().selected_motor) as usize;
            let topo_name = match assembly.topology() {
                TopologyPreset::Four1Dof => "4x1-DOF",
                TopologyPreset::Two2Dof => "2x2-DOF",
                TopologyPreset::One1DofOne2Dof => "1x1-DOF+1x2-DOF",
                TopologyPreset::Two1DofOne2Dof => "2x1-DOF+1x2-DOF",
                TopologyPreset::Custom => "Custom",
            };

            rprintln!(
                "[{}] Mot {}: tgt={:.2} | ang={:.2} ({:.1}°) | vel={:.1} | I={:.1} mA | en={}",
                topo_name,
                sel,
                motor_states[sel].target,
                shaft_angles[sel],
                shaft_angles[sel] * 180.0 / core::f32::consts::PI,
                shaft_vels[sel],
                measured_currents[sel],
                if motors[sel].enabled != 0 { 1 } else { 0 }
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Unified Command Handler (Motors A..D, Kinematics K, Diagnostics)
// ---------------------------------------------------------------------------
fn handle_unified_command(
    cmd: &str,
    commander: &mut Commander4Axis<EmbassyClock>,
    assembly: &mut ActuatorAssembly<4, 4>,
) {
    let trimmed = cmd.trim();
    if trimmed.is_empty() {
        return;
    }

    let first = trimmed.chars().next().unwrap_or(' ');
    match first {
        'A' | 'B' | 'C' | 'D' | 'a' | 'b' | 'c' | 'd' => {
            let mot_idx = match first.to_ascii_uppercase() {
                'A' => 0,
                'B' => 1,
                'C' => 2,
                'D' => 3,
                _ => 0,
            };
            let payload = trimmed[1..].trim_start();
            if payload.is_empty() {
                SHARED_QUAD.lock(|sq| sq.borrow_mut().selected_motor = mot_idx as u8);
                rprintln!("-> Selected Motor {} ('{}')", mot_idx, first.to_ascii_uppercase());
            } else if let Ok(val) = barefoc_commander::parse_f32(payload) {
                SHARED_QUAD.lock(|sq| {
                    let mut sq = sq.borrow_mut();
                    sq.motors[mot_idx].target = val;
                    sq.motors[mot_idx].enabled = true;
                });
                rprintln!("-> Motor {} target set to {:.3}", mot_idx, val);
            } else {
                commander.run(trimmed);
            }
        }
        'K' | 'k' => {
            let payload = trimmed[1..].trim_start();
            if payload.starts_with('M') || payload.starts_with('m') {
                let m_arg = payload[1..].trim_start();
                if let Ok(mode) = m_arg.parse::<u8>() {
                    let preset = TopologyPreset::from_u8(mode);
                    assembly.set_topology(preset);
                    SHARED_QUAD.lock(|sq| {
                        sq.borrow_mut().topology_mode = mode;
                    });
                    rprintln!("-> Kinematics topology mode set to {} ({:?})", mode, preset);
                } else {
                    rprintln!("-> Current topology mode: {}", assembly.topology_mode);
                }
            } else if payload.starts_with('T') || payload.starts_with('t') {
                let parts: heapless::Vec<&str, 4> = payload[1..].split_whitespace().collect();
                if parts.len() >= 2 {
                    if let (Ok(j), Ok(rad)) = (parts[0].parse::<usize>(), barefoc_commander::parse_f32(parts[1])) {
                        if j < 4 {
                            assembly.set_joint_target(j, rad);
                            SHARED_QUAD.lock(|sq| {
                                sq.borrow_mut().joint_targets[j] = rad;
                            });
                            rprintln!("-> Joint {} target set to {:.3} rad", j, rad);
                        }
                    }
                }
            } else {
                let _ = KinematicsCommander::handle(&mut commander.kinematics, payload);
            }
        }
        't' => {
            let parts: heapless::Vec<&str, 4> = trimmed.split_whitespace().collect();
            if parts.len() > 1 {
                if let Ok(val) = barefoc_commander::parse_f32(parts[1]) {
                    SHARED_QUAD.lock(|sq| {
                        let mut sq = sq.borrow_mut();
                        let sel = sq.selected_motor as usize;
                        sq.motors[sel].target = val;
                        sq.motors[sel].enabled = true;
                    });
                    rprintln!("-> Target set to {:.3} on selected motor", val);
                }
            }
        }
        'e' => {
            let parts: heapless::Vec<&str, 4> = trimmed.split_whitespace().collect();
            let en = if parts.len() > 1 {
                parts[1] == "1" || parts[1] == "on"
            } else {
                true
            };
            SHARED_QUAD.lock(|sq| {
                let mut sq = sq.borrow_mut();
                let sel = sq.selected_motor as usize;
                sq.motors[sel].enabled = en;
            });
            rprintln!("-> Selected motor enabled: {}", en);
        }
        's' => {
            let (sq, topo) = SHARED_QUAD.lock(|s| (*s.borrow(), assembly.topology()));
            rprintln!("=== FISHYSERVO 4-AXIS STATUS ===");
            rprintln!("  Topology: {:?} (mode={})", topo, sq.topology_mode);
            rprintln!("  Sensor Map: {:?}", sq.sensor_map);
            for i in 0..4 {
                let m = &sq.motors[i];
                rprintln!(
                    "  Motor {}: en={}, tgt={:.3}, ang={:.3} rad, vel={:.1} rad/s, I={:.1} mA",
                    i,
                    m.enabled,
                    m.target,
                    m.current_angle,
                    m.current_velocity,
                    m.current_ma
                );
            }
        }
        'h' | '?' => {
            rprintln!("Available Commands:");
            rprintln!("  A..D [val]     - Select motor or set target (e.g. 'A 1.57')");
            rprintln!("  KM <0..3>      - Set topology mode (0=4x1Dof, 1=2x2Dof, 2=1x1+1x2, 3=2x1+1x2)");
            rprintln!("  KT <j> <rad>   - Set joint target position");
            rprintln!("  KA <j>         - Query joint angle");
            rprintln!("  e 1 / 0        - Enable / Disable selected motor");
            rprintln!("  t <val>        - Set target on selected motor");
            rprintln!("  s              - Print 4-axis multi-topology status");
        }
        _ => {
            rprintln!("Unknown command: '{}'. Type 'h' for help.", trimmed);
        }
    }
}

// ---------------------------------------------------------------------------
// Handsfree Bootloader Entry & Strobe Hooks
// ---------------------------------------------------------------------------
fn bootloader_write_hook(frame: &[u8]) {
    if frame.len() == 1 && frame[0] == 0xF0 {
        let active_node_id = {
            let id = SHARED_CONTROL_TABLE.lock(|ct| ct.borrow().node_id());
            if (0x08..=0x77).contains(&id) {
                id
            } else {
                ACTIVE_NODE_ID.load(Ordering::Relaxed)
            }
        };
        let mb = SoftResetMailbox::new(REQUEST_CANBOOT, active_node_id, 1);
        unsafe { write_soft_reset_mailbox(&mb) };
        barefoc_stm32::clear_boot_loop_guard();
        barefoc_stm32::bootloader::enter_bareboot(barefoc_stm32::bootloader::default_target());
    } else if (frame.len() >= 2 && u16::from(frame[0]) == REG_SAVE_CONFIG_MULTIBUS && frame[1] == CMD_SAVE_CONFIG)
        || (frame.len() >= 2 && frame[0] == (REG_SAVE_CONFIG_CMD as u8) && frame[1] == CMD_SAVE_CONFIG)
    {
        SAVE_CONFIG_REQUESTED.store(true, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// MultiBus Save Config Write Hook (0x64)
// ---------------------------------------------------------------------------
fn multibus_save_config_hook<const N: usize, B: RegisterBridge>(
    _ctx: &mut MultiMotorState<N, B>,
    _offset: u16,
    bytes: &[u8],
) -> HookResult {
    if !bytes.is_empty() && bytes[0] == CMD_SAVE_CONFIG {
        SAVE_CONFIG_REQUESTED.store(true, Ordering::Relaxed);
        HookResult::SelfClear
    } else {
        HookResult::Applied
    }
}

// Dedicated interrupt executor for I3C2 housekeeping task (1 ms telemetry sync, stuck-SDA, IWDG)
static I2C_TARGET_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static I3C_TARGET_READY: AtomicBool = AtomicBool::new(false);

#[interrupt]
unsafe fn TIM6() {
    unsafe { I2C_TARGET_EXECUTOR.on_interrupt() }
}

unsafe fn service_i3c2_isr() {
    if !I3C_TARGET_READY.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `I3C_TARGET_READY` is set after `TARGET_STORE` is initialized, and
    // `i3c_target_task` masks both `I3C2_EV` and `I3C2_ER` (`Priority::P1`) in NVIC
    // whenever it accesses `TARGET_STORE`.
    let target = unsafe { &mut *(&raw mut TARGET_STORE).cast::<TargetType>() };
    for _ in 0..16 {
        match target.service_once() {
            Ok(ServiceOutcome::Idle) => {
                if !target.irq_events_pending() {
                    break;
                }
            }
            Ok(ServiceOutcome::WritePhase { .. }) => {
                latch_write_phase_to_shared_quad(target);
            }
            Ok(ServiceOutcome::WriteRejected { .. }) | Ok(ServiceOutcome::ReadPhase { .. }) => {}
            Err(_) => break,
        }
    }
    target.unmask_irqs();
}

#[interrupt]
unsafe fn I3C2_EV() {
    unsafe { service_i3c2_isr() };
}

#[interrupt]
unsafe fn I3C2_ER() {
    unsafe { service_i3c2_isr() };
}

// ---------------------------------------------------------------------------
// MultiBus Topology Write Hook (0x78)
// ---------------------------------------------------------------------------
fn topology_mode_hook<const N: usize, B: RegisterBridge>(
    _ctx: &mut MultiMotorState<N, B>,
    _offset: u16,
    bytes: &[u8],
) -> HookResult {
    if !bytes.is_empty() {
        let mode = bytes[0];
        SHARED_QUAD.lock(|s| {
            let mut s = s.borrow_mut();
            s.topology_mode = mode;
            s.kin.topology_mode = mode;
        });
        HookResult::Applied
    } else {
        HookResult::SelfClear
    }
}

fn sensor_map_hook<const N: usize, B: RegisterBridge>(
    _ctx: &mut MultiMotorState<N, B>,
    _offset: u16,
    bytes: &[u8],
) -> HookResult {
    if bytes.len() >= 4 {
        let map = [bytes[0], bytes[1], bytes[2], bytes[3]];
        SHARED_QUAD.lock(|s| {
            let mut s = s.borrow_mut();
            s.sensor_map = map;
            s.kin.sensor_map = map;
        });
        HookResult::Applied
    } else {
        HookResult::SelfClear
    }
}

// ---------------------------------------------------------------------------
// I3C2 Multi-Motor Multibus Server Task (Address 0x40)
// ---------------------------------------------------------------------------
type TargetType = Stm32I3cTarget<'static, MultiMotorState<4, SharedServoState>, 640, 160>;
static mut TARGET_STORE: core::mem::MaybeUninit<TargetType> = core::mem::MaybeUninit::uninit();

fn with_i3c2_target<R>(f: impl FnOnce(&mut TargetType) -> R) -> R {
    interrupt::I3C2_EV.disable();
    interrupt::I3C2_ER.disable();
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    // SAFETY: Both `I3C2_EV` and `I3C2_ER` (`Priority::P1`) are masked in NVIC
    // with `dsb`/`isb` barriers, so no ISR can alias `TARGET_STORE`.
    let target = unsafe { &mut *(&raw mut TARGET_STORE).cast::<TargetType>() };
    let res = f(target);
    unsafe {
        interrupt::I3C2_EV.enable();
        interrupt::I3C2_ER.enable();
    }
    res
}

fn latch_write_phase_to_shared_quad(target: &mut TargetType) {
    let handler = target.server_mut().map_mut().handler();
    let sel = handler.selected();
    let motors = handler.motors;
    SHARED_QUAD.lock(|s| {
        let mut s = s.borrow_mut();
        s.selected_motor = sel;
        for i in 0..4 {
            s.motors[i].target = motors[i].target;
            s.motors[i].enabled = motors[i].enabled;
            s.motors[i].controller_mode = motors[i].controller_mode;
            s.motors[i].p_angle = motors[i].p_angle;
            s.motors[i].pid_vel_p = motors[i].pid_vel_p;
            s.motors[i].pid_vel_i = motors[i].pid_vel_i;
            s.motors[i].pid_vel_d = motors[i].pid_vel_d;
            s.motors[i].voltage_limit = motors[i].voltage_limit;
            s.motors[i].current_limit = motors[i].current_limit;
            s.motors[i].velocity_limit = motors[i].velocity_limit;
            s.motors[i].min_position_limit = motors[i].min_position_limit;
            s.motors[i].max_position_limit = motors[i].max_position_limit;
            s.motors[i].stall_timeout_ms = motors[i].stall_timeout_ms;
            s.motors[i].calib_cmd = motors[i].calib_cmd;
            s.motors[i].sensor_bus_freq_hz = motors[i].sensor_bus_freq_hz;
            s.motors[i].sensor_range = motors[i].sensor_range;
            s.motors[i].sensor_averaging = motors[i].sensor_averaging;
            s.motors[i].sensor_crc_enable = motors[i].sensor_crc_enable;
        }
    });
}

static RW_SYNC_NEEDED: AtomicBool = AtomicBool::new(false);

fn sync_telemetry_into_target(target: &mut TargetType) {
    let (selected, sq_motors, sq_topo, sq_smap) = SHARED_QUAD.lock(|s| {
        let s = s.borrow();
        (s.selected_motor, s.motors, s.topology_mode, s.sensor_map)
    });
    let map = target.server_mut().map_mut();
    let _ = map.handler().set_selected(selected);
    for i in 0..4 {
        map.handler().motors[i] = sq_motors[i];
    }
    if RW_SYNC_NEEDED.swap(false, Ordering::AcqRel) {
        sync_rw_from_bridge_multi(map);
        let _ = map.set(REG_TOPOLOGY_MODE, &[sq_topo]);
        let _ = map.set(REG_SENSOR_MAP_MULTIBUS, &sq_smap);
    }
    refresh_telemetry_multi(map);
}

#[embassy_executor::task]
async fn i3c_target_task() {
    rprintln!("I3C2 Multi-Motor Multibus target server listening on address 0x40.");
    I3C_TARGET_ALIVE.store(true, Ordering::Relaxed);
    with_i3c2_target(|target| {
        sync_telemetry_into_target(target);
        target.refresh_read_buffer();
        target.unmask_irqs();
    });
    let mut sda_low_ms: u32 = 0;
    let mut last_sda_check = Instant::now();
    let mut last_telemetry_sync = Instant::now();

    loop {
        if I3C2_BUS_HEALTHY.load(Ordering::Relaxed) {
            barefoc_stm32::feed_iwdg();
        }
        let now = Instant::now();
        if now.checked_duration_since(last_sda_check).map(|d| d.as_millis()).unwrap_or(0) >= 1 {
            last_sda_check = now;
            let (recovered, still_wedged, sda_low) = with_i3c2_target(|target| {
                let (recovered, still_wedged) =
                    target.check_and_recover_stuck_sda(&mut sda_low_ms, 50);
                if recovered {
                    target.unmask_irqs();
                }
                let sda_low = target.is_sda_low();
                (recovered, still_wedged, sda_low)
            });
            if recovered {
                I3C2_BUS_HEALTHY.store(!still_wedged, Ordering::Relaxed);
                if still_wedged {
                    rprintln!("ERROR: I3C2 SDA stuck low after RCC_APB3RSTR; withholding IWDG feed.");
                } else {
                    rprintln!("WARN: I3C2 stuck-SDA (>50 ms) recovered via RCC_APB3RSTR.");
                }
            } else if !sda_low {
                I3C2_BUS_HEALTHY.store(true, Ordering::Relaxed);
            }
        }
        if now
            .checked_duration_since(last_telemetry_sync)
            .map(|d| d.as_millis())
            .unwrap_or(0)
            >= 1
        {
            let synced = with_i3c2_target(|target| {
                if target.is_frame_idle() {
                    sync_telemetry_into_target(target);
                    true
                } else {
                    false
                }
            });
            if synced {
                last_telemetry_sync = now;
            }
        }
        Timer::after_millis(1).await;
    }
}

// ---------------------------------------------------------------------------
// Main System Initialization
// ---------------------------------------------------------------------------
#[inline(never)]
fn load_boot_config(
    flash_peri: embassy_stm32::Peri<'static, embassy_stm32::peripherals::FLASH>,
    dynamic_node_id: u8,
    mailbox_valid: bool,
) -> (u8, [SharedServoState; 4]) {
    let flash = embassy_stm32::flash::Flash::new_blocking(
        flash_peri,
        embassy_stm32::flash::Config::default(),
    );
    let mut storage = H5FlashStorage::new(flash);
    let mut boot_table = ControlTable::new();
    let mut boot_kin = KinematicsFlashConfig::default();
    if storage.load_config_v2(&mut boot_table, &mut boot_kin).is_err() {
        boot_table.set_error_flag(error_flags::FLASH_CORRUPT);
    }

    let mut uid = [0u8; 12];
    let sector15_first_word: u32;
    let icache_was_en = embassy_stm32::pac::ICACHE.cr().read().en();
    if icache_was_en {
        embassy_stm32::pac::ICACHE.cr().modify(|w| w.set_en(false));
        while embassy_stm32::pac::ICACHE.sr().read().busyf() {}
    }
    unsafe {
        let uid_ptr = UID_BASE as *const u32;
        let w0 = core::ptr::read_volatile(uid_ptr.add(0)).to_le_bytes();
        let w1 = core::ptr::read_volatile(uid_ptr.add(1)).to_le_bytes();
        let w2 = core::ptr::read_volatile(uid_ptr.add(2)).to_le_bytes();
        uid[0..4].copy_from_slice(&w0);
        uid[4..8].copy_from_slice(&w1);
        uid[8..12].copy_from_slice(&w2);
        let sector15_ptr =
            (0x0800_0000u32 + barefoc_stm32::stm32h5::flash_storage::SECTOR_15_OFFSET) as *const u32;
        sector15_first_word = core::ptr::read_volatile(sector15_ptr);
    }
    if icache_was_en {
        embassy_stm32::pac::ICACHE.cr().modify(|w| w.set_en(true));
    }
    boot_table.set_hardware_uid(&uid);

    let (active_node_id, id_source) = if mailbox_valid && (0x08..=0x77).contains(&dynamic_node_id) {
        boot_table.set_node_id(dynamic_node_id);
        (dynamic_node_id, "SoftResetMailbox (Tier 1)")
    } else if (0x08..=0x77).contains(&boot_table.node_id()) {
        (boot_table.node_id(), "NVM FlashStorage (Tier 2)")
    } else {
        let uid_id = derive_uid_node_id(&uid);
        boot_table.set_node_id(uid_id);
        (uid_id, "Silicon UID Hash (Tier 3)")
    };
    rprintln!("Active NodeId = 0x{:02X} (source: {})", active_node_id, id_source);
    ACTIVE_NODE_ID.store(active_node_id, Ordering::Relaxed);

    if (boot_table.hardware_error_status() & error_flags::FLASH_CORRUPT) != 0
        && sector15_first_word == 0xFFFF_FFFF
    {
        if storage.save_config_v2(&boot_table, &boot_kin).is_ok() {
            boot_table.clear_error_flag(error_flags::FLASH_CORRUPT);
            rprintln!("Blank Sector 15 auto-provisioned (v2); FLASH_CORRUPT cleared.");
        }
    }

    let mut initial_motors = [SharedServoState::new(); 4];
    let persisted_p_ang = boot_table.read_f32(REG_POS_P_GAIN);
    let persisted_v_p = boot_table.read_f32(REG_VEL_P_GAIN);
    let persisted_v_i = boot_table.read_f32(REG_VEL_I_GAIN);
    let persisted_v_d = boot_table.read_f32(REG_VEL_D_GAIN);
    let persisted_v_lim = boot_table.read_f32(REG_VOLTAGE_LIMIT);
    let persisted_vel_lim = boot_table.read_f32(REG_VELOCITY_LIMIT);
    let persisted_bus_hz = boot_table.sensor_bus_freq_hz();
    let persisted_range = boot_table.sensor_range();
    let persisted_avg = boot_table.sensor_averaging();
    let persisted_crc = if boot_table.sensor_crc_enable() { 1u8 } else { 0u8 };
    if persisted_p_ang.is_finite() && persisted_p_ang > 0.0 {
        for m in initial_motors.iter_mut() {
            m.p_angle = persisted_p_ang;
            if persisted_v_p.is_finite() && persisted_v_p > 0.0 { m.pid_vel_p = persisted_v_p; }
            if persisted_v_i.is_finite() && persisted_v_i >= 0.0 { m.pid_vel_i = persisted_v_i; }
            if persisted_v_d.is_finite() && persisted_v_d >= 0.0 { m.pid_vel_d = persisted_v_d; }
            if persisted_v_lim.is_finite() && persisted_v_lim > 0.0 { m.voltage_limit = persisted_v_lim; }
            if persisted_vel_lim.is_finite() && persisted_vel_lim > 0.0 { m.velocity_limit = persisted_vel_lim; }
            if persisted_bus_hz > 0 { m.sensor_bus_freq_hz = persisted_bus_hz; }
            m.sensor_range = persisted_range;
            m.sensor_averaging = persisted_avg;
            m.sensor_crc_enable = persisted_crc;
        }
    }

    SHARED_CONTROL_TABLE.lock(|ct| *ct.borrow_mut() = boot_table);
    GLOBAL_STORAGE.lock(|gs| *gs.borrow_mut() = Some(storage));
    SHARED_QUAD.lock(|sq| {
        let mut sq = sq.borrow_mut();
        sq.motors = initial_motors;
        sq.kin = boot_kin;
        sq.topology_mode = boot_kin.topology_mode;
        sq.sensor_map = boot_kin.sensor_map;
    });

    (active_node_id, initial_motors)
}

#[inline(never)]
fn build_multibus_map(
    initial_motors: &[SharedServoState; 4],
) -> barefoc_multibus::table::RegMap<MultiMotorState<4, SharedServoState>, 640, 160> {
    let mut multibus_map = multi_motor_profile(*initial_motors);

    // Seed runtime sensor & bus R/W registers in the RegMap bank from initial state
    let _ = multibus_map.set(
        reg_offset(RegisterId::SensorBusFreqHz),
        &initial_motors[0].sensor_bus_freq_hz.to_le_bytes(),
    );
    let _ = multibus_map.set(
        reg_offset(RegisterId::SensorRange),
        &[initial_motors[0].sensor_range],
    );
    let _ = multibus_map.set(
        reg_offset(RegisterId::SensorAveraging),
        &[initial_motors[0].sensor_averaging],
    );
    let _ = multibus_map.set(
        reg_offset(RegisterId::SensorCrcEnable),
        &[initial_motors[0].sensor_crc_enable],
    );

    // Register SaveConfig hook (0x55 persists live tuning parameters to Bank 2)
    register_save_config_hook(&mut multibus_map, multibus_save_config_hook)
        .expect("Failed to register SaveConfig hook");

    // Register TopologyMode RW hook for runtime topology switching
    register_topology_mode_hook(&mut multibus_map, topology_mode_hook)
        .expect("Failed to register TopologyMode hook");

    // Seed & register SensorMap RW hook for runtime sensor multiplexing
    let (boot_topo, boot_smap) = SHARED_QUAD.lock(|sq| {
        let s = sq.borrow();
        (s.topology_mode, s.sensor_map)
    });
    let _ = multibus_map.set(REG_TOPOLOGY_MODE, &[boot_topo]);
    let _ = multibus_map.set(REG_SENSOR_MAP_MULTIBUS, &boot_smap);
    let _ = multibus_map.set(
        REG_SUPPORTED_SENSORS_MULTIBUS,
        &crate::bsp::SUPPORTED_SENSORS_MASK.to_le_bytes(),
    );
    register_sensor_map_hook(&mut multibus_map, sensor_map_hook)
        .expect("Failed to register SensorMap hook");

    multibus_map
}

#[inline(never)]
fn init_i3c_target(target_node_id: u8, initial_motors: [SharedServoState; 4]) {
    let server = RegServer::new(build_multibus_map(&initial_motors));
    let mut target =
        Stm32I3cTarget::new(target_node_id, server).expect("I3C2 Target binding failed");
    target.enable_bootloader(barefoc_stm32::bootloader::default_target());
    target.set_write_frame_hook(Some(bootloader_write_hook));

    unsafe {
        (*core::ptr::addr_of_mut!(TARGET_STORE)).write(target);
    }
    I3C_TARGET_READY.store(true, Ordering::Release);
}

#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn init_board_and_motors(
    leds: crate::bsp::embassy_bsp::StatusLedPeripherals,
    gd: crate::bsp::embassy_bsp::QuadGateDriverPeripherals,
    vbus_adc: crate::bsp::embassy_bsp::VbusAdcPeripherals,
    sbus: crate::bsp::embassy_bsp::SensorBusPeripherals,
    initial_motors: &[SharedServoState; 4],
    boot_start: Instant,
) -> (
    &'static mut [DCMotorType; 4],
    Stm32I2cBusControl,
    Output<'static>,
    Output<'static>,
    bool,
    Stm32AdcPin,
) {
    let led_r = Output::new(leds.led0, Level::High, Speed::Low);
    let mut led_b = Output::new(leds.led1, Level::Low, Speed::Low);

    let tim1_ch1_pin = PwmPin::new(gd.ch1_in1, OutputType::PushPull);
    let tim1_ch2_pin = PwmPin::new(gd.ch0_in1, OutputType::PushPull);
    let tim1_ch3_pin = PwmPin::new(gd.ch3_in2, OutputType::PushPull);
    let pwm1 = SimplePwm::new(
        gd.tim0,
        Some(tim1_ch1_pin),
        Some(tim1_ch2_pin),
        Some(tim1_ch3_pin),
        None,
        Hertz(crate::bsp::PWM_FREQUENCY_HZ),
        CountingMode::EdgeAlignedUp,
    );
    let tim3_ch1_pin = PwmPin::new(gd.ch2_in1, OutputType::PushPull);
    let tim3_ch2_pin = PwmPin::new(gd.ch2_in2, OutputType::PushPull);
    let tim3_ch3_pin = PwmPin::new(gd.ch0_in2, OutputType::PushPull);
    let tim3_ch4_pin = PwmPin::new(gd.ch1_in2, OutputType::PushPull);
    let pwm3 = SimplePwm::new(
        gd.tim1,
        Some(tim3_ch1_pin),
        Some(tim3_ch2_pin),
        Some(tim3_ch3_pin),
        Some(tim3_ch4_pin),
        Hertz(crate::bsp::PWM_FREQUENCY_HZ),
        CountingMode::EdgeAlignedUp,
    );

    let mut tim1_chs = pwm1.split();
    let mut tim3_chs = pwm3.split();

    tim1_chs.ch1.enable();
    tim1_chs.ch2.enable();
    tim1_chs.ch3.enable();
    tim3_chs.ch1.enable();
    tim3_chs.ch2.enable();
    tim3_chs.ch3.enable();
    tim3_chs.ch4.enable();

    let ch_b1 = tim1_chs.ch1;
    let ch_d1 = tim1_chs.ch2;
    let ch_a2 = tim1_chs.ch3;
    let ch_c1 = tim3_chs.ch1;
    let ch_c2 = tim3_chs.ch2;
    let ch_d2 = tim3_chs.ch3;
    let ch_b2 = tim3_chs.ch4;

    let pin_a_in1 = Output::new(gd.ch3_in1, Level::Low, Speed::VeryHigh);
    let mut driver_a = BridgeADriver::new(pin_a_in1, ch_a2);
    driver_a.params_mut().voltage_power_supply = crate::bsp::DEFAULT_VOLTAGE_LIMIT;
    driver_a.params_mut().voltage_limit = crate::bsp::DEFAULT_VOLTAGE_LIMIT;
    driver_a.params_mut().pwm_frequency = crate::bsp::PWM_FREQUENCY_HZ;
    driver_a.init().expect("Driver A init failed");

    let mut driver_b = DCDriver2PWM::without_enable(ch_b1, ch_b2);
    driver_b.params_mut().voltage_power_supply = crate::bsp::DEFAULT_VOLTAGE_LIMIT;
    driver_b.params_mut().voltage_limit = crate::bsp::DEFAULT_VOLTAGE_LIMIT;
    driver_b.params_mut().pwm_frequency = crate::bsp::PWM_FREQUENCY_HZ;
    driver_b.init().expect("Driver B init failed");

    let mut driver_c = DCDriver2PWM::without_enable(ch_c1, ch_c2);
    driver_c.params_mut().voltage_power_supply = crate::bsp::DEFAULT_VOLTAGE_LIMIT;
    driver_c.params_mut().voltage_limit = crate::bsp::DEFAULT_VOLTAGE_LIMIT;
    driver_c.params_mut().pwm_frequency = crate::bsp::PWM_FREQUENCY_HZ;
    driver_c.init().expect("Driver C init failed");

    let mut driver_d = DCDriver2PWM::without_enable(ch_d1, ch_d2);
    driver_d.params_mut().voltage_power_supply = crate::bsp::DEFAULT_VOLTAGE_LIMIT;
    driver_d.params_mut().voltage_limit = crate::bsp::DEFAULT_VOLTAGE_LIMIT;
    driver_d.params_mut().pwm_frequency = crate::bsp::PWM_FREQUENCY_HZ;
    driver_d.init().expect("Driver D init failed");

    driver_a.set_pwm(0.0);
    driver_b.set_pwm(0.0);
    driver_c.set_pwm(0.0);
    driver_d.set_pwm(0.0);
    CLOCK.delay_ms(10);

    let adc1 = Adc::new_blocking(vbus_adc.adc, embassy_stm32::adc::Config::default());
    GLOBAL_ADC.lock(|cell| {
        *cell.borrow_mut() = Some(adc1);
    });

    let adc_pin_a = Stm32AdcPin::new(AdcInputPin::ChA(gd.ch3_adc), SampleTime::Cycles65);
    let adc_pin_b = Stm32AdcPin::new(AdcInputPin::ChB(gd.ch1_adc), SampleTime::Cycles65);
    let adc_pin_c = Stm32AdcPin::new(AdcInputPin::ChC(gd.ch2_adc), SampleTime::Cycles65);
    let adc_pin_d = Stm32AdcPin::new(AdcInputPin::ChD(gd.ch0_adc), SampleTime::Cycles65);
    let vsens_pin = Stm32AdcPin::new(AdcInputPin::Vsens(vbus_adc.pin), SampleTime::Cycles65);

    let mut cs_a = CurrentMirrorSense::new(adc_pin_a, 200.0, 1000.0, 3.3, 12);
    let mut cs_b = CurrentMirrorSense::new(adc_pin_b, 200.0, 1000.0, 3.3, 12);
    let mut cs_c = CurrentMirrorSense::new(adc_pin_c, 200.0, 1000.0, 3.3, 12);
    let mut cs_d = CurrentMirrorSense::new(adc_pin_d, 200.0, 1000.0, 3.3, 12);

    barefoc_stm32::feed_iwdg();
    cs_a.calibrate_offsets_rounds(&CLOCK, 200);
    barefoc_stm32::feed_iwdg();
    cs_b.calibrate_offsets_rounds(&CLOCK, 200);
    barefoc_stm32::feed_iwdg();
    cs_c.calibrate_offsets_rounds(&CLOCK, 200);
    barefoc_stm32::feed_iwdg();
    cs_d.calibrate_offsets_rounds(&CLOCK, 200);
    barefoc_stm32::feed_iwdg();

    let mut i2c1_cfg = I2cConfig::default();
    i2c1_cfg.sda_pullup = true;
    i2c1_cfg.scl_pullup = true;
    i2c1_cfg.frequency = Hertz::khz(50);
    i2c1_cfg.timeout = Duration::from_millis(10);
    let mut i2c1 = I2c::new_blocking(sbus.bus, sbus.scl, sbus.sda, i2c1_cfg);
    let mut i2c1_bus = Stm32I2cBusControl::new(embassy_stm32::pac::I2C1, 64_000_000, 100_000);

    let speeds_khz = [10u32, 20, 50, 100];
    let max_slots = crate::bsp::MAX_SENSOR_SLOTS.min(4);
    let mut detected_tmags: [(u8, DeviceVariant); 4] = [(0, DeviceVariant::A1); 4];
    let mut detected_count: usize = 0;

    for &khz in &speeds_khz {
        barefoc_stm32::feed_iwdg();
        let _ = i2c1_bus.set_bus_frequency_hz(khz * 1000);
        for &addr in crate::bsp::SENSOR_CANDIDATE_ADDRS {
            if detected_tmags[..detected_count].iter().any(|&(a, _)| a == addr) {
                continue;
            }
            let mut mfg = [0u8; 2];
            if i2c1.blocking_write_read(addr, &[0x0E], &mut mfg).is_ok() && mfg == [0x49, 0x54] {
                let mut dev_id = [0u8; 1];
                let variant = if i2c1.blocking_write_read(addr, &[0x0D], &mut dev_id).is_ok() {
                    if (dev_id[0] & 0x03) == 0x02 { DeviceVariant::A2 } else { DeviceVariant::A1 }
                } else {
                    DeviceVariant::A1
                };
                if detected_count < max_slots {
                    detected_tmags[detected_count] = (addr, variant);
                    detected_count += 1;
                    rprintln!("Detected TMAG5273 slot {} at 0x{:02X} ({:?})", detected_count - 1, addr, variant);
                }
            }
        }
        if detected_count > 0 {
            break;
        }
    }
    TMAG_DETECTED.store(detected_count > 0, Ordering::Relaxed);

    embassy_time::block_for(Duration::from_millis(1));
    let probed_max_hz = if detected_count > 0 {
        i2c1_bus.probe_max_frequency_hz(
            &[1_000_000, 800_000, 400_000, 200_000, 100_000, 50_000],
            50_000,
            |_hz| {
                detected_tmags[..detected_count].iter().all(|&(addr, _)| {
                    let mut ok_cnt = 0u32;
                    for _ in 0..20 {
                        let mut buf = [0u8; 2];
                        if i2c1.blocking_write_read(addr, &[0x19], &mut buf).is_ok() {
                            ok_cnt += 1;
                        }
                    }
                    ok_cnt == 20
                })
            },
        )
    } else {
        50_000
    };
    let desired_hz = initial_motors[0].sensor_bus_freq_hz;
    let best_i2c1_hz = if desired_hz > 0 {
        core::cmp::min(desired_hz, probed_max_hz)
    } else {
        probed_max_hz
    };
    let _ = i2c1_bus.set_bus_frequency_hz(best_i2c1_hz);
    SHARED_QUAD.lock(|sq| {
        sq.borrow_mut().motors[0].sensor_bus_freq_hz = best_i2c1_hz;
    });

    // SAFETY: `SENSOR_I2C_BUS` is initialized once here before any `SharedI2cDevice` is created.
    let shared_i2c: &'static RefCell<I2c<'static, Blocking, Master>> = unsafe {
        let ptr = (*core::ptr::addr_of_mut!(SENSOR_I2C_BUS)).as_mut_ptr();
        ptr.write(RefCell::new(i2c1));
        &*ptr
    };

    let mut slot_sensors: [Option<MotorSensorType>; 4] = [None, None, None, None];
    for slot_idx in 0..detected_count {
        let (addr, variant) = detected_tmags[slot_idx];
        let motor_idx = crate::bsp::SENSOR_SLOTS
            .get(slot_idx)
            .map(|s| (s.motor as usize).min(3))
            .unwrap_or(slot_idx.min(3));
        let init_range = if initial_motors[motor_idx].sensor_range == 0 {
            Range::Low
        } else {
            Range::High
        };
        let init_avg = tmag_rs::ConversionAverage::try_from(initial_motors[motor_idx].sensor_averaging)
            .unwrap_or(tmag_rs::ConversionAverage::Avg1x);
        let init_crc = initial_motors[motor_idx].sensor_crc_enable != 0;

        if let Ok(dev) = Tmag5273::builder(crate::hal::SharedI2cDevice::new(shared_i2c), addr)
            .variant(variant)
            .operating_mode(OperatingMode::ContinuousMeasure)
            .channels(MagneticChannel::XYZ)
            .angle_enabled(AngleEnable::XY)
            .xy_range(init_range)
            .z_range(init_range)
            .averaging(init_avg)
            .crc_enabled(init_crc)
            .build()
        {
            slot_sensors[slot_idx] = Some(N20Sensor::Tmag(Tmag5273Sensor::new(dev)));
        }
    }

    let sensor_map = SHARED_QUAD.lock(|sq| sq.borrow().sensor_map);
    let mut take_sensor_for_motor = |motor_ch: usize| -> (MotorSensorType, bool) {
        let slot = sensor_map.get(motor_ch).copied().unwrap_or(motor_ch as u8) as usize;
        if slot < 4 {
            if let Some(s) = slot_sensors[slot].take() {
                return (s, true);
            }
        }
        (N20Sensor::Fallback(DummySensor::new()), false)
    };

    let (sensor_a, has_sensor_a) = take_sensor_for_motor(0);
    let (sensor_b, has_sensor_b) = take_sensor_for_motor(1);
    let (sensor_c, has_sensor_c) = take_sensor_for_motor(2);
    let (sensor_d, has_sensor_d) = take_sensor_for_motor(3);
    let has_sensor = has_sensor_a || has_sensor_b || has_sensor_c || has_sensor_d;

    let mut motor_a = DCMotor::new(&CLOCK);
    motor_a.link_driver(QuadDCDriver::D(driver_d));
    motor_a.link_sensor(sensor_a);
    motor_a.link_current_sense(cs_d);
    motor_a.sensor_direction = Direction::CCW;
    motor_a.voltage_limit = 4.0;
    motor_a.velocity_limit = 50.0;
    motor_a.current_limit = 0.5;
    motor_a.controller = if has_sensor_a { MotionControlType::angle } else { MotionControlType::velocity_openloop };
    motor_a.torque_controller = TorqueControlType::voltage;
    motor_a.P_angle.set_p(initial_motors[0].p_angle);
    motor_a.PID_velocity.set_p(initial_motors[0].pid_vel_p);
    motor_a.PID_velocity.set_i(initial_motors[0].pid_vel_i);
    motor_a.PID_velocity.set_d(initial_motors[0].pid_vel_d);
    motor_a.voltage_limit = initial_motors[0].voltage_limit;
    motor_a.velocity_limit = initial_motors[0].velocity_limit;
    let _ = motor_a.init();

    let mut motor_b = DCMotor::new(&CLOCK);
    motor_b.link_driver(QuadDCDriver::B(driver_b));
    motor_b.link_sensor(sensor_b);
    motor_b.link_current_sense(cs_b);
    motor_b.sensor_direction = Direction::CCW;
    motor_b.voltage_limit = initial_motors[1].voltage_limit;
    motor_b.velocity_limit = initial_motors[1].velocity_limit;
    motor_b.current_limit = 0.5;
    motor_b.controller = MotionControlType::angle;
    motor_b.torque_controller = TorqueControlType::voltage;
    motor_b.P_angle.set_p(initial_motors[1].p_angle);
    motor_b.PID_velocity.set_p(initial_motors[1].pid_vel_p);
    motor_b.PID_velocity.set_i(initial_motors[1].pid_vel_i);
    motor_b.PID_velocity.set_d(initial_motors[1].pid_vel_d);
    let _ = motor_b.init();

    let mut motor_c = DCMotor::new(&CLOCK);
    motor_c.link_driver(QuadDCDriver::C(driver_c));
    motor_c.link_sensor(sensor_c);
    motor_c.link_current_sense(cs_c);
    motor_c.sensor_direction = Direction::CCW;
    motor_c.voltage_limit = initial_motors[2].voltage_limit;
    motor_c.velocity_limit = initial_motors[2].velocity_limit;
    motor_c.current_limit = 0.5;
    motor_c.controller = MotionControlType::angle;
    motor_c.torque_controller = TorqueControlType::voltage;
    motor_c.P_angle.set_p(initial_motors[2].p_angle);
    motor_c.PID_velocity.set_p(initial_motors[2].pid_vel_p);
    motor_c.PID_velocity.set_i(initial_motors[2].pid_vel_i);
    motor_c.PID_velocity.set_d(initial_motors[2].pid_vel_d);
    let _ = motor_c.init();

    let mut motor_d = DCMotor::new(&CLOCK);
    motor_d.link_driver(QuadDCDriver::A(driver_a));
    motor_d.link_sensor(sensor_d);
    motor_d.link_current_sense(cs_a);
    motor_d.sensor_direction = Direction::CCW;
    motor_d.voltage_limit = initial_motors[3].voltage_limit;
    motor_d.velocity_limit = initial_motors[3].velocity_limit;
    motor_d.current_limit = 0.5;
    motor_d.controller = MotionControlType::angle;
    motor_d.torque_controller = TorqueControlType::voltage;
    motor_d.P_angle.set_p(initial_motors[3].p_angle);
    motor_d.PID_velocity.set_p(initial_motors[3].pid_vel_p);
    motor_d.PID_velocity.set_i(initial_motors[3].pid_vel_i);
    motor_d.PID_velocity.set_d(initial_motors[3].pid_vel_d);
    let _ = motor_d.init();

    let init_ang_a = motor_a.shaft_angle();
    let init_ang_b = motor_b.shaft_angle();
    let init_ang_c = motor_c.shaft_angle();
    let init_ang_d = motor_d.shaft_angle();
    motor_a.target = init_ang_a;
    motor_b.target = init_ang_b;
    motor_c.target = init_ang_c;
    motor_d.target = init_ang_d;
    let init_angles = [init_ang_a, init_ang_b, init_ang_c, init_ang_d];

    SHARED_QUAD.lock(|sq| {
        let mut sq = sq.borrow_mut();
        for i in 0..4 {
            sq.motors[i].enabled = false;
            sq.motors[i].target = init_angles[i];
            sq.motors[i].current_angle = init_angles[i];
            sq.joint_targets[i] = init_angles[i];
            sq.joint_angles[i] = init_angles[i];
        }
    });
    RW_SYNC_NEEDED.store(true, Ordering::Release);

    motor_a.disable();
    motor_b.disable();
    motor_c.disable();
    motor_d.disable();

    SHARED_CONTROL_TABLE.lock(|ct| {
        let mut table = ct.borrow_mut();
        table.write_f32(REG_GOAL_POSITION, init_ang_a);
        table.write_f32(REG_PRESENT_POSITION, init_ang_a);
    });

    led_b.set_high();
    let elapsed_ms = Instant::now().checked_duration_since(boot_start).map(|d| d.as_millis()).unwrap_or(0);
    rprintln!("Boot complete: 4 motors initialized ({} ms).", elapsed_ms);

    let motors_ref = unsafe {
        let ptr = (*core::ptr::addr_of_mut!(MOTORS_STORE)).as_mut_ptr();
        ptr.write([motor_a, motor_b, motor_c, motor_d]);
        &mut *ptr
    };
    (motors_ref, i2c1_bus, led_b, led_r, has_sensor, vsens_pin)
}

pub async fn run(_spawner: Spawner) {
    // Ensure PRIMASK=0 on entry even if a bootloader jumped with interrupts masked.
    // SAFETY: No shared mutable state or peripheral IRQs are enabled yet.
    unsafe { cortex_m::interrupt::enable() };
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 2048,
                mode: rtt_target::ChannelMode::NoBlockTrim,
                name: "Terminal"
            }
        }
        down: {
            0: {
                size: 128,
                mode: rtt_target::ChannelMode::NoBlockTrim,
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(channels.up.0);
    let rtt_down = channels.down.0;

    rprintln!("\r\n=======================================================");
    rprintln!("     FishyServo Quad H-Bridge Multi-Topology Servo     ");
    rprintln!("     Microcontroller: STM32H503KBU6 (Arm Cortex-M33)  ");
    rprintln!("     Bridges: A (PA15/PB5), B (PA8/PB15), C (PA6/PA7), D (PA9/PB0)");
    rprintln!("     Sensors: TMAG5273 on I2C1 (PB6/PB7)               ");
    rprintln!("     Host Bus: MultiBus on I2C2 (PB3/PB4 @ 0x40)      ");
    rprintln!("=======================================================");

    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = Some(embassy_stm32::rcc::HSIPrescaler::Div1);
    config.rcc.sys = embassy_stm32::rcc::Sysclk::Hsi;
    config.rcc.voltage_scale = embassy_stm32::rcc::VoltageScale::Scale3;
    let p = embassy_stm32::init(config);
    let boot_start = Instant::now();

    // DBGMCU: Keep CoreSight debug clocks running during sleep/stop
    embassy_stm32::pac::DBGMCU.cr().modify(|w| {
        w.set_stop(true);
        w.set_standby(true);
    });

    unsafe {
        let dbgmcu_cr = 0x4402_4004 as *mut u32;
        core::ptr::write_volatile(dbgmcu_cr, core::ptr::read_volatile(dbgmcu_cr) | 0x07);

        let dbgmcu_apb1lfzr = 0x4402_4008 as *mut u32;
        core::ptr::write_volatile(
            dbgmcu_apb1lfzr,
            core::ptr::read_volatile(dbgmcu_apb1lfzr) | (1 << 11) | (1 << 12),
        );
    }

    // -----------------------------------------------------------------------
    // Debug UART Console (from barefoc.toml)
    // -----------------------------------------------------------------------
    let dbg_uart = crate::bsp::embassy_bsp::take_debug_uart_peripherals!(p);
    let mut usart_cfg = UartConfig::default();
    usart_cfg.baudrate = crate::bsp::embassy_bsp::DEBUG_UART_BITRATE;
    let uart = Uart::new_blocking(dbg_uart.bus, dbg_uart.tx, dbg_uart.rx, usart_cfg).ok();

    // -----------------------------------------------------------------------
    // Soft-Reset Mailbox (0x2000_7FE0) Inspection & Address Selection
    // -----------------------------------------------------------------------
    let mailbox = unsafe { read_soft_reset_mailbox() };
    let (dynamic_node_id, mailbox_valid) = match mailbox {
        Some(mb) => {
            rprintln!(
                "Valid SoftResetMailbox detected at 0x{:08X}: magic = 0x{:016X}, dynamic_node_id = 0x{:02X}, bus_type = {}",
                SOFT_RESET_MAILBOX_ADDR,
                mb.magic,
                mb.dynamic_node_id,
                mb.bus_type
            );
            (mb.dynamic_node_id, true)
        }
        None => {
            rprintln!("SoftResetMailbox invalid or cold boot; falling back to Flash Sectors 14 & 15");
            (0, false)
        }
    };

    let (active_node_id, initial_motors) =
        load_boot_config(p.FLASH, dynamic_node_id, mailbox_valid);

    // -----------------------------------------------------------------------
    // Host MultiBus Target (Address from persisted NodeId / barefoc.toml)
    // -----------------------------------------------------------------------
    let _comm = crate::bsp::embassy_bsp::take_comm_peripherals!(p);

    let target_node_id = if (0x08..=0x77).contains(&active_node_id) {
        active_node_id
    } else {
        DEFAULT_COMM_ADDRESS
    };
    barefoc_stm32::set_active_dynamic_node_id(target_node_id);
    barefoc_stm32::set_active_bus_type(1);
    init_i3c_target(target_node_id, initial_motors);
    unsafe {
        (&*(&raw const TARGET_STORE).cast::<TargetType>()).unmask_irqs();
        interrupt::I3C2_EV.set_priority(Priority::P1);
        interrupt::I3C2_EV.enable();
        interrupt::I3C2_ER.set_priority(Priority::P1);
        interrupt::I3C2_ER.enable();
    }
    interrupt::TIM6.set_priority(Priority::P2);
    let i2c_spawner = I2C_TARGET_EXECUTOR.start(interrupt::TIM6);
    i2c_spawner.spawn(i3c_target_task().expect("Failed to create i3c target task token"));

    Timer::after_millis(1).await;
    let elapsed_ms = Instant::now().checked_duration_since(boot_start).map(|d| d.as_millis()).unwrap_or(0);
    rprintln!("I3C2 target ready at 0x{:02X} ({} ms after init).", active_node_id, elapsed_ms);


    let leds = crate::bsp::embassy_bsp::take_status_led_peripherals!(p);
    let gd = crate::bsp::embassy_bsp::take_quad_gate_driver_peripherals!(p);
    let vbus_adc = crate::bsp::embassy_bsp::take_vbus_adc_peripherals!(p);
    let sbus = crate::bsp::embassy_bsp::take_sensor_bus_peripherals!(p);

    let (motors_ref, i2c1_bus, led_b, led_r, has_sensor, vsens_pin) = init_board_and_motors(
        leds,
        gd,
        vbus_adc,
        sbus,
        &initial_motors,
        boot_start,
    );
    rprintln!("System running.");
    motor_task(motors_ref, i2c1_bus, led_b, led_r, has_sensor, rtt_down, uart, vsens_pin).await;
}
