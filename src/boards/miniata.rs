//! Miniata motor-only position test (single MT6701 on the motor shaft).
//!
//! * USART1 PB6=TX / PB7=RX @ 115200 8N1 (WCH-LinkE CDC)
//! * SPI2 PA12=SCK / PA15=MISO, shared SSI bus, Mode 1 @ 8 MHz
//!   - motor shaft encoder CS = PA11
//! * TIM1 3-PWM @ 25 kHz center-aligned: PB5 (A), PA9 (B), PA8 (C)
//! * DRV8311H: INL_A/B/C = PB1/PB0/PA7, nSLEEP = PB15
//!
//! Control architecture:
//!   * `P<deg>` motor bring-up mode (motor encoder only): motor-position P
//!     loop -> smoothed velocity setpoint (rad/s) -> velocity PI -> signed
//!     voltage magnitude -> apply_vector(ea, u), where
//!     ea = (mot_raw - zero) * dir * POLE_PAIRS, using the calibration
//!     zero/dir so the closed-loop field path is exercised.
//!   * `J<deg>` joint servo mode (`barefoc-kinematics`): cascaded dual loop
//!     with the output encoder on PC15 closed around `Actuator1DOF` +
//!     `LinearGear(275/13, inverted)`. The outer joint loop samples the
//!     output encoder, runs a P controller in joint space, projects the
//!     joint velocity through the transmission Jacobian (gain scheduling
//!     v_mot = v_joint / J(θ), J = -13/275), and injects the motor velocity
//!     target into the same inner velocity PI + apply_vector commutation.
//!
//! The transmission inversion is real: on hardware the commanded +20 rpm
//! forward spin reads negative on the motor encoder and the output encoder
//! reads positive (ratio ≈ -21, see progress.md), so the gear is created
//! with `.with_inverted(true)` and a positive 275/13 ratio.
//!
//! Test modes:
//!   - `P<deg>` position mode: arm and move the rotor to a motor-encoder angle
//!     target (signed offset in encoder degrees from the current position;
//!     e.g. P+30 rotates +30 deg). The velocity setpoint is slew-limited so
//!     the position PI does not step-saturate the loop. This proves the rotor
//!     can be positioned at all with the current commutation path.
//!   - `J<deg>` joint position mode: arm and move the *output shaft* to a
//!     target (signed output-encoder degrees from the current joint angle).
//!     Requires calibration (c) first. Exercises the harness identical to
//!     the production cascade but shares the debug commutation path.
//!   - `S` sweep mode: state machine that holds the current closed-loop
//!     electrical angle, then steps ea by -90deg / 0 / +90deg at 0.5V and
//!     1.0V, holding each ~=400ms, so we see whether the alignment law lands
//!     the field on the rotor, 90deg off, or reversed. If the rotor moves only
//!     at one sweep offset the zero is shifted by 90deg; if it moves at all
//!     magnitudes the field is fine; if nothing moves at all magnitudes the
//!     fault is downstream of the MCU.
//!   All armed modes share commutation, so comparing them separates the
//!   angle-law case (sweep finds a torque angle but position modes do not)
//!   from the downstream case (neither moves).
//!
//! Commands (host -> MCU, RX path verified working):
//!   c        calibrate sensor zero + direction (rotates ~13 deg)
//!   P<deg>   motor bring-up position mode: signed offset in motor-encoder
//!            degrees from the current rotor angle; e.g. P+30 or P-30
//!   J<deg>   joint servo position mode: signed offset in output-encoder
//!            degrees from the current joint angle (requires calibration)
//!   O<deg>   open-loop position mode: integrate a field angle for a signed
//!            rotor travel with NO encoder feedback (mode 5, 1 rad/s,
//!            voltage capped at 1.0 V, 10 s timeout). Command sign matches
//!            measured rotor direction; ~5 deg coast overshoot is expected.
//!   S        sweep mode: test the field at 3 electrical offsets from the
//!            current ea, at 0.5V and 1.0V; reads encoder between steps
//!   s        stop / disarm
//!   v        step the voltage limit up by 0.5 V (wraps at the 5 V rail)
//!   V<volts> set an absolute voltage limit (0-5 V supply rail)
//!   L<rad/s> set the position-loop velocity limit (slow-speed tests)
//!   W<rad/s> open-loop rotor speed; A<rad/s^2> open-loop ramp acceleration
//!            (no software caps; hardware OCP is the protection)
//!   K<ki> velocity-loop integral gain; R<rad/s^2> velocity slew (accel)
//!   f        save the current calibration to flash (skips 'c' on next boot)
//!   ?        help
//!
//! Faults (auto-disarm): 1 = position/joint error > STALL_LIMIT for 1 s,
//!                        2 = overspeed, 3 = host command timeout,
//!                        4 = calibration failed.
//!
//! Because the board -> host UART wire is unreliable, all values are also
//! published in `#[no_mangle]` atomics for SWD reads (resolve addresses with
//! `arm-none-eabi-nm`; they shift on every rebuild).

#![allow(dead_code, unused_variables, unused_assignments, unused_imports)]

use core::cell::{Cell, RefCell};
use core::f32::consts::PI;
use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering};

use rtt_target::{rprintln, ChannelMode::NoBlockSkip};

// RTT console transport (barefoc-rtt) + standard SimpleFOC line assembly.
// I2C multibus on PB6/PB7 and RTT over SWD are independent and both stay live.
use barefoc_commander::LineProcessor;
use barefoc_rtt::RttStream;

/// Reset the MCU (used by panic / HardFault so a standalone demo self-heals
/// instead of freezing forever).
/// Independent watchdog: ~8 s timeout, kicked once per control-loop tick.
/// Any lockup resets the board and the demo restarts automatically.
const IWDG_BASE: usize = 0x4000_3000;
fn iwdg_init() {
    unsafe {
        let kr = IWDG_BASE as *mut u32;
        let pr = (IWDG_BASE + 0x04) as *mut u32;
        let rlr = (IWDG_BASE + 0x08) as *mut u32;
        core::ptr::write_volatile(kr, 0x5555); // unlock
        core::ptr::write_volatile(pr, 0x04);   // /64
        core::ptr::write_volatile(rlr, 0x0FFF); // ~8 s at 32 kHz LSI
        core::ptr::write_volatile(kr, 0xAAAA); // initial reload
        core::ptr::write_volatile(kr, 0xCCCC); // start
    }
}
fn iwdg_feed() {
    unsafe { core::ptr::write_volatile(IWDG_BASE as *mut u32, 0xAAAA); }
}

fn self_reset() -> ! {
    unsafe {
        cortex_m::asm::dsb();
        core::ptr::write_volatile(0xE000_ED0C as *mut u32, 0x05FA0004);
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    barefoc_stm32::bootloader::barefoc_failsafe_recovery();
}

/// Any hard fault (including a debugger-induced DEBUGEVT) trips the failsafe
/// recovery trampoline: PWM disabled, DRV_SLEEP low, mailbox token written,
/// and system resets into BareBoot.
#[no_mangle]
pub unsafe extern "C" fn HardFault() -> ! {
    barefoc_stm32::bootloader::barefoc_failsafe_recovery();
}

use embassy_executor::Spawner;
use embassy_stm32::flash::Flash;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::spi::{Config as SpiConfig, Spi, MODE_1};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
#[cfg(feature = "uart")]
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_time::{Instant, Timer};
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{Operation, SpiBus, SpiDevice};

use barefoc::common::pid::PIDController;
use barefoc::common::time::Clock;
use barefoc::math::{_2PI, _SQRT3_2, normalize_angle, sincos, NOT_SET};
use barefoc::drivers::bldc_3pwm::BLDCDriver3PWM;
use barefoc::traits::driver::{BLDCDriver, FOCDriver};
use barefoc::traits::sensor::Sensor;
use barefoc::kinematics::actuator::MotorControl;
#[cfg(not(feature = "uart"))]
use barefoc_multibus::transport::RegServer;
#[cfg(not(feature = "uart"))]
use barefoc_stm32::bus_target_i2c::Stm32I2cTarget;
use barefoc::kinematics::{Actuator1DOF, LinearGear};
use drv83xx_rs::{Drv8311CsaGain, Drv8311HDriver3PWM};
use mt6xxx_rs::{Mt6701, SpiMode};

#[path = "miniata_config.rs"]
mod config;
#[cfg(not(feature = "uart"))]
#[path = "miniata_table.rs"]
mod table;

use self::config::FlashConfigData;
#[cfg(all(not(feature = "uart"), not(feature = "multi-motor")))]
use self::table::{MotorState, build_table, read_f32, write_f32};
#[cfg(all(not(feature = "uart"), feature = "multi-motor"))]
use self::table::{MultiMotorState, build_multi_motor_table, read_f32, write_f32};

/// Blocking-delay clock (default HSI).
const SYS_CLK_HZ: u32 = 64_000_000;

const POLE_PAIRS: f32 = 7.0;
const SUPPLY_VOLTAGE: f32 = 12.0;
const VOLTAGE_LIMIT: f32 = 1.5;
/// Physical supply rail. The only upper bound on the voltage command is what
/// the hardware can actually deliver; there is no documented stationary
/// voltage/current envelope.
const VOLTAGE_RAIL: f32 = 5.0;

/// Position loop: motor rad/s per rad of motor-position error.
const KP_POS: f32 = 3.0;
/// Motor velocity target clamp (rad/s) ~= 286 rpm.
const VEL_CLAMP: f32 = 30.0;
const KP_VEL_DEFAULT: f32 = 0.30;
const KI_VEL_DEFAULT: f32 = 0.3;
/// Low-pass factor for the velocity estimate.
const VEL_ALPHA: f32 = 0.05;
/// Align voltage during calibration.
/// Align voltage: full 5.0 V rail, to overcome gearbox stiction.
const V_ALIGN: f32 = 5.0;
/// Position error beyond this for > 1 s while armed => stall fault.
const STALL_LIMIT: f32 = 2.0;
/// Disarm if the host sends nothing for this long while armed.
const HOST_TIMEOUT_US: u32 = 30_000_000;
/// Speed limit while armed (rad/s).
const OVERSPEED: f32 = 60.0;

// ---------------------------------------------------------------------------
// Joint servo (barefoc-kinematics, mode J)
// ---------------------------------------------------------------------------

/// Nominal multi-stage planetary reduction: 275/13 ≈ 21.1538:1.
const GEAR_RATIO: f32 = 275.0 / 13.0;
/// Motor and output turn in opposite directions: commanded +20 rpm forward
/// reads negative on the motor encoder and positive on the output encoder
/// (measured ratio ≈ -21, see progress.md), so the transmission is inverted.
const GEAR_INVERTED: bool = true;
/// Outer joint position gain: joint rad/s per rad of joint error. The motor
/// velocity command = v_joint / |J| = v_joint * GEAR_RATIO, so this matches
/// the mode-1 motor-side P gain of KP_POS per rad of rotor error.
const KP_JOINT: f32 = 3.0;
/// Joint velocity command limit (rad/s): equivalent to VEL_CLAMP at the motor.
const JOINT_VEL_LIMIT: f32 = VEL_CLAMP / GEAR_RATIO;
/// Outer joint loop decimation: run the output-encoder sample + P update every
/// N control ticks (~1.5 ms at the 2 kHz tick).
const JOINT_OUTER_EVERY: u32 = 3;
/// Joint stall fault: sustained joint-position error while armed (rad, 1 s).
const JOINT_STALL_LIMIT: f32 = 0.6;
/// Slew limit for the mode-1 velocity setpoint (rad/s per second): a P step
/// ramps the velocity PI instead of stepping it into integrator saturation.
const VEL_SLEW_DEFAULT: f32 = 60.0;

// ---------------------------------------------------------------------------
// Open-loop position mode (command `O<deg>`, mode 5)
// ---------------------------------------------------------------------------
/// Default/open-loop speed and acceleration; both are runtime-settable via
/// `W<rad/s>` and `A<rad/s^2>` for limit characterization.
const OL_SPEED_DEFAULT: f32 = 1.0;
const OL_ACCEL_DEFAULT: f32 = 20.0;
/// Hard timeout so a lost-sync open-loop move can never run indefinitely.
const OL_TIMEOUT_MS: u32 = 10000;
/// Field is held stationary at a boosted voltage while the rotor aligns;
/// this is what overcomes gearbox stiction before the move starts.
const OL_ALIGN_S: f32 = 0.5;
/// Align voltage multiplier (clamped to the supply rail).
const OL_ALIGN_GAIN: f32 = 2.5;
/// After commanded travel, hold the final field while tapering voltage to
/// zero, so the rotor settles instead of coasting past the target.
const OL_HOLD_S: f32 = 0.4;


/// Trajectory recorder: one sample every `TRAJ_PERIOD_MS` while armed, so a
/// full move (position target or sweep) can be read back in a single SWD burst
/// after the move (the WCH-LinkE SWD link drops if the core is halted during
/// motion).
const TRAJ_N: usize = 480; // 480 * 20 ms = 9.6 s
const TRAJ_STRIDE: usize = 4; // t_ms, mot_pos, mot_vel, applied_voltage
const TRAJ_PERIOD_MS: u64 = 20;

// ---------------------------------------------------------------------------
// Scrub / sweep state (sweep mode is a small state machine on the loop)
// ---------------------------------------------------------------------------

const SWEEP_N: usize = 6; // 3 offsets x 2 magnitudes
const SWEEP_DEG: [f32; SWEEP_N] = [-90.0, 0.0, 90.0, -90.0, 0.0, 90.0];
const SWEEP_V: [f32; SWEEP_N] = [0.5, 0.5, 0.5, 1.0, 1.0, 1.0];

#[derive(Clone, Copy)]
struct SweepStep {
    offset: f32,   // electrical offset from the closed-loop ea, rad
    voltage: f32,
    since: u64,    // us since this step became active
}

// ---------------------------------------------------------------------------
// SWD-readable telemetry
// ---------------------------------------------------------------------------

#[no_mangle]
pub static LOOP_COUNT: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static RUN_STATE: AtomicU32 = AtomicU32::new(0); // 0 disarmed, 1 armed
#[no_mangle]
pub static FAULT_CODE: AtomicU32 = AtomicU32::new(0); // see module docs
#[no_mangle]
pub static UART_RX_BYTES: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static CALIB_DONE: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static SENSOR_DIR: AtomicU32 = AtomicU32::new(0); // +1 / -1
#[no_mangle]
pub static ZERO_ANGLE_BITS: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static MOT_POS_BITS: AtomicU32 = AtomicU32::new(0); // unwrapped rad
#[no_mangle]
pub static MOT_VEL_BITS: AtomicU32 = AtomicU32::new(0); // rad/s
#[no_mangle]
pub static MOT_VEL_TARGET_BITS: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static APPLIED_VOLTAGE_BITS: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static VOLTAGE_LIMIT_BITS: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PEAK_ERROR_BITS: AtomicU32 = AtomicU32::new(0); // since last command
#[no_mangle]
pub static SETTLE_MS: AtomicU32 = AtomicU32::new(0); // 0 = not settled yet
#[no_mangle]
pub static MOT_CRC_OK: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static MOT_CRC_FAILED: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static JOINT_POS_BITS: AtomicU32 = AtomicU32::new(0); // unwrapped output-encoder rad
#[no_mangle]
pub static JOINT_TARGET_BITS: AtomicU32 = AtomicU32::new(0); // target joint rad
#[no_mangle]
pub static JOINT_ERR_BITS: AtomicU32 = AtomicU32::new(0); // joint error rad
#[no_mangle]
pub static OUT_CRC_OK: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static OUT_CRC_FAILED: AtomicU32 = AtomicU32::new(0);
/// Flat trajectory buffer: sample i lives at `[i*4 .. i*4+4)` =
/// (t_ms, mot_pos_bits, mot_vel_bits, applied_voltage_bits).
#[no_mangle]
pub static TRAJ: [AtomicU32; TRAJ_N * TRAJ_STRIDE] =
    [const { AtomicU32::new(0) }; TRAJ_N * TRAJ_STRIDE];
#[no_mangle]
pub static TRAJ_LEN: AtomicU32 = AtomicU32::new(0);

fn store_f32(slot: &AtomicU32, v: f32) {
    slot.store(v.to_bits(), Ordering::Relaxed);
}

fn wrap_pi(mut a: f32) -> f32 {
    while a > PI {
        a -= _2PI;
    }
    while a < -PI {
        a += _2PI;
    }
    a
}

/// Accumulates single-turn angle samples into a continuous position.
struct Unwrapper {
    prev: f32,
    acc: f32,
    primed: bool,
}

impl Unwrapper {
    fn new() -> Self {
        Self {
            prev: 0.0,
            acc: 0.0,
            primed: false,
        }
    }

    fn next(&mut self, angle: f32) -> f32 {
        if !self.primed {
            self.prev = angle;
            self.acc = angle;
            self.primed = true;
        } else {
            self.acc += wrap_pi(angle - self.prev);
            self.prev = angle;
        }
        self.acc
    }
}

/// Fixed-capacity formatter so console lines need no heap.
struct StackWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> StackWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0 }
    }
    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

impl core::fmt::Write for StackWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let rem = self.buf.len().saturating_sub(self.len);
        let n = s.len().min(rem);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

/// Write one command reply to the dedicated RTT console up-channel.
///
/// Telemetry (`rprintln!`) stays on up-channel 0; replies go through the
/// `RttStream` on up-channel 1 so chatty telemetry cannot truncate them.
fn console_reply<W: embedded_io::Write>(writer: &mut W, args: core::fmt::Arguments<'_>) {
    let mut buf = [0u8; 192];
    let mut w = StackWriter::new(&mut buf);
    let _ = w.write_fmt(args);
    let _ = core::fmt::Write::write_str(&mut w, "\r\n");
    let _ = embedded_io::Write::write_all(writer, w.as_bytes());
}

/// Shared SPI bus adapter: asserts one CS, runs the transaction, releases it.
struct SharedSpiDevice<'a, BUS, CS> {
    bus: &'a RefCell<BUS>,
    cs: CS,
}

impl<'a, BUS: SpiBus, CS: OutputPin> SharedSpiDevice<'a, BUS, CS> {
    fn new(bus: &'a RefCell<BUS>, mut cs: CS) -> Self {
        let _ = cs.set_high();
        Self { bus, cs }
    }
}

impl<BUS: SpiBus, CS: OutputPin> embedded_hal::spi::ErrorType for SharedSpiDevice<'_, BUS, CS> {
    type Error = BUS::Error;
}

impl<BUS: SpiBus, CS: OutputPin> SpiDevice for SharedSpiDevice<'_, BUS, CS> {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        let mut bus = self.bus.borrow_mut();
        let _ = self.cs.set_low();
        cortex_m::asm::delay(SYS_CLK_HZ / 1_000_000); // ~1 us CS setup

        for op in operations {
            match op {
                Operation::Read(buf) => bus.read(buf)?,
                Operation::Write(buf) => bus.write(buf)?,
                Operation::Transfer(r, w) => bus.transfer(r, w)?,
                Operation::TransferInPlace(buf) => bus.transfer_in_place(buf)?,
                Operation::DelayNs(ns) => {
                    let cycles =
                        (*ns as u64 * SYS_CLK_HZ as u64 / 1_000_000_000) as u32;
                    cortex_m::asm::delay(cycles.max(1));
                }
            }
        }

        cortex_m::asm::delay(SYS_CLK_HZ / 1_000_000); // ~1 us CS hold
        let _ = self.cs.set_high();
        cortex_m::asm::delay(SYS_CLK_HZ / 1_000_000); // ~1 us bus release
        Ok(())
    }
}

/// Drive one voltage vector at electrical angle `ea` with signed magnitude
/// `mag` (a negative magnitude reverses the vector by 180 deg).
fn apply_vector<D: BLDCDriver>(driver: &mut D, ea: f32, mag: f32) {
    let (ang, m) = if mag >= 0.0 {
        (ea, mag)
    } else {
        (normalize_angle(ea + PI), -mag)
    };
    let (sa, ca) = sincos(ang);
    let ualpha = -sa * m;
    let ubeta = ca * m;

    let center = driver.params().voltage_limit / 2.0;
    let ua = ualpha + center;
    let ub = -0.5 * ualpha + _SQRT3_2 * ubeta + center;
    let uc = -0.5 * ualpha - _SQRT3_2 * ubeta + center;
    driver.set_pwm(ua, ub, uc);
}

// ---------------------------------------------------------------------------
// barefoc-kinematics adapters (joint servo mode J)
// ---------------------------------------------------------------------------

/// Microsecond clock view of the control-loop time base for the outer joint
/// loop ([`PIDController`] keeps its own timestamps from [`Clock::now_us`]).
///
/// Advanced every control tick from the measured embassy `Instant` delta, so
/// the PID sees the same real dt as the velocity PI. Interior mutability is
/// sound because the Embassy executor runs this single task thread and no
/// other context reads the counter.
struct TickClock {
    us: Cell<u32>,
}

impl Clock for TickClock {
    #[inline]
    fn now_us(&self) -> u32 {
        self.us.get()
    }
}

/// Mirror of the rotor-loop state for the [`MotorControl`] trait.
///
/// The inner velocity PI + `apply_vector` commutation run directly in the
/// main control tick (not behind [`MotorControl::loop_foc`]), so this adapter
/// only mirrors the measured rotor position/velocity into [`Actuator1DOF`]'s
/// outer loop and receives the motor velocity command back out of
/// [`Actuator1DOF::step_outer`].
struct RotorLoop {
    /// Unwrapped motor rotor position (rad) from the `Unwrapper` accumulator.
    shaft_pos: f32,
    /// Low-pass-filtered motor rotor velocity (rad/s).
    shaft_vel: f32,
    /// Motor velocity target written by the outer joint loop (rad/s).
    vel_target: f32,
    /// Motor velocity clamp handed to the outer loop (rad/s).
    vel_limit: f32,
}

impl<C: Clock> MotorControl<C> for RotorLoop {
    #[inline]
    fn shaft_angle(&self) -> f32 {
        self.shaft_pos
    }

    #[inline]
    fn shaft_velocity(&mut self) -> f32 {
        self.shaft_vel
    }

    #[inline]
    fn velocity_limit(&self) -> f32 {
        self.vel_limit
    }

    #[inline]
    fn move_motor(&mut self, target: f32, _clock: &C) {
        self.vel_target = target;
    }

    /// Inner-loop tick: the main control loop performs the velocity PI +
    /// commutation itself, so there is nothing to do inside the outer loop.
    #[inline]
    fn loop_foc(&mut self, _clock: &C) {}
}

/// SimpleFOC-style sensor alignment: hold electrical angle 0, then 90 deg,
/// and read which way the rotor moved. Returns (zero_angle, direction);
/// direction 0.0 means the rotor did not move (calibration failed).
async fn calibrate<D, S>(driver: &mut D, mot_enc: &Mt6701<S>) -> (f32, f32)
where
    D: BLDCDriver,
    S: SpiDevice,
{
    let mut th0 = 0.0f32;
    let mut delta = 0.0f32;
    // Retry: occasionally the rotor does not break away on the first 90 deg
    // step (stiction or a CRC-corrupted sample). Up to 3 attempts.
    for attempt in 0..3 {
        driver.set_pwm(0.0, 0.0, 0.0);
        // The DRV8311 phase enables (INL_x) gate the half-bridges in 3x PWM
        // mode, so the driver must be enabled or the align vectors do nothing.
        driver.enable();
        Timer::after_millis(100).await;

        apply_vector(driver, 0.0, V_ALIGN);
        Timer::after_millis(100).await;
        let a0a = mot_enc.read_angle_radians();
        Timer::after_millis(200).await;
        let a0b = mot_enc.read_angle_radians();
        Timer::after_millis(300).await;
        th0 = mot_enc.read_angle_radians();
        let t0 = mot_enc.telemetry();
        rprintln!("[CAL] attempt{} align0 @100/300/600ms = {:.1}/{:.1}/{:.1} deg, crc_fail {}",
                  attempt, a0a * 180.0 / PI, a0b * 180.0 / PI, th0 * 180.0 / PI, t0.crc_failed);

        apply_vector(driver, PI / 2.0, V_ALIGN);
        Timer::after_millis(100).await;
        let a1a = mot_enc.read_angle_radians();
        Timer::after_millis(200).await;
        let a1b = mot_enc.read_angle_radians();
        Timer::after_millis(300).await;
        let th1 = mot_enc.read_angle_radians();
        let t1 = mot_enc.telemetry();
        rprintln!("[CAL] attempt{} align1 @100/300/600ms = {:.1}/{:.1}/{:.1} deg, crc_fail {}",
                  attempt, a1a * 180.0 / PI, a1b * 180.0 / PI, th1 * 180.0 / PI, t1.crc_failed);

        delta = wrap_pi(th1 - th0);
        if delta.abs() >= 0.002 {
            break;
        }
    }

    driver.set_pwm(0.0, 0.0, 0.0);
    driver.disable();

    if delta.abs() < 0.002 {
        return (f32::NAN, f32::NAN);
    }
    let dir = if delta >= 0.0 { 1.0 } else { -1.0 };
    // Aligning at electrical angle 0 leaves the rotor 90 electrical degrees
    // behind where `apply_vector(el)` places the field (apply_vector encodes
    // ea as the field angle, so a field at 0 gives a motor-phase vector that
    // points the field 90 el off the rotor), so the half-turn aligns with the
    // rotor at the rotor's measured position. Shift the zero by that quarter
    // turn so uq produces maximum torque instead.
    let zero = th0 - (PI / 2.0) / (POLE_PAIRS * dir);
    rprintln!("[CAL] delta={:.2}deg zero={:.2}deg dir={}", delta * 180.0 / PI, zero * 180.0 / PI, dir);
    (zero, dir)
}

#[cfg(feature = "uart")]
/// Read a single byte from USART1 if available.
fn try_read_uart_byte() -> Option<u8> {
    let isr = embassy_stm32::pac::USART1.isr().read();
    if isr.rxne() {
        Some(embassy_stm32::pac::USART1.rdr().read().dr() as u8)
    } else {
        None
    }
}

/// Parse an optional sign plus decimal digits from a byte slice (e.g. `+30`, `-5`, `12.5`).
fn parse_deg_slice(s: &[u8]) -> Option<f32> {
    let mut sign = 1.0f32;
    let mut val = 0.0f32;
    let mut dec_div = 0.0f32;
    let mut idx = 0;
    while idx < s.len() && (s[idx] == b' ' || s[idx] == b'\t') {
        idx += 1;
    }
    if idx < s.len() && s[idx] == b'+' {
        idx += 1;
    } else if idx < s.len() && s[idx] == b'-' {
        sign = -1.0;
        idx += 1;
    }
    let mut digits = 0;
    while idx < s.len() {
        let b = s[idx];
        if b.is_ascii_digit() {
            digits += 1;
            let d = (b - b'0') as f32;
            if dec_div > 0.0 {
                val += d / dec_div;
                dec_div *= 10.0;
            } else {
                val = val * 10.0 + d;
            }
        } else if b == b'.' && dec_div == 0.0 {
            dec_div = 10.0;
        } else {
            break;
        }
        idx += 1;
    }
    if digits > 0 {
        Some(sign * val)
    } else {
        None
    }
}

pub async fn run(_spawner: Spawner) {
    // SAFETY: when Katapult owns the boot, the CPU comes up with VTOR = 0
    // (aliased to Katapult's vector table at 0x08000000) and Katapult itself
    // never re-points it, so every interrupt this firmware enables would
    // dispatch through the bootloader's table. Writing SCB->VTOR once here,
    // before any interrupt is enabled, redirects vectors to this image's own
    // table. The value must match `memory.x`'s `FLASH ORIGIN` (where
    // cortex-m-rt links the vector table for the Katapult layout) and is
    // 0x100-aligned as VTOR requires.
    unsafe {
        core::ptr::write_volatile(0xE000_ED08 as *mut u32, 0x0800_2000);
    }

    iwdg_init();

    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 1024,
                mode: NoBlockSkip,
                name: "Terminal"
            }
            1: {
                size: 512,
                mode: NoBlockSkip,
                name: "Console"
            }
        }
        down: {
            0: {
                size: 128,
                mode: NoBlockSkip,
                name: "Terminal"
            }
        }
    };
    // up0 = telemetry (rprintln!), up1 = interactive replies (RttStream),
    // down0 = commands. I2C1 multibus on PB6/PB7 remains fully independent.
    rtt_target::set_print_channel(channels.up.0);
    let mut console = RttStream::from_channels(channels.up.1, channels.down.0);
    let mut line_processor = LineProcessor::<64>::new();
    // Discard stale down-channel bytes left by a previous debug session: a
    // replayed 's' would otherwise stop the auto-starting demo at boot.
    {
        let mut junk = [0u8; 16];
        loop {
            let n = embedded_io::Read::read(&mut console, &mut junk).unwrap_or(0);
            if n == 0 { break; }
        }
    }
    rprintln!("\r\n=== Miniata joint servo (barefoc-kinematics) ===");
    rprintln!("RTT console active (barefoc-rtt). DISARMED.");
    rprintln!("c=calibrate p<deg>=cl-servo P<deg>=motor J<deg>=joint O<deg>=open-loop S=sweep s=stop v=cycle V<v>=set-v L<rps>=vel-limit W<w>=ol-speed A<a>=ol-accel K<ki>=vel-ki R<r>=vel-slew F<deg>=comm-offset Z=drv-reset D=demo k<kp> d<kd> b<v>=vel-tune ?=help");

    let mut config = embassy_stm32::Config::default();
    config.rcc.mux.spi2sel = embassy_stm32::pac::rcc::vals::Spisel::Per;
    let p = embassy_stm32::init(config);

    let mut led_r = Output::new(p.PC14, Level::Low, Speed::Low);
    let mut led_b = Output::new(p.PB8, Level::Low, Speed::Low);

    #[cfg(not(feature = "uart"))]
    let mut i2c_cfg = embassy_stm32::i2c::Config::default();
    #[cfg(not(feature = "uart"))]
    {
        i2c_cfg.frequency = embassy_stm32::time::khz(400);
    }
    #[cfg(all(not(feature = "uart"), feature = "time"))]
    {
        i2c_cfg.timeout = Duration::from_millis(2);
    }
    #[cfg(not(feature = "uart"))]
    let i2c = embassy_stm32::i2c::I2c::new_blocking(
        p.I2C1,
        p.PB6,
        p.PB7,
        i2c_cfg,
    );
    #[cfg(feature = "uart")]
    // ---- USART1 console (PB6=TX, PB7=RX) --------------------------------
    let mut uart = {
        let mut uc = UartConfig::default();
        uc.baudrate = 115_200;
        let mut u = Uart::new_blocking(p.USART1, p.PB6, p.PB7, uc).expect("USART1 init failed");
        // Enable RX (PB7 not auto-configured by tx-only constructor).
        u.blocking_write(b"\r\n=== Miniata joint servo (barefoc-kinematics) ===\r\n").ok();
        u.blocking_write(b"USART1 PB6=TX PB7=RX 115200 8N1. DISARMED.\r\n").ok();
        u.blocking_write(b"c=calibrate p<deg>=cl-servo P<deg>=motor J<deg>=joint O<deg>=open-loop S=sweep s=stop v=cycle V<v>=set-v L<rps>=vel-limit W<w>=ol-speed A<a>=ol-accel K<ki>=vel-ki R<r>=vel-slew F<deg>=comm-offset Z=drv-reset D=demo k<kp> d<kd> b<v>=vel-tune ?=help\r\n").ok();
        u.blocking_flush().ok();
        unsafe { core::ptr::write_volatile(0x20007F00 as *mut u32, 0x7BEAF00D); }
        u
    };
    // ---- Factory UID (96-bit) read via PAC base ------------------------------
    // STM32H503 factory UID base verified against stm32-metapac:
    // `embassy_stm32::pac::UID` is constructed from `0x08FFF800` (see
    // stm32-metapac src/chips/stm32h503kb/pac.rs `pub const UID: uid::Uid =
    // unsafe { uid::Uid::from_ptr(0x08ff_f800usize as _) };`). The task's
    // suggested 0x1FFF7590 is the legacy F1/F4 system-memory UID base and is
    // NOT valid for H5; the metapac value is authoritative.
    const STM32_UID_BASE: usize = 0x08FFF800;
    let mut uid_bytes = [0u8; 12];
    // SAFETY: 0x08FFF800 is the read-only factory-programmed UID region exposed
    // by the STM32H503 (3 little-endian 32-bit words = 12 bytes). It is not in
    // the executed flash range of this image and is only read here, once, at
    // init, before any task touches the bus. Volatile reads prevent the
    // compiler from caching or reordering across the MMIO-ish region.
    unsafe {
        let base = STM32_UID_BASE as *const u32;
        for i in 0..3usize {
            let w = core::ptr::read_volatile(base.add(i));
            uid_bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
    }

    #[cfg(all(not(feature = "uart"), not(feature = "multi-motor")))]
    let mut ctx = MotorState::default();
    #[cfg(all(not(feature = "uart"), not(feature = "multi-motor")))]
    {
        ctx.uid = uid_bytes;
    }
    #[cfg(all(not(feature = "uart"), not(feature = "multi-motor")))]
    let table = build_table(ctx);
    #[cfg(all(not(feature = "uart"), feature = "multi-motor"))]
    let table = build_multi_motor_table::<1>();
    #[cfg(not(feature = "uart"))]
    let mut server = RegServer::new(table);
    #[cfg(not(feature = "uart"))]
    {
        // Publish the UID into the RO register (app-side accessor; RO is bus-write
        // protected but app-side `set` is allowed for telemetry/identity).
        let _ = server.map_mut().set(0x58, &uid_bytes);
    }
    #[cfg(all(not(feature = "uart"), feature = "multi-motor"))]
    {
        server.map_mut().handler().uid = uid_bytes;
    }
    // Self-test gate (I2C control-table path only).
    #[cfg(all(feature = "self-test", not(feature = "uart")))]
    {
        const STATUS_ADDR: usize = 0x20007F00;
        let status = STATUS_ADDR as *mut u32;
        unsafe { core::ptr::write_volatile(status, 1) }; // running
        let mut result_code: u32 = 0;
        {
            let mut map = server.map_mut();
            // 1. device id RO @ 0x00 should be 0x4D
            let mut buf = [0u8; 1];
            if map.get(0x00, &mut buf).is_ok() && buf[0] == 0x4D {
                result_code |= 1 << 0;
            }
            // 2. write control mode = 2 (direct pos) RW @ 0x02
            if map.write_phase(&[0x02, 2]).is_ok() {
                let mut b = [0u8; 1];
                map.write_phase(&[0x02]).ok();
                if map.read_phase(1, &mut b).ok() == Some(1) && b[0] == 2 {
                    result_code |= 1 << 1;
                }
            }
            // 3. write target_pos (0x08) = 1.5 f32 LE, read back
            let v: f32 = 1.5;
            let vb = v.to_le_bytes();
            let mut phase = [0u8; 5];
            phase[0] = 0x08;
            phase[1..].copy_from_slice(&vb);
            if map.write_phase(&phase).is_ok() {
                map.write_phase(&[0x08]).ok();
                let mut out = [0u8; 4];
                if map.read_phase(4, &mut out).ok() == Some(4)
                    && f32::from_le_bytes(out) == v
                {
                    result_code |= 1 << 2;
                }
            }
            // 4. system_cmd (0x50) write 4 (zero pos) self-clears
            if map.write_phase(&[0x50, 4]).is_ok() {
                let mut b = [0u8; 1];
                map.write_phase(&[0x50]).ok();
                if map.read_phase(1, &mut b).ok() == Some(1) && b[0] == 0 {
                    result_code |= 1 << 3;
                }
            }
            // 5. RO rejection: write 0x20 (present_out_pos) must fail
            if map.write_phase(&[0x20, 1, 2, 3, 4]).is_err() {
                result_code |= 1 << 4;
            }
            // 6. UID published: read 0x58 (12 bytes) == uid_bytes
            map.write_phase(&[0x58]).ok();
            let mut ubuf = [0u8; 12];
            if map.read_phase(12, &mut ubuf).ok() == Some(12) && ubuf == uid_bytes {
                result_code |= 1 << 5;
            }
        }
        let final_status = if result_code == 0b111111 {
            0xDEAD0001u32
        } else {
            0xDEAD0000u32 | result_code
        };
        unsafe { core::ptr::write_volatile(status, final_status) };
        #[cfg(feature = "defmt")]
        defmt::info!("self-test result_code={} (0b{:06b})", result_code, result_code);
    }

    #[cfg(not(feature = "uart"))]
    let mut i2c_target = Stm32I2cTarget::new(i2c, 0x30, server).expect("I2C target init failed");
    #[cfg(not(feature = "uart"))]
    i2c_target.enable_bootloader(barefoc_stm32::bootloader::default_target());

    // ---- Shared SSI bus + motor MT6701 encoder -----------------------------
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = embassy_stm32::time::mhz(8);
    spi_config.mode = MODE_1;
    let spi = Spi::new_blocking_rxonly(p.SPI2, p.PA12, p.PA15, spi_config);
    let spi_bus = RefCell::new(spi);

    let mot_enc = Mt6701::new(SharedSpiDevice::new(
        &spi_bus,
        Output::new(p.PA11, Level::High, Speed::VeryHigh),
    ))
    .with_spi_mode(SpiMode::Mode1)
    .with_max_retries(3);

    // Output-shaft MT6701 (CS = PC15) closes the outer joint loop; its angle
    // is unwrapped by the barefoc `SensorBase` machinery inside step_outer.
    let out_enc = Mt6701::new(SharedSpiDevice::new(
        &spi_bus,
        Output::new(p.PC15, Level::High, Speed::VeryHigh),
    ))
    .with_spi_mode(SpiMode::Mode1)
    .with_max_retries(3);

    // ---- TIM1 3-PWM @ 25 kHz + DRV8311H ------------------------------------
    let ch1 = PwmPin::new(p.PA8, OutputType::PushPull);
    let ch2 = PwmPin::new(p.PA9, OutputType::PushPull);
    let ch3 = PwmPin::new(p.PB5, OutputType::PushPull);

    let pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1),
        Some(ch2),
        Some(ch3),
        None,
        embassy_stm32::time::khz(25),
        CountingMode::CenterAlignedBothInterrupts,
    );
    let mut channels = pwm.split();
    channels.ch1.enable();
    channels.ch2.enable();
    channels.ch3.enable();

    let en_a = Output::new(p.PB1, Level::Low, Speed::High);
    let en_b = Output::new(p.PB0, Level::Low, Speed::High);
    let en_c = Output::new(p.PA7, Level::Low, Speed::High);
    let drv_sleep = Output::new(p.PB15, Level::High, Speed::Low);

    let bldc_pwm = BLDCDriver3PWM::new(
        channels.ch3, // phase A = PB5
        channels.ch2, // phase B = PA9
        channels.ch1, // phase C = PA8
        Some(en_a),
        Some(en_b),
        Some(en_c),
    );

    let mut driver = Drv8311HDriver3PWM::new(bldc_pwm)
        .with_sleep_pin(drv_sleep)
        .with_csa_gain(Drv8311CsaGain::Gain0V25);
    driver.params_mut().voltage_power_supply = SUPPLY_VOLTAGE;
    driver.params_mut().voltage_limit = VOLTAGE_LIMIT;
    let _ = driver.init();
    driver.disable();

    // ---- Joint actuator rig (barefoc-kinematics, mode J) -------------------
    // 275/13 planetary with motor-encoder and output-encoder conventions on
    // opposite signs (measured ratio -21 on hardware), so the transmission is
    // created inverted and a positive 275/13 ratio. step_outer projects the
    // joint-space P output through the negative Jacobian (gain scheduling),
    // clamps to VEL_CLAMP, and writes the motor velocity target into the
    // `RotorLoop` mirror below.
    let gear = LinearGear::new(GEAR_RATIO).with_inverted(GEAR_INVERTED);
    let rot = RotorLoop {
        shaft_pos: 0.0,
        shaft_vel: 0.0,
        vel_target: 0.0,
        vel_limit: VEL_CLAMP,
    };
    let clock = TickClock { us: Cell::new(0) };
    let pid_joint = PIDController::new(
        KP_JOINT,
        0.0,
        0.0,
        NOT_SET,
        JOINT_VEL_LIMIT,
        NOT_SET,
        &clock,
    );
    let mut act = Actuator1DOF::new(rot, gear, out_enc, pid_joint)
        .with_downsample(JOINT_OUTER_EVERY);

    // ---- Controller state --------------------------------------------------
    let mut armed = false;
    let mut fault = 0u32;
    let mut voltage_limit = VOLTAGE_LIMIT;
    let mut zero_angle = 0.0f32;
    let mut sensor_dir = 0.0f32; // 0.0 = not calibrated
    let _rx_bytes = 0u32;
    let mut mode: u32 = 0; // 0 = hold, 1 = motor position, 2 = sweep, 3 = joint position
    let _prev_control_mode: u8 = 0;

    let mut joint_target: f32 = 0.0; // joint target (unwrapped output-encoder rad)
    let mut joint_err_deg: f32 = 0.0; // last joint error in degrees (telemetry)
    let mut sweep: [SweepStep; SWEEP_N] = core::array::from_fn(|_| SweepStep { offset: 0.0, voltage: 0.0, since: 0 });
    let mut sweep_idx: usize = 0;
    // Sweep and motor-position both use control mode 2; this flag keeps the
    // sweep voltage state machine from driving the P-mode position loop.
    let mut sweep_active = false;

    // Open-loop position state (mode 5). No encoder feedback is used during
    // the move; the encoder only seeds the starting electrical angle and is
    // read afterwards to verify how far the rotor actually travelled.
    let mut ol_active = false;
    let mut ol_target: f32 = 0.0;   // commanded rotor travel, rad (signed)
    let mut ol_pos: f32 = 0.0;      // integrated commanded rotor travel, rad
    let mut ol_start_ea: f32 = 0.0; // electrical angle at arming
    let mut ol_elapsed: f32 = 0.0;  // seconds since arming
    // Field angle persists across commands so consecutive moves do not each
    // pay a fresh align jump. Encoder readings bracket the travel for proof.
    let mut ol_ea: f32 = 0.0;
    let mut ol_seeded = false;
    let mut ol_start_mot: f32 = 0.0;
    let mut ol_start_out: f32 = 0.0;
    let mut ol_meas_start = false;
    let mut ol_hold = false;
    let mut ol_hold_elapsed: f32 = 0.0;
    let mut ol_speed = OL_SPEED_DEFAULT;   // commanded rotor speed, rad/s
    let mut ol_accel = OL_ACCEL_DEFAULT;   // trapezoidal ramp, rad/s^2
    let mut ol_cur_vel = 0.0f32;
    let mut ol_peak_vel = 0.0f32;          // peak encoder-measured rotor speed

    let mut mot_unwrap = Unwrapper::new();
    let mut prev_mot_pos: f32;
    let mut mot_vel = 0.0f32;
    let mut mot_vel_target = 0.0f32;
    let mut vel_integ = 0.0f32;
    let mut applied_voltage = 0.0f32;

    // position metrics
    let mut peak_error = 0.0f32;
    let mut settle_timer = 0.0f32;
    let mut settle_ms: u32 = 0;
    let mut settling = false;
    let mut settle_start = Instant::now();

    let mut stall_timer = 0.0f32;
    let _last_cmd = Instant::now();
    // tick + wall-clock anchors for the control loop and telemetry cadence.
    let mut last = Instant::now();
    let mut last_telem = last;
    let mut tick: u32 = 0;

    // trajectory recorder state
    let mut traj_idx: usize = 0;
    let mut traj_next_ms: u64 = 0;
    let mut traj_start = Instant::now();

    store_f32(&VOLTAGE_LIMIT_BITS, voltage_limit);

    // ---- Transport-mode state (defaulted; I2C path refreshes each tick) ------
    let mut target_pos: f32 = 0.0;
    let mut target_vel: f32 = 0.0;
    let mut target_effort: f32 = 0.0;
    let mut vel_max: f32 = 30.0;
    let mut kp_vel = KP_VEL_DEFAULT;
    let mut ki_vel = KI_VEL_DEFAULT;
    let mut kd_vel = 0.0f32;      // acceleration damping, V per rad/s^2
    let mut v_boost = 0.0f32;    // breakaway voltage while rotor is stalled
    let mut prev_mot_vel = 0.0f32;
    let mut overspeed_timer = 0.0f32;
    // Closed-loop field-servo (mode 6): encoder position closes the loop around
    // the verified open-loop field direction. No FOC commutation sign involved.
    let mut servo_ea = 0.0f32;
    // Demo sequencer: +360 deg, 200 ms, -360 deg, 500 ms, repeat.
    let mut demo_active = false;
    let mut demo_state = 0u8; // 1=+360, 2=wait200, 3=-360, 4=wait500
    let mut demo_wait = 0.0f32;
    let mut servo_last_pos = 0.0f32;
    let mut servo_stall = 0.0f32;
    // Output-shaft field servo (mode 7): same proven field actuator, but the
    // encoder feedback is the output shaft and the command is geared.
    let mut out_servo_ea = 0.0f32;
    let mut out_servo_last = 0.0f32;
    let mut out_servo_stall = 0.0f32;
    let mut out_servo_start = Instant::now();
    let mut cl_offset = 0.0f32; // closed-loop commutation phase offset, rad
    let mut vel_slew = VEL_SLEW_DEFAULT;
    let mut acc_max: f32 = 60.0;
    let mut pos_kp: f32 = 3.0;
    let mut pos_ki: f32 = 0.0;
    let mut pos_kd: f32 = 0.0;
    let mut limit_voltage: f32 = 1.5;
    let mut zero_offset: f32 = 0.0;

    // ---- Persistent calibration load (Phase E1) -----------------------------
    // Zero electrical angle + sensor direction live in the last 8 KB sector of
    // flash bank 2 (never executed from — erase/program do not stall the
    // control loop). Loaded here so a power cycle skips the rotating 'c' step.
    let mut flash = Flash::new_blocking(p.FLASH, embassy_stm32::flash::Config::default());
    match config::load(&mut flash) {
        Some(cfg) => {
            zero_angle = cfg.zero_electrical_angle;
            sensor_dir = cfg.sensor_dir;
            if sensor_dir != 0.0 {
                CALIB_DONE.store(1, Ordering::Relaxed);
            }
            SENSOR_DIR.store(sensor_dir as i32 as u32, Ordering::Relaxed);
            store_f32(&ZERO_ANGLE_BITS, zero_angle);
            #[cfg(not(feature = "uart"))]
            {
                // Apply persisted identity into the control table.
                let map = i2c_target.server_mut().map_mut();
                let _ = map.set(0x01, &[cfg.i2c_addr]);
                let _ = map.set(0x64, &[cfg.joint_slot]);
            }
            #[cfg(feature = "uart")]
            {
                let mut buf = [0u8; 80];
                let mut w = StackWriter::new(&mut buf);
                let _ = write!(w, "[CFG] calibration restored: zero {} mrad dir {}\r\n", (zero_angle * 1000.0) as i32, sensor_dir as i32);
                uart.blocking_write(w.as_bytes()).ok();
            }
            rprintln!("[CFG] calibration restored: zero {} mrad dir {}", (zero_angle * 1000.0) as i32, sensor_dir as i32);
        }
        None => {
            #[cfg(feature = "uart")]
            uart.blocking_write(b"[CFG] no calibration in flash; run 'c' then 'f'\r\n").ok();
            rprintln!("[CFG] no calibration in flash; run 'c' then 'f'");
        }
    }

    // ---- Auto-start demo setup ------------------------------------------
    // The board is a standalone demo: calibrate if needed, set safe demo
    // speed/voltage, then start the continuous loop without any host input.
    // Always calibrate: a persisted zero from a previous power cycle may
    // correspond to a different pole-equilibrium of the 7-pole-pair rotor.
    let (z, d) = calibrate(&mut driver, &mot_enc).await;
    zero_angle = z;
    sensor_dir = d;
    CALIB_DONE.store(if d != 0.0 { 1 } else { 0 }, Ordering::Relaxed);
    SENSOR_DIR.store(d as i32 as u32, Ordering::Relaxed);
    store_f32(&ZERO_ANGLE_BITS, z);
    voltage_limit = 2.0;
    driver.params_mut().voltage_limit = voltage_limit;
    vel_max = 5.0;
    act.motor.vel_limit = vel_max;
    kd_vel = 0.0;
    store_f32(&VOLTAGE_LIMIT_BITS, voltage_limit);
    #[cfg(not(feature = "uart"))]
    {
        let map = i2c_target.server_mut().map_mut();
        let _ = map.set(0x48, &voltage_limit.to_le_bytes());
        let _ = map.set(0x14, &vel_max.to_le_bytes());
        let _ = map.set(0x02, &[6u8]);
    }
    demo_active = true;
    demo_state = 1;
    demo_wait = 1.0;
    rprintln!("[DEMO] auto-start: output shaft +/-45deg, V={} L={}", voltage_limit, vel_max);

    // Prime the motor position so the first velocity estimate is not a spike.
    // The first delta is suppressed by the `tick > 1` guard in the loop, so a
    // single read here seeds the unwrapped accumulator.
    prev_mot_pos = mot_unwrap.next(mot_enc.read_angle_radians());

    // Prime the joint actuator's output encoder and the PID/control clock so
    // the first step_outer call measures with a sane baseline rather than a
    // zero dt from an empty clock.
    act.output_sensor.update(&clock);
    clock.us.set(1_000);

    let mut input_buf = [0u8; 32];
    let mut input_len: usize = 0;

    // Command replies go to the dedicated RTT console channel (up1), keeping
    // telemetry on up0. Expanded only inside the command-dispatch loop below.
    macro_rules! creply {
        ($($arg:tt)*) => {
            console_reply(console.up(), format_args!($($arg)*))
        };
    }

    // Mirror RTT command state into the I2C control table. Without this, the
    // multibus sync block would overwrite mode/targets from registers on the
    // very next tick and the RTT console could only read, never command.
    macro_rules! table_set {
        ($reg:expr, $data:expr) => {{
            #[cfg(not(feature = "uart"))]
            {
                let _ = i2c_target.server_mut().map_mut().set($reg, $data);
            }
            #[cfg(feature = "uart")]
            let _ = ($reg, $data);
        }};
    }

    loop {
        iwdg_feed();
        let now = Instant::now();
        let mut dt = (now - last).as_micros() as f32 * 1e-6;
        last = now;
        if dt <= 0.0 {
            dt = 5e-4;
        }
        if dt > 0.05 {
            dt = 0.05;
        }
        // Integrator step is capped so a telemetry stall cannot wind it up.
        let dt_i = dt.min(0.002);
        tick = tick.wrapping_add(1);

        // ---- Encoder sampling (every tick) --------------------------------
        let mot_raw = mot_enc.read_angle_radians();
        let mot_pos = mot_unwrap.next(mot_raw);
        let d_mot = if tick > 1 { mot_pos - prev_mot_pos } else { 0.0 };
        prev_mot_pos = mot_pos;
        let vel_raw = if dt > 0.0 { d_mot / dt } else { 0.0 };
        mot_vel += VEL_ALPHA * (vel_raw - mot_vel);

        // Keep the barefoc-kinematics time base in step with the loop delta
        // (dt is always > 0 here), and mirror the live rotor state into the
        // MotorControl adapter used by the outer joint loop.
        clock.us.set(clock.us.get().wrapping_add((dt * 1.0e6) as u32));
        act.motor.shaft_pos = mot_pos;
        act.motor.shaft_vel = mot_vel;

        // ---- Read commands from RTT down-channel via barefoc-rtt ----------
        // RttStream provides non-blocking embedded-io reads; LineProcessor
        // assembles complete lines, which are fed to the existing byte-wise
        // command dispatcher below (same grammar on every transport).
        let mut rtt_rx = [0u8; 16];
        if let Ok(n_rtt) = embedded_io::Read::read(&mut console, &mut rtt_rx) {
            if n_rtt > 0 {
                let _ = line_processor.feed_slice(&rtt_rx[..n_rtt], |line| {
                    for &b in line.as_bytes() {
                        if input_len < input_buf.len() {
                            input_buf[input_len] = b;
                            input_len += 1;
                        }
                    }
                    if input_len < input_buf.len() {
                        input_buf[input_len] = b'\n';
                        input_len += 1;
                    }
                });
            }
        }

        #[cfg(feature = "uart")]
        while let Some(b) = try_read_uart_byte() {
            rx_bytes = rx_bytes.wrapping_add(1);
            UART_RX_BYTES.store(rx_bytes, Ordering::Relaxed);
            if input_len < input_buf.len() {
                input_buf[input_len] = b;
                input_len += 1;
            }
        }

        while input_len > 0 {
            let b = input_buf[0];
            match b {
                b'\r' | b'\n' | b' ' | b'\t' => {
                    input_buf.copy_within(1..input_len, 0);
                    input_len -= 1;
                }
                b'k' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(v) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            kp_vel = v.clamp(0.0, 20.0);
                            creply!("[CMD] velocity Kp={}", kp_vel);
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else { break; }
                }
                b'd' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(v) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            kd_vel = v.clamp(0.0, 5.0);
                            creply!("[CMD] velocity Kd={}", kd_vel);
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else { break; }
                }
                b'b' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(v) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            v_boost = v.clamp(0.0, 5.0);
                            creply!("[CMD] breakaway boost={}V", v_boost);
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else { break; }
                }
                b'Z' if !armed => {
                    input_buf.copy_within(1..input_len, 0);
                    input_len -= 1;
                    // Pulse nSLEEP low/high to clear a latched DRV8311 fault.
                    driver.set_pwm(0.0, 0.0, 0.0);
                    let _ = driver.reset_faults();
                    creply!("[CMD] DRV8311 nSLEEP fault reset");
                }
                b'c' if !armed => {
                    input_buf.copy_within(1..input_len, 0);
                    input_len -= 1;
                    creply!("[CMD] calibrating (rotates ~13 deg)...");
                    #[cfg(feature = "uart")]
                    uart.blocking_write(b"[CMD] calibrating (rotates ~13 deg)...\r\n").ok();
                    let (z, d) = calibrate(&mut driver, &mot_enc).await;
                    zero_angle = z;
                    sensor_dir = d;
                    CALIB_DONE.store(if d != 0.0 { 1 } else { 0 }, Ordering::Relaxed);
                    SENSOR_DIR.store(d as i32 as u32, Ordering::Relaxed);
                    store_f32(&ZERO_ANGLE_BITS, z);
                    if d == 0.0 {
                        fault = 4;
                        FAULT_CODE.store(4, Ordering::Relaxed);
                        creply!("[FAULT 4] calibration failed (no rotor motion)");
                        #[cfg(feature = "uart")]
                        uart.blocking_write(b"[FAULT 4] calibration failed (no rotor motion)\r\n").ok();
                    } else {
                        fault = 0;
                        FAULT_CODE.store(0, Ordering::Relaxed);
                        creply!("[CMD] calibrated: zero={}mrad dir={}", (z * 1000.0) as i32, d as i32);
                        #[cfg(feature = "uart")]
                        uart.blocking_write(b"[CMD] calibrated\r\n").ok();
                    }
                }
                b'O' if !armed && !ol_active => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(deg) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            if deg.abs() < 0.1 {
                                creply!("[CMD] OL: use |deg| >= 0.1");
                            } else {
                                ol_target = deg * PI / 180.0;
                                ol_pos = 0.0;
                                ol_elapsed = 0.0;
                                // First move seeds from the calibrated rotor
                                // angle; later moves continue from the field
                                // angle left by the previous move.
                                ol_ea = normalize_angle((mot_raw - zero_angle) * sensor_dir * POLE_PAIRS);
                                ol_seeded = true;
                                ol_start_ea = ol_ea;
                                ol_meas_start = false;
                                ol_hold = false;
                                ol_hold_elapsed = 0.0;
                                ol_cur_vel = 0.0;
                                ol_peak_vel = 0.0;
                                ol_active = true;
                                mode = 5;
                                armed = true;
                                sweep_active = false;
                                fault = 0;
                                FAULT_CODE.store(0, Ordering::Relaxed);
                                driver.enable();
                                applied_voltage = 0.0;
                                RUN_STATE.store(1, Ordering::Relaxed);
                                // LEDs stay dark during normal demo operation
                                TRAJ_LEN.store(0, Ordering::Relaxed);
                                traj_idx = 0;
                                traj_next_ms = 0;
                                traj_start = now;
                                table_set!(0x02, &[5u8]);
                                creply!("[CMD] OL ARMED target={}deg start_ea={}deg", deg as i32, (ol_start_ea * 180.0 / PI) as i32);
                            }
                        } else {
                            creply!("[CMD] usage: O<deg>  e.g. O+30 or O-30");
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else {
                        break;
                    }
                }
                b'D' if !armed => {
                    input_buf.copy_within(1..input_len, 0);
                    input_len -= 1;
                    demo_active = !demo_active;
                    demo_state = if demo_active { 1 } else { 0 };
                    demo_wait = 0.0;
                    led_b.set_low();
                    creply!("[CMD] DEMO {}", if demo_active { "START +360/-360" } else { "STOP" });
                }
                b'j' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(deg) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            act.output_sensor.update(&clock);
                            let out_angle = Sensor::<TickClock>::get_angle(&act.output_sensor);
                            target_pos = out_angle + deg * PI / 180.0;
                            out_servo_ea = servo_ea;
                            out_servo_last = out_angle;
                            out_servo_stall = 0.0;
                            out_servo_start = now;
                            mode = 7;
                            armed = true;
                            demo_active = false;
                            fault = 0;
                            FAULT_CODE.store(0, Ordering::Relaxed);
                            driver.enable();
                            applied_voltage = 0.0;
                            RUN_STATE.store(1, Ordering::Relaxed);
                            table_set!(0x02, &[7u8]);
                            creply!("[CMD] OUT-SERVO ARMED target={}deg", deg as i32);
                        } else {
                            creply!("[CMD] usage: j<deg>  e.g. j+45");
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else { break; }
                }
                b'p' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(deg) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            if deg.abs() >= 0.1 {
                                target_pos = mot_pos + deg * PI / 180.0;
                                servo_ea = normalize_angle((mot_raw - zero_angle) * sensor_dir * POLE_PAIRS);
                servo_last_pos = mot_pos;
                servo_stall = 0.0;
                                mode = 6;
                                armed = true;
                                sweep_active = false;
                                ol_active = false;
                                fault = 0;
                                FAULT_CODE.store(0, Ordering::Relaxed);
                                driver.enable();
                                vel_integ = 0.0;
                                prev_mot_vel = mot_vel;
                                overspeed_timer = 0.0;
                                applied_voltage = 0.0;
                                RUN_STATE.store(1, Ordering::Relaxed);
                                // LEDs stay dark during normal demo operation
                                TRAJ_LEN.store(0, Ordering::Relaxed);
                                traj_idx = 0;
                                traj_next_ms = 0;
                                traj_start = now;
                                table_set!(0x02, &[6u8]);
                                creply!("[CMD] CL-SERVO ARMED target={}deg", deg as i32);
                            } else {
                                creply!("[CMD] usage: p<deg>  e.g. p+30");
                            }
                        } else {
                            creply!("[CMD] usage: p<deg>  e.g. p+30");
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else { break; }
                }
                b'P' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        let num_slice = &input_buf[1..1 + pos];
                        if let Some(deg) = parse_deg_slice(num_slice) {
                            target_pos = mot_pos + deg * PI / 180.0;
                            zero_offset = 0.0;
                            // Route the standard P command through the proven
                            // encoder-closed field servo (mode 6).
                            mode = 6;
                            servo_ea = normalize_angle((mot_raw - zero_angle) * sensor_dir * POLE_PAIRS);
                servo_last_pos = mot_pos;
                servo_stall = 0.0;
                            sweep_active = false;
                            ol_active = false;
                            // Keep the I2C control table in sync: the multibus
                            // path refreshes these registers every tick.
                            table_set!(0x02, &[6u8]);
                            table_set!(0x08, &target_pos.to_le_bytes());
                            table_set!(0x4C, &0f32.to_le_bytes());
                            armed = true;
                            fault = 0;
                            FAULT_CODE.store(0, Ordering::Relaxed);
                            driver.enable();
                            mot_vel_target = 0.0;
                            vel_integ = 0.0;
                            prev_mot_vel = 0.0;
                            overspeed_timer = 0.0;
                            stall_timer = 0.0;
                            peak_error = 0.0;
                            settle_timer = 0.0;
                            settle_ms = 0;
                            settling = false;
                            settle_start = now;
                            applied_voltage = 0.0;
                            RUN_STATE.store(1, Ordering::Relaxed);
                            // LEDs stay dark during normal demo operation
                            TRAJ_LEN.store(0, Ordering::Relaxed);
                            traj_idx = 0;
                            traj_next_ms = 0;
                            traj_start = now;
                            creply!("[CMD] ARMED pos target={}deg (offset={}deg)", (target_pos * 180.0 / PI) as i32, deg as i32);
                            #[cfg(feature = "uart")]
                            uart.blocking_write(b"[CMD] ARMED pos\r\n").ok();
                        } else {
                            creply!("[CMD] usage: P<deg>  e.g. P+30 or P-30");
                            #[cfg(feature = "uart")]
                            uart.blocking_write(b"[CMD] usage: P<deg>  e.g. P+30 or P-30\r\n").ok();
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else {
                        break;
                    }
                }
                b'J' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if sensor_dir == 0.0 || zero_angle == 0.0 {
                            creply!("[CMD] calibrate first (c)");
                            #[cfg(feature = "uart")]
                            uart.blocking_write(b"[CMD] calibrate first (c)\r\n").ok();
                        } else if let Some(deg) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            act.output_sensor.update(&clock);
                            target_pos = Sensor::<TickClock>::get_angle(&act.output_sensor) + deg * PI / 180.0;
                            zero_offset = 0.0;
                            mode = 1;
                            sweep_active = false;
                            table_set!(0x02, &[1u8]);
                            table_set!(0x08, &target_pos.to_le_bytes());
                            table_set!(0x4C, &0f32.to_le_bytes());
                            armed = true;
                            fault = 0;
                            FAULT_CODE.store(0, Ordering::Relaxed);
                            driver.enable();
                            mot_vel_target = 0.0;
                            vel_integ = 0.0;
                            prev_mot_vel = 0.0;
                            overspeed_timer = 0.0;
                            stall_timer = 0.0;
                            peak_error = 0.0;
                            settle_timer = 0.0;
                            settle_ms = 0;
                            settling = false;
                            settle_start = now;
                            applied_voltage = 0.0;
                            RUN_STATE.store(1, Ordering::Relaxed);
                            // LEDs stay dark during normal demo operation
                            TRAJ_LEN.store(0, Ordering::Relaxed);
                            traj_idx = 0;
                            traj_next_ms = 0;
                            traj_start = now;
                            creply!("[CMD] ARMED joint target={}deg (offset={}deg)", (target_pos * 180.0 / PI) as i32, deg as i32);
                            #[cfg(feature = "uart")]
                            uart.blocking_write(b"[CMD] ARMED joint\r\n").ok();
                        } else {
                            creply!("[CMD] usage: J<deg>");
                            #[cfg(feature = "uart")]
                            uart.blocking_write(b"[CMD] usage: J<deg>\r\n").ok();
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else {
                        break;
                    }
                }
                b'S' if !armed => {
                    input_buf.copy_within(1..input_len, 0);
                    input_len -= 1;
                    if sensor_dir == 0.0 || zero_angle == 0.0 {
                        creply!("[CMD] calibrate first (c)");
                        #[cfg(feature = "uart")]
                        uart.blocking_write(b"[CMD] calibrate first (c)\r\n").ok();
                        continue;
                    }
                    for i in 0..SWEEP_N {
                        let off_rad = SWEEP_DEG[i] * PI / 180.0;
                        sweep[i] = SweepStep {
                            offset: off_rad,
                            voltage: SWEEP_V[i],
                            since: 0,
                        };
                    }
                    sweep_idx = 0;
                    sweep[0].since = 0;
                    sweep_active = true;
                    mode = 2;
                    table_set!(0x02, &[2u8]);
                    armed = true;
                    fault = 0;
                    FAULT_CODE.store(0, Ordering::Relaxed);
                    driver.enable();
                    mot_vel_target = 0.0;
                    vel_integ = 0.0;
                    stall_timer = 0.0;
                    peak_error = 0.0;
                    settle_timer = 0.0;
                    settle_ms = 0;
                    settling = false;
                    settle_start = now;
                    applied_voltage = 0.0;
                    RUN_STATE.store(1, Ordering::Relaxed);
                    // LEDs stay dark during normal demo operation
                    TRAJ_LEN.store(0, Ordering::Relaxed);
                    traj_idx = 0;
                    traj_next_ms = 0;
                    traj_start = now;
                    creply!("[CMD] ARMED sweep");
                    #[cfg(feature = "uart")]
                    uart.blocking_write(b"[CMD] ARMED sweep\r\n").ok();
                }
                b's' => {
                    input_buf.copy_within(1..input_len, 0);
                    input_len -= 1;
                    armed = false;
                    mode = 0;
                    sweep_active = false;
                    ol_active = false;
                    demo_active = false;
                    demo_state = 0;
                    demo_wait = 0.0;
                    table_set!(0x02, &[0u8]);
                    vel_integ = 0.0;
                    mot_vel_target = 0.0;
                    applied_voltage = 0.0;
                    driver.set_pwm(0.0, 0.0, 0.0);
                    driver.disable();
                    led_r.set_low();
                    RUN_STATE.store(0, Ordering::Relaxed);
                    creply!("[CMD] STOPPED");
                    #[cfg(feature = "uart")]
                    uart.blocking_write(b"[CMD] STOPPED\r\n").ok();
                }
                b'L' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(rps) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            let rps = rps.clamp(0.1, 100.0);
                            vel_max = rps;
                            act.motor.vel_limit = rps;
                            creply!("[CMD] vel limit={}rad/s", rps);
                        } else {
                            creply!("[CMD] usage: L<rad/s>  e.g. L1 or L0.5");
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else {
                        break;
                    }
                }
                b'F' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(deg) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            cl_offset = (deg.clamp(-360.0, 360.0)) * PI / 180.0;
                            creply!("[CMD] commutation offset={}deg", deg as i32);
                        } else {
                            creply!("[CMD] usage: F<deg>  e.g. F180");
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else {
                        break;
                    }
                }
                b'K' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(ki) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            ki_vel = ki.clamp(0.0, 20.0);
                            creply!("[CMD] velocity Ki={}", ki_vel);
                        } else {
                            creply!("[CMD] usage: K<ki>  e.g. K0.2");
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else {
                        break;
                    }
                }
                b'R' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(slew) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            vel_slew = slew.clamp(1.0, 2000.0);
                            creply!("[CMD] vel slew={}rad/s2", vel_slew);
                        } else {
                            creply!("[CMD] usage: R<rad/s2>  e.g. R200");
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else {
                        break;
                    }
                }
                b'W' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(rps) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            ol_speed = rps.clamp(0.1, 300.0);
                            creply!("[CMD] OL speed={}rad/s", ol_speed);
                        } else {
                            creply!("[CMD] usage: W<rad/s>  e.g. W10");
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else {
                        break;
                    }
                }
                b'A' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(acc) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            ol_accel = acc.clamp(1.0, 2000.0);
                            creply!("[CMD] OL accel={}rad/s2", ol_accel);
                        } else {
                            creply!("[CMD] usage: A<rad/s2>  e.g. A50");
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else {
                        break;
                    }
                }
                b'V' if !armed => {
                    let end_idx = input_buf[1..input_len].iter().position(|&c| c == b'\r' || c == b'\n' || c == b' ');
                    if let Some(pos) = end_idx {
                        if let Some(volts) = parse_deg_slice(&input_buf[1..1 + pos]) {
                            let volts = volts.clamp(0.0, VOLTAGE_RAIL);
                            voltage_limit = volts;
                            driver.params_mut().voltage_limit = volts;
                            limit_voltage = volts;
                            table_set!(0x48, &volts.to_le_bytes());
                            store_f32(&VOLTAGE_LIMIT_BITS, volts);
                            creply!("[CMD] voltage limit={}mV", (volts * 1000.0) as i32);
                        } else {
                            creply!("[CMD] usage: V<volts>  e.g. V1.5 or V2.5");
                        }
                        let consume = (1 + pos + 1).min(input_len);
                        input_buf.copy_within(consume..input_len, 0);
                        input_len -= consume;
                    } else if input_len >= 16 {
                        input_buf.copy_within(1..input_len, 0);
                        input_len -= 1;
                    } else {
                        break;
                    }
                }
                b'v' => {
                    input_buf.copy_within(1..input_len, 0);
                    input_len -= 1;
                    voltage_limit = if voltage_limit >= VOLTAGE_RAIL {
                        0.5
                    } else {
                        (voltage_limit + 0.5).min(VOLTAGE_RAIL)
                    };
                    driver.params_mut().voltage_limit = voltage_limit;
                    limit_voltage = voltage_limit;
                    table_set!(0x48, &voltage_limit.to_le_bytes());
                    store_f32(&VOLTAGE_LIMIT_BITS, voltage_limit);
                    creply!("[CMD] voltage limit={}mV", (voltage_limit * 1000.0) as i32);
                }
                b'f' if !armed => {
                    input_buf.copy_within(1..input_len, 0);
                    input_len -= 1;
                    if sensor_dir == 0.0 || zero_angle == 0.0 {
                        creply!("[CMD] calibrate first (c)");
                        #[cfg(feature = "uart")]
                        uart.blocking_write(b"[CMD] calibrate first (c)\r\n").ok();
                        continue;
                    }
                    let mut record = FlashConfigData::new(zero_angle, sensor_dir, 0x30, 0);
                    match config::store(&mut flash, &mut record) {
                        Ok(()) => {
                            creply!("[CMD] calibration saved to flash");
                            #[cfg(feature = "uart")]
                            uart.blocking_write(b"[CMD] calibration saved to flash\r\n").ok();
                        }
                        Err(()) => {
                            creply!("[CMD] flash store failed");
                            #[cfg(feature = "uart")]
                            uart.blocking_write(b"[CMD] flash store failed\r\n").ok();
                        }
                    }
                }
                b'?' => {
                    input_buf.copy_within(1..input_len, 0);
                    input_len -= 1;
                    creply!("cmds: c=calibrate p<deg>=cl-servo P<deg>=motor-pos J<deg>=joint-pos O<deg>=open-loop S=sweep s=stop v=cycle V<v>=set-v L<rps>=vel-limit W<w>=ol-speed A<a>=ol-accel K<ki>=vel-ki R<r>=vel-slew F<deg>=comm-offset Z=drv-reset D=demo k<kp> d<kd> b<v>=vel-tune f=save-cal ?=help");
                    #[cfg(feature = "uart")]
                    uart.blocking_write(b"cmds: c=calibrate p<deg>=cl-servo P<deg>=motor-pos J<deg>=joint-pos O<deg>=open-loop S=sweep s=stop v=cycle V<v>=set-v L<rps>=vel-limit W<w>=ol-speed A<a>=ol-accel K<ki>=vel-ki R<r>=vel-slew F<deg>=comm-offset Z=drv-reset D=demo k<kp> d<kd> b<v>=vel-tune f=save-cal ?=help\r\n").ok();
                }
                _ => {
                    input_buf.copy_within(1..input_len, 0);
                    input_len -= 1;
                }
            }
        }

        // ---- Demo sequencer: +360 / 200 ms / -360 / 500 ms / repeat -------
        if demo_active && !armed {
            if demo_wait > 0.0 {
                demo_wait -= dt;
                if demo_wait < 0.0 { demo_wait = 0.0; }
            } else if demo_state == 1 || demo_state == 3 {
                let dir = if demo_state == 1 { 1.0 } else { -1.0 };
                act.output_sensor.update(&clock);
                let out_angle = Sensor::<TickClock>::get_angle(&act.output_sensor);
                target_pos = out_angle + dir * 45.0 * PI / 180.0;
                out_servo_ea = normalize_angle((mot_raw - zero_angle) * sensor_dir * POLE_PAIRS);
                out_servo_last = out_angle;
                out_servo_stall = 0.0;
                out_servo_start = now;
                mode = 7;
                armed = true;
                sweep_active = false;
                ol_active = false;
                fault = 0;
                FAULT_CODE.store(0, Ordering::Relaxed);
                driver.enable();
                vel_integ = 0.0;
                prev_mot_vel = mot_vel;
                overspeed_timer = 0.0;
                applied_voltage = 0.0;
                RUN_STATE.store(1, Ordering::Relaxed);
                // LEDs stay dark during normal demo operation
                TRAJ_LEN.store(0, Ordering::Relaxed);
                traj_idx = 0;
                traj_next_ms = 0;
                traj_start = now;
                table_set!(0x02, &[6u8]);
                creply!("[CMD] DEMO output {}45", if dir > 0.0 { "+" } else { "-" });
            } else {
                demo_state = if demo_state == 2 { 3 } else { 1 };
            }
        }

        #[cfg(not(feature = "uart"))]
        {
        // ---- I2C control-table service (before motor step) -----------------
        let _ = i2c_target.service_once::<embassy_stm32::peripherals::I2C1>();

        // Handle command flags raised by register hooks.
        let (do_calib, do_zero, do_reboot_bl, do_reboot_sys, do_save, do_load, do_erase) = {
            let ctx = i2c_target.server_mut().map_mut().handler();
            let r = (
                ctx.calibrate_requested,
                ctx.zero_pos_requested,
                ctx.reboot_bootloader_requested,
                ctx.reboot_system_requested,
                ctx.save_requested,
                ctx.load_requested,
                ctx.erase_requested,
            );
            ctx.calibrate_requested = false;
            ctx.zero_pos_requested = false;
            ctx.reboot_bootloader_requested = false;
            ctx.reboot_system_requested = false;
            ctx.save_requested = false;
            ctx.load_requested = false;
            ctx.erase_requested = false;
            r
        };

        if do_calib && !armed {
            let (z, d) = calibrate(&mut driver, &mot_enc).await;
            zero_angle = z;
            sensor_dir = d;
            CALIB_DONE.store(if d != 0.0 { 1 } else { 0 }, Ordering::Relaxed);
            SENSOR_DIR.store(d as i32 as u32, Ordering::Relaxed);
            store_f32(&ZERO_ANGLE_BITS, z);
            let map = i2c_target.server_mut().map_mut();
            let _ = map.set(0x05, &[if d != 0.0 { 1u8 } else { 0u8 }]);
            #[cfg(feature = "defmt")]
            defmt::info!("calibrated zero {} dir {}", z, d);
        }
        if do_zero {
            mot_unwrap = Unwrapper::new();
            prev_mot_pos = mot_unwrap.next(mot_enc.read_angle_radians());
            #[cfg(feature = "defmt")]
            defmt::info!("zero pos reset");
        }
        if do_save {
            // Persist the current calibration + identity into the flash config
            // block (blocking write; only safe while disarmed, which the
            // command path guarantees by being invoked from the motor loop).
            let mut i2c_buf = [0u8; 1];
            let i2c_addr = i2c_target
                .server()
                .map()
                .get(0x01, &mut i2c_buf)
                .ok()
                .map(|_| i2c_buf[0])
                .unwrap_or(self::table::DEFAULT_I2C_ADDR);
            let joint_slot = i2c_target
                .server()
                .map()
                .get(0x64, &mut i2c_buf)
                .ok()
                .map(|_| i2c_buf[0])
                .unwrap_or(0);
            let mut record =
                FlashConfigData::new(zero_angle, sensor_dir, i2c_addr, joint_slot);
            match config::store(&mut flash, &mut record) {
                Ok(()) => {
                    #[cfg(feature = "defmt")]
                    defmt::info!("save config ok");
                }
                Err(()) => {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("save config failed");
                }
            }
        }
        if do_load {
            match config::load(&mut flash) {
                Some(cfg) => {
                    zero_angle = cfg.zero_electrical_angle;
                    sensor_dir = cfg.sensor_dir;
                    if sensor_dir != 0.0 {
                        CALIB_DONE.store(1, Ordering::Relaxed);
                    }
                    SENSOR_DIR.store(sensor_dir as i32 as u32, Ordering::Relaxed);
                    store_f32(&ZERO_ANGLE_BITS, zero_angle);
                    {
                        let map = i2c_target.server_mut().map_mut();
                        let _ = map.set(0x01, &[cfg.i2c_addr]);
                        let _ = map.set(0x64, &[cfg.joint_slot]);
                    }
                    #[cfg(feature = "defmt")]
                    defmt::info!("load config ok");
                }
                None => {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("load config: no valid record");
                }
            }
        }
        if do_erase {
            #[cfg(feature = "defmt")]
            defmt::info!("erase config stub");
        }
        if do_reboot_bl {
            armed = false;
            mode = 0;
            driver.set_pwm(0.0, 0.0, 0.0);
            driver.disable();
            barefoc_stm32::bootloader::enter_bareboot(barefoc_stm32::bootloader::default_target());
        }
        if do_reboot_sys {
            unsafe {
                core::ptr::write_volatile(0xE000ED0C as *mut u32, 0x05FA0004);
            }
            loop {}
        }

        // Sync mode from control table.
        let mut cm_buf = [0u8; 1];
        let control_mode = i2c_target
            .server()
            .map()
            .get(0x02, &mut cm_buf)
            .ok()
            .map(|_| cm_buf[0])
            .unwrap_or(0);
        mode = control_mode as u32;
        armed = mode != 0;

        // ---- Read control-table targets/gains each tick --------------------
        target_pos = read_f32(i2c_target.server().map(), 0x08);
        target_vel = read_f32(i2c_target.server().map(), 0x0C);
        target_effort = read_f32(i2c_target.server().map(), 0x10);
        vel_max = read_f32(i2c_target.server().map(), 0x14);
        acc_max = read_f32(i2c_target.server().map(), 0x18);
        pos_kp = read_f32(i2c_target.server().map(), 0x3C);
        pos_ki = read_f32(i2c_target.server().map(), 0x40);
        pos_kd = read_f32(i2c_target.server().map(), 0x44);
        limit_voltage = read_f32(i2c_target.server().map(), 0x48);
        zero_offset = read_f32(i2c_target.server().map(), 0x4C);

        // Apply PID gains each tick.
        act.pid_position.set_p(pos_kp);
        act.pid_position.set_i(pos_ki);
        act.pid_position.set_d(pos_kd);
        act.pid_position.set_limit(limit_voltage);
        act.motor.vel_limit = vel_max;
        driver.params_mut().voltage_limit = limit_voltage;
        voltage_limit = limit_voltage;
        }

        // Sync arm state register? written at telemetry.

        // ---- DIRECT_POS motor-direct loop (mode 2) -------------------------
        if armed && mode == 2 {
            // DIRECT_POS-style motor-direct P loop using table target (rad).
            let target = target_pos + zero_offset;
            let err = target - mot_pos;

            if err.abs() > peak_error {
                peak_error = err.abs();
                store_f32(&PEAK_ERROR_BITS, peak_error);
            }
            if settling {
                if err.abs() < 0.003 {
                    settle_timer += dt;
                    if settle_timer >= 0.1 && settle_ms == 0 {
                        settle_ms =
                            ((now - settle_start).as_micros() / 1000) as u32;
                        settling = false;
                        SETTLE_MS.store(settle_ms, Ordering::Relaxed);
                    }
                } else {
                    settle_timer = 0.0;
                }
            }
            // also disarm on stall: sustained position error while armed
            if err.abs() > STALL_LIMIT {
                stall_timer += dt;
                if stall_timer > 1.0 {
                    fault = 1;
                }
            } else {
                stall_timer = 0.0;
            }
            // slew the velocity setpoint toward the finite P output so the
            // velocity PI never sees a step: a large single-step target
            // saturates the integrator before the rotor can respond
            // (bring-up finding, see progress.md).
            let v_out = (pos_kp * err).clamp(-vel_max, vel_max);
            mot_vel_target +=
                (v_out - mot_vel_target).clamp(-vel_slew * dt, vel_slew * dt);
            store_f32(&APPLIED_VOLTAGE_BITS, applied_voltage);
        }

        // ---- TRAP_POS joint trajectory loop (mode 1) ----------------------
        if armed && mode == 1 {
            // The outer loop samples the output encoder synchronously inside
            // step_outer, runs the joint-space P controller, and projects the
            // joint velocity through the inverted Jacobian (v_mot =
            // v_joint / J, |J| floor + sign kept by Actuator1DOF), clamped to
            // the motor velocity limit before reaching move_motor().
            joint_target = target_pos + zero_offset;
            let _v_mot = act.step_outer(joint_target, &clock);
            mot_vel_target = act.motor.vel_target;

            let joint_err = joint_target - act.current_joint_angle;
            joint_err_deg = joint_err * 180.0 / PI;
            store_f32(&JOINT_POS_BITS, act.current_joint_angle);
            store_f32(&JOINT_ERR_BITS, joint_err);

            if joint_err.abs() > peak_error {
                peak_error = joint_err.abs();
                store_f32(&PEAK_ERROR_BITS, peak_error);
            }
            if settling {
                if joint_err.abs() < 0.02 {
                    settle_timer += dt;
                    if settle_timer >= 0.1 && settle_ms == 0 {
                        settle_ms = ((now - settle_start).as_micros() / 1000) as u32;
                        settling = false;
                        SETTLE_MS.store(settle_ms, Ordering::Relaxed);
                    }
                } else {
                    settle_timer = 0.0;
                }
            }
            // stall = sustained joint error (the joint encoder does not move)
            if joint_err.abs() > JOINT_STALL_LIMIT {
                stall_timer += dt;
                if stall_timer > 1.0 {
                    fault = 1;
                }
            } else {
                stall_timer = 0.0;
            }
            store_f32(&APPLIED_VOLTAGE_BITS, applied_voltage);
        }

        // ---- Sweep mode (mode 2, RTT `S` command only) --------------------
        if armed && mode == 2 && sweep_active {
            // Sweep mode applies a known field at a known electrical offset
            // from the *current* rotor position. We do NOT use the commutation
            // ea law here (that is exactly what we are testing); we synthesize
            // the field angle from the commanded offset and the live motor angle
            // so the field visibly moves relative to the rotor between steps.
            let ea_now = normalize_angle(
                (mot_raw - zero_angle) * sensor_dir * POLE_PAIRS,
            );
            // advance the sweep if the current step has been held long enough
            sweep[sweep_idx].since += (now - last).as_micros();
            if sweep[sweep_idx].since >= 500_000 {
                if sweep_idx + 1 < SWEEP_N {
                    sweep_idx += 1;
                    sweep[sweep_idx].since = 0;
                }
            }
            let step = sweep[sweep_idx];
            // Sweep field angle = current rotor field angle, plus the commanded
            // electrical offset. All arithmetic in the field (electrical) frame,
            // normalized into [-PI, PI).
            let sweep_ea = normalize_angle(ea_now + step.offset);
            // In sweep mode we apply a known field at a known offset; speed is
            // zero (no velocity loop), so the applied voltage is just the
            // signed magnitude requested, clamped to the limit.
            let u = step.voltage;
            applied_voltage = u.clamp(-voltage_limit, voltage_limit);
            // Torque direction: a positive sweep offset with positive dir means
            // the field leads the rotor; we use the same sign convention as
            // apply_vector so that ea is the field angle and the rotor responds
            // to the created torque.
            let ea = normalize_angle(sweep_ea);
            apply_vector(&mut driver, ea, applied_voltage);
        }

        // ---- Open-loop position mode (mode 5, RTT `O<deg>`) ---------------
        // Commanded rotor travel is integrated into an electrical angle;
        // the encoder is NOT in the loop, so this exercises the commutation
        // path independently of encoder/calibration errors.
        if ol_active && mode == 5 {
            ol_elapsed += dt;
            let sign = if ol_target >= 0.0 { 1.0 } else { -1.0 };
            let remaining = ol_target.abs() - ol_pos.abs();
            let align_done = ol_elapsed >= OL_ALIGN_S;
            let timed_out = ol_elapsed * 1000.0 >= OL_TIMEOUT_MS as f32;

            if !align_done {
                // Hold the field at a boosted voltage so the rotor aligns
                // against gearbox stiction before any travel is commanded.
                let v_align = (voltage_limit * OL_ALIGN_GAIN).min(VOLTAGE_RAIL);
                applied_voltage = v_align;
                apply_vector(&mut driver, ol_ea, v_align);
            } else if !ol_hold {
                if !ol_meas_start {
                    // Bracket the move with BOTH encoders, after alignment.
                    ol_start_mot = mot_pos;
                    act.output_sensor.update(&clock);
                    ol_start_out = Sensor::<TickClock>::get_angle(&act.output_sensor);
                    ol_meas_start = true;
                }
                // Trapezoidal ramp: accelerate to ol_speed, then brake so the
                // field stops at the commanded position.
                let stop_dist = ol_cur_vel * ol_cur_vel / (2.0 * ol_accel);
                let v_target = if remaining <= stop_dist { 0.0 } else { ol_speed };
                let dv = ol_accel * dt;
                ol_cur_vel = if v_target > ol_cur_vel {
                    (ol_cur_vel + dv).min(v_target)
                } else {
                    (ol_cur_vel - dv).max(v_target)
                };
                let step = (ol_cur_vel * dt).min(remaining.max(0.0));
                ol_pos += sign * step;
                if mot_vel.abs() > ol_peak_vel {
                    ol_peak_vel = mot_vel.abs();
                }
                // Decreasing electrical angle drives the rotor positive;
                // independent of the encoder sign convention.
                ol_ea = normalize_angle(ol_start_ea - POLE_PAIRS * ol_pos);
                applied_voltage = voltage_limit;
                apply_vector(&mut driver, ol_ea, applied_voltage);
                if remaining <= 1e-4 {
                    ol_hold = true;
                    ol_hold_elapsed = 0.0;
                }
            } else {
                // Soft stop: hold the final field while tapering to zero.
                ol_hold_elapsed += dt;
                let k = (1.0 - ol_hold_elapsed / OL_HOLD_S).clamp(0.0, 1.0);
                applied_voltage = voltage_limit * k;
                apply_vector(&mut driver, ol_ea, applied_voltage);
            }

            if (ol_hold && ol_hold_elapsed >= OL_HOLD_S) || timed_out {
                act.output_sensor.update(&clock);
                let end_out = Sensor::<TickClock>::get_angle(&act.output_sensor);
                let meas_mot = (mot_pos - ol_start_mot) * 180.0 / PI;
                let meas_out = (end_out - ol_start_out) * 180.0 / PI;
                rprintln!("[CMD] OL DONE cmd={}deg mot={}deg out={}deg t={}ms W={} A={} vpeak={}rad/s",
                          (ol_target * 180.0 / PI) as i32, meas_mot as i32,
                          meas_out as i32, (ol_elapsed * 1000.0) as u32,
                          ol_speed as i32, ol_accel as i32, ol_peak_vel);
                driver.set_pwm(0.0, 0.0, 0.0);
                driver.disable();
                ol_active = false;
                armed = false;
                mode = 0;
                applied_voltage = 0.0;
                RUN_STATE.store(0, Ordering::Relaxed);
                led_r.set_low();
                table_set!(0x02, &[0u8]);
            }
        }

        // ---- Closed-loop field servo (mode 6, RTT `p<deg>`) ---------------
        // Encoder position closes the loop; the actuator is the verified
        // open-loop field direction (decreasing ea -> positive rotor motion).
        if armed && mode == 6 {
            let err = target_pos - mot_pos;
            // PD position controller -> field rotation rate.
            let v_cmd = (pos_kp * err - kd_vel * mot_vel).clamp(-vel_max, vel_max);
            servo_ea = normalize_angle(servo_ea - POLE_PAIRS * v_cmd * dt);
            applied_voltage = voltage_limit;
            apply_vector(&mut driver, servo_ea, applied_voltage);

            // Stall watchdog: if the rotor does not move for 1.5 s while the
            // field is sweeping, the pole alignment is wrong. Abort, abort
            // the driver, re-calibrate, and let the demo retry the move.
            if (mot_pos - servo_last_pos).abs() < 0.01 {
                servo_stall += dt;
            } else {
                servo_stall = 0.0;
                servo_last_pos = mot_pos;
            }
            if servo_stall > 1.5 {
                driver.set_pwm(0.0, 0.0, 0.0);
                driver.disable();
                armed = false;
                mode = 0;
                applied_voltage = 0.0;
                RUN_STATE.store(0, Ordering::Relaxed);
                table_set!(0x02, &[0u8]);
                let (z, d) = calibrate(&mut driver, &mot_enc).await;
                zero_angle = z;
                sensor_dir = d;
                CALIB_DONE.store(if d != 0.0 { 1 } else { 0 }, Ordering::Relaxed);
                SENSOR_DIR.store(d as i32 as u32, Ordering::Relaxed);
                store_f32(&ZERO_ANGLE_BITS, z);
                servo_stall = 0.0;
                servo_last_pos = mot_pos;
                if demo_active {
                    demo_state = if demo_state == 1 { 2 } else { 4 };
                    demo_wait = 0.3;
                }
                rprintln!("[DEMO] stall -> recalibrated, retry");
            } else if err.abs() < 0.02 && mot_vel.abs() < 0.2 {
                driver.set_pwm(0.0, 0.0, 0.0);
                driver.disable();
                armed = false;
                mode = 0;
                applied_voltage = 0.0;
                RUN_STATE.store(0, Ordering::Relaxed);
                led_r.set_low();
                table_set!(0x02, &[0u8]);
                if demo_active {
                    if demo_state == 1 {
                        demo_state = 2;
                        demo_wait = 0.2;
                    } else if demo_state == 3 {
                        demo_state = 4;
                        demo_wait = 0.5;
                    }
                }
                rprintln!("[CMD] CL-SERVO DONE err={}mrad", (err * 1000.0) as i32);
            } else if (now - traj_start).as_secs() > 15 {
                driver.set_pwm(0.0, 0.0, 0.0);
                driver.disable();
                armed = false;
                mode = 0;
                applied_voltage = 0.0;
                RUN_STATE.store(0, Ordering::Relaxed);
                table_set!(0x02, &[0u8]);
                demo_active = false;
                demo_state = 0;
                demo_wait = 0.0;
                rprintln!("[CMD] CL-SERVO TIMEOUT");
            }
        }

        // ---- Output-shaft field servo (mode 7, RTT `j<deg>`) --------------
        // Closed loop on the output encoder; the motor field is commanded via
        // the verified open-loop direction with the inverted 275/13 gear.
        if armed && mode == 7 {
            act.output_sensor.update(&clock);
            let out_angle = Sensor::<TickClock>::get_angle(&act.output_sensor);
            let err = target_pos - out_angle;
            // v_mot = -gear_ratio * v_out (the planetary stage inverts).
            let v_out = (KP_JOINT * err).clamp(
                -vel_max / GEAR_RATIO,
                vel_max / GEAR_RATIO,
            );
            let v_mot = (-GEAR_RATIO * v_out).clamp(-vel_max, vel_max);
            out_servo_ea = normalize_angle(out_servo_ea - POLE_PAIRS * v_mot * dt);
            applied_voltage = voltage_limit;
            apply_vector(&mut driver, out_servo_ea, applied_voltage);

            if (out_angle - out_servo_last).abs() < 0.005 {
                out_servo_stall += dt;
            } else {
                out_servo_stall = 0.0;
                out_servo_last = out_angle;
            }
            if out_servo_stall > 2.0 || (now - out_servo_start).as_secs() > 20 {
                // Re-align and retry rather than freezing.
                driver.set_pwm(0.0, 0.0, 0.0);
                driver.disable();
                armed = false;
                mode = 0;
                applied_voltage = 0.0;
                RUN_STATE.store(0, Ordering::Relaxed);
                table_set!(0x02, &[0u8]);
                let (z, d) = calibrate(&mut driver, &mot_enc).await;
                zero_angle = z;
                sensor_dir = d;
                store_f32(&ZERO_ANGLE_BITS, z);
                servo_ea = normalize_angle((mot_raw - zero_angle) * sensor_dir * POLE_PAIRS);
                out_servo_ea = servo_ea;
                out_servo_stall = 0.0;
                out_servo_last = out_angle;
                rprintln!("[DEMO] out-servo realign, retry");
            } else if err.abs() < 0.01 {
                driver.set_pwm(0.0, 0.0, 0.0);
                driver.disable();
                armed = false;
                mode = 0;
                applied_voltage = 0.0;
                RUN_STATE.store(0, Ordering::Relaxed);
                table_set!(0x02, &[0u8]);
                if demo_active {
                    if demo_state == 1 { demo_state = 2; demo_wait = 0.2; }
                    else if demo_state == 3 { demo_state = 4; demo_wait = 0.5; }
                }
                rprintln!("[CMD] OUT-SERVO DONE err={}mrad", (err * 1000.0) as i32);
            }
        }

        // ---- Inner velocity loop + commutation -----------------------------
        // Runs for the motor bring-up (mode 1) and the joint servo (mode 3);
        // the transmission inversion lives in the outer loop's Jacobian, so
        // the field-path math is identical here for both modes.
        // Velocity mode: feed target velocity directly.
        if armed && mode == 3 {
            mot_vel_target = target_vel.clamp(-vel_max, vel_max);
        }
        if armed && (mode == 1 || mode == 2 || mode == 3) {
            // sensor_dir = -1 means a positive electrical rotation moves the
            // rotor *negative* in the encoder's own convention, so the loop
            // error is taken in the field's direction to keep it negative
            // feedback in both cases.
            let e = -sensor_dir * (mot_vel_target - mot_vel);
            // Acceleration damping from the filtered encoder velocity.
            let vel_dot = ((mot_vel - prev_mot_vel) / dt.max(1e-4)).clamp(-50.0, 50.0);
            let mut u_unsat = kp_vel * e + vel_integ - kd_vel * vel_dot;
            // Stiction breakaway: while the rotor is stalled and a velocity is
            // demanded, add a bounded voltage kick in the demanded direction.
            if e.abs() > 0.05 && mot_vel.abs() < 1.0 && v_boost > 0.0 {
                u_unsat += if e > 0.0 { v_boost } else { -v_boost };
            }
            let u = u_unsat.clamp(-voltage_limit, voltage_limit);
            // Conditional integration, with a tight integral clamp: the rotor
            // is stationary for a long time at breakaway, so an unbounded
            // integrator would wind up and cause a huge overshoot.
            let saturated_same = (u_unsat > voltage_limit && e > 0.0)
                || (u_unsat < -voltage_limit && e < 0.0);
            if !saturated_same {
                vel_integ = (vel_integ + ki_vel * e * dt_i)
                    .clamp(-0.5 * voltage_limit, 0.5 * voltage_limit);
            }
            prev_mot_vel = mot_vel;
            applied_voltage = u;

            let ea = normalize_angle((mot_raw - zero_angle) * sensor_dir * POLE_PAIRS + cl_offset);
            apply_vector(&mut driver, ea, u);

            // ---- Faults ---------------------------------------------------
            // Require sustained overspeed: a single encoder glitch/CRC spike
            // must not abort a perfectly good move.
            if mot_vel.abs() > OVERSPEED {
                overspeed_timer += dt;
                if overspeed_timer > 0.2 {
                    fault = 2;
                }
            } else {
                overspeed_timer = 0.0;
            }

            if fault != 0 {
                armed = false;
                mode = 0;
                sweep_active = false;
                vel_integ = 0.0;
                mot_vel_target = 0.0;
                applied_voltage = 0.0;
                #[cfg(not(feature = "uart"))]
                let _ = i2c_target.server_mut().map_mut().set(0x02, &[0u8]);
                driver.set_pwm(0.0, 0.0, 0.0);
                driver.disable();
                led_r.set_low();
                RUN_STATE.store(0, Ordering::Relaxed);
                FAULT_CODE.store(fault, Ordering::Relaxed);
                rprintln!("[FAULT {}] -> disarmed", fault);
            }
        } else if armed && mode == 4 {
            // TORQUE: voltage effort directly.
            applied_voltage = target_effort.clamp(-limit_voltage, limit_voltage);
            let ea = normalize_angle((mot_raw - zero_angle) * sensor_dir * POLE_PAIRS + cl_offset);
            apply_vector(&mut driver, ea, applied_voltage);
            // Require sustained overspeed: a single encoder glitch/CRC spike
            // must not abort a perfectly good move.
            if mot_vel.abs() > OVERSPEED {
                overspeed_timer += dt;
                if overspeed_timer > 0.2 {
                    fault = 2;
                }
            } else {
                overspeed_timer = 0.0;
            }
        } else if armed && mode == 2 {
            // sweep mode already applied; no velocity loop
        } else if !armed && applied_voltage != 0.0 {
            applied_voltage = 0.0;
            driver.set_pwm(0.0, 0.0, 0.0);
        }

            // ---- Trajectory recorder (while armed) ----------------------------
        if armed && traj_idx < TRAJ_N {
            // Sample every TRAJ_PERIOD_MS. Because the control loop ticks at
            // ~2 kHz (Timer::after_micros(500)), spacing samples this way keeps
            // the buffer from filling too fast and gives a readable time axis.
            if traj_next_ms == 0 {
                traj_next_ms = TRAJ_PERIOD_MS;
            }
            let t_ms = (now - traj_start).as_millis() as u32;
            if (t_ms as u64) >= traj_next_ms {
                let base = traj_idx * TRAJ_STRIDE;
                TRAJ[base].store(t_ms as u32, Ordering::Relaxed);
                TRAJ[base + 1].store(mot_pos.to_bits(), Ordering::Relaxed);
                TRAJ[base + 2].store(mot_vel.to_bits(), Ordering::Relaxed);
                TRAJ[base + 3].store(applied_voltage.to_bits(), Ordering::Relaxed);
                traj_idx += 1;
                TRAJ_LEN.store(traj_idx as u32, Ordering::Relaxed);
                traj_next_ms += TRAJ_PERIOD_MS;
            }
        }

        // ---- Telemetry (5 Hz) ---------------------------------------------
        if now.duration_since(last_telem).as_millis() >= 200 {
            last_telem = now;

            let mt = mot_enc.telemetry();
            MOT_CRC_OK.store(mt.crc_ok, Ordering::Relaxed);
            MOT_CRC_FAILED.store(mt.crc_failed, Ordering::Relaxed);

            store_f32(&MOT_POS_BITS, mot_pos);
            store_f32(&MOT_VEL_BITS, mot_vel);
            store_f32(&MOT_VEL_TARGET_BITS, mot_vel_target);
            store_f32(&APPLIED_VOLTAGE_BITS, applied_voltage);

            if mt.fault_flag {
                rprintln!("[WARN] encoder CRC exhausted");
                mot_enc.clear_fault();
            }

            // Output-encoder / joint telemetry (always emitted so the host
            // monitor can track the joint loop even before it is driven).
            act.output_sensor.update(&clock);
            let ot = act.output_sensor.telemetry();
            OUT_CRC_OK.store(ot.crc_ok, Ordering::Relaxed);
            OUT_CRC_FAILED.store(ot.crc_failed, Ordering::Relaxed);
            let out_angle = Sensor::<TickClock>::get_angle(&act.output_sensor);
            store_f32(&JOINT_POS_BITS, out_angle);
            // LED_B stays dark during normal demo operation

            let mot_d = (mot_pos * (180.0 / PI) * 10.0) as i32;
            let out_d = (out_angle * (180.0 / PI) * 10.0) as i32;
            let mot_rpm = (mot_vel * (30.0 / PI)) as i32;
            let u_mv = (applied_voltage * 1000.0) as i32;
            rprintln!(
                "mot={}.{}deg {}rpm | out={}.{}deg | u={}mV | st={}/{}",
                mot_d / 10,
                (mot_d % 10).abs(),
                mot_rpm,
                out_d / 10,
                (out_d % 10).abs(),
                u_mv,
                if armed { 1 } else { 0 },
                fault,
            );
        }

        // ---- Fill control-table telemetry each tick (I2C path only) ---------
        #[cfg(not(feature = "uart"))]
        {
            act.output_sensor.update(&clock);
            let out_pos = Sensor::<TickClock>::get_angle(&act.output_sensor);
            let out_vel = 0.0f32;
            let torque_iq = applied_voltage;
            let mut map = i2c_target.server_mut().map_mut();
            let _ = map.set(0x00, &[0x4Du8]);
            let _ = map.set(0x03, &[fault as u8]);
            let _ = map.set(0x04, &[armed as u8]);
            let _ = map.set(
                0x05,
                &[if sensor_dir != 0.0 { 1u8 } else { 0u8 }],
            );
            write_f32(&mut map, 0x20, out_pos);
            write_f32(&mut map, 0x24, out_vel);
            write_f32(&mut map, 0x28, torque_iq);
            write_f32(&mut map, 0x2C, mot_pos);
            write_f32(&mut map, 0x30, mot_vel);
            write_f32(&mut map, 0x34, SUPPLY_VOLTAGE);
            write_f32(&mut map, 0x38, 0.0);
        }

        LOOP_COUNT.fetch_add(1, Ordering::Relaxed);
        Timer::after_micros(500).await;
    }
}
