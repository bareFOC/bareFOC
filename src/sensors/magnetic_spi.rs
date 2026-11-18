//! # sensors::magnetic_spi — SPI magnetic position sensor
//!
//! Ports
//! [`MagneticSensorSPI.h`](source/Arduino-FOC/src/sensors/MagneticSensorSPI.h)
//! and
//! [`MagneticSensorSPI.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorSPI.cpp)
//! from Arduino-FOC.
//!
//! `MagneticSensorSPI` reads an angle from a magnetic position sensor
//! over an `embedded-hal` v1.0 SPI device. The transaction is a
//! 16-bit command write followed by a 16-bit response read,
//! matching the upstream `transfer16` pattern.
//!
//! ## Command frame
//!
//! The upstream builds a 16-bit command word from the angle
//! register, the R/W bit, and an even-parity bit. The
//! extraction logic after the read applies a right-shift and
//! a `bit_resolution`-bit mask to the response, stripping the
//! parity and error-flag bits. The FOC-rs port implements
//! both halves identically.
//!
//! Users supply a [`MagneticSensorSPIConfig`] specifying the SPI mode,
//! clock speed, bit resolution, angle register, data start bit,
//! and R/W / parity bit positions for their target sensor IC.
//!
//! ## SPI mode and clock
//!
//! The upstream stores `spi_mode` and `clock_speed` and
//! applies them per-transaction through Arduino's
//! `SPISettings`. In v1.0 `embedded-hal` the SPI mode and
//! clock are baked into the `SpiDevice` implementation at
//! construction time; the FOC-rs port retains the
//! `spi_mode` and `clock_speed` fields for documentation
//! and tests but does not pass them to the bus. The user
//! configures the mode/clock when they construct their
//! `SpiDevice`.

#![allow(non_camel_case_types)]

use crate::common::time::Clock;
use crate::traits::sensor::{Sensor, SensorBase};
use core::cell::RefCell;
use embedded_hal::spi;

/// Default angle register used when the constructor receives
/// `angle_register == 0`. Mirrors the upstream
/// `#define DEF_ANGLE_REGISTER 0x3FFF`.
pub const DEF_ANGLE_REGISTER: u16 = 0x3FFF;

/// SPI configuration for a magnetic position sensor.
///
/// Ports the upstream `MagneticSensorSPIConfig_s` struct
/// ([`MagneticSensorSPI.h`](source/Arduino-FOC/src/sensors/MagneticSensorSPI.h)
/// lines 13–21).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MagneticSensorSPIConfig {
    /// SPI mode (0..3). Stored as a `u8` to match the
    /// upstream `int`. The FOC-rs port does not pass this to
    /// the bus; the user configures the mode in their
    /// `SpiDevice` impl.
    pub spi_mode: u8,
    /// SPI clock speed in Hz. Stored for documentation; the
    /// user configures the clock in their `SpiDevice` impl.
    pub clock_speed: u32,
    /// Sensor bit resolution (e.g. 14 bits).
    pub bit_resolution: u8,
    /// Default angle register. `0` means "use
    /// [`DEF_ANGLE_REGISTER`]" (matches the upstream ternary
    /// in the constructor).
    pub angle_register: u16,
    /// Bit position of the first data bit in the response
    /// (e.g. 13 or 15).
    pub data_start_bit: u8,
    /// Bit position of the R/W flag in the command word
    /// (`0` disables the flag).
    pub command_rw_bit: u8,
    /// Bit position of the parity flag in the command word
    /// (`0` disables parity).
    pub command_parity_bit: u8,
}

// ---------------------------------------------------------------------------
// MagneticSensorSPI
// ---------------------------------------------------------------------------

/// SPI magnetic position sensor.
///
/// Ports the upstream `MagneticSensorSPI` class
/// ([`MagneticSensorSPI.h`](source/Arduino-FOC/src/sensors/MagneticSensorSPI.h)).
///
/// The struct is generic over the [`Clock`] type and over the
/// SPI device type. The SPI device instance is owned by the
/// struct (stored in a `RefCell` for interior mutability
/// through `&self`). The 16-bit word size matches the
/// upstream `transfer16` API.
#[derive(Debug)]
pub struct MagneticSensorSPI<C: Clock, SPI: spi::SpiDevice<u16>> {
    base: SensorBase,

    /// Maximum range of the magnetic sensor. Computed as
    /// `1 << bit_resolution` (matches the upstream
    /// `_powtwo(bit_resolution)`).
    pub cpr: f32,

    /// Active configuration.
    pub conf: MagneticSensorSPIConfig,

    /// Active angle register. `0` is replaced with
    /// [`DEF_ANGLE_REGISTER`] on construction.
    pub angle_register: u16,

    /// SPI device instance.
    spi: RefCell<SPI>,

    _clock: core::marker::PhantomData<C>,
}

impl<C: Clock, SPI: spi::SpiDevice<u16>> MagneticSensorSPI<C, SPI> {
    /// Construct a new sensor from an SPI device and a
    /// configuration. Mirrors the upstream
    /// `MagneticSensorSPI::MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs, long _clock_speed)`
    /// constructor
    /// ([`MagneticSensorSPI.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorSPI.cpp)
    /// lines 69–84).
    pub fn new(spi: SPI, conf: MagneticSensorSPIConfig) -> Self {
        let cpr = (1u32 << conf.bit_resolution) as f32;
        let angle_register = if conf.angle_register == 0 {
            DEF_ANGLE_REGISTER
        } else {
            conf.angle_register
        };
        Self {
            base: SensorBase::new(),
            cpr,
            conf,
            angle_register,
            spi: RefCell::new(spi),
            _clock: core::marker::PhantomData,
        }
    }

    /// Seed the base state with two `get_sensor_angle()`
    /// reads. The body of the upstream `Sensor::init` default
    /// method is inlined here for the same reason as
    /// [`crate::sensors::generic::GenericSensor::init`].
    pub fn init(&mut self, clock: &C) {
        let _ = self.get_sensor_angle();
        let a = self.get_sensor_angle();
        let now = clock.now_us();
        {
            let base = self.base_mut();
            base.vel_angle_prev = a;
            base.vel_angle_prev_ts = now;
        }
        let _ = self.get_sensor_angle();
        let a2 = self.get_sensor_angle();
        let now2 = clock.now_us();
        {
            let base = self.base_mut();
            base.angle_prev = a2;
            base.angle_prev_ts = now2;
        }
    }

    /// Build the 16-bit command word for the active angle
    /// register. Applies the R/W bit and the even-parity
    /// bit if their positions are non-zero.
    fn build_command(&self) -> u16 {
        let mut command = self.angle_register;
        if self.conf.command_rw_bit > 0 {
            command |= 1 << self.conf.command_rw_bit;
        }
        if self.conf.command_parity_bit > 0 {
            let parity = calc_even_parity(command);
            command |= parity << self.conf.command_parity_bit;
        }
        command
    }

    /// Issue the two 16-bit SPI transactions (write command,
    /// read response) and return the masked `bit_resolution`-
    /// bit value.
    pub fn get_raw_count(&self) -> Result<u16, SPI::Error> {
        let command = self.build_command();

        // Transaction 1: write the command.
        self.spi
            .borrow_mut()
            .transaction(&mut [spi::Operation::Write(&[command])])?;

        // Transaction 2: read the response. The upstream adds
        // a 1 µs delay between the two transactions
        // (`delayMicroseconds(1)`) — the FOC-rs port does
        // not enforce this; the user's `SpiDevice` impl
        // should add the CS-to-clock delay if the sensor
        // requires it.
        let mut buf = [0u16; 1];
        self.spi
            .borrow_mut()
            .transaction(&mut [spi::Operation::Read(&mut buf)])?;
        let response = buf[0];

        // Upstream shift:
        //   `register_value >> (1 + data_start_bit - bit_resolution)`
        // then mask with `(0xFFFF >> (16 - bit_resolution))`.
        let shift = 1 + self.conf.data_start_bit - self.conf.bit_resolution;
        let data_mask = 0xFFFF_u16 >> (16 - self.conf.bit_resolution);
        Ok((response >> shift) & data_mask)
    }
}

impl<C: Clock, SPI: spi::SpiDevice<u16>> Sensor<C> for MagneticSensorSPI<C, SPI> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    /// Read the angle register over SPI and return the shaft
    /// angle in `[0, 2π)`. On SPI error, returns `0.0` (the
    /// FOC-rs port is slightly safer than the upstream C++
    /// which would return a stale value).
    fn get_sensor_angle(&self) -> f32 {
        match self.get_raw_count() {
            Ok(raw) => (raw as f32) * crate::math::_2PI / self.cpr,
            Err(_) => 0.0,
        }
    }
}

/// Compute the even-parity bit of a 16-bit value. Mirrors
/// `MagneticSensorSPI::spiCalcEvenParity`
/// ([`MagneticSensorSPI.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorSPI.cpp)
/// lines 116–126).
const fn calc_even_parity(mut value: u16) -> u16 {
    let mut count = 0u8;
    let mut i = 0;
    while i < 16 {
        if value & 1 != 0 {
            count += 1;
        }
        value >>= 1;
        i += 1;
    }
    (count & 1) as u16
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::time::mock::MockClock;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

    extern crate std;
    use std::vec;

    // -----------------------------------------------------------------
    // Mock configs for testing
    // -----------------------------------------------------------------
    fn mock_14bit_spi_config() -> MagneticSensorSPIConfig {
        MagneticSensorSPIConfig {
            spi_mode: 1,
            clock_speed: 1_000_000,
            bit_resolution: 14,
            angle_register: 0x3FFF,
            data_start_bit: 13,
            command_rw_bit: 14,
            command_parity_bit: 15,
        }
    }

    fn mock_14bit_spi_no_parity_config() -> MagneticSensorSPIConfig {
        MagneticSensorSPIConfig {
            spi_mode: 0,
            clock_speed: 1_000_000,
            bit_resolution: 14,
            angle_register: 0x0000,
            data_start_bit: 15,
            command_rw_bit: 0,
            command_parity_bit: 0,
        }
    }

    #[test]
    fn mock_config_fields() {
        let c = mock_14bit_spi_config();
        assert_eq!(c.spi_mode, 1);
        assert_eq!(c.clock_speed, 1_000_000);
        assert_eq!(c.bit_resolution, 14);
        assert_eq!(c.angle_register, 0x3FFF);
        assert_eq!(c.data_start_bit, 13);
        assert_eq!(c.command_rw_bit, 14);
        assert_eq!(c.command_parity_bit, 15);

        let c_np = mock_14bit_spi_no_parity_config();
        assert_eq!(c_np.spi_mode, 0);
        assert_eq!(c_np.command_parity_bit, 0);
    }

    #[test]
    fn def_angle_register_is_0x3fff() {
        assert_eq!(DEF_ANGLE_REGISTER, 0x3FFF);
    }

    // -----------------------------------------------------------------
    // cpr = 1 << bit_resolution
    // -----------------------------------------------------------------
    #[test]
    fn cpr_is_powtwo_of_bit_resolution() {
        let mut spi = SpiMock::new(&[]);
        let spi_for_sensor = spi.clone();
        let s = MagneticSensorSPI::<MockClock, _>::new(spi_for_sensor, mock_14bit_spi_config());
        assert_eq!(s.cpr, 16384.0);
        drop(s);
        spi.done();

        let mut spi = SpiMock::new(&[]);
        let spi_for_sensor = spi.clone();
        let s = MagneticSensorSPI::<MockClock, _>::new(spi_for_sensor, mock_14bit_spi_no_parity_config());
        assert_eq!(s.cpr, 16384.0);
        drop(s);
        spi.done();
    }

    // -----------------------------------------------------------------
    // Default angle register
    // -----------------------------------------------------------------
    #[test]
    fn angle_register_zero_is_replaced_with_default() {
        let mut spi = SpiMock::new(&[]);
        let spi_for_sensor = spi.clone();
        let s = MagneticSensorSPI::<MockClock, _>::new(spi_for_sensor, mock_14bit_spi_no_parity_config());
        // mock_14bit_spi_no_parity has angle_register = 0 in the config, so it
        // should be replaced with DEF_ANGLE_REGISTER.
        assert_eq!(s.angle_register, DEF_ANGLE_REGISTER);
        drop(s);
        spi.done();
    }

    #[test]
    fn angle_register_nonzero_is_preserved() {
        let mut spi = SpiMock::new(&[]);
        let spi_for_sensor = spi.clone();
        let s = MagneticSensorSPI::<MockClock, _>::new(spi_for_sensor, mock_14bit_spi_config());
        // mock_14bit_spi_config has angle_register = 0x3FFF, not zero, so it
        // should be preserved.
        assert_eq!(s.angle_register, 0x3FFF);
        drop(s);
        spi.done();
    }

    // -----------------------------------------------------------------
    // Parity calculation
    // -----------------------------------------------------------------
    #[test]
    fn parity_of_zero_is_zero() {
        assert_eq!(calc_even_parity(0x0000), 0);
    }

    #[test]
    fn parity_of_ones_is_zero() {
        assert_eq!(calc_even_parity(0xFFFF), 0); // 16 ones = even
    }

    #[test]
    fn parity_of_single_bit_is_one() {
        assert_eq!(calc_even_parity(0x0001), 1); // 1 one = odd
        assert_eq!(calc_even_parity(0x8000), 1); // 1 one = odd
    }

    // -----------------------------------------------------------------
    // SPI transaction
    // -----------------------------------------------------------------
    #[test]
    fn get_angle_uses_embedded_hal_mock_14bit() {
        // 14-bit with parity: command = 0x3FFF | (1 << 14) | (parity << 15)
        // Parity of 0x3FFF | (1 << 14) = 0x7FFF:
        //   0x7FFF = 0111_1111_1111_1111 → 15 ones = odd
        //   parity = 1
        // command = 0x7FFF | (1 << 15) = 0xFFFF
        //
        // Response: 0xABCD (example). shift = 0, mask = 0x3FFF.
        // raw = 0xABCD & 0x3FFF = 0x2BCD = 11213
        // angle = 11213 / 16384 * 2π ≈ 4.30 rad
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0xFFFF_u16]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::read_vec(vec![0xABCD_u16]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let spi_for_sensor = spi.clone();
        let s = MagneticSensorSPI::<MockClock, _>::new(spi_for_sensor, mock_14bit_spi_config());

        let angle = s.get_sensor_angle();
        let expected = 11213.0_f32 * core::f32::consts::TAU / 16384.0;
        assert!((angle - expected).abs() < 1e-3);

        drop(s);
        spi.done();
    }

    #[test]
    fn get_angle_uses_shifted_read() {
        // mock_14bit_spi_no_parity: command_rw_bit=0, command_parity_bit=0.
        // command = angle_register = 0x3FFF (after default
        // substitution).
        // Response: 0xC000. shift = 1+15-14 = 2, mask = 0x3FFF.
        // raw = (0xC000 >> 2) & 0x3FFF = 0x3000 & 0x3FFF = 0x3000 = 12288
        // angle = 12288 / 16384 * 2π ≈ 4.712 rad
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x3FFF_u16]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::read_vec(vec![0xC000_u16]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let spi_for_sensor = spi.clone();
        let s = MagneticSensorSPI::<MockClock, _>::new(spi_for_sensor, mock_14bit_spi_no_parity_config());

        let angle = s.get_sensor_angle();
        let expected = 12288.0_f32 * core::f32::consts::TAU / 16384.0;
        assert!((angle - expected).abs() < 1e-3);

        drop(s);
        spi.done();
    }

    // -----------------------------------------------------------------
    // 14-bit mask extraction
    // -----------------------------------------------------------------
    #[test]
    fn msb_mask_extraction_14bit() {
        // Verify the data_mask is exactly 14 bits wide.
        // max raw = (0x3FFF & 0x3FFF) = 16383
        // angle = 16383 / 16384 * 2π ≈ 6.282 rad (almost 2π)
        let expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0xFFFF_u16]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::read_vec(vec![0xFFFF_u16]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let spi_for_sensor = spi.clone();
        let s = MagneticSensorSPI::<MockClock, _>::new(spi_for_sensor, mock_14bit_spi_config());

        let angle = s.get_sensor_angle();
        // 16383 / 16384 * 2π = 2π * (1 - 1/16384)
        let expected = (1.0_f32 - 1.0 / 16384.0) * core::f32::consts::TAU;
        assert!((angle - expected).abs() < 1e-3);

        drop(s);
        spi.done();
    }

    // -----------------------------------------------------------------
    // Constructor + init
    // -----------------------------------------------------------------
    #[test]
    fn init_seeds_base_state() {
        let clock = MockClock::new();
        clock.set_us(1_000);

        let expectations = [
            // get_sensor_angle #1 (discarded)
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0xFFFF_u16]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::read_vec(vec![0x1234_u16]),
            SpiTransaction::transaction_end(),
            // get_sensor_angle #2 → vel_angle_prev
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0xFFFF_u16]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::read_vec(vec![0x1234_u16]),
            SpiTransaction::transaction_end(),
            // get_sensor_angle #3 (discarded)
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0xFFFF_u16]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::read_vec(vec![0x1234_u16]),
            SpiTransaction::transaction_end(),
            // get_sensor_angle #4 → angle_prev
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0xFFFF_u16]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::read_vec(vec![0x1234_u16]),
            SpiTransaction::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let spi_for_sensor = spi.clone();
        let mut s = MagneticSensorSPI::<MockClock, _>::new(spi_for_sensor, mock_14bit_spi_config());
        s.init(&clock);
        // 0x1234 = 4660. angle = 4660/16384*2π ≈ 1.787 rad
        assert!((s.base().angle_prev - 1.787).abs() < 1e-3);
        assert_eq!(s.base().angle_prev_ts, 1_000);

        drop(s);
        spi.done();
    }
}
