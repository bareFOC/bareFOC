//! # sensors::magnetic_i2c — I²C magnetic position sensor
//!
//! Ports
//! [`MagneticSensorI2C.h`](source/Arduino-FOC/src/sensors/MagneticSensorI2C.h)
//! and
//! [`MagneticSensorI2C.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorI2C.cpp)
//! from Arduino-FOC.
//!
//! `MagneticSensorI2C` reads an angle from a magnetic position sensor
//! over an `embedded-hal` v1.0 I²C bus. The transaction is a single
//! `write_read` call — equivalent to the upstream's
//! `beginTransmission` + `write` + `endTransmission` +
//! `requestFrom` + two `read` calls, but using the v1.0
//! transaction-with-repeated-start pattern.
//!
//! ## Bit extraction
//!
//! The configuration struct uses four mask/shift pairs to
//! extract a `bit_resolution`-bit value from the two bytes
//! returned by the sensor. The extraction formula is
//!
//! ```text
//! raw = (MSB & msb_mask) << msb_shift | (LSB & lsb_mask) >> lsb_shift
//! ```
//!
//! Users supply a [`MagneticSensorI2CConfig`] describing the target
//! sensor's I²C address, angle register, bit resolution, and bit masks/shifts.
//!
//! ## I²C ownership and `RefCell`
//!
//! The struct owns the I²C instance and stores it in a
//! `RefCell`. The [`Sensor::get_sensor_angle`] trait method is
//! `&self`-only (so the FOC hot loop can call it through a
//! shared reference), but reading the I²C requires `&mut`.
//! The `RefCell` provides the interior mutability bridge;
//! its runtime borrow check has a fixed `O(1)` cost that
//! disappears next to the I²C transaction itself.

#![allow(non_camel_case_types)]

use crate::common::time::Clock;
use crate::traits::sensor::{Sensor, SensorBase};
use core::cell::{Cell, RefCell};
use embedded_hal::i2c;

/// I²C configuration for a magnetic position sensor.
///
/// Ports the upstream `MagneticSensorI2CConfig_s` struct
/// ([`MagneticSensorI2C.h`](source/Arduino-FOC/src/sensors/MagneticSensorI2C.h)
/// lines 10–18).
///
/// The seven fields describe the I²C address, the register
/// holding the angle, and how to combine the two returned
/// bytes into a single `bit_resolution`-bit value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MagneticSensorI2CConfig {
    /// 7-bit I²C address (right-aligned, in `0x00..=0x7F`).
    pub chip_address: u8,
    /// Sensor bit resolution (e.g. 12 or 14 bits).
    pub bit_resolution: u8,
    /// Address of the angle register (the byte sent in the
    /// `write` half of the transaction).
    pub angle_register: u8,
    /// Mask applied to the MSB byte. Typically
    /// `(1 << bits_used_in_msb) - 1`.
    pub msb_mask: u8,
    /// Left-shift applied to the masked MSB.
    pub msb_shift: u8,
    /// Mask applied to the LSB byte.
    pub lsb_mask: u8,
    /// Right-shift applied to the masked LSB.
    pub lsb_shift: u8,
}

/// I²C magnetic position sensor.
///
/// Ports the upstream `MagneticSensorI2C` class
/// ([`MagneticSensorI2C.h`](source/Arduino-FOC/src/sensors/MagneticSensorI2C.h)).
///
/// The struct is generic over the [`Clock`] type used by the
/// base-class `init()` and over the I²C bus type. The I²C
/// instance is owned by the struct (stored in a `RefCell`
/// for interior mutability through `&self`).
#[derive(Debug)]
pub struct MagneticSensorI2C<C: Clock, I2C: i2c::I2c> {
    base: SensorBase,

    /// Maximum range of the magnetic sensor. Computed as
    /// `1 << bit_resolution` (matches the upstream
    /// `_powtwo(bit_resolution)`).
    pub cpr: f32,

    /// Active configuration (address, register, masks, shifts).
    pub conf: MagneticSensorI2CConfig,

    /// I²C bus instance. The user must initialise the
    /// peripheral externally before constructing the sensor.
    i2c: RefCell<I2C>,

    /// Last I²C error code. `0` if the last transfer
    /// succeeded, `1` if it failed. Mirrors the upstream
    /// `uint8_t currWireError` field.
    pub curr_wire_error: Cell<u8>,

    _clock: core::marker::PhantomData<C>,
}

impl<C: Clock, I2C: i2c::I2c> MagneticSensorI2C<C, I2C> {
    /// Construct a new sensor from an I²C peripheral and a
    /// configuration.
    ///
    /// The user is responsible for initialising the I²C
    /// peripheral (e.g. `i2c.begin(...)`) before passing it
    /// in. The FOC-rs port does not call any I²C-begin method
    /// of its own — the upstream `MagneticSensorI2C::init`
    /// called `wire->begin()`, but the v1.0 `embedded-hal`
    /// I²C trait does not have an equivalent; the user does
    /// it once for the bus before constructing the sensor.
    ///
    /// Mirrors the upstream
    /// `MagneticSensorI2C::MagneticSensorI2C(MagneticSensorI2CConfig_s config)`
    /// constructor
    /// ([`MagneticSensorI2C.cpp`](source/Arduino-FOC/src/sensors/MagneticSensorI2C.cpp)
    /// lines 63–67).
    pub fn new(i2c: I2C, conf: MagneticSensorI2CConfig) -> Self {
        let cpr = (1u32 << conf.bit_resolution) as f32;
        Self {
            base: SensorBase::new(),
            cpr,
            conf,
            i2c: RefCell::new(i2c),
            curr_wire_error: Cell::new(0),
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

    /// Read the sensor register and return the raw
    /// `bit_resolution`-bit value. Returns `Err` if the I²C
    /// transfer fails.
    pub fn get_raw_count(&self) -> Result<i32, I2C::Error> {
        let mut buf = [0u8; 2];
        let res = self.i2c.borrow_mut().write_read(
            self.conf.chip_address,
            core::slice::from_ref(&self.conf.angle_register),
            &mut buf,
        );
        match res {
            Ok(()) => {
                self.curr_wire_error.set(0);
                let raw_msb = (buf[0] & self.conf.msb_mask) as i32;
                let raw_lsb = (buf[1] & self.conf.lsb_mask) as i32;
                Ok((raw_msb << self.conf.msb_shift) | (raw_lsb >> self.conf.lsb_shift))
            }
            Err(e) => {
                self.curr_wire_error.set(1);
                Err(e)
            }
        }
    }
}

impl<C: Clock, I2C: i2c::I2c> Sensor<C> for MagneticSensorI2C<C, I2C> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    /// Read the angle register and return the shaft angle in
    /// `[0, 2π)`. On I²C error, returns `0.0` and sets
    /// `curr_wire_error` to `1` — the FOC-rs port is
    /// slightly safer than the upstream C++ which would
    /// return whatever stale data was in the buffer.
    ///
    /// The upstream uses the formula
    /// `(raw / cpr) × 2π` (matches
    /// [`crate::sensors::magnetic_analog::MagneticSensorAnalog::get_sensor_angle`]).
    fn get_sensor_angle(&self) -> f32 {
        match self.get_raw_count() {
            Ok(raw) => (raw as f32) * crate::math::_2PI / self.cpr,
            Err(_) => 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::time::mock::MockClock;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    // `embedded-hal-mock` is a `std` crate (it uses Arc, Mutex
    // and Vec internally), so the test module is implicitly
    // `std`-flavoured. Re-introduce the `vec!` macro and
    // the `Vec` type here.
    extern crate std;
    use std::vec;

    // -----------------------------------------------------------------
    // Mock configs for testing
    // -----------------------------------------------------------------
    fn mock_12bit_i2c_config() -> MagneticSensorI2CConfig {
        MagneticSensorI2CConfig {
            chip_address: 0x36,
            bit_resolution: 12,
            angle_register: 0x0C,
            msb_mask: 0x0F,
            msb_shift: 8,
            lsb_mask: 0xFF,
            lsb_shift: 0,
        }
    }

    fn mock_14bit_i2c_config() -> MagneticSensorI2CConfig {
        MagneticSensorI2CConfig {
            chip_address: 0x40,
            bit_resolution: 14,
            angle_register: 0xFE,
            msb_mask: 0xFF,
            msb_shift: 6,
            lsb_mask: 0x3F,
            lsb_shift: 0,
        }
    }

    fn mock_14bit_shifted_i2c_config() -> MagneticSensorI2CConfig {
        MagneticSensorI2CConfig {
            chip_address: 0x06,
            bit_resolution: 14,
            angle_register: 0x03,
            msb_mask: 0xFF,
            msb_shift: 6,
            lsb_mask: 0xFC,
            lsb_shift: 2,
        }
    }

    #[test]
    fn mock_config_fields() {
        let c = mock_12bit_i2c_config();
        assert_eq!(c.chip_address, 0x36);
        assert_eq!(c.bit_resolution, 12);
        assert_eq!(c.angle_register, 0x0C);
        assert_eq!(c.msb_mask, 0x0F);
        assert_eq!(c.msb_shift, 8);
        assert_eq!(c.lsb_mask, 0xFF);
        assert_eq!(c.lsb_shift, 0);

        let c14 = mock_14bit_i2c_config();
        assert_eq!(c14.bit_resolution, 14);
        assert_eq!(c14.angle_register, 0xFE);

        let c_sh = mock_14bit_shifted_i2c_config();
        assert_eq!(c_sh.bit_resolution, 14);
        assert_eq!(c_sh.lsb_shift, 2);
    }

    // -----------------------------------------------------------------
    // cpr = 1 << bit_resolution
    // -----------------------------------------------------------------
    #[test]
    fn cpr_is_powtwo_of_bit_resolution() {
        let mut i2c = I2cMock::new(&[]);
        let i2c_for_sensor = i2c.clone();
        let s = MagneticSensorI2C::<MockClock, _>::new(i2c_for_sensor, mock_12bit_i2c_config());
        assert_eq!(s.cpr, 4096.0);
        drop(s);
        i2c.done();

        let mut i2c = I2cMock::new(&[]);
        let i2c_for_sensor = i2c.clone();
        let s = MagneticSensorI2C::<MockClock, _>::new(i2c_for_sensor, mock_14bit_i2c_config());
        assert_eq!(s.cpr, 16384.0);
        drop(s);
        i2c.done();

        let mut i2c = I2cMock::new(&[]);
        let i2c_for_sensor = i2c.clone();
        let s = MagneticSensorI2C::<MockClock, _>::new(i2c_for_sensor, mock_14bit_shifted_i2c_config());
        assert_eq!(s.cpr, 16384.0);
        drop(s);
        i2c.done();
    }

    // -----------------------------------------------------------------
    // I2C transaction
    // -----------------------------------------------------------------
    #[test]
    fn get_angle_uses_embedded_hal_mock() {
        // 12-bit: read 0x0C, get back [0x12, 0x34].
        // raw = (0x12 & 0x0F) << 8 | (0x34 & 0xFF) >> 0
        //     = 0x02 << 8 | 0x34
        //     = 0x0234 = 564
        // angle = 564 / 4096 * 2π ≈ 0.865 rad
        let expectations = [I2cTransaction::write_read(
            0x36,
            vec![0x0C],
            vec![0x12, 0x34],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let i2c_for_sensor = i2c.clone();
        let s = MagneticSensorI2C::<MockClock, _>::new(i2c_for_sensor, mock_12bit_i2c_config());

        let angle = s.get_sensor_angle();
        let expected = 564.0_f32 * core::f32::consts::TAU / 4096.0;
        assert!((angle - expected).abs() < 1e-4);
        assert_eq!(s.curr_wire_error.get(), 0);

        drop(s);
        i2c.done();
    }

    #[test]
    fn get_angle_with_14bit_uses_six_bit_shift() {
        // 14-bit: msb_mask=0xFF, msb_shift=6, lsb_mask=0x3F, lsb_shift=0
        // raw = (0xFF & 0xFF) << 6 | (0xFC & 0x3F) >> 0
        //     = 0xFF << 6 | 0x3C
        //     = 16320 | 60 = 16380
        // angle = 16380 / 16384 * 2π ≈ 6.279 rad
        let expectations = [I2cTransaction::write_read(
            0x40,
            vec![0xFE],
            vec![0xFF, 0xFC],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let i2c_for_sensor = i2c.clone();
        let s = MagneticSensorI2C::<MockClock, _>::new(i2c_for_sensor, mock_14bit_i2c_config());

        let angle = s.get_sensor_angle();
        let expected = 16380.0_f32 * core::f32::consts::TAU / 16384.0;
        assert!((angle - expected).abs() < 1e-3);

        drop(s);
        i2c.done();
    }

    #[test]
    fn get_angle_with_14bit_uses_two_bit_shift() {
        // 14-bit shifted: msb_mask=0xFF, msb_shift=6, lsb_mask=0xFC, lsb_shift=2
        // raw = (0xFF & 0xFF) << 6 | (0xFC & 0xFC) >> 2
        //     = 0xFF << 6 | 0x3F
        //     = 16320 | 63 = 16383
        // angle = 16383 / 16384 * 2π ≈ 6.282 rad
        let expectations = [I2cTransaction::write_read(
            0x06,
            vec![0x03],
            vec![0xFF, 0xFC],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let i2c_for_sensor = i2c.clone();
        let s = MagneticSensorI2C::<MockClock, _>::new(i2c_for_sensor, mock_14bit_shifted_i2c_config());

        let angle = s.get_sensor_angle();
        let expected = 16383.0_f32 * core::f32::consts::TAU / 16384.0;
        assert!((angle - expected).abs() < 1e-3);

        drop(s);
        i2c.done();
    }

    // -----------------------------------------------------------------
    // LSB-aligned mask extraction
    // -----------------------------------------------------------------
    #[test]
    fn lsb_aligned_mask_extraction() {
        // 12-bit sensor: raw=1024 (1/4 turn)
        // angle = 1024/4096 * 2π = π/2
        let expectations = [I2cTransaction::write_read(
            0x36,
            vec![0x0C],
            vec![0x04, 0x00],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let i2c_for_sensor = i2c.clone();
        let s = MagneticSensorI2C::<MockClock, _>::new(i2c_for_sensor, mock_12bit_i2c_config());
        let angle = s.get_sensor_angle();
        assert!((angle - core::f32::consts::FRAC_PI_2).abs() < 1e-4);

        drop(s);
        i2c.done();
    }

    // -----------------------------------------------------------------
    // I2C error handling
    // -----------------------------------------------------------------
    #[test]
    fn get_angle_records_error_on_i2c_failure() {
        use embedded_hal::i2c::ErrorKind;
        let expectations = [I2cTransaction::write_read(
            0x36,
            vec![0x0C],
            vec![0x00, 0x00],
        )
        .with_error(ErrorKind::Other)];
        let mut i2c = I2cMock::new(&expectations);
        let i2c_for_sensor = i2c.clone();
        let s = MagneticSensorI2C::<MockClock, _>::new(i2c_for_sensor, mock_12bit_i2c_config());

        assert_eq!(s.get_sensor_angle(), 0.0);
        assert_eq!(s.curr_wire_error.get(), 1);

        drop(s);
        i2c.done();
    }

    // -----------------------------------------------------------------
    // Constructor + init
    // -----------------------------------------------------------------
    #[test]
    fn init_seeds_base_state() {
        let clock = MockClock::new();
        clock.set_us(1_000);

        let expectations = [
            I2cTransaction::write_read(0x36, vec![0x0C], vec![0x04, 0x00]),
            I2cTransaction::write_read(0x36, vec![0x0C], vec![0x04, 0x00]),
            I2cTransaction::write_read(0x36, vec![0x0C], vec![0x04, 0x00]),
            I2cTransaction::write_read(0x36, vec![0x0C], vec![0x04, 0x00]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let i2c_for_sensor = i2c.clone();
        let mut s = MagneticSensorI2C::<MockClock, _>::new(i2c_for_sensor, mock_12bit_i2c_config());
        s.init(&clock);
        // After init, angle_prev = 1024/4096*2π = π/2.
        assert!((s.base().angle_prev - core::f32::consts::FRAC_PI_2).abs() < 1e-4);
        assert_eq!(s.base().angle_prev_ts, 1_000);

        drop(s);
        i2c.done();
    }
}
