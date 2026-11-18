//! # sensors — concrete sensor implementations
//!
//! Ports the seven sensor classes from
//! `source/Arduino-FOC/src/sensors/` to the FOC-rs [`Sensor<C>`]
//! trait.
//!
//! | Module | Upstream | Hardware | Peripherals |
//! |---|---|---|---|
//! | [`encoder`] | `Encoder` | Quadrature optical / magnetic encoder | EXTI interrupts on 2 or 3 pins |
//! | [`hall`] | `HallSensor` | BLDC Hall sensors (3 digital pins) | EXTI interrupts on 3 pins |
//! | [`generic`] | `GenericSensor` | User-supplied callback | None (function pointers) |
//! | [`magnetic_analog`] | `MagneticSensorAnalog` | Linear analog Hall sensor | `Adc` + 1 pin |
//! | [`magnetic_i2c`] | `MagneticSensorI2C` | Generic I²C magnetic sensor (AS5600, AS5048B, MT6701, etc.) | `I2c` (7-bit) |
//! | [`magnetic_pwm`] | `MagneticSensorPWM` | Generic PWM pulse-width sensor (AS5600, etc.) | PWM input (interrupt or blocking) |
//! | [`magnetic_spi`] | `MagneticSensorSPI` | Generic SPI magnetic sensor (AS5047P, AS5147, AS5048, MA730, etc.) | `SpiDevice` |
//!
//! Every concrete sensor is a struct that embeds [`SensorBase`]
//! (defined in [`crate::traits::sensor`]) and implements
//! [`Sensor<C>`]. The struct's generics are the clock type plus any
//! peripheral types the sensor needs (e.g. `I2C: I2c` for
//! [`magnetic_i2c::MagneticSensorI2C`]). The motor's FOC loop is
//! generic over `S: Sensor<C>`, so it can use any of these
//! implementations without modification.
//!
//! [`Sensor<C>`]: crate::traits::sensor::Sensor
//! [`SensorBase`]: crate::traits::sensor::SensorBase

#![allow(non_camel_case_types)]

pub mod encoder;
pub mod generic;
pub mod hall;
pub mod magnetic_analog;
pub mod magnetic_i2c;
pub mod magnetic_pwm;
pub mod magnetic_spi;
