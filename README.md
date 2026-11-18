# barefoc

[![Crates.io](https://img.shields.io/crates/v/barefoc.svg)](https://crates.io/crates/barefoc)
[![Documentation](https://docs.rs/barefoc/badge.svg)](https://docs.rs/barefoc)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![no_std](https://img.shields.io/badge/no__std-compatible-blue.svg)](https://github.com/rust-embedded)

**BareFOC** is a pure-Rust, `#![no_std]` port of the renowned [Arduino SimpleFOC](https://simplefoc.com) library ([GitHub: Arduino-FOC](https://github.com/simplefoc/Arduino-FOC)) targeting embedded microcontrollers via the `embedded-hal` v1.0 trait ecosystem.

It provides modular, battle-tested Field Oriented Control (FOC) algorithms for Brushless DC (BLDC) and Stepper motors with zero dynamic heap allocations.

---

## Features

* **Strict `#![no_std]` & Zero Allocation**:
  * Runs on bare-metal microcontrollers without `alloc`, `std`, or an operating system.
  * Uses `libm` for portable, accurate floating-point trigonometry and transformations.
* **Core FOC Algorithms**:
  * Forward and Inverse Clarke & Park transformations.
  * Modulation modes: Sine PWM, Space Vector PWM (SVPWM), Trapezoidal 120°, and Trapezoidal 150°.
  * Voltage, DC current, and true DQ current (FOC) torque control.
* **Control Modes**:
  * **Torque Mode**: Direct voltage, DC current, or FOC current mode.
  * **Velocity Mode**: Closed-loop PID velocity control or open-loop velocity control.
  * **Angle Mode**: Cascaded position-to-velocity closed-loop PID control or open-loop angle control.
* **Sensors**:
  * Quadrature Encoders (`Encoder`) with hardware counter or software edge detection.
  * 3-Phase Hall Effect Sensors (`HallSensor`).
  * Generic SPI Magnetic Encoders (`MagneticSensorSPI`: AS5047P, AS5048A, MA730, etc.).
  * Generic I2C Magnetic Encoders (`MagneticSensorI2C`: AS5600, AS5048B, MT6701, etc.).
  * Analog & PWM Magnetic Sensors (`MagneticSensorAnalog`, `MagneticSensorPWM`).
* **Drivers**:
  * 3-PWM and complementary 6-PWM with hardware/software dead-time for BLDC motors.
  * 2-PWM and 4-PWM drivers for 2-phase Stepper motors.
* **Current Sensing**:
  * Inline current sensing (`InlineCurrentSense`).
  * Low-side current sensing (`LowsideCurrentSense`).
* **Hardware Ecosystem**:
  * Platform HAL bridges for STM32 (`barefoc-stm32` via Embassy) and ESP32 (`barefoc-esp32` via `esp-hal` + `esp-rtos`).
  * Communication and GUI integration via `barefoc-commander` and `barefoc-fibre` (ODrive GUI bridge).

---

## Supported Hardware

BareFOC is strictly chip-agnostic: its core algorithms rely on `embedded-hal` v1.0 traits, while specific silicon peripherals can be integrated through generic protocol drivers or existing community driver crates on [crates.io](https://crates.io).

### Microcontroller Platforms & HAL Bridges

| Platform / MCU | Bridge Crate | Backend HAL | Features & Capabilities |
|---|---|---|---|
| **STM32 Series** (F4, G4, etc.) | `barefoc-stm32` | `embassy-stm32` | Complementary 6-PWM with hardware dead-time & break input (`BKIN`), hardware QEI encoder decoding (`Stm32Encoder`), DWT cycle counter (`DwtClock`), STM32G4 hardware CORDIC math accelerator (`Stm32G4Cordic`), and internal programmable gain OPAMPs (`Stm32G4OpAmp`). |
| **ESP32** | `barefoc-esp32` (`esp32`) | `esp-hal` + `esp-rtos` | Complementary 6-PWM via MCPWM with hardware dead-time & fault trip (`Esp32McpwmDriver`), 3-PWM fallback via LEDC (`Esp32LedcDriver`), hardware quadrature pulse counter (`Esp32PcntSensor`), Embassy & RTOS clock support (`Esp32Clock`, `EmbassyClock`). |
| **Any `embedded-hal` MCU** | Core `barefoc` | `embedded-hal` v1.0 | Any target implementing `embedded-hal` (RP2040, nRF52, SAMD, RISC-V, etc.) using generic PWM and sensor drivers. |

### Position & Angle Sensors

| Sensor Type | Core Driver / Abstraction | Compatible ICs & Hardware | Driver Interface | Notes & Community Crates |
|---|---|---|---|---|
| **Quadrature Encoder** | `barefoc::sensors::encoder::Encoder` | Optical/magnetic incremental encoders (e.g. AMT102, AEDT-9810, generic AB/ABZ) | 2 or 3 digital GPIOs | Supports software interrupt decoding or zero-overhead hardware timer decoders (`embassy-stm32::qei`, `esp-hal::pcnt`). |
| **Hall Effect** | `barefoc::sensors::hall::HallSensor` | 3-phase BLDC Hall sensor signals (120° or 60° placement) | 3 digital GPIOs | Interrupt-driven edge detection and 6-sector state machine. |
| **Generic I²C Magnetic** | `barefoc::sensors::magnetic_i2c::MagneticSensorI2C` | AMS AS5600, AS5048B, MagnTek MT6701, etc. | `embedded_hal::i2c::I2c` | Completely chip-agnostic via user-defined `MagneticSensorI2CConfig` (register address, bit resolution, bit shifts). |
| **Generic SPI Magnetic** | `barefoc::sensors::magnetic_spi::MagneticSensorSPI` | AMS AS5047P/D, AS5048A, AS5147, MPS MA730, etc. | `embedded_hal::spi::SpiDevice` | Completely chip-agnostic via user-defined `MagneticSensorSPIConfig` (16-bit command format, parity, bit resolution). |
| **Analog & PWM Magnetic** | `MagneticSensorAnalog`, `MagneticSensorPWM` | AS5600 (analog OUT or PWM), MT6701, generic linear Halls | `Adc` / GPIO timer | Low-pin-count absolute angle sensing. |
| **Off-the-shelf Crates** | Custom wrapper on `barefoc::traits::sensor::Sensor` | AS5600, AS5048A, MT6816, MLX90393, TMAG5273 | Crates.io drivers | Directly adapt any external device driver crate in ~20 lines of code (see example below). |

### Inverter Topologies & Gate Drivers

| Driver Stage | Core Abstraction | Hardware & Smart Gate Drivers | Control Interface |
|---|---|---|---|
| **BLDC 3-PWM** | `barefoc::drivers::bldc_3pwm::BLDCDriver3PWM` | TI DRV8313, ST L6234, discrete 3-phase half-bridge modules with integrated gate drivers | 3x PWM + optional enable pins |
| **BLDC 6-PWM** | `barefoc::drivers::bldc_6pwm::BLDCDriver6PWM` | TI DRV8301, DRV8302, DRV8323, ST STSPIN32G4, discrete high/low MOSFETs | 6x complementary PWM with dead-time and active-low/high enable lines |
| **Stepper 2-PWM / 4-PWM** | `StepperDriver2PWM`, `StepperDriver4PWM` | Dual H-bridge drivers (L298N, DRV8833, discrete H-bridges) | 2 or 4 PWM channels for 2-phase bipolar steppers |

### Current Sensing Architectures

| Sensing Topology | Core Abstraction | Compatible Hardware | Notes |
|---|---|---|---|
| **Phase Inline** | `barefoc::current_sense::inline::InlineCurrentSense` | Allegro ACS712, TI INA240, integrated inline shunt amplifiers | Samples phase currents at any point in the PWM period; 2-shunt or 3-shunt. |
| **Low-Side Shunt** | `barefoc::current_sense::lowside::LowsideCurrentSense` | Discrete ground-return shunts, smart gate driver integrated CSAs (DRV8301, DRV8323, STSPIN32G4) | Synchronized with low-side FET conduction windows; 2-shunt or 3-shunt. |
| **Generic Current** | `barefoc::current_sense::generic::GenericCurrentSense` | Custom ADC callbacks or external digital monitors (e.g. INA226) | User-provided closures for current sampling. |

---

### Adapting Off-the-Shelf Sensor Crates

To keep `barefoc` lightweight and chip-agnostic, third-party sensor driver crates (such as [`as5600`](https://crates.io/crates/as5600) on crates.io) can be linked to `barefoc` by wrapping the driver and implementing the `barefoc::traits::sensor::Sensor` trait:

```rust,ignore
use barefoc::common::time::Clock;
use barefoc::traits::sensor::{Sensor, SensorBase};
use core::cell::RefCell;
use embedded_hal::i2c::I2c;

/// Adapter wrapping the external `as5600::As5600` driver crate
pub struct As5600Sensor<C: Clock, I2C: I2c> {
    base: SensorBase,
    driver: RefCell<as5600::As5600<I2C>>,
    _clock: core::marker::PhantomData<C>,
}

impl<C: Clock, I2C: I2c> As5600Sensor<C, I2C> {
    pub fn new(driver: as5600::As5600<I2C>) -> Self {
        Self {
            base: SensorBase::new(),
            driver: RefCell::new(driver),
            _clock: core::marker::PhantomData,
        }
    }
}

impl<C: Clock, I2C: I2c> Sensor<C> for As5600Sensor<C, I2C> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        // Read raw 12-bit angle (0..4095) from external as5600 driver
        match self.driver.borrow_mut().raw_angle() {
            Ok(raw) => (raw as f32) * (barefoc::math::_2PI / 4096.0),
            Err(_) => 0.0,
        }
    }
}
```

## Getting Started

Add `barefoc` to your `Cargo.toml`:

```toml
[dependencies]
barefoc = { version = "0.1", default-features = false }
```

### Basic BLDC Motor Setup

```rust,no_run
#![no_std]

use barefoc::common::foc_motor::{FOCModulationType, MotionControlType, TorqueControlType};
use barefoc::common::pid::PIDController;
use barefoc::common::time::Clock;
use barefoc::motor::BLDCMotor;

// 1. Instantiate motor (e.g. 7 pole pairs, 12V supply)
let mut motor = BLDCMotor::new(7);
motor.voltage_power_supply = 12.0;
motor.voltage_limit = 6.0;
motor.velocity_limit = 20.0;

// 2. Configure control loops and modulation
motor.controller = MotionControlType::velocity;
motor.torque_controller = TorqueControlType::voltage;
motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

// 3. Configure PID parameters
motor.p_angle.p = 20.0;
motor.pid_velocity.p = 0.5;
motor.pid_velocity.i = 10.0;
motor.pid_velocity.output_ramp = 1000.0;
motor.lpf_velocity.tf = 0.01;

// 4. Initialize hardware (sensor and driver supplied by your HAL)
// motor.link_sensor(&mut sensor);
// motor.link_driver(&mut driver);
// motor.init();
// motor.init_foc();

// 5. Real-time control loop
// loop {
//     motor.loop_foc();
//     motor.move_motor(target_velocity);
// }
```

---

## Cargo Features

| Feature | Description |
|---|---|
| `default` | Enables `std` for host unit testing. |
| `std` | Host test environment only. Disable for embedded targets. |
| `defmt` | Enables logging via RTT and `defmt`. |
| `rtic` | Enables integration helpers with RTIC (Real-Time Interrupt-driven Concurrency). |
| `inline-current-sense` | Enables inline current sensing structures. |
| `magnetic-pwm-blocking`| Enables blocking pulse measurement fallback for PWM magnetic sensors. |

---

## Upstream Attribution

BareFOC is a faithful translation of [SimpleFOC](https://simplefoc.com), The core algorithms, mathematical foundations, coordinate transformations, and control loops are direct ports of SimpleFOC's C++ implementation.

>> SimpleFOC: A Field Oriented Control (FOC) Library for Controlling Brushless Direct Current (BLDC) and Stepper Motors.
A. Skuric, HS. Bank, R. Unger, O. Williams, D. González-Reyes
Journal of Open Source Software, 7(74), 4232, https://doi.org/10.21105/joss.04232 

The name 'barefoc' was chosen to avoid confusion with the original 'simplefoc' project.

---

### Vibe Code Notice

This port was primarily written by llms, not manually reviewed yet. Use with caution.

## License

This project is licensed under the [MIT License](LICENSE).
Upstream Arduino SimpleFOC is also licensed under the MIT License (Copyright © 2020-2024 Antun Skuric and the SimpleFOC community).
