# BareFOC

[![Crates.io](https://img.shields.io/crates/v/barefoc.svg)](https://crates.io/crates/barefoc)
[![Documentation](https://docs.rs/barefoc/badge.svg)](https://docs.rs/barefoc)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![no_std](https://img.shields.io/badge/no__std-compatible-blue.svg)](https://github.com/rust-embedded)

`barefoc` is a `#![no_std]` Rust implementation of Field Oriented Control (FOC) for Brushless DC (BLDC) and stepper motors, targeting the [`embedded-hal`](https://docs.rs/embedded-hal) v1.0 trait ecosystem.

The library operates with zero dynamic memory allocation and targets resource-constrained embedded microcontrollers.

---

## Attribution & Origins

BareFOC is a Rust port of [Arduino SimpleFOC](https://simplefoc.com) ([Arduino-FOC](https://github.com/simplefoc/Arduino-FOC)) and [Arduino-FOC-drivers](https://github.com/simplefoc/Arduino-FOC-drivers), originally created and maintained by Antun Skuric and the SimpleFOC community.

Core mathematical transformations (Clarke, Park), space vector PWM, lowpass filters, PID controllers, driver topologies, and sensor abstractions are derived from the upstream C++ implementation.

- Official SimpleFOC Project: <https://simplefoc.com>
- Upstream Core Repository: <https://github.com/simplefoc/Arduino-FOC>
- Upstream Drivers Repository: <https://github.com/simplefoc/Arduino-FOC-drivers>
- Upstream Documentation: <https://docs.simplefoc.com>

---

## Ecosystem Repositories

BareFOC is organized into specialized repositories under the [BareFOC GitHub Organization](https://github.com/bareFOC):

- **[`bareFOC`](https://github.com/bareFOC/bareFOC)** (this repository): Core FOC engine crate (`barefoc`), containing transformations, modulation, PID/filters, core traits, motor models, kinematics, and bootloader re-entry routines.
- **[`barefoc-drivers`](https://github.com/bareFOC/barefoc-drivers)**: Rotary angle sensors (AS5600, AS5047P, AS5048A, MT6701), smart gate drivers (TI DRV8316, DRV8311), 3D Hall sensors (TI TMAG5273), and DC motor control.
- **[`barefoc-comms`](https://github.com/bareFOC/barefoc-comms)**: Communication protocols and serial processors (`barefoc-commander` ASCII parser, `barefoc-fibre` RPC server for ODrive Web GUI, `barefoc-multibus` register map, and `barefoc-rtt`).
- **[`barefoc-hal`](https://github.com/bareFOC/barefoc-hal)**: Platform hardware abstraction bridges for STM32 (`barefoc-stm32` via Embassy), ESP32 (`barefoc-esp32` via `esp-hal`), and WCH CH32 RISC-V (`barefoc-ch32`).
- **[`barefoc-sim`](https://github.com/bareFOC/barefoc-sim)**: Software-in-the-Loop (SIL) electromechanical motor physics simulator and ODrive GUI live tuning bridge.

---

## Architecture & Design Principles

- **Strict `#![no_std]` & Zero Heap Allocation**: Operates without `alloc`, `std`, or an operating system.
- **Compile-Time Generics**: Sensor, driver, current sense, and clock traits are resolved at compile time to eliminate dynamic dispatch overhead in inner commutation loops.
- **Portable Float Arithmetic**: Uses `libm` for portable single-precision floating-point trigonometry and transforms.
- **`embedded-hal` Integration**: Decoupled from specific vendor register layouts via standard `embedded-hal` 1.0 digital I/O, PWM, SPI, and I2C traits.

---

## Hardware & Architecture Support

### Microcontroller Platform Bridges (`barefoc-hal`)

| Platform | Bridge Crate | Backend HAL | Features & Capabilities |
|---|---|---|---|
| **STM32** (F4, G4, H5) | `barefoc-stm32` | `embassy-stm32` | Complementary 6-PWM with hardware dead-time and break input (`BKIN`), hardware quadrature decoding (`Stm32Encoder`), DWT cycle counter (`DwtClock`), STM32G4 CORDIC accelerator (`Stm32G4Cordic`), internal OPAMP PGA configuration (`Stm32G4OpAmp`), and bus target transports (`barefoc-multibus`). |
| **ESP32** (ESP32, S3, C3, C6) | `barefoc-esp32` | `esp-hal` | Complementary 6-PWM via MCPWM with hardware dead-time and fault protection (`Esp32McpwmDriver`), 3-PWM fallback via LEDC (`Esp32LedcDriver`), hardware pulse counter decoding (`Esp32PcntSensor`), and system clock integration (`Esp32Clock`). |
| **WCH CH32** (V307) | `barefoc-ch32` | `ch32-hal` | Complementary 6-PWM, timer clocking, and ADC current sensing on RISC-V targets. |
| **Generic MCU** | Core `barefoc` | `embedded-hal` v1.0 | Any target implementing `embedded-hal` traits (RP2040, nRF52, SAMD, etc.). |

### Sensor Abstractions

| Sensor Interface | Core Driver / Trait | Supported Peripherals | Interface |
|---|---|---|---|
| **Quadrature Encoder** | `barefoc::sensors::encoder::Encoder` | Optical/magnetic AB/ABZ incremental encoders | 2–3 GPIO pins / Hardware timer QEI |
| **Hall Effect** | `barefoc::sensors::hall::HallSensor` | 120° and 60° commutation Hall sensors | 3 digital GPIO pins |
| **Generic I²C Magnetic** | `barefoc::sensors::magnetic_i2c::MagneticSensorI2C` | AS5600, AS5048B, MT6701, and custom registers | `embedded_hal::i2c::I2c` |
| **Generic SPI Magnetic** | `barefoc::sensors::magnetic_spi::MagneticSensorSPI` | AS5047P/D, AS5048A, MA730, and custom registers | `embedded_hal::spi::SpiDevice` |
| **Analog & PWM Magnetic** | `MagneticSensorAnalog`, `MagneticSensorPWM` | Absolute voltage or duty-cycle sensors | ADC channel / Timer input capture |

### Inverter & Gate Driver Topologies

| Topology | Abstraction | Supported Power Stages | Interface |
|---|---|---|---|
| **BLDC 3-PWM** | `BLDCDriver3PWM` | Integrated 3-phase bridges (DRV8313, L6234, discrete half-bridges) | 3 PWM channels + optional enable pins |
| **BLDC 6-PWM** | `BLDCDriver6PWM` | Smart gate drivers (DRV8301, DRV8323, STSPIN32G4, discrete MOSFET bridges) | 6 complementary PWM channels with dead-time |
| **Stepper 2-PWM / 4-PWM** | `StepperDriver2PWM`, `StepperDriver4PWM` | Dual H-bridge drivers (L298N, DRV8833, discrete H-bridges) | 2 or 4 PWM channels |
| **DC H-Bridge (1-Phase / 2-PWM)** | `DCDriver2PWM` (`barefoc-drivers`) | Single-phase H-bridge drivers (TI DRV8411A, DRV887x, L298N, discrete H-bridges) | 2 PWM channels with shoot-through protection + optional enable |

### Current Sensing Architectures

| Method | Abstraction | Supported Circuitry | Notes |
|---|---|---|---|
| **Inline Shunts** | `InlineCurrentSense` | Phase-series shunts (INA240, ACS712) | Phase currents sampled independently of PWM state; 2-shunt or 3-shunt. |
| **Low-Side Shunts** | `LowsideCurrentSense` | Ground-return shunts, internal CSAs (DRV8301, STSPIN32G4) | Sampling synchronized to low-side active conduction window; 2-shunt or 3-shunt. |
| **Current Mirror Sense** | `CurrentMirrorSense` (`barefoc-drivers`) | Integrated driver current mirrors (TI DRV8411A, DRV887x, IPROPI) | Single-pin unipolar current mirror with direction sign reconstruction. |
| **Generic Current** | `GenericCurrentSense` | Digital bus monitors or custom ADC routines | Custom closures for current acquisition. |

---

## Control Modes & Modulation

- **Modulation Strategies**:
  - Sine PWM
  - Space Vector PWM (SVPWM)
  - Trapezoidal 120° and 150°
- **Motion Control Modes**:
  - **Torque Control**: Voltage mode, DC current mode, or FOC field-aligned DQ current mode.
  - **Velocity Control**: Closed-loop PID velocity control and open-loop velocity generation.
  - **Angle Control**: Cascaded position-to-velocity closed-loop PID control and open-loop positioning.
- **Actuators Supported**:
  - 3-Phase Brushless DC Motors (`BLDCMotor`).
  - 2-Phase Bipolar Stepper Motors (`StepperMotor`).
  - Hybrid 3-Phase Steppers (`HybridStepperMotor`).
  - Brushed DC Motors (`DCMotor` via `barefoc-drivers` supporting voltage, velocity, and position closed loops).
- **Robotics Kinematics** (`barefoc::kinematics`):
  - Cascaded joint and actuator transmission abstractions (`Transmission`, `LinearGear`, `LutTransmission`).
  - Kinematic calibration tools (`AutonomousCalibrator`, `InteractiveCalibrator`).
- **Bootloader Re-entry** (`barefoc::boot`):
  - Mailbox cookie encoding and soft-reset coordination for Katapult and BareBoot bootloaders.

---

## Crate Layout

```
barefoc/src/
├── math/             # Clarke, Park, Space Vector PWM, angle normalization
├── common/           # PID controllers, low-pass filters, time abstractions, defaults
├── traits/           # Core traits: Sensor, BLDCDriver, StepperDriver, CurrentSense, FOCDriver
├── sensors/          # Quadrature encoders, Hall sensors, generic SPI/I2C magnetic sensors
├── drivers/          # BLDC 3-PWM, BLDC 6-PWM, Stepper 2-PWM, Stepper 4-PWM drivers
├── current_sense/    # Phase inline, low-side shunt, and generic current sensing
├── motor/            # BLDCMotor, StepperMotor, HybridStepperMotor, and base state machines
├── kinematics/       # Actuator gear transmissions, 1DOF/2DOF linkage models, auto-calibration
└── boot/             # BareBoot / Katapult mailbox cookies and bootloader re-entry routines
```

---

## Usage Example

```rust,no_run
use barefoc::math::*;
use barefoc::common::pid::PIDController;
use barefoc::common::time::Clock;
use barefoc::motor::bldc::BLDCMotor;
use barefoc::motor::enums::{FOCModulationType, MotionControlType, TorqueControlType};
use barefoc::traits::driver::BLDCDriver;
use barefoc::traits::sensor::Sensor;
use barefoc::traits::current_sense::CurrentSense;

fn setup_and_run<S, D, CS, C>(
    sensor: S,
    driver: D,
    current_sense: CS,
    clock: C,
    pole_pairs: u8,
) where
    S: Sensor,
    D: BLDCDriver,
    CS: CurrentSense,
    C: Clock + Copy,
{
    let mut motor = BLDCMotor::new(sensor, driver, current_sense, clock, pole_pairs);

    // Motor electrical limits
    motor.cfg_mut().voltage_limit = 12.0;
    motor.cfg_mut().velocity_limit = 50.0;
    motor.cfg_mut().foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.cfg_mut().torque_controller = TorqueControlType::voltage;
    motor.cfg_mut().controller = MotionControlType::velocity;

    // Velocity loop PID tuning
    motor.cfg_mut().PID_velocity.P = 0.20;
    motor.cfg_mut().PID_velocity.I = 2.00;
    motor.cfg_mut().PID_velocity.D = 0.00;
    motor.cfg_mut().LPF_velocity.Tf = 0.01;

    // Initialize hardware and FOC electrical alignment
    motor.init();
    motor.init_foc();

    // Main execution loop
    loop {
        motor.loop_foc();
        motor.move_motor(20.0); // Target velocity: 20 rad/s
    }
}
```

---

## Adapting External Sensor Crates

To connect existing community device drivers from [crates.io](https://crates.io) (e.g. an AS5600 driver) into BareFOC, implement the `barefoc::traits::sensor::Sensor` trait:

```rust,ignore
use barefoc::common::time::Clock;
use barefoc::traits::sensor::{Sensor, SensorBase};
use core::cell::RefCell;
use embedded_hal::i2c::I2c;

pub struct As5600Sensor<C: Clock, I2C: I2c> {
    base: SensorBase,
    driver: RefCell<as5600::As5600<I2C>>,
    _clock: core::marker::PhantomData<C>,
}

impl<C: Clock, I2C: I2c> Sensor for As5600Sensor<C, I2C> {
    fn base(&self) -> &SensorBase { &self.base }
    fn base_mut(&mut self) -> &mut SensorBase { &mut self.base }

    fn get_mechanical_angle(&self) -> f32 {
        let raw = self.driver.borrow_mut().angle().unwrap_or(0);
        (raw as f32 / 4096.0) * core::f32::consts::TAU
    }

    fn init(&mut self) -> Result<(), ()> {
        self.base.init();
        Ok(())
    }
}
```

---

## Cargo Features

| Feature | Description | Default |
|---|---|---|
| `std` | Enables standard library for host compilation and tests. | Enabled |
| `kinematics` | Enables transmission models and actuator kinematics (`barefoc::kinematics`). | Enabled |
| `boot` | Enables bootloader mailbox cookie helpers (`barefoc::boot`). | Enabled |
| `defmt` | Enables formatting implementations for `defmt` logging. | Disabled |
| `rtic` | Enables integrations for the RTIC embedded framework. | Disabled |

For bare-metal targets, disable default features:

```toml
[dependencies]
barefoc = { version = "0.1", default-features = false, features = ["kinematics", "boot"] }
```

---

## Testing

Host unit tests verify transforms, PID control, low-pass filtering, and commutation models:

```bash
cargo test -p barefoc
```

---

## License

This project is licensed under the MIT License. See [`LICENSE`](LICENSE) for details.
