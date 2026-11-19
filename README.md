# barefoc

[![Crates.io](https://img.shields.io/crates/v/barefoc.svg)](https://crates.io/crates/barefoc)
[![Documentation](https://docs.rs/barefoc/badge.svg)](https://docs.rs/barefoc)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![no_std](https://img.shields.io/badge/no__std-compatible-blue.svg)](https://github.com/rust-embedded)

`barefoc` is a `#![no_std]` Rust implementation of Field Oriented Control (FOC) for Brushless DC (BLDC) and stepper motors. It is derived from [Arduino SimpleFOC](https://github.com/simplefoc/Arduino-FOC) and targets the [`embedded-hal`](https://docs.rs/embedded-hal) v1.0 trait ecosystem.

The library operates with zero dynamic memory allocation and targets resource-constrained microcontrollers.

---

## Architecture & Features

### Core Design Principles
- **Strict `#![no_std]`**: Compatible with bare-metal microcontrollers without standard library or heap runtime requirements.
- **Generic Trait Composition**: Uses Rust compile-time generics for sensors, drivers, current sensing, and clocks to prevent virtual dispatch overhead in inner commutation loops.
- **Mathematical Foundations**: Uses `libm` for portable single-precision floating-point arithmetic.
- **Portability**: All hardware access is decoupled through `embedded-hal` 1.0 traits (`I2c`, `SpiDevice`, `SetDutyCycle`).

### Capabilities
- **Modulation Strategies**:
  - Sine PWM
  - Space Vector PWM (SVPWM)
  - Trapezoidal 120° and 150°
- **Control Modes**:
  - **Torque Control**: Voltage mode, DC current mode, or FOC field-aligned DQ current mode.
  - **Velocity Control**: Closed-loop PID velocity control and open-loop velocity generation.
  - **Angle Control**: Cascaded position-to-velocity closed-loop PID control and open-loop positioning.
- **Supported Actuators**:
  - 3-Phase Brushless DC Motors (`BLDCMotor`).
  - 2-Phase Bipolar Stepper Motors (`StepperMotor`).
  - Hybrid 3-Phase Steppers (`HybridStepperMotor`).
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

The following example illustrates setting up a BLDC motor with Space Vector PWM and velocity control:

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

    // Motor electrical parameters
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

    // Initialize hardware and FOC alignment
    motor.init();
    motor.init_foc();

    // Main control loop
    loop {
        motor.loop_foc();
        motor.move_motor(20.0); // Target: 20 rad/s
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

Host-side unit tests verify mathematical transforms, PID control, low-pass filtering, and state transitions:

```bash
cargo test -p barefoc
```

---

## License

This project is licensed under the MIT License. See [`LICENSE`](LICENSE) for details.
