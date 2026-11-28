# bareFOC

TOML-driven, `#![no_std]` Field Oriented Control (FOC) firmware for brushless and gimbal actuators.

Flash once per board using a declarative `barefoc.toml`, then configure node IDs, kinematics, sensor mapping, and PID gains at runtime over MIPI I3C, CAN, I2C, or UART with non-volatile flash persistence.

---

## Supported Example Boards

| Board | MCU | Gate Driver | Sensor | Host Bus | Config |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **MicroSpora** *(Recommended)* | STM32G431CBU6 | DRV8316 (6-PWM + SPI) | MT6701 (SPI) | FDCAN | [examples/microspora/barefoc.toml](examples/microspora/barefoc.toml) |
| **FishyServo** | STM32H503KBU6 | Quad DRV8411A | 4x TMAG5273 (I2C Mux) | MIPI I3C | [examples/fishyservo/barefoc.toml](examples/fishyservo/barefoc.toml) |
| **Miniata** | STM32H503KBU6 | DRV8311H (3-PWM) | Dual MT6701 (Shared SPI) | MIPI I3C / UART | [examples/miniata/barefoc.toml](examples/miniata/barefoc.toml) |

---

## Prerequisites

Install Rust targets for your MCU:

```bash
# STM32G4 (MicroSpora)
rustup target add thumbv7em-none-eabihf

# STM32H5 (FishyServo, Miniata)
rustup target add thumbv8m.main-none-eabihf
```

Optional tools:
- `probe-rs` (`cargo install probe-rs-tools`) for SWD flashing and RTT logs.
- `arm-none-eabi-objcopy` (or `cargo-binutils`) for generating `.bin` images for BareBoot OTA.

---

## Building Firmware

Select any board configuration with `BAREFOC_BOARD`:

### MicroSpora (`STM32G431`, FDCAN)
```bash
BAREFOC_BOARD=examples/microspora/barefoc.toml \
  cargo build --release --target thumbv7em-none-eabihf --features stm32g4
```

### FishyServo (`STM32H503`, MIPI I3C)
```bash
BAREFOC_BOARD=examples/fishyservo/barefoc.toml \
  cargo build --release --target thumbv8m.main-none-eabihf --features stm32h5
```

### Miniata (`STM32H503`, DRV8311H)
```bash
BAREFOC_BOARD=examples/miniata/barefoc.toml \
  cargo build --release --target thumbv8m.main-none-eabihf --features stm32h5
```

To generate a raw binary for BareBoot flashing:
```bash
arm-none-eabi-objcopy -O binary \
  target/thumbv8m.main-none-eabihf/release/barefoc-firmware \
  firmware.bin
```

---

## Flashing

### Via BareBoot (Handsfree over I3C / I2C / CAN / UART)
```bash
python3 ../bareBoot/host/scripts/flash_app_over_i3c_usb.py \
  --freq 1000 --addr 0x40 firmware.bin
```

### Via SWD (`probe-rs`)
```bash
BAREFOC_BOARD=examples/microspora/barefoc.toml \
  cargo run --release --target thumbv7em-none-eabihf --features stm32g4
```

---

## Runtime Configuration (Flash Once, Configure as Needed)

After flashing, you do not need to recompile firmware to change:
- **Bus identity**: `NodeId` and joint names
- **Actuator topology**: `four_1dof`, `two_2dof`, `one_1dof_one_2dof`, `two_1dof_one_2dof`, and `sensor_map`
- **Transmissions**: `linear_gear` (gear ratio, backlash, compliance), `coupled_differential` (2x2 matrix), `four_bar_linkage`, `eccentric_cam`, or `lut`
- **External sensors**: runtime selection from compiled-in `supported` sensors
- **Tuning & limits**: PID gains, LPF time constants, voltage/current/velocity limits

Settings can be applied and saved to non-volatile flash using:
- **bareConfig Workbench** (Desktop / WebAssembly GUI)
- **TOML CLI** (`configure_fishyservo_i3c.py --addr 0x40 --config my_joint.toml --save`)

---

## Documentation

- **Configuration Reference**: https://barefoc.github.io/config_reference/
- **Examples**: https://barefoc.github.io/examples/
- **bareConfig Workbench**: https://barefoc.github.io/tools/bareconfig/
- **BareBoot Bootloader**: https://barefoc.github.io/bareboot/
- **Core Library (`barefoc-core`)**: https://github.com/bareFOC/barefoc-core
