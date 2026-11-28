//! Generic 6-PWM BLDC FOC runner (DRV8316 + SPI encoder + FDCAN/ADC), driven by `crate::bsp`.

use barefoc::current_sense::generic::GenericCurrentSense;
use barefoc::math::PhaseCurrent;
use barefoc::motor::base::FOCMotor;
use barefoc::motor::bldc::BLDCMotor;
use barefoc::motor::enums::{FOCModulationType, MotionControlType, TorqueControlType};
use barefoc_stm32::stm32g4::FocConfig;
use barefoc_stm32::DwtClock;
use drv83xx_rs::{Drv8316CsaGain, Drv8316Driver6PWM, Drv8316PwmMode, Drv8316S, Drv8316SlewRate};
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, Config as AdcConfig};
use embassy_stm32::can::{CanConfigurator, Frame};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{Config as SpiConfig, Spi, MODE_1};
use embassy_stm32::time::Hertz;
use embedded_can::StandardId;
use mt6xxx_rs::{Config as Mt6701Config, Mt6701, SpiMode};
use rtt_target::{rprintln, rtt_init_print};

use crate::hal::{configure_g4_clocks, ExclusiveSpiDevice};

/// ADC reference voltage in volts.
const VREF: f32 = 3.3;

pub async fn run(_spawner: Spawner) {
    let p = embassy_stm32::init(configure_g4_clocks());
    rtt_init_print!();
    rprintln!(
        "{} BareFOC 6-PWM starting ({} Hz)...",
        crate::bsp::BOARD_NAME,
        crate::bsp::SYSTEM_CLOCK_HZ
    );

    // Status LED and CAN transceiver enable (typed from barefoc.toml)
    let leds = crate::bsp::embassy_bsp::take_status_led_peripherals!(p);
    let mut led = Output::new(leds.led0, Level::High, Speed::Low);
    let comm = crate::bsp::embassy_bsp::take_comm_peripherals!(p);
    let _can_en = Output::new(comm.can_enable, Level::High, Speed::Low);

    // FDCAN (typed from barefoc.toml)
    let mut can_cfg = CanConfigurator::new(
        comm.bus,
        comm.rx_sda,
        comm.tx_scl,
        crate::bsp::embassy_bsp::CommIrqs,
    );
    can_cfg.set_bitrate(crate::bsp::COMM_BITRATE);
    let mut can = can_cfg.into_normal_mode();
    if let Some(id) = StandardId::new(0x100) {
        if let Ok(frame) = Frame::new_data(id, b"BFOC") {
            let _ = can.write(&frame).await;
        }
    }

    // Bus voltage sense (typed from barefoc.toml)
    let vbus_adc = crate::bsp::embassy_bsp::take_vbus_adc_peripherals!(p);
    let mut adc = Adc::new_blocking(vbus_adc.adc, AdcConfig::default());
    let sample_time = adc.sample_time_for_us(2);
    let mut vsens_pin = vbus_adc.pin;
    let raw_vbus = adc.blocking_read(&mut vsens_pin, sample_time);
    let vbus = (raw_vbus as f32 / 4095.0) * VREF * crate::bsp::VBUS_DIVIDER_SCALE;
    rprintln!("Measured Vbus: {} V (raw={})", vbus, raw_vbus);

    let v_supply = if vbus > 5.0 { vbus } else { 12.0 };
    let v_limit = (v_supply * 0.5).min(crate::bsp::DEFAULT_VOLTAGE_LIMIT);

    // 1. Hardware FOC configuration & DWT microsecond clock
    let foc_cfg = FocConfig::new()
        .pwm_frequency(crate::bsp::PWM_FREQUENCY_HZ)
        .dead_time_ns(500)
        .voltage_power_supply(v_supply)
        .voltage_limit(v_limit);
    let clock = foc_cfg.init_clock(crate::bsp::SYSTEM_CLOCK_HZ);

    // 2. TIM1 6-PWM stage (typed from barefoc.toml)
    let gd = crate::bsp::embassy_bsp::take_gate_driver_peripherals!(p);
    let pwm_driver = foc_cfg.build_6pwm_driver(
        gd.timer,
        gd.inh_c,
        gd.inl_c,
        gd.inh_b,
        gd.inl_b,
        gd.inh_a,
        gd.inl_a,
    );

    // 3. DRV8316 gate driver over SPI3 (typed from barefoc.toml)
    let mut drv_spi_cfg = SpiConfig::default();
    drv_spi_cfg.frequency = Hertz(1_000_000);
    drv_spi_cfg.mode = MODE_1;
    let spi3 = Spi::new_blocking(gd.spi_bus, gd.spi_sck, gd.spi_mosi, gd.spi_miso, drv_spi_cfg);
    let drv_cs = Output::new(gd.cs, Level::High, Speed::High);
    let drv_spi_dev = ExclusiveSpiDevice::new(spi3, drv_cs);

    let drv8316 = Drv8316S::new(drv_spi_dev)
        .with_pwm_mode(Drv8316PwmMode::Pwm6)
        .with_csa_gain(Drv8316CsaGain::Gain0V15)
        .with_slew_rate(Drv8316SlewRate::Slew150Vus);

    let driver = Drv8316Driver6PWM::new(drv8316, pwm_driver);

    // 4. MT6701 14-bit SSI magnetic encoder over SPI1 (typed from barefoc.toml)
    let sens = crate::bsp::embassy_bsp::take_sensor_peripherals!(p);
    let mut enc_spi_cfg = SpiConfig::default();
    enc_spi_cfg.frequency = Hertz(8_000_000);
    enc_spi_cfg.mode = MODE_1;
    let spi1 = Spi::new_blocking(sens.bus, sens.clk, sens.mosi, sens.data, enc_spi_cfg);
    let enc_cs = Output::new(sens.cs, Level::High, Speed::High);
    let enc_spi_dev = ExclusiveSpiDevice::new(spi1, enc_cs);

    let encoder = Mt6701::with_config(
        enc_spi_dev,
        Mt6701Config {
            spi_mode: SpiMode::Mode1,
            crc_init: 0,
            max_retries: 3,
        },
    );

    // 5. Wire BLDCMotor
    let mut motor: BLDCMotor<'_, _, _, GenericCurrentSense<fn() -> PhaseCurrent, DwtClock>, DwtClock> =
        BLDCMotor::new(crate::bsp::POLE_PAIRS as i32, 0.5, 100.0, 0.0, 0.0, &clock);

    motor.link_driver(driver);
    motor.link_sensor(encoder);

    motor.config.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.config.torque_controller = TorqueControlType::voltage;
    motor.config.controller = MotionControlType::velocity;
    motor.config.voltage_limit = v_limit;
    motor.config.voltage_sensor_align = 2.0;
    motor.config.PID_velocity.set_p(0.25);
    motor.config.PID_velocity.set_i(3.5);
    motor.config.PID_velocity.set_d(0.001);
    motor.config.PID_velocity.set_output_ramp(100.0);
    motor.config.LPF_velocity.set_tf(0.015);

    let init_ok = motor.init();
    let foc_ok = motor.init_foc(&clock);
    rprintln!("Motor init={} init_foc={}", init_ok, foc_ok);

    let target_velocity: f32 = 10.0; // rad/s
    let mut tick: u32 = 0;

    loop {
        motor.loop_foc(&clock);
        motor.move_motor(target_velocity, &clock);

        tick = tick.wrapping_add(1);
        if tick % 2000 == 0 {
            led.toggle();
            let raw_v = adc.blocking_read(&mut vsens_pin, sample_time);
            let v_now = (raw_v as f32 / 4095.0) * VREF * crate::bsp::VBUS_DIVIDER_SCALE;
            rprintln!(
                "angle={} rad, vel={} rad/s, vbus={} V",
                motor.shaft_angle(),
                motor.shaft_velocity(),
                v_now
            );
        }
    }
}
