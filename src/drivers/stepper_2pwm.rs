//! # drivers::stepper_2pwm — 2-phase stepper driver (2 PWM, 2 dir, optional 2 enables)
//!
//! Ports
//! [`StepperDriver2PWM.h`](source/Arduino-FOC/src/drivers/StepperDriver2PWM.h)
//! and
//! [`StepperDriver2PWM.cpp`](source/Arduino-FOC/src/drivers/StepperDriver2PWM.cpp)
//! from Arduino-FOC.
//!
//! The `StepperDriver2PWM` uses one PWM and one direction pin per phase.
//! (The upstream also supports a 2-dir-pin mode `IN1A`/`IN1B`, but this
//! port uses the simpler 1-dir-pin mode where `IN1B = !IN1A`.)
//!
//! ## Voltage-to-duty scaling
//!
//! The upstream formula is
//!
//! ```text
//! Ua = constrain(Ua, -voltage_limit, voltage_limit)
//! dc_a = constrain(abs(Ua) / voltage_power_supply, 0, 1)
//! ```
//!
//! i.e. voltages are clamped to `[-voltage_limit, voltage_limit]`,
//! then the absolute value is divided by the supply rail. The sign
//! of the voltage determines the direction pin state.

#![allow(non_camel_case_types)]

use crate::traits::driver::{
    DriverType, FOCDriver, FocDriverParams, PhaseState, StepperDriver,
};
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

/// 2-phase stepper motor driver (2 PWM channels, 2 direction pins,
/// up to 2 enable pins).
///
/// Ports the upstream `StepperDriver2PWM` class
/// ([`StepperDriver2PWM.h`](source/Arduino-FOC/src/drivers/StepperDriver2PWM.h)).
///
/// The struct is generic over:
/// - `P1`, `P2`: PWM channel types implementing [`SetDutyCycle`].
/// - `D1`, `D2`: direction pin types implementing [`OutputPin`].
/// - `EN1`, `EN2`: enable pin types implementing [`OutputPin`].
pub struct StepperDriver2PWM<P1, D1, P2, D2, EN1, EN2>
where
    P1: SetDutyCycle,
    D1: OutputPin,
    P2: SetDutyCycle,
    D2: OutputPin,
    EN1: OutputPin,
    EN2: OutputPin,
{
    params: FocDriverParams,
    pwm1: P1,
    dir1: D1,
    pwm2: P2,
    dir2: D2,
    enable1: Option<EN1>,
    enable2: Option<EN2>,
}

impl<P1, D1, P2, D2, EN1, EN2> StepperDriver2PWM<P1, D1, P2, D2, EN1, EN2>
where
    P1: SetDutyCycle,
    D1: OutputPin,
    P2: SetDutyCycle,
    D2: OutputPin,
    EN1: OutputPin,
    EN2: OutputPin,
{
    /// Construct a new 2-PWM stepper driver.
    pub fn new(
        pwm1: P1,
        dir1: D1,
        pwm2: P2,
        dir2: D2,
        enable1: Option<EN1>,
        enable2: Option<EN2>,
    ) -> Self {
        Self {
            params: FocDriverParams::new(),
            pwm1,
            dir1,
            pwm2,
            dir2,
            enable1,
            enable2,
        }
    }
}

impl<P1, D1, P2, D2, EN1, EN2> FOCDriver for StepperDriver2PWM<P1, D1, P2, D2, EN1, EN2>
where
    P1: SetDutyCycle,
    D1: OutputPin,
    P2: SetDutyCycle,
    D2: OutputPin,
    EN1: OutputPin,
    EN2: OutputPin,
{
    type Error = core::convert::Infallible;

    fn init(&mut self) -> Result<(), Self::Error> {
        if self.params.voltage_limit <= 0.0
            || self.params.voltage_limit > self.params.voltage_power_supply
        {
            self.params.voltage_limit = self.params.voltage_power_supply;
        }
        self.params.initialized = true;
        Ok(())
    }

    fn enable(&mut self) {
        let active = self.params.enable_active_high;
        if let Some(p) = self.enable1.as_mut() {
            let _ = if active { p.set_high() } else { p.set_low() };
        }
        if let Some(p) = self.enable2.as_mut() {
            let _ = if active { p.set_high() } else { p.set_low() };
        }
        self.set_pwm(0.0, 0.0);
    }

    fn disable(&mut self) {
        self.set_pwm(0.0, 0.0);
        let active = self.params.enable_active_high;
        if let Some(p) = self.enable1.as_mut() {
            let _ = if active { p.set_low() } else { p.set_high() };
        }
        if let Some(p) = self.enable2.as_mut() {
            let _ = if active { p.set_low() } else { p.set_high() };
        }
    }

    fn driver_type(&self) -> DriverType {
        DriverType::Stepper
    }

    fn params(&self) -> &FocDriverParams {
        &self.params
    }

    fn params_mut(&mut self) -> &mut FocDriverParams {
        &mut self.params
    }
}

impl<P1, D1, P2, D2, EN1, EN2> StepperDriver for StepperDriver2PWM<P1, D1, P2, D2, EN1, EN2>
where
    P1: SetDutyCycle,
    D1: OutputPin,
    P2: SetDutyCycle,
    D2: OutputPin,
    EN1: OutputPin,
    EN2: OutputPin,
{
    fn set_pwm(&mut self, ua: f32, ub: f32) {
        let v_limit = self.params.voltage_limit;
        let v_supply = self.params.voltage_power_supply;
        let ua = ua.clamp(-v_limit, v_limit);
        let ub = ub.clamp(-v_limit, v_limit);

        let max1 = self.pwm1.max_duty_cycle() as f32;
        let max2 = self.pwm2.max_duty_cycle() as f32;

        let dc1 = ((ua.abs() / v_supply).clamp(0.0, 1.0) * max1) as u16;
        let dc2 = ((ub.abs() / v_supply).clamp(0.0, 1.0) * max2) as u16;

        let _ = self.dir1.set_state(if ua >= 0.0 {
            embedded_hal::digital::PinState::Low
        } else {
            embedded_hal::digital::PinState::High
        });
        let _ = self.dir2.set_state(if ub >= 0.0 {
            embedded_hal::digital::PinState::Low
        } else {
            embedded_hal::digital::PinState::High
        });

        let _ = self.pwm1.set_duty_cycle(dc1);
        let _ = self.pwm2.set_duty_cycle(dc2);
    }

    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState) {
        if self.enable1.is_some() && self.enable2.is_some() {
            let active = self.params.enable_active_high;
            let drive = |s: PhaseState| -> bool {
                s == PhaseState::PHASE_ON
            };
            if let (Some(p1), Some(p2)) = (self.enable1.as_mut(), self.enable2.as_mut()) {
                let _ = p1.set_state(if drive(sa) == active {
                    embedded_hal::digital::PinState::High
                } else {
                    embedded_hal::digital::PinState::Low
                });
                let _ = p2.set_state(if drive(sb) == active {
                    embedded_hal::digital::PinState::High
                } else {
                    embedded_hal::digital::PinState::Low
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::pin::{Mock as PinMock, State as PinStateMock, Transaction as PinTransaction};
    use embedded_hal_mock::eh1::pwm::{Mock as PwmMock, Transaction as PwmTransaction};

    const MAX_DUTY: u16 = 1000;

    fn pwm_empty() -> (PwmMock, PwmMock) {
        (PwmMock::new(&[]), PwmMock::new(&[]))
    }

    fn pwm_zeroed() -> (PwmMock, PwmMock) {
        let exp = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        (PwmMock::new(&exp), PwmMock::new(&exp))
    }

    #[test]
    fn init_sets_initialized_flag() {
        let (mut p1, mut p2) = pwm_empty();
        let mut dir1 = PinMock::new(&[]);
        let mut dir2 = PinMock::new(&[]);
        let mut d = StepperDriver2PWM::new(
            &mut p1, &mut dir1, &mut p2, &mut dir2,
            None::<&mut PinMock>, None::<&mut PinMock>,
        );
        assert!(!d.params().initialized);
        d.init().unwrap();
        assert!(d.params().initialized);
        p1.done(); p2.done();
        dir1.done(); dir2.done();
    }

    #[test]
    fn set_pwm_scales_and_sets_direction() {
        let exp1 = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp2 = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let mut p1 = PwmMock::new(&exp1);
        let mut p2 = PwmMock::new(&exp2);
        let dir1_exp = [PinTransaction::set(PinStateMock::Low)];
        let dir2_exp = [PinTransaction::set(PinStateMock::Low)];
        let mut dir1 = PinMock::new(&dir1_exp);
        let mut dir2 = PinMock::new(&dir2_exp);

        let mut d = StepperDriver2PWM::new(
            &mut p1, &mut dir1, &mut p2, &mut dir2,
            None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 12.0;
        d.set_pwm(6.0, 6.0);

        p1.done(); p2.done();
        dir1.done(); dir2.done();
    }

    #[test]
    fn set_pwm_negative_voltage_flips_direction() {
        let exp1 = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp2 = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let mut p1 = PwmMock::new(&exp1);
        let mut p2 = PwmMock::new(&exp2);
        let dir1_exp = [PinTransaction::set(PinStateMock::High)];
        let dir2_exp = [PinTransaction::set(PinStateMock::High)];
        let mut dir1 = PinMock::new(&dir1_exp);
        let mut dir2 = PinMock::new(&dir2_exp);

        let mut d = StepperDriver2PWM::new(
            &mut p1, &mut dir1, &mut p2, &mut dir2,
            None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 12.0;
        d.set_pwm(-6.0, -6.0);

        p1.done(); p2.done();
        dir1.done(); dir2.done();
    }

    #[test]
    fn driver_type_is_stepper() {
        let (mut p1, mut p2) = pwm_empty();
        let mut dir1 = PinMock::new(&[]);
        let mut dir2 = PinMock::new(&[]);
        let d = StepperDriver2PWM::new(
            &mut p1, &mut dir1, &mut p2, &mut dir2,
            None::<&mut PinMock>, None::<&mut PinMock>,
        );
        assert_eq!(d.driver_type(), DriverType::Stepper);
        p1.done(); p2.done();
        dir1.done(); dir2.done();
    }

    #[test]
    fn disable_drives_pins_low() {
        let (mut p1, mut p2) = pwm_zeroed();
        let en1_exp = [PinTransaction::set(PinStateMock::Low)];
        let en2_exp = [PinTransaction::set(PinStateMock::Low)];
        let mut en1 = PinMock::new(&en1_exp);
        let mut en2 = PinMock::new(&en2_exp);
        let dir1_exp = [PinTransaction::set(PinStateMock::Low)];
        let dir2_exp = [PinTransaction::set(PinStateMock::Low)];
        let mut dir1 = PinMock::new(&dir1_exp);
        let mut dir2 = PinMock::new(&dir2_exp);

        let mut d = StepperDriver2PWM::new(
            &mut p1, &mut dir1, &mut p2, &mut dir2,
            Some(&mut en1), Some(&mut en2),
        );
        d.disable();
        p1.done(); p2.done();
        en1.done(); en2.done();
        dir1.done(); dir2.done();
    }
}
