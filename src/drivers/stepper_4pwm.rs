//! # drivers::stepper_4pwm — 2-phase stepper driver (4 PWM, optional 2 enables)
//!
//! Ports
//! [`StepperDriver4PWM.h`](source/Arduino-FOC/src/drivers/StepperDriver4PWM.h)
//! and
//! [`StepperDriver4PWM.cpp`](source/Arduino-FOC/src/drivers/StepperDriver4PWM.cpp)
//! from Arduino-FOC.
//!
//! The `StepperDriver4PWM` uses two PWM channels per phase (e.g., `1A`/`1B`).
//! This allows for more efficient braking and finer control than the 2-PWM
//! variant.

#![allow(non_camel_case_types)]

use crate::traits::driver::{
    DriverType, FOCDriver, FocDriverParams, PhaseState, StepperDriver,
};
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

/// 2-phase stepper motor driver (4 PWM channels, up to 2 enable pins).
///
/// Ports the upstream `StepperDriver4PWM` class
/// ([`StepperDriver4PWM.h`](source/Arduino-FOC/src/drivers/StepperDriver4PWM.h)).
pub struct StepperDriver4PWM<P1A, P1B, P2A, P2B, EN1, EN2>
where
    P1A: SetDutyCycle,
    P1B: SetDutyCycle,
    P2A: SetDutyCycle,
    P2B: SetDutyCycle,
    EN1: OutputPin,
    EN2: OutputPin,
{
    params: FocDriverParams,
    pwm1a: P1A,
    pwm1b: P1B,
    pwm2a: P2A,
    pwm2b: P2B,
    enable1: Option<EN1>,
    enable2: Option<EN2>,
}

impl<P1A, P1B, P2A, P2B, EN1, EN2> StepperDriver4PWM<P1A, P1B, P2A, P2B, EN1, EN2>
where
    P1A: SetDutyCycle,
    P1B: SetDutyCycle,
    P2A: SetDutyCycle,
    P2B: SetDutyCycle,
    EN1: OutputPin,
    EN2: OutputPin,
{
    /// Construct a new 4-PWM stepper driver.
    pub fn new(
        pwm1a: P1A,
        pwm1b: P1B,
        pwm2a: P2A,
        pwm2b: P2B,
        enable1: Option<EN1>,
        enable2: Option<EN2>,
    ) -> Self {
        Self {
            params: FocDriverParams::new(),
            pwm1a,
            pwm1b,
            pwm2a,
            pwm2b,
            enable1,
            enable2,
        }
    }
}

impl<P1A, P1B, P2A, P2B, EN1, EN2> FOCDriver for StepperDriver4PWM<P1A, P1B, P2A, P2B, EN1, EN2>
where
    P1A: SetDutyCycle,
    P1B: SetDutyCycle,
    P2A: SetDutyCycle,
    P2B: SetDutyCycle,
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

impl<P1A, P1B, P2A, P2B, EN1, EN2> StepperDriver for StepperDriver4PWM<P1A, P1B, P2A, P2B, EN1, EN2>
where
    P1A: SetDutyCycle,
    P1B: SetDutyCycle,
    P2A: SetDutyCycle,
    P2B: SetDutyCycle,
    EN1: OutputPin,
    EN2: OutputPin,
{
    fn set_pwm(&mut self, ua: f32, ub: f32) {
        let v_limit = self.params.voltage_limit;
        let v_supply = self.params.voltage_power_supply;
        let ua = ua.clamp(-v_limit, v_limit);
        let ub = ub.clamp(-v_limit, v_limit);

        let max1a = self.pwm1a.max_duty_cycle() as f32;
        let max1b = self.pwm1b.max_duty_cycle() as f32;
        let max2a = self.pwm2a.max_duty_cycle() as f32;
        let max2b = self.pwm2b.max_duty_cycle() as f32;

        // Upstream: if Ua >= 0, pwm1a = duty, pwm1b = 0; else pwm1a = 0, pwm1b = duty
        let (dc1a, dc1b) = if ua >= 0.0 {
            (
                ((ua / v_supply).clamp(0.0, 1.0) * max1a) as u16,
                0,
            )
        } else {
            (
                0,
                ((ua.abs() / v_supply).clamp(0.0, 1.0) * max1b) as u16,
            )
        };

        let (dc2a, dc2b) = if ub >= 0.0 {
            (
                ((ub / v_supply).clamp(0.0, 1.0) * max2a) as u16,
                0,
            )
        } else {
            (
                0,
                ((ub.abs() / v_supply).clamp(0.0, 1.0) * max2b) as u16,
            )
        };

        let _ = self.pwm1a.set_duty_cycle(dc1a);
        let _ = self.pwm1b.set_duty_cycle(dc1b);
        let _ = self.pwm2a.set_duty_cycle(dc2a);
        let _ = self.pwm2b.set_duty_cycle(dc2b);
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
    use embedded_hal_mock::eh1::pin::Mock as PinMock;
    use embedded_hal_mock::eh1::pwm::{Mock as PwmMock, Transaction as PwmTransaction};

    const MAX_DUTY: u16 = 1000;

    fn pwm_empty() -> (PwmMock, PwmMock, PwmMock, PwmMock) {
        (PwmMock::new(&[]), PwmMock::new(&[]), PwmMock::new(&[]), PwmMock::new(&[]))
    }

    #[test]
    fn init_sets_initialized_flag() {
        let (mut p1a, mut p1b, mut p2a, mut p2b) = pwm_empty();
        let mut d = StepperDriver4PWM::new(
            &mut p1a, &mut p1b, &mut p2a, &mut p2b,
            None::<&mut PinMock>, None::<&mut PinMock>,
        );
        assert!(!d.params().initialized);
        d.init().unwrap();
        assert!(d.params().initialized);
        p1a.done(); p1b.done(); p2a.done(); p2b.done();
    }

    #[test]
    fn set_pwm_positive_voltage_uses_a_pins() {
        let exp1a = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp1b = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let exp2a = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp2b = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let mut p1a = PwmMock::new(&exp1a);
        let mut p1b = PwmMock::new(&exp1b);
        let mut p2a = PwmMock::new(&exp2a);
        let mut p2b = PwmMock::new(&exp2b);

        let mut d = StepperDriver4PWM::new(
            &mut p1a, &mut p1b, &mut p2a, &mut p2b,
            None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 12.0;
        d.set_pwm(6.0, 6.0);

        p1a.done(); p1b.done(); p2a.done(); p2b.done();
    }

    #[test]
    fn set_pwm_negative_voltage_uses_b_pins() {
        let exp1a = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let exp1b = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp2a = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let exp2b = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let mut p1a = PwmMock::new(&exp1a);
        let mut p1b = PwmMock::new(&exp1b);
        let mut p2a = PwmMock::new(&exp2a);
        let mut p2b = PwmMock::new(&exp2b);

        let mut d = StepperDriver4PWM::new(
            &mut p1a, &mut p1b, &mut p2a, &mut p2b,
            None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 12.0;
        d.set_pwm(-6.0, -6.0);

        p1a.done(); p1b.done(); p2a.done(); p2b.done();
    }

    #[test]
    fn driver_type_is_stepper() {
        let (mut p1a, mut p1b, mut p2a, mut p2b) = pwm_empty();
        let d = StepperDriver4PWM::new(
            &mut p1a, &mut p1b, &mut p2a, &mut p2b,
            None::<&mut PinMock>, None::<&mut PinMock>,
        );
        assert_eq!(d.driver_type(), DriverType::Stepper);
        p1a.done(); p1b.done(); p2a.done(); p2b.done();
    }
}
