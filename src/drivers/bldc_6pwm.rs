//! # drivers::bldc_6pwm — 3-phase BLDC driver (6 PWM, optional 1 enable, dead zone)
//!
//! Ports
//! [`BLDCDriver6PWM.h`](source/Arduino-FOC/src/drivers/BLDCDriver6PWM.h)
//! and
//! [`BLDCDriver6PWM.cpp`](source/Arduino-FOC/src/drivers/BLDCDriver6PWM.cpp)
//! from Arduino-FOC.
//!
//! The `BLDCDriver6PWM` uses two PWM channels per phase (high-side and
//! low-side). This allows for active braking and finer control than the
//! 3-PWM variant. It also supports a configurable `dead_zone` to prevent
//! shoot-through.

#![allow(non_camel_case_types)]

use crate::traits::driver::{
    BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState,
};
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

/// 3-phase BLDC motor driver (6 PWM channels, optional 1 enable pin).
///
/// Ports the upstream `BLDCDriver6PWM` class
/// ([`BLDCDriver6PWM.h`](source/Arduino-FOC/src/drivers/BLDCDriver6PWM.h)).
pub struct BLDCDriver6PWM<PAH, PAL, PBH, PBL, PCH, PCL, EN>
where
    PAH: SetDutyCycle,
    PAL: SetDutyCycle,
    PBH: SetDutyCycle,
    PBL: SetDutyCycle,
    PCH: SetDutyCycle,
    PCL: SetDutyCycle,
    EN: OutputPin,
{
    params: FocDriverParams,
    pwm_a_h: PAH,
    pwm_a_l: PAL,
    pwm_b_h: PBH,
    pwm_b_l: PBL,
    pwm_c_h: PCH,
    pwm_c_l: PCL,
    enable: Option<EN>,
    /// Dead-zone as a fraction of the PWM cycle `[0, 1]`.
    pub dead_zone: f32,
    /// Phase states (active / disabled).
    pub phase_state: [PhaseState; 3],
}

impl<PAH, PAL, PBH, PBL, PCH, PCL, EN> BLDCDriver6PWM<PAH, PAL, PBH, PBL, PCH, PCL, EN>
where
    PAH: SetDutyCycle,
    PAL: SetDutyCycle,
    PBH: SetDutyCycle,
    PBL: SetDutyCycle,
    PCH: SetDutyCycle,
    PCL: SetDutyCycle,
    EN: OutputPin,
{
    /// Construct a new 6-PWM BLDC driver.
    pub fn new(
        pwm_a_h: PAH,
        pwm_a_l: PAL,
        pwm_b_h: PBH,
        pwm_b_l: PBL,
        pwm_c_h: PCH,
        pwm_c_l: PCL,
        enable: Option<EN>,
    ) -> Self {
        Self {
            params: FocDriverParams::new(),
            pwm_a_h,
            pwm_a_l,
            pwm_b_h,
            pwm_b_l,
            pwm_c_h,
            pwm_c_l,
            enable,
            dead_zone: 0.0,
            phase_state: [PhaseState::PHASE_ON; 3],
        }
    }
}

impl<PAH, PAL, PBH, PBL, PCH, PCL, EN> FOCDriver for BLDCDriver6PWM<PAH, PAL, PBH, PBL, PCH, PCL, EN>
where
    PAH: SetDutyCycle,
    PAL: SetDutyCycle,
    PBH: SetDutyCycle,
    PBL: SetDutyCycle,
    PCH: SetDutyCycle,
    PCL: SetDutyCycle,
    EN: OutputPin,
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
        if let Some(p) = self.enable.as_mut() {
            let _ = if self.params.enable_active_high {
                p.set_high()
            } else {
                p.set_low()
            };
        }
        self.set_pwm(0.0, 0.0, 0.0);
    }

    fn disable(&mut self) {
        self.set_pwm(0.0, 0.0, 0.0);
        if let Some(p) = self.enable.as_mut() {
            let _ = if self.params.enable_active_high {
                p.set_low()
            } else {
                p.set_high()
            };
        }
    }

    fn driver_type(&self) -> DriverType {
        DriverType::BLDC
    }

    fn params(&self) -> &FocDriverParams {
        &self.params
    }

    fn params_mut(&mut self) -> &mut FocDriverParams {
        &mut self.params
    }
}

impl<PAH, PAL, PBH, PBL, PCH, PCL, EN> BLDCDriver for BLDCDriver6PWM<PAH, PAL, PBH, PBL, PCH, PCL, EN>
where
    PAH: SetDutyCycle,
    PAL: SetDutyCycle,
    PBH: SetDutyCycle,
    PBL: SetDutyCycle,
    PCH: SetDutyCycle,
    PCL: SetDutyCycle,
    EN: OutputPin,
{
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        let v_limit = self.params.voltage_limit;
        let v_supply = self.params.voltage_power_supply;
        let ua = ua.clamp(-v_limit, v_limit);
        let ub = ub.clamp(-v_limit, v_limit);
        let uc = uc.clamp(-v_limit, v_limit);

        let max_ah = self.pwm_a_h.max_duty_cycle() as f32;
        let max_al = self.pwm_a_l.max_duty_cycle() as f32;
        let max_bh = self.pwm_b_h.max_duty_cycle() as f32;
        let max_bl = self.pwm_b_l.max_duty_cycle() as f32;
        let max_ch = self.pwm_c_h.max_duty_cycle() as f32;
        let max_cl = self.pwm_c_l.max_duty_cycle() as f32;

        // Upstream: if Ua >= 0, pwm_a_h = duty, pwm_a_l = 0; else pwm_a_h = 0, pwm_a_l = duty
        let (dc_ah, dc_al) = if ua >= 0.0 {
            (
                ((ua / v_supply).clamp(0.0, 1.0) * max_ah) as u16,
                0,
            )
        } else {
            (
                0,
                ((ua.abs() / v_supply).clamp(0.0, 1.0) * max_al) as u16,
            )
        };

        let (dc_bh, dc_bl) = if ub >= 0.0 {
            (
                ((ub / v_supply).clamp(0.0, 1.0) * max_bh) as u16,
                0,
            )
        } else {
            (
                0,
                ((ub.abs() / v_supply).clamp(0.0, 1.0) * max_bl) as u16,
            )
        };

        let (dc_ch, dc_cl) = if uc >= 0.0 {
            (
                ((uc / v_supply).clamp(0.0, 1.0) * max_ch) as u16,
                0,
            )
        } else {
            (
                0,
                ((uc.abs() / v_supply).clamp(0.0, 1.0) * max_cl) as u16,
            )
        };

        let _ = self.pwm_a_h.set_duty_cycle(dc_ah);
        let _ = self.pwm_a_l.set_duty_cycle(dc_al);
        let _ = self.pwm_b_h.set_duty_cycle(dc_bh);
        let _ = self.pwm_b_l.set_duty_cycle(dc_bl);
        let _ = self.pwm_c_h.set_duty_cycle(dc_ch);
        let _ = self.pwm_c_l.set_duty_cycle(dc_cl);
    }

    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState, sc: PhaseState) {
        self.phase_state = [sa, sb, sc];
        if let Some(p) = self.enable.as_mut() {
            let active = self.params.enable_active_high;
            let all_on = sa == PhaseState::PHASE_ON
                && sb == PhaseState::PHASE_ON
                && sc == PhaseState::PHASE_ON;
            let _ = p.set_state(if all_on == active {
                embedded_hal::digital::PinState::High
            } else {
                embedded_hal::digital::PinState::Low
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::pin::Mock as PinMock;
    use embedded_hal_mock::eh1::pwm::{Mock as PwmMock, Transaction as PwmTransaction};

    const MAX_DUTY: u16 = 1000;

    fn pwm_empty() -> (PwmMock, PwmMock, PwmMock, PwmMock, PwmMock, PwmMock) {
        (
            PwmMock::new(&[]),
            PwmMock::new(&[]),
            PwmMock::new(&[]),
            PwmMock::new(&[]),
            PwmMock::new(&[]),
            PwmMock::new(&[]),
        )
    }

    #[test]
    fn init_sets_initialized_flag() {
        let (mut ah, mut al, mut bh, mut bl, mut ch, mut cl) = pwm_empty();
        let mut d = BLDCDriver6PWM::new(
            &mut ah, &mut al, &mut bh, &mut bl, &mut ch, &mut cl,
            None::<&mut PinMock>,
        );
        assert!(!d.params().initialized);
        d.init().unwrap();
        assert!(d.params().initialized);
        ah.done(); al.done(); bh.done(); bl.done(); ch.done(); cl.done();
    }

    #[test]
    fn set_pwm_positive_voltage_uses_h_pins() {
        let exp_ah = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp_al = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let exp_bh = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp_bl = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let exp_ch = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp_cl = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let mut ah = PwmMock::new(&exp_ah);
        let mut al = PwmMock::new(&exp_al);
        let mut bh = PwmMock::new(&exp_bh);
        let mut bl = PwmMock::new(&exp_bl);
        let mut ch = PwmMock::new(&exp_ch);
        let mut cl = PwmMock::new(&exp_cl);

        let mut d = BLDCDriver6PWM::new(
            &mut ah, &mut al, &mut bh, &mut bl, &mut ch, &mut cl,
            None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 12.0;
        d.set_pwm(6.0, 6.0, 6.0);

        ah.done(); al.done(); bh.done(); bl.done(); ch.done(); cl.done();
    }

    #[test]
    fn set_pwm_negative_voltage_uses_l_pins() {
        let exp_ah = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let exp_al = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp_bh = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let exp_bl = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let exp_ch = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let exp_cl = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let mut ah = PwmMock::new(&exp_ah);
        let mut al = PwmMock::new(&exp_al);
        let mut bh = PwmMock::new(&exp_bh);
        let mut bl = PwmMock::new(&exp_bl);
        let mut ch = PwmMock::new(&exp_ch);
        let mut cl = PwmMock::new(&exp_cl);

        let mut d = BLDCDriver6PWM::new(
            &mut ah, &mut al, &mut bh, &mut bl, &mut ch, &mut cl,
            None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 12.0;
        d.set_pwm(-6.0, -6.0, -6.0);

        ah.done(); al.done(); bh.done(); bl.done(); ch.done(); cl.done();
    }

    #[test]
    fn driver_type_is_bldc() {
        let (mut ah, mut al, mut bh, mut bl, mut ch, mut cl) = pwm_empty();
        let d = BLDCDriver6PWM::new(
            &mut ah, &mut al, &mut bh, &mut bl, &mut ch, &mut cl,
            None::<&mut PinMock>,
        );
        assert_eq!(d.driver_type(), DriverType::BLDC);
        ah.done(); al.done(); bh.done(); bl.done(); ch.done(); cl.done();
    }
}
