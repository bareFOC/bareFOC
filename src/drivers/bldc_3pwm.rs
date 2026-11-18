//! # drivers::bldc_3pwm — 3-phase BLDC driver (3 PWM, optional 3 enables)
//!
//! Ports
//! [`BLDCDriver3PWM.h`](source/Arduino-FOC/src/drivers/BLDCDriver3PWM.h)
//! and
//! [`BLDCDriver3PWM.cpp`](source/Arduino-FOC/src/drivers/BLDCDriver3PWM.cpp)
//! from Arduino-FOC.
//!
//! The `BLDCDriver3PWM` is the simplest 3-phase driver: each
//! phase has a single PWM channel, and (optionally) a
//! dedicated enable pin. The upstream supports 3 separate
//! enable pins — one per phase — so the FOC engine can
//! individually switch a phase to high-impedance via
//! [`BLDCDriver::set_phase_state`] (used during step-response
//! calibration).
//!
//! ## Voltage-to-duty scaling
//!
//! The upstream formula is
//!
//! ```text
//! Ua = clamp(Ua, 0, voltage_limit)
//! dc_a = clamp(Ua / voltage_power_supply, 0, 1)
//! ```
//!
//! i.e. voltages are clamped to `[0, voltage_limit]` and
//! then divided by the supply rail. The FOC engine has
//! already subtracted the centred-modulation offset, so
//! negative inputs are clamped to `0`. The driver does
//! **not** produce negative duty cycles.
//!
//! ## `embedded-hal` v1.0 wiring
//!
//! The struct is generic over 3 PWM channel types and 3
//! optional enable pin types. All six bounds are
//! monomorphised at construction time (no `dyn` in the FOC
//! hot loop, per `PORTING_GUIDELINES.md` §4 Pattern 1).
//!
//! ## Why `type Error = Infallible`
//!
//! The upstream `init()` returns `int` (0/1) with no error
//! details; `setPwm` / `setPhaseState` are `void`. FOC-rs
//! matches this by setting `type Error = Infallible` and
//! ignoring the per-peripheral `Result`s from
//! [`embedded_hal::pwm::SetDutyCycle::set_duty_cycle`] and
//! [`embedded_hal::digital::OutputPin::set_*`]. Production
//! HALs rarely fail on these calls; the FOC engine has no
//! recovery path for "PWM write failed" anyway.

#![allow(non_camel_case_types)]

use crate::traits::driver::{
    BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState,
};
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

/// 3-phase BLDC motor driver (3 PWM channels, up to 3 enable
/// pins).
///
/// Ports the upstream `BLDCDriver3PWM` class
/// ([`BLDCDriver3PWM.h`](source/Arduino-FOC/src/drivers/BLDCDriver3PWM.h)).
///
/// The struct is generic over:
/// - `P1`, `P2`, `P3`: PWM channel types implementing
///   [`SetDutyCycle`].
/// - `EN1`, `EN2`, `EN3`: enable pin types implementing
///   [`OutputPin`]. All three enables share the same shape
///   but may be `None` (the upstream uses `NOT_SET` to mark
///   "not connected").
pub struct BLDCDriver3PWM<P1, P2, P3, EN1, EN2, EN3>
where
    P1: SetDutyCycle,
    P2: SetDutyCycle,
    P3: SetDutyCycle,
    EN1: OutputPin,
    EN2: OutputPin,
    EN3: OutputPin,
{
    params: FocDriverParams,
    pwm_a: P1,
    pwm_b: P2,
    pwm_c: P3,
    enable_a: Option<EN1>,
    enable_b: Option<EN2>,
    enable_c: Option<EN3>,
}

impl<P1, P2, P3, EN1, EN2, EN3> BLDCDriver3PWM<P1, P2, P3, EN1, EN2, EN3>
where
    P1: SetDutyCycle,
    P2: SetDutyCycle,
    P3: SetDutyCycle,
    EN1: OutputPin,
    EN2: OutputPin,
    EN3: OutputPin,
{
    /// Construct a new 3-PWM driver. Each `enable_*` may be
    /// `None` if the corresponding enable pin is not
    /// connected.
    ///
    /// Mirrors the upstream `BLDCDriver3PWM::BLDCDriver3PWM(
    /// int phA, int phB, int phC, int en1, int en2, int en3)`
    /// constructor
    /// ([`BLDCDriver3PWM.cpp`](source/Arduino-FOC/src/drivers/BLDCDriver3PWM.cpp)
    /// lines 3–19).
    pub fn new(
        pwm_a: P1,
        pwm_b: P2,
        pwm_c: P3,
        enable_a: Option<EN1>,
        enable_b: Option<EN2>,
        enable_c: Option<EN3>,
    ) -> Self {
        Self {
            params: FocDriverParams::new(),
            pwm_a,
            pwm_b,
            pwm_c,
            enable_a,
            enable_b,
            enable_c,
        }
    }

    /// Borrow the PWM channel for phase A.
    pub fn pwm_a(&mut self) -> &mut P1 {
        &mut self.pwm_a
    }
    /// Borrow the PWM channel for phase B.
    pub fn pwm_b(&mut self) -> &mut P2 {
        &mut self.pwm_b
    }
    /// Borrow the PWM channel for phase C.
    pub fn pwm_c(&mut self) -> &mut P3 {
        &mut self.pwm_c
    }
    /// Borrow the enable pin for phase A.
    pub fn enable_a(&mut self) -> Option<&mut EN1> {
        self.enable_a.as_mut()
    }
    /// Borrow the enable pin for phase B.
    pub fn enable_b(&mut self) -> Option<&mut EN2> {
        self.enable_b.as_mut()
    }
    /// Borrow the enable pin for phase C.
    pub fn enable_c(&mut self) -> Option<&mut EN3> {
        self.enable_c.as_mut()
    }
}

impl<P1, P2, P3, EN1, EN2, EN3> FOCDriver for BLDCDriver3PWM<P1, P2, P3, EN1, EN2, EN3>
where
    P1: SetDutyCycle,
    P2: SetDutyCycle,
    P3: SetDutyCycle,
    EN1: OutputPin,
    EN2: OutputPin,
    EN3: OutputPin,
{
    type Error = core::convert::Infallible;

    /// Initialise the driver. Mirrors the upstream
    /// `BLDCDriver3PWM::init` (sets `params.initialized` and
    /// clamps `voltage_limit` to `voltage_power_supply` if
    /// not set). The FOC-rs port does not call any
    /// hardware-specific `_configure3PWM` — the user
    /// configures their PWM peripheral externally before
    /// constructing the driver.
    ///
    /// ([`BLDCDriver3PWM.cpp`](source/Arduino-FOC/src/drivers/BLDCDriver3PWM.cpp)
    /// lines 44–62).
    fn init(&mut self) -> Result<(), Self::Error> {
        // Clamp voltage_limit to voltage_power_supply if it
        // is not set (default 12.0) or out of range.
        if self.params.voltage_limit <= 0.0
            || self.params.voltage_limit > self.params.voltage_power_supply
        {
            self.params.voltage_limit = self.params.voltage_power_supply;
        }
        self.params.initialized = true;
        Ok(())
    }

    /// Drive all three enable pins to their active level
    /// (depending on `enable_active_high`) and zero the
    /// PWM outputs.
    ///
    /// Mirrors upstream `BLDCDriver3PWM::enable`
    /// ([`BLDCDriver3PWM.cpp`](source/Arduino-FOC/src/drivers/BLDCDriver3PWM.cpp)
    /// lines 22–29).
    fn enable(&mut self) {
        let active = self.params.enable_active_high;
        if let Some(p) = self.enable_a.as_mut() {
            let _ = if active { p.set_high() } else { p.set_low() };
        }
        if let Some(p) = self.enable_b.as_mut() {
            let _ = if active { p.set_high() } else { p.set_low() };
        }
        if let Some(p) = self.enable_c.as_mut() {
            let _ = if active { p.set_high() } else { p.set_low() };
        }
        self.set_pwm(0.0, 0.0, 0.0);
    }

    /// Zero the PWM outputs and drive all enable pins to
    /// their inactive level.
    ///
    /// Mirrors upstream `BLDCDriver3PWM::disable`
    /// ([`BLDCDriver3PWM.cpp`](source/Arduino-FOC/src/drivers/BLDCDriver3PWM.cpp)
    /// lines 32–41).
    fn disable(&mut self) {
        self.set_pwm(0.0, 0.0, 0.0);
        let active = self.params.enable_active_high;
        if let Some(p) = self.enable_a.as_mut() {
            let _ = if active { p.set_low() } else { p.set_high() };
        }
        if let Some(p) = self.enable_b.as_mut() {
            let _ = if active { p.set_low() } else { p.set_high() };
        }
        if let Some(p) = self.enable_c.as_mut() {
            let _ = if active { p.set_low() } else { p.set_high() };
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

impl<P1, P2, P3, EN1, EN2, EN3> BLDCDriver for BLDCDriver3PWM<P1, P2, P3, EN1, EN2, EN3>
where
    P1: SetDutyCycle,
    P2: SetDutyCycle,
    P3: SetDutyCycle,
    EN1: OutputPin,
    EN2: OutputPin,
    EN3: OutputPin,
{
    /// Apply the three phase voltages. Each voltage is
    /// clamped to `[0, voltage_limit]`, then divided by
    /// `voltage_power_supply` and re-clamped to `[0, 1]`
    /// before being scaled to the PWM channel's
    /// `max_duty_cycle()`.
    ///
    /// Mirrors upstream `BLDCDriver3PWM::setPwm`
    /// ([`BLDCDriver3PWM.cpp`](source/Arduino-FOC/src/drivers/BLDCDriver3PWM.cpp)
    /// lines 77–92).
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        let v_limit = self.params.voltage_limit;
        let v_supply = self.params.voltage_power_supply;
        let ua = ua.clamp(0.0, v_limit);
        let ub = ub.clamp(0.0, v_limit);
        let uc = uc.clamp(0.0, v_limit);
        let max_a = self.pwm_a.max_duty_cycle() as f32;
        let max_b = self.pwm_b.max_duty_cycle() as f32;
        let max_c = self.pwm_c.max_duty_cycle() as f32;
        let dc_a = ((ua / v_supply).clamp(0.0, 1.0) * max_a) as u16;
        let dc_b = ((ub / v_supply).clamp(0.0, 1.0) * max_b) as u16;
        let dc_c = ((uc / v_supply).clamp(0.0, 1.0) * max_c) as u16;
        let _ = self.pwm_a.set_duty_cycle(dc_a);
        let _ = self.pwm_b.set_duty_cycle(dc_b);
        let _ = self.pwm_c.set_duty_cycle(dc_c);
    }

    /// Override the per-phase output state. Only effective
    /// if **all three** enable pins are present (matches
    /// the upstream `if (_isset(enableA_pin) && _isset(...)`
    /// guard).
    ///
    /// Mirrors upstream `BLDCDriver3PWM::setPhaseState`
    /// ([`BLDCDriver3PWM.cpp`](source/Arduino-FOC/src/drivers/BLDCDriver3PWM.cpp)
    /// lines 67–74).
    fn set_phase_state(
        &mut self,
        sa: PhaseState,
        sb: PhaseState,
        sc: PhaseState,
    ) {
        if let (Some(pin_a), Some(pin_b), Some(pin_c)) = (
            self.enable_a.as_mut(),
            self.enable_b.as_mut(),
            self.enable_c.as_mut(),
        ) {
            let active = self.params.enable_active_high;
            let drive = |s: PhaseState| -> bool {
                s == PhaseState::PHASE_ON
            };
            let _ = pin_a.set_state(
                if drive(sa) == active {
                    embedded_hal::digital::PinState::High
                } else {
                    embedded_hal::digital::PinState::Low
                },
            );
            let _ = pin_b.set_state(
                if drive(sb) == active {
                    embedded_hal::digital::PinState::High
                } else {
                    embedded_hal::digital::PinState::Low
                },
            );
            let _ = pin_c.set_state(
                if drive(sc) == active {
                    embedded_hal::digital::PinState::High
                } else {
                    embedded_hal::digital::PinState::Low
                },
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::digital::{OutputPin, PinState};
    use embedded_hal_mock::eh1::pin::{Mock as PinMock, State as PinStateMock, Transaction as PinTransaction};
    use embedded_hal_mock::eh1::pwm::{Mock as PwmMock, Transaction as PwmTransaction};

    const MAX_DUTY: u16 = 1000;

    fn pwm_empty() -> (PwmMock, PwmMock, PwmMock) {
        (PwmMock::new(&[]), PwmMock::new(&[]), PwmMock::new(&[]))
    }

    fn pwm_zeroed() -> (PwmMock, PwmMock, PwmMock) {
        let exp = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        (PwmMock::new(&exp), PwmMock::new(&exp), PwmMock::new(&exp))
    }

    // -----------------------------------------------------------------
    // init
    // -----------------------------------------------------------------
    #[test]
    fn init_sets_initialized_flag() {
        let (mut a, mut b, mut c) = pwm_empty();
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            None::<&mut PinMock>, None::<&mut PinMock>, None::<&mut PinMock>,
        );
        assert!(!d.params().initialized);
        d.init().unwrap();
        assert!(d.params().initialized);
        a.done(); b.done(); c.done();
    }

    #[test]
    fn init_clamps_voltage_limit_to_supply() {
        let (mut a, mut b, mut c) = pwm_empty();
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            None::<&mut PinMock>, None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 24.0;
        d.init().unwrap();
        assert_eq!(d.params().voltage_limit, 12.0);
        a.done(); b.done(); c.done();
    }

    #[test]
    fn init_clamps_unset_voltage_limit() {
        let (mut a, mut b, mut c) = pwm_empty();
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            None::<&mut PinMock>, None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 0.0;
        d.init().unwrap();
        assert_eq!(d.params().voltage_limit, 12.0);
        a.done(); b.done(); c.done();
    }

    // -----------------------------------------------------------------
    // set_pwm — voltage scaling
    // -----------------------------------------------------------------
    #[test]
    fn set_pwm_scales_to_supply() {
        let expectations_a = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let expectations_b = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let expectations_c = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let mut a = PwmMock::new(&expectations_a);
        let mut b = PwmMock::new(&expectations_b);
        let mut c = PwmMock::new(&expectations_c);
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            None::<&mut PinMock>, None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 12.0;
        d.set_pwm(6.0, 6.0, 6.0);
        a.done(); b.done(); c.done();
    }

    #[test]
    fn set_pwm_clamps_to_max_duty() {
        let expectations_a = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(1000),
        ];
        let expectations_b = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(1000),
        ];
        let expectations_c = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(1000),
        ];
        let mut a = PwmMock::new(&expectations_a);
        let mut b = PwmMock::new(&expectations_b);
        let mut c = PwmMock::new(&expectations_c);
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            None::<&mut PinMock>, None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 12.0;
        d.set_pwm(24.0, 24.0, 24.0);
        a.done(); b.done(); c.done();
    }

    #[test]
    fn set_pwm_clamps_to_zero() {
        let expectations_a = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let expectations_b = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let expectations_c = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let mut a = PwmMock::new(&expectations_a);
        let mut b = PwmMock::new(&expectations_b);
        let mut c = PwmMock::new(&expectations_c);
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            None::<&mut PinMock>, None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 12.0;
        d.set_pwm(-6.0, -6.0, -6.0);
        a.done(); b.done(); c.done();
    }

    #[test]
    fn set_pwm_clamps_to_voltage_limit() {
        let expectations_a = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let expectations_b = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(0),
        ];
        let expectations_c = [
            PwmTransaction::max_duty_cycle(MAX_DUTY),
            PwmTransaction::set_duty_cycle(500),
        ];
        let mut a = PwmMock::new(&expectations_a);
        let mut b = PwmMock::new(&expectations_b);
        let mut c = PwmMock::new(&expectations_c);
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            None::<&mut PinMock>, None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().voltage_power_supply = 12.0;
        d.params_mut().voltage_limit = 6.0;
        d.set_pwm(8.0, 0.0, 12.0);
        a.done(); b.done(); c.done();
    }

    // -----------------------------------------------------------------
    // enable / disable — enable pin polarity
    // -----------------------------------------------------------------
    #[test]
    fn enable_drives_active_high_pins_high() {
        let (mut a, mut b, mut c) = pwm_zeroed();
        let pin_expectations_a = [PinTransaction::set(PinStateMock::High)];
        let pin_expectations_b = [PinTransaction::set(PinStateMock::High)];
        let pin_expectations_c = [PinTransaction::set(PinStateMock::High)];
        let mut pin_a = PinMock::new(&pin_expectations_a);
        let mut pin_b = PinMock::new(&pin_expectations_b);
        let mut pin_c = PinMock::new(&pin_expectations_c);

        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            Some(&mut pin_a), Some(&mut pin_b), Some(&mut pin_c),
        );
        d.enable();
        pin_a.done(); pin_b.done(); pin_c.done();
        a.done(); b.done(); c.done();
    }

    #[test]
    fn enable_active_low_drives_pins_low() {
        let (mut a, mut b, mut c) = pwm_zeroed();
        let pin_expectations_a = [PinTransaction::set(PinStateMock::Low)];
        let pin_expectations_b = [PinTransaction::set(PinStateMock::Low)];
        let pin_expectations_c = [PinTransaction::set(PinStateMock::Low)];
        let mut pin_a = PinMock::new(&pin_expectations_a);
        let mut pin_b = PinMock::new(&pin_expectations_b);
        let mut pin_c = PinMock::new(&pin_expectations_c);

        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            Some(&mut pin_a), Some(&mut pin_b), Some(&mut pin_c),
        );
        d.params_mut().enable_active_high = false;
        d.enable();
        pin_a.done(); pin_b.done(); pin_c.done();
        a.done(); b.done(); c.done();
    }

    #[test]
    fn disable_drives_active_high_pins_low() {
        let (mut a, mut b, mut c) = pwm_zeroed();
        let pin_expectations_a = [PinTransaction::set(PinStateMock::Low)];
        let pin_expectations_b = [PinTransaction::set(PinStateMock::Low)];
        let pin_expectations_c = [PinTransaction::set(PinStateMock::Low)];
        let mut pin_a = PinMock::new(&pin_expectations_a);
        let mut pin_b = PinMock::new(&pin_expectations_b);
        let mut pin_c = PinMock::new(&pin_expectations_c);

        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            Some(&mut pin_a), Some(&mut pin_b), Some(&mut pin_c),
        );
        d.disable();
        pin_a.done(); pin_b.done(); pin_c.done();
        a.done(); b.done(); c.done();
    }

    #[test]
    fn enable_works_without_enable_pins() {
        let (mut a, mut b, mut c) = pwm_zeroed();
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            None::<&mut PinMock>, None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.enable();
        a.done(); b.done(); c.done();
    }

    // -----------------------------------------------------------------
    // set_phase_state
    // -----------------------------------------------------------------
    #[test]
    fn set_phase_state_writes_enable_pins() {
        let (mut a, mut b, mut c) = pwm_empty();
        let pin_expectations_a = [PinTransaction::set(PinStateMock::High)];
        let pin_expectations_b = [PinTransaction::set(PinStateMock::Low)];
        let pin_expectations_c = [PinTransaction::set(PinStateMock::High)];
        let mut pin_a = PinMock::new(&pin_expectations_a);
        let mut pin_b = PinMock::new(&pin_expectations_b);
        let mut pin_c = PinMock::new(&pin_expectations_c);

        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            Some(&mut pin_a), Some(&mut pin_b), Some(&mut pin_c),
        );
        d.set_phase_state(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON);
        pin_a.done(); pin_b.done(); pin_c.done();
        a.done(); b.done(); c.done();
    }

    #[test]
    fn set_phase_state_no_op_without_all_enable_pins() {
        let (mut a, mut b, mut c) = pwm_empty();
        let pin_expectations_a: [embedded_hal_mock::eh1::pin::Transaction; 0] = [];
        let mut pin_a = PinMock::new(&pin_expectations_a);
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            Some(&mut pin_a), None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.set_phase_state(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
        pin_a.done();
        a.done(); b.done(); c.done();
    }

    // -----------------------------------------------------------------
    // driver_type
    // -----------------------------------------------------------------
    #[test]
    fn driver_type_is_bldc() {
        let (mut a, mut b, mut c) = pwm_empty();
        let d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            None::<&mut PinMock>, None::<&mut PinMock>, None::<&mut PinMock>,
        );
        assert_eq!(d.driver_type(), DriverType::BLDC);
        a.done(); b.done(); c.done();
    }

    #[test]
    fn params_and_params_mut_round_trip() {
        let (mut a, mut b, mut c) = pwm_empty();
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            None::<&mut PinMock>, None::<&mut PinMock>, None::<&mut PinMock>,
        );
        d.params_mut().pwm_frequency = 30_000;
        d.params_mut().voltage_limit = 8.0;
        assert_eq!(d.params().pwm_frequency, 30_000);
        assert_eq!(d.params().voltage_limit, 8.0);
        a.done(); b.done(); c.done();
    }

    // -----------------------------------------------------------------
    // PWM / pin accessors
    // -----------------------------------------------------------------
    // PWM / pin accessors
    // -----------------------------------------------------------------
    #[test]
    fn pwm_and_enable_accessors() {
        let (mut a, mut b, mut c) = pwm_empty();
        let pin_expectations = [PinTransaction::set(PinStateMock::High)];
        let mut pin_a = PinMock::new(&pin_expectations);
        let mut d = BLDCDriver3PWM::new(
            &mut a, &mut b, &mut c,
            Some(&mut pin_a), None::<&mut PinMock>, None::<&mut PinMock>,
        );
        let _: &mut PwmMock = d.pwm_a();
        let _: &mut PwmMock = d.pwm_b();
        let _: &mut PwmMock = d.pwm_c();
        assert!(d.enable_a().is_some());
        assert!(d.enable_b().is_none());
        assert!(d.enable_c().is_none());
        let pin = d.enable_a().unwrap();
        let _ = pin.set_high();
        pin_a.done();
        a.done(); b.done(); c.done();
    }

    #[test]
    fn pin_state_from_bool() {
        assert_eq!(PinState::from(false), PinState::Low);
        assert_eq!(PinState::from(true), PinState::High);
    }
}
