//! # traits::driver — abstract driver base classes
//!
//! Ports three upstream headers from Arduino-FOC's
//! `common/base_classes/`:
//!
//! - [`FOCDriver.h`](source/Arduino-FOC/src/common/base_classes/FOCDriver.h)
//!   — the abstract base for all motor drivers (init / enable /
//!   disable / driver_type query + per-driver configuration
//!   fields). Split here into a [`FOCDriver`] trait plus a
//!   [`FocDriverParams`] struct that concrete drivers embed.
//! - [`BLDCDriver.h`](source/Arduino-FOC/src/common/base_classes/BLDCDriver.h)
//!   — adds the three-phase `set_pwm` and `set_phase_state`
//!   methods. Ported as the [`BLDCDriver`] trait.
//! - [`StepperDriver.h`](source/Arduino-FOC/src/common/base_classes/StepperDriver.h)
//!   — adds the two-phase `set_pwm` and `set_phase_state`
//!   methods. Ported as the [`StepperDriver`] trait.
//!
//! The two enums ([`DriverType`] and [`PhaseState`]) are
//! declared here because they are shared between all three
//! traits.
//!
//! # Layout in concrete drivers
//!
//! Upstream C++ derives all driver classes from `FOCDriver` and
//! keeps the configuration fields (`pwm_frequency`,
//! `voltage_power_supply`, `voltage_limit`, `initialized`,
//! `params`, `enable_active_high`) on the class itself. FOC-rs
//! follows the same shape:
//!
//! ```ignore
//! pub struct BLDCDriver3PWM<P1, P2, P3, EN> {
//!     params: FocDriverParams,
//!     dc_a: f32,
//!     dc_b: f32,
//!     dc_c: f32,
//!     pwm_a: P1,
//!     pwm_b: P2,
//!     pwm_c: P3,
//!     enable_pin: EN,
//! }
//!
//! impl<P1, P2, P3, EN> FOCDriver for BLDCDriver3PWM<P1, P2, P3, EN> {
//!     type Error = PwmError;
//!     fn init(&mut self) -> Result<(), Self::Error> { ... }
//!     fn enable(&mut self) { ... }
//!     fn disable(&mut self) { ... }
//!     fn driver_type(&self) -> DriverType { DriverType::BLDC }
//!     fn params(&self) -> &FocDriverParams { &self.params }
//!     fn params_mut(&mut self) -> &mut FocDriverParams { &mut self.params }
//! }
//! ```
//!
//! The motor generic (`BLDCMotor<S, D, CS, C>`) bounds on
//! `D: BLDCDriver` (or `StepperDriver`), which inherits from
//! `FOCDriver`, so the motor can call `D::init` / `D::enable` /
//! `D::disable` polymorphically without `dyn` — see
//! `PORTING_GUIDELINES.md` §4 Pattern 1.
//!
//! ## Driver type identification
//!
//! [`FOCDriver::driver_type`] returns a [`DriverType`] enum.
//! The motor uses it to dispatch the right `set_pwm` /
//! `set_phase_state` signature. C++ inlines the answer
//! (`type() override { return DriverType::BLDC; }`); the FOC-rs
//! port does the same — each concrete driver returns its own
//! constant.
//!
//! ## Why no `dyn FOCDriver`
//!
//! Per `PORTING_GUIDELINES.md` §4 Pattern 1, the FOC hot loop
//! never goes through a vtable. The motor is generic over the
//! concrete driver type, so all calls are monomorphized.

#![allow(non_camel_case_types)]

// ---------------------------------------------------------------------------
// Supporting enums
// ---------------------------------------------------------------------------

/// What kind of driver is connected to the motor.
///
/// Ports the upstream `DriverType` enum from
/// [`FOCDriver.h`](source/Arduino-FOC/src/common/base_classes/FOCDriver.h)
/// lines 15–20. The motor uses this to dispatch
/// driver-specific behaviour (e.g. the current-sense
/// alignment routine switches on `driver_type` to call the
/// BLDC vs. Stepper variant).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum DriverType {
    /// No driver attached (or type not yet determined).
    #[default]
    UnknownDriver = 0,
    /// Three-phase brushless DC motor driver
    /// (`BLDCDriver3PWM`, `BLDCDriver6PWM`, …).
    BLDC = 1,
    /// Two-phase stepper motor driver
    /// (`StepperDriver2PWM`, `StepperDriver4PWM`, …).
    Stepper = 2,
    /// Hybrid stepper / BLDC combo driver. Used by
    /// `HybridStepperMotor` and the corresponding current
    /// sense alignment routine.
    Hybrid = 3,
}

/// Per-phase drive state for a three-phase or two-phase
/// driver.
///
/// Ports the upstream `PhaseState` enum from
/// [`FOCDriver.h`](source/Arduino-FOC/src/common/base_classes/FOCDriver.h)
/// lines 7–12. The high-side / low-side distinction
/// (`PHASE_HI` / `PHASE_LO`) is only meaningful for
/// 6-PWM BLDC drivers; 3-PWM and stepper drivers treat the
/// phase as a single switch and use `PHASE_ON` / `PHASE_OFF`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum PhaseState {
    /// Both sides of the phase are off (high-impedance).
    #[default]
    PHASE_OFF = 0,
    /// Both sides of the phase are driven with PWM. Dead time
    /// is applied in 6-PWM mode.
    PHASE_ON = 1,
    /// Only the high side of the phase is driven with PWM
    /// (6-PWM mode only).
    PHASE_HI = 2,
    /// Only the low side of the phase is driven with PWM
    /// (6-PWM mode only).
    PHASE_LO = 3,
}

impl TryFrom<u8> for PhaseState {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(PhaseState::PHASE_OFF),
            1 => Ok(PhaseState::PHASE_ON),
            2 => Ok(PhaseState::PHASE_HI),
            3 => Ok(PhaseState::PHASE_LO),
            _ => Err(()),
        }
    }
}

// ---------------------------------------------------------------------------
// FocDriverParams — the public configuration fields
// ---------------------------------------------------------------------------

/// Per-driver configuration and status. Mirrors the public
/// fields of the upstream `FOCDriver` class
/// ([`FOCDriver.h`](source/Arduino-FOC/src/common/base_classes/FOCDriver.h)
/// lines 35–42). Concrete driver structs embed a
/// [`FocDriverParams`] field and expose it through
/// [`FOCDriver::params`] / [`FOCDriver::params_mut`].
///
/// # Fields
///
/// - [`pwm_frequency`](Self::pwm_frequency) — PWM frequency
///   in Hz. The motor's `initFOC()` typically does not touch
///   this; it is set at construction time by the
///   hardware-specific driver (`BLDCDriver3PWM::new(pwm_freq,
///   …)`).
/// - [`voltage_power_supply`](Self::voltage_power_supply) —
///   bus voltage in volts. Used by SVPWM to scale the
///   phase-voltage requests into duty cycles that respect
///   the supply rail.
/// - [`voltage_limit`](Self::voltage_limit) — per-phase
///   voltage clamp in volts. The FOC engine clips
///   `set_phase_voltage` output to ±`voltage_limit`/2
///   (centred modulation).
/// - [`initialized`](Self::initialized) — `true` after
///   `init()` has succeeded. The motor checks this before
///   enabling the driver.
/// - [`params`](Self::params) — opaque hardware-specific
///   pointer in C++ (e.g. STM32 timer handle). In Rust this
///   is just a `usize`; concrete drivers in `drivers::*`
///   store the actual HAL handle in a typed field of their
///   own struct, so this slot is a placeholder for
///   `Default`-completeness.
/// - [`enable_active_high`](Self::enable_active_high) — `true`
///   if the driver's enable pin must be driven high to
///   enable; `false` if the enable is active-low. The
///   `init()` default is `true`, matching upstream
///   (`FOCDriver.h` line 42).
#[derive(Clone, Copy, Debug)]
pub struct FocDriverParams {
    /// PWM frequency in Hz. The driver hardware is
    /// configured for this frequency at `init()`.
    pub pwm_frequency: u32,
    /// Bus voltage in volts (e.g. `12.0` for a 12 V supply).
    pub voltage_power_supply: f32,
    /// Per-phase voltage clamp in volts. The FOC engine
    /// clips the output of `set_phase_voltage` to
    /// `±voltage_limit / 2` (centred modulation).
    pub voltage_limit: f32,
    /// `true` after `init()` has succeeded. The motor
    /// checks this before calling `enable()`.
    pub initialized: bool,
    /// Opaque hardware-specific parameters. In C++ this is
    /// `void*`; in Rust we keep just a `usize` placeholder
    /// because the concrete driver struct owns the typed
    /// HAL handle itself (e.g. `TimerHandle`).
    pub params: usize,
    /// `true` if the enable pin must be driven high to
    /// enable the driver. `false` for active-low enable.
    pub enable_active_high: bool,
}

impl FocDriverParams {
    /// New configuration with upstream defaults: 25 kHz PWM,
    /// 12 V supply, 12 V limit, `initialized = false`,
    /// `params = 0`, `enable_active_high = true`.
    pub const fn new() -> Self {
        Self {
            pwm_frequency: 25_000,
            voltage_power_supply: 12.0,
            voltage_limit: 12.0,
            initialized: false,
            params: 0,
            enable_active_high: true,
        }
    }
}

impl Default for FocDriverParams {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// FOCDriver trait
// ---------------------------------------------------------------------------

/// Abstract three-phase / two-phase motor driver interface.
///
/// Ports the upstream
/// [`FOCDriver`](source/Arduino-FOC/src/common/base_classes/FOCDriver.h)
/// class. The trait carries the four cross-driver
/// methods (`init`, `enable`, `disable`, `driver_type`).
/// Concrete drivers also implement either [`BLDCDriver`] or
/// [`StepperDriver`] depending on the topology.
///
/// # Associated `Error` type
///
/// The upstream C++ `init()` returns `int` (`0` on failure,
/// `1` on success). FOC-rs follows the convention in
/// `PORTING_GUIDELINES.md` §5: every fallible operation
/// returns `Result<(), E>`. The concrete driver picks its
/// own error type via the [`FOCDriver::Error`] associated
/// type. Typical choices:
///
/// - `core::convert::Infallible` if `init()` can never fail.
/// - A custom `DriverError` enum (PWM-config failure, GPIO
///   error, …).
///
/// # Hot-loop usage
///
/// This trait is **not** in the per-iteration hot path
/// (the motor calls `init()` and `enable()` once at
/// startup; `disable()` on shutdown). The FOC hot loop goes
/// through the per-topology traits ([`BLDCDriver::set_pwm`]
/// or [`StepperDriver::set_pwm`]) and the
/// [`FOCDriver::driver_type`] query.
pub trait FOCDriver {
    /// Error type for [`init`](FOCDriver::init). Concrete
    /// drivers pick their own error type.
    type Error;

    /// Initialise the driver hardware (PWM timers, enable
    /// pin, ADC sync, …). Must set
    /// [`params.initialized`](FocDriverParams::initialized)
    /// to `true` on success.
    ///
    /// Returns `Ok(())` on success or the driver-specific
    /// error on failure.
    fn init(&mut self) -> Result<(), Self::Error>;

    /// Enable the driver's output stage. The motor calls
    /// this once after successful `init()`. Idempotent.
    fn enable(&mut self);

    /// Disable the driver's output stage (high-impedance).
    /// Idempotent. The motor calls this on shutdown.
    fn disable(&mut self);

    /// Identify which driver topology is in use. Concrete
    /// drivers return a constant (`DriverType::BLDC`,
    /// `DriverType::Stepper`, or `DriverType::Hybrid`).
    ///
    /// Renamed from upstream `type()` to avoid the Rust
    /// keyword collision — see `PORTING_GUIDELINES.md` §3.
    fn driver_type(&self) -> DriverType;

    /// Borrow the embedded configuration block.
    fn params(&self) -> &FocDriverParams;

    /// Borrow the embedded configuration block mutably.
    fn params_mut(&mut self) -> &mut FocDriverParams;
}

// ---------------------------------------------------------------------------
// BLDCDriver trait
// ---------------------------------------------------------------------------

/// Three-phase BLDC motor driver interface.
///
/// Ports the upstream
/// [`BLDCDriver`](source/Arduino-FOC/src/common/base_classes/BLDCDriver.h)
/// class. Extends [`FOCDriver`] with the two methods the
/// FOC hot loop calls every iteration:
///
/// - [`set_pwm`](BLDCDriver::set_pwm) — apply duty cycles
///   to phases A, B, C. The arguments are normalised duty
///   cycles in `[0.0, 1.0]` (centred modulation is handled
///   upstream by the FOC engine).
/// - [`set_phase_state`](BLDCDriver::set_phase_state) —
///   override the per-phase output state (off / on /
///   high-side-only / low-side-only). Used during
///   step-response calibration to set a single phase high
///   while the others stay low.
///
/// `driver_type()` returns [`DriverType::BLDC`].
pub trait BLDCDriver: FOCDriver {
    /// Set the PWM duty cycle for each of the three phases.
    /// Implementations should clip to the driver's valid
    /// duty range and apply any required dead-time.
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32);

    /// Override the per-phase output state. The arguments
    /// are the states for phases A, B, C respectively.
    fn set_phase_state(
        &mut self,
        sa: PhaseState,
        sb: PhaseState,
        sc: PhaseState,
    );
}

// ---------------------------------------------------------------------------
// StepperDriver trait
// ---------------------------------------------------------------------------

/// Two-phase stepper motor driver interface.
///
/// Ports the upstream
/// [`StepperDriver`](source/Arduino-FOC/src/common/base_classes/StepperDriver.h)
/// class. Extends [`FOCDriver`] with the two-phase versions
/// of the BLDC methods.
///
/// `driver_type()` returns [`DriverType::Stepper`].
pub trait StepperDriver: FOCDriver {
    /// Set the PWM duty cycle for each of the two phases.
    fn set_pwm(&mut self, ua: f32, ub: f32);

    /// Override the per-phase output state. The arguments
    /// are the states for phases A and B respectively.
    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState);
}

// ---------------------------------------------------------------------------
// Test-only mocks
// ---------------------------------------------------------------------------

/// Test-only mock drivers. `MockBldcDriver` and `MockStepperDriver`
/// are `pub(crate)` so the motor unit tests (`barefoc/src/motor/*`)
/// can construct drivers without pulling in real hardware.
#[cfg(test)]
pub(crate) mod mock {
    use super::{
        BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState,
        StepperDriver,
    };

    /// Mock driver error type.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    #[allow(dead_code)]
    pub enum MockError {
        NotInitialised,
    }

    /// Minimum-viable BLDC driver used by host tests. Records every
    /// `set_pwm` and `set_phase_state` call so tests can assert on
    /// the dispatch.
    pub struct MockBldcDriver {
        pub params: FocDriverParams,
        pub dc_a: f32,
        pub dc_b: f32,
        pub dc_c: f32,
        pub last_phase: Option<(PhaseState, PhaseState, PhaseState)>,
        pub init_count: u32,
    }

    impl MockBldcDriver {
        /// New mock BLDC driver with upstream-default parameters.
        pub fn new() -> Self {
            Self {
                params: FocDriverParams::new(),
                dc_a: 0.0,
                dc_b: 0.0,
                dc_c: 0.0,
                last_phase: None,
                init_count: 0,
            }
        }
    }

    impl FOCDriver for MockBldcDriver {
        type Error = MockError;

        fn init(&mut self) -> Result<(), Self::Error> {
            self.init_count += 1;
            self.params.initialized = true;
            Ok(())
        }

        fn enable(&mut self) {
            // In real hardware this would set the enable pin.
        }

        fn disable(&mut self) {
            // In real hardware this would clear the enable pin.
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

    impl BLDCDriver for MockBldcDriver {
        fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
            self.dc_a = ua;
            self.dc_b = ub;
            self.dc_c = uc;
        }

        fn set_phase_state(
            &mut self,
            sa: PhaseState,
            sb: PhaseState,
            sc: PhaseState,
        ) {
            self.last_phase = Some((sa, sb, sc));
        }
    }

    /// Minimum-viable 2-phase stepper driver used by host tests.
    pub struct MockStepperDriver {
        pub params: FocDriverParams,
        pub dc_a: f32,
        pub dc_b: f32,
    }

    impl MockStepperDriver {
        /// New mock stepper driver with upstream-default parameters.
        pub fn new() -> Self {
            Self {
                params: FocDriverParams::new(),
                dc_a: 0.0,
                dc_b: 0.0,
            }
        }
    }

    impl FOCDriver for MockStepperDriver {
        type Error = MockError;
        fn init(&mut self) -> Result<(), Self::Error> {
            self.params.initialized = true;
            Ok(())
        }
        fn enable(&mut self) {}
        fn disable(&mut self) {}
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

    impl StepperDriver for MockStepperDriver {
        fn set_pwm(&mut self, ua: f32, ub: f32) {
            self.dc_a = ua;
            self.dc_b = ub;
        }
        fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState) {
            let _ = (sa, sb);
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::mock::{MockBldcDriver, MockStepperDriver};
    use super::*;

    // -----------------------------------------------------------------
    // Enum discriminants must match upstream so any code that
    // serialises them (or compares against a C++ peer) stays
    // bit-for-bit compatible.
    // -----------------------------------------------------------------
    #[test]
    fn driver_type_discriminants_match_upstream() {
        assert_eq!(DriverType::UnknownDriver as u8, 0);
        assert_eq!(DriverType::BLDC as u8, 1);
        assert_eq!(DriverType::Stepper as u8, 2);
        assert_eq!(DriverType::Hybrid as u8, 3);
    }

    #[test]
    fn driver_type_default_is_unknown() {
        assert_eq!(DriverType::default(), DriverType::UnknownDriver);
    }

    #[test]
    fn phase_state_discriminants_match_upstream() {
        assert_eq!(PhaseState::PHASE_OFF as u8, 0);
        assert_eq!(PhaseState::PHASE_ON as u8, 1);
        assert_eq!(PhaseState::PHASE_HI as u8, 2);
        assert_eq!(PhaseState::PHASE_LO as u8, 3);
    }

    // -----------------------------------------------------------------
    // FocDriverParams — defaults match upstream
    // -----------------------------------------------------------------
    #[test]
    fn foc_driver_params_defaults_match_upstream() {
        let p = FocDriverParams::new();
        assert_eq!(p.pwm_frequency, 25_000);
        assert_eq!(p.voltage_power_supply, 12.0);
        assert_eq!(p.voltage_limit, 12.0);
        assert!(!p.initialized);
        assert_eq!(p.params, 0);
        assert!(p.enable_active_high);
    }

    // -----------------------------------------------------------------
    // Trait object-safety check: FOCDriver must be usable through
    // `&dyn FOCDriver` for non-hot-path uses (e.g. the
    // CurrentSense::link_driver helper). The trait's only
    // generic-ish piece is the associated `Error` type, which
    // is fine.
    // -----------------------------------------------------------------
    #[test]
    fn foc_driver_is_object_safe() {
        fn _accepts(_: &dyn FOCDriver<Error = super::mock::MockError>) {}
        // The test compiles if and only if FOCDriver is
        // object-safe. No runtime assertion needed.
    }

    #[test]
    fn bldc_driver_round_trip() {
        let mut d = MockBldcDriver::new();
        assert!(!d.params().initialized);
        d.init().unwrap();
        assert!(d.params().initialized);
        assert_eq!(d.init_count, 1);
        d.set_pwm(0.1, 0.2, 0.3);
        assert_eq!(d.dc_a, 0.1);
        assert_eq!(d.dc_b, 0.2);
        assert_eq!(d.dc_c, 0.3);
        d.set_phase_state(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_HI);
        assert_eq!(d.last_phase, Some((PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_HI)));
        assert_eq!(d.driver_type(), DriverType::BLDC);
    }

    #[test]
    fn bldc_driver_params_mut_round_trip() {
        let mut d = MockBldcDriver::new();
        d.params_mut().voltage_limit = 6.0;
        d.params_mut().pwm_frequency = 40_000;
        assert_eq!(d.params().voltage_limit, 6.0);
        assert_eq!(d.params().pwm_frequency, 40_000);
    }

    #[test]
    fn stepper_driver_round_trip() {
        let mut d = MockStepperDriver::new();
        d.init().unwrap();
        assert!(d.params().initialized);
        d.set_pwm(0.5, -0.5);
        assert_eq!(d.dc_a, 0.5);
        assert_eq!(d.dc_b, -0.5);
        assert_eq!(d.driver_type(), DriverType::Stepper);
    }
}
