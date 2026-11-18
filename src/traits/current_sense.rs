//! # traits::current_sense — abstract current-sense base class
//!
//! Ports [`CurrentSense.h`](source/Arduino-FOC/src/common/base_classes/CurrentSense.h)
//! and [`CurrentSense.cpp`](source/Arduino-FOC/src/common/base_classes/CurrentSense.cpp)
//! from Arduino-FOC.
//!
//! Upstream splits the abstraction into:
//!
//! - A base class that owns 14 public fields (`skip_align`,
//!   `driver`, `initialized`, `params`, `driver_type`,
//!   `gain_a`/`b`/`c`, `offset_ia`/`ib`/`ic`, `pinA`/`B`/`C`).
//! - A `getPhaseCurrents()` pure-virtual hook that concrete
//!   current-sense implementations override.
//! - A `getFOCCurrents()` / `getABCurrents()` /
//!   `getDQCurrents()` set of non-virtual helper methods that
//!   the base class implements (they call
//!   `getPhaseCurrents()` and the math transforms).
//! - A `driverAlign()` virtual with a default implementation
//!   that dispatches on `driver_type` to one of the three
//!   `align*Driver()` helper methods.
//! - `enable()` / `disable()` virtual with empty default
//!   bodies.
//!
//! FOC-rs follows the same shape with two twists:
//!
//! 1. The `FOCDriver* driver` raw pointer in upstream is
//!    **not** represented in [`CurrentSenseBase`]. Concrete
//!    current-sense structs in `current_sense::*` store their
//!    own typed driver reference (a generic or a trait
//!    object) where the upstream just stored a raw pointer.
//!    The base only keeps the `driver_type` enum so the
//!    Clarke-transform dispatch (BLDC vs Stepper vs Hybrid)
//!    still works.
//! 2. The trait is **not** object-safe (the `link_driver`
//!    default method takes a generic `D: FOCDriver`). This is
//!    fine because the motor bounds the current-sense generic
//!    concretely, and we never need `dyn CurrentSense` in the
//!    FOC hot loop.
//!
//! The Clarke and Park transforms used by the default
//! implementations live in [`crate::math::clarke`] and
//! [`crate::math::park`].
//!
//! ## Standalone align helpers
//!
//! The upstream `align*Driver()` helpers are pure functions
//! of the driver, the voltage, and the current-sense
//! configuration. They don't use any other instance method.
//! FOC-rs exposes them as standalone functions
//! ([`align_bldc_driver`], [`align_stepper_driver`],
//! [`align_hybrid_driver`]) so concrete current-sense
//! implementations can call them from their
//! `driver_align()` override without putting all the
//! alignment logic in the trait default.

#![allow(non_camel_case_types)]

use crate::math::{self, ABCurrent, DQCurrent, PhaseCurrent};
use crate::traits::driver::{FOCDriver, PhaseState};

// ---------------------------------------------------------------------------
// CurrentSenseBase — the public configuration / status fields
// ---------------------------------------------------------------------------

/// Current-sense configuration and runtime state. Mirrors the
/// 14 public fields of the upstream `CurrentSense` class
/// ([`CurrentSense.h`](source/Arduino-FOC/src/common/base_classes/CurrentSense.h)
/// lines 31–54), minus the raw `FOCDriver* driver` pointer
/// (which concrete impls store in a typed form on their own
/// struct).
///
/// # Fields
///
/// - [`skip_align`](Self::skip_align) — when `true`, skip the
///   driver-align procedure during `initFOC()`. Default
///   `false`.
/// - [`initialized`](Self::initialized) — `true` after
///   successful `init()`. The motor checks this before
///   enabling current sense.
/// - [`params`](Self::params) — opaque hardware-specific
///   pointer in C++ (e.g. ADC handle). In Rust this is a
///   `usize` placeholder; concrete drivers in
///   `current_sense::*` store the typed handle in a field of
///   their own struct.
/// - [`driver_type`](Self::driver_type) — copy of
///   `FOCDriver::driver_type()` for the linked driver.
///   Defaults to [`DriverType::UnknownDriver`](crate::traits::driver::DriverType::UnknownDriver)
///   (no driver attached).
/// - [`gain_a`](Self::gain_a) / [`gain_b`](Self::gain_b) /
///   [`gain_c`](Self::gain_c) — ADC-to-amps conversion
///   factor for each phase. Set by the constructor
///   (`1.0 / (shunt_resistor * gain)` for inline current
///   sense) and adjusted by `driver_align()` to flip the
///   sign of a phase if its current is inverted.
/// - [`offset_ia`](Self::offset_ia) /
///   [`offset_ib`](Self::offset_ib) /
///   [`offset_ic`](Self::offset_ic) — zero-current ADC
///   reading for each phase. Set by the calibration
///   procedure (`calibrateOffsets()` in the concrete
///   impls).
/// - [`pin_a`](Self::pin_a) / [`pin_b`](Self::pin_b) /
///   [`pin_c`](Self::pin_c) — ADC pin numbers for each
///   phase. `-1` (i32, the same as
///   [`math::_NC`](crate::math::_NC)) means "not connected".
#[derive(Clone, Copy, Debug)]
pub struct CurrentSenseBase {
    /// If `true`, the motor's `initFOC()` will not run the
    /// driver-align procedure (used when the user has
    /// pre-validated the phase-current direction at build
    /// time).
    pub skip_align: bool,

    /// `true` after `init()` has succeeded.
    pub initialized: bool,

    /// Opaque hardware-specific parameters. The concrete
    /// current-sense struct owns the typed handle; this slot
    /// is a `usize` placeholder for `Default`-completeness.
    pub params: usize,

    /// Copy of `FOCDriver::driver_type()` for the linked
    /// driver. Controls the BLDC vs Stepper vs Hybrid
    /// branch in [`CurrentSense::get_ab_currents`] and the
    /// `driver_align()` dispatch.
    pub driver_type: crate::traits::driver::DriverType,

    /// ADC → amps conversion factor for phase A.
    pub gain_a: f32,
    /// ADC → amps conversion factor for phase B.
    pub gain_b: f32,
    /// ADC → amps conversion factor for phase C. Stays
    /// zero for two-wire (no phase-C pin) configurations.
    pub gain_c: f32,

    /// Zero-current ADC reading for phase A (the centre of
    /// the ADC range for that phase).
    pub offset_ia: f32,
    /// Zero-current ADC reading for phase B.
    pub offset_ib: f32,
    /// Zero-current ADC reading for phase C. Stays zero
    /// for two-wire configurations.
    pub offset_ic: f32,

    /// ADC pin number for phase A. `-1` (i.e.
    /// [`math::_NC`](crate::math::_NC)) means "not
    /// connected".
    pub pin_a: i32,
    /// ADC pin number for phase B.
    pub pin_b: i32,
    /// ADC pin number for phase C. `-1` for two-wire.
    pub pin_c: i32,
}

impl CurrentSenseBase {
    /// New configuration with upstream defaults: every
    /// gain / offset starts at zero, no pins assigned, no
    /// driver linked, not yet initialised, and
    /// `skip_align = false`.
    pub const fn new() -> Self {
        Self {
            skip_align: false,
            initialized: false,
            params: 0,
            driver_type:
                crate::traits::driver::DriverType::UnknownDriver,
            gain_a: 0.0,
            gain_b: 0.0,
            gain_c: 0.0,
            offset_ia: 0.0,
            offset_ib: 0.0,
            offset_ic: 0.0,
            pin_a: math::_NC,
            pin_b: math::_NC,
            pin_c: math::_NC,
        }
    }
}

impl Default for CurrentSenseBase {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// CurrentSense trait
// ---------------------------------------------------------------------------

/// Abstract current-sense interface.
///
/// Ports the upstream
/// [`CurrentSense`](source/Arduino-FOC/src/common/base_classes/CurrentSense.h)
/// class. See the module-level docs for the
/// base/trait split.
///
/// # Required methods
///
/// - [`init`](CurrentSense::init) — initialise the ADC and
///   any hardware synchronisation with the driver.
/// - [`get_phase_currents`](CurrentSense::get_phase_currents) —
///   read all three phase currents, applying the gain and
///   offset of each channel.
///
/// # Default methods
///
/// The Clarke / Park / average-reading / enable / disable
/// / link_driver logic has default implementations that
/// work for any current-sense topology. Concrete impls
/// (`current_sense::inline`, `current_sense::lowside`, …)
/// override only `init` and `get_phase_currents` plus, in
/// most cases, `driver_align`.
pub trait CurrentSense {
    /// Error type for [`init`](CurrentSense::init). Concrete
    /// current-sense impls pick their own error type.
    type Error;

    /// Borrow the embedded [`CurrentSenseBase`] (read-only).
    fn base(&self) -> &CurrentSenseBase;

    /// Borrow the embedded [`CurrentSenseBase`] mutably.
    fn base_mut(&mut self) -> &mut CurrentSenseBase;

    /// Initialise the current-sense hardware (ADC, DMA,
    /// sync with PWM, …). Sets
    /// [`base.initialized`](CurrentSenseBase::initialized)
    /// to `true` on success.
    fn init(&mut self) -> Result<(), Self::Error>;

    /// Read the three phase currents, with gain and offset
    /// correction applied. The returned `c` value is `0.0`
    /// for two-wire (no phase-C pin) configurations.
    fn get_phase_currents(&mut self) -> PhaseCurrent;

    /// Link the current sense to its driver. The default
    /// implementation only records the driver type — the
    /// concrete current-sense impl is free to override and
    /// store an additional reference to the driver (e.g. to
    /// call `set_pwm` during `init` or alignment).
    fn link_driver<D: FOCDriver>(&mut self, driver: &D) {
        self.base_mut().driver_type = driver.driver_type();
    }

    /// Driver-alignment entry point. Returns:
    /// - `0` — failure
    /// - `1` — success, nothing changed
    /// - `2` — success, pins reconfigured
    /// - `3` — success, gains inverted
    /// - `4` — success, both
    ///
    /// The default implementation respects
    /// [`CurrentSenseBase::skip_align`] and
    /// [`CurrentSenseBase::initialized`] and otherwise
    /// returns `0` (because it can't dispatch to the typed
    /// `align_*_driver` helpers without a driver
    /// reference). Concrete impls override this to call one
    /// of the standalone helpers ([`align_bldc_driver`],
    /// [`align_stepper_driver`], [`align_hybrid_driver`]).
    fn driver_align(
        &mut self,
        _voltage: f32,
        _modulation_centered: bool,
    ) -> i32 {
        // Match upstream early-outs. The actual alignment
        // logic is delegated to the concrete impl via an
        // override that calls the standalone helpers in
        // this module.
        let base = self.base();
        if base.skip_align {
            return 1;
        }
        if !base.initialized {
            return 0;
        }
        0
    }

    /// Enable the current-sense hardware. Default is a
    /// no-op; concrete impls override to start the ADC /
    /// DMA.
    fn enable(&mut self) {
        // No-op, matching upstream `CurrentSense::enable()`.
    }

    /// Disable the current-sense hardware. Default is a
    /// no-op; concrete impls override to stop the ADC /
    /// DMA.
    fn disable(&mut self) {
        // No-op, matching upstream `CurrentSense::disable()`.
    }

    /// Read the DC-bus current magnitude.
    ///
    /// Ports [`CurrentSense::getDCCurrent`](source/Arduino-FOC/src/common/base_classes/CurrentSense.cpp)
    /// (lines 8–29). When `angle_el` is `0.0`, the result
    /// is the unsigned magnitude. When `angle_el` is
    /// provided, the result is signed: the sign matches
    /// the projected q-axis current.
    fn get_dc_current(&mut self, angle_el: f32) -> f32 {
        let phase = self.get_phase_currents();
        let ab = self.get_ab_currents(phase);
        // If no electrical angle is provided, the upstream
        // returns an unsigned magnitude (sign = 1). If an
        // angle is provided, the sign is determined by
        // (β·cos(θ) − α·sin(θ)) — the q-axis projection
        // without dividing by r. (Upstream does
        // `_sincos(motor_electrical_angle, &st, &ct)` and
        // uses ct = cos, st = sin.)
        let sign = if angle_el != 0.0 {
            let s = libm::sinf(angle_el);
            let c = libm::cosf(angle_el);
            if ab.beta * c - ab.alpha * s > 0.0 { 1.0 } else { -1.0 }
        } else {
            1.0
        };
        sign * math::sqrt(ab.alpha * ab.alpha + ab.beta * ab.beta)
    }

    /// Read the d/q currents in the rotating frame.
    ///
    /// Ports [`CurrentSense::getFOCCurrents`](source/Arduino-FOC/src/common/base_classes/CurrentSense.cpp)
    /// (lines 35–46). Returns
    /// `park(clarke(get_phase_currents()), angle_el)`.
    fn get_foc_currents(&mut self, angle_el: f32) -> DQCurrent {
        let phase = self.get_phase_currents();
        let ab = self.get_ab_currents(phase);
        math::park(ab, angle_el)
    }

    /// Apply the Clarke transform to the phase currents.
    ///
    /// Ports [`CurrentSense::getABCurrents`](source/Arduino-FOC/src/common/base_classes/CurrentSense.cpp)
    /// (lines 51–108). The branch selected depends on
    /// `base.driver_type`:
    ///
    /// - **Stepper:** `α = a`, `β = b` (no Clarke transform
    ///   needed).
    /// - **Hybrid:** handles the case where one of the
    ///   three currents is `0` (the missing phase is
    ///   reconstructed from `a + b + c = 0`).
    /// - **BLDC (default):** full three-phase Clarke using
    ///   the common-mode-rejection variant
    ///   (`a − mean(a,b,c)`, `b − mean(a,b,c)`).
    fn get_ab_currents(&self, current: PhaseCurrent) -> ABCurrent {
        use crate::traits::driver::DriverType;
        let dt = self.base().driver_type;
        match dt {
            DriverType::Stepper => ABCurrent {
                alpha: current.a,
                beta: current.b,
            },
            DriverType::Hybrid => {
                if current.a == 0.0 {
                    ABCurrent {
                        alpha: -current.c - current.b,
                        beta: current.b,
                    }
                } else if current.b == 0.0 {
                    ABCurrent {
                        alpha: current.a,
                        beta: -current.c - current.a,
                    }
                } else {
                    ABCurrent {
                        alpha: current.a,
                        beta: current.b,
                    }
                }
            }
            _ => {
                // BLDC / Unknown — use the upstream common-mode
                // rejection variant.
                let mid = (1.0 / 3.0) * (current.a + current.b + current.c);
                let a = current.a - mid;
                let b = current.b - mid;
                ABCurrent {
                    alpha: a,
                    beta: math::_1_SQRT3 * a + math::_2_SQRT3 * b,
                }
            }
        }
    }

    /// Apply the Park transform to αβ-frame currents at
    /// the given electrical angle. See [`crate::math::park`]
    /// for the math.
    fn get_dq_currents(
        &self,
        current: ABCurrent,
        angle_el: f32,
    ) -> DQCurrent {
        math::park(current, angle_el)
    }

    /// Average `n` successive phase-current readings. Uses
    /// the upstream `c = 0.6*c + 0.4*c1` infinite-impulse
    /// filter (matches
    /// [`CurrentSense::readAverageCurrents`](source/Arduino-FOC/src/common/base_classes/CurrentSense.cpp)
    /// lines 177–187). The sleep between samples is the
    /// caller's responsibility; this method does **not**
    /// block.
    fn read_average_currents(&mut self, n: i32) -> PhaseCurrent {
        let mut acc = self.get_phase_currents();
        for _ in 0..n {
            let next = self.get_phase_currents();
            acc.a = 0.6 * acc.a + 0.4 * next.a;
            acc.b = 0.6 * acc.b + 0.4 * next.b;
            acc.c = 0.6 * acc.c + 0.4 * next.c;
        }
        acc
    }
}

// ---------------------------------------------------------------------------
// Standalone align helpers
// ---------------------------------------------------------------------------
//
// These mirror the three `align*Driver()` member functions
// in upstream. They are exposed as free functions (not
// trait methods) because each one needs a typed driver
// reference (`BLDCDriver` / `StepperDriver`), and a
// single trait method can't be generic over multiple
// driver kinds without a lot of `Any` plumbing.
//
// Concrete current-sense impls call these from their
// `driver_align()` override. The defaults in the trait
// above don't call them; they only handle the early-out
// `skip_align` and `!initialized` cases.

/// Helper for `driver_align()` when the driver is a
/// 3-phase BLDC. Returns the upstream exit flag (1, 2, 3,
/// or 4; `0` on failure).
///
/// # Status: not yet implemented
///
/// The body is a TODO — porting the full upstream
/// algorithm (lines 199–407 of `CurrentSense.cpp`) is
/// scheduled for the concrete `InlineCurrentSense` and
/// `LowsideCurrentSense` ports in phase 6. The function
/// exists now so concrete impls can call it.
pub fn align_bldc_driver<D: FOCDriver>(
    _base: &mut CurrentSenseBase,
    _driver: &mut D,
    _voltage: f32,
    _modulation_centered: bool,
) -> i32 {
    // TODO: port the full upstream `alignBLDCDriver` body
    // (CurrentSense.cpp lines 199–407). For now return
    // "success, nothing changed" so the function compiles
    // and the trait API is fully wired.
    let _ = PhaseState::PHASE_ON;
    1
}

/// Helper for `driver_align()` when the driver is a
/// 2-phase stepper. Returns the upstream exit flag.
///
/// # Status: not yet implemented
///
/// See [`align_bldc_driver`] for the rationale.
pub fn align_stepper_driver<D: FOCDriver>(
    _base: &mut CurrentSenseBase,
    _driver: &mut D,
    _voltage: f32,
    _modulation_centered: bool,
) -> i32 {
    1
}

/// Helper for `driver_align()` when the driver is a
/// hybrid stepper / BLDC combo. Returns the upstream exit
/// flag.
///
/// # Status: not yet implemented
///
/// See [`align_bldc_driver`] for the rationale.
pub fn align_hybrid_driver<D: FOCDriver>(
    _base: &mut CurrentSenseBase,
    _driver: &mut D,
    _voltage: f32,
    _modulation_centered: bool,
) -> i32 {
    1
}

// ---------------------------------------------------------------------------
// Test-only mock
// ---------------------------------------------------------------------------

/// Test-only mock current sense. Implements
/// [`CurrentSense`] for any error type and exposes a settable
/// phase-current staging.
#[cfg(test)]
pub(crate) mod mock {
    use super::*;
    use crate::math::PhaseCurrent;
    use crate::traits::driver::{DriverType, FOCDriver};

    /// Mock current-sense error type.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    #[allow(dead_code)]
    pub enum MockCsError {
        AdcBusy,
    }

    /// Minimum-viable current-sense impl. The mock returns the
    /// same value every call to `get_phase_currents()`; tests
    /// write to `phase` to change it.
    pub struct MockCurrentSense {
        pub base: CurrentSenseBase,
        pub phase: PhaseCurrent,
    }

    impl MockCurrentSense {
        /// New mock with a zero phase-current and upstream-default
        /// base state.
        pub fn new() -> Self {
            Self {
                base: CurrentSenseBase::new(),
                phase: PhaseCurrent {
                    a: 0.0,
                    b: 0.0,
                    c: 0.0,
                },
            }
        }

        /// Set the phase currents the next `get_phase_currents()`
        /// call will return.
        pub fn set_phase(&mut self, p: PhaseCurrent) {
            self.phase = p;
        }
    }

    impl CurrentSense for MockCurrentSense {
        type Error = MockCsError;

        fn base(&self) -> &CurrentSenseBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut CurrentSenseBase {
            &mut self.base
        }

        fn init(&mut self) -> Result<(), Self::Error> {
            self.base.initialized = true;
            Ok(())
        }

        fn get_phase_currents(&mut self) -> PhaseCurrent {
            self.phase
        }
    }

    /// Reference mock BLDC driver used by current-sense tests that
    /// need to call [`CurrentSense::link_driver`]. Kept local to
    /// the mock module so the trait file's tests don't depend on
    /// `crate::traits::driver::mock`.
    pub struct MockCsLinkDriver {
        pub params: crate::traits::driver::FocDriverParams,
    }

    impl MockCsLinkDriver {
        /// New mock driver.
        pub fn new() -> Self {
            Self {
                params: crate::traits::driver::FocDriverParams::new(),
            }
        }
    }

    impl FOCDriver for MockCsLinkDriver {
        type Error = ();
        fn init(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
        fn enable(&mut self) {}
        fn disable(&mut self) {}
        fn driver_type(&self) -> DriverType {
            DriverType::BLDC
        }
        fn params(&self) -> &crate::traits::driver::FocDriverParams {
            &self.params
        }
        fn params_mut(
            &mut self,
        ) -> &mut crate::traits::driver::FocDriverParams {
            &mut self.params
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::mock::{MockCsLinkDriver, MockCurrentSense};
    use super::*;
    use crate::math::PhaseCurrent;
    use crate::traits::driver::DriverType;
    use crate::traits::sensor::Sensor;

    // -----------------------------------------------------------------
    // CurrentSenseBase — defaults match upstream (minus the
    // raw FOCDriver* driver pointer, which is omitted).
    // -----------------------------------------------------------------
    #[test]
    fn current_sense_base_defaults_match_upstream() {
        let b = CurrentSenseBase::new();
        assert!(!b.skip_align);
        assert!(!b.initialized);
        assert_eq!(b.params, 0);
        assert_eq!(b.driver_type, DriverType::UnknownDriver);
        assert_eq!(b.gain_a, 0.0);
        assert_eq!(b.gain_b, 0.0);
        assert_eq!(b.gain_c, 0.0);
        assert_eq!(b.offset_ia, 0.0);
        assert_eq!(b.offset_ib, 0.0);
        assert_eq!(b.offset_ic, 0.0);
        assert_eq!(b.pin_a, math::_NC);
        assert_eq!(b.pin_b, math::_NC);
        assert_eq!(b.pin_c, math::_NC);
    }

    #[test]
    fn init_sets_initialized_flag() {
        let mut cs = MockCurrentSense::new();
        assert!(!cs.base().initialized);
        cs.init().unwrap();
        assert!(cs.base().initialized);
    }

    #[test]
    fn link_driver_records_driver_type() {
        // Reuse a mock BLDC driver.
        let d = MockCsLinkDriver::new();
        let mut cs = MockCurrentSense::new();
        assert_eq!(cs.base().driver_type, DriverType::UnknownDriver);
        cs.link_driver(&d);
        assert_eq!(cs.base().driver_type, DriverType::BLDC);
    }

    // -----------------------------------------------------------------
    // get_ab_currents — three branches (Stepper / Hybrid / BLDC)
    // -----------------------------------------------------------------
    #[test]
    fn get_ab_currents_stepper_branch_passes_through() {
        let mut cs = MockCurrentSense::new();
        cs.base_mut().driver_type = DriverType::Stepper;
        cs.set_phase(PhaseCurrent { a: 0.7, b: -0.4, c: 0.1 });
        let phase = cs.get_phase_currents();
        let ab = cs.get_ab_currents(phase);
        assert_eq!(ab.alpha, 0.7);
        assert_eq!(ab.beta, -0.4);
    }

    // -----------------------------------------------------------------
    // get_dq_currents — must match math::park
    // -----------------------------------------------------------------
    #[test]
    fn get_dq_currents_matches_math_park() {
        let cs = MockCurrentSense::new();
        let ab = ABCurrent { alpha: 0.3, beta: -0.7 };
        let dq_via_trait = cs.get_dq_currents(ab, math::_PI_3);
        let dq_via_math = math::park(ab, math::_PI_3);
        assert!((dq_via_trait.d - dq_via_math.d).abs() < 1e-5);
        assert!((dq_via_trait.q - dq_via_math.q).abs() < 1e-5);
    }

    // -----------------------------------------------------------------
    // get_foc_currents — composition of get_ab + park
    // -----------------------------------------------------------------
    #[test]
    fn get_foc_currents_combines_clarke_and_park() {
        let mut cs = MockCurrentSense::new();
        cs.base_mut().driver_type = DriverType::BLDC;
        cs.set_phase(PhaseCurrent { a: 1.0, b: 0.0, c: -1.0 });
        let dq = cs.get_foc_currents(0.0);
        // At angle 0, Park is the identity.
        // Clarke(a=1, b=0, c=-1) gives α=1, β=1/√3.
        assert!((dq.d - 1.0).abs() < 1e-5);
        assert!((dq.q - math::_1_SQRT3).abs() < 1e-4);
    }

    // -----------------------------------------------------------------
    // get_dc_current — unsigned when angle = 0
    // -----------------------------------------------------------------
    #[test]
    fn get_dc_current_unsigned_at_zero_angle() {
        let mut cs = MockCurrentSense::new();
        cs.base_mut().driver_type = DriverType::BLDC;
        // Magnitude of the αβ vector: |(1, 1/√3)| = sqrt(4/3)
        cs.set_phase(PhaseCurrent { a: 1.0, b: 0.0, c: -1.0 });
        let i = cs.get_dc_current(0.0);
        let expected = math::sqrt(4.0 / 3.0);
        assert!((i - expected).abs() < 1e-3);
        assert!(i > 0.0);
    }

    // -----------------------------------------------------------------
    // read_average_currents — IIR averaging
    // -----------------------------------------------------------------
    #[test]
    fn read_average_currents_iir_averages() {
        // 10 successive identical readings → final value
        // must be exactly that reading.
        let mut cs = MockCurrentSense::new();
        cs.set_phase(PhaseCurrent { a: 0.5, b: 0.3, c: 0.1 });
        let avg = cs.read_average_currents(10);
        assert!((avg.a - 0.5).abs() < 1e-5);
        assert!((avg.b - 0.3).abs() < 1e-5);
        assert!((avg.c - 0.1).abs() < 1e-5);
    }

    // -----------------------------------------------------------------
    // driver_align — default: skip_align short-circuits to 1
    // -----------------------------------------------------------------
    #[test]
    fn driver_align_skip_align_returns_success() {
        let mut cs = MockCurrentSense::new();
        cs.base_mut().skip_align = true;
        assert_eq!(cs.driver_align(1.0, false), 1);
    }

    #[test]
    fn driver_align_uninitialised_returns_failure() {
        let mut cs = MockCurrentSense::new();
        // base.initialized is false by default.
        assert_eq!(cs.driver_align(1.0, false), 0);
    }

    // -----------------------------------------------------------------
    // enable / disable — default noop must compile and be
    // callable; no state change.
    // -----------------------------------------------------------------
    #[test]
    fn enable_disable_default_noop() {
        let mut cs = MockCurrentSense::new();
        cs.enable();
        cs.disable();
        // No state to assert on, but the call site must
        // compile and not panic.
    }

    // -----------------------------------------------------------------
    // Trait composition — verify that a struct can implement
    // both `Sensor` and `CurrentSense` (some drivers do
    // this in the upstream). We just compile-check here.
    // -----------------------------------------------------------------
    fn _assert_sensor_current_sense_composable<S, CS>(_: &S, _: &CS)
    where
        S: Sensor<crate::common::time::mock::MockClock>,
        CS: CurrentSense,
    {
    }
}
