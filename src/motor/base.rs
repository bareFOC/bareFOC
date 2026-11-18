//! # motor::base — shared FOC engine
//!
//! Ports [`FOCMotor.h`](source/Arduino-FOC/src/common/base_classes/FOCMotor.h)
//! and [`FOCMotor.cpp`](source/Arduino-FOC/src/common/base_classes/FOCMotor.cpp)
//! from Arduino-FOC.
//!
//! The upstream class is split into two Rust types:
//!
//! - [`MotorConfig<C>`] — a pure-data struct that owns the ~50 state
//!   / config / status / timing fields, the four `update*Limit`
//!   helpers, the two `update*ControlType` helpers, and the two
//!   open-loop motion helpers. These are all pure data
//!   manipulations: no polymorphic dispatch on the sensor / driver
//!   / current-sense, so they live on a struct instead of on a
//!   trait.
//! - [`FOCMotor<S, D, CS, C>`] — a trait that carries the
//!   sensor / driver / current-sense-aware algorithms:
//!   `shaft_angle`, `shaft_velocity`, `electrical_angle`,
//!   `init_foc`, `loop_foc`, `move_motor`, `characterise_motor`,
//!   `tune_current_controller`, `estimate_bemf`, and the three
//!   `link_*` setters. The trait is generic over four type
//!   parameters so the compiler can monomorphise the FOC hot
//!   loop — see `PORTING_GUIDELINES.md` §4 Pattern 1.
//!
//! ## Why four generic parameters?
//!
//! The four-arity `FOCMotor<S, D, CS, C>` is forced on us by:
//!
//! - `Sensor<C>` is generic over the clock type (see
//!   `traits::sensor::Sensor`).
//! - `CurrentSense` is **not** object-safe: its `link_driver`
//!   default takes a generic `D: FOCDriver`.
//! - The concrete motor's `set_phase_voltage` calls
//!   `D::set_pwm(ua, ub, uc)` (or `set_pwm(ua, ub)` for
//!   2-phase), so the concrete motor must bound on
//!   `D: BLDCDriver` or `D: StepperDriver`.
//! - The 4 PIDs and 4 LPFs in [`MotorConfig`] are generic over
//!   `C: Clock` for the adaptive-mode timing.
//!
//! ## LPF / PID sample time
//!
//! The four `LowPassFilter` controllers in `MotorConfig` are
//! seeded with a fixed sample time of `1 ms` (instead of the
//! upstream adaptive `now_us()` polling). This is a deliberate
//! deviation: it keeps the trait's `shaft_velocity` signature
//! clock-free (the LPF no longer needs a `&C` per call), and the
//! resulting filter is identical for any real FOC-loop rate
//! within an order of magnitude of 1 kHz.
//!
//! ## Calibration routines
//!
//! `characterise_motor` (200 lines in upstream) is the only piece
//! that runs the long `_delay(...)` and `delayMicroseconds(...)`
//! sequences. The Rust port uses `clock.delay_ms(N)` for the
//! millisecond delays and `clock.delay_ms(risetime_us / 1000 + 1)`
//! for the microsecond ones; the default `Clock::delay_ms` body
//! is a no-op so host tests run instantly. Real hardware clocks
//! (phase 8) override the default with `cortex_m::asm::delay` /
//! `esp_hal::delay::Delay`.

#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(clippy::approx_constant, clippy::excessive_precision)]

use crate::common::defaults;
use crate::common::filter::LowPassFilter;
use crate::common::pid::PIDController;
use crate::common::time::Clock;
use crate::math::{self, _2PI, _RPM_TO_RADS, _SQRT3, NOT_SET};
use crate::motor::enums::{
    FOCModulationType, FOCMotorStatus, MotionControlType, TorqueControlType,
};
use crate::traits::current_sense::CurrentSense;
use crate::traits::driver::FOCDriver;
use crate::traits::sensor::{Direction, Sensor};

use crate::math::{DQCurrent, DQVoltage};

// ---------------------------------------------------------------------------
// MotorConfig — pure state + helpers
// ---------------------------------------------------------------------------

/// Pure data + helper methods owned by the FOC engine.
///
/// Ports the public fields of the upstream
/// [`FOCMotor`](source/Arduino-FOC/src/common/base_classes/FOCMotor.h)
/// class. The struct is generic over the clock type `C: Clock` so
/// the four PIDs and four LPFs can be monomorphised per clock
/// type with zero `dyn` dispatch in the FOC hot loop.
///
/// # Field groups
///
/// - **State** (updated each FOC iteration): `target`,
///   `feed_forward_velocity`, `shaft_angle`, `electrical_angle`,
///   `shaft_velocity`, `current_sp`, `shaft_velocity_sp`,
///   `shaft_angle_sp`, `voltage {d,q}`, `current {d,q}`,
///   `voltage_bemf`, `Ualpha`, `Ubeta`,
///   `feed_forward_current {d,q}`, `feed_forward_voltage {d,q}`.
/// - **Configuration** (set once at construction): pole pairs,
///   phase resistance / inductance, KV rating, axis inductance,
///   the three limit clamps, sensor-align voltage, index-search
///   velocity, and the per-motor `bemf_coefficient` (used by the
///   default `estimate_bemf` impl on the trait).
/// - **Status** (lifecycle / mode flags): `enabled`,
///   `motor_status`, `foc_modulation`, `modulation_centered`,
///   `torque_controller`, `controller`, `sensor_offset`,
///   `zero_electric_angle`, `sensor_direction`,
///   `pp_check_result`.
/// - **Timing**: motion-loop decimation, microsecond timestamps
///   for the loop-FOC and move execution, and the open-loop
///   timestamp used by `velocity_openloop` / `angle_openloop`.
/// - **Controllers** (8 generic fields): 4 PIDs and 4 LPFs.
#[derive(Clone)]
pub struct MotorConfig<C: Clock> {
    // ---- state ----
    /// Current target value (interpretation depends on
    /// `controller` / `torque_controller`).
    pub target: f32,
    /// Feed-forward velocity term (rad/s).
    pub feed_forward_velocity: f32,
    /// Current motor shaft angle (rad).
    pub shaft_angle: f32,
    /// Current motor electrical angle (rad, normalised to `[0, 2π)`).
    pub electrical_angle: f32,
    /// Current motor shaft velocity (rad/s).
    pub shaft_velocity: f32,
    /// Target current (q-axis).
    pub current_sp: f32,
    /// Current target velocity.
    pub shaft_velocity_sp: f32,
    /// Current target angle.
    pub shaft_angle_sp: f32,
    /// Current d / q voltage setpoint.
    pub voltage: DQVoltage,
    /// Current d / q current measurement.
    pub current: DQCurrent,
    /// Estimated back-EMF voltage (when `KV_rating` is set).
    pub voltage_bemf: f32,
    /// Phase voltage Uα used for inverse Park + Clarke.
    pub Ualpha: f32,
    /// Phase voltage Uβ used for inverse Park + Clarke.
    pub Ubeta: f32,
    /// Feed-forward current (d / q).
    pub feed_forward_current: DQCurrent,
    /// Feed-forward voltage (d / q).
    pub feed_forward_voltage: DQVoltage,

    // ---- configuration ----
    /// Sensor / motor alignment voltage (used in `init_foc`).
    pub voltage_sensor_align: f32,
    /// Target velocity for the absolute-zero index search (rad/s).
    pub velocity_index_search: f32,
    /// Motor phase resistance (Ω). `NOT_SET` until
    /// `characterise_motor` runs.
    pub phase_resistance: f32,
    /// Number of magnetic pole pairs.
    pub pole_pairs: i32,
    /// Motor KV rating (rpm/V). `NOT_SET` disables BEMF estimation.
    pub KV_rating: f32,
    /// Motor phase inductance (H) — kept for backward compat
    /// with upstream; new code should use `axis_inductance`.
    pub phase_inductance: f32,
    /// Per-axis inductance (d, q) in henries. `NOT_SET` until
    /// `characterise_motor` runs.
    pub axis_inductance: DQCurrent,
    /// Global voltage limit (V). PIDs are clamped to this value.
    pub voltage_limit: f32,
    /// Global current limit (A).
    pub current_limit: f32,
    /// Global velocity limit (rad/s). Angle / velocity PIDs are
    /// clamped to this value.
    pub velocity_limit: f32,
    /// BEMF coefficient used by the default `estimate_bemf`
    /// implementation on the trait. `_SQRT3` for BLDC motors,
    /// `_SQRT2` for stepper / hybrid stepper motors.
    pub bemf_coefficient: f32,

    // ---- status / mode ----
    /// Non-zero when the motor is enabled.
    pub enabled: i8,
    /// Lifecycle state machine.
    pub motor_status: FOCMotorStatus,
    /// Selected PWM modulation strategy.
    pub foc_modulation: FOCModulationType,
    /// Centred (1) vs pulled-to-0 (0) modulation.
    pub modulation_centered: i8,
    /// Selected torque-control mode.
    pub torque_controller: TorqueControlType,
    /// Selected motion-control mode.
    pub controller: MotionControlType,
    /// User-defined sensor zero offset (rad).
    pub sensor_offset: f32,
    /// Absolute zero electric angle (rad). `NOT_SET` until
    /// `align_sensor` runs.
    pub zero_electric_angle: f32,
    /// Sensor rotation direction.
    pub sensor_direction: Direction,
    /// Result of the pole-pair check during `align_sensor`.
    pub pp_check_result: bool,

    // ---- timing ----
    /// Motion-loop decimation factor (`0` = no decimation).
    pub motion_downsample: u32,
    /// Motion-loop decimation counter.
    pub motion_cnt: u32,
    /// Filtered `loop_foc` execution time (µs).
    pub loopfoc_time_us: u32,
    /// Filtered `move` execution time (µs).
    pub move_time_us: u32,
    /// `now_us()` at the last `loop_foc` call.
    pub last_loopfoc_timestamp_us: u32,
    /// Elapsed time of the last `loop_foc` call (µs, pre-filter).
    pub last_loopfoc_time_us: u32,
    /// `now_us()` at the last `move` call.
    pub last_move_timestamp_us: u32,
    /// Elapsed time of the last `move` call (µs, pre-filter).
    pub last_move_time_us: u32,
    /// `now_us()` at the last open-loop call (velocity /
    /// angle).
    pub open_loop_timestamp: u32,

    // ---- controllers (8 generic fields) ----
    /// Q-axis current PI controller.
    pub PID_current_q: PIDController<C>,
    /// D-axis current PI controller.
    pub PID_current_d: PIDController<C>,
    /// Velocity PI controller.
    pub PID_velocity: PIDController<C>,
    /// Angle P controller.
    pub P_angle: PIDController<C>,
    /// Q-axis current low-pass filter.
    pub LPF_current_q: LowPassFilter<C>,
    /// D-axis current low-pass filter.
    pub LPF_current_d: LowPassFilter<C>,
    /// Velocity low-pass filter.
    pub LPF_velocity: LowPassFilter<C>,
    /// Angle low-pass filter.
    pub LPF_angle: LowPassFilter<C>,
}

impl<C: Clock> MotorConfig<C> {
    /// New motor config seeded with upstream defaults.
    ///
    /// The eight controllers use the upstream `DEF_*` constants
    /// (see [`crate::common::defaults`]). All PIDs / LPFs get a
    /// fixed `Ts = 1 ms` — see the module-level docs for why.
    pub fn new(clock: &C) -> Self {
        let ts: f32 = 0.001;
        Self {
            target: 0.0,
            feed_forward_velocity: 0.0,
            shaft_angle: 0.0,
            electrical_angle: 0.0,
            shaft_velocity: 0.0,
            current_sp: 0.0,
            shaft_velocity_sp: 0.0,
            shaft_angle_sp: 0.0,
            voltage: DQVoltage { d: 0.0, q: 0.0 },
            current: DQCurrent { d: 0.0, q: 0.0 },
            voltage_bemf: 0.0,
            Ualpha: 0.0,
            Ubeta: 0.0,
            feed_forward_current: DQCurrent { d: 0.0, q: 0.0 },
            feed_forward_voltage: DQVoltage { d: 0.0, q: 0.0 },

            voltage_sensor_align: defaults::DEF_VOLTAGE_SENSOR_ALIGN,
            velocity_index_search:
                defaults::DEF_INDEX_SEARCH_TARGET_VELOCITY,
            phase_resistance: NOT_SET,
            pole_pairs: 1,
            KV_rating: NOT_SET,
            phase_inductance: NOT_SET,
            axis_inductance: DQCurrent {
                d: NOT_SET,
                q: NOT_SET,
            },
            voltage_limit: defaults::DEF_POWER_SUPPLY,
            current_limit: defaults::DEF_CURRENT_LIM,
            velocity_limit: defaults::DEF_VEL_LIM,
            bemf_coefficient: _SQRT3,

            enabled: 0,
            motor_status: FOCMotorStatus::motor_uninitialized,
            foc_modulation: FOCModulationType::SinePWM,
            modulation_centered: 1,
            torque_controller: TorqueControlType::voltage,
            controller: MotionControlType::torque,
            sensor_offset: 0.0,
            zero_electric_angle: NOT_SET,
            sensor_direction: Direction::UNKNOWN,
            pp_check_result: false,

            motion_downsample: defaults::DEF_MOTION_DOWNSMAPLE as u32,
            motion_cnt: 0,
            loopfoc_time_us: 0,
            move_time_us: 0,
            last_loopfoc_timestamp_us: 0,
            last_loopfoc_time_us: 0,
            last_move_timestamp_us: 0,
            last_move_time_us: 0,
            open_loop_timestamp: 0,

            PID_current_q: PIDController::new(
                defaults::DEF_PID_CURR_P as f32,
                defaults::DEF_PID_CURR_I,
                defaults::DEF_PID_CURR_D,
                defaults::DEF_PID_CURR_RAMP as f32,
                defaults::DEF_POWER_SUPPLY,
                ts,
                clock,
            ),
            PID_current_d: PIDController::new(
                defaults::DEF_PID_CURR_P as f32,
                defaults::DEF_PID_CURR_I,
                defaults::DEF_PID_CURR_D,
                defaults::DEF_PID_CURR_RAMP as f32,
                defaults::DEF_POWER_SUPPLY,
                ts,
                clock,
            ),
            PID_velocity: PIDController::new(
                defaults::DEF_PID_VEL_P,
                defaults::DEF_PID_VEL_I,
                defaults::DEF_PID_VEL_D,
                defaults::DEF_PID_VEL_RAMP,
                defaults::DEF_PID_VEL_LIMIT,
                ts,
                clock,
            ),
            P_angle: PIDController::new(
                defaults::DEF_P_ANGLE_P,
                0.0,
                0.0,
                0.0,
                defaults::DEF_VEL_LIM,
                ts,
                clock,
            ),
            LPF_current_q: LowPassFilter::new(
                defaults::DEF_CURR_FILTER_Tf,
                ts,
                clock,
            ),
            LPF_current_d: LowPassFilter::new(
                defaults::DEF_CURR_FILTER_Tf,
                ts,
                clock,
            ),
            LPF_velocity: LowPassFilter::new(
                defaults::DEF_VEL_FILTER_Tf,
                ts,
                clock,
            ),
            LPF_angle: LowPassFilter::new(0.0, ts, clock),
        }
    }

    // -----------------------------------------------------------------
    // update*Limit helpers — port of FOCMotor.cpp L446-477
    // -----------------------------------------------------------------

    /// Update the velocity limit. Propagates the new value to
    /// `P_angle.limit` when the controller is **not**
    /// `angle_nocascade` (in which case the angle controller is
    /// bounded by velocity limit anyway).
    pub fn update_velocity_limit(&mut self, new_velocity_limit: f32) {
        self.velocity_limit = new_velocity_limit;
        if self.controller != MotionControlType::angle_nocascade {
            self.P_angle.set_limit(math::abs(new_velocity_limit));
        }
    }

    /// Update the current limit. When `torque_controller != voltage`,
    /// the velocity PID's limit is also updated to the new value
    /// (the velocity PID output is a current setpoint), and the
    /// angle controller's limit is updated under `angle_nocascade`.
    pub fn update_current_limit(&mut self, new_current_limit: f32) {
        self.current_limit = new_current_limit;
        if self.torque_controller != TorqueControlType::voltage {
            self.PID_velocity.set_limit(new_current_limit);
            if self.controller == MotionControlType::angle_nocascade {
                self.P_angle.set_limit(new_current_limit);
            }
        }
    }

    /// Update the voltage limit. Propagates to the two current
    /// PIDs and (when the torque controller is `voltage`) to the
    /// velocity PID and the `angle_nocascade` angle P.
    pub fn update_voltage_limit(&mut self, new_voltage_limit: f32) {
        self.voltage_limit = new_voltage_limit;
        self.PID_current_q.set_limit(new_voltage_limit);
        self.PID_current_d.set_limit(new_voltage_limit);
        if self.torque_controller == TorqueControlType::voltage {
            self.PID_velocity.set_limit(new_voltage_limit);
            if self.controller == MotionControlType::angle_nocascade {
                self.P_angle.set_limit(new_voltage_limit);
            }
        }
    }

    /// Switch the torque-control type and update the related
    /// controller limits.
    pub fn update_torque_control_type(
        &mut self,
        new_torque_controller: TorqueControlType,
    ) {
        self.torque_controller = new_torque_controller;
        if new_torque_controller == TorqueControlType::voltage {
            self.update_voltage_limit(self.voltage_limit);
        } else {
            self.update_current_limit(self.current_limit);
        }
    }

    /// Switch the motion-control type and update the target
    /// value. After updating the target, all three
    /// `update*Limit` helpers are re-invoked so any
    /// controller-specific clamping takes effect.
    pub fn update_motion_control_type(
        &mut self,
        new_motion_controller: MotionControlType,
    ) {
        if self.controller == new_motion_controller {
            return;
        }
        match new_motion_controller {
            MotionControlType::angle_nocascade => {
                if self.controller == MotionControlType::angle
                    || self.controller
                        == MotionControlType::angle_openloop
                {
                    // fall through (target not reset)
                } else {
                    self.target = self.shaft_angle;
                }
            }
            MotionControlType::angle => {
                if self.controller == MotionControlType::angle_openloop
                    || self.controller
                        == MotionControlType::angle_nocascade
                {
                    // fall through
                } else {
                    self.target = self.shaft_angle;
                }
            }
            MotionControlType::angle_openloop => {
                if self.controller == MotionControlType::angle
                    || self.controller
                        == MotionControlType::angle_nocascade
                {
                    // fall through
                } else {
                    self.target = self.shaft_angle;
                }
            }
            MotionControlType::velocity => {
                if self.controller == MotionControlType::velocity_openloop {
                    // fall through
                } else {
                    self.target = 0.0;
                }
            }
            MotionControlType::velocity_openloop => {
                if self.controller == MotionControlType::velocity {
                    // fall through
                } else {
                    self.target = 0.0;
                }
            }
            MotionControlType::torque => {
                self.target = 0.0;
            }
            MotionControlType::custom => {
                self.target = 0.0;
            }
        }
        self.controller = new_motion_controller;
        self.update_velocity_limit(self.velocity_limit);
        self.update_current_limit(self.current_limit);
        self.update_voltage_limit(self.voltage_limit);
    }

    // -----------------------------------------------------------------
    // Open-loop motion helpers — port of FOCMotor.cpp L389-443
    // -----------------------------------------------------------------

    /// Open-loop velocity step. Advances `shaft_angle` by
    /// `target_velocity * Ts` (with the upstream overflow /
    /// long-pause fallback `Ts = 1e-3`).
    ///
    /// Returns the saturation value to feed into the torque
    /// controller: `voltage_limit` for `voltage`-mode torque
    /// control, `current_limit` otherwise.
    pub fn velocity_openloop(
        &mut self,
        target_velocity: f32,
        clock: &C,
    ) -> f32 {
        let now_us = clock.now_us();
        let ts = now_us.wrapping_sub(self.open_loop_timestamp) as f32
            * 1e-6_f32;
        let ts = if ts <= 0.0 || ts > 0.5 { 1e-3_f32 } else { ts };

        self.shaft_angle = math::normalize_angle(
            self.shaft_angle + target_velocity * ts,
        );
        self.shaft_velocity = target_velocity;
        self.open_loop_timestamp = now_us;

        if self.torque_controller == TorqueControlType::voltage {
            self.voltage_limit
        } else {
            self.current_limit
        }
    }

    /// Open-loop angle step. Ramps `shaft_angle` toward
    /// `target_angle` at `velocity_limit` rad/s, snapping when
    /// within one step.
    pub fn angle_openloop(
        &mut self,
        target_angle: f32,
        clock: &C,
    ) -> f32 {
        let now_us = clock.now_us();
        let ts = now_us.wrapping_sub(self.open_loop_timestamp) as f32
            * 1e-6_f32;
        let ts = if ts <= 0.0 || ts > 0.5 { 1e-3_f32 } else { ts };

        let step = math::abs(self.velocity_limit * ts);
        if math::abs(target_angle - self.shaft_angle) > step {
            let sign = math::sign(target_angle - self.shaft_angle);
            self.shaft_angle += sign as f32 * step;
            self.shaft_velocity = self.velocity_limit;
        } else {
            self.shaft_angle = target_angle;
            self.shaft_velocity = 0.0;
        }
        self.open_loop_timestamp = now_us;

        if self.torque_controller == TorqueControlType::voltage {
            self.voltage_limit
        } else {
            self.current_limit
        }
    }
}

// ---------------------------------------------------------------------------
// FOCMotor trait
// ---------------------------------------------------------------------------

/// Shared FOC engine: configuration accessors, the sensor- /
/// driver-aware algorithms, and the calibration routines.
pub trait FOCMotor<S, D, CS, C>
where
    S: Sensor<C>,
    D: FOCDriver,
    CS: CurrentSense,
    C: Clock,
{
    // ----- configuration + linked-component accessors -----

    fn cfg(&self) -> &MotorConfig<C>;
    fn cfg_mut(&mut self) -> &mut MotorConfig<C>;
    fn sensor(&self) -> Option<&S>;
    /// Mutable slot accessor for the sensor — returns a
    /// `&mut Option<S>` so callers can both read the slot
    /// (`as_ref().unwrap()`, `if let Some(...) = .as_mut()`)
    /// and replace it (`*self.sensor_mut() = Some(...)`).
    fn sensor_mut(&mut self) -> &mut Option<S>;
    fn driver(&self) -> Option<&D>;
    /// Mutable slot accessor for the driver — see
    /// [`FOCMotor::sensor_mut`] for the slot-vs-value
    /// rationale.
    fn driver_mut(&mut self) -> &mut Option<D>;
    fn current_sense(&self) -> Option<&CS>;
    /// Mutable slot accessor for the current sense — see
    /// [`FOCMotor::sensor_mut`] for the slot-vs-value
    /// rationale.
    fn current_sense_mut(&mut self) -> &mut Option<CS>;

    // ----- 4 required methods (pure virtual in upstream) -----

    fn init(&mut self) -> i32;
    fn disable(&mut self);
    fn enable(&mut self);
    fn set_phase_voltage(&mut self, uq: f32, ud: f32, angle_el: f32);

    // ----- default link helpers -----

    fn link_sensor(&mut self, sensor: S) {
        *self.sensor_mut() = Some(sensor);
    }

    fn link_driver(&mut self, driver: D) {
        *self.driver_mut() = Some(driver);
    }

    /// Default link: just store the current sense. The hybrid
    /// stepper motor overrides this to also set
    /// [`CurrentSenseBase::driver_type`](crate::traits::current_sense::CurrentSenseBase::driver_type)
    /// to
    /// [`DriverType::Hybrid`](crate::traits::driver::DriverType::Hybrid).
    fn link_current_sense(&mut self, cs: CS) {
        *self.current_sense_mut() = Some(cs);
    }

    // ----- default `update*Limit` forwarders -----
    //
    // These are pure `MotorConfig` mutations, so concrete
    // motors don't have to re-implement them. The default
    // body simply delegates to the embedded `MotorConfig`.

    fn update_voltage_limit(&mut self, new_voltage_limit: f32) {
        self.cfg_mut().update_voltage_limit(new_voltage_limit);
    }

    fn update_current_limit(&mut self, new_current_limit: f32) {
        self.cfg_mut().update_current_limit(new_current_limit);
    }

    fn update_velocity_limit(&mut self, new_velocity_limit: f32) {
        self.cfg_mut().update_velocity_limit(new_velocity_limit);
    }

    fn update_torque_control_type(
        &mut self,
        new_torque_controller: TorqueControlType,
    ) {
        self.cfg_mut()
            .update_torque_control_type(new_torque_controller);
    }

    fn update_motion_control_type(
        &mut self,
        new_motion_controller: MotionControlType,
    ) {
        self.cfg_mut()
            .update_motion_control_type(new_motion_controller);
    }

    // ----- state calculation methods (port of FOCMotor.cpp L65-81) -----

    /// Shaft angle (rad).
    fn shaft_angle(&self) -> f32 {
        let cfg = self.cfg();
        match self.sensor() {
            None => cfg.shaft_angle,
            Some(sensor) => {
                let sign = cfg.sensor_direction.sign() as f32;
                sign * sensor.get_angle() - cfg.sensor_offset
            }
        }
    }

    /// Shaft velocity (rad/s), low-pass filtered.
    ///
    /// The LPF is configured with a fixed 1 ms sample time
    /// (see the module docs), so this method doesn't need a
    /// clock argument. The [`LowPassFilter::update`] API
    /// takes `&dyn Clock`; we pass a [`NullClock`] placeholder
    /// that the LPF never consults (the fixed `Ts` branch
    /// skips the `clock.now_us()` call entirely).
    fn shaft_velocity(&mut self) -> f32 {
        let cfg_signal = self.cfg().sensor_direction.sign() as f32;
        match self.sensor_mut().as_mut() {
            None => self.cfg().shaft_velocity,
            Some(sensor) => {
                let raw = sensor.get_velocity();
                let filtered = self
                    .cfg_mut()
                    .LPF_velocity
                    .update(raw, &NullClock as &dyn Clock);
                cfg_signal * filtered
            }
        }
    }

    /// Electrical angle (rad, normalised to `[0, 2π)`).
    fn electrical_angle(&self) -> f32 {
        let cfg = self.cfg();
        match self.sensor() {
            None => cfg.electrical_angle,
            Some(sensor) => {
                let sd = cfg.sensor_direction.sign() as f32;
                let pp = cfg.pole_pairs as f32;
                let mech = sensor.get_mechanical_angle();
                math::normalize_angle(sd * pp * mech - cfg.zero_electric_angle)
            }
        }
    }

    // ----- init / loop / move (port of FOCMotor.cpp L580-770) -----

    /// Initialise the FOC algorithm.
    fn init_foc(&mut self, clock: &C) -> i32 {
        self.cfg_mut().motor_status = FOCMotorStatus::motor_calibrating;

        let mut exit_flag: i32 = 1;

        if self.sensor().is_some() {
            exit_flag *= self.align_sensor(clock);
            if let Some(sensor) = self.sensor_mut().as_mut() {
                sensor.update(clock);
            }
            self.cfg_mut().shaft_angle = self.shaft_angle();
        } else {
            if self.cfg().controller == MotionControlType::angle_openloop
                || self.cfg().controller
                    == MotionControlType::velocity_openloop
            {
                exit_flag = 1;
            } else {
                exit_flag = 0;
            }
        }

        if exit_flag != 0 {
            if let Some(cs) = self.current_sense() {
                if !cs.base().initialized {
                    self.cfg_mut().motor_status =
                        FOCMotorStatus::motor_calib_failed;
                    exit_flag = 0;
                } else {
                    let r = self.align_current_sense();
                    exit_flag *= r;
                }
            }
        }

        if exit_flag != 0 {
            self.cfg_mut().motor_status = FOCMotorStatus::motor_ready;
        } else {
            self.cfg_mut().motor_status = FOCMotorStatus::motor_calib_failed;
            self.disable();
        }
        exit_flag
    }

    /// FOC hot loop.
    fn loop_foc(&mut self, clock: &C) {
        self.update_loopfoc_time(clock);

        if self.cfg().motor_status == FOCMotorStatus::motor_calibrating {
            return;
        }

        if let Some(sensor) = self.sensor_mut().as_mut() {
            sensor.update(clock);
        }

        if self.cfg().enabled == 0 {
            return;
        }

        let controller = self.cfg().controller;
        let new_elec = if controller == MotionControlType::angle_openloop
            || controller == MotionControlType::velocity_openloop
        {
            math::electrical_angle(
                self.cfg().shaft_angle,
                self.cfg().pole_pairs,
            )
        } else {
            self.electrical_angle()
        };
        self.cfg_mut().electrical_angle = new_elec;

        let torque_controller = self.cfg().torque_controller;
        let voltage_limit = self.cfg().voltage_limit;
        let current_limit = self.cfg().current_limit;
        let ffw_v = self.cfg().feed_forward_voltage;
        let ffw_i = self.cfg().feed_forward_current;
        let phase_resistance = self.cfg().phase_resistance;
        let kv_rating = self.cfg().KV_rating;
        let pole_pairs_f = self.cfg().pole_pairs as f32;
        let shaft_velocity = self.cfg().shaft_velocity;
        let current_sp_pre = self.cfg().current_sp;

        let mut voltage_q: f32;
        let mut voltage_d: f32;

        match torque_controller {
            TorqueControlType::voltage => {
                voltage_q = math::constrain(
                    current_sp_pre,
                    -voltage_limit,
                    voltage_limit,
                ) + ffw_v.q;
                voltage_d = ffw_v.d;
            }
            TorqueControlType::estimated_current => {
                if !math::isset(phase_resistance) {
                    return;
                }
                let current_sp_clamped = math::constrain(
                    current_sp_pre,
                    -current_limit,
                    current_limit,
                ) + ffw_i.q;
                let voltage_bemf = if math::isset(kv_rating) {
                    self.estimate_bemf(shaft_velocity)
                } else {
                    0.0
                };
                let current_q_filtered = self
                    .cfg_mut()
                    .LPF_current_q
                    .update(current_sp_clamped, clock);
                voltage_q = current_q_filtered * phase_resistance
                    + voltage_bemf;
                voltage_q = math::constrain(
                    voltage_q,
                    -voltage_limit,
                    voltage_limit,
                ) + ffw_v.q;
                if math::isset(self.cfg().axis_inductance.q) {
                    voltage_d = math::constrain(
                        -current_sp_clamped
                            * shaft_velocity
                            * pole_pairs_f
                            * self.cfg().axis_inductance.q,
                        -voltage_limit,
                        voltage_limit,
                    ) + ffw_v.d;
                } else {
                    voltage_d = ffw_v.d;
                }
            }
            TorqueControlType::dc_current => {
                if self.current_sense().is_none() {
                    return;
                }
                let current_sp_clamped = math::constrain(
                    current_sp_pre,
                    -current_limit,
                    current_limit,
                ) + ffw_i.q;
                let elec_angle = self.cfg().electrical_angle;
                let current_q_raw = self
                    .current_sense_mut()
                    .as_mut()
                    .unwrap()
                    .get_dc_current(elec_angle);
                let current_q_filtered = self
                    .cfg_mut()
                    .LPF_current_q
                    .update(current_q_raw, clock);
                voltage_q = self
                    .cfg_mut()
                    .PID_current_q
                    .update(current_sp_clamped - current_q_filtered, clock)
                    + ffw_v.q;
                if math::isset(self.cfg().axis_inductance.q) {
                    voltage_d = math::constrain(
                        -current_sp_clamped
                            * shaft_velocity
                            * pole_pairs_f
                            * self.cfg().axis_inductance.q,
                        -voltage_limit,
                        voltage_limit,
                    ) + ffw_v.d;
                } else {
                    voltage_d = ffw_v.d;
                }
            }
            TorqueControlType::foc_current => {
                if self.current_sense().is_none() {
                    return;
                }
                let current_sp_clamped = math::constrain(
                    current_sp_pre,
                    -current_limit,
                    current_limit,
                ) + ffw_i.q;
                let elec_angle = self.cfg().electrical_angle;
                let dq_measured = self
                    .current_sense_mut()
                    .as_mut()
                    .unwrap()
                    .get_foc_currents(elec_angle);
                let current_q_filtered = self
                    .cfg_mut()
                    .LPF_current_q
                    .update(dq_measured.q, clock);
                let current_d_filtered = self
                    .cfg_mut()
                    .LPF_current_d
                    .update(dq_measured.d, clock);
                voltage_q = self
                    .cfg_mut()
                    .PID_current_q
                    .update(current_sp_clamped - current_q_filtered, clock);
                voltage_d = self.cfg_mut().PID_current_d.update(
                    ffw_i.d - current_d_filtered,
                    clock,
                );
                if math::isset(self.cfg().axis_inductance.q) {
                    voltage_d = math::constrain(
                        voltage_d
                            - current_sp_clamped
                                * shaft_velocity
                                * pole_pairs_f
                                * self.cfg().axis_inductance.q,
                        -voltage_limit,
                        voltage_limit,
                    );
                }
                if math::isset(self.cfg().axis_inductance.d) {
                    voltage_q = math::constrain(
                        voltage_q
                            + current_d_filtered
                                * shaft_velocity
                                * pole_pairs_f
                                * self.cfg().axis_inductance.d,
                        -voltage_limit,
                        voltage_limit,
                    );
                }
                voltage_q += ffw_v.q;
                voltage_d += ffw_v.d;
            }
        }

        self.cfg_mut().voltage = DQVoltage {
            d: voltage_d,
            q: voltage_q,
        };
        let elec_angle = self.cfg().electrical_angle;
        self.set_phase_voltage(voltage_q, voltage_d, elec_angle);
    }

    /// Outer-loop control step. Decimated by `motion_downsample`.
    fn move_motor(&mut self, target: f32, clock: &C) {
        if math::isset(target) {
            self.cfg_mut().target = target;
        }

        self.cfg_mut().motion_cnt += 1;
        if self.cfg().motion_cnt <= self.cfg().motion_downsample {
            return;
        }
        self.cfg_mut().motion_cnt = 0;

        if self.cfg().motor_status == FOCMotorStatus::motor_calibrating {
            return;
        }

        self.update_move_time(clock);

        let controller = self.cfg().controller;
        if controller != MotionControlType::angle_openloop
            && controller != MotionControlType::velocity_openloop
        {
            self.cfg_mut().shaft_angle = self.shaft_angle();
            self.cfg_mut().shaft_velocity = self.shaft_velocity();
        }

        if self.cfg().enabled == 0 {
            return;
        }

        match controller {
            MotionControlType::torque => {
                self.cfg_mut().current_sp = self.cfg().target;
            }
            MotionControlType::angle_nocascade => {
                let target = self.cfg().target;
                let shaft_angle = self.cfg().shaft_angle;
                self.cfg_mut().shaft_angle_sp = target;
                let lpf = self
                    .cfg_mut()
                    .LPF_angle
                    .update(shaft_angle, clock);
                self.cfg_mut().current_sp =
                    self.cfg_mut().P_angle.update(target - lpf, clock);
            }
            MotionControlType::angle => {
                let target = self.cfg().target;
                let shaft_angle = self.cfg().shaft_angle;
                let shaft_velocity = self.cfg().shaft_velocity;
                let velocity_limit = self.cfg().velocity_limit;
                let ffv = self.cfg().feed_forward_velocity;
                self.cfg_mut().shaft_angle_sp = target;
                let lpf = self
                    .cfg_mut()
                    .LPF_angle
                    .update(shaft_angle, clock);
                let shaft_angle_sp = self.cfg().shaft_angle_sp;
                let mut v_sp = ffv
                    + self
                        .cfg_mut()
                        .P_angle
                        .update(shaft_angle_sp - lpf, clock);
                v_sp = math::constrain(v_sp, -velocity_limit, velocity_limit);
                self.cfg_mut().shaft_velocity_sp = v_sp;
                self.cfg_mut().current_sp = self
                    .cfg_mut()
                    .PID_velocity
                    .update(v_sp - shaft_velocity, clock);
            }
            MotionControlType::velocity => {
                let target = self.cfg().target;
                let shaft_velocity = self.cfg().shaft_velocity;
                self.cfg_mut().shaft_velocity_sp = target;
                self.cfg_mut().current_sp = self
                    .cfg_mut()
                    .PID_velocity
                    .update(target - shaft_velocity, clock);
            }
            MotionControlType::velocity_openloop => {
                let target = self.cfg().target;
                self.cfg_mut().shaft_velocity_sp = target;
                let u =
                    self.cfg_mut().velocity_openloop(target, clock);
                self.cfg_mut().current_sp = u;
            }
            MotionControlType::angle_openloop => {
                let target = self.cfg().target;
                self.cfg_mut().shaft_angle_sp = target;
                let u = self.cfg_mut().angle_openloop(target, clock);
                self.cfg_mut().current_sp = u;
            }
            MotionControlType::custom => {
                // Custom user-supplied callback — deferred to
                // phase 7.
            }
        }
    }

    // ----- sensor / current-sense alignment -----

    /// Sensor alignment to electrical 0. Runs the upstream
    /// 501-iteration forward / 501-iteration reverse direction
    /// detection, the pole-pair check, and the zero-electric-angle
    /// calibration.
    fn align_sensor(&mut self, clock: &C) -> i32 {
        if self.sensor().is_none() {
            return 1;
        }

        if self.sensor().unwrap().needs_search() != 0
            && self.absolute_zero_search(clock) == 0
        {
            return 0;
        }

        let voltage_align = self.cfg().voltage_sensor_align;

        if self.cfg().sensor_direction == Direction::UNKNOWN {
            for i in 0..=500_i32 {
                let angle = math::_3PI_2 + _2PI * (i as f32) / 500.0;
                self.set_phase_voltage(voltage_align, 0.0, angle);
                if let Some(s) = self.sensor_mut() {
                    s.update(clock);
                }
                clock.delay_ms(2);
            }
            if let Some(s) = self.sensor_mut() {
                s.update(clock);
            }
            let mid_angle = self.sensor().unwrap().get_angle();

            for i in (0..=500_i32).rev() {
                let angle = math::_3PI_2 + _2PI * (i as f32) / 500.0;
                self.set_phase_voltage(voltage_align, 0.0, angle);
                if let Some(s) = self.sensor_mut() {
                    s.update(clock);
                }
                clock.delay_ms(2);
            }
            if let Some(s) = self.sensor_mut() {
                s.update(clock);
            }
            let end_angle = self.sensor().unwrap().get_angle();

            clock.delay_ms(200);

            let moved = math::abs(mid_angle - end_angle);
            if moved < math::MIN_ANGLE_DETECT_MOVEMENT {
                return 0;
            } else if mid_angle < end_angle {
                self.cfg_mut().sensor_direction = Direction::CCW;
            } else {
                self.cfg_mut().sensor_direction = Direction::CW;
            }

            let pp = self.cfg().pole_pairs as f32;
            self.cfg_mut().pp_check_result =
                math::abs(moved * pp - _2PI) <= 0.5_f32;
        }

        if !math::isset(self.cfg().zero_electric_angle) {
            self.set_phase_voltage(voltage_align, 0.0, math::_3PI_2);
            clock.delay_ms(700);
            if let Some(s) = self.sensor_mut().as_mut() {
                s.update(clock);
            }
            self.cfg_mut().zero_electric_angle = 0.0;
            self.cfg_mut().zero_electric_angle = self.electrical_angle();
            clock.delay_ms(20);
            self.set_phase_voltage(0.0, 0.0, 0.0);
            clock.delay_ms(200);
        }
        1
    }

    /// Absolute zero search (for indexed encoders / magnetic
    /// sensors). The motor runs in open-loop at
    /// `velocity_index_search` until the sensor reports
    /// `needs_search == 0`.
    fn absolute_zero_search(&mut self, clock: &C) -> i32 {
        let limit_vel = self.cfg().velocity_limit;
        let limit_volt = self.cfg().voltage_limit;
        self.cfg_mut().velocity_limit = self.cfg().velocity_index_search;
        self.cfg_mut().voltage_limit = self.cfg().voltage_sensor_align;
        self.cfg_mut().shaft_angle = 0.0;
        while self.sensor().is_some()
            && self.sensor().unwrap().needs_search() != 0
            && self.cfg().shaft_angle < _2PI
        {
            self.cfg_mut().angle_openloop(1.5 * _2PI, clock);
            if let Some(s) = self.sensor_mut().as_mut() {
                s.update(clock);
            }
        }
        self.set_phase_voltage(0.0, 0.0, 0.0);
        self.cfg_mut().velocity_limit = limit_vel;
        self.cfg_mut().voltage_limit = limit_volt;
        if self.sensor().is_some() && self.sensor().unwrap().needs_search() != 0
        {
            0
        } else {
            1
        }
    }

    /// Current-sense / motor phase alignment. Defers to the
    /// current sense's `driver_align` method.
    fn align_current_sense(&mut self) -> i32 {
        let voltage_sensor_align = self.cfg().voltage_sensor_align;
        let modulation_centered = self.cfg().modulation_centered != 0;
        match self.current_sense_mut().as_mut() {
            None => 0,
            Some(cs) => {
                let r = cs
                    .driver_align(voltage_sensor_align, modulation_centered);
                if r == 0 {
                    0
                } else {
                    1
                }
            }
        }
    }

    // ----- characterise motor (port of FOCMotor.cpp L96-317) -----

    /// Measure the motor's phase resistance and per-axis
    /// inductance.
    fn characterise_motor(
        &mut self,
        voltage: f32,
        correction_factor: f32,
        clock: &C,
    ) -> i32 {
        let cs_init = self
            .current_sense()
            .map(|cs| cs.base().initialized)
            .unwrap_or(false);
        if !cs_init {
            return 1;
        }

        if voltage <= 0.0 {
            return 2;
        }
        let voltage = math::constrain(
            voltage,
            0.0,
            self.cfg().voltage_limit,
        );

        // --- resistance measurement ---
        let mut current_electric_angle = self.electrical_angle();
        self.set_phase_voltage(0.0, 0.0, current_electric_angle);
        clock.delay_ms(500);

        let zerocurrent_raw = self
            .current_sense_mut()
            .as_mut()
            .unwrap()
            .read_average_currents(1);
        let zerocurrent = {
            let cs = self.current_sense().unwrap();
            let ab = cs.get_ab_currents(zerocurrent_raw);
            cs.get_dq_currents(ab, current_electric_angle)
        };

        current_electric_angle = self.electrical_angle();
        for i in 0..100 {
            self.set_phase_voltage(
                0.0,
                voltage / 100.0 * (i as f32),
                current_electric_angle,
            );
            clock.delay_ms(3);
        }
        clock.delay_ms(10);
        let r_currents_raw = self
            .current_sense_mut()
            .as_mut()
            .unwrap()
            .read_average_currents(1);
        let r_currents = {
            let cs = self.current_sense().unwrap();
            let ab = cs.get_ab_currents(r_currents_raw);
            cs.get_dq_currents(ab, current_electric_angle)
        };
        self.set_phase_voltage(0.0, 0.0, current_electric_angle);

        if math::abs(r_currents.d - zerocurrent.d) < 0.2 {
            return 3;
        }
        let resistance =
            voltage / (correction_factor * (r_currents.d - zerocurrent.d));
        if resistance <= 0.0 {
            return 4;
        }
        clock.delay_ms(100);

        // --- inductance measurement ---
        let mut ltemp: f32 = 0.0;
        let mut ld: f32 = 0.0;
        let mut lq: f32 = 0.0;
        let mut d_electrical_angle: f32 = 0.0;

        let mut iterations: u32 = 40;
        let cycles: u32 = 3;
        let mut risetime_us: u32 = 200;
        let mut settle_us: u32 = 100_000;

        current_electric_angle += 0.5 * math::_PI;
        current_electric_angle =
            math::normalize_angle(current_electric_angle);

        for axis in 0..2_u32 {
            for i in 0..iterations {
                let mut inductanced: f32 = 0.0;
                let mut qcurrent: f32 = 0.0;
                let mut dcurrent: f32 = 0.0;
                for _ in 0..cycles {
                    let zc_raw = self
                        .current_sense_mut()
                        .as_mut()
                        .unwrap()
                        .read_average_currents(20);
                    let zc = {
                        let cs = self.current_sense().unwrap();
                        let ab = cs.get_ab_currents(zc_raw);
                        cs.get_dq_currents(ab, current_electric_angle)
                    };

                    self.set_phase_voltage(
                        0.0,
                        voltage,
                        current_electric_angle,
                    );
                    let t0 = clock.now_us();
                    clock.delay_ms((risetime_us / 1000).max(1));
                    let l_currents_raw = self
                        .current_sense_mut()
                        .as_mut()
                        .unwrap()
                        .get_phase_currents();
                    let t1 = clock.now_us();
                    self.set_phase_voltage(0.0, 0.0, current_electric_angle);
                    let l_currents = {
                        let cs = self.current_sense().unwrap();
                        let ab = cs.get_ab_currents(l_currents_raw);
                        cs.get_dq_currents(ab, current_electric_angle)
                    };
                    clock.delay_ms((settle_us / 1000).max(1));

                    if t0 > t1 {
                        continue;
                    }
                    let dt = (t1 - t0) as f32 / 1_000_000.0_f32;
                    if l_currents.d - zc.d <= 0.0
                        || (voltage - resistance * (l_currents.d - zc.d))
                            <= 0.0
                    {
                        continue;
                    }
                    let ratio = (voltage - resistance * (l_currents.d - zc.d))
                        / voltage;
                    if ratio > 0.0 {
                        inductanced += math::abs(
                            -(resistance * dt) / libm::logf(ratio),
                        ) / correction_factor;
                    }
                    qcurrent += l_currents.q - zc.q;
                    dcurrent += l_currents.d - zc.d;
                }
                qcurrent /= cycles as f32;
                dcurrent /= cycles as f32;
                inductanced /= cycles as f32;
                ltemp = if i < 2 {
                    inductanced
                } else {
                    ltemp * 0.6 + inductanced * 0.4
                };
                let timeconstant = math::abs(ltemp / resistance);
                risetime_us = math::constrain(
                    risetime_us as f32 * 0.6
                        + 0.4 * 1_000_000.0 * 0.6 * timeconstant,
                    100.0,
                    10_000.0,
                ) as u32;
                settle_us = 10 * risetime_us;

                let delta = qcurrent
                    / (math::abs(dcurrent) + math::abs(qcurrent));
                if axis != 0 {
                    qcurrent *= -1.0;
                }
                if qcurrent < 0.0 {
                    current_electric_angle += math::abs(delta);
                } else {
                    current_electric_angle -= math::abs(delta);
                }
                current_electric_angle =
                    math::normalize_angle(current_electric_angle);
                if axis != 0 {
                    d_electrical_angle = if i < 2 {
                        current_electric_angle
                    } else {
                        d_electrical_angle * 0.9
                            + current_electric_angle * 0.1
                    };
                }
            }
            current_electric_angle += 0.5 * math::_PI;
            current_electric_angle =
                math::normalize_angle(current_electric_angle);
            iterations /= 2;
            if axis == 0 {
                lq = ltemp;
            } else {
                ld = ltemp;
            }
        }

        if self.sensor().is_some() {
            let sd = self.cfg().sensor_direction.sign() as f32;
            let pp = self.cfg().pole_pairs as f32;
            let mech = self.sensor().unwrap().get_mechanical_angle();
            let a = math::normalize_angle(
                sd * pp * mech - d_electrical_angle,
            );
            let b = math::normalize_angle(
                sd * pp * mech - d_electrical_angle + math::_PI,
            );
            let _ = (a, b);
        }

        self.cfg_mut().phase_resistance = 2.0 * resistance;
        self.cfg_mut().axis_inductance = DQCurrent { d: ld, q: lq };
        self.cfg_mut().phase_inductance = (ld + lq) / 2.0;
        0
    }

    // ----- tune current controller (port of FOCMotor.cpp L538-574) -----

    /// Auto-tune the current-loop PI gains.
    fn tune_current_controller(
        &mut self,
        bandwidth: f32,
        clock: &C,
    ) -> i32 {
        if bandwidth <= 0.0 {
            return 1;
        }
        if self.cfg().loopfoc_time_us != 0
            && bandwidth > 0.5 * (1e6_f32 / self.cfg().loopfoc_time_us as f32)
        {
            return 2;
        }
        let phase_resistance = self.cfg().phase_resistance;
        let phase_inductance = self.cfg().phase_inductance;
        let axis_q = self.cfg().axis_inductance.q;
        if !math::isset(phase_resistance)
            || (!math::isset(phase_inductance) && !math::isset(axis_q))
        {
            let voltage_sensor_align = self.cfg().voltage_sensor_align;
            if self.characterise_motor(voltage_sensor_align, 1.0, clock) != 0
            {
                return 3;
            }
        } else if math::isset(phase_inductance) && !math::isset(axis_q) {
            self.cfg_mut().axis_inductance = DQCurrent {
                d: phase_inductance,
                q: phase_inductance,
            };
        }
        let r = self.cfg().phase_resistance;
        let lq = self.cfg().axis_inductance.q;
        let ld = self.cfg().axis_inductance.d;
        let two_pi_bw = _2PI * bandwidth;
        self.cfg_mut().PID_current_q.set_p(lq * two_pi_bw);
        self.cfg_mut().PID_current_q.set_i(r * two_pi_bw);
        self.cfg_mut().PID_current_d.set_p(ld * two_pi_bw);
        self.cfg_mut().PID_current_d.set_i(r * two_pi_bw);
        let lpf_tf = 1.0_f32 / (_2PI * bandwidth * 5.0_f32);
        self.cfg_mut().LPF_current_d.set_tf(lpf_tf);
        self.cfg_mut().LPF_current_q.set_tf(lpf_tf);
        0
    }

    /// Back-EMF voltage estimate (V) for a given shaft velocity
    /// (rad/s).
    fn estimate_bemf(&self, velocity: f32) -> f32 {
        let cfg = self.cfg();
        if !math::isset(cfg.KV_rating) {
            return 0.0;
        }
        velocity
            / (cfg.KV_rating * cfg.bemf_coefficient * _RPM_TO_RADS)
    }

    // ----- private timing helpers (port of FOCMotor.cpp L418-423) -----

    fn update_loopfoc_time(&mut self, clock: &C) {
        let now = clock.now_us();
        let elapsed = now.wrapping_sub(self.cfg().last_loopfoc_timestamp_us);
        self.cfg_mut().last_loopfoc_time_us = elapsed;
        self.cfg_mut().loopfoc_time_us = (0.9
            * self.cfg().loopfoc_time_us as f32
            + 0.1 * elapsed as f32) as u32;
        self.cfg_mut().last_loopfoc_timestamp_us = now;
    }

    fn update_move_time(&mut self, clock: &C) {
        let now = clock.now_us();
        let elapsed = now.wrapping_sub(self.cfg().last_move_timestamp_us);
        self.cfg_mut().last_move_time_us = elapsed;
        self.cfg_mut().move_time_us = (0.9
            * self.cfg().move_time_us as f32
            + 0.1 * elapsed as f32) as u32;
        self.cfg_mut().last_move_timestamp_us = now;
    }
}

// ---------------------------------------------------------------------------
// Null clock — used as a no-op `&C` when the LPF's fixed-Ts branch
// is taken (the embedded clock is never actually consulted).
// ---------------------------------------------------------------------------

/// A zero-sized `Clock` impl that always returns `0` for
/// `now_us` and no-ops for `delay_ms`. The motor's
/// `shaft_velocity` default method uses it as a placeholder
/// `&C` for `LowPassFilter::update` when the LPF was configured
/// with a fixed sample time (the actual `Ts` is used and the
/// clock is never consulted).
///
/// This type is `pub(crate)` so the motor unit tests can use it
/// without exposing it in the public API.
pub struct NullClock;

impl Clock for NullClock {
    fn now_us(&self) -> u32 {
        0
    }
    fn delay_ms(&self, _ms: u32) {
        // no-op (inherited default)
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::time::mock::MockClock;
    use crate::motor::enums::{
        FOCModulationType, FOCMotorStatus, MotionControlType, TorqueControlType,
    };
    use crate::traits::current_sense::mock::MockCurrentSense;
    use crate::traits::current_sense::CurrentSense;
    use crate::traits::driver::mock::MockBldcDriver;
    use crate::traits::sensor::mock::MockSensor;
    use crate::traits::sensor::Direction;

    // -----------------------------------------------------------------
    // NullClock — host-only placeholder clock
    // -----------------------------------------------------------------
    #[test]
    fn null_clock_now_us_is_zero() {
        let c = NullClock;
        assert_eq!(c.now_us(), 0);
    }

    #[test]
    fn null_clock_delay_ms_is_a_noop() {
        let c = NullClock;
        c.delay_ms(500); // should not panic / loop
    }

    // -----------------------------------------------------------------
    // MotorConfig::new — defaults match upstream
    // -----------------------------------------------------------------
    #[test]
    fn motor_config_defaults_match_upstream() {
        let clock = MockClock::new();
        let c = MotorConfig::new(&clock);
        assert_eq!(c.target, 0.0);
        assert_eq!(c.feed_forward_velocity, 0.0);
        assert_eq!(c.shaft_angle, 0.0);
        assert_eq!(c.electrical_angle, 0.0);
        assert_eq!(c.shaft_velocity, 0.0);
        assert_eq!(c.current_sp, 0.0);
        assert_eq!(c.voltage_sensor_align, 3.0);
        assert_eq!(c.velocity_index_search, 1.0);
        assert_eq!(c.pole_pairs, 1);
        assert_eq!(c.voltage_limit, 12.0);
        assert_eq!(c.current_limit, 2.0);
        assert_eq!(c.velocity_limit, 20.0);
        assert_eq!(c.bemf_coefficient, _SQRT3);
        assert_eq!(c.enabled, 0);
        assert_eq!(c.motor_status, FOCMotorStatus::motor_uninitialized);
        assert!(matches!(c.foc_modulation, FOCModulationType::SinePWM));
        assert_eq!(c.modulation_centered, 1);
        assert!(matches!(c.torque_controller, TorqueControlType::voltage));
        assert!(matches!(c.controller, MotionControlType::torque));
        assert_eq!(c.sensor_offset, 0.0);
        assert!(!math::isset(c.zero_electric_angle));
        assert!(matches!(c.sensor_direction, Direction::UNKNOWN));
        assert!(!c.pp_check_result);
    }

    // -----------------------------------------------------------------
    // update*Limit — propagates the limit into the embedded
    // controllers.
    // -----------------------------------------------------------------
    #[test]
    fn update_voltage_limit_propagates_to_current_pids() {
        let clock = MockClock::new();
        let mut c = MotorConfig::new(&clock);
        c.controller = MotionControlType::angle;
        c.torque_controller = TorqueControlType::voltage;
        c.update_voltage_limit(6.0);
        assert_eq!(c.voltage_limit, 6.0);
    }

    #[test]
    fn update_velocity_limit_propagates_to_angle_p_in_cascade() {
        let clock = MockClock::new();
        let mut c = MotorConfig::new(&clock);
        c.controller = MotionControlType::angle;
        c.update_velocity_limit(15.0);
        assert_eq!(c.velocity_limit, 15.0);
    }

    #[test]
    fn update_current_limit_skips_pids_in_voltage_mode() {
        let clock = MockClock::new();
        let mut c = MotorConfig::new(&clock);
        c.torque_controller = TorqueControlType::voltage;
        c.update_current_limit(5.0);
        assert_eq!(c.current_limit, 5.0);
    }

    // -----------------------------------------------------------------
    // update_motion_control_type — the seven case branches
    // -----------------------------------------------------------------
    #[test]
    fn update_motion_control_type_torque_resets_target() {
        let clock = MockClock::new();
        let mut c = MotorConfig::new(&clock);
        c.controller = MotionControlType::angle;
        c.target = 42.0;
        c.update_motion_control_type(MotionControlType::torque);
        assert_eq!(c.target, 0.0);
        assert!(matches!(c.controller, MotionControlType::torque));
    }

    #[test]
    fn update_motion_control_type_angle_preserves_target_from_openloop() {
        let clock = MockClock::new();
        let mut c = MotorConfig::new(&clock);
        // Start in angle_openloop with a non-zero target.
        c.controller = MotionControlType::angle_openloop;
        c.target = 1.5;
        // Switching to `angle` from `angle_openloop` should NOT
        // reset the target (upstream "fall through" case).
        c.update_motion_control_type(MotionControlType::angle);
        assert_eq!(c.target, 1.5);
        assert!(matches!(c.controller, MotionControlType::angle));
    }

    #[test]
    fn update_motion_control_type_angle_resets_target_from_velocity() {
        let clock = MockClock::new();
        let mut c = MotorConfig::new(&clock);
        c.controller = MotionControlType::torque;
        c.shaft_angle = 0.75;
        c.update_motion_control_type(MotionControlType::velocity);
        c.update_motion_control_type(MotionControlType::angle);
        // Coming from velocity, target is reset to shaft_angle.
        assert_eq!(c.target, 0.75);
    }

    #[test]
    fn update_motion_control_type_angle_nocascade_preserves_target_from_angle() {
        let clock = MockClock::new();
        let mut c = MotorConfig::new(&clock);
        // Start in `angle` with a non-zero target.
        c.controller = MotionControlType::angle;
        c.target = 2.0;
        // Switching to `angle_nocascade` from `angle` should NOT
        // reset the target (upstream "fall through" case).
        c.update_motion_control_type(MotionControlType::angle_nocascade);
        assert_eq!(c.target, 2.0);
    }

    #[test]
    fn update_motion_control_type_velocity_openloop_preserves_target_from_velocity() {
        let clock = MockClock::new();
        let mut c = MotorConfig::new(&clock);
        // Start in `velocity` with a non-zero target.
        c.controller = MotionControlType::velocity;
        c.target = 1.0;
        // Switching to `velocity_openloop` from `velocity` should
        // NOT reset the target (upstream "fall through" case).
        c.update_motion_control_type(MotionControlType::velocity_openloop);
        assert_eq!(c.target, 1.0);
    }

    #[test]
    fn update_motion_control_type_custom_resets_target() {
        let clock = MockClock::new();
        let mut c = MotorConfig::new(&clock);
        c.controller = MotionControlType::angle;
        c.target = 7.0;
        c.update_motion_control_type(MotionControlType::custom);
        assert_eq!(c.target, 0.0);
    }

    // -----------------------------------------------------------------
    // open-loop motion helpers
    // -----------------------------------------------------------------
    #[test]
    fn velocity_openloop_advances_shaft_angle() {
        let clock = MockClock::new();
        clock.set_us(1_000);
        let mut c = MotorConfig::new(&clock);
        c.open_loop_timestamp = 0;
        let u = c.velocity_openloop(10.0, &clock);
        // ts = 1ms, shaft += 10*0.001 = 0.01 rad
        assert!(c.shaft_angle > 0.0);
        assert_eq!(c.shaft_velocity, 10.0);
        // voltage-mode torque controller returns voltage_limit.
        assert_eq!(u, c.voltage_limit);
    }

    #[test]
    fn angle_openloop_clamps_to_target_when_close() {
        let clock = MockClock::new();
        clock.set_us(1_000);
        let mut c = MotorConfig::new(&clock);
        c.open_loop_timestamp = 0;
        // velocity_limit = 20 rad/s, ts = 1ms → step = 0.02 rad.
        // Place target within one step.
        c.shaft_angle = 0.0;
        let _ = c.angle_openloop(0.01, &clock);
        assert_eq!(c.shaft_angle, 0.01);
        assert_eq!(c.shaft_velocity, 0.0);
    }

    // -----------------------------------------------------------------
    // estimate_bemf — no-op when KV_rating unset
    // -----------------------------------------------------------------
    #[test]
    fn estimate_bemf_returns_zero_when_kv_unset() {
        // We need a concrete FOCMotor impl to call this. The
        // trait default is generic; we exercise it via BLDCMotor.
        use crate::motor::bldc::BLDCMotor;
        let clock = MockClock::new();
        let mut m: BLDCMotor<'_, MockSensor, MockBldcDriver, MockCurrentSense, MockClock> =
            BLDCMotor::new(1, 1.0, NOT_SET, 0.0, 0.0, &clock);
        // Without a sensor, default returns the cfg value (0).
        let v = FOCMotor::<
            MockSensor,
            MockBldcDriver,
            MockCurrentSense,
            MockClock,
        >::estimate_bemf(&m, 100.0);
        assert_eq!(v, 0.0);
        // With KV set, returns the formula value.
        m.cfg_mut().KV_rating = 100.0;
        m.cfg_mut().bemf_coefficient = _SQRT3;
        let v = FOCMotor::<
            MockSensor,
            MockBldcDriver,
            MockCurrentSense,
            MockClock,
        >::estimate_bemf(&m, 100.0);
        // v = 100 / (100 * sqrt(3) * _RPM_TO_RADS)
        let expected = 100.0 / (100.0 * _SQRT3 * _RPM_TO_RADS);
        assert!((v - expected).abs() < 1e-3);
    }

    // -----------------------------------------------------------------
    // link_* defaults via the trait — exercises the `&mut Option`
    // slot accessor pattern.
    // -----------------------------------------------------------------
    #[test]
    fn link_sensor_via_trait_default() {
        use crate::motor::bldc::BLDCMotor;
        let clock = MockClock::new();
        let mut m: BLDCMotor<'_, MockSensor, MockBldcDriver, MockCurrentSense, MockClock> =
            BLDCMotor::new(1, 1.0, NOT_SET, 0.0, 0.0, &clock);
        assert!(m.sensor().is_none());
        let s = MockSensor::new(0.0);
        m.link_sensor(s);
        assert!(m.sensor().is_some());
    }

    #[test]
    fn link_driver_via_trait_default() {
        use crate::motor::bldc::BLDCMotor;
        let clock = MockClock::new();
        let mut m: BLDCMotor<'_, MockSensor, MockBldcDriver, MockCurrentSense, MockClock> =
            BLDCMotor::new(1, 1.0, NOT_SET, 0.0, 0.0, &clock);
        assert!(m.driver().is_none());
        let d = MockBldcDriver::new();
        m.link_driver(d);
        assert!(m.driver().is_some());
    }

    #[test]
    fn link_current_sense_default_keeps_driver_type_unknown() {
        use crate::motor::bldc::BLDCMotor;
        use crate::traits::driver::DriverType;
        let clock = MockClock::new();
        let mut m: BLDCMotor<'_, MockSensor, MockBldcDriver, MockCurrentSense, MockClock> =
            BLDCMotor::new(1, 1.0, NOT_SET, 0.0, 0.0, &clock);
        let cs = MockCurrentSense::new();
        m.link_current_sense(cs);
        // Default link does NOT set the driver_type override.
        let stored = m.current_sense().unwrap();
        assert_eq!(stored.base().driver_type, DriverType::UnknownDriver);
    }

    // -----------------------------------------------------------------
    // characterise_motor — guard clauses
    // -----------------------------------------------------------------
    #[test]
    fn characterise_motor_returns_1_when_cs_uninitialised() {
        use crate::motor::bldc::BLDCMotor;
        let clock = MockClock::new();
        let mut m: BLDCMotor<'_, MockSensor, MockBldcDriver, MockCurrentSense, MockClock> =
            BLDCMotor::new(1, 1.0, NOT_SET, 0.0, 0.0, &clock);
        // No current sense linked → guard 1.
        let r = FOCMotor::<
            MockSensor,
            MockBldcDriver,
            MockCurrentSense,
            MockClock,
        >::characterise_motor(&mut m, 1.0, 1.0, &clock);
        assert_eq!(r, 1);
    }

    #[test]
    fn characterise_motor_returns_2_when_voltage_non_positive() {
        use crate::motor::bldc::BLDCMotor;
        let clock = MockClock::new();
        let mut m: BLDCMotor<'_, MockSensor, MockBldcDriver, MockCurrentSense, MockClock> =
            BLDCMotor::new(1, 1.0, NOT_SET, 0.0, 0.0, &clock);
        // Link a current sense (mock, uninitialised by default).
        let cs = MockCurrentSense::new();
        m.link_current_sense(cs);
        // initialise it
        m.current_sense_mut().as_mut().unwrap().init().unwrap();
        let r = FOCMotor::<
            MockSensor,
            MockBldcDriver,
            MockCurrentSense,
            MockClock,
        >::characterise_motor(&mut m, 0.0, 1.0, &clock);
        assert_eq!(r, 2);
    }

    // -----------------------------------------------------------------
    // shaft_angle / shaft_velocity — with and without sensor
    // -----------------------------------------------------------------
    #[test]
    fn shaft_angle_returns_cached_when_no_sensor() {
        use crate::motor::bldc::BLDCMotor;
        let clock = MockClock::new();
        let mut m: BLDCMotor<'_, MockSensor, MockBldcDriver, MockCurrentSense, MockClock> =
            BLDCMotor::new(1, 1.0, NOT_SET, 0.0, 0.0, &clock);
        m.cfg_mut().shaft_angle = 1.23;
        let a = FOCMotor::<
            MockSensor,
            MockBldcDriver,
            MockCurrentSense,
            MockClock,
        >::shaft_angle(&m);
        assert_eq!(a, 1.23);
    }
}
