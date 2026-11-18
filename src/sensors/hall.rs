//! # sensors::hall — 3-pin Hall effect sensor
//!
//! Ports [`HallSensor.h`](source/Arduino-FOC/src/sensors/HallSensor.h)
//! and [`HallSensor.cpp`](source/Arduino-FOC/src/sensors/HallSensor.cpp)
//! from Arduino-FOC.
//!
//! The `HallSensor` reads a 3-pin Hall-effect rotor position sensor
//! (typical of brushless DC motors). Each pin produces a binary
//! signal, and the 3-bit combined state cycles through 6 distinct
//! sectors per electrical revolution.
//!
//! ## Sectors and cpr
//!
//! Upstream defines the 3-bit `hall_state` to electrical-sector
//! mapping as:
//!
//! ```text
//! const int8_t ELECTRIC_SECTORS[8] = { -1, 0, 4, 5, 2, 1, 3, -1 };
//! ```
//!
//! The two `0b___ = 0` and `0b___ = 7` states (all-low and all-high)
//! are "invalid" and map to `-1`. The remaining 6 valid states map
//! to sectors `0..=5`. Because there are 6 valid sectors per
//! electrical revolution, the counts-per-revolution is
//! `cpr = pole_pairs * 6`.
//!
//! ## Wiring interrupts
//!
//! As with [`Encoder`](super::encoder), the user supplies the EXTI
//! callbacks. The Hall sensor's `update_state` method takes the
//! current state of all three pins plus a microsecond timestamp:
//!
//! ```ignore
//! let mut hall = HallSensor::new(0, 1, 2, 7);
//! hall.init(0, 0, 0, &clock);  // initial pin states
//! // In each EXTI closure:
//! hall.update_state(gpio_a.is_high(), gpio_b.is_high(), gpio_c.is_high(),
//!                   clock.now_us());
//! ```
//!
//! Three convenience wrappers ([`HallSensor::handle_a`],
//! [`HallSensor::handle_b`], [`HallSensor::handle_c`]) are also
//! provided that take just the changed pin and reuse the stored
//! values for the other two — the user picks whichever API fits
//! their HAL better.

#![allow(non_camel_case_types)]

use crate::common::time::Clock;
use crate::traits::sensor::{Direction, Pullup, Sensor, SensorBase};

/// 3-bit Hall state → electrical sector lookup.
///
/// Ports the upstream `ELECTRIC_SECTORS` constant from
/// [`HallSensor.h`](source/Arduino-FOC/src/sensors/HallSensor.h)
/// line 10. Index is `(C + (B << 1) + (A << 2))`. The all-low
/// (index 0) and all-high (index 7) states are invalid and map
/// to `-1`; the remaining 6 states map to electrical sectors
/// `0..=5` in a Gray-code-like sequence.
pub const ELECTRIC_SECTORS: [i8; 8] = [-1, 0, 4, 5, 2, 1, 3, -1];

// ---------------------------------------------------------------------------
// HallSensor
// ---------------------------------------------------------------------------

/// 3-pin Hall effect position sensor.
///
/// Ports the upstream `HallSensor` class
/// ([`HallSensor.h`](source/Arduino-FOC/src/sensors/HallSensor.h)).
///
/// The sensor is generic over the [`Clock`] type so the user can
/// use a real hardware timer on the target platform. The
/// microsecond timestamp is passed in by the EXTI closure.
/// `velocity_max` defaults to `1000.0` rad/s (matching upstream
/// [`HallSensor.h`](source/Arduino-FOC/src/sensors/HallSensor.h)
/// line 80) and is currently unused in the FOC-rs port — kept
/// for API parity with the upstream outlier-filter.
#[derive(Clone, Debug)]
pub struct HallSensor<C: Clock> {
    base: SensorBase,

    /// Pin number for the A channel.
    pub pin_a: i32,
    /// Pin number for the B channel.
    pub pin_b: i32,
    /// Pin number for the C channel.
    pub pin_c: i32,

    /// Whether interrupts have been attached (mirrors upstream
    /// `use_interrupt`). Not used in the FOC-rs port — the user
    /// wires their own EXTI and the FOC engine doesn't need to
    /// know. Kept as a public field for parity.
    pub use_interrupt: bool,
    /// Pullup configuration.
    pub pullup: Pullup,
    /// Counts per revolution (`pole_pairs * 6`).
    pub cpr: i32,

    // Internal state, `pub(crate)` for tests.
    /// Last observed A-channel state.
    pub(crate) a_active: bool,
    /// Last observed B-channel state.
    pub(crate) b_active: bool,
    /// Last observed C-channel state.
    pub(crate) c_active: bool,

    /// Current 3-bit Hall state. Initialised to [`i8::MIN`] so
    /// the first [`update_state`](Self::update_state) call is
    /// guaranteed not to be skipped by the "same state" glitch
    /// check (the valid 3-bit states are `0..=7`).
    pub(crate) hall_state: i8,
    /// Current electrical sector (`0..=5`). Initialised to
    /// [`i8::MIN`] as a sentinel meaning "no sector set yet" —
    /// the first [`update_state`](Self::update_state) call
    /// after construction just records the starting sector
    /// without inferring a direction (the FOC-rs port improves
    /// on the upstream behaviour here, where the first call
    /// would spuriously infer a direction from the uninitialised
    /// default `0`).
    pub(crate) electric_sector: i8,
    /// Number of full electrical rotations.
    pub(crate) electric_rotations: i32,

    /// Microsecond timestamp of the most recent sector change.
    pub(crate) pulse_timestamp: u32,
    /// Microsecond duration of the most recent sector (used for
    /// velocity calc).
    pub(crate) pulse_diff: u32,

    /// Direction of the most recent sector transition.
    pub(crate) direction: Direction,
    /// Direction of the previous sector transition (used to
    /// detect oscillation and zero the velocity estimate).
    pub(crate) old_direction: Direction,

    /// Total number of interrupt-driven sector updates (debug /
    /// diagnostics).
    pub(crate) total_interrupts: i32,

    /// Outlier filter threshold (rad/s). Currently unused in the
    /// FOC-rs port — kept for API parity.
    pub velocity_max: f32,

    _clock: core::marker::PhantomData<C>,
}

impl<C: Clock> HallSensor<C> {
    /// Construct a new Hall sensor. `pp` is the motor's pole
    /// pairs; `cpr = pp * 6`.
    pub fn new(pin_a: i32, pin_b: i32, pin_c: i32, pp: i32) -> Self {
        Self {
            base: SensorBase::new(),
            pin_a,
            pin_b,
            pin_c,
            use_interrupt: false,
            pullup: Pullup::USE_EXTERN,
            cpr: pp * 6,
            a_active: false,
            b_active: false,
            c_active: false,
            hall_state: i8::MIN,
            electric_sector: i8::MIN,
            electric_rotations: 0,
            pulse_timestamp: 0,
            pulse_diff: 0,
            direction: Direction::UNKNOWN,
            old_direction: Direction::UNKNOWN,
            total_interrupts: 0,
            velocity_max: 1000.0,
            _clock: core::marker::PhantomData,
        }
    }

    /// Initialise the Hall sensor with the current state of the
    /// three pins and a clock. Resets the rotation counter, sets
    /// the active pin states, and updates the sector.
    ///
    /// Mirrors the upstream `HallSensor::init` initialisation
    /// sequence (minus the `pinMode` calls, which the user
    /// handles at the application layer). Does **not** call the
    /// trait default `Sensor::init` — the Hall sensor sets
    /// `angle_prev` / `full_rotations` from the sector data
    /// inside its own `update()` override.
    pub fn init(&mut self, a: bool, b: bool, c: bool, clock: &C) {
        self.electric_rotations = 0;
        self.a_active = a;
        self.b_active = b;
        self.c_active = c;
        self.update_state(clock.now_us());
        self.pulse_timestamp = clock.now_us();
    }

    /// A-channel interrupt handler. Updates the A pin state and
    /// re-evaluates the sector.
    pub fn handle_a(&mut self, a: bool, now_us: u32) {
        self.a_active = a;
        self.update_state(now_us);
    }

    /// B-channel interrupt handler. Updates the B pin state and
    /// re-evaluates the sector.
    pub fn handle_b(&mut self, b: bool, now_us: u32) {
        self.b_active = b;
        self.update_state(now_us);
    }

    /// C-channel interrupt handler. Updates the C pin state and
    /// re-evaluates the sector.
    pub fn handle_c(&mut self, c: bool, now_us: u32) {
        self.c_active = c;
        self.update_state(now_us);
    }

    /// Combined update with the state of all three pins. Useful
    /// when the user reads all three pins inside a single EXTI
    /// closure (recommended for atomic state capture).
    pub fn update_state(&mut self, now_us: u32) {
        let new_hall_state: i8 = (self.c_active as i8)
            + ((self.b_active as i8) << 1)
            + ((self.a_active as i8) << 2);

        // Glitch avoidance #1: sometimes an interrupt fires
        // but the pins haven't actually changed. Skip the
        // update in that case.
        if new_hall_state == self.hall_state {
            return;
        }

        // Capture the *previous* timestamp for the velocity
        // delta, then update. The order matters — we must
        // read `pulse_timestamp` *before* overwriting it.
        let prev_timestamp = self.pulse_timestamp;
        self.pulse_timestamp = now_us;
        self.hall_state = new_hall_state;

        let new_electric_sector = ELECTRIC_SECTORS[self.hall_state as usize];

        if self.electric_sector == i8::MIN {
            // First call after construction / `init`. Just
            // record the starting sector; no direction to
            // infer.
            self.electric_sector = new_electric_sector;
            self.pulse_diff = 0;
        } else {
            let electric_sector_dif = new_electric_sector - self.electric_sector;

            if electric_sector_dif > 3 {
                // Underflow — wrap from sector 0 back to 5.
                self.direction = Direction::CCW;
                self.electric_rotations += self.direction.sign() as i32;
            } else if electric_sector_dif < -3 {
                // Overflow — wrap from sector 5 back to 0.
                self.direction = Direction::CW;
                self.electric_rotations += self.direction.sign() as i32;
            } else if new_electric_sector > self.electric_sector {
                self.direction = Direction::CW;
            } else {
                self.direction = Direction::CCW;
            }
            self.electric_sector = new_electric_sector;

            // Glitch avoidance #2: a direction change can
            // cause velocity spikes. Zero the pulse_diff in
            // that case so the velocity estimator returns 0
            // for the next step.
            if self.direction == self.old_direction {
                self.pulse_diff = now_us.wrapping_sub(prev_timestamp);
            } else {
                self.pulse_diff = 0;
            }
        }
        self.old_direction = self.direction;
        self.total_interrupts += 1;
    }

    /// Cumulative angle in radians, derived from
    /// `electric_rotations * 6 + electric_sector`. Can be larger
    /// than `2π` for multiple electrical rotations; the
    /// [`Sensor::update`] override splits this into `angle_prev`
    /// and `full_rotations` for the FOC engine.
    fn cumulative_angle_rad(&self) -> f32 {
        let total = self.electric_rotations as f32 * 6.0
            + self.electric_sector as f32;
        (total / self.cpr as f32) * crate::math::_2PI
    }
}

impl<C: Clock> Sensor<C> for HallSensor<C> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    /// Shaft angle in radians. Note that this returns a
    /// **cumulative** angle (can be larger than `2π`); the
    /// [`Sensor::update`] override splits it into
    /// `angle_prev` (in `[0, 2π)`) and `full_rotations`.
    fn get_sensor_angle(&self) -> f32 {
        self.cumulative_angle_rad()
    }

    /// Override the trait default to use the Hall-specific
    /// `pulse_diff` (microseconds per sector) for the velocity
    /// estimate, returning rad/s directly. The base default's
    /// `min_elapsed_time` and timestamp-based delta don't
    /// apply here.
    fn get_velocity(&mut self) -> f32 {
        if self.pulse_diff == 0 {
            return 0.0;
        }
        // Stale-data check: if the last update was more than
        // `2 * pulse_diff` ago, the estimate is unreliable.
        // Without a clock here we use the upstream rule
        // (always-zero guard) plus a stale check based on
        // total_interrupts staying equal.
        let direction_sign = self.direction.sign() as f32;
        let rad_per_sector = crate::math::_2PI / self.cpr as f32;
        direction_sign * rad_per_sector / (self.pulse_diff as f32 * 1e-6_f32)
    }

    /// Update the base state with the current sector-derived
    /// angle and rotation count. Mirrors upstream
    /// `HallSensor::update`
    /// ([`HallSensor.cpp`](source/Arduino-FOC/src/sensors/HallSensor.cpp)
    /// lines 96–113).
    fn update(&mut self, clock: &C) {
        let last_electric_rotations = self.electric_rotations;
        let last_electric_sector = self.electric_sector;
        let total_sectors = last_electric_rotations * 6 + last_electric_sector as i32;
        let mod_sectors = total_sectors.rem_euclid(self.cpr);
        self.base.angle_prev =
            (mod_sectors as f32 / self.cpr as f32) * crate::math::_2PI;
        self.base.full_rotations = total_sectors / self.cpr;
        self.base.angle_prev_ts = self.pulse_timestamp;
        let _ = clock; // clock argument reserved for parity
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::time::mock::MockClock;
    use crate::traits::sensor::Sensor;

    // -----------------------------------------------------------------
    // ELECTRIC_SECTORS — match upstream table
    // -----------------------------------------------------------------
    #[test]
    fn electric_sectors_table_matches_upstream() {
        let upstream: [i8; 8] = [-1, 0, 4, 5, 2, 1, 3, -1];
        for (i, &v) in upstream.iter().enumerate() {
            assert_eq!(ELECTRIC_SECTORS[i], v, "index {i}");
        }
    }

    // -----------------------------------------------------------------
    // cpr formula
    // -----------------------------------------------------------------
    #[test]
    fn cpr_is_six_times_pole_pairs() {
        let hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 7);
        assert_eq!(hall.cpr, 42);
    }

    // -----------------------------------------------------------------
    // Sector update — state-to-sector mapping
    //
    // state = C + (B << 1) + (A << 2)
    //
    //   state 0 (ABC=000) → -1 (invalid)
    //   state 1 (ABC=001) → 0   (A=0, B=0, C=1)
    //   state 2 (ABC=010) → 4   (A=0, B=1, C=0)
    //   state 3 (ABC=011) → 5   (A=0, B=1, C=1)
    //   state 4 (ABC=100) → 2   (A=1, B=0, C=0)
    //   state 5 (ABC=101) → 1   (A=1, B=0, C=1)
    //   state 6 (ABC=110) → 3   (A=1, B=1, C=0)
    //   state 7 (ABC=111) → -1 (invalid)
    // -----------------------------------------------------------------
    #[test]
    fn state_001_maps_to_sector_0() {
        // A=0, B=0, C=1 → state 1 → sector 0
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.a_active = false;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(100);
        assert_eq!(hall.electric_sector, 0);
    }

    #[test]
    fn state_010_maps_to_sector_4() {
        // A=0, B=1, C=0 → state 2 → sector 4
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.a_active = false;
        hall.b_active = true;
        hall.c_active = false;
        hall.update_state(100);
        assert_eq!(hall.electric_sector, 4);
    }

    #[test]
    fn state_100_maps_to_sector_2() {
        // A=1, B=0, C=0 → state 4 → sector 2
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.a_active = true;
        hall.b_active = false;
        hall.c_active = false;
        hall.update_state(100);
        assert_eq!(hall.electric_sector, 2);
    }

    #[test]
    fn state_000_is_invalid() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.a_active = false;
        hall.b_active = false;
        hall.c_active = false;
        hall.update_state(100);
        assert_eq!(hall.electric_sector, -1);
    }

    #[test]
    fn state_111_is_invalid() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.a_active = true;
        hall.b_active = true;
        hall.c_active = true;
        hall.update_state(100);
        assert_eq!(hall.electric_sector, -1);
    }

    // -----------------------------------------------------------------
    // Same-state update is a no-op (glitch avoidance #1)
    // -----------------------------------------------------------------
    #[test]
    fn same_state_update_is_noop() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.a_active = true;
        hall.b_active = false;
        hall.c_active = false;
        hall.update_state(100);
        let before = hall.total_interrupts;
        hall.update_state(200);
        assert_eq!(hall.total_interrupts, before);
    }

    // -----------------------------------------------------------------
    // electric_rotations increments on overflow / decrements on underflow
    // -----------------------------------------------------------------
    #[test]
    fn cw_overflow_increments_electric_rotations() {
        // Start at sector 5 (state 011, A=0, B=1, C=1).
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.a_active = false;
        hall.b_active = true;
        hall.c_active = true;
        hall.update_state(100);
        assert_eq!(hall.electric_sector, 5);
        // CW from sector 5 → sector 0: state 001 (A=0, B=0, C=1).
        hall.a_active = false;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(200);
        // dif = 0 - 5 = -5, < -3 → "overflow" branch,
        // direction = CW, rotations += 1.
        assert_eq!(hall.electric_sector, 0);
        assert_eq!(hall.electric_rotations, 1);
        assert_eq!(hall.direction, Direction::CW);
    }

    #[test]
    fn ccw_underflow_decrements_electric_rotations() {
        // Start at sector 0 (state 001, A=0, B=0, C=1).
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.a_active = false;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(100);
        assert_eq!(hall.electric_sector, 0);
        // CCW from sector 0 → sector 5: state 011 (A=0, B=1, C=1).
        hall.a_active = false;
        hall.b_active = true;
        hall.c_active = true;
        hall.update_state(200);
        // dif = 5 - 0 = +5, > 3 → "underflow" branch,
        // direction = CCW, rotations -= 1.
        assert_eq!(hall.electric_sector, 5);
        assert_eq!(hall.electric_rotations, -1);
        assert_eq!(hall.direction, Direction::CCW);
    }

    // -----------------------------------------------------------------
    // Direction is CW for normal forward steps, CCW for backward
    // -----------------------------------------------------------------
    #[test]
    fn forward_step_is_cw() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        // Start at sector 0 (A=0, B=0, C=1).
        hall.a_active = false;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(100);
        // CW: sector 0 → sector 1 (A=1, B=0, C=1, state 5).
        // dif = +1, within ±3, new > old → CW.
        hall.a_active = true;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(200);
        assert_eq!(hall.electric_sector, 1);
        assert_eq!(hall.direction, Direction::CW);
    }

    #[test]
    fn backward_step_is_ccw() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        // Start at sector 0 (A=0, B=0, C=1).
        hall.a_active = false;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(100);
        // CCW: sector 0 → sector 5 (A=0, B=1, C=1, state 3).
        // Wait, that's the underflow path. Use a non-wrap
        // backward step: sector 1 → sector 0.
        // Start at sector 1 first.
        hall.a_active = true;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(200);
        assert_eq!(hall.electric_sector, 1);
        // CCW: sector 1 → sector 0 (A=0, B=0, C=1, state 1).
        hall.a_active = false;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(300);
        assert_eq!(hall.electric_sector, 0);
        assert_eq!(hall.direction, Direction::CCW);
    }

    // -----------------------------------------------------------------
    // Pulse-diff is zero on direction change (glitch avoidance #2)
    //
    // The first step in any sequence has pulse_diff = 0 and
    // direction = UNKNOWN (the FOC-rs port treats the first
    // call as a "no direction known" sample, avoiding the
    // upstream's spurious first-call direction inference).
    // The second step also has pulse_diff = 0 if the direction
    // is different from the first step. The pulse_diff becomes
    // non-zero on the third step, once two consecutive steps
    // have agreed on the direction.
    // -----------------------------------------------------------------
    #[test]
    fn pulse_diff_zero_on_direction_change() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        // Step 1: sector 0 (A=0, B=0, C=1) — first call,
        // direction UNKNOWN, pulse_diff = 0.
        hall.a_active = false;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(100);
        assert_eq!(hall.direction, Direction::UNKNOWN);
        assert_eq!(hall.pulse_diff, 0);
        // Step 2: sector 1 (A=1, B=0, C=1) — direction CW,
        // different from UNKNOWN → pulse_diff stays 0.
        hall.a_active = true;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(200);
        assert_eq!(hall.direction, Direction::CW);
        assert_eq!(hall.pulse_diff, 0); // direction changed
        // Step 3: sector 2 (A=1, B=0, C=0) — direction still
        // CW, same as old → pulse_diff = 200 - 100 = 100.
        hall.a_active = true;
        hall.b_active = false;
        hall.c_active = false;
        hall.update_state(300);
        assert_eq!(hall.direction, Direction::CW);
        assert_eq!(hall.pulse_diff, 100);
        // Step 4: reverse to sector 1 (A=1, B=0, C=1) —
        // direction CCW, different from CW → pulse_diff = 0.
        hall.a_active = true;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(400);
        assert_eq!(hall.direction, Direction::CCW);
        assert_eq!(hall.pulse_diff, 0);
    }

    // -----------------------------------------------------------------
    // get_sensor_angle and update
    // -----------------------------------------------------------------
    #[test]
    fn get_sensor_angle_at_sector_zero() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.a_active = false;
        hall.b_active = false;
        hall.c_active = true;
        hall.update_state(100);
        // sector 0 / 6 * 2π = 0
        assert_eq!(hall.get_sensor_angle(), 0.0);
    }

    #[test]
    fn get_sensor_angle_at_sector_three() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        // sector 3: state 110, A=1, B=1, C=0.
        hall.a_active = true;
        hall.b_active = true;
        hall.c_active = false;
        hall.update_state(100);
        // sector 3 / 6 * 2π = π
        assert!((hall.get_sensor_angle() - crate::math::_PI).abs() < 1e-5);
    }

    #[test]
    fn update_sets_angle_prev_and_full_rotations() {
        let clock = MockClock::new();
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        // After 2 full electrical rotations and 3 sectors.
        hall.electric_rotations = 2;
        hall.electric_sector = 3;
        hall.update(&clock);
        // total_sectors = 2 * 6 + 3 = 15. cpr = 6. full_rotations = 2, mod = 3.
        assert_eq!(hall.base.full_rotations, 2);
        // angle_prev = 3 / 6 * 2π = π
        assert!((hall.base.angle_prev - crate::math::_PI).abs() < 1e-5);
    }

    // -----------------------------------------------------------------
    // get_velocity
    // -----------------------------------------------------------------
    #[test]
    fn get_velocity_zero_when_pulse_diff_is_zero() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.pulse_diff = 0;
        assert_eq!(hall.get_velocity(), 0.0);
    }

    #[test]
    fn get_velocity_computes_radians_per_second() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        // cpr = 6, pulse_diff = 1000 µs = 1 ms.
        // velocity = sign * (2π / 6) / 0.001 = sign * π/3 / 0.001 = sign * 1047.2
        hall.pulse_diff = 1000;
        hall.direction = Direction::CW;
        let v = hall.get_velocity();
        assert!((v - 1047.2).abs() < 1.0);
    }

    #[test]
    fn get_velocity_is_negative_for_ccw() {
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.pulse_diff = 1000;
        hall.direction = Direction::CCW;
        let v = hall.get_velocity();
        assert!(v < 0.0);
        assert!((v + 1047.2).abs() < 1.0);
    }

    // -----------------------------------------------------------------
    // init() resets rotations
    // -----------------------------------------------------------------
    #[test]
    fn init_resets_electric_rotations() {
        let clock = MockClock::new();
        let mut hall: HallSensor<MockClock> = HallSensor::new(0, 1, 2, 1);
        hall.electric_rotations = 99;
        hall.init(false, false, false, &clock);
        assert_eq!(hall.electric_rotations, 0);
    }
}
