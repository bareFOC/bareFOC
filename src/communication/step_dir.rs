//! # communication::step_dir — Step/Direction listener
//!
//! Ports [`StepDirListener.h`](source/Arduino-FOC/src/communication/StepDirListener.h)
//! and [`StepDirListener.cpp`](source/Arduino-FOC/src/communication/StepDirListener.cpp)
//! from Arduino-FOC.
//!
//! The `StepDirListener` tracks a counter based on step and direction pin
//! states. It is designed to be called from an interrupt handler or polled
//! in a main loop.

#![allow(non_camel_case_types)]

use embedded_hal::digital::InputPin;

/// Step/Direction listener class for easier interaction with this communication interface.
pub struct StepDirListener<STEP, DIR> {
    #[allow(dead_code)]
    step_pin: STEP,
    dir_pin: DIR,
    pub count: i32,
    pub counter_to_value: f32,
}

impl<STEP, DIR> StepDirListener<STEP, DIR>
where
    STEP: InputPin,
    DIR: InputPin,
{
    /// Constructor for step/direction interface.
    ///
    /// # Arguments
    /// * `step_pin` - The step input pin.
    /// * `dir_pin` - The direction input pin.
    /// * `counter_to_value` - The multiplier to convert count to the target value.
    pub fn new(step_pin: STEP, dir_pin: DIR, counter_to_value: f32) -> Self {
        Self {
            step_pin,
            dir_pin,
            count: 0,
            counter_to_value,
        }
    }

    /// Step handler. Call this from your step pin interrupt handler or polling loop.
    /// It reads the direction pin and updates the count accordingly.
    pub fn handle_step(&mut self) {
        let dir_is_high = self.dir_pin.is_high().unwrap_or(false);
        if dir_is_high {
            self.count += 1;
        } else {
            self.count -= 1;
        }
    }

    /// Get the current value based on count and counter_to_value.
    pub fn get_value(&self) -> f32 {
        self.count as f32 * self.counter_to_value
    }

    /// Reset the counter to zero (e.g., for homing).
    pub fn reset_count(&mut self) {
        self.count = 0;
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::pin::{Mock as PinMock, State as PinStateMock, Transaction as PinTransaction};

    #[test]
    fn step_dir_listener_initializes_correctly() {
        let step_exp: [PinTransaction; 0] = [];
        let dir_exp: [PinTransaction; 0] = [];
        let mut step_pin = PinMock::new(&step_exp);
        let mut dir_pin = PinMock::new(&dir_exp);

        let listener = StepDirListener::new(&mut step_pin, &mut dir_pin, 0.01);
        assert_eq!(listener.count, 0);
        assert_eq!(listener.counter_to_value, 0.01);
        assert_eq!(listener.get_value(), 0.0);
        
        step_pin.done();
        dir_pin.done();
    }

    #[test]
    fn step_dir_listener_counts_up_on_high_dir() {
        let step_exp: [PinTransaction; 0] = [];
        let dir_exp = [PinTransaction::get(PinStateMock::High)];
        let mut step_pin = PinMock::new(&step_exp);
        let mut dir_pin = PinMock::new(&dir_exp);

        let mut listener = StepDirListener::new(&mut step_pin, &mut dir_pin, 0.01);
        listener.handle_step();
        
        assert_eq!(listener.count, 1);
        assert_eq!(listener.get_value(), 0.01);
        
        step_pin.done();
        dir_pin.done();
    }

    #[test]
    fn step_dir_listener_reset_count() {
        let step_exp: [PinTransaction; 0] = [];
        let dir_exp: [PinTransaction; 0] = [];
        let mut step_pin = PinMock::new(&step_exp);
        let mut dir_pin = PinMock::new(&dir_exp);

        let mut listener = StepDirListener::new(&mut step_pin, &mut dir_pin, 0.01);
        listener.count = 100;
        listener.reset_count();
        
        assert_eq!(listener.count, 0);
        
        step_pin.done();
        dir_pin.done();
    }
}
