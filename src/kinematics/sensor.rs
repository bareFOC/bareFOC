//! # sensor — 2-DOF multi-axis synchronous sensing abstraction
//!
//! Provides the [`Sensor2D`] trait for multi-axis magnetic sensors (e.g. TMAG5273,
//! dual-axis encoders, or gimbals), adapter views to standard 1D [`Sensor`]s,
//! and combining adapters.

use crate::common::time::Clock;
use crate::traits::sensor::{Sensor, SensorBase};

/// Synchronous multi-axis 2-DOF sensor trait.
pub trait Sensor2D<C: Clock> {
    /// Read both axes synchronously in a single bus transaction and update internal state.
    fn update_2d(&mut self, clock: &C);

    /// Get current joint angles $[\alpha, \beta]$ in radians.
    fn get_angles(&self) -> [f32; 2];

    /// Get current joint velocities $[\dot{\alpha}, \dot{\beta}]$ in rad/s.
    fn get_velocities(&self) -> [f32; 2];
}

/// Dual 1D sensor adapter: combines two standard [`Sensor`] instances into a [`Sensor2D`].
pub struct DualSensorAdapter<S1, S2, C: Clock> {
    pub sensor_1: S1,
    pub sensor_2: S2,
    angles: [f32; 2],
    velocities: [f32; 2],
    _clock: core::marker::PhantomData<C>,
}

impl<S1, S2, C: Clock> DualSensorAdapter<S1, S2, C> {
    /// Create a new adapter wrapping two 1-DOF sensors.
    pub fn new(sensor_1: S1, sensor_2: S2) -> Self {
        Self {
            sensor_1,
            sensor_2,
            angles: [0.0, 0.0],
            velocities: [0.0, 0.0],
            _clock: core::marker::PhantomData,
        }
    }
}

impl<S1, S2, C: Clock> Sensor2D<C> for DualSensorAdapter<S1, S2, C>
where
    S1: Sensor<C>,
    S2: Sensor<C>,
{
    fn update_2d(&mut self, clock: &C) {
        self.sensor_1.update(clock);
        self.sensor_2.update(clock);
        self.angles = [self.sensor_1.get_angle(), self.sensor_2.get_angle()];
        self.velocities = [self.sensor_1.get_velocity(), self.sensor_2.get_velocity()];
    }

    fn get_angles(&self) -> [f32; 2] {
        self.angles
    }

    fn get_velocities(&self) -> [f32; 2] {
        self.velocities
    }
}

/// 1D adapter view extracting a single axis from a [`Sensor2D`] reference.
/// Implements standard [`Sensor`] for seamless integration with 1D motor loops.
pub struct AxisView<'a, S, C: Clock> {
    sensor_2d: &'a S,
    axis: usize,
    base: SensorBase,
    _clock: core::marker::PhantomData<C>,
}

impl<'a, S, C: Clock> AxisView<'a, S, C>
where
    S: Sensor2D<C>,
{
    /// Create a 1D adapter view for `axis` (0 for $\alpha$, 1 for $\beta$).
    pub fn new(sensor_2d: &'a S, axis: usize) -> Self {
        Self {
            sensor_2d,
            axis: if axis > 1 { 0 } else { axis },
            base: SensorBase::new(),
            _clock: core::marker::PhantomData,
        }
    }
}

impl<'a, S, C: Clock> Sensor<C> for AxisView<'a, S, C>
where
    S: Sensor2D<C>,
{
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        self.sensor_2d.get_angles()[self.axis]
    }
}

/// Mock 2-DOF sensor for offline host testing.
#[derive(Clone, Debug, Default)]
pub struct MockSensor2D {
    pub angles: [f32; 2],
    pub velocities: [f32; 2],
}

impl MockSensor2D {
    pub fn new(angles: [f32; 2]) -> Self {
        Self {
            angles,
            velocities: [0.0, 0.0],
        }
    }

    pub fn set_state(&mut self, angles: [f32; 2], velocities: [f32; 2]) {
        self.angles = angles;
        self.velocities = velocities;
    }
}

impl<C: Clock> Sensor2D<C> for MockSensor2D {
    fn update_2d(&mut self, _clock: &C) {}

    fn get_angles(&self) -> [f32; 2] {
        self.angles
    }

    fn get_velocities(&self) -> [f32; 2] {
        self.velocities
    }
}
