//! # transmission — kinematic transmission abstractions
//!
//! Provides bidirectional mapping, velocity transformations, and effort reflection
//! between motor rotor coordinates and robotic joint coordinates.

pub mod analytical;
pub mod differential;
pub mod linear;
pub mod lut;

pub use analytical::{AnalyticalLinkage, EccentricCam, FourBarBranch, FourBarLinkage};
pub use differential::CoupledDifferential;
pub use linear::LinearGear;
pub use lut::LutTransmission;

/// Dynamic runtime transmission container.
///
/// Wraps all supported transmission kinematics models in a stack-allocated enum,
/// allowing runtime reconfiguration without dynamic dispatch or heap allocation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DynamicTransmission<const LUT_N: usize> {
    /// Linear gear reduction with backlash and compliance.
    Linear(LinearGear),
    /// Coupled 2-DOF differential transmission (wrist / gimbal).
    Differential(CoupledDifferential),
    /// Non-linear uniform Look-Up Table transmission.
    Lut(LutTransmission<LUT_N>),
    /// Planar four-bar linkage (Freudenstein analytical solution).
    FourBar(FourBarLinkage),
    /// Eccentric cam transmission.
    Cam(EccentricCam),
}

impl<const LUT_N: usize> From<LinearGear> for DynamicTransmission<LUT_N> {
    #[inline]
    fn from(gear: LinearGear) -> Self {
        Self::Linear(gear)
    }
}

impl<const LUT_N: usize> From<CoupledDifferential> for DynamicTransmission<LUT_N> {
    #[inline]
    fn from(diff: CoupledDifferential) -> Self {
        Self::Differential(diff)
    }
}

impl<const LUT_N: usize> From<LutTransmission<LUT_N>> for DynamicTransmission<LUT_N> {
    #[inline]
    fn from(lut: LutTransmission<LUT_N>) -> Self {
        Self::Lut(lut)
    }
}

impl<const LUT_N: usize> From<FourBarLinkage> for DynamicTransmission<LUT_N> {
    #[inline]
    fn from(fb: FourBarLinkage) -> Self {
        Self::FourBar(fb)
    }
}

impl<const LUT_N: usize> From<EccentricCam> for DynamicTransmission<LUT_N> {
    #[inline]
    fn from(cam: EccentricCam) -> Self {
        Self::Cam(cam)
    }
}

impl<const LUT_N: usize> Transmission<2, 2> for DynamicTransmission<LUT_N> {
    fn forward(&self, mot_angles: &[f32; 2]) -> [f32; 2] {
        match self {
            Self::Linear(lg) => [lg.forward_1d(mot_angles[0]), mot_angles[1]],
            Self::Differential(diff) => diff.forward(mot_angles),
            Self::Lut(lut) => [lut.forward_1d(mot_angles[0]), mot_angles[1]],
            Self::FourBar(fb) => [fb.forward_1d(mot_angles[0]), mot_angles[1]],
            Self::Cam(cam) => [cam.forward_1d(mot_angles[0]), mot_angles[1]],
        }
    }

    fn inverse(&self, joint_angles: &[f32; 2]) -> [f32; 2] {
        match self {
            Self::Linear(lg) => [lg.inverse_1d(joint_angles[0]), joint_angles[1]],
            Self::Differential(diff) => diff.inverse(joint_angles),
            Self::Lut(lut) => [lut.inverse_1d(joint_angles[0]), joint_angles[1]],
            Self::FourBar(fb) => [fb.inverse_1d(joint_angles[0]), joint_angles[1]],
            Self::Cam(cam) => [cam.inverse_1d(joint_angles[0]), joint_angles[1]],
        }
    }

    fn jacobian(&self, mot_angles: &[f32; 2]) -> [[f32; 2]; 2] {
        match self {
            Self::Linear(lg) => [
                [lg.jacobian_1d(), 0.0],
                [0.0, 1.0],
            ],
            Self::Differential(diff) => diff.jacobian(mot_angles),
            Self::Lut(lut) => [
                [lut.jacobian_1d(mot_angles[0]), 0.0],
                [0.0, 1.0],
            ],
            Self::FourBar(fb) => [
                [fb.jacobian_1d(mot_angles[0]), 0.0],
                [0.0, 1.0],
            ],
            Self::Cam(cam) => [
                [cam.jacobian_1d(mot_angles[0]), 0.0],
                [0.0, 1.0],
            ],
        }
    }
}

impl<const LUT_N: usize> Transmission<1, 1> for DynamicTransmission<LUT_N> {
    fn forward(&self, mot_angles: &[f32; 1]) -> [f32; 1] {
        match self {
            Self::Linear(lg) => [lg.forward_1d(mot_angles[0])],
            Self::Differential(diff) => [diff.forward_2d([mot_angles[0], 0.0])[0]],
            Self::Lut(lut) => [lut.forward_1d(mot_angles[0])],
            Self::FourBar(fb) => [fb.forward_1d(mot_angles[0])],
            Self::Cam(cam) => [cam.forward_1d(mot_angles[0])],
        }
    }

    fn inverse(&self, joint_angles: &[f32; 1]) -> [f32; 1] {
        match self {
            Self::Linear(lg) => [lg.inverse_1d(joint_angles[0])],
            Self::Differential(diff) => [diff.inverse_2d([joint_angles[0], 0.0])[0]],
            Self::Lut(lut) => [lut.inverse_1d(joint_angles[0])],
            Self::FourBar(fb) => [fb.inverse_1d(joint_angles[0])],
            Self::Cam(cam) => [cam.inverse_1d(joint_angles[0])],
        }
    }

    fn jacobian(&self, mot_angles: &[f32; 1]) -> [[f32; 1]; 1] {
        match self {
            Self::Linear(lg) => [[lg.jacobian_1d()]],
            Self::Differential(diff) => [[diff.jacobian_2d()[0][0]]],
            Self::Lut(lut) => [[lut.jacobian_1d(mot_angles[0])]],
            Self::FourBar(fb) => [[fb.jacobian_1d(mot_angles[0])]],
            Self::Cam(cam) => [[cam.jacobian_1d(mot_angles[0])]],
        }
    }
}

/// Generic kinematic transmission mapping between `MOTORS` motor axes
/// and `JOINTS` joint axes.
pub trait Transmission<const MOTORS: usize, const JOINTS: usize> {
    /// Forward kinematics: Maps motor rotor angles (rad) to joint output angles (rad).
    fn forward(&self, mot_angles: &[f32; MOTORS]) -> [f32; JOINTS];

    /// Inverse kinematics: Maps joint output angles (rad) to motor rotor angles (rad).
    fn inverse(&self, joint_angles: &[f32; JOINTS]) -> [f32; MOTORS];

    /// Instantaneous Jacobian matrix $J$: partial derivatives $\partial \theta_{\text{joint}, j} / \partial \theta_{\text{mot}, m}$.
    ///
    /// Row index corresponds to joint index $j$, column index corresponds to motor index $m$:
    /// `jacobian[j][m] = d(theta_joint[j]) / d(theta_mot[m])`.
    fn jacobian(&self, mot_angles: &[f32; MOTORS]) -> [[f32; MOTORS]; JOINTS];

    /// Forward velocity mapping: computes joint velocities from motor velocities
    /// via the instantaneous Jacobian: $\mathbf{v}_{\text{joint}} = \mathbf{J} \mathbf{v}_{\text{mot}}$.
    fn forward_velocity(
        &self,
        mot_angles: &[f32; MOTORS],
        mot_velocities: &[f32; MOTORS],
    ) -> [f32; JOINTS] {
        let jac = self.jacobian(mot_angles);
        let mut joint_vel = [0.0; JOINTS];
        for j in 0..JOINTS {
            let mut sum = 0.0;
            for m in 0..MOTORS {
                sum += jac[j][m] * mot_velocities[m];
            }
            joint_vel[j] = sum;
        }
        joint_vel
    }

    /// Effort reflection: projects joint torques to motor torques via the transpose
    /// Jacobian matrix: $\boldsymbol{\tau}_m = \mathbf{J}^T \boldsymbol{\tau}_j$.
    fn reflect_effort(
        &self,
        mot_angles: &[f32; MOTORS],
        joint_torques: &[f32; JOINTS],
    ) -> [f32; MOTORS] {
        let jac = self.jacobian(mot_angles);
        let mut mot_torque = [0.0; MOTORS];
        for m in 0..MOTORS {
            let mut sum = 0.0;
            for j in 0..JOINTS {
                sum += jac[j][m] * joint_torques[j];
            }
            mot_torque[m] = sum;
        }
        mot_torque
    }
}
