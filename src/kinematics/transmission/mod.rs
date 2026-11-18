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
