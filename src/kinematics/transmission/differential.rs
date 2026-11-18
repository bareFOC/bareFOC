//! # transmission::differential — 2-DOF coupled differential transmission
//!
//! Models coupled 2-motor to 2-joint transmissions (such as robotic pitch/roll
//! differential wrists and spherical gimbals) with forward/inverse mapping and
//! transpose-Jacobian effort reflection.

use super::Transmission;

/// Coupled 2-motor to 2-joint differential transmission.
///
/// Represents the linear transformation:
/// $\mathbf{\theta}_j = \mathbf{T} \mathbf{\theta}_m$
/// where $\mathbf{T} = \begin{bmatrix} t_{11} & t_{12} \\ t_{21} & t_{22} \end{bmatrix}$.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoupledDifferential {
    /// 2x2 forward transformation matrix $\mathbf{T}$.
    pub matrix: [[f32; 2]; 2],
    /// Joint zero offsets $[\alpha_0, \beta_0]$ in radians.
    pub joint_offsets: [f32; 2],
}

impl CoupledDifferential {
    /// Create a standard symmetric differential wrist with reduction ratio $R$.
    ///
    /// Relationship:
    /// $\alpha = \frac{\theta_{m1} + \theta_{m2}}{2 R}$
    /// $\beta = \frac{\theta_{m1} - \theta_{m2}}{2 R}$
    pub fn symmetric(gear_ratio: f32) -> Self {
        let r = if gear_ratio == 0.0 { 1.0 } else { gear_ratio.abs() };
        let scale = 0.5 / r;
        Self {
            matrix: [
                [scale, scale],
                [scale, -scale],
            ],
            joint_offsets: [0.0, 0.0],
        }
    }

    /// Create a custom coupled differential from a 2x2 coupling matrix.
    pub fn from_matrix(matrix: [[f32; 2]; 2]) -> Self {
        Self {
            matrix,
            joint_offsets: [0.0, 0.0],
        }
    }

    /// Set joint zero offsets $[\alpha_0, \beta_0]$.
    pub fn with_offsets(mut self, offsets: [f32; 2]) -> Self {
        self.joint_offsets = offsets;
        self
    }

    /// Forward kinematics: computes joint angles $[\alpha, \beta]$ from motor angles $[\theta_{m1}, \theta_{m2}]$.
    pub fn forward_2d(&self, mot_angles: [f32; 2]) -> [f32; 2] {
        let alpha = self.matrix[0][0] * mot_angles[0] + self.matrix[0][1] * mot_angles[1] + self.joint_offsets[0];
        let beta = self.matrix[1][0] * mot_angles[0] + self.matrix[1][1] * mot_angles[1] + self.joint_offsets[1];
        [alpha, beta]
    }

    /// Inverse kinematics: computes motor angles $[\theta_{m1}, \theta_{m2}]$ from joint angles $[\alpha, \beta]$.
    pub fn inverse_2d(&self, joint_angles: [f32; 2]) -> [f32; 2] {
        let adj_alpha = joint_angles[0] - self.joint_offsets[0];
        let adj_beta = joint_angles[1] - self.joint_offsets[1];

        // Inverse of 2x2 matrix:
        // [a b; c d]^-1 = 1/(ad - bc) * [d -b; -c a]
        let a = self.matrix[0][0];
        let b = self.matrix[0][1];
        let c = self.matrix[1][0];
        let d = self.matrix[1][1];

        let det = a * d - b * c;
        if det.abs() < 1e-9 {
            return [0.0, 0.0];
        }

        let inv_det = 1.0 / det;
        let m1 = inv_det * (d * adj_alpha - b * adj_beta);
        let m2 = inv_det * (-c * adj_alpha + a * adj_beta);

        [m1, m2]
    }

    /// Instantaneous 2x2 Jacobian matrix: $\mathbf{J} = \mathbf{T}$.
    #[inline]
    pub fn jacobian_2d(&self) -> [[f32; 2]; 2] {
        self.matrix
    }

    /// Effort reflection: maps joint torques $[\tau_\alpha, \tau_\beta]$ to motor torques
    /// via the transpose Jacobian: $\boldsymbol{\tau}_m = \mathbf{J}^T \boldsymbol{\tau}_j$.
    pub fn reflect_torques(&self, joint_torques: [f32; 2]) -> [f32; 2] {
        let tau_m1 = self.matrix[0][0] * joint_torques[0] + self.matrix[1][0] * joint_torques[1];
        let tau_m2 = self.matrix[0][1] * joint_torques[0] + self.matrix[1][1] * joint_torques[1];
        [tau_m1, tau_m2]
    }
}

impl Transmission<2, 2> for CoupledDifferential {
    fn forward(&self, mot_angles: &[f32; 2]) -> [f32; 2] {
        self.forward_2d(*mot_angles)
    }

    fn inverse(&self, joint_angles: &[f32; 2]) -> [f32; 2] {
        self.inverse_2d(*joint_angles)
    }

    fn jacobian(&self, _mot_angles: &[f32; 2]) -> [[f32; 2]; 2] {
        self.jacobian_2d()
    }
}
