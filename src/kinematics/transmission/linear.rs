//! # transmission::linear — linear gear reduction with backlash and elasticity compensation
//!
//! Models reduction ratio, direction sign, zero offset, continuous backlash deadband
//! hysteresis ($\delta \cdot \tanh(\omega / \omega_{\text{thresh}})$), and structural
//! compliance feedforward ($\tau / K_{\text{gear}}$).

use super::Transmission;

/// Linear gear transmission mapping 1 motor to 1 joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearGear {
    /// Gear reduction ratio ($R > 0$). Motor turns $R$ radians for 1 joint radian.
    pub gear_ratio: f32,
    /// Inversion flag. If true, motion is reversed (sign = -1.0).
    pub inverted: bool,
    /// Rotor zero offset in radians.
    pub zero_offset: f32,
    /// Backlash half-deadband amplitude $\delta$ in radians.
    pub backlash_delta: f32,
    /// Smooth backlash transition velocity threshold $\omega_{\text{thresh}}$ in rad/s.
    pub omega_thresh: f32,
    /// Torsional stiffness $K_{\text{gear}}$ in Nm/rad (0.0 disables compliance feedforward).
    pub torsional_stiffness: f32,
}

impl LinearGear {
    /// Create a new linear gear transmission with nominal reduction ratio.
    pub fn new(gear_ratio: f32) -> Self {
        Self {
            gear_ratio: if gear_ratio == 0.0 { 1.0 } else { gear_ratio.abs() },
            inverted: gear_ratio < 0.0,
            zero_offset: 0.0,
            backlash_delta: 0.0,
            omega_thresh: 0.05,
            torsional_stiffness: 0.0,
        }
    }

    /// Set inversion flag.
    pub fn with_inverted(mut self, inverted: bool) -> Self {
        self.inverted = inverted;
        self
    }

    /// Set motor zero offset in radians.
    pub fn with_zero_offset(mut self, zero_offset: f32) -> Self {
        self.zero_offset = zero_offset;
        self
    }

    /// Set active backlash compensation parameters.
    pub fn with_backlash(mut self, delta: f32, omega_thresh: f32) -> Self {
        self.backlash_delta = delta.abs();
        self.omega_thresh = if omega_thresh <= 0.0 { 0.05 } else { omega_thresh };
        self
    }

    /// Set torsional stiffness $K_{\text{gear}}$ in Nm/rad.
    pub fn with_torsional_stiffness(mut self, k_gear: f32) -> Self {
        self.torsional_stiffness = if k_gear < 0.0 { 0.0 } else { k_gear };
        self
    }

    /// Multiplicative direction sign (+1.0 or -1.0).
    #[inline]
    pub fn sign(&self) -> f32 {
        if self.inverted {
            -1.0
        } else {
            1.0
        }
    }

    /// Scalar forward kinematics: joint output from motor angle (radians).
    #[inline]
    pub fn forward_1d(&self, mot_angle: f32) -> f32 {
        self.sign() * (mot_angle - self.zero_offset) / self.gear_ratio
    }

    /// Scalar inverse kinematics: motor angle from joint angle (radians).
    #[inline]
    pub fn inverse_1d(&self, joint_angle: f32) -> f32 {
        self.sign() * joint_angle * self.gear_ratio + self.zero_offset
    }

    /// Instantaneous scalar Jacobian: $d\theta_{\text{joint}} / d\theta_{\text{mot}}$.
    #[inline]
    pub fn jacobian_1d(&self) -> f32 {
        self.sign() / self.gear_ratio
    }

    /// Active backlash hysteresis deadband offset:
    /// $\Delta\theta_{\text{backlash}} = \delta \cdot \tanh(\omega / \omega_{\text{thresh}})$.
    #[inline]
    pub fn backlash_compensation(&self, velocity: f32) -> f32 {
        if self.backlash_delta == 0.0 {
            0.0
        } else {
            let x = velocity / self.omega_thresh;
            self.backlash_delta * libm::tanhf(x)
        }
    }

    /// Structural torsional elasticity angular deflection:
    /// $\Delta\theta_{\text{elasticity}} = \tau / K_{\text{gear}}$.
    #[inline]
    pub fn elasticity_deflection(&self, torque: f32) -> f32 {
        if self.torsional_stiffness > 0.0 {
            torque / self.torsional_stiffness
        } else {
            0.0
        }
    }

    /// Compensated forward kinematics: computes actual joint position taking into account
    /// nominal kinematics, active backlash deadband, and elasticity deflection.
    pub fn compensate_forward(&self, mot_angle: f32, joint_vel: f32, joint_torque: f32) -> f32 {
        let nominal = self.forward_1d(mot_angle);
        nominal + self.backlash_compensation(joint_vel) + self.elasticity_deflection(joint_torque)
    }

    /// Compensated inverse command: adjusts target joint angle to pre-compensate for
    /// backlash and deflection before computing motor command.
    pub fn compensate_inverse(&self, target_joint: f32, joint_vel: f32, joint_torque: f32) -> f32 {
        let adjusted_joint =
            target_joint - self.backlash_compensation(joint_vel) - self.elasticity_deflection(joint_torque);
        self.inverse_1d(adjusted_joint)
    }
}

impl Transmission<1, 1> for LinearGear {
    fn forward(&self, mot_angles: &[f32; 1]) -> [f32; 1] {
        [self.forward_1d(mot_angles[0])]
    }

    fn inverse(&self, joint_angles: &[f32; 1]) -> [f32; 1] {
        [self.inverse_1d(joint_angles[0])]
    }

    fn jacobian(&self, _mot_angles: &[f32; 1]) -> [[f32; 1]; 1] {
        [[self.jacobian_1d()]]
    }
}

