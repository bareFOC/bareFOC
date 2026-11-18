//! # transmission::analytical — closed-form analytical linkages and eccentric cams
//!
//! Provides exact closed-form kinematic solutions and exact analytical Jacobians
//! for planar four-bar linkages (Freudenstein's equation) and eccentric cam mechanisms.

use super::Transmission;

/// Branch configuration for four-bar linkage assembly mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FourBarBranch {
    /// Open linkage assembly configuration (+ quadratic root).
    Open,
    /// Crossed linkage assembly configuration (- quadratic root).
    Crossed,
}

/// Planar four-bar linkage mechanism.
///
/// Links:
/// - $L_1$: Ground frame length (distance between crank ground pivot and follower ground pivot).
/// - $L_2$: Crank / input link length driven by motor.
/// - $L_3$: Coupler link length.
/// - $L_4$: Follower / output link length.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FourBarLinkage {
    pub l1: f32,
    pub l2: f32,
    pub l3: f32,
    pub l4: f32,
    pub branch: FourBarBranch,
}

impl FourBarLinkage {
    /// Construct a new four-bar linkage with the given link lengths.
    pub fn new(l1: f32, l2: f32, l3: f32, l4: f32, branch: FourBarBranch) -> Self {
        Self {
            l1: l1.abs(),
            l2: l2.abs(),
            l3: l3.abs(),
            l4: l4.abs(),
            branch,
        }
    }

    /// Freudenstein constants: $(K_1, K_2, K_3)$.
    #[inline]
    fn freudenstein_constants(&self) -> (f32, f32, f32) {
        let k1 = self.l1 / self.l2;
        let k2 = self.l1 / self.l4;
        let k3 = (self.l1 * self.l1 + self.l2 * self.l2 - self.l3 * self.l3 + self.l4 * self.l4)
            / (2.0 * self.l2 * self.l4);
        (k1, k2, k3)
    }

    /// Forward kinematics: computes follower angle $\theta_4$ given crank angle $\theta_2$ (radians).
    pub fn forward_1d(&self, theta_in: f32) -> f32 {
        let (k1, k2, k3) = self.freudenstein_constants();
        let cos_in = libm::cosf(theta_in);
        let sin_in = libm::sinf(theta_in);

        let term1 = k3 - k2 * cos_in;
        let term2 = k1 - cos_in;

        let a = term1 - term2;
        let b = -2.0 * sin_in;
        let c = term1 + term2;

        let t = if a.abs() < 1e-6 {
            if b.abs() < 1e-6 {
                0.0
            } else {
                -c / b
            }
        } else {
            let disc = b * b - 4.0 * a * c;
            if disc < 0.0 {
                return 0.0;
            }
            let sqrt_disc = libm::sqrtf(disc);
            match self.branch {
                FourBarBranch::Open => (-b + sqrt_disc) / (2.0 * a),
                FourBarBranch::Crossed => (-b - sqrt_disc) / (2.0 * a),
            }
        };

        2.0 * libm::atanf(t)
    }

    /// Inverse kinematics: computes crank angle $\theta_2$ given follower angle $\theta_4$ (radians).
    pub fn inverse_1d(&self, theta_out: f32) -> f32 {
        let (k1, k2, k3) = self.freudenstein_constants();
        let cos_out = libm::cosf(theta_out);
        let sin_out = libm::sinf(theta_out);

        let term_p = k1 * cos_out + k3;
        let term_q = k2 + cos_out;

        let a = term_p + term_q;
        let b = -2.0 * sin_out;
        let c = term_p - term_q;

        let u = if a.abs() < 1e-6 {
            if b.abs() < 1e-6 {
                0.0
            } else {
                -c / b
            }
        } else {
            let disc = b * b - 4.0 * a * c;
            if disc < 0.0 {
                return 0.0;
            }
            let sqrt_disc = libm::sqrtf(disc);
            match self.branch {
                FourBarBranch::Open => (-b + sqrt_disc) / (2.0 * a),
                FourBarBranch::Crossed => (-b - sqrt_disc) / (2.0 * a),
            }
        };

        2.0 * libm::atanf(u)
    }

    /// Exact analytical Jacobian derivative $d\theta_4 / d\theta_2$.
    pub fn jacobian_1d(&self, theta_in: f32) -> f32 {
        let theta_out = self.forward_1d(theta_in);
        let (k1, k2, _) = self.freudenstein_constants();

        let sin_in = libm::sinf(theta_in);
        let sin_out = libm::sinf(theta_out);
        let sin_diff = libm::sinf(theta_in - theta_out);

        let num = k2 * sin_in + sin_diff;
        let den = k1 * sin_out + sin_diff;

        if den.abs() < 1e-6 {
            1.0
        } else {
            num / den
        }
    }
}

impl Transmission<1, 1> for FourBarLinkage {
    fn forward(&self, mot_angles: &[f32; 1]) -> [f32; 1] {
        [self.forward_1d(mot_angles[0])]
    }

    fn inverse(&self, joint_angles: &[f32; 1]) -> [f32; 1] {
        [self.inverse_1d(joint_angles[0])]
    }

    fn jacobian(&self, mot_angles: &[f32; 1]) -> [[f32; 1]; 1] {
        [[self.jacobian_1d(mot_angles[0])]]
    }
}

/// Eccentric cam transmission with eccentricity $e$ and follower arm length $L$.
///
/// Models angular follower rocker motion:
/// $\theta_{\text{out}} = \arcsin\left(\frac{e}{L} \sin(\theta_{\text{in}})\right)$.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EccentricCam {
    pub eccentricity: f32,
    pub arm_length: f32,
}

impl EccentricCam {
    /// Create a new eccentric cam transmission.
    pub fn new(eccentricity: f32, arm_length: f32) -> Self {
        let l = if arm_length <= 0.0 { 1.0 } else { arm_length };
        let e = eccentricity.abs().min(l * 0.99);
        Self {
            eccentricity: e,
            arm_length: l,
        }
    }

    /// Forward kinematics: joint angle from crank angle.
    pub fn forward_1d(&self, theta_in: f32) -> f32 {
        let ratio = (self.eccentricity / self.arm_length) * libm::sinf(theta_in);
        libm::asinf(ratio.clamp(-0.9999, 0.9999))
    }

    /// Inverse kinematics: crank angle from joint angle.
    pub fn inverse_1d(&self, theta_out: f32) -> f32 {
        let sin_out = libm::sinf(theta_out);
        let val = (self.arm_length / self.eccentricity) * sin_out;
        libm::asinf(val.clamp(-1.0, 1.0))
    }

    /// Exact analytical Jacobian $d\theta_{\text{out}} / d\theta_{\text{in}}$.
    pub fn jacobian_1d(&self, theta_in: f32) -> f32 {
        let k = self.eccentricity / self.arm_length;
        let s = k * libm::sinf(theta_in);
        let cos_in = libm::cosf(theta_in);
        let denom_sq = 1.0 - s * s;
        if denom_sq <= 1e-6 {
            0.0
        } else {
            (k * cos_in) / libm::sqrtf(denom_sq)
        }
    }
}

impl Transmission<1, 1> for EccentricCam {
    fn forward(&self, mot_angles: &[f32; 1]) -> [f32; 1] {
        [self.forward_1d(mot_angles[0])]
    }

    fn inverse(&self, joint_angles: &[f32; 1]) -> [f32; 1] {
        [self.inverse_1d(joint_angles[0])]
    }

    fn jacobian(&self, mot_angles: &[f32; 1]) -> [[f32; 1]; 1] {
        [[self.jacobian_1d(mot_angles[0])]]
    }
}

/// Unified analytical linkage transmission enum.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AnalyticalLinkage {
    FourBar(FourBarLinkage),
    Cam(EccentricCam),
}

impl Transmission<1, 1> for AnalyticalLinkage {
    fn forward(&self, mot_angles: &[f32; 1]) -> [f32; 1] {
        match self {
            AnalyticalLinkage::FourBar(fb) => fb.forward(mot_angles),
            AnalyticalLinkage::Cam(cam) => cam.forward(mot_angles),
        }
    }

    fn inverse(&self, joint_angles: &[f32; 1]) -> [f32; 1] {
        match self {
            AnalyticalLinkage::FourBar(fb) => fb.inverse(joint_angles),
            AnalyticalLinkage::Cam(cam) => cam.inverse(joint_angles),
        }
    }

    fn jacobian(&self, mot_angles: &[f32; 1]) -> [[f32; 1]; 1] {
        match self {
            AnalyticalLinkage::FourBar(fb) => fb.jacobian(mot_angles),
            AnalyticalLinkage::Cam(cam) => cam.jacobian(mot_angles),
        }
    }
}
