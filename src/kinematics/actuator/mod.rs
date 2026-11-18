//! # actuator — cascaded dual-loop robotic joint actuators
//!
//! Bridges high-speed inner motor FOC commutation with medium-speed outer
//! joint position control, Jacobian gain scheduling, and 2-DOF differential coordination.

pub mod actuator_1dof;
pub mod actuator_2dof;
pub mod motor;

pub use actuator_1dof::Actuator1DOF;
pub use actuator_2dof::Actuator2DOF;
pub use motor::MotorControl;
