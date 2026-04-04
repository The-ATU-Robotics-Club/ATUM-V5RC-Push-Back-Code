//! Subsystems
//!
//! This module contains the shared robot subsystems that can be used
//! across different parts of the program, such as autonomous routines
//! or operator control.
//!
//! - [`drivetrain`] – handles the robot drive system, including motor groups and odometry
//! - [`intake`] – manages the intake motors, door mechanism, and color sorting logic
pub mod drivetrain;
pub mod intakes;
