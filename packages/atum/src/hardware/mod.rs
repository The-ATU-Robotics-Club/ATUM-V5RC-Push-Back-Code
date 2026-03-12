//! Hardware Abstraction Layer
//!
//! This module provides abstractions for the robot's physical hardware.
//! It separates direct hardware access from higher-level control logic,
//! allowing the rest of the code to work with unified interfaces.
//!
//! Submodules:
//! - [`imu`] – Inertial Measurement Unit wrapper for heading and rotation.
//! - [`motor_group`] – Groups of motors with convenient control methods.
//! - [`tracking_wheel`] – Passive tracking wheels used for odometry.
//!
//! # Utilities
//! - `average` – Computes the arithmetic mean of a list of f64 values
pub mod imu;
pub mod motor_group;
pub mod tracking_wheel;

fn average(values: Vec<f64>) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}
