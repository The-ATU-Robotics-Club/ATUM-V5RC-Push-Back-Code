//! Controllers
//!
//! This module contains feedback controllers used throughout the robot's
//! motion and subsystem control systems. Controllers compute outputs based
//! on the difference between a desired target and the current system state.
//!
//! - [`pid`] – proportional–integral–derivative controller
pub mod pid;
