//! Localization module.
//!
//! Provides robot pose estimation and supporting math utilities.
//!
//! - [`rcl`] – pose reset system using distance sensors
//! - [`odometry`] – pose estimation system
//! - [`pose`] – robot pose representation
//! - [`vec2`] – 2D vector math utilities

pub mod rcl;
pub mod odometry;
pub mod pose;
pub mod shape;
pub mod vec2;
