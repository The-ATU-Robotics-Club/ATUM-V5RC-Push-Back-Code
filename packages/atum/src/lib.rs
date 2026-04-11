//! # ATUM Lib
//!
//! Central hub for managing robot subsystems, control logic,
//! and hardware interactions in a cohesive framework.
//!
//! ## Modules
//!
//! - [`backend`] – GUI integration
//! - [`controllers`] – control algorithms
//! - [`hardware`] – abstraction for motors and sensors
//! - [`localization`] – odometry and pose representation
//! - [`motion`] – higher-level motion commands
//! - [`subsystems`] – shared subsystems across robots
//! - [`logger`] – console logging utilities
//! - [`mappings`] – input mappings for controller state
//! - [`settings`] – runtime settings for robot
//! - [`theme`] – prints a custom startup banner

pub mod backend;
pub mod controllers;
pub mod hardware;
pub mod localization;
pub mod motion;
pub mod subsystems;

pub mod logger;
pub mod mappings;
pub mod settings;
pub mod theme;
pub mod utils;
