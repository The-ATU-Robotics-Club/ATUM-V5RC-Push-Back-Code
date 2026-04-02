//! Robot settings configuration
//!
//! Provides a centralized way to store and manage various runtime settings
//! for autonomous routines, color selection, and subsystems behavior.

use std::ops::{Not, Range};

/// Stores the main settings for a run of the robot.
///
/// These values are used to control autonomous behavior, color selection,
/// and subsystem features (like sorting or testing modes).
#[derive(Clone, Copy)]
pub struct Settings {
    /// Selected color for the robot to interact with
    pub color: Color,
    /// Index for autonomous routine or selection
    pub index: usize,
    /// Whether to run the test autonomous routine
    pub test_auton: bool,
    /// Whether to override automatic color selection
    pub color_override: bool,
}

/// Represents the colors the robot can track
#[derive(Clone, Copy)]
pub enum Color {
    Red,
    Blue,
}

impl Color {
    /// Hue ranges for Red and Blue (in degrees)
    // pub const RED_HUE: Range<f64> = 300.0..360.0;
    pub const RED_HUE: Range<f64> = 0.0..55.0;
    pub const BLUE_HUE: Range<f64> = 70.0..300.0;

    /// Returns the hue range associated with this color
    pub fn hue_range(&self) -> Range<f64> {
        match self {
            Color::Red => Self::RED_HUE,
            Color::Blue => Self::BLUE_HUE,
        }
    }
}

/// Allows flipping a color: Red -> Blue, Blue -> Red
impl Not for Color {
    type Output = Self;

    fn not(self) -> Self::Output {
        match self {
            Color::Red => Color::Blue,
            Color::Blue => Color::Red,
        }
    }
}
