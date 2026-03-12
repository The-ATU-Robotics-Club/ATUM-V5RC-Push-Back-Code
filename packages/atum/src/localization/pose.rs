use std::fmt::Display;

use vexide::math::Angle;

use crate::localization::vec2::Vec2;

/// Represents the robot's pose and motion state.
///
/// Pose consists of:
/// - position on the field (x, y)
/// - heading (h)
/// - robot-relative velocities
///
/// Coordinate conventions:
/// - +x = forward
/// - +y = left
///
/// Velocities are expressed in the robot frame.
#[derive(Clone, Copy, Default)]
pub struct Pose {
    /// Global x position on the field
    pub x: f64,

    /// Global y position on the field
    pub y: f64,

    /// Robot heading
    pub h: Angle,

    /// Forward velocity (robot frame)
    pub vf: f64,

    /// Sideways velocity (robot frame)
    pub vs: f64,

    /// Angular velocity (radians/sec)
    pub omega: f64,
}

impl Pose {
    /// Creates a new pose with zero velocity.
    pub fn new(x: f64, y: f64, h: Angle) -> Self {
        Self {
            x,
            y,
            h,
            vf: 0.0,
            vs: 0.0,
            omega: 0.0,
        }
    }
}

impl Pose {
    /// Returns the position component of the pose as a vector.
    pub fn position(&self) -> Vec2<f64> {
        Vec2::new(self.x, self.y)
    }
}

impl Display for Pose {
    /// Formats the pose for logging/debug output.
    ///
    /// Output format:
    ///
    /// `(x, y, heading_deg, vf, vs, omega)`
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "({:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4})",
            self.x,
            self.y,
            self.h.as_degrees(),
            self.vf,
            self.vs,
            self.omega
        )
    }
}
