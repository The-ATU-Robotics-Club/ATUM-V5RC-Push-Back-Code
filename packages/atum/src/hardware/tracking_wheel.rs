use std::f64::consts::PI;

use vexide::{
    adi::AdiPort,
    math::Angle,
    prelude::AdiEncoder,
};

use crate::localization::vec2::Vec2;

/// Represents a passive tracking wheel used for robot localization.
///
/// A tracking wheel measures linear displacement using an encoder
/// attached to a freely rotating wheel. The wheel's position relative
/// to the robot and its orientation are used by the odometry system
/// to estimate the robot's movement.
pub struct TrackingWheel {
    /// Encoder attached to the tracking wheel.
    encoder: AdiEncoder<4096>,

    /// Circumference of the tracking wheel.
    wheel_circum: f64,

    /// Position of the wheel relative to the robot center.
    from_center: Vec2<f64>,

    /// Direction the wheel measures motion in.
    angle: Angle,

    /// Previous encoder position used to compute incremental movement.
    prev_position: Angle,
}

impl TrackingWheel {
    /// Creates a new tracking wheel.
    ///
    /// - `wheel_diameter` is used to compute the wheel circumference.
    /// - `from_center` defines the wheel's position relative to the robot.
    /// - `angle` defines the direction the wheel measures motion.
    pub fn new(
        top_port: AdiPort,
        bottom_port: AdiPort,
        wheel_diameter: f64,
        from_center: Vec2<f64>,
        angle: Angle,
    ) -> Self {
        let encoder = AdiEncoder::new(top_port, bottom_port);
        let prev_position = encoder.position().unwrap_or_default();

        Self {
            encoder,
            wheel_circum: wheel_diameter * PI,
            from_center,
            angle,
            prev_position,
        }
    }

    /// Returns the wheel's position relative to the robot center.
    pub fn from_center(&self) -> Vec2<f64> {
        self.from_center
    }

    /// Returns the direction the wheel measures motion.
    pub fn angle(&self) -> Angle {
        self.angle
    }

    /// Returns the distance traveled by the tracking wheel
    /// since the previous update.
    pub fn traveled(&mut self) -> f64 {
        let position = self.encoder.position().unwrap_or_default();
        let change = position - self.prev_position;
        self.prev_position = position;

        self.wheel_circum * change.as_turns()
    }
}
