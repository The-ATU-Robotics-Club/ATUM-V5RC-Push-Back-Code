use core::f64::consts::PI;

use vexide::prelude::{AdiPort, Direction, Position};

use crate::math::length::Length;

use super::encoder::Encoder;

pub struct TrackingWheel {
    encoder: Encoder<4096>,
    wheel_diameter: Length,
    from_center: Length,
    prev_position: Position,
}

impl TrackingWheel {
    pub fn new(
        top_port: AdiPort,
        bottom_port: AdiPort,
        direction: Direction,
        wheel_diameter: Length,
        from_center: Length,
    ) -> Self {
        Self {
            encoder: Encoder::new(top_port, bottom_port, direction),
            wheel_diameter,
            from_center,
            prev_position: Position::from_revolutions(0.0),
        }
    }

    pub fn from_center(&self) -> Length {
        self.from_center
    }

    pub fn traveled(&mut self) -> Length {
        let position = self.encoder.position().unwrap_or_default();
        let change = position - self.prev_position;
        self.prev_position = position;

        self.wheel_diameter * change.as_revolutions() * PI
    }
}
