use core::f64::consts::PI;

use log::debug;
use vexide::prelude::{AdiPort, Direction, Position};

use super::encoder::Encoder;
use crate::units::length::Length;

pub struct TrackingWheel {
    encoder: Encoder<4096>,
    wheel_circum: Length,
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
        let encoder = Encoder::new(top_port, bottom_port, direction);
        let prev_position = encoder.position().unwrap_or_default();

        Self {
            encoder,
            wheel_circum: wheel_diameter * PI,
            from_center,
            prev_position,
        }
    }

    pub fn from_center(&self) -> Length {
        self.from_center
    }

    pub fn traveled(&mut self) -> Length {
        let position = self.encoder.position().unwrap_or_default();
        let change = position - self.prev_position;
        debug!("p, c: {}, {}", self.prev_position.as_degrees(), change.as_degrees());
        self.prev_position = position;

        self.wheel_circum * change.as_revolutions()
    }
}
