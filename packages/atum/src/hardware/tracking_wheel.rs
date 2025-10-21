use std::f64::consts::PI;

use uom::si::f64::Length;
use vexide::{
    adi::AdiPort,
    math::{Angle, Direction},
    prelude::AdiEncoder,
};

pub struct TrackingWheel {
    encoder: AdiEncoder<4096>,
    direction: Direction,
    wheel_circum: Length,
    from_center: Length,
    prev_position: Angle,
}

impl TrackingWheel {
    pub fn new(
        top_port: AdiPort,
        bottom_port: AdiPort,
        direction: Direction,
        wheel_diameter: Length,
        from_center: Length,
    ) -> Self {
        let encoder = AdiEncoder::new(top_port, bottom_port);
        let prev_position = encoder.position().unwrap_or_default();

        Self {
            encoder,
            direction,
            wheel_circum: wheel_diameter * PI,
            from_center,
            prev_position,
        }
    }

    pub fn from_center(&self) -> Length {
        self.from_center
    }

    pub fn traveled(&mut self) -> Length {
        let position = self.encoder.position().unwrap_or_default()
            * match self.direction {
                Direction::Forward => 1.0,
                Direction::Reverse => -1.0,
            };
        let change = position - self.prev_position;
        self.prev_position = position;

        self.wheel_circum * change.as_turns()
    }
}
