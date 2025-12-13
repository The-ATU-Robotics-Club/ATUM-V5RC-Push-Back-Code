use std::f64::consts::PI;

use uom::si::{
    angle::{radian, revolution},
    f64::{Angle, Length},
};
use vexide::{adi::AdiPort, math::Direction, prelude::AdiEncoder};

use crate::localization::vec2::Vec2;

pub struct TrackingWheel {
    encoder: AdiEncoder<4096>,
    direction: Direction,
    wheel_circum: Length,
    from_center: Vec2<Length>,
    angle: Angle,
    prev_position: Angle,
}

impl TrackingWheel {
    pub fn new(
        top_port: AdiPort,
        bottom_port: AdiPort,
        direction: Direction,
        wheel_diameter: Length,
        from_center: Vec2<Length>,
        angle: Angle,
    ) -> Self {
        let encoder = AdiEncoder::new(top_port, bottom_port);
        let prev_position =
            Angle::new::<radian>(encoder.position().unwrap_or_default().as_radians());

        Self {
            encoder,
            direction,
            wheel_circum: wheel_diameter * PI,
            from_center,
            angle,
            prev_position,
        }
    }

    pub fn from_center(&self) -> Vec2<Length> {
        self.from_center
    }

    pub fn angle(&self) -> Angle {
        self.angle
    }

    pub fn traveled(&mut self) -> Length {
        let position =
            Angle::new::<radian>(self.encoder.position().unwrap_or_default().as_radians())
                * match self.direction {
                    Direction::Forward => 1.0,
                    Direction::Reverse => -1.0,
                };
        let change = position - self.prev_position;
        self.prev_position = position;

        self.wheel_circum * change.get::<revolution>()
    }
}
