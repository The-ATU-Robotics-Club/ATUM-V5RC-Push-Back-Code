use std::f64::consts::PI;

use vexide::{
    adi::AdiPort,
    math::Angle,
    prelude::AdiEncoder,
};

use crate::localization::vec2::Vec2;

pub struct TrackingWheel {
    encoder: AdiEncoder<4096>,
    wheel_circum: f64,
    from_center: Vec2<f64>,
    angle: Angle,
    prev_position: Angle,
}

impl TrackingWheel {
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

    pub fn from_center(&self) -> Vec2<f64> {
        self.from_center
    }

    pub fn angle(&self) -> Angle {
        self.angle
    }

    pub fn traveled(&mut self) -> f64 {
        let position = self.encoder.position().unwrap_or_default();
        let change = position - self.prev_position;
        self.prev_position = position;

        self.wheel_circum * change.as_turns()
    }
}
