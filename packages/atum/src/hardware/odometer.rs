use core::f64::consts::PI;

use log::debug;
use vexide::{
    devices::PortError,
    prelude::{AdiEncoder, AdiPort, Direction, Position},
};

pub struct Odometer<const TPR: u32> {
    encoder: AdiEncoder,
    direction: Direction,
    wheel_diameter: f64,
    // wheel offset from the center of robot
    from_center: f64,
    prev_position: Position,
}

impl<const TPR: u32> Odometer<TPR> {
    pub fn new(
        top_port: AdiPort,
        bottom_port: AdiPort,
        direction: Direction,
        wheel_diameter: f64,
        from_center: f64,
    ) -> Self {
        let encoder = AdiEncoder::new(top_port, bottom_port);

        Self {
            encoder,
            direction,
            wheel_diameter,
            from_center,
            prev_position: Position::from_degrees(0.0),
        }
    }

    pub fn from_center(&self) -> f64 {
        self.from_center
    }

    pub fn traveled(&mut self) -> f64 {
        let position = self.position().unwrap_or_default();
        let change = position - self.prev_position;
        self.prev_position = position;

        change.as_revolutions() * self.wheel_diameter * PI
    }

    pub fn position(&self) -> Result<Position, PortError> {
        Ok(Position::from_ticks(
            self.encoder
                .position()?
                .as_ticks(AdiEncoder::TICKS_PER_REVOLUTION)
                * match self.direction {
                    Direction::Forward => 1,
                    Direction::Reverse => -1,
                },
            TPR,
        ))
    }
}
