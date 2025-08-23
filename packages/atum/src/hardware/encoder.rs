use vexide::{
    devices::PortError,
    prelude::{AdiEncoder, AdiPort, Direction, Position},
};

pub struct Encoder<const TPR: u32> {
    encoder: AdiEncoder,
    direction: Direction,
}

impl<const TPR: u32> Encoder<TPR> {
    pub fn new(
        top_port: AdiPort,
        bottom_port: AdiPort,
        direction: Direction,
    ) -> Self {
        Self {
            encoder: AdiEncoder::new(top_port, bottom_port),
            direction,
        }
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
