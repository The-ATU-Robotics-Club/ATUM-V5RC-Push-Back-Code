pub mod drivetrain;
pub mod intake;

#[derive(Clone, Copy)]
pub struct RobotSettings {
    pub color: Color,
    pub enable_color: bool,
}

#[derive(Clone, Copy)]
pub enum Color {
    Red,
    Blue,
}
