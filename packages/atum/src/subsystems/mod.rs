pub mod drivetrain;
pub mod intake;

#[derive(Clone, Copy)]
pub struct RobotSettings {
    color: Color,
    enable_color: bool,
}

#[derive(Clone, Copy)]
pub enum Color {
    Red,
    Blue,
}
