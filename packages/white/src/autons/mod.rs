pub mod rushelims;
pub mod skills;

use atum::{controllers::pid::Pid, localization::vec2::Vec2};

pub const LINEAR_PID: Pid = Pid::new(0.44, 0.00, 0.06, 12.0);
pub const ANGULAR_PID: Pid = Pid::new(1.67, 0.76, 0.19, 40.0);

const RED_LEFT_LOADER: Vec2<f64> = Vec2::new(28.0, 2.5);
const RED_RIGHT_LOADER: Vec2<f64> = Vec2::new(144.0 - 24.0, 2.5);
const BLUE_LEFT_LOADER: Vec2<f64> = mirror_coordinate(RED_LEFT_LOADER);
const BLUE_RIGHT_LOADER: Vec2<f64> = mirror_coordinate(RED_RIGHT_LOADER);

const RED_LEFT_GOAL: Vec2<f64> = Vec2::new(26.0, 48.0);
const RED_RIGHT_GOAL: Vec2<f64> = Vec2::new(144.0 - 25.0, 48.0);
const BLUE_LEFT_GOAL: Vec2<f64> = mirror_coordinate(RED_LEFT_GOAL);
const BLUE_RIGHT_GOAL: Vec2<f64> = mirror_coordinate(RED_RIGHT_GOAL);

#[inline]
const fn mirror_coordinate(coordinate: Vec2<f64>) -> Vec2<f64> {
    Vec2::new(144.0 - coordinate.x, 144.0 - coordinate.y)
}
