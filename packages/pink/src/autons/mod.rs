pub mod rushcontrol;
pub mod skills;

use atum::{controllers::pid::Pid, localization::vec2::Vec2};

const LINEAR_PID: Pid = Pid::new(1.17/12.0, 0.0, 0.1/12.0, 0.3);
const ANGULAR_PID: Pid = Pid::new(17.0/12.0, 0.75/12.0, 1.1/12.0, 40.0);

const RED_LEFT_LOADER: Vec2<f64> = Vec2::new(27.0, 2.5);
const RED_RIGHT_LOADER: Vec2<f64> = Vec2::new(144.0 - 24.0, 2.5);
const BLUE_LEFT_LOADER: Vec2<f64> = mirror_coordinate(RED_LEFT_LOADER);
const BLUE_RIGHT_LOADER: Vec2<f64> = mirror_coordinate(RED_RIGHT_LOADER);

const RED_LEFT_GOAL: Vec2<f64> = Vec2::new(24.0, 48.0);
const RED_RIGHT_GOAL: Vec2<f64> = Vec2::new(144.0 - 27.5, 48.0);
const BLUE_LEFT_GOAL: Vec2<f64> = mirror_coordinate(RED_LEFT_GOAL);
const BLUE_RIGHT_GOAL: Vec2<f64> = mirror_coordinate(RED_RIGHT_GOAL);

#[inline]
const fn mirror_coordinate(coordinate: Vec2<f64>) -> Vec2<f64> {
    Vec2::new(144.0 - coordinate.x, 144.0 - coordinate.y)
}
