pub mod elims;
pub mod quals;
pub mod rushcontrol;
pub mod rushelims;
pub mod safequals;
pub mod skills;

use atum::{controllers::pid::Pid, localization::vec2::Vec2};
use uom::si::{f64::Length, length::inch};

const LINEAR_PID: Pid = Pid::new(46.0, 0.0, 3.95, 12.0);
const ANGULAR_PID: Pid = Pid::new(17.0, 0.75, 1.1, 40.0);
const _SETTLE_LIN_VEL: f64 = 2.5; // INCHES
const SETTLE_ANG_VEL: f64 = 15.0; // DEGREES

const RED_LEFT_LOADER: Vec2<f64> = Vec2::new(24.0, 2.5);
const RED_RIGHT_LOADER: Vec2<f64> = Vec2::new(144.0 - 24.0, 2.5);
const BLUE_LEFT_LOADER: Vec2<f64> = mirror_coordinate(RED_LEFT_LOADER);
const BLUE_RIGHT_LOADER: Vec2<f64> = mirror_coordinate(RED_RIGHT_LOADER);

const RED_LEFT_GOAL: Vec2<f64> = Vec2::new(24.0, 48.0);
const RED_RIGHT_GOAL: Vec2<f64> = Vec2::new(144.0 - 25.0, 48.0);
const BLUE_LEFT_GOAL: Vec2<f64> = mirror_coordinate(RED_LEFT_GOAL);
const BLUE_RIGHT_GOAL: Vec2<f64> = mirror_coordinate(RED_RIGHT_GOAL);

#[inline]
const fn mirror_coordinate(coordinate: Vec2<f64>) -> Vec2<f64> {
    Vec2::new(144.0 - coordinate.x, 144.0 - coordinate.y)
}

// UOM doesn't have const initialization so this function reduces the boilerplate required to
// convert the locations into units
//
// UOM will be removed after the first competition due to complications with writing clean code
#[inline]
fn vec2_length(coordinate: Vec2<f64>) -> Vec2<Length> {
    Vec2::new(
        Length::new::<inch>(coordinate.x),
        Length::new::<inch>(coordinate.y),
    )
}
