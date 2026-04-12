pub mod betterthanbetterthanlaykeauto;
pub mod inch;

use atum::{controllers::pid::Pid, localization::vec2::Vec2};

pub const LINEAR_PID: Pid = Pid::new(0.1, 0.0, 0.008, 0.0);
pub const ANGULAR_PID: Pid = Pid::new(0.5, 0.05, 0.08, 13.5);
