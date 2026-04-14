pub mod betterthanbetterthanlaykeauto;
pub mod inch;
pub mod skills;

use atum::{controllers::pid::Pid, localization::vec2::Vec2};

pub const LINEAR_PID: Pid = Pid::new(0.06, 0.02, 0.004, 5.0);
pub const ANGULAR_PID: Pid = Pid::new(0.78, 0.05, 0.05, 13.5);
