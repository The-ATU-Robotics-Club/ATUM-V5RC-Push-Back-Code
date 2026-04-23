mod midthenrush;
mod rushthenmid;

use atum::controllers::pid::Pid;

pub const LINEAR_PID: Pid = Pid::new(0.06, 0.02, 0.004, 5.0);
pub const ANGULAR_PID: Pid = Pid::new(0.78, 0.05, 0.05, 13.5);