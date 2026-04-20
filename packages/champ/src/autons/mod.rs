use atum::controllers::pid::Pid;

pub const LINEAR_PID: Pid = Pid::new(0.1, 0.0, 0.009, 0.0);
pub const ANGULAR_PID: Pid = Pid::new(0.67, 0.05, 0.067, 13.5);
