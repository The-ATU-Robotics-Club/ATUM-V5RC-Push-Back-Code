use core::time::Duration;

use log::debug;
use vexide::{
    prelude::{Float, Motor},
    time::{sleep, Instant},
};

use crate::{
    controllers::pid::Pid,
    math::angle::Angle,
    pose::Vec2,
    subsystems::drivetrain::Drivetrain,
};

pub struct Turn {
    pid: Pid,
    tolerance: Angle,
}

impl Turn {
    pub fn new(pid: Pid, tolerance: Angle) -> Self {
        Self { pid, tolerance }
    }

    pub async fn turn_to_point(&mut self, dt: &mut Drivetrain, point: Vec2, timeout: Duration) {
        let pose = dt.get_pose();
        let target = pose.angular_distance(point);
        self.turn_to(dt, target, timeout).await;
    }

    pub async fn turn_to(&mut self, dt: &mut Drivetrain, target: Angle, timeout: Duration) {
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::WRITE_INTERVAL).await;
            time += Motor::WRITE_INTERVAL;
            let elapsed_time = prev_time.elapsed();
            prev_time = Instant::now();

            let heading = dt.get_pose().h;
            let error = (target - heading).wrap();
            let output = self.pid.output(error.as_radians(), elapsed_time);

            if error.abs() < self.tolerance {
                debug!("Turn complete");
                break;
            }

            if time > timeout {
                debug!("Turn interrupted");
                break;
            }

            dt.set_voltages(output, -output);
        }

        dt.set_voltages(0.0, 0.0);
    }
}
