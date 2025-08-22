use core::{f64::consts::PI, time::Duration};

use vexide::{
    prelude::{Direction, Float, Motor},
    time::{sleep, Instant},
};

use crate::{
    controllers::pid::Pid,
    math::{angle::Angle, length::{IntoLength, Length}},
    pose::Vec2,
    subsystems::drivetrain::Drivetrain,
};

pub struct MoveTo {
    linear: Pid,
    angular: Pid,
    tolerance: Length,
    turn_threshold: Angle,
}

impl MoveTo {
    pub fn new(linear: Pid, angular: Pid, tolerance: Length, turn_threshold: Angle) -> Self {
        Self {
            linear,
            angular,
            tolerance,
            turn_threshold,
        }
    }

    pub async fn move_to_point(
        &mut self,
        dt: &mut Drivetrain,
        target: Vec2,
        timeout: Duration,
        direction: Direction,
    ) {
        let time = Duration::ZERO;
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::WRITE_INTERVAL).await;
            let elapsed_time = prev_time.elapsed();
            prev_time = Instant::now();

            let pose = dt.get_pose();
            let distance = pose.distance(target);
            let mut linear_output = self.linear.output(distance.as_inches(), elapsed_time).inch();
            let mut angle = pose.angular_distance(target);

            if linear_output.abs() < self.tolerance || time > timeout {
                break;
            }

            if matches!(direction, Direction::Reverse) {
                linear_output *= -1.0;
                angle += PI;
            }

            let herror = (angle - pose.h).wrap();
            let angular_output = self.angular.output(herror.as_radians(), elapsed_time);

            let left = herror.cos() * linear_output + angular_output;
            let right = herror.cos() * linear_output - angular_output;

            dt.set_voltages(left, right);
        }

        dt.set_voltages(0.0, 0.0);
    }
}
