use core::time::Duration;

use log::{debug, info, warn};
use vexide::{
    prelude::{Direction, Float, Motor},
    time::{sleep, Instant},
};

use crate::{
    controllers::pid::Pid,
    pose::Vec2,
    subsystems::drivetrain::Drivetrain,
    units::{angle::IntoAngle, length::Length},
};

pub struct MoveTo {
    linear: Pid,
    angular: Pid,
    tolerance: Length,
    velocity_tolerance: Length,
    turn_threshold: Length,
}

impl MoveTo {
    pub fn new(
        linear: Pid,
        angular: Pid,
        tolerance: Length,
        velocity_tolerance: Length,
        turn_threshold: Length,
    ) -> Self {
        Self {
            linear,
            angular,
            tolerance,
            velocity_tolerance,
            turn_threshold,
        }
    }

    pub async fn move_to_point(
        &mut self,
        dt: &mut Drivetrain,
        target: Vec2<Length>,
        timeout: Duration,
        direction: Direction,
    ) {
        let start_time = Instant::now();
        let mut prev_time = Instant::now();
        debug!("attempting to go to: {:?}", target);

        let start_heading = dt.get_pose().h;
        loop {
            sleep(Motor::WRITE_INTERVAL).await;
            let elapsed_time = prev_time.elapsed();
            prev_time = Instant::now();

            let pose = dt.get_pose();
            let position = Vec2::new(pose.x, pose.y);
            let heading = pose.h;

            let position_error = target - position;
            let distance = position_error.magnitude();
            let mut linear_output = self.linear.output(distance.as_inches(), elapsed_time);
            linear_output = linear_output.clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);
            let target_h = if distance.abs() < self.turn_threshold {
                start_heading
            } else {
                position_error.angle().as_inches().rad()
            };

            // debug!(
            //     "(pose, target): ({}, ({}, {}))",
            //     pose,
            //     target.x.as_inches(),
            //     target.y.as_inches()
            // );

            if distance.abs() < self.tolerance && pose.vf.abs() < self.velocity_tolerance {
                info!("turn success");
                break;
            }

            if start_time.elapsed() > timeout {
                warn!("Moving failed");
                break;
            }

            let mut herror = (heading - target_h - 270.0.deg()).wrap();

            debug!(
                "d, a : {:?}, {:?}",
                distance.as_inches(),
                herror.as_degrees()
            );

            if direction.is_reverse() || herror.abs() > 90.0.deg() {
                linear_output *= -1.0;
                herror += 180.0.deg();
            }

            let angular_output = self
                .angular
                .output(herror.wrap().as_radians(), elapsed_time);

            debug!(
                "({}, {})",
                linear_output * herror.cos().as_radians() + angular_output,
                linear_output * herror.cos().as_radians() - angular_output
            );

            // do cosine scaling on rewrite: herror.cos() * linear_output
            dt.arcade(linear_output * herror.cos().as_radians(), angular_output);
        }

        dt.set_voltages(0.0, 0.0);
    }
}
