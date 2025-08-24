use core::time::Duration;

use log::{debug, warn};
use vexide::{
    prelude::{Float, Motor},
    time::{sleep, Instant},
};

use crate::{
    controllers::pid::Pid,
    pose::Vec2,
    subsystems::drivetrain::Drivetrain,
    units::{angle::Angle, length::Length},
};

pub struct Turn {
    small_pid: Pid,
    large_pid: Pid,
    tolerance: Angle,
    velocity_tolerance: Angle,
    threshold: Angle,
}

impl Turn {
    pub fn new(
        small_pid: Pid,
        large_pid: Pid,
        tolerance: Angle,
        velocity_tolerance: Angle,
        threshold: Angle,
    ) -> Self {
        Self {
            small_pid,
            large_pid,
            tolerance,
            velocity_tolerance,
            threshold,
        }
    }

    pub async fn turn_to_point(
        &mut self,
        dt: &mut Drivetrain,
        point: Vec2<Length>,
        timeout: Duration,
    ) {
        let pose = dt.get_pose();
        let target = pose.angular_distance(point);
        self.turn_to(dt, target, timeout).await;
    }

    pub async fn turn_to(&mut self, dt: &mut Drivetrain, target: Angle, timeout: Duration) {
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();

        let starting_error = (target - dt.get_pose().h).wrap().abs();
        let pid = if starting_error < self.threshold {
            &mut self.small_pid
        } else {
            &mut self.large_pid
        };

        loop {
            sleep(Motor::WRITE_INTERVAL).await;
            time += Motor::WRITE_INTERVAL;
            let elapsed_time = prev_time.elapsed();
            prev_time = Instant::now();

            let heading = dt.get_pose().h;
            let error = (target - heading).wrap();
            let output = pid.output(error.as_radians(), elapsed_time);
            let omega = dt.get_pose().omega;

            debug!(
                "(Heading, Velocity): ({}, {})",
                error.as_degrees(),
                omega.as_degrees()
            );
            if error.abs() < self.tolerance && omega.abs() < self.velocity_tolerance {
                debug!("Turn complete at: {}", starting_error.as_degrees());
                break;
            }

            if time > timeout {
                warn!("Turn interrupted at: {}", starting_error.as_degrees());
                break;
            }

            dt.set_voltages(output, -output);
        }

        dt.set_voltages(0.0, 0.0);
    }
}
