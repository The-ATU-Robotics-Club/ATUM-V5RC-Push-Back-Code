use std::time::{Duration, Instant};

use log::{debug, info, warn};
use uom::si::{
    angle::{degree, radian},
    angular_velocity::degree_per_second,
    f64::{Angle, AngularVelocity, Length},
};
use vexide::time::sleep;

use crate::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    subsystems::drivetrain::Drivetrain,
    utils::{angular_distance, wrap},
};

pub struct Turn {
    pid: Pid,
    tolerance: Angle,
    velocity_tolerance: AngularVelocity,
}

impl Turn {
    pub fn new(pid: Pid, tolerance: Angle, velocity_tolerance: AngularVelocity) -> Self {
        Self {
            pid,
            tolerance,
            velocity_tolerance,
        }
    }

    pub async fn turn_to_point(
        &mut self,
        dt: &mut Drivetrain,
        point: Vec2<Length>,
        timeout: Duration,
    ) {
        let pose = dt.pose();
        let target = angular_distance(pose, point);
        self.turn_to(dt, target, timeout).await;
    }

    pub async fn turn_to(&mut self, dt: &mut Drivetrain, target: Angle, timeout: Duration) {
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();

        let starting_error = wrap(target - dt.pose().h).abs();

        loop {
            sleep(Duration::from_millis(10)).await;
            let elapsed_time = prev_time.elapsed();
            time += elapsed_time;
            prev_time = Instant::now();

            let heading = dt.pose().h;
            let error = wrap(target - heading);
            let output = self.pid.output(error.get::<radian>(), elapsed_time);
            let omega = dt.pose().omega;

            debug!(
                "(Heading, Velocity): ({}, {})",
                error.get::<degree>(),
                omega.get::<degree_per_second>()
            );
            if error.abs() < self.tolerance && omega.abs() < self.velocity_tolerance {
                info!(
                    "Turn complete at: {} with {}ms",
                    starting_error.get::<degree>(),
                    time.as_millis()
                );
                break;
            }

            if time > timeout {
                warn!("Turn interrupted at: {}", starting_error.get::<degree>());
                break;
            }

            dt.set_voltages(output, -output);
        }

        dt.set_voltages(0.0, 0.0);
    }
}
