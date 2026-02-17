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
    velocity_tolerance: Option<AngularVelocity>,
    timeout: Option<Duration>,
    tolerance_scale: f64,
}

impl Turn {
    pub fn new(pid: Pid, tolerance: Angle) -> Self {
        Self {
            pid,
            tolerance,
            velocity_tolerance: None,
            timeout: None,
            tolerance_scale: 1.0,
        }
    }

    pub async fn turn_to_point(
        &mut self,
        dt: &mut Drivetrain,
        point: Vec2<Length>,
    ) {
        let pose = dt.pose();
        let target = angular_distance(pose, point);
        self.turn_to(dt, target).await;
    }

    pub async fn turn_to(&mut self, dt: &mut Drivetrain, target: Angle) {
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
                "(Error, Velocity): ({}, {})",
                error.get::<degree>(),
                omega.get::<degree_per_second>()
            );
            if error.abs() < self.tolerance * self.tolerance_scale
                && self
                    .velocity_tolerance
                    .is_none_or(|tolerance| omega.abs() < tolerance)
            {
                info!(
                    "Turn complete at: {} with {}ms",
                    starting_error.get::<degree>(),
                    time.as_millis()
                );
                break;
            }

            if error.abs() < self.tolerance {
                debug!("time: {}", time.as_millis());
            }

            if self.timeout.is_some_and(|timeout| time > timeout) {
                warn!("Turn interrupted at: {}", starting_error.get::<degree>());
                break;
            }

            dt.set_voltages(-output, output);
        }

        self.velocity_tolerance = None;
        self.timeout = None;
        self.tolerance_scale = 1.0;

        dt.set_voltages(0.0, 0.0);
    }

    pub fn settle_velocity(&mut self, velocity: AngularVelocity) -> &mut Self {
        self.velocity_tolerance = Some(velocity);
        self
    }

    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.timeout = Some(duration);
        self
    }

    pub fn chain(&mut self, scale: f64) -> &mut Self {
        self.tolerance_scale = scale;
        self
    }
}
