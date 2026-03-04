use std::time::{Duration, Instant};

use log::{info, warn};
use vexide::{math::Angle, prelude::Gearset, time::sleep};

use crate::{
    controllers::pid::Pid, motion::desaturate, subsystems::drivetrain::Drivetrain,
};

pub struct Swing {
    pid: Pid,
    tolerance: Angle,
    velocity_tolerance: Option<f64>,
    timeout: Option<Duration>,
}

impl Swing {
    pub fn new(pid: Pid, tolerance: Angle) -> Self {
        Self {
            pid,
            tolerance,
            velocity_tolerance: None,
            timeout: None,
        }
    }

    pub async fn swing_to(&mut self, dt: &mut Drivetrain, target: Angle, radius: f64) {
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();

        let length = dt.track();

        loop {
            sleep(Duration::from_millis(10)).await;
            let elapsed_time = prev_time.elapsed();
            time += elapsed_time;
            prev_time = Instant::now();

            let heading = dt.pose().h;
            let error = (target - heading).wrapped_half();
            let output = self.pid.output(error.as_radians(), elapsed_time);
            let omega = dt.pose().omega;

            if error.abs() < self.tolerance
                && self
                    .velocity_tolerance
                    .is_none_or(|tolerance| omega.abs() < tolerance)
            {
                info!(
                    "Swing complete at: {} with {}ms",
                    target.as_degrees(),
                    time.as_millis()
                );
                break;
            }

            if self.timeout.is_some_and(|timeout| time > timeout) {
                warn!("Swing interrupted at: {}", target.as_degrees());
                break;
            }

            let left = output * (radius - length / 2.0);
            let right = output * (radius + length / 2.0);

            let [left, right] = desaturate(
                [left, right],
                Gearset::MAX_BLUE_RPM,
            );

            dt.set_velocity(left, right);
        }

        self.velocity_tolerance = None;
        self.timeout = None;

        dt.set_voltages(0.0, 0.0);
    }

    pub fn settle_velocity(&mut self, velocity: f64) -> &mut Self {
        self.velocity_tolerance = Some(velocity);
        self
    }

    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.timeout = Some(duration);
        self
    }
}
