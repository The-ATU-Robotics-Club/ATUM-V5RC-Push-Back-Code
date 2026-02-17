use std::time::{Duration, Instant};

use log::{info, warn};
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, AngularVelocity, Length},
    length::meter,
};
use vexide::{prelude::Gearset, time::sleep};

use crate::{
    controllers::pid::Pid, motion::desaturate, subsystems::drivetrain::Drivetrain, utils::wrap,
};

pub struct Swing {
    pid: Pid,
    tolerance: Angle,
    velocity_tolerance: Option<AngularVelocity>,
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

    pub async fn swing_to(&mut self, dt: &mut Drivetrain, target: Angle, radius: Length) {
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();

        let starting_error = wrap(target - dt.pose().h).abs();
        let length = dt.track();

        loop {
            sleep(Duration::from_millis(10)).await;
            let elapsed_time = prev_time.elapsed();
            time += elapsed_time;
            prev_time = Instant::now();

            let heading = dt.pose().h;
            let error = wrap(target - heading);
            let output = self.pid.output(error.get::<radian>(), elapsed_time);
            let omega = dt.pose().omega;

            if error.abs() < self.tolerance
                && self
                    .velocity_tolerance
                    .is_none_or(|tolerance| omega.abs() < tolerance)
            {
                info!(
                    "Swing complete at: {} with {}ms",
                    starting_error.get::<degree>(),
                    time.as_millis()
                );
                break;
            }

            if self.timeout.is_some_and(|timeout| time > timeout) {
                warn!("Swing interrupted at: {}", starting_error.get::<degree>());
                break;
            }

            let left = output * (radius - length / 2.0);
            let right = output * (radius + length / 2.0);

            let [left, right] = desaturate(
                [left.get::<meter>(), right.get::<meter>()],
                Gearset::MAX_BLUE_RPM,
            );

            dt.set_velocity(left, right);
        }

        self.velocity_tolerance = None;
        self.timeout = None;

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
}
