use std::time::{Duration, Instant};

use log::{info, warn};
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, AngularVelocity, Length},
    length::meter,
};
use vexide::time::sleep;

use crate::{controllers::pid::Pid, subsystems::drivetrain::Drivetrain, utils::wrap};

pub struct Swing {
    pid: Pid,
    tolerance: Angle,
    velocity_tolerance: AngularVelocity,
}

impl Swing {
    pub fn new(pid: Pid, tolerance: Angle, velocity_tolerance: AngularVelocity) -> Self {
        Self {
            pid,
            tolerance,
            velocity_tolerance,
        }
    }

    pub async fn swing_to(
        &mut self,
        dt: &mut Drivetrain,
        target: Angle,
        radius: Length,
        timeout: Duration,
    ) {
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

            if error.abs() < self.tolerance && omega.abs() < self.velocity_tolerance {
                info!(
                    "Swing complete at: {} with {}ms",
                    starting_error.get::<degree>(),
                    time.as_millis()
                );
                break;
            }

            if time > timeout {
                warn!("Swing interrupted at: {}", starting_error.get::<degree>());
                break;
            }

            let left = output * (radius - length / 2.0);
            let right = output * (radius + length / 2.0);
            dt.set_velocity(left.get::<meter>() as i32, right.get::<meter>() as i32);
        }

        dt.set_voltages(0.0, 0.0);
    }
}
