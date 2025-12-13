use std::time::{Duration, Instant};

use log::{debug, info};
use uom::{
    si::{
        f64::{Length, Time, Velocity},
        length::{inch, meter},
        time::second,
        velocity::inch_per_second,
    },
    ConstZero,
};
use vexide::time::sleep;

use crate::{controllers::pid::Pid, localization::vec2::Vec2, subsystems::drivetrain::Drivetrain};

pub struct Linear {
    pid: Pid,
    tolerance: Length,
    velocity_tolerance: Velocity,
}

impl Linear {
    pub fn new(pid: Pid, tolerance: Length, velocity_tolerance: Velocity) -> Self {
        Self {
            pid,
            tolerance,
            velocity_tolerance,
        }
    }

    pub async fn drive_to_point(
        &mut self,
        dt: &mut Drivetrain,
        point: Vec2<Length>,
        timeout: Duration,
    ) {
        let point = Vec2::new(point.x.get::<meter>(), point.y.get::<meter>());
        let pose = Vec2::new(dt.pose().x.get::<meter>(), dt.pose().y.get::<meter>());
        let target_distance = Length::new::<meter>((point - pose).length());
        self.drive_distance(dt, target_distance, timeout).await;
    }

    pub async fn drive_distance(&mut self, dt: &mut Drivetrain, target: Length, timeout: Duration) {
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();

        let mut traveled = Length::ZERO;

        loop {
            sleep(Duration::from_millis(10)).await;
            let elapsed_time = prev_time.elapsed();
            time += elapsed_time;
            prev_time = Instant::now();

            // add the total distance traveled to error
            let pose = dt.pose();
            traveled += pose.vf * Time::new::<second>(elapsed_time.as_secs_f64());
            let error = target - traveled;
            let output = self.pid.output(error.get::<meter>(), elapsed_time);

            debug!(
                "(Distance, Velocity): ({}, {})",
                traveled.get::<inch>(),
                pose.vf.get::<inch_per_second>()
            );

            if (error.abs() < self.tolerance && pose.vf.abs() < self.velocity_tolerance)
                || time > timeout
            {
                info!("Time: {}", time.as_millis());
                break;
            }

            dt.set_voltages(output, output);
        }

        dt.set_voltages(0.0, 0.0);
    }
}
