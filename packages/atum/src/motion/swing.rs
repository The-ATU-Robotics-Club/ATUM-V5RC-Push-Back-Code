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

    pub async fn swing_to(&mut self, dt: &mut Drivetrain, target: Angle, radius: Inch, timeout: Duration){
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();

        let starting_error = wrap(target - dt.pose().h).abs();
        let length = dt.track();


        loop{
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

            dt.set_voltages(output*(radius-length/2), output*(radius+length/2));

        }
    }
}


