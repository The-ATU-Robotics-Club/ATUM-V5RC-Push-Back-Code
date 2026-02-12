// Rewrite UOM implementation when std support gets stabilized

use std::time::{Duration, Instant};

use log::{debug, info, warn};
use uom::si::{
    angle::radian,
    f64::{Angle, Length, Velocity},
    length::meter,
};
use vexide::{prelude::Motor, time::sleep};

use crate::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    subsystems::drivetrain::Drivetrain,
    utils::wrap,
};

pub struct MoveTo {
    linear: Pid,
    sideways: Pid,
    tolerance: Length,
    velocity_tolerance: Velocity,
}

impl MoveTo {
    pub fn new(
        linear: Pid,
        sideways: Pid,
        tolerance: Length,
        velocity_tolerance: Velocity,
    ) -> Self {
        Self {
            linear,
            sideways,
            tolerance,
            velocity_tolerance,
        }
    }

    pub async fn move_to_point(
        &mut self,
        dt: &mut Drivetrain,
        target: Vec2<Length>,
        timeout: Duration,
    ) {
        let start_time = Instant::now();
        let mut prev_time = Instant::now();
        debug!("attempting to go to: {:?}", target);

        loop {
            sleep(Duration::from_millis(10)).await;
            let elapsed_time = prev_time.elapsed();
            prev_time = Instant::now();

            let pose = dt.pose();
            let heading = pose.h;

            let position_error = Vec2::new(
                (target.x - pose.x).get::<meter>(),
                (target.y - pose.y).get::<meter>(),
            );
            let mut distance = position_error.length();
            let target_h = Angle::new::<radian>(position_error.angle());

            if distance.abs() < self.tolerance.get::<meter>()
                && pose.vf.abs() < self.velocity_tolerance
            {
                info!("turn success");
                break;
            }

            if start_time.elapsed() > timeout {
                warn!("Moving failed");
                break;
            }
            let herror = wrap(target_h - heading);

            let mut projected_cte = distance * herror.get::<radian>().sin();

            if herror.abs() > Angle::HALF_TURN / 2.0 {
                projected_cte *= -1.0;
                distance *= -1.0;
            }

            let angular_output = self.sideways.output(-projected_cte, elapsed_time);
            let linear_output = self
                .linear
                .output(distance, elapsed_time)
                .clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE)
                * herror.get::<radian>().cos().abs();

            debug!("Position: ({})", pose);

            dt.arcade(linear_output, angular_output);
        }

        dt.set_voltages(0.0, 0.0);
    }
}
