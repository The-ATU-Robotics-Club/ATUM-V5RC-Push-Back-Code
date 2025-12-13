// Rewrite UOM implementation when std support gets stabilized

use std::time::{Duration, Instant};

use log::{debug, info, warn};
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, Length, Velocity},
    length::meter,
};
use vexide::{
    prelude::{Direction, Motor},
    time::sleep,
};

use crate::{
    controllers::pid::Pid, localization::vec2::Vec2, subsystems::drivetrain::Drivetrain,
    utils::wrap,
};

pub struct MoveTo {
    linear: Pid,
    angular: Pid,
    tolerance: Length,
    velocity_tolerance: Velocity,
    turn_threshold: Length,
}

impl MoveTo {
    pub fn new(
        linear: Pid,
        angular: Pid,
        tolerance: Length,
        velocity_tolerance: Velocity,
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
            let distance = position_error.magnitude();
            let linear_output = self
                .linear
                .output(distance, elapsed_time)
                .clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);
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

            let mut herror = wrap(target_h - heading);
            let scaling = herror.get::<radian>().cos();

            debug!("a, s: {:.4}, {:.4}", herror.get::<degree>(), scaling);

            if direction.is_reverse() || herror.abs() > Angle::new::<degree>(90.0) {
                herror = wrap(herror + Angle::HALF_TURN);
            }

            debug!("d, a, c: {:.4}, {:.4}", linear_output, herror.get::<degree>());

            let angular_output = if distance < self.turn_threshold.get::<meter>() {
                0.0
            } else {
                -self.angular.output(herror.get::<radian>(), elapsed_time)
            };

            debug!("la ({:.4}, {:.4})", linear_output * scaling, angular_output);
            debug!("lr ({:.4}, {:.4})", linear_output * scaling + angular_output, linear_output * scaling - angular_output);
            debug!("");

            dt.arcade(linear_output * scaling, angular_output);
        }

        dt.set_voltages(0.0, 0.0);
    }
}
