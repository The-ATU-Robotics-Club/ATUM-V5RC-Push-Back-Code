// Rewrite UOM implementation when std support gets stabilized

use core::time::Duration;

use log::{debug, info, warn};
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, Length, Velocity},
    length::inch,
};
use vexide::{
    prelude::{Direction, Float, Motor},
    time::{sleep, Instant},
};

use crate::{controllers::pid::Pid, pose::Vec2, subsystems::drivetrain::Drivetrain, utils::wrap};

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
            sleep(Motor::WRITE_INTERVAL).await;
            let elapsed_time = prev_time.elapsed();
            prev_time = Instant::now();

            let pose = dt.get_pose();
            let position = Vec2::new(pose.x, pose.y);
            let heading = pose.h;

            let position_error = target - position;
            let position_error = Vec2::new(
                position_error.x.get::<inch>(),
                position_error.y.get::<inch>(),
            );
            let distance = position_error.magnitude();
            let linear_output = self
                .linear
                .output(distance, elapsed_time)
                .clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);
            let target_h = Angle::new::<radian>(position_error.angle());

            if distance.abs() < self.tolerance.get::<inch>()
                && pose.vf.abs() < self.velocity_tolerance
            {
                info!("turn success");
                break;
            }

            if start_time.elapsed() > timeout {
                warn!("Moving failed");
                break;
            }

            let mut herror = wrap(heading - target_h + Angle::new::<degree>(90.0));
            let scaling = herror.get::<radian>().cos();

            debug!("d, a : {:?}, {:?}", distance, herror.get::<degree>());

            if direction.is_reverse() || herror.abs() > Angle::new::<degree>(90.0) {
                herror = wrap(herror + Angle::HALF_TURN);
            }

            let angular_output = if distance < self.turn_threshold.get::<inch>() {
                0.0
            } else {
                self.angular.output(herror.get::<radian>(), elapsed_time)
            };

            debug!("({}, {})", linear_output * scaling, angular_output);

            // do cosine scaling on rewrite: herror.cos() * linear_output
            dt.arcade(linear_output * scaling, angular_output);
        }

        dt.set_voltages(0.0, 0.0);
    }
}
