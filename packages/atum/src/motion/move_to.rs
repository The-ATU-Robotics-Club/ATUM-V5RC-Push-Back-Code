use std::time::{Duration, Instant};

use log::{debug, info, warn};
use uom::si::{
    angle::radian,
    f64::{Angle, Length, Velocity},
    length::meter,
};
use vexide::{prelude::Motor, time::sleep};

use crate::{
    controllers::pid::Pid, localization::vec2::Vec2, motion::desaturate,
    subsystems::drivetrain::Drivetrain, utils::wrap,
};

pub struct MoveTo {
    linear: Pid,
    sideways: Pid,
    tolerance: Length,
    velocity_tolerance: Option<Velocity>,
    timeout: Option<Duration>,
    tolerance_scale: f64,
}

impl MoveTo {
    pub fn new(linear: Pid, sideways: Pid, tolerance: Length) -> Self {
        Self {
            linear,
            sideways,
            tolerance,
            velocity_tolerance: None,
            timeout: None,
            tolerance_scale: 1.0,
        }
    }

    pub async fn move_to_point(&mut self, dt: &mut Drivetrain, target: Vec2<Length>) {
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

            if distance.abs() < self.tolerance.get::<meter>() * self.tolerance_scale
                && (self
                    .velocity_tolerance
                    .is_none_or(|tolerance| pose.vf.abs() < tolerance))
            {
                info!("turn success");
                break;
            }

            if self
                .timeout
                .is_some_and(|timeout| start_time.elapsed() > timeout)
            {
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
            let linear_output =
                self.linear.output(distance, elapsed_time) * herror.get::<radian>().cos().abs();

            debug!("Position: ({})", pose);

            let [left, right] = desaturate(
                [
                    linear_output + angular_output,
                    linear_output - angular_output,
                ],
                Motor::V5_MAX_VOLTAGE,
            );

            dt.set_voltages(left, right);
        }

        self.timeout = None;
        self.tolerance_scale = 1.0;
        self.velocity_tolerance = None;

        dt.set_voltages(0.0, 0.0);
    }

    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.timeout = Some(duration);
        self
    }

    pub fn chain(&mut self, scale: f64) -> &mut Self {
        self.tolerance_scale = scale;
        self
    }

    pub fn settle_velocity(&mut self, velocity: Velocity) -> &mut Self {
        self.velocity_tolerance = Some(velocity);
        self
    }
}
