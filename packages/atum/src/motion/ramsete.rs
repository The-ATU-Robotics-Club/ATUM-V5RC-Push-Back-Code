use core::f64::consts::PI;

use uom::si::{angle::radian, f64::{AngularVelocity, Velocity}, length::inch};
use vexide::{
    prelude::{Direction, Float, Motor},
    time::{sleep, Instant},
};
use crate::{motion::profiling::trajectory, pose::{Pose, Vec2}, subsystems::drivetrain::Drivetrain, utils::wrapped};
use super::profiling::trajectory::Trajectory;




#[derive(PartialEq)]
pub struct Ramsete {
    pub b: f64,
    pub zeta: f64,
    pub track_width: f64,
    pub wheel_diameter: f64,
    pub external_gearing: f64,
}

#[inline]
pub const fn to_motor_rpm(in_sec: f64, wheel_diameter: f64, external_gearing: f64) -> f64 {
    (in_sec * 60.0) / (PI * wheel_diameter * external_gearing)
}

impl Ramsete {
    pub async fn follow (
        &mut self,
        drivetrain: &mut Drivetrain,
        trajectory: Trajectory,
    ) {
        let mut prev_position = Vec2::new(
            drivetrain.pose().x.get::<inch>(),
            drivetrain.pose().y.get::<inch>()
        );
        let mut distance = 0.3; // Starting on trajectory.profile[1] because trajectory.profile[0] has no starting velocity

        loop {
            sleep(Motor::WRITE_INTERVAL).await;

            let pose = drivetrain.pose();
            let position = Vec2::new(
                pose.x.get::<inch>(),
                pose.y.get::<inch>()
            );
            distance += position.distance(prev_position);
            prev_position = position;

            let profile = trajectory.at(distance);

            if *trajectory.profile.last().unwrap() == profile {
                break;
            }

            let desired_linear_velocity = profile.linear_velocity;
            let desired_angular_velocity = profile.angular_velocity;

            // Compute gain value `k`
            let k = 2.0
                * self.zeta
                * ((desired_angular_velocity * desired_angular_velocity)
                    + self.b * (desired_linear_velocity * desired_linear_velocity))
                    .sqrt();

            // Compute error in the local reference frame of the robot (+x is forward)
            let position_error =
                (profile.position - position).rotated(-pose.h.get::<radian>()); 
            let heading_error = profile.heading - pose.h.get::<radian>(); //swtiched pose.get::<radian>() and profile heading so we should have a negative heading error

            // Linear/angular velocity commands
            let angular_velocity = (desired_angular_velocity
                + k * wrapped(heading_error)
                + self.b
                    * desired_linear_velocity
                    * (heading_error.sin() / heading_error)
                    * position_error.y)
                / 2.0
                * self.track_width;
            let linear_velocity =
                desired_linear_velocity * heading_error.cos() + k * position_error.x;

            // rewrite velocities and normalization
            let mut velocities = (
                to_motor_rpm(
                    linear_velocity - angular_velocity,
                    self.wheel_diameter,
                    self.external_gearing,
                ),
                to_motor_rpm(
                    linear_velocity + angular_velocity,
                    self.wheel_diameter,
                    self.external_gearing,
                ),
            );
            let larger_magnitude = velocities.0.abs().max(velocities.1.abs()) / 450.0;

            if larger_magnitude > 1.0 {
                velocities.0 /= larger_magnitude;
                velocities.1 /= larger_magnitude;
            }


            // Spin motors with builtin PID for now.
            drivetrain.set_velocity(velocities.0 as i32, velocities.1 as i32);
        }

        _ = drivetrain.set_voltages(0.0, 0.0);
    }
}
