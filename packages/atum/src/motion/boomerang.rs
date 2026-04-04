use std::time::{Duration, Instant};

use vexide::{math::Angle, time::sleep};

use super::{MotionError, MotionParameters, MotionResult};
use crate::{controllers::pid::Pid, localization::{pose::Pose, vec2::Vec2}, subsystems::drivetrain::Drivetrain};

pub struct MoveToPose {
    linear: Pid,

    angular: Pid,

    params: MotionParameters<f64>,
}

impl MoveToPose {
    pub fn new(linear: Pid, angular: Pid, params: MotionParameters<f64>) -> Self {
        Self {
            linear,
            angular,
            params,
        }
    }

    pub async fn move_to_pose(
        &mut self,
        drivetrain: &mut Drivetrain,
        target: Pose,
        lead: f64,
    ) -> MotionResult<Pose> {
        let start_time = Instant::now();
        let mut prev_time = start_time;

        // Reset PID parameters
        self.linear.reset();
        self.angular.reset();
        
        loop{
            // Run control loop at 100Hz
            sleep(Duration::from_millis(10)).await;

            let now = Instant::now();
            let dt = now - prev_time;
            prev_time = now;

            let pose = drivetrain.pose();
            let position = pose.position();
            let heading = pose.h;

            let carrot = target.position()
            - Vec2::from_polar(
                (target.position() - position).length() * lead,
                target.h.as_radians(),
            );

            let position_error = carrot - position;

            let mut distance = position_error.length();
            // Desired heading toward the target

            let target_h = Angle::from_radians(position_error.angle());
            let mut herror = (target_h - heading).wrapped_half();

            if distance.abs() < self.params.tolerance
                && (self
                    .params
                    .velocity_tolerance
                    .is_none_or(|tolerance| pose.vf.abs() < tolerance))
            {
                break;
            }

            if self
                .params
                .timeout
                .is_some_and(|timeout| start_time.elapsed() > timeout)
            {
                drivetrain.set_voltages(0.0, 0.0);
                return Err(MotionError::Timeout(Pose::new(position_error.x,position_error.y, herror)));
            }

            if herror.abs() > Angle::QUARTER_TURN {
                herror *= -1.0;
                distance *= -1.0;
            }

            // Forward motion scaled by the alignment with the target
            let linear_output = self.linear.output(distance, dt) * herror.cos().abs();

            // Steering correction from cross-track error
            let angular_output = if distance < 6.0 {
                0.0
            } else {
                self.angular.output(herror.as_radians(), dt)
            };

            // Apply output to motors
            drivetrain.set_arcade(
                linear_output * self.params.speed,
                angular_output * self.params.speed,
            );
        }

        Ok(())
    }
}