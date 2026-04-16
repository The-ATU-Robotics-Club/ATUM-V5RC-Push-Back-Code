use std::time::{Duration, Instant};

use vexide::{math::Angle, time::sleep};

use super::{MotionError, MotionParameters, MotionResult};
use crate::{
    controllers::pid::Pid, localization::vec2::Vec2, subsystems::drivetrain::Drivetrain,
    utils::desaturate,
};

/// Controller that drives the robot to a 2D point.
///
/// Uses two PID controllers:
/// - `linear` controls forward/backward motion toward the target
/// - `sideways` controls lateral error (cross-track error) to steer toward the point
///
/// This produces smooth arc-like paths instead of stopping to turn first.
pub struct MoveTo {
    /// Controls forward/backward distance to the target
    linear: Pid,

    /// Controls heading correction based on cross-track error
    lateral: Pid,

    /// Motion configuration parameters
    params: MotionParameters<f64>,
}

impl MoveTo {
    /// Creates a new move-to-point controller.
    pub fn new(linear: Pid, lateral: Pid, params: MotionParameters<f64>) -> Self {
        Self {
            linear,
            lateral,
            params,
        }
    }

    /// Drives the robot toward a target point in global coordinates.
    ///
    /// The robot simultaneously drives forward and steers toward the point
    /// rather than performing separate turn and drive motions.
    pub async fn move_to_point(
        &mut self,
        drivetrain: &mut Drivetrain,
        target: Vec2<f64>,
    ) -> MotionResult<Vec2<f64>> {
        let start_time = Instant::now();
        let mut prev_time = start_time;

        // Reset PID parameters
        self.linear.reset();
        self.lateral.reset();

        loop {
            // Run control loop at 100Hz
            sleep(Duration::from_millis(10)).await;

            let now = Instant::now();
            let dt = now - prev_time;
            prev_time = now;

            // Current robot pose
            let pose = drivetrain.pose();
            let position = pose.position();
            let heading = pose.h;

            // Vector from robot to target
            let position_error = target - position;

            // Distance to target
            let mut distance = position_error.length();

            // Desired heading toward the target
            let target_h = Angle::from_radians(position_error.angle());

            // Motion is complete if:
            // 1. Error is within tolerance
            // 2. Velocity is sufficiently small (robot has settled)
            if distance.abs() < self.params.tolerance
                && self
                    .params
                    .velocity_tolerance
                    .is_none_or(|tolerance| pose.vf.abs() < tolerance)
                || self.params.min_velocity.is_some_and(|velocity| {
                    pose.vf.abs() < velocity && distance.abs() < self.params.tolerance * 3.0
                })
            {
                break;
            }

            // Stop if the motion exceeds the allowed timeout
            if self
                .params
                .timeout
                .is_some_and(|timeout| start_time.elapsed() > timeout)
            {
                drivetrain.set_voltages(0.0, 0.0);
                return Err(MotionError::Timeout(position_error));
            }

            let herror = (target_h - heading).wrapped_half();
            let mut cross_track_error = distance * herror.sin();

            // If the target is behind the robot, invert control logic
            if herror.abs() > Angle::QUARTER_TURN {
                cross_track_error *= -1.0;
                distance *= -1.0;
            }

            let linear_output = self.linear.output(distance, dt) * herror.cos().abs();
            let angular_output = if distance.abs() < 3.0 {
                0.0
            } else {
                self.lateral.output(cross_track_error, dt)
            };

            let [left, right] = desaturate(
                [
                    linear_output + angular_output,
                    linear_output - angular_output,
                ],
                self.params.speed,
            );

            drivetrain.set_voltages(left, right);
        }

        // Stop drivetrain after motion completes
        drivetrain.set_voltages(0.0, 0.0);

        Ok(())
    }

    /// Sets the position tolerance required to finish the motion.
    pub fn tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.params.tolerance = tolerance;
        self
    }

    /// Sets the velocity threshold used to determine when the robot has settled.
    pub fn settle_velocity(&mut self, velocity: f64) -> &mut Self {
        self.params.velocity_tolerance = Some(velocity);
        self
    }

    /// Sets a timeout for the motion.
    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.params.timeout = Some(duration);
        self
    }

    /// Scales the maximum speed used during the motion.
    pub fn speed(&mut self, speed: f64) -> &mut Self {
        self.params.speed = speed;
        self
    }
}
