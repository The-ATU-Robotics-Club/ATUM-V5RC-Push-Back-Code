use std::time::{Duration, Instant};

use log::{debug, info, warn};
use vexide::{math::Angle, prelude::Motor, time::sleep};

use crate::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    motion::{MotionParameters, desaturate},
    subsystems::drivetrain::Drivetrain,
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
    pub async fn move_to_point(&mut self, dt: &mut Drivetrain, target: Vec2<f64>) {
        let start_time = Instant::now();
        let mut prev_time = Instant::now();

        // Reset PID parameters
        self.linear.reset();
        self.lateral.reset();

        loop {
            // Run control loop at 100Hz
            sleep(Duration::from_millis(10)).await;

            let elapsed_time = prev_time.elapsed();
            prev_time = Instant::now();

            // Current robot pose
            let pose = dt.pose();
            let position = pose.position();
            let heading = pose.h;
            debug!("Position: ({})", pose);

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
                && (self
                    .params
                    .velocity_tolerance
                    .is_none_or(|tolerance| pose.vf.abs() < tolerance))
            {
                info!("move_to success");
                break;
            }

            // Stop if the motion exceeds the allowed timeout
            if self
                .params
                .timeout
                .is_some_and(|timeout| start_time.elapsed() > timeout)
            {
                warn!("Moving failed");
                break;
            }

            // Heading error between robot and target direction
            let herror = (target_h - heading).wrapped_half();

            // Sideways displacement relative to target direction
            let mut cross_track_error = distance * herror.sin();

            // If the target is behind the robot, invert control logic
            if herror.abs() > Angle::QUARTER_TURN {
                cross_track_error *= -1.0;
                distance *= -1.0;
            } 

            // Forward motion scaled by the alignment with the target
            let linear_output = self.linear.output(distance, elapsed_time) * herror.cos().abs();

            // Steering correction from cross-track error
            let angular_output = self.lateral.output(-cross_track_error, elapsed_time);

            // Combine forward and angular commands while scaling wheel
            // voltages proportionally so their ratio is preserved
            let [left, right] = desaturate(
                [
                    linear_output + angular_output,
                    linear_output - angular_output,
                ],
                self.params.speed,
            );

            // Apply output to motors
            dt.set_voltages(left, right);
        }

        // Stop drivetrain after motion completes
        dt.set_voltages(0.0, 0.0);
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
        self.params.speed = Motor::V5_MAX_VOLTAGE * speed;
        self
    }
}
