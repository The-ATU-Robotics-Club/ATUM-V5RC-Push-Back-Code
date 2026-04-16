use std::time::{Duration, Instant};

use vexide::{math::Angle, time::sleep};

use super::{MotionError, MotionParameters, MotionResult};
use crate::{controllers::pid::Pid, subsystems::drivetrain::Drivetrain};

/// Controller for performing a swing turn.
///
/// A swing turn rotates the robot around an arc instead of turning in place.
/// This allows the robot to follow a circular path with a specified radius.
///
/// If the radius equals half the track width, the robot pivots around one wheel.
/// Larger radii produce wider arcs.
pub struct Swing {
    /// PID controller used to track heading error
    pid: Pid,

    /// Shared motion parameters (tolerance, timeout, etc.)
    params: MotionParameters<Angle>,
}

impl Swing {
    /// Creates a new swing controller.
    pub fn new(pid: Pid, params: MotionParameters<Angle>) -> Self {
        Self { pid, params }
    }

    /// Rotates the robot toward a target heading while following
    /// a circular arc with the specified radius.
    ///
    /// `radius` is measured from the robot's center of rotation.
    pub async fn swing_to(
        &mut self,
        drivetrain: &mut Drivetrain,
        target: Angle,
        radius: f64,
    ) -> MotionResult<Angle> {
        let start_time = Instant::now();
        let mut prev_time = start_time;

        // Distance between the left and right wheels
        let length = drivetrain.track();

        // Reset PID parameters
        self.pid.reset();

        loop {
            // Run control loop at 100 Hz
            sleep(Duration::from_millis(10)).await;

            let now = Instant::now();
            let dt = now - prev_time;
            prev_time = now;

            // Current heading
            let heading = drivetrain.pose().h;

            // Shortest angular difference to the target
            let error = (target - heading).wrapped_half();

            let output = self.pid.output(error.as_radians(), dt);

            // Current angular velocity from odometry
            let omega = drivetrain.pose().omega;

            // Motion is complete if:
            // 1. Angular error is within tolerance
            // 2. Angular velocity is sufficiently small (robot has settled)
            if error.abs() < self.params.tolerance
                && self
                    .params
                    .velocity_tolerance
                    .is_none_or(|tolerance| omega.abs() < tolerance)
                || self.params.min_velocity.is_some_and(|velocity| {
                    omega.abs() < velocity && error.abs() < self.params.tolerance * 3.0
                })
            {
                break;
            }

            // Timeout safety
            if self
                .params
                .timeout
                .is_some_and(|timeout| start_time.elapsed() > timeout)
            {
                drivetrain.set_voltages(0.0, 0.0);
                return Err(MotionError::Timeout(error));
            }

            // Compute wheel velocities required to follow a circular arc.
            let left = output * (radius - length / 2.0);
            let right = output * (radius + length / 2.0);

            // Apply wheel velocities
            drivetrain.set_velocity(left * self.params.speed, right * self.params.speed);
        }

        // Stop drivetrain after motion completes
        drivetrain.set_voltages(0.0, 0.0);

        Ok(())
    }

    /// Sets the heading angular required to finish the swing.
    pub fn tolerance(&mut self, tolerance: Angle) -> &mut Self {
        self.params.tolerance = tolerance;
        self
    }

    /// Sets the angular velocity threshold used to determine when the robot has settled.
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
