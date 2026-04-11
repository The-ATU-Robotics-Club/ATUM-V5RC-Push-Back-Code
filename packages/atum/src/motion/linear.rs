use std::time::{Duration, Instant};

use vexide::time::sleep;

use super::{MotionError, MotionParameters, MotionResult};
use crate::{controllers::pid::Pid, localization::vec2::Vec2, subsystems::drivetrain::Drivetrain};

/// Linear motion controller.
///
/// Uses a PID controller to drive the robot forward or backward
/// to reach a target distance or position.
pub struct Linear {
    /// PID controller used for distance control
    pid: Pid,

    /// Motion configuration parameters
    params: MotionParameters<f64>,
}

impl Linear {
    /// Creates a new linear motion controller.
    pub fn new(pid: Pid, params: MotionParameters<f64>) -> Self {
        Self { pid, params }
    }

    /// Drives the robot to a target point on the field.
    ///
    /// The distance to the point is computed and then passed
    /// to `drive_distance`.
    pub async fn drive_to_point(
        &mut self,
        drivetrain: &mut Drivetrain,
        point: Vec2<f64>,
        reverse: bool,
    ) -> MotionResult<f64> {
        let pose = drivetrain.pose().position();
        let mut target_distance = (point - pose).length();

        if reverse {
            target_distance *= -1.0;
        }

        self.drive_distance(drivetrain, target_distance).await?;

        Ok(())
    }

    /// Drives the robot forward or backward by a specified distance.
    pub async fn drive_distance(
        &mut self,
        drivetrain: &mut Drivetrain,
        target: f64,
    ) -> MotionResult<f64> {
        let start_time = Instant::now();
        let mut prev_time = start_time;

        // Estimated distance traveled during this motion
        let mut traveled = 0.0;

        // Reset PID parameters
        self.pid.reset();

        loop {
            // Run controller at 100 Hz
            sleep(Duration::from_millis(10)).await;

            let now = Instant::now();
            let dt = now - prev_time;
            prev_time = now;

            let pose = drivetrain.pose();

            // Integrate forward velocity to estimate distance traveled
            traveled += pose.vf * dt.as_secs_f64();
            let error = target - traveled;
            let output = self.pid.output(error, dt).clamp(-1.0, 1.0);

            // Motion is complete if:
            // 1. Error is within tolerance
            // 2. Velocity is sufficiently small (robot has settled)
            if error.abs() < self.params.tolerance
                && (self
                    .params
                    .velocity_tolerance
                    .is_none_or(|tolerance| pose.vf.abs() < tolerance))
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
                return Err(MotionError::Timeout(error));
            }

            // Apply equal voltage to both sides to drive straight
            drivetrain.set_arcade(output * self.params.speed, 0.0);
        }

        // Stop drivetrain after motion completes
        drivetrain.set_voltages(0.0, 0.0);

        Ok(())
    }

    /// Sets the distance tolerance required for success.
    pub fn tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.params.tolerance = tolerance;
        self
    }

    /// Sets the velocity threshold used to determine when the robot
    /// has settled at the target.
    pub fn settle_velocity(&mut self, velocity: f64) -> &mut Self {
        self.params.velocity_tolerance = Some(velocity);
        self
    }

    /// Sets a timeout for the motion.
    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.params.timeout = Some(duration);
        self
    }

    /// Scales the maximum speed used for the motion.
    pub fn speed(&mut self, speed: f64) -> &mut Self {
        self.params.speed = speed;
        self
    }
}
