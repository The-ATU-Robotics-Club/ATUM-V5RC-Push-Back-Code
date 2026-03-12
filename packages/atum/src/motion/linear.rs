use std::time::{Duration, Instant};

use log::info;
use vexide::{prelude::Motor, time::sleep};

use crate::{
    controllers::pid::Pid, localization::vec2::Vec2, motion::MotionParameters,
    subsystems::drivetrain::Drivetrain,
};

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
    pub async fn drive_to_point(&mut self, dt: &mut Drivetrain, point: Vec2<f64>, reverse: bool) {
        let pose = Vec2::new(dt.pose().x, dt.pose().y);
        let mut target_distance = (point - pose).length();

        if reverse {
            target_distance *= -1.0;
        }

        self.drive_distance(dt, target_distance).await;
    }

    /// Drives the robot forward or backward by a specified distance.
    pub async fn drive_distance(&mut self, dt: &mut Drivetrain, target: f64) {
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();

        // Estimated distance traveled during this motion
        let mut traveled = 0.0;

        // Reset PID parameters
        self.pid.reset();

        loop {
            // Run controller at 100 Hz
            sleep(Duration::from_millis(10)).await;

            let elapsed_time = prev_time.elapsed();
            time += elapsed_time;
            prev_time = Instant::now();

            // Integrate forward velocity to estimate distance traveled
            let pose = dt.pose();
            traveled += pose.vf * elapsed_time.as_secs_f64();

            // Remaining distance to target
            let error = target - traveled;

            // PID controller output (clamped to voltage limits)
            let output = self
                .pid
                .output(error, elapsed_time)
                .clamp(self.params.speed, self.params.speed);

            // Motion is complete if:
            // 1. Error is within tolerance
            // 2. Velocity is sufficiently small (robot has settled)
            if error.abs() < self.params.tolerance
                && (self
                    .params
                    .velocity_tolerance
                    .is_none_or(|tolerance| pose.vf.abs() < tolerance))
            {
                info!("turn success");
                break;
            }

            // Stop if the motion exceeds the allowed timeout
            if self.params.timeout.is_some_and(|timeout| time > timeout) {
                break;
            }

            // Apply equal voltage to both sides to drive straight
            dt.set_voltages(output, output);
        }

        // Stop drivetrain after motion completes
        dt.set_voltages(0.0, 0.0);
    }

    /// Sets the distance tolerance required for success.
    pub fn chain(&mut self, tolerance: f64) -> &mut Self {
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
        self.params.speed = Motor::V5_MAX_VOLTAGE * speed;
        self
    }
}

