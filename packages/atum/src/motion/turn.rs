use std::time::{Duration, Instant};

use log::{debug, info};
use vexide::{math::Angle, prelude::Motor, time::sleep};

use crate::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    motion::MotionParameters,
    subsystems::drivetrain::Drivetrain,
};

/// Controller responsible for rotational robot movement.
///
/// Uses a PID loop to rotate the robot to a desired heading
/// or toward a specific point on the field.
pub struct Turn {
    /// PID controller used for heading correction
    pid: Pid,

    /// Motion configuration parameters
    params: MotionParameters<Angle>,
}

impl Turn {
    /// Creates a new turn controller.
    pub fn new(pid: Pid, params: MotionParameters<Angle>) -> Self {
        Self { pid, params }
    }

    /// Rotates the robot to face a target point.
    ///
    /// The heading to the point is calculated using `atan2`
    /// and then passed to `turn_to`.
    pub async fn turn_to_point(&mut self, dt: &mut Drivetrain, point: Vec2<f64>, reverse: bool) {
        let pose = Vec2::new(dt.pose().x, dt.pose().y);
        let mut target = Angle::from_radians((point - pose).angle());

        if reverse {
            target += Angle::HALF_TURN;
        }

        self.turn_to(dt, target).await;
    }

    /// Rotates the robot to a specific heading.
    pub async fn turn_to(&mut self, dt: &mut Drivetrain, target: Angle) {
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();

        // Reset PID parameters
        self.pid.reset();

        loop {
            // Run controller loop at 100 Hz
            sleep(Duration::from_millis(10)).await;

            let elapsed_time = prev_time.elapsed();
            time += elapsed_time;
            prev_time = Instant::now();

            // Current robot heading
            let heading = dt.pose().h;

            // Compute shortest angular error between target and heading
            // `wrapped_half()` ensures the error stays within [-π, π)
            let error = (target - heading).wrapped_half();

            // PID output converted to motor voltage
            let output = self
                .pid
                .output(error.as_radians(), elapsed_time)
                .clamp(-self.params.speed, self.params.speed);

            // Current angular velocity from odometry
            let omega = dt.pose().omega;

            debug!("(Error, Velocity): ({}, {})", error.as_degrees(), omega);

            // Motion is complete if:
            // 1. Angular error is within tolerance
            // 2. Angular velocity is sufficiently small (robot has settled)
            if error.abs() < self.params.tolerance
                && self
                    .params
                    .velocity_tolerance
                    .is_none_or(|tolerance| omega.abs() < tolerance)
            {
                info!(
                    "Turn complete at: {} with {}ms",
                    target.as_degrees(),
                    time.as_millis(),
                );
                break;
            }

            // Stop if the motion exceeds the allowed timeout
            if self.params.timeout.is_some_and(|timeout| time > timeout) {
                break;
            }

            // Apply opposite voltages to create rotation
            dt.set_voltages(output, -output);
        }

        // Stop drivetrain after the turn completes
        dt.set_voltages(0.0, 0.0);
    }

    /// Sets the angular tolerance required to finish the turn.
    pub fn tolerance(&mut self, tolerance: Angle) -> &mut Self {
        self.params.tolerance = tolerance;
        self
    }

    /// Sets the angular velocity threshold used to determine
    /// when the robot has settled.
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
