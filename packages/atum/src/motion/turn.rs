use std::time::{Duration, Instant};

use vexide::{math::Angle, prelude::Motor, smart::motor::BrakeMode, time::sleep};

use super::{MotionError, MotionParameters, MotionResult};
use crate::{controllers::pid::Pid, localization::vec2::Vec2, subsystems::drivetrain::Drivetrain};

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
    pub async fn turn_to_point(
        &mut self,
        drivetrain: &mut Drivetrain,
        point: Vec2<f64>,
        reverse: bool,
    ) -> MotionResult<Angle> {
        let pose = drivetrain.pose().position();
        let mut target = Angle::from_radians((point - pose).angle());

        if reverse {
            target += Angle::HALF_TURN;
        }

        self.speed(self.params.speed).turn_to(drivetrain, target).await?;

        Ok(())
    }

    /// Rotates the robot to a specific heading.
    pub async fn turn_to(
        &mut self,
        drivetrain: &mut Drivetrain,
        target: Angle,
    ) -> MotionResult<Angle> {
        let start_time = Instant::now();
        let mut prev_time = start_time;

        // Reset PID parameters
        self.pid.reset();

        loop {
            // Run controller loop at 100 Hz
            sleep(Duration::from_millis(10)).await;

            let now = Instant::now();
            let dt = now - prev_time;
            prev_time = now;

            let heading = drivetrain.pose().h;
            let omega = drivetrain.pose().omega;

            // Compute shortest angular error between target and heading
            // `wrapped_half()` ensures the error stays within [-π, π)
            let error = (target - heading).wrapped_half();
            let output = self.pid.output(error.as_radians(), dt).clamp(-1.0, 1.0);

            // Motion is complete if:
            // 1. Angular error is within tolerance
            // 2. Angular velocity is sufficiently small (robot has settled)
            if error.abs() < self.params.tolerance
                && self
                    .params
                    .velocity_tolerance
                    .is_none_or(|tolerance| omega.abs() < tolerance)
            {
                break;
            }

            // Stop if the motion exceeds the allowed timeout
            if self
                .params
                .timeout
                .is_some_and(|timeout| start_time.elapsed() > timeout)
            {
                drivetrain.brake(BrakeMode::Brake);
                return Err(MotionError::Timeout(error));
            }

            // Apply opposite voltages to create rotation
            drivetrain.set_arcade(0.0, output * self.params.speed);
        }

        // Stop drivetrain after the turn completes
        drivetrain.brake(BrakeMode::Brake);

        Ok(())
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
