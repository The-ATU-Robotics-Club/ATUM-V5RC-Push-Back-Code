use std::time::{Duration, Instant};

use log::{debug, info};
use vexide::{math::Angle, prelude::Motor, time::sleep};

use crate::{controllers::pid::Pid, localization::vec2::Vec2, subsystems::drivetrain::Drivetrain};

pub struct Turn {
    pid: Pid,
    tolerance: Angle,
    velocity_tolerance: Option<f64>,
    timeout: Option<Duration>,
    speed: f64,
    tolerance_scale: f64,
}

impl Turn {
    pub fn new(pid: Pid, tolerance: Angle) -> Self {
        Self {
            pid,
            tolerance,
            velocity_tolerance: None,
            timeout: None,
            speed: Motor::V5_MAX_VOLTAGE * 0.6,
            tolerance_scale: 1.0,
        }
    }

    pub async fn turn_to_point(&mut self, dt: &mut Drivetrain, point: Vec2<f64>, reverse: bool) {
        let pose = Vec2::new(dt.pose().x, dt.pose().y);
        let mut target = Angle::from_radians((pose - point).angle());

        if reverse {
            target += Angle::HALF_TURN;
        }

        self.turn_to(dt, target).await;
    }

    pub async fn turn_to(&mut self, dt: &mut Drivetrain, target: Angle) {
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();

        loop {
            sleep(Duration::from_millis(10)).await;
            let elapsed_time = prev_time.elapsed();
            time += elapsed_time;
            prev_time = Instant::now();

            let heading = dt.pose().h;
            let error = (target - heading).wrapped_half();
            let output = self
                .pid
                .output(error.as_radians(), elapsed_time)
                .clamp(-self.speed, self.speed);
            let omega = dt.pose().omega;

            debug!("(Error, Velocity): ({}, {})", error.as_degrees(), omega);
            if error.abs() < self.tolerance * self.tolerance_scale
                && self
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

            if self.timeout.is_some_and(|timeout| time > timeout) {
                break;
            }

            dt.set_voltages(-output, output);
        }

        self.velocity_tolerance = None;
        self.timeout = None;
        self.tolerance_scale = 1.0;

        dt.set_voltages(0.0, 0.0);
    }

    pub fn settle_velocity(&mut self, velocity: f64) -> &mut Self {
        self.velocity_tolerance = Some(velocity);
        self
    }

    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.timeout = Some(duration);
        self
    }

    pub fn chain(&mut self, scale: f64) -> &mut Self {
        self.tolerance_scale = scale;
        self
    }
}
