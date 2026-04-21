//! Motor Abstractions
//!
//! This module provides unified interfaces for controlling one or more motors.
//! It abstracts away direct motor calls, allowing PID control, feedforward,
//! and bulk operations on groups of motors.
//!
//! Submodules / structs:
//! - `MotorGroup` – Represents a group of motors that should move together.
//! - `MotorController` – PID + feedforward controller for a motor.

use std::time::Instant;
use vexide::{prelude::Motor, smart::motor::BrakeMode};
use super::average;
use crate::controllers::pid::Pid;

/// A group of motors that can be controlled together
pub struct MotorGroup {
    motors: Vec<Motor>,
    motor_controller: Option<MotorController>,
}

impl MotorGroup {
    /// Create a new motor group with optional motor controller for closed-loop control
    pub fn new(motors: Vec<Motor>, motor_controller: Option<MotorController>) -> Self {
        Self {
            motors,
            motor_controller,
        }
    }

    /// Apply the same voltage to all motors in the group
    pub fn set_voltage(&mut self, voltage: f64) {
        for motor in self.motors.iter_mut() {
            _ = motor.set_voltage(voltage);
        }
    }

    /// Apply a velocity setpoint to all motors, using VEX's default velocity controller
    /// if no velocity controller is provided
    pub fn set_velocity(&mut self, velocity: f64) {
        for motor in self.motors.iter_mut() {
            match self.motor_controller {
                Some(mut controller) => {
                    let motor_velocity = motor.velocity().unwrap_or_default();
                    let voltage = controller.output(velocity, motor_velocity, 0.0);
                    _ = motor.set_voltage(voltage);
                }
                None => {
                    _ = motor.set_velocity(velocity as i32);
                }
            }
        }
    }

    /// Set the brake mode for all motors in the group
    pub fn brake(&mut self, brake: BrakeMode) {
        for motor in self.motors.iter_mut() {
            _ = motor.brake(brake);
        }
    }

    /// Compute the average voltage of all motors in the group
    pub fn voltage(&self) -> f64 {
        let mut voltages = Vec::new();
        for motor in self.motors.iter() {
            if let Ok(voltage) = motor.voltage() {
                voltages.push(voltage);
            }
        }
        average(&voltages)
    }

    /// Compute the average velocity of all motors in the group
    pub fn velocity(&self) -> f64 {
        let mut velocities = Vec::new();
        for motor in self.motors.iter() {
            if let Ok(velocity) = motor.velocity() {
                velocities.push(velocity);
            }
        }
        average(&velocities)
    }

    /// Get a mutable iterator over the motors
    pub fn iter_mut(&mut self) -> core::slice::IterMut<'_, Motor> {
        self.motors.iter_mut()
    }
}

/// PID + Feedforward motor controller
#[derive(Clone, Copy)]
pub struct MotorController {
    pid: Pid,
    ks: f64, // static friction voltage
    kv: f64, // velocity feedforward
    ka: f64, // acceleration feedforward
    time: Instant,
}

impl MotorController {
    /// Create a new MotorController with PID and feedforward gains
    pub fn new(pid: Pid, ks: f64, kv: f64, ka: f64) -> Self {
        Self {
            pid,
            ks,
            kv,
            ka,
            time: Instant::now(),
        }
    }

    /// Compute the voltage output for a motor based on a target RPM, actual RPM, and acceleration
    pub fn output(&mut self, target_rpm: f64, actual_rpm: f64, acceleration: f64) -> f64 {
        let now = Instant::now();
        let dt = now.duration_since(self.time);
        self.time = now;

        let error = target_rpm - actual_rpm;

        // Feedforward: static friction + velocity + acceleration
        let ff = if target_rpm.abs() > 1e-6 {
            self.ks * target_rpm.signum() + self.kv * target_rpm + self.ka * acceleration
        } else {
            0.0
        };

        // PID correction
        let pid = self.pid.output(error, dt);

        ff + pid
    }
}
