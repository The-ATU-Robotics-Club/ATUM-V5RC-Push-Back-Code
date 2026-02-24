use std::time::{Duration, Instant};

use vexide::prelude::Motor;

use super::average;
use crate::controllers::pid::Pid;

pub struct MotorGroup {
    motors: Vec<Motor>,
    motor_controller: Option<MotorController>,
}

impl MotorGroup {
    pub fn new(motors: Vec<Motor>, motor_controller: Option<MotorController>) -> Self {
        Self {
            motors,
            motor_controller,
        }
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        for motor in self.motors.iter_mut() {
            _ = motor.set_voltage(voltage);
        }
    }

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

    pub fn voltage(&self) -> f64 {
        let mut voltages = Vec::new();
        for motor in self.motors.iter() {
            if let Ok(voltage) = motor.voltage() {
                voltages.push(voltage);
            }
        }

        average(voltages)
    }

    pub fn velocity(&self) -> f64 {
        let mut velocities = Vec::new();
        for motor in self.motors.iter() {
            if let Ok(velocity) = motor.velocity() {
                velocities.push(velocity);
            }
        }

        average(velocities) // * ratio
    }

    pub fn iter_mut(&mut self) -> core::slice::IterMut<'_, Motor> {
        self.motors.iter_mut()
    }
}

#[derive(Clone, Copy)]
pub struct MotorController {
    pid: Pid,
    ks: f64,
    kv: f64,
    ka: f64,
    time: Instant,
}

impl MotorController {
    pub fn new(pid: Pid, ks: f64, kv: f64, ka: f64) -> Self {
        Self {
            pid,
            ks,
            kv,
            ka,
            time: Instant::now(),
        }
    }

    pub fn output(&mut self, target_rpm: f64, actual_rpm: f64, acceleration: f64) -> f64 {
        let now = Instant::now();
        let dt = now.duration_since(self.time);
        self.time = now;

        let error = target_rpm - actual_rpm;

        let ff = if target_rpm.abs() > 1e-6 {
            self.ks * target_rpm.signum()
                + self.kv * target_rpm
                + self.ka * acceleration
        } else {
            0.0
        };

        let pid = self.pid.output(error, dt);

        ff + pid
    }
}
