use alloc::vec::Vec;

use vexide::prelude::Motor;

pub struct MotorGroup {
    motors: Vec<Motor>,
}

impl MotorGroup {
    pub fn new(motors: Vec<Motor>) -> Self {
        Self { motors }
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        for motor in self.motors.iter_mut() {
            _ = motor.set_voltage(voltage);
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
}

pub fn average(values: Vec<f64>) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}
