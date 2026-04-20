use std::f64::consts::TAU;

use log::{debug, error, info};
use vexide::{math::Angle, prelude::InertialSensor};

use super::average;

/// Wrapper around one or more inertial sensors.
///
/// This abstraction allows multiple IMUs to be used together to improve
/// reliability by averaging their reported rotations.
pub struct Imu {
    /// Collection of inertial sensors used for heading estimation.
    imus: Vec<InertialSensor>,
    ratio: f64
}

impl Imu {
    /// Creates a new IMU system from a list of inertial sensors.
    pub fn new(imus: Vec<InertialSensor>, ratio: f64) -> Self {
        Self { imus, ratio}
    }

    /// Calibrates all inertial sensors.
    ///
    /// Each sensor is calibrated sequentially. Errors are logged if a
    /// calibration fails.
    pub async fn calibrate(&mut self) {
        for imu in self.imus.iter_mut() {
            match imu.calibrate().await {
                Ok(_) => info!("Calibration Successful"),
                Err(e) => error!("Error {:?}", e),
            }
        }
    }

    /// Sets the heading reference for all inertial sensors.
    ///
    /// This resets both the sensor rotation and heading to the provided value.
    pub fn set_rotation(&mut self, heading: Angle) {
        for imu in self.imus.iter_mut() {
            _ = imu.set_rotation(heading);
            _ = imu.set_heading(heading.wrapped_full())
        }
    }

    /// Returns the averaged raw rotation from all sensors.
    ///
    /// Rotations are converted to radians and averaged to reduce noise
    /// when multiple sensors are present.
    pub fn rotation(&self) -> Angle {
        let mut angles = Vec::new();
        for imu in self.imus.iter() {
            if let Ok(rotation) = imu.rotation() {
                angles.push(TAU - rotation.as_radians());
            }
        }

        Angle::from_radians(average(&angles)*self.ratio)
    }

    /// Returns the robot heading normalized to `[0, 2π)`.
    pub fn heading(&self) -> Angle {
        self.rotation().wrapped_full()
    }

    pub fn pitch(&self) -> Vec<Angle> {
        let mut pitch = vec![];
        for imu in self.imus.iter() {
            if let Ok(euler) = imu.euler() {
                pitch.push(euler.a);
            }
        }

        pitch
    }
}
