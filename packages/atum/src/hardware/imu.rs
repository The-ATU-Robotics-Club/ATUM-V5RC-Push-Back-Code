use std::f64::consts::TAU;

use log::{error, info};
use uom::si::{angle::radian, f64::Angle};
use vexide::{math::Angle as VAngle, prelude::InertialSensor};

use super::average;

pub struct Imu {
    imus: Vec<InertialSensor>,
}

impl Imu {
    pub fn new(imus: Vec<InertialSensor>) -> Self {
        Self { imus }
    }

    pub async fn calibrate(&mut self) {
        for imu in self.imus.iter_mut() {
            match imu.calibrate().await {
                Ok(_) => info!("Calibration Successful"),
                Err(e) => error!("Error {:?}", e),
            }
        }
    }

    pub fn set_heading(&mut self, heading: Angle) {
        for imu in self.imus.iter_mut() {
            _ = imu.set_rotation(VAngle::from_radians(heading.get::<radian>()));
        }
    }

    pub fn rotation(&self) -> Angle {
        let mut angles = Vec::new();
        for imu in self.imus.iter() {
            if let Ok(rotation) = imu.rotation() {
                angles.push(TAU - rotation.as_radians());
            }
        }

        Angle::new::<radian>(average(angles))
    }

    pub fn heading(&self) -> Angle {
        let mut angles = Vec::new();
        for imu in self.imus.iter() {
            if let Ok(rotation) = imu.rotation() {
                angles.push(TAU - rotation.as_radians());
            }
        }

        Angle::new::<radian>(average(angles).rem_euclid(TAU))
    }
}
