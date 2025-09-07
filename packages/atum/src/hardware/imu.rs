use alloc::vec::Vec;

use log::{error, info};
use vexide::prelude::InertialSensor;

use super::average;
use crate::units::angle::{Angle, IntoAngle};

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
            _ = imu.set_rotation(heading.as_degrees());
        }
    }

    pub fn heading(&self) -> Angle {
        let mut angles = Vec::new();
        for imu in self.imus.iter() {
            if let Ok(rotation) = imu.rotation() {
                angles.push(rotation);
            }
        }

        (average(angles) % 360.0).deg()
    }
}

