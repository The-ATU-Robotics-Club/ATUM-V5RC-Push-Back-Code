use alloc::vec::Vec;

use log::{error, info};
use uom::si::{angle::degree, f64::Angle};
use vexide::prelude::InertialSensor;

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
            _ = imu.set_rotation(heading.get::<degree>());
        }
    }

    pub fn heading(&self) -> Angle {
        let mut angles = Vec::new();
        for imu in self.imus.iter() {
            if let Ok(rotation) = imu.rotation() {
                angles.push(360.0 - rotation);
            }
        }

        Angle::new::<degree>(average(angles) % 360.0)
    }
}

