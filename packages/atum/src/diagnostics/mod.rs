pub mod battery;
pub mod odometer;

use std::fs::File;
use std::io::Write;
use std::{cell::RefCell, rc::Rc};
use crate::{
    diagnostics::{
        battery::Battery,
        odometer::Odometer
    },
    pose::Pose,
};

pub struct Diagnostics {
    odometer: Odometer,
    battery: Battery,
}

impl Diagnostics {
    pub fn new(pose: Rc<RefCell<Pose>>) -> Self {
        Self {
            odometer: Odometer::new(pose),
            battery: Battery::default(),
        }
    }

    pub fn update(&mut self) {
        self.odometer.update();
        self.battery.update();
    }

    pub fn log(&self) {
        let mut file = File::create("log.txt").expect("Unable to create log file");
        writeln!(
            file,
            "Distance Traveled: {:.2} inches",
            self.odometer.traveled()
        )
        .expect("Unable to write to log file");
    }
}
