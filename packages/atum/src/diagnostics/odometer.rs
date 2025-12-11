use std::{cell::RefCell, rc::Rc};

use uom::si::length::foot;

use crate::pose::Pose;

pub struct Odometer {
    prev_pose: Pose,
    pose: Rc<RefCell<Pose>>,
    traveled: f64,
}

impl Odometer {
    pub fn new(pose: Rc<RefCell<Pose>>) -> Self {
        let initial_pose = *pose.borrow();
        Self {
            prev_pose: initial_pose,
            pose,
            traveled: 0.0,
        }
    }

    pub fn update(&mut self) {
        let current_pose = *self.pose.borrow();
        let delta_x = (current_pose.x - self.prev_pose.x).get::<foot>();
        let delta_y = (current_pose.y - self.prev_pose.y).get::<foot>();
        let distance = (delta_x.powi(2) + delta_y.powi(2)).sqrt();

        self.traveled += if distance.is_nan() {
            0.0
        } else {
            distance
        };
        self.prev_pose = current_pose;
    }

    pub fn traveled(&self) -> f64 {
        self.traveled
    }
}
