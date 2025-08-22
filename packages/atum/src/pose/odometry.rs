use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use log::{debug, warn};
use vexide::{
    prelude::{Float, InertialSensor, Task},
    task::spawn,
    time::{sleep, Instant},
};

use super::Pose;
use crate::hardware::odometer::Odometer;

pub struct Odometry {
    pose: Rc<RefCell<Pose>>,
    _task: Task<()>,
}

impl Odometry {
    pub fn new(
        mut starting_pose: Pose,
        mut forward: Odometer<4096>,
        mut side: Odometer<4096>,
        imu: InertialSensor,
    ) -> Self {
        starting_pose.h = starting_pose.h.to_radians();
        let pose = Rc::new(RefCell::new(starting_pose));

        Self {
            pose: pose.clone(),
            _task: spawn(async move {
                let mut prev_time = Instant::now();
                let mut prev_heading = imu.heading().unwrap_or_default().to_radians();
                loop {
                    let mut dx = side.traveled();
                    let mut dy = forward.traveled();
                    let heading = imu.heading().unwrap_or_default().to_radians();
                    let mut dh = heading - prev_heading;
                    prev_heading = heading;

                    if dh != 0.0 { // Prevent divide by zero error
                        dx = 2.0 * (dh / 2.0).sin() * (dx / dh + side.from_center());
                        dy = 2.0 * (dh / 2.0).sin() * (dy / dh + forward.from_center());
                    }

                    if dx.is_infinite() || dy.is_infinite() || dh.is_infinite() {
                        warn!("Invalid values read from odometers");
                        dx = 0.0;
                        dy = 0.0;
                        dh = 0.0;
                    }

                    // debug!("DELTA pos (x, y, h): ({}, {}, {})", dx, dy, dh);

                    let dt = prev_time.elapsed();
                    pose.replace_with(|prev| {
                        let heading_avg = prev.h + dh / 2.0;
                        Pose {
                            // Doing vector rotation for odom and adding to position
                            x: prev.x + (heading_avg.cos() * dx + heading_avg.sin() * dy),
                            y: prev.y + (-heading_avg.sin() * dx + heading_avg.cos() * dy),
                            h: prev.h + dh,
                            vf: dy / dt.as_secs_f64(),
                            vs: dx / dt.as_secs_f64(),
                            omega: dh / dt.as_secs_f64(),
                        }
                    });

                    prev_time = Instant::now();
                    sleep(Duration::from_millis(10)).await;
                }
            }),
        }
    }

    pub fn get_pose(&self) -> Pose {
        *self.pose.borrow() // Gets the position as a vector
    }

    pub fn set_pose(&mut self, pose: Pose) {
        *self.pose.borrow_mut() = pose; // Sets the position vector
    }
}
