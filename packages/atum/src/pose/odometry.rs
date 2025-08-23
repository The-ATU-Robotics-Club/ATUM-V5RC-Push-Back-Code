use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use log::{debug, warn};
use vexide::{
    prelude::{Float, InertialSensor, Task},
    task::spawn,
    time::{sleep, Instant},
};

use super::Pose;
use crate::{hardware::tracking_wheel::TrackingWheel, math::{angle::{Angle, IntoAngle}, length::Length}};

pub struct Odometry {
    pose: Rc<RefCell<Pose>>,
    _task: Task<()>,
}

impl Odometry {
    pub fn new(
        mut starting_pose: Pose,
        mut forward: TrackingWheel,
        mut side: TrackingWheel,
        imu: InertialSensor,
    ) -> Self {
        starting_pose.h = starting_pose.h;
        let pose = Rc::new(RefCell::new(starting_pose));

        Self {
            pose: pose.clone(),
            _task: spawn(async move {
                let mut prev_time = Instant::now();
                let mut prev_heading = imu.heading().unwrap_or_default().deg();
                loop {
                    let mut dx = side.traveled();
                    let mut dy = forward.traveled();
                    let heading = imu.heading().unwrap_or_default().deg();
                    let mut dh = heading - prev_heading;
                    prev_heading = heading;

                    if dh != Angle::ZERO {
                        // Prevent divide by zero error
                        dx = (dx / dh.as_radians() + side.from_center()) * (dh / 2.0).sin().as_radians() * 2.0;
                        dy = (dy / dh.as_radians() + forward.from_center()) * (dh / 2.0).sin().as_radians() * 2.0;
                    }

                    if dx.is_infinite() || dy.is_infinite() || dh.is_infinite() {
                        warn!("Invalid values read from odometers");
                        dx = Length::ZERO;
                        dy = Length::ZERO;
                        dh = Angle::ZERO;
                    }

                    // debug!("DELTA pos (x, y, h): ({}, {}, {})", dx, dy, dh);

                    pose.replace_with(|prev| {
                        let heading_avg = prev.h + dh / 2.0;
                        let dt = prev_time.elapsed();
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
