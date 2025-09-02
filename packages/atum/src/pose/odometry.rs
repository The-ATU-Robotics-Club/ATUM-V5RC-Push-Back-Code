use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use log::warn;
use vexide::{
    prelude::{Float, InertialSensor, Task},
    task::spawn,
    time::{sleep, Instant},
};

use super::Pose;
use crate::{
    hardware::tracking_wheel::TrackingWheel,
    units::{
        angle::{Angle, IntoAngle},
        length::Length,
    },
};

pub struct Odometry {
    pose: Rc<RefCell<Pose>>,
    _task: Task<()>,
}

impl Odometry {
    pub fn new(
        starting_pose: Pose,
        mut forward: TrackingWheel,
        mut side: TrackingWheel,
        mut imu: InertialSensor,
    ) -> Self {
        let pose = Rc::new(RefCell::new(starting_pose));
        _ = imu.set_heading(starting_pose.h.as_degrees());

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
                        dx = 2.0
                            * (dh / 2.0).sin().as_radians()
                            * (dx / dh.as_radians() + side.from_center());
                        dy = 2.0
                            * (dh / 2.0).sin().as_radians()
                            * (dy / dh.as_radians() + forward.from_center());
                    }

                    if dx.is_infinite() || dy.is_infinite() || dh.is_infinite() {
                        warn!("Invalid values read from odometers");
                        dx = Length::ZERO;
                        dy = Length::ZERO;
                        dh = Angle::ZERO;
                    }

                    pose.replace_with(|prev| {
                        let heading_avg = prev.h + dh / 2.0;
                        let dt = prev_time.elapsed();
                        Pose {
                            // Doing vector rotation for odom and adding to position
                            x: prev.x
                                + (heading_avg.cos().as_radians() * dx
                                    + heading_avg.sin().as_radians() * dy),
                            y: prev.y
                                + (-heading_avg.sin().as_radians() * dx
                                    + heading_avg.cos().as_radians() * dy),
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
