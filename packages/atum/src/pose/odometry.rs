use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use log::warn;
use uom::{
    si::{
        angle::radian,
        f64::{Angle, Length, Time},
        time::second,
    },
    ConstZero,
};
use vexide::{prelude::Task, task::spawn, time::sleep};

use super::Pose;
use crate::hardware::{imu::Imu, tracking_wheel::TrackingWheel};

pub struct Odometry {
    pose: Rc<RefCell<Pose>>,
    _task: Task<()>,
}

impl Odometry {
    pub fn new(
        starting_pose: Pose,
        mut forward: TrackingWheel,
        mut side: TrackingWheel,
        mut imu: Imu,
    ) -> Self {
        let pose = Rc::new(RefCell::new(starting_pose));
        imu.set_heading(starting_pose.h);

        Self {
            pose: pose.clone(),
            _task: spawn(async move {
                let mut prev_time = Instant::now();
                let mut prev_heading = imu.heading();
                loop {
                    let mut dx = side.traveled();
                    let mut dy = forward.traveled();
                    let heading = imu.heading();
                    let mut dh = heading - prev_heading;
                    prev_heading = heading;

                    if dh != Angle::ZERO {
                        // Prevent divide by zero error
                        dx = 2.0
                            * (dh.get::<radian>() / 2.0).sin()
                            * (dx / dh.get::<radian>() + side.from_center());
                        dy = 2.0
                            * (dh.get::<radian>() / 2.0).sin()
                            * (dy / dh.get::<radian>() + forward.from_center());
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
                                + (heading_avg.get::<radian>().cos() * dx
                                    + heading_avg.get::<radian>().sin() * dy),
                            y: prev.y
                                + (-heading_avg.get::<radian>().sin() * dx
                                    + heading_avg.get::<radian>().cos() * dy),
                            h: prev.h + dh,
                            vf: dy / Time::new::<second>(dt.as_secs_f64()),
                            vs: dx / Time::new::<second>(dt.as_secs_f64()),
                            omega: (dh / Time::new::<second>(dt.as_secs_f64())).into(),
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
