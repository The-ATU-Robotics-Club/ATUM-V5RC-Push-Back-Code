use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use vexide::{
    task::{Task, spawn},
    time::sleep,
};

use super::pose::Pose;
use crate::{
    hardware::{imu::Imu, tracking_wheel::TrackingWheel},
    localization::vec2::Vec2,
};

pub struct Odometry {
    pose: Rc<RefCell<Pose>>,
    _task: Task<()>,
}

impl Odometry {
    pub fn new(
        pose: Rc<RefCell<Pose>>,
        mut wheel_1: TrackingWheel,
        mut wheel_2: TrackingWheel,
        imu: Imu,
    ) -> Self {
        Self {
            pose: pose.clone(),
            _task: spawn(async move {
                let mut prev_time = Instant::now();
                let mut prev_heading = imu.rotation();

                let offset1 = wheel_1.from_center();
                let offset2 = wheel_2.from_center();

                let axis1_angle = wheel_1.angle();
                let axis2_angle = wheel_2.angle();

                let det = (axis2_angle - axis1_angle).sin();
                assert!(det.abs() > 1e-6, "Wheels cannot be parallel");

                let axis1 = Vec2::new(axis1_angle.cos(), axis1_angle.sin());
                let axis2 = Vec2::new(axis2_angle.cos(), axis2_angle.sin());

                loop {
                    let ds1 = wheel_1.traveled();
                    let ds2 = wheel_2.traveled();

                    let heading = imu.rotation();

                    let dh_angle = heading - prev_heading;
                    let dh = dh_angle.as_radians();
                    prev_heading = heading;

                    let rot1 = Vec2::new(-dh * offset1.y, dh * offset1.x);
                    let rot2 = Vec2::new(-dh * offset2.y, dh * offset2.x);

                    let ds1_corr = ds1 - rot1.dot(axis1);
                    let ds2_corr = ds2 - rot2.dot(axis2);

                    let delta = Vec2::new(
                        (axis2.y * ds1_corr - axis1.y * ds2_corr) / det,
                        (-axis2.x * ds1_corr - axis1.x * ds2_corr) / det,
                    );

                    // Time since last update
                    let dt = prev_time.elapsed().as_secs_f64();
                    prev_time = Instant::now();

                    pose.replace_with(|prev| {
                        let heading_avg = prev.h + dh_angle / 2.0;

                        let global_delta = delta.rotated(heading_avg.as_radians());

                        Pose {
                            // Doing vector rotation for odom and adding to position
                            // Update global position
                            x: prev.x + global_delta.x,
                            y: prev.y + global_delta.y,
                            h: prev.h + dh_angle,

                            // Robot-relative velocities
                            vf: delta.x / dt, // forward velocity
                            vs: delta.y / dt, // sideways velocity
                            omega: dh / dt,   // angular velocity
                        }
                    });

                    sleep(Duration::from_millis(10)).await;
                }
            }),
        }
    }

    pub fn pose(&self) -> Pose {
        *self.pose.borrow()
    }

    pub fn set_pose(&mut self, pose: Pose) {
        *self.pose.borrow_mut() = pose;
    }
}
