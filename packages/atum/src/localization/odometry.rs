use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use uom::si::{f64::Time, time::second};
use vexide::{
    task::{Task, spawn},
    time::sleep,
};

use super::pose::Pose;
use crate::hardware::{imu::Imu, tracking_wheel::TrackingWheel};

pub struct Odometry {
    pose: Rc<RefCell<Pose>>,
    _task: Task<()>,
}

impl Odometry {
    pub fn new(
        starting_pose: Pose,
        mut wheel_1: TrackingWheel,
        mut wheel_2: TrackingWheel,
        imu: Imu,
    ) -> Self {
        let pose = Rc::new(RefCell::new(starting_pose));

        Self {
            pose: pose.clone(),
            _task: spawn(async move {
                let mut prev_time = Instant::now();
                let mut prev_heading = imu.heading();
                loop {
                    let ds1 = wheel_1.traveled();
                    let ds2 = wheel_2.traveled();

                    let heading = imu.rotation();
                    let dh = heading - prev_heading;
                    prev_heading = heading;

                    let phi1 = wheel_1.angle();
                    let phi2 = wheel_2.angle();

                    let offset1 = wheel_1.from_center();
                    let offset2 = wheel_2.from_center();

                    let dx_rot1 = -dh * offset1.y;
                    let dy_rot1 = dh * offset1.x;
                    let dx_rot2 = -dh * offset2.y;
                    let dy_rot2 = dh * offset2.x;

                    // projection of that rotation along each wheel axis (Length)
                    let ds1_rot = dx_rot1 * phi1.cos() + dy_rot1 * phi1.sin();
                    let ds2_rot = dx_rot2 * phi2.cos() + dy_rot2 * phi2.sin();

                    // corrected wheel distances (translation only)
                    let ds1_corr = ds1 - ds1_rot;
                    let ds2_corr = ds2 - ds2_rot;

                    let det = (phi2 - phi1).sin();

                    let dx = (phi2.sin() * ds1_corr - phi1.sin() * ds2_corr) / det;
                    let dy = (-phi2.cos() * ds1_corr + phi1.cos() * ds2_corr) / det;

                    pose.replace_with(|prev| {
                        let heading_avg = prev.h + dh / 2.0;
                        let dt = prev_time.elapsed().as_secs_f64();
                        Pose {
                            // Doing vector rotation for odom and adding to position
                            x: prev.x + (heading_avg.cos() * dx - heading_avg.sin() * dy),
                            y: prev.y + (heading_avg.sin() * dx + heading_avg.cos() * dy),
                            h: prev.h + dh,
                            vf: dx / Time::new::<second>(dt),
                            vs: dy / Time::new::<second>(dt),
                            omega: (dh / Time::new::<second>(dt)).into(),
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
