use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use uom::si::{angle::radian, f64::Time, time::second};
use vexide::{
    task::{spawn, Task},
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
                    let ds1 = side.traveled();
                    let ds2 = forward.traveled();

                    let heading = imu.rotation();
                    let dh = heading - prev_heading;
                    prev_heading = heading;

                    let phi1 = side.angle();
                    let phi2 = forward.angle();

                    let offset1 = side.from_center();
                    let offset2 = forward.from_center();

                    let dx_rot1 = -dh.get::<radian>() * offset1.y;
                    let dy_rot1 = dh.get::<radian>() * offset1.x;
                    let dx_rot2 = -dh.get::<radian>() * offset2.y;
                    let dy_rot2 = dh.get::<radian>() * offset2.x;

                    // projection of that rotation along each wheel axis (Length)
                    let ds1_rot =
                        dx_rot1 * phi1.get::<radian>().cos() + dy_rot1 * phi1.get::<radian>().sin();
                    let ds2_rot =
                        dx_rot2 * phi2.get::<radian>().cos() + dy_rot2 * phi2.get::<radian>().sin();

                    // corrected wheel distances (translation only)
                    let ds1_corr = ds1 - ds1_rot;
                    let ds2_corr = ds2 - ds2_rot;

                    let det = (phi2.get::<radian>() - phi1.get::<radian>()).sin();
                    let inv_det = 1.0 / det;

                    let c1 = phi1.get::<radian>().cos();
                    let s1 = phi1.get::<radian>().sin();
                    let c2 = phi2.get::<radian>().cos();
                    let s2 = phi2.get::<radian>().sin();

                    let dx = (s2 * ds1_corr - s1 * ds2_corr) * inv_det;
                    let dy = (-c2 * ds1_corr + c1 * ds2_corr) * inv_det;

                    pose.replace_with(|prev| {
                        let heading_avg = prev.h + dh / 2.0;
                        let dt = prev_time.elapsed();
                        Pose {
                            // Doing vector rotation for odom and adding to position
                            x: prev.x + (heading_avg.cos() * dx + heading_avg.sin() * dy),
                            y: prev.y + (-heading_avg.sin() * dx + heading_avg.cos() * dy),
                            h: imu.heading(),
                            vf: dx / Time::new::<second>(dt.as_secs_f64()),
                            vs: dy / Time::new::<second>(dt.as_secs_f64()),
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
