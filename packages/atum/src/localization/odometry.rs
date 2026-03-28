//! Odometry System
//!
//! This module continuously estimates the robot's pose on the field using
//! two tracking wheels and an IMU.
//!
//! The estimated pose consists of:
//!
//!     (x, y, h)
//!
//! where
//!     x = global x position
//!     y = global y position
//!     h = robot heading
//!
//! The odometry runs in a background async task that periodically reads the
//! tracking wheels and IMU, computes the robot's motion since the last update,
//! and integrates it into the global pose estimate.
//!
//! Two tracking wheels measure motion along known axes relative to the robot.
//! Using those measurements along with the robot's change in heading, we
//! reconstruct the robot's translation and rotation over time.

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
    /// Shared robot pose that other subsystems can read.
    pose: Rc<RefCell<Pose>>,

    /// Handle to the background odometry task.
    _task: Task<()>,
}

impl Odometry {
    /// Creates a new odometry system and starts the background update loop.
    ///
    /// The loop runs roughly every 10ms (~100 Hz) and continuously updates
    /// the shared robot pose using the tracking wheels and IMU.
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

                // Position of each wheel relative to robot center
                let offset1 = wheel_1.from_center();
                let offset2 = wheel_2.from_center();

                // Wheel orientations relative to robot frame
                let axis1_angle = wheel_1.angle();
                let axis2_angle = wheel_2.angle();

                // Determinant used when solving the 2x2 system that reconstructs
                // robot translation from the two wheel measurements.
                let det = (axis2_angle - axis1_angle).sin();
                assert!(det.abs() > 1e-6, "Wheels cannot be parallel");

                // Unit vectors describing each wheel's measurement axis
                let axis1 = Vec2::new(axis1_angle.cos(), axis1_angle.sin());
                let axis2 = Vec2::new(axis2_angle.cos(), axis2_angle.sin());

                loop {
                    // Distance each wheel has moved since the last update
                    let ds1 = wheel_1.traveled();
                    let ds2 = wheel_2.traveled();

                    let heading = imu.rotation();

                    // Change in robot heading since last update
                    let dh_angle = heading - prev_heading;
                    let dh = dh_angle.as_radians();
                    prev_heading = heading;

                    // Calculate distance traveled by wheel offsets during rotation
                    let rot1 = Vec2::new(-dh * offset1.y, dh * offset1.x);
                    let rot2 = Vec2::new(-dh * offset2.y, dh * offset2.x);

                    // Remove the rotational component from the wheel measurements
                    // leaving only the translation component of the robot motion
                    let ds1_corr = ds1 - rot1.dot(axis1);
                    let ds2_corr = ds2 - rot2.dot(axis2);

                    // Calculate the change in position using the inverse of
                    // the 2x2 matrix formed by the wheel axes.
                    let delta = Vec2::new(
                        (axis2.y * ds1_corr - axis1.y * ds2_corr) / det,
                        (-axis2.x * ds1_corr + axis1.x * ds2_corr) / det,
                    );

                    // Time since last update
                    let dt = prev_time.elapsed().as_secs_f64();
                    prev_time = Instant::now();

                    pose.replace_with(|prev| {
                        // Use midpoint heading for better integration accuracy
                        // when converting robot-frame motion into global coordinates
                        let heading_avg = prev.h + dh_angle / 2.0;

                        // Rotate local robot translation into global field coordinates
                        let global_delta = delta.rotated(heading_avg.as_radians());

                        Pose {
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

    /// Returns the current estimated robot pose.
    ///
    /// This performs a cheap copy of the Pose stored inside the RefCell.
    pub fn pose(&self) -> Pose {
        *self.pose.borrow()
    }

    /// Manually sets the robot pose.
    ///
    /// Useful for resetting odometry at the start of autonomous or when
    /// synchronizing with a known field position.
    pub fn set_pose(&mut self, pose: Pose) {
        *self.pose.borrow_mut() = pose;
    }
}
