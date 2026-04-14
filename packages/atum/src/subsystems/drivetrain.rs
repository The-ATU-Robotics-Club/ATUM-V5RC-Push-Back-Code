//! Drivetrain control
//!
//! Provides high-level interfaces for controlling the robot's drivetrain, including
//! motor groups, voltage/velocity commands, and drive modes (Arcade/Tank).

use std::f64::consts::PI;

use vexide::{
    prelude::{Gearset, Motor},
    smart::motor::BrakeMode,
};

use crate::{
    hardware::motor_group::MotorGroup,
    localization::{odometry::Odometry, pose::Pose},
    mappings::DriveMode,
    utils::{desaturate, apply_curve},
};

/// Represents a differential drivetrain with left and right motor groups
/// and integrated odometry for pose estimation.
pub struct Drivetrain {
    pub left: MotorGroup,
    pub right: MotorGroup,
    odometry: Odometry,
    wheel_circum: f64,
    track: f64,
}

impl Drivetrain {
    /// Create a new drivetrain instance
    pub fn new(
        left: MotorGroup,
        right: MotorGroup,
        odometry: Odometry,
        wheel_diameter: f64,
        track: f64,
    ) -> Self {
        Self {
            left,
            right,
            odometry,
            wheel_circum: wheel_diameter * PI,
            track,
        }
    }

    /// Set motor voltages directly
    pub fn set_voltages(&mut self, left: f64, right: f64) {
        let [left, right] = desaturate([left, right], 1.0);
        self.left.set_voltage(left * Motor::V5_MAX_VOLTAGE);
        self.right.set_voltage(right * Motor::V5_MAX_VOLTAGE);
    }

    /// Set motor target velocities
    pub fn set_velocity(&mut self, left: f64, right: f64) {
        let [left, right] = desaturate([left, right], 1.0);
        self.left.set_velocity(left * Gearset::MAX_BLUE_RPM);
        self.right.set_velocity(right * Gearset::MAX_BLUE_RPM);
    }

    /// Set brake mode for both motor groups
    pub fn brake(&mut self, brake: BrakeMode) {
        self.left.brake(brake);
        self.right.brake(brake);
    }

    /// Simple arcade drive control using power and turn
    pub fn set_arcade(&mut self, power: f64, turn: f64) {
        let left = power + turn;
        let right = power - turn;
        self.set_voltages(left, right);
    }

    /// Drive method supporting both Arcade and Tank modes
    pub fn drive(&mut self, drive_mode: &DriveMode) {
        let mut power_val = 0.0;
        let mut turn_val = 0.0;
        let mut left_val = 0.0;
        let mut right_val = 0.0;

        // Extract control inputs from DriveMode
        match drive_mode {
            DriveMode::Arcade { power, turn } => {
                power_val = power.y(); // forward/backward
                turn_val = turn.x(); // rotation
            }
            DriveMode::Tank { left, right } => {
                left_val = left.y(); // left side
                right_val = right.y(); // right side
            }
        }

        // Apply acceleration curve for Arcade drive
        if matches!(drive_mode, DriveMode::Arcade { .. }) {
            power_val = apply_curve(power_val, 1, 1.0);
            turn_val = apply_curve(turn_val, 2, 1.0) * 0.75;

            left_val = power_val + turn_val;
            right_val = power_val - turn_val;
        }

        // Scale voltages to V5 max
        self.set_voltages(left_val, right_val);
    }

    /// Returns the average voltage of the drivetrain motors
    pub fn voltages(&self) -> [f64; 2] {
        [self.left.voltage(), self.right.voltage()]
    }

    /// Computes linear velocity based on wheel RPMs
    pub fn velocity(&self) -> f64 {
        let rpm = (self.left.velocity() + self.right.velocity()) / 2.0;
        (self.wheel_circum * rpm) / 60.0
    }

    /// Computes angular velocity of the robot
    pub fn angular_velocity(&self) -> f64 {
        let vdiff = self.wheel_circum * (self.left.velocity() - self.right.velocity()) / 60.0;

        vdiff / self.track
    }

    /// Returns the current robot pose
    pub fn pose(&self) -> Pose {
        self.odometry.pose()
    }

    /// Set the robot's pose
    pub fn set_pose(&mut self, pose: Pose) {
        self.odometry.set_pose(pose);
    }

    /// Returns the distance between left and right wheels
    pub fn track(&self) -> f64 {
        self.track
    }
}
