use std::f64::consts::PI;

use uom::si::{
    angular_velocity::radian_per_second,
    f64::{AngularVelocity, Length, Time, Velocity},
    length::meter,
    time::second,
};
use vexide::prelude::Motor;

use crate::{
    hardware::motor_group::MotorGroup,
    localization::{odometry::Odometry, pose::Pose},
    mappings::DriveMode,
};

pub struct Drivetrain {
    pub left: MotorGroup,
    pub right: MotorGroup,
    odometry: Odometry,
    wheel_circum: Length,
    track: Length,
}

impl Drivetrain {
    pub fn new(
        left: MotorGroup,
        right: MotorGroup,
        odometry: Odometry,
        wheel_diameter: Length,
        track: Length,
    ) -> Self {
        Self {
            left,
            right,
            odometry,
            wheel_circum: wheel_diameter * PI,
            track,
        }
    }

    pub fn set_voltages(&mut self, left: f64, right: f64) {
        self.left.set_voltage(left);
        self.right.set_voltage(right);
    }

    pub fn set_velocity(&mut self, left: i32, right: i32) {
        self.left.set_velocity(left);
        self.right.set_velocity(right);
    }

    pub fn arcade(&mut self, power: f64, turn: f64) {
        let left = power + turn;
        let right = power - turn;
        self.set_voltages(left, right);
    }

    /// Computes the left and right motor power values based on the selected drive mode.
    /// Supports both Arcade and Tank drive configurations.
    pub fn drive(&mut self, drive_mode: &DriveMode) {
        let mut power_val = 0.0;
        let mut turn_val = 0.0;
        let mut left_val = 0.0;
        let mut right_val = 0.0;

        // Extract joystick values based on the configured drive mode
        match drive_mode {
            DriveMode::Arcade { power, turn } => {
                power_val = power.y(); // Forward/backward movement
                turn_val = turn.x(); // Turning movement
            }
            DriveMode::Tank { left, right } => {
                left_val = left.y(); // Left side control
                right_val = right.y(); // Right side control
            }
        }

        // Apply acceleration function if using Arcade drive
        if matches!(drive_mode, DriveMode::Arcade { .. }) {
            turn_val = apply_curve(turn_val, 2);
            left_val = power_val + turn_val;
            right_val = power_val - turn_val;
        }

        // Scale the final voltage values to the V5 motor's maximum voltage
        self.set_voltages(
            left_val * Motor::V5_MAX_VOLTAGE,
            right_val * Motor::V5_MAX_VOLTAGE,
        );
    }

    pub fn voltages(&self) -> [f64; 2] {
        [self.left.voltage(), self.right.voltage()]
    }

    pub fn velocity(&self) -> Velocity {
        let rpm = (self.left.velocity() + self.right.velocity()) / 2.0;
        (self.wheel_circum * rpm) / Time::new::<second>(60.0)
    }

    pub fn angular_velocity(&self) -> AngularVelocity {
        let vdiff = self.wheel_circum.get::<meter>()
            * (self.left.velocity() - self.right.velocity())
            / 60.0;

        AngularVelocity::new::<radian_per_second>(vdiff / self.track.get::<meter>())
    }

    pub fn pose(&self) -> Pose {
        self.odometry.get_pose()
    }

    pub fn set_pose(&mut self, pose: Pose) {
        self.odometry.set_pose(pose);
    }

    pub fn track(&mut self) -> Length {
        self.track
    }
}

/// Applies an acceleration function to the given power value.
/// Uses polynomial scaling based on the acceleration factor.
fn apply_curve(power: f64, acceleration: i32) -> f64 {
    if acceleration == 1 {
        return power; // If acceleration is 1, return power as is (linear mapping)
    }

    // Polynomial acceleration adjustment
    power.powi(acceleration - 1)
        * if acceleration % 2 == 0 {
            power.abs() // Even acceleration preserves absolute magnitude
        } else {
            power // Odd acceleration preserves sign
        }
}
