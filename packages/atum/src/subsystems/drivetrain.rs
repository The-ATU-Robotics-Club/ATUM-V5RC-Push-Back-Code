use core::f64::consts::PI;

use vexide::prelude::{Float, Motor};

use crate::{
    hardware::motor_group::MotorGroup,
    mappings::DriveMode,
    pose::{odometry::Odometry, Pose},
    units::length::Length,
};

pub struct Drivetrain {
    left: MotorGroup,
    right: MotorGroup,
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

    pub fn arcade(&mut self, power: f64, turn: f64) {
        let left = power + turn;
        let right = power - turn;
        self.set_voltages(left, right);
    }

    pub fn get_voltages(&self) -> [f64; 2] {
        [self.left.voltage(), self.right.voltage()]
    }

    pub fn velocity(&self) -> f64 {
        let rpm = (self.left.velocity() + self.right.velocity()) / 2.0;
        self.wheel_circum.as_inches() * rpm / 60.0 // figure out if this is right
    }

    // change this to return `AngularVelocity`
    pub fn angular_velocity(&self) -> f64 {
        let vdiff =
            self.wheel_circum.as_inches() * (self.left.velocity() + self.right.velocity()) / 60.0;

        vdiff / self.track.as_inches()
    }

    pub fn get_pose(&self) -> Pose {
        self.odometry.get_pose()
    }

    pub fn set_pose(&mut self, pose: Pose) {
        self.odometry.set_pose(pose);
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

/// Computes the left and right motor power values based on the selected drive mode.
/// Supports both Arcade and Tank drive configurations.
pub fn differential(drive_mode: &DriveMode) -> (f64, f64) {
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
    (
        left_val * Motor::V5_MAX_VOLTAGE,
        right_val * Motor::V5_MAX_VOLTAGE,
    )
}
