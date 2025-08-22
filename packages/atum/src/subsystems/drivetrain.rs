use core::f64::consts::TAU;

use vexide::prelude::{Float, InertialSensor, Motor};

use crate::{
    hardware::{motor_group::MotorGroup, odometer::Odometer},
    mappings::DriveMode,
    pose::{odometry::Odometry, Pose},
};

pub struct Drivetrain {
    // front_left: [Motor; 2],
    // front_right: [Motor; 2],
    // back_left: [Motor; 2],
    // back_right: [Motor; 2],
    left: MotorGroup,
    right: MotorGroup,
    odometry: Odometry,
    r: f64,
    track: f64,
}

impl Drivetrain {
    pub fn new(
        // front_left: [Motor; 2],
        // front_right: [Motor; 2],
        // back_left: [Motor; 2],
        // back_right: [Motor; 2],
        left: MotorGroup,
        right: MotorGroup,
        starting_pos: Pose,
        forward: Odometer<4096>,
        side: Odometer<4096>,
        imu: InertialSensor,
        r: f64,
        track: f64,
    ) -> Self {
        Self {
            // front_left,
            // front_right,
            // back_left,
            // back_right,
            left,
            right,
            odometry: Odometry::new(starting_pos, forward, side, imu),
            r,
            track,
        }
    }

    pub fn set_voltages(
        &mut self,
        // front_left: f64,
        // front_right: f64,
        // back_left: f64,
        // back_right: f64,
        left: f64,
        right: f64,
    ) {
        self.left.set_voltage(left);
        self.right.set_voltage(right);
        // for motor in self.front_left.iter_mut() {
        //     _ = motor.set_voltage(front_left * Motor::V5_MAX_VOLTAGE);
        // }
        // for motor in self.front_right.iter_mut() {
        //     _ = motor.set_voltage(front_right * Motor::V5_MAX_VOLTAGE);
        // }
        // for motor in self.back_left.iter_mut() {
        //     _ = motor.set_voltage(back_left * Motor::V5_MAX_VOLTAGE);
        // }
        // for motor in self.back_right.iter_mut() {
        //     _ = motor.set_voltage(back_right * Motor::V5_MAX_VOLTAGE);
        // }
    }

    pub fn get_voltages(&self) -> [f64; 2] {
        [self.left.voltage(), self.right.voltage()]
    }

    pub fn velocity(&self) -> f64 {
        let rpm = (self.left.velocity() + self.right.velocity()) / 2.0;
        TAU * self.r * rpm / 60.0 // figure out if this is right
    }

    pub fn angular_velocity(&self) -> f64 {
        let vdiff = TAU * self.r * (self.left.velocity() + self.right.velocity()) / 60.0; // figure
        // out if this is also right
        vdiff / self.track
    }

    pub fn get_pose(&self) -> Pose {
        self.odometry.get_pose()
    }
}

// pub fn holomonic(drivetrain: &mut Drivetrain, forward: f64, strafe: f64, turn: f64) {
//     let magnitude = (strafe.powi(2) + forward.powi(2)).sqrt();
//     let theta = forward.atan2(strafe) + drivetrain.odometry.pose.borrow().angle;
//     let x = magnitude * theta.cos();
//     let y = magnitude * theta.sin();
//
//     let front_left = x + y + turn;
//     let front_right = x - y - turn;
//     let back_left = x - y + turn;
//     let back_right = x + y - turn;
//
//     drivetrain.set_voltages(front_left, front_right, back_left, back_right);
// }

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
