#![no_main]
#![no_std]

extern crate alloc;

use alloc::vec;
use core::time::Duration;

use atum::{
    controllers::pid::Pid,
    hardware::{imu::Imu, motor_group::MotorGroup, otos::Otos, tracking_wheel::TrackingWheel},
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    motion::{move_to::MoveTo, turn::Turn},
    pose::{odometry::Odometry, Pose, Vec2},
    subsystems::{drivetrain::Drivetrain, intake::Intake},
};
use log::{info, LevelFilter};
use uom::{
    si::{
        angle::degree,
        angular_velocity::degree_per_second,
        f64::{Angle, AngularVelocity, Length, Velocity},
        length::inch,
        velocity::inch_per_second,
    },
    ConstZero,
};
use vexide::prelude::*;

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain,
    intake: Intake,
    otos: Otos,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let mut move_to = MoveTo::new(
            Pid::new(2.0, 0.0, 0.0, 0.0),
            Pid::new(0.0, 0.0, 0.0, 0.0),
            Length::new::<inch>(0.5),
            Velocity::new::<inch_per_second>(0.0),
            Length::new::<inch>(24.0),
        );

        move_to
            .move_to_point(
                &mut self.drivetrain,
                Vec2::new(Length::new::<inch>(10.0), Length::new::<inch>(10.0)),
                Duration::from_secs(2),
                Direction::Forward,
            )
            .await;
    }

    async fn driver(&mut self) {
        info!("Driver Control Started");

        let mut move_to = MoveTo::new(
            Pid::new(0.25, 0.0, 0.05, 0.0),
            Pid::new(10.0, 0.0, 0.0, 0.0),
            Length::new::<inch>(1.0),
            Velocity::new::<inch_per_second>(2.0),
            Length::new::<inch>(6.0),
        );

        let mut turn = Turn::new(
            Pid::new(24.0, 0.08, 1.1, 20.0),
            Pid::new(28.0, 0.02, 1.8, 10.0),
            Angle::new::<degree>(0.5),
            AngularVelocity::new::<degree_per_second>(5.0),
            Angle::new::<degree>(85.0),
        );

        loop {
            let state = self.controller.state().unwrap_or_default();
            let mappings = ControllerMappings {
                drive_mode: DriveMode::Arcade {
                    power: state.left_stick,
                    turn: state.right_stick,
                },
                intake: state.button_r1,
                outake: state.button_r2,
            };

            self.drivetrain.drive(&mappings.drive_mode);

            if mappings.intake.is_pressed() {
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            } else if mappings.outake.is_pressed() {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                self.intake.set_voltage(0.0);
            }

            info!("Position: {}", self.drivetrain.get_pose());
            // let vf = self.otos.vf().as_inches();
            // let vs = self.otos.vs().as_inches();
            // let omega = self.otos.omega().as_degrees();
            // info!("Velocity: ({}, {}, {})", vf, vs, omega);
            // let x = self.otos.x().as_inches();
            // let y = self.otos.y().as_inches();
            // let h = self.otos.h().as_degrees();
            // info!("Position: ({}, {}, {})", x, y, h);

            if state.button_down.is_now_pressed() {
                self.drivetrain.set_pose(Pose::new(
                    Length::ZERO,
                    Length::ZERO,
                    self.drivetrain.get_pose().h,
                ))
            }

            // tuning PID constants for angular movement
            if state.button_left.is_pressed() {
                turn.turn_to(
                    &mut self.drivetrain,
                    Angle::ZERO,
                    Duration::from_millis(1000),
                )
                .await;
            }

            // testing and tuning seeking movement
            if state.button_up.is_pressed() {
                move_to
                    .move_to_point(
                        &mut self.drivetrain,
                        Vec2::new(Length::ZERO, Length::new::<inch>(24.0)),
                        Duration::from_secs(8),
                        Direction::Forward,
                    )
                    .await;
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    Logger.init(LevelFilter::Trace).unwrap();

    let mut imu = Imu::new(vec![
        InertialSensor::new(peripherals.port_10),
        InertialSensor::new(peripherals.port_19),
    ]);

    imu.calibrate().await;

    let starting_position = Pose::new(Length::ZERO, Length::ZERO, Angle::ZERO);

    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            MotorGroup::new(vec![
                Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
            ]),
            MotorGroup::new(vec![
                Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
            ]),
            Odometry::new(
                starting_position,
                TrackingWheel::new(
                    peripherals.adi_c,
                    peripherals.adi_d,
                    Direction::Reverse,
                    Length::new::<inch>(2.5),
                    Length::ZERO,
                ),
                TrackingWheel::new(
                    peripherals.adi_a,
                    peripherals.adi_b,
                    Direction::Reverse,
                    Length::new::<inch>(2.5),
                    Length::new::<inch>(-1.7175),
                ),
                imu,
            ),
            Length::new::<inch>(2.5),
            Length::new::<inch>(12.0),
        ),
        intake: Intake::new(MotorGroup::new(vec![
            Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
        ])),
        otos: Otos::new(
            peripherals.port_20,
            starting_position,
            Pose::new(Length::ZERO, Length::ZERO, Angle::new::<degree>(-90.0)),
        )
        .await,
    };

    robot.compete().await;
}
