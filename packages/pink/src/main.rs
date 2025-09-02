#![no_main]
#![no_std]

extern crate alloc;

use alloc::vec;
use core::time::Duration;

use atum::{
    controllers::pid::Pid,
    hardware::{motor_group::MotorGroup, otos::Otos, tracking_wheel::TrackingWheel},
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    motion::{move_to::MoveTo, turn::Turn},
    pose::{odometry::Odometry, Pose, Vec2},
    subsystems::{
        drivetrain::{differential, Drivetrain},
        intake::Intake,
    },
    units::{angle::IntoAngle, length::IntoLength},
};
use log::{error, info, LevelFilter};
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
            0.5.inch(),
            0.0.inch(),
            1.0.tile(),
        );

        move_to
            .move_to_point(
                &mut self.drivetrain,
                Vec2::new(10.0.inch(), 10.0.inch()),
                Duration::from_secs(2),
                Direction::Forward,
            )
            .await;
    }

    async fn driver(&mut self) {
        info!("Driver Control Started");

        let mut move_to = MoveTo::new(
            Pid::new(0.5, 0.0, 0.05, 0.0),
            Pid::new(6.0, 0.0, 0.0, 0.0),
            1.0.inch(),
            10.0.inch(),
            0.5.tile(),
        );

        let mut turn = Turn::new(
            Pid::new(24.0, 0.08, 1.1, 20.0),
            Pid::new(28.0, 0.02, 1.8, 10.0),
            0.5.deg(),
            5.0.deg(),
            85.0.deg(),
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

            let power = differential(&mappings.drive_mode);
            self.drivetrain.set_voltages(power.0, power.1);

            if mappings.intake.is_pressed() {
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            } else if mappings.outake.is_pressed() {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                self.intake.set_voltage(0.0);
            }

            // info!("Position: {}", self.drivetrain.get_pose());
            // let vf = self.otos.vf();
            // let vs = self.otos.vs();
            // let omega = self.otos.omega();
            // info!("Velocity: ({:?}, {:?}, {:?})", vf, vs, omega);
            let x = self.otos.x().as_inches();
            let y = self.otos.y().as_inches();
            let h = self.otos.h().as_degrees();
            info!("Position: ({:?}, {:?}, {:?})", x, y, h);
            
            if state.button_down.is_now_pressed() {
                self.drivetrain.set_pose(Pose::new(0.0.inch(), 0.0.inch(), self.drivetrain.get_pose().h))
            }

            if state.button_left.is_pressed() {
                turn.turn_to(&mut self.drivetrain, 0.0.deg(), Duration::from_millis(1000))
                    .await;
            }

            if state.button_up.is_pressed() {
                move_to
                    .move_to_point(
                        &mut self.drivetrain,
                        Vec2::new(0.0.tile(), 24.0.inch()),
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

    let mut imu = InertialSensor::new(peripherals.port_10);
    match imu.calibrate().await {
        Ok(_) => info!("Calibration Successful"),
        Err(e) => error!("Error {:?}", e),
    }

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
                Pose::new(0.0.inch(), 0.0.inch(), 0.0.deg()),
                TrackingWheel::new(
                    peripherals.adi_c,
                    peripherals.adi_d,
                    Direction::Reverse,
                    2.5.inch(),
                    2.0.inch(),
                ),
                TrackingWheel::new(
                    peripherals.adi_a,
                    peripherals.adi_b,
                    Direction::Forward,
                    2.5.inch(),
                    2.0.inch(),
                ),
                imu,
            ),
            2.5.inch(),
            12.0.inch(),
        ),
        intake: Intake::new(MotorGroup::new(vec![
            Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
        ])),
        otos: Otos::new(
            peripherals.port_20,
            Pose::new(0.0.inch(), 0.0.inch(), -90.0.deg()),
        )
        .await,
    };

    robot.compete().await;
}
