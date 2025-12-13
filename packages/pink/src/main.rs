use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    hardware::{imu::Imu, motor_group::MotorGroup, otos::Otos, tracking_wheel::TrackingWheel},
    localization::{odometry::Odometry, pose::Pose, vec2::Vec2},
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    motion::move_to::MoveTo,
    subsystems::drivetrain::Drivetrain,
};
use log::{LevelFilter, info};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        f64::{Angle, Length, Velocity},
        length::{inch, millimeter},
        velocity::inch_per_second,
    },
};
use vexide::prelude::*;

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain,
    intake: Vec<Motor>,
    lift: AdiDigitalOut,
    duck_bill: AdiDigitalOut,
    match_loader: AdiDigitalOut,
    wing: AdiDigitalOut,
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

        loop {
            let state = self.controller.state().unwrap_or_default();
            let mappings = ControllerMappings {
                drive_mode: DriveMode::Arcade {
                    power: state.left_stick,
                    turn: state.right_stick,
                },
                intake_high: state.button_r1,
                intake_low: state.button_r2,
                outake_high: state.button_l1,
                outake_low: state.button_l2,
                lift: state.button_y,
                duck_bill: state.button_right,
            };

            self.drivetrain.drive(&mappings.drive_mode);

            if mappings.intake_high.is_pressed() {
                _ = self.intake[0].set_voltage(Motor::V5_MAX_VOLTAGE);
                _ = self.intake[1].set_voltage(Motor::V5_MAX_VOLTAGE);
            } else if mappings.intake_low.is_pressed() {
                _ = self.intake[0].set_voltage(-Motor::V5_MAX_VOLTAGE);
                _ = self.intake[1].set_voltage(-Motor::V5_MAX_VOLTAGE);
            }
            if mappings.outake_high.is_pressed() {
                _ = self.intake[2].set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else if mappings.outake_low.is_pressed() {
                _ = self.intake[2].set_voltage(Motor::V5_MAX_VOLTAGE);
            }

            if !mappings.intake_high.is_pressed()
                && !mappings.intake_low.is_pressed()
                && !mappings.outake_high.is_pressed()
                && !mappings.outake_low.is_pressed()
            {
                _ = self.intake[0].set_voltage(0.0);
                _ = self.intake[1].set_voltage(0.0);
                _ = self.intake[2].set_voltage(0.0);
            }

            if mappings.lift.is_now_pressed() {
                _ = self.lift.toggle();
            }
            if mappings.duck_bill.is_now_pressed() {
                _ = self.duck_bill.toggle();
            }

            info!("Drivetrain: {}", self.drivetrain.pose());
            info!("OTOS: {}", self.otos.pose());

            if state.button_down.is_now_pressed() {
                self.drivetrain.set_pose(Pose::new(
                    Length::ZERO,
                    Length::ZERO,
                    self.drivetrain.pose().h,
                ))
            }

            if state.button_left.is_now_pressed() {
                _ = self.match_loader.toggle();
            }

            if state.button_x.is_now_pressed() {
                _ = self.wing.toggle();
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    Logger.init(LevelFilter::Trace).unwrap();

    let mut imu = Imu::new(vec![
        InertialSensor::new(peripherals.port_3),
        InertialSensor::new(peripherals.port_4),
    ]);

    imu.calibrate().await;

    let starting_position = Pose::new(Length::ZERO, Length::ZERO, Angle::ZERO);

    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
                ],
                None,
            ),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                ],
                None,
            ),
            Odometry::new(
                starting_position,
                TrackingWheel::new(
                    peripherals.adi_e,
                    peripherals.adi_f,
                    Direction::Forward,
                    Length::new::<millimeter>(64.8),
                    Vec2::new(Length::new::<inch>(0.086), Length::new::<inch>(0.0)),
                    Angle::new::<degree>(90.0),
                ),
                TrackingWheel::new(
                    peripherals.adi_g,
                    peripherals.adi_h,
                    Direction::Forward,
                    Length::new::<millimeter>(64.8),
                    Vec2::new(Length::new::<inch>(0.0), Length::new::<inch>(-1.685)),
                    Angle::new::<degree>(0.0),
                ),
                imu,
            ),
            Length::new::<inch>(2.5),
            Length::new::<inch>(12.0),
        ),
        intake: vec![
            Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
        ],
        lift: AdiDigitalOut::new(peripherals.adi_a),
        duck_bill: AdiDigitalOut::new(peripherals.adi_b),
        match_loader: AdiDigitalOut::new(peripherals.adi_c),
        wing: AdiDigitalOut::new(peripherals.adi_d),
        otos: Otos::new(
            peripherals.port_2,
            starting_position,
            Pose::new(
                Length::ZERO,
                Length::new::<inch>(3.275),
                Angle::new::<degree>(-90.0),
            ),
        )
        .await,
    };

    robot.compete().await;
}
