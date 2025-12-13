use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    hardware::{imu::Imu, motor_group::MotorGroup, tracking_wheel::TrackingWheel},
    localization::{odometry::Odometry, pose::Pose, vec2::Vec2},
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    motion::{linear::Linear, move_to::MoveTo, turn::Turn},
    subsystems::drivetrain::Drivetrain,
    theme::STOUT_ROBOT,
};
use log::{LevelFilter, info};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        angular_velocity::degree_per_second,
        f64::{Angle, AngularVelocity, Length, Velocity},
        length::{inch, millimeter},
        velocity::inch_per_second,
    },
};
use vexide::prelude::*;

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain,
    intake: Vec<Motor>,
    // otos: Otos,
}

impl Compete for Robot {
    // Make an autonomous selector and separate paths in a different file
    async fn autonomous(&mut self) {
        let mut linear = Linear::new(
            Pid::new(1.2, 0.64, 0.10, 12.0),
            Length::new::<inch>(0.5),
            Velocity::new::<inch_per_second>(2.5),
        );

        let mut angular = Turn::new(
            Pid::new(20.0, 2.0, 0.85, 25.0),
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(1.0),
        );

        self.drivetrain.set_pose(Pose::new(
            Length::ZERO,
            Length::ZERO,
            Angle::new::<degree>(-90.0),
        ));

        let dt = &mut self.drivetrain;

        // linear
        //     .drive_distance(
        //         &mut self.drivetrain,
        //         Length::new::<inch>(41.0),
        //         Duration::from_millis(1000),
        //     )
        //     .await;
        // linear
        //     .drive_distance(
        //         &mut self.drivetrain,
        //         Length::new::<inch>(-25.0),
        //         Duration::from_secs(1000),
        //     )
        //     .await;

        // angular.turn_to(
        //     &mut self.drivetrain,
        //     Angle::new::<degree>(-130.0),
        //     Duration::from_secs(1),
        // )
        // .await;

        // linear
        //     .drive_distance(
        //         &mut self.drivetrain,
        //         Length::new::<inch>(28.0),
        //         Duration::from_millis(1000),
        //     )
        //     .await;

        // angular.turn_to(
        //     &mut self.drivetrain,
        //     Angle::new::<degree>(180.0),
        //     Duration::from_secs(1),
        // )
        // .await;

        // linear
        //     .drive_distance(
        //         &mut self.drivetrain,
        //         Length::new::<inch>(11.0),
        //         Duration::from_secs(1),
        //     )
        //     .await;

        // sleep(Duration::from_secs(1)).await;

        // linear
        //     .drive_distance(
        //         &mut self.drivetrain,
        //         Length::new::<inch>(-5.0),
        //         Duration::from_secs(1),
        //     )
        //     .await;

        // angular.turn_to(
        //     &mut self.drivetrain,
        //     Angle::ZERO,
        //     Duration::from_secs(1),
        // )
        // .await;

        // linear
        //     .drive_distance(
        //         &mut self.drivetrain,
        //         Length::new::<inch>(20.0),
        //         Duration::from_secs(1),
        //     )
        //     .await;

        // linear.drive_distance(dt, Length::new::<inch>(36.0), Duration::from_secs(2)).await;
        // angular.turn_to(dt, Angle::new::<degree>(-180.0), Duration::from_millis(750)).await;
        // linear.drive_distance(dt, Length::new::<inch>(12.0), Duration::from_secs(1)).await;
        // sleep(Duration::from_secs(1)).await;
        // linear.drive_distance(dt, Length::new::<inch>(-12.0), Duration::from_secs(2)).await;
        // angular.turn_to(dt, Angle::new::<degree>(5.0),Duration::from_millis(1000)).await;
        // linear.drive_distance(dt, Length::new::<inch>(15.0), Duration::from_secs(2)).await;
        // sleep(Duration::from_millis(1500)).await;
        // linear.drive_distance(dt, Length::new::<inch>(-15.0), Duration::from_secs(2)).await;
        // angular.turn_to(dt, Angle::new::<degree>(-180.0), Duration::from_millis(1250)).await;
        // linear.drive_distance(dt, Length::new::<inch>(12.0), Duration::from_secs(1)).await;
        // sleep(Duration::from_secs(2)).await;
        // linear.drive_distance(dt, Length::new::<inch>(-12.0), Duration::from_secs(2)).await;
        // angular.turn_to(dt, Angle::new::<degree>(5.0), Duration::from_millis(1000)).await;
        // linear.drive_distance(dt, Length::new::<inch>(17.0), Duration::from_secs(2)).await;
        // sleep(Duration::from_millis(1500)).await;

        linear
            .drive_distance(dt, Length::new::<inch>(36.0), Duration::from_secs(2))
            .await;
        angular
            .turn_to(dt, Angle::new::<degree>(-180.0), Duration::from_millis(750))
            .await;
        linear
            .drive_distance(dt, Length::new::<inch>(12.0), Duration::from_secs(1))
            .await;
        sleep(Duration::from_secs(1)).await;
        linear
            .drive_distance(dt, Length::new::<inch>(-12.0), Duration::from_secs(2))
            .await;
        angular
            .turn_to(dt, Angle::new::<degree>(47.0), Duration::from_millis(1000))
            .await;
        linear
            .drive_distance(dt, Length::new::<inch>(47.0), Duration::from_millis(1750))
            .await;
        sleep(Duration::from_millis(1000)).await;
        linear
            .drive_distance(dt, Length::new::<inch>(-43.0), Duration::from_secs(2))
            .await;
        angular
            .turn_to(
                dt,
                Angle::new::<degree>(-176.0),
                Duration::from_millis(1250),
            )
            .await;
        linear
            .drive_distance(dt, Length::new::<inch>(15.0), Duration::from_secs(1))
            .await;
        sleep(Duration::from_secs(2)).await;
        linear
            .drive_distance(dt, Length::new::<inch>(-12.0), Duration::from_secs(2))
            .await;
        angular
            .turn_to(dt, Angle::new::<degree>(0.0), Duration::from_millis(1000))
            .await;
        linear
            .drive_distance(dt, Length::new::<inch>(17.0), Duration::from_secs(2))
            .await;

        sleep(Duration::from_millis(1500)).await;
        linear
            .drive_distance(dt, Length::new::<inch>(-7.0), Duration::from_secs(2))
            .await;
        angular
            .turn_to(dt, Angle::new::<degree>(90.0), Duration::from_secs(2))
            .await;
        linear
            .drive_distance(dt, Length::new::<inch>(48.0), Duration::from_secs(2))
            .await;
        angular
            .turn_to(dt, Angle::new::<degree>(180.0), Duration::from_secs(2))
            .await;
        linear
            .drive_distance(dt, Length::new::<inch>(20.0), Duration::from_secs(1))
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

        let mut linear = Linear::new(
            Pid::new(1.2, 0.64, 0.10, 12.0),
            Length::new::<inch>(0.5),
            Velocity::new::<inch_per_second>(2.5),
        );

        let mut turn = Turn::new(
            Pid::new(20.0, 2.0, 0.85, 25.0),
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(1.0),
        );

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
                _ = self.intake[2].set_voltage(-Motor::EXP_MAX_VOLTAGE);
            } else if mappings.outake_low.is_pressed() {
                _ = self.intake[2].set_voltage(Motor::EXP_MAX_VOLTAGE);
            }

            if !mappings.intake_high.is_pressed()
                && !mappings.intake_low.is_pressed()
                && !mappings.outake_high.is_pressed()
                && !mappings.outake_low.is_pressed()
            {
                _ = self.intake[0].set_voltage(0.0);
                _ = self.intake[1].set_voltage(0.0);
            }

            info!("Drivetrain: {}", self.drivetrain.pose());
            // info!("OTOS: {}", self.otos.pose());

            if state.button_down.is_now_pressed() {
                self.drivetrain.set_pose(Pose::new(
                    Length::ZERO,
                    Length::ZERO,
                    self.drivetrain.pose().h,
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
                // move_to
                //     .move_to_point(
                //         &mut self.drivetrain,
                //         Vec2::new(Length::ZERO, Length::new::<inch>(24.0)),
                //         Duration::from_secs(8),
                //         Direction::Forward,
                //     )
                //     .await;
                linear
                    .drive_distance(
                        &mut self.drivetrain,
                        Length::new::<inch>(41.0),
                        Duration::from_millis(1000),
                    )
                    .await;
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(theme = STOUT_ROBOT))]
async fn main(peripherals: Peripherals) {
    Logger.init(LevelFilter::Trace).unwrap();

    let mut imu = Imu::new(vec![
        InertialSensor::new(peripherals.port_15),
        InertialSensor::new(peripherals.port_5),
    ]);

    imu.calibrate().await;

    let starting_position = Pose::new(Length::ZERO, Length::ZERO, Angle::ZERO);

    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
                ],
                None,
            ),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_16, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
                ],
                None,
            ),
            Odometry::new(
                starting_position,
                TrackingWheel::new(
                    peripherals.adi_c,
                    peripherals.adi_d,
                    Direction::Forward,
                    Length::new::<millimeter>(60.0),
                    Vec2::new(
                        Length::new::<inch>(-5.93803586),
                        Length::new::<inch>(-1.0057073),
                    ),
                    Angle::new::<degree>(-45.0),
                ),
                TrackingWheel::new(
                    peripherals.adi_a,
                    peripherals.adi_b,
                    Direction::Forward,
                    Length::new::<millimeter>(60.0),
                    Vec2::new(
                        Length::new::<inch>(-5.93803586),
                        Length::new::<inch>(1.0057073),
                    ),
                    Angle::new::<degree>(45.0),
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
        // otos: Otos::new(
        //     peripherals.port_2,
        //     starting_position,
        //     Pose::new(
        //         Length::ZERO,
        //         Length::new::<inch>(3.275),
        //         Angle::new::<degree>(-90.0),
        //     ),
        // )
        // .await,
    };

    robot.compete().await;
}
