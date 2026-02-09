use std::{cell::RefCell, rc::Rc, time::Duration};

use atum::{
    controllers::pid::Pid,
    hardware::{
        imu::Imu,
        motor_group::{MotorController, MotorGroup},
        tracking_wheel::TrackingWheel,
    },
    localization::{odometry::Odometry, pose::Pose, vec2::Vec2},
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    motion::{move_to::MoveTo, turn::Turn},
    subsystems::{Color, RobotSettings, drivetrain::Drivetrain, intake::Intake},
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
    intake: Intake,
    lift: AdiDigitalOut,
    duck_bill: AdiDigitalOut,
    // match_loader: AdiDigitalOut,
    wing: AdiDigitalOut,
    // otos: Otos,
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
            Angle::new::<degree>(0.5),
            AngularVelocity::new::<degree_per_second>(5.0),
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
                lift: state.button_right,
                duck_bill: state.button_down,
                swap_color: state.button_power,
                enable_color: state.button_power,
            };

            self.drivetrain.drive(&mappings.drive_mode);

            if mappings.intake.is_pressed() {
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            } else if mappings.outake.is_pressed() {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                self.intake.set_voltage(0.0);
            }

            if mappings.lift.is_now_pressed() {
                _ = self.lift.toggle();
            }
            if mappings.duck_bill.is_now_pressed() {
                _ = self.duck_bill.toggle();
            }
            if state.button_y.is_now_pressed() {
                _ = self.wing.toggle();
            }

            // info!("Drivetrain: {}", self.drivetrain.pose());
            // info!("OTOS: {}", self.otos.pose());

            if state.button_x.is_pressed() {
                if state.button_down.is_now_pressed() {
                    self.drivetrain.set_pose(Pose::new(
                        Length::new::<inch>(0.0),
                        Length::new::<inch>(0.0),
                        Angle::ZERO,
                    ))
                }

                // tuning PID constants for angular movement
                if state.button_left.is_now_pressed() {
                    turn.turn_to(
                        &mut self.drivetrain,
                        Angle::ZERO,
                        Duration::from_millis(1000),
                    )
                    .await;
                }

                // testing and tuning seeking movement
                if state.button_up.is_now_pressed() {
                    move_to
                        .move_to_point(
                            &mut self.drivetrain,
                            Vec2::new(Length::new::<inch>(24.0), Length::ZERO),
                            Duration::from_secs(8),
                            Direction::Forward,
                        )
                        .await;
                }
            } 

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    Logger.init(LevelFilter::Trace).unwrap();

    let mut imu = Imu::new(vec![
        InertialSensor::new(peripherals.port_5),
        InertialSensor::new(peripherals.port_15),
    ]);

    imu.calibrate().await;

    let starting_position = Pose::new(Length::ZERO, Length::ZERO, Angle::ZERO);

    let mut color_sort = OpticalSensor::new(peripherals.port_4);
    _ = color_sort.set_led_brightness(1.0);
    _ = color_sort.set_integration_time(Duration::from_millis(20));

    let settings = Rc::new(RefCell::new(RobotSettings {
        color: Color::Red,
        enable_color: true,
    }));

    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
                ],
                Some(MotorController::new(
                    Pid::new(0.0, 0.0, 0.0, 0.0),
                    0.0,
                    0.0,
                    0.0,
                )),
            ),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
                ],
                Some(MotorController::new(
                    Pid::new(0.0, 0.0, 0.0, 0.0),
                    0.0,
                    0.0,
                    0.0,
                )),
            ),
            Odometry::new(
                starting_position,
                TrackingWheel::new(
                    peripherals.adi_h,
                    peripherals.adi_g,
                    Direction::Forward,
                    Length::new::<millimeter>(64.8),
                    Vec2::new(Length::new::<inch>(0.086), Length::new::<inch>(0.0)),
                    Angle::new::<degree>(90.0),
                ),
                TrackingWheel::new(
                    peripherals.adi_e,
                    peripherals.adi_f,
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
        intake: Intake::new(
            Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
            AdiDigitalOut::new(peripherals.adi_c),
            color_sort,
            Duration::from_millis(100),
            settings.clone(),
        ),
        lift: AdiDigitalOut::new(peripherals.adi_b),
        duck_bill: AdiDigitalOut::new(peripherals.adi_d),
        wing: AdiDigitalOut::new(peripherals.adi_a),
    };

    robot.compete().await;
}
