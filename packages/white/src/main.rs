use std::{cell::RefCell, rc::Rc, time::Duration};

use atum::{
    controllers::pid::Pid,
    hardware::{imu::Imu, motor_group::MotorGroup, tracking_wheel::TrackingWheel},
    localization::{odometry::Odometry, pose::Pose, vec2::Vec2},
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    motion::{linear::Linear, move_to::MoveTo, swing::Swing, turn::Turn},
    subsystems::{drivetrain::Drivetrain, intake::Intake, Color, RobotSettings},
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
    intake: Intake,
    lift: AdiDigitalOut,
    duck_bill: AdiDigitalOut,
    match_loader: AdiDigitalOut,
    wing: AdiDigitalOut,
    // otos: Otos,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {}

    async fn driver(&mut self) {
        info!("Driver Control Started");

        let mut move_to = MoveTo::new(
            Pid::new(30.0, 1.0, 6.0, 12.0),
            Pid::new(21.0, 2.0, 0.0, 18.0),
            Length::new::<inch>(1.0),
            Velocity::new::<inch_per_second>(2.0),
            Length::new::<inch>(6.0),
        );

        let mut linear = Linear::new(
            Pid::new(47.25, 25.2, 3.95, 12.0),
            Length::new::<inch>(0.5),
            Velocity::new::<inch_per_second>(2.5),
        );

        let mut turn = Turn::new(
            Pid::new(20.0, 2.0, 0.85, 25.0),
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(1.0),
        );

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0,0.0,90.0),
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(5.0),
        );

        // sleep(Duration::from_millis(1500)).await;
        // self.drivetrain.set_pose(Pose::new(
        //     Length::ZERO,
        //     Length::ZERO,
        //     Angle::ZERO,
        // ));

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

            info!("Drivetrain: {}", self.drivetrain.pose());
            // info!("OTOS: {}", self.otos.pose());

            if state.button_down.is_now_pressed() {
                self.drivetrain.set_pose(Pose::new(
                    Length::ZERO,
                    Length::ZERO,
                    Angle::ZERO,
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

            if state.button_a.is_pressed() {
                // linear.drive_distance(&mut self.drivetrain, Length::new::<inch>(24.0), Duration::from_millis(1000)).await;
                //swing.swing_to(&mut self.drivetrain, Angle::new::<degree>(90.0), Length::new::<inch>(7.0), Duration::from_millis(10000)).await;
                move_to.
                    move_to_point(
                        &mut self.drivetrain,
                        Vec2::new(Length::new::<inch>(24.0),Length::new::<inch>(0.0)),
                        Duration::from_secs(4),
                        Direction::Forward
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
                        false,
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

    let adi_expander = AdiExpander::new(peripherals.port_3);

    let mut imu = Imu::new(vec![
        InertialSensor::new(peripherals.port_9),
        InertialSensor::new(peripherals.port_10),
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
                    Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_15, Gearset::Blue, Direction::Forward),
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
                    peripherals.adi_a,
                    peripherals.adi_b,
                    Direction::Forward,
                    Length::new::<millimeter>(60.0),
                    Vec2::new(
                        Length::new::<inch>(-5.93824103),
                        Length::new::<inch>(-1.00288550),
                    ),
                    Angle::new::<degree>(45.0),
                ),
                TrackingWheel::new(
                    peripherals.adi_c,  
                    peripherals.adi_d,
                    Direction::Forward,
                    Length::new::<millimeter>(60.0),
                    Vec2::new(
                        Length::new::<inch>(-5.93824103),
                        Length::new::<inch>(1.00288550),
                    ),
                    Angle::new::<degree>(-45.0),
                ),
                imu,
            ),

            Length::new::<inch>(2.5),
            Length::new::<inch>(12.0),
        ),
        intake: Intake::new(
            Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
            AdiDigitalOut::new(peripherals.adi_e),
            color_sort,
            Duration::from_millis(100),
            settings.clone(),
        ),
        lift: AdiDigitalOut::new(peripherals.adi_f),
        duck_bill: AdiDigitalOut::new(peripherals.adi_g),
        match_loader: AdiDigitalOut::new(adi_expander.adi_a),
        wing: AdiDigitalOut::new(peripherals.adi_h),
    };

    robot.compete().await;
}
