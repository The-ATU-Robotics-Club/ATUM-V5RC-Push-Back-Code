mod autos;

use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use atum::{
    controllers::pid::Pid,
    hardware::{imu::Imu, motor_group::MotorGroup, tracking_wheel::TrackingWheel},
    localization::{odometry::Odometry, pose::Pose, vec2::Vec2},
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    motion::move_to::MoveTo,
    subsystems::{Color, RobotSettings, drivetrain::Drivetrain, intake::Intake},
    theme::STOUT_ROBOT,
};
use log::{LevelFilter, info};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        f64::{Angle, Length},
        length::{inch, millimeter},
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
    brake: AdiDigitalOut,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let time = Instant::now();
        let path = 5;

        match path {
            0 => self.qual().await,
            1 => self.elims().await,
            2 => self.safequals().await,
            3 => self.rushelims().await,
            4 => self.rushcontrol().await,
            5 => self.skills().await,
            _ => (),
        }

        info!("Time elapsed: {:?}", time.elapsed());
    }

    async fn driver(&mut self) {
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
                wing: state.button_b,
                match_load: state.button_a,
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

            if mappings.wing.is_now_pressed() {
                _ = self.wing.toggle();
            }

            if mappings.match_load.is_now_pressed() {
                _ = self.match_loader.toggle();
            }

            if state.button_x.is_pressed() {
                if state.button_down.is_pressed() {
                    self.drivetrain.set_pose(Pose::new(
                        Length::new::<inch>(0.0),
                        Length::new::<inch>(0.0),
                        Angle::HALF_TURN,
                    ));
                }

                if state.button_right.is_pressed() {
                    let mut move_to = MoveTo::new(
                        Pid::new(30.0, 2.0, 6.0, 12.0),
                        Pid::new(20.0, 0.0, 0.0, 0.0),
                        Length::new::<inch>(0.5),
                    );

                    move_to
                        .timeout(Duration::from_millis(3000))
                        .move_to_point(
                            &mut self.drivetrain,
                            Vec2::new(Length::new::<inch>(60.0), Length::new::<inch>(-4.0)),
                        )
                        .await;
                }
            }

            info!("Drivetrain: {}", self.drivetrain.pose());

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
            Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
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
        brake: AdiDigitalOut::new(adi_expander.adi_b),
    };

    robot.compete().await;
}
