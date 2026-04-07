use core::f64;
use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use atum::{
    backend::start_ui,
    controllers::pid::Pid,
    hardware::{
        imu::Imu,
        motor_group::{MotorController, MotorGroup},
        tracking_wheel::TrackingWheel,
        wall_distance_sensor::WallDistanceSensor,
    },
    localization::{
        rcl::{RaycastLocalization, MAX_ERROR}, odometry::Odometry, pose::Pose, shape::Circle, vec2::Vec2
    },
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    motion::{linear::Linear, move_to::MoveTo, turn::Turn, MotionError, MotionParameters},
    settings::{Color, Settings},
    subsystems::{
        drivetrain::Drivetrain,
        intake::{DoorCommands, Intake},
    },
    theme::STOUT_ROBOT,
};
use log::{LevelFilter, debug, info};
use vexide::{math::Angle, prelude::*, smart::motor::BrakeMode};

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain,
    intake: Intake,
    lift: AdiDigitalOut,
    duck_bill: AdiDigitalOut,
    match_loader: AdiDigitalOut,
    wing: AdiDigitalOut,
    brake: AdiDigitalOut,
    pose: Rc<RefCell<Pose>>,
    settings: Rc<RefCell<Settings>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let time = Instant::now();
        let route = self.settings.borrow().index;

        let mut linear = Linear::new(
            Pid::new(0.1, 0.0, 0.009, 0.0),
            MotionParameters {
                tolerance: 0.5,
                velocity_tolerance: Some(10.0),
                timeout: Some(Duration::from_millis(5000)),
                ..Default::default()
            },
            None,
        );
        let mut angular = Turn::new(
            Pid::new(0.78, 0.05, 0.05, 13.5),
            MotionParameters {
                tolerance: Angle::from_degrees(1.0),
                timeout: Some(Duration::from_millis(5000)),
                speed: 0.75,
                ..Default::default()
            },
        );

        self.drivetrain.set_pose(Pose::new(77.0, 23.4, Angle::ZERO));

        _ = linear
            .drive_to_point(&mut self.drivetrain, Vec2::new(116.0, 23.0), false)
            .await;
        _ = angular
            .turn_to(&mut self.drivetrain, -Angle::QUARTER_TURN)
            .await;
        _ = linear
            .drive_to_point(&mut self.drivetrain, Vec2::new(116.0, 15.0), false)
            .await;

        // match route {
        //     _ => (),
        // }

        info!("Time elapsed: {:?}", time.elapsed());
    }

    async fn driver(&mut self) {
        let mut brake = false;

        loop {
            let state = self.controller.state().unwrap_or_default();
            let mappings = ControllerMappings {
                drive_mode: DriveMode::Arcade {
                    power: state.left_stick,
                    turn: state.right_stick,
                },
                intake: state.button_r1,
                outake: state.button_r2,
                lift: state.button_y,
                duck_bill: state.button_l1,
                wing: state.button_l2,
                match_load: state.button_right,
                brake: state.button_b,
                swap_color: state.button_left,
                enable_color: state.button_x,
                back_door: state.button_a,
            };

            if mappings.brake.is_now_pressed() {
                brake = !brake;
            }

            if brake {
                self.drivetrain.brake(BrakeMode::Hold);
            } else {
                self.drivetrain.drive(&mappings.drive_mode);
            }

            if mappings.intake.is_pressed() {
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            } else if mappings.outake.is_pressed() {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                self.intake.set_voltage(0.0);
            }

            if mappings.lift.is_now_pressed() {
                _ = self.lift.toggle();
                let door_command = self.intake.door();
                self.intake.set_door(match door_command {
                    DoorCommands::On => DoorCommands::Off,
                    DoorCommands::Off => DoorCommands::On,
                    _ => door_command,
                });
            }

            if mappings.duck_bill.is_pressed() {
                _ = self.duck_bill.set_high();
            } else {
                _ = self.duck_bill.set_low();
            }

            if mappings.match_load.is_now_pressed() {
                _ = self.match_loader.toggle();
            }

            if mappings.wing.is_pressed() {
                _ = self.wing.set_low();
            } else if self.lift.level().is_ok_and(|level| level.is_high()) {
                _ = self.wing.set_high();
            } else {
                _ = self.wing.set_low();
            }

            if mappings.brake.is_pressed() {
                _ = self.brake.set_high();
            } else {
                _ = self.brake.set_low();
            }

            if mappings.enable_color.is_now_pressed() {
                self.intake.set_door(match self.intake.door() {
                    DoorCommands::ForceOff => DoorCommands::On,
                    _ => DoorCommands::ForceOff,
                });
            }

            if mappings.swap_color.is_now_pressed() {
                let mut settings = self.settings.borrow_mut();
                settings.color = !settings.color;
            }

            self.settings.borrow_mut().color_override = mappings.back_door.is_now_pressed();

            // run autonomous when button is pressed to prevent the need of a competition switch
            if self.settings.borrow().test_auton {
                self.autonomous().await;
                self.settings.borrow_mut().test_auton = false;
            }

            if state.button_x.is_pressed() {
                if state.button_left.is_now_pressed() {
                    self.settings.borrow_mut().test_auton = true;
                }

                if state.button_down.is_pressed() {
                    // self.drivetrain.set_pose(Pose::new(97.0, 21.5, Angle::ZERO));
                    self.drivetrain.set_pose(Pose::default());
                }

                if state.button_up.is_pressed() {
                    // let mut linear = Linear::new(
                    //     Pid::new(0.1, 0.0, 0.009, 0.0),
                    //     MotionParameters {
                    //         tolerance: 0.0,
                    //         timeout: Some(Duration::from_millis(5000)),
                    //         ..Default::default()
                    //     },
                    //     None,
                    // );
                    // let mut angular = Turn::new(
                    //     Pid::new(0.78, 0.05, 0.05, 13.5),
                    //     MotionParameters {
                    //         tolerance: Angle::from_degrees(1.0),
                    //         timeout: Some(Duration::from_millis(5000)),
                    //         speed: 0.75,
                    //         ..Default::default()
                    //     },
                    // );
                    // let result = linear.drive_distance(&mut self.drivetrain, 12.0).await;
                    // let result = angular.turn_to(&mut self.drivetrain, Angle::QUARTER_TURN).await;
                    let mut move_to = MoveTo::new(
                        Pid::new(0.1, 0.0, 0.02, 12.0),
                        Pid::new(0.067, 0.0, 0.0, 13.5),
                        MotionParameters {
                            tolerance: 1.0,
                            timeout: Some(Duration::from_millis(2500)),
                            speed: 0.5,
                            ..Default::default()
                        },
                    );
                    let target = Vec2::new(116.0, 23.5);
                    let result = move_to.move_to_point(&mut self.drivetrain, target).await;
                    if let Err(MotionError::Timeout(error)) = result {
                        debug!("timeout {:?}", error);
                    }
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

    // RADIO PORTS DO NOT REMOVE
    drop(peripherals.port_9);
    drop(peripherals.port_21);

    let adi_expander = AdiExpander::new(peripherals.port_8);

    let mut color_sort = OpticalSensor::new(peripherals.port_7);
    _ = color_sort.set_led_brightness(1.0);
    _ = color_sort.set_integration_time(Duration::from_millis(10));

    let wheel_1 = TrackingWheel::new(
        peripherals.adi_a,
        peripherals.adi_b,
        2.362204724,
        Vec2::new(-1.00183612, -1.61226751),
        Angle::from_degrees(45.0),
    );
    let wheel_2 = TrackingWheel::new(
        peripherals.adi_c,
        peripherals.adi_d,
        2.362204724,
        Vec2::new(1.00288550, -1.61226751),
        // Vec2::new(-5.531519437, 0.905980563),
        // Vec2::new(1.00288550, -5.41131450),
        Angle::from_degrees(-45.0),
    );

    // TODO - make imu calibrate at the same time
    let mut imu = Imu::new(
        vec![
            InertialSensor::new(peripherals.port_10),
            // InertialSensor::new(peripherals.port_9),
        ],
        1.0084555556,
    );
    imu.calibrate().await;

    let rcl = RaycastLocalization::new(
        vec![
            WallDistanceSensor::new(
                peripherals.port_1,
                Vec2::new(6.03128976, 0.0),
                Angle::ZERO
            ),
            WallDistanceSensor::new(
                peripherals.port_2,
                Vec2::new(-3.94528346, 0.0),
                Angle::HALF_TURN,
            ),
            WallDistanceSensor::new(
                peripherals.port_3,
                Vec2::new(-5.21868874, -1.40196062),
                -Angle::QUARTER_TURN,
            ),
            WallDistanceSensor::new(
                peripherals.port_4,
                Vec2::new(-5.21868874, 1.40196062),
                Angle::QUARTER_TURN,
            ),
        ],
        vec![
            Circle::new(Vec2::new(23.5, 2.375), 2.375),
            Circle::new(Vec2::new(116.92, 2.375), 2.375),
            Circle::new(Vec2::new(23.5, 138.045), 2.375),
            Circle::new(Vec2::new(116.92, 138.045), 2.375),
        ],
    );

    let relative_position = Pose::new(77.0, 23.0, Angle::ZERO);
    let corrected = rcl.corrected_pose(relative_position, f64::INFINITY);
    let starting_position = Rc::new(RefCell::new(corrected));
    let cloned_pose = starting_position.clone();

    spawn(async move {
        loop {
            let corrected = rcl.corrected_pose(*cloned_pose.borrow(), MAX_ERROR);
            cloned_pose.replace(corrected);
            sleep(Duration::from_millis(30)).await;
        }
    })
    .detach();

    let settings = Rc::new(RefCell::new(Settings {
        color: Color::Red,
        index: 0,
        test_auton: false,
        color_override: false,
    }));

    let motor_controller = Some(MotorController::new(
        Pid::new(0.025, 0.0, 0.01, 0.014),
        0.83,
        0.0167,
        0.0,
    ));

    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
                ],
                None,
            ),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_16, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_17, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
                ],
                None,
            ),
            Odometry::new(starting_position.clone(), wheel_1, wheel_2, imu),
            2.5,
            12.0,
        ),
        intake: Intake::new(
            Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
            AdiDigitalOut::new(peripherals.adi_f),
            color_sort,
            Duration::from_millis(85),
            DoorCommands::On,
            settings.clone(),
        ),
        lift: AdiDigitalOut::new(peripherals.adi_g),
        duck_bill: AdiDigitalOut::new(peripherals.adi_h),
        match_loader: AdiDigitalOut::new(adi_expander.adi_a),
        wing: AdiDigitalOut::new(peripherals.adi_e),
        brake: AdiDigitalOut::new(adi_expander.adi_b),
        pose: starting_position,
        settings: settings.clone(),
    };

    spawn(async move {
        robot.compete().await;
    })
    .detach();

    start_ui(
        peripherals.display,
        vec!["Select Auton", "rush control", "skills"],
        settings.clone(),
    );
}
