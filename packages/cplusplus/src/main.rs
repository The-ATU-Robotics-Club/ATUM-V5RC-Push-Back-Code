mod autons;

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
        odometry::Odometry,
        pose::Pose,
        rcl::{MAX_ERROR, RaycastLocalization},
        shape::Circle,
        vec2::Vec2,
    },
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, turn::Turn},
    settings::{Color, Settings},
    subsystems::{drivetrain::Drivetrain, intakes::basic::Basic},
    theme::STOUT_ROBOT,
};
use lazy_static::lazy_static;
use log::{LevelFilter, info};
use vexide::{adi::digital::LogicLevel, math::Angle, prelude::*, smart::motor::BrakeMode};

use crate::autons::{ANGULAR_PID, LINEAR_PID};

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain,
    intake: Basic,
    lift: AdiDigitalOut,
    duck_bill: AdiDigitalOut,
    rake: AdiDigitalOut,
    pose: Rc<RefCell<Pose>>,
    settings: Rc<RefCell<Settings>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("autnomous");
        let time = Instant::now();
        let route = self.settings.borrow().index;

        match route {
            1 => self.skills().await,
            _ => (),
        }

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
                duck_bill: state.button_l2,
                wing: state.button_l1,
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
                self.intake.set_bottom(Motor::V5_MAX_VOLTAGE);
                self.intake.set_top(6.0);
            } else if mappings.outake.is_pressed() {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                self.intake.set_voltage(0.0);
            }

            if mappings.lift.is_now_pressed() {
                _ = self.lift.toggle();
            }

            if mappings.wing.is_now_pressed() {
                _ = self.rake.toggle();
            }

            if mappings.duck_bill.is_pressed() {
                _ = self.duck_bill.set_high();
            } else {
                _ = self.duck_bill.set_low();
            }

            if mappings.swap_color.is_now_pressed() {
                let mut setting = self.settings.borrow_mut();
                setting.color = !setting.color;
            }

            if self.settings.borrow().test_auton {
                self.autonomous().await;
                self.settings.borrow_mut().test_auton = false;
            }

            if state.button_x.is_pressed() {
                if state.button_down.is_now_pressed() {
                    self.drivetrain.set_pose(Pose::default());
                }

                if state.button_up.is_now_pressed() {
                    // let mut linear = Linear::new(
                    //     LINEAR_PID,
                    //     MotionParameters {
                    //         tolerance: 0.5,
                    //         velocity_tolerance: Some(2.5),
                    //         ..Default::default()
                    //     },
                    // );
                    //
                    // _ = linear.drive_distance(&mut self.drivetrain, 12.0).await;
                    self.drivetrain.set_arcade(0.4, 0.0);
                    // let mut pitch = self.drivetrain.odometry.pitch();
                    while self.drivetrain.odometry.pitch() > Angle::from_degrees(-1.0) {
                        sleep(Duration::from_millis(10)).await;
                        // pitch = self.drivetrain.odometry.pitch();
                    }

                    brake = true;
                }

                if state.button_right.is_now_pressed() {
                    // let mut turn = Turn::new(
                    //     ANGULAR_PID,
                    //     MotionParameters {
                    //         tolerance: Angle::from_degrees(1.0),
                    //         velocity_tolerance: Some(10.0_f64.to_radians()),
                    //         timeout: Some(Duration::from_millis(2000)),
                    //         ..Default::default()
                    //     },
                    // );
                    //
                    // _ = turn
                    //     .turn_to(&mut self.drivetrain, -Angle::QUARTER_TURN)
                    //     .await;
                    self.intake.set_voltage(-6.0);
                    sleep(Duration::from_millis(1000)).await;
                    self.intake.set_voltage(-4.5);
                    sleep(Duration::from_millis(1000)).await;
                    self.intake.set_voltage(-3.0);
                    sleep(Duration::from_millis(500)).await;
                }

                if state.button_left.is_now_pressed() {
                    // let mut move_to = MoveTo::new(
                    //     Pid::new(0.04, 0.0, 0.0025, 12.0),
                    //     Pid::new(0.03, 0.0, 0.003, 0.0),
                    //     MotionParameters {
                    //         tolerance: 0.5,
                    //         velocity_tolerance: Some(2.5),
                    //         timeout: Some(Duration::from_millis(3000)),
                    //         ..Default::default()
                    //     },
                    // );
                    //
                    // _ = move_to.move_to_point(&mut self.drivetrain, Vec2::new(24.0, 0.0)).await;
                    self.autonomous().await;
                }
            }

            info!("{}", self.drivetrain.pose());

            sleep(Duration::from_millis(10)).await;
        }
    }
}

lazy_static! {
    static ref LOGGER: Logger = Logger::new();
}

#[vexide::main(banner(theme = STOUT_ROBOT))]
async fn main(peripherals: Peripherals) {
    LOGGER.init(LevelFilter::Trace).unwrap();

    let mut color_sort = OpticalSensor::new(peripherals.port_17);
    _ = color_sort.set_led_brightness(1.0);
    _ = color_sort.set_integration_time(Duration::from_millis(10));

    let wheel_1 = TrackingWheel::new(
        peripherals.adi_a,
        peripherals.adi_b,
        2.362204724,
        Vec2::new(-0.90288550, -5.70824103),
        // Vec2::new(-5.531519437, -0.905980563),
        // Vec2::new(-1.00288550, -5.41131450),
        Angle::from_degrees(45.0),
    );
    let wheel_2 = TrackingWheel::new(
        peripherals.adi_c,
        peripherals.adi_d,
        2.362204724,
        Vec2::new(0.90288550, -5.70824103),
        // Vec2::new(-5.531519437, 0.905980563),
        // Vec2::new(1.00288550, -5.41131450),
        Angle::from_degrees(-45.0),
    );

    let mut imu = Imu::new(
        vec![
            InertialSensor::new(peripherals.port_19),
            InertialSensor::new(peripherals.port_18),
        ],
        0.996124876,
    );

    imu.calibrate().await;

    let rcl = RaycastLocalization::new(
        vec![
            WallDistanceSensor::new(
                peripherals.port_20,
                Vec2::new(7.341, 4.495), //14.9/ 2 14.791
                Angle::ZERO,
                70..130,
            ),
            WallDistanceSensor::new(
                peripherals.port_9,
                Vec2::new(-1.5085, -5.44),
                -Angle::QUARTER_TURN,
                70..130,
            ),
            WallDistanceSensor::new(
                peripherals.port_21,
                Vec2::new(-1.5085, 5.44),
                Angle::QUARTER_TURN,
                70..130,
            ),
            WallDistanceSensor::new(
                peripherals.port_10,
                Vec2::new(-7.45, -1.2645),
                Angle::HALF_TURN,
                70..130,
            ),
        ],
        vec![
            Circle::new(Vec2::new(23.5, 2.375), 3.0),
            Circle::new(Vec2::new(116.92, 2.375), 3.0),
            Circle::new(Vec2::new(23.5, 138.045), 3.0),
            Circle::new(Vec2::new(116.92, 138.045), 3.0),
        ],
    );

    let relative_position = Pose::new(70.2, 23.0, -Angle::QUARTER_TURN);
    let corrected = rcl.corrected_pose(relative_position, 48.0);
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
        index: 1,
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
                    Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
                ],
                motor_controller,
            ),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
                ],
                motor_controller,
            ),
            Odometry::new(starting_position.clone(), wheel_1, wheel_2, imu),
            2.5,
            12.0,
        ),
        intake: Basic::new(
            Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),
                ],
                None,
            ),
        ),
        lift: AdiDigitalOut::new(peripherals.adi_f),
        duck_bill: AdiDigitalOut::new(peripherals.adi_g),
        rake: AdiDigitalOut::with_initial_level(peripherals.adi_h, LogicLevel::High),
        pose: starting_position,
        settings: settings.clone(),
    };

    spawn(async move {
        robot.compete().await;
    })
    .detach();

    start_ui(
        peripherals.display,
        vec!["Select Auton", "Skills"],
        LOGGER.clone_messages(),
        settings.clone(),
    );
}
