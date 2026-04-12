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
    mappings::{ControllerMappingsLever, DriveMode},
    settings::{Color, Settings},
    subsystems::{
        drivetrain::Drivetrain,
        intakes::lever::{Lever, LeverStage},
    },
    theme::STOUT_ROBOT,
};
use lazy_static::lazy_static;
use log::{LevelFilter, info};
use vexide::{adi::digital::LogicLevel, math::Angle, prelude::*};

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain,
    lever: Lever,
    lift: AdiDigitalOut,
    duck_bill: AdiDigitalOut,
    match_loader: AdiDigitalOut,
    wing: AdiDigitalOut,
    pose: Rc<RefCell<Pose>>,
    settings: Rc<RefCell<Settings>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let time = Instant::now();
        let route = self.settings.borrow().index;

        match route {
            1 => self.shhhhhh().await,
            2 => self.inch().await,
            _ => (),
        }

        info!("Time elapsed: {:?}", time.elapsed());
    }

    async fn driver(&mut self) {
        let mut lever_voltage = Motor::V5_MAX_VOLTAGE/2.0;
        let mut open_bill = false;
        let mut smart_score = true;
        _ = self
            .controller
            .set_text(format!("lever voltage: {lever_voltage}"), 1, 1)
            .await;
        _ = self
            .controller
            .set_text(format!("smart score: {smart_score}"), 2, 1)
            .await;

        loop {
            let state = self.controller.state().unwrap_or_default();
            let mappings = ControllerMappingsLever {
                drive_mode: DriveMode::Arcade {
                    power: state.left_stick,
                    turn: state.right_stick,
                },
                intake: state.button_r1,
                outake: state.button_r2,
                lever: state.button_l1,
                lspeed: state.button_left,
                lift: state.button_x,
                duck_bill: state.button_a,
                wing: state.button_l2,
                match_load: state.button_up,
                brake: state.button_b,
            };

            self.drivetrain.drive(&mappings.drive_mode);

            if mappings.intake.is_pressed() {
                self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
            } else if mappings.outake.is_pressed() {
                self.lever.set_intake(-Motor::V5_MAX_VOLTAGE);
            } else {
                self.lever.set_intake(0.0);
            }

            if mappings.duck_bill.is_now_pressed() {
                open_bill = !open_bill;
            }

            if state.button_power.is_now_pressed() {
                smart_score = !smart_score;
                _ = self
                    .controller
                    .set_text(format!("smart score: {smart_score}"), 2, 1)
                    .await;
            }

            match self.lever.stage() {
                LeverStage::Score(..) => _ = self.duck_bill.set_high(),
                LeverStage::Reset => (),
                LeverStage::Idle => {
                    if open_bill {
                        _ = self.duck_bill.set_high();
                    } else if !mappings.lever.is_pressed() {
                        _ = self.duck_bill.set_low();
                    }
                }
            }

            if mappings.lever.is_now_pressed() {
                let lever_stage = match self.lever.stage() {
                    LeverStage::Score(..) => LeverStage::Reset,
                    LeverStage::Reset => LeverStage::Idle,
                    LeverStage::Idle => LeverStage::Score(lever_voltage, smart_score),
                };

                self.lever.score(lever_stage);
            }

            if mappings.lspeed.is_now_pressed() {
                if lever_voltage == 4.0 {
                    lever_voltage += 2.0;
                } else {
                    lever_voltage += 3.0;
                }

                if lever_voltage > 12.0 {
                    lever_voltage = 4.0;
                }

                _ = self
                    .controller
                    .set_text(format!("lever voltage: {lever_voltage:2}"), 1, 1)
                    .await;
            }

            if mappings.lift.is_now_pressed() {
                _ = self.lift.toggle();
            }

            if mappings.wing.is_pressed() {
                _ = self.wing.set_high();
            } else {
                _ = self.wing.set_low();
            }

            if mappings.match_load.is_now_pressed() {
                _ = self.match_loader.toggle();
            }

            if self.settings.borrow().test_auton {
                self.autonomous().await;
                self.settings.borrow_mut().test_auton = false;
            }

            if state.button_x.is_pressed() {
                if state.button_down.is_now_pressed() {
                    // self.drivetrain.set_pose(Pose::new(0.0, 0.0, Angle::QUARTER_TURN));
                    self.drivetrain.set_pose(Pose::default());
                }
            }

            info!("Pose: {}",self.drivetrain.pose() );

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

lazy_static! {
    static ref LOGGER: Logger = Logger::new();
}

#[vexide::main(banner(theme = STOUT_ROBOT))]
async fn main(peripherals: Peripherals) {
    LOGGER.init(LevelFilter::Trace).unwrap();

    // RADIO PORTS DO NOT REMOVE
    // drop(peripherals.port_1);
    drop(peripherals.port_3);

    let wheel_1 = TrackingWheel::new(
        peripherals.adi_e,
        peripherals.adi_f,
        2.362204724,
        Vec2::new(1.61226751, 1.00183612),
        Angle::from_degrees(45.0),
    );
    let wheel_2 = TrackingWheel::new(
        peripherals.adi_g,
        peripherals.adi_h,
        2.362204724,
        Vec2::new(1.61226751, -1.00183612),
        Angle::from_degrees(-45.0),
    );

    // TODO - make imu calibrate at the same time
    let mut imu = Imu::new(
        vec![
            InertialSensor::new(peripherals.port_5),
            InertialSensor::new(peripherals.port_6),
        ],
        1.0,
    );
    imu.calibrate().await;

    let rcl = RaycastLocalization::new(
        vec![
            WallDistanceSensor::new(
                peripherals.port_2,
                Vec2::new(-4.783, -4.806), //14.9/ 2 14.791
                Angle::HALF_TURN,
                70..130,
            ),
            WallDistanceSensor::new(
                peripherals.port_1,
                Vec2::new(-4.635, -4.61),
                -Angle::QUARTER_TURN,
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

    let relative_position = Pose::new(70.2, 23.0, Angle::ZERO);
    let corrected = rcl.corrected_pose(relative_position, 10.0);
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
                    Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_17, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
                ],
                motor_controller,
            ),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
                ],
                motor_controller,
            ),
            Odometry::new(starting_position.clone(), wheel_1, wheel_2, imu),
            2.5,
            12.0,
        ),
        lever: Lever::new(
            Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                ],
                None,
            ),
            RotationSensor::new(peripherals.port_7, Direction::Forward),
        ),
        lift: AdiDigitalOut::with_initial_level(peripherals.adi_a, LogicLevel::Low),
        duck_bill: AdiDigitalOut::with_initial_level(peripherals.adi_d, LogicLevel::Low),
        match_loader: AdiDigitalOut::with_initial_level(peripherals.adi_c, LogicLevel::Low),
        wing: AdiDigitalOut::with_initial_level(peripherals.adi_b, LogicLevel::Low),
        pose: starting_position,
        settings: settings.clone(),
    };

    spawn(async move {
        robot.compete().await;
    })
    .detach();

     start_ui(
        peripherals.display,
        vec!["Select Auton", "super stout better than layke's chud ass auton route", "INCH"],
        LOGGER.clone_messages(),
        settings.clone(),
    );
}
