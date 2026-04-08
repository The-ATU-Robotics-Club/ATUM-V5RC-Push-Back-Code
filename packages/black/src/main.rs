mod autons;

use std::{cell::RefCell, rc::Rc, time::Instant};

use atum::{
    backend::start_ui,
    controllers::pid::Pid,
    hardware::{
        imu::Imu,
        motor_group::{MotorController, MotorGroup},
        tracking_wheel::TrackingWheel,
    },
    localization::{odometry::Odometry, pose::Pose, vec2::Vec2},
    logger::Logger,
    mappings::{ControllerMappingsLever, DriveMode},
    settings::{Color, Settings},
    subsystems::{
        drivetrain::Drivetrain,
        intakes::lever::{Lever, LeverStage},
    },
    theme::STOUT_ROBOT,
};
use log::{LevelFilter, info};
use vexide::{math::Angle, prelude::*};

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
            _ => (),
        }

        info!("Time elapsed: {:?}", time.elapsed());
    }

    async fn driver(&mut self) {
        let mut lever_voltage = Motor::V5_MAX_VOLTAGE;
        let mut open_bill = false;
        let mut smart_score = false;
        _ = self.controller.set_text(format!("lever voltage: {lever_voltage}"), 1, 1).await;
        _ = self.controller.set_text(format!("smart score: {smart_score}"), 2, 1).await;

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
                lspeed: state.button_a,
                lift: state.button_y,
                duck_bill: state.button_up,
                wing: state.button_l2,
                match_load: state.button_right,
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
                _ = self.controller.set_text(format!("smart score: {smart_score}"), 2, 1).await;
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

                _ = self.controller.set_text(format!("lever voltage: {lever_voltage:2}"), 1, 1).await;
                
            }

            if mappings.lift.is_now_pressed() {
                _ = self.lift.toggle();
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

            // run autonomous when button is pressed to prevent the need of a competition switch
            if self.settings.borrow().test_auton {
                self.autonomous().await;
                self.settings.borrow_mut().test_auton = false;
            }

            if state.button_x.is_pressed() {
                if state.button_down.is_pressed() {
                    // self.drivetrain.set_pose(Pose::new(97.0, 21.5, Angle::ZERO));
                    self.drivetrain.set_pose(Pose::default());
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
    // drop(peripherals.port_1);
    drop(peripherals.port_21);

    // TODO - make imu calibrate at the same time
    let mut imu = Imu::new(vec![
        // InertialSensor::new(peripherals.port_14),
        // InertialSensor::new(peripherals.port_15),
    ]);
    imu.calibrate().await;

    let starting_position = Rc::new(RefCell::new(Pose::default()));

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
                    Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_15, Gearset::Blue, Direction::Forward),
                ],
                motor_controller,
            ),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
                ],
                motor_controller,
            ),
            Odometry::new(
                starting_position.clone(),
                TrackingWheel::new(
                    peripherals.adi_a,
                    peripherals.adi_b,
                    2.362204724,
                    Vec2::new(-5.93824103, 1.00288550),
                    Angle::from_degrees(45.0),
                ),
                TrackingWheel::new(
                    peripherals.adi_c,
                    peripherals.adi_d,
                    2.362204724,
                    Vec2::new(-5.93824103, -1.00288550),
                    Angle::from_degrees(-45.0),
                ),
                imu,
            ),
            2.5,
            12.0,
        ),
        lever: Lever::new(
            Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
                ],
                None,
            ),
            RotationSensor::new(peripherals.port_6, Direction::Forward),
        ),
        lift: AdiDigitalOut::new(peripherals.adi_e),
        duck_bill: AdiDigitalOut::new(peripherals.adi_f),
        match_loader: AdiDigitalOut::new(peripherals.adi_g),
        wing: AdiDigitalOut::new(peripherals.adi_h),
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
