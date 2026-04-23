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
            1 => self.shhhhhh().await,
            2 => self.shhhhhh_pt2().await,
            _ => (),
        }

        info!("Time elapsed: {:?}", time.elapsed());
    }

    async fn driver(&mut self) {
        let scoring = [LeverStage::Score(8.0, 3.0), LeverStage::Score(6.0, 12.0)];
        let mut selected = 1;
        let mut open_bill = false;

        _ = self.controller.set_text(format!("Upper Stage"), 1, 1).await;

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
                duck_bill: state.button_left,
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

            // make automatic when the lift is raised
            if state.button_power.is_now_pressed() {
                if selected == 1 {
                    _ = self.controller.set_text("Upper Stage", 1, 1).await;
                    selected = 0;
                } else {
                    _ = self.controller.set_text("Lower Stage", 1, 1).await;
                    selected = 1;
                }
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
                    LeverStage::Idle => scoring[selected],
                };

                self.lever.score(lever_stage);
            }

            // if mappings.

            // _ = self
            //     .controller
            //     .set_text(format!("lever voltage: {lever_voltage:2}"), 1, 1)
            //     .await;
            // }

            if mappings.lift.is_now_pressed() {
                _ = self.lift.toggle();
            }

            if mappings.match_load.is_now_pressed() {
                _ = self.match_loader.toggle();
            }

            if mappings.wing.is_pressed() {
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

                if state.button_left.is_now_pressed() {
                    self.settings.borrow_mut().test_auton = true;
                }
            }

            // info!("Drivetrain: {}", self.drivetrain.pose());

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
    drop(peripherals.port_21);

    let wheel_1 = TrackingWheel::new(
        peripherals.adi_g,
        peripherals.adi_h,
        2.362204724,
        Vec2::new(1.61226751, 1.00183612),
        Angle::from_degrees(45.0),
    );
    let wheel_2 = TrackingWheel::new(
        peripherals.adi_e,
        peripherals.adi_f,
        2.362204724,
        Vec2::new(1.61226751, -1.00183612),
        Angle::from_degrees(-45.0),
    );

    // TODO - make imu calibrate at the same time
    let mut imu = Imu::new(
        vec![
            InertialSensor::new(peripherals.port_7),
            InertialSensor::new(peripherals.port_8),
        ],
        1.0,
    );
    imu.calibrate().await;

    let rcl = RaycastLocalization::new(
        vec![
            WallDistanceSensor::new(
                peripherals.port_3,
                Vec2::new(-4.783, 4.806), //14.9/ 2 14.791
                Angle::HALF_TURN,
                70..130,
            ),
            WallDistanceSensor::new(
                peripherals.port_4,
                Vec2::new(-4.635, 4.61),
                Angle::QUARTER_TURN,
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

    let relative_position = Pose::new(70.2, 23.0, Angle::HALF_TURN);
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
        index: 2,
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
                    Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
                ],
                motor_controller,
            ),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_15, Gearset::Blue, Direction::Forward),
                    
                ],
                motor_controller,
            ),
            Odometry::new(starting_position.clone(), wheel_1, wheel_2, imu),
            2.5,
            12.0,
        ),
        lever: Lever::new(
            Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
                ],
                None,
            ),
            RotationSensor::new(peripherals.port_5, Direction::Forward),
        ),
        lift: AdiDigitalOut::new(peripherals.adi_d),
        duck_bill: AdiDigitalOut::new(peripherals.adi_a),
        match_loader: AdiDigitalOut::new(peripherals.adi_c),
        wing: AdiDigitalOut::new(peripherals.adi_b),
        pose: starting_position,
        settings: settings.clone(),
    };

    spawn(async move {
        robot.compete().await;
    })
    .detach();

    start_ui(
        peripherals.display,
        vec![
            "Select Auton",
            "super stout better than layke's chud auton route",
            "not so super stout and probably not better than layke's chud auton route",
        ],
        LOGGER.clone_messages(),
        settings.clone(),
    );
}
