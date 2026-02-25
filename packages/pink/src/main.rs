mod autons;

use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use atum::{
    backend::start_ui,
    controllers::pid::Pid,
    hardware::{imu::Imu, motor_group::MotorGroup, tracking_wheel::TrackingWheel},
    localization::{odometry::Odometry, pose::Pose, vec2::Vec2},
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    motion::move_to::MoveTo,
    settings::{Color, Settings},
    subsystems::{drivetrain::Drivetrain, intake::Intake},
    theme::STOUT_ROBOT,
};
use log::{LevelFilter, debug, info};
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
    settings: Rc<RefCell<Settings>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("autnomous");
        let time = Instant::now();
        let route = self.settings.borrow().index;

        match route {
            1 => self.qual().await,
            2 => self.elims().await,
            3 => self.safequals().await,
            4 => self.rushelims().await,
            5 => self.rushcontrol().await,
            6 => self.skills().await,
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
                lift: state.button_y,
                duck_bill: state.button_l1,
                wing: state.button_l2,
                match_load: state.button_right,
                brake: state.button_b,
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

            if mappings.match_load.is_now_pressed() {
                _ = self.match_loader.toggle();
            }

            // run autonomous when button is pressed to prevent the need of a competition switch
            if self.settings.borrow().test_auton {
                {
                    let mut settings = RefCell::borrow_mut(&self.settings);
                    settings.test_auton = false;
                }

                self.autonomous().await;
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
        InertialSensor::new(peripherals.port_5),
        InertialSensor::new(peripherals.port_15),
    ]);

    imu.calibrate().await;

    let starting_position = Rc::new(RefCell::new(Pose::new(Length::ZERO, Length::ZERO, Angle::ZERO)));

    let mut color_sort = OpticalSensor::new(peripherals.port_4);
    _ = color_sort.set_led_brightness(1.0);
    _ = color_sort.set_integration_time(Duration::from_millis(20));

    let settings = Rc::new(RefCell::new(Settings {
        color: Color::Red,
        index: 0,
        test_auton: false,
        enable_sort: true,
    }));

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
                None,
            ),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
                ],
                None,
            ),
            Odometry::new(
                starting_position.clone(),
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
            "elims",
            "quals",
            "rush control",
            "rush elims",
            "safe quals",
        ],
        settings.clone(),
    );
}
