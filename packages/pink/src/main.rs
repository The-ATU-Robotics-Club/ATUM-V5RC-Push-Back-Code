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
    },
    localization::{odometry::Odometry, pose::Pose, vec2::Vec2},
    logger::Logger,
    mappings::{ControllerMappings, DriveMode},
    settings::{Color, Settings},
    subsystems::{
        drivetrain::Drivetrain,
        intakes::cshape::{CShape, DoorCommands},
    },
    theme::STOUT_ROBOT,
};
use log::{LevelFilter, info};
use vexide::{math::Angle, prelude::*, smart::motor::BrakeMode};

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain,
    intake: CShape,
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

        match route {
            1 => self.rushcontrol().await,
            2 => self.skills().await,
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
    drop(peripherals.port_1);
    drop(peripherals.port_21);

    let adi_expander = AdiExpander::new(peripherals.port_2);

    let mut color_sort = OpticalSensor::new(peripherals.port_3);
    _ = color_sort.set_led_brightness(1.0);
    _ = color_sort.set_integration_time(Duration::from_millis(10));

    let wheel_1 = TrackingWheel::new(
        peripherals.adi_a,
        peripherals.adi_b,
        2.362204724,
        Vec2::new(-1.00288550, -5.93824103),
        Angle::from_degrees(45.0),
    );
    let wheel_2 = TrackingWheel::new(
        peripherals.adi_c,
        peripherals.adi_d,
        2.362204724,
        Vec2::new(1.00288550, -5.93824103),
        // Vec2::new(-5.531519437, 0.905980563),
        // Vec2::new(1.00288550, -5.41131450),
        Angle::from_degrees(-45.0),
    );

    // TODO - make imu calibrate at the same time
    let mut imu = Imu::new(vec![
        InertialSensor::new(peripherals.port_14),
        InertialSensor::new(peripherals.port_15),
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
                    Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
                ],
                motor_controller,
            ),
            Odometry::new(starting_position.clone(), wheel_1, wheel_2, imu),
            2.5,
            12.0,
        ),
        intake: CShape::new(
            Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
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
