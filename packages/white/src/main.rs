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
        intake::{DoorCommands, Intake},
    },
    theme::STOUT_ROBOT,
};
use log::{LevelFilter, info};
use vexide::{math::Angle, prelude::*, smart::motor::BrakeMode};

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain,
    intake: Intake,
    lift: AdiDigitalOut,
    duck_bill: AdiDigitalOut,
    match_loader: AdiDigitalOut,
    wing: AdiDigitalOut,
    pose: Rc<RefCell<Pose>>,
    settings: Rc<RefCell<Settings>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("autnomous");
        let time = Instant::now();
        let route = self.settings.borrow().index;
        self.intake.set_door(DoorCommands::On);

        match route {
            1 => self.rushelims().await,
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

            if mappings.wing.is_pressed() {
                _ = self.wing.set_low();
            } else if self.lift.level().is_ok_and(|level| level.is_high()) {
                _ = self.wing.set_high();
            } else {
                _ = self.wing.set_low();
            }

            if mappings.match_load.is_now_pressed() {
                _ = self.match_loader.toggle();
            }

            if mappings.enable_color.is_now_pressed() {
                self.intake.set_door(match self.intake.door() {
                    DoorCommands::ForceOff => DoorCommands::On,
                    _ => DoorCommands::ForceOff,
                });
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
                if state.button_down.is_pressed() {
                    // self.drivetrain.set_pose(Pose::new(0.0, 0.0, Angle::QUARTER_TURN));
                    self.drivetrain.set_pose(Pose::default());
                }
            }

            info!("{}", self.drivetrain.pose());

            sleep(Duration::from_millis(10)).await;
        }
    }
}
#[vexide::main(banner(theme = STOUT_ROBOT))]
async fn main(peripherals: Peripherals) {
    Logger.init(LevelFilter::Trace).unwrap();

    let adi_expander = AdiExpander::new(peripherals.port_3);

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

    let mut imu = Imu::new(vec![
        InertialSensor::new(peripherals.port_19),
        InertialSensor::new(peripherals.port_18),
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
                    Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
                ],
                motor_controller,
            ),
            MotorGroup::new(
                vec![
                    Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
                ],
                motor_controller,
            ),
            Odometry::new(starting_position.clone(), wheel_1, wheel_2, imu),
            2.5,
            12.0,
        ),
        intake: Intake::new(
            Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
            AdiDigitalOut::new(adi_expander.adi_b),
            color_sort,
            Duration::from_millis(85),
            DoorCommands::On,
            settings.clone(),
        ),
        lift: AdiDigitalOut::new(peripherals.adi_f),
        duck_bill: AdiDigitalOut::new(peripherals.adi_g),
        match_loader: AdiDigitalOut::new(adi_expander.adi_a),
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
        vec!["Select Auton", "Rush Elims", "Skills"],
        settings.clone(),
    );
}
