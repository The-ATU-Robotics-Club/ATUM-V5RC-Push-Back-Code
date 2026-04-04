use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, swing::Swing, turn::Turn},
    subsystems::intakes::cshape::DoorCommands,
};
use futures_lite::future::zip;
use vexide::{
    math::Angle,
    prelude::{Motor, sleep},
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, BLUE_LEFT_GOAL, LINEAR_PID, RED_RIGHT_GOAL, RED_RIGHT_LOADER},
};

impl Robot {
    pub async fn skills(&mut self) {
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 0.5,
                velocity_tolerance: Some(2.5),
                ..Default::default() 
            },
            None,
        );

        let mut turn = Turn::new(
            ANGULAR_PID,
            MotionParameters {
                tolerance: Angle::from_degrees(1.0),
                velocity_tolerance: Some(10.0_f64.to_radians()),
                speed: 0.65,
                ..Default::default()
            },
        );

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            MotionParameters {
                tolerance: Angle::from_degrees(1.0),
                ..Default::default()
            },
        );

        let mut move_to = MoveTo::new(
            Pid::new(30.0, 2.0, 6.0, 12.0),
            Pid::new(20.0, 0.0, 0.0, 0.0),
            MotionParameters {
                tolerance: 0.5,
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(95.945, 20.926, Angle::ZERO));

        self.intake.set_door(DoorCommands::Open);

        // Drive to the match loader
        _ = linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, 23.0)
            .await;

        self.intake.set_door(DoorCommands::Close);

        _ = turn.timeout(Duration::from_millis(1000))
            .turn_to_point(dt, RED_RIGHT_LOADER, false)
            .await;

        self.intake.set_door(DoorCommands::Off);

        // Grab balls from match loader
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        _ = self.match_loader.set_high();

        _ = linear
            .timeout(Duration::from_millis(7500))
            .speed(0.3)
            .drive_distance(dt, 13.0)
            .await;

        // Drive to goal
        _ = linear.drive_distance(dt, -7.5).await;
        self.intake.set_voltage(0.0);
        _ = self.match_loader.set_low();
        sleep(Duration::from_millis(500)).await; // wait for balls to settle in robot
        _ = self.lift.set_high();
        _ = self.wing.set_high();
        self.intake.set_door(DoorCommands::Off);
        _ = turn.timeout(Duration::from_millis(1000))
            .turn_to_point(dt, RED_RIGHT_GOAL, false)
            .await;

        // Score on goal
        _ = self.duck_bill.set_high();
        zip(
            async {
                _ = move_to
                    .timeout(Duration::from_millis(5000))
                    .move_to_point(dt, RED_RIGHT_GOAL)
                    .await;
            },
            async {
                // sleep so robot gains velocity from driving
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
                sleep(Duration::from_millis(1500)).await;
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
                sleep(Duration::from_millis(300)).await;
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            },
        )
        .await;

        // Back up and shove balls into goal
        _ = self.duck_bill.set_low();
        _ = linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, -5.0)
            .await;
        _ = swing
            .timeout(Duration::from_millis(750))
            .swing_to(dt, Angle::ZERO, 5.0)
            .await;

        _ = linear.drive_distance(dt, -7.0).await;
        _ = turn.timeout(Duration::from_millis(1500))
            .turn_to(dt, Angle::from_degrees(90.0))
            .await;
        _ = self.wing.set_low();

        _ = move_to
            .timeout(Duration::from_millis(1500))
            .speed(0.6)
            .move_to_point(dt, Vec2::new(107.0, 55.0))
            .await;

        _ = self.wing.set_high();

        // drive to the second match load
        let target = Vec2::new(self.pose.borrow().x, 144.0 - 24.0);
        _ = linear.drive_to_point(dt, target, false).await;

        _ = self.lift.set_low();
        _ = self.wing.set_low();

        _ = turn.turn_to(dt, Angle::ZERO).await;
        let target = Vec2::new(144.0 - 24.0, self.pose.borrow().y);
        _ = linear.drive_to_point(dt, target, false).await;

        _ = turn.turn_to(dt, Angle::from_degrees(90.0)).await;
        _ = self.match_loader.set_high();
        _ = linear.drive_distance(dt, 20.0).await;

        // MIRROR ON OTHER SIDE
        // grab balls and score on long goal
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        _ = self.match_loader.set_high();

        _ = linear
            .timeout(Duration::from_millis(7500))
            .speed(0.3)
            // .drive_to_point(&mut self.drivetrain, vec2_length(RED_RIGHT_LOADER))
            .drive_distance(dt, 20.0)
            .await;

        // Drive to goal
        _ = linear.drive_distance(dt, -7.5).await;
        self.intake.set_voltage(0.0);
        _ = self.match_loader.set_low();
        sleep(Duration::from_millis(500)).await; // wait for balls to settle in robot
        _ = self.lift.set_high();
        _ = self.wing.set_high();
        self.intake.set_door(DoorCommands::Off);
        _ = turn.settle_velocity(10.0_f64.to_radians())
            .timeout(Duration::from_millis(1000))
            .turn_to_point(dt, BLUE_LEFT_GOAL, false)
            .await;

        // Score on goal
        _ = self.duck_bill.set_high();
        zip(
            async {
                _ = move_to
                    .timeout(Duration::from_millis(5000))
                    .move_to_point(dt, BLUE_LEFT_GOAL)
                    .await;
            },
            async {
                // sleep so robot gains velocity from driving
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
                sleep(Duration::from_millis(1500)).await;
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
                sleep(Duration::from_millis(300)).await;
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            },
        )
        .await;

        _ = linear.drive_distance(dt, -5.0).await;
    }
}
