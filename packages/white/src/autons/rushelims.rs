use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, turn::Turn},
    subsystems::intake::DoorCommands,
};
use futures_lite::future::zip;
use vexide::{
    math::Angle,
    prelude::{Motor, sleep},
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID, RED_LEFT_GOAL, RED_LEFT_LOADER},
};

impl Robot {
    pub async fn rushelims(&mut self) {
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 1.0,
                ..Default::default()
            },
            None,
        );

        let mut turn = Turn::new(
            ANGULAR_PID,
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

        dt.set_pose(Pose::new(56.0, 21.5, Angle::from_degrees(90.0)));

        self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
        zip(
            async {
                _ = linear.drive_distance(dt, 28.0).await;
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);

                _ = move_to
                    .speed(0.75)
                    .timeout(Duration::from_millis(1500))
                    .move_to_point(dt, Vec2::new(44.5, 69.0))
                    .await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                _ = self.lift.set_high();
                sleep(Duration::from_millis(500)).await;
                _ = self.lift.set_low();
            },
        )
        .await;
        _ = turn.tolerance(Angle::from_degrees(5.0))
            .turn_to_point(dt, Vec2::new(55.0, 53.0), true)
            .await;
        _ = move_to
            .timeout(Duration::from_millis(1000))
            .move_to_point(dt, Vec2::new(55.0, 52.0))
            .await;
        _ = turn.timeout(Duration::from_millis(750))
            .turn_to(dt, Angle::from_degrees(45.0))
            .await;
        self.intake.set_voltage(0.0);
        _ = self.duck_bill.set_high();
        _ = linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, 4.5)
            .await;
        self.intake.set_voltage(10.0);
        sleep(Duration::from_millis(750)).await;
        _ = linear
            .timeout(Duration::from_millis(2000))
            .drive_to_point(dt, Vec2::new(26.0, 26.0), true)
            .await;
        _ = self.duck_bill.set_low();
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        _ = turn.timeout(Duration::from_millis(2000))
            .settle_velocity(5.0_f64.to_radians())
            .turn_to_point(dt, RED_LEFT_LOADER, false)
            .await;
        _ = self.match_loader.set_high();
        _ = linear
            .speed(0.3)
            .timeout(Duration::from_millis(7500))
            .drive_distance(dt, 20.0)
            .await;

        _ = linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, -7.5)
            .await;
        self.intake.set_voltage(0.0);
        _ = self.match_loader.set_low();
        sleep(Duration::from_millis(500)).await; // wait for balls to settle in robot
        self.intake.set_door(DoorCommands::Close);
        sleep(Duration::from_millis(10)).await;
        self.intake.set_door(DoorCommands::Off);

        _ = self.lift.set_high();
        _ = self.wing.set_high();

        // change to radians / second
        _ = turn.settle_velocity(10.0_f64.to_radians())
            .timeout(Duration::from_millis(1000))
            .turn_to_point(dt, RED_LEFT_GOAL, false)
            .await;

        // Score on goal
        _ = self.duck_bill.set_high();
        zip(
            async {
                _ = move_to
                    .timeout(Duration::from_millis(5000))
                    .move_to_point(dt, RED_LEFT_GOAL)
                    .await;
            },
            async {
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
            .drive_distance(dt, -10.0)
            .await;
        _ = linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, 10.0)
            .await;

        sleep(Duration::from_millis(5000)).await;
    }
}
