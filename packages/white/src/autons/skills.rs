use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{linear::Linear, move_to::MoveTo, turn::Turn},
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
    pub async fn skills(&mut self) {
        let mut linear = Linear::new(LINEAR_PID, 1.0);
        let mut turn = Turn::new(ANGULAR_PID, Angle::from_degrees(1.0));

        let mut move_to = MoveTo::new(
            Pid::new(30.0, 2.0, 6.0, 12.0),
            Pid::new(20.0, 0.0, 0.0, 0.0),
            0.5,
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(56.0, 21.5, Angle::QUARTER_TURN));
        self.settings.borrow_mut().enable_sort = DoorCommands::Off;

        self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
        zip(
            async {
                linear.drive_distance(dt, 28.0).await;
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);

                move_to
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
        turn.chain(5.0)
            .turn_to_point(dt, Vec2::new(55.0, 53.0), true)
            .await;
        move_to
            .timeout(Duration::from_millis(1000))
            .move_to_point(dt, Vec2::new(55.0, 52.0))
            .await;
        turn.timeout(Duration::from_millis(750))
            .turn_to(dt, Angle::from_degrees(45.0))
            .await;
        self.intake.set_voltage(0.0);
        _ = self.duck_bill.set_high();
        linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, 6.0)
            .await;
        self.intake.set_voltage(7.0);
        sleep(Duration::from_millis(750)).await;
        linear
            .timeout(Duration::from_millis(2000))
            .drive_to_point(dt, Vec2::new(25.0, 25.0), true)
            .await;
        _ = self.duck_bill.set_low();
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        turn.timeout(Duration::from_millis(2000))
            // change to radians / second
            .settle_velocity(5.0)
            .turn_to_point(dt, RED_LEFT_LOADER, false)
            .await;
        _ = self.match_loader.set_high();
        linear
            .speed(0.3)
            .timeout(Duration::from_millis(7500))
            .drive_distance(dt, 20.0)
            .await;

        linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, -7.5)
            .await;
        self.intake.set_voltage(0.0);
        _ = self.match_loader.set_low();
        sleep(Duration::from_millis(500)).await; // wait for balls to settle in robot
        sleep(Duration::from_millis(10)).await;

        _ = self.lift.set_high();
        _ = self.wing.set_high();

        turn.settle_velocity(10.0)
            .timeout(Duration::from_millis(1000))
            .turn_to_point(dt, RED_LEFT_GOAL, false)
            .await;

        // Score on goal
        _ = self.duck_bill.set_high();
        zip(
            async {
                move_to
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
        move_to
            .timeout(Duration::from_millis(2500))
            .move_to_point(dt, Vec2::new(48.0, 10.0))
            .await;

        turn.timeout(Duration::from_millis(1000))
            .turn_to(dt, Angle::ZERO)
            .await;

        let target = Vec2::new(72.0, self.pose.borrow().y);

        linear
            .timeout(Duration::from_millis(2500))
            .drive_to_point(dt, target, false)
            .await;
    }
}
