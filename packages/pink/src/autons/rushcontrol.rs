use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{linear::Linear, move_to::MoveTo, swing::Swing, turn::Turn},
    subsystems::intake::DoorCommands,
};
use futures_lite::future::zip;
use vexide::{
    math::Angle,
    prelude::{Motor, sleep},
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID, RED_RIGHT_GOAL, RED_RIGHT_LOADER},
};

impl Robot {
    pub async fn rushcontrol(&mut self) {
        let mut linear = Linear::new(LINEAR_PID, 0.5);
        let mut turn = Turn::new(ANGULAR_PID, Angle::from_degrees(1.0));

        let mut swing = Swing::new(Pid::new(1000.0, 150.0, 0.0, 90.0), Angle::from_degrees(1.0));
        let mut move_to = MoveTo::new(
            Pid::new(30.0, 2.0, 6.0, 12.0),
            Pid::new(20.0, 0.0, 0.0, 0.0),
            0.5,
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(95.945, 20.926, Angle::ZERO));

        self.settings.borrow_mut().enable_sort = DoorCommands::Open;

        // Drive to the match loader
        linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, 23.0)
            .await;

        self.settings.borrow_mut().enable_sort = DoorCommands::Close;

        turn.timeout(Duration::from_millis(1000))
            .settle_velocity(10.0_f64.to_radians())
            .turn_to_point(dt, RED_RIGHT_LOADER, false)
            .await;

        // Grab balls from match loader
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        _ = self.match_loader.set_high();

        linear
            .timeout(Duration::from_millis(7500))
            .speed(0.3)
            .drive_distance(dt, 13.0)
            .await;

        // Drive to goal
        linear.drive_distance(dt, -7.5).await;
        self.intake.set_voltage(0.0);
        _ = self.match_loader.set_low();
        sleep(Duration::from_millis(500)).await; // wait for balls to settle in robot
        _ = self.lift.set_high();
        _ = self.wing.set_high();
        self.settings.borrow_mut().enable_sort = DoorCommands::Off;
        turn.settle_velocity(10.0_f64.to_radians())
            .timeout(Duration::from_millis(1000))
            // .turn_to(dt, Angle::new::<degree>(90.0))
            .turn_to_point(dt, RED_RIGHT_GOAL, false)
            .await;

        // Score on goal
        _ = self.duck_bill.set_high();
        zip(
            async {
                move_to
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
        linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, -5.0)
            .await;
        swing
            .timeout(Duration::from_millis(750))
            .swing_to(dt, Angle::ZERO, 5.0)
            .await;

        linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, -6.5)
            .await;
        turn.timeout(Duration::from_millis(1500))
            .settle_velocity(10.0_f64.to_radians())
            .turn_to(dt, Angle::from_degrees(90.0))
            .await;
        _ = self.wing.set_low();

        move_to
            .timeout(Duration::from_millis(1500))
            .move_to_point(dt, Vec2::new(108.0, 62.0))
            .await;

        _ = self.brake.set_high();
        sleep(Duration::from_millis(2500)).await;
    }
}
