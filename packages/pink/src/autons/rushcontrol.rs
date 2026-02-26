use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{linear::Linear, move_to::MoveTo, swing::Swing, turn::Turn},
};
use futures_lite::future::zip;
use uom::{
    ConstZero,
    si::{
        angle::degree,
        angular_velocity::{AngularVelocity, degree_per_second},
        f64::{Angle, Length, Velocity},
        length::inch,
        velocity::inch_per_second,
    },
};
use vexide::prelude::{Motor, sleep};

use crate::{
    Robot,
    autons::{
        ANGULAR_PID, LINEAR_PID, RED_RIGHT_GOAL, RED_RIGHT_LOADER, SETTLE_ANG_VEL, vec2_length,
    },
};

impl Robot {
    pub async fn rushcontrol(&mut self) {
        let mut linear = Linear::new(LINEAR_PID, Length::new::<inch>(0.5));
        let mut turn = Turn::new(ANGULAR_PID, Angle::new::<degree>(1.0));

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
        );
        let mut move_to = MoveTo::new(
            Pid::new(30.0, 2.0, 6.0, 12.0),
            Pid::new(20.0, 0.0, 0.0, 0.0),
            Length::new::<inch>(0.5),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(95.945),
            Length::new::<inch>(20.926),
            Angle::ZERO,
        ));

        // Drive to the match loader
        linear.drive_distance(dt, Length::new::<inch>(22.0)).await;

        turn.timeout(Duration::from_millis(1000))
            .settle_velocity(AngularVelocity::new::<degree_per_second>(10.0))
            // .turn_to(dt, Angle::new::<degree>(-90.0))
            .turn_to_point(dt, vec2_length(RED_RIGHT_LOADER))
            .await;

        // Grab balls from match loader
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        _ = self.match_loader.set_high();

        linear
            .timeout(Duration::from_millis(7500))
            .speed(0.3)
            // .drive_to_point(&mut self.drivetrain, vec2_length(RED_RIGHT_LOADER))
            .drive_distance(dt, Length::new::<inch>(12.0))
            .await;

        // Drive to goal
        linear.drive_distance(dt, Length::new::<inch>(-5.0)).await;
        self.intake.set_voltage(0.0);
        _ = self.match_loader.set_low();
        sleep(Duration::from_millis(500)).await; // wait for balls to settle in robot
        _ = self.lift.set_high();
        _ = self.wing.set_high();
        self.settings.borrow_mut().enable_sort = false;
        turn.settle_velocity(AngularVelocity::new::<degree_per_second>(10.0))
            .timeout(Duration::from_millis(1000))
            // .turn_to(dt, Angle::new::<degree>(90.0))
            .turn_to_point(dt, vec2_length(RED_RIGHT_GOAL))
            .await;

        // Score on goal
        _ = self.duck_bill.set_high();
        zip(
            async {
                move_to
                    .speed(0.25)
                    .timeout(Duration::from_millis(5000))
                    .move_to_point(dt, vec2_length(RED_RIGHT_GOAL))
                    .await;
            },
            async {
                // sleep so robot gains velocity from driving
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > Velocity::new::<inch_per_second>(1.0) {
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
        linear.timeout(Duration::from_millis(1000)).drive_distance(dt, Length::new::<inch>(-5.0)).await;
        swing
            .timeout(Duration::from_millis(750))
            .swing_to(dt, Angle::ZERO, Length::new::<inch>(5.0))
            .await;

        linear.drive_distance(dt, Length::new::<inch>(-6.5)).await;
        turn.timeout(Duration::from_millis(1500))
            .settle_velocity(AngularVelocity::new::<degree_per_second>(10.0))
            // .chain(5.0)
            .turn_to(dt, Angle::new::<degree>(90.0))
            .await;
        _ = self.wing.set_low();

        let target = Vec2::new(
            dt.pose().x,
            dt.pose().y + Length::new::<inch>(30.0),
        );
        move_to.move_to_point(dt, target).await;

        _ = self.brake.set_high();
        sleep(Duration::from_millis(2500)).await;
    }
}
