use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{linear::Linear, move_to::MoveTo, turn::Turn, MotionParameters}, subsystems::intakes::lever::LeverStage,
};
use futures_lite::future::zip;
use log::debug;
use vexide::{
    math::Angle,
    prelude::{sleep, Motor}, smart::motor::BrakeMode,
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID},
};

impl Robot {
    pub async fn shhhhhh(&mut self) {
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 0.5,
                velocity_tolerance: Some(2.5),
                timeout: Some(Duration::from_millis(1500)),
                ..Default::default()
            },
        );

        let mut turn = Turn::new(
            ANGULAR_PID,
            MotionParameters {
                tolerance: Angle::from_degrees(0.75),
                velocity_tolerance: Some(10.0_f64.to_radians()),
                timeout: Some(Duration::from_millis(1000)),
                ..Default::default()
            },
        );

        let mut move_to = MoveTo::new(
            Pid::new(25.0/12.0, 0.0/12.0, 3.0/12.0, 12.0),
            Pid::new(20.0/12.0, 0.0, 0.0, 0.0),
            MotionParameters {
                tolerance: 1.0,
                speed: 0.75,
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;
        
        _ = self.match_loader.set_high();
        let target = Vec2::new(116.5, self.pose.borrow().y);
        _ = linear.speed(0.6).drive_to_point(dt, target, true).await;
        _ = self.lift.set_high();

        _ = turn.speed(0.6).turn_to(dt, Angle::from_degrees(90.0)).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);

        _ = linear.timeout(Duration::from_millis(1100)).speed(0.3).drive_distance(dt, -12.0).await;
        // _ = turn.speed(1.25).turn_to(dt, Angle::from_degrees(90.0)).await;

        zip(
            async {
                _ = linear.timeout(Duration::from_millis(3000)).speed(0.6).drive_distance(dt, 32.0).await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(6.0, 12.0));
            },
        ).await;

        _ = self.match_loader.set_low();
        _ = turn.speed(0.575).turn_to(dt, Angle::from_degrees(-90.0)).await;
        _ = self.wing.set_high();
        _ = linear.speed(1.0).drive_distance(dt, -28.0).await;

        self.drivetrain.brake(BrakeMode::Hold);

        // delay autonomous
        sleep(Duration::from_secs(4)).await;
    }

    pub async fn shhhhhh_pt2(&mut self) {
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 0.5,
                velocity_tolerance: Some(2.5),
                timeout: Some(Duration::from_millis(1500)),
                ..Default::default()
            },
        );

        let mut turn = Turn::new(
            ANGULAR_PID,
            MotionParameters {
                tolerance: Angle::from_degrees(0.75),
                velocity_tolerance: Some(10.0_f64.to_radians()),
                timeout: Some(Duration::from_millis(1000)),
                ..Default::default()
            },
        );

        let mut move_to = MoveTo::new(
            Pid::new(25.0/12.0, 0.0/12.0, 3.0/12.0, 12.0),
            Pid::new(20.0/12.0, 0.0, 0.0, 0.0),
            MotionParameters {
                tolerance: 1.0,
                speed: 0.75,
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;
        
        _ = self.match_loader.set_high();
        let target = Vec2::new(140.0 - 116.5, self.pose.borrow().y);
        _ = linear.speed(0.6).drive_to_point(dt, target, true).await;
        _ = self.lift.set_high();

        _ = turn.speed(0.6).turn_to(dt, Angle::from_degrees(90.0)).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);

        _ = linear.timeout(Duration::from_millis(1100)).speed(0.4).drive_distance(dt, -12.0).await;
        // _ = turn.speed(1.25).turn_to(dt, Angle::from_degrees(90.0)).await;

        zip(
            async {
                _ = linear.timeout(Duration::from_millis(3000)).speed(0.6).drive_distance(dt, 32.0).await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(6.0, 12.0));
            },
        ).await;

        _ = self.match_loader.set_low();
        _ = turn.speed(0.575).turn_to(dt, Angle::from_degrees(-90.0)).await;
        _ = self.wing.set_high();
        _ = linear.speed(1.0).drive_distance(dt, -28.0).await;

        self.drivetrain.brake(BrakeMode::Hold);

        // delay autonomous
        sleep(Duration::from_secs(4)).await;
    }
}
