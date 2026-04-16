use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    motion::{MotionParameters, linear::Linear, move_to::{self, MoveTo}, turn::Turn},
    subsystems::intakes::lever::LeverStage,
};
use futures_lite::future::zip;
use vexide::{
    math::Angle,
    prelude::{Motor, sleep}, smart::motor::BrakeMode,
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID},
};

impl Robot {
    pub async fn skills2(&mut self){
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 1.0,
                velocity_tolerance: Some(2.5),
                timeout: Some(Duration::from_millis(2500)),
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
            Pid::new(0.045, 0.0, 0.002, 12.0),
            Pid::new(0.08, 0.0, 0.005, 0.0),
            MotionParameters {
                tolerance: 1.0,
                speed: 1.0,
                timeout: Some(Duration::from_millis(1300)),
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;

        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);

        _ = self.lift.set_high();


        _ = move_to.move_to_point(dt, Vec2::new(7.1, 23.5)).await;
        // _ = linear.speed(0.67).timeout(Duration::from_millis(1800)).drive_to_point(dt, target, true).await;
        _ = linear.speed(1.0).timeout(Duration::from_millis(500)).drive_distance(dt, 5.0).await;
        _ = linear.speed(1.0).timeout(Duration::from_millis(500)).drive_distance(dt, -5.0).await;

        
        let mut target = Vec2::new(23.5, self.pose.borrow().y);

        _ = linear.timeout(Duration::from_millis(1400)).speed(1.0).drive_to_point(dt, target, false).await;
        _ = turn.timeout(Duration::from_millis(1000)).turn_to_point(dt, Vec2::new(23.0,42.0), false).await;
        _ = self.match_loader.set_high();
        sleep(Duration::from_millis(300)).await;

        zip(
            async {
                _ = linear.timeout(Duration::from_millis(2300)).speed(0.6).drive_distance(dt, 27.0).await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(4.0, 3.0));
            },
        ).await;

        _ = self.duck_bill.set_low();
        _ = move_to.min_velocity(0.5).speed(0.4).move_to_point(dt, Vec2::new(23.0,11.5)).await;
        dt.brake(BrakeMode::Hold);
        sleep(Duration::from_millis(1500)).await;

        
        ////////////// Grabs first 6 balls ^^
        _ = linear.speed(1.0).drive_distance(dt, 24.0).await;
        self.lever.set_intake(0.0);
        
        _ = self.match_loader.set_low();
        _ = turn.turn_to(dt, Angle::ZERO).await;
        let mut target = Vec2::new(10.0, self.pose.borrow().y);
        _ = linear.drive_to_point(dt, target, true).await;
        _ = turn.turn_to(dt, -Angle::QUARTER_TURN).await;

        target = Vec2::new(self.pose.borrow().x, 105.0);
        _ = linear.timeout(Duration::from_millis(1800)).speed(1.0).drive_to_point(dt, target, true).await;
        _ = turn.turn_to(dt, Angle::ZERO).await;

        target = Vec2::new(23.0, self.pose.borrow().y);
        _ = linear.drive_to_point(dt, target, false).await;
        _ = self.match_loader.set_high();
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = turn.turn_to_point(dt, Vec2::new(23.0, 95.0), false).await;



        zip(
            async {
                _ = linear.timeout(Duration::from_millis(2000)).speed(0.6).drive_distance(dt, 25.0).await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(5.0, 4.0));
            },
        ).await;

        //////// Scores the first 6 ^^^
        _ = self.duck_bill.set_low();
        _ = move_to.speed(0.4).min_velocity(0.5).move_to_point(dt, Vec2::new(23.0,129.0)).await;
        dt.brake(BrakeMode::Hold);
        sleep(Duration::from_millis(1500)).await;


        zip(
            async {
                _ = move_to.timeout(Duration::from_millis(3000)).speed(0.5).move_to_point(dt, Vec2::new(23.5, 95.0)).await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 0.5 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(3.5, 2.1));
            },
        ).await;

        ////// Scores the second 6 ^^^^
        


        sleep(Duration::from_millis(300)).await;
        _ = linear.speed(1.0).drive_distance(dt, -18.0).await;
        _ = self.duck_bill.set_low();
        _ = self.match_loader.set_low();
        _ = turn.speed(1.0).turn_to(dt, Angle::ZERO).await;

        target = Vec2::new(8.0, self.pose.borrow().y);

        _ = linear.timeout(Duration::from_millis(1500)).drive_to_point(dt, target, true).await;
        _ = linear.speed(1.0).timeout(Duration::from_millis(500)).drive_distance(dt, 5.0).await;
        _ = linear.speed(1.0).timeout(Duration::from_millis(500)).drive_distance(dt, -8.0).await;

        target = Vec2::new(116.0, self.pose.borrow().y);

        _ = linear.timeout(Duration::from_millis(2300)).drive_to_point(dt, target, false).await;

        _ = turn.turn_to_point(dt, Vec2::new(117.0, 95.0), false).await;
        _ = self.match_loader.set_high();

        zip(
            async {
                _ = move_to.timeout(Duration::from_millis(1500)).speed(0.7).move_to_point(dt, Vec2::new(117.0, 95.0)).await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(4.0, 3.0));
            },
        ).await;
        _ = self.duck_bill.set_low();

        _ = move_to.min_velocity(0.25).speed(0.4).move_to_point(dt, Vec2::new(116.0, 129.0)).await;
        dt.brake(BrakeMode::Hold);
        sleep(Duration::from_millis(1500)).await;
        
    

        
       target = Vec2::new(self.pose.borrow().x, 103.0);

        _ = linear.speed(1.0).drive_to_point(dt, target, false).await;
        _ = self.match_loader.set_low();
        
        _ = turn.turn_to(dt, Angle::HALF_TURN).await;
        target = Vec2::new(131.0, self.pose.borrow().y);
        _ = linear.drive_to_point(dt, target, true).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;
        
        _ = linear.timeout(Duration::from_millis(1800)).speed(1.0).drive_distance(dt, -67.0).await;
        _ = turn.turn_to(dt, Angle::HALF_TURN).await;
        target = Vec2::new(117.0, self.pose.borrow().y);

        _ = linear.drive_to_point(dt, target, false).await;

        _ = turn.speed(1.0).turn_to_point(dt, Vec2::new(117.0, 48.0), false).await;

        _ = self.match_loader.set_high();
        zip(
            async {
                _ = move_to.move_to_point(dt, Vec2::new(117.0, 48.0)).await;

            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(3.7, 3.7));
            },
        ).await;
        _ = self.duck_bill.set_low();
        _ = move_to.min_velocity(0.5).speed(0.4).move_to_point(dt, Vec2::new(116.0,11.0)).await;
        sleep(Duration::from_millis(1750)).await;
         zip(
            async {
                _ = move_to.timeout(Duration::from_millis(3000)).move_to_point(dt, Vec2::new(117.0, 48.0)).await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(3.5, 2.1));
            },
        ).await;
        sleep(Duration::from_millis(500)).await;

        _ = linear.drive_distance(dt,-24.0).await;

        


        
    }
}
