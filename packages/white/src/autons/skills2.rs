use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, turn::Turn},
    subsystems::intakes::lever::LeverStage,
};
use futures_lite::future::zip;
use vexide::{
    math::Angle,
    prelude::{sleep, Motor},
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
            Pid::new(25.0/12.0, 0.0/12.0, 3.0/12.0, 12.0),
            Pid::new(20.0/12.0, 0.0, 0.0, 0.0),
            MotionParameters {
                tolerance: 1.0,
                speed: 0.75,
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;

        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);

        _ = self.lift.set_high();

        let mut target = Vec2::new(7.1, self.pose.borrow().y);

        _ = linear.speed(0.67).timeout(Duration::from_millis(1800)).drive_to_point(dt, target, true).await;
        _ = linear.speed(1.0).timeout(Duration::from_millis(500)).drive_distance(dt, 5.0).await;
        _ = linear.speed(1.0).timeout(Duration::from_millis(500)).drive_distance(dt, -5.0).await;

        
        target = Vec2::new(23.5, self.pose.borrow().y);

        _ = linear.timeout(Duration::from_millis(2500)).speed(0.8).drive_to_point(dt, target, false).await;
        _ = turn.timeout(Duration::from_millis(1000)).turn_to(dt,Angle::QUARTER_TURN).await;
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
                self.lever.score(LeverStage::Score(5.0, 5.0));
            },
        ).await;

        _ = linear.speed(1.0).drive_to_point(dt, Vec2::new(23.5, 20.0), true).await; 
        
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;
        _ = self.duck_bill.set_low();
        _ = linear.speed(0.4).drive_distance(dt, -15.0).await;
        
        ////////////// Grabs first 6 balls ^^
        
        _ = linear.speed(1.0).drive_distance(dt, 24.0).await;
        self.lever.set_intake(0.0);
        
        _ = self.match_loader.set_low();
        _ = turn.turn_to(dt, Angle::ZERO).await;
        let mut target = Vec2::new(10.0, self.pose.borrow().y);
        _ = linear.drive_to_point(dt, target, true).await;
        _ = turn.turn_to(dt, -Angle::QUARTER_TURN).await;
        _ = linear.timeout(Duration::from_millis(1800)).speed(1.0).drive_distance(dt, -64.0).await;
        _ = turn.turn_to(dt, Angle::ZERO).await;

        target = Vec2::new(23.0, self.pose.borrow().y);
        _ = linear.drive_to_point(dt, target, false).await;
        _ = self.match_loader.set_high();
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = turn.turn_to(dt, -Angle::QUARTER_TURN).await;


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
                self.lever.score(LeverStage::Score(4.0, 3.0));
            },
        ).await;

        //////// Scores the first 6 ^^^
        
        _ = linear.drive_distance(dt, -10.0).await;
        _ = self.duck_bill.set_low();

        _ = turn.speed(2.4).turn_to(dt, Angle::from_degrees(-95.0)).await;

        _ = linear.speed(0.35).drive_distance(dt, -35.0).await;
        sleep(Duration::from_millis(600)).await;
        _ = linear.speed(1.0).drive_distance(dt,7.0).await;
        _ = turn.turn_to(dt, Angle::from_degrees(-87.0)).await;

        zip(
            async {
                _ = linear.timeout(Duration::from_millis(3000)).speed(0.5).drive_distance(dt, 28.0).await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(3.5, 2.0));
            },
        ).await;
        ////// Scores the second 6 ^^^^
        sleep(Duration::from_millis(750)).await;
        _ = linear.speed(1.0).drive_distance(dt, -18.0).await;
        _ = self.duck_bill.set_low();
        _ = self.match_loader.set_low();
        _ = turn.speed(1.0).turn_to(dt, Angle::ZERO).await;

        target = Vec2::new(8.0, self.pose.borrow().y);

        _ = linear.timeout(Duration::from_millis(1500)).drive_to_point(dt, target, true).await;
        _ = linear.speed(1.0).timeout(Duration::from_millis(500)).drive_distance(dt, 5.0).await;
        _ = linear.speed(1.0).timeout(Duration::from_millis(500)).drive_distance(dt, -8.0).await;

        target = Vec2::new(116.5, self.pose.borrow().y);

        _ = linear.timeout(Duration::from_millis(2000)).drive_to_point(dt, target, false).await;

        _ = turn.turn_to(dt, -Angle::QUARTER_TURN).await;
        _ = self.match_loader.set_high();

        zip(
            async {
                _ = linear.timeout(Duration::from_millis(1500)).speed(0.6).drive_distance(dt, 28.0).await;
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

        target = Vec2::new(self.pose.borrow().x, 120.0);
        _ = self.duck_bill.set_low();
        _ = linear.drive_to_point(dt, target, true).await;
        _ = linear.speed(0.4).drive_distance(dt, -15.0).await;
        sleep(Duration::from_millis(500)).await;
        
        target = Vec2::new(self.pose.borrow().x, 103.0);

        _ = linear.speed(1.0).drive_to_point(dt, target, false).await;
        _ = self.match_loader.set_low();
        
        _ = turn.turn_to(dt, Angle::HALF_TURN).await;
        target = Vec2::new(131.0, self.pose.borrow().y);
        _ = linear.drive_to_point(dt, target, true).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;
        
        _ = linear.timeout(Duration::from_millis(1800)).speed(1.0).drive_distance(dt, -65.0).await;
        _ = turn.turn_to(dt, Angle::HALF_TURN).await;
        target = Vec2::new(117.0, self.pose.borrow().y);

        _ = linear.drive_to_point(dt, target, false).await;

        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;

        _ = self.match_loader.set_high();
        zip(
            async {
                _ = linear.timeout(Duration::from_millis(1500)).speed(0.5).drive_distance(dt, 28.0).await;
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

        target = Vec2::new(self.pose.borrow().x, 20.0);
        _ = self.duck_bill.set_low();

        _ = linear.drive_to_point(dt, target, true).await;

        _ = linear.speed(0.4).drive_distance(dt, -15.0).await;
        sleep(Duration::from_millis(600)).await;
        _ = linear.drive_to_point(dt, target, false).await;

        _ = turn.speed(2.0).turn_to_point(dt, Vec2::new(116.0, 48.0), false).await;
        
        zip(
            async {
                _ = linear.timeout(Duration::from_millis(2200)).speed(0.5).drive_distance(dt, 28.0).await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(3.5, 2.0));
            },
        ).await;

        _ = linear.drive_distance(dt, -20.0).await;
        _ = self.duck_bill.set_low();



        






    }
}
