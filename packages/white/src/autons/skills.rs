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
    pub async fn skills(&mut self){
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 0.5,
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

        _ = linear.speed(0.75).drive_to_point(dt, Vec2::new(23.5, 22.0), true).await; 
        _ = self.match_loader.set_high();

        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;
        sleep(Duration::from_millis(500)).await;

        _ = linear.drive_distance(dt, -13.0).await;
        sleep(Duration::from_millis(1000)).await;

        ////////////// Grabs first 6 balls ^^
        
        _ = linear.drive_distance(dt, 24.0).await;
        self.lever.set_intake(0.0);
        
        _ = self.match_loader.set_low();
        _ = turn.turn_to(dt, Angle::ZERO).await;

        _ = linear.drive_distance(dt, -9.0).await;
        _ = turn.turn_to(dt, -Angle::QUARTER_TURN).await;
        _ = linear.drive_distance(dt, -65.0).await;
        _ = turn.turn_to(dt, Angle::ZERO).await;

        let target = Vec2::new(23.0, self.pose.borrow().y);
        _ = linear.drive_to_point(dt, target, false).await;

        _ = turn.turn_to(dt, -Angle::QUARTER_TURN).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);


        zip(
            async {
                _ = linear.timeout(Duration::from_millis(3000)).speed(0.6).drive_distance(dt, 25.0).await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(3.0, 3.0));
            },
        ).await;
        sleep(Duration::from_millis(1000)).await;

        //////// Scores the first 6 ^^^
        
        _ = self.match_loader.set_high();
        sleep(Duration::from_millis(300)).await;
        _ = linear.drive_distance(dt, -10.0).await;
        _ = self.duck_bill.set_low();

        _ = turn.speed(0.2).turn_to(dt, Angle::from_degrees(-95.0)).await;

        _ = linear.speed(0.35).drive_distance(dt, -24.0).await;
        sleep(Duration::from_millis(1000)).await;
        _ = linear.speed(0.6).drive_distance(dt,7.0).await;
        _ = turn.turn_to(dt, Angle::from_degrees(-87.0)).await;

        zip(
            async {
                _ = linear.timeout(Duration::from_millis(3000)).speed(0.6).drive_distance(dt, 25.0).await;
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








    }
}
