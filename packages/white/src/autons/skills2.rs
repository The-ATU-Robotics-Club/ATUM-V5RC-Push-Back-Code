use std::time::{Duration, Instant};

use atum::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, turn::Turn},
    subsystems::intakes::lever::LeverStage,
};
use futures_lite::future::zip;
use vexide::{
    math::Angle,
    prelude::{Motor, sleep}, smart::motor::BrakeMode, time::sleep_until,
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID},
};

impl Robot {
    pub async fn skills2(&mut self){
        let start = Instant::now();
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
        _ = linear.speed(1.0).timeout(Duration::from_millis(500)).drive_distance(dt, -10.0).await;

        
        let mut target = Vec2::new(23.5, self.pose.borrow().y);

        _ = linear.timeout(Duration::from_millis(1400)).speed(1.0).drive_to_point(dt, target, false).await;
        _ = turn.timeout(Duration::from_millis(750)).turn_to_point(dt, Vec2::new(23.0,42.0), false).await;
        _ = self.match_loader.set_high();
        sleep(Duration::from_millis(300)).await;

        zip(
            async {
                _ = linear.timeout(Duration::from_millis(1400)).speed(0.6).drive_distance(dt, 27.0).await;
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
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);

        _ = move_to.min_velocity(Some(0.5)).speed(0.45).move_to_point(dt, Vec2::new(23.0,12.75)).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);

        // dt.brake(BrakeMode::Hold);
        dt.set_arcade(-0.5, 0.0);
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        sleep(Duration::from_millis(2000)).await;
        
        ////////////// Grabs first 6 balls ^^
        _ = self.match_loader.set_low();
        _ = move_to.tolerance(15.0).speed(1.0).move_to_point(dt, Vec2::new(7.5, 54.5)).await;
        self.lever.set_intake(0.0);
        _ = move_to.speed(0.8).timeout(Duration::from_millis(2500)).tolerance(7.5).move_to_point(dt, Vec2::new(12.0, 95.0)).await;
        _ = move_to.timeout(Duration::from_millis(1500)).speed(0.4).tolerance(5.0).move_to_point(dt, Vec2::new(23.5, 110.0)).await;
        _ = turn.turn_to_point(dt, Vec2::new(23.5, 95.0), false).await;
        _ = self.match_loader.set_high();
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        zip(
            async {
                _ = move_to.tolerance(1.0).min_velocity(None).timeout(Duration::from_millis(2000)).speed(0.5).move_to_point(dt, Vec2::new(22.75, 95.0)).await;
                dt.set_arcade(2.0, 0.0);
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 0.5 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(4.0, 4.0));
            },
        ).await;



        _ = self.duck_bill.set_low();
        _ = move_to.speed(0.5).min_velocity(Some(0.25)).move_to_point(dt, Vec2::new(23.5,127.5)).await;
        // dt.brake(BrakeMode::Hold);
        dt.set_arcade(-0.35, 0.0);
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        sleep(Duration::from_millis(1500)).await;


        zip(
            async {
                _ = move_to.min_velocity(None).timeout(Duration::from_millis(2500)).speed(0.4).move_to_point(dt, Vec2::new(22.25, 94.0)).await;
                dt.set_arcade(2.0, 0.0);
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 0.5 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(3.5, 2.1));
                sleep(Duration::from_millis(100)).await;
            },
        ).await;
        _ = linear.speed(1.0).drive_distance(dt, -18.5).await;
        _ = self.duck_bill.set_low();
        _ = self.match_loader.set_low();
        _ = turn.speed(1.0).turn_to(dt, Angle::ZERO).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);

        target = Vec2::new(8.0, self.pose.borrow().y);

        _ = linear.timeout(Duration::from_millis(1500)).drive_to_point(dt, target, true).await;
        _ = linear.speed(1.0).timeout(Duration::from_millis(350)).drive_distance(dt, 5.0).await;
        _ = linear.speed(1.0).timeout(Duration::from_millis(400)).drive_distance(dt, -10.0).await;
        sleep_until(start + Duration::from_secs(22)).await;

        target = Vec2::new(116.0, self.pose.borrow().y);

        _ = linear.timeout(Duration::from_millis(2300)).drive_to_point(dt, target, false).await;

        _ = turn.turn_to_point(dt, Vec2::new(117.0, 95.0), false).await;
        _ = self.match_loader.set_high();

        zip(
            async {
                _ = move_to.min_velocity(None).timeout(Duration::from_millis(1250)).speed(0.7).move_to_point(dt, Vec2::new(117.5, 95.0)).await;
                dt.set_arcade(2.0, 0.0);

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
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.min_velocity(Some(0.25)).speed(0.4).move_to_point(dt, Vec2::new(116.0, 127.75)).await;
        // dt.brake(BrakeMode::Hold);
        dt.set_arcade(-0.35, 0.0);
    
        sleep(Duration::from_millis(1700)).await;
        _ = self.match_loader.set_low();

        _ = move_to.tolerance(5.0).speed(1.0).move_to_point(dt, Vec2::new(100.0, 80.0)).await;
        self.lever.set_intake(0.0);
        _ = turn.turn_to_point(dt, Vec2::new(90.0, 56.0), false).await;
        _ = move_to.tolerance(1.0).speed(0.6).move_to_point(dt, Vec2::new(90.0, 56.0)).await;
        _ = turn.turn_to(dt, Angle::from_degrees(-45.0)).await;

        _ = linear.drive_distance(dt, -6.75).await;
        dt.brake(BrakeMode::Hold);
        sleep(Duration::from_millis(8000)).await;


        /*_ = move_to.tolerance(15.0).speed(1.0).move_to_point(dt, Vec2::new(132.5, 86.0)).await;
        _ = move_to.speed(0.8).timeout(Duration::from_millis(2500)).tolerance(10.0).move_to_point(dt, Vec2::new(126.0, 45.0)).await; */
        _ = move_to.timeout(Duration::from_millis(1500)).speed(0.8).tolerance(1.0).move_to_point(dt, Vec2::new(118.0, 24.0)).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);

        _ = turn.speed(1.0).turn_to_point(dt, Vec2::new(117.0, 48.0), false).await;

        _ = self.match_loader.set_high();
        zip(
            async {
                _ = move_to.min_velocity(None).timeout(Duration::from_millis(2000)).speed(0.5).move_to_point(dt, Vec2::new(117.5, 48.0)).await;
                dt.set_arcade(2.0, 0.0);

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
        _ = self.duck_bill.set_low();
        _ = move_to.min_velocity(Some(0.5)).speed(0.4).move_to_point(dt, Vec2::new(117.5 ,12.75)).await;
        dt.set_arcade(-0.35, 0.0);
        sleep(Duration::from_millis(1750)).await;

        zip(
            async {
                _ = move_to.min_velocity(None).speed(0.4).timeout(Duration::from_millis(3000)).move_to_point(dt, Vec2::new(117.5 , 48.0)).await;
                dt.set_arcade(2.0, 0.0);
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(3.5, 2.1));
                sleep(Duration::from_millis(100)).await;
            },
        ).await;
        _ = self.match_loader.set_low();
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        dt.set_arcade(-4.0, 0.0);
        sleep(Duration::from_millis(250)).await;

        dt.set_arcade(-4.0, 0.0);
        sleep(Duration::from_millis(500)).await;

        _ = move_to.speed(0.6).move_to_point(dt, Vec2::new(100.0, 10.0)).await;
        self.lever.set_intake(0.0);
        _ = turn.turn_to(dt, Angle::from_degrees(7.5)).await;

        dt.set_arcade(-0.65, 0.0);
        sleep(Duration::from_millis(800)).await;
        dt.set_arcade(-0.3, 0.0);
        sleep(Duration::from_millis(500)).await;
        dt.brake(BrakeMode::Hold);

    /*  if (Jhon == fat){
            skills.perfect_run().await;
        else{
        
            skills.fail().await;
        
        } */

        
    }
}
