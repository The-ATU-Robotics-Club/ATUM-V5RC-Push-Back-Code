use std::time::{Duration, Instant};

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, turn::Turn}, subsystems::intakes::lever::LeverStage, utils::wait_with_timeout,
};
use futures_lite::future::zip;
use log::{debug, info};
use vexide::{
    math::Angle,
    prelude::{Motor, sleep}, smart::motor::BrakeMode, time::{Sleep, sleep_until},
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID},
};

impl Robot {
    pub async fn sjtu(&mut self) {
        
        let start = Instant::now();
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 1.0,
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
            Pid::new(0.045, 0.0, 0.002, 12.0),
            Pid::new(0.08, 0.0, 0.005, 0.0),
            MotionParameters {
                tolerance: 1.0,
                speed: 0.75,
                timeout: Some(Duration::from_millis(2500)),
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;
        _ = self.lift.set_high();
        let mut target = Vec2::new(115.0, self.pose.borrow().y);
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.move_to_point(dt, target).await;
        _ = self.match_loader.set_high();
        // _ = turn.turn_to_point(dt, Vec2::new(115.0, 13.0), true).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;

        
        _ = move_to.min_velocity(Some(0.5)).speed(0.8).move_to_point(dt, Vec2::new(117.0 ,12.5)).await;
        dt.set_arcade(-0.6, 0.0);
        sleep(Duration::from_millis(400)).await;
        _ = move_to.speed(1.0).tolerance(1.0).min_velocity(Some(1.0)).move_to_point(dt, Vec2::new(117.5, 41.5)).await;
        _ = self.duck_bill.set_high();
        self.lever.score(LeverStage::Score(6.0, 6.0));
        // sleep(Duration::from_millis(300)).await;
        // wait_with_timeout(Duration::from_millis(700), || {
        //     !matches!(self.lever.stage(), LeverStage::Reset)
        // }).await;
        sleep(Duration::from_millis(1000)).await;

        self.lever.score(LeverStage::Reset);
        _ = move_to.min_velocity(Some(0.5)).speed(0.8).move_to_point(dt, Vec2::new(117.0 ,12.0)).await;
        _ = self.duck_bill.set_low();
        dt.set_arcade(-0.25, 0.0);
        sleep(Duration::from_millis(1000)).await;
        target = Vec2::new(self.pose.borrow().x, 20.0);
        _ = move_to.move_to_point(dt,target).await;
        _ = self.match_loader.set_low();
        _ = turn.turn_to(dt, Angle::from_degrees(135.0)).await;
        

        self.lever.set_intake(-Motor::V5_MAX_VOLTAGE);

        sleep(Duration::from_millis(1000)).await;

         _ = self.match_loader.set_high();
        // _ = turn.tolerance(Angle::from_degrees(1.0)).turn_to_point(dt, Vec2::new(117.0, 13.0), true).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.min_velocity(Some(0.5)).speed(0.7).move_to_point(dt, Vec2::new(117.0 ,12.5)).await;
        _ = self.duck_bill.set_low();

        dt.set_arcade(-0.25, 0.0);


        sleep(Duration::from_millis(1500)).await;
        target = Vec2::new(self.pose.borrow().x, 20.0);
        _ = move_to.move_to_point(dt, target).await;
        _ = turn.turn_to_point(dt, Vec2::new(80.0,56.0), true).await;
        _ = self.match_loader.set_low();
        _ = linear.speed(0.5).drive_to_point(dt, Vec2::new(82.0, 60.0), true).await;
        // _ = move_to.move_to_point(dt, Vec2::new(86.0, 55.0)).await;
        self.lever.set_intake(-Motor::V5_MAX_VOLTAGE/2.25);
        sleep(Duration::from_millis(1500)).await;
        _ = move_to.speed(0.5).timeout(Duration::from_millis(1500)).move_to_point(dt, Vec2::new(117.0,24.0)).await;
        _ = self.match_loader.set_high();
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.min_velocity(Some(0.5)).speed(0.7).move_to_point(dt, Vec2::new(117.0 ,12.5)).await;
        _ = self.duck_bill.set_low();

        dt.set_arcade(-0.25, 0.0);
        sleep(Duration::from_millis(1600)).await;

        _ = self.match_loader.set_low();

        _ = move_to.move_to_point(dt, Vec2::new(87.0, 50.0)).await;
        _ = turn.turn_to(dt, Angle::ZERO).await;
        target = Vec2::new(54.5, self.pose.borrow().y);
        _ = move_to.move_to_point(dt, target).await;
        _ = turn.turn_to(dt, Angle::from_degrees(45.0)).await;
        _ = self.lift.set_low();
        _ = linear.drive_distance(dt, 10.0).await;

        self.lever.score(LeverStage::Score(4.0, 2.5));

        _ = sleep(Duration::from_millis(3000)).await;
        _ = self.duck_bill.set_low();



        
    }
    pub async fn thorns(&mut self){
        let start = Instant::now();
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 1.0,
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
            Pid::new(0.045, 0.0, 0.002, 12.0),
            Pid::new(0.08, 0.0, 0.005, 0.0),
            MotionParameters {
                tolerance: 1.0,
                speed: 0.75,
                timeout: Some(Duration::from_millis(2500)),
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;
        _ = self.lift.set_high();
        let mut target = Vec2::new(115.0, self.pose.borrow().y);
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.move_to_point(dt, target).await;
        _ = self.match_loader.set_high();
        // _ = turn.turn_to_point(dt, Vec2::new(115.0, 13.0), true).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;

        
        _ = move_to.min_velocity(Some(0.5)).speed(0.8).move_to_point(dt, Vec2::new(117.0 ,12.5)).await;
        dt.set_arcade(-0.6, 0.0);
        sleep(Duration::from_millis(400)).await;
        _ = move_to.speed(1.0).tolerance(1.0).min_velocity(Some(1.0)).move_to_point(dt, Vec2::new(117.5, 41.5)).await;
        _ = self.duck_bill.set_high();
        self.lever.score(LeverStage::Score(6.0, 6.0));
        // sleep(Duration::from_millis(300)).await;
        // wait_with_timeout(Duration::from_millis(700), || {
        //     !matches!(self.lever.stage(), LeverStage::Reset)
        // }).await;
        sleep(Duration::from_millis(1000)).await;

        self.lever.score(LeverStage::Reset);
        _ = move_to.min_velocity(Some(0.5)).speed(0.8).move_to_point(dt, Vec2::new(117.0 ,12.0)).await;
        _ = self.duck_bill.set_low();
        dt.set_arcade(-0.25, 0.0);
        sleep(Duration::from_millis(1000)).await;
        target = Vec2::new(self.pose.borrow().x, 20.0);
        _ = move_to.move_to_point(dt,target).await;
        _ = self.match_loader.set_low();
        _ = turn.turn_to(dt, Angle::from_degrees(135.0)).await;
        

        self.lever.set_intake(-Motor::V5_MAX_VOLTAGE);

        sleep(Duration::from_millis(1000)).await;

         _ = self.match_loader.set_high();
        // _ = turn.tolerance(Angle::from_degrees(1.0)).turn_to_point(dt, Vec2::new(117.0, 13.0), true).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.min_velocity(Some(0.5)).speed(0.7).move_to_point(dt, Vec2::new(117.0 ,12.5)).await;
        _ = self.duck_bill.set_low();

        dt.set_arcade(-0.25, 0.0);


        sleep(Duration::from_millis(1500)).await;
        target = Vec2::new(self.pose.borrow().x, 20.0);
        _ = move_to.move_to_point(dt, target).await;
        _ = turn.turn_to_point(dt, Vec2::new(80.0,56.0), true).await;
        _ = self.match_loader.set_low();
        _ = linear.speed(0.5).drive_to_point(dt, Vec2::new(82.0, 60.0), true).await;
        // _ = move_to.move_to_point(dt, Vec2::new(86.0, 55.0)).await;
        self.lever.set_intake(-Motor::V5_MAX_VOLTAGE/2.25);
        sleep(Duration::from_millis(1500)).await;

        _ = move_to.speed(0.5).timeout(Duration::from_millis(1000)).move_to_point(dt, Vec2::new(106.0,30.0)).await;
        _ = turn.tolerance(Angle::from_degrees(2.0)).turn_to(dt, Angle::QUARTER_TURN).await;
        _ = self.wing.set_high();
        _ = move_to.speed(1.0).move_to_point(dt, Vec2::new(107.0,64.0)).await; 
        dt.brake(BrakeMode::Hold);
        _ = self.duck_bill.set_low();
        info!("Time: {}", start.elapsed().as_millis());
        _ = sleep_until(start+Duration::from_secs_f64(29.75)).await;
        _ = self.wing.set_low();

        
        
    }

    pub async fn benten(&mut self) {
        let start = Instant::now();
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 1.0,
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
                timeout: Some(Duration::from_millis(1100)),
                ..Default::default()
            },
        );

        let mut move_to = MoveTo::new(
            Pid::new(0.045, 0.0, 0.002, 12.0),
            Pid::new(0.08, 0.0, 0.005, 0.0),
            MotionParameters {
                tolerance: 1.0,
                speed: 0.75,
                timeout: Some(Duration::from_millis(1250)),
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;
        _ = self.lift.set_high();
        let mut target = Vec2::new(115.0, self.pose.borrow().y);
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.move_to_point(dt, target).await;
        _ = self.match_loader.set_high();
        // _ = turn.turn_to_point(dt, Vec2::new(115.0, 13.0), true).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;

        _ = move_to.min_velocity(Some(0.5)).speed(0.8).move_to_point(dt, Vec2::new(117.0 ,12.0)).await;
        dt.set_arcade(-0.6, 0.0);
        sleep(Duration::from_millis(400)).await;
        _ = move_to.speed(1.0).tolerance(1.0).min_velocity(Some(1.0)).move_to_point(dt, Vec2::new(117.5, 41.5)).await;
        _ = self.duck_bill.set_high();
        self.lever.score(LeverStage::Score(6.0, 6.0));
        // sleep(Duration::from_millis(300)).await;
        wait_with_timeout(Duration::from_millis(700), || {
            !matches!(self.lever.stage(), LeverStage::Reset)
        }).await;


        _ = move_to.min_velocity(Some(0.5)).speed(0.8).move_to_point(dt, Vec2::new(117.0 ,12.0)).await;
        _ = self.duck_bill.set_low();
        dt.set_arcade(-0.25, 0.0);
        sleep(Duration::from_millis(1000)).await;
        target = Vec2::new(self.pose.borrow().x, 20.0);
        _ = move_to.move_to_point(dt,target).await;
        _ = self.match_loader.set_low();
        _ = turn.turn_to(dt, Angle::from_degrees(135.0)).await;
        

        self.lever.set_intake(-Motor::V5_MAX_VOLTAGE);

        sleep(Duration::from_millis(1000)).await;


        _ = self.match_loader.set_high();
        _ = turn.tolerance(Angle::from_degrees(1.0)).turn_to_point(dt, Vec2::new(117.0, 13.0), true).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);

        _ = move_to.min_velocity(Some(0.5)).speed(0.7).move_to_point(dt, Vec2::new(117.5 ,13.0)).await;
        _ = self.duck_bill.set_low();

        dt.set_arcade(-0.25, 0.0);


        sleep(Duration::from_millis(2250)).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.timeout(Duration::from_millis(1000)).speed(1.0).tolerance(2.0).min_velocity(Some(1.0)).move_to_point(dt, Vec2::new(117.0, 42.0)).await;
        dt.set_arcade(2.0, 0.0);
        _ = self.duck_bill.set_high();
        self.lever.score(LeverStage::Score(6.0, 4.0));
        wait_with_timeout(Duration::from_millis(1000), || {
             !matches!(self.lever.stage(), LeverStage::Reset)
        }).await;
        _ = move_to.speed(0.4).move_to_point(dt, Vec2::new(109.0,33.0)).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;
        _ = self.wing.set_high();
        _ = move_to.speed(0.5).move_to_point(dt, Vec2::new(107.0,64.0)).await; 
        dt.brake(BrakeMode::Hold);
        _ = self.duck_bill.set_low();
        _ = self.match_loader.set_low();
        info!("Time: {}", start.elapsed().as_millis());

        _ = sleep_until(start+Duration::from_secs_f64(29.75)).await;
        _ = self.wing.set_low();


        
    }
    pub async fn candycrush(&mut self){
        let start = Instant::now();
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 1.0,
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
            Pid::new(0.045, 0.0, 0.002, 12.0),
            Pid::new(0.08, 0.0, 0.005, 0.0),
            MotionParameters {
                tolerance: 1.0,
                speed: 0.75,
                timeout: Some(Duration::from_millis(1500)),
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;
        _ = self.lift.set_high();
        let mut target = Vec2::new(115.0, self.pose.borrow().y);
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.speed(1.0).move_to_point(dt, target).await;
        _ = self.match_loader.set_high();
        // _ = turn.timeout(Duration::from_millis(750)).turn_to_point(dt, Vec2::new(115.0, 13.0), true).await;
        _ = turn.timeout(Duration::from_millis(750)).turn_to(dt, Angle::QUARTER_TURN).await;
        let target = Vec2::new(self.pose.borrow().x-0.75, 12.0);
        _ = move_to.min_velocity(Some(0.5)).speed(0.8).move_to_point(dt, target).await;
        dt.set_arcade(-0.5, 0.0);
        sleep(Duration::from_millis(400)).await;
        _ = move_to.speed(1.0).tolerance(1.0).min_velocity(Some(0.5)).move_to_point(dt, Vec2::new(116.0, 42.0)).await;
        _ = self.duck_bill.set_high();
        self.lever.score(LeverStage::Score(8.0, 8.0));
        // sleep(Duration::from_millis(300)).await;
        wait_with_timeout(Duration::from_millis(300), || {
            !matches!(self.lever.stage(), LeverStage::Reset)
        }).await;
        _ = move_to.speed(0.7).timeout(Duration::from_millis(750)).move_to_point(dt, Vec2::new(111.0,33.0)).await;
        _ = turn.tolerance(Angle::from_degrees(2.0)).turn_to(dt, Angle::QUARTER_TURN).await;
        _ = self.wing.set_high();
        _ = move_to.speed(1.2).move_to_point(dt, Vec2::new(107.0,64.0)).await; 
        dt.brake(BrakeMode::Hold);
        info!("Time: {}", start.elapsed().as_millis());

        _ = self.duck_bill.set_low();
        _ = self.match_loader.set_low();
        

        _ = sleep_until(start+Duration::from_secs_f64(29.75)).await;
        _ = self.wing.set_low();
        

    }

    pub async fn bmw(&mut self){
        let start = Instant::now();
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 1.0,
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
            Pid::new(0.045, 0.0, 0.002, 12.0),
            Pid::new(0.08, 0.0, 0.005, 0.0),
            MotionParameters {
                tolerance: 1.0,
                speed: 0.75,
                timeout: Some(Duration::from_millis(2500)),
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;
        _ = self.lift.set_high();
        let mut target = Vec2::new(115.0, self.pose.borrow().y);
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = linear.drive_to_point(dt, target,true).await;
        _ = self.match_loader.set_high();
        _ = turn.turn_to_point(dt, Vec2::new(115.0, 13.0), true).await;

        _ = move_to.min_velocity(Some(1.0)).timeout(Duration::from_millis(1000)).speed(0.7).move_to_point(dt, Vec2::new(117.0 ,12.5)).await;
        dt.set_arcade(-0.25, 0.0);
        sleep(Duration::from_millis(350)).await;
        target = Vec2::new(self.pose.borrow().x, 24.0);


        _ = move_to.move_to_point(dt, target).await;

        _ = turn.turn_to_point(dt, Vec2::new(80.0,56.0), true).await;
        _ = self.match_loader.set_low();


        _ = linear.speed(0.5).drive_to_point(dt, Vec2::new(86.0, 60.0), true).await;
        // _ = move_to.move_to_point(dt, Vec2::new(86.0, 55.0)).await;
        

        self.lever.set_intake(-Motor::V5_MAX_VOLTAGE/2.5);
        sleep(Duration::from_millis(1500)).await;
        _ = move_to.speed(1.0).tolerance(0.5).timeout(Duration::from_millis(1500)).move_to_point(dt, Vec2::new(119.0, 20.0)).await;
        _ = self.match_loader.set_high();
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;

        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        sleep(Duration::from_millis(100)).await;
        _ = move_to.min_velocity(Some(0.5)).speed(0.8).move_to_point(dt, Vec2::new(117.0 ,12.5)).await;
        dt.set_arcade(-0.25, 0.0);
        sleep(Duration::from_millis(1000)).await;

        _ = move_to.move_to_point(dt, Vec2::new(117.0, 24.0)).await;
        _ = self.match_loader.set_low();
        _ = turn.turn_to(dt, Angle::from_degrees(135.0)).await;
        

        self.lever.set_intake(-Motor::V5_MAX_VOLTAGE);

        sleep(Duration::from_millis(1000)).await;


        _ = self.match_loader.set_high();
        // _ = turn.speed(2.0).timeout(Duration::from_millis(1000)).turn_to_point(dt, Vec2::new(117.0, 13.0), true).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        

        _ = move_to.min_velocity(Some(0.5)).speed(0.7).move_to_point(dt, Vec2::new(116.0 ,12.0)).await;
        _ = self.duck_bill.set_low();

        dt.set_arcade(-0.25, 0.0);


        sleep(Duration::from_millis(2250)).await;
        _ = move_to.speed(1.0).tolerance(2.0).min_velocity(Some(0.5)).move_to_point(dt, Vec2::new(117.5, 42.5)).await;
        dt.set_arcade(2.0, 0.0);
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = self.duck_bill.set_high();
        self.lever.score(LeverStage::Score(8.0, 6.0));

        sleep(Duration::from_millis(1200)).await;  
        _ = move_to.speed(0.4).move_to_point(dt, Vec2::new(109.0,33.0)).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;
        _ = self.wing.set_high();
        _ = move_to.speed(1.0).move_to_point(dt, Vec2::new(107.0,64.0)).await; 
        dt.brake(BrakeMode::Hold);
        _ = self.duck_bill.set_low();
        _ = self.match_loader.set_low();
        info!("Time: {}", start.elapsed().as_millis());

        _ = sleep_until(start+Duration::from_secs_f64(29.75)).await;
        _ = self.wing.set_low(); 





    }



}
