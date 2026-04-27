use core::borrow;
use std::time::{Duration, Instant};

use atum::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, swing, turn::Turn},
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
    pub async fn rushthenmid(&mut self) {
        let timer = Instant::now();

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
                tolerance: Angle::from_degrees(1.0),
                velocity_tolerance: Some(10.0_f64.to_radians()),
                timeout: Some(Duration::from_millis(750)),
                ..Default::default()
            },
        );

        let mut move_to = MoveTo::new(
            Pid::new(0.045, 0.0, 0.002, 12.0),
            Pid::new(0.08, 0.0, 0.005, 0.0),
            MotionParameters {
                tolerance: 1.0,
                speed: 0.75,
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;

        // Drive to and collect matchload balls
        let mut target = Vec2::new(24.5, self.pose.borrow().y);
        _ = move_to.timeout(Duration::from_millis(1000)).move_to_point(dt, target).await;
        _ = self.lift.set_high();
        _ = self.match_loader.set_high();
        _ = turn.speed(0.6).timeout(Duration::from_millis(500)).turn_to(dt, Angle::from_degrees(-90.0)).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = linear.speed(0.8).timeout(Duration::from_millis(885)).drive_distance(dt, 12.0).await;

        // Score on long goal
        _ = move_to.timeout(Duration::from_millis(1000)).move_to_point(dt, Vec2::new(23.5, 42.25)).await;
        _ = self.duck_bill.set_high();
        self.lever.score(LeverStage::Score(5.0, 8.0));
        sleep(Duration::from_millis(500)).await;

        // Return to matchloader
        _ = linear.timeout(Duration::from_millis(500)).drive_to_point(dt, Vec2::new(24.0, 24.0), false).await;
        _ = self.duck_bill.set_low();
        _ = self.lift.set_high();
        _ = turn.speed(0.8).timeout(Duration::from_millis(500)).turn_to_point(dt, Vec2::new(24.5, 0.0), false).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = linear.speed(0.4).timeout(Duration::from_millis(1500)).drive_distance(dt, 23.0).await;

        // Spit opposing color balls into wall
        _ = linear.speed(1.0).drive_distance(dt, -12.0).await;
        _ = self.match_loader.set_low();
        _ = turn.tolerance(Angle::from_degrees(5.0)).speed(0.6).turn_to(dt, Angle::from_degrees(-135.0)).await;
        self.lever.set_intake(-Motor::V5_MAX_VOLTAGE);
        sleep(Duration::from_millis(750)).await;

        // Return to matchloader
        _ = self.match_loader.set_high();
        _ = turn.timeout(Duration::from_millis(500)).tolerance(Angle::from_degrees(5.0)).speed(1.0).turn_to_point(dt, Vec2::new(24.5, 0.0), false).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = linear.speed(0.4).timeout(Duration::from_millis(1500)).drive_distance(dt, 22.0).await;
        
        // Drive to mid goal
        _ = self.lift.set_low();
        _ = linear.speed(0.7).timeout(Duration::from_millis(1100)).drive_to_point(dt, Vec2::new(24.0, 24.0), true).await;
        _ = turn.turn_to(dt, Angle::from_degrees(225.0)).await;
        _ = move_to.speed(0.60).timeout(Duration::from_millis(2000)).move_to_point(dt, Vec2::new(57.0, 57.0)).await;

        // Score mid
        _ = self.duck_bill.set_high();
        self.lever.score(LeverStage::Score(2.0, 5.75));
        sleep(Duration::from_millis(1000)).await;

        // Wing
        _ = move_to.speed(0.35).move_to_point(dt, Vec2::new(32.5, 28.0)).await;
        _ = self.lift.set_high();
        _ = self.duck_bill.set_low();
        _ = turn.tolerance(Angle::from_degrees(1.0)).speed(0.6).turn_to(dt, Angle::from_degrees(90.0)).await;
        _ = self.wing.toggle();
        _ = move_to.speed(1.0).move_to_point(dt, Vec2::new(32.3, 64.0)).await;
        dt.brake(BrakeMode::Hold);

        sleep_until(timer + Duration::from_secs(29)).await;
        _ = self.wing.set_low();
    }
}
 