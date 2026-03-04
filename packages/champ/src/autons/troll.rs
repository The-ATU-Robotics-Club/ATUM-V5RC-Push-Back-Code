use core::borrow;
use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, swing, turn::Turn},
    subsystems::intakes::lever::LeverStage,
};
use futures_lite::future::zip;
use vexide::{
    math::Angle,
    prelude::{sleep, Motor}, smart::motor::BrakeMode,
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID},
};

impl Robot {
    pub async fn troll(&mut self) {
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
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;

        // Drive to and collect mid block
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.speed(0.7).timeout(Duration::from_millis(2500)).move_to_point(dt, Vec2::new(48.5, 72.0)).await;

        // Drive to and block lower goal
        _ = turn.turn_to(dt, Angle::from_degrees(60.0)).await;
        _ = move_to.speed(0.6).timeout(Duration::from_millis(2500)).move_to_point(dt, Vec2::new(54.0, 83.0)).await;
        _ = turn.timeout(Duration::from_millis(500)).turn_to_point(dt, Vec2::new(72.0, 72.0), false).await;
        _ = linear.timeout(Duration::from_millis(750)).drive_distance(dt, 6.0).await;
        _ = self.lift.set_high();
    
        // Wait.
        dt.brake(BrakeMode::Hold);
        self.lever.set_intake(-5.30);
        sleep(Duration::from_millis(15000)).await;
        dt.brake(BrakeMode::Brake);

        // Drive to and collect matchload balls
        _ = move_to.move_to_point(dt, Vec2::new(52.0, 83.0)).await;
        _ = turn.turn_to(dt, Angle::from_degrees(240.0)).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = move_to.speed(0.6).timeout(Duration::from_millis(2500)).move_to_point(dt, Vec2::new(26.5,24.5)).await;
        _ = self.lift.set_high();
        _ = self.match_loader.set_high();
        _ = turn.speed(0.6).timeout(Duration::from_millis(500)).turn_to(dt, Angle::from_degrees(-90.0)).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = linear.speed(0.9).timeout(Duration::from_millis(860)).drive_distance(dt, 16.0).await;

        // Score on long goal
        _ = move_to.speed(0.8).timeout(Duration::from_millis(1000)).move_to_point(dt, Vec2::new(23.0, 42.0)).await;
        _ = self.match_loader.set_low();
        _ = self.duck_bill.set_high();
        self.lever.score(LeverStage::Score(5.0, 8.0));
        sleep(Duration::from_millis(500)).await;

        // Wing
        _ = move_to.speed(0.3).move_to_point(dt, Vec2::new(34.5, 28.0)).await;
        _ = turn.tolerance(Angle::from_degrees(1.0)).speed(0.6).turn_to(dt, Angle::from_degrees(90.0)).await;
        _ = self.wing.toggle();
        _ = move_to.speed(0.9).move_to_point(dt, Vec2::new(32.5, 60.0)).await;
        dt.brake(BrakeMode::Hold);
        sleep(Duration::from_millis(30000)).await;
    }
}
 