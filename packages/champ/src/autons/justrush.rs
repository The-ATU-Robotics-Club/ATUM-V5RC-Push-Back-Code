use core::borrow;
use std::time::{Duration, Instant};

use atum::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, swing, turn::Turn},
    subsystems::intakes::lever::LeverStage,
};
use futures_lite::future::zip;
use log::{debug, info};
use vexide::{
    math::Angle,
    prelude::{sleep, Motor}, smart::motor::BrakeMode, time::sleep_until,
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID},
};

impl Robot {
    pub async fn justrush(&mut self) {
        let timer = Instant::now();

        let time = Instant::now();

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
        _ = move_to.timeout(Duration::from_millis(900)).move_to_point(dt, target).await;
        _ = self.lift.set_high();
        _ = self.match_loader.set_high();
        _ = turn.speed(0.8).timeout(Duration::from_millis(500)).turn_to(dt, Angle::from_degrees(-90.0)).await;
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        _ = linear.speed(0.9).timeout(Duration::from_millis(850)).drive_distance(dt, 13.0).await;

        // Score on long goal
        _ = move_to.speed(1.0).timeout(Duration::from_millis(1000)).move_to_point(dt, Vec2::new(23.5, 42.25)).await;
        _ = self.duck_bill.set_high();
        self.lever.score(LeverStage::Score(5.0, 6.0));
        sleep(Duration::from_millis(500)).await;

        // Wing
        _ = move_to.speed(0.5).move_to_point(dt, Vec2::new(33.5, 33.0)).await;
        _ = turn.tolerance(Angle::from_degrees(1.0)).speed(0.6).turn_to(dt, Angle::QUARTER_TURN).await;
        _ = self.wing.toggle();
        _ = self.match_loader.set_low();
        _ = move_to.speed(0.5).move_to_point(dt, Vec2::new(33.5, 64.0)).await;
        _ = linear.min_velocity(5.0).speed(0.7).drive_distance(dt, 8.0).await;
        dt.brake(BrakeMode::Hold);
        info!("JustRush: {}", timer.elapsed().as_millis());
        sleep_until(timer + Duration::from_secs_f64(29.5)).await;
        // dt.set_arcade(-12.0, 0.0);

    }
}
 
 