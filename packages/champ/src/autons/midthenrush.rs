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
    prelude::{sleep, Motor}, smart::motor::BrakeMode,
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID},
};

impl Robot {
    pub async fn rush(&mut self) {
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
        _ = self.lift.set_high();
        self.lever.set_intake(Motor::V5_MAX_VOLTAGE);
        let mut target = Vec2::new(35.5, self.pose.borrow().y);
        _ = linear.settle_velocity(7.5).timeout(Duration::from_millis(750)).speed(2.0).drive_to_point(dt, target, true).await;
        _ = self.match_loader.set_high();
        _ = turn.speed(2.0).min_velocity(1.0_f64.to_radians()).tolerance(Angle::from_degrees(3.0)).timeout(Duration::from_millis(600)).turn_to_point(dt, Vec2::new(23.5,12.0), true).await;
        _ = move_to.timeout(Duration::from_millis(975)).min_velocity(Some(0.5)).speed(1.0).move_to_point(dt, Vec2::new(23.5,11.0)).await;

        zip(
            async {
                _ = move_to.speed(3.0).move_to_point(dt, Vec2::new(23.0,46.0)).await;
                dt.set_arcade(2.0, 0.0);
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > 1.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.duck_bill.set_high();
                self.lever.score(LeverStage::Score(6.0, 6.0));
            },
        ).await;
        _ = move_to.speed(5.0).timeout(Duration::from_millis(650)).move_to_point(dt, Vec2::new(12.0, 35.0)).await;
        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;








        
        
    }
}
