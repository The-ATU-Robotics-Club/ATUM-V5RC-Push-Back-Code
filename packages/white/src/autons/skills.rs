use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, turn::Turn},
    subsystems::intake::DoorCommands,
};
use futures_lite::future::zip;
use vexide::{
    math::Angle,
    prelude::{Motor, sleep}, time::Sleep,
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID, RED_LEFT_GOAL, RED_LEFT_LOADER},
};

impl Robot {
    pub async fn skills(&mut self) {
        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 1.0,
                velocity_tolerance: Some(2.5),
                ..Default::default()
            },
        );

        let mut turn = Turn::new(
            ANGULAR_PID,
            MotionParameters {
                tolerance: Angle::from_degrees(1.0),
                velocity_tolerance: Some(10.0_f64.to_radians()),
                timeout: Some(Duration::from_millis(2000)),
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
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        sleep(Duration::from_millis(500)).await;

       _ = linear.speed(0.3).drive_distance(dt, -10.0).await;
        sleep(Duration::from_millis(300)).await;
       _ = linear.speed(0.3).drive_distance(dt, 5.0).await;
       _ = linear.speed(0.3).drive_distance(dt, -5.0).await;


       _ = turn.speed(0.3).turn_to(dt, Angle::ZERO).await;

        _ = self.wing.set_low();

       _ = move_to.timeout(Duration::from_millis(2500)).move_to_point(dt, Vec2::new(130.5,21.0)).await;
        sleep(Duration::from_millis(2500)).await;

    //    _ = move_to.speed(0.3).settle_velocity(5.0).move_to_point(dt, Vec2::new(92.0,22.0)).await;
    }
}
