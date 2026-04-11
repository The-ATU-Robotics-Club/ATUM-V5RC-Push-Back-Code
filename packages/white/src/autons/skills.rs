use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, turn::Turn},
    subsystems::intake::DoorCommands,
};
use futures_lite::future::zip;
use log::debug;
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
        self.intake.set_bottom(Motor::V5_MAX_VOLTAGE);
        sleep(Duration::from_millis(500)).await;

       _ = linear.speed(0.3).drive_distance(dt, -10.0).await;
        sleep(Duration::from_millis(300)).await;
       _ = linear.speed(0.3).drive_distance(dt, 5.0).await;
       _ = linear.speed(0.3).drive_distance(dt, -5.0).await;


       _ = turn.turn_to(dt, Angle::ZERO).await;

        _ = self.wing.set_low();

       _ = move_to.timeout(Duration::from_millis(2500)).move_to_point(dt, Vec2::new(130.5,23.0)).await;
        sleep(Duration::from_millis(500)).await;

       _ = move_to.speed(0.3).settle_velocity(5.0).move_to_point(dt, Vec2::new(87.0,22.0)).await;

        _ = turn.turn_to(dt, Angle::QUARTER_TURN).await;

        _ = move_to.speed(0.5).settle_velocity(5.0).move_to_point(dt, Vec2::new(91.0,64.0)).await;
        
        sleep(Duration::from_millis(1000)).await;
        self.intake.set_bottom(0.0);
        _ = move_to.speed(0.3).settle_velocity(5.0).tolerance(0.75).move_to_point(dt, Vec2::new(86.5,86.5)).await;
        
        _ = turn.tolerance(Angle::from_degrees(0.5)).turn_to(dt, Angle::from_degrees(-135.0)).await;

        _ = self.duck_bill.set_high();
        // _ = self.wing.set_high();
        _ = linear.speed(0.3).timeout(Duration::from_millis(2000)).drive_distance(dt, 6.5).await;

        self.intake.set_top(12.0);
        sleep(Duration::from_millis(1000)).await;
        self.intake.set_top(6.0);
        self.intake.set_bottom(12.0);

        sleep(Duration::from_millis(3000)).await;
        self.intake.set_top(3.0);
        sleep(Duration::from_millis(3000)).await;

        _ = move_to.speed(0.6).move_to_point(dt, Vec2::new(98.0,116.0)).await;
        _ = self.duck_bill.set_low();
        _ = turn.turn_to(dt, Angle::HALF_TURN).await;

        sleep(Duration::from_millis(300)).await;

        dt.set_pose(Pose::new(98.0, 116.0, dt.pose().h));
        sleep(Duration::from_millis(250)).await;    
        debug!("pose: {}", dt.pose());
        sleep(Duration::from_millis(300)).await;

        _ = turn.turn_to(dt, Angle::ZERO).await;
        
        _ = move_to.speed(0.75).move_to_point(dt, Vec2::new(131.5,118.0)).await;

        _ = move_to.speed(0.4).move_to_point(dt, Vec2::new(69.0, 110.0)).await;

        _ = turn.timeout(Duration::from_millis(750)).tolerance(Angle::from_degrees(1.0)).speed(0.2).turn_to(dt, Angle::QUARTER_TURN).await;

        _ = linear.timeout(Duration::from_millis(1500)).speed(0.2).drive_distance(dt, 6.5).await;

        _ = self.wing.set_high();

        sleep(Duration::from_millis(500)).await;

        _ = linear.speed(0.3).drive_distance(dt, -10.0).await;
        sleep(Duration::from_millis(300)).await;
       _ = linear.speed(0.3).drive_distance(dt, 5.0).await;
       _ = linear.speed(0.3).drive_distance(dt, -5.0).await;
        
       _ = linear.speed(0.3).drive_distance(dt, 5.0).await;
       _ = linear.speed(0.3).drive_distance(dt, -5.0).await;


        _ = turn.speed(0.3).turn_to_point(dt, Vec2::new(53.0,76.0), false).await;
         
        _ = move_to.move_to_point(dt, Vec2::new(60.0,76.0)).await;




    }
}
