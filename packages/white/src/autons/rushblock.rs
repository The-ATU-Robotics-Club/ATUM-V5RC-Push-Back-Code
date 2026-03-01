use std::{cmp::Reverse, time::Duration, vec};

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{linear::Linear, move_to::MoveTo, swing::Swing, turn::Turn},
};
use futures_lite::future::zip;
use uom::{
    ConstZero,
    si::{
        angle::degree, angular_velocity::{AngularVelocity, degree_per_second}, f64::{Angle, Length}, length::inch
    },
};
use vexide::prelude::{Motor, sleep};

use crate::{Robot, autons::{ANGULAR_PID, LINEAR_PID, RED_LEFT_LOADER, vec2_length}};

impl Robot {
    pub async fn rushblock(&mut self){
        let mut linear = Linear::new(LINEAR_PID, Length::new::<inch>(1.0));

        let mut turn = Turn::new(ANGULAR_PID, Angle::new::<degree>(1.0));

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
        );
        let mut move_to = MoveTo::new(
            Pid::new(30.0, 2.0, 6.0, 12.0),
            Pid::new(20.0, 0.0, 0.0, 0.0),
            Length::new::<inch>(0.5),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(56.0),
            Length::new::<inch>(21.5),
            Angle::new::<degree>(-90.0),
        ));

        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        _ = self.lift.set_high();

        linear.drive_distance(dt, Length::new::<inch>(-56.0)).await;

        _ = self.brake.set_high();


        sleep(Duration::from_millis(5000)).await;

        // turn.chain(10.0).turn_to_point(dt, vec2_length(Vec2::new(24.0,24.0)), true).await;
        // move_to.timeout(Duration::from_millis(1500)).move_to_point(dt, vec2_length(Vec2::new(24.0,24.0))).await;
        // turn.turn_to(dt, Angle::new::<degree>(270.0)).await;

        // _ = self.match_loader.is_high();

        // linear.speed(0.3).timeout(Duration::from_millis(3000)).drive_distance(dt,Length::new::<inch>(15.0)).await;

        // linear.timeout(Duration::from_millis(750)).drive_distance(dt, Length::new::<inch>(-10.0)).await;
        // self.intake.set_voltage(0.0);
        // _ = self.match_loader.set_low();
        // sleep(Duration::from_millis(500)).await; // wait for balls to settle in robot
        // _ = self.lift.set_high();
        // _ = self.wing.set_high();
        // self.settings.borrow_mut().enable_sort = false;
    }
}