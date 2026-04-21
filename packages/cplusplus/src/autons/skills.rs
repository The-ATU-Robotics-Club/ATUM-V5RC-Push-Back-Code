use std::time::{Duration, Instant};

use atum::{
    controllers::pid::Pid,
    localization::vec2::Vec2,
    motion::{MotionParameters, linear::Linear, move_to::MoveTo, turn::Turn},
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
    pub async fn skills(&mut self) {
        let start = Instant::now();

        let mut linear = Linear::new(
            LINEAR_PID,
            MotionParameters {
                tolerance: 1.0,
                velocity_tolerance: Some(2.5),
                timeout: Some(Duration::from_millis(1000)),
                speed: 0.75,
                ..Default::default()
            },
        );

        let mut turn = Turn::new(
            ANGULAR_PID,
            MotionParameters {
                tolerance: Angle::from_degrees(2.5),
                timeout: Some(Duration::from_millis(750)),
                ..Default::default()
            },
        );

        let mut move_to = MoveTo::new(
            Pid::new(0.04, 0.0, 0.0025, 12.0),
            Pid::new(0.03, 0.0, 0.003, 0.0),
            MotionParameters {
                tolerance: 1.0,
                velocity_tolerance: Some(2.5),
                timeout: Some(Duration::from_millis(2000)),
                ..Default::default()
            },
        );

        let dt = &mut self.drivetrain;

        self.intake.set_bottom(Motor::V5_MAX_VOLTAGE);

        // grab balls from park zone
        _ = linear.drive_distance(dt, -10.0).await;

        let target = Vec2::new(130.0, 24.0);
        zip(
            async {
                _ = move_to.tolerance(1.0).move_to_point(dt, target).await;
            },
            async {
                // wait until the robot is 32 inches to lift the rake
                while (target - self.pose.borrow().position()).length() > 32.0 {
                    sleep(Duration::from_millis(10)).await;
                }

                _ = self.rake.set_low();
            }
        ).await;
        dt.set_arcade(8.0, 0.0);
        sleep(Duration::from_millis(250)).await;
        _ = move_to.move_to_point(dt, Vec2::new(117.0, 23.0)).await;
        self.intake.set_bottom(0.0);

        // align to the long goal
        _ = self.lift.set_high();
        _ = turn.tolerance(Angle::from_degrees(5.0)).turn_to_point(dt, Vec2::new(118.0, 40.0), false).await;
        _ = move_to.timeout(Duration::from_millis(1000)).move_to_point(dt, Vec2::new(118.0, 40.0)).await;
        dt.set_arcade(3.0, 0.0);

        // score one red block
        _ = self.duck_bill.set_high();
        self.intake.set_top(12.0);
        sleep(Duration::from_millis(250)).await;
        self.intake.set_voltage(0.0);
        sleep(Duration::from_millis(125)).await;
        _ = self.duck_bill.set_low();

        // move away while reseting pnuematic postion
        _ = self.rake.set_high();
        _ = self.lift.set_low();
        _ = move_to.timeout(Duration::from_millis(2000)).move_to_point(dt, Vec2::new(85.0, 26.0)).await;
        _ = self.rake.set_low();

        // collect block in the middle
        _ = turn.speed(1.5).tolerance(Angle::from_degrees(10.0)).turn_to(dt, Angle::from_degrees(75.0)).await;
        sleep(Duration::from_millis(90)).await;
        self.intake.set_bottom(12.0);
        _ = move_to.timeout(Duration::from_millis(1500)).move_to_point(dt, Vec2::new(91.0, 66.0)).await;
        _ = turn.speed(1.0).tolerance(Angle::from_degrees(1.0)).turn_to(dt, Angle::from_degrees(-95.0)).await;
        // _ = self.lift.set_low();

        // align to the upper goal
        _ = move_to.timeout(Duration::from_millis(2000)).move_to_point(dt, Vec2::new(85.5, 86.0)).await;
        self.intake.set_bottom(0.0);

        _ = turn.turn_to(dt, Angle::from_degrees(-135.0)).await;
        _ = self.duck_bill.set_high();
        _ = linear.min_velocity(1.0).drive_distance(dt, 7.0).await;
        sleep(Duration::from_millis(50)).await;
        dt.brake(BrakeMode::Hold);

        // score blocks in the upper goal
        _ = self.intake.set_bottom(-12.0);
        sleep(Duration::from_millis(250)).await;
        self.intake.set_bottom(12.0);
        self.intake.set_top(5.0);
        sleep(Duration::from_millis(3000)).await;
        dt.set_arcade(0.2, 0.0);
        sleep(Duration::from_millis(250)).await;
        
        // collect wall balls
        _ = linear.speed(1.0).drive_distance(dt, -46.0).await;
        self.intake.set_top(0.0);
        _ = self.duck_bill.set_low();
        _ = self.lift.set_high();
        _ = turn.turn_to(dt, Angle::ZERO).await;
        _ = linear.drive_distance(dt, 14.0).await;
        dt.set_arcade(8.0, 0.0);
        sleep(Duration::from_millis(250)).await;

        // move towards the park to collect balls
        _ = move_to.min_velocity(None).move_to_point(dt, Vec2::new(71.5, 95.0)).await;
        _ = turn.timeout(Duration::from_millis(1000)).turn_to(dt, Angle::QUARTER_TURN).await;

        // wait until the robot has moved to the other side
        sleep_until(start + Duration::from_secs(23)).await;

        _ = linear.speed(0.75).drive_distance(dt, 22.5).await;
        _ = self.rake.set_high();
        _ = self.lift.set_low();
        sleep(Duration::from_millis(325)).await;

        _ = linear.drive_distance(dt, -7.5).await;
        _ = linear.speed(1.2).drive_distance(dt, 5.0).await;
        _ = linear.speed(1.0).drive_distance(dt, -5.0).await;

        // collect block in the middle
        sleep(Duration::from_millis(250)).await;
        _ = turn.turn_to(dt, Angle::from_degrees(-170.0)).await;
        _ = linear.drive_distance(dt, 7.5).await;
        _ = self.rake.set_low();
        let target = Vec2::new(54.5, 76.0);
        zip(
            async {
                _ = move_to.move_to_point(dt, target).await;
            },
            async {
                while (target - self.pose.borrow().position()).length() > 12.0 {
                    sleep(Duration::from_millis(10)).await;
                }
                _ = self.rake.set_low();
            }
        ).await;

        // score in the lower goal
        _ = turn.tolerance(Angle::from_degrees(5.0)).turn_to_point(dt, Vec2::new(52.0, 89.0), true).await;
        _ = self.lift.set_high();
        _ = move_to.speed(1.25).move_to_point(dt, Vec2::new(52.0, 89.0)).await;
        _ = turn.settle_velocity(10.0).tolerance(Angle::from_degrees(0.75)).turn_to(dt, Angle::from_degrees(-42.5)).await;
        _ = linear.drive_distance(dt, 9.75).await;
        dt.brake(BrakeMode::Hold);
        info!("score low goal: {}", start.elapsed().as_millis());
        self.intake.set_voltage(-12.0);
        sleep(Duration::from_millis(2750)).await;
        self.intake.set_voltage(0.0);
        sleep(Duration::from_millis(250)).await;
        _ = linear.speed(0.5).drive_distance(dt, -4.0).await;

        // park
        _ = linear.drive_distance(dt, -20.0).await;
        self.intake.set_bottom(12.0);
        _ = move_to.speed(1.0).move_to_point(dt, Vec2::new(36.0, 8.0)).await;
        _ = turn.turn_to(dt, Angle::from_degrees(-17.5)).await;
        info!("parking: {}", start.elapsed().as_millis());

        dt.set_arcade(0.45, 0.0);
        sleep(Duration::from_millis(500)).await;
        let mut scuff = Duration::ZERO;
        while dt.odometry.pitch() > Angle::from_degrees(0.0) && scuff < Duration::from_millis(900) {
            sleep(Duration::from_millis(10)).await;
            scuff += Duration::from_millis(10);
            debug!("pitch {}", dt.odometry.pitch().as_degrees());
        }

        info!("parked {}", scuff.as_millis());

        self.intake.set_bottom(0.0);
        dt.brake(BrakeMode::Hold);

        // check position of the robot after it parks
        sleep(Duration::from_secs(1)).await;
        debug!("pitch {}", dt.odometry.pitch().as_degrees());

        sleep_until(start + Duration::from_mins(1)).await;
    }
}
