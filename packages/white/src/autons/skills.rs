use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
    motion::{linear::Linear, move_to::MoveTo, swing::Swing, turn::Turn}, subsystems::intake::DoorCommands,
};
use futures_lite::future::zip;
use uom::{
    ConstZero,
    si::{
        angle::degree, angular_velocity::{AngularVelocity, degree_per_second}, f64::{Angle, Length, Velocity}, length::inch, velocity::inch_per_second
    },
};
use vexide::prelude::{Motor, sleep};

use crate::{Robot, autons::{ANGULAR_PID, BLUE_RIGHT_GOAL, BLUE_RIGHT_LOADER, LINEAR_PID, RED_LEFT_GOAL, RED_LEFT_LOADER, vec2_length}};

impl Robot {
    pub async fn skills(&mut self){
        
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
            Angle::new::<degree>(90.0),
        ));
        self.settings.borrow_mut().enable_sort = DoorCommands::Off;

        self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
        zip(
            async {
                linear.drive_distance(dt, Length::new::<inch>(28.0)).await;
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);

                move_to.speed(0.75)
                    .timeout(Duration::from_millis(1500))
                    .move_to_point(dt, vec2_length(Vec2::new(44.5,69.0)))
                .await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                _ = self.lift.set_high();
                sleep(Duration::from_millis(500)).await;
                _ = self.lift.set_low();
            }
        )
        .await;
        turn.chain(5.0).turn_to_point(dt, vec2_length(Vec2::new(55.0,53.0)), true).await;
        move_to.timeout(Duration::from_millis(1000)).move_to_point(dt, vec2_length(Vec2::new(55.0,52.0))).await;
        turn.timeout(Duration::from_millis(750)).turn_to(dt, Angle::new::<degree>(45.0)).await;
        self.intake.set_voltage(0.0); 
        _ = self.duck_bill.set_high();
        linear.timeout(Duration::from_millis(1000)).drive_distance(dt, Length::new::<inch>(6.0)).await;
        self.intake.set_voltage(7.0);
        sleep(Duration::from_millis(750)).await;
        linear.timeout(Duration::from_millis(2000)).drive_to_point(dt, vec2_length(Vec2::new(25.0, 25.0)),true).await;
        _ = self.duck_bill.set_low();
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        turn.timeout(Duration::from_millis(2000)).settle_velocity(AngularVelocity::new::<degree_per_second>(5.0)).turn_to_point(dt, vec2_length(RED_LEFT_LOADER), false).await;  
        _ = self.match_loader.set_high();
        linear.speed(0.3).timeout(Duration::from_millis(7500)).drive_distance(dt,Length::new::<inch>(20.0)).await;

        linear.timeout(Duration::from_millis(1000)).drive_distance(dt, Length::new::<inch>(-7.5)).await;
        self.intake.set_voltage(0.0);
        _ = self.match_loader.set_low();
        sleep(Duration::from_millis(500)).await; // wait for balls to settle in robot
        sleep(Duration::from_millis(10)).await;

        _ = self.lift.set_high();
        _ = self.wing.set_high();

        turn.settle_velocity(AngularVelocity::new::<degree_per_second>(10.0))
            .timeout(Duration::from_millis(1000))
            .turn_to_point(dt, vec2_length(RED_LEFT_GOAL), false)
            .await;

        // Score on goal
        _ = self.duck_bill.set_high();
        zip(
            async {
                move_to
                    .timeout(Duration::from_millis(5000))
                    .move_to_point(dt, vec2_length(RED_LEFT_GOAL))
                    .await;
            },
            async {
                sleep(Duration::from_millis(250)).await;
                while self.pose.borrow().vf > Velocity::new::<inch_per_second>(1.0) {
                    sleep(Duration::from_millis(10)).await;
                }
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
                sleep(Duration::from_millis(1500)).await;
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
                sleep(Duration::from_millis(300)).await;
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            }
        )
        .await;

        // Back up and shove balls into goal
        _ = self.duck_bill.set_low();
        move_to.timeout(Duration::from_millis(2500)).move_to_point(dt, vec2_length(Vec2::new(48.0,10.0))).await;


        turn.timeout(Duration::from_millis(1000)).turn_to(dt, Angle::ZERO).await;

        let target = Vec2::new(Length::new::<inch>(72.0), self.pose.borrow().y);

        linear.timeout(Duration::from_millis(2500)).drive_to_point(dt, target, false).await;



        // let mut linear = Linear::new(LINEAR_PID, Length::new::<inch>(0.5));

        // let mut turn = Turn::new(ANGULAR_PID, Angle::new::<degree>(1.0));

        // let mut swing = Swing::new(
        //     Pid::new(1000.0, 150.0, 0.0, 90.0),
        //     Angle::new::<degree>(1.0),
        // );

        //  let mut move_to = MoveTo::new(
        //     Pid::new(30.0, 2.0, 6.0, 12.0),
        //     Pid::new(20.0, 0.0, 0.0, 0.0),
        //     Length::new::<inch>(0.5),
        // );

        // let dt = &mut self.drivetrain;

        // dt.set_pose(Pose::new(
        //     Length::new::<inch>(56.0),
        //     Length::new::<inch>(21.5),
        //     Angle::new::<degree>(90.0),
        // ));
        // self.settings.borrow_mut().enable_sort = DoorCommands::Off;

        // self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);

        // zip(
        //     async {
        //         linear.drive_to_point(dt, vec2_length(Vec2::new(28.0,21.5)), false).await;
        //     },
        //     async {
        //         sleep(Duration::from_millis(250)).await;
        //         _ = self.lift.set_high();
        //         self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
        //     }
        // )
        // .await;
        // linear.drive_to_point(dt, vec2_length(Vec2::new(28.0,21.5)), false).await;

        // turn.settle_velocity(AngularVelocity::new::<degree_per_second>(5.0)).turn_to_point(dt, vec2_length(RED_LEFT_LOADER), false).await;  

        // _ = self.match_loader.set_high();
        // linear.speed(0.3).timeout(Duration::from_millis(7500)).drive_distance(dt,Length::new::<inch>(20.0)).await;

        // linear.timeout(Duration::from_millis(1000)).drive_distance(dt, Length::new::<inch>(-7.5)).await;
        // self.intake.set_voltage(0.0);
        // _ = self.match_loader.set_low();
        // sleep(Duration::from_millis(500)).await; // wait for balls to settle in robot


        // move_to.move_to_point(dt, vec2_length(Vec2::new(48.0,48.0))).await;

        // turn.turn_to_point(dt, vec2_length(Vec2::new(48.0,96.0)), true).await;

        // linear.drive_to_point(dt, vec2_length(Vec2::new(48.0,96.0)), false).await;

        // turn.turn_to_point(dt, vec2_length(BLUE_RIGHT_GOAL), true).await;


        // _ = self.duck_bill.set_high();
        // zip(
        //     async {
        //         move_to
        //             .timeout(Duration::from_millis(5000))
        //             .move_to_point(dt, vec2_length(BLUE_RIGHT_GOAL))
        //             .await;
        //     },
        //     async {
        //         sleep(Duration::from_millis(250)).await;
        //         while self.pose.borrow().vf > Velocity::new::<inch_per_second>(1.0) {
        //             sleep(Duration::from_millis(10)).await;
        //         }
        //         self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        //         sleep(Duration::from_millis(1500)).await;
        //         self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
        //         sleep(Duration::from_millis(300)).await;
        //         self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        //         sleep(Duration::from_millis(1250)).await;
        //     }
        // )
        // .await;

        //  _ = self.duck_bill.set_low();
        // linear
        //     .timeout(Duration::from_millis(1000))
        //     .drive_distance(dt, Length::new::<inch>(-15.0))
        //     .await;
        // turn.turn_to_point(dt, vec2_length(BLUE_RIGHT_LOADER), false).await;

        // _ = self.match_loader.set_high();

        // linear.speed(0.3).timeout(Duration::from_millis(5000)).drive_distance(dt,Length::new::<inch>(25.0)).await;

        // linear.timeout(Duration::from_millis(1000)).drive_distance(dt, Length::new::<inch>(-7.5)).await;
        // self.intake.set_voltage(0.0);
        // _ = self.match_loader.set_low();
        // sleep(Duration::from_millis(500)).await;

        // turn.turn_to_point(dt, vec2_length(BLUE_RIGHT_GOAL), true).await;


        // _ = self.duck_bill.set_high();
        // zip(
        //     async {
        //         move_to
        //             .timeout(Duration::from_millis(5000))
        //             .move_to_point(dt, vec2_length(BLUE_RIGHT_GOAL))
        //             .await;
        //     },
        //     async {
        //         sleep(Duration::from_millis(250)).await;
        //         while self.pose.borrow().vf > Velocity::new::<inch_per_second>(1.0) {
        //             sleep(Duration::from_millis(10)).await;
        //         }
        //         self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        //         sleep(Duration::from_millis(1500)).await;
        //         self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
        //         sleep(Duration::from_millis(300)).await;
        //         self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
        //         sleep(Duration::from_millis(1250)).await;
        //     }
        // )
        // .await;





        
    }
}