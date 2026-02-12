use std::time::{Duration, Instant};

use atum::{
    controllers::pid::Pid,
    localization::pose::Pose,
    motion::{linear::Linear, move_to::MoveTo, swing::Swing, turn::Turn},
};
use log::info;
use uom::{
    ConstZero,
    si::{
        angle::degree,
        angular_velocity::{AngularVelocity, degree_per_second},
        f64::{Angle, Length, Velocity},
        length::inch,
        velocity::inch_per_second,
    },
};
use vexide::prelude::{Motor, sleep};

const LINEAR_PID: Pid = Pid::new(46.0, 0.0, 3.95, 12.0);
const ANGULAR_PID: Pid = Pid::new(19.0, 0.25, 1.4, 25.0);

use crate::Robot;

impl Robot {
    pub async fn qual(&mut self) {
        let _move_to = MoveTo::new(
            Pid::new(30.0, 1.0, 6.0, 12.0),
            Pid::new(21.0, 2.0, 0.0, 18.0),
            Length::new::<inch>(1.0),
            Velocity::new::<inch_per_second>(2.5),
        );

        let mut linear = Linear::new(
            LINEAR_PID,
            Length::new::<inch>(0.5),
            Velocity::new::<inch_per_second>(2.5),
        );

        let mut turn = Turn::new(
            ANGULAR_PID,
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(1.0),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(84.0),
            Length::new::<inch>(24.0),
            Angle::new::<degree>(55.0),
        ));

        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);

        linear
            .drive_distance(
                dt,
                Length::new::<inch>(45.0),
                false,
                Duration::from_millis(1000),
            )
            .await;

        linear
            .drive_distance(
                dt,
                Length::new::<inch>(-20.0),
                false,
                Duration::from_millis(1000),
            )
            .await;

        turn.turn_to(dt, Angle::new::<degree>(135.0), Duration::from_millis(1000))
            .await;

        linear
            .drive_distance(
                dt,
                Length::new::<inch>(20.0),
                false,
                Duration::from_millis(1000),
            )
            .await;
    }

    pub async fn elims(&mut self) {
        let mut linear = Linear::new(
            LINEAR_PID,
            Length::new::<inch>(0.5),
            Velocity::new::<inch_per_second>(2.5),
        );

        let mut turn = Turn::new(
            ANGULAR_PID,
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(2.5),
        );

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(5.0),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(0.0),
            Length::new::<inch>(0.0),
            Angle::new::<degree>(0.0),
        ));

        let time = Instant::now();

        linear
            .drive_distance(
                dt,
                Length::new::<inch>(17.0),
                false,
                Duration::from_millis(525),
            )
            .await;
        swing
            .swing_to(
                dt,
                Angle::new::<degree>(-90.0),
                Length::new::<inch>(-5.0),
                Duration::from_millis(725),
            )
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(4.0),
                false,
                Duration::from_millis(400),
            )
            .await;
        sleep(Duration::from_millis(2500)).await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(-12.0),
                false,
                Duration::from_millis(570),
            )
            .await;
        turn.turn_to(dt, Angle::new::<degree>(87.0), Duration::from_millis(625))
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(12.0),
                false,
                Duration::from_millis(525),
            )
            .await;
        sleep(Duration::from_millis(3000)).await;

        info!("Time elapsed: {:?}", time.elapsed());

        swing
            .swing_to(
                dt,
                Angle::ZERO,
                Length::new::<inch>(7.0),
                Duration::from_millis(550),
            )
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(-5.0),
                false,
                Duration::from_millis(550),
            )
            .await;
        turn.turn_to(dt, Angle::new::<degree>(87.0), Duration::from_millis(850))
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(25.0),
                false,
                Duration::from_millis(700),
            )
            .await;

        _ = self.brake.set_high();

        info!("Time elapsed: {:?}", time.elapsed());
    }
    pub async fn safequals(&mut self) {
        let mut linear = Linear::new(
            LINEAR_PID,
            Length::new::<inch>(0.5),
            Velocity::new::<inch_per_second>(2.5),
        );

        let mut turn = Turn::new(
            ANGULAR_PID,
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(2.5),
        );

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(5.0),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(0.0),
            Length::new::<inch>(0.0),
            Angle::new::<degree>(90.0),
        ));
        let time = Instant::now();
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(33.0),
                false,
                Duration::from_millis(1000),
            )
            .await;
        turn.turn_to(dt, Angle::new::<degree>(135.0), Duration::from_millis(750))
            .await;
        self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(4.0),
                false,
                Duration::from_millis(850),
            )
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(-49.0),
                false,
                Duration::from_millis(1500),
            )
            .await;
        turn.turn_to(dt, Angle::new::<degree>(-90.0), Duration::from_millis(850))
            .await;
        self.intake.set_voltage(0.0);
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(8.0),
                false,
                Duration::from_millis(850),
            )
            .await;
        sleep(Duration::from_millis(2500)).await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(-12.0),
                false,
                Duration::from_millis(570),
            )
            .await;
        turn.turn_to(dt, Angle::new::<degree>(85.0), Duration::from_millis(625))
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(15.0),
                false,
                Duration::from_millis(525),
            )
            .await;
        sleep(Duration::from_millis(3000)).await;

        info!("Time elapsed: {:?}", time.elapsed());

        swing
            .swing_to(
                dt,
                Angle::ZERO,
                Length::new::<inch>(7.0),
                Duration::from_millis(550),
            )
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(-5.0),
                false,
                Duration::from_millis(550),
            )
            .await;
        turn.turn_to(dt, Angle::new::<degree>(85.0), Duration::from_millis(850))
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(28.0),
                false,
                Duration::from_millis(700),
            )
            .await;

        info!("Time elapsed: {:?}", time.elapsed());
    }

    pub async fn rushelims(&mut self) {
        let mut linear = Linear::new(
            LINEAR_PID,
            Length::new::<inch>(0.5),
            Velocity::new::<inch_per_second>(2.5),
        );

        let mut turn = Turn::new(
            ANGULAR_PID,
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(2.5),
        );

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
            AngularVelocity::new::<degree_per_second>(5.0),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(0.0),
            Length::new::<inch>(0.0),
            Angle::new::<degree>(105.27),
        ));
        let time = Instant::now();

        linear
            .drive_distance(
                dt,
                Length::new::<inch>(27.5),
                true,
                Duration::from_millis(1500),
            )
            .await;
        self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
        sleep(Duration::from_millis(300)).await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(-10.5),
                true,
                Duration::from_millis(750),
            )
            .await;
        turn.turn_to(dt, Angle::new::<degree>(-35.0), Duration::from_millis(1000))
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(40.0),
                false,
                Duration::from_millis(1500),
            )
            .await;
        turn.turn_to(dt, Angle::new::<degree>(-92.0), Duration::from_millis(850))
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(8.0),
                true,
                Duration::from_millis(850),
            )
            .await;

        sleep(Duration::from_millis(2500)).await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(-11.0),
                false,
                Duration::from_millis(570),
            )
            .await;
        turn.turn_to(dt, Angle::new::<degree>(87.0), Duration::from_millis(625))
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(15.0),
                false,
                Duration::from_millis(525),
            )
            .await;
        sleep(Duration::from_millis(3000)).await;

        // info!("Time elapsed: {:?}", time.elapsed());

        swing
            .swing_to(
                dt,
                Angle::ZERO,
                Length::new::<inch>(7.0),
                Duration::from_millis(750),
            )
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(-5.0),
                false,
                Duration::from_millis(850),
            )
            .await;
        turn.turn_to(dt, Angle::new::<degree>(90.0), Duration::from_millis(850))
            .await;
        linear
            .drive_distance(
                dt,
                Length::new::<inch>(28.0),
                false,
                Duration::from_millis(700),
            )
            .await;

        info!("Time elapsed: {:?}", time.elapsed());
    }
}
