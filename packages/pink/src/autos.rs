use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::pose::Pose,
    motion::{linear::Linear, move_to::MoveTo, swing::Swing, turn::Turn},
};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        f64::{Angle, Length},
        length::inch,
    },
};
use vexide::prelude::{Motor, sleep};

const LINEAR_PID: Pid = Pid::new(46.0, 0.0, 3.95, 12.0);
const ANGULAR_PID: Pid = Pid::new(19.0, 0.25, 1.4, 25.0);

const _SETTLE_LIN_VEL: f64 = 2.5; // INCHES
const _SETTLE_ANG_VEL: f64 = 1.0; // DEGREES

use crate::Robot;

impl Robot {
    pub async fn qual(&mut self) {
        let _move_to = MoveTo::new(
            Pid::new(30.0, 1.0, 6.0, 12.0),
            Pid::new(21.0, 2.0, 0.0, 18.0),
            Length::new::<inch>(1.0),
        );

        let mut linear = Linear::new(LINEAR_PID, Length::new::<inch>(0.5));

        let mut turn = Turn::new(ANGULAR_PID, Angle::new::<degree>(1.0));

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(84.0),
            Length::new::<inch>(24.0),
            Angle::new::<degree>(55.0),
        ));

        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);

        linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, Length::new::<inch>(45.0))
            .await;

        linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, Length::new::<inch>(-20.0))
            .await;

        turn.timeout(Duration::from_millis(1000))
            .turn_to(dt, Angle::new::<degree>(135.0))
            .await;

        linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, Length::new::<inch>(20.0))
            .await;
    }

    pub async fn elims(&mut self) {
        let mut linear = Linear::new(LINEAR_PID, Length::new::<inch>(0.5));

        let mut turn = Turn::new(ANGULAR_PID, Angle::new::<degree>(1.0));

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(0.0),
            Length::new::<inch>(0.0),
            Angle::new::<degree>(0.0),
        ));

        linear
            .timeout(Duration::from_millis(525))
            .drive_distance(dt, Length::new::<inch>(17.0))
            .await;

        swing
            .swing_to(dt, Angle::new::<degree>(-90.0), Length::new::<inch>(-5.0))
            .await;

        linear
            .timeout(Duration::from_millis(400))
            .drive_distance(dt, Length::new::<inch>(4.0))
            .await;

        sleep(Duration::from_millis(2500)).await;

        linear
            .timeout(Duration::from_millis(570))
            .drive_distance(dt, Length::new::<inch>(-12.0))
            .await;
        turn.timeout(Duration::from_millis(625))
            .turn_to(dt, Angle::new::<degree>(87.0))
            .await;
        linear
            .timeout(Duration::from_millis(525))
            .drive_distance(dt, Length::new::<inch>(12.0))
            .await;
        sleep(Duration::from_millis(3000)).await;

        swing
            .timeout(Duration::from_millis(550))
            .swing_to(dt, Angle::ZERO, Length::new::<inch>(7.0))
            .await;
        linear
            .timeout(Duration::from_millis(550))
            .drive_distance(dt, Length::new::<inch>(-5.0))
            .await;
        turn.timeout(Duration::from_millis(850))
            .turn_to(dt, Angle::new::<degree>(87.0))
            .await;
        linear
            .timeout(Duration::from_millis(700))
            .drive_distance(dt, Length::new::<inch>(25.0))
            .await;

        _ = self.brake.set_high();
    }

    pub async fn safequals(&mut self) {
        let mut linear = Linear::new(LINEAR_PID, Length::new::<inch>(0.5));

        let mut turn = Turn::new(ANGULAR_PID, Angle::new::<degree>(1.0));

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(0.0),
            Length::new::<inch>(0.0),
            Angle::new::<degree>(90.0),
        ));

        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(33.0))
            .await;
        self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
        swing
            .timeout(Duration::from_millis(1500))
            .swing_to(dt, Angle::new::<degree>(135.0), Length::new::<inch>(4.0))
            .await;

        linear
            .timeout(Duration::from_millis(1500))
            .drive_distance(dt, Length::new::<inch>(-47.0))
            .await;
        turn.timeout(Duration::from_millis(850))
            .turn_to(dt, Angle::new::<degree>(-90.0))
            .await;

        self.intake.set_voltage(0.0);

        linear
            .timeout(Duration::from_millis(850))
            .drive_distance(dt, Length::new::<inch>(8.0))
            .await;
        sleep(Duration::from_millis(2500)).await;
        linear
            .timeout(Duration::from_millis(570))
            .drive_distance(dt, Length::new::<inch>(-12.0))
            .await;
        turn.timeout(Duration::from_millis(850))
            .turn_to(dt, Angle::new::<degree>(87.0))
            .await;

        linear
            .timeout(Duration::from_millis(525))
            .drive_distance(dt, Length::new::<inch>(15.0))
            .await;
        sleep(Duration::from_millis(3000)).await;

        swing
            .timeout(Duration::from_millis(550))
            .swing_to(dt, Angle::ZERO, Length::new::<inch>(7.0))
            .await;

        linear
            .timeout(Duration::from_millis(550))
            .drive_distance(dt, Length::new::<inch>(-6.0))
            .await;
        turn.timeout(Duration::from_millis(850))
            .turn_to(dt, Angle::new::<degree>(87.0))
            .await;
        linear
            .timeout(Duration::from_millis(700))
            .drive_distance(dt, Length::new::<inch>(28.0))
            .await;
    }

    pub async fn rushelims(&mut self) {
        let mut linear = Linear::new(LINEAR_PID, Length::new::<inch>(0.5));

        let mut turn = Turn::new(ANGULAR_PID, Angle::new::<degree>(1.0));

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(0.0),
            Length::new::<inch>(0.0),
            Angle::new::<degree>(105.27),
        ));

        linear
            .timeout(Duration::from_millis(1500))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(27.5))
            .await;
        self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
        sleep(Duration::from_millis(300)).await;

        linear
            .timeout(Duration::from_millis(750))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(-10.5))
            .await;
        turn.timeout(Duration::from_millis(1000))
            .turn_to(dt, Angle::new::<degree>(-35.0))
            .await;
        linear
            .timeout(Duration::from_millis(1500))
            .drive_distance(dt, Length::new::<inch>(40.0))
            .await;
        turn.timeout(Duration::from_millis(850))
            .turn_to(dt, Angle::new::<degree>(-92.0))
            .await;
        linear
            .timeout(Duration::from_millis(850))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(8.0))
            .await;

        sleep(Duration::from_millis(2500)).await;
        linear
            .timeout(Duration::from_millis(570))
            .drive_distance(dt, Length::new::<inch>(-11.0))
            .await;
        turn.timeout(Duration::from_millis(625))
            .turn_to(dt, Angle::new::<degree>(87.0))
            .await;
        linear
            .timeout(Duration::from_millis(525))
            .drive_distance(dt, Length::new::<inch>(15.0))
            .await;
        sleep(Duration::from_millis(3000)).await;

        // info!("Time elapsed: {:?}", time.elapsed());

        swing
            .timeout(Duration::from_millis(750))
            .swing_to(dt, Angle::ZERO, Length::new::<inch>(7.0))
            .await;
        linear
            .timeout(Duration::from_millis(850))
            .drive_distance(dt, Length::new::<inch>(-5.0))
            .await;
        turn.timeout(Duration::from_millis(850))
            .turn_to(dt, Angle::new::<degree>(90.0))
            .await;
        linear
            .timeout(Duration::from_millis(700))
            .drive_distance(dt, Length::new::<inch>(28.0))
            .await;
    }

    pub async fn rushcontrol(&mut self) {
        let mut linear = Linear::new(LINEAR_PID, Length::new::<inch>(0.5));

        let mut turn = Turn::new(ANGULAR_PID, Angle::new::<degree>(1.0));

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(0.0),
            Length::new::<inch>(0.0),
            Angle::new::<degree>(0.0),
        ));

        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(24.0))
            .await;
        turn.timeout(Duration::from_millis(575))
            .turn_to(dt, Angle::new::<degree>(-90.0))
            .await;
        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(11.0))
            .await;
        sleep(Duration::from_millis(750)).await;
        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(-16.0))
            .await;
        turn.timeout(Duration::from_millis(650))
            .turn_to(dt, Angle::new::<degree>(89.0))
            .await;
        linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, Length::new::<inch>(11.5))
            .await;
        sleep(Duration::from_millis(750)).await;

        swing
            .timeout(Duration::from_millis(500))
            .swing_to(dt, Angle::ZERO, Length::new::<inch>(2.0))
            .await;
        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(-13.0))
            .await;
        turn.timeout(Duration::from_millis(575))
            .turn_to(dt, Angle::new::<degree>(83.0))
            .await;
        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(26.0))
            .await;

        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(24.0))
            .await;
        turn.timeout(Duration::from_millis(575))
            .turn_to(dt, Angle::new::<degree>(-90.0))
            .await;
        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(11.0))
            .await;
        sleep(Duration::from_millis(750)).await;
        linear
            .timeout(Duration::from_millis(1000))
            .drive_distance(dt, Length::new::<inch>(-16.0))
            .await;
        turn.timeout(Duration::from_millis(650))
            .turn_to(dt, Angle::new::<degree>(89.0))
            .await;
        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(11.5))
            .await;
        sleep(Duration::from_millis(750)).await;

        swing
            .timeout(Duration::from_millis(500))
            .swing_to(dt, Angle::ZERO, Length::new::<inch>(2.0))
            .await;
        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(-13.0))
            .await;
        turn.timeout(Duration::from_millis(575))
            .turn_to(dt, Angle::new::<degree>(83.0))
            .await;
        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(26.0))
            .await;

        //Activate Brakes here
    }

    pub async fn skills(&mut self) {
        let mut linear = Linear::new(LINEAR_PID, Length::new::<inch>(0.5));

        let mut turn = Turn::new(ANGULAR_PID, Angle::new::<degree>(1.0));

        let _swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
        );

        let dt = &mut self.drivetrain;
        dt.set_pose(Pose::new(
            Length::new::<inch>(0.0),
            Length::new::<inch>(0.0),
            Angle::new::<degree>(0.0),
        ));

        linear
            .timeout(Duration::from_millis(1000))
            .chain(2.0)
            .drive_distance(dt, Length::new::<inch>(19.0))
            .await;
        turn.timeout(Duration::from_millis(1000))
            .turn_to(dt, Angle::new::<degree>(90.0))
            .await;
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);

        linear
            .timeout(Duration::from_millis(2500))
            .speed(0.5)
            .drive_distance(dt, Length::new::<inch>(65.0))
            .await;
        turn.timeout(Duration::from_millis(1000))
            .turn_to(dt, Angle::new::<degree>(45.0))
            .await;
    }
}
