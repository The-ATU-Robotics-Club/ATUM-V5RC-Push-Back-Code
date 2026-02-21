use std::time::Duration;

use atum::{
    controllers::pid::Pid,
    localization::{pose::Pose, vec2::Vec2},
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

use crate::{autons::{ANGULAR_PID, LINEAR_PID}, Robot};

impl Robot {
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
}
