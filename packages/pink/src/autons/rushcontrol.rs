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
}
