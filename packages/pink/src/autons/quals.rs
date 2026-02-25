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
    pub async fn quals(&mut self) {
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

}
