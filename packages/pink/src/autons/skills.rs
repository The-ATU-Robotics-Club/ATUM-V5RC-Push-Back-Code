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
use vexide::{
    prelude::{LinkType, Motor, RadioLink, sleep},
    smart::SmartPort,
};

use crate::{
    Robot,
    autons::{ANGULAR_PID, LINEAR_PID},
};

impl Robot {
    pub async fn skills(&mut self) {
        let communication =
            RadioLink::open(unsafe { SmartPort::new(1) }, "ATUM", LinkType::Manager);

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
