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
    pub async fn safetyskills(&mut self){
        let mut linear = Linear::new(LINEAR_PID, Length::new::<inch>(0.5));

        let mut turn = Turn::new(ANGULAR_PID, Angle::new::<degree>(1.0));

        let mut swing = Swing::new(
            Pid::new(1000.0, 150.0, 0.0, 90.0),
            Angle::new::<degree>(1.0),
        );

        let dt = &mut self.drivetrain;

        dt.set_pose(Pose::new(
            Length::new::<inch>(56.0),
            Length::new::<inch>(23.5),
            Angle::new::<degree>(90.0),
        ));
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
    }
}