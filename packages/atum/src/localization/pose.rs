use std::fmt::Display;

use uom::{
    si::{
        angle::degree,
        angular_velocity::degree_per_second,
        f64::{Angle, AngularVelocity, Length, Velocity},
        length::inch,
        velocity::inch_per_second,
    },
    ConstZero,
};

#[derive(Clone, Copy, Default)]
pub struct Pose {
    pub x: Length,
    pub y: Length,
    pub h: Angle,
    pub vf: Velocity,
    pub vs: Velocity,
    pub omega: AngularVelocity,
}

impl Pose {
    pub fn new(x: Length, y: Length, h: Angle) -> Self {
        Self {
            x,
            y,
            h,
            vf: Velocity::ZERO,
            vs: Velocity::ZERO,
            omega: AngularVelocity::ZERO,
        }
    }
}

impl Display for Pose {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let x = self.x.get::<inch>();
        let y = self.y.get::<inch>();
        let h = self.h.get::<degree>();
        let vf = self.vf.get::<inch_per_second>();
        let vs = self.vs.get::<inch_per_second>();
        let omega = self.omega.get::<degree_per_second>();
        write!(
            f,
            "({:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4})",
            x, y, h, vf, vs, omega
        )
    }
}
