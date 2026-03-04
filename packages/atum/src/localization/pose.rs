use std::fmt::Display;

use vexide::math::Angle;

#[derive(Clone, Copy, Default)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub h: Angle,
    pub vf: f64,
    pub vs: f64,
    pub omega: f64,
}

impl Pose {
    pub fn new(x: f64, y: f64, h: Angle) -> Self {
        Self {
            x,
            y,
            h,
            vf: 0.0,
            vs: 0.0,
            omega: 0.0,
        }
    }
}

impl Display for Pose {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "({:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4})",
            self.x,
            self.y,
            self.h.as_degrees(),
            self.vf,
            self.vs,
            self.omega
        )
    }
}
