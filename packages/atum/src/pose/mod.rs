pub mod odometry;

use core::ops::Sub;

use vexide::prelude::Float;

use crate::math::{angle::{Angle, IntoAngle}, length::{IntoLength, Length}};

#[derive(Clone, Copy, Default)]
pub struct Pose {
    pub x: Length,
    pub y: Length,
    pub h: Angle,
    pub vf: Length,
    pub vs: Length,
    pub omega: Angle,
}

impl Pose {
    pub fn new(x: Length, y: Length, h: Angle) -> Self {
        Self {
            x,
            y,
            h,
            vf: 0.0.inch(),
            vs: 0.0.inch(),
            omega: 0.0.rad(),
        }
    }

    pub fn magnitude(&self) -> Length {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn distance(&self, other: Vec2) -> Length {
        let pose = Self::new(other.x - self.x, other.y - self.y, self.h);
        pose.magnitude()
    }

    pub fn angular_distance(&self, other: Vec2) -> Angle {
        let dx = other.x - self.x;
        let dy = other.y - self.y; 
        dy.atan2(dx)
    }
}

#[derive(Clone, Copy, Default)]
pub struct Vec2 {
    pub x: Length,
    pub y: Length,
}

impl Vec2 {
    pub fn new(x: Length, y: Length) -> Self {
        Vec2 { x, y }
    }

    pub const fn x(&self) -> Length {
        self.x
    }

    pub const fn y(&self) -> Length {
        self.y
    }

    pub fn magnitude(&self) -> Length {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn distance(&self, other: Vec2) -> Length {
        (other - *self).magnitude()
    }

    pub fn angle(&self) -> Angle {
        self.y.atan2(self.x)
    }

    pub fn angular_distance(&self, other: Vec2) -> Angle {
        (other - *self).angle()
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}
