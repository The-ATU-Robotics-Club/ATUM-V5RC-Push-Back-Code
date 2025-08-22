pub mod odometry;

use core::ops::Sub;

use vexide::prelude::Float;

use crate::math::{angle::{Angle, IntoAngle}, length::{IntoLength, Length}};

#[derive(Clone, Copy, Default)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub h: f64,
    pub vf: f64,
    pub vs: f64,
    pub omega: f64,
}

impl Pose {
    pub fn new(x: f64, y: f64, h: f64) -> Self {
        Self {
            x,
            y,
            h,
            vf: 0.0,
            vs: 0.0,
            omega: 0.0,
        }
    }

    pub fn magnitude(&self) -> Length {
        (self.x.powi(2) + self.y.powi(2)).sqrt().inch()
    }

    pub fn distance(&self, other: Vec2) -> Length {
        let pose = Self::new(other.x - self.x, other.y - self.y, self.h);
        pose.magnitude()
    }

    pub fn angular_distance(&self, other: Vec2) -> Angle {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        dy.atan2(dx).rad()
    }
}

#[derive(Clone, Copy, Default)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub fn new(x: f64, y: f64) -> Self {
        Vec2 { x, y }
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }

    pub fn magnitude(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    pub fn distance(&self, other: Vec2) -> f64 {
        (other - *self).magnitude()
    }

    pub fn angle(&self) -> f64 {
        self.y.atan2(self.x)
    }

    pub fn angular_distance(&self, other: Vec2) -> f64 {
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
