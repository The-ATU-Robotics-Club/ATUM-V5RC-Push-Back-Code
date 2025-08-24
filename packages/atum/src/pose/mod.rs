pub mod odometry;

use core::ops::Sub;

use vexide::float::Float;

use crate::units::{
    angle::{Angle, IntoAngle},
    length::Length,
};

#[derive(Clone, Copy, Default)]
pub struct Pose {
    pub x: Length,
    pub y: Length,
    pub h: Angle,
    // change these to `LinearVelocity` and `AngularVelocity`
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
            vf: Length::ZERO,
            vs: Length::ZERO,
            omega: Angle::ZERO,
        }
    }

    pub fn distance(&self, other: Vec2<Length>) -> Length {
        Vec2::new(self.x, self.y).distance(other)
    }

    pub fn angular_distance(&self, other: Vec2<Length>) -> Angle {
        Vec2::new(self.x, self.y)
            .angular_distance(other)
            .as_inches()
            .rad()
    }
}

#[derive(Clone, Copy, Default)]
pub struct Vec2<T> {
    pub x: T,
    pub y: T,
}

impl<T> Vec2<T> {
    pub fn new(x: T, y: T) -> Self {
        Vec2 { x, y }
    }
}

impl<T: Copy> Vec2<T> {
    pub const fn x(&self) -> T {
        self.x
    }

    pub const fn y(&self) -> T {
        self.y
    }
}

impl<T: Copy + Float> Vec2<T> {
    pub fn angle(&self) -> T {
        self.y.atan2(self.x)
    }

    pub fn magnitude(&self) -> T {
        self.x.hypot(self.y)
    }
}

impl<T: Copy + Float + Sub<Output = T>> Vec2<T> {
    pub fn distance(&self, other: Self) -> T {
        (other - *self).magnitude()
    }

    pub fn angular_distance(&self, other: Self) -> T {
        (other - *self).angle()
    }
}

impl<T: Sub<Output = T>> Sub for Vec2<T> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}
