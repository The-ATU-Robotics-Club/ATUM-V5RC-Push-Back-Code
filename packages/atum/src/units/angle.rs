use core::f64::consts::{PI, TAU};

use vexide::prelude::Position;

#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Angle(f64);

impl Angle {
    pub const ZERO: Self = Self(0.0);

    pub const fn from_radians(radians: f64) -> Self {
        Self(radians)
    }

    pub const fn from_degrees(degrees: f64) -> Self {
        Self(degrees.to_radians())
    }

    pub const fn from_revolutions(revolutions: f64) -> Self {
        Self(revolutions * TAU)
    }

    pub const fn as_radians(&self) -> f64 {
        self.0
    }

    pub const fn as_degrees(&self) -> f64 {
        self.0.to_degrees()
    }

    pub const fn as_revolutions(&self) -> f64 {
        self.0 / TAU
    }

    pub fn wrap(&self) -> Self {
        Self((self.0 + PI).rem_euclid(TAU) - PI)
    }

    pub const fn is_infinite(&self) -> bool {
        self.0.is_infinite()
    }
}

impl From<Position> for Angle {
    fn from(value: Position) -> Self {
        Self::from_degrees(value.as_degrees())
    }
}

pub trait IntoAngle {
    fn deg(self) -> Angle;
    fn rad(self) -> Angle;
}

impl IntoAngle for f64 {
    fn deg(self) -> Angle {
        Angle::from_degrees(self)
    }

    fn rad(self) -> Angle {
        Angle::from_radians(self)
    }
}

super::float::impl_float!(Angle, f64);
super::ops::impl_ops!(Angle, f64);
