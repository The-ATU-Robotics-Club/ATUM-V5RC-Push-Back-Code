use core::{f64::consts::{PI, TAU}, ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign}};

use vexide::prelude::{Float, Position};

use crate::math::length::Length;

#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Angle(f64);

impl Angle {
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
}

impl From<Position> for Angle {
    fn from(value: Position) -> Self {
        Self::from_degrees(value.as_degrees())
    }
}

impl Add for Angle {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Angle {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Mul for Angle {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}

impl Div for Angle {
    type Output = Self;

    #[inline]
    fn div(self, rhs: Self) -> Self::Output {
        Self(self.0 / rhs.0)
    }
}

impl Add<f64> for Angle {
    type Output = Self;

    #[inline]
    fn add(self, rhs: f64) -> Self::Output {
        Self(self.0 + rhs)
    }
}

impl Sub<f64> for Angle {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: f64) -> Self::Output {
        Self(self.0 - rhs)
    }
}

impl Mul<f64> for Angle {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs)
    }
}

impl Mul<Length> for Angle {
    type Output = f64;

    #[inline]
    fn mul(self, rhs: Length) -> Self::Output {
        self.0 * rhs.as_inches()
    }
}

impl Div<f64> for Angle {
    type Output = Self;

    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        Self(self.0 / rhs)
    }
}

impl AddAssign for Angle {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl AddAssign<f64> for Angle {
    #[inline]
    fn add_assign(&mut self, rhs: f64) {
        self.0 += rhs;
    }
}

impl SubAssign for Angle {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl MulAssign for Angle {
    #[inline]
    fn mul_assign(&mut self, rhs: Self) {
        self.0 *= rhs.0;
    }
}

impl DivAssign for Angle {
    #[inline]
    fn div_assign(&mut self, rhs: Self) {
        self.0 /= rhs.0;
    }
}

impl Neg for Angle {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self(-self.0)
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

crate::impl_float!(Angle, f64);
