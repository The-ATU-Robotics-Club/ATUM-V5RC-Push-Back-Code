use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use vexide::prelude::Float;

// distances will default to inches
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Length(f64);

impl Length {
    const METERS_TO_INCHES: f64 = 39.3700787402;

    pub const fn from_inches(inches: f64) -> Self {
        Self(inches)
    }

    pub const fn from_meters(meters: f64) -> Self {
        Self(meters * Self::METERS_TO_INCHES)
    }

    pub const fn from_millimeters(millimeters: f64) -> Self {
        Self(millimeters * Self::METERS_TO_INCHES / 1000.0)
    }

    pub const fn as_inches(&self) -> f64 {
        self.0
    }

    pub const fn as_meters(&self) -> f64 {
        self.0 / Self::METERS_TO_INCHES
    }

    pub const fn as_millimeters(&self) -> f64 {
        self.0 / Self::METERS_TO_INCHES * 1000.0
    }
}

// impl From<Position> for Angle {
//     fn from(value: Position) -> Self {
//         Self::from_degrees(value.as_degrees())
//     }
// }

impl Add for Length {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Length {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Mul for Length {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}

impl Div for Length {
    type Output = Self;

    #[inline]
    fn div(self, rhs: Self) -> Self::Output {
        Self(self.0 / rhs.0)
    }
}

impl Add<f64> for Length {
    type Output = Self;

    #[inline]
    fn add(self, rhs: f64) -> Self::Output {
        Self(self.0 + rhs)
    }
}

impl Sub<f64> for Length {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: f64) -> Self::Output {
        Self(self.0 - rhs)
    }
}

impl Mul<f64> for Length {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs)
    }
}

impl Div<f64> for Length {
    type Output = Self;

    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        Self(self.0 / rhs)
    }
}

impl AddAssign for Length {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl AddAssign<f64> for Length {
    #[inline]
    fn add_assign(&mut self, rhs: f64) {
        self.0 += rhs;
    }
}

impl SubAssign for Length {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl MulAssign for Length {
    #[inline]
    fn mul_assign(&mut self, rhs: Self) {
        self.0 *= rhs.0;
    }
}

impl MulAssign<f64> for Length {
    #[inline]
    fn mul_assign(&mut self, rhs: f64) {
        self.0 *= rhs;
    }
}

impl DivAssign for Length {
    #[inline]
    fn div_assign(&mut self, rhs: Self) {
        self.0 /= rhs.0;
    }
}

impl Neg for Length {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

pub trait IntoLength {
    fn inch(self) -> Length;
    fn meter(self) -> Length;
    fn millimeters(self) -> Length;
}

impl IntoLength for f64 {
    fn inch(self) -> Length {
        Length::from_inches(self)
    }

    fn meter(self) -> Length {
        Length::from_meters(self)
    }

    fn millimeters(self) -> Length {
        Length::from_millimeters(self)
    }
}

crate::impl_float!(Length, f64);
