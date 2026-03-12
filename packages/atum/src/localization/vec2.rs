use std::ops::{Add, Mul, Sub};

use num_traits::Float;

/// A simple 2D vector.
///
/// Used throughout the localization system for positions,
/// directions, and robot-frame motion.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2<T> {
    pub x: T,
    pub y: T,
}

impl<T> Vec2<T> {
    /// Creates a new vector.
    pub const fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
}

impl<T: Copy + Float> Vec2<T> {
    /// Creates a vector from polar coordinates.
    ///
    /// `r` = magnitude  
    /// `theta` = angle in radians
    pub fn from_polar(r: T, theta: T) -> Self {
        Self {
            x: r * theta.cos(),
            y: r * theta.sin(),
        }
    }

    /// Returns the angle of the vector in radians.
    pub fn angle(&self) -> T {
        self.y.atan2(self.x)
    }

    /// Returns the vector length using hypot().
    pub fn length(&self) -> T {
        self.x.hypot(self.y)
    }
}

impl<T: Float + Copy + Mul<Output = T> + Sub<Output = T>> Vec2<T> {
    /// 2D cross product.
    ///
    /// Returns the scalar magnitude of the perpendicular vector.
    pub fn cross(&self, other: Vec2<T>) -> T {
        self.x * other.y - self.y * other.x
    }
}

impl<T: Copy + Mul<Output = T> + Add<Output = T>> Vec2<T> {
    /// Dot product between two vectors.
    pub fn dot(&self, other: Vec2<T>) -> T {
        self.x * other.x + self.y * other.y
    }
}

impl<T: Float + Copy + Add<Output = T> + Sub<Output = T> + Mul<Output = T>> Vec2<T> {
    /// Returns a new vector rotated by the given angle (radians).
    pub fn rotated(&self, angle: T) -> Self {
        let (sin, cos) = angle.sin_cos();

        Self {
            x: self.x * cos - self.y * sin,
            y: self.x * sin + self.y * cos,
        }
    }
}

impl<T> From<(T, T)> for Vec2<T> {
    /// Converts a tuple `(x, y)` into a vector.
    fn from(tuple: (T, T)) -> Self {
        Self {
            x: tuple.0,
            y: tuple.1,
        }
    }
}

impl<T: Add<Output = T>> Add for Vec2<T> {
    type Output = Self;

    /// Vector addition.
    fn add(self, other: Vec2<T>) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl<T: Add<Output = T> + Copy> Add<T> for Vec2<T> {
    type Output = Self;

    /// Adds a scalar to both components.
    fn add(self, scalar: T) -> Self {
        Self {
            x: self.x + scalar,
            y: self.y + scalar,
        }
    }
}

impl<T: Sub<Output = T>> Sub for Vec2<T> {
    type Output = Self;

    /// Vector subtraction.
    fn sub(self, rhs: Self) -> Self::Output {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl<T: Sub<Output = T> + Copy> Sub<T> for Vec2<T> {
    type Output = Self;

    /// Subtracts a scalar from both components.
    fn sub(self, scalar: T) -> Self {
        Self {
            x: self.x - scalar,
            y: self.y - scalar,
        }
    }
}

impl<T: Mul<Output = T> + Copy> Mul<T> for Vec2<T> {
    type Output = Self;

    /// Multiplies both components by a scalar.
    fn mul(self, scalar: T) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

