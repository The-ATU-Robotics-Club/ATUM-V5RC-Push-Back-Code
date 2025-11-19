pub mod odometry;

use core::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use uom::{
    si::{
        angle::degree, angular_velocity::degree_per_second, f64::{Angle, AngularVelocity, Length, Velocity}, length::inch, velocity::inch_per_second
    },
    ConstZero,
};
use vexide::float::Float;

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
        write!(f, "({:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4})", x, y, h, vf, vs, omega)
    }
}

#[derive(Clone, Copy, Debug, Default,PartialEq)]
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

    pub fn length(&self) -> T {
        self.x.hypot(self.y)
    }
}

impl<T: Copy + Float + Add<Output = T> + Mul<Output = T>> Vec2<T> {
    pub fn magnitude(&self) -> T {
        let x = self.x;
        let y = self.y;

        (x * x + y * y).sqrt()
    }
}

impl<T: Copy + Float + Sub<Output = T>> Vec2<T> {
    pub fn distance(&self, other: Self) -> T {
        (*self - other).length()
    }

    pub fn angular_distance(&self, other: Self) -> T {
        (*self - other).angle()
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

impl<T: Float + Copy + Mul<Output = T> + Sub<Output = T>> Vec2<T>{
    pub fn cross(&self, other: Vec2<T>) -> T{
        self.x*other.y-self.y*other.x
    }
}

impl<T: Float + Copy + Mul<Output = T> + Add<Output = T>> Vec2<T>{
    pub fn dot(&self,other: Vec2<T>)->T{
        self.x*other.x+self.y+other.y

    }
}

impl<T: Float + Copy + Add<Output = T> + Sub<Output = T> + Mul<Output = T>> Vec2<T> {
    /// Create a new vector with its coordinates rotated by a given angle
    /// in radians
    pub fn rotated(&self, angle: T) -> Self {
        let (sin, cos) = angle.sin_cos();

        Self {
            x: self.x * cos - self.y * sin,
            y: self.x * sin + self.y * cos,
        }
    }
}

impl<T> From<(T, T)> for Vec2<T> {
    fn from(tuple: (T, T)) -> Self {
        Self {
            x: tuple.0,
            y: tuple.1,
        }
    }
}

impl<T: Add<Output = T>> Add for Vec2<T> {
    type Output = Self;

    fn add(self, other: Vec2<T>) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl<T: Add<Output = T> + Copy> Add<T> for Vec2<T> {
    type Output = Self;

    fn add(self, scalar: T) -> Self {
        Self {
            x: self.x + scalar,
            y: self.y + scalar,
        }
    }
}

impl<T: Sub<Output = T> + Copy> Sub<T> for Vec2<T> {
    type Output = Self;

    fn sub(self, scalar: T) -> Self {
        Self {
            x: self.x - scalar,
            y: self.y - scalar,
        }
    }
}

impl<T: Mul<Output = T> + Copy> Mul<T> for Vec2<T> {
    type Output = Self;

    fn mul(self, scalar: T) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}
