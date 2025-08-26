// distances will default to inches
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Length(f64);

impl Length {
    pub const ZERO: Self = Self(0.0);

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

    // 24 being the size of one VEX field tile
    pub const fn from_tiles(tiles: f64) -> Self {
        Self(tiles / 24.0)
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

    pub const fn as_tiles(&self) -> f64 {
        self.0 * 24.0
    }

    pub const fn is_infinite(&self) -> bool {
        self.0.is_infinite()
    }
}

pub trait IntoLength {
    fn inch(self) -> Length;
    fn meter(self) -> Length;
    fn millimeters(self) -> Length;
    fn tile(self) -> Length;
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

    fn tile(self) -> Length {
        Length::from_tiles(self)
    }
}

impl Mul<Length> for f64 {
    type Output = Length;

    fn mul(self, rhs: Length) -> Self::Output {
        Length::from_inches(self * rhs.0)
    }
}

super::float::impl_float!(Length, f64);
super::ops::impl_ops!(Length, f64);
