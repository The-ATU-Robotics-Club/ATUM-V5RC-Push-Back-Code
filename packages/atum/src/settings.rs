use std::ops::{Not, Range};

#[derive(Clone, Copy)]
pub struct Settings {
    pub color: Color,
    pub index: usize,
    pub test_auton: bool,
    pub enable_sort: bool,
}

#[derive(Clone, Copy)]
pub enum Color {
    Red,
    Blue,
}

impl Color {
    pub const RED_HUE: Range<f64> = 20.0..55.0;
    pub const BLUE_HUE: Range<f64> = 70.0..210.0;

    pub fn hue_range(&self) -> Range<f64> {
        match self {
            Color::Red => Self::RED_HUE,
            Color::Blue => Self::BLUE_HUE,
        }
    }
}

impl Not for Color {
    type Output = Self;

    fn not(self) -> Self::Output {
        match self {
            Color::Red => Color::Blue,
            Color::Blue => Color::Red,
        }
    }
}