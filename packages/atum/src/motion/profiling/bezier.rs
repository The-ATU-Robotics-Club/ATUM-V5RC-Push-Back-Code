use crate::pose::Vec2;

pub struct Bezier {
    p0: Vec2<f64>,
    p1: Vec2<f64>,
    p2: Vec2<f64>,
    p3: Vec2<f64>,
}

impl Bezier {
    pub fn new(p0: Vec2<f64>, p1: Vec2<f64>, p2: Vec2<f64>, p3: Vec2<f64>) -> Self {
        Self { p0, p1, p2, p3 }
    }

    pub fn point(&self, t: f64) -> Vec2<f64> {
        todo!()
    }

    pub fn derivative(&self, t: f64) -> Vec2<f64> {
        todo!()
    }

    pub fn second_derivative(&self, t: f64) -> Vec2<f64> {
        todo!()
    }
}
