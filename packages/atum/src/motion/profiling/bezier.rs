use crate::pose::Vec2;

pub struct Bezier {
    p0: Vec2<f64>,
    p1: Vec2<f64>,
    p2: Vec2<f64>,
    p3: Vec2<f64>,
}

impl Bezier {
    pub fn new<P: Into<Vec2<f64>>>(p0: P, p1: P, p2: P, p3: P) -> Self {
        Self {
            p0: p0.into(),
            p1: p1.into(),
            p2: p2.into(),
            p3: p3.into(),
        }
    }

    pub fn point(&self, t: f64) -> Vec2<f64> {
        (self.p3 + (self.p1 - self.p2) * 3.0 - self.p0) * (t * t * t)
            + (self.p0 - self.p1 * 2.0 + self.p2) * (3.0 * t * t)
            + (self.p1 - self.p0) * (3.0 * t)
            + self.p0
    }

    pub fn derivative(&self, t: f64) -> Vec2<f64> {
        ((self.p3 + (self.p1 - self.p2) * 3.0 - self.p0) * (t * t)
            + (self.p0 - self.p1 * 2.0 + self.p2) * (2.0 * t)
            + (self.p1 - self.p0))
            * 3.0
    }

    pub fn second_derivative(&self, t: f64) -> Vec2<f64> {
        ((self.p3 + (self.p1 - self.p2) * 3.0 - self.p0) * t + (self.p0 - self.p1 * 2.0 + self.p2))
            * 6.0
    }
}
