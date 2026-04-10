use vexide::math::Angle;

use crate::localization::vec2::Vec2;

pub trait Shape {
    fn is_intersecting(&self, spos: Vec2<f64>, sh: Angle, dist: f64) -> bool;
}

pub struct Circle {
    center: Vec2<f64>,
    radius: f64,
}

impl Circle {
    pub fn new(center: Vec2<f64>, radius: f64) -> Self {
        Self {
            center,
            radius,
        }
    }
    
    pub fn center(&self) -> Vec2<f64> {
        self.center
    }
}

impl Shape for Circle {
    fn is_intersecting(&self, spos: Vec2<f64>, sh: Angle, dist: f64) -> bool {
        let dx = sh.cos();
        let dy = sh.sin();

        // Vector from ray start to circle center
        let cx = self.center.x - spos.x;
        let cy = self.center.y - spos.y;

        // Projection onto ray direction
        let mut t = cx * dx + cy * dy;

        // Clamp to finite sensor range
        t = t.clamp(0.0, dist);

        // Closest point on the ray segment
        let closest_x = spos.x + t * dx;
        let closest_y = spos.y + t * dy;

        // Distance from closest point to circle center
        let dist_x = self.center.x - closest_x;
        let dist_y = self.center.y - closest_y;

        let dist_sq = dist_x * dist_x + dist_y * dist_y;

        dist_sq <= self.radius * self.radius
    }
}
