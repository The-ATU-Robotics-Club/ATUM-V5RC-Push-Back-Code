use std::f64::consts::{PI, TAU};

use uom::si::{
    angle::radian,
    f64::{Angle, Length},
    length::meter,
};

use crate::localization::{pose::Pose, vec2::Vec2};

pub fn wrap(angle: Angle) -> Angle {
    let angle = angle.get::<radian>();
    Angle::new::<radian>((-angle + PI).rem_euclid(TAU) - PI)
}
pub fn wrapped(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

pub fn angular_distance(pose: Pose, other: Vec2<Length>) -> Angle {
    let pose = Vec2::new(pose.x.get::<meter>(), pose.y.get::<meter>());
    let other = Vec2::new(other.x.get::<meter>(), other.y.get::<meter>());

    Angle::new::<radian>(other.angular_distance(pose))
}
