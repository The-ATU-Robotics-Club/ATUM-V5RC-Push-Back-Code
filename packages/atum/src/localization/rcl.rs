use super::{pose::Pose, vec2::Vec2};
use crate::{
    hardware::wall_distance_sensor::{Wall, WallDistanceSensor},
    localization::shape::{Circle, Shape},
};

const FIELD_SIZE: f64 = 140.42;
pub const MAX_ERROR: f64 = 5.0;
const MAX_RAYCAST_DIST: f64 = FIELD_SIZE * 2.0;
const MIN_AXIS_COMPONENT: f64 = 0.95;

#[derive(Debug, Clone, Copy, Default)]
pub struct PoseCorrection {
    pub x: Option<f64>,
    pub y: Option<f64>,
}

impl PoseCorrection {
    pub fn is_empty(&self) -> bool {
        self.x.is_none() && self.y.is_none()
    }
}

pub struct RaycastLocalization {
    sensors: Vec<WallDistanceSensor>,
    objects: Vec<Circle>,
}

impl RaycastLocalization {
    pub fn new(sensors: Vec<WallDistanceSensor>, objects: Vec<Circle>) -> Self {
        Self { sensors, objects }
    }

    pub fn correction(&self, pose: Pose, max_error: f64) -> Option<PoseCorrection> {
        let robot_position = pose.position();

        let mut xs = Vec::new();
        let mut ys = Vec::new();

        'outer: for sensor in &self.sensors {
            let measured = match sensor.distance() {
                Ok(Some(d)) => d,
                _ => continue,
            };

            let hit =
                match sensor.predicted_hit(robot_position, pose.h, FIELD_SIZE, MAX_RAYCAST_DIST) {
                    Some(hit) => hit,
                    None => continue,
                };

            if (measured - hit.distance).abs() > max_error {
                continue;
            }

            let world_angle = sensor.world_angle(pose.h);
            let theta = world_angle.as_radians();
            let dx = theta.cos();
            let dy = theta.sin();

            for object in self.objects.iter() {
                if object.is_intersecting(Vec2::new(dx, dy), world_angle, hit.distance) {
                    continue 'outer;
                }
            }

            match hit.wall {
                Wall::Left | Wall::Right if dx.abs() < MIN_AXIS_COMPONENT => continue,
                Wall::Bottom | Wall::Top if dy.abs() < MIN_AXIS_COMPONENT => continue,
                _ => {}
            }

            // rotated sensor offset in world frame
            let rotated_offset = sensor.world_position(Vec2::new(0.0, 0.0), pose.h);

            match hit.wall {
                Wall::Left => {
                    let sensor_x = -dx * measured;
                    let robot_x = sensor_x - rotated_offset.x;

                    if in_field(robot_x) && (robot_x - pose.x).abs() <= max_error {
                        xs.push(robot_x);
                    }
                }
                Wall::Right => {
                    let sensor_x = FIELD_SIZE - dx * measured;
                    let robot_x = sensor_x - rotated_offset.x;

                    if in_field(robot_x) && (robot_x - pose.x).abs() <= max_error {
                        xs.push(robot_x);
                    }
                }
                Wall::Bottom => {
                    let sensor_y = -dy * measured;
                    let robot_y = sensor_y - rotated_offset.y;

                    if in_field(robot_y) && (robot_y - pose.y).abs() <= max_error {
                        ys.push(robot_y);
                    }
                }
                Wall::Top => {
                    let sensor_y = FIELD_SIZE - dy * measured;
                    let robot_y = sensor_y - rotated_offset.y;

                    if in_field(robot_y) && (robot_y - pose.y).abs() <= max_error {
                        ys.push(robot_y);
                    }
                }
            }
        }

        let correction = PoseCorrection {
            x: mean(&xs),
            y: mean(&ys),
        };

        if correction.is_empty() {
            None
        } else {
            Some(correction)
        }
    }

    pub fn corrected_pose(&self, pose: Pose, max_error: f64) -> Pose {
        let mut corrected = pose;

        if let Some(correction) = self.correction(pose, max_error) {
            if let Some(x) = correction.x {
                corrected.x = x;
            }

            if let Some(y) = correction.y {
                corrected.y = y;
            }
        }

        corrected
    }
}

fn mean(values: &[f64]) -> Option<f64> {
    if values.is_empty() {
        None
    } else {
        Some(values.iter().sum::<f64>() / values.len() as f64)
    }
}

fn in_field(value: f64) -> bool {
    (0.0..=FIELD_SIZE).contains(&value)
}
