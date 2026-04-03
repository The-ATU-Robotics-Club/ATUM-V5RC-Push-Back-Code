use super::{pose::Pose, vec2::Vec2};
use crate::hardware::wall_distance_sensor::{Wall, WallDistanceSensor};

const FIELD_SIZE: f64 = 140.42;
const MAX_ERROR: f64 = 6.0;
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

pub struct DSL {
    sensors: Vec<WallDistanceSensor>,
}

impl DSL {
    pub fn new(sensors: Vec<WallDistanceSensor>) -> Self {
        Self { sensors }
    }

    pub fn correction(&self, pose: Pose) -> Option<PoseCorrection> {
        let robot_position = pose.position();

        let mut xs = Vec::new();
        let mut ys = Vec::new();

        for sensor in &self.sensors {
            let measured = match sensor.distance() {
                Ok(Some(d)) => d,
                _ => continue,
            };

            let hit = match sensor.predicted_hit(
                robot_position,
                pose.h,
                FIELD_SIZE,
                MAX_RAYCAST_DIST,
            ) {
                Some(hit) => hit,
                None => continue,
            };

            if (measured - hit.distance).abs() > MAX_ERROR {
                continue;
            }

            let world_angle = sensor.world_angle(pose.h);
            let theta = world_angle.as_radians();
            let dx = theta.cos();
            let dy = theta.sin();

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

                    if in_field(robot_x) && (robot_x - pose.x).abs() <= MAX_ERROR {
                        xs.push(robot_x);
                    }
                }
                Wall::Right => {
                    let sensor_x = FIELD_SIZE - dx * measured;
                    let robot_x = sensor_x - rotated_offset.x;

                    if in_field(robot_x) && (robot_x - pose.x).abs() <= MAX_ERROR {
                        xs.push(robot_x);
                    }
                }
                Wall::Bottom => {
                    let sensor_y = -dy * measured;
                    let robot_y = sensor_y - rotated_offset.y;

                    if in_field(robot_y) && (robot_y - pose.y).abs() <= MAX_ERROR {
                        ys.push(robot_y);
                    }
                }
                Wall::Top => {
                    let sensor_y = FIELD_SIZE - dy * measured;
                    let robot_y = sensor_y - rotated_offset.y;

                    if in_field(robot_y) && (robot_y - pose.y).abs() <= MAX_ERROR {
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

    pub fn corrected_pose(&self, pose: Pose) -> Pose {
        let mut corrected = pose;

        if let Some(correction) = self.correction(pose) {
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