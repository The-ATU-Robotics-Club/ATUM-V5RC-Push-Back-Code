use std::ops::Range;

use log::debug;
use vexide::{math::Angle, prelude::DistanceSensor, smart::{SmartPort, distance::DistanceObjectError}};

use crate::localization::vec2::Vec2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wall {
    Left,
    Right,
    Bottom,
    Top,
}

#[derive(Debug, Clone, Copy)]
pub struct WallHit {
    pub distance: f64,
    pub wall: Wall,
}

pub struct WallDistanceSensor {
    sensor: DistanceSensor,
    offset: Vec2<f64>,
    angle: Angle,
    wall_size: Range<u32>,
}

impl WallDistanceSensor {
    const MILLIMETER_TO_INCH: f64 = 1.0 / 25.4;

    const DIST_RANGE: Range<f64> = (20.0 * Self::MILLIMETER_TO_INCH..2000.0 * Self::MILLIMETER_TO_INCH);


    /// Creates a new Localization Sensor, which is a Distance Sensor that is primarily used for localization.
    ///
    /// - `offset` x and y position of the sensor relative to the tracking center
    /// - `angle`  Sensor beam direction relative to the robot frame.
    pub fn new(
        port: SmartPort,
        offset: Vec2<f64>,
        angle: Angle,
        wall_size: Range<u32>,
    ) -> Self {
        Self {
            sensor: DistanceSensor::new(port),
            offset,
            angle,
            wall_size,
        }
    }

    pub fn world_angle(&self, robot_heading: Angle) -> Angle {
        robot_heading + self.angle
    }

    pub fn world_position(&self, robot_position: Vec2<f64>,robot_heading: Angle) -> Vec2<f64> {
        robot_position + self.offset.rotated(robot_heading.as_radians())
    }

    pub fn distance(&self) -> Result<Option<f64>, DistanceObjectError> {
        let object = match self.sensor.object()? {
            Some(obj) => obj,
            None => return Ok(None),
        };

        let dist = object.distance as f64 * Self::MILLIMETER_TO_INCH;
        let size = object.relative_size.unwrap_or_default();
        // debug!("size {size}");

        if !Self::DIST_RANGE.contains(&dist) || !self.wall_size.contains(&size) {
            return Ok(None);
        }

        Ok(Some(dist))
    }
    
    pub fn angle(&self) -> Angle {
        self.angle
    }

    pub fn offset(&self) -> Vec2<f64> {
        self.offset
    }

    pub fn predicted_hit(
        &self,
        robot_position: Vec2<f64>,
        robot_heading: Angle,
        field_size: f64,
        max_dist: f64,
    ) -> Option<WallHit> {
        let origin = self.world_position(robot_position, robot_heading);
        let theta = self.world_angle(robot_heading).as_radians();

        let dx = theta.cos();
        let dy = theta.sin();

        let mut wall_hit: Option<WallHit> = None;

        if dx.abs() > 1e-6 {
            let t = (0.0 - origin.x) / dx;
            if t > 0.0 && t <= max_dist {
                let y_hit = origin.y + t * dy;
                if y_hit >= 0.0 && y_hit <= field_size {
                    wall_hit = Some(WallHit {
                        distance: t,
                        wall: Wall::Left,
                    });
                }
            }

            let t = (field_size - origin.x) / dx;
            if t > 0.0 && t <= max_dist {
                let y_hit = origin.y + t * dy;
                if y_hit >= 0.0 && y_hit <= field_size {
                    match wall_hit {
                        Some(hit) if t >= hit.distance => {}
                        _ => {
                            wall_hit = Some(WallHit {
                                distance: t,
                                wall: Wall::Right,
                            });
                        }
                    }
                }
            }
        }

        if dy.abs() > 1e-6 {
            let t = (0.0 - origin.y) / dy;
            if t > 0.0 && t <= max_dist {
                let x_hit = origin.x + t * dx;
                if x_hit >= 0.0 && x_hit <= field_size {
                    match wall_hit {
                        Some(hit) if t >= hit.distance => {}
                        _ => {
                            wall_hit = Some(WallHit {
                                distance: t,
                                wall: Wall::Bottom,
                            });
                        }
                    }
                }
            }

            let t = (field_size - origin.y) / dy;
            if t > 0.0 && t <= max_dist {
                let x_hit = origin.x + t * dx;
                if x_hit >= 0.0 && x_hit <= field_size {
                    match wall_hit {
                        Some(hit) if t >= hit.distance => {}
                        _ => {
                            wall_hit = Some(WallHit {
                                distance: t,
                                wall: Wall::Top,
                            });
                        }
                    }
                }
            }
        }

        wall_hit
    }

    pub fn measurement_error(
        &self,
        robot_position: Vec2<f64>,
        robot_heading: Angle,
        field_size: f64,
        max_dist: f64,
    ) -> Result<Option<(WallHit, f64)>, DistanceObjectError> {
        let measured = match self.distance()? {
            Some(d) => d,
            None => return Ok(None),
        };

        let predicted = match self.predicted_hit(robot_position, robot_heading, field_size, max_dist) {
            Some(hit) => hit,
            None => return Ok(None),
        };

        Ok(Some((predicted, measured - predicted.distance)))
    }    
}
