use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use bytemuck::{Pod, Zeroable};
use log::{error, info};
use uom::si::{
    angle::degree,
    angular_velocity::degree_per_second,
    f64::{Angle, AngularVelocity, Length, Velocity},
    length::inch,
    velocity::inch_per_second,
};
use vexide::{
    prelude::SerialPort,
    smart::{SmartPort, serial::SerialError},
    task::{Task, spawn},
    time::sleep,
};

use super::{packet::Packet, serial_device::SerialDevice};
use crate::localization::pose::Pose;

struct Command;

impl Command {
    const INITIALIZE: u8 = 0;
    const CALIBRATE: u8 = 1;
    const IS_CALIBRATING: u8 = 2;
    const RESET: u8 = 3;
    const SET_OFFSET: u8 = 4;
    const SET_POSITION: u8 = 5;
    const GET_POSITION: u8 = 6;
    const GET_VELOCITY: u8 = 7;
    const CHECK: u8 = 8;
    const SELF_TEST: u8 = 9;
}

pub enum Response {
    Success,
    Error,
    Waiting,
    Unknown,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct OTOSData {
    pub x: f32,
    pub y: f32,
    pub h: f32,
}

pub struct Otos {
    pose: Rc<RefCell<Pose>>,
    _task: Task<()>,
}

impl Otos {
    const CALIBRATION_TIMEOUT: Duration = Duration::from_secs(1);

    // sending messages to the OTOS requires 2 bytes, the response ID and checksum
    const SENDING_SIZE: usize = 2;
    // receiving messages to the OTOS requires 14 bytes, the response ID, checksum, and
    // positional data.
    const RECEIVING_SIZE: usize = 14;

    #[must_use]
    pub async fn new(port: SmartPort, start: Pose, offset: Pose) -> Self {
        let port = SerialPort::open(port, 115200).await;
        let mut otos = SerialDevice::new(port, Duration::from_millis(5));

        _ = otos
            .msg(Packet::new(Command::INITIALIZE), Self::SENDING_SIZE)
            .await;

        sleep(Duration::from_millis(500)).await;

        _ = otos
            .msg(Packet::new(Command::RESET), Self::SENDING_SIZE)
            .await;

        let starting_piece = OTOSData {
            x: start.x.get::<inch>() as f32,
            y: start.y.get::<inch>() as f32,
            h: start.h.get::<degree>() as f32,
        };

        let bytes = bytemuck::bytes_of(&starting_piece);
        let data = bytes.to_vec();
        _ = otos
            .msg(
                Packet::with_data(Command::SET_POSITION, data),
                Self::SENDING_SIZE,
            )
            .await;

        let offset_piece = OTOSData {
            x: offset.x.get::<inch>() as f32,
            y: offset.y.get::<inch>() as f32,
            h: offset.h.get::<degree>() as f32,
        };

        let bytes = bytemuck::bytes_of(&offset_piece);
        let data = bytes.to_vec();
        _ = otos
            .msg(
                Packet::with_data(Command::SET_OFFSET, data),
                Self::SENDING_SIZE,
            )
            .await;

        _ = otos
            .msg(Packet::new(Command::CALIBRATE), Self::SENDING_SIZE)
            .await;

        info!("attempting to calibrate");

        let start_time = Instant::now();
        while let Ok(msg) = otos
            .msg(Packet::new(Command::IS_CALIBRATING), Self::SENDING_SIZE)
            .await
        {
            if msg.id != Response::Waiting as u8 {
                info!("calibration success");
                break;
            }

            if start_time.elapsed() > Self::CALIBRATION_TIMEOUT {
                error!("calibration timeout");
                break;
            }

            sleep(Duration::from_millis(10)).await;
        }

        _ = otos
            .msg(Packet::new(Command::SELF_TEST), Self::SENDING_SIZE)
            .await;
        sleep(Duration::from_millis(100)).await;
        Self::check(&mut otos).await;
        info!("OTOS constructed!");
        let pose = Rc::new(RefCell::new(start));

        Self {
            pose: pose.clone(),
            _task: spawn(async move {
                sleep(Duration::from_millis(100)).await;

                loop {
                    match Self::get_pose(&mut otos).await {
                        Ok(updated_pose) => {
                            pose.replace(updated_pose);
                        }
                        Err(e) => {
                            error!("Otos failed: {}", e);
                        }
                    }

                    sleep(Duration::from_millis(10)).await;
                }
            }),
        }
    }

    pub fn pose(&self) -> Pose {
        *self.pose.borrow()
    }

    pub fn x(&self) -> Length {
        self.pose.borrow().x
    }

    pub fn y(&self) -> Length {
        self.pose.borrow().y
    }

    pub fn h(&self) -> Angle {
        self.pose.borrow().h
    }

    pub fn vf(&self) -> Velocity {
        self.pose.borrow().vf
    }

    pub fn vs(&self) -> Velocity {
        self.pose.borrow().vs
    }

    pub fn omega(&self) -> AngularVelocity {
        self.pose.borrow().omega
    }

    async fn get_pose(otos: &mut SerialDevice) -> Result<Pose, SerialError> {
        let pos_packet = otos
            .msg(Packet::new(Command::GET_POSITION), Self::RECEIVING_SIZE)
            .await?;

        if pos_packet.id != Response::Success as u8 || !pos_packet.is_correct() {
            return Err(SerialError::ReadFailed);
        }

        let pos = bytemuck::from_bytes::<OTOSData>(&pos_packet.data);

        let vel_packet = otos
            .msg(Packet::new(Command::GET_VELOCITY), Self::RECEIVING_SIZE)
            .await?;

        if vel_packet.id != Response::Success as u8 || !vel_packet.is_correct() {
            return Err(SerialError::ReadFailed);
        }

        let vel = bytemuck::from_bytes::<OTOSData>(&vel_packet.data);

        Ok(Pose {
            x: Length::new::<inch>(pos.x as f64),
            y: Length::new::<inch>(pos.y as f64),
            h: Angle::new::<degree>(pos.h as f64),
            vf: Velocity::new::<inch_per_second>(-0.97 * vel.x as f64),
            vs: Velocity::new::<inch_per_second>(0.97 * vel.y as f64),
            omega: AngularVelocity::new::<degree_per_second>(-0.9825 * vel.h as f64),
        })
    }

    // confirm if this function works as intended
    async fn check(otos: &mut SerialDevice) -> bool {
        let response = otos
            .msg(Packet::new(Command::CHECK), 2)
            .await
            .unwrap_or_default();
        let good = response.is_correct() && response.id == Response::Success as u8;

        if !good {
            error!("Detected issue with OTOS.");
        }

        good
    }
}
