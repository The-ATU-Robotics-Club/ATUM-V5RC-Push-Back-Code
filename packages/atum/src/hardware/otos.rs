use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use bytemuck::{Pod, Zeroable};
use log::{error, info};
use vexide::{
    devices::smart::serial::SerialError,
    prelude::{SerialPort, SmartPort},
    task::{spawn, Task},
    time::{sleep, Instant},
};

use super::{packet::Packet, serial_device::SerialDevice};
use crate::{
    pose::Pose,
    units::{
        angle::{Angle, IntoAngle},
        length::{IntoLength, Length},
    },
};

struct Command;

#[allow(unused)]
impl Command {
    const INITIALIZE: u8 = 0;
    const CALIBRATE: u8 = 1;
    const IS_CALIBRATING: u8 = 2;
    const RESET: u8 = 3;
    const RESET_TRACKING: u8 = 4;
    const SET_OFFSET: u8 = 5;
    const SET_POSITION: u8 = 6;
    const GET_POSITION: u8 = 7;
    const GET_VELOCITY: u8 = 8;
    const CHECK: u8 = 9;
    const SELF_TEST: u8 = 10;
    const INVALID: u8 = 11;
}

#[allow(unused)]
enum Response {
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
    pub async fn new(port: SmartPort, offset: Pose) -> Self {
        let port = SerialPort::open(port, 115200).await;
        let mut otos = SerialDevice::new(port, Response::Error as u8, 5);

        _ = otos
            .msg(Packet::new(Command::INITIALIZE), Self::SENDING_SIZE)
            .await;

        sleep(Duration::from_millis(500)).await;

        _ = otos
            .msg(Packet::new(Command::RESET), Self::SENDING_SIZE)
            .await;

        let offset_piece = OTOSData {
            x: offset.x.as_inches() as f32,
            y: offset.y.as_inches() as f32,
            h: offset.h.as_degrees() as f32,
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
            .msg(Packet::new(Command::RESET_TRACKING), Self::SENDING_SIZE)
            .await;

        _ = otos
            .msg(Packet::new(Command::SELF_TEST), Self::SENDING_SIZE)
            .await;
        sleep(Duration::from_millis(100)).await;
        info!("OTOS constructed!");
        let pose = Rc::new(RefCell::new(Pose::default()));

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

    pub fn vf(&self) -> Length {
        self.pose.borrow().vf
    }

    pub fn vs(&self) -> Length {
        self.pose.borrow().vs
    }

    pub fn omega(&self) -> Angle {
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
            x: (pos.x as f64).meter(),
            y: (pos.y as f64).meter(),
            h: (pos.h as f64).rad(),
            vf: (-0.97 * vel.x as f64).meter(),
            vs: (0.97 * vel.y as f64).meter(),
            omega: (-0.9825 * vel.h as f64).rad(),
        })
    }

    // async fn check(&mut self) -> bool {
    //     let response = self
    //         .otos
    //         .msg(Packet::new(Command::CHECK), 2)
    //         .await
    //         .unwrap();
    //     let good = response.is_correct() && response.id == Response::Success as u8;
    //
    //     if !good {
    //         // ERROR
    //     }
    //
    //     good
    // }
}
