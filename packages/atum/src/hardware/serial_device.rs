use core::time::Duration;

use vexide::{
    devices::smart::serial::SerialError,
    prelude::SerialPort,
    time::{sleep, Instant},
};

use super::packet::Packet;

pub struct SerialDevice {
    serial: SerialPort,
    error_id: u8,
    timeout: Duration,
}

impl SerialDevice {
    pub fn new(serial: SerialPort, error_id: u8, timeout: u64) -> Self {
        Self {
            serial,
            error_id,
            timeout: Duration::from_millis(timeout),
        }
    }

    pub async fn msg(
        &mut self,
        command: Packet,
        expected_response_size: usize,
    ) -> Result<Packet, SerialError> {
        self.serial.clear_buffers();
        self.write(&command)?;
        let start_time = Instant::now();

        while self.serial.unread_bytes()? < expected_response_size
            && Instant::now() - start_time < self.timeout
        {
            sleep(Duration::from_millis(0)).await;
        }

        if self.serial.unread_bytes()? >= expected_response_size {
            let response = self.read();
            if response.is_correct() {
                return Ok(response);
            }
        }

        Err(SerialError::WriteFailed)
    }

    pub fn write(&mut self, command: &Packet) -> Result<(), SerialError> {
        self.serial.write_byte(command.id)?;
        self.serial.write_byte(command.checksum)?;

        for &byte in command.data.iter() {
            self.serial.write_byte(byte)?;
        }

        Ok(())
    }

    pub fn read(&mut self) -> Packet {
        let mut packet = Packet::default();

        packet.id = self.serial.read_byte().unwrap_or_default();
        packet.checksum = self.serial.read_byte().unwrap_or_default();

        while let Some(byte) = self.serial.read_byte() {
            packet.data.push(byte);
        }

        packet
    }
}
