use alloc::vec::Vec;

// Packets used to transfer data over from the Arduino to Vex brain side
#[derive(Default)]
pub struct Packet {
    pub id: u8,
    pub checksum: u8,
    pub data: Vec<u8>,
}

impl Packet {
    pub fn new(id: u8) -> Self {
        Packet::with_data(id, Vec::new())
    }

    pub fn with_data(id: u8, data: Vec<u8>) -> Self {
        let checksum = data.iter().fold(id, |acc, &b| acc ^ b);
        Self { id, checksum, data }
    }

    pub fn is_correct(&self) -> bool {
        let parity = self
            .data
            .iter()
            .fold(self.id ^ self.checksum, |acc, &b| acc ^ b);
        parity == 0
    }
}
