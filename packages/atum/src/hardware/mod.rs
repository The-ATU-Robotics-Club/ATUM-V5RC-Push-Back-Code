pub mod imu;
pub mod motor_group;
pub mod otos;
pub mod tracking_wheel;

mod packet;
mod serial_device;

fn average(values: Vec<f64>) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}
