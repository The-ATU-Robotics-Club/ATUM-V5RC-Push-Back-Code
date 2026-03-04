pub mod imu;
pub mod motor_group;
pub mod tracking_wheel;

fn average(values: Vec<f64>) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}
