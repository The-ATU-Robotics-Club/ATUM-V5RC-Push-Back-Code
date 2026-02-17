pub mod linear;
pub mod move_to;
pub mod swing;
pub mod turn;

pub fn desaturate<const N: usize>(values: [f64; N], max: f64) -> [f64; N] {
    let largest_magnitude = values.iter().map(|v| v.abs()).fold(0.0, f64::max);

    if largest_magnitude > max {
        values.map(|v| v * max / largest_magnitude)
    } else {
        values
    }
}
