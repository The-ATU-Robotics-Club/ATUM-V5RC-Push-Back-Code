/// Apply a polynomial acceleration curve to a joystick input
///
/// - `power` – input value from -1.0 to 1.0
/// - `acceleration` – exponent for scaling
pub fn apply_curve(power: f64, acceleration: i32, max: f64) -> f64 {
    power.powi(acceleration - 1)
        * if acceleration % 2 == 0 {
            power.abs()
        } else {
            power
        }
        / max.powi(acceleration - 1)
}

/// Scales a set of outputs proportionally so the largest magnitude
/// equals `max`, preserving the ratio between all values.
pub fn desaturate<const N: usize>(values: [f64; N], max: f64) -> [f64; N] {
    let largest_magnitude = values.iter().map(|v| v.abs()).fold(0.0, f64::max);

    if largest_magnitude > max {
        values.map(|v| v * max / largest_magnitude)
    } else {
        values
    }
}
