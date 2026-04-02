/// Motion control module.
///
/// Contains motion primitives used to move the robot in autonomous.
/// Each submodule implements a specific type of movement controller.
pub mod linear;
pub mod move_to;
pub mod swing;
pub mod turn;

use std::time::Duration;

/// Common configuration parameters used by motion controllers.
///
/// These parameters control when a motion is considered complete
/// and how aggressively the robot is allowed to move.
///
/// This struct is generic over the tolerance type so it can be reused
/// for different error units (distance, angle, etc.).
pub struct MotionParameters<T: Copy + PartialEq + PartialOrd + Default> {
    /// Acceptable position error required to consider the motion complete.
    pub tolerance: T,

    /// Optional velocity threshold used to determine if the robot has
    /// fully settled at the target.
    pub velocity_tolerance: Option<f64>,

    /// Optional timeout used to terminate the motion if it takes too long.
    pub timeout: Option<Duration>,

    /// Speed scaling factor used to limit maximum output.
    pub speed: f64,
}

/// Provides default motion parameters.
///
/// Defaults are intentionally permissive so controllers can run without
/// requiring full configuration:
///
/// - `tolerance` is set to the type's default value.
/// - `velocity_tolerance` is disabled (`None`), meaning settling is based
///   only on positional error unless explicitly configured.
/// - `timeout` is disabled (`None`), allowing the motion to run indefinitely.
/// - `speed` is set to `1.0`, representing full output scaling.
impl<T: Copy + PartialEq + PartialOrd + Default> Default for MotionParameters<T> {
    fn default() -> Self {
        Self {
            tolerance: T::default(),
            velocity_tolerance: Default::default(),
            timeout: Default::default(),
            speed: 1.0,
        }
    }
}

pub type MotionResult<T> = Result<(), MotionError<T>>;

pub enum MotionError<T> {
    /// The distance from a motion's target after a timeout.
    Timeout(T),

    /// Sensor failure.
    Sensor,
}

/// Scales a set of outputs proportionally so the largest magnitude
/// equals `max`, preserving the ratio between all values.
pub fn desaturate<const N: usize>(values: [f64; N], max: f64) -> [f64; N] {
    // Determine the largest magnitude in the input array
    let largest_magnitude = values.iter().map(|v| v.abs()).fold(0.0, f64::max);

    // If any value exceeds the allowed maximum,
    // scale all values proportionally.
    if largest_magnitude > max {
        values.map(|v| v * max / largest_magnitude)
    } else {
        values
    }
}
