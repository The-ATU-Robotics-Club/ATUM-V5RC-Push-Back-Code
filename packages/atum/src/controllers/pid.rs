//! PID Controller
//!
//! Implements a basic PID (Proportional–Integral–Derivative) controller
//! commonly used in robotics and feedback control systems.
//!
//! A PID controller produces an output based on the error between a
//! desired target value and the current measurement:
//!
//!     error = target - measurement
//!
//! The controller output is the sum of three terms:
//!
//!     output = kp * error
//!            + ki * integral(error)
//!            + kd * derivative(error)
//!
//! - **Proportional (P)** reacts to the current error.
//! - **Integral (I)** accumulates past error to remove steady-state bias.
//! - **Derivative (D)** reacts to how quickly the error is changing,
//!   helping reduce overshoot.
//!
//! ## Implementation Details
//!
//! This implementation includes:
//!
//! - Integral accumulation limited by an `integral_threshold`
//! - Automatic integral reset when the error changes sign
//! - Derivative calculated from the change in error
//!
//! ## Usage
//!
//! ```
//! let mut pid = Pid::new(kp, ki, kd, threshold);
//! let output = pid.output(error, dt);
//! ```
//!
//! where `dt` is the time since the previous update.

use std::time::Duration;

#[derive(Clone, Copy)]
pub struct Pid {
    /// Proportional gain
    kp: f64,

    /// Integral gain
    ki: f64,

    /// Derivative gain
    kd: f64,

    /// Maximum absolute error where the integral term is allowed to accumulate.
    /// Prevents integral windup when far from the target.
    integral_threshold: f64,

    /// Error from the previous update.
    /// Used to compute the derivative term.
    prev_error: f64,

    /// Accumulated integral of error over time.
    integral: f64,
}

impl Pid {
    /// Creates a new PID controller with the given gains and integral threshold.
    pub const fn new(kp: f64, ki: f64, kd: f64, integral_threshold: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_threshold,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    /// Computes the controller output based on the current error and timestep.
    ///
    /// `error` = target - measurement
    ///
    /// `dt` is the time since the previous update.
    pub fn output(&mut self, error: f64, dt: Duration) -> f64 {
        let dt = dt.as_secs_f64();

        // Only accumulate the integral when the error is reasonably small.
        // This prevents large integral buildup while far from the target.
        if error.abs() < self.integral_threshold {
            self.integral += error * dt;
        } else {
            self.integral = 0.0;
        }

        // Reset the integral if we cross the setpoint.
        // This prevents the controller from overshooting due to previously
        // accumulated error pushing in the opposite direction.
        if error.signum() != self.prev_error.signum() {
            self.integral = 0.0;
        }

        // Compute the rate of change of error (derivative term).
        let derivative = (error - self.prev_error) / dt;

        // Store error for the next update.
        self.prev_error = error;

        // Combine P, I, and D terms to produce the final output.
        error * self.kp
            + self.integral * self.ki
            + derivative * self.kd
    }

    /// Resets the internal state of the PID controller.
    ///
    /// Clears the accumulated integral and previous error so the
    /// controller restarts on the next update.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}
