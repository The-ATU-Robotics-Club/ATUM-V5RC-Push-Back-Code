use core::time::Duration;

pub struct Pid {
    kp: f64,
    ki: f64,
    kd: f64,
    integral_threshold: f64,
    prev_error: f64,
    integral: f64,
}

impl Pid {
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

    pub fn output(&mut self, error: f64, dt: Duration) -> f64 {
        if error.abs() < self.integral_threshold {
            self.integral += error * dt.as_secs_f64();
        } else {
            self.integral = 0.0;
        }

        if error.signum() != self.prev_error.signum() {
            self.integral = 0.0;
        }

        let derivative = (error - self.prev_error) / dt.as_secs_f64();
        self.prev_error = error;

        error * self.kp + self.integral * self.ki + derivative * self.kd
    }
}
