use std::time::Duration;

use vexide::battery;

#[derive(Default)]
pub struct Battery {
    capacity: f64,
    time: Duration,
}

impl Battery {
    const AMP_HOURS: f64 = 1.1;

    pub fn update(&mut self) {
        let current = battery::current();
        let capacity = battery::capacity();
        self.capacity = capacity * 100.0;
        self.time = Duration::from_secs_f64(capacity * Self::AMP_HOURS / current * 3600.0);
    }

    pub fn capacity(&self) -> f64 {
        self.capacity
    }

    pub fn time_remaining(&self) -> String {
        let time = self.time.as_secs();

        let secs = time % 60;
        let mins = time / 60;
        let hours = mins / 60;

        if hours == 0 {
            format!("{}m {}s", mins, secs)
        } else {
            format!("{}h {}m", hours, mins)
        }
    }
}
