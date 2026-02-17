use std::time::{Duration, Instant};

use log::{debug, info};
use uom::{
    ConstZero,
    si::{
        f64::{Length, Time, Velocity},
        length::{inch, meter},
        time::second,
        velocity::inch_per_second,
    },
};
use vexide::{prelude::Motor, time::sleep};

use crate::{controllers::pid::Pid, localization::vec2::Vec2, subsystems::drivetrain::Drivetrain};

pub struct Linear {
    pid: Pid,
    tolerance: Length,
    velocity_tolerance: Velocity,
    timeout: Option<Duration>,
    speed: f64,
    chain: bool,
}

impl Linear {
    pub fn new(pid: Pid, tolerance: Length, velocity_tolerance: Velocity) -> Self {
        Self {
            pid,
            tolerance,
            velocity_tolerance,
            timeout: None,
            speed: Motor::V5_MAX_VOLTAGE,
            chain: false,
        }
    }

    pub async fn drive_to_point(&mut self, dt: &mut Drivetrain, point: Vec2<Length>) {
        let point = Vec2::new(point.x.get::<meter>(), point.y.get::<meter>());
        let pose = Vec2::new(dt.pose().x.get::<meter>(), dt.pose().y.get::<meter>());
        let target_distance = Length::new::<meter>((point - pose).length());
        self.drive_distance(dt, target_distance).await;
    }

    pub async fn drive_distance(&mut self, dt: &mut Drivetrain, target: Length) {
        let mut time = Duration::ZERO;
        let mut prev_time = Instant::now();
        let mut traveled = Length::ZERO;

        loop {
            sleep(Duration::from_millis(10)).await;
            let elapsed_time = prev_time.elapsed();
            time += elapsed_time;
            prev_time = Instant::now();

            // add the total distance traveled to error
            let pose = dt.pose();
            traveled += pose.vf * Time::new::<second>(elapsed_time.as_secs_f64());
            let error = target - traveled;
            let output = self
                .pid
                .output(error.get::<meter>(), elapsed_time)
                .clamp(-self.speed, self.speed);

            if self.is_settled(error.abs(), pose.vf.abs(), time) {
                info!("Time: {}", time.as_millis());
                break;
            }

            dt.set_voltages(output, output);
        }

        self.timeout = None;
        self.speed = Motor::V5_MAX_VOLTAGE;
        self.chain = false;

        dt.set_voltages(0.0, 0.0);
    }

    fn is_settled(&mut self, error: Length, velocity: Velocity, time: Duration) -> bool {
        let within_tolerance = error < self.tolerance;
        let within_velocity = velocity < self.velocity_tolerance;
        let timed_out = self.timeout.is_some_and(|timeout| time > timeout);
        let chained = self.chain && error < self.tolerance * 2.0;

        (within_tolerance && within_velocity) || timed_out || chained
    }

    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.timeout = Some(duration);
        self
    }

    pub fn speed(&mut self, speed: f64) -> &mut Self {
        self.speed = Motor::V5_MAX_VOLTAGE * speed;
        self
    }

    pub fn chain(&mut self) -> &mut Self {
        self.chain = true;
        self
    }
}
