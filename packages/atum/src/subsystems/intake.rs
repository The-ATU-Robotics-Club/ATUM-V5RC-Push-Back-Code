use std::{cell::RefCell, rc::Rc, time::Duration};

use log::info;
use vexide::{
    prelude::{AdiDigitalOut, Motor, OpticalSensor},
    task::{Task, spawn},
    time::sleep,
};

use super::RobotSettings;
use crate::subsystems::Color;

pub struct Intake {
    voltage: Rc<RefCell<f64>>,
    _task: Task<()>,
}

impl Intake {
    pub fn new(
        mut top: Motor,
        mut bottom: Motor,
        mut door: AdiDigitalOut,
        color_sort: OpticalSensor,
        delay: Duration,
        settings: Rc<RefCell<RobotSettings>>,
    ) -> Self {
        let voltage = Rc::new(RefCell::new(0.0));

        Self {
            voltage: voltage.clone(),
            _task: spawn(async move {
                let mut ball_timer = Duration::ZERO;

                loop {
                    let voltage = *voltage.borrow();
                    let settings = *settings.borrow();

                    _ = top.set_voltage(voltage);
                    _ = bottom.set_voltage(voltage);

                    if settings.enable_color {
                        // Red hue -> 0-60
                        // Blue hue -> 120-240
                        let (alliance, opposing) = match settings.color {
                            Color::Red => (20.0..55.0, 70.0..210.0),
                            Color::Blue => (70.0..210.0, 20.0..55.0),
                        };

                        let hue = color_sort.hue().unwrap_or_default();
                        let proximity = color_sort.proximity().unwrap_or_default();

                        if proximity > 0.1 {
                            if alliance.contains(&hue) {
                                info!("red: {}", proximity);
                                sleep(delay).await;
                                _ = door.set_low();
                            } else if opposing.contains(&hue) {
                                info!("blue: {}", proximity);
                                _ = door.set_high();
                            }
                            ball_timer = Duration::ZERO;
                        } else if ball_timer > Duration::from_millis(1000) {
                            _ = door.set_low();
                            ball_timer = Duration::ZERO;
                        } else if door.level().is_ok_and(|x| x.is_high()) {
                            ball_timer += Duration::from_millis(10);
                        }

                        // debug!("{}", proximity);
                    }

                    sleep(Duration::from_millis(10)).await;
                }
            }),
        }
    }

    pub fn set_voltage(&self, voltage: f64) -> f64 {
        self.voltage.replace(voltage)
    }

    pub fn test_door(&mut self) {}
}
