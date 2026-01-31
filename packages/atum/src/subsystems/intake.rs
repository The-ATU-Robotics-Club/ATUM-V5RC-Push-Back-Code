use std::{cell::RefCell, rc::Rc, time::Duration};

use vexide::{
    adi::digital::LogicLevel, prelude::{AdiDigitalOut, Motor, OpticalSensor}, task::{spawn, Task}, time::sleep
};

use crate::subsystems::Color;

use super::RobotSettings;

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
                loop {
                    let voltage = *voltage.borrow();
                    let settings = *settings.borrow();

                    _ = top.set_voltage(voltage);
                    _ = bottom.set_voltage(voltage);

                    if settings.enable_color {
                        // Red hue -> 0-60
                        // Blue hue -> 120-240
                        let (alliance, opposing) = match settings.color {
                            Color::Red => (0.0..60.0, 120.0..240.0),
                            Color::Blue => (120.0..240.0, 0.0..60.0),
                        };

                        let hue = color_sort.hue().unwrap_or_default();
                        let proximity = color_sort.proximity().unwrap_or_default();

                        // switch low and high if piston starts differently
                        let level = if alliance.contains(&hue) && proximity == 1.0 {
                            Some(LogicLevel::Low)
                        } else if opposing.contains(&hue) && proximity == 1.0 {
                            Some(LogicLevel::High)
                        } else {
                            None
                        };

                        if let Some(level) = level {
                            sleep(delay).await;
                            _ = door.set_level(level);
                        }
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
