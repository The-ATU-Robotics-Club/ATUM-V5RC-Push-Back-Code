use std::{cell::RefCell, rc::Rc, time::Duration};

use log::info;
use vexide::{
    prelude::{AdiDigitalOut, Motor, OpticalSensor},
    task::{Task, spawn},
    time::sleep,
};

use crate::settings::Settings;

#[derive(Clone, Copy)]
pub enum DoorCommands {
    On,
    Off,
    ForceOff,
    Open,  // for autonomous
    Close, //    ^^^
}

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
        settings: Rc<RefCell<Settings>>,
    ) -> Self {
        let voltage = Rc::new(RefCell::new(0.0));

        Self {
            voltage: voltage.clone(),
            _task: spawn(async move {
                let mut ball_timer = Duration::ZERO;
                // let mut balls_collected = 0;

                loop {
                    let voltage = *voltage.borrow();
                    let color_settings = *settings.borrow();

                    _ = bottom.set_voltage(voltage);
                    _ = top.set_voltage(voltage);

                    settings.borrow_mut().enable_sort = match color_settings.enable_sort {
                        DoorCommands::Open => {
                            _ = door.set_high();
                            DoorCommands::On
                        }
                        DoorCommands::Close => {
                            _ = door.set_low();
                            DoorCommands::On
                        }
                        _ => color_settings.enable_sort,
                    };

                    if color_settings.color_override {
                        _ = door.toggle();
                    }

                    if matches!(color_settings.enable_sort, DoorCommands::On) {
                        let alliance = color_settings.color.hue_range();
                        let opposing = (!color_settings.color).hue_range();

                        let hue = color_sort.hue().unwrap_or_default();
                        let proximity = color_sort.proximity().unwrap_or_default();

                        if proximity > 0.3 {
                            info!("hue: {hue}");
                            if alliance.contains(&hue) {
                                info!("alliance: {}", proximity);
                                sleep(delay).await;
                                _ = door.set_low();
                                // balls_collected += 1;
                            } else if opposing.contains(&hue) {
                                info!("opposing: {}", proximity);
                                _ = door.set_high();
                            }
                            ball_timer = Duration::ZERO;
                        } else if ball_timer > Duration::from_millis(1000) {
                            _ = door.set_low();
                            ball_timer = Duration::ZERO;
                        } else if door.level().is_ok_and(|x| x.is_high()) {
                            ball_timer += Duration::from_millis(10);
                        }

                        // info!("{}", proximity);
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
