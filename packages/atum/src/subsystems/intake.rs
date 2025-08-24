use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use vexide::{
    prelude::OpticalSensor,
    task::{spawn, Task},
    time::sleep,
};

use crate::{
    backend::{Settings, SlintColor},
    hardware::motor_group::MotorGroup,
};

pub struct Intake {
    color_sort: Rc<RefCell<bool>>,
    voltage: Rc<RefCell<f64>>,
    _task: Task<()>,
}

impl Intake {
    pub fn new(
        mut motors: MotorGroup,
        optical_sensor: OpticalSensor,
        settings: Rc<RefCell<Settings>>,
    ) -> Self {
        let color_sort = Rc::new(RefCell::new(true));
        let voltage = Rc::new(RefCell::new(0.0));

        Self {
            color_sort: color_sort.clone(),
            voltage: voltage.clone(),
            _task: spawn(async move {
                loop {
                    motors.set_voltage(*voltage.borrow());

                    if *color_sort.borrow() {
                        // Blue balls have a hue of 120 to 240 and red ring has a hue of 0 to 60
                        // so the oppisite hue is what we are checking for
                        let color_range = match settings.borrow().side {
                            SlintColor::Red => 120.0..240.0,
                            SlintColor::Blue => 0.0..60.0,
                        };

                        let color_hue = optical_sensor.hue().unwrap_or_default();
                        let proximity = optical_sensor.proximity().unwrap_or_default();

                        // filter out the wrong color that has obstructed the view
                        if color_range.contains(&color_hue) && proximity == 1.0 {
                            // do stuff to filter
                        }
                    }

                    sleep(Duration::from_millis(10)).await;
                }
            }),
        }
    }

    pub fn set_voltage(&self, voltage: f64) {
        self.voltage.replace(voltage);
    }
}
