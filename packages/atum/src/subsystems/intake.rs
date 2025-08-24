use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use vexide::{
    task::{spawn, Task},
    time::sleep,
};

use crate::hardware::motor_group::MotorGroup;

pub struct Intake {
    voltage: Rc<RefCell<f64>>,
    _task: Task<()>,
}

impl Intake {
    pub fn new(mut motors: MotorGroup) -> Self {
        let voltage = Rc::new(RefCell::new(0.0));

        Self {
            voltage: voltage.clone(),
            _task: spawn(async move {
                loop {
                    motors.set_voltage(*voltage.borrow());
                    sleep(Duration::from_millis(10)).await;
                }
            }),
        }
    }

    pub fn set_voltage(&self, voltage: f64) {
        self.voltage.replace(voltage);
    }
}
