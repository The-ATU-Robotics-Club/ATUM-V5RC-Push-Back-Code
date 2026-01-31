use std::{cell::RefCell, rc::Rc, time::Duration};

use vexide::{
    prelude::Motor, task::{spawn, Task}, time::sleep
};

pub struct Intake {
    voltage: Rc<RefCell<f64>>,
    _task: Task<()>,
}

impl Intake {
    pub fn new(mut top: Motor, mut bottom: Motor) -> Self {
        let voltage = Rc::new(RefCell::new(0.0));

        Self {
            voltage: voltage.clone(),
            _task: spawn(async move {
                loop {
                    let voltage = *voltage.borrow();

                    _ = top.set_voltage(voltage);
                    _ = bottom.set_voltage(voltage);

                    sleep(Duration::from_millis(10)).await;
                }
            }),
        }
    }

    pub fn set_command(&self, voltage: f64) -> f64 {
        self.voltage.replace(voltage)
    }
}
