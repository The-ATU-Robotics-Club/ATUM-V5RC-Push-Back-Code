use std::{cell::RefCell, rc::Rc, time::Duration};

use vexide::{
    prelude::Motor,
    task::{Task, spawn},
    time::sleep,
};

pub struct Basic {
    voltage: Rc<RefCell<(f64, f64)>>,
    _task: Task<()>,
}

impl Basic {
    pub fn new(mut top: Motor, mut bottom: Motor) -> Self {
        let voltage = Rc::new(RefCell::new((0.0, 0.0)));

        Self {
            voltage: voltage.clone(),
            _task: spawn(async move {
                loop {
                    sleep(Duration::from_millis(10)).await;

                    let voltage = *voltage.borrow(); 
                    _ = bottom.set_voltage(voltage.0);
                    _ = top.set_voltage(voltage.1);
                }
            }),
        }
    }

    pub fn set_voltage(&self, voltage: f64) {
        self.voltage.replace((voltage, voltage));
    }

    pub fn set_bottom(&self, voltage: f64) {
        self.voltage.borrow_mut().0 = voltage;
    }

    pub fn set_top(&self, voltage: f64) {
        self.voltage.borrow_mut().1 = voltage;
    }
}
