use std::{cell::RefCell, rc::Rc, time::Duration};

use vexide::{
    task::{spawn, Task},
    time::sleep,
};

use crate::hardware::motor_group::MotorGroup;

#[derive(Copy, Clone)]
pub enum IntakeCommand {
    ScoreHigh(f64),
    ScoreMiddle(f64),
    ScoreLow(f64),
}

pub struct Intake {
    command: Rc<RefCell<Option<IntakeCommand>>>,
    _task: Task<()>,
}

impl Intake {
    pub fn new(mut motors: MotorGroup) -> Self {
        let command = Rc::new(RefCell::new(None));

        Self {
            command: command.clone(),
            _task: spawn(async move {
                loop {
                    if let Some(command) = *command.borrow() {
                        let voltages = match command {
                            IntakeCommand::ScoreHigh(voltage) => {
                                vec![-voltage, -voltage, voltage]
                            }
                            IntakeCommand::ScoreMiddle(voltage) => {
                                vec![-voltage, voltage, voltage]
                            }
                            IntakeCommand::ScoreLow(voltage) => {
                                vec![voltage, voltage, -voltage]
                            }
                        };

                        for (motor, voltage) in motors.iter_mut().zip(voltages) {
                            _ = motor.set_voltage(voltage);
                        }
                    } else {
                        motors.set_voltage(0.0);
                    }

                    sleep(Duration::from_millis(10)).await;
                }
            }),
        }
    }

    pub fn set_command(&self, command: Option<IntakeCommand>) {
        self.command.replace(command);
    }
}
