use std::{cell::RefCell, rc::Rc, time::Duration};

use log::debug;
use vexide::{
    math::Angle,
    prelude::{Motor, RotationSensor},
    task::{Task, spawn},
    time::sleep,
};

use crate::hardware::motor_group::MotorGroup;

#[derive(Copy, Clone)]
pub enum LeverStage {
    Score(f64),
    Reset,
    Idle,
}

pub struct Lever {
    voltage: Rc<RefCell<f64>>,
    lever_stage: Rc<RefCell<LeverStage>>,
    _task: Task<()>,
}

impl Lever {
    pub fn new(mut intake: Motor, mut lever: MotorGroup, mut rotation: RotationSensor) -> Self {
        let voltage = Rc::new(RefCell::new(0.0));
        let lever_stage = Rc::new(RefCell::new(LeverStage::Idle));

        Self {
            voltage: voltage.clone(),
            lever_stage: lever_stage.clone(),
            _task: spawn(async move {
                loop {
                    sleep(Duration::from_millis(10)).await;

                    let voltage = *voltage.borrow();
                    _ = intake.set_voltage(voltage);

                    let mut lever_stage = lever_stage.borrow_mut();
                    let position = -rotation.position().unwrap_or_default().wrapped_half();
                    debug!("{}", position.as_degrees());
                    match *lever_stage {
                        LeverStage::Score(voltage) => {
                            if position > Angle::from_degrees(120.0) {
                                *lever_stage = LeverStage::Reset;
                                drop(lever_stage);
                                sleep(Duration::from_millis(50)).await;
                            }

                            lever.set_voltage(voltage);
                        }
                        LeverStage::Reset => {
                            if position < Angle::from_degrees(5.0) {
                                *lever_stage = LeverStage::Idle;
                            }

                            lever.set_voltage(-Motor::V5_MAX_VOLTAGE);
                        }
                        LeverStage::Idle => {
                            lever.set_voltage(-0.5);
                        }
                    }
                }
            }),
        }
    }

    pub fn set_intake(&self, voltage: f64) {
        self.voltage.replace(voltage);
    }

    pub fn score(&self, stage: LeverStage) {
        self.lever_stage.replace(stage);
    }

    pub fn stage(&self) -> LeverStage {
        *self.lever_stage.borrow()
    }
}
