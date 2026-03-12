use std::{cell::RefCell, rc::Rc, time::Duration};

use log::info;
use vexide::{
    prelude::{AdiDigitalOut, Motor, OpticalSensor},
    task::{Task, spawn},
    time::sleep,
};

use crate::settings::Settings;

/// Represents the different commands the intake door can receive.
#[derive(Clone, Copy)]
pub enum DoorCommands {
    On,       // Keep color sorting active
    Off,      // Deactivate color sorting
    ForceOff, // Deactivate color sorting, preventing other commands from overwritting
    Open,
    Close,
}

/// Manages intake motors and the door mechanism
/// Handles ball color detection and automatic door operation.
pub struct Intake {
    voltage: Rc<RefCell<f64>>, // Shared voltage to be applied to motors
    _task: Task<()>,           // Background task that loops continuously
}

impl Intake {
    /// Creates a new Intake system.
    ///
    /// # Arguments
    /// - `top` / `bottom`: intake motors
    /// - `door`: pneumatic door control
    /// - `color_sort`: optical sensor for ball detection
    /// - `delay`: delay to wait when door opens/closes on a ball
    /// - `settings`: shared configuration and control flags
    pub fn new(
        mut top: Motor,
        mut bottom: Motor,
        mut door: AdiDigitalOut,
        color_sort: OpticalSensor,
        delay: Duration,
        settings: Rc<RefCell<Settings>>,
    ) -> Self {
        let voltage = Rc::new(RefCell::new(0.0));

        // TODO: Separate logic so intake isn't overwhelmingly long
        Self {
            voltage: voltage.clone(),
            _task: spawn(async move {
                let mut ball_timer = Duration::ZERO;

                // Main control loop for the intake task
                loop {
                    // Read desired motor voltage
                    let voltage = *voltage.borrow();
                    // Snapshot current settings
                    let color_settings = *settings.borrow();

                    // Apply voltage to intake motors
                    _ = bottom.set_voltage(voltage);
                    _ = top.set_voltage(voltage);

                    // Handle door commands
                    settings.borrow_mut().door_commands = match color_settings.door_commands {
                        DoorCommands::Open => {
                            _ = door.set_high();
                            DoorCommands::On
                        }
                        DoorCommands::Close => {
                            _ = door.set_low();
                            DoorCommands::On
                        }
                        _ => color_settings.door_commands,
                    };

                    // Override door state if requested
                    if color_settings.color_override {
                        _ = door.toggle();
                    }

                    // Main color-based sorting logic
                    if matches!(color_settings.door_commands, DoorCommands::On) {
                        let alliance = color_settings.color.hue_range(); // Hue range for alliance balls
                        let opposing = (!color_settings.color).hue_range(); // Hue range for opponent balls

                        let hue = color_sort.hue().unwrap_or_default();
                        let proximity = color_sort.proximity().unwrap_or_default();

                        // Ball detected close enough to sensor
                        if proximity > 0.3 {
                            info!("hue: {hue}");
                            if alliance.contains(&hue) {
                                // Wait to allow ball to move before closing the door
                                sleep(delay).await;
                                _ = door.set_low();
                            } else if opposing.contains(&hue) {
                                // Filter out ball
                                _ = door.set_high();
                            }
                            ball_timer = Duration::ZERO; // Reset timer
                        } else if ball_timer > Duration::from_millis(1000) {
                            // If no ball detected for 1 second, close door
                            _ = door.set_low();
                            ball_timer = Duration::ZERO;
                        } else if door.level().is_ok_and(|x| x.is_high()) {
                            // Increment timer if door is still open
                            ball_timer += Duration::from_millis(10);
                        }
                    }

                    // Sleep briefly to yield scheduler
                    sleep(Duration::from_millis(10)).await;
                }
            }),
        }
    }

    /// Sets the intake motor voltage.
    ///
    /// This updates the shared voltage reference that the background task reads.
    pub fn set_voltage(&self, voltage: f64) -> f64 {
        self.voltage.replace(voltage)
    }
}

