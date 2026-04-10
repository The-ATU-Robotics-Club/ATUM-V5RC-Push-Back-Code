use std::{cell::RefCell, rc::Rc, time::Duration};

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
    voltage: Rc<RefCell<(f64, f64)>>, // Shared voltage to be applied to motors
    door_commands: Rc<RefCell<DoorCommands>>,
    _task: Task<()>, // Background task that loops continuously
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
        door_commands: DoorCommands,
        settings: Rc<RefCell<Settings>>,
    ) -> Self {
        let voltage = Rc::new(RefCell::new((0.0, 0.0)));
        let door_commands = Rc::new(RefCell::new(door_commands));

        // TODO: Separate logic so intake isn't overwhelmingly long
        Self {
            voltage: voltage.clone(),
            door_commands: door_commands.clone(),
            _task: spawn(async move {
                let mut ball_timer = Duration::ZERO;

                // Main control loop for the intake task
                loop {
                    // Sleep briefly to yield scheduler
                    sleep(Duration::from_millis(10)).await;

                    // Read desired motor voltage and apply to motors
                    let voltage = *voltage.borrow(); 
                    _ = bottom.set_voltage(voltage.0);
                    _ = top.set_voltage(voltage.1);

                    // Manual door commands
                    door_commands.replace_with(|prev| {
                        match *prev {
                            DoorCommands::Open => {
                                _ = door.set_high();
                                DoorCommands::On
                            }
                            DoorCommands::Close => {
                                _ = door.set_low();
                                DoorCommands::Off
                            }
                            _ => *prev,
                        }
                    });

                    // Snapshot current settings
                    let color_settings = *settings.borrow();
                    let door_settings = *door_commands.borrow();

                    // Override door state if requested
                    if color_settings.color_override {
                        _ = door.toggle();
                    }

                    // Main color-based sorting logic
                    if matches!(door_settings, DoorCommands::On) {
                        let alliance = color_settings.color.hue_range(); // Hue range for alliance balls
                        let opposing = (!color_settings.color).hue_range(); // Hue range for opponent balls

                        let hue = color_sort.hue().unwrap_or_default();
                        let proximity = color_sort.proximity().unwrap_or_default();

                        // Ball detected close enough to sensor
                        if proximity > 0.3 {
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
                }
            }),
        }
    }

    /// Sets the intake motor voltage.
    ///
    /// This updates the shared voltage reference that the background task reads.
    pub fn set_voltage(&self, voltage: f64) {
        self.voltage.replace((voltage, voltage));
    }

    pub fn set_bottom(&self, voltage: f64) {
        self.voltage.borrow_mut().0 = voltage;
    }

    pub fn set_top(&self, voltage: f64) {
        self.voltage.borrow_mut().1 = voltage;
    }

    pub fn set_door(&self, door_command: DoorCommands) {
        self.door_commands.replace(door_command);
    }

    pub fn door(&self) -> DoorCommands {
        *self.door_commands.borrow()
    }
}
