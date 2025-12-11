use vexide::controller::{ButtonState, JoystickState};

// Different drive mods that the driver can switch to
pub enum DriveMode {
    Arcade {
        power: JoystickState,
        turn: JoystickState,
    },
    Tank {
        left: JoystickState,
        right: JoystickState,
    },
}

// TODO: Create ui to allow user to change mappings
// Map all the controller keybinds with their respective subsystem
pub struct ControllerMappings {
    pub drive_mode: DriveMode,

    pub intake_high: ButtonState,
    pub intake_low: ButtonState,
    pub outake_high: ButtonState,
    pub outake_low: ButtonState,
    pub lift: ButtonState,
    pub duck_bill: ButtonState,
}
