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

    pub intake: ButtonState,
    pub outake: ButtonState,
    pub lift: ButtonState,
    pub duck_bill: ButtonState,
    pub wing: ButtonState,
    pub match_load: ButtonState,

    // color sort stuff
    pub swap_color: ButtonState,
    pub enable_color: ButtonState,
}
