//! Controller Mappings
//!
//! This module defines how the physical controller inputs are mapped to
//! robot actions. It supports multiple drive modes and includes buttons
//! for subsystems and other special functions.
//!
//! - [`DriveMode`] – Different drive configurations the driver can select
//!   * Arcade – forward/backward + turning
//!   * Tank – independent left/right control
//! - [`ControllerMappings`] – Stores all joystick and button states for
//!   driving, subsystems, and extra features.
use vexide::controller::{ButtonState, JoystickState};

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

pub struct ControllerMappings {
    pub drive_mode: DriveMode,

    pub intake: ButtonState,
    pub outake: ButtonState,
    pub lift: ButtonState,
    pub duck_bill: ButtonState,
    pub wing: ButtonState,
    pub match_load: ButtonState,
    pub brake: ButtonState,
    pub back_door: ButtonState,

    // color sort stuff
    pub swap_color: ButtonState,
    pub enable_color: ButtonState,
}

pub struct ControllerMappingsLever {
    pub drive_mode: DriveMode,

    pub intake: ButtonState,
    pub outake: ButtonState,
    pub lever: ButtonState,
    pub lift: ButtonState,
    pub duck_bill: ButtonState,
    pub wing: ButtonState,
    pub match_load: ButtonState,
    pub brake: ButtonState,
}
