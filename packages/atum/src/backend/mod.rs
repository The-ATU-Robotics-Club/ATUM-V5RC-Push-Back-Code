use std::{
    cell::RefCell,
    collections::VecDeque,
    rc::Rc,
    sync::{Arc, Mutex}, time::Duration,
};

use slint::{Color as SColor, ModelRc, SharedString, ToSharedString, VecModel};
use vexide::prelude::Display;
use vexide_slint::initialize_slint_platform;

use crate::{
    logger::LogEntry,
    settings::{Color, Settings},
};

slint::include_modules!();

/// Initializes and runs the Slint GUI for autonomous selection and settings configuration.
///
/// This function sets up the frontend UI and bridges it with the backend robot settings:
/// - Converts a list of autonomous paths into a Slint model.
/// - Updates the shared `Settings` struct when a user selects event buttons.
/// - Runs the Slint event loop.
pub fn start_ui(
    display: Display,
    paths: Vec<&str>,
    logs: Arc<Mutex<VecDeque<LogEntry>>>,
    settings: Rc<RefCell<Settings>>,
) {
    // Initialize the Slint platform using the robot's display
    initialize_slint_platform(display);

    // Create the main application window
    let app = AppWindow::new().unwrap();

    // Convert backend autonomous paths names
    let modes: Vec<SharedString> = paths.into_iter().map(SharedString::from).collect();
    let model = ModelRc::new(VecModel::from(modes));

    // Assign the model to the GUI component
    app.set_paths(model);

    // Event: When an autonomous is selected in the UI
    app.global::<Selector>().on_autonomous({
        let settings = settings.clone();
        move |autonomous| {
            let index = autonomous.index as usize;

            // Update shared backend settings
            let mut settings = settings.borrow_mut();
            settings.index = index;
            settings.color = match autonomous.color {
                SlintColor::Red => Color::Red,
                SlintColor::Blue => Color::Blue,
            };
        }
    });

    // Event: When the test button is pressed in the UI
    app.global::<Selector>().on_test({
        let settings = settings.clone();
        move || {
            let mut settings = settings.borrow_mut();
            settings.test_auton = true;
        }
    });

    let weak = app.as_weak();

    let timer = slint::Timer::default();
    timer.start(
        slint::TimerMode::Repeated,
        Duration::from_millis(10),
        move || {
            if let Some(app) = weak.upgrade() {
                let model = {
                    let logs = logs.lock().unwrap();
                    let vec = logs.iter().map(to_slint_log).collect::<Vec<_>>();
                    ModelRc::new(VecModel::from(vec))
                };

                app.set_logs(model);
            }
        },
    );

    // Show the GUI window and start the Slint event loop
    _ = app.show();
    _ = slint::run_event_loop();
}

pub fn to_slint_log(log: &LogEntry) -> Log {
    Log {
        time: log.time.to_shared_string(),
        level: log.level.to_shared_string(),
        level_color: SColor::from_rgb_u8(log.level_color.0, log.level_color.1, log.level_color.2),
        message: log.message.to_shared_string(),
    }
}
