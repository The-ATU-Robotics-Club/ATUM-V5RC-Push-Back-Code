use std::{cell::RefCell, rc::Rc};

use slint::{ModelRc, SharedString, VecModel};
use vexide::prelude::Display;
use vexide_slint::initialize_slint_platform;

use crate::settings::{Color, Settings};

slint::include_modules!();

pub fn start_ui(display: Display, paths: Vec<&str>, settings: Rc<RefCell<Settings>>) {
    initialize_slint_platform(display);

    let app = AppWindow::new().unwrap();

    // Convert backend paths into a Slint model
    let modes: Vec<SharedString> = paths
        .into_iter()
        .map(SharedString::from)
        .collect();

    let model = ModelRc::new(VecModel::from(modes));

    // Set the property on the component
    app.set_paths(model);

    app.global::<Selector>().on_autonomous({
        let settings = settings.clone();

        move |autonomous| {
            let index = autonomous.index as usize;

            let mut settings = settings.borrow_mut();
            settings.index = index;
            settings.color = match autonomous.color {
                SlintColor::Red => Color::Red,
                SlintColor::Blue => Color::Blue,
            };
        }
    });

    app.global::<Selector>().on_test({
        let settings = settings.clone();

        move || {
            let mut settings = settings.borrow_mut();
            settings.test_auton = true;
        }
    });

    _ = app.show();
    _ = slint::run_event_loop();
}
