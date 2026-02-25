use std::{cell::RefCell, rc::Rc, time::Duration};

use vexide::{prelude::{spawn, Display}, time::sleep};
use vexide_slint::initialize_slint_platform;

use crate::settings::{Color, Settings};

slint::include_modules!();

pub fn start_ui(display: Display, settings: Rc<RefCell<Settings>>) {
    initialize_slint_platform(display);

    let app = AppWindow::new().unwrap();

    app.global::<Selector>().on_autonomous({
        let settings = settings.clone();

        move |autonomous| {
            println!("save");
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
            println!("test");
            let mut settings = settings.borrow_mut();
            settings.test_auton = true;
        }
    });

    _ = app.show();
    _ = slint::run_event_loop();
}
