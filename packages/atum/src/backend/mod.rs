pub mod canvas;

use alloc::rc::Rc;
use core::cell::RefCell;

use vexide::prelude::{spawn, Display};
use vexide_slint::initialize_slint_platform;

use crate::{backend::canvas::Canvas, pose::Vec2};

slint::include_modules!();

pub struct Settings {
    pub test_auton: bool,
    pub side: SlintColor,
}

pub fn start_ui(display: Display, settings: Rc<RefCell<Settings>>) {
    initialize_slint_platform(display);

    let app = AppWindow::new().unwrap();

    app.global::<Selector>().on_autonomous({
        let ui_handler = app.as_weak();
        let settings = settings.clone();

        move |autonomous| {
            let _index = autonomous.index as usize;

            let coords = [
                Vec2::new(0.0, 0.0),
                Vec2::new(20.0, 20.0),
                Vec2::new(40.0, 15.0),
            ];

            let mut canvas = Canvas::new(144, 144, autonomous.color);
            canvas.draw_coords(&coords);

            if let Some(ui) = ui_handler.upgrade() {
                ui.global::<Selector>().set_path_image(canvas.to_image());
            }

            let mut settings = settings.borrow_mut();
            settings.side = autonomous.color;
        }
    });

    app.global::<Selector>().on_test({
        let settings = settings.clone();

        move || {
            let mut settings = settings.borrow_mut();
            settings.test_auton = true;
        }
    });

    spawn(async move {
        _ = app.show();
        _ = slint::run_event_loop();
    })
    .detach();
}
