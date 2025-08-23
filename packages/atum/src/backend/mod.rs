pub mod canvas;

use alloc::rc::Rc;
use vexide_slint::initialize_slint_platform;
use core::cell::RefCell;

use vexide::prelude::{Display, spawn};

use crate::{backend::canvas::Canvas, math::length::IntoLength, pose::Vec2};

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
                Vec2::new(0.0.inch(), 0.0.inch()),
                Vec2::new(20.0.inch(), 20.0.inch()),
                Vec2::new(40.0.inch(), 15.0.inch()),
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
