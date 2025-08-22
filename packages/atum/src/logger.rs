use core::time::Duration;

use log::{LevelFilter, Log, Metadata, Record, SetLoggerError};

const ESCAPES: [Option<&str>; 6] = [
    None,             // Default foreground
    Some("\x1B[31m"), // Error (red)
    Some("\x1B[33m"), // Warn (yellow)
    Some("\x1B[34m"), // Info (blue)
    Some("\x1B[36m"), // Debug (cyan)
    Some("\x1B[37m"), // Trace (white)
];

pub struct Logger;

impl Logger {
    pub fn init(&'static self, level: LevelFilter) -> Result<(), SetLoggerError> {
        log::set_logger(self)?;
        log::set_max_level(level);

        Ok(())
    }
}

impl Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let timestamp = Duration::from_micros(unsafe { vex_sdk::vexSystemHighResTimeGet() });
            let mins = timestamp.as_secs() / 60;
            let submin_secs = timestamp.as_secs() % 60;

            vexide::io::println!(
                "{:02}:{:02}:{:02} {}[{}]\x1B[0m {}",
                mins,
                submin_secs,
                timestamp.subsec_millis(),
                ESCAPES[record.level() as usize].unwrap_or_default(),
                record.level(),
                record.args()
            );
        }
    }

    fn flush(&self) {}
}
