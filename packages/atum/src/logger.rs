use std::{
    collections::VecDeque,
    fmt::Display,
    sync::{Arc, Mutex},
};

use log::{Level, LevelFilter, Log, Metadata, Record, SetLoggerError};

const ESCAPES: [Option<&str>; 6] = [
    None,             // Default foreground
    Some("\x1B[31m"), // Error (red)
    Some("\x1B[33m"), // Warn (yellow)
    Some("\x1B[34m"), // Info (blue)
    Some("\x1B[36m"), // Debug (cyan)
    Some("\x1B[37m"), // Trace (white)
];

#[derive(Clone)]
pub struct LogEntry {
    pub time: String,
    pub level: Level,
    pub level_color: (u8, u8, u8),
    pub message: String,
}

impl Default for LogEntry {
    fn default() -> Self {
        Self {
            time: String::new(),
            level: Level::Trace,
            level_color: (255, 255, 255),
            message: String::new(),
        }
    }
}

impl Display for LogEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} {}[{}]\x1B[0m {}",
            self.time,
            ESCAPES[self.level as usize].unwrap_or_default(),
            self.level,
            self.message,
        )
    }
}

pub struct Logger {
    messages: Arc<Mutex<VecDeque<LogEntry>>>,
}

impl Logger {
    pub fn new() -> Self {
        Self {
            messages: Arc::new(Mutex::new(VecDeque::from(vec![LogEntry::default(); 10]))),
        }
    }

    pub fn init(&'static self, level: LevelFilter) -> Result<(), SetLoggerError> {
        log::set_logger(self)?;
        log::set_max_level(level);

        Ok(())
    }

    pub fn clone_messages(&self) -> Arc<Mutex<VecDeque<LogEntry>>> {
        self.messages.clone()
    }
}

impl Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }

        let timestamp = vexide::time::system_uptime();
        let mins = timestamp.as_secs() / 60;
        let submin_secs = timestamp.as_secs() % 60;

        let entry = LogEntry {
            time: format!(
                "{:02}:{:02}:{:03}",
                mins,
                submin_secs,
                timestamp.subsec_millis()
            ),
            level: record.level(),
            level_color: match record.level() {
                Level::Error => (0xFF, 0x4C, 0x4C), // Red
                Level::Warn => (0xFF, 0xA5, 0x00),  // Orange
                Level::Info => (0x4C, 0x9A, 0xFF),  // Blue
                Level::Debug => (0x00, 0xFF, 0xFF), // Cyan
                Level::Trace => (0xFF, 0xFF, 0xFF), // White
            },
            message: record.args().to_string(),
        };

        println!("{}", entry);

        if let Ok(mut messages) = self.messages.lock() {
            messages.pop_front();
            messages.push_back(entry);
        }
    }

    fn flush(&self) {}
}
