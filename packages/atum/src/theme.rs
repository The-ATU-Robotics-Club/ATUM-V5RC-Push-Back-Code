use vexide::startup::banner::themes::BannerTheme;

macro_rules! ansi_rgb_bold {
    ($r:expr, $g:expr, $b:expr) => {
        concat!("\x1B[1;38;2;", $r, ";", $g, ";", $b, "m")
    };
}

pub const STOUT_ROBOT: BannerTheme = BannerTheme {
    emoji: "🐗",
    logo_primary: [
        ansi_rgb_bold!(157, 34, 53),
        ansi_rgb_bold!(255, 255, 255),
        ansi_rgb_bold!(157, 34, 53),
        ansi_rgb_bold!(255, 255, 255),
        ansi_rgb_bold!(157, 34, 53),
        ansi_rgb_bold!(255, 255, 255),
        ansi_rgb_bold!(157, 34, 53),
    ],
    logo_secondary: ansi_rgb_bold!(255, 215, 0),
    crate_version: ansi_rgb_bold!(157, 34, 53),
    metadata_key: ansi_rgb_bold!(255, 255, 255),
};
