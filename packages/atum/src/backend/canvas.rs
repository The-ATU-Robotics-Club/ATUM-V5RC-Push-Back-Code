use alloc::{vec, vec::Vec};

use slint::{Image, SharedPixelBuffer};

use crate::{pose::Vec2};
use super::SlintColor;

// Drawing platform for different lines and curves for the autonomous selector
pub struct Canvas {
    width: u32,
    height: u32,
    buffer: Vec<u8>, // Stores pixel data in RGBA format
    color: [u8; 4],  // Current drawing color
}

impl Canvas {
    /// Create a new canvas with the specified width, height, and color
    pub fn new(width: u32, height: u32, color: SlintColor) -> Self {
        // RGBA format, initialized to transparent black
        let buffer = vec![0; (width * height * 4) as usize];
        Self {
            width,
            height,
            buffer,
            color: match color {
                SlintColor::Red => [255u8, 0u8, 0u8, 255u8],
                SlintColor::Blue => [0u8, 0u8, 255u8, 255u8],
            },
        }
    }


    /// Set a pixel at (x, y) to the current color
    fn set_pixel(&mut self, x: u32, y: u32) {
        if x >= self.width || y >= self.height {
            return;
        }
        let index = ((y * self.width + x) * 4) as usize;
        self.buffer[index..index + 4].copy_from_slice(&self.color);
    }

    /// Draw a line using Bresenham's algorithm
    fn draw_line(&mut self, x0: i32, y0: i32, x1: i32, y1: i32) {
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;

        let (mut x, mut y) = (x0, y0);
        while x != x1 || y != y1 {
            self.set_pixel(x as u32, y as u32);
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }

    pub fn draw_coords(&mut self, coords: &[Vec2<f64>]) {
        let mut prev: Option<(i32, i32)> = None;

        for coord in coords {
            let current = (coord.x() as i32, coord.y() as i32);
            if let Some(prev) = prev {
                self.draw_line(prev.0, prev.1, current.0, current.1);
            }
            prev = Some(current);
        }
    }

    /// Converts the pixel buffer to an image format that Slint can display
    pub fn to_image(&self) -> Image {
        let mut pixel_buffer = SharedPixelBuffer::new(self.width, self.height);

        let buffer = pixel_buffer.make_mut_bytes();
        buffer.copy_from_slice(&self.buffer);

        Image::from_rgba8(pixel_buffer)
    }
}
