//! Screenshot mode: the window saves a picture of its own rendering and closes.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use eframe::egui::{self, ColorImage};

/// How long to wait for the picture after it was asked for.
const PICTURE_TIMEOUT: Duration = Duration::from_secs(5);
const POLL: Duration = Duration::from_millis(50);

/// Filled once: `Ok` when the file was written, `Err` with the reason otherwise.
pub type Outcome = Arc<Mutex<Option<Result<(), String>>>>;

pub struct Screenshot {
    path: PathBuf,
    after: Duration,
    started: Instant,
    slot: Arc<Mutex<Option<Arc<ColorImage>>>>,
    asked: Option<Instant>,
    finished: bool,
    outcome: Outcome,
}

impl Screenshot {
    pub fn new(path: PathBuf, after: Duration, outcome: Outcome) -> Self {
        Screenshot {
            path,
            after,
            started: Instant::now(),
            slot: Arc::new(Mutex::new(None)),
            asked: None,
            finished: false,
            outcome,
        }
    }

    /// Call once per frame.
    pub fn tick(&mut self, ctx: &egui::Context) {
        if self.finished {
            return;
        }
        let Some(asked) = self.asked else {
            let elapsed = self.started.elapsed();
            if elapsed >= self.after {
                let slot = Arc::clone(&self.slot);
                ctx.request_screenshot(move |image| {
                    *slot.lock().unwrap_or_else(|e| e.into_inner()) = Some(image);
                });
                self.asked = Some(Instant::now());
                ctx.request_repaint();
            } else {
                ctx.request_repaint_after((self.after - elapsed).min(POLL));
            }
            return;
        };
        let image = self.slot.lock().unwrap_or_else(|e| e.into_inner()).take();
        let result = match image {
            Some(image) => write_png(&self.path, &image),
            None if asked.elapsed() > PICTURE_TIMEOUT => {
                Err("the window did not deliver a picture".to_string())
            }
            None => {
                ctx.request_repaint_after(POLL);
                return;
            }
        };
        *self.outcome.lock().unwrap_or_else(|e| e.into_inner()) = Some(result);
        self.finished = true;
        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
    }
}

pub fn write_png(path: &Path, image: &ColorImage) -> Result<(), String> {
    let fail = |e: &dyn std::fmt::Display| format!("cannot write {}: {e}", path.display());
    let mut rgba = Vec::with_capacity(image.pixels.len() * 4);
    for pixel in &image.pixels {
        rgba.extend_from_slice(&[pixel.r(), pixel.g(), pixel.b(), pixel.a()]);
    }
    let file = std::fs::File::create(path).map_err(|e| fail(&e))?;
    let mut encoder = png::Encoder::new(
        std::io::BufWriter::new(file),
        image.size[0] as u32,
        image.size[1] as u32,
    );
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().map_err(|e| fail(&e))?;
    writer.write_image_data(&rgba).map_err(|e| fail(&e))?;
    writer.finish().map_err(|e| fail(&e))
}
