//! MARV ground control station: the desktop window.
//!
//! Layer rule: only `gcs-link` and the applications (this crate, `marv-cli`) depend on both
//! `gcs-transport` and `gcs-mavlink`; only this crate depends on `eframe`, `walkers` and `rfd`.

use std::sync::{Arc, Mutex};

use eframe::egui;

pub mod app;
pub mod cli;
pub mod connection;
pub mod link_test;
pub mod logfile;
pub mod screenshot;
pub mod sessions;
pub mod status;
pub mod views;

/// Open the window and run it until it closes. Returns the process exit status: 0, or 1 when the
/// window cannot be created or the requested screenshot could not be taken or written.
pub fn run(cli: cli::Cli) -> i32 {
    let outcome: screenshot::Outcome = Arc::new(Mutex::new(None));
    let wants_screenshot = cli.screenshot.is_some();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("MARV GCS")
            .with_inner_size([1100.0, 700.0]),
        ..Default::default()
    };
    let result = eframe::run_native(
        "MARV GCS",
        options,
        Box::new({
            let outcome = Arc::clone(&outcome);
            move |cc| Ok(Box::new(app::App::new(cc, cli, outcome)))
        }),
    );
    if let Err(e) = result {
        eprintln!("marv-gcs: cannot run the window: {e}");
        return 1;
    }
    if !wants_screenshot {
        return 0;
    }
    let result = outcome.lock().unwrap_or_else(|e| e.into_inner()).take();
    match result {
        Some(Ok(())) => 0,
        Some(Err(e)) => {
            eprintln!("marv-gcs: screenshot failed: {e}");
            1
        }
        None => {
            eprintln!(
                "marv-gcs: screenshot failed: the window closed before the picture was taken"
            );
            1
        }
    }
}
