//! Plot helpers shared by the views.

use eframe::egui::{self, AsIdSalt};
use egui_plot::{Corner, Legend, Line, Plot};
use gcs_mavlink::services::messages::Series;

/// The time windows the user can choose, in seconds.
pub const WINDOWS: [u32; 3] = [10, 30, 60];
pub const DEFAULT_WINDOW: u32 = 30;

/// A row of buttons that picks the time window; the choice lives in the egui memory under `id_salt`.
/// Returns the window in seconds.
pub fn window_selector(ui: &mut egui::Ui, id_salt: impl AsIdSalt) -> f64 {
    let id = ui.make_persistent_id(("plot_window", id_salt));
    let mut window = ui.data(|d| d.get_temp::<u32>(id)).unwrap_or(DEFAULT_WINDOW);
    ui.horizontal(|ui| {
        ui.label("Window");
        for w in WINDOWS {
            ui.selectable_value(&mut window, w, format!("{w} s"));
        }
    });
    ui.data_mut(|d| d.insert_temp(id, window));
    f64::from(window)
}

/// Draws named series over the last `window` seconds before the newest point of any of them.
/// Only the points inside the window are copied; points that are not finite are skipped.
pub fn draw(
    ui: &mut egui::Ui,
    id_salt: impl AsIdSalt,
    height: f32,
    window: f64,
    traces: &[(String, &Series)],
) {
    let newest = traces
        .iter()
        .filter_map(|(_, s)| s.points.back().map(|p| p[0]))
        .fold(f64::NEG_INFINITY, f64::max);
    let newest = if newest.is_finite() { newest } else { window };
    let start = newest - window;

    let mut lines = Vec::with_capacity(traces.len());
    for (name, series) in traces {
        let first = series.points.partition_point(|p| p[0] < start);
        let points: Vec<[f64; 2]> = series
            .points
            .range(first..)
            .filter(|p| p[0].is_finite() && p[1].is_finite())
            .copied()
            .collect();
        lines.push(Line::new(name.clone(), points));
    }

    let plot = Plot::new(id_salt)
        .height(height)
        .legend(Legend::default().position(Corner::LeftTop))
        .allow_drag(false)
        .allow_zoom(false)
        .allow_scroll(false)
        .allow_boxed_zoom(false)
        .allow_double_click_reset(false);
    plot.show(ui, move |plot_ui| {
        plot_ui.set_plot_bounds_x(start..=newest);
        for line in lines {
            plot_ui.line(line);
        }
    });
}
