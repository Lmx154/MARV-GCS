//! Plot helpers shared by the views.

use std::collections::VecDeque;

use eframe::egui::{self, AsIdSalt, Color32};
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
    let labels = WINDOWS.map(|w| format!("{w} s"));
    let choices: Vec<(u32, &str)> = WINDOWS
        .iter()
        .zip(&labels)
        .map(|(w, label)| (*w, label.as_str()))
        .collect();
    ui.horizontal(|ui| {
        ui.label(crate::theme::muted("Window"));
        crate::theme::segmented(ui, &mut window, &choices);
    });
    ui.data_mut(|d| d.insert_temp(id, window));
    f64::from(window)
}

/// A line within this many screen pixels (vertically) of the pointer is marked in the readout.
pub const NEAR_PX: f64 = 6.0;

/// The colour of the `index`-th line: the theme's plot palette. The lines are given it explicitly so
/// the readout's swatch is the line's colour by construction.
fn auto_color(index: usize) -> Color32 {
    crate::theme::series(index)
}

/// One series as the readout sees it: the legend name, the line colour and the points `[t, value]`
/// in increasing time.
pub struct ReadoutSeries<'a> {
    pub name: &'a str,
    pub color: Color32,
    pub points: &'a VecDeque<[f64; 2]>,
}

/// One row of the readout.
#[derive(Debug, Clone, PartialEq)]
pub struct Row {
    pub name: String,
    pub color: Color32,
    /// The value of the sample nearest in time to the pointer (not interpolated).
    pub value: f64,
    /// The line is within [`NEAR_PX`] of the pointer, vertically, at the pointer's time.
    pub near: bool,
}

/// The index of the point nearest in time to `t` (binary search); `None` for no point.
fn nearest_index(points: &VecDeque<[f64; 2]>, t: f64) -> Option<usize> {
    let after = points.partition_point(|p| p[0] < t);
    match (
        after.checked_sub(1),
        (after < points.len()).then_some(after),
    ) {
        (Some(b), Some(a)) => Some(if t - points[b][0] <= points[a][0] - t {
            b
        } else {
            a
        }),
        (Some(i), None) | (None, Some(i)) => Some(i),
        (None, None) => None,
    }
}

/// The height of the drawn line at time `t`: straight between the two samples around it; `None`
/// before the first or after the last sample, where the line does not exist.
fn line_value_at(points: &VecDeque<[f64; 2]>, t: f64) -> Option<f64> {
    let after = points.partition_point(|p| p[0] < t);
    let next = points.get(after)?;
    if next[0] == t {
        return Some(next[1]);
    }
    let prev = points.get(after.checked_sub(1)?)?;
    let f = (t - prev[0]) / (next[0] - prev[0]);
    Some(prev[1] + f * (next[1] - prev[1]))
}

/// The readout at time `t` with the pointer at height `pointer_y` (plot units; `px_per_unit` screen
/// pixels per unit vertically): a row per series with a finite sample, the value of the sample nearest
/// in time, highest value first as the lines are drawn. A row is `near` when the drawn line passes
/// within [`NEAR_PX`] of the pointer.
pub fn readout(series: &[ReadoutSeries<'_>], t: f64, pointer_y: f64, px_per_unit: f64) -> Vec<Row> {
    let mut rows: Vec<Row> = series
        .iter()
        .filter_map(|s| {
            let value = s.points[nearest_index(s.points, t)?][1];
            value.is_finite().then_some(())?;
            let near = line_value_at(s.points, t)
                .is_some_and(|y| ((y - pointer_y) * px_per_unit).abs() <= NEAR_PX);
            Some(Row {
                name: s.name.to_string(),
                color: s.color,
                value,
                near,
            })
        })
        .collect();
    rows.sort_by(|a, b| b.value.total_cmp(&a.value));
    rows
}

fn show_readout(ui: &egui::Ui, response: &egui::Response, rows: &[Row], t: f64, unit: &str) {
    egui::Tooltip::always_open(
        ui.ctx().clone(),
        response.layer_id,
        response.id,
        egui::PopupAnchor::Pointer,
    )
    .gap(12.0)
    .show(|ui| {
        ui.label(format!("t = {t:.2} s"));
        egui::Grid::new("plot_readout")
            .num_columns(3)
            .spacing([6.0, 2.0])
            .show(ui, |ui| {
                for row in rows {
                    let (rect, _) =
                        ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
                    ui.painter().rect_filled(rect, 2.0, row.color);
                    if row.near {
                        ui.painter().rect_stroke(
                            rect.expand(2.0),
                            2.0,
                            egui::Stroke::new(1.5, ui.visuals().strong_text_color()),
                            egui::StrokeKind::Outside,
                        );
                        ui.strong(&row.name);
                        ui.strong(format!("{:.4} {unit}", row.value));
                    } else {
                        ui.label(&row.name);
                        ui.label(format!("{:.4} {unit}", row.value));
                    }
                    ui.end_row();
                }
            });
    });
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
    draw_inner(ui, id_salt, height, window, traces, None);
}

/// [`draw`] with a hover readout: while the pointer is over the plot, a popup near it lists, for every
/// series, the value at the pointer's time (`unit` after it), and marks the lines under the pointer.
pub fn draw_with_readout(
    ui: &mut egui::Ui,
    id_salt: impl AsIdSalt,
    height: f32,
    window: f64,
    traces: &[(String, &Series)],
    unit: &str,
) {
    draw_inner(ui, id_salt, height, window, traces, Some(unit));
}

fn draw_inner(
    ui: &mut egui::Ui,
    id_salt: impl AsIdSalt,
    height: f32,
    window: f64,
    traces: &[(String, &Series)],
    unit: Option<&str>,
) {
    let newest = traces
        .iter()
        .filter_map(|(_, s)| s.points.back().map(|p| p[0]))
        .fold(f64::NEG_INFINITY, f64::max);
    let newest = if newest.is_finite() { newest } else { window };
    let start = newest - window;

    let mut lines = Vec::with_capacity(traces.len());
    let mut shown = Vec::with_capacity(traces.len());
    for (name, series) in traces {
        let first = series.points.partition_point(|p| p[0] < start);
        let points: Vec<[f64; 2]> = series
            .points
            .range(first..)
            .filter(|p| p[0].is_finite() && p[1].is_finite())
            .copied()
            .collect();
        if points.is_empty() {
            continue;
        }
        let color = auto_color(shown.len());
        shown.push(ReadoutSeries {
            name,
            color,
            points: &series.points,
        });
        lines.push(Line::new(name.clone(), points).color(color));
    }

    let plot = Plot::new(id_salt)
        .height(height)
        .legend(Legend::default().position(Corner::LeftTop))
        .allow_drag(false)
        .allow_zoom(false)
        .allow_scroll(false)
        .allow_boxed_zoom(false)
        .allow_double_click_reset(false);
    let shown_plot = plot.show(ui, move |plot_ui| {
        plot_ui.set_plot_bounds_x(start..=newest);
        for line in lines {
            plot_ui.line(line);
        }
    });
    if let (Some(unit), Some(pointer)) = (unit, shown_plot.response.hover_pos()) {
        let at = shown_plot.transform.value_from_position(pointer);
        let rows = readout(
            &shown,
            at.x,
            at.y,
            shown_plot.transform.dpos_dvalue_y().abs(),
        );
        if !rows.is_empty() {
            show_readout(ui, &shown_plot.response, &rows, at.x, unit);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pts(points: &[[f64; 2]]) -> VecDeque<[f64; 2]> {
        points.iter().copied().collect()
    }

    fn series<'a>(name: &'a str, points: &'a VecDeque<[f64; 2]>) -> ReadoutSeries<'a> {
        ReadoutSeries {
            name,
            color: Color32::RED,
            points,
        }
    }

    fn values(rows: &[Row]) -> Vec<(&str, f64)> {
        rows.iter().map(|r| (r.name.as_str(), r.value)).collect()
    }

    #[test]
    fn the_value_is_that_of_the_nearest_sample_not_an_interpolation() {
        let a = pts(&[[1.0, 10.0], [2.0, 20.0], [4.0, 40.0]]);
        let s = [series("a", &a)];
        assert_eq!(values(&readout(&s, 1.4, 0.0, 1.0)), [("a", 10.0)]);
        assert_eq!(values(&readout(&s, 1.6, 0.0, 1.0)), [("a", 20.0)]);
        assert_eq!(values(&readout(&s, 2.9, 0.0, 1.0)), [("a", 20.0)]);
        assert_eq!(values(&readout(&s, 3.1, 0.0, 1.0)), [("a", 40.0)]);
        assert_eq!(values(&readout(&s, 2.0, 0.0, 1.0)), [("a", 20.0)]);
    }

    #[test]
    fn before_the_first_and_after_the_last_sample_the_end_sample_is_taken() {
        let a = pts(&[[1.0, 10.0], [2.0, 20.0]]);
        let s = [series("a", &a)];
        assert_eq!(values(&readout(&s, -50.0, 0.0, 1.0)), [("a", 10.0)]);
        assert_eq!(values(&readout(&s, 99.0, 0.0, 1.0)), [("a", 20.0)]);
    }

    #[test]
    fn each_series_has_its_own_nearest_sample_and_rows_go_by_value_downwards() {
        let low = pts(&[[0.0, -3.0], [10.0, -3.5]]);
        let high = pts(&[[0.0, 5.0], [6.0, 7.0]]);
        let mid = pts(&[[4.0, 1.0], [5.0, 2.0]]);
        let s = [
            series("low", &low),
            series("high", &high),
            series("mid", &mid),
        ];
        let rows = readout(&s, 5.2, 0.0, 1.0);
        assert_eq!(values(&rows), [("high", 7.0), ("mid", 2.0), ("low", -3.5)]);
        assert!(rows.iter().all(|r| r.color == Color32::RED));
    }

    #[test]
    fn empty_series_and_non_finite_samples_give_no_row() {
        let none = pts(&[]);
        let nan = pts(&[[1.0, f64::NAN]]);
        let ok = pts(&[[1.0, 2.0]]);
        let s = [
            series("none", &none),
            series("nan", &nan),
            series("ok", &ok),
        ];
        assert_eq!(values(&readout(&s, 1.0, 0.0, 1.0)), [("ok", 2.0)]);
    }

    #[test]
    fn lines_within_six_pixels_of_the_pointer_are_marked_and_a_distant_one_is_not() {
        let a = pts(&[[0.0, 1.00], [10.0, 1.00]]);
        let b = pts(&[[0.0, 1.04], [10.0, 1.04]]);
        let far = pts(&[[0.0, 3.0], [10.0, 3.0]]);
        let s = [series("a", &a), series("b", &b), series("far", &far)];
        // 100 px per unit: a is 2 px and b 2 px from the pointer at 1.02, far 198 px.
        let rows = readout(&s, 5.0, 1.02, 100.0);
        let near: Vec<(&str, bool)> = rows.iter().map(|r| (r.name.as_str(), r.near)).collect();
        assert_eq!(near, [("far", false), ("b", true), ("a", true)]);
        // 7 px away from a: not marked; 5.5 px: marked.
        assert!(
            !readout(&s, 5.0, 1.07, 100.0)
                .iter()
                .any(|r| r.name == "a" && r.near)
        );
        assert!(
            readout(&s, 5.0, 1.055, 100.0)
                .iter()
                .any(|r| r.name == "a" && r.near)
        );
    }

    #[test]
    fn the_marking_follows_the_drawn_line_between_samples_and_not_beyond_its_ends() {
        let a = pts(&[[0.0, 0.0], [10.0, 10.0]]);
        let s = [series("a", &a)];
        // At t = 2.5 the line is at 2.5 though the nearest sample is 0.
        let rows = readout(&s, 2.5, 2.5, 100.0);
        assert!(rows[0].near);
        assert_eq!(rows[0].value, 0.0);
        let rows = readout(&s, 2.5, 0.0, 100.0);
        assert!(!rows[0].near);
        // After the last sample there is no line: not marked, the last value still shown.
        let rows = readout(&s, 12.0, 10.0, 100.0);
        assert!(!rows[0].near);
        assert_eq!(rows[0].value, 10.0);
    }

    #[test]
    fn a_long_series_gives_the_right_sample() {
        let long: VecDeque<[f64; 2]> = (0..1_000_000)
            .map(|i| [f64::from(i), f64::from(i)])
            .collect();
        let s = [series("long", &long)];
        assert_eq!(
            values(&readout(&s, 654_321.4, 0.0, 1.0)),
            [("long", 654_321.0)]
        );
    }
}
