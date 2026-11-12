//! Sensors view: preset live plots for the board's sensors, and one line of GPS text.

use eframe::egui;
use gcs_mavlink::services::messages::MessageStore;

use super::plot;
use super::{View, ViewCtx};

/// One plot: the fields of one message instance. Adding a plot is adding a row.
struct Preset {
    title: &'static str,
    message: &'static str,
    instance: u8,
    fields: &'static [&'static str],
}

const PRESETS: &[Preset] = &[
    Preset {
        title: "Main IMU acceleration (m/s²)",
        message: "HIGHRES_IMU",
        instance: 0,
        fields: &["xacc", "yacc", "zacc"],
    },
    Preset {
        title: "Main IMU rotation (rad/s)",
        message: "HIGHRES_IMU",
        instance: 0,
        fields: &["xgyro", "ygyro", "zgyro"],
    },
    Preset {
        title: "Second IMU acceleration (m/s²)",
        message: "HIGHRES_IMU",
        instance: 1,
        fields: &["xacc", "yacc", "zacc"],
    },
    Preset {
        title: "Second IMU rotation (rad/s)",
        message: "HIGHRES_IMU",
        instance: 1,
        fields: &["xgyro", "ygyro", "zgyro"],
    },
    Preset {
        title: "High-g acceleration (m/s²)",
        message: "HIGHRES_IMU",
        instance: 2,
        fields: &["xacc", "yacc", "zacc"],
    },
    Preset {
        title: "Magnetic field (gauss)",
        message: "HIGHRES_IMU",
        instance: 3,
        fields: &["xmag", "ymag", "zmag"],
    },
    Preset {
        title: "Pressure (hPa)",
        message: "SCALED_PRESSURE",
        instance: 0,
        fields: &["press_abs"],
    },
];

const PLOT_HEIGHT: f32 = 170.0;

#[derive(Default)]
pub struct Sensors;

impl View for Sensors {
    fn name(&self) -> &'static str {
        "Sensors"
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let store = &cx.state.messages;
        gps_line(ui, store);
        let window = plot::window_selector(ui, "sensors");
        ui.separator();
        egui::ScrollArea::vertical().show(ui, |ui| {
            for pair in PRESETS.chunks(2) {
                ui.columns(2, |columns| {
                    for (ui, preset) in columns.iter_mut().zip(pair) {
                        draw_preset(ui, store, preset, window);
                    }
                });
            }
        });
    }
}

fn draw_preset(ui: &mut egui::Ui, store: &MessageStore, preset: &Preset, window: f64) {
    ui.strong(preset.title);
    let traces: Vec<_> = preset
        .fields
        .iter()
        .filter_map(|field| {
            store
                .series(preset.message, preset.instance, field)
                .map(|series| ((*field).to_string(), series))
        })
        .collect();
    if traces.is_empty() {
        ui.label("no data");
    } else {
        plot::draw(
            ui,
            ("sensors_plot", preset.title),
            PLOT_HEIGHT,
            window,
            &traces,
        );
    }
}

fn gps_line(ui: &mut egui::Ui, store: &MessageStore) {
    let Some(gps) = store.entry("GPS_RAW_INT", 0) else {
        ui.label("GPS: no data");
        return;
    };
    let fix = match gps.text("fix_type") {
        Some(text) => text.to_string(),
        None => number(gps.number("fix_type"), |n| format!("{n}")),
    };
    let satellites = number(gps.number("satellites_visible"), |n| format!("{n}"));
    let latitude = number(gps.number("lat"), |n| format!("{:.7}°", n / 1e7));
    let longitude = number(gps.number("lon"), |n| format!("{:.7}°", n / 1e7));
    let altitude = number(gps.number("alt"), |n| format!("{:.1} m", n / 1e3));
    ui.horizontal(|ui| {
        ui.strong("GPS");
        ui.label(format!("fix {fix}"));
        ui.label(format!("{satellites} satellites"));
        ui.label(format!("lat {latitude}"));
        ui.label(format!("lon {longitude}"));
        ui.label(format!("alt {altitude}"));
    });
}

fn number(value: Option<f64>, format: impl Fn(f64) -> String) -> String {
    value.map_or_else(|| "—".to_string(), format)
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use super::PRESETS;

    #[test]
    fn every_preset_has_a_unique_title_and_a_field() {
        let mut titles = HashSet::new();
        for preset in PRESETS {
            assert!(
                titles.insert(preset.title),
                "duplicate title {}",
                preset.title
            );
            assert!(!preset.fields.is_empty(), "{} has no field", preset.title);
            assert!(!preset.message.is_empty());
        }
    }
}
