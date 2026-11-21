//! Sensors view: preset live plots for the board's sensors, and one line of GPS text.

use std::collections::VecDeque;

use eframe::egui;
use gcs_mavlink::services::messages::{MessageStore, Series};

use super::plot;
use super::{View, ViewCtx};
use crate::{sessions, theme};

/// One plot: the fields of one message instance, each value times `scale`, a raw value in
/// `unavailable` left out (the message's "not available"). Adding a plot is adding a row. Every row
/// names a message and fields the real firmware sends on its USB port (`MARV-V2-firmware`, `src/main.rs`).
pub struct Preset {
    pub title: &'static str,
    pub message: &'static str,
    pub instance: u8,
    pub fields: &'static [&'static str],
    /// Multiplied into every value: the conversion to the unit in the title.
    pub scale: f64,
    /// Raw values that mean "not available" (SYS_STATUS: 65535 mV, -1 cA).
    pub unavailable: &'static [f64],
}

/// Standard gravity as the firmware uses it (m/s² per g); SCALED_IMU2 is in milli-g.
const MS2_PER_MG: f64 = 9.80665 / 1000.0;
/// rad/s per mrad/s (SCALED_IMU2's rotation is in mrad/s).
const RADS_PER_MRADS: f64 = 0.001;

pub const PRESETS: &[Preset] = &[
    Preset {
        title: "IMU1 acceleration (m/s²)",
        message: "HIGHRES_IMU",
        instance: 0,
        fields: &["xacc", "yacc", "zacc"],
        scale: 1.0,
        unavailable: &[],
    },
    Preset {
        title: "IMU1 rotation (rad/s)",
        message: "HIGHRES_IMU",
        instance: 0,
        fields: &["xgyro", "ygyro", "zgyro"],
        scale: 1.0,
        unavailable: &[],
    },
    Preset {
        title: "IMU2 acceleration (m/s², SCALED_IMU2 mG converted)",
        message: "SCALED_IMU2",
        instance: 0,
        fields: &["xacc", "yacc", "zacc"],
        scale: MS2_PER_MG,
        unavailable: &[],
    },
    Preset {
        title: "IMU2 rotation (rad/s, SCALED_IMU2 mrad/s converted)",
        message: "SCALED_IMU2",
        instance: 0,
        fields: &["xgyro", "ygyro", "zgyro"],
        scale: RADS_PER_MRADS,
        unavailable: &[],
    },
    Preset {
        title: "Magnetic field (gauss, HIGHRES_IMU)",
        message: "HIGHRES_IMU",
        instance: 0,
        fields: &["xmag", "ymag", "zmag"],
        scale: 1.0,
        unavailable: &[],
    },
    Preset {
        title: "Pressure (hPa)",
        message: "SCALED_PRESSURE",
        instance: 0,
        fields: &["press_abs"],
        scale: 1.0,
        unavailable: &[],
    },
    Preset {
        title: "Battery voltage (V, SYS_STATUS mV converted)",
        message: "SYS_STATUS",
        instance: 0,
        fields: &["voltage_battery"],
        scale: 0.001,
        unavailable: &[65_535.0],
    },
    Preset {
        title: "Battery current (A, SYS_STATUS cA converted)",
        message: "SYS_STATUS",
        instance: 0,
        fields: &["current_battery"],
        scale: 0.01,
        unavailable: &[-1.0],
    },
];

const PLOT_HEIGHT: f32 = 170.0;

#[derive(Default)]
pub struct Sensors {
    session: sessions::BarInput,
}

impl View for Sensors {
    fn name(&self) -> &'static str {
        "Sensors"
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let store = &cx.state.messages;
        let window = theme::card()
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                sessions::bar_ui(ui, &mut self.session);
                ui.separator();
                gps_line(ui, store);
                ui.label(theme::value(battery_text(store)));
                plot::window_selector(ui, "sensors")
            })
            .inner;
        ui.add_space(theme::GAP);
        for pair in PRESETS.chunks(2) {
            ui.columns(2, |columns| {
                for (ui, preset) in columns.iter_mut().zip(pair) {
                    theme::card().show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        draw_preset(ui, store, preset, window);
                    });
                }
            });
            ui.add_space(theme::GAP);
        }
    }
}

/// The traces of a preset over the last `window` seconds: values converted, "not available" and
/// non-finite points left out. A field with no point at all gives no trace.
pub fn preset_traces(store: &MessageStore, preset: &Preset, window: f64) -> Vec<(String, Series)> {
    preset
        .fields
        .iter()
        .filter_map(|field| {
            let series = store.series(preset.message, preset.instance, field)?;
            let newest = series.points.back()?[0];
            let points: VecDeque<[f64; 2]> = series
                .points
                .iter()
                .filter(|p| p[0] >= newest - window && !preset.unavailable.contains(&p[1]))
                .map(|p| [p[0], p[1] * preset.scale])
                .filter(|p| p[1].is_finite())
                .collect();
            (!points.is_empty()).then(|| ((*field).to_string(), Series { points }))
        })
        .collect()
}

/// For each preset its title and the number of points of its traces over the whole history kept: a
/// preset whose message the vehicle does not send has 0.
pub fn preset_sample_counts(store: &MessageStore) -> Vec<(&'static str, usize)> {
    PRESETS
        .iter()
        .map(|preset| {
            let n = preset_traces(store, preset, f64::INFINITY)
                .iter()
                .map(|(_, series)| series.points.len())
                .sum();
            (preset.title, n)
        })
        .collect()
}

fn draw_preset(ui: &mut egui::Ui, store: &MessageStore, preset: &Preset, window: f64) {
    theme::section_title(ui, preset.title);
    let owned = preset_traces(store, preset, window);
    let traces: Vec<(String, &Series)> = owned.iter().map(|(n, s)| (n.clone(), s)).collect();
    if traces.is_empty() {
        ui.allocate_ui(egui::vec2(ui.available_width(), PLOT_HEIGHT), |ui| {
            ui.centered_and_justified(|ui| ui.label(theme::muted("no data")));
        });
    } else {
        plot::draw_with_readout(
            ui,
            ("sensors_plot", preset.title),
            PLOT_HEIGHT,
            window,
            &traces,
            title_unit(preset.title),
        );
    }
}

/// The unit in a plot title: the first word inside the parentheses ("IMU1 rotation (rad/s)" gives
/// "rad/s", "Magnetic field (gauss, HIGHRES_IMU)" gives "gauss").
pub fn title_unit(title: &str) -> &str {
    let inside = title.split_once('(').map_or("", |(_, rest)| rest);
    inside.split([',', ')']).next().unwrap_or("").trim()
}

/// The battery in one line: volts and amps from SYS_STATUS, or "not available" (65535 mV, -1 cA).
pub fn battery_text(store: &MessageStore) -> String {
    let Some(entry) = store.entry("SYS_STATUS", 0) else {
        return "Battery: no data".to_string();
    };
    let volts = entry
        .number("voltage_battery")
        .filter(|v| *v != 65_535.0)
        .map_or_else(
            || "not available".to_string(),
            |v| format!("{:.2} V", v / 1000.0),
        );
    let amps = entry
        .number("current_battery")
        .filter(|v| *v != -1.0)
        .map_or_else(
            || "not available".to_string(),
            |v| format!("{:.2} A", v / 100.0),
        );
    format!("Battery: {volts}, {amps}")
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
    ui.horizontal_wrapped(|ui| {
        ui.strong("GPS");
        ui.label(theme::value(format!("fix {fix}")));
        ui.label(theme::value(format!("{satellites} satellites")));
        ui.label(theme::value(format!("lat {latitude}")));
        ui.label(theme::value(format!("lon {longitude}")));
        ui.label(theme::value(format!("alt {altitude}")));
    });
}

fn number(value: Option<f64>, format: impl Fn(f64) -> String) -> String {
    value.map_or_else(|| "—".to_string(), format)
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;
    use std::time::Duration;

    use gcs_mavlink::services::messages::{MessageEntry, MessageKey, SeriesKey};

    use super::*;

    /// The messages and fields the real firmware sends on USB (`push_*` and `Board::sample` of its
    /// `src/main.rs`; `rc.rs`), as far as a preset may use them.
    const FIRMWARE: &[(&str, &[&str])] = &[
        (
            "HIGHRES_IMU",
            &[
                "xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag",
            ],
        ),
        (
            "SCALED_IMU2",
            &["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro"],
        ),
        ("SCALED_PRESSURE", &["press_abs"]),
        ("SYS_STATUS", &["voltage_battery", "current_battery"]),
    ];

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

    #[test]
    fn every_preset_is_a_message_and_field_the_firmware_sends() {
        for preset in PRESETS {
            let (_, fields) = FIRMWARE
                .iter()
                .find(|(message, _)| *message == preset.message)
                .unwrap_or_else(|| {
                    panic!("{}: the firmware sends no {}", preset.title, preset.message)
                });
            assert_eq!(
                preset.instance, 0,
                "{}: only IMU1 is HIGHRES_IMU",
                preset.title
            );
            for field in preset.fields {
                assert!(fields.contains(field), "{}: {field}", preset.title);
            }
        }
        assert!(PRESETS.iter().all(|p| !p.title.contains("High-g")));
        for needed in [
            "Battery voltage",
            "Battery current",
            "Magnetic field",
            "IMU2 acceleration",
        ] {
            assert!(
                PRESETS.iter().any(|p| p.title.starts_with(needed)),
                "{needed}"
            );
        }
    }

    #[test]
    fn every_preset_title_names_its_unit() {
        let units: Vec<&str> = PRESETS.iter().map(|p| title_unit(p.title)).collect();
        assert_eq!(
            units,
            ["m/s²", "rad/s", "m/s²", "rad/s", "gauss", "hPa", "V", "A"]
        );
        assert_eq!(title_unit("no unit"), "");
    }

    fn put(store: &mut MessageStore, message: &str, field: &str, points: &[[f64; 2]]) {
        store.series.insert(
            SeriesKey {
                system_id: 1,
                component_id: 1,
                message: message.into(),
                instance: 0,
                field: field.into(),
            },
            Series {
                points: points.iter().copied().collect(),
            },
        );
    }

    fn preset(title_start: &str) -> &'static Preset {
        PRESETS
            .iter()
            .find(|p| p.title.starts_with(title_start))
            .unwrap()
    }

    fn values(traces: &[(String, Series)], field: &str) -> Vec<f64> {
        traces
            .iter()
            .find(|(n, _)| n == field)
            .map(|(_, s)| s.points.iter().map(|p| p[1]).collect())
            .unwrap_or_default()
    }

    #[test]
    fn imu2_is_converted_from_milli_g_and_milli_rad_per_second() {
        let mut store = MessageStore::default();
        put(
            &mut store,
            "SCALED_IMU2",
            "zacc",
            &[[1.0, -1000.0], [2.0, 500.0]],
        );
        put(
            &mut store,
            "SCALED_IMU2",
            "xgyro",
            &[[1.0, 1000.0], [2.0, -250.0]],
        );
        let acc = preset_traces(&store, preset("IMU2 acceleration"), 30.0);
        let z = values(&acc, "zacc");
        assert!(
            (z[0] + 9.80665).abs() < 1e-9 && (z[1] - 4.903325).abs() < 1e-9,
            "{z:?}"
        );
        let gyro = preset_traces(&store, preset("IMU2 rotation"), 30.0);
        assert_eq!(values(&gyro, "xgyro"), [1.0, -0.25]);
        assert!(preset("IMU2 acceleration").title.contains("mG converted"));
    }

    #[test]
    fn imu1_and_the_magnetometer_are_not_converted() {
        let mut store = MessageStore::default();
        put(&mut store, "HIGHRES_IMU", "zacc", &[[1.0, -9.81]]);
        put(&mut store, "HIGHRES_IMU", "xmag", &[[1.0, 0.21]]);
        assert_eq!(
            values(
                &preset_traces(&store, preset("IMU1 acceleration"), 30.0),
                "zacc"
            ),
            [-9.81]
        );
        assert_eq!(
            values(
                &preset_traces(&store, preset("Magnetic field"), 30.0),
                "xmag"
            ),
            [0.21]
        );
    }

    #[test]
    fn the_battery_is_in_volts_and_amps_and_not_available_is_left_out() {
        let mut store = MessageStore::default();
        put(
            &mut store,
            "SYS_STATUS",
            "voltage_battery",
            &[[1.0, 15_980.0], [2.0, 65_535.0], [3.0, 16_000.0]],
        );
        put(
            &mut store,
            "SYS_STATUS",
            "current_battery",
            &[[1.0, 120.0], [2.0, -1.0]],
        );
        let v = preset_traces(&store, preset("Battery voltage"), 30.0);
        assert_eq!(values(&v, "voltage_battery"), [15.98, 16.0]);
        let a = preset_traces(&store, preset("Battery current"), 30.0);
        assert_eq!(values(&a, "current_battery"), [1.2]);
    }

    #[test]
    fn only_the_window_is_kept_and_a_missing_message_has_no_trace() {
        let mut store = MessageStore::default();
        put(
            &mut store,
            "SCALED_PRESSURE",
            "press_abs",
            &[[1.0, 1000.0], [50.0, 1001.0], [60.0, 1002.0]],
        );
        let p = preset_traces(&store, preset("Pressure"), 5.0);
        assert_eq!(values(&p, "press_abs"), [1002.0]);
        assert!(preset_traces(&store, preset("IMU1 rotation"), 10.0).is_empty());
        let counts = preset_sample_counts(&store);
        assert_eq!(counts.len(), PRESETS.len());
        assert_eq!(counts.iter().filter(|(_, n)| *n > 0).count(), 1);
    }

    fn sys_status(voltage: f64, current: f64) -> MessageStore {
        let mut store = MessageStore::default();
        let number = |name: &str, v: f64| gcs_mavlink::services::messages::Field {
            name: name.into(),
            value: gcs_mavlink::services::messages::FieldValue::Number(v),
        };
        store.entries.insert(
            MessageKey {
                system_id: 1,
                component_id: 1,
                message_id: 1,
                instance: 0,
            },
            MessageEntry {
                name: "SYS_STATUS".into(),
                label: "SYS_STATUS".into(),
                count: 1,
                rate_hz: 1.0,
                last_seen: Duration::from_secs(1),
                fields: vec![
                    number("voltage_battery", voltage),
                    number("current_battery", current),
                ],
            },
        );
        store
    }

    #[test]
    fn the_battery_line_has_two_numbers_or_says_not_available() {
        assert_eq!(battery_text(&MessageStore::default()), "Battery: no data");
        assert_eq!(
            battery_text(&sys_status(15_980.0, 120.0)),
            "Battery: 15.98 V, 1.20 A"
        );
        assert_eq!(
            battery_text(&sys_status(65_535.0, -1.0)),
            "Battery: not available, not available"
        );
    }
}
