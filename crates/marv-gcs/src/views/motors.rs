//! Motors view: the four ESCs from `ESC_STATUS` (rpm, voltage, current) and `ESC_INFO` (online,
//! temperature, error counters), `docs/link-contract.md` section 14. Read only: it sends no command.

use std::time::Duration;

use eframe::egui::{self, Color32, RichText};
use gcs_mavlink::services::messages::{MessageEntry, MessageStore, Series};

use super::devices;
use super::plot;
use super::sensors::{self, Preset};
use super::{View, ViewCtx};
use crate::status::{GREEN, GREY, RED};

const STATUS: &str = "ESC_STATUS";
const INFO: &str = "ESC_INFO";
/// A message older than this is shown grey (the HUD's `ATTITUDE_STALE`).
const STALE_AFTER: Duration = Duration::from_secs(2);
/// Two `ESC_INFO` further apart than this give no error percentage.
const MAX_INFO_GAP_US: u64 = 60_000_000;
/// `MARV_DEVICE` of the ESCs.
const ESC_DEVICE: u8 = 12;
/// `ESC_INFO.temperature` when the ESC gave none.
const NO_TEMPERATURE: f64 = 32_767.0;
const PLOT_HEIGHT: f32 = 220.0;

/// The four rpm series as one preset, so the plot is drawn as the Sensors tab draws its own.
const RPM: Preset = Preset {
    title: "Motor speed (rpm)",
    message: STATUS,
    instance: 0,
    fields: &["rpm[0]", "rpm[1]", "rpm[2]", "rpm[3]"],
    scale: 1.0,
    unavailable: &[],
};

#[derive(Default)]
pub struct Motors;

/// What one `ESC_INFO` says about the error counters of one ESC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InfoSample {
    pub time_usec: u64,
    pub counter: u16,
    pub error_count: u32,
}

/// The share of the exchanges between two `ESC_INFO` in which the ESC gave no valid reply, in percent:
/// 100 x change of `error_count` / change of `counter` (modulo 65536). `None` when the two are more
/// than 60 s apart, when `time_usec` went back, or when the counter did not move.
pub fn error_percent(previous: InfoSample, latest: InfoSample) -> Option<f64> {
    let elapsed = latest.time_usec.checked_sub(previous.time_usec)?;
    if elapsed > MAX_INFO_GAP_US {
        return None;
    }
    let exchanges = latest.counter.wrapping_sub(previous.counter);
    if exchanges == 0 {
        return None;
    }
    let errors = latest.error_count.wrapping_sub(previous.error_count);
    Some(100.0 * f64::from(errors) / f64::from(exchanges))
}

/// The two latest `ESC_INFO` of ESC `index` as the store's 60 s series keep them; `None` while there are
/// fewer than two, or when the three series do not agree on the time of their last two points.
fn info_samples(store: &MessageStore, index: usize) -> Option<(InfoSample, InfoSample)> {
    let error_field = format!("error_count[{index}]");
    let last_two = |field: &str| -> Option<[[f64; 2]; 2]> {
        let points = &store.series(INFO, 0, field)?.points;
        let n = points.len();
        (n >= 2).then(|| [points[n - 2], points[n - 1]])
    };
    let time = last_two("time_usec")?;
    let counter = last_two("counter")?;
    let errors = last_two(&error_field)?;
    if (0..2).any(|k| time[k][0] != counter[k][0] || time[k][0] != errors[k][0]) {
        return None;
    }
    let sample = |k: usize| InfoSample {
        time_usec: time[k][1] as u64,
        counter: counter[k][1] as u16,
        error_count: errors[k][1] as u32,
    };
    Some((sample(0), sample(1)))
}

/// How old a message is: `None` when it never arrived.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Age {
    Never,
    Fresh,
    Stale,
}

fn age_of(entry: Option<&MessageEntry>, now: Option<Duration>) -> Age {
    let Some(entry) = entry else {
        return Age::Never;
    };
    match now {
        Some(now) if now.saturating_sub(entry.last_seen) <= STALE_AFTER => Age::Fresh,
        _ => Age::Stale,
    }
}

/// The colour of a value: its own when the message is fresh, grey otherwise.
fn value_colour(age: Age, fresh: Color32) -> Color32 {
    if age == Age::Fresh { fresh } else { GREY }
}

/// The frame of a box: green online, red offline, grey when that is not known or not current.
fn frame_colour(online: Option<bool>, info: Age) -> Color32 {
    match (online, info) {
        (Some(true), Age::Fresh) => GREEN,
        (Some(false), Age::Fresh) => RED,
        _ => GREY,
    }
}

fn voltage_text(volts: Option<f64>) -> String {
    volts
        .filter(|v| v.is_finite())
        .map_or_else(|| "n/a".to_string(), |v| format!("{v:.2} V"))
}

fn current_text(amps: Option<f64>) -> String {
    amps.filter(|a| a.is_finite())
        .map_or_else(|| "n/a".to_string(), |a| format!("{a:.2} A"))
}

/// `ESC_INFO.temperature` is in cdegC; 32767 (INT16_MAX) means the ESC gave none.
fn temperature_text(centidegrees: Option<f64>) -> String {
    centidegrees
        .filter(|t| t.is_finite() && *t != NO_TEMPERATURE)
        .map_or_else(|| "n/a".to_string(), |t| format!("{:.1} °C", t / 100.0))
}

fn error_text(percent: Option<f64>) -> String {
    percent.map_or_else(|| "—".to_string(), |p| format!("{p:.1} %"))
}

fn rpm_text(rpm: Option<f64>) -> String {
    rpm.filter(|r| r.is_finite())
        .map_or_else(|| "n/a".to_string(), |r| format!("{r:.0}"))
}

/// Whether ESC `index` is online by `ESC_INFO.info`; `None` when there is no `info`.
fn online(info: Option<&MessageEntry>, index: usize) -> Option<bool> {
    let bits = info?.number("info")? as u32;
    Some(bits >> index & 1 == 1)
}

/// ESCs online of the `count` that `ESC_INFO` announces (at most 4), with that count.
fn online_count(info: &MessageEntry) -> Option<(usize, usize)> {
    let count = (info.number("count")? as usize).min(4);
    let n = (0..count)
        .filter(|&i| online(Some(info), i) == Some(true))
        .count();
    Some((n, count))
}

/// The link type: `ESC_CONNECTION_TYPE_DSHOT` reads "DShot".
fn link_text(info: &MessageEntry) -> String {
    match info.text("connection_type") {
        Some(text) => match devices::variant(text, "ESC_CONNECTION_TYPE_").as_str() {
            "DSHOT" => "DShot".to_string(),
            other => other.to_string(),
        },
        None => info
            .number("connection_type")
            .map_or_else(|| "—".to_string(), |n| format!("type {n}")),
    }
}

impl View for Motors {
    fn name(&self) -> &'static str {
        "Motors"
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let store = &cx.state.messages;
        let status = store.entry(STATUS, 0);
        let info = store.entry(INFO, 0);
        let status_age = age_of(status, cx.now);
        let info_age = age_of(info, cx.now);
        if status.is_none() && info.is_none() {
            ui.label("No ESC telemetry from the vehicle");
            return;
        }
        summary(ui, store, info, info_age, status_age, status, cx.now);
        ui.separator();
        ui.columns(4, |columns| {
            for (index, ui) in columns.iter_mut().enumerate() {
                motor_box(ui, store, index, status, info, status_age, info_age);
            }
        });
        ui.separator();
        let window = plot::window_selector(ui, "motors");
        ui.strong(RPM.title);
        let owned = sensors::preset_traces(store, &RPM, window);
        let traces: Vec<(String, &Series)> = owned
            .iter()
            .map(|(field, series)| (motor_name(field), series))
            .collect();
        if traces.is_empty() {
            ui.label("no data");
        } else {
            plot::draw_with_readout(ui, "motors_plot", PLOT_HEIGHT, window, &traces, "rpm");
        }
    }
}

/// `rpm[2]` is "Motor 3".
fn motor_name(field: &str) -> String {
    let index = field
        .trim_start_matches("rpm[")
        .trim_end_matches(']')
        .parse::<usize>()
        .unwrap_or(0);
    format!("Motor {}", index + 1)
}

fn summary(
    ui: &mut egui::Ui,
    store: &MessageStore,
    info: Option<&MessageEntry>,
    info_age: Age,
    status_age: Age,
    status: Option<&MessageEntry>,
    now: Option<Duration>,
) {
    ui.horizontal_wrapped(|ui| {
        let online_text = info
            .and_then(online_count)
            .map_or_else(|| "n/a".to_string(), |(n, count)| format!("{n}/{count}"));
        ui.label(
            RichText::new(format!("ESCs online {online_text}"))
                .strong()
                .color(value_colour(info_age, ui.visuals().text_color())),
        );
        ui.separator();
        match devices::rows(store, now)
            .iter()
            .find(|row| row.device == ESC_DEVICE)
        {
            Some(row) => {
                let colour = if row.stale {
                    GREY
                } else {
                    devices::state_colour(&row.state)
                };
                ui.label("ESC device");
                ui.label(RichText::new(row.state.clone()).strong().color(colour));
            }
            None => {
                ui.label("ESC device: no status");
            }
        }
        ui.separator();
        ui.label(format!(
            "Link {}",
            info.map_or_else(|| "—".to_string(), link_text)
        ));
        if let (Some(entry), Some(now)) = (status, now) {
            ui.separator();
            let age = now.saturating_sub(entry.last_seen);
            let stale = if status_age == Age::Stale {
                " (stale)"
            } else {
                ""
            };
            ui.label(format!("Updated {:.1} s ago{stale}", age.as_secs_f64()));
        }
    });
}

fn motor_box(
    ui: &mut egui::Ui,
    store: &MessageStore,
    index: usize,
    status: Option<&MessageEntry>,
    info: Option<&MessageEntry>,
    status_age: Age,
    info_age: Age,
) {
    let online = online(info, index);
    let title = if online == Some(false) {
        format!("Motor {} OFFLINE", index + 1)
    } else {
        format!("Motor {}", index + 1)
    };
    let number = |entry: Option<&MessageEntry>, field: &str| {
        entry.and_then(|e| e.number(&format!("{field}[{index}]")))
    };
    let percent = info_samples(store, index).and_then(|(a, b)| error_percent(a, b));
    let normal = ui.visuals().text_color();
    let status_colour = value_colour(status_age, normal);
    let headline_colour = value_colour(status_age, ui.visuals().strong_text_color());
    let info_colour = value_colour(info_age, normal);
    egui::Frame::group(ui.style())
        .stroke(egui::Stroke::new(2.0, frame_colour(online, info_age)))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(
                RichText::new(title)
                    .strong()
                    .color(frame_colour(online, info_age)),
            );
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new(rpm_text(number(status, "rpm")))
                        .size(32.0)
                        .strong()
                        .color(headline_colour),
                );
                ui.label(RichText::new("rpm").color(status_colour));
            });
            let line = |ui: &mut egui::Ui, name: &str, text: String, colour: Color32| {
                ui.label(RichText::new(format!("{name} {text}")).color(colour));
            };
            line(
                ui,
                "Voltage",
                voltage_text(number(status, "voltage")),
                status_colour,
            );
            line(
                ui,
                "Temperature",
                temperature_text(number(info, "temperature")),
                info_colour,
            );
            line(
                ui,
                "Current",
                current_text(number(status, "current")),
                status_colour,
            );
            line(ui, "Errors", error_text(percent), info_colour);
        });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(time_s: u64, counter: u16, error_count: u32) -> InfoSample {
        InfoSample {
            time_usec: time_s * 1_000_000,
            counter,
            error_count,
        }
    }

    #[test]
    fn the_error_percentage_is_errors_over_exchanges() {
        let got = error_percent(sample(10, 1000, 5), sample(11, 2000, 15)).expect("a value");
        assert!((got - 1.0).abs() < 1e-9, "{got}");
        assert_eq!(
            error_percent(sample(10, 1000, 5), sample(11, 2000, 5)),
            Some(0.0)
        );
    }

    #[test]
    fn the_counter_wraps_at_65536_and_so_does_the_error_count() {
        let got = error_percent(sample(10, 65_036, 100), sample(11, 500, 110)).expect("a value");
        // 65036 -> 500 is 1000 exchanges.
        assert!((got - 1.0).abs() < 1e-9, "{got}");
        let got =
            error_percent(sample(10, 1000, u32::MAX - 4), sample(11, 2000, 5)).expect("a value");
        assert!((got - 1.0).abs() < 1e-9, "{got}");
    }

    #[test]
    fn two_samples_more_than_60_s_apart_give_no_percentage() {
        assert_eq!(
            error_percent(sample(10, 1000, 0), sample(71, 2000, 10)),
            None
        );
        assert!(error_percent(sample(10, 1000, 0), sample(70, 2000, 10)).is_some());
    }

    #[test]
    fn time_going_back_gives_no_percentage() {
        assert_eq!(
            error_percent(sample(20, 1000, 0), sample(19, 2000, 10)),
            None
        );
    }

    #[test]
    fn a_counter_that_did_not_move_gives_no_percentage() {
        assert_eq!(
            error_percent(sample(10, 1000, 0), sample(11, 1000, 10)),
            None
        );
    }

    #[test]
    fn missing_values_read_n_a() {
        assert_eq!(voltage_text(Some(f64::NAN)), "n/a");
        assert_eq!(voltage_text(None), "n/a");
        assert_eq!(voltage_text(Some(15.5)), "15.50 V");
        assert_eq!(current_text(Some(f64::NAN)), "n/a");
        assert_eq!(temperature_text(Some(32_767.0)), "n/a");
        assert_eq!(temperature_text(Some(3_750.0)), "37.5 °C");
        assert_eq!(error_text(None), "—");
        assert_eq!(error_text(Some(1.04)), "1.0 %");
    }

    #[test]
    fn colours_follow_online_and_age() {
        assert_eq!(frame_colour(Some(true), Age::Fresh), GREEN);
        assert_eq!(frame_colour(Some(false), Age::Fresh), RED);
        assert_eq!(frame_colour(Some(true), Age::Stale), GREY);
        assert_eq!(frame_colour(None, Age::Never), GREY);
        assert_eq!(value_colour(Age::Stale, GREEN), GREY);
        assert_eq!(value_colour(Age::Fresh, GREEN), GREEN);
    }
}
