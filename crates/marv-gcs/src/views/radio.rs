//! Radio view: the sixteen channels of `RC_CHANNELS` as bars, the link quality, the age of the last stick
//! report, the RC row of `MARV_DEVICE_STATUS` and `RADIO_STATUS` when it is present. Read only: no
//! mapping and no calibration.

use std::time::Duration;

use eframe::egui::{self, Color32, Rect, Sense, Stroke, StrokeKind, vec2};
use gcs_mavlink::services::messages::{MessageEntry, MessageStore};

use super::devices;
use super::{View, ViewCtx};
use crate::status::{GREY, YELLOW};

/// The RC row's `device_id` in `MARV_DEVICE_STATUS`.
const RC_DEVICE: u8 = 7;
/// The bar's scale in microseconds, and the marks drawn on it.
pub const SCALE_US: (f64, f64) = (800.0, 2200.0);
pub const MARKS_US: [f64; 3] = [1000.0, 1500.0, 2000.0];
/// RC_CHANNELS older than this (or never seen) is "no stick data".
pub const STICK_STALE: Duration = Duration::from_secs(2);
/// The channels the message carries.
pub const CHANNELS: usize = 16;
const BAR_SIZE: egui::Vec2 = vec2(420.0, 16.0);

#[derive(Default)]
pub struct Radio;

impl View for Radio {
    fn name(&self) -> &'static str {
        "Radio"
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let store = &cx.state.messages;
        let sticks = Sticks::of(store, cx.now);
        let (text, colour) = sticks.age_line();
        ui.colored_label(colour, text);
        ui.label(link_quality_text(sticks.rssi));
        let rc_row = devices::rows(store, cx.now)
            .into_iter()
            .find(|row| row.device == RC_DEVICE);
        match &rc_row {
            Some(row) => {
                let colour = if row.stale {
                    GREY
                } else {
                    devices::state_colour(&row.state)
                };
                ui.colored_label(
                    colour,
                    format!(
                        "RC row: {}, last error {}, {}",
                        row.state, row.last_error, row.received
                    ),
                );
            }
            None => {
                ui.label("RC row: no RC row yet");
            }
        }
        ui.label(radio_status_text(store.entry("RADIO_STATUS", 0)));
        ui.separator();
        marks_header(ui);
        egui::ScrollArea::vertical().show(ui, |ui| {
            for channel in 0..sticks.shown() {
                let value = sticks
                    .values
                    .get(channel)
                    .copied()
                    .flatten()
                    .filter(|_| sticks.fresh);
                ui.horizontal(|ui| {
                    ui.monospace(format!("CH {:>2}", channel + 1));
                    bar(ui, value);
                    ui.label(value_text(value));
                });
            }
        });
    }
}

/// What the latest `RC_CHANNELS` says.
pub struct Sticks {
    /// Microseconds of channel 1 to 16 as received; `None` for a channel the message marks unused (65535) or
    /// does not carry (beyond `chancount`).
    pub values: Vec<Option<u16>>,
    /// `chancount`, at most 16; 16 when nothing was received.
    pub count: usize,
    pub rssi: Option<u8>,
    /// How old the message is; `None` when never seen or no session.
    pub age: Option<Duration>,
    /// Seen within [`STICK_STALE`].
    pub fresh: bool,
}

impl Sticks {
    pub fn of(store: &MessageStore, now: Option<Duration>) -> Sticks {
        let Some(entry) = store.entry("RC_CHANNELS", 0) else {
            return Sticks::none();
        };
        let age = now.map(|now| now.saturating_sub(entry.last_seen));
        let count = entry
            .number("chancount")
            .map_or(CHANNELS, |n| (n as usize).min(CHANNELS));
        let values = (0..CHANNELS)
            .map(|i| {
                if i >= count {
                    return None;
                }
                let raw = entry.number(&format!("chan{}_raw", i + 1))? as u16;
                (raw != u16::MAX).then_some(raw)
            })
            .collect();
        Sticks {
            values,
            count,
            rssi: entry.number("rssi").map(|r| r as u8),
            age,
            fresh: age.is_some_and(|a| a <= STICK_STALE),
        }
    }

    fn none() -> Sticks {
        Sticks {
            values: vec![None; CHANNELS],
            count: CHANNELS,
            rssi: None,
            age: None,
            fresh: false,
        }
    }

    /// How many bars: `chancount` of the last message, 16 before any.
    pub fn shown(&self) -> usize {
        self.count
    }

    /// The age of the last RC_CHANNELS in words; "no stick data" when it is old or was never seen.
    pub fn age_line(&self) -> (String, Color32) {
        match (self.fresh, self.age) {
            (true, Some(age)) => (
                format!("RC_CHANNELS {:.1} s ago", age.as_secs_f64()),
                crate::status::GREEN,
            ),
            (false, Some(age)) => (
                format!(
                    "no stick data (last RC_CHANNELS {:.1} s ago)",
                    age.as_secs_f64()
                ),
                YELLOW,
            ),
            _ => ("no stick data".to_string(), YELLOW),
        }
    }
}

/// `rssi` of RC_CHANNELS: 0 to 254, 255 unknown.
pub fn link_quality_text(rssi: Option<u8>) -> String {
    match rssi {
        None | Some(255) => "Link quality: unknown".to_string(),
        Some(r) => format!(
            "Link quality: rssi {r} of 254 ({} %)",
            u32::from(r) * 100 / 254
        ),
    }
}

/// The share of the bar (0 to 1) a value in microseconds fills on the 800 to 2200 scale.
pub fn fraction(us: u16) -> f32 {
    ((f64::from(us) - SCALE_US.0) / (SCALE_US.1 - SCALE_US.0)).clamp(0.0, 1.0) as f32
}

fn value_text(value: Option<u16>) -> String {
    value.map_or_else(|| "—".to_string(), |us| us.to_string())
}

/// `RADIO_STATUS` in one line, or that there is none.
pub fn radio_status_text(entry: Option<&MessageEntry>) -> String {
    let Some(entry) = entry else {
        return "RADIO_STATUS: none received".to_string();
    };
    let field = |name: &str| {
        entry
            .number(name)
            .map_or_else(|| "—".to_string(), |n| format!("{n}"))
    };
    format!(
        "RADIO_STATUS: rssi {}, remrssi {}, txbuf {} %, noise {}, remnoise {}, rxerrors {}, fixed {}",
        field("rssi"),
        field("remrssi"),
        field("txbuf"),
        field("noise"),
        field("remnoise"),
        field("rxerrors"),
        field("fixed"),
    )
}

fn marks_header(ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.monospace("     ");
        let (rect, _) = ui.allocate_exact_size(vec2(BAR_SIZE.x, 14.0), Sense::hover());
        let painter = ui.painter_at(rect);
        for mark in MARKS_US {
            let x = rect.left() + fraction(mark as u16) * rect.width();
            painter.text(
                egui::pos2(x, rect.center().y),
                egui::Align2::CENTER_CENTER,
                format!("{mark}"),
                egui::FontId::proportional(11.0),
                ui.visuals().weak_text_color(),
            );
        }
    });
}

fn bar(ui: &mut egui::Ui, value: Option<u16>) {
    let (rect, _) = ui.allocate_exact_size(BAR_SIZE, Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 2.0, ui.visuals().extreme_bg_color);
    if let Some(us) = value {
        let filled =
            Rect::from_min_size(rect.min, vec2(rect.width() * fraction(us), rect.height()));
        painter.rect_filled(filled, 2.0, Color32::from_rgb(70, 150, 220));
    }
    for mark in MARKS_US {
        let x = rect.left() + fraction(mark as u16) * rect.width();
        painter.line_segment(
            [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
            Stroke::new(1.0, ui.visuals().weak_text_color()),
        );
    }
    painter.rect_stroke(rect, 2.0, Stroke::new(1.0, GREY), StrokeKind::Inside);
}

#[cfg(test)]
mod tests {
    use gcs_mavlink::services::messages::{Field, FieldValue, MessageKey};

    use super::*;

    fn entry(name: &str, fields: &[(&str, f64)], last_seen: f64) -> (MessageKey, MessageEntry) {
        (
            MessageKey {
                system_id: 1,
                component_id: 1,
                message_id: 65,
                instance: 0,
            },
            MessageEntry {
                name: name.into(),
                label: name.into(),
                count: 1,
                rate_hz: 5.0,
                last_seen: Duration::from_secs_f64(last_seen),
                fields: fields
                    .iter()
                    .map(|(n, v)| Field {
                        name: (*n).into(),
                        value: FieldValue::Number(*v),
                    })
                    .collect(),
            },
        )
    }

    fn store_with(fields: &[(&str, f64)], last_seen: f64) -> MessageStore {
        let mut store = MessageStore::default();
        let (key, e) = entry("RC_CHANNELS", fields, last_seen);
        store.entries.insert(key, e);
        store
    }

    fn secs(s: f64) -> Option<Duration> {
        Some(Duration::from_secs_f64(s))
    }

    #[test]
    fn the_bar_scale_is_800_to_2200_with_marks_at_1000_1500_2000() {
        assert_eq!(fraction(800), 0.0);
        assert_eq!(fraction(2200), 1.0);
        assert_eq!(fraction(1500), 0.5);
        assert_eq!(fraction(700), 0.0);
        assert_eq!(fraction(3000), 1.0);
        assert!((fraction(1000) - 200.0 / 1400.0).abs() < 1e-6);
        assert!((fraction(2000) - 1200.0 / 1400.0).abs() < 1e-6);
        assert_eq!(MARKS_US, [1000.0, 1500.0, 2000.0]);
    }

    #[test]
    fn the_channels_follow_chancount_and_an_unused_channel_is_blank() {
        let store = store_with(
            &[
                ("chan1_raw", 1000.0),
                ("chan2_raw", 1500.0),
                ("chan3_raw", 65_535.0),
                ("chan4_raw", 2000.0),
                ("chancount", 3.0),
                ("rssi", 127.0),
            ],
            9.0,
        );
        let sticks = Sticks::of(&store, secs(9.5));
        assert_eq!(sticks.shown(), 3);
        assert_eq!(&sticks.values[..4], [Some(1000), Some(1500), None, None]);
        assert_eq!(sticks.rssi, Some(127));
        assert!(sticks.fresh);
    }

    #[test]
    fn sixteen_bars_before_any_data_and_no_stick_data() {
        let sticks = Sticks::of(&MessageStore::default(), secs(5.0));
        assert_eq!(sticks.shown(), 16);
        assert!(sticks.values.iter().all(Option::is_none));
        assert_eq!(sticks.age_line().0, "no stick data");
        assert_eq!(
            Sticks::of(&MessageStore::default(), None).age_line().0,
            "no stick data"
        );
    }

    #[test]
    fn stick_data_older_than_two_seconds_is_no_stick_data_and_two_is_not() {
        let store = store_with(&[("chan1_raw", 1500.0), ("chancount", 16.0)], 10.0);
        let at = |now| Sticks::of(&store, secs(now));
        assert!(at(12.0).fresh);
        assert_eq!(at(10.25).age_line().0, "RC_CHANNELS 0.2 s ago");
        assert!(!at(12.001).fresh);
        assert_eq!(
            at(15.0).age_line().0,
            "no stick data (last RC_CHANNELS 5.0 s ago)"
        );
        assert!(!Sticks::of(&store, None).fresh);
    }

    #[test]
    fn link_quality_255_is_unknown() {
        assert_eq!(link_quality_text(Some(255)), "Link quality: unknown");
        assert_eq!(link_quality_text(None), "Link quality: unknown");
        assert_eq!(
            link_quality_text(Some(254)),
            "Link quality: rssi 254 of 254 (100 %)"
        );
        assert_eq!(
            link_quality_text(Some(0)),
            "Link quality: rssi 0 of 254 (0 %)"
        );
    }

    #[test]
    fn radio_status_is_shown_when_present() {
        assert_eq!(radio_status_text(None), "RADIO_STATUS: none received");
        let (_, e) = entry(
            "RADIO_STATUS",
            &[
                ("rssi", 200.0),
                ("remrssi", 190.0),
                ("txbuf", 90.0),
                ("noise", 10.0),
                ("remnoise", 12.0),
                ("rxerrors", 3.0),
                ("fixed", 1.0),
            ],
            1.0,
        );
        assert_eq!(
            radio_status_text(Some(&e)),
            "RADIO_STATUS: rssi 200, remrssi 190, txbuf 90 %, noise 10, remnoise 12, rxerrors 3, fixed 1"
        );
    }
}
