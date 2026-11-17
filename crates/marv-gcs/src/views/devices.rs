//! Devices view: the health of every device of the flight controller, one row per `device_id`
//! (`MARV_DEVICE_STATUS`, `docs/link-contract.md` section 8). Read only. The logic is in plain
//! functions that know the message's field names; the store they read from knows no device.

use std::time::Duration;

use eframe::egui::{self, Color32};
use egui_extras::{Column, TableBuilder};
use gcs_mavlink::Command;
use gcs_mavlink::dialect::MavCmd;
use gcs_mavlink::services::messages::{MessageEntry, MessageKey, MessageStore, SeriesKey};

use super::actions;
use super::{View, ViewCtx};
use crate::status::{self, GREEN, GREY, RED, YELLOW};

const ROW_HEIGHT: f32 = 20.0;
const MESSAGE: &str = "MARV_DEVICE_STATUS";
/// `last_sample_age_ms` of a device that has delivered no sample yet.
const NEVER: u16 = u16::MAX;
/// A row received more than this long ago is stale (a row received exactly this long ago is not).
pub const STALE_AFTER: Duration = Duration::from_secs(15);

const TITLES: [&str; 15] = [
    "Device",
    "State",
    "Last error",
    "Samples",
    "Samples/s",
    "Last sample",
    "Errors",
    "Timeouts",
    "Recoveries",
    "Attempts",
    "Overflows",
    "Config restored",
    "Flags",
    "Received",
    "Sender",
];

#[derive(Default)]
pub struct Devices {
    presets: super::presets::SensorPresets,
}

impl View for Devices {
    fn name(&self) -> &'static str {
        "Devices"
    }

    fn session_changed(&mut self) {
        self.presets.session_changed();
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let gate = status::command_gate(cx.access, cx.state, cx.now.is_some());
        calibration_buttons(ui, cx, gate);
        self.presets.ui(ui, cx);
        let rows = rows(&cx.state.messages, cx.now);
        if rows.is_empty() {
            ui.label("No device status yet");
            return;
        }
        let mut table = TableBuilder::new(ui).id_salt("devices_table").striped(true);
        for _ in 0..TITLES.len() {
            table = table.column(Column::auto().at_least(60.0));
        }
        table
            .column(Column::remainder().at_least(60.0))
            .header(ROW_HEIGHT, |mut header| {
                for (i, title) in TITLES.into_iter().enumerate() {
                    header.col(|ui| {
                        ui.strong(title);
                    });
                    if i == 0 {
                        header.col(|ui| {
                            ui.strong("Actions");
                        });
                    }
                }
            })
            .body(|body| {
                body.rows(ROW_HEIGHT, rows.len(), |mut row| {
                    let device = &rows[row.index()];
                    let colour = if device.stale {
                        GREY
                    } else {
                        state_colour(&device.state)
                    };
                    for (i, text) in device.cells().into_iter().enumerate() {
                        row.col(|ui| {
                            if i == 1 {
                                ui.colored_label(colour, text);
                            } else if device.stale {
                                ui.colored_label(GREY, text);
                            } else {
                                ui.label(text);
                            }
                        });
                        if i == 0 {
                            row.col(|ui| {
                                ui.horizontal(|ui| {
                                    if can_restart(device.device)
                                        && actions::button(ui, "Restart", gate)
                                    {
                                        cx.commands.push(device_command(device.device, 1.0));
                                    }
                                    if can_self_test(device.device)
                                        && actions::button(ui, "Self-test", gate)
                                    {
                                        cx.commands.push(device_command(device.device, 2.0));
                                    }
                                });
                            });
                        }
                    }
                });
            });
    }
}

/// The devices that can be restarted (`MARV_DEVICE`: IMU1, IMU2, BARO, MAG, GNSS, BATT and SD).
pub fn can_restart(device: u8) -> bool {
    matches!(device, 1..=6 | SD)
}

/// `MARV_DEVICE_SD`: the SD card, whose restart the Log view offers too.
pub const SD: u8 = 9;

/// The devices that have a self-test (IMU1, IMU2, MAG).
pub fn can_self_test(device: u8) -> bool {
    matches!(device, 1 | 2 | 4)
}

/// `MAV_CMD_MARV_DEVICE`: param1 the device, param2 the action (1 restart, 2 self-test).
pub(super) fn device_command(device: u8, action: f32) -> Command {
    Command::SendCommand {
        command: MavCmd::MAV_CMD_MARV_DEVICE,
        params: [f32::from(device), action, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
}

/// Above the table: the calibrations, the cancel and the status line of the last command.
fn calibration_buttons(ui: &mut egui::Ui, cx: &mut ViewCtx, gate: Option<&'static str>) {
    let calibrate = |params: [f32; 7]| Command::SendCommand {
        command: MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
        params,
    };
    let cancel_gate = status::cancel_gate(cx.access, cx.state, cx.now.is_some());
    ui.horizontal(|ui| {
        if actions::button(ui, "Calibrate gyro", gate) {
            cx.commands
                .push(calibrate([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
        }
        if actions::button(ui, "Calibrate accelerometer (level)", gate) {
            cx.commands
                .push(calibrate([0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0]));
        }
        if actions::button(ui, "Calibrate compass", gate) {
            cx.commands
                .push(calibrate([0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
        }
        if actions::button(ui, "Cancel calibration", cancel_gate) {
            cx.commands.push(calibrate([0.0; 7]));
        }
    });
    if let Some(reason) = gate
        && cx.access == gcs_mavlink::Access::Configure
    {
        ui.colored_label(YELLOW, reason);
    }
    ui.horizontal(|ui| {
        actions::status_line(ui, cx.state);
        if let Some(hint) = status::compass_hint(cx.state) {
            ui.colored_label(YELLOW, hint);
        }
    });
}

/// One row of the table, every cell as the text shown.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceRow {
    /// `device_id`; the rows are in this order.
    pub device: u8,
    pub name: String,
    pub state: String,
    pub last_error: String,
    pub samples: String,
    pub samples_per_second: String,
    pub last_sample: String,
    pub errors: String,
    pub timeouts: String,
    pub recoveries: String,
    pub attempts: String,
    pub overflows: String,
    pub config_restored: String,
    pub flags: String,
    /// How long ago the row was received, with `(stale)` when it is.
    pub received: String,
    pub sender: String,
    pub stale: bool,
}

impl DeviceRow {
    fn cells(&self) -> [String; 15] {
        [
            self.name.clone(),
            self.state.clone(),
            self.last_error.clone(),
            self.samples.clone(),
            self.samples_per_second.clone(),
            self.last_sample.clone(),
            self.errors.clone(),
            self.timeouts.clone(),
            self.recoveries.clone(),
            self.attempts.clone(),
            self.overflows.clone(),
            self.config_restored.clone(),
            self.flags.clone(),
            self.received.clone(),
            self.sender.clone(),
        ]
    }
}

/// The rows of the table: every decoded `MARV_DEVICE_STATUS` of the store, in `device_id` order
/// (then sender). `now` is the session time; without it no row is stale and none has an age.
pub fn rows(store: &MessageStore, now: Option<Duration>) -> Vec<DeviceRow> {
    let mut rows: Vec<DeviceRow> = store
        .entries
        .iter()
        .filter(|(_, entry)| entry.name == MESSAGE && !entry.fields.is_empty())
        .map(|(key, entry)| row(store, key, entry, now))
        .collect();
    rows.sort_by_key(|row| (row.device, row.sender.clone()));
    rows
}

fn row(
    store: &MessageStore,
    key: &MessageKey,
    entry: &MessageEntry,
    now: Option<Duration>,
) -> DeviceRow {
    let count = |field: &str| entry.number(field).map_or_else(String::new, counter);
    let word = |field: &str, prefix: &str| {
        entry
            .text(field)
            .map_or_else(String::new, |text| variant(text, prefix))
    };
    let age = now.map(|now| now.saturating_sub(entry.last_seen));
    DeviceRow {
        device: key.instance,
        name: word("device_id", "MARV_DEVICE_"),
        state: word("state", "MARV_DEVICE_STATE_"),
        last_error: word("last_error", "MARV_DEVICE_ERROR_"),
        samples: count("samples"),
        samples_per_second: rate_of(store, key)
            .map_or_else(String::new, |rate| format!("{rate:.1}")),
        last_sample: entry
            .number("last_sample_age_ms")
            .map_or_else(String::new, |ms| last_sample_text(ms as u16)),
        errors: count("errors"),
        timeouts: count("timeouts"),
        recoveries: count("recoveries"),
        attempts: count("attempts"),
        overflows: count("overflows"),
        config_restored: count("config_restored"),
        flags: entry
            .number("flags")
            .map_or_else(String::new, |flags| flags_text(flags as u8)),
        received: age.map_or_else(|| "—".to_string(), received_text),
        sender: format!("{}/{}", key.system_id, key.component_id),
        stale: age.is_some_and(is_stale),
    }
}

/// A counter as it is shown: a whole number.
fn counter(n: f64) -> String {
    format!("{}", n as u64)
}

/// The enum entry without its prefix: `MARV_DEVICE_STATE_OK` is `OK`. Anything else stays as it is.
pub fn variant(text: &str, prefix: &str) -> String {
    text.strip_prefix(prefix).unwrap_or(text).to_string()
}

/// `last_sample_age_ms`: 65535 means no sample yet.
pub fn last_sample_text(ms: u16) -> String {
    if ms == NEVER {
        "never".to_string()
    } else {
        format!("{ms} ms")
    }
}

/// `flags`: bit 0 the sample source is HIL, bit 1 a self-test is running; `-` for none; a bit the
/// station does not know is shown as `bit N`.
pub fn flags_text(flags: u8) -> String {
    let mut words = Vec::new();
    if flags & 1 != 0 {
        words.push("HIL".to_string());
    }
    if flags & 2 != 0 {
        words.push("self-test".to_string());
    }
    for bit in 2..8 {
        if flags & (1 << bit) != 0 {
            words.push(format!("bit {bit}"));
        }
    }
    if words.is_empty() {
        "-".to_string()
    } else {
        words.join(", ")
    }
}

/// Samples per second from two successive rows of one device, each `(time_boot_ms, samples)`: the
/// difference of `samples` over the difference of `time_boot_ms`. `None` when either went backwards
/// (the vehicle restarted) or the time did not advance.
pub fn samples_per_second(previous: (u32, u32), now: (u32, u32)) -> Option<f64> {
    let milliseconds = now.0.checked_sub(previous.0).filter(|&ms| ms > 0)?;
    let samples = now.1.checked_sub(previous.1)?;
    Some(f64::from(samples) * 1000.0 / f64::from(milliseconds))
}

/// The rate of one device from the last two points of its `time_boot_ms` and `samples` series (they
/// are appended together, so they pair up by their session time).
fn rate_of(store: &MessageStore, key: &MessageKey) -> Option<f64> {
    let last_two = |field: &str| {
        let series = store.series.get(&SeriesKey {
            system_id: key.system_id,
            component_id: key.component_id,
            message: MESSAGE.to_string(),
            instance: key.instance,
            field: field.to_string(),
        })?;
        let mut points = series.points.iter().rev();
        let (latest, before) = (points.next()?, points.next()?);
        Some((*before, *latest))
    };
    let (time_before, time_now) = last_two("time_boot_ms")?;
    let (samples_before, samples_now) = last_two("samples")?;
    if time_before[0] != samples_before[0] || time_now[0] != samples_now[0] {
        return None;
    }
    samples_per_second(
        (time_before[1] as u32, samples_before[1] as u32),
        (time_now[1] as u32, samples_now[1] as u32),
    )
}

/// Whether a row received `age` ago is stale: more than [`STALE_AFTER`], not exactly that.
pub fn is_stale(age: Duration) -> bool {
    age > STALE_AFTER
}

/// How long ago a row was received; a stale row says so.
pub fn received_text(age: Duration) -> String {
    let text = format!("{:.1} s", age.as_secs_f64());
    if is_stale(age) {
        format!("{text} (stale)")
    } else {
        text
    }
}

/// The colour of a state's text, in the application's status colours.
pub fn state_colour(state: &str) -> Color32 {
    match state {
        "OK" => GREEN,
        "STARTING" | "DEGRADED" | "RECOVERING" => YELLOW,
        "FAILED" => RED,
        _ => GREY,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcs_mavlink::services::messages::{Field, FieldValue, Series};

    fn number(name: &str, value: f64) -> Field {
        Field {
            name: name.into(),
            value: FieldValue::Number(value),
        }
    }

    fn text(name: &str, value: &str) -> Field {
        Field {
            name: name.into(),
            value: FieldValue::Text(value.into()),
        }
    }

    fn key(device: u8) -> MessageKey {
        MessageKey {
            system_id: 1,
            component_id: 1,
            message_id: 42_421,
            instance: device,
        }
    }

    fn series_key(device: u8, field: &str) -> SeriesKey {
        SeriesKey {
            system_id: 1,
            component_id: 1,
            message: MESSAGE.into(),
            instance: device,
            field: field.into(),
        }
    }

    /// One device row as the message store would hold it, with the rows of `history` (session
    /// second, `time_boot_ms`, `samples`) behind it in the series.
    fn put(
        store: &mut MessageStore,
        device: u8,
        name: &str,
        last_seen: Duration,
        history: &[(f64, u32, u32)],
    ) {
        let &(_, time_boot_ms, samples) = history.last().expect("one row at least");
        store.entries.insert(
            key(device),
            MessageEntry {
                name: MESSAGE.into(),
                label: format!("{MESSAGE}[{device}]"),
                count: history.len() as u64,
                rate_hz: 1.0,
                last_seen,
                fields: vec![
                    number("time_boot_ms", f64::from(time_boot_ms)),
                    number("samples", f64::from(samples)),
                    number("errors", 3.0),
                    number("timeouts", 2.0),
                    number("last_sample_age_ms", 65_535.0),
                    number("recoveries", 4.0),
                    number("attempts", 5.0),
                    number("overflows", 6.0),
                    number("config_restored", 7.0),
                    text("device_id", &format!("MARV_DEVICE_{name}")),
                    text("state", "MARV_DEVICE_STATE_OK"),
                    text("last_error", "MARV_DEVICE_ERROR_NONE"),
                    number("flags", 3.0),
                ],
            },
        );
        for (field, pick) in [("time_boot_ms", 0), ("samples", 1)] {
            let points = history
                .iter()
                .map(|&(at, time, samples)| [at, f64::from([time, samples][pick])])
                .collect();
            store
                .series
                .insert(series_key(device, field), Series { points });
        }
    }

    fn secs(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    #[test]
    fn rate_is_the_difference_of_samples_over_the_difference_of_boot_time() {
        assert_eq!(
            samples_per_second((10_000, 1_000), (11_000, 1_640)),
            Some(640.0)
        );
        assert_eq!(samples_per_second((10_000, 0), (12_000, 100)), Some(50.0));
        assert_eq!(samples_per_second((10_000, 5), (10_500, 5)), Some(0.0));
    }

    #[test]
    fn rate_is_blank_when_the_time_does_not_advance_or_a_counter_goes_backwards() {
        assert_eq!(samples_per_second((10_000, 1), (10_000, 2)), None);
        assert_eq!(samples_per_second((10_000, 100), (11_000, 99)), None);
        assert_eq!(samples_per_second((10_000, 100), (9_000, 200)), None);
    }

    #[test]
    fn the_row_shows_the_rate_from_two_rows_of_the_same_device() {
        let mut store = MessageStore::default();
        put(
            &mut store,
            1,
            "IMU1",
            secs(2.0),
            &[(1.0, 10_000, 1_000), (2.0, 11_000, 1_640)],
        );
        let rows = rows(&store, Some(secs(2.0)));
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].samples, "1640");
        assert_eq!(rows[0].samples_per_second, "640.0");
    }

    #[test]
    fn the_rate_is_blank_with_one_row() {
        let mut store = MessageStore::default();
        put(&mut store, 1, "IMU1", secs(1.0), &[(1.0, 10_000, 1_000)]);
        assert_eq!(rows(&store, Some(secs(1.0)))[0].samples_per_second, "");
    }

    #[test]
    fn after_a_counter_reset_the_rate_is_blank_once() {
        let mut store = MessageStore::default();
        put(
            &mut store,
            1,
            "IMU1",
            secs(2.0),
            &[(1.0, 10_000, 5_000), (2.0, 1_000, 40)],
        );
        assert_eq!(rows(&store, Some(secs(2.0)))[0].samples_per_second, "");
        put(
            &mut store,
            1,
            "IMU1",
            secs(3.0),
            &[(1.0, 10_000, 5_000), (2.0, 1_000, 40), (3.0, 2_000, 90)],
        );
        assert_eq!(rows(&store, Some(secs(3.0)))[0].samples_per_second, "50.0");
    }

    #[test]
    fn series_that_do_not_pair_up_give_no_rate() {
        let mut store = MessageStore::default();
        put(
            &mut store,
            1,
            "IMU1",
            secs(2.0),
            &[(1.0, 10_000, 1_000), (2.0, 11_000, 1_640)],
        );
        store
            .series
            .get_mut(&series_key(1, "samples"))
            .unwrap()
            .points[1][0] = 2.5;
        assert_eq!(rows(&store, Some(secs(2.0)))[0].samples_per_second, "");
    }

    #[test]
    fn a_sample_age_of_65535_is_never() {
        assert_eq!(last_sample_text(65_535), "never");
        assert_eq!(last_sample_text(65_534), "65534 ms");
        assert_eq!(last_sample_text(0), "0 ms");
        let mut store = MessageStore::default();
        put(&mut store, 1, "IMU1", secs(1.0), &[(1.0, 1, 1)]);
        assert_eq!(rows(&store, Some(secs(1.0)))[0].last_sample, "never");
    }

    #[test]
    fn a_row_is_stale_after_more_than_fifteen_seconds_and_not_at_fifteen() {
        assert!(!is_stale(Duration::from_millis(14_999)));
        assert!(!is_stale(Duration::from_secs(15)));
        assert!(is_stale(Duration::from_millis(15_001)));
        let mut store = MessageStore::default();
        put(&mut store, 1, "IMU1", secs(10.0), &[(10.0, 1, 1)]);
        let at = |now| rows(&store, Some(secs(now)))[0].clone();
        assert!(!at(25.0).stale);
        assert_eq!(at(25.0).received, "15.0 s");
        assert!(at(25.001).stale);
        assert_eq!(at(26.0).received, "16.0 s (stale)");
        // A stale row stays in the table.
        assert_eq!(rows(&store, Some(secs(600.0))).len(), 1);
        // No session: no age, never stale.
        assert!(!rows(&store, None)[0].stale);
        assert_eq!(rows(&store, None)[0].received, "—");
    }

    #[test]
    fn every_state_error_and_device_is_shown_as_its_word() {
        for (text, word) in [
            ("MARV_DEVICE_STATE_ABSENT", "ABSENT"),
            ("MARV_DEVICE_STATE_STARTING", "STARTING"),
            ("MARV_DEVICE_STATE_OK", "OK"),
            ("MARV_DEVICE_STATE_DEGRADED", "DEGRADED"),
            ("MARV_DEVICE_STATE_RECOVERING", "RECOVERING"),
            ("MARV_DEVICE_STATE_FAILED", "FAILED"),
        ] {
            assert_eq!(variant(text, "MARV_DEVICE_STATE_"), word);
        }
        for (text, word) in [
            ("MARV_DEVICE_ERROR_NONE", "NONE"),
            ("MARV_DEVICE_ERROR_BUS", "BUS"),
            ("MARV_DEVICE_ERROR_TIMEOUT", "TIMEOUT"),
            ("MARV_DEVICE_ERROR_ID_MISMATCH", "ID_MISMATCH"),
            ("MARV_DEVICE_ERROR_CONFIG_MISMATCH", "CONFIG_MISMATCH"),
            ("MARV_DEVICE_ERROR_POR", "POR"),
            ("MARV_DEVICE_ERROR_STUCK", "STUCK"),
        ] {
            assert_eq!(variant(text, "MARV_DEVICE_ERROR_"), word);
        }
        for name in [
            "IMU1", "IMU2", "BARO", "MAG", "GNSS", "BATT", "RC", "SD", "PSRAM", "LED", "ESC",
        ] {
            assert_eq!(
                variant(&format!("MARV_DEVICE_{name}"), "MARV_DEVICE_"),
                name
            );
        }
        assert_eq!(variant("something else", "MARV_DEVICE_"), "something else");
    }

    #[test]
    fn the_flags_are_words() {
        assert_eq!(flags_text(0), "-");
        assert_eq!(flags_text(1), "HIL");
        assert_eq!(flags_text(2), "self-test");
        assert_eq!(flags_text(3), "HIL, self-test");
        assert_eq!(flags_text(0b1000_0001), "HIL, bit 7");
    }

    #[test]
    fn a_state_has_a_colour_of_the_applications_status_colours() {
        assert_eq!(state_colour("OK"), GREEN);
        for yellow in ["STARTING", "DEGRADED", "RECOVERING"] {
            assert_eq!(state_colour(yellow), YELLOW);
        }
        assert_eq!(state_colour("FAILED"), RED);
        assert_eq!(state_colour("ABSENT"), GREY);
        assert_eq!(state_colour(""), GREY);
    }

    #[test]
    fn the_cells_of_a_row() {
        let mut store = MessageStore::default();
        put(
            &mut store,
            3,
            "BARO",
            secs(4.0),
            &[(3.0, 10_000, 100), (4.0, 11_000, 110)],
        );
        let row = &rows(&store, Some(secs(4.5)))[0];
        assert_eq!(
            row.cells(),
            [
                "BARO",
                "OK",
                "NONE",
                "110",
                "10.0",
                "never",
                "3",
                "2",
                "4",
                "5",
                "6",
                "7",
                "HIL, self-test",
                "0.5 s",
                "1/1"
            ]
        );
    }

    #[test]
    fn rows_are_in_device_id_order_and_undecoded_rows_are_left_out() {
        let mut store = MessageStore::default();
        for (device, name) in [(12, "ESC"), (3, "BARO"), (1, "IMU1"), (9, "SD")] {
            put(&mut store, device, name, secs(1.0), &[(1.0, 1, 1)]);
        }
        // The row of a frame the dialect refused: no fields.
        store.entries.insert(
            key(0),
            MessageEntry {
                name: MESSAGE.into(),
                label: format!("{MESSAGE}[0]"),
                count: 1,
                rate_hz: 0.0,
                last_seen: secs(1.0),
                fields: Vec::new(),
            },
        );
        let names: Vec<_> = rows(&store, Some(secs(1.0)))
            .into_iter()
            .map(|row| (row.device, row.name))
            .collect();
        assert_eq!(
            names,
            [
                (1, "IMU1".to_string()),
                (3, "BARO".to_string()),
                (9, "SD".to_string()),
                (12, "ESC".to_string())
            ]
        );
    }

    #[test]
    fn other_messages_are_not_devices() {
        let mut store = MessageStore::default();
        put(&mut store, 1, "IMU1", secs(1.0), &[(1.0, 1, 1)]);
        let mut other = store.entries[&key(1)].clone();
        other.name = "HEARTBEAT".into();
        store.entries.insert(
            MessageKey {
                message_id: 0,
                ..key(0)
            },
            other,
        );
        assert_eq!(rows(&store, Some(secs(1.0))).len(), 1);
    }
}
