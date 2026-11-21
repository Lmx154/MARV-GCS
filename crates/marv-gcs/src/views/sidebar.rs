//! The status sections under the HUD's drawing, on every tab: Link, GPS, Battery, Vehicle (armed and
//! motors), Sensors and Messages. Each is a small heading and a few lines; a section whose parts are all
//! fine says so in one line and otherwise lists only what is wrong. It draws from the state with the
//! staleness rules of the tabs the data comes from and sends nothing.
//!
//! Link: `status::describe`'s state in short form, the HEARTBEAT age, the frame counters, RADIO_STATUS and
//! RC_CHANNELS (`radio::Sticks`). GPS: GPS_RAW_INT. Battery: SYS_STATUS. Vehicle: HEARTBEAT `base_mode`
//! (armed), ESC_INFO as the Motors tab reads it. Sensors: the `MARV_DEVICE_STATUS` rows (`devices::rows`)
//! and the SYS_STATUS present/enabled/health words. Messages: the latest STATUSTEXT.

use std::time::Duration;

use eframe::egui::{self, Color32, RichText};
use gcs_mavlink::services::messages::{MessageEntry, MessageStore};
use gcs_mavlink::services::texts::Severity;
use gcs_mavlink::{LinkState, LinkStatus, VehicleState};

use super::hud::{self, GPS_STALE};
use super::motors::{self, Age};
use super::radio::{self, Sticks};
use super::{devices, sensors};
use crate::status::{GREEN, GREY, RED, YELLOW};
use crate::theme;

/// A HEARTBEAT older than this says nothing about armed.
const HEARTBEAT_STALE: Duration = Duration::from_secs(3);
/// A SYS_STATUS older than this is not shown.
const SYS_STATUS_STALE: Duration = Duration::from_secs(5);
/// A RADIO_STATUS older than this is not shown.
const RADIO_STALE: Duration = radio::STICK_STALE;
/// How many status texts are shown.
const TEXTS_SHOWN: usize = 3;
/// SYS_STATUS `battery_remaining` when unknown.
const UNKNOWN_PERCENT: f64 = -1.0;
/// Motors an ESC_INFO without `count` is taken to have.
const DEFAULT_MOTORS: usize = 4;

/// The sections, in order, under the numbers of the HUD.
pub fn ui(ui: &mut egui::Ui, state: &VehicleState, now: Option<Duration>) {
    let store = &state.messages;
    section(ui, "Link status", |ui| link(ui, state, now));
    section(ui, "GPS", |ui| gps(ui, store, now));
    section(ui, "Battery", |ui| battery(ui, store, now));
    section(ui, "Vehicle", |ui| vehicle(ui, store, now));
    section(ui, "Sensor status", |ui| sensors_section(ui, store, now));
    section(ui, "Recent messages", |ui| messages(ui, state));
}

/// A compact card: a small title over the section's lines.
fn section(ui: &mut egui::Ui, title: &str, add: impl FnOnce(&mut egui::Ui)) {
    ui.add_space(6.0);
    theme::card()
        .inner_margin(egui::Margin::symmetric(10, 8))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.spacing_mut().item_spacing.y = 3.0;
            ui.label(
                RichText::new(title)
                    .size(11.5)
                    .strong()
                    .color(theme::TEXT_SECONDARY),
            );
            ui.add_space(2.0);
            add(ui);
        });
}

/// One line of a section. A status colour gets a dot in front; a good state is written in the normal
/// text colour next to its green dot, a warning or a fault in its colour.
fn line(ui: &mut egui::Ui, colour: Color32, text: impl Into<String>) {
    let text = text.into();
    if [GREEN, YELLOW, RED].contains(&colour) {
        let words = if colour == GREEN { theme::TEXT } else { colour };
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 6.0;
            ui.spacing_mut().interact_size.y = 0.0;
            theme::status_dot(ui, colour);
            ui.label(RichText::new(text).color(words));
        });
    } else {
        ui.label(RichText::new(text).color(colour));
    }
}

/// The entry when it arrived within `limit` of `now`.
fn fresh<'a>(
    store: &'a MessageStore,
    name: &str,
    now: Option<Duration>,
    limit: Duration,
) -> Option<&'a MessageEntry> {
    hud::fresh(store, name, now, limit)
}

// ---- Link ----

/// The link state in a few words and its colour.
pub(crate) fn link_short(link: &LinkState, now: Option<Duration>) -> (String, Color32) {
    let Some(now) = now else {
        return ("Not connected".to_string(), GREY);
    };
    match &link.status {
        LinkStatus::Connecting => ("Connecting…".to_string(), YELLOW),
        LinkStatus::Connected => ("Connected".to_string(), GREEN),
        LinkStatus::Lost => {
            let silent = now.saturating_sub(link.last_heartbeat.unwrap_or(Duration::ZERO));
            (format!("Lost ({:.1} s silent)", silent.as_secs_f64()), RED)
        }
        LinkStatus::Closed(_) => ("Not connected".to_string(), RED),
    }
}

/// Share of frames lost, in percent, from the sequence gaps and the frames received; `None` before any.
pub(crate) fn loss_percent(lost: u64, received: u64) -> Option<f64> {
    let total = lost.checked_add(received).filter(|t| *t > 0)?;
    Some(100.0 * lost as f64 / total as f64)
}

fn loss_colour(percent: f64) -> Color32 {
    if percent < 1.0 {
        GREEN
    } else if percent < 5.0 {
        YELLOW
    } else {
        RED
    }
}

fn link(ui: &mut egui::Ui, state: &VehicleState, now: Option<Duration>) {
    let link = &state.link;
    let (text, colour) = link_short(link, now);
    line(ui, colour, text);
    let connected = now.is_some() && link.status == LinkStatus::Connected;
    let store = &state.messages;
    match (now, link.last_heartbeat) {
        (Some(now), Some(last)) => {
            let silent = now.saturating_sub(last);
            line(
                ui,
                if connected { GREEN } else { GREY },
                format!("Heartbeat {:.1} s ago", silent.as_secs_f64()),
            );
        }
        _ => line(ui, GREY, "Heartbeat: no data"),
    }
    match loss_percent(store.lost_frames, link.rx.frames) {
        Some(p) if now.is_some() => line(ui, loss_colour(p), format!("Packet loss {p:.1} %")),
        _ => line(ui, GREY, "Packet loss: no data"),
    }
    if now.is_some() && link.rx.crc_errors > 0 {
        line(
            ui,
            YELLOW,
            format!("Checksum errors {}", link.rx.crc_errors),
        );
    }
    if let Some(entry) = fresh(store, "RADIO_STATUS", now, RADIO_STALE) {
        let field = |name: &str| {
            entry
                .number(name)
                .map_or_else(|| "—".to_string(), |n| format!("{n}"))
        };
        ui.label(format!(
            "Radio: rssi {}, remote {}",
            field("rssi"),
            field("remrssi")
        ));
    }
    let sticks = Sticks::of(store, now);
    if sticks.fresh {
        line(ui, GREEN, "RC link: sticks received");
        ui.label(radio::link_quality_text(sticks.rssi));
    } else {
        line(ui, GREY, "RC link: no RC data");
    }
}

// ---- GPS and battery ----

/// Latitude and longitude in degrees when the fix is a fix and the position is not 0, 0.
fn position(entry: &MessageEntry) -> Option<(f64, f64)> {
    let has_fix = match entry.text("fix_type") {
        Some(text) => !text.contains("NO_"),
        None => entry.number("fix_type").is_some_and(|n| n >= 2.0),
    };
    let lat = entry.number("lat")? / 1e7;
    let lon = entry.number("lon")? / 1e7;
    (has_fix && (lat != 0.0 || lon != 0.0)).then_some((lat, lon))
}

fn position_text(store: &MessageStore, now: Option<Duration>) -> String {
    match fresh(store, "GPS_RAW_INT", now, GPS_STALE).and_then(position) {
        Some((lat, lon)) => format!("{lat:.6}°, {lon:.6}°"),
        None => "no position".to_string(),
    }
}

fn gps(ui: &mut egui::Ui, store: &MessageStore, now: Option<Duration>) {
    let data = hud::data(store, now);
    let colour = if now.is_some() && data.fix.is_some() {
        ui.visuals().text_color()
    } else {
        GREY
    };
    line(
        ui,
        colour,
        format!(
            "GPS: {}, {} satellites",
            data.fix.as_deref().unwrap_or("--"),
            data.satellites.map_or("--".into(), |n| n.to_string())
        ),
    );
    line(
        ui,
        colour,
        format!("Position {}", position_text(store, now)),
    );
}

fn battery_colour(percent: f64) -> Color32 {
    if percent < 20.0 {
        RED
    } else if percent < 40.0 {
        YELLOW
    } else {
        GREEN
    }
}

fn battery(ui: &mut egui::Ui, store: &MessageStore, now: Option<Duration>) {
    match fresh(store, "SYS_STATUS", now, SYS_STATUS_STALE) {
        Some(entry) => {
            ui.label(sensors::battery_text(store));
            match entry
                .number("battery_remaining")
                .filter(|p| *p != UNKNOWN_PERCENT)
            {
                Some(p) => line(ui, battery_colour(p), format!("Charge {p:.0} %")),
                None => line(ui, GREY, "Charge: not available"),
            }
        }
        None => line(ui, GREY, "Battery: no data"),
    }
}

// ---- Vehicle ----

/// The names in a flag field such as `"A | B"`; empty for "" or "0".
fn flag_names(text: &str) -> Vec<&str> {
    text.split('|')
        .map(str::trim)
        .filter(|n| !n.is_empty() && *n != "0")
        .collect()
}

/// Armed from HEARTBEAT `base_mode` (flag text or number); `None` when it cannot be read.
fn armed(entry: &MessageEntry) -> Option<bool> {
    if let Some(text) = entry.text("base_mode") {
        return Some(flag_names(text).contains(&"MAV_MODE_FLAG_SAFETY_ARMED"));
    }
    entry.number("base_mode").map(|n| (n as u32) & 128 != 0)
}

/// What the motors say.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum MotorSummary {
    /// No ESC_INFO, or an old one.
    NoData,
    /// Every one of this many ESCs is online.
    AllOnline(usize),
    /// How many are online of how many, and a line for each ESC that is not.
    Faulty {
        online: usize,
        total: usize,
        lines: Vec<String>,
    },
}

pub(crate) fn motor_summary(store: &MessageStore, now: Option<Duration>) -> MotorSummary {
    let info = store.entry("ESC_INFO", 0);
    if motors::age_of(info, now) != Age::Fresh {
        return MotorSummary::NoData;
    }
    let total = info
        .and_then(motors::online_count)
        .map_or(DEFAULT_MOTORS, |(_, count)| count)
        .max(1);
    let lines: Vec<String> = (0..total)
        .filter(|&i| motors::online(info, i) != Some(true))
        .map(|i| match motors::online(info, i) {
            Some(false) => format!("Motor {}: offline", i + 1),
            _ => format!("Motor {}: state unknown", i + 1),
        })
        .collect();
    if lines.is_empty() {
        MotorSummary::AllOnline(total)
    } else {
        MotorSummary::Faulty {
            online: total - lines.len(),
            total,
            lines,
        }
    }
}

fn vehicle(ui: &mut egui::Ui, store: &MessageStore, now: Option<Duration>) {
    match fresh(store, "HEARTBEAT", now, HEARTBEAT_STALE).and_then(armed) {
        Some(true) => line(ui, YELLOW, "ARMED"),
        Some(false) => line(ui, GREEN, "Disarmed"),
        None => line(ui, GREY, "Armed state: no data"),
    }
    match motor_summary(store, now) {
        MotorSummary::NoData => line(ui, GREY, "Motors: no data"),
        MotorSummary::AllOnline(n) => line(ui, GREEN, format!("{n}/{n} ESCs online")),
        MotorSummary::Faulty {
            online,
            total,
            lines,
        } => {
            line(ui, RED, format!("{online}/{total} ESCs online"));
            for text in lines {
                line(ui, RED, text);
            }
        }
    }
}

// ---- Sensors ----

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Level {
    Ok,
    Fault,
    Off,
}

/// `MAV_SYS_STATUS_SENSOR_3D_GYRO` as `3D GYRO`.
fn sensor_label(name: &str) -> String {
    name.strip_prefix("MAV_SYS_STATUS_SENSOR_")
        .unwrap_or(name)
        .replace('_', " ")
}

/// Every sensor SYS_STATUS says is present: healthy when its health bit is set, a fault when it is
/// enabled and not healthy, off when it is not enabled.
fn sensor_levels(present: &str, enabled: &str, health: &str) -> Vec<(String, Level)> {
    let enabled = flag_names(enabled);
    let health = flag_names(health);
    flag_names(present)
        .into_iter()
        .map(|name| {
            let level = if health.contains(&name) {
                Level::Ok
            } else if enabled.contains(&name) {
                Level::Fault
            } else {
                Level::Off
            };
            (sensor_label(name), level)
        })
        .collect()
}

/// What the sensors say.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum SensorSummary {
    /// No session, or nothing current.
    NoData,
    /// All this many sensors are healthy.
    AllOk(usize),
    /// Only the ones that are not, each with its state and colour.
    Problems(Vec<(String, Color32)>),
}

/// The device rows and SYS_STATUS sensor bits as one summary. A device row is healthy when it is current
/// and its state is OK; a SYS_STATUS sensor when its health bit is set. A sensor that is present but not
/// enabled is switched off, neither healthy nor a problem, and is left out. An old row is listed, grey.
pub(crate) fn sensor_summary(store: &MessageStore, now: Option<Duration>) -> SensorSummary {
    if now.is_none() {
        return SensorSummary::NoData;
    }
    let rows = devices::rows(store, now);
    let sys = fresh(store, "SYS_STATUS", now, SYS_STATUS_STALE).map(|e| {
        let word = |f: &str| e.text(f).unwrap_or("").to_string();
        sensor_levels(
            &word("onboard_control_sensors_present"),
            &word("onboard_control_sensors_enabled"),
            &word("onboard_control_sensors_health"),
        )
    });
    if rows.iter().all(|r| r.stale) && sys.is_none() {
        return SensorSummary::NoData;
    }
    let mut ok = 0;
    let mut problems = Vec::new();
    for row in &rows {
        if row.stale {
            problems.push((format!("{} stale", row.name), GREY));
        } else if row.state == "OK" {
            ok += 1;
        } else {
            problems.push((
                format!("{} {}", row.name, row.state),
                devices::state_colour(&row.state),
            ));
        }
    }
    for (name, level) in sys.iter().flatten() {
        match level {
            Level::Ok => ok += 1,
            Level::Fault => problems.push((format!("{name} fault"), RED)),
            Level::Off => {}
        }
    }
    if !problems.is_empty() {
        SensorSummary::Problems(problems)
    } else if ok == 0 {
        SensorSummary::NoData
    } else {
        SensorSummary::AllOk(ok)
    }
}

fn sensors_section(ui: &mut egui::Ui, store: &MessageStore, now: Option<Duration>) {
    match sensor_summary(store, now) {
        SensorSummary::NoData => line(ui, GREY, "no sensor data"),
        SensorSummary::AllOk(n) => line(ui, GREEN, format!("All sensors OK ({n})")),
        SensorSummary::Problems(list) => {
            for (text, colour) in list {
                line(ui, colour, text);
            }
        }
    }
}

// ---- Messages ----

fn severity_colour(severity: Severity) -> Option<Color32> {
    match severity {
        Severity::Emergency | Severity::Alert | Severity::Critical | Severity::Error => Some(RED),
        Severity::Warning => Some(YELLOW),
        Severity::Notice | Severity::Info => None,
        Severity::Debug => Some(GREY),
    }
}

fn messages(ui: &mut egui::Ui, state: &VehicleState) {
    let entries = &state.texts.entries;
    if entries.is_empty() {
        line(ui, GREY, "No messages yet");
        return;
    }
    for entry in entries.iter().rev().take(TEXTS_SHOWN) {
        let mut text = RichText::new(format!("{}: {}", entry.severity.name(), entry.text));
        if let Some(colour) = severity_colour(entry.severity) {
            text = text.color(colour);
        }
        ui.label(text);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcs_mavlink::services::messages::{Field, FieldValue, MessageKey};

    fn text(name: &str, value: &str) -> Field {
        Field {
            name: name.into(),
            value: FieldValue::Text(value.into()),
        }
    }

    fn number(name: &str, value: f64) -> Field {
        Field {
            name: name.into(),
            value: FieldValue::Number(value),
        }
    }

    fn entry(fields: Vec<Field>) -> MessageEntry {
        MessageEntry {
            name: "X".into(),
            label: "X".into(),
            count: 1,
            rate_hz: 1.0,
            last_seen: Duration::ZERO,
            fields,
        }
    }

    fn put(store: &mut MessageStore, name: &str, instance: u8, fields: Vec<Field>, seen: u64) {
        store.entries.insert(
            MessageKey {
                system_id: 1,
                component_id: 1,
                message_id: name.len() as u32,
                instance,
            },
            MessageEntry {
                name: name.into(),
                label: name.into(),
                count: 1,
                rate_hz: 1.0,
                last_seen: Duration::from_secs(seen),
                fields,
            },
        );
    }

    fn device(store: &mut MessageStore, id: u8, name: &str, state: &str, seen: u64) {
        put(
            store,
            "MARV_DEVICE_STATUS",
            id,
            vec![
                text("device_id", &format!("MARV_DEVICE_{name}")),
                text("state", &format!("MARV_DEVICE_STATE_{state}")),
            ],
            seen,
        );
    }

    fn sys(store: &mut MessageStore, present: &str, enabled: &str, health: &str, seen: u64) {
        put(
            store,
            "SYS_STATUS",
            0,
            vec![
                text("onboard_control_sensors_present", present),
                text("onboard_control_sensors_enabled", enabled),
                text("onboard_control_sensors_health", health),
            ],
            seen,
        );
    }

    fn secs(s: u64) -> Option<Duration> {
        Some(Duration::from_secs(s))
    }

    const GYRO: &str = "MAV_SYS_STATUS_SENSOR_3D_GYRO";
    const ACCEL: &str = "MAV_SYS_STATUS_SENSOR_3D_ACCEL";
    const GPS: &str = "MAV_SYS_STATUS_SENSOR_GPS";

    #[test]
    fn loss_is_lost_over_all() {
        assert_eq!(loss_percent(0, 0), None);
        assert_eq!(loss_percent(0, 100), Some(0.0));
        assert_eq!(loss_percent(1, 3), Some(25.0));
        assert_eq!(loss_colour(0.0), GREEN);
        assert_eq!(loss_colour(2.0), YELLOW);
        assert_eq!(loss_colour(10.0), RED);
    }

    #[test]
    fn the_link_state_is_short() {
        let mut link = LinkState::default();
        assert_eq!(link_short(&link, None).0, "Not connected");
        assert_eq!(link_short(&link, secs(1)).0, "Connecting…");
        link.status = LinkStatus::Connected;
        assert_eq!(link_short(&link, secs(1)), ("Connected".into(), GREEN));
        link.status = LinkStatus::Lost;
        link.last_heartbeat = secs(4);
        assert_eq!(link_short(&link, secs(10)).0, "Lost (6.0 s silent)");
    }

    #[test]
    fn flags_split_on_bars() {
        assert_eq!(flag_names("A | B"), ["A", "B"]);
        assert!(flag_names("").is_empty());
        assert!(flag_names("0").is_empty());
    }

    #[test]
    fn a_sensor_is_ok_faulty_or_off() {
        let p = format!("{GYRO} | {ACCEL} | {GPS}");
        let e = format!("{GYRO} | {ACCEL}");
        assert_eq!(
            sensor_levels(&p, &e, GYRO),
            [
                ("3D GYRO".to_string(), Level::Ok),
                ("3D ACCEL".to_string(), Level::Fault),
                ("GPS".to_string(), Level::Off)
            ]
        );
    }

    #[test]
    fn all_sensors_ok_is_one_count() {
        let mut store = MessageStore::default();
        device(&mut store, 1, "IMU1", "OK", 9);
        device(&mut store, 2, "BARO", "OK", 9);
        let both = format!("{GYRO} | {ACCEL}");
        sys(&mut store, &both, &both, &both, 9);
        assert_eq!(sensor_summary(&store, secs(10)), SensorSummary::AllOk(4));
    }

    #[test]
    fn a_sensor_that_is_switched_off_is_not_a_problem() {
        let mut store = MessageStore::default();
        let both = format!("{GYRO} | {GPS}");
        sys(&mut store, &both, GYRO, GYRO, 9);
        assert_eq!(sensor_summary(&store, secs(10)), SensorSummary::AllOk(1));
    }

    #[test]
    fn one_fault_lists_only_that_one() {
        let mut store = MessageStore::default();
        device(&mut store, 1, "IMU1", "OK", 9);
        device(&mut store, 2, "BARO", "FAILED", 9);
        let both = format!("{GYRO} | {ACCEL}");
        sys(&mut store, &both, &both, &both, 9);
        assert_eq!(
            sensor_summary(&store, secs(10)),
            SensorSummary::Problems(vec![("BARO FAILED".into(), RED)])
        );
        let mut store = MessageStore::default();
        device(&mut store, 1, "IMU1", "OK", 9);
        sys(
            &mut store,
            &format!("{GYRO} | {ACCEL}"),
            &format!("{GYRO} | {ACCEL}"),
            GYRO,
            9,
        );
        assert_eq!(
            sensor_summary(&store, secs(10)),
            SensorSummary::Problems(vec![("3D ACCEL fault".into(), RED)])
        );
    }

    #[test]
    fn stale_or_no_session_is_no_data() {
        let mut store = MessageStore::default();
        device(&mut store, 1, "IMU1", "OK", 1);
        sys(&mut store, GYRO, GYRO, GYRO, 1);
        assert_eq!(sensor_summary(&store, secs(100)), SensorSummary::NoData);
        assert_eq!(sensor_summary(&store, None), SensorSummary::NoData);
        assert_eq!(
            sensor_summary(&MessageStore::default(), secs(10)),
            SensorSummary::NoData
        );
    }

    #[test]
    fn one_old_row_among_live_ones_is_listed_grey() {
        let mut store = MessageStore::default();
        device(&mut store, 1, "IMU1", "OK", 99);
        device(&mut store, 2, "BARO", "OK", 1);
        assert_eq!(
            sensor_summary(&store, secs(100)),
            SensorSummary::Problems(vec![("BARO stale".into(), GREY)])
        );
    }

    fn esc_info(store: &mut MessageStore, count: f64, bits: f64, seen: u64) {
        put(
            store,
            "ESC_INFO",
            0,
            vec![number("count", count), number("info", bits)],
            seen,
        );
    }

    #[test]
    fn motors_all_online_is_one_line() {
        let mut store = MessageStore::default();
        esc_info(&mut store, 4.0, 15.0, 9);
        assert_eq!(motor_summary(&store, secs(10)), MotorSummary::AllOnline(4));
    }

    #[test]
    fn only_the_offline_motors_are_listed() {
        let mut store = MessageStore::default();
        esc_info(&mut store, 4.0, 11.0, 9);
        assert_eq!(
            motor_summary(&store, secs(10)),
            MotorSummary::Faulty {
                online: 3,
                total: 4,
                lines: vec!["Motor 3: offline".into()]
            }
        );
    }

    #[test]
    fn motors_without_current_data_say_so() {
        let mut store = MessageStore::default();
        assert_eq!(motor_summary(&store, secs(10)), MotorSummary::NoData);
        esc_info(&mut store, 4.0, 15.0, 1);
        assert_eq!(motor_summary(&store, secs(10)), MotorSummary::NoData);
        assert_eq!(motor_summary(&store, None), MotorSummary::NoData);
    }

    #[test]
    fn armed_reads_text_and_number() {
        let flag = "MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED";
        assert_eq!(armed(&entry(vec![text("base_mode", flag)])), Some(true));
        assert_eq!(
            armed(&entry(vec![text(
                "base_mode",
                "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED"
            )])),
            Some(false)
        );
        assert_eq!(armed(&entry(vec![number("base_mode", 129.0)])), Some(true));
        assert_eq!(armed(&entry(vec![number("base_mode", 1.0)])), Some(false));
        assert_eq!(armed(&entry(vec![])), None);
    }

    #[test]
    fn position_needs_a_fix_and_is_not_zero() {
        let fix = |t: &str, lat: f64| {
            entry(vec![
                text("fix_type", t),
                number("lat", lat),
                number("lon", 80_000_000.0),
            ])
        };
        assert_eq!(
            position(&fix("GPS_FIX_TYPE_3D_FIX", 470_000_000.0)),
            Some((47.0, 8.0))
        );
        assert_eq!(position(&fix("GPS_FIX_TYPE_NO_FIX", 470_000_000.0)), None);
        let zero = entry(vec![
            number("fix_type", 3.0),
            number("lat", 0.0),
            number("lon", 0.0),
        ]);
        assert_eq!(position(&zero), None);
    }

    #[test]
    fn battery_and_severity_colours() {
        assert_eq!(battery_colour(10.0), RED);
        assert_eq!(battery_colour(30.0), YELLOW);
        assert_eq!(battery_colour(80.0), GREEN);
        assert_eq!(severity_colour(Severity::Error), Some(RED));
        assert_eq!(severity_colour(Severity::Warning), Some(YELLOW));
        assert_eq!(severity_colour(Severity::Info), None);
    }
}
