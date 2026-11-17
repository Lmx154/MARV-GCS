//! Headless test of the HUD with a hand-built message store, alone and in the window's layout.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use gcs_mavlink::services::messages::{Field, FieldValue, MessageEntry, MessageKey};
use gcs_mavlink::{LinkState, LinkStatus, VehicleState};
use marv_gcs::app::App;
use marv_gcs::cli::Cli;
use marv_gcs::views::{self, ViewCtx};

fn number(name: &str, v: f64) -> Field {
    Field {
        name: name.into(),
        value: FieldValue::Number(v),
    }
}

fn put(state: &mut VehicleState, id: u32, name: &str, fields: Vec<Field>, seen: f64) {
    state.messages.entries.insert(
        MessageKey {
            system_id: 1,
            component_id: 1,
            message_id: id,
            instance: 0,
        },
        MessageEntry {
            name: name.into(),
            label: name.into(),
            count: 1,
            rate_hz: 1.0,
            last_seen: Duration::from_secs_f64(seen),
            fields,
        },
    );
}

fn connected() -> VehicleState {
    VehicleState {
        link: LinkState {
            status: LinkStatus::Connected,
            ..LinkState::default()
        },
        ..VehicleState::default()
    }
}

fn with_attitude(state: &mut VehicleState, roll: f64, pitch: f64, yaw: f64, seen: f64) {
    put(
        state,
        30,
        "ATTITUDE",
        vec![
            number("roll", roll.to_radians()),
            number("pitch", pitch.to_radians()),
            number("yaw", yaw.to_radians()),
        ],
        seen,
    );
}

fn with_gps(state: &mut VehicleState, vel: f64, alt_mm: f64, seen: f64) {
    put(
        state,
        24,
        "GPS_RAW_INT",
        vec![
            Field {
                name: "fix_type".into(),
                value: FieldValue::Text("GPS_FIX_TYPE_3D_FIX".into()),
            },
            number("vel", vel),
            number("cog", 9000.0),
            number("alt", alt_mm),
            number("satellites_visible", 12.0),
        ],
        seen,
    );
}

fn draw(state: &VehicleState, now: Option<Duration>, check: impl FnOnce(&Harness<'_>)) {
    let mut harness = Harness::new_ui(|ui| {
        let mut commands = Vec::new();
        let cx = ViewCtx {
            state,
            commands: &mut commands,
            now,
            access: gcs_mavlink::Access::Operate,
        };
        views::hud::ui(ui, cx.state, cx.now);
        assert!(commands.is_empty(), "the HUD is read only");
    });
    harness.run();
    check(&harness);
}

fn secs(s: u64) -> Option<Duration> {
    Some(Duration::from_secs(s))
}

#[test]
fn attitude_values_are_shown_in_degrees() {
    let mut state = connected();
    with_attitude(&mut state, 20.0, -10.0, 90.0, 9.0);
    draw(&state, secs(10), |h| {
        h.get_by_label("Roll 20.0°");
        h.get_by_label("Pitch -10.0°");
        h.get_by_label("Heading 90°");
        assert!(h.query_by_label("no attitude from the vehicle").is_none());
        h.get_by_label("Link: connected");
    });
}

#[test]
fn a_negative_yaw_is_a_heading_from_0_to_359() {
    let mut state = connected();
    with_attitude(&mut state, 0.0, 0.0, -90.0, 9.0);
    draw(&state, secs(10), |h| {
        h.get_by_label("Heading 270°");
    });
}

#[test]
fn no_attitude_says_so() {
    draw(&connected(), secs(10), |h| {
        h.get_by_label("no attitude from the vehicle");
        h.get_by_label("Roll --°");
        h.get_by_label("Heading --");
    });
}

#[test]
fn an_attitude_older_than_two_seconds_says_so_and_two_is_not_old() {
    let mut state = connected();
    with_attitude(&mut state, 20.0, -10.0, 90.0, 7.0);
    draw(&state, secs(10), |h| {
        h.get_by_label("no attitude from the vehicle");
        assert!(h.query_by_label("Roll 20.0°").is_none());
    });
    let mut state = connected();
    with_attitude(&mut state, 20.0, -10.0, 90.0, 8.0);
    draw(&state, secs(10), |h| {
        h.get_by_label("Roll 20.0°");
    });
}

#[test]
fn gps_speed_and_altitude_are_converted() {
    let mut state = connected();
    with_gps(&mut state, 1234.0, 488_000.0, 9.0);
    draw(&state, secs(10), |h| {
        h.get_by_label("12.3 m/s");
        h.get_by_label("488.0 m");
        h.get_by_label("GPS: 3D FIX, 12 satellites");
    });
}

#[test]
fn unknown_or_old_gps_values_are_dashes() {
    let mut state = connected();
    with_gps(&mut state, 65_535.0, 488_000.0, 9.0);
    draw(&state, secs(10), |h| {
        h.get_by_label("--");
        h.get_by_label("488.0 m");
    });
    let mut state = connected();
    with_gps(&mut state, 1234.0, 488_000.0, 6.0);
    draw(&state, secs(10), |h| {
        assert_eq!(h.query_all_by_label("--").count(), 2);
        assert!(h.query_by_label("12.3 m/s").is_none());
        h.get_by_label("GPS: --, -- satellites");
    });
    draw(&connected(), secs(10), |h| {
        assert_eq!(h.query_all_by_label("--").count(), 2);
    });
}

#[test]
fn the_battery_line_is_the_sensors_tabs() {
    let mut state = connected();
    put(
        &mut state,
        1,
        "SYS_STATUS",
        vec![
            number("voltage_battery", 16_000.0),
            number("current_battery", 120.0),
        ],
        9.0,
    );
    draw(&state, secs(10), |h| {
        h.get_by_label("Battery: 16.00 V, 1.20 A");
    });
}

#[test]
fn not_connected_says_so() {
    let mut state = connected();
    state.link.status = LinkStatus::Lost;
    draw(&state, secs(10), |h| {
        h.get_by_label("Link: not connected");
    });
    draw(&VehicleState::default(), None, |h| {
        h.get_by_label("Link: not connected");
        h.get_by_label("no attitude from the vehicle");
    });
}

#[test]
fn the_hud_is_on_every_tab() {
    let names: Vec<_> = views::all().iter().map(|v| v.name()).collect();
    assert!(names.len() >= 2);
    for index in [0, 1, names.len() - 1] {
        let cli = Cli {
            view: Some(index),
            no_log: true,
            ..Cli::default()
        };
        let ctx = eframe::egui::Context::default();
        let mut app = App::with_context(&ctx, cli, Arc::new(Mutex::new(None)));
        let mut harness = Harness::new_ui(|ui| app.frame(ui));
        harness.run();
        harness.get_by_label("no attitude from the vehicle");
        harness.get_by_label("Link: not connected");
    }
}
