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

/// The width the side panel opens with.
const WIDTH: f32 = 320.0;

fn draw(state: &VehicleState, now: Option<Duration>, check: impl FnOnce(&Harness<'_>)) {
    draw_at(WIDTH, state, now, check);
}

/// The panel's content in a column `width` wide, as in a panel dragged to that width, in a window tall
/// enough that the drawing's height cap does not bite.
fn draw_at(
    width: f32,
    state: &VehicleState,
    now: Option<Duration>,
    check: impl FnOnce(&Harness<'_>),
) {
    let mut harness = Harness::builder()
        .with_size(eframe::egui::vec2(800.0, TALL))
        .build_ui(|ui| {
            ui.set_max_width(width);
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

/// A window height at which the drawing is as wide as the panel.
const TALL: f32 = 1400.0;

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
        h.get_by_label("Connected");
    });
}

#[test]
fn the_text_lines_start_below_the_drawing() {
    let mut state = connected();
    with_attitude(&mut state, 20.0, -10.0, 90.0, 9.0);
    with_gps(&mut state, 1234.0, 488_000.0, 9.0);
    draw(&state, secs(10), |h| {
        let drawing = h.get_by_label(views::hud::DRAWING_NAME).rect();
        assert!(drawing.height() >= 200.0, "{drawing:?}");
        for text in [
            "Roll 20.0°",
            "Pitch -10.0°",
            "Heading 90°",
            "GPS: 3D FIX, 12 satellites",
            "Connected",
        ] {
            let rect = h.get_by_label(text).rect();
            assert!(
                rect.top() >= drawing.bottom(),
                "{text}: {rect:?} vs {drawing:?}"
            );
        }
        for inside in ["12.3 m/s", "488.0 m"] {
            let rect = h.get_by_label(inside).rect();
            assert!(
                drawing.contains_rect(rect),
                "{inside}: {rect:?} vs {drawing:?}"
            );
        }
    });
    draw(&connected(), secs(10), |h| {
        let drawing = h.get_by_label(views::hud::DRAWING_NAME).rect();
        let rect = h.get_by_label("no attitude from the vehicle").rect();
        assert!(drawing.contains_rect(rect), "{rect:?} vs {drawing:?}");
        let roll = h.get_by_label("Roll --°").rect();
        assert!(roll.top() >= drawing.bottom(), "{roll:?} vs {drawing:?}");
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
        h.get_by_label("Lost (10.0 s silent)");
    });
    draw(&VehicleState::default(), None, |h| {
        h.get_by_label("Not connected");
        h.get_by_label("no sensor data");
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
        harness.get_by_label("Heartbeat: no data");
        harness.get_by_label("no sensor data");
    }
}

fn text_field(name: &str, v: &str) -> Field {
    Field {
        name: name.into(),
        value: FieldValue::Text(v.into()),
    }
}

fn with_device(state: &mut VehicleState, id: u8, name: &str, device_state: &str, seen: f64) {
    state.messages.entries.insert(
        MessageKey {
            system_id: 1,
            component_id: 1,
            message_id: 1000,
            instance: id,
        },
        MessageEntry {
            name: "MARV_DEVICE_STATUS".into(),
            label: "MARV_DEVICE_STATUS".into(),
            count: 1,
            rate_hz: 1.0,
            last_seen: Duration::from_secs_f64(seen),
            fields: vec![
                text_field("device_id", &format!("MARV_DEVICE_{name}")),
                text_field("state", &format!("MARV_DEVICE_STATE_{device_state}")),
            ],
        },
    );
}

#[test]
fn all_sensors_ok_is_one_line() {
    let mut state = connected();
    with_device(&mut state, 1, "IMU1", "OK", 9.0);
    with_device(&mut state, 2, "BARO", "OK", 9.0);
    draw(&state, secs(10), |h| {
        h.get_by_label("All sensors OK (2)");
        assert!(h.query_by_label("IMU1 OK").is_none());
        assert!(h.query_by_label("BARO OK").is_none());
    });
}

#[test]
fn one_faulty_sensor_is_the_only_one_listed() {
    let mut state = connected();
    with_device(&mut state, 1, "IMU1", "OK", 9.0);
    with_device(&mut state, 2, "BARO", "FAILED", 9.0);
    draw(&state, secs(10), |h| {
        h.get_by_label("BARO FAILED");
        assert!(h.query_by_label("IMU1 OK").is_none());
        assert!(h.query_by_label("IMU1").is_none());
        assert!(h.query_by_label_contains("All sensors OK").is_none());
    });
}

#[test]
fn old_sensor_data_is_no_data() {
    let mut state = connected();
    with_device(&mut state, 1, "IMU1", "OK", 1.0);
    draw(&state, secs(100), |h| {
        h.get_by_label("no sensor data");
    });
}

#[test]
fn motors_are_one_line_or_only_the_offline_ones() {
    let mut state = connected();
    put(
        &mut state,
        290,
        "ESC_INFO",
        vec![number("count", 4.0), number("info", 15.0)],
        9.0,
    );
    draw(&state, secs(10), |h| {
        h.get_by_label("4/4 ESCs online");
    });
    put(
        &mut state,
        290,
        "ESC_INFO",
        vec![number("count", 4.0), number("info", 13.0)],
        9.0,
    );
    draw(&state, secs(10), |h| {
        h.get_by_label("3/4 ESCs online");
        h.get_by_label("Motor 2: offline");
        assert!(h.query_by_label_contains("Motor 1").is_none());
    });
}

#[test]
fn the_panel_scales_with_its_width() {
    let mut state = connected();
    with_attitude(&mut state, 20.0, -10.0, 90.0, 9.0);
    with_gps(&mut state, 1234.0, 488_000.0, 9.0);
    with_device(&mut state, 1, "IMU1", "OK", 9.0);
    for width in [240.0, 320.0, 480.0] {
        draw_at(width, &state, secs(10), |h| {
            let drawing = h.get_by_label(views::hud::DRAWING_NAME).rect();
            assert!(
                (drawing.width() - drawing.height()).abs() < 0.5,
                "{width}: {drawing:?} is not square"
            );
            assert!(
                drawing.width() > width - 40.0 && drawing.width() <= width,
                "{width}: {drawing:?}"
            );
            for inside in ["12.3 m/s", "488.0 m"] {
                let rect = h.get_by_label(inside).rect();
                assert!(
                    drawing.contains_rect(rect),
                    "{width}: {inside}: {rect:?} vs {drawing:?}"
                );
            }
            h.get_by_label("All sensors OK (1)");
        });
        draw_at(width, &VehicleState::default(), None, |h| {
            h.get_by_label("no attitude from the vehicle");
        });
    }
}

#[test]
fn the_side_panel_renders_at_its_smallest_and_largest_width() {
    let mut state = connected();
    with_attitude(&mut state, 20.0, -10.0, 90.0, 9.0);
    with_gps(&mut state, 1234.0, 488_000.0, 9.0);
    for width in [views::hud::PANEL_MIN_WIDTH, views::hud::PANEL_MAX_WIDTH] {
        let mut harness = Harness::builder()
            .with_size(eframe::egui::vec2(1000.0, TALL))
            .build_ui(|ui| {
                eframe::egui::Panel::left("hud_test")
                    .exact_size(width)
                    .resizable(false)
                    .show(ui, |ui| views::hud::ui(ui, &state, secs(10)));
            });
        harness.run();
        let drawing = harness.get_by_label(views::hud::DRAWING_NAME).rect();
        assert!(
            drawing.width() > width - 40.0 && drawing.width() < width,
            "{width}: {drawing:?}"
        );
        harness.get_by_label("12.3 m/s");
    }
}

#[test]
fn in_a_short_window_the_drawing_gives_way_to_the_sections() {
    let mut state = connected();
    with_attitude(&mut state, 20.0, -10.0, 90.0, 9.0);
    with_gps(&mut state, 1234.0, 488_000.0, 9.0);
    for (height, capped) in [(700.0, true), (TALL, false)] {
        let mut harness = Harness::builder()
            .with_size(eframe::egui::vec2(1000.0, height))
            .build_ui(|ui| {
                eframe::egui::Panel::left("hud_test")
                    .exact_size(WIDTH)
                    .resizable(false)
                    .show(ui, |ui| views::hud::ui(ui, &state, secs(10)));
            });
        harness.run();
        let drawing = harness.get_by_label(views::hud::DRAWING_NAME).rect();
        assert!(
            (drawing.width() - drawing.height()).abs() < 0.5,
            "{height}: {drawing:?} is not square"
        );
        if capped {
            assert!(
                drawing.height() <= views::hud::MAX_HEIGHT_SHARE * height,
                "{height}: {drawing:?}"
            );
        } else {
            assert!(drawing.width() > WIDTH - 40.0, "{height}: {drawing:?}");
            let last = harness.get_by_label("Recent messages").rect();
            assert!(last.bottom() < height, "{height}: {last:?} is out of view");
        }
        for inside in ["12.3 m/s", "488.0 m"] {
            let rect = harness.get_by_label(inside).rect();
            assert!(
                drawing.contains_rect(rect),
                "{height}: {inside}: {rect:?} vs {drawing:?}"
            );
        }
    }
}
