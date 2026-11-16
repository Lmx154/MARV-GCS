//! Headless test of the Radio view with a hand-built message store.

use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use gcs_mavlink::Access;
use gcs_mavlink::VehicleState;
use gcs_mavlink::services::messages::{Field, FieldValue, MessageEntry, MessageKey};
use gcs_mavlink::services::params::{FetchStatus, Param, ParamTable, ParamType};
use marv_gcs::views::{self, ViewCtx};

fn number(name: &str, v: f64) -> Field {
    Field {
        name: name.into(),
        value: FieldValue::Number(v),
    }
}

fn key(id: u32, instance: u8) -> MessageKey {
    MessageKey {
        system_id: 1,
        component_id: 1,
        message_id: id,
        instance,
    }
}

fn state(chancount: f64, seen: u64) -> VehicleState {
    let mut state = VehicleState::default();
    let mut fields: Vec<Field> = (1..=16)
        .map(|i| number(&format!("chan{i}_raw"), 1000.0 + 10.0 * i as f64 + 1.0))
        .collect();
    fields.push(number("chancount", chancount));
    fields.push(number("rssi", 255.0));
    state.messages.entries.insert(
        key(65, 0),
        MessageEntry {
            name: "RC_CHANNELS".into(),
            label: "RC_CHANNELS".into(),
            count: 5,
            rate_hz: 5.0,
            last_seen: Duration::from_secs(seen),
            fields,
        },
    );
    state.messages.entries.insert(
        key(42_421, 7),
        MessageEntry {
            name: "MARV_DEVICE_STATUS".into(),
            label: "MARV_DEVICE_STATUS[7]".into(),
            count: 1,
            rate_hz: 1.0,
            last_seen: Duration::from_secs(seen),
            fields: vec![
                Field {
                    name: "device_id".into(),
                    value: FieldValue::Text("MARV_DEVICE_RC".into()),
                },
                Field {
                    name: "state".into(),
                    value: FieldValue::Text("MARV_DEVICE_STATE_OK".into()),
                },
                Field {
                    name: "last_error".into(),
                    value: FieldValue::Text("MARV_DEVICE_ERROR_NONE".into()),
                },
            ],
        },
    );
    state
}

fn draw(state: &VehicleState, now: Option<Duration>, check: impl FnOnce(&Harness<'_>)) {
    draw_as(Access::Operate, state, now, check);
}

fn draw_as(
    access: Access,
    state: &VehicleState,
    now: Option<Duration>,
    check: impl FnOnce(&Harness<'_>),
) {
    let mut view = views::all()
        .into_iter()
        .find(|v| v.name() == "Radio")
        .expect("Radio view is registered");
    let mut harness = Harness::new_ui(|ui| {
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state,
            commands: &mut commands,
            now,
            access,
        };
        view.ui(ui, &mut cx);
        assert!(commands.is_empty(), "the view is read only");
    });
    harness.run();
    check(&harness);
}

#[test]
fn the_radio_tab_follows_the_sensors_tab() {
    let names: Vec<_> = views::all().iter().map(|v| v.name()).collect();
    let sensors = names.iter().position(|n| *n == "Sensors").unwrap();
    assert_eq!(names[sensors + 1], "Radio");
}

#[test]
fn sixteen_channels_with_their_numbers_and_the_row_state() {
    draw(&state(16.0, 10), Some(Duration::from_secs(10)), |h| {
        for i in 1..=16 {
            h.get_by_label(&format!("CH {i:>2}"));
            h.get_by_label(&format!("{}", 1000 + 10 * i + 1));
        }
        h.get_by_label("Link quality: unknown");
        h.get_by_label_contains("RC row: OK, last error NONE");
        h.get_by_label("RADIO_STATUS: none received");
    });
}

#[test]
fn chancount_limits_the_bars() {
    draw(&state(8.0, 10), Some(Duration::from_secs(10)), |h| {
        h.get_by_label("CH  8");
        assert!(h.query_by_label("CH  9").is_none());
    });
}

#[test]
fn old_or_no_data_says_no_stick_data_and_blanks_the_numbers() {
    draw(&state(16.0, 1), Some(Duration::from_secs(10)), |h| {
        h.get_by_label_contains("no stick data");
        assert!(h.query_by_label("1011").is_none());
    });
    draw(&VehicleState::default(), None, |h| {
        h.get_by_label("no stick data");
        h.get_by_label("CH 16");
        h.get_by_label("RC row: no RC row yet");
    });
}

/// A state with one MANUAL_CONTROL (69) seen at `seen` s.
fn state_with_manual_control(x: f64, y: f64, z: f64, r: f64, seen: u64) -> VehicleState {
    let mut state = state(16.0, seen);
    state.messages.entries.insert(
        key(69, 0),
        MessageEntry {
            name: "MANUAL_CONTROL".into(),
            label: "MANUAL_CONTROL".into(),
            count: 5,
            rate_hz: 5.0,
            last_seen: Duration::from_secs(seen),
            fields: vec![
                number("target", 1.0),
                number("x", x),
                number("y", y),
                number("z", z),
                number("r", r),
                number("buttons", 0.0),
            ],
        },
    );
    state
}

#[test]
fn each_function_bar_shows_its_own_field_of_manual_control() {
    // x = pitch, y = roll, z = throttle, r = yaw: four different values, so a swap shows.
    let state = state_with_manual_control(-400.0, 250.0, 600.0, 700.0, 10);
    for access in [Access::Operate, Access::Configure] {
        draw_as(access, &state, Some(Duration::from_secs(11)), |h| {
            h.get_by_label("Roll: 250 (stick right: +)");
            h.get_by_label("Pitch: -400 (stick forward: +)");
            h.get_by_label("Yaw: 700 (yaw right: +)");
            h.get_by_label("Throttle: 600 (throttle up: 1000)");
        });
    }
}

#[test]
fn an_old_or_missing_manual_control_says_no_data() {
    let old = state_with_manual_control(-400.0, 250.0, 600.0, 700.0, 1);
    draw(&old, Some(Duration::from_secs(10)), |h| {
        h.get_by_label("Roll: no data (stick right: +)");
        h.get_by_label("Pitch: no data (stick forward: +)");
        h.get_by_label("Yaw: no data (yaw right: +)");
        h.get_by_label("Throttle: no data (throttle up: 1000)");
        assert!(h.query_by_label_contains("250").is_none());
    });
    // Fresh RC_CHANNELS but no MANUAL_CONTROL at all.
    draw(&state(16.0, 10), Some(Duration::from_secs(10)), |h| {
        h.get_by_label("Roll: no data (stick right: +)");
    });
    // A fresh one with no session clock is not trusted either.
    let fresh = state_with_manual_control(1.0, 2.0, 3.0, 4.0, 10);
    draw(&fresh, None, |h| {
        h.get_by_label("Throttle: no data (throttle up: 1000)");
    });
}

/// A complete parameter list with the RC block: channel n has MIN 1000 + n, TRIM 1500 + n and
/// MAX 2000 + n; `map` is (roll, pitch, throttle, yaw).
fn rc_params(map: [i32; 4], fetch: FetchStatus) -> ParamTable {
    let mut table = ParamTable {
        fetch,
        encoding_known: true,
        ..ParamTable::default()
    };
    let mut add = |name: String, value: i32| {
        let index = table.params.len() as u16;
        table.params.insert(
            name.clone(),
            Param {
                name,
                index,
                kind: ParamType::I32,
                raw: value.to_le_bytes(),
            },
        );
    };
    for n in 1..=8 {
        add(format!("RC{n}_MIN"), 1000 + n);
        add(format!("RC{n}_TRIM"), 1500 + n);
        add(format!("RC{n}_MAX"), 2000 + n);
        add(format!("RC{n}_REVERSED"), 0);
        add(format!("RC{n}_DZ"), 0);
    }
    for (name, channel) in ["RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW"]
        .into_iter()
        .zip(map)
    {
        add(name.to_string(), channel);
    }
    table
}

const NO_RC: &str = "this firmware has no RC mapping parameters";

#[test]
fn channels_1_to_8_show_min_trim_max_and_their_function_from_the_parameters() {
    let mut state = state(16.0, 10);
    // Roll on channel 2, pitch on channel 1, throttle on 3, yaw on 3 as well (a duplicate is accepted).
    state.params = rc_params([2, 1, 3, 3], FetchStatus::Complete);
    for access in [Access::Operate, Access::Configure] {
        draw_as(access, &state, Some(Duration::from_secs(10)), |h| {
            h.get_by_label("MIN 1001 TRIM 1501 MAX 2001  [Pitch]");
            h.get_by_label("MIN 1002 TRIM 1502 MAX 2002  [Roll]");
            h.get_by_label("MIN 1003 TRIM 1503 MAX 2003  [Throttle, Yaw]");
            h.get_by_label("MIN 1004 TRIM 1504 MAX 2004");
            h.get_by_label("MIN 1008 TRIM 1508 MAX 2008");
            assert!(
                h.query_by_label_contains("MIN 1009").is_none(),
                "no markers past channel 8"
            );
            assert!(h.query_by_label(NO_RC).is_none());
        });
    }
}

#[test]
fn no_markers_until_the_parameter_list_is_complete() {
    let mut state = state(16.0, 10);
    state.params = rc_params(
        [1, 2, 3, 4],
        FetchStatus::Fetching {
            received: 10,
            expected: 44,
        },
    );
    draw(&state, Some(Duration::from_secs(10)), |h| {
        assert!(h.query_by_label_contains("MIN 1001").is_none());
        assert!(h.query_by_label(NO_RC).is_none());
    });
}

#[test]
fn a_vehicle_without_rc1_min_says_it_has_no_rc_mapping_parameters() {
    let mut state = state(16.0, 10);
    let mut table = rc_params([1, 2, 3, 4], FetchStatus::Complete);
    table.params.remove("RC1_MIN");
    state.params = table;
    draw(&state, Some(Duration::from_secs(10)), |h| {
        h.get_by_label(NO_RC);
        assert!(h.query_by_label_contains("TRIM 1501").is_none());
    });
    // A complete list with no RC block at all.
    state.params = ParamTable {
        fetch: FetchStatus::Complete,
        ..ParamTable::default()
    };
    draw(&state, Some(Duration::from_secs(10)), |h| {
        h.get_by_label(NO_RC);
    });
}
