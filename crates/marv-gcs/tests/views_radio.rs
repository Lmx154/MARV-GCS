//! Headless test of the Radio view with a hand-built message store.

use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use gcs_mavlink::VehicleState;
use gcs_mavlink::services::messages::{Field, FieldValue, MessageEntry, MessageKey};
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
            access: gcs_mavlink::Access::Operate,
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
