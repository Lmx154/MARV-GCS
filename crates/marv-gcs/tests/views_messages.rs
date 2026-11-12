//! Headless tests of the Messages and Sensors views with a hand-built message store.

use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use gcs_mavlink::VehicleState;
use gcs_mavlink::services::messages::{
    Field, FieldValue, MessageEntry, MessageKey, MessageStore, Series, SeriesKey,
};
use marv_gcs::views::{self, View, ViewCtx};

fn view(name: &str) -> Box<dyn View> {
    views::all()
        .into_iter()
        .find(|v| v.name() == name)
        .unwrap_or_else(|| panic!("{name} view is registered"))
}

fn number(name: &str, value: f64) -> Field {
    Field {
        name: name.into(),
        value: FieldValue::Number(value),
    }
}

fn entry(name: &str, label: &str, rate_hz: f32, fields: Vec<Field>) -> MessageEntry {
    MessageEntry {
        name: name.into(),
        label: label.into(),
        count: 100,
        rate_hz,
        last_seen: Duration::from_secs(9),
        fields,
    }
}

fn series(points: impl IntoIterator<Item = [f64; 2]>) -> Series {
    Series {
        points: points.into_iter().collect(),
    }
}

fn filled_store() -> MessageStore {
    let mut store = MessageStore {
        lost_frames: 3,
        ..MessageStore::default()
    };
    store.entries.insert(
        MessageKey {
            system_id: 1,
            component_id: 1,
            message_id: 105,
            instance: 2,
        },
        entry(
            "HIGHRES_IMU",
            "HIGHRES_IMU[2]",
            50.0,
            vec![
                number("xacc", 1.5),
                number("yacc", f64::NAN),
                Field {
                    name: "note".into(),
                    value: FieldValue::Text("hello".into()),
                },
            ],
        ),
    );
    store.entries.insert(
        MessageKey {
            system_id: 1,
            component_id: 1,
            message_id: 24,
            instance: 0,
        },
        entry(
            "GPS_RAW_INT",
            "GPS_RAW_INT",
            5.0,
            vec![
                Field {
                    name: "fix_type".into(),
                    value: FieldValue::Text("GPS_FIX_TYPE_3D_FIX".into()),
                },
                number("lat", 473_977_420.0),
                number("lon", 85_455_940.0),
                number("alt", 408_000.0),
                number("satellites_visible", 9.0),
            ],
        ),
    );
    for (message, instance, field) in [
        ("HIGHRES_IMU", 2, "xacc"),
        ("HIGHRES_IMU", 0, "xacc"),
        ("HIGHRES_IMU", 0, "yacc"),
        ("HIGHRES_IMU", 0, "zacc"),
    ] {
        store.series.insert(
            SeriesKey {
                system_id: 1,
                component_id: 1,
                message: message.into(),
                instance,
                field: field.into(),
            },
            series((0..200).map(|i| [f64::from(i) * 0.1, f64::from(i % 7)])),
        );
    }
    store
}

fn run_view(name: &str, state: &VehicleState, now: Option<Duration>) -> Harness<'static> {
    let mut view = view(name);
    // The state must outlive the harness closure; clone it into the closure.
    let state = state.clone();
    let mut harness = Harness::new_ui(move |ui| {
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now,
        };
        view.ui(ui, &mut cx);
    });
    harness.run();
    harness
}

fn state_with(messages: MessageStore) -> VehicleState {
    VehicleState {
        messages,
        ..VehicleState::default()
    }
}

#[test]
fn both_views_draw_empty_and_filled_stores() {
    for name in ["Messages", "Sensors"] {
        run_view(name, &VehicleState::default(), None);
        run_view(
            name,
            &VehicleState::default(),
            Some(Duration::from_secs(10)),
        );
        run_view(name, &state_with(filled_store()), None);
        run_view(
            name,
            &state_with(filled_store()),
            Some(Duration::from_secs(10)),
        );
    }
}

#[test]
fn messages_view_says_so_when_empty() {
    let harness = run_view("Messages", &VehicleState::default(), None);
    harness.get_by_label("No messages yet");
}

#[test]
fn messages_view_shows_a_row_with_rate_age_and_lost_frames() {
    let harness = run_view(
        "Messages",
        &state_with(filled_store()),
        Some(Duration::from_secs(10)),
    );
    harness.get_by_label("HIGHRES_IMU[2]");
    harness.get_by_label("50.0 Hz");
    assert_eq!(harness.get_all_by_label("1.0 s").count(), 2);
    harness.get_by_label("Lost frames: 3");
}

#[test]
fn messages_view_shows_a_dash_for_the_age_without_a_session() {
    let harness = run_view("Messages", &state_with(filled_store()), None);
    assert_eq!(harness.get_all_by_label("—").count(), 2);
}

#[test]
fn clicking_a_row_shows_its_fields_and_they_can_be_plotted() {
    let state = state_with(filled_store());
    let mut view = view("Messages");
    let mut harness = Harness::new_ui(|ui| {
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now: Some(Duration::from_secs(10)),
        };
        view.ui(ui, &mut cx);
    });
    harness.run();
    assert!(harness.query_by_label("xacc").is_none());

    harness.get_by_label("HIGHRES_IMU[2]").click();
    harness.run();
    harness.get_by_label("xacc");
    harness.get_by_label("yacc");
    harness.get_by_label("note");
    harness.get_by_label("1.5");
    harness.get_by_label("NaN");
    harness.get_by_label("hello");

    harness.get_by_label("xacc").click();
    harness.run();
    harness.get_by_label("Clear plot").click();
    harness.run();
    harness.get_by_label("Tick a numeric field to plot it.");
}

#[test]
fn sensors_view_shows_the_gps_line() {
    let harness = run_view(
        "Sensors",
        &state_with(filled_store()),
        Some(Duration::from_secs(10)),
    );
    harness.get_by_label("9 satellites");
    harness.get_by_label("lat 47.3977420°");
    harness.get_by_label("lon 8.5455940°");
    harness.get_by_label("alt 408.0 m");
    harness.get_by_label("fix GPS_FIX_TYPE_3D_FIX");
}

#[test]
fn sensors_view_says_no_data_for_a_preset_without_series() {
    let harness = run_view("Sensors", &state_with(filled_store()), None);
    assert!(harness.query_all_by_label("no data").count() >= 1);
    let empty = run_view("Sensors", &VehicleState::default(), None);
    empty.get_by_label("GPS: no data");
    assert_eq!(empty.get_all_by_label("no data").count(), 7);
}

#[test]
fn a_new_session_drops_the_selection_and_the_ticked_fields() {
    use std::cell::RefCell;
    use std::rc::Rc;

    let state = state_with(filled_store());
    let shared = Rc::new(RefCell::new(view("Messages")));
    let inner = shared.clone();
    let mut harness = Harness::new_ui(move |ui| {
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now: Some(Duration::from_secs(10)),
        };
        inner.borrow_mut().ui(ui, &mut cx);
    });
    harness.run();
    harness.get_by_label("HIGHRES_IMU[2]").click();
    harness.run();
    harness.get_by_label("xacc").click();
    harness.run();
    assert!(
        harness
            .query_by_label("Tick a numeric field to plot it.")
            .is_none()
    );

    shared.borrow_mut().session_changed();
    harness.run();
    harness.get_by_label("Click a message to see its fields.");
    harness.get_by_label("Tick a numeric field to plot it.");
    assert!(harness.query_by_label("xacc").is_none());
    harness.get_by_label("HIGHRES_IMU[2]").click();
    harness.run();
    harness.get_by_label("Tick a numeric field to plot it.");
}
