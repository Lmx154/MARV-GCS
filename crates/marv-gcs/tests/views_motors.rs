//! Headless test of the Motors view with a hand-built message store.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::services::messages::{
    Field, FieldValue, MessageEntry, MessageKey, Series, SeriesKey,
};
use gcs_mavlink::{Access, Command, VehicleState};
use marv_gcs::views::{self, ViewCtx};

fn key(id: u32) -> MessageKey {
    MessageKey {
        system_id: 1,
        component_id: 1,
        message_id: id,
        instance: 0,
    }
}

fn numbers(values: &[(&str, f64)]) -> Vec<Field> {
    values
        .iter()
        .map(|(name, value)| Field {
            name: (*name).into(),
            value: FieldValue::Number(*value),
        })
        .collect()
}

fn entry(name: &str, last_seen_s: u64, fields: Vec<Field>) -> MessageEntry {
    MessageEntry {
        name: name.into(),
        label: name.into(),
        count: 5,
        rate_hz: 1.0,
        last_seen: Duration::from_secs(last_seen_s),
        fields,
    }
}

/// ESC_STATUS and ESC_INFO as the firmware sends them, `info` bits as given, both last seen at the
/// given session seconds.
fn state(info_bits: f64, status_seen: u64, info_seen: u64) -> VehicleState {
    let mut state = VehicleState::default();
    let mut status = numbers(&[
        ("rpm[0]", 3100.0),
        ("rpm[1]", 3200.0),
        ("rpm[2]", 3300.0),
        ("rpm[3]", 3400.0),
    ]);
    for i in 0..4 {
        status.extend(numbers(&[
            (&format!("voltage[{i}]"), 15.0 + 0.5 * i as f64),
            (&format!("current[{i}]"), f64::NAN),
        ]));
    }
    state
        .messages
        .entries
        .insert(key(291), entry("ESC_STATUS", status_seen, status));
    let mut info = numbers(&[("count", 4.0), ("info", info_bits), ("counter", 2000.0)]);
    for i in 0..4 {
        info.extend(numbers(&[
            (&format!("temperature[{i}]"), 3000.0 + 500.0 * i as f64),
            (&format!("error_count[{i}]"), 20.0),
        ]));
    }
    info.push(Field {
        name: "connection_type".into(),
        value: FieldValue::Text("ESC_CONNECTION_TYPE_DSHOT".into()),
    });
    state
        .messages
        .entries
        .insert(key(290), entry("ESC_INFO", info_seen, info));
    // Errors between the two ESC_INFO (1000 exchanges): 0, 10, 20 and 30, i.e. 0 to 3 %.
    let series = |field: &str, a: f64, b: f64| {
        (
            SeriesKey {
                system_id: 1,
                component_id: 1,
                message: "ESC_INFO".into(),
                instance: 0,
                field: field.into(),
            },
            Series {
                points: VecDeque::from([[8.0, a], [9.0, b]]),
            },
        )
    };
    for (field, a, b) in [
        ("time_usec", 1_000_000.0, 2_000_000.0),
        ("counter", 1000.0, 2000.0),
        ("error_count[0]", 0.0, 0.0),
        ("error_count[1]", 5.0, 15.0),
        ("error_count[2]", 0.0, 20.0),
        ("error_count[3]", 7.0, 37.0),
    ] {
        let (k, s) = series(field, a, b);
        state.messages.series.insert(k, s);
    }
    state
}

fn view() -> Box<dyn views::View> {
    views::all()
        .into_iter()
        .find(|v| v.name() == "Motors")
        .expect("Motors view is registered")
}

fn harness<'a>(state: &'a VehicleState, now: u64) -> Harness<'a> {
    let mut view = view();
    let mut harness = Harness::builder()
        .with_size(eframe::egui::vec2(1000.0, 800.0))
        .build_ui(move |ui| {
            let mut commands = Vec::new();
            let mut cx = ViewCtx {
                state,
                commands: &mut commands,
                now: Some(Duration::from_secs(now)),
                access: gcs_mavlink::Access::Configure,
            };
            view.ui(ui, &mut cx);
            assert!(
                commands.is_empty(),
                "the Motors view sends none without a click"
            );
        });
    harness.run();
    harness
}

#[test]
fn nothing_received_says_so() {
    let state = VehicleState::default();
    let harness = harness(&state, 10);
    harness.get_by_label("No ESC telemetry from the vehicle");
    assert!(harness.query_by_label_contains("Motor 1").is_none());
}

#[test]
fn four_online_motors_show_their_values() {
    let state = state(15.0, 10, 10);
    let harness = harness(&state, 10);
    harness.get_by_label("ESCs online 4/4");
    harness.get_by_label("Link DShot");
    // Document order is box order: every field must come in element order (the first "Motor " is the
    // motor test heading).
    let in_order = |prefix: &str| -> Vec<String> {
        harness
            .query_all_by_label_contains(prefix)
            .filter_map(|n| n.value())
            .map(|l| l.to_string())
            .collect()
    };
    assert_eq!(
        in_order("Motor ")[1..5].to_vec(),
        ["Motor 1", "Motor 2", "Motor 3", "Motor 4"]
    );
    assert_eq!(
        in_order("Voltage"),
        [
            "Voltage 15.00 V",
            "Voltage 15.50 V",
            "Voltage 16.00 V",
            "Voltage 16.50 V"
        ]
    );
    assert_eq!(
        in_order("Temperature"),
        [
            "Temperature 30.0 °C",
            "Temperature 35.0 °C",
            "Temperature 40.0 °C",
            "Temperature 45.0 °C"
        ]
    );
    assert_eq!(
        in_order("Errors"),
        [
            "Errors 0.0 %",
            "Errors 1.0 %",
            "Errors 2.0 %",
            "Errors 3.0 %"
        ]
    );
    assert_eq!(in_order("Current"), ["Current n/a"; 4]);
    for rpm in ["3100", "3200", "3300", "3400"] {
        harness.get_by_label(rpm);
    }
    let rpms: Vec<String> = in_order("")
        .into_iter()
        .filter(|l| l.len() == 4 && l.chars().all(|c| c.is_ascii_digit()))
        .collect();
    assert_eq!(rpms, ["3100", "3200", "3300", "3400"]);
    assert!(harness.query_by_label_contains("OFFLINE").is_none());
    assert!(harness.query_by_label_contains("(stale)").is_none());
}

#[test]
fn an_offline_motor_says_offline() {
    // Motor 3 (bit 2) has no valid reply.
    let state = state(0b1011 as f64, 10, 10);
    let harness = harness(&state, 10);
    harness.get_by_label("ESCs online 3/4");
    harness.get_by_label("Motor 3 OFFLINE");
    assert_eq!(harness.query_all_by_label_contains("OFFLINE").count(), 1);
}

#[test]
fn a_message_older_than_two_seconds_is_not_shown_as_fresh() {
    let state = state(15.0, 10, 10);
    let harness = harness(&state, 13);
    harness.get_by_label_contains("(stale)");
}

const SWITCH: &str = "Propellers are removed and I accept the risk";

type Sent = Rc<RefCell<Vec<Command>>>;

/// The view kept across frames, with every command it pushes collected in `sent`.
fn harness_sending<'a>(state: &'a VehicleState, access: Access, sent: &Sent) -> Harness<'a> {
    let mut view = view();
    let sent = Rc::clone(sent);
    let mut harness = Harness::builder()
        .with_size(eframe::egui::vec2(1000.0, 800.0))
        .build_ui(move |ui| {
            let mut commands = Vec::new();
            let mut cx = ViewCtx {
                state,
                commands: &mut commands,
                now: Some(Duration::from_secs(10)),
                access,
            };
            view.ui(ui, &mut cx);
            sent.borrow_mut().append(&mut commands);
        });
    harness.run();
    harness
}

fn params_of(command: &Command) -> (u32, [f32; 7]) {
    match command {
        Command::SendCommand { command, params } => (*command as u32, *params),
        other => panic!("not a SendCommand: {other:?}"),
    }
}

fn spin_buttons_disabled(harness: &Harness<'_>) -> Vec<bool> {
    harness
        .get_all_by_label("Spin")
        .map(|n| n.accesskit_node().is_disabled())
        .collect()
}

#[test]
fn spin_needs_the_switch_and_a_configure_session() {
    let state = state(15.0, 10, 10);
    let sent = Sent::default();
    let mut harness = harness_sending(&state, Access::Configure, &sent);
    harness.get_by_label("Motor test (propellers off)");
    assert_eq!(spin_buttons_disabled(&harness), [true; 4]);
    harness.get_by_label_contains("Spin disabled: switch on");
    assert!(!harness.get_by_label("Stop").accesskit_node().is_disabled());
    harness.get_by_label(SWITCH).click();
    harness.run();
    assert_eq!(spin_buttons_disabled(&harness), [false; 4]);
    assert!(harness.query_by_label_contains("Spin disabled").is_none());
    assert!(sent.borrow().is_empty());
}

#[test]
fn on_an_operate_session_every_test_control_is_disabled_with_the_reason() {
    let state = state(15.0, 10, 10);
    let sent = Sent::default();
    let mut harness = harness_sending(&state, Access::Operate, &sent);
    harness.get_by_label(SWITCH).click();
    harness.run();
    assert_eq!(spin_buttons_disabled(&harness), [true; 4]);
    assert!(harness.get_by_label("Stop").accesskit_node().is_disabled());
    harness.get_by_label_contains("Configure needs a serial or TCP link");
    assert!(sent.borrow().is_empty());
}

#[test]
fn a_click_on_motor_3s_spin_sends_exactly_one_command_with_the_two_fields() {
    let state = state(15.0, 10, 10);
    let sent = Sent::default();
    let mut harness = harness_sending(&state, Access::Configure, &sent);
    harness.get_by_label(SWITCH).click();
    harness.run();
    harness
        .get_all_by_label("Spin")
        .nth(2)
        .expect("a third Spin button")
        .click();
    harness.run();
    harness.run();
    let sent = sent.borrow();
    assert_eq!(sent.len(), 1, "{sent:?}");
    assert_eq!(
        params_of(&sent[0]),
        (209, [3.0, 0.0, 5.0, 2.0, 1.0, 2.0, 0.0])
    );
}

#[test]
fn stop_needs_no_switch_and_addresses_the_motor_last_spun() {
    let state = state(15.0, 10, 10);
    let sent = Sent::default();
    let mut harness = harness_sending(&state, Access::Configure, &sent);
    harness.get_by_label("Stop").click();
    harness.run();
    assert_eq!(sent.borrow().len(), 1);
    assert_eq!(
        params_of(&sent.borrow()[0]),
        (209, [1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.0])
    );
    harness.get_by_label(SWITCH).click();
    harness.run();
    harness
        .get_all_by_label("Spin")
        .nth(1)
        .expect("a second Spin button")
        .click();
    harness.run();
    harness.get_by_label("Stop").click();
    harness.run();
    let sent = sent.borrow();
    assert_eq!(sent.len(), 3, "{sent:?}");
    let (command, params) = params_of(&sent[2]);
    assert_eq!((command, params[0], params[2]), (209, 2.0, 0.0));
}

#[test]
fn a_refused_test_shows_the_result_and_the_vehicles_text_in_the_strip() {
    use gcs_mavlink::services::texts::{Severity, TextEntry};
    use gcs_mavlink::{CommandRecord, CommandStatus};
    let mut state = state(15.0, 10, 10);
    state.command = Some(CommandRecord {
        command: 209,
        name: None,
        params: [2.0, 0.0, 5.0, 2.0, 1.0, 2.0, 0.0],
        status: CommandStatus::Finished {
            text: "denied",
            accepted: false,
        },
        sends: 1,
        started: Duration::from_secs(8),
        ended: Some(Duration::from_secs(9)),
        refused: None,
        cancel: None,
    });
    state.texts.entries.push_back(TextEntry {
        time: Duration::from_secs(9),
        severity: Severity::Warning,
        text: "motor test: throttle must be 0 to 5 percent".into(),
        system_id: 1,
        component_id: 1,
    });
    let sent = Sent::default();
    let harness = harness_sending(&state, Access::Configure, &sent);
    harness.get_by_label("Motor 2 spin: denied");
    harness.get_by_label("Vehicle: motor test: throttle must be 0 to 5 percent");
}
