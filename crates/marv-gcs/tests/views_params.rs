//! The Parameters view drawn by egui_kittest, no window, over a table built by hand.

use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use eframe::egui::{Key, Modifiers};
use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::services::params::{
    FetchStatus, Param, ParamTable, ParamType, ParamValue, WriteStatus,
};
use gcs_mavlink::{Command, VehicleState};
use marv_gcs::views::{self, View, ViewCtx};

type Commands = Rc<RefCell<Vec<Command>>>;

fn param(name: &str, index: u16, kind: ParamType, raw: [u8; 4]) -> Param {
    Param {
        name: name.to_string(),
        index,
        kind,
        raw,
    }
}

/// Five parameters in two groups, with one more without an underscore.
fn complete_table() -> ParamTable {
    let list = [
        param("AAA_ONE", 0, ParamType::U8, [5, 0, 0, 0]),
        param("AAA_TWO", 1, ParamType::F32, 1.5f32.to_le_bytes()),
        param(
            "AAA_THREE",
            2,
            ParamType::I16,
            (-7i16).to_le_bytes().into_array(),
        ),
        param("BBB_ONE", 3, ParamType::U32, 100_000u32.to_le_bytes()),
        param("BBB_TWO", 4, ParamType::U8, [1, 0, 0, 0]),
        param("LONER", 5, ParamType::U8, [2, 0, 0, 0]),
    ];
    let mut table = ParamTable {
        fetch: FetchStatus::Complete,
        encoding_known: true,
        ..ParamTable::default()
    };
    for p in list {
        table.params.insert(p.name.clone(), p);
    }
    table
}

trait IntoArray {
    fn into_array(self) -> [u8; 4];
}

impl IntoArray for [u8; 2] {
    fn into_array(self) -> [u8; 4] {
        [self[0], self[1], 0, 0]
    }
}

fn harness(table: ParamTable, now: Option<Duration>) -> (Harness<'static>, Commands) {
    let commands: Commands = Rc::default();
    let sink = commands.clone();
    let state = VehicleState {
        params: table,
        ..VehicleState::default()
    };
    let mut view = views::all()
        .into_iter()
        .find(|v| v.name() == "Parameters")
        .expect("Parameters view is registered");
    let mut harness = Harness::builder()
        .with_size(eframe::egui::vec2(900.0, 500.0))
        .build_ui(move |ui| {
            let mut out = sink.borrow_mut();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut out,
                now,
            };
            view.ui(ui, &mut cx);
        });
    harness.run();
    (harness, commands)
}

fn connected(table: ParamTable) -> (Harness<'static>, Commands) {
    harness(table, Some(Duration::from_secs(1)))
}

fn type_into(harness: &mut Harness<'static>, name: &str, text: &str) {
    let field = harness.get_by_label(name);
    field.focus();
    harness.run();
    harness.key_down_modifiers(Modifiers::COMMAND, Key::A);
    harness.run();
    harness.get_by_label(name).type_text(text);
    harness.run();
}

#[test]
fn draws_every_fetch_status_and_no_session() {
    let mut table = ParamTable::default();
    let (h, _) = connected(table.clone());
    h.get_by_label("Not loaded");

    table.fetch = FetchStatus::Fetching {
        received: 3,
        expected: 10,
    };
    let (h, _) = connected(table.clone());
    h.get_by_label_contains("3 / 10");

    let mut table = complete_table();
    let (h, _) = connected(table.clone());
    h.get_by_label("6 parameters");

    table.fetch = FetchStatus::Failed("missing 2".into());
    let (h, _) = connected(table.clone());
    h.get_by_label_contains("missing 2");
    h.get_by_label_contains("6 parameters arrived");

    let (h, commands) = harness(table, None);
    h.get_by_label("Not connected");
    assert!(h.get_by_label("Refresh").accesskit_node().is_disabled());
    h.get_by_label("Refresh").click();
    assert!(commands.borrow().is_empty());
}

#[test]
fn names_are_visible_and_search_and_group_filter_them() {
    let (mut h, _) = connected(complete_table());
    for name in [
        "AAA_ONE",
        "AAA_TWO",
        "AAA_THREE",
        "BBB_ONE",
        "BBB_TWO",
        "LONER",
    ] {
        h.get_by_label(name);
    }

    h.get_by_label("Search").focus();
    h.run();
    h.get_by_label("Search").type_text("two");
    h.run();
    h.get_by_label("AAA_TWO");
    h.get_by_label("BBB_TWO");
    assert!(h.query_by_label("AAA_ONE").is_none());
    assert!(h.query_by_label("LONER").is_none());
}

#[test]
fn choosing_a_group_hides_the_others() {
    let (mut h, _) = connected(complete_table());
    h.get_by_label("Group").click();
    h.run();
    h.get_by_label("BBB").click();
    h.run();
    h.get_by_label("BBB_ONE");
    h.get_by_label("BBB_TWO");
    assert!(h.query_by_label("AAA_ONE").is_none());
    assert!(h.query_by_label("LONER").is_none());

    h.get_by_label("Group").click();
    h.run();
    h.get_by_label("(other)").click();
    h.run();
    h.get_by_label("LONER");
    assert!(h.query_by_label("BBB_ONE").is_none());
}

#[test]
fn refresh_sends_the_command() {
    let (h, commands) = connected(complete_table());
    h.get_by_label("Refresh").click();
    let mut h = h;
    h.run();
    let sent = commands.borrow();
    assert_eq!(sent.len(), 1);
    assert!(matches!(sent[0], Command::RefreshParams));
}

#[test]
fn a_valid_edit_is_written_once_in_the_parameters_own_type() {
    let (mut h, commands) = connected(complete_table());
    type_into(&mut h, "value of AAA_ONE", "7");
    h.get_by_label("edited");
    h.get_by_label("Write changes (1)").click();
    h.run();
    {
        let sent = commands.borrow();
        assert_eq!(sent.len(), 1);
        match &sent[0] {
            Command::SetParam { name, value } => {
                assert_eq!(name, "AAA_ONE");
                assert_eq!(*value, ParamValue::U8(7));
            }
            other => panic!("unexpected command {other:?}"),
        }
    }
    // The sent edit stays until the table shows the new value, but it is not sent twice.
    h.get_by_label("edited");
    h.get_by_label("Write changes (0)");
}

#[test]
fn an_invalid_edit_is_shown_and_never_sent() {
    let (mut h, commands) = connected(complete_table());
    type_into(&mut h, "value of AAA_ONE", "300");
    h.get_by_label("invalid");
    assert!(
        h.get_by_label("Write changes (0)")
            .accesskit_node()
            .is_disabled()
    );
    h.get_by_label("Write changes (0)").click();
    h.run();
    assert!(commands.borrow().is_empty());
}

#[test]
fn discard_drops_the_edits() {
    let (mut h, _) = connected(complete_table());
    type_into(&mut h, "value of AAA_ONE", "7");
    h.get_by_label("edited");
    h.get_by_label("Discard changes").click();
    h.run();
    assert!(h.query_by_label("edited").is_none());
    h.get_by_label("Write changes (0)");
}

#[test]
fn a_refused_write_shows_the_value_in_effect() {
    let mut table = complete_table();
    table.writes.insert(
        "AAA_ONE".into(),
        WriteStatus::Refused {
            value: ParamValue::U8(9),
            in_effect: ParamValue::U8(5),
        },
    );
    table.writes.insert(
        "BBB_TWO".into(),
        WriteStatus::Unanswered {
            value: ParamValue::U8(3),
        },
    );
    table.writes.insert(
        "LONER".into(),
        WriteStatus::Pending {
            value: ParamValue::U8(4),
        },
    );
    let (h, _) = connected(table);
    h.get_by_label("refused: vehicle uses 5");
    h.get_by_label("no answer");
    h.get_by_label("writing…");
}

#[test]
fn a_sent_edit_clears_when_the_table_shows_the_value_and_stays_when_refused() {
    let table = Rc::new(RefCell::new(complete_table()));
    let shared = table.clone();
    let mut view = views::all()
        .into_iter()
        .find(|v| v.name() == "Parameters")
        .expect("Parameters view is registered");
    let mut h = Harness::builder()
        .with_size(eframe::egui::vec2(900.0, 500.0))
        .build_ui(move |ui| {
            let state = VehicleState {
                params: shared.borrow().clone(),
                ..VehicleState::default()
            };
            let mut commands = Vec::new();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut commands,
                now: Some(Duration::from_secs(1)),
            };
            view.ui(ui, &mut cx);
        });
    h.run();
    type_into(&mut h, "value of AAA_ONE", "7");
    type_into(&mut h, "value of BBB_TWO", "8");
    h.get_by_label("Write changes (2)").click();
    h.run();

    {
        let mut t = table.borrow_mut();
        t.params.get_mut("AAA_ONE").unwrap().raw = [7, 0, 0, 0];
        t.writes.insert(
            "BBB_TWO".into(),
            WriteStatus::Refused {
                value: ParamValue::U8(8),
                in_effect: ParamValue::U8(1),
            },
        );
    }
    h.run();
    h.get_by_label("Write changes (1)");
    h.get_by_label("refused: vehicle uses 1");
}

type SharedView = Rc<RefCell<Box<dyn View>>>;

/// The Parameters view kept outside the harness, so the test can call `session_changed` on it.
fn shared_harness(table: ParamTable) -> (Harness<'static>, Commands, SharedView) {
    let commands: Commands = Rc::default();
    let sink = commands.clone();
    let state = VehicleState {
        params: table,
        ..VehicleState::default()
    };
    let view: SharedView = Rc::new(RefCell::new(
        views::all()
            .into_iter()
            .find(|v| v.name() == "Parameters")
            .expect("Parameters view is registered"),
    ));
    let shared = view.clone();
    let mut h = Harness::builder()
        .with_size(eframe::egui::vec2(900.0, 500.0))
        .build_ui(move |ui| {
            let mut out = sink.borrow_mut();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut out,
                now: Some(Duration::from_secs(1)),
            };
            shared.borrow_mut().ui(ui, &mut cx);
        });
    h.run();
    (h, commands, view)
}

#[test]
fn a_new_session_starts_without_the_edits() {
    let (mut h, commands, view) = shared_harness(complete_table());
    type_into(&mut h, "value of AAA_ONE", "7");
    h.get_by_label("Write changes (1)");

    view.borrow_mut().session_changed();
    h.run();
    h.get_by_label("Write changes (0)").click();
    h.run();
    assert!(commands.borrow().is_empty(), "{:?}", commands.borrow());
    h.get_by_label("Discard changes");
}

#[test]
fn a_new_session_forgets_what_was_sent_and_the_file_outcome() {
    let (mut h, commands, view) = shared_harness(complete_table());
    type_into(&mut h, "value of AAA_ONE", "7");
    h.get_by_label("Write changes (1)").click();
    h.run();
    assert_eq!(commands.borrow().len(), 1);
    h.get_by_label("Load").click();
    h.run();
    assert!(h.query_by_label_contains("Load failed").is_some());

    view.borrow_mut().session_changed();
    h.run();
    assert!(h.query_by_label_contains("Load failed").is_none());
    type_into(&mut h, "value of AAA_ONE", "7");
    h.get_by_label("Write changes (1)").click();
    h.run();
    assert_eq!(
        commands.borrow().len(),
        2,
        "the same edit is sendable again"
    );
}

#[test]
fn two_clicks_before_the_table_shows_the_write_send_it_once() {
    let (mut h, commands, _view) = shared_harness(complete_table());
    type_into(&mut h, "value of AAA_ONE", "7");
    h.get_by_label("Write changes (1)").click();
    h.run();
    h.get_by_label("Write changes (0)").click();
    h.run();
    let sent = commands.borrow();
    assert_eq!(sent.len(), 1, "{sent:?}");
    assert!(matches!(
        &sent[0],
        Command::SetParam { name, value: ParamValue::U8(7) } if name == "AAA_ONE"
    ));
}
