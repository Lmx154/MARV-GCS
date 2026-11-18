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
use gcs_mavlink::{Access, Command, VehicleState};
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
    harness_access(table, now, Access::Configure)
}

/// Open the groups of `complete_table` that are shown (they start closed).
fn open_groups(harness: &mut Harness<'static>) {
    for title in ["AAA (3)", "BBB (2)", "(other) (1)"] {
        if let Some(header) = harness.query_by_label(title) {
            header.click();
            harness.run();
        }
    }
}

fn harness_access(
    table: ParamTable,
    now: Option<Duration>,
    access: Access,
) -> (Harness<'static>, Commands) {
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
                access,
            };
            view.ui(ui, &mut cx);
        });
    harness.run();
    open_groups(&mut harness);
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
fn names_are_visible_and_search_filters_them() {
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

/// Names of three scopes: two groups of the flight controller (with `LOG_BACKEND_TYPE` and one name
/// without an underscore), the RC controller with `RC10` after `RC2`, and nothing for the ESC.
fn scoped_table() -> ParamTable {
    let mut table = ParamTable {
        fetch: FetchStatus::Complete,
        encoding_known: true,
        ..ParamTable::default()
    };
    for (i, name) in [
        "BARO_MIN",
        "BARO_OFFSET",
        "LOG_BACKEND_TYPE",
        "LONER",
        "RC10_MIN",
        "RC1_MAX",
        "RC1_MIN",
        "RC2_MIN",
        "RCMAP_ROLL",
    ]
    .into_iter()
    .enumerate()
    {
        let p = param(name, i as u16, ParamType::U8, [5, 0, 0, 0]);
        table.params.insert(p.name.clone(), p);
    }
    table
}

#[test]
fn the_three_scopes_are_shown_with_their_counts_and_the_empty_one_says_so() {
    let (h, _) = connected(scoped_table());
    h.get_by_label("Flight controller (4)");
    h.get_by_label("RC controller (5)");
    h.get_by_label("ESC (0)");
    h.get_by_label("no parameters from the vehicle yet");
    for group in [
        "BARO (2)",
        "LOG (1)",
        "(other) (1)",
        "RC1 (2)",
        "RC2 (1)",
        "RC10 (1)",
        "RCMAP (1)",
    ] {
        h.get_by_label(group);
    }
    let top = |label: &str| h.get_by_label(label).rect().min.y;
    assert!(top("Flight controller (4)") < top("RC controller (5)"));
    assert!(top("RC controller (5)") < top("ESC (0)"));
    assert!(top("RC2 (1)") < top("RC10 (1)"), "natural order");
}

#[test]
fn a_group_shows_its_rows_only_while_it_is_open() {
    let (mut h, _) = connected(scoped_table());
    assert!(h.query_by_label("BARO_MIN").is_none());
    assert!(h.query_by_label("RC1_MIN").is_none());

    h.get_by_label("BARO (2)").click();
    h.run();
    h.get_by_label("BARO_MIN");
    h.get_by_label("BARO_OFFSET");
    assert!(h.query_by_label("RC1_MIN").is_none());

    h.get_by_label("BARO (2)").click();
    h.run();
    assert!(h.query_by_label("BARO_MIN").is_none());
}

#[test]
fn searching_opens_what_matches_hides_the_rest_and_clearing_brings_back_the_closed_tree() {
    let (mut h, _) = connected(scoped_table());
    h.get_by_label("Search").focus();
    h.run();
    h.get_by_label("Search").type_text("MIN");
    h.run();
    for name in ["BARO_MIN", "RC1_MIN", "RC2_MIN", "RC10_MIN"] {
        h.get_by_label(name);
    }
    for name in [
        "BARO_OFFSET",
        "RC1_MAX",
        "RCMAP_ROLL",
        "LOG_BACKEND_TYPE",
        "LONER",
    ] {
        assert!(h.query_by_label(name).is_none(), "{name}");
    }
    h.get_by_label("Flight controller (1 of 4)");
    h.get_by_label("RC controller (3 of 5)");
    h.get_by_label("ESC (0 of 0)");
    h.get_by_label("BARO (1 of 2)");
    h.get_by_label("RC1 (1 of 2)");
    assert!(h.query_by_label("LOG (0 of 1)").is_none());
    assert!(h.query_by_label("RCMAP (0 of 1)").is_none());

    h.get_by_label("Search").focus();
    h.run();
    h.key_down_modifiers(Modifiers::COMMAND, Key::A);
    h.run();
    h.get_by_label("Search").type_text("");
    h.key_press(Key::Backspace);
    h.run();
    h.get_by_label("Flight controller (4)");
    assert!(h.query_by_label("BARO_MIN").is_none());
    assert!(h.query_by_label("RC1_MIN").is_none());
}

#[test]
fn a_value_edited_inside_a_group_is_written_as_before() {
    let (mut h, commands) = connected(scoped_table());
    h.get_by_label("RC1 (2)").click();
    h.run();
    type_into(&mut h, "value of RC1_MIN", "7");
    h.get_by_label("Write changes (1)").click();
    h.run();
    let sent = commands.borrow();
    assert_eq!(sent.len(), 1, "{sent:?}");
    assert!(matches!(
        &sent[0],
        Command::SetParam { name, value: ParamValue::U8(7) } if name == "RC1_MIN"
    ));
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
                access: gcs_mavlink::Access::Configure,
            };
            view.ui(ui, &mut cx);
        });
    h.run();
    open_groups(&mut h);
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
                access: gcs_mavlink::Access::Configure,
            };
            shared.borrow_mut().ui(ui, &mut cx);
        });
    h.run();
    open_groups(&mut h);
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

const REASON: &str = "Configure needs a serial or TCP link; this link is operate-only";

#[test]
fn on_an_operate_link_every_configure_control_is_shown_disabled_with_the_reason() {
    let (mut h, commands) = harness_access(
        complete_table(),
        Some(Duration::from_secs(1)),
        Access::Operate,
    );
    h.get_by_label(REASON);
    for label in [
        "value of AAA_ONE",
        "value of BBB_ONE",
        "Write changes (0)",
        "Load",
    ] {
        assert!(
            h.get_by_label(label).accesskit_node().is_disabled(),
            "{label} must be shown, and disabled"
        );
    }
    assert!(!h.get_by_label("Refresh").accesskit_node().is_disabled());
    assert!(!h.get_by_label("Save").accesskit_node().is_disabled());
    h.get_by_label("Write changes (0)").click();
    h.get_by_label("Load").click();
    h.run();
    assert!(commands.borrow().is_empty());
}

#[test]
fn on_a_configure_link_the_configure_controls_are_enabled_and_no_reason_is_shown() {
    let (mut h, _) = connected(complete_table());
    assert!(h.query_by_label(REASON).is_none());
    assert!(
        !h.get_by_label("value of AAA_ONE")
            .accesskit_node()
            .is_disabled()
    );
    assert!(!h.get_by_label("Load").accesskit_node().is_disabled());
    type_into(&mut h, "value of AAA_ONE", "7");
    assert!(
        !h.get_by_label("Write changes (1)")
            .accesskit_node()
            .is_disabled()
    );
}

#[test]
fn an_operate_link_that_has_not_downloaded_says_so_and_a_refresh_is_allowed() {
    let (mut h, commands) = harness_access(
        ParamTable::default(),
        Some(Duration::from_secs(1)),
        Access::Operate,
    );
    h.get_by_label("Not loaded: press Refresh to download the list");
    h.get_by_label("Refresh").click();
    h.run();
    assert!(matches!(
        commands.borrow().as_slice(),
        [Command::RefreshParams]
    ));
}

fn firmware_table() -> ParamTable {
    let mut table = ParamTable {
        fetch: FetchStatus::Complete,
        encoding_known: true,
        ..ParamTable::default()
    };
    for (i, (name, kind, raw)) in [
        ("MAG_OFFX", ParamType::F32, 0.0f32.to_le_bytes()),
        ("MAG_AVG", ParamType::I32, 2i32.to_le_bytes()),
        ("MAG_UNLISTED", ParamType::I32, 1i32.to_le_bytes()),
    ]
    .into_iter()
    .enumerate()
    {
        table
            .params
            .insert(name.to_string(), param(name, i as u16, kind, raw));
    }
    table
}

fn firmware_harness() -> Harness<'static> {
    let (mut harness, _) = connected(firmware_table());
    harness.get_by_label("MAG (3)").click();
    harness.run();
    harness
}

#[test]
fn a_known_parameters_unit_is_shown_in_its_own_column() {
    let harness = firmware_harness();
    harness.get_by_label("Unit");
    harness.get_by_label("MAG_OFFX");
    harness.get_by_label("uT");
    assert_eq!(harness.query_all_by_label("uT").count(), 1);
}

#[test]
fn hovering_a_known_name_shows_its_description_and_range_and_an_unknown_one_nothing() {
    let mut harness = firmware_harness();
    assert!(harness.query_by_label_contains("Allowed:").is_none());

    let name = harness.get_by_label("MAG_OFFX").rect();
    harness.hover_at(name.center());
    harness.run();
    harness.get_by_label_contains("Compass offset of the X axis");
    harness.get_by_label_contains("Allowed: -200 to 200");

    harness.hover_at(eframe::egui::pos2(0.0, 0.0));
    harness.run();
    assert!(harness.query_by_label_contains("Allowed:").is_none());

    let unlisted = harness.get_by_label("MAG_UNLISTED").rect();
    harness.hover_at(unlisted.center());
    harness.run();
    assert!(harness.query_by_label_contains("Allowed:").is_none());
    assert_eq!(harness.query_all_by_label("uT").count(), 1);
}
