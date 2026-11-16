//! Headless test of the Radio view with a hand-built message store.

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::Duration;

use eframe::egui;
use eframe::egui::accesskit::Role;
use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::dialect::MavCmd;
use gcs_mavlink::services::messages::{Field, FieldValue, MessageEntry, MessageKey};
use gcs_mavlink::services::params::{FetchStatus, Param, ParamTable, ParamType, ParamValue};
use gcs_mavlink::{Access, Command, CommandRecord, CommandStatus, VehicleState};
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

// The calibration (docs/link-contract.md section 12, "Calibration").

/// A complete list with the contract's defaults: MIN 1100, TRIM 1500, MAX 1900 on channels 1 to 8.
fn default_params() -> ParamTable {
    let mut table = rc_params([1, 2, 3, 4], FetchStatus::Complete);
    for n in 1..=8 {
        for (suffix, value) in [("MIN", 1100), ("TRIM", 1500), ("MAX", 1900)] {
            let param = table.params.get_mut(&format!("RC{n}_{suffix}")).unwrap();
            param.raw = i32::to_le_bytes(value);
        }
    }
    table
}

type Shared<T> = Rc<RefCell<T>>;

/// The Radio view in a window whose state and clock the test changes between frames.
struct Rig {
    harness: Harness<'static>,
    state: Shared<VehicleState>,
    now: Rc<Cell<Duration>>,
    commands: Shared<Vec<Command>>,
}

impl Rig {
    fn new(access: Access) -> Rig {
        let mut vehicle = state(16.0, 10);
        vehicle.params = default_params();
        let state = Rc::new(RefCell::new(vehicle));
        let now = Rc::new(Cell::new(Duration::from_secs(10)));
        let commands: Shared<Vec<Command>> = Rc::default();
        let mut view = views::all()
            .into_iter()
            .find(|v| v.name() == "Radio")
            .expect("Radio view is registered");
        let (s, n, c) = (state.clone(), now.clone(), commands.clone());
        let mut harness = Harness::builder()
            .with_size(egui::vec2(1000.0, 1800.0))
            .build_ui(move |ui| {
                let state = s.borrow();
                let mut out = c.borrow_mut();
                let mut cx = ViewCtx {
                    state: &state,
                    commands: &mut out,
                    now: Some(n.get()),
                    access,
                };
                view.ui(ui, &mut cx);
            });
        harness.run();
        Rig {
            harness,
            state,
            now,
            commands,
        }
    }

    /// One RC_CHANNELS with these microseconds, received now; the clock moves 200 ms.
    fn feed(&mut self, values: [u16; 16]) {
        self.now.set(self.now.get() + Duration::from_millis(200));
        let mut state = self.state.borrow_mut();
        let entry = state.messages.entries.get_mut(&key(65, 0)).unwrap();
        entry.last_seen = self.now.get();
        for (i, v) in values.iter().enumerate() {
            let name = format!("chan{}_raw", i + 1);
            entry
                .fields
                .iter_mut()
                .find(|f| f.name == name)
                .unwrap()
                .value = FieldValue::Number(f64::from(*v));
        }
        drop(state);
        self.harness.run();
    }

    fn click(&mut self, label: &str) {
        self.harness.get_by_label(label).click();
        self.harness.run();
    }

    fn enabled(&self, label: &str) -> bool {
        !self
            .harness
            .get_by_label(label)
            .accesskit_node()
            .is_disabled()
    }

    fn has(&self, text: &str) -> bool {
        self.harness
            .query_all_by_label_contains(text)
            .next()
            .is_some()
    }

    /// The `SetParam`s sent so far, sorted, and the other commands.
    fn sent(&self) -> (Vec<(String, i32)>, Vec<String>) {
        let mut writes = Vec::new();
        let mut others = Vec::new();
        for command in self.commands.borrow().iter() {
            match command {
                Command::SetParam {
                    name,
                    value: ParamValue::I32(v),
                } => writes.push((name.clone(), *v)),
                other => others.push(format!("{other:?}")),
            }
        }
        writes.sort();
        (writes, others)
    }

    /// Calibrate, sweep channels 1 to 4 between 988 and 2012 (9 to 16 too), leave 5 to 8 still, press
    /// Done, centre channels 1 to 4 at `centre` and press OK. Stops at the review.
    fn calibrate(&mut self, centre: u16) {
        let still = |a: u16| {
            let mut v = [1500; 16];
            v[..4].fill(a);
            v[8..].fill(a);
            v
        };
        self.click("Calibrate radio");
        self.feed(still(988));
        self.feed(still(2012));
        self.feed(still(1500));
        self.click("Done");
        let mut at_centre = [1500; 16];
        at_centre[..4].fill(centre);
        self.feed(at_centre);
        self.click("OK");
    }
}

fn names(writes: &[(String, i32)]) -> Vec<&str> {
    writes.iter().map(|(n, _)| n.as_str()).collect()
}

#[test]
fn the_whole_flow_writes_twelve_values_and_nothing_else_and_never_a_save() {
    let mut rig = Rig::new(Access::Configure);
    assert!(rig.enabled("Calibrate radio"));
    assert!(rig.has(views::radio::PREPARE_TEXT));
    rig.calibrate(1520);
    assert!(rig.has("CH  1: MIN 1100 -> 988  TRIM 1500 -> 1520  MAX 1900 -> 2012"));
    assert!(rig.has("CH  5: not moved"));
    assert!(rig.has("CH  9: captured 988 to 2012 (not written"));
    assert!(
        rig.commands.borrow().is_empty(),
        "nothing is sent before Apply"
    );
    rig.click("Apply");
    let (writes, others) = rig.sent();
    let mut want = Vec::new();
    for n in 1..=4 {
        want.push((format!("RC{n}_MAX"), 2012));
        want.push((format!("RC{n}_MIN"), 988));
        want.push((format!("RC{n}_TRIM"), 1520));
    }
    assert_eq!(writes, want);
    assert!(
        others.is_empty(),
        "no command besides the writes: {others:?}"
    );
    assert!(rig.has(views::radio::NOT_SAVED_TEXT));
    assert!(rig.enabled("Save to flash"));
}

#[test]
fn a_value_equal_to_the_vehicles_is_not_written() {
    let mut rig = Rig::new(Access::Configure);
    rig.calibrate(1500);
    rig.click("Apply");
    let (writes, others) = rig.sent();
    assert_eq!(writes.len(), 8, "{writes:?}");
    assert!(names(&writes).iter().all(|n| !n.ends_with("_TRIM")));
    assert!(others.is_empty());
}

#[test]
fn trim_is_clamped_into_min_and_max_and_a_narrow_channel_is_not_moved() {
    let mut rig = Rig::new(Access::Configure);
    rig.click("Calibrate radio");
    let mut v = [1500; 16];
    v[0] = 988;
    v[4] = 1400;
    rig.feed(v);
    v[0] = 2012;
    v[4] = 1600;
    rig.feed(v);
    rig.click("Done");
    v[0] = 2100;
    rig.feed(v);
    rig.click("OK");
    assert!(rig.has("CH  5: not moved"));
    rig.click("Apply");
    let (writes, _) = rig.sent();
    assert_eq!(
        writes,
        [
            ("RC1_MAX".to_string(), 2012),
            ("RC1_MIN".to_string(), 988),
            ("RC1_TRIM".to_string(), 2012),
        ]
    );
}

#[test]
fn cancel_in_every_step_sends_nothing_and_goes_back_to_the_button() {
    for steps in 0..3 {
        let mut rig = Rig::new(Access::Configure);
        rig.click("Calibrate radio");
        rig.feed([988; 16]);
        rig.feed([2012; 16]);
        if steps > 0 {
            rig.click("Done");
        }
        if steps > 1 {
            rig.click("OK");
        }
        rig.click("Cancel");
        assert!(rig.commands.borrow().is_empty(), "step {steps}");
        assert!(rig.enabled("Calibrate radio"), "step {steps}");
    }
}

#[test]
fn an_operate_session_has_the_button_disabled_with_its_reason_and_writes_nothing() {
    let mut rig = Rig::new(Access::Operate);
    assert!(!rig.enabled("Calibrate radio"));
    assert!(rig.has("Configure needs a serial or TCP link"));
    rig.click("Calibrate radio");
    rig.feed([988; 16]);
    rig.feed([2012; 16]);
    assert!(rig.harness.query_by_label("Done").is_none());
    assert!(rig.commands.borrow().is_empty());
}

#[test]
fn stale_sticks_or_an_incomplete_list_disable_the_button_with_the_reason() {
    let mut rig = Rig::new(Access::Configure);
    assert!(rig.enabled("Calibrate radio"));
    rig.now.set(Duration::from_secs(20));
    rig.harness.run();
    assert!(!rig.enabled("Calibrate radio"));
    assert!(rig.harness.query_by_label("no stick data").is_some());
    rig.feed([1500; 16]);
    assert!(rig.enabled("Calibrate radio"));
    rig.state.borrow_mut().params.fetch = FetchStatus::Fetching {
        received: 1,
        expected: 44,
    };
    rig.harness.run();
    assert!(!rig.enabled("Calibrate radio"));
    assert!(rig.has("the parameter list is not complete"));
}

fn storage_record(started: Duration, status: CommandStatus) -> CommandRecord {
    CommandRecord {
        command: 245,
        name: None,
        params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        status,
        sends: 1,
        started,
        ended: None,
        refused: None,
        cancel: None,
    }
}

#[test]
fn save_to_flash_sends_one_command_and_the_line_stays_until_an_accepted_save() {
    let mut rig = Rig::new(Access::Configure);
    // An accepted save from before the Apply does not count.
    rig.state.borrow_mut().command = Some(storage_record(
        Duration::from_secs(1),
        CommandStatus::Finished {
            text: "accepted",
            accepted: true,
        },
    ));
    rig.calibrate(1520);
    rig.click("Apply");
    assert!(rig.has(views::radio::NOT_SAVED_TEXT));
    rig.click("Save to flash");
    let (_, others) = rig.sent();
    assert_eq!(others.len(), 1, "{others:?}");
    assert!(others[0].contains(&format!("{:?}", MavCmd::MAV_CMD_PREFLIGHT_STORAGE)));
    assert!(others[0].contains("[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"));
    let started = rig.now.get();
    rig.state.borrow_mut().command = Some(storage_record(started, CommandStatus::Waiting));
    rig.harness.run();
    assert!(
        rig.has(views::radio::NOT_SAVED_TEXT),
        "waiting is not saved"
    );
    assert!(!rig.enabled("Save to flash"), "a command is running");
    rig.state.borrow_mut().command = Some(storage_record(
        started,
        CommandStatus::Finished {
            text: "denied",
            accepted: false,
        },
    ));
    rig.harness.run();
    assert!(
        rig.has(views::radio::NOT_SAVED_TEXT),
        "a refused save is not saved"
    );
    rig.state.borrow_mut().command = Some(storage_record(
        started,
        CommandStatus::Finished {
            text: "accepted",
            accepted: true,
        },
    ));
    rig.harness.run();
    assert!(!rig.has(views::radio::NOT_SAVED_TEXT));
    assert!(rig.harness.query_by_label("Save to flash").is_none());
}

// The mapping editor (docs/link-contract.md section 12, "Mapping").

impl Rig {
    /// Choose `channel` in the drop-down of the `function`th function (0 Roll, 1 Pitch, 2 Throttle, 3 Yaw).
    fn pick(&mut self, function: usize, channel: u8) {
        self.harness
            .get_all_by_role(Role::ComboBox)
            .nth(function)
            .expect("drop-down")
            .click();
        self.harness.run();
        self.harness
            .get_all_by_role_and_label(Role::Button, &channel.to_string())
            .next()
            .expect("the list of channels")
            .click();
        self.harness.run();
    }

    fn tick_reversed(&mut self, function: usize) {
        self.harness
            .get_all_by_label("Reversed")
            .nth(function)
            .expect("checkbox")
            .click();
        self.harness.run();
    }

    fn set_dead_zone(&mut self, function: usize, us: f64) {
        self.harness
            .get_all_by_role(Role::SpinButton)
            .nth(function)
            .expect("dead zone field")
            .set_value_accesskit(us);
        self.harness.run();
        self.harness.run();
    }

    /// The channel each drop-down shows, and whether each Reversed box is ticked.
    fn shown(&self) -> (Vec<String>, Vec<bool>) {
        let channels = self
            .harness
            .get_all_by_role(Role::ComboBox)
            .map(|n| n.accesskit_node().label().unwrap_or_default())
            .collect();
        let ticks = self
            .harness
            .get_all_by_label("Reversed")
            .map(|n| n.accesskit_node().toggled() == Some(egui::accesskit::Toggled::True))
            .collect();
        (channels, ticks)
    }
}

#[test]
fn staging_a_channel_sends_nothing_and_apply_writes_that_one_value() {
    let mut rig = Rig::new(Access::Configure);
    rig.pick(0, 2);
    assert!(rig.commands.borrow().is_empty(), "staging sends nothing");
    assert!(rig.has("Staged: RCMAP_ROLL = 2"));
    rig.click("Apply mapping");
    let (writes, others) = rig.sent();
    assert_eq!(writes, [("RCMAP_ROLL".to_string(), 2)]);
    assert!(others.is_empty(), "no command 245: {others:?}");
    assert!(rig.has(views::radio::NOT_SAVED_TEXT));
}

#[test]
fn reversed_and_dead_zone_go_to_the_channel_the_function_is_mapped_to() {
    let mut rig = Rig::new(Access::Configure);
    // The default map is roll 1, pitch 2, throttle 3, yaw 4; the list order is roll, pitch, throttle, yaw.
    rig.tick_reversed(1);
    rig.set_dead_zone(3, 20.0);
    rig.click("Apply mapping");
    let (writes, others) = rig.sent();
    assert_eq!(
        writes,
        [("RC2_REVERSED".to_string(), 1), ("RC4_DZ".to_string(), 20)]
    );
    assert!(others.is_empty());
}

#[test]
fn reversed_of_a_restaged_function_is_the_new_channels() {
    let mut rig = Rig::new(Access::Configure);
    rig.pick(0, 5);
    rig.tick_reversed(0);
    rig.click("Apply mapping");
    let (writes, _) = rig.sent();
    assert_eq!(
        writes,
        [
            ("RC5_REVERSED".to_string(), 1),
            ("RCMAP_ROLL".to_string(), 5)
        ]
    );
}

#[test]
fn a_duplicate_map_shows_a_warning_and_apply_still_works() {
    let mut rig = Rig::new(Access::Configure);
    assert!(!rig.has("Warning"));
    rig.pick(0, 4);
    assert!(rig.has("Warning: Roll, Yaw are all mapped to channel 4"));
    assert!(rig.enabled("Apply mapping"));
    rig.click("Apply mapping");
    let (writes, _) = rig.sent();
    assert_eq!(writes, [("RCMAP_ROLL".to_string(), 4)]);
}

#[test]
fn revert_sends_nothing_and_the_controls_show_the_vehicles_values_again() {
    let mut rig = Rig::new(Access::Configure);
    let before = rig.shown();
    assert!(!rig.enabled("Revert"));
    rig.pick(0, 5);
    rig.tick_reversed(0);
    rig.set_dead_zone(1, 30.0);
    assert_ne!(rig.shown(), before);
    assert!(!rig.enabled("Calibrate radio"));
    assert!(rig.has(views::radio::STAGED_TEXT));
    rig.click("Revert");
    assert!(rig.commands.borrow().is_empty());
    assert_eq!(rig.shown(), before);
    assert!(!rig.has("Staged:"));
    assert!(rig.enabled("Calibrate radio"));
    assert!(!rig.enabled("Apply mapping"));
}

#[test]
fn an_operate_session_has_the_mapping_disabled_with_the_reason_and_writes_nothing() {
    let mut rig = Rig::new(Access::Operate);
    assert!(rig.has("Configure needs a serial or TCP link"));
    for combo in rig.harness.get_all_by_role(Role::ComboBox) {
        assert!(combo.accesskit_node().is_disabled());
    }
    for tick in rig.harness.get_all_by_label("Reversed") {
        assert!(tick.accesskit_node().is_disabled());
    }
    assert!(!rig.enabled("Apply mapping"));
    rig.harness
        .get_all_by_role(Role::ComboBox)
        .next()
        .unwrap()
        .click();
    rig.harness.run();
    rig.click("Apply mapping");
    assert!(rig.commands.borrow().is_empty());
}

#[test]
fn the_mapping_is_disabled_during_a_calibration_and_without_parameters() {
    let mut rig = Rig::new(Access::Configure);
    rig.click("Calibrate radio");
    assert!(rig.has(views::radio::CALIBRATING_TEXT));
    for combo in rig.harness.get_all_by_role(Role::ComboBox) {
        assert!(combo.accesskit_node().is_disabled());
    }
    rig.click("Cancel");
    assert!(!rig.has(views::radio::CALIBRATING_TEXT));
    rig.state.borrow_mut().params.fetch = FetchStatus::Fetching {
        received: 1,
        expected: 44,
    };
    rig.harness.run();
    assert!(rig.has("the parameter list is not complete"));
    assert!(
        rig.harness
            .query_all_by_role(Role::ComboBox)
            .next()
            .is_none()
    );
    rig.state.borrow_mut().params.fetch = FetchStatus::Complete;
    rig.state.borrow_mut().params.params.remove("RC1_MIN");
    rig.harness.run();
    assert!(rig.has(NO_RC));
    assert!(
        rig.harness
            .query_all_by_role(Role::ComboBox)
            .next()
            .is_none()
    );
}

#[test]
fn after_a_mapping_apply_save_to_flash_sends_one_command_245() {
    let mut rig = Rig::new(Access::Configure);
    rig.pick(0, 2);
    rig.click("Apply mapping");
    assert!(rig.has(views::radio::NOT_SAVED_TEXT));
    rig.click("Save to flash");
    let (writes, others) = rig.sent();
    assert_eq!(writes.len(), 1);
    assert_eq!(others.len(), 1, "{others:?}");
    assert!(others[0].contains(&format!("{:?}", MavCmd::MAV_CMD_PREFLIGHT_STORAGE)));
    assert!(others[0].contains("[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"));
}
