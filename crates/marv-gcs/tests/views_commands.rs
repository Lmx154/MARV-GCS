//! The command buttons of the Parameters, Link and Devices views, drawn by egui_kittest.

use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::dialect::MavCmd;
use gcs_mavlink::services::messages::{Field, FieldValue, MessageEntry, MessageKey};
use gcs_mavlink::{
    Access, CancelStatus, Command, CommandRecord, CommandStatus, RefusedCommand, VehicleState,
};
use marv_gcs::views::{self, ViewCtx};

type Commands = Rc<RefCell<Vec<Command>>>;

fn harness(view: &str, state: VehicleState, access: Access) -> (Harness<'static>, Commands) {
    let commands: Commands = Rc::default();
    let sink = commands.clone();
    let mut view = views::all()
        .into_iter()
        .find(|v| v.name() == view)
        .expect("the view is registered");
    let harness = Harness::builder()
        .with_size(eframe::egui::vec2(2600.0, 700.0))
        .build_ui(move |ui| {
            let mut out = Vec::new();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut out,
                now: Some(Duration::from_secs(30)),
                access,
            };
            view.ui(ui, &mut cx);
            sink.borrow_mut().extend(out);
        });
    (harness, commands)
}

fn record(command: u32, params: [f32; 7], status: CommandStatus) -> CommandRecord {
    CommandRecord {
        command,
        name: None,
        params,
        status,
        sends: 1,
        started: Duration::ZERO,
        ended: None,
        refused: None,
        cancel: None,
    }
}

fn with(record: CommandRecord) -> VehicleState {
    VehicleState {
        command: Some(record),
        ..VehicleState::default()
    }
}

fn p(index: usize, value: f32) -> [f32; 7] {
    let mut a = [0.0; 7];
    a[index] = value;
    a
}

fn device(instance: u8, name: &str) -> (MessageKey, MessageEntry) {
    (
        MessageKey {
            system_id: 1,
            component_id: 1,
            message_id: 42_421,
            instance,
        },
        MessageEntry {
            name: "MARV_DEVICE_STATUS".into(),
            label: format!("MARV_DEVICE_STATUS[{instance}]"),
            count: 1,
            rate_hz: 1.0,
            last_seen: Duration::from_secs(29),
            fields: vec![Field {
                name: "device_id".into(),
                value: FieldValue::Text(format!("MARV_DEVICE_{name}")),
            }],
        },
    )
}

fn devices_state() -> VehicleState {
    let mut state = VehicleState::default();
    for (i, name) in [
        (1, "IMU1"),
        (2, "IMU2"),
        (3, "BARO"),
        (4, "MAG"),
        (5, "GNSS"),
        (6, "BATT"),
        (7, "RC"),
    ] {
        let (key, entry) = device(i, name);
        state.messages.entries.insert(key, entry);
    }
    state
}

const PARAMS_BUTTONS: [&str; 4] = [
    "Save to flash",
    "Reload from flash",
    "Reset all to defaults",
    "Reset calibration",
];

#[test]
fn on_operate_every_parameter_button_is_disabled_and_sends_nothing() {
    let (mut h, commands) = harness("Parameters", VehicleState::default(), Access::Operate);
    h.run();
    for label in PARAMS_BUTTONS {
        assert!(
            h.get_by_label(label).accesskit_node().is_disabled(),
            "{label}"
        );
        h.get_by_label(label).click();
    }
    h.run();
    assert!(commands.borrow().is_empty());
    h.get_all_by_label_contains("Configure needs a serial or TCP link")
        .next()
        .expect("the reason is shown");
}

#[test]
fn on_configure_the_parameter_buttons_are_enabled_and_send_their_command() {
    let (mut h, commands) = harness("Parameters", VehicleState::default(), Access::Configure);
    h.run();
    for label in PARAMS_BUTTONS {
        assert!(
            !h.get_by_label(label).accesskit_node().is_disabled(),
            "{label}"
        );
    }
    h.get_by_label("Save to flash").click();
    h.run();
    h.get_by_label("Reload from flash").click();
    h.run();
    let sent = commands.borrow();
    let params: Vec<(MavCmd, f32)> = sent
        .iter()
        .map(|c| match c {
            Command::SendCommand { command, params } => (*command, params[0]),
            other => panic!("unexpected {other:?}"),
        })
        .collect();
    assert_eq!(
        params,
        vec![
            (MavCmd::MAV_CMD_PREFLIGHT_STORAGE, 1.0),
            (MavCmd::MAV_CMD_PREFLIGHT_STORAGE, 0.0)
        ]
    );
}

#[test]
fn the_two_resets_need_a_second_click_on_confirm() {
    let (mut h, commands) = harness("Parameters", VehicleState::default(), Access::Configure);
    h.run();
    for (label, action) in [("Reset all to defaults", 2.0), ("Reset calibration", 3.0)] {
        h.get_by_label(label).click();
        h.run();
        assert!(commands.borrow().is_empty(), "one click sends nothing");
        assert!(h.query_by_label(label).is_none(), "the button is replaced");
        h.get_by_label(&format!("Confirm: {label}")).click();
        h.run();
        let sent = commands.borrow();
        assert_eq!(sent.len(), 1, "{label}");
        let Command::SendCommand { command, params } = &sent[0] else {
            panic!("not a SendCommand");
        };
        assert_eq!(*command, MavCmd::MAV_CMD_PREFLIGHT_STORAGE);
        assert_eq!(params[0], action);
        drop(sent);
        commands.borrow_mut().clear();
        assert!(h.query_by_label(label).is_some(), "the button is back");
    }
}

#[test]
fn while_a_command_runs_every_button_is_disabled_with_the_reason() {
    let state = with(record(
        245,
        p(0, 1.0),
        CommandStatus::InProgress { percent: Some(40) },
    ));
    let (mut h, commands) = harness("Parameters", state.clone(), Access::Configure);
    h.run();
    for label in PARAMS_BUTTONS {
        assert!(
            h.get_by_label(label).accesskit_node().is_disabled(),
            "{label}"
        );
    }
    h.get_by_label("a command is running");
    h.get_by_label("Save to flash: 40 %");
    h.get_by_label("Save to flash").click();
    h.run();
    assert!(commands.borrow().is_empty());

    let (mut h, _) = harness("Link", state, Access::Configure);
    h.run();
    assert!(
        h.get_by_label("Reboot vehicle")
            .accesskit_node()
            .is_disabled()
    );
}

#[test]
fn reboot_is_in_the_link_view_with_a_confirmation() {
    let (mut h, commands) = harness("Link", VehicleState::default(), Access::Operate);
    h.run();
    assert!(
        h.get_by_label("Reboot vehicle")
            .accesskit_node()
            .is_disabled()
    );
    drop(h);

    let (mut h, commands2) = harness("Link", VehicleState::default(), Access::Configure);
    h.run();
    h.get_by_label("Reboot vehicle").click();
    h.run();
    assert!(commands2.borrow().is_empty());
    h.get_by_label("Confirm: Reboot vehicle").click();
    h.run();
    let sent = commands2.borrow();
    assert_eq!(sent.len(), 1);
    let Command::SendCommand { command, params } = &sent[0] else {
        panic!("not a SendCommand");
    };
    assert_eq!(*command, MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
    assert_eq!(params[0], 1.0);
    assert!(commands.borrow().is_empty());
}

#[test]
fn the_status_line_texts() {
    let line = |record: CommandRecord| {
        let (mut h, _) = harness("Link", with(record), Access::Configure);
        h.run();
        h
    };
    line(record(
        241,
        p(0, 1.0),
        CommandStatus::InProgress { percent: Some(40) },
    ))
    .get_by_label("Gyro calibration: 40 %");
    line(record(
        245,
        p(0, 1.0),
        CommandStatus::Finished {
            text: "accepted",
            accepted: true,
        },
    ))
    .get_by_label("Save to flash: accepted");
    let mut reboot = record(246, p(0, 1.0), CommandStatus::NoAnswer);
    reboot.sends = 3;
    line(reboot).get_by_label("Reboot: no answer after 3 sends");
    line(record(
        42_430,
        [3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        CommandStatus::Finished {
            text: "denied",
            accepted: false,
        },
    ))
    .get_by_label("Restart BARO: denied");
    line(record(241, p(4, 4.0), CommandStatus::Waiting))
        .get_by_label("Accelerometer calibration: waiting for the vehicle");
    line(record(246, p(0, 1.0), CommandStatus::NoResult))
        .get_by_label("Reboot: no result (the vehicle stopped reporting)");
    let mut busy = record(245, p(0, 0.0), CommandStatus::Waiting);
    busy.refused = Some(RefusedCommand {
        command: 246,
        count: 2,
    });
    line(busy).get_by_label_contains(
        "Reload from flash: waiting for the vehicle — 2 other command(s) refused",
    );
}

#[test]
fn the_compass_status_texts_and_the_hint_only_while_it_waits_or_runs() {
    const HINT: &str = "rotate the vehicle through all orientations";
    let shown = |record: CommandRecord| {
        let (mut h, _) = harness(
            "Devices",
            {
                let mut s = devices_state();
                s.command = Some(record);
                s
            },
            Access::Configure,
        );
        h.run();
        h.query_by_label(HINT).is_some()
    };
    let compass = |status| record(241, p(1, 1.0), status);
    assert!(shown(compass(CommandStatus::Waiting)));
    assert!(shown(compass(CommandStatus::InProgress {
        percent: Some(40)
    })));
    assert!(!shown(compass(CommandStatus::Finished {
        text: "accepted",
        accepted: true
    })));
    assert!(!shown(compass(CommandStatus::Finished {
        text: "cancelled",
        accepted: false
    })));
    assert!(!shown(compass(CommandStatus::NoAnswer)));
    assert!(!shown(record(
        241,
        p(0, 1.0),
        CommandStatus::InProgress { percent: Some(40) }
    )));
    assert!(!shown(record(
        241,
        [0.0; 7],
        CommandStatus::InProgress { percent: Some(40) }
    )));
    let (mut h, _) = harness("Devices", devices_state(), Access::Configure);
    h.run();
    assert!(h.query_by_label(HINT).is_none());
    drop(h);

    let line = |record: CommandRecord| {
        let (mut h, _) = harness("Link", with(record), Access::Configure);
        h.run();
        h
    };
    line(compass(CommandStatus::InProgress { percent: Some(40) }))
        .get_by_label("Compass calibration: 40 %");
    line(compass(CommandStatus::Finished {
        text: "accepted",
        accepted: true,
    }))
    .get_by_label("Compass calibration: accepted");
}

#[test]
fn the_status_line_of_a_cancel() {
    let line = |record: CommandRecord| {
        let (mut h, _) = harness("Link", with(record), Access::Configure);
        h.run();
        h
    };
    let ended = |text: &'static str, accepted: bool| {
        let mut r = record(241, p(0, 1.0), CommandStatus::Finished { text, accepted });
        r.cancel = Some(CancelStatus::Answered("accepted"));
        r
    };
    line(ended("accepted (finished before the cancel)", true))
        .get_by_label("Gyro calibration: accepted (finished before the cancel)");
    line(ended("cancel accepted, no final result", false))
        .get_by_label("Gyro calibration: cancel accepted, no final result");
    line(ended("nothing to cancel", false)).get_by_label("Gyro calibration: nothing to cancel");
    let mut running = record(
        241,
        p(0, 1.0),
        CommandStatus::InProgress { percent: Some(40) },
    );
    running.cancel = Some(CancelStatus::Waiting);
    line(running).get_by_label("Gyro calibration: 40 % (cancel sent)");
}

#[test]
fn the_device_buttons_appear_only_for_the_devices_that_have_the_action() {
    let (mut h, commands) = harness("Devices", devices_state(), Access::Configure);
    h.run();
    // Restart: IMU1, IMU2, BARO, MAG, GNSS, BATT (6). Self-test: IMU1, IMU2, MAG (3). Not RC.
    assert_eq!(h.get_all_by_label("Restart").count(), 6);
    assert_eq!(h.get_all_by_label("Self-test").count(), 3);
    // The first row is IMU1: its Restart is first, and the third row (BARO) has only a Restart.
    h.get_all_by_label("Restart").nth(2).expect("BARO").click();
    h.run();
    h.get_all_by_label("Self-test").nth(2).expect("MAG").click();
    h.run();
    let sent = commands.borrow();
    let got: Vec<(MavCmd, f32, f32)> = sent
        .iter()
        .map(|c| match c {
            Command::SendCommand { command, params } => (*command, params[0], params[1]),
            other => panic!("unexpected {other:?}"),
        })
        .collect();
    assert_eq!(
        got,
        vec![
            (MavCmd::MAV_CMD_MARV_DEVICE, 3.0, 1.0),
            (MavCmd::MAV_CMD_MARV_DEVICE, 4.0, 2.0)
        ]
    );
}

#[test]
fn on_operate_the_device_and_calibration_buttons_are_disabled() {
    let (mut h, commands) = harness("Devices", devices_state(), Access::Operate);
    h.run();
    for label in [
        "Calibrate gyro",
        "Calibrate accelerometer (level)",
        "Calibrate compass",
        "Cancel calibration",
    ] {
        assert!(
            h.get_by_label(label).accesskit_node().is_disabled(),
            "{label}"
        );
        h.get_by_label(label).click();
    }
    for node in h
        .get_all_by_label("Restart")
        .chain(h.get_all_by_label("Self-test"))
    {
        assert!(node.accesskit_node().is_disabled());
        node.click();
    }
    h.run();
    assert!(commands.borrow().is_empty());
}

#[test]
fn calibration_buttons_send_their_parameters_and_cancel_is_enabled_only_during_a_calibration() {
    let (mut h, commands) = harness("Devices", devices_state(), Access::Configure);
    h.run();
    assert!(
        !h.get_by_label("Calibrate gyro")
            .accesskit_node()
            .is_disabled()
    );
    assert!(
        h.get_by_label("Cancel calibration")
            .accesskit_node()
            .is_disabled(),
        "nothing to cancel"
    );
    h.get_by_label("Calibrate gyro").click();
    h.run();
    h.get_by_label("Calibrate accelerometer (level)").click();
    h.run();
    h.get_by_label("Calibrate compass").click();
    h.run();
    {
        let sent = commands.borrow();
        let params: Vec<[f32; 7]> = sent
            .iter()
            .map(|c| match c {
                Command::SendCommand { command, params } => {
                    assert_eq!(*command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
                    *params
                }
                other => panic!("unexpected {other:?}"),
            })
            .collect();
        assert_eq!(params, vec![p(0, 1.0), p(4, 4.0), p(1, 1.0)]);
    }
    drop(h);

    // A calibration runs: every command button is disabled, except Cancel.
    let mut state = devices_state();
    state.command = Some(record(
        241,
        p(0, 1.0),
        CommandStatus::InProgress { percent: Some(10) },
    ));
    let (mut h, commands) = harness("Devices", state.clone(), Access::Configure);
    h.run();
    for label in [
        "Calibrate gyro",
        "Calibrate accelerometer (level)",
        "Calibrate compass",
    ] {
        assert!(
            h.get_by_label(label).accesskit_node().is_disabled(),
            "{label}"
        );
    }
    for node in h
        .get_all_by_label("Restart")
        .chain(h.get_all_by_label("Self-test"))
    {
        assert!(node.accesskit_node().is_disabled());
    }
    assert!(
        !h.get_by_label("Cancel calibration")
            .accesskit_node()
            .is_disabled()
    );
    h.get_by_label("Cancel calibration").click();
    h.run();
    assert_eq!(
        commands
            .borrow()
            .iter()
            .map(|c| match c {
                Command::SendCommand { params, .. } => *params,
                other => panic!("unexpected {other:?}"),
            })
            .collect::<Vec<_>>(),
        vec![[0.0; 7]]
    );
    drop(h);

    // Not during another command, and not once the cancel was sent.
    for other in [record(245, p(0, 1.0), CommandStatus::Waiting), {
        let mut r = record(241, p(0, 1.0), CommandStatus::Waiting);
        r.cancel = Some(CancelStatus::Waiting);
        r
    }] {
        let mut state = devices_state();
        state.command = Some(other);
        let (mut h, _) = harness("Devices", state, Access::Configure);
        h.run();
        assert!(
            h.get_by_label("Cancel calibration")
                .accesskit_node()
                .is_disabled()
        );
    }
}
