use super::*;
use crate::{VehicleId, VehicleState};

const STORAGE: MavCmd = MavCmd::MAV_CMD_PREFLIGHT_STORAGE;
const REBOOT: MavCmd = MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;

struct Rig {
    service: Commands,
    state: VehicleState,
    /// Every message sent, with the call's time in ms.
    log: Vec<(u64, Message)>,
}

impl Rig {
    fn new() -> Self {
        let mut state = VehicleState::default();
        state.link.status = LinkStatus::Connected;
        state.link.vehicle = Some(VehicleId {
            system_id: 1,
            component_id: 1,
            mav_type: "MAV_TYPE_GENERIC".into(),
            autopilot: "MAV_AUTOPILOT_GENERIC".into(),
        });
        Self {
            service: Commands::new(&SessionConfig::default()),
            state,
            log: Vec::new(),
        }
    }

    fn call(&mut self, now_ms: u64, f: impl FnOnce(&mut Commands, &mut Ctx)) {
        let mut out = Vec::new();
        let mut cx = Ctx {
            now: Duration::from_millis(now_ms),
            wall_time_us: 0,
            state: &mut self.state,
            out: &mut out,
            changed: false,
        };
        f(&mut self.service, &mut cx);
        for message in out {
            self.log.push((now_ms, message));
        }
    }

    fn send(&mut self, command: MavCmd, params: [f32; 7], now_ms: u64) {
        let command = Command::SendCommand { command, params };
        self.call(now_ms, |s, cx| s.on_command(&command, cx));
    }

    fn tick(&mut self, now_ms: u64) {
        self.call(now_ms, |s, cx| s.on_tick(cx));
    }

    /// Ticks every 50 ms from `from` to `to` inclusive.
    fn tick_range(&mut self, from: u64, to: u64) {
        let mut t = from;
        while t <= to {
            self.tick(t);
            t += 50;
        }
    }

    fn ack_for(&mut self, ack: COMMAND_ACK_DATA, system: u8, now_ms: u64) {
        let frame = Frame {
            system_id: system,
            component_id: 1,
            sequence: 0,
            message_id: 0,
            raw: Vec::new(),
            body: Body::Known(Message::COMMAND_ACK(ack)),
        };
        self.call(now_ms, |s, cx| s.on_frame(&frame, cx));
    }

    fn ack(&mut self, command: MavCmd, result: MavResult, progress: u8, now_ms: u64) {
        self.ack_for(ack_data(command, result, progress, 255, 190), 1, now_ms);
    }

    /// (time, command, confirmation, param1) of each COMMAND_LONG sent.
    fn sends(&self) -> Vec<(u64, MavCmd, u8, f32)> {
        self.log
            .iter()
            .filter_map(|(t, m)| match m {
                Message::COMMAND_LONG(c) => Some((*t, c.command, c.confirmation, c.param1)),
                _ => None,
            })
            .collect()
    }

    fn record(&self) -> &CommandRecord {
        self.state.command.as_ref().expect("a command record")
    }

    fn status(&self) -> CommandStatus {
        self.record().status.clone()
    }
}

fn ack_data(
    command: MavCmd,
    result: MavResult,
    progress: u8,
    target_system: u8,
    target_component: u8,
) -> COMMAND_ACK_DATA {
    COMMAND_ACK_DATA {
        command,
        result,
        progress,
        result_param2: 0,
        target_system,
        target_component,
    }
}

fn save() -> [f32; 7] {
    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}

fn gyro() -> [f32; 7] {
    save()
}

const CAL: MavCmd = MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION;

#[test]
fn an_ack_at_once_ends_it_accepted_and_nothing_is_resent() {
    let mut rig = Rig::new();
    rig.send(STORAGE, save(), 100);
    assert_eq!(rig.sends(), vec![(100, STORAGE, 0, 1.0)]);
    assert_eq!(rig.status(), CommandStatus::Waiting);
    rig.ack(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 150);
    assert_eq!(
        rig.status(),
        CommandStatus::Finished {
            text: "accepted",
            accepted: true
        }
    );
    assert_eq!(rig.record().ended, Some(Duration::from_millis(150)));
    assert_eq!(rig.record().started, Duration::from_millis(100));
    assert_eq!(rig.record().sends, 1);
    rig.tick_range(200, 10_000);
    assert_eq!(rig.sends().len(), 1);
}

const MOTOR_TEST: MavCmd = MavCmd::MAV_CMD_DO_MOTOR_TEST;

#[test]
fn an_unanswered_motor_spin_is_sent_once_then_no_answer() {
    let mut rig = Rig::new();
    rig.send(MOTOR_TEST, [2.0, 0.0, 5.0, 2.0, 1.0, 2.0, 0.0], 0);
    rig.tick_range(50, 10_000);
    assert_eq!(rig.sends().len(), 1, "a spin is never re-sent");
    assert_eq!(rig.status(), CommandStatus::NoAnswer);
    assert_eq!(rig.record().ended, Some(Duration::from_millis(1_500)));
}

#[test]
fn an_unanswered_motor_stop_is_sent_three_times_like_any_command() {
    let mut rig = Rig::new();
    rig.send(MOTOR_TEST, [2.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.0], 0);
    rig.tick_range(50, 10_000);
    assert_eq!(rig.sends().len(), 3);
    assert_eq!(rig.status(), CommandStatus::NoAnswer);
    assert_eq!(rig.record().ended, Some(Duration::from_millis(4_500)));
}

#[test]
fn the_command_is_addressed_to_the_vehicle_with_its_parameters_and_the_record_names_it() {
    let mut rig = Rig::new();
    rig.send(
        MavCmd::MAV_CMD_MARV_DEVICE,
        [3.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        0,
    );
    let Message::COMMAND_LONG(c) = &rig.log[0].1 else {
        panic!("not a COMMAND_LONG");
    };
    assert_eq!((c.target_system, c.target_component), (1, 1));
    assert_eq!((c.param1, c.param2, c.param3), (3.0, 2.0, 0.0));
    assert_eq!(c.command, MavCmd::MAV_CMD_MARV_DEVICE);
    let record = rig.record();
    assert_eq!(record.command, 42_430);
    assert_eq!(record.name.as_deref(), Some("MAV_CMD_MARV_DEVICE"));
    assert_eq!(record.params[0], 3.0);
}

#[test]
fn without_an_ack_it_is_sent_at_0_1500_and_3000_ms_with_confirmation_0_1_2_then_no_answer_at_4500()
{
    let mut rig = Rig::new();
    rig.send(STORAGE, save(), 0);
    rig.tick_range(50, 1_450);
    assert_eq!(rig.sends().len(), 1);
    rig.tick_range(1_500, 4_450);
    assert_eq!(
        rig.sends()
            .iter()
            .map(|(t, _, c, _)| (*t, *c))
            .collect::<Vec<_>>(),
        vec![(0, 0), (1_500, 1), (3_000, 2)]
    );
    assert_eq!(rig.status(), CommandStatus::Waiting);
    assert_eq!(rig.record().sends, 3);
    rig.tick(4_500);
    assert_eq!(rig.status(), CommandStatus::NoAnswer);
    assert_eq!(rig.record().ended, Some(Duration::from_millis(4_500)));
    rig.tick_range(4_550, 30_000);
    assert_eq!(rig.sends().len(), 3, "nothing is sent after no answer");
    assert_eq!(rig.status(), CommandStatus::NoAnswer);
}

#[test]
fn an_ack_after_the_second_send_ends_it_and_stops_the_third() {
    let mut rig = Rig::new();
    rig.send(STORAGE, save(), 0);
    rig.tick_range(50, 1_550);
    assert_eq!(rig.sends().len(), 2);
    rig.ack(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 1_600);
    rig.tick_range(1_650, 6_000);
    assert_eq!(rig.sends().len(), 2);
    assert_eq!(rig.record().sends, 2);
}

#[test]
fn in_progress_stops_the_retransmission_and_the_progress_is_followed_to_the_final_ack() {
    let mut rig = Rig::new();
    rig.send(STORAGE, save(), 0);
    rig.ack(STORAGE, MavResult::MAV_RESULT_IN_PROGRESS, 0, 100);
    assert_eq!(rig.status(), CommandStatus::InProgress { percent: Some(0) });
    rig.tick_range(150, 1_950);
    assert_eq!(rig.sends().len(), 1, "no resend after IN_PROGRESS");
    rig.ack(STORAGE, MavResult::MAV_RESULT_IN_PROGRESS, 40, 2_000);
    assert_eq!(
        rig.status(),
        CommandStatus::InProgress { percent: Some(40) }
    );
    rig.ack(STORAGE, MavResult::MAV_RESULT_IN_PROGRESS, 255, 2_500);
    assert_eq!(rig.status(), CommandStatus::InProgress { percent: None });
    rig.ack(STORAGE, MavResult::MAV_RESULT_IN_PROGRESS, 75, 3_000);
    rig.ack(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 3_500);
    assert_eq!(
        rig.status(),
        CommandStatus::Finished {
            text: "accepted",
            accepted: true
        }
    );
    assert_eq!(rig.record().ended, Some(Duration::from_millis(3_500)));
    rig.tick_range(3_550, 40_000);
    assert_eq!(rig.sends().len(), 1);
    assert_eq!(
        rig.status(),
        CommandStatus::Finished {
            text: "accepted",
            accepted: true
        }
    );
}

#[test]
fn in_progress_then_silence_is_no_result_after_20_s_counted_from_the_last_ack() {
    let mut rig = Rig::new();
    rig.send(STORAGE, save(), 0);
    rig.ack(STORAGE, MavResult::MAV_RESULT_IN_PROGRESS, 0, 100);
    rig.ack(STORAGE, MavResult::MAV_RESULT_IN_PROGRESS, 50, 5_000);
    rig.tick_range(5_050, 24_950);
    assert_eq!(
        rig.status(),
        CommandStatus::InProgress { percent: Some(50) }
    );
    rig.tick(25_000);
    assert_eq!(rig.status(), CommandStatus::NoResult);
    assert_eq!(rig.record().ended, Some(Duration::from_millis(25_000)));
    assert_eq!(rig.sends().len(), 1);
    // A late final ACK does not bring it back.
    rig.ack(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 26_000);
    assert_eq!(rig.status(), CommandStatus::NoResult);
}

#[test]
fn an_ack_for_another_command_number_is_ignored() {
    let mut rig = Rig::new();
    rig.send(STORAGE, save(), 0);
    rig.ack(REBOOT, MavResult::MAV_RESULT_ACCEPTED, 0, 100);
    rig.ack(REBOOT, MavResult::MAV_RESULT_IN_PROGRESS, 30, 120);
    assert_eq!(rig.status(), CommandStatus::Waiting);
    rig.tick_range(150, 1_500);
    assert_eq!(rig.sends().len(), 2, "still waiting, so it is resent");
}

#[test]
fn an_ack_addressed_to_another_system_or_component_is_ignored_and_zero_counts() {
    let mut rig = Rig::new();
    rig.send(STORAGE, save(), 0);
    rig.ack_for(
        ack_data(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 7, 190),
        1,
        100,
    );
    rig.ack_for(
        ack_data(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 255, 7),
        1,
        120,
    );
    assert_eq!(rig.status(), CommandStatus::Waiting);
    rig.ack_for(
        ack_data(STORAGE, MavResult::MAV_RESULT_IN_PROGRESS, 10, 0, 0),
        1,
        140,
    );
    assert_eq!(
        rig.status(),
        CommandStatus::InProgress { percent: Some(10) }
    );
}

#[test]
fn an_ack_from_another_system_is_ignored() {
    let mut rig = Rig::new();
    rig.send(STORAGE, save(), 0);
    rig.ack_for(
        ack_data(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 255, 190),
        9,
        100,
    );
    assert_eq!(rig.status(), CommandStatus::Waiting);
}

#[test]
fn a_second_send_while_one_is_outstanding_is_refused_sends_nothing_and_is_counted_in_the_record() {
    let mut rig = Rig::new();
    rig.send(STORAGE, save(), 0);
    rig.send(REBOOT, save(), 100);
    rig.send(REBOOT, save(), 200);
    assert_eq!(rig.sends().len(), 1);
    assert_eq!(rig.record().command, 245, "the outstanding one goes on");
    assert_eq!(
        rig.record().refused,
        Some(RefusedCommand {
            command: 246,
            count: 2
        })
    );
    assert_eq!(rig.status(), CommandStatus::Waiting);
    // It is refused in progress too.
    rig.ack(STORAGE, MavResult::MAV_RESULT_IN_PROGRESS, 10, 300);
    rig.send(REBOOT, save(), 400);
    assert_eq!(rig.sends().len(), 1);
    assert_eq!(rig.record().refused.as_ref().map(|r| r.count), Some(3));
    // Once it has ended a new one is sent and its record is clean.
    rig.ack(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 500);
    rig.send(REBOOT, save(), 600);
    assert_eq!(rig.sends().len(), 2);
    assert_eq!(rig.record().command, 246);
    assert_eq!(rig.record().refused, None);
}

#[test]
fn nothing_is_sent_without_a_connected_vehicle_and_the_record_says_refused() {
    let mut rig = Rig::new();
    rig.state.link.status = LinkStatus::Lost;
    rig.send(STORAGE, save(), 0);
    assert!(rig.sends().is_empty());
    assert!(matches!(rig.status(), CommandStatus::Refused(_)));
    assert!(!rig.record().outstanding());
    rig.state.link.status = LinkStatus::Connected;
    rig.send(STORAGE, save(), 100);
    assert_eq!(rig.sends().len(), 1);
    assert_eq!(rig.status(), CommandStatus::Waiting);

    let mut rig = Rig::new();
    rig.state.link.vehicle = None;
    rig.send(STORAGE, save(), 0);
    assert!(rig.sends().is_empty());
}

#[test]
fn unrelated_commands_are_ignored() {
    let mut rig = Rig::new();
    rig.call(0, |s, cx| s.on_command(&Command::RefreshParams, cx));
    assert!(rig.state.command.is_none());
    assert!(rig.log.is_empty());
}

#[test]
fn every_result_has_its_text_and_only_accepted_is_accepted() {
    let cases = [
        (MavResult::MAV_RESULT_ACCEPTED, "accepted"),
        (
            MavResult::MAV_RESULT_TEMPORARILY_REJECTED,
            "temporarily rejected",
        ),
        (MavResult::MAV_RESULT_DENIED, "denied"),
        (MavResult::MAV_RESULT_UNSUPPORTED, "unsupported"),
        (MavResult::MAV_RESULT_FAILED, "failed"),
        (MavResult::MAV_RESULT_IN_PROGRESS, "in progress"),
        (MavResult::MAV_RESULT_CANCELLED, "cancelled"),
        (
            MavResult::MAV_RESULT_COMMAND_LONG_ONLY,
            "only as COMMAND_LONG",
        ),
        (
            MavResult::MAV_RESULT_COMMAND_INT_ONLY,
            "only as COMMAND_INT",
        ),
        (
            MavResult::MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME,
            "frame not supported",
        ),
        (MavResult::MAV_RESULT_NOT_IN_CONTROL, "not in control"),
    ];
    for (result, text) in cases {
        assert_eq!(result_text(result), text, "{result:?}");
        if result == MavResult::MAV_RESULT_IN_PROGRESS {
            continue;
        }
        let mut rig = Rig::new();
        rig.send(STORAGE, save(), 0);
        rig.ack(STORAGE, result, 0, 10);
        assert_eq!(
            rig.status(),
            CommandStatus::Finished {
                text,
                accepted: result == MavResult::MAV_RESULT_ACCEPTED
            },
            "{result:?}"
        );
    }
}

// ---------------------------------------------------------------- the cancel

fn cancel() -> [f32; 7] {
    [0.0; 7]
}

fn running_calibration() -> Rig {
    let mut rig = Rig::new();
    rig.send(CAL, gyro(), 0);
    rig.ack(CAL, MavResult::MAV_RESULT_IN_PROGRESS, 0, 50);
    rig.ack(CAL, MavResult::MAV_RESULT_IN_PROGRESS, 30, 600);
    rig
}

#[test]
fn a_compass_calibration_is_a_calibration_for_the_cancel_rule() {
    let mut rig = Rig::new();
    rig.send(CAL, [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0);
    rig.ack(CAL, MavResult::MAV_RESULT_IN_PROGRESS, 0, 50);
    rig.ack(CAL, MavResult::MAV_RESULT_IN_PROGRESS, 30, 600);
    rig.send(CAL, cancel(), 1_000);
    assert_eq!(rig.sends().len(), 2, "the cancel is sent");
    assert_eq!(rig.record().cancel, Some(CancelStatus::Waiting));
    rig.ack(CAL, MavResult::MAV_RESULT_ACCEPTED, 0, 1_050);
    rig.ack(CAL, MavResult::MAV_RESULT_CANCELLED, 0, 1_200);
    assert_eq!(
        rig.status(),
        CommandStatus::Finished {
            text: "cancelled",
            accepted: false
        }
    );
}

#[test]
fn a_cancel_is_sent_although_the_calibration_is_outstanding() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    let sends = rig.sends();
    assert_eq!(sends.len(), 2);
    assert_eq!((sends[1].0, sends[1].2, sends[1].3), (1_000, 0, 0.0));
    assert_eq!(rig.record().cancel, Some(CancelStatus::Waiting));
    assert_eq!(rig.record().refused, None);
    assert_eq!(rig.record().command, 241);
    assert_eq!(
        rig.record().sends,
        1,
        "the cancel's sends are not counted in the record"
    );
}

#[test]
fn the_cancels_own_ack_is_not_the_calibrations_final_ack_and_the_cancelled_one_is() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    // The vehicle answers the cancel at once; the calibration is still going.
    rig.ack(CAL, MavResult::MAV_RESULT_ACCEPTED, 0, 1_050);
    assert_eq!(
        rig.status(),
        CommandStatus::InProgress { percent: Some(30) }
    );
    assert_eq!(
        rig.record().cancel,
        Some(CancelStatus::Answered("accepted"))
    );
    assert!(rig.record().outstanding());
    // Progress ACKs still count for the calibration.
    rig.ack(CAL, MavResult::MAV_RESULT_IN_PROGRESS, 45, 1_100);
    assert_eq!(
        rig.status(),
        CommandStatus::InProgress { percent: Some(45) }
    );
    // The final one, CANCELLED, ends it.
    rig.ack(CAL, MavResult::MAV_RESULT_CANCELLED, 0, 1_200);
    assert_eq!(
        rig.status(),
        CommandStatus::Finished {
            text: "cancelled",
            accepted: false
        }
    );
    assert_eq!(rig.record().ended, Some(Duration::from_millis(1_200)));
    rig.tick_range(1_250, 30_000);
    assert_eq!(rig.sends().len(), 2, "no resend of either");
}

#[test]
fn the_cancelled_ack_may_come_before_the_cancels_own_answer() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.ack(CAL, MavResult::MAV_RESULT_CANCELLED, 0, 1_050);
    assert_eq!(
        rig.status(),
        CommandStatus::Finished {
            text: "cancelled",
            accepted: false
        }
    );
    // The cancel was dropped with the calibration: its late answer and its timers do nothing.
    assert_eq!(rig.record().cancel, None);
    rig.ack(CAL, MavResult::MAV_RESULT_ACCEPTED, 0, 1_100);
    rig.tick_range(1_150, 10_000);
    assert_eq!(rig.sends().len(), 2);
    assert_eq!(
        rig.status(),
        CommandStatus::Finished {
            text: "cancelled",
            accepted: false
        }
    );
}

#[test]
fn without_a_waiting_cancel_an_accepted_241_ack_is_the_calibrations_final_ack() {
    let mut rig = running_calibration();
    rig.ack(CAL, MavResult::MAV_RESULT_ACCEPTED, 0, 3_000);
    assert_eq!(
        rig.status(),
        CommandStatus::Finished {
            text: "accepted",
            accepted: true
        }
    );
}

#[test]
fn a_cancel_without_an_answer_is_sent_three_times_then_no_answer_and_the_calibration_goes_on() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.tick_range(1_050, 5_500);
    let cancels: Vec<(u64, u8)> = rig
        .sends()
        .iter()
        .filter(|(_, _, _, p1)| *p1 == 0.0)
        .map(|(t, _, c, _)| (*t, *c))
        .collect();
    assert_eq!(cancels, vec![(1_000, 0), (2_500, 1), (4_000, 2)]);
    assert_eq!(rig.record().cancel, Some(CancelStatus::NoAnswer));
    assert_eq!(
        rig.status(),
        CommandStatus::InProgress { percent: Some(30) }
    );
    assert_eq!(rig.sends().len(), 4);
    // The calibration still ends normally.
    rig.ack(CAL, MavResult::MAV_RESULT_CANCELLED, 0, 6_000);
    assert!(!rig.record().outstanding());
}

#[test]
fn a_second_cancel_while_one_waits_is_refused_and_a_zero_command_alone_is_not_a_cancel() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.send(CAL, cancel(), 1_100);
    assert_eq!(rig.sends().len(), 2);
    assert_eq!(rig.record().refused.as_ref().map(|r| r.count), Some(1));

    // With another command outstanding an all-zero 241 is just a refused command.
    let mut rig = Rig::new();
    rig.send(STORAGE, save(), 0);
    rig.send(CAL, cancel(), 100);
    assert_eq!(rig.sends().len(), 1);
    assert_eq!(rig.record().cancel, None);
    assert_eq!(rig.record().refused.as_ref().map(|r| r.command), Some(241));

    // With nothing outstanding it is an ordinary command.
    let mut rig = Rig::new();
    rig.send(CAL, cancel(), 0);
    assert_eq!(rig.sends().len(), 1);
    assert_eq!(rig.record().cancel, None);
    // And a second all-zero one while it is outstanding is not a cancel of it.
    rig.send(CAL, cancel(), 100);
    assert_eq!(rig.sends().len(), 1);
    assert_eq!(rig.record().refused.as_ref().map(|r| r.count), Some(1));
}

const ACCEPTED: MavResult = MavResult::MAV_RESULT_ACCEPTED;
const DENIED: MavResult = MavResult::MAV_RESULT_DENIED;
const FAILED: MavResult = MavResult::MAV_RESULT_FAILED;
const CANCELLED: MavResult = MavResult::MAV_RESULT_CANCELLED;
const IN_PROGRESS: MavResult = MavResult::MAV_RESULT_IN_PROGRESS;

fn done(text: &'static str, accepted: bool) -> CommandStatus {
    CommandStatus::Finished { text, accepted }
}

/// COMMAND_LONGs of the calibration itself (param1 = 1) that were sent.
fn calibration_sends(rig: &Rig) -> Vec<u64> {
    rig.sends()
        .iter()
        .filter(|(_, _, _, p1)| *p1 == 1.0)
        .map(|(t, _, _, _)| *t)
        .collect()
}

#[test]
fn d2_a_lost_first_progress_ack_does_not_make_the_calibration_be_sent_again_after_a_cancel() {
    let mut rig = Rig::new();
    rig.send(CAL, gyro(), 0);
    // The calibration's first IN_PROGRESS is lost: it is still waiting. The user presses cancel.
    rig.send(CAL, cancel(), 1_000);
    // The vehicle answers the cancel ACCEPTED; its CANCELLED is lost.
    rig.ack(CAL, ACCEPTED, 0, 1_050);
    rig.tick_range(1_100, 10_000);
    assert_eq!(
        calibration_sends(&rig),
        vec![0],
        "the calibration is never sent again"
    );
    assert_eq!(
        rig.sends().len(),
        2,
        "only the calibration once and the cancel once"
    );
    assert_eq!(
        rig.status(),
        done("cancel accepted, no final result", false)
    );
    assert_eq!(rig.record().ended, Some(Duration::from_millis(4_050)));
}

#[test]
fn a_cancel_that_is_never_answered_does_not_bring_the_calibration_back_either() {
    let mut rig = Rig::new();
    rig.send(CAL, gyro(), 0);
    rig.send(CAL, cancel(), 1_000);
    rig.tick_range(1_050, 6_000);
    assert_eq!(calibration_sends(&rig), vec![0]);
    assert_eq!(rig.record().cancel, Some(CancelStatus::NoAnswer));
    // Followed as before: its own 20 s rule, counted from the send of the cancel.
    rig.tick_range(6_050, 21_000);
    assert_eq!(rig.status(), CommandStatus::NoResult);
    assert_eq!(calibration_sends(&rig), vec![0]);
    assert_eq!(
        rig.sends().len(),
        4,
        "the calibration once and the cancel three times"
    );
}

#[test]
fn d1_accepted_then_denied_is_a_calibration_that_finished_before_the_cancel() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    // The calibration ends by itself, then the vehicle has nothing to cancel.
    rig.ack(CAL, ACCEPTED, 0, 1_020);
    assert!(rig.record().outstanding(), "the answer so far is only held");
    rig.ack(CAL, DENIED, 0, 1_050);
    assert_eq!(
        rig.status(),
        done("accepted (finished before the cancel)", true)
    );
    assert_eq!(rig.record().ended, Some(Duration::from_millis(1_050)));
}

#[test]
fn failed_then_denied_is_failed_and_finished_before_the_cancel() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.ack(CAL, FAILED, 0, 1_020);
    rig.ack(CAL, DENIED, 0, 1_050);
    assert_eq!(
        rig.status(),
        done("failed (finished before the cancel)", false)
    );
}

#[test]
fn accepted_then_cancelled_is_cancelled() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.ack(CAL, ACCEPTED, 0, 1_020);
    rig.ack(CAL, CANCELLED, 0, 1_050);
    assert_eq!(rig.status(), done("cancelled", false));
    rig.tick_range(1_100, 10_000);
    assert_eq!(rig.status(), done("cancelled", false));
}

#[test]
fn cancelled_first_is_cancelled_and_a_late_answer_changes_nothing() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.ack(CAL, CANCELLED, 0, 1_020);
    assert_eq!(rig.status(), done("cancelled", false));
    rig.ack(CAL, ACCEPTED, 0, 1_050);
    assert_eq!(rig.status(), done("cancelled", false));
}

#[test]
fn accepted_alone_ends_3_s_later_as_cancel_accepted_without_a_final_result() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.ack(CAL, ACCEPTED, 0, 1_050);
    rig.tick_range(1_100, 4_000);
    assert!(rig.record().outstanding());
    rig.tick(4_050);
    assert_eq!(
        rig.status(),
        done("cancel accepted, no final result", false)
    );
    assert_eq!(rig.record().ended, Some(Duration::from_millis(4_050)));
}

#[test]
fn denied_alone_ends_3_s_later_as_nothing_to_cancel() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.ack(CAL, DENIED, 0, 1_050);
    rig.tick_range(1_100, 4_000);
    assert!(rig.record().outstanding());
    rig.tick(4_050);
    assert_eq!(rig.status(), done("nothing to cancel", false));
}

#[test]
fn another_result_alone_ends_3_s_later_with_its_own_text() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.ack(CAL, FAILED, 0, 1_050);
    rig.tick(4_050);
    assert_eq!(rig.status(), done("failed", false));
}

#[test]
fn a_second_result_other_than_denied_ends_with_its_own_text() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.ack(CAL, DENIED, 0, 1_050);
    rig.ack(CAL, FAILED, 0, 1_100);
    assert_eq!(rig.status(), done("failed", false));
}

#[test]
fn progress_between_the_results_of_a_cancel_stays_progress() {
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.ack(CAL, IN_PROGRESS, 40, 1_010);
    assert_eq!(
        rig.status(),
        CommandStatus::InProgress { percent: Some(40) }
    );
    rig.ack(CAL, ACCEPTED, 0, 1_020);
    rig.ack(CAL, IN_PROGRESS, 60, 1_030);
    assert_eq!(
        rig.status(),
        CommandStatus::InProgress { percent: Some(60) }
    );
    assert_eq!(
        rig.record().cancel,
        Some(CancelStatus::Answered("accepted"))
    );
    rig.ack(CAL, IN_PROGRESS, 80, 1_040);
    rig.ack(CAL, DENIED, 0, 1_050);
    assert_eq!(
        rig.status(),
        done("accepted (finished before the cancel)", true)
    );
    // The hold of 3 s is counted from the first result, progress does not extend it.
    let mut rig = running_calibration();
    rig.send(CAL, cancel(), 1_000);
    rig.ack(CAL, ACCEPTED, 0, 1_020);
    rig.ack(CAL, IN_PROGRESS, 60, 3_000);
    rig.tick(4_020);
    assert_eq!(
        rig.status(),
        done("cancel accepted, no final result", false)
    );
}

#[test]
fn the_calibration_waiting_for_its_first_ack_can_be_cancelled_too() {
    let mut rig = Rig::new();
    rig.send(CAL, gyro(), 0);
    rig.send(CAL, cancel(), 100);
    assert_eq!(rig.sends().len(), 2);
    assert_eq!(rig.record().cancel, Some(CancelStatus::Waiting));
    rig.ack(CAL, MavResult::MAV_RESULT_CANCELLED, 0, 200);
    assert!(!rig.record().outstanding());
}

// ---------------------------------------------------------------- calibrations in use and not saved

fn accel() -> [f32; 7] {
    [0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0]
}

fn compass() -> [f32; 7] {
    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}

/// A calibration with these parameters that the vehicle answers with `result`.
fn calibrate(rig: &mut Rig, params: [f32; 7], result: MavResult, at_ms: u64) {
    rig.send(CAL, params, at_ms);
    rig.ack(CAL, MavResult::MAV_RESULT_IN_PROGRESS, 50, at_ms + 100);
    rig.ack(CAL, result, 0, at_ms + 200);
}

fn unsaved(rig: &Rig) -> (bool, bool, bool) {
    let c = &rig.state.calibration;
    (c.gyro, c.accel, c.compass)
}

fn boot_status(rig: &mut Rig, time_boot_ms: u32, now_ms: u64) {
    let frame = Frame {
        system_id: 1,
        component_id: 1,
        sequence: 0,
        message_id: 0,
        raw: Vec::new(),
        body: Body::Known(Message::MARV_DEVICE_STATUS(
            crate::dialect::MARV_DEVICE_STATUS_DATA {
                time_boot_ms,
                ..Default::default()
            },
        )),
    };
    rig.call(now_ms, |s, cx| s.on_frame(&frame, cx));
}

#[test]
fn only_an_accepted_calibration_is_in_use_and_unsaved_and_several_accumulate() {
    let mut rig = Rig::new();
    calibrate(&mut rig, gyro(), MavResult::MAV_RESULT_FAILED, 0);
    assert_eq!(unsaved(&rig), (false, false, false));
    calibrate(&mut rig, accel(), MavResult::MAV_RESULT_CANCELLED, 1_000);
    assert_eq!(unsaved(&rig), (false, false, false));
    calibrate(&mut rig, gyro(), MavResult::MAV_RESULT_ACCEPTED, 2_000);
    assert_eq!(unsaved(&rig), (true, false, false));
    calibrate(&mut rig, compass(), MavResult::MAV_RESULT_ACCEPTED, 3_000);
    assert_eq!(unsaved(&rig), (true, false, true));
    // A later failure takes nothing back.
    calibrate(&mut rig, accel(), MavResult::MAV_RESULT_FAILED, 4_000);
    assert_eq!(unsaved(&rig), (true, false, true));
    assert!(rig.state.calibration.any());
}

#[test]
fn an_accepted_save_or_reload_from_any_tab_clears_them_and_the_other_storage_actions_do_not() {
    for action in [1.0, 0.0] {
        let mut rig = Rig::new();
        calibrate(&mut rig, gyro(), MavResult::MAV_RESULT_ACCEPTED, 0);
        calibrate(&mut rig, accel(), MavResult::MAV_RESULT_ACCEPTED, 1_000);
        rig.send(STORAGE, [action, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2_000);
        rig.ack(STORAGE, MavResult::MAV_RESULT_DENIED, 0, 2_100);
        assert_eq!(unsaved(&rig), (true, true, false), "a refused save");
        rig.send(STORAGE, [action, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3_000);
        rig.ack(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 3_100);
        assert_eq!(unsaved(&rig), (false, false, false), "storage {action}");
        assert!(!rig.state.calibration.lost_at_reboot);
    }
    let mut rig = Rig::new();
    calibrate(&mut rig, gyro(), MavResult::MAV_RESULT_ACCEPTED, 0);
    rig.send(STORAGE, [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1_000);
    rig.ack(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 1_100);
    assert_eq!(unsaved(&rig), (true, false, false));
}

#[test]
fn a_restart_of_the_vehicle_drops_them_and_leaves_a_note_until_the_next_calibration() {
    let mut rig = Rig::new();
    boot_status(&mut rig, 60_000, 0);
    calibrate(&mut rig, gyro(), MavResult::MAV_RESULT_ACCEPTED, 100);
    // A frame slightly out of order is not a restart.
    boot_status(&mut rig, 59_500, 1_000);
    assert_eq!(unsaved(&rig), (true, false, false));
    boot_status(&mut rig, 2_000, 5_000);
    assert_eq!(unsaved(&rig), (false, false, false));
    assert!(rig.state.calibration.lost_at_reboot);
    calibrate(&mut rig, compass(), MavResult::MAV_RESULT_ACCEPTED, 6_000);
    assert_eq!(unsaved(&rig), (false, false, true));
    assert!(!rig.state.calibration.lost_at_reboot);
}

mod flight {
    use super::*;
    use crate::flight::{FlightCommand, FlightOutcome, FlightResult};

    fn flight(rig: &mut Rig, f: FlightCommand, now_ms: u64) {
        rig.call(now_ms, |s, cx| s.on_command(&Command::Flight(f), cx));
    }

    fn last(rig: &Rig) -> Option<FlightResult> {
        rig.state.flight.last
    }

    #[test]
    fn each_button_sends_its_command_long() {
        for f in FlightCommand::ALL {
            let mut rig = Rig::new();
            flight(&mut rig, f, 0);
            let [(_, Message::COMMAND_LONG(c))] = rig.log.as_slice() else {
                panic!("one COMMAND_LONG for {f:?}");
            };
            let (command, params) = f.wire();
            assert_eq!(c.command, command);
            assert_eq!(
                [
                    c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7
                ],
                params
            );
            assert_eq!(
                (c.target_system, c.target_component, c.confirmation),
                (1, 1, 0)
            );
        }
    }

    #[test]
    fn a_denied_ack_keeps_its_reason_code() {
        let mut rig = Rig::new();
        flight(&mut rig, FlightCommand::Start, 0);
        let mut ack = ack_data(
            MavCmd::MAV_CMD_MISSION_START,
            MavResult::MAV_RESULT_DENIED,
            0,
            255,
            190,
        );
        ack.result_param2 = 6;
        rig.ack_for(ack, 1, 100);
        assert_eq!(
            last(&rig),
            Some(FlightResult {
                command: FlightCommand::Start,
                outcome: FlightOutcome::Denied { code: 6 }
            })
        );
    }

    #[test]
    fn accepted_is_accepted_and_replaces_an_earlier_error() {
        let mut rig = Rig::new();
        flight(&mut rig, FlightCommand::Pause, 0);
        rig.ack(
            MavCmd::MAV_CMD_DO_PAUSE_CONTINUE,
            MavResult::MAV_RESULT_DENIED,
            0,
            50,
        );
        assert!(last(&rig).unwrap().error().is_some());
        flight(&mut rig, FlightCommand::Pause, 100);
        rig.ack(
            MavCmd::MAV_CMD_DO_PAUSE_CONTINUE,
            MavResult::MAV_RESULT_ACCEPTED,
            0,
            150,
        );
        assert_eq!(last(&rig).unwrap().outcome, FlightOutcome::Accepted);
    }

    #[test]
    fn a_new_flight_command_clears_the_old_result_and_times_the_new_one() {
        let mut rig = Rig::new();
        flight(&mut rig, FlightCommand::Start, 1_000);
        let mut ack = ack_data(
            MavCmd::MAV_CMD_MISSION_START,
            MavResult::MAV_RESULT_DENIED,
            0,
            255,
            190,
        );
        ack.result_param2 = 6;
        rig.ack_for(ack, 1, 1_500);
        let f = &rig.state.flight;
        assert_eq!(f.pending, None);
        assert_eq!(f.sent, Some(Duration::from_millis(1_000)));
        assert_eq!(f.answered, Some(Duration::from_millis(1_500)));
        flight(&mut rig, FlightCommand::Start, 5_000);
        let f = &rig.state.flight;
        assert_eq!(f.last, None);
        assert_eq!(f.pending, Some(FlightCommand::Start));
        assert_eq!(f.sent, Some(Duration::from_millis(5_000)));
        assert_eq!(f.answered, None);
        rig.ack(
            MavCmd::MAV_CMD_MISSION_START,
            MavResult::MAV_RESULT_ACCEPTED,
            0,
            5_200,
        );
        let f = &rig.state.flight;
        assert_eq!(f.last.unwrap().outcome, FlightOutcome::Accepted);
        assert_eq!(f.pending, None);
        assert_eq!(f.answered, Some(Duration::from_millis(5_200)));
    }

    #[test]
    fn no_ack_ends_as_no_answer_after_three_sends() {
        let mut rig = Rig::new();
        flight(&mut rig, FlightCommand::Land, 0);
        rig.tick_range(50, 10_000);
        assert_eq!(rig.sends().len(), 3);
        assert_eq!(last(&rig).unwrap().outcome, FlightOutcome::NoAnswer);
    }

    #[test]
    fn a_plain_send_command_does_not_touch_the_flight_result() {
        let mut rig = Rig::new();
        rig.send(STORAGE, save(), 0);
        rig.ack(STORAGE, MavResult::MAV_RESULT_ACCEPTED, 0, 50);
        assert_eq!(last(&rig), None);
    }
}
