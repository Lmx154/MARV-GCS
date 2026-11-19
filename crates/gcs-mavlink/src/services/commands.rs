//! Command service: sends one MAVLink command at a time and follows its COMMAND_ACKs
//! (docs/link-contract.md, section 10). Sans-IO: the time comes from `cx.now`.

use std::time::Duration;

use super::{Ctx, Service};
use crate::dialect::{COMMAND_ACK_DATA, COMMAND_LONG_DATA, MavCmd, MavResult};
use crate::session::SessionConfig;
use crate::state::{
    CancelStatus, CommandRecord, CommandStatus, RefusedCommand, UnsavedCalibration,
};
use crate::{Body, Command, Frame, LinkStatus, Message};

/// Without any ACK, a command is sent again after this long.
pub const RESEND: Duration = Duration::from_millis(1500);
/// Sends in all, the first included.
pub const SENDS: u8 = 3;
/// Sends in all for this command: a motor test spin (209 with param3 above 0) is sent once, because a
/// second copy would replace the running spin on the vehicle and restart its time limit.
fn max_sends(command: MavCmd, params: &[f32; 7]) -> u8 {
    if command == MavCmd::MAV_CMD_DO_MOTOR_TEST && params[2] > 0.0 {
        1
    } else {
        SENDS
    }
}

/// After IN_PROGRESS, this long without a further ACK ends the command as "no result".
pub const SILENCE: Duration = Duration::from_secs(20);

/// With a cancel answered and nothing more heard for this long, the calibration ends with what is known.
pub const CANCEL_HOLD: Duration = Duration::from_secs(3);

/// The calibration command, the only one a cancel belongs to.
const CALIBRATION: MavCmd = MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION;
/// `progress` of an ACK that gives no percentage.
const PROGRESS_UNKNOWN: u8 = 255;

/// The result in words, one text per MAV_RESULT. Lower case: the status line puts it after a colon.
pub fn result_text(result: MavResult) -> &'static str {
    match result {
        MavResult::MAV_RESULT_ACCEPTED => "accepted",
        MavResult::MAV_RESULT_TEMPORARILY_REJECTED => "temporarily rejected",
        MavResult::MAV_RESULT_DENIED => "denied",
        MavResult::MAV_RESULT_UNSUPPORTED => "unsupported",
        MavResult::MAV_RESULT_FAILED => "failed",
        MavResult::MAV_RESULT_IN_PROGRESS => "in progress",
        MavResult::MAV_RESULT_CANCELLED => "cancelled",
        MavResult::MAV_RESULT_COMMAND_LONG_ONLY => "only as COMMAND_LONG",
        MavResult::MAV_RESULT_COMMAND_INT_ONLY => "only as COMMAND_INT",
        MavResult::MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME => "frame not supported",
        MavResult::MAV_RESULT_NOT_IN_CONTROL => "not in control",
    }
}

/// A time on the vehicle's clock that goes back by more than this is a restart, not a reordered frame.
const REBOOT_BACK_MS: u32 = 1000;

/// An accepted final ACK: a calibration is now in use and unsaved; a storage save or reload ends that.
fn note_accepted(command: MavCmd, params: [f32; 7], calibration: &mut UnsavedCalibration) {
    if command == CALIBRATION {
        let (gyro, accel, compass) = (params[0] == 1.0, params[4] == 4.0, params[1] == 1.0);
        if gyro || accel || compass {
            calibration.gyro |= gyro;
            calibration.accel |= accel;
            calibration.compass |= compass;
            calibration.lost_at_reboot = false;
        }
    } else if command == MavCmd::MAV_CMD_PREFLIGHT_STORAGE && (params[0] == 0.0 || params[0] == 1.0)
    {
        *calibration = UnsavedCalibration::default();
    }
}

/// The send timers of one command (a command or its cancel).
#[derive(Debug)]
struct Sending {
    sends: u8,
    last_send: Duration,
}

/// A cancel in play (docs/link-contract.md section 10).
#[derive(Debug)]
struct Cancel {
    sending: Sending,
    /// The first result other than IN_PROGRESS and CANCELLED, held as "the answer so far", and when.
    first: Option<(MavResult, Duration)>,
}

/// What a calibration that finished by itself before the cancel reached the vehicle says.
fn before_cancel_text(result: MavResult) -> &'static str {
    match result {
        MavResult::MAV_RESULT_ACCEPTED => "accepted (finished before the cancel)",
        MavResult::MAV_RESULT_TEMPORARILY_REJECTED => {
            "temporarily rejected (finished before the cancel)"
        }
        MavResult::MAV_RESULT_DENIED => "denied (finished before the cancel)",
        MavResult::MAV_RESULT_UNSUPPORTED => "unsupported (finished before the cancel)",
        MavResult::MAV_RESULT_FAILED => "failed (finished before the cancel)",
        MavResult::MAV_RESULT_IN_PROGRESS => "in progress (finished before the cancel)",
        MavResult::MAV_RESULT_CANCELLED => "cancelled (finished before the cancel)",
        MavResult::MAV_RESULT_COMMAND_LONG_ONLY => {
            "only as COMMAND_LONG (finished before the cancel)"
        }
        MavResult::MAV_RESULT_COMMAND_INT_ONLY => {
            "only as COMMAND_INT (finished before the cancel)"
        }
        MavResult::MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME => {
            "frame not supported (finished before the cancel)"
        }
        MavResult::MAV_RESULT_NOT_IN_CONTROL => "not in control (finished before the cancel)",
    }
}

/// How a calibration ends when only the cancel's answer `first` was heard for [`CANCEL_HOLD`].
fn held_text(first: MavResult) -> &'static str {
    match first {
        MavResult::MAV_RESULT_ACCEPTED => "cancel accepted, no final result",
        MavResult::MAV_RESULT_DENIED => "nothing to cancel",
        other => result_text(other),
    }
}

/// The outstanding command's timers; its state is in `VehicleState.command`.
#[derive(Debug)]
struct Track {
    /// The vehicle's system and component id, as when it was sent.
    target: (u8, u8),
    command: MavCmd,
    params: [f32; 7],
    sending: Sending,
    /// Session time of the last ACK, once there was one: from then on nothing is resent.
    last_ack: Option<Duration>,
    /// The cancel of a calibration, from its send until the calibration ends or the cancel is given up.
    cancel: Option<Cancel>,
}

#[derive(Debug)]
pub struct Commands {
    system_id: u8,
    component_id: u8,
    track: Option<Track>,
    /// The last `time_boot_ms` of the vehicle's device status, to see a restart.
    last_boot_ms: Option<u32>,
}

impl Commands {
    pub fn new(config: &SessionConfig) -> Self {
        Self {
            system_id: config.system_id,
            component_id: config.component_id,
            track: None,
            last_boot_ms: None,
        }
    }

    fn long(target: (u8, u8), command: MavCmd, params: &[f32; 7], confirmation: u8) -> Message {
        Message::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            param5: params[4],
            param6: params[5],
            param7: params[6],
            command,
            target_system: target.0,
            target_component: target.1,
            confirmation,
        })
    }

    fn target(cx: &Ctx) -> Option<(u8, u8)> {
        let vehicle = cx.state.link.vehicle.as_ref()?;
        (cx.state.link.status == LinkStatus::Connected)
            .then_some((vehicle.system_id, vehicle.component_id))
    }

    fn send_command(&mut self, command: MavCmd, params: [f32; 7], cx: &mut Ctx) {
        let outstanding = cx.state.command.as_ref().is_some_and(|r| r.outstanding());
        if outstanding {
            self.send_while_outstanding(command, params, cx);
            return;
        }
        let mut record = CommandRecord {
            command: command as u32,
            name: Some(format!("{command:?}")),
            params,
            status: CommandStatus::Waiting,
            sends: 0,
            started: cx.now,
            ended: Some(cx.now),
            refused: None,
            cancel: None,
        };
        let Some(target) = Self::target(cx) else {
            record.status = CommandStatus::Refused("the vehicle is not connected".to_string());
            cx.state.command = Some(record);
            cx.changed = true;
            return;
        };
        record.sends = 1;
        record.ended = None;
        cx.out.push(Self::long(target, command, &params, 0));
        cx.state.command = Some(record);
        cx.changed = true;
        self.track = Some(Track {
            target,
            command,
            params,
            sending: Sending {
                sends: 1,
                last_send: cx.now,
            },
            last_ack: None,
            cancel: None,
        });
    }

    /// Another command while one is outstanding: refused, except the cancel of a calibration.
    fn send_while_outstanding(&mut self, command: MavCmd, params: [f32; 7], cx: &mut Ctx) {
        let target = Self::target(cx);
        if let (Some(track), Some(target)) = (self.track.as_mut(), target)
            && command == CALIBRATION
            && params.iter().all(|p| *p == 0.0)
            && track.command == CALIBRATION
            && track.params.iter().any(|p| *p != 0.0)
            && track.cancel.is_none()
        {
            cx.out.push(Self::long(target, command, &params, 0));
            track.cancel = Some(Cancel {
                sending: Sending {
                    sends: 1,
                    last_send: cx.now,
                },
                first: None,
            });
            // From now on the calibration is never sent again: its silence rule starts here.
            track.last_ack.get_or_insert(cx.now);
            if let Some(record) = cx.state.command.as_mut() {
                record.cancel = Some(CancelStatus::Waiting);
            }
            cx.changed = true;
            return;
        }
        if let Some(record) = cx.state.command.as_mut() {
            let count = record.refused.as_ref().map_or(0, |r| r.count);
            record.refused = Some(RefusedCommand {
                command: command as u32,
                count: count + 1,
            });
            cx.changed = true;
        }
    }

    fn addressed(&self, ack: &COMMAND_ACK_DATA) -> bool {
        (ack.target_system == 0 || ack.target_system == self.system_id)
            && (ack.target_component == 0 || ack.target_component == self.component_id)
    }

    fn on_ack(&mut self, ack: &COMMAND_ACK_DATA, cx: &mut Ctx) {
        let addressed = self.addressed(ack);
        let Some(track) = self.track.as_mut() else {
            return;
        };
        if ack.command != track.command || !addressed {
            return;
        }
        let result = ack.result;
        let Some(record) = cx.state.command.as_mut() else {
            return;
        };
        cx.changed = true;
        if result == MavResult::MAV_RESULT_IN_PROGRESS {
            track.last_ack = Some(cx.now);
            record.status = CommandStatus::InProgress {
                percent: (ack.progress != PROGRESS_UNKNOWN).then_some(ack.progress.min(100)),
            };
            return;
        }
        let (text, accepted) = match track.cancel.as_mut() {
            None => (
                result_text(result),
                result == MavResult::MAV_RESULT_ACCEPTED,
            ),
            Some(_) if result == MavResult::MAV_RESULT_CANCELLED => ("cancelled", false),
            Some(cancel) => match cancel.first {
                None => {
                    cancel.first = Some((result, cx.now));
                    record.cancel = Some(CancelStatus::Answered(result_text(result)));
                    return;
                }
                Some((first, _)) if result == MavResult::MAV_RESULT_DENIED => (
                    before_cancel_text(first),
                    first == MavResult::MAV_RESULT_ACCEPTED,
                ),
                Some(_) => (
                    result_text(result),
                    result == MavResult::MAV_RESULT_ACCEPTED,
                ),
            },
        };
        record.status = CommandStatus::Finished { text, accepted };
        if accepted {
            note_accepted(track.command, track.params, &mut cx.state.calibration);
        }
        self.end(cx);
    }

    /// The outstanding command ended: record the time, drop its timers and a cancel still waiting.
    fn end(&mut self, cx: &mut Ctx) {
        self.track = None;
        if let Some(record) = cx.state.command.as_mut() {
            record.ended = Some(cx.now);
            if record.cancel == Some(CancelStatus::Waiting) {
                record.cancel = None;
            }
        }
    }

    fn tick_cancel(&mut self, cx: &mut Ctx) {
        let Some(track) = self.track.as_mut() else {
            return;
        };
        let Some(cancel) = track.cancel.as_mut() else {
            return;
        };
        if let Some((first, at)) = cancel.first {
            if cx.now.saturating_sub(at) >= CANCEL_HOLD {
                if let Some(record) = cx.state.command.as_mut() {
                    record.status = CommandStatus::Finished {
                        text: held_text(first),
                        accepted: false,
                    };
                }
                self.end(cx);
                cx.changed = true;
            }
            return;
        }
        let sending = &mut cancel.sending;
        if cx.now.saturating_sub(sending.last_send) < RESEND {
            return;
        }
        if sending.sends < SENDS {
            cx.out.push(Self::long(
                track.target,
                track.command,
                &[0.0; 7],
                sending.sends,
            ));
            sending.sends += 1;
            sending.last_send = cx.now;
        } else {
            track.cancel = None;
            if let Some(record) = cx.state.command.as_mut() {
                record.cancel = Some(CancelStatus::NoAnswer);
            }
            cx.changed = true;
        }
    }
}

impl Service for Commands {
    fn on_frame(&mut self, frame: &Frame, cx: &mut Ctx) {
        let Some((system, _)) = cx
            .state
            .link
            .vehicle
            .as_ref()
            .map(|v| (v.system_id, v.component_id))
        else {
            return;
        };
        if frame.system_id != system {
            return;
        }
        match &frame.body {
            Body::Known(Message::COMMAND_ACK(ack)) => self.on_ack(ack, cx),
            Body::Known(Message::MARV_DEVICE_STATUS(status)) => {
                let back = self
                    .last_boot_ms
                    .is_some_and(|last| status.time_boot_ms.saturating_add(REBOOT_BACK_MS) < last);
                self.last_boot_ms = Some(status.time_boot_ms);
                if back && cx.state.calibration.any() {
                    cx.state.calibration = UnsavedCalibration {
                        lost_at_reboot: true,
                        ..UnsavedCalibration::default()
                    };
                    cx.changed = true;
                }
            }
            _ => {}
        }
    }

    fn on_tick(&mut self, cx: &mut Ctx) {
        self.tick_cancel(cx);
        let Some(track) = self.track.as_mut() else {
            return;
        };
        match track.last_ack {
            Some(last) => {
                if cx.now.saturating_sub(last) >= SILENCE {
                    if let Some(record) = cx.state.command.as_mut() {
                        record.status = CommandStatus::NoResult;
                    }
                    self.end(cx);
                    cx.changed = true;
                }
            }
            None => {
                if cx.now.saturating_sub(track.sending.last_send) < RESEND {
                    return;
                }
                if track.sending.sends < max_sends(track.command, &track.params) {
                    cx.out.push(Self::long(
                        track.target,
                        track.command,
                        &track.params,
                        track.sending.sends,
                    ));
                    track.sending.sends += 1;
                    track.sending.last_send = cx.now;
                    if let Some(record) = cx.state.command.as_mut() {
                        record.sends = track.sending.sends;
                    }
                    cx.changed = true;
                } else {
                    if let Some(record) = cx.state.command.as_mut() {
                        record.status = CommandStatus::NoAnswer;
                    }
                    self.end(cx);
                    cx.changed = true;
                }
            }
        }
    }

    fn on_command(&mut self, command: &Command, cx: &mut Ctx) {
        if let Command::SendCommand { command, params } = command {
            self.send_command(*command, *params, cx);
        }
    }
}

#[cfg(test)]
#[path = "commands_tests.rs"]
mod tests;
