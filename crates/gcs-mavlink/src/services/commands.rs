//! Command service: sends one MAVLink command at a time and follows its COMMAND_ACKs
//! (docs/link-contract.md, section 10). Sans-IO: the time comes from `cx.now`.

use std::time::Duration;

use super::{Ctx, Service};
use crate::dialect::{COMMAND_ACK_DATA, COMMAND_LONG_DATA, MavCmd, MavResult};
use crate::session::SessionConfig;
use crate::state::{CancelStatus, CommandRecord, CommandStatus, RefusedCommand};
use crate::{Body, Command, Frame, LinkStatus, Message};

/// Without any ACK, a command is sent again after this long.
pub const RESEND: Duration = Duration::from_millis(1500);
/// Sends in all, the first included.
pub const SENDS: u8 = 3;
/// After IN_PROGRESS, this long without a further ACK ends the command as "no result".
pub const SILENCE: Duration = Duration::from_secs(20);

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

/// The send timers of one command (a command or its cancel).
#[derive(Debug)]
struct Sending {
    sends: u8,
    last_send: Duration,
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
    /// The cancel of a calibration, while it waits for its answer.
    cancel: Option<Sending>,
}

#[derive(Debug)]
pub struct Commands {
    system_id: u8,
    component_id: u8,
    track: Option<Track>,
}

impl Commands {
    pub fn new(config: &SessionConfig) -> Self {
        Self {
            system_id: config.system_id,
            component_id: config.component_id,
            track: None,
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
            track.cancel = Some(Sending {
                sends: 1,
                last_send: cx.now,
            });
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
        let in_progress = ack.result == MavResult::MAV_RESULT_IN_PROGRESS;
        if track.cancel.is_some() && !in_progress && ack.result != MavResult::MAV_RESULT_CANCELLED {
            track.cancel = None;
            if let Some(record) = cx.state.command.as_mut() {
                record.cancel = Some(CancelStatus::Answered(result_text(ack.result)));
            }
            cx.changed = true;
            return;
        }
        let Some(record) = cx.state.command.as_mut() else {
            return;
        };
        if in_progress {
            track.last_ack = Some(cx.now);
            record.status = CommandStatus::InProgress {
                percent: (ack.progress != PROGRESS_UNKNOWN).then_some(ack.progress.min(100)),
            };
        } else {
            record.status = CommandStatus::Finished {
                text: result_text(ack.result),
                accepted: ack.result == MavResult::MAV_RESULT_ACCEPTED,
            };
            self.end(cx);
        }
        cx.changed = true;
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
        if cx.now.saturating_sub(cancel.last_send) < RESEND {
            return;
        }
        if cancel.sends < SENDS {
            cx.out.push(Self::long(
                track.target,
                track.command,
                &[0.0; 7],
                cancel.sends,
            ));
            cancel.sends += 1;
            cancel.last_send = cx.now;
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
        if let Body::Known(Message::COMMAND_ACK(ack)) = &frame.body {
            self.on_ack(ack, cx);
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
                if track.sending.sends < SENDS {
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
