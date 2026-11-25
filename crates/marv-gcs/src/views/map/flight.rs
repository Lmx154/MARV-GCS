//! The Map tab's flight commands (docs/link-contract.md, section 10: Amendments A and C): the buttons Start, Pause,
//! Abort, Land and Return, the vehicle's mode in words, the error of a refused command and the take-over panel.
//! What to show is decided by the pure functions below, tested without a display; `Controls::ui` only draws it.
//! There is no button here that arms, disarms or stops the motors.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use eframe::egui::{self, RichText};
use gcs_mavlink::{
    Command, FlightCommand, FlightMode, FlightOutcome, FlightState, LinkState, LinkStatus,
    VehicleState,
};

use super::ViewCtx;
use super::mission::CONFIRM_BEFORE_SENDING;
use crate::theme;
use crate::views::actions;

/// The take-over wording of the contract (section 10, Amendment N19), verbatim.
pub const TAKEOVER: &str = "MISSION SWITCH ON: the laptop flies. TO TAKE CONTROL: mode switch POSITION, ARM ON, throttle in the middle (advice), then MISSION SWITCH OFF: you fly in Position at once. Not in Position: control stays locked until it is. TO LAND NOW: ARM OFF with the mission switch ON. TO STOP THE MOTORS: ARM OFF and the mission switch OFF (they stop at once, in the air too). Do NOT change the transmitter model in the air: the vehicle ignores it for the rest of the flight.";
/// Added in TAKEOVER.
pub const LOCKED: &str = "Control locked: mode switch to POSITION, ARM ON";
/// The conditional line shown with it. The station does not learn whether home was set (open point 14), so it
/// names both halves instead of choosing one.
pub const LINK_LOST: &str = "If the link is lost: the vehicle holds and, after 10 s without the link, returns to take-off (if home was set) / lands where it is (if no home)";
/// Added to the link-loss line.
pub const LINK_RETURNS: &str =
    "When the link returns with the mission switch ON the vehicle carries on.";
pub const WAITING: &str = "a command is waiting for the vehicle";
pub const START_QUESTION: &str = "Start the stored mission?";

/// Why the buttons are disabled; `None` when the link is up and no command waits for the vehicle.
pub fn gate(link: &LinkState, has_session: bool, outstanding: bool) -> Option<&'static str> {
    if !has_session || link.status != LinkStatus::Connected || link.vehicle.is_none() {
        return Some(crate::status::NOT_CONNECTED);
    }
    outstanding.then_some(WAITING)
}

/// The mode line: the vehicle's effective mode in plain words.
pub fn mode_line(flight: &FlightState, link_up: bool) -> String {
    match (link_up, flight.mode_code) {
        (true, Some(code)) => format!("Mode: {}", FlightMode::from_code(code).text()),
        _ => "Mode: unknown (no vehicle)".to_string(),
    }
}

/// How the result line is coloured.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tone {
    Waiting,
    Accepted,
    Failed,
}

/// The result line under the buttons: what the vehicle answered to the last flight command and when, or that it has
/// not answered yet. `clock` turns a session time into the laptop's local time of day. `dismissed` is the `sent` time
/// of the result the user dismissed (or that a new press cleared); that result is not shown.
pub fn result_line(
    flight: &FlightState,
    dismissed: Option<Option<Duration>>,
    clock: impl Fn(Duration) -> String,
) -> Option<(Tone, String)> {
    if dismissed == Some(flight.sent) {
        return None;
    }
    let sent = flight.sent.map(&clock).unwrap_or_default();
    if let Some(command) = flight.pending {
        return Some((
            Tone::Waiting,
            format!("{}: sent, waiting for the vehicle", command.label()),
        ));
    }
    let last = flight.last?;
    let button = last.command.label();
    let at = flight.answered.map(&clock).unwrap_or_default();
    Some(match last.outcome {
        FlightOutcome::Accepted => (
            Tone::Accepted,
            format!("{button} ACCEPTED by the vehicle at {at}"),
        ),
        FlightOutcome::NoAnswer => (
            Tone::Failed,
            format!("{button}: NO ANSWER from the vehicle (sent {sent})"),
        ),
        FlightOutcome::NotSent => (
            Tone::Failed,
            format!("{button} NOT SENT at {at}: the vehicle is not connected"),
        ),
        FlightOutcome::Denied { .. } | FlightOutcome::Other(_) => (
            Tone::Failed,
            format!(
                "{button} REFUSED at {at}: {}",
                last.reason().unwrap_or_default()
            ),
        ),
    })
}

/// The laptop's local time of day, HH:MM:SS, at `unix_secs` seconds after the Unix epoch.
pub fn local_clock(unix_secs: u64) -> String {
    let t = libc::time_t::try_from(unix_secs).unwrap_or(0);
    // SAFETY: `tm` is plain data that `localtime_r` fills in; it is read only when `localtime_r` returned non-null.
    let tm = unsafe {
        let mut tm: libc::tm = std::mem::zeroed();
        if libc::localtime_r(&t, &mut tm).is_null() {
            let day = unix_secs % 86_400;
            return format!("{:02}:{:02}:{:02} UTC", day / 3600, day / 60 % 60, day % 60);
        }
        tm
    };
    format!("{:02}:{:02}:{:02}", tm.tm_hour, tm.tm_min, tm.tm_sec)
}

/// The take-over panel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Panel {
    Hidden,
    /// The vehicle's latest HEARTBEAT reports a hands-off mode.
    Standing(FlightMode),
    /// The heartbeat stopped while the last reported mode was hands-off; the age of that last report.
    FullWidth(FlightMode, Option<Duration>),
}

pub fn panel(state: &VehicleState, now: Option<Duration>) -> Panel {
    let Some(mode) = state.flight.mode_code.map(FlightMode::from_code) else {
        return Panel::Hidden;
    };
    if !(mode.hands_off() || mode == FlightMode::Takeover) {
        return Panel::Hidden;
    }
    match state.link.status {
        LinkStatus::Connected => Panel::Standing(mode),
        LinkStatus::Lost => Panel::FullWidth(
            mode,
            now.zip(state.link.last_heartbeat)
                .map(|(now, at)| now.saturating_sub(at)),
        ),
        LinkStatus::Connecting | LinkStatus::Closed(_) => Panel::Hidden,
    }
}

/// The last reported mode behind a shown panel.
pub fn panel_mode(panel: Panel) -> Option<FlightMode> {
    match panel {
        Panel::Hidden => None,
        Panel::Standing(mode) | Panel::FullWidth(mode, _) => Some(mode),
    }
}

/// The line above the instruction: what the station last heard, nothing it cannot know.
pub fn panel_header(panel: Panel) -> Option<String> {
    match panel {
        Panel::Hidden => None,
        Panel::Standing(mode) => Some(format!("The vehicle reports: {}", mode.text())),
        Panel::FullWidth(mode, age) => Some(match age {
            Some(age) => format!(
                "The link to the vehicle stopped. Last reported {} s ago: {}",
                age.as_secs(),
                mode.text()
            ),
            None => format!(
                "The link to the vehicle stopped. Last reported: {}",
                mode.text()
            ),
        }),
    }
}

/// What the buttons wait for: the Start confirmation and the dismissed (or cleared) result, by its send time.
#[derive(Default)]
pub struct Controls {
    pending_start: bool,
    dismissed: Option<Option<Duration>>,
    /// Unix seconds at session time 0, fixed at the first frame of the session so the shown times do not wobble.
    epoch: Option<u64>,
}

impl Controls {
    pub fn session_changed(&mut self) {
        *self = Self::default();
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let state = cx.state;
        if self.epoch.is_none()
            && let Some(now) = cx.now
        {
            let unix = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_or(0, |d| d.as_secs());
            self.epoch = Some(unix.saturating_sub(now.as_secs()));
        }
        let outstanding = state.command.as_ref().is_some_and(|r| r.outstanding());
        let gate = gate(&state.link, cx.now.is_some(), outstanding);
        if gate.is_some() {
            self.pending_start = false;
        }
        ui.horizontal_wrapped(|ui| {
            for flight in FlightCommand::ALL {
                if actions::button(ui, flight.label(), gate) {
                    if flight == FlightCommand::Start && CONFIRM_BEFORE_SENDING {
                        self.pending_start = true;
                    } else {
                        self.pending_start = false;
                        self.dismissed = Some(state.flight.sent);
                        cx.commands.push(Command::Flight(flight));
                    }
                }
            }
            ui.label(theme::value(mode_line(
                &state.flight,
                gate != Some(crate::status::NOT_CONNECTED),
            )));
        });
        if self.pending_start {
            ui.horizontal_wrapped(|ui| {
                ui.colored_label(theme::WARN, START_QUESTION);
                if actions::button(ui, "Confirm", gate) {
                    self.pending_start = false;
                    self.dismissed = Some(state.flight.sent);
                    cx.commands.push(Command::Flight(FlightCommand::Start));
                }
                if actions::button(ui, "Cancel", None) {
                    self.pending_start = false;
                }
            });
        }
        let epoch = self.epoch.unwrap_or(0);
        let clock = |t: Duration| local_clock(epoch + t.as_secs());
        if let Some((tone, text)) = result_line(&state.flight, self.dismissed, clock) {
            ui.horizontal_wrapped(|ui| {
                match tone {
                    Tone::Waiting => ui.label(theme::value(text)),
                    Tone::Accepted => ui.colored_label(theme::OK, RichText::new(text).strong()),
                    Tone::Failed => ui.colored_label(theme::ERROR, RichText::new(text).strong()),
                };
                if tone != Tone::Waiting && actions::button(ui, "Dismiss", None) {
                    self.dismissed = Some(state.flight.sent);
                }
            });
        }
        let panel = panel(state, cx.now);
        if let Some(header) = panel_header(panel) {
            let colour = if matches!(panel, Panel::FullWidth(..)) {
                theme::ERROR
            } else {
                theme::WARN
            };
            egui::Frame::group(ui.style())
                .stroke(egui::Stroke::new(2.0, colour))
                .show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.colored_label(colour, RichText::new(header).strong());
                    ui.label(TAKEOVER);
                    if panel_mode(panel) == Some(FlightMode::Takeover) {
                        ui.label(RichText::new(LOCKED).strong());
                    }
                    ui.label(LINK_LOST);
                    ui.label(LINK_RETURNS);
                });
        }
    }
}

#[cfg(test)]
#[path = "flight_tests.rs"]
mod tests;
