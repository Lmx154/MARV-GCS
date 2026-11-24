//! The Map tab's flight commands (docs/link-contract.md, section 10: Amendments A and C): the buttons Start, Pause,
//! Abort, Land and Return, the vehicle's mode in words, the error of a refused command and the take-over panel.
//! What to show is decided by the pure functions below, tested without a display; `Controls::ui` only draws it.
//! There is no button here that arms, disarms or stops the motors.

use std::time::Duration;

use eframe::egui::{self, RichText};
use gcs_mavlink::{
    Command, FlightCommand, FlightMode, FlightResult, FlightState, LinkState, LinkStatus,
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

/// The error under the buttons: the last result when it is not an acceptance and was not dismissed. `key`
/// tells one result from the next, equal ones included (the session time the command started).
pub fn error_line(
    last: Option<FlightResult>,
    key: Option<Duration>,
    dismissed: Option<(FlightResult, Option<Duration>)>,
) -> Option<String> {
    let last = last?;
    if dismissed == Some((last, key)) {
        return None;
    }
    last.error()
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

/// What the buttons wait for: the Start confirmation and the dismissed error.
#[derive(Default)]
pub struct Controls {
    pending_start: bool,
    dismissed: Option<(FlightResult, Option<Duration>)>,
}

impl Controls {
    pub fn session_changed(&mut self) {
        *self = Self::default();
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let state = cx.state;
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
                    cx.commands.push(Command::Flight(FlightCommand::Start));
                }
                if actions::button(ui, "Cancel", None) {
                    self.pending_start = false;
                }
            });
        }
        let key = state.command.as_ref().map(|r| r.started);
        if let Some(error) = error_line(state.flight.last, key, self.dismissed) {
            ui.horizontal_wrapped(|ui| {
                ui.colored_label(theme::ERROR, RichText::new(error).strong());
                if actions::button(ui, "Dismiss", None)
                    && let Some(last) = state.flight.last
                {
                    self.dismissed = Some((last, key));
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
