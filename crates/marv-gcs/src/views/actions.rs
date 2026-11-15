//! What the views with command buttons share: a button that is shown disabled with its reason, the
//! confirmation step of the buttons that are hard to undo, and the status line of the last command.

use std::time::Duration;

use eframe::egui;
use gcs_mavlink::VehicleState;

use crate::status;

/// A confirmation that is not clicked within this long is forgotten.
pub const CONFIRM_FOR: Duration = Duration::from_secs(5);

/// A button disabled with `reason` as its hover text when there is one; true when it was clicked.
pub fn button(ui: &mut egui::Ui, label: &str, reason: Option<&str>) -> bool {
    let response = ui.add_enabled(reason.is_none(), egui::Button::new(label));
    if let Some(reason) = reason {
        response.clone().on_disabled_hover_text(reason);
    }
    response.clicked()
}

/// Which button waits for its second click, and since when (session time).
#[derive(Default)]
pub struct Confirm {
    armed: Option<(String, Duration)>,
}

impl Confirm {
    /// The first click arms the button, which is then replaced by "Confirm: ..." for
    /// [`CONFIRM_FOR`]; true on the second click.
    pub fn button(
        &mut self,
        ui: &mut egui::Ui,
        now: Option<Duration>,
        label: &str,
        reason: Option<&str>,
    ) -> bool {
        let now = now.unwrap_or_default();
        if self
            .armed
            .as_ref()
            .is_some_and(|(_, at)| reason.is_some() || now.saturating_sub(*at) > CONFIRM_FOR)
        {
            self.armed = None;
        }
        if self.armed.as_ref().is_some_and(|(armed, _)| armed == label) {
            ui.ctx().request_repaint_after(Duration::from_millis(500));
            if button(ui, &format!("Confirm: {label}"), None) {
                self.armed = None;
                return true;
            }
        } else if button(ui, label, reason) {
            self.armed = Some((label.to_string(), now));
        }
        false
    }
}

/// The last command in words, when there was one.
pub fn status_line(ui: &mut egui::Ui, state: &VehicleState) {
    if let Some(record) = &state.command {
        let (text, colour) = status::command_line(record);
        ui.colored_label(colour, text);
    }
}
