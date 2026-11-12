//! Log view: the vehicle's text messages and the state of the recording.

use std::path::Path;
use std::time::Duration;

use eframe::egui::{self, Color32};
use gcs_mavlink::Command;
use gcs_mavlink::services::texts::Severity;
use gcs_mavlink::tlog::LogState;

use super::{View, ViewCtx};
use crate::{logfile, status};

/// What the user chose in this view.
struct Choices {
    /// Show this severity and the worse ones; `Debug` shows everything.
    min_severity: Severity,
    /// Entries up to this session time were hidden with Clear.
    cleared_until: Option<Duration>,
}

impl Default for Choices {
    fn default() -> Self {
        Choices {
            min_severity: Severity::Debug,
            cleared_until: None,
        }
    }
}

#[derive(Default)]
pub struct Log {
    choices: Choices,
}

impl View for Log {
    fn name(&self) -> &'static str {
        "Log"
    }

    fn session_changed(&mut self) {
        self.choices.cleared_until = None;
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        recording_part(ui, cx);
        ui.separator();
        texts_part(ui, cx, &mut self.choices);
    }
}

fn recording_part(ui: &mut egui::Ui, cx: &mut ViewCtx) {
    let connected = cx.now.is_some();
    let replaying = logfile::replaying(ui.ctx());
    let log: &LogState = &cx.state.log;
    ui.horizontal_wrapped(|ui| match &log.recording {
        Some(label) => {
            ui.label(recording_text(label, log)).on_hover_text(label);
            if ui
                .add_enabled(connected, egui::Button::new("Stop"))
                .clicked()
            {
                cx.commands.push(Command::StopLog);
            }
        }
        None => {
            ui.label("Not recording");
            let start = ui.add_enabled(connected && !replaying, egui::Button::new("Start"));
            if replaying {
                start.on_disabled_hover_text(REPLAY_NOTE);
            } else if start.clicked() {
                logfile::request_new_log(ui.ctx());
            }
        }
    });
    if let Some(error) = &log.error {
        ui.colored_label(status::RED, error);
    }
    if let Some(error) = logfile::error(ui.ctx()) {
        ui.colored_label(status::RED, error);
    }
}

const REPLAY_NOTE: &str = "Nothing is recorded while a log file is replayed";

/// The log's file name, not its whole path, so the figures stay in sight.
fn recording_text(label: &str, log: &LogState) -> String {
    let name = Path::new(label)
        .file_name()
        .map_or(label, |n| n.to_str().unwrap_or(label));
    format!(
        "Recording to {name}: {} frames, {:.1} kB",
        log.frames,
        log.bytes as f64 / 1000.0
    )
}

fn severity_choice(severity: Severity) -> String {
    match severity {
        Severity::Debug => "Everything".to_string(),
        other => format!("{} and worse", other.name()),
    }
}

fn texts_part(ui: &mut egui::Ui, cx: &ViewCtx, choices: &mut Choices) {
    let entries = &cx.state.texts.entries;
    // A new session starts its clock again: an old marker would hide its first messages.
    match (choices.cleared_until, entries.back()) {
        (Some(_), None) => choices.cleared_until = None,
        (Some(cleared), Some(last)) if last.time < cleared => choices.cleared_until = None,
        _ => {}
    }

    ui.horizontal(|ui| {
        let label = ui.label("Minimum severity");
        egui::ComboBox::from_id_salt("log_min_severity")
            .selected_text(severity_choice(choices.min_severity))
            .show_ui(ui, |ui| {
                for severity in [
                    Severity::Debug,
                    Severity::Info,
                    Severity::Notice,
                    Severity::Warning,
                    Severity::Error,
                    Severity::Critical,
                    Severity::Alert,
                    Severity::Emergency,
                ] {
                    ui.selectable_value(
                        &mut choices.min_severity,
                        severity,
                        severity_choice(severity),
                    );
                }
            })
            .response
            .labelled_by(label.id);
        if ui.button("Clear").clicked() {
            choices.cleared_until = entries.back().map(|e| e.time);
        }
    });

    let shown: Vec<_> = entries
        .iter()
        .filter(|e| e.severity <= choices.min_severity)
        .filter(|e| choices.cleared_until.is_none_or(|c| e.time > c))
        .collect();
    if shown.is_empty() {
        let text = if entries.is_empty() {
            "No messages from the vehicle yet"
        } else {
            "No messages to show"
        };
        ui.label(text);
        return;
    }
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .stick_to_bottom(true)
        .show(ui, |ui| {
            for entry in shown {
                let line = format!(
                    "{:>8.1}  {:<9}  {}",
                    entry.time.as_secs_f64(),
                    entry.severity.name(),
                    entry.text
                );
                let mut text = egui::RichText::new(line).monospace();
                if let Some(colour) = colour(entry.severity) {
                    text = text.color(colour);
                }
                ui.label(text);
            }
        });
}

fn colour(severity: Severity) -> Option<Color32> {
    if severity <= Severity::Error {
        Some(status::RED)
    } else if severity == Severity::Warning {
        Some(status::YELLOW)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn log(frames: u64, bytes: u64) -> LogState {
        LogState {
            frames,
            bytes,
            ..LogState::default()
        }
    }

    #[test]
    fn the_recording_line_names_the_file_not_the_path() {
        let long = "/home/someone/a/very/deep/directory/of/flight/logs/2026-11-12_01-23-45Z.tlog";
        assert_eq!(
            recording_text(long, &log(120, 45_600)),
            "Recording to 2026-11-12_01-23-45Z.tlog: 120 frames, 45.6 kB"
        );
        assert_eq!(
            recording_text("a.tlog", &log(1, 0)),
            "Recording to a.tlog: 1 frames, 0.0 kB"
        );
        assert_eq!(
            recording_text("logs/", &log(1, 0)),
            "Recording to logs: 1 frames, 0.0 kB"
        );
    }
}
