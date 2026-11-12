//! Parameters view: every parameter the vehicle reports, searched, grouped and edited, with the
//! edits written in one go and the table saved to / loaded from a parameter file.
//!
//! The view owns its [`State`], plain data handled by the functions of that type; a new session
//! drops it.

use std::collections::{BTreeMap, BTreeSet};

use eframe::egui;
use egui_extras::{Column, TableBuilder};
use gcs_mavlink::Command;
use gcs_mavlink::param_file;
use gcs_mavlink::services::params::{FetchStatus, ParamTable, ParamValue, WriteStatus};

use super::{View, ViewCtx};
use crate::status;

const OTHER_GROUP: &str = "(other)";
const DEFAULT_PATH: &str = "marv.param";

#[derive(Default)]
pub struct Params {
    state: State,
}

impl View for Params {
    fn name(&self) -> &'static str {
        "Parameters"
    }

    fn session_changed(&mut self) {
        self.state = State::default();
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        draw(ui, cx, &mut self.state);
    }
}

/// What the typed text of an edit means for one parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Meaning {
    /// A number of the parameter's type that is the value it already has.
    Same,
    /// A number of the parameter's type that differs from its value.
    Valid(ParamValue),
    /// Not a number, or one that does not fit the type.
    Invalid,
}

#[derive(Debug, Clone, PartialEq)]
enum RowState {
    Clear,
    Edited,
    Invalid,
    Writing,
    Refused(ParamValue),
    Unanswered,
}

struct State {
    /// The typed text, by parameter name.
    edits: BTreeMap<String, String>,
    /// Edits sent by `Write changes` and not yet confirmed, with the value that was sent. While the
    /// table has no write entry for the name, the same value is not sent again.
    sent: BTreeMap<String, ParamValue>,
    search: String,
    /// `None` is `All`.
    group: Option<String>,
    path: String,
    /// What the last save or load did, as one line.
    outcome: String,
    /// The problem lines of the last load.
    problems: Vec<String>,
}

impl Default for State {
    fn default() -> Self {
        Self {
            edits: BTreeMap::new(),
            sent: BTreeMap::new(),
            search: String::new(),
            group: None,
            path: DEFAULT_PATH.to_string(),
            outcome: String::new(),
            problems: Vec::new(),
        }
    }
}

fn group_of(name: &str) -> &str {
    match name.split_once('_') {
        Some((prefix, _)) if !prefix.is_empty() => prefix,
        _ => OTHER_GROUP,
    }
}

fn groups(table: &ParamTable) -> BTreeSet<String> {
    table
        .params
        .keys()
        .map(|n| group_of(n).to_string())
        .collect()
}

fn matches(name: &str, search: &str, group: Option<&str>) -> bool {
    let needle = search.trim().to_lowercase();
    (needle.is_empty() || name.to_lowercase().contains(&needle))
        && group.is_none_or(|g| group_of(name) == g)
}

fn meaning(table: &ParamTable, name: &str, text: &str) -> Option<Meaning> {
    let param = table.params.get(name)?;
    let current = param.value(table.encoding);
    let parsed = text
        .trim()
        .parse::<f64>()
        .ok()
        .and_then(|n| param.kind.value_from(n));
    Some(match parsed {
        Some(v) if v == current => Meaning::Same,
        Some(v) => Meaning::Valid(v),
        None => Meaning::Invalid,
    })
}

fn written_value(status: &WriteStatus) -> ParamValue {
    match status {
        WriteStatus::Pending { value }
        | WriteStatus::Refused { value, .. }
        | WriteStatus::Unanswered { value } => *value,
    }
}

impl State {
    /// Keep `text` as the edit of `name`, unless it is exactly the value shown.
    fn set(&mut self, table: &ParamTable, name: &str, text: String) {
        let shown = table.value(name).map(|v| v.to_string());
        if shown.as_deref() == Some(text.trim()) {
            self.edits.remove(name);
        } else {
            self.edits.insert(name.to_string(), text);
        }
    }

    /// Clear the sent edits the table now confirms.
    fn prune(&mut self, table: &ParamTable) {
        let confirmed: Vec<String> = self
            .sent
            .keys()
            .filter(|name| match self.edits.get(*name) {
                None => true,
                Some(text) => meaning(table, name, text) == Some(Meaning::Same),
            })
            .cloned()
            .collect();
        for name in confirmed {
            self.sent.remove(&name);
            self.edits.remove(&name);
        }
    }

    fn discard(&mut self) {
        self.edits.clear();
        self.sent.clear();
    }

    /// The valid edits that `Write changes` would send, in name order. An edit whose same value is
    /// already on its way is not sent twice: it is pending in the table, or it was sent a moment ago
    /// and the table has no write entry for it yet.
    fn sendable(&self, table: &ParamTable) -> Vec<(String, ParamValue)> {
        self.edits
            .iter()
            .filter_map(|(name, text)| match meaning(table, name, text)? {
                Meaning::Valid(value) => Some((name.clone(), value)),
                _ => None,
            })
            .filter(|(name, value)| match table.writes.get(name) {
                Some(WriteStatus::Pending { value: sent }) => sent != value,
                Some(_) => true,
                None => self.sent.get(name) != Some(value),
            })
            .collect()
    }

    /// Queue one `SetParam` per sendable edit and remember that they were sent.
    fn write(&mut self, table: &ParamTable, commands: &mut Vec<Command>) {
        for (name, value) in self.sendable(table) {
            self.sent.insert(name.clone(), value);
            commands.push(Command::SetParam { name, value });
        }
    }

    fn row_state(&self, table: &ParamTable, name: &str) -> RowState {
        let edit = self
            .edits
            .get(name)
            .and_then(|text| meaning(table, name, text));
        match (table.writes.get(name), edit) {
            (Some(write), None | Some(Meaning::Same)) => write_state(write),
            (Some(write), Some(Meaning::Valid(v))) if v == written_value(write) => {
                write_state(write)
            }
            (_, Some(Meaning::Invalid)) => RowState::Invalid,
            (_, Some(Meaning::Valid(_))) => RowState::Edited,
            (None, _) => RowState::Clear,
        }
    }

    /// Turn the file's changes into edits as if typed; the problems are kept for display.
    fn load(&mut self, table: &ParamTable) {
        self.problems.clear();
        let text = match std::fs::read_to_string(&self.path) {
            Ok(text) => text,
            Err(e) => {
                self.outcome = format!("Load failed: {}: {e}", self.path);
                return;
            }
        };
        let wanted = match param_file::parse(&text) {
            Ok(wanted) => wanted,
            Err(e) => {
                self.outcome = format!("Load failed: {}: {e}", self.path);
                return;
            }
        };
        let (changes, problems) = param_file::changes(table, &wanted);
        for (name, value) in &changes {
            self.set(table, name, value.to_string());
        }
        self.outcome = format!(
            "Loaded {}: {} changes, {} problems",
            self.path,
            changes.len(),
            problems.len()
        );
        self.problems = problems;
    }

    fn save(&mut self, table: &ParamTable) {
        self.problems.clear();
        self.outcome = match std::fs::write(&self.path, param_file::to_text(table)) {
            Ok(()) => format!("Saved {} parameters to {}", table.params.len(), self.path),
            Err(e) => format!("Save failed: {}: {e}", self.path),
        };
    }
}

fn write_state(write: &WriteStatus) -> RowState {
    match write {
        WriteStatus::Pending { .. } => RowState::Writing,
        WriteStatus::Refused { in_effect, .. } => RowState::Refused(*in_effect),
        WriteStatus::Unanswered { .. } => RowState::Unanswered,
    }
}

fn state_text(state: &RowState) -> String {
    match state {
        RowState::Clear => String::new(),
        RowState::Edited => "edited".to_string(),
        RowState::Invalid => "invalid".to_string(),
        RowState::Writing => "writing…".to_string(),
        RowState::Refused(in_effect) => format!("refused: vehicle uses {in_effect}"),
        RowState::Unanswered => "no answer".to_string(),
    }
}

fn state_colour(state: &RowState) -> Option<egui::Color32> {
    match state {
        RowState::Clear => None,
        RowState::Edited | RowState::Writing => Some(status::YELLOW),
        RowState::Invalid | RowState::Refused(_) | RowState::Unanswered => Some(status::RED),
    }
}

fn draw(ui: &mut egui::Ui, cx: &mut ViewCtx, st: &mut State) {
    let table = &cx.state.params;
    let connected = cx.now.is_some();
    st.prune(table);

    ui.horizontal(|ui| {
        if connected {
            fetch_status(ui, table);
        } else {
            let (text, colour) = status::not_connected();
            ui.colored_label(colour, text);
        }
        ui.add_enabled_ui(connected, |ui| {
            if ui.button("Refresh").clicked() {
                cx.commands.push(Command::RefreshParams);
            }
            let sendable = st.sendable(table).len();
            let write = ui.add_enabled(
                sendable > 0,
                egui::Button::new(format!("Write changes ({sendable})")),
            );
            if write.clicked() {
                st.write(table, cx.commands);
            }
            if ui
                .add_enabled(!st.edits.is_empty(), egui::Button::new("Discard changes"))
                .clicked()
            {
                st.discard();
            }
        });
    });

    ui.add_enabled_ui(connected, |ui| {
        let all_groups = groups(table);
        if st.group.as_ref().is_some_and(|g| !all_groups.contains(g)) {
            st.group = None;
        }
        ui.horizontal(|ui| {
            ui.add(
                egui::TextEdit::singleline(&mut st.search)
                    .hint_text("Search")
                    .desired_width(200.0),
            )
            .accessible_name("Search");
            let selected = st.group.clone().unwrap_or_else(|| "All".to_string());
            egui::ComboBox::from_label("Group")
                .selected_text(selected)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut st.group, None, "All");
                    for g in &all_groups {
                        ui.selectable_value(&mut st.group, Some(g.clone()), g);
                    }
                });
        });

        ui.horizontal(|ui| {
            ui.label("File");
            ui.add(egui::TextEdit::singleline(&mut st.path).desired_width(240.0))
                .accessible_name("parameter file path");
            if ui
                .add_enabled(!table.params.is_empty(), egui::Button::new("Save"))
                .clicked()
            {
                st.save(table);
            }
            if ui.button("Load").clicked() {
                st.load(table);
            }
        });
    });
    if !st.outcome.is_empty() {
        ui.label(&st.outcome);
    }
    for problem in &st.problems {
        ui.colored_label(status::YELLOW, problem);
    }
    ui.separator();

    let names: Vec<&String> = table
        .params
        .keys()
        .filter(|n| matches(n, &st.search, st.group.as_deref()))
        .collect();
    let row_height = ui.spacing().interact_size.y;
    ui.add_enabled_ui(connected, |ui| {
        TableBuilder::new(ui)
            .striped(true)
            .resizable(true)
            .column(Column::initial(220.0).at_least(60.0).clip(true))
            .column(Column::initial(160.0).at_least(60.0).clip(true))
            .column(Column::exact(50.0))
            .column(Column::remainder().clip(true))
            .header(row_height, |mut header| {
                for title in ["Name", "Value", "Type", "State"] {
                    header.col(|ui| {
                        ui.strong(title);
                    });
                }
            })
            .body(|body| {
                body.rows(row_height, names.len(), |mut row| {
                    let name = names[row.index()].as_str();
                    let Some(param) = table.params.get(name) else {
                        return;
                    };
                    let row_state = st.row_state(table, name);
                    let colour = state_colour(&row_state);
                    row.col(|ui| {
                        match colour {
                            Some(c) => ui.colored_label(c, name),
                            None => ui.label(name),
                        };
                    });
                    row.col(|ui| {
                        let mut text = st
                            .edits
                            .get(name)
                            .cloned()
                            .unwrap_or_else(|| param.value(table.encoding).to_string());
                        let mut field =
                            egui::TextEdit::singleline(&mut text).desired_width(f32::INFINITY);
                        if let Some(c) = colour {
                            field = field.text_color(c);
                        }
                        let response = ui.add(field).accessible_name(format!("value of {name}"));
                        if response.changed() {
                            st.set(table, name, text);
                        }
                    });
                    row.col(|ui| {
                        ui.label(param.kind.name());
                    });
                    row.col(|ui| {
                        let text = state_text(&row_state);
                        match colour {
                            Some(c) => ui.colored_label(c, text),
                            None => ui.label(text),
                        };
                    });
                });
            });
    });
}

fn fetch_status(ui: &mut egui::Ui, table: &ParamTable) {
    match &table.fetch {
        FetchStatus::Idle => {
            ui.label("Not loaded");
        }
        FetchStatus::Fetching { received, expected } => {
            let fraction = if *expected == 0 {
                0.0
            } else {
                f32::from(*received) / f32::from(*expected)
            };
            ui.add(
                egui::ProgressBar::new(fraction)
                    .desired_width(200.0)
                    .text(format!("{received} / {expected}")),
            );
        }
        FetchStatus::Complete => {
            ui.label(format!("{} parameters", table.params.len()));
        }
        FetchStatus::Failed(reason) => {
            ui.colored_label(
                status::RED,
                format!("{reason} ({} parameters arrived)", table.params.len()),
            );
        }
    }
}
