//! Parameters view: every parameter the vehicle reports, searched, shown in a tree by scope and group
//! and edited, with the
//! edits written in one go and the table saved to / loaded from a parameter file.
//!
//! The view owns its [`State`], plain data handled by the functions of that type; a new session
//! drops it.

use std::cmp::Ordering;
use std::collections::BTreeMap;

use eframe::egui;
use egui_extras::{Column, TableBuilder};
use gcs_mavlink::Command;
use gcs_mavlink::dialect::MavCmd;
use gcs_mavlink::param_file;
use gcs_mavlink::services::params::{FetchStatus, ParamTable, ParamValue, WriteStatus};

use super::actions::{self, Confirm};
use super::presets::{self, DebugFlag};
use super::{View, ViewCtx};
use crate::status;

const OTHER_GROUP: &str = "(other)";
const DEFAULT_PATH: &str = "marv.param";

pub struct Params {
    state: State,
    /// The station's debug switch, set on the Devices tab: off, the selectors and governed fields of
    /// the sensor presets are read only here.
    debug: DebugFlag,
}

impl Params {
    pub fn new(debug: DebugFlag) -> Self {
        Self {
            state: State::default(),
            debug,
        }
    }
}

impl View for Params {
    fn name(&self) -> &'static str {
        "Parameters"
    }

    fn session_changed(&mut self) {
        self.state = State::default();
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        draw(ui, cx, &mut self.state, self.debug.get());
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
    path: String,
    /// What the last save or load did, as one line.
    outcome: String,
    /// The problem lines of the last load.
    problems: Vec<String>,
    /// The second click of the two resets.
    confirm: Confirm,
}

impl Default for State {
    fn default() -> Self {
        Self {
            edits: BTreeMap::new(),
            sent: BTreeMap::new(),
            search: String::new(),
            path: DEFAULT_PATH.to_string(),
            outcome: String::new(),
            problems: Vec::new(),
            confirm: Confirm::default(),
        }
    }
}

fn group_of(name: &str) -> &str {
    match name.split_once('_') {
        Some((prefix, _)) if !prefix.is_empty() => prefix,
        _ => OTHER_GROUP,
    }
}

/// The top level of the tree. The vehicle sends no such thing: the scope follows from the group, the
/// prefix of the names (`docs/link-contract.md` section 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Scope {
    FlightController,
    RcController,
    Esc,
}

/// The scopes in the order they are shown.
const SCOPES: [Scope; 3] = [Scope::FlightController, Scope::RcController, Scope::Esc];

impl Scope {
    fn title(self) -> &'static str {
        match self {
            Self::FlightController => "Flight controller",
            Self::RcController => "RC controller",
            Self::Esc => "ESC",
        }
    }
}

/// The only place that knows a group name: `RC` and digits, and `RCMAP`, are the RC controller;
/// `ESC`, `MOT` and `DSHOT` the ESC; every other group the flight controller.
fn scope_of(group: &str) -> Scope {
    let rc_channel = group
        .strip_prefix("RC")
        .is_some_and(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()));
    match group {
        "RCMAP" => Scope::RcController,
        _ if rc_channel => Scope::RcController,
        "ESC" | "MOT" | "DSHOT" => Scope::Esc,
        _ => Scope::FlightController,
    }
}

/// The text up to the first change between digits and other characters, and the rest.
fn chunk(s: &str) -> (&str, &str) {
    let digit = s.as_bytes().first().is_some_and(u8::is_ascii_digit);
    let end = s
        .find(|c: char| c.is_ascii_digit() != digit)
        .unwrap_or(s.len());
    s.split_at(end)
}

/// Order with numbers by value: `RC2` before `RC10`. Equal values written differently (`RC01`,
/// `RC1`) fall back to the plain order.
fn natural_cmp(a: &str, b: &str) -> Ordering {
    let (mut x, mut y) = (a, b);
    while !x.is_empty() && !y.is_empty() {
        let ((cx, rx), (cy, ry)) = (chunk(x), chunk(y));
        let order = if cx.starts_with(|c: char| c.is_ascii_digit())
            && cy.starts_with(|c: char| c.is_ascii_digit())
        {
            let (nx, ny) = (cx.trim_start_matches('0'), cy.trim_start_matches('0'));
            nx.len().cmp(&ny.len()).then_with(|| nx.cmp(ny))
        } else {
            cx.cmp(cy)
        };
        if order != Ordering::Equal {
            return order;
        }
        (x, y) = (rx, ry);
    }
    x.len().cmp(&y.len()).then_with(|| a.cmp(b))
}

/// One group of the tree: its name, all its parameters and those that match the search.
struct Group<'a> {
    name: String,
    total: usize,
    shown: Vec<&'a String>,
}

/// The tree for the table and the search, one entry per scope in [`SCOPES`] order, the groups of a
/// scope in natural order. A group with no parameter that matches is left out while searching.
fn tree<'a>(table: &'a ParamTable, search: &str) -> Vec<(Scope, Vec<Group<'a>>)> {
    let needle = search.trim().to_lowercase();
    let mut by_group: BTreeMap<&str, Group<'a>> = BTreeMap::new();
    for name in table.params.keys() {
        let group = group_of(name);
        let entry = by_group.entry(group).or_insert_with(|| Group {
            name: group.to_string(),
            total: 0,
            shown: Vec::new(),
        });
        entry.total += 1;
        if needle.is_empty() || name.to_lowercase().contains(&needle) {
            entry.shown.push(name);
        }
    }
    let mut groups: Vec<Group<'a>> = by_group.into_values().collect();
    groups.sort_by(|a, b| natural_cmp(&a.name, &b.name));
    SCOPES
        .into_iter()
        .map(|scope| {
            let mine = groups
                .iter()
                .filter(|g| scope_of(&g.name) == scope)
                .map(|g| Group {
                    name: g.name.clone(),
                    total: g.total,
                    shown: g.shown.clone(),
                })
                .collect();
            (scope, mine)
        })
        .collect()
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
    fn sendable(&self, table: &ParamTable, debug: bool) -> Vec<(String, ParamValue)> {
        self.edits
            .iter()
            .filter(|(name, _)| debug || !presets::is_governed(name))
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
    fn write(&mut self, table: &ParamTable, debug: bool, commands: &mut Vec<Command>) {
        for (name, value) in self.sendable(table, debug) {
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

/// The storage actions of the flight controller (`MAV_CMD_PREFLIGHT_STORAGE`), and the status line
/// of the last command. The two resets ask for a second click.
fn storage_buttons(ui: &mut egui::Ui, cx: &mut ViewCtx, confirm: &mut Confirm) {
    let gate = status::command_gate(cx.access, cx.state, cx.now.is_some());
    let storage = |action: f32| Command::SendCommand {
        command: MavCmd::MAV_CMD_PREFLIGHT_STORAGE,
        params: [action, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    ui.horizontal(|ui| {
        if actions::button(ui, "Save to flash", gate) {
            cx.commands.push(storage(1.0));
        }
        if actions::button(ui, "Reload from flash", gate) {
            cx.commands.push(storage(0.0));
        }
        if confirm.button(ui, cx.now, "Reset all to defaults", gate) {
            cx.commands.push(storage(2.0));
        }
        if confirm.button(ui, cx.now, "Reset calibration", gate) {
            cx.commands.push(storage(3.0));
        }
    });
    if let Some(reason) = gate
        && cx.access == gcs_mavlink::Access::Configure
    {
        ui.colored_label(status::YELLOW, reason);
    }
    actions::status_line(ui, cx.state);
}

fn draw(ui: &mut egui::Ui, cx: &mut ViewCtx, st: &mut State, debug: bool) {
    let table = &cx.state.params;
    let connected = cx.now.is_some();
    let blocked = status::configure_blocked(cx.access);
    st.prune(table);

    ui.horizontal(|ui| {
        if connected {
            fetch_status(ui, table, blocked.is_some());
        } else {
            let (text, colour) = status::not_connected();
            ui.colored_label(colour, text);
        }
        ui.add_enabled_ui(connected, |ui| {
            if ui.button("Refresh").clicked() {
                cx.commands.push(Command::RefreshParams);
            }
            let sendable = st.sendable(table, debug).len();
            let write = ui.add_enabled(
                sendable > 0 && blocked.is_none(),
                egui::Button::new(format!("Write changes ({sendable})")),
            );
            if let Some(reason) = blocked {
                write.clone().on_disabled_hover_text(reason);
            }
            if write.clicked() {
                st.write(table, debug, cx.commands);
            }
            if ui
                .add_enabled(!st.edits.is_empty(), egui::Button::new("Discard changes"))
                .clicked()
            {
                st.discard();
            }
        });
    });
    if connected && let Some(reason) = blocked {
        ui.colored_label(status::YELLOW, reason);
    }

    storage_buttons(ui, cx, &mut st.confirm);

    ui.add_enabled_ui(connected, |ui| {
        ui.add(
            egui::TextEdit::singleline(&mut st.search)
                .hint_text("Search")
                .desired_width(200.0),
        )
        .accessible_name("Search");

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
            let load = ui.add_enabled(blocked.is_none(), egui::Button::new("Load"));
            if let Some(reason) = blocked {
                load.clone().on_disabled_hover_text(reason);
            }
            if load.clicked() {
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

    let searching = !st.search.trim().is_empty();
    let tree = tree(table, &st.search);
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            for (scope, groups) in &tree {
                scope_section(
                    ui, table, st, blocked, connected, searching, debug, *scope, groups,
                );
            }
        });
}

fn counts(shown: usize, total: usize, searching: bool) -> String {
    if searching {
        format!("{shown} of {total}")
    } else {
        total.to_string()
    }
}

/// One top-level section. While searching, the ids differ from the ones of the user's own state, and
/// a section or group with a match is held open, so that clearing the search brings back the user's.
#[allow(clippy::too_many_arguments)]
fn scope_section(
    ui: &mut egui::Ui,
    table: &ParamTable,
    st: &mut State,
    blocked: Option<&'static str>,
    connected: bool,
    searching: bool,
    debug: bool,
    scope: Scope,
    groups: &[Group<'_>],
) {
    let total: usize = groups.iter().map(|g| g.total).sum();
    let shown: usize = groups.iter().map(|g| g.shown.len()).sum();
    let title = format!("{} ({})", scope.title(), counts(shown, total, searching));
    let mut header = egui::CollapsingHeader::new(title)
        .id_salt(("scope", scope.title(), searching))
        .default_open(true);
    if searching {
        header = header.open(Some(shown > 0 || total == 0));
    }
    header.show(ui, |ui| {
        if total == 0 {
            ui.label("no parameters from the vehicle yet");
        }
        for group in groups.iter().filter(|g| !searching || !g.shown.is_empty()) {
            let title = format!(
                "{} ({})",
                group.name,
                counts(group.shown.len(), group.total, searching)
            );
            let mut header = egui::CollapsingHeader::new(title).id_salt((
                "group",
                scope.title(),
                &group.name,
                searching,
            ));
            if searching {
                header = header.open(Some(true));
            }
            header.show(ui, |ui| {
                ui.add_enabled_ui(connected, |ui| {
                    rows(ui, table, st, blocked, debug, &group.name, &group.shown);
                });
            });
        }
    });
}

fn rows(
    ui: &mut egui::Ui,
    table: &ParamTable,
    st: &mut State,
    blocked: Option<&'static str>,
    debug: bool,
    id: &str,
    names: &[&String],
) {
    let row_height = ui.spacing().interact_size.y;
    ui.push_id(id, |ui| {
        TableBuilder::new(ui)
            .striped(true)
            .resizable(true)
            .vscroll(false)
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
                    let locked = !debug && presets::is_governed(name);
                    let row_state = if locked {
                        RowState::Clear
                    } else {
                        st.row_state(table, name)
                    };
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
                            .filter(|_| !locked)
                            .cloned()
                            .unwrap_or_else(|| param.value(table.encoding).to_string());
                        let mut field =
                            egui::TextEdit::singleline(&mut text).desired_width(f32::INFINITY);
                        if let Some(c) = colour {
                            field = field.text_color(c);
                        }
                        let response = ui
                            .add_enabled(blocked.is_none() && !locked, field)
                            .accessible_name(format!("value of {name}"));
                        if locked {
                            response
                                .clone()
                                .on_disabled_hover_text(presets::SET_IN_DEVICES_TEXT);
                        } else if let Some(reason) = blocked {
                            response.clone().on_disabled_hover_text(reason);
                        }
                        if response.changed() {
                            st.set(table, name, text);
                        }
                    });
                    row.col(|ui| {
                        ui.label(param.kind.name());
                    });
                    row.col(|ui| {
                        let mut text = state_text(&row_state);
                        if text.is_empty() && locked {
                            text = presets::SET_IN_DEVICES_TEXT.to_string();
                        } else if text.is_empty() && presets::applies_after_reboot(name) {
                            text = presets::AFTER_REBOOT_TEXT.to_string();
                        }
                        match colour {
                            Some(c) => ui.colored_label(c, text),
                            None => ui.label(text),
                        };
                    });
                });
            });
    });
}

fn fetch_status(ui: &mut egui::Ui, table: &ParamTable, operate_only: bool) {
    match &table.fetch {
        FetchStatus::Idle if operate_only => {
            ui.label("Not loaded: press Refresh to download the list");
        }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scopes_follow_the_group() {
        for (group, scope) in [
            ("RC1", Scope::RcController),
            ("RC8", Scope::RcController),
            ("RC12", Scope::RcController),
            ("RCMAP", Scope::RcController),
            ("RCX", Scope::FlightController),
            ("RC", Scope::FlightController),
            ("RC1X", Scope::FlightController),
            ("ESC", Scope::Esc),
            ("MOT", Scope::Esc),
            ("DSHOT", Scope::Esc),
            ("BARO", Scope::FlightController),
            ("IMU1", Scope::FlightController),
            ("LOG", Scope::FlightController),
            ("(other)", Scope::FlightController),
        ] {
            assert_eq!(scope_of(group), scope, "{group}");
        }
    }

    #[test]
    fn groups_come_from_the_prefix() {
        assert_eq!(group_of("RC1_MIN"), "RC1");
        assert_eq!(group_of("LOG_BACKEND_TYPE"), "LOG");
        assert_eq!(group_of("LONER"), OTHER_GROUP);
        assert_eq!(group_of("_X"), OTHER_GROUP);
    }

    #[test]
    fn numbers_in_group_names_order_by_value() {
        let mut names = [
            "RC10", "RC2", "RCMAP", "RC1", "IMU2", "IMU1", "RC01", "BARO",
        ];
        names.sort_by(|a, b| natural_cmp(a, b));
        assert_eq!(
            names,
            [
                "BARO", "IMU1", "IMU2", "RC01", "RC1", "RC2", "RC10", "RCMAP"
            ]
        );
    }
}
