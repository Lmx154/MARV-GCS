//! The Map tab's part of the vehicle's mission service (docs/link-contract.md, section 17): the edited list
//! converted to the service's waypoints, the three actions (Upload, Read, Clear) with the progress and the
//! verdict the service reports, and the comparison of what the vehicle holds with the edited list. The vehicle
//! only stores the list. The station claims no more than the service concluded: "verified" only when
//! `MissionStatus::Verified` says so.

use std::time::Duration;

use eframe::egui::{self, Color32, text::LayoutJob};
use gcs_mavlink::services::missions::{
    MissionFailure, MissionKind, MissionPhase, MissionPoint, MissionStatus, MissionsState,
    VehicleList, degrees_to_e7, plan_id,
};
use gcs_mavlink::{Command, LinkStatus};

use super::{AMBER, ViewCtx};
use crate::missions::Waypoint;
use crate::theme;
use crate::views::actions;

/// What the vehicle holds at most; a longer list is still sent, and the vehicle refuses it with its own code.
pub const CAPACITY: usize = 16;
pub const OVER_CAPACITY: &str =
    "More than 16 points: the vehicle holds at most 16 and will refuse this list.";
pub const ONLY_STORES: &str =
    "The vehicle only stores the list; nothing here makes it follow the list.";
pub const RUNNING: &str = "a mission operation is running";
pub const EMPTY_LIST: &str =
    "the edited list is empty (Clear on vehicle empties the vehicle's list)";
pub const EMPTY_VEHICLE_LIST: &str = "the vehicle's list is empty";
pub const SENT: &str = "Request sent, waiting for the vehicle";
/// Colour of the vehicle's list on the map and in the legend (the edited list is amber).
pub const ON_VEHICLE: Color32 = theme::ACCENT;

/// Upload and Clear change the vehicle's stored list, so they ask first. Set to false and they send on one
/// press again (the `Pending` below is then never created).
pub const CONFIRM_BEFORE_SENDING: bool = true;

/// A command that the station's session has not started yet is forgotten after this long (session time).
const AWAIT_MAX: Duration = Duration::from_secs(3);

/// A waypoint of the edited list that cannot be converted to a waypoint of the service.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Unsendable {
    /// As the table numbers it (from 1).
    pub number: usize,
    pub problem: &'static str,
}

/// The service's waypoints for the edited list: latitude and longitude by `degrees_to_e7`, altitude as given. The
/// home placeholder of the file is not in the list (`crate::missions` skips row 0), so every point is sent. Ranges
/// of the vehicle are checked by the service itself, which refuses the whole list and names the waypoint.
pub fn to_items(list: &[Waypoint]) -> Result<Vec<MissionPoint>, Unsendable> {
    list.iter()
        .enumerate()
        .map(|(at, w)| {
            let refuse = |problem| Unsendable {
                number: at + 1,
                problem,
            };
            let lat_e7 = if w.lat.is_finite() {
                degrees_to_e7(w.lat).ok_or(refuse("latitude out of range"))?
            } else {
                return Err(refuse("latitude is not finite"));
            };
            let lon_e7 = if w.lon.is_finite() {
                degrees_to_e7(w.lon).ok_or(refuse("longitude out of range"))?
            } else {
                return Err(refuse("longitude is not finite"));
            };
            if !w.alt_m.is_finite() {
                return Err(refuse("altitude is not finite"));
            }
            Ok(MissionPoint {
                lat_e7,
                lon_e7,
                z: w.alt_m,
            })
        })
        .collect()
}

/// The vehicle's waypoints as editor waypoints (degrees).
pub fn waypoints_of(items: &[MissionPoint]) -> Vec<Waypoint> {
    items
        .iter()
        .map(|p| Waypoint {
            lat: f64::from(p.lat_e7) / 1e7,
            lon: f64::from(p.lon_e7) / 1e7,
            alt_m: p.z,
        })
        .collect()
}

/// How the vehicle's list differs from the edited list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Difference {
    Count {
        vehicle: usize,
        edited: usize,
    },
    /// The first waypoint (from 1) that is not the same.
    Waypoint {
        number: usize,
    },
    PlanId {
        vehicle: u32,
        edited: u32,
    },
    Unsendable(Unsendable),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Comparison {
    Matches,
    Differs(Difference),
}

/// The edited list as the service would send it against what the vehicle holds: the points as `MissionPoint`
/// (`degrees_to_e7`, altitude compared as 4 bytes) and the plan ids.
pub fn compare(edited: &[Waypoint], vehicle: &VehicleList) -> Comparison {
    let items = match to_items(edited) {
        Ok(items) => items,
        Err(unsendable) => return Comparison::Differs(Difference::Unsendable(unsendable)),
    };
    if items.len() != vehicle.items.len() {
        return Comparison::Differs(Difference::Count {
            vehicle: vehicle.items.len(),
            edited: items.len(),
        });
    }
    if let Some(at) = items.iter().zip(&vehicle.items).position(|(a, b)| a != b) {
        return Comparison::Differs(Difference::Waypoint { number: at + 1 });
    }
    let edited_id = plan_id(&items);
    if edited_id != vehicle.plan_id {
        return Comparison::Differs(Difference::PlanId {
            vehicle: vehicle.plan_id,
            edited: edited_id,
        });
    }
    Comparison::Matches
}

pub fn count_text(count: usize) -> String {
    if count == 1 {
        "1 waypoint".to_string()
    } else {
        format!("{count} waypoints")
    }
}

pub fn comparison_line(comparison: Comparison) -> (String, Color32) {
    const DIFFERS: &str = "The vehicle's list differs from the edited list";
    match comparison {
        Comparison::Matches => (
            "The vehicle's list matches the edited list".to_string(),
            theme::OK,
        ),
        Comparison::Differs(difference) => {
            let why = match difference {
                Difference::Count { vehicle, edited } => format!(
                    "the vehicle holds {}, the edited list has {}",
                    count_text(vehicle),
                    count_text(edited)
                ),
                Difference::Waypoint { number } => format!("waypoint {number} is not the same"),
                Difference::PlanId { vehicle, edited } => format!(
                    "plan id {vehicle:#010X} on the vehicle, {edited:#010X} for the edited list"
                ),
                Difference::Unsendable(Unsendable { number, problem }) => {
                    format!("waypoint {number} of the edited list cannot be sent ({problem})")
                }
            };
            (format!("{DIFFERS}: {why}"), theme::WARN)
        }
    }
}

/// An operation of the service is running.
pub fn running(status: &MissionStatus) -> bool {
    matches!(
        status,
        MissionStatus::Uploading { .. }
            | MissionStatus::Clearing
            | MissionStatus::Downloading { .. }
            | MissionStatus::ReadingBack { .. }
    )
}

fn holds(list: &VehicleList) -> String {
    format!(
        "the vehicle holds {}, plan id {:#010X}",
        count_text(list.items.len()),
        list.plan_id
    )
}

/// The failure in words: the vehicle's refusal with its code and meaning, or what the service says.
fn failure_text(reason: &MissionFailure) -> String {
    match reason {
        MissionFailure::Refused(result) => format!(
            "refused by the vehicle, code {} ({result:?}): {}",
            *result as u32,
            gcs_mavlink::services::missions::result_meaning(*result)
        ),
        MissionFailure::InvalidWaypoint { seq, problem } => {
            format!(
                "waypoint {}: {problem}; nothing was sent",
                u32::from(*seq) + 1
            )
        }
        other => other.to_string(),
    }
}

/// The progress or the verdict of the current or last operation of the service, in words and colour; `None`
/// before the first one. A success is shown only for `Verified` (an upload or a clear) and `Complete` (a read).
pub fn line(state: &MissionsState) -> Option<(String, Color32)> {
    if state.runs == 0 {
        return None;
    }
    let what = match state.kind {
        Some(MissionKind::Upload) => "Upload",
        Some(MissionKind::Clear) => "Clear",
        Some(MissionKind::Download) | None => "Read",
    };
    let progress = |received: u16, total: Option<u16>, lead: &str| match total {
        Some(total) => format!("{lead} {received} of {}", count_text(usize::from(total))),
        None => format!("{lead}: waiting for the vehicle"),
    };
    let held = state.vehicle.as_ref().map(holds);
    Some(match &state.status {
        MissionStatus::Idle => return None,
        MissionStatus::Uploading { sent, total } => (
            format!("Uploading {sent} of {}", count_text(usize::from(*total))),
            theme::WARN,
        ),
        MissionStatus::Clearing => ("Clearing the vehicle's list".to_string(), theme::WARN),
        MissionStatus::Downloading { received, total } => {
            (progress(*received, *total, "Reading"), theme::WARN)
        }
        MissionStatus::ReadingBack { received, total } => {
            (progress(*received, *total, "Reading back"), theme::WARN)
        }
        MissionStatus::Verified => (
            match held {
                Some(held) => format!("{what} verified: {held}"),
                None => format!("{what} verified"),
            },
            theme::OK,
        ),
        MissionStatus::Complete => (
            match held {
                Some(held) => format!("{what} complete: {held}"),
                None => format!("{what} complete"),
            },
            theme::OK,
        ),
        MissionStatus::Failed { reason, phase } => {
            let stage = match phase {
                MissionPhase::Transfer => "",
                MissionPhase::ReadBack => " (read-back)",
            };
            let held = held.map_or_else(String::new, |held| format!("; {held}"));
            (
                format!("{what} failed{stage}: {}{held}", failure_text(reason)),
                theme::ERROR,
            )
        }
    })
}

/// A command sent from here that the service has not started (or refused) yet.
#[derive(Debug, Clone, Copy)]
struct Awaiting {
    runs: u32,
    refused: u32,
    since: Duration,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Action {
    Upload,
    Clear,
}

/// An Upload or Clear that waits for Confirm, with the list and selection it was armed on: a change to
/// either one cancels it.
#[derive(Debug, Clone, PartialEq)]
struct Pending {
    action: Action,
    list: Vec<Waypoint>,
    selected: Option<usize>,
}

/// What the Map tab remembers of its dealings with the service: the service's state is in `VehicleState`.
#[derive(Debug, Default)]
pub struct Controls {
    /// A command was sent and the service's `runs` has not moved yet: the status in the state is still the
    /// previous operation's, so it is not shown as this one's.
    awaiting: Option<Awaiting>,
    /// The station's own refusal before sending: the value of `runs` at that time and the line.
    refusal: Option<(u32, String)>,
    pending: Option<Pending>,
}

impl Controls {
    pub fn session_changed(&mut self) {
        *self = Self::default();
    }

    fn settle(&mut self, cx: &ViewCtx) {
        let missions = &cx.state.missions;
        let now = cx.now.unwrap_or_default();
        if self.awaiting.is_some_and(|a| {
            missions.runs != a.runs
                || missions.refused != a.refused
                || cx.now.is_none()
                || now.saturating_sub(a.since) > AWAIT_MAX
        }) {
            self.awaiting = None;
        }
    }

    /// Why the three actions are disabled; `None` when a vehicle is connected and nothing is running.
    fn gate(&self, cx: &ViewCtx) -> Option<&'static str> {
        let link = &cx.state.link;
        if cx.now.is_none() || link.status != LinkStatus::Connected || link.vehicle.is_none() {
            return Some(crate::status::NOT_CONNECTED);
        }
        if self.awaiting.is_some() || running(&cx.state.missions.status) {
            return Some(RUNNING);
        }
        None
    }

    fn send(&mut self, cx: &mut ViewCtx, command: Command) {
        self.refusal = None;
        self.awaiting = Some(Awaiting {
            runs: cx.state.missions.runs,
            refused: cx.state.missions.refused,
            since: cx.now.unwrap_or_default(),
        });
        cx.commands.push(command);
    }

    fn upload(&mut self, cx: &mut ViewCtx, list: &[Waypoint]) {
        match to_items(list) {
            Ok(items) => self.send(cx, Command::UploadMission { items }),
            Err(Unsendable { number, problem }) => {
                self.refusal = Some((
                    cx.state.missions.runs,
                    format!("Upload failed: waypoint {number}: {problem}; nothing was sent"),
                ));
            }
        }
    }

    fn press(
        &mut self,
        cx: &mut ViewCtx,
        action: Action,
        list: &[Waypoint],
        selected: Option<usize>,
    ) {
        if CONFIRM_BEFORE_SENDING {
            self.pending = Some(Pending {
                action,
                list: list.to_vec(),
                selected,
            });
        } else {
            self.run(cx, action, list);
        }
    }

    fn run(&mut self, cx: &mut ViewCtx, action: Action, list: &[Waypoint]) {
        match action {
            Action::Upload => self.upload(cx, list),
            Action::Clear => self.send(cx, Command::ClearMission),
        }
    }

    fn confirm(&mut self, cx: &mut ViewCtx, list: &[Waypoint]) {
        if let Some(pending) = self.pending.take() {
            self.run(cx, pending.action, list);
        }
    }

    /// Forget a pending action when the list or the selection moved or the buttons are disabled.
    fn keep_pending(&mut self, list: &[Waypoint], selected: Option<usize>, gate: Option<&str>) {
        if self
            .pending
            .as_ref()
            .is_some_and(|p| gate.is_some() || p.list != list || p.selected != selected)
        {
            self.pending = None;
        }
    }

    fn question(pending: &Pending) -> String {
        match pending.action {
            Action::Upload => format!("Upload {} to the vehicle?", count_text(pending.list.len())),
            Action::Clear => "Clear the vehicle's waypoint list?".to_string(),
        }
    }

    /// The line under the buttons: the request just sent, the station's own refusal, or the service's.
    fn status_line(&self, state: &MissionsState) -> Option<(String, Color32)> {
        if self.awaiting.is_some() {
            return Some((SENT.to_string(), theme::WARN));
        }
        if let Some((runs, text)) = &self.refusal
            && *runs == state.runs
        {
            return Some((text.clone(), theme::ERROR));
        }
        line(state)
    }

    /// The vehicle section of the waypoint panel. True when "Copy to a new list" was pressed.
    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        cx: &mut ViewCtx,
        list: &[Waypoint],
        selected: Option<usize>,
    ) -> bool {
        self.settle(cx);
        theme::section_title(ui, "On the vehicle");
        ui.label(theme::muted(ONLY_STORES).small());
        let gate = self.gate(cx);
        self.keep_pending(list, selected, gate);
        ui.horizontal_wrapped(|ui| {
            let upload_gate = gate.or(list.is_empty().then_some(EMPTY_LIST));
            if actions::button(ui, "Upload to vehicle", upload_gate) {
                self.press(cx, Action::Upload, list, selected);
            }
            if actions::button(ui, "Read from vehicle", gate) {
                self.pending = None;
                self.send(cx, Command::DownloadMission);
            }
            if actions::button(ui, "Clear on vehicle", gate) {
                self.press(cx, Action::Clear, list, selected);
            }
        });
        if let Some(pending) = &self.pending {
            let question = Self::question(pending);
            let (mut yes, mut no) = (false, false);
            ui.horizontal_wrapped(|ui| {
                ui.colored_label(theme::WARN, question);
                yes = actions::button(ui, "Confirm", None);
                no = actions::button(ui, "Cancel", None);
            });
            if yes {
                self.confirm(cx, list);
            } else if no {
                self.pending = None;
            }
        }
        if let Some((text, colour)) = self.status_line(&cx.state.missions) {
            ui.colored_label(colour, text);
        }
        let mut copy = false;
        if let Some(vehicle) = &cx.state.missions.vehicle {
            ui.label(legend(ui));
            let (text, colour) = comparison_line(compare(list, vehicle));
            ui.colored_label(colour, text);
            let reason = vehicle.items.is_empty().then_some(EMPTY_VEHICLE_LIST);
            copy = actions::button(ui, "Copy to a new list", reason);
        }
        copy
    }
}

/// The legend of the two lists on the map.
fn legend(ui: &egui::Ui) -> LayoutJob {
    let mut job = LayoutJob::default();
    let font = egui::TextStyle::Small.resolve(ui.style());
    let mut add = |text: &str, colour: Color32| {
        job.append(
            text,
            0.0,
            egui::TextFormat {
                font_id: font.clone(),
                color: colour,
                ..egui::TextFormat::default()
            },
        );
    };
    add("Edited list: amber discs. ", AMBER);
    add(
        "Vehicle's list as last read: blue dashed rings.",
        ON_VEHICLE,
    );
    job
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcs_mavlink::dialect::MavMissionResult;

    fn example() -> Vec<Waypoint> {
        [
            (10.0, -30.0, 10.0),
            (10.001, -30.0, 12.5),
            (10.001, -29.999, 15.0),
        ]
        .into_iter()
        .map(|(lat, lon, alt_m)| Waypoint { lat, lon, alt_m })
        .collect()
    }

    fn held(items: Vec<MissionPoint>) -> VehicleList {
        VehicleList {
            plan_id: plan_id(&items),
            items,
        }
    }

    fn state(kind: MissionKind, status: MissionStatus) -> MissionsState {
        MissionsState {
            status,
            kind: Some(kind),
            runs: 1,
            ..MissionsState::default()
        }
    }

    #[test]
    fn the_contracts_example_converts_to_its_items_and_plan_id() {
        let items = to_items(&example()).unwrap();
        let wire: Vec<(i32, i32, f32)> = items.iter().map(|p| (p.lat_e7, p.lon_e7, p.z)).collect();
        assert_eq!(
            wire,
            [
                (100_000_000, -300_000_000, 10.0),
                (100_010_000, -300_000_000, 12.5),
                (100_010_000, -299_990_000, 15.0),
            ]
        );
        assert_eq!(plan_id(&items), 0x99C3_BD49);
        assert_eq!(to_items(&[]), Ok(Vec::new()));
    }

    #[test]
    fn degrees_are_rounded_to_the_nearest_e7_halves_away_from_zero() {
        let one = |lat, lon| {
            to_items(&[Waypoint {
                lat,
                lon,
                alt_m: 0.0,
            }])
            .unwrap()[0]
        };
        assert_eq!(one(0.000_000_05, -0.000_000_05).lat_e7, 1);
        assert_eq!(one(0.000_000_05, -0.000_000_05).lon_e7, -1);
        assert_eq!(one(90.0, 180.0).lat_e7, 900_000_000);
        assert_eq!(one(-90.0, -180.0).lon_e7, -1_800_000_000);
    }

    #[test]
    fn a_point_that_cannot_be_converted_names_its_number_and_reason() {
        let at = |lat, lon, alt_m| {
            let mut list = example();
            list[1] = Waypoint { lat, lon, alt_m };
            to_items(&list).unwrap_err()
        };
        let refuse = |problem| Unsendable { number: 2, problem };
        assert_eq!(at(f64::NAN, 0.0, 0.0), refuse("latitude is not finite"));
        assert_eq!(
            at(0.0, f64::INFINITY, 0.0),
            refuse("longitude is not finite")
        );
        assert_eq!(at(1e300, 0.0, 0.0), refuse("latitude out of range"));
        assert_eq!(at(0.0, -1e300, 0.0), refuse("longitude out of range"));
        assert_eq!(at(0.0, 0.0, f32::NAN), refuse("altitude is not finite"));
        // Past the globe but inside an i32: the service refuses it, naming the waypoint.
        assert!(
            to_items(&[Waypoint {
                lat: 100.0,
                lon: 0.0,
                alt_m: 0.0
            }])
            .is_ok()
        );
    }

    #[test]
    fn more_than_16_points_are_converted_and_not_cut() {
        let list = vec![example()[0]; 17];
        assert_eq!(to_items(&list).unwrap().len(), 17);
        assert_eq!(CAPACITY, 16);
        assert!(OVER_CAPACITY.contains("16"));
    }

    #[test]
    fn the_vehicles_points_come_back_as_degrees() {
        let back = waypoints_of(&to_items(&example()).unwrap());
        assert_eq!(back, example());
    }

    #[test]
    fn the_same_points_and_plan_id_match() {
        let vehicle = held(to_items(&example()).unwrap());
        assert_eq!(compare(&example(), &vehicle), Comparison::Matches);
        assert_eq!(
            compare(
                &[],
                &VehicleList {
                    plan_id: 0,
                    items: Vec::new()
                }
            ),
            Comparison::Matches
        );
        assert_eq!(
            comparison_line(Comparison::Matches),
            (
                "The vehicle's list matches the edited list".to_string(),
                theme::OK
            )
        );
    }

    #[test]
    fn a_count_a_point_or_the_plan_id_that_differs_is_named() {
        let vehicle = held(to_items(&example()).unwrap());
        let mut edited = example();
        edited[1].alt_m = 12.75;
        assert_eq!(
            compare(&edited, &vehicle),
            Comparison::Differs(Difference::Waypoint { number: 2 })
        );
        edited.pop();
        assert_eq!(
            compare(&edited, &vehicle),
            Comparison::Differs(Difference::Count {
                vehicle: 3,
                edited: 2
            })
        );
        // An altitude is compared as 4 bytes: -0.0 is not 0.0.
        let zero = held(
            to_items(&[Waypoint {
                lat: 1.0,
                lon: 2.0,
                alt_m: 0.0,
            }])
            .unwrap(),
        );
        assert_eq!(
            compare(
                &[Waypoint {
                    lat: 1.0,
                    lon: 2.0,
                    alt_m: -0.0
                }],
                &zero
            ),
            Comparison::Differs(Difference::Waypoint { number: 1 })
        );
        // Same points, another plan id.
        let other = VehicleList {
            plan_id: 7,
            ..held(to_items(&example()).unwrap())
        };
        assert_eq!(
            compare(&example(), &other),
            Comparison::Differs(Difference::PlanId {
                vehicle: 7,
                edited: 0x99C3_BD49
            })
        );
        let mut broken = example();
        broken[2].lat = f64::NAN;
        assert_eq!(
            compare(&broken, &vehicle),
            Comparison::Differs(Difference::Unsendable(Unsendable {
                number: 3,
                problem: "latitude is not finite"
            }))
        );
    }

    #[test]
    fn the_comparison_lines_say_what_differs() {
        let line = |difference| comparison_line(Comparison::Differs(difference));
        assert_eq!(
            line(Difference::Count {
                vehicle: 3,
                edited: 4
            }),
            (
                "The vehicle's list differs from the edited list: the vehicle holds 3 waypoints, the edited list has 4 waypoints"
                    .to_string(),
                theme::WARN
            )
        );
        assert_eq!(
            line(Difference::Waypoint { number: 2 }).0,
            "The vehicle's list differs from the edited list: waypoint 2 is not the same"
        );
        assert_eq!(
            line(Difference::PlanId {
                vehicle: 1,
                edited: 0x99C3_BD49
            })
            .0,
            "The vehicle's list differs from the edited list: plan id 0x00000001 on the vehicle, 0x99C3BD49 for the edited list"
        );
        assert_eq!(
            line(Difference::Unsendable(Unsendable {
                number: 4,
                problem: "altitude is not finite"
            }))
            .0,
            "The vehicle's list differs from the edited list: waypoint 4 of the edited list cannot be sent (altitude is not finite)"
        );
    }

    #[test]
    fn nothing_is_said_before_the_first_operation() {
        assert_eq!(line(&MissionsState::default()), None);
        assert_eq!(
            line(&MissionsState {
                kind: Some(MissionKind::Upload),
                status: MissionStatus::Verified,
                ..MissionsState::default()
            }),
            None,
            "a status with no operation started is not shown"
        );
    }

    #[test]
    fn the_progress_lines() {
        let progress = |kind, status| line(&state(kind, status)).unwrap();
        assert_eq!(
            progress(
                MissionKind::Upload,
                MissionStatus::Uploading { sent: 2, total: 3 }
            ),
            ("Uploading 2 of 3 waypoints".to_string(), theme::WARN)
        );
        assert_eq!(
            progress(
                MissionKind::Upload,
                MissionStatus::Uploading { sent: 0, total: 1 }
            )
            .0,
            "Uploading 0 of 1 waypoint"
        );
        assert_eq!(
            progress(MissionKind::Clear, MissionStatus::Clearing).0,
            "Clearing the vehicle's list"
        );
        assert_eq!(
            progress(
                MissionKind::Download,
                MissionStatus::Downloading {
                    received: 0,
                    total: None
                }
            )
            .0,
            "Reading: waiting for the vehicle"
        );
        assert_eq!(
            progress(
                MissionKind::Download,
                MissionStatus::Downloading {
                    received: 1,
                    total: Some(3)
                }
            )
            .0,
            "Reading 1 of 3 waypoints"
        );
        assert_eq!(
            progress(
                MissionKind::Upload,
                MissionStatus::ReadingBack {
                    received: 2,
                    total: Some(3)
                }
            )
            .0,
            "Reading back 2 of 3 waypoints"
        );
        assert_eq!(
            progress(
                MissionKind::Clear,
                MissionStatus::ReadingBack {
                    received: 0,
                    total: None
                }
            )
            .0,
            "Reading back: waiting for the vehicle"
        );
        for status in [
            MissionStatus::Uploading { sent: 0, total: 3 },
            MissionStatus::Clearing,
            MissionStatus::Downloading {
                received: 0,
                total: None,
            },
            MissionStatus::ReadingBack {
                received: 0,
                total: None,
            },
        ] {
            assert!(running(&status), "{status:?}");
            assert_eq!(progress(MissionKind::Upload, status).1, theme::WARN);
        }
    }

    #[test]
    fn success_is_shown_only_for_verified_and_complete_with_the_count_and_plan_id() {
        let vehicle = held(to_items(&example()).unwrap());
        let with = |kind, status| MissionsState {
            vehicle: Some(vehicle.clone()),
            ..state(kind, status)
        };
        assert_eq!(
            line(&with(MissionKind::Upload, MissionStatus::Verified)),
            Some((
                "Upload verified: the vehicle holds 3 waypoints, plan id 0x99C3BD49".to_string(),
                theme::OK
            ))
        );
        assert_eq!(
            line(&with(MissionKind::Download, MissionStatus::Complete)),
            Some((
                "Read complete: the vehicle holds 3 waypoints, plan id 0x99C3BD49".to_string(),
                theme::OK
            ))
        );
        let empty = MissionsState {
            vehicle: Some(held(Vec::new())),
            ..state(MissionKind::Clear, MissionStatus::Verified)
        };
        assert_eq!(
            line(&empty),
            Some((
                "Clear verified: the vehicle holds 0 waypoints, plan id 0x00000000".to_string(),
                theme::OK
            ))
        );
        for status in [
            MissionStatus::Idle,
            MissionStatus::Uploading { sent: 3, total: 3 },
            MissionStatus::ReadingBack {
                received: 3,
                total: Some(3),
            },
        ] {
            let text = line(&with(MissionKind::Upload, status)).map_or(String::new(), |l| l.0);
            assert!(
                !text.contains("verified") && !text.contains("complete"),
                "{text}"
            );
        }
    }

    fn failed(reason: MissionFailure, phase: MissionPhase) -> MissionsState {
        state(MissionKind::Upload, MissionStatus::Failed { reason, phase })
    }

    #[test]
    fn a_refusal_shows_the_vehicles_code_and_its_meaning_and_what_it_holds() {
        let mut shown = failed(
            MissionFailure::Refused(MavMissionResult::MAV_MISSION_NO_SPACE),
            MissionPhase::Transfer,
        );
        assert_eq!(
            line(&shown),
            Some((
                "Upload failed: refused by the vehicle, code 4 (MAV_MISSION_NO_SPACE): the vehicle holds at most 16 waypoints"
                    .to_string(),
                theme::ERROR
            ))
        );
        shown.vehicle = Some(held(to_items(&example()).unwrap()));
        assert_eq!(
            line(&shown).unwrap().0,
            "Upload failed: refused by the vehicle, code 4 (MAV_MISSION_NO_SPACE): the vehicle holds at most 16 waypoints; the vehicle holds 3 waypoints, plan id 0x99C3BD49"
        );
        let armed = failed(
            MissionFailure::Refused(MavMissionResult::MAV_MISSION_ERROR),
            MissionPhase::Transfer,
        );
        assert_eq!(
            line(&armed).unwrap().0,
            "Upload failed: refused by the vehicle, code 1 (MAV_MISSION_ERROR): the vehicle is armed, or no transfer was running"
        );
    }

    #[test]
    fn the_other_failures_in_words() {
        let text = |reason, phase| line(&failed(reason, phase)).unwrap().0;
        assert_eq!(
            text(MissionFailure::TimedOut, MissionPhase::Transfer),
            "Upload failed: no answer from the vehicle"
        );
        assert_eq!(
            text(MissionFailure::LinkLost, MissionPhase::ReadBack),
            "Upload failed (read-back): link lost"
        );
        assert_eq!(
            text(MissionFailure::NoVehicle, MissionPhase::Transfer),
            "Upload failed: the vehicle is not connected"
        );
        assert_eq!(
            text(
                MissionFailure::CountDiffers { sent: 3, found: 2 },
                MissionPhase::ReadBack
            ),
            "Upload failed (read-back): the vehicle holds 2 waypoints, 3 were sent"
        );
        assert_eq!(
            text(
                MissionFailure::ItemDiffers { seq: 1 },
                MissionPhase::ReadBack
            ),
            "Upload failed (read-back): waypoint 1 read back differs from the one sent"
        );
        assert_eq!(
            text(
                MissionFailure::PlanIdDiffers {
                    sent: 0x99C3_BD49,
                    found: 5
                },
                MissionPhase::ReadBack
            ),
            "Upload failed (read-back): plan id 0x00000005 read back, 0x99C3BD49 sent"
        );
        // The service counts from 0; the table from 1.
        assert_eq!(
            text(
                MissionFailure::InvalidWaypoint {
                    seq: 1,
                    problem: "latitude out of range"
                },
                MissionPhase::Transfer
            ),
            "Upload failed: waypoint 2: latitude out of range; nothing was sent"
        );
        let read = MissionsState {
            kind: Some(MissionKind::Download),
            ..failed(
                MissionFailure::Refused(MavMissionResult::MAV_MISSION_DENIED),
                MissionPhase::Transfer,
            )
        };
        assert_eq!(
            line(&read).unwrap().0,
            "Read failed: refused by the vehicle, code 14 (MAV_MISSION_DENIED): another owner's transfer is running"
        );
    }

    #[test]
    fn a_list_that_cannot_be_converted_sends_nothing_and_names_the_waypoint() {
        let state = gcs_mavlink::VehicleState::default();
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now: Some(Duration::from_secs(1)),
            access: gcs_mavlink::Access::Configure,
        };
        let mut controls = Controls::default();
        let mut list = example();
        list[1].lat = f64::NAN;
        controls.upload(&mut cx, &list);
        assert!(commands.is_empty());
        assert!(
            controls.awaiting.is_none(),
            "nothing was sent: nothing to wait for"
        );
        assert_eq!(
            controls.status_line(&state.missions),
            Some((
                "Upload failed: waypoint 2: latitude is not finite; nothing was sent".to_string(),
                theme::ERROR
            ))
        );
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now: Some(Duration::from_secs(2)),
            access: gcs_mavlink::Access::Configure,
        };
        controls.upload(&mut cx, &example());
        assert!(matches!(commands[..], [Command::UploadMission { .. }]));
        assert_eq!(controls.status_line(&state.missions).unwrap().0, SENT);
    }

    #[test]
    fn the_stations_own_refusal_is_shown_only_for_the_state_it_was_made_in() {
        let mut controls = Controls {
            refusal: Some((4, "Upload failed: waypoint 2: x; nothing was sent".into())),
            ..Controls::default()
        };
        let old = MissionsState {
            runs: 4,
            ..state(MissionKind::Upload, MissionStatus::Verified)
        };
        assert_eq!(
            controls.status_line(&old).unwrap().0,
            "Upload failed: waypoint 2: x; nothing was sent"
        );
        let newer = MissionsState { runs: 5, ..old };
        assert_eq!(controls.status_line(&newer).unwrap().0, "Upload verified");
        controls.awaiting = Some(Awaiting {
            runs: 5,
            refused: 0,
            since: Duration::ZERO,
        });
        assert_eq!(controls.status_line(&newer).unwrap().0, SENT);
    }

    fn connected() -> gcs_mavlink::VehicleState {
        let mut state = gcs_mavlink::VehicleState::default();
        state.link.status = LinkStatus::Connected;
        state
    }

    fn only_command(commands: &[Command]) -> &Command {
        assert_eq!(commands.len(), 1, "{commands:?}");
        &commands[0]
    }

    #[test]
    fn upload_asks_first_then_sends_the_same_points_once_on_confirm() {
        let state = connected();
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now: Some(Duration::from_secs(1)),
            access: gcs_mavlink::Access::Configure,
        };
        let mut controls = Controls::default();
        let list = example();
        controls.press(&mut cx, Action::Upload, &list, None);
        assert!(cx.commands.is_empty(), "the first press sends nothing");
        let pending = controls.pending.clone().unwrap();
        assert_eq!(
            Controls::question(&pending),
            "Upload 3 waypoints to the vehicle?"
        );
        controls.confirm(&mut cx, &list);
        assert!(controls.pending.is_none());
        let Command::UploadMission { items } = only_command(cx.commands) else {
            panic!("not an upload");
        };
        assert_eq!(*items, to_items(&list).unwrap());
    }

    #[test]
    fn clear_asks_first_then_sends_one_clear_on_confirm() {
        let state = connected();
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now: Some(Duration::from_secs(1)),
            access: gcs_mavlink::Access::Configure,
        };
        let mut controls = Controls::default();
        let list = example();
        controls.press(&mut cx, Action::Clear, &list, None);
        assert!(cx.commands.is_empty(), "the first press sends nothing");
        assert_eq!(
            Controls::question(controls.pending.as_ref().unwrap()),
            "Clear the vehicle's waypoint list?"
        );
        controls.confirm(&mut cx, &list);
        assert!(matches!(only_command(cx.commands), Command::ClearMission));
    }

    #[test]
    fn cancel_and_any_change_of_list_selection_or_gate_send_nothing() {
        let mut controls = Controls::default();
        let list = example();
        for action in [Action::Upload, Action::Clear] {
            let arm = |controls: &mut Controls| {
                controls.pending = Some(Pending {
                    action,
                    list: list.clone(),
                    selected: Some(1),
                });
            };
            arm(&mut controls);
            controls.keep_pending(&list, Some(1), None);
            assert!(controls.pending.is_some(), "unchanged stays armed");
            controls.keep_pending(&list, Some(0), None);
            assert!(controls.pending.is_none(), "selection moved");
            arm(&mut controls);
            controls.keep_pending(&list[..2], Some(1), None);
            assert!(controls.pending.is_none(), "list changed");
            arm(&mut controls);
            controls.keep_pending(&list, Some(1), Some(RUNNING));
            assert!(controls.pending.is_none(), "buttons disabled");
            arm(&mut controls);
            controls.pending = None;
            let state = connected();
            let mut commands = Vec::new();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut commands,
                now: Some(Duration::from_secs(1)),
                access: gcs_mavlink::Access::Configure,
            };
            controls.confirm(&mut cx, &list);
            assert!(cx.commands.is_empty(), "cancelled: nothing is sent");
        }
    }

    #[test]
    fn read_still_sends_on_one_press() {
        let state = connected();
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now: Some(Duration::from_secs(1)),
            access: gcs_mavlink::Access::Configure,
        };
        let mut controls = Controls::default();
        controls.send(&mut cx, Command::DownloadMission);
        assert!(matches!(
            only_command(cx.commands),
            Command::DownloadMission
        ));
        assert!(controls.pending.is_none());
    }
}
