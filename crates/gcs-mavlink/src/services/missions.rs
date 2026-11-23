//! Mission service: uploads a waypoint list to the vehicle, reads it back, clears it (docs/link-contract.md,
//! section 17). Sans-IO: the time comes from `cx.now`. An upload or a clear is done only when a read-back
//! shows the vehicle holds what was sent (the station's verdict).

use std::fmt;
use std::time::Duration;

use super::{Ctx, Service};
use crate::dialect::{
    MISSION_ACK_DATA, MISSION_CLEAR_ALL_DATA, MISSION_COUNT_DATA, MISSION_ITEM_INT_DATA,
    MISSION_REQUEST_INT_DATA, MISSION_REQUEST_LIST_DATA, MavCmd, MavFrame, MavMissionResult,
    MavMissionType,
};
use crate::session::SessionConfig;
use crate::{Body, Command, Frame, LinkStatus, Message};

/// A message with no response is sent again this long after its previous send.
pub const RETRY_AFTER: Duration = Duration::from_millis(1500);
/// Sends again after the first send: 6 sends in all, then the sender gives up.
pub const RETRIES: u8 = 5;
/// After an item other than the last, the station waits this long for the vehicle's next message.
pub const ITEM_WAIT: Duration = Duration::from_millis(9000);

const FRAME: MavFrame = MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT;
const WAYPOINT: MavCmd = MavCmd::MAV_CMD_NAV_WAYPOINT;
const PLAN: MavMissionType = MavMissionType::MAV_MISSION_TYPE_MISSION;

/// One waypoint as the station sends it: `x`, `y` in degrees x 10^7, `z` in metres above home.
#[derive(Debug, Clone, Copy)]
pub struct MissionPoint {
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub z: f32,
}

/// `z` is compared as 4 bytes (the contract's rule), so a NaN equals itself and 0.0 differs from -0.0.
impl PartialEq for MissionPoint {
    fn eq(&self, other: &Self) -> bool {
        self.lat_e7 == other.lat_e7
            && self.lon_e7 == other.lon_e7
            && self.z.to_bits() == other.z.to_bits()
    }
}

impl Eq for MissionPoint {}

/// Degrees to the wire's degrees x 10^7: computed in f64, rounded to the nearest integer, halves away
/// from zero. `None` when the result is not a number or does not fit an `i32` (nothing is sent for it).
pub fn degrees_to_e7(degrees: f64) -> Option<i32> {
    let scaled = (degrees * 1e7).round();
    (scaled.is_finite() && scaled >= f64::from(i32::MIN) && scaled <= f64::from(i32::MAX))
        .then_some(scaled as i32)
}

/// Why the station itself refuses a waypoint before anything is sent: the vehicle's ranges of `x` and `y` (both
/// ends included) and a finite `z` (section 17, "What the station sends").
fn point_problem(point: &MissionPoint) -> Option<&'static str> {
    if !(-900_000_000..=900_000_000).contains(&point.lat_e7) {
        Some("latitude out of range")
    } else if !(-1_800_000_000..=1_800_000_000).contains(&point.lon_e7) {
        Some("longitude out of range")
    } else if !point.z.is_finite() {
        Some("altitude is not finite")
    } else {
        None
    }
}

/// The IEEE CRC-32: reflected polynomial 0xEDB88320, initial value and final XOR 0xFFFFFFFF.
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = u32::MAX;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// The plan id of a list: the CRC-32 over `x`, `y`, `z` of each item, little-endian, 12 bytes an item.
/// 0 for the empty list; a non-empty list whose CRC is 0 has the id 1.
pub fn plan_id(points: &[MissionPoint]) -> u32 {
    if points.is_empty() {
        return 0;
    }
    let mut bytes = Vec::with_capacity(points.len() * 12);
    for point in points {
        bytes.extend_from_slice(&point.lat_e7.to_le_bytes());
        bytes.extend_from_slice(&point.lon_e7.to_le_bytes());
        bytes.extend_from_slice(&point.z.to_bits().to_le_bytes());
    }
    match crc32(&bytes) {
        0 => 1,
        id => id,
    }
}

/// What a `MAV_MISSION_RESULT` means in this vehicle (the contract's table of results).
pub fn result_meaning(result: MavMissionResult) -> &'static str {
    match result {
        MavMissionResult::MAV_MISSION_ACCEPTED => "accepted",
        MavMissionResult::MAV_MISSION_ERROR => "the vehicle is armed, or no transfer was running",
        MavMissionResult::MAV_MISSION_UNSUPPORTED_FRAME => {
            "the vehicle takes only the frames 3 and 6"
        }
        MavMissionResult::MAV_MISSION_UNSUPPORTED => {
            "the vehicle takes only mission type 0 and the command 16"
        }
        MavMissionResult::MAV_MISSION_NO_SPACE => "the vehicle holds at most 16 waypoints",
        MavMissionResult::MAV_MISSION_INVALID_PARAM5_X => "latitude out of range",
        MavMissionResult::MAV_MISSION_INVALID_PARAM6_Y => "longitude out of range",
        MavMissionResult::MAV_MISSION_INVALID_SEQUENCE => {
            "a waypoint at or above the count was asked for"
        }
        MavMissionResult::MAV_MISSION_DENIED => "another owner's transfer is running",
        MavMissionResult::MAV_MISSION_OPERATION_CANCELLED => {
            "the vehicle gave up after its retries (timeout)"
        }
        _ => "a result this vehicle does not give",
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionKind {
    Upload,
    Download,
    Clear,
}

/// Which part of an operation failed: the upload, clear or download itself, or the read-back after it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionPhase {
    Transfer,
    ReadBack,
}

/// Why an operation is not done.
#[derive(Debug, Clone, PartialEq)]
pub enum MissionFailure {
    /// Not started: no vehicle is connected.
    NoVehicle,
    /// Not started: the list has more waypoints than a `MISSION_COUNT` can say.
    CountTooLarge,
    /// Not started: waypoint `seq` of the list is out of the vehicle's ranges or has a non-finite altitude; nothing
    /// was sent and no read-back is made.
    InvalidWaypoint { seq: u16, problem: &'static str },
    /// The vehicle's MISSION_ACK with this `type`.
    Refused(MavMissionResult),
    /// Timed out: no answer after every send ("no answer"). The station sent MISSION_ACK CANCELLED.
    TimedOut,
    /// The link closed.
    LinkLost,
    /// The vehicle asked for waypoint `seq`, which is not in the list; the station sent INVALID_SEQUENCE.
    InvalidSequence { seq: u16 },
    /// The read-back holds `found` waypoints, `sent` were sent.
    CountDiffers { sent: u16, found: u16 },
    /// The first waypoint of the read-back that differs from the one sent (`frame` 3 and `command` 16
    /// included).
    ItemDiffers { seq: u16 },
    /// The read-back's plan id is not the one computed from what was sent.
    PlanIdDiffers { sent: u32, found: u32 },
}

impl fmt::Display for MissionFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MissionFailure::NoVehicle => f.write_str("the vehicle is not connected"),
            MissionFailure::CountTooLarge => f.write_str("too many waypoints to send"),
            MissionFailure::InvalidWaypoint { seq, problem } => {
                write!(f, "waypoint {seq}: {problem}; nothing was sent")
            }
            MissionFailure::Refused(result) => {
                write!(f, "refused: {}", result_meaning(*result))
            }
            MissionFailure::TimedOut => f.write_str("no answer from the vehicle"),
            MissionFailure::LinkLost => f.write_str("link lost"),
            MissionFailure::InvalidSequence { seq } => {
                write!(
                    f,
                    "the vehicle asked for waypoint {seq}, which is not in the list"
                )
            }
            MissionFailure::CountDiffers { sent, found } => {
                write!(f, "the vehicle holds {found} waypoints, {sent} were sent")
            }
            MissionFailure::ItemDiffers { seq } => {
                write!(f, "waypoint {seq} read back differs from the one sent")
            }
            MissionFailure::PlanIdDiffers { sent, found } => {
                write!(f, "plan id {found:#010X} read back, {sent:#010X} sent")
            }
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub enum MissionStatus {
    /// Nothing asked yet.
    #[default]
    Idle,
    /// Upload: `sent` waypoints handed over so far (the highest asked for + 1), of `total`.
    Uploading {
        sent: u16,
        total: u16,
    },
    Clearing,
    /// A download asked for by itself; `total` is the vehicle's count once it said it.
    Downloading {
        received: u16,
        total: Option<u16>,
    },
    /// The read-back after an upload or a clear.
    ReadingBack {
        received: u16,
        total: Option<u16>,
    },
    /// An upload or a clear is done: accepted, and the read-back holds what was sent.
    Verified,
    /// A download asked for by itself is complete: `vehicle` has the list.
    Complete,
    Failed {
        reason: MissionFailure,
        phase: MissionPhase,
    },
}

/// The vehicle's list as read back (or downloaded): the waypoints and the plan id of its MISSION_COUNT.
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleList {
    pub plan_id: u32,
    pub items: Vec<MissionPoint>,
}

/// What the window shows of this service: the current or last operation.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MissionsState {
    pub status: MissionStatus,
    /// What the current or last operation was.
    pub kind: Option<MissionKind>,
    /// The list an upload sent (empty for the other operations).
    pub sent: Vec<MissionPoint>,
    /// What the vehicle holds, once a read-back or a download has it complete.
    pub vehicle: Option<VehicleList>,
    pub started: Option<Duration>,
    /// Session time at which it ended; `None` while it runs.
    pub ended: Option<Duration>,
    /// Commands refused because an operation was running.
    pub refused: u32,
    /// Operations started (a failed start counts): a caller that sent a command waits for the counter
    /// to move before it believes the status.
    pub runs: u32,
}

/// A message that is sent again when it has no response.
#[derive(Debug, Clone, Copy)]
enum Retry {
    Count,
    ClearAll,
    LastItem,
    RequestList,
    RequestInt(u16),
}

#[derive(Debug, Clone, Copy)]
enum Step {
    /// `what` was sent `sends` times, the last time at `since`.
    Await {
        what: Retry,
        sends: u8,
        since: Duration,
    },
    /// An item other than the last went out at `since`; the vehicle asks for the next.
    ItemWait { since: Duration },
}

#[derive(Default)]
struct Download {
    count: Option<u16>,
    plan_id: u32,
    items: Vec<MissionPoint>,
    /// The first received item whose `frame` is not 3 or whose `command` is not 16.
    odd: Option<u16>,
}

struct Run {
    target: (u8, u8),
    kind: MissionKind,
    plan: Vec<MissionPoint>,
    plan_id: u32,
    count: u16,
    /// Item n - 1 has been sent (for n = 0 and for a clear: the start message has).
    last_sent: bool,
    sent_items: u16,
    step: Step,
    /// The read-back after an upload or a clear is running.
    read_back: bool,
    /// How the upload or the clear ended, once the read-back runs.
    transfer: Option<Result<(), MissionFailure>>,
    down: Download,
}

impl Run {
    fn downloading(&self) -> bool {
        self.kind == MissionKind::Download || self.read_back
    }

    fn message(&self, what: Retry) -> Message {
        let (system, component) = self.target;
        match what {
            Retry::Count => Message::MISSION_COUNT(MISSION_COUNT_DATA {
                count: self.count,
                target_system: system,
                target_component: component,
                mission_type: PLAN,
                opaque_id: 0,
            }),
            Retry::ClearAll => Message::MISSION_CLEAR_ALL(MISSION_CLEAR_ALL_DATA {
                target_system: system,
                target_component: component,
                mission_type: PLAN,
            }),
            Retry::LastItem => self.item(self.count - 1),
            Retry::RequestList => Message::MISSION_REQUEST_LIST(MISSION_REQUEST_LIST_DATA {
                target_system: system,
                target_component: component,
                mission_type: PLAN,
            }),
            Retry::RequestInt(seq) => Message::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
                seq,
                target_system: system,
                target_component: component,
                mission_type: PLAN,
            }),
        }
    }

    fn item(&self, seq: u16) -> Message {
        let point = self.plan[usize::from(seq)];
        Message::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: point.lat_e7,
            y: point.lon_e7,
            z: point.z,
            seq,
            command: WAYPOINT,
            target_system: self.target.0,
            target_component: self.target.1,
            frame: FRAME,
            current: 0,
            autocontinue: 1,
            mission_type: PLAN,
        })
    }

    fn ack(&self, result: MavMissionResult) -> Message {
        Message::MISSION_ACK(MISSION_ACK_DATA {
            target_system: self.target.0,
            target_component: self.target.1,
            mavtype: result,
            mission_type: PLAN,
            opaque_id: 0,
        })
    }

    fn download_status(&self) -> MissionStatus {
        let received = u16::try_from(self.down.items.len()).unwrap_or(u16::MAX);
        let total = self.down.count;
        if self.read_back {
            MissionStatus::ReadingBack { received, total }
        } else {
            MissionStatus::Downloading { received, total }
        }
    }
}

pub struct Missions {
    system_id: u8,
    component_id: u8,
    run: Option<Run>,
}

impl Missions {
    pub fn new(config: &SessionConfig) -> Self {
        Self {
            system_id: config.system_id,
            component_id: config.component_id,
            run: None,
        }
    }

    fn target(cx: &Ctx) -> Option<(u8, u8)> {
        let vehicle = cx.state.link.vehicle.as_ref()?;
        (cx.state.link.status == LinkStatus::Connected)
            .then_some((vehicle.system_id, vehicle.component_id))
    }

    fn begin(&mut self, kind: MissionKind, plan: Vec<MissionPoint>, cx: &mut Ctx) {
        cx.changed = true;
        if self.run.is_some() {
            cx.state.missions.refused += 1;
            return;
        }
        let state = &mut cx.state.missions;
        state.runs += 1;
        state.kind = Some(kind);
        state.sent = plan.clone();
        state.vehicle = None;
        state.started = Some(cx.now);
        state.ended = None;
        let target = Self::target(cx);
        let count = u16::try_from(plan.len());
        let invalid = plan
            .iter()
            .enumerate()
            .find_map(|(at, point)| Some((at, point_problem(point)?)));
        let (Some(target), Ok(count), None) = (target, count, invalid) else {
            let reason = if target.is_none() {
                MissionFailure::NoVehicle
            } else if count.is_err() {
                MissionFailure::CountTooLarge
            } else {
                let (at, problem) = invalid.unwrap_or_default();
                MissionFailure::InvalidWaypoint {
                    seq: u16::try_from(at).unwrap_or(u16::MAX),
                    problem,
                }
            };
            cx.state.missions.status = MissionStatus::Failed {
                reason,
                phase: MissionPhase::Transfer,
            };
            cx.state.missions.ended = Some(cx.now);
            return;
        };
        let (what, status) = match kind {
            MissionKind::Upload => (
                Retry::Count,
                MissionStatus::Uploading {
                    sent: 0,
                    total: count,
                },
            ),
            MissionKind::Clear => (Retry::ClearAll, MissionStatus::Clearing),
            MissionKind::Download => (
                Retry::RequestList,
                MissionStatus::Downloading {
                    received: 0,
                    total: None,
                },
            ),
        };
        let run = Run {
            target,
            kind,
            plan_id: plan_id(&plan),
            plan,
            count,
            last_sent: kind == MissionKind::Clear || count == 0,
            sent_items: 0,
            step: Step::Await {
                what,
                sends: 1,
                since: cx.now,
            },
            read_back: false,
            transfer: None,
            down: Download::default(),
        };
        cx.out.push(run.message(what));
        cx.state.missions.status = status;
        self.run = Some(run);
    }

    /// Ends the operation with `status`.
    fn end(&mut self, status: MissionStatus, cx: &mut Ctx) {
        self.run = None;
        cx.state.missions.status = status;
        cx.state.missions.ended = Some(cx.now);
        cx.changed = true;
    }

    /// The upload or the clear is over, in any way but a lost link: the read-back starts at once.
    fn finish_transfer(&mut self, result: Result<(), MissionFailure>, cx: &mut Ctx) {
        let Some(run) = self.run.as_mut() else {
            return;
        };
        run.transfer = Some(result);
        run.read_back = true;
        run.down = Download::default();
        run.step = Step::Await {
            what: Retry::RequestList,
            sends: 1,
            since: cx.now,
        };
        cx.out.push(run.message(Retry::RequestList));
        cx.state.missions.status = run.download_status();
        cx.changed = true;
    }

    /// The download (or the read-back) ended with the whole list.
    fn finish_download(&mut self, cx: &mut Ctx) {
        let Some(run) = self.run.as_mut() else {
            return;
        };
        let list = VehicleList {
            plan_id: run.down.plan_id,
            items: std::mem::take(&mut run.down.items),
        };
        let status = if !run.read_back {
            MissionStatus::Complete
        } else if let Some(Err(reason)) = &run.transfer {
            MissionStatus::Failed {
                reason: reason.clone(),
                phase: MissionPhase::Transfer,
            }
        } else {
            match Self::compare(run, &list) {
                None => MissionStatus::Verified,
                Some(reason) => MissionStatus::Failed {
                    reason,
                    phase: MissionPhase::ReadBack,
                },
            }
        };
        cx.state.missions.vehicle = Some(list);
        self.end(status, cx);
    }

    /// Why the read-back is not what was sent; `None` when it is.
    fn compare(run: &Run, list: &VehicleList) -> Option<MissionFailure> {
        let found = u16::try_from(list.items.len()).unwrap_or(u16::MAX);
        if found != run.count {
            return Some(MissionFailure::CountDiffers {
                sent: run.count,
                found,
            });
        }
        let first = (0..run.count).find(|&seq| {
            let at = usize::from(seq);
            list.items[at] != run.plan[at] || run.down.odd == Some(seq)
        });
        if let Some(seq) = first {
            return Some(MissionFailure::ItemDiffers { seq });
        }
        (list.plan_id != run.plan_id).then_some(MissionFailure::PlanIdDiffers {
            sent: run.plan_id,
            found: list.plan_id,
        })
    }

    /// The download (or the read-back) failed.
    fn fail_download(&mut self, reason: MissionFailure, cx: &mut Ctx) {
        let Some(run) = self.run.as_ref() else {
            return;
        };
        let status = match &run.transfer {
            Some(Err(first)) => MissionStatus::Failed {
                reason: first.clone(),
                phase: MissionPhase::Transfer,
            },
            _ => MissionStatus::Failed {
                reason,
                phase: if run.read_back {
                    MissionPhase::ReadBack
                } else {
                    MissionPhase::Transfer
                },
            },
        };
        self.end(status, cx);
    }

    /// Every send went unanswered: tells the vehicle, then ends the operation as timed out.
    fn give_up(&mut self, cx: &mut Ctx) {
        let Some(run) = self.run.as_ref() else {
            return;
        };
        cx.out
            .push(run.ack(MavMissionResult::MAV_MISSION_OPERATION_CANCELLED));
        if run.downloading() {
            self.fail_download(MissionFailure::TimedOut, cx);
        } else {
            self.finish_transfer(Err(MissionFailure::TimedOut), cx);
        }
    }

    fn on_request(&mut self, seq: u16, cx: &mut Ctx) {
        let Some(run) = self.run.as_mut() else {
            return;
        };
        if run.kind != MissionKind::Upload || run.read_back {
            return;
        }
        if seq >= run.count {
            cx.out
                .push(run.ack(MavMissionResult::MAV_MISSION_INVALID_SEQUENCE));
            self.finish_transfer(Err(MissionFailure::InvalidSequence { seq }), cx);
            return;
        }
        cx.out.push(run.item(seq));
        run.sent_items = run.sent_items.max(seq + 1);
        run.step = if seq + 1 == run.count {
            run.last_sent = true;
            Step::Await {
                what: Retry::LastItem,
                sends: 1,
                since: cx.now,
            }
        } else {
            Step::ItemWait { since: cx.now }
        };
        cx.state.missions.status = MissionStatus::Uploading {
            sent: run.sent_items,
            total: run.count,
        };
        cx.changed = true;
    }

    fn on_ack(&mut self, ack: &MISSION_ACK_DATA, cx: &mut Ctx) {
        let Some(run) = self.run.as_ref() else {
            return;
        };
        let result = ack.mavtype;
        let accepted = result == MavMissionResult::MAV_MISSION_ACCEPTED;
        if run.downloading() {
            if !accepted {
                self.fail_download(MissionFailure::Refused(result), cx);
            }
        } else if !accepted {
            self.finish_transfer(Err(MissionFailure::Refused(result)), cx);
        } else if run.last_sent && ack.opaque_id == run.plan_id {
            self.finish_transfer(Ok(()), cx);
        }
    }

    fn on_count(&mut self, message: &MISSION_COUNT_DATA, cx: &mut Ctx) {
        let Some(run) = self.run.as_mut() else {
            return;
        };
        if !run.downloading()
            || !matches!(
                run.step,
                Step::Await {
                    what: Retry::RequestList,
                    ..
                }
            )
        {
            return;
        }
        run.down.count = Some(message.count);
        run.down.plan_id = message.opaque_id;
        if message.count == 0 {
            cx.out.push(run.ack(MavMissionResult::MAV_MISSION_ACCEPTED));
            self.finish_download(cx);
            return;
        }
        run.step = Step::Await {
            what: Retry::RequestInt(0),
            sends: 1,
            since: cx.now,
        };
        cx.out.push(run.message(Retry::RequestInt(0)));
        cx.state.missions.status = run.download_status();
        cx.changed = true;
    }

    fn on_item(&mut self, item: &MISSION_ITEM_INT_DATA, cx: &mut Ctx) {
        let Some(run) = self.run.as_mut() else {
            return;
        };
        let Step::Await {
            what: Retry::RequestInt(asked),
            sends,
            ..
        } = run.step
        else {
            return;
        };
        if !run.downloading() {
            return;
        }
        if item.seq != asked {
            // Dropped; the item asked for is asked for again: not a retry, its timer restarts.
            run.step = Step::Await {
                what: Retry::RequestInt(asked),
                sends,
                since: cx.now,
            };
            cx.out.push(run.message(Retry::RequestInt(asked)));
            return;
        }
        if (item.frame != FRAME || item.command != WAYPOINT) && run.down.odd.is_none() {
            run.down.odd = Some(asked);
        }
        run.down.items.push(MissionPoint {
            lat_e7: item.x,
            lon_e7: item.y,
            z: item.z,
        });
        cx.changed = true;
        if Some(asked + 1) == run.down.count {
            cx.out.push(run.ack(MavMissionResult::MAV_MISSION_ACCEPTED));
            self.finish_download(cx);
            return;
        }
        run.step = Step::Await {
            what: Retry::RequestInt(asked + 1),
            sends: 1,
            since: cx.now,
        };
        cx.out.push(run.message(Retry::RequestInt(asked + 1)));
        cx.state.missions.status = run.download_status();
    }
}

impl Service for Missions {
    fn on_frame(&mut self, frame: &Frame, cx: &mut Ctx) {
        let Some(run) = self.run.as_ref() else {
            return;
        };
        let Body::Known(message) = &frame.body else {
            return;
        };
        if (frame.system_id, frame.component_id) != run.target {
            return;
        }
        let (system, component, plan) = match message {
            Message::MISSION_COUNT(m) => (m.target_system, m.target_component, m.mission_type),
            Message::MISSION_ACK(m) => (m.target_system, m.target_component, m.mission_type),
            Message::MISSION_REQUEST_INT(m) => {
                (m.target_system, m.target_component, m.mission_type)
            }
            Message::MISSION_ITEM_INT(m) => (m.target_system, m.target_component, m.mission_type),
            _ => return,
        };
        let ours = (system == 0 || system == self.system_id)
            && (component == 0 || component == self.component_id);
        if !ours || plan != PLAN {
            return;
        }
        match message {
            Message::MISSION_COUNT(m) => self.on_count(m, cx),
            Message::MISSION_ACK(m) => self.on_ack(m, cx),
            Message::MISSION_REQUEST_INT(m) => self.on_request(m.seq, cx),
            Message::MISSION_ITEM_INT(m) => self.on_item(m, cx),
            _ => {}
        }
    }

    fn on_tick(&mut self, cx: &mut Ctx) {
        let Some(run) = self.run.as_mut() else {
            return;
        };
        match run.step {
            Step::Await { what, sends, since } => {
                if cx.now.saturating_sub(since) < RETRY_AFTER {
                    return;
                }
                if sends <= RETRIES {
                    run.step = Step::Await {
                        what,
                        sends: sends + 1,
                        since: cx.now,
                    };
                    cx.out.push(run.message(what));
                    cx.changed = true;
                } else {
                    self.give_up(cx);
                }
            }
            Step::ItemWait { since } => {
                if cx.now.saturating_sub(since) >= ITEM_WAIT {
                    self.give_up(cx);
                }
            }
        }
    }

    fn on_close(&mut self, cx: &mut Ctx) {
        let Some(run) = self.run.as_ref() else {
            return;
        };
        let phase = if run.read_back {
            MissionPhase::ReadBack
        } else {
            MissionPhase::Transfer
        };
        self.end(
            MissionStatus::Failed {
                reason: MissionFailure::LinkLost,
                phase,
            },
            cx,
        );
    }

    fn on_command(&mut self, command: &Command, cx: &mut Ctx) {
        match command {
            Command::UploadMission { items } => self.begin(MissionKind::Upload, items.clone(), cx),
            Command::ClearMission => self.begin(MissionKind::Clear, Vec::new(), cx),
            Command::DownloadMission => self.begin(MissionKind::Download, Vec::new(), cx),
            _ => {}
        }
    }
}

#[cfg(test)]
#[path = "missions_tests.rs"]
mod tests;
