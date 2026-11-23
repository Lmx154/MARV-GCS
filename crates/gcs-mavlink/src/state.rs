//! What the window shows: one section per service.

use std::time::Duration;

use crate::RxStats;
use crate::dialect::MavCmd;
use crate::services::logs::LogsState;
use crate::services::messages::MessageStore;
use crate::services::missions::{MissionPoint, MissionsState};
use crate::services::params::{ParamTable, ParamValue};
use crate::services::texts::TextLog;
use crate::tlog::{LogSink, LogState, SessionLogState};

/// Everything the window shows. One section per service; a new service adds a section.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct VehicleState {
    pub link: LinkState,
    pub messages: MessageStore,
    pub params: ParamTable,
    pub texts: TextLog,
    pub log: LogState,
    /// The recording of a monitoring session, written beside the automatic one.
    pub session_log: SessionLogState,
    /// The vehicle's logs: the last listing and the current or last download.
    pub logs: LogsState,
    /// The mission service: the current or last upload, download or clear.
    pub missions: MissionsState,
    /// The current or last MAVLink command the user sent; `None` before the first.
    pub command: Option<CommandRecord>,
    /// The calibrations the vehicle applies in RAM and has not saved to flash.
    pub calibration: UnsavedCalibration,
}

/// The calibrations that ended accepted and are not yet saved (`MAV_CMD_PREFLIGHT_STORAGE` param1 = 1)
/// or dropped (param1 = 0). The vehicle applies the offsets at once; a reboot loses them.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct UnsavedCalibration {
    pub gyro: bool,
    pub accel: bool,
    pub compass: bool,
    /// The vehicle restarted while some were unsaved: they are gone. Kept until the next calibration or storage.
    pub lost_at_reboot: bool,
}

impl UnsavedCalibration {
    /// Whether any calibration waits to be saved.
    pub fn any(&self) -> bool {
        self.gyro || self.accel || self.compass
    }

    /// The parameters a calibration writes, which the vehicle does not report by itself.
    pub fn offsets(gyro: bool, accel: bool, compass: bool) -> Vec<&'static str> {
        let mut names = Vec::new();
        if gyro {
            names.extend([
                "IMU1_GYR_OFFX",
                "IMU1_GYR_OFFY",
                "IMU1_GYR_OFFZ",
                "IMU2_GYR_OFFX",
                "IMU2_GYR_OFFY",
                "IMU2_GYR_OFFZ",
            ]);
        }
        if accel {
            names.extend([
                "IMU1_ACC_OFFX",
                "IMU1_ACC_OFFY",
                "IMU1_ACC_OFFZ",
                "IMU2_ACC_OFFX",
                "IMU2_ACC_OFFY",
                "IMU2_ACC_OFFZ",
            ]);
        }
        if compass {
            names.extend(["MAG_OFFX", "MAG_OFFY", "MAG_OFFZ"]);
        }
        names
    }
}

/// Where a command stands (docs/link-contract.md, section 10).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CommandStatus {
    /// Sent, no ACK yet.
    Waiting,
    /// The vehicle said IN_PROGRESS; `percent` is `None` when it gave no percentage (255).
    InProgress { percent: Option<u8> },
    /// The vehicle's final ACK. `text` is the result in words, `accepted` whether it is ACCEPTED.
    Finished { text: &'static str, accepted: bool },
    /// Every send went unanswered.
    NoAnswer,
    /// IN_PROGRESS was seen, then the vehicle stopped reporting.
    NoResult,
    /// Not sent; the text says why.
    Refused(String),
}

/// The state of a cancel sent while a calibration runs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CancelStatus {
    Waiting,
    /// The vehicle's answer to the cancel, in words.
    Answered(&'static str),
    NoAnswer,
}

/// A `SendCommand` that was refused because another command was outstanding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RefusedCommand {
    /// The `MAV_CMD` number of the last refused command.
    pub command: u32,
    /// How many were refused while this command was outstanding.
    pub count: u32,
}

/// The current or last command: what was asked, how often it was sent, how it stands.
#[derive(Debug, Clone, PartialEq)]
pub struct CommandRecord {
    /// The `MAV_CMD` number.
    pub command: u32,
    /// The dialect's name for it.
    pub name: Option<String>,
    pub params: [f32; 7],
    pub status: CommandStatus,
    /// COMMAND_LONG frames sent for it (not counting a cancel's).
    pub sends: u8,
    /// Session time of the first send.
    pub started: Duration,
    /// Session time at which it ended; `None` while it is outstanding.
    pub ended: Option<Duration>,
    /// A `SendCommand` refused while this one was outstanding.
    pub refused: Option<RefusedCommand>,
    /// A cancel sent while this calibration was outstanding.
    pub cancel: Option<CancelStatus>,
}

impl CommandRecord {
    /// Whether a new command has to wait for this one.
    pub fn outstanding(&self) -> bool {
        matches!(
            self.status,
            CommandStatus::Waiting | CommandStatus::InProgress { .. }
        )
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum LinkStatus {
    /// Link open, no vehicle heartbeat yet.
    #[default]
    Connecting,
    /// Vehicle heartbeat seen within the timeout.
    Connected,
    /// Link open, heartbeats stopped.
    Lost,
    /// Transport is down; the text says why.
    Closed(String),
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct LinkState {
    pub status: LinkStatus,
    /// From the vehicle's HEARTBEAT.
    pub vehicle: Option<VehicleId>,
    /// Session time of the last vehicle heartbeat.
    pub last_heartbeat: Option<Duration>,
    pub rx: RxStats,
    pub tx_frames: u64,
    /// The last command the session refused for its access, and how many were refused in all.
    pub refused: Option<Refused>,
}

/// A command that `Session::send` refused because the session is operate-only.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Refused {
    /// `Command::name` of the last refused command.
    pub command: &'static str,
    pub count: u64,
}

impl Refused {
    pub const TEXT: &'static str = "refused: this link is operate-only";
}

/// What a session may do. Fixed when it starts; the application derives it from the link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    /// A telemetry display and controller: reads, and what only concerns the station.
    Operate,
    /// Also the flight controller's configurator: everything `Operate` does, and the writes.
    Configure,
}

impl Access {
    /// Whether a session with this access may run a command that needs `needed`.
    pub fn allows(self, needed: Access) -> bool {
        match (self, needed) {
            (Access::Configure, _) | (Access::Operate, Access::Operate) => true,
            (Access::Operate, Access::Configure) => false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VehicleId {
    pub system_id: u8,
    pub component_id: u8,
    pub mav_type: String,
    pub autopilot: String,
}

/// What the window can ask for. One variant per user action; grows with the features.
#[derive(Debug)]
pub enum Command {
    /// Fetch the whole parameter list again.
    RefreshParams,
    /// Write one parameter. The outcome shows in `params.writes`.
    SetParam {
        name: String,
        value: ParamValue,
    },
    /// Record every frame, in both directions, to this sink. The window opens the file.
    StartLog {
        sink: LogSink,
        label: String,
    },
    StopLog,
    /// Record every frame, in both directions, to this sink as well: the recording of a monitoring session,
    /// independent of `StartLog` (the automatic recording goes on). A running one is closed first.
    StartSessionLog {
        sink: LogSink,
        label: String,
    },
    /// Close the session recording; `session_log.closed` then names it.
    StopSessionLog,
    /// Send one MAVLink command to the vehicle (`COMMAND_LONG`, docs/link-contract.md section 10).
    /// The outcome shows in `VehicleState.command`.
    SendCommand {
        command: MavCmd,
        params: [f32; 7],
    },
    /// List the logs on the vehicle's card (docs/link-contract.md section 11). The outcome shows in
    /// `VehicleState.logs.list`.
    ListLogs,
    /// Download log `number`, of `size` bytes as the listing said, to `sink`. The outcome and the progress
    /// show in `VehicleState.logs.download`.
    DownloadLog {
        number: u16,
        size: u32,
        sink: LogSink,
    },
    /// Abandon the running download and tell the vehicle to stop sending.
    CancelLogDownload,
    /// Send this waypoint list to the vehicle and read it back (docs/link-contract.md section 17). An empty
    /// list clears. The outcome shows in `VehicleState.missions`.
    UploadMission {
        items: Vec<MissionPoint>,
    },
    /// Read the vehicle's list. The outcome shows in `VehicleState.missions`.
    DownloadMission,
    /// Empty the vehicle's list and read it back. The outcome shows in `VehicleState.missions`.
    ClearMission,
}

impl Command {
    /// The access a session needs to run this command: `Configure` for what writes to the vehicle,
    /// `Operate` for what only reads or only concerns the station. One match with no wildcard arm: a
    /// new variant does not compile until it has a class (docs/link-contract.md, "Access").
    pub fn access(&self) -> Access {
        match self {
            Command::RefreshParams => Access::Operate,
            Command::SetParam { .. } => Access::Configure,
            Command::StartLog { .. } => Access::Operate,
            Command::StopLog => Access::Operate,
            Command::StartSessionLog { .. } => Access::Operate,
            Command::StopSessionLog => Access::Operate,
            // Every command id: an unknown one counts as Configure too.
            Command::SendCommand { .. } => Access::Configure,
            // The vehicle offers log download on its USB link only (section 11).
            Command::ListLogs => Access::Configure,
            Command::DownloadLog { .. } => Access::Configure,
            Command::CancelLogDownload => Access::Configure,
            // A mission write changes no setting of the vehicle (section 9, the mission amendment).
            Command::UploadMission { .. } => Access::Operate,
            Command::DownloadMission => Access::Operate,
            Command::ClearMission => Access::Operate,
        }
    }

    /// A short name for the record of a refusal. Also without a wildcard arm.
    pub fn name(&self) -> &'static str {
        match self {
            Command::RefreshParams => "RefreshParams",
            Command::SetParam { .. } => "SetParam",
            Command::StartLog { .. } => "StartLog",
            Command::StopLog => "StopLog",
            Command::StartSessionLog { .. } => "StartSessionLog",
            Command::StopSessionLog => "StopSessionLog",
            Command::SendCommand { .. } => "SendCommand",
            Command::ListLogs => "ListLogs",
            Command::DownloadLog { .. } => "DownloadLog",
            Command::CancelLogDownload => "CancelLogDownload",
            Command::UploadMission { .. } => "UploadMission",
            Command::DownloadMission => "DownloadMission",
            Command::ClearMission => "ClearMission",
        }
    }
}
