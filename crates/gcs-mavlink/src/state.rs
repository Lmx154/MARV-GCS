//! What the window shows: one section per service.

use std::time::Duration;

use crate::RxStats;
use crate::services::messages::MessageStore;
use crate::services::params::{ParamTable, ParamValue};
use crate::services::texts::TextLog;
use crate::tlog::{LogSink, LogState};

/// Everything the window shows. One section per service; a new service adds a section.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct VehicleState {
    pub link: LinkState,
    pub messages: MessageStore,
    pub params: ParamTable,
    pub texts: TextLog,
    pub log: LogState,
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
        }
    }

    /// A short name for the record of a refusal. Also without a wildcard arm.
    pub fn name(&self) -> &'static str {
        match self {
            Command::RefreshParams => "RefreshParams",
            Command::SetParam { .. } => "SetParam",
            Command::StartLog { .. } => "StartLog",
            Command::StopLog => "StopLog",
        }
    }
}
