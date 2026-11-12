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
