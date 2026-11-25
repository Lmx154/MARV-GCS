//! The five flight commands and the vehicle's reported mode (docs/link-contract.md, section 10: "Flight
//! commands and the effective mode", Amendment C). Pure data: no command here arms, disarms or stops the motors.

use std::time::Duration;

use crate::dialect::{MavCmd, MavResult};

/// The vehicle's HEARTBEAT `custom_mode` of PILOT, TAKEOVER, HOLD, AUTO, RETURN, LAND, STOP and BLIND.
const HOLD: u32 = 3;
const LAND: u32 = 6;
/// `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`, param1 of `MAV_CMD_DO_SET_MODE`.
const CUSTOM_MODE_ENABLED: f32 = 1.0;

/// The station's flight buttons.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightCommand {
    Start,
    Pause,
    Abort,
    Land,
    Return,
}

impl FlightCommand {
    pub const ALL: [FlightCommand; 5] = [
        FlightCommand::Start,
        FlightCommand::Pause,
        FlightCommand::Abort,
        FlightCommand::Land,
        FlightCommand::Return,
    ];

    /// The button's name.
    pub fn label(self) -> &'static str {
        match self {
            FlightCommand::Start => "Start",
            FlightCommand::Pause => "Pause",
            FlightCommand::Abort => "Abort",
            FlightCommand::Land => "Land",
            FlightCommand::Return => "Return",
        }
    }

    /// The `COMMAND_LONG` command and its seven parameters; the contract's table.
    pub fn wire(self) -> (MavCmd, [f32; 7]) {
        let mode = |custom: f32| [CUSTOM_MODE_ENABLED, custom, 0.0, 0.0, 0.0, 0.0, 0.0];
        match self {
            FlightCommand::Start => (MavCmd::MAV_CMD_MISSION_START, [0.0; 7]),
            FlightCommand::Pause => (MavCmd::MAV_CMD_DO_PAUSE_CONTINUE, [0.0; 7]),
            FlightCommand::Abort => (MavCmd::MAV_CMD_DO_SET_MODE, mode(HOLD as f32)),
            FlightCommand::Land => (MavCmd::MAV_CMD_DO_SET_MODE, mode(LAND as f32)),
            FlightCommand::Return => (MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7]),
        }
    }

    /// The flight command a sent `COMMAND_LONG` is, if it is exactly one of the five.
    pub fn from_wire(command: MavCmd, params: &[f32; 7]) -> Option<FlightCommand> {
        FlightCommand::ALL
            .into_iter()
            .find(|f| f.wire() == (command, *params))
    }
}

/// How a flight command ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightOutcome {
    Accepted,
    /// DENIED with the reason code of `COMMAND_ACK.result_param2`.
    Denied {
        code: i32,
    },
    /// Any other final result, in words.
    Other(&'static str),
    /// Every send went unanswered: never read as accepted.
    NoAnswer,
    /// Not sent: the vehicle is not connected.
    NotSent,
}

/// The last flight command and how it ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FlightResult {
    pub command: FlightCommand,
    pub outcome: FlightOutcome,
}

impl FlightResult {
    /// What the vehicle answered, in words without the button's name; `None` when it was accepted.
    pub fn reason(&self) -> Option<String> {
        match self.outcome {
            FlightOutcome::Accepted | FlightOutcome::NoAnswer => None,
            FlightOutcome::Denied { code: 15 } if self.command == FlightCommand::Abort => {
                Some(format!("{}: the landing continues", deny_text(15)))
            }
            FlightOutcome::Denied { code } => Some(deny_text(code)),
            FlightOutcome::Other(text) => Some(text.to_string()),
            FlightOutcome::NotSent => Some("the vehicle is not connected".to_string()),
        }
    }

    /// The error to show, or `None` when it was accepted.
    pub fn error(&self) -> Option<String> {
        let button = self.command.label();
        match self.outcome {
            FlightOutcome::Accepted => None,
            FlightOutcome::Denied { code: 15 } if self.command == FlightCommand::Abort => Some(
                format!("{}: the landing continues: {button} refused", deny_text(15)),
            ),
            FlightOutcome::Denied { code } => {
                Some(format!("{}: {button} refused", deny_text(code)))
            }
            FlightOutcome::Other(text) => Some(format!("{button} not accepted: {text}")),
            FlightOutcome::NoAnswer => Some(format!("{button}: no answer from the vehicle")),
            FlightOutcome::NotSent => Some(format!("{button}: the vehicle is not connected")),
        }
    }
}

/// The outcome of a final `COMMAND_ACK` result.
pub fn outcome_of(result: MavResult, result_param2: i32, text: &'static str) -> FlightOutcome {
    match result {
        MavResult::MAV_RESULT_ACCEPTED => FlightOutcome::Accepted,
        MavResult::MAV_RESULT_DENIED => FlightOutcome::Denied {
            code: result_param2,
        },
        _ => FlightOutcome::Other(text),
    }
}

/// The text of a deny reason code (the contract's table, "The text the station shows").
pub fn deny_text(code: i32) -> String {
    match code {
        0 => "Refused, no reason given",
        1 => "Not on the radio link: connect through the transmitter's WiFi backpack",
        2 => "The transmitter is on the CRSF model: missions fly on the MAVLink model only",
        3 => "Hands-off flight is off: FS_HOLD_EN is 0, or this flight was armed on the CRSF model",
        4 => "The vehicle is not in a state that takes this command",
        5 => {
            "The vehicle does not hear this station on the radio link: check the link and try again"
        }
        6 => "No waypoints are stored on the vehicle: upload a mission first",
        7 => "A stored height is outside the height limits in force: upload the mission again",
        8 => "The vehicle has no home point: it is set at arming, before take-off",
        9 => "The vehicle is more than 1 km from the take-off point",
        10 => "The distance to a waypoint is unknown: the vehicle has no valid position",
        11 => "A waypoint is more than 1 km from the take-off point",
        12 => "The vehicle's heading is not OK: wait for it to settle",
        13 => "The vehicle has lost its position estimate",
        14 => "The battery is not OK",
        15 => "The arm switch is OFF",
        16 => "The mode switch is not in Position",
        17 => "The battery is critical",
        18 => "The link to this station was lost for more than 10 s",
        19 => "The mission switch (channel 8) is not confirmed ON: no live stick report with it ON",
        20 => {
            "The vehicle holds a mission but is set to ignore it (GDN_SRC is 0): set GDN_SRC to 1 in Parameters, then Start again"
        }
        other => return format!("Refused, reason not known (code {other})"),
    }
    .to_string()
}

/// The effective mode of the vehicle, from HEARTBEAT `custom_mode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightMode {
    /// Code 0: the vehicle does not report a mode.
    NotReported,
    Pilot,
    Takeover,
    Hold,
    Auto,
    Return,
    Land,
    Stop,
    Blind,
    /// Codes 9 and above.
    Unknown(u32),
}

impl FlightMode {
    pub fn from_code(code: u32) -> FlightMode {
        match code {
            0 => FlightMode::NotReported,
            1 => FlightMode::Pilot,
            2 => FlightMode::Takeover,
            3 => FlightMode::Hold,
            4 => FlightMode::Auto,
            5 => FlightMode::Return,
            6 => FlightMode::Land,
            7 => FlightMode::Stop,
            8 => FlightMode::Blind,
            other => FlightMode::Unknown(other),
        }
    }

    /// The hands-off modes the take-over panel follows: HOLD, AUTO, RETURN, LAND, BLIND (3, 4, 5, 6, 8).
    pub fn hands_off(self) -> bool {
        matches!(
            self,
            FlightMode::Hold
                | FlightMode::Auto
                | FlightMode::Return
                | FlightMode::Land
                | FlightMode::Blind
        )
    }

    /// The mode in plain words.
    pub fn text(self) -> String {
        match self {
            FlightMode::NotReported => "Mode not reported by the vehicle".to_string(),
            FlightMode::Pilot => "Pilot flying (radio control)".to_string(),
            FlightMode::Takeover => {
                "Take-over: holding, control locked until the switches are set".to_string()
            }
            FlightMode::Hold => "Holding position".to_string(),
            FlightMode::Auto => "Flying the stored mission".to_string(),
            FlightMode::Return => "Returning to the take-off point".to_string(),
            FlightMode::Land => "Landing".to_string(),
            FlightMode::Stop => "Stopped (disarmed)".to_string(),
            FlightMode::Blind => "Descending level, no height estimate".to_string(),
            FlightMode::Unknown(code) => format!("Unknown mode ({code})"),
        }
    }
}

/// What the station knows of the vehicle's flight state.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct FlightState {
    /// The last HEARTBEAT `custom_mode` of the vehicle; kept when the heartbeat stops.
    pub mode_code: Option<u32>,
    /// The last flight command and how it ended; replaced by the next.
    pub last: Option<FlightResult>,
    /// The flight command sent and not yet answered.
    pub pending: Option<FlightCommand>,
    /// Session time at which the last flight command was sent.
    pub sent: Option<Duration>,
    /// Session time at which `last` was answered (or given up on); `None` while `pending`.
    pub answered: Option<Duration>,
}

impl FlightState {
    /// A flight command goes out: the previous result is gone.
    pub fn begin(&mut self, command: FlightCommand, now: Duration) {
        self.last = None;
        self.pending = Some(command);
        self.sent = Some(now);
        self.answered = None;
    }

    /// The flight command ended.
    pub fn finish(&mut self, result: FlightResult, now: Duration) {
        self.last = Some(result);
        self.pending = None;
        self.answered = Some(now);
    }
}

#[cfg(test)]
#[path = "flight_tests.rs"]
mod tests;
