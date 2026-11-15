//! The link status line: a pure function from the link state and the session time.

use std::time::Duration;

use eframe::egui::Color32;
use gcs_mavlink::{
    Access, CancelStatus, CommandRecord, CommandStatus, LinkState, LinkStatus, VehicleState,
};

pub const YELLOW: Color32 = Color32::from_rgb(230, 190, 40);
pub const GREEN: Color32 = Color32::from_rgb(70, 200, 100);
pub const RED: Color32 = Color32::from_rgb(235, 80, 80);
pub const GREY: Color32 = Color32::from_rgb(150, 150, 150);

/// The session's role as shown in the bar and the Link view.
pub fn role(access: Access) -> &'static str {
    match access {
        Access::Operate => "Operate (read only)",
        Access::Configure => "Configure",
    }
}

/// Why a configure control is disabled on an operate-only session; `None` when it is allowed.
pub fn configure_blocked(access: Access) -> Option<&'static str> {
    match access {
        Access::Operate => Some(CONFIGURE_BLOCKED),
        Access::Configure => None,
    }
}

pub const CONFIGURE_BLOCKED: &str =
    "Configure needs a serial or TCP link; this link is operate-only";

/// Why every command button is disabled while a command is outstanding.
pub const COMMAND_RUNNING: &str = "a command is running";
/// Why a command button is disabled with no session.
pub const NOT_CONNECTED: &str = "not connected";

/// Why a command button is disabled; `None` when it may be pressed. The role first, then the
/// session, then a command that is still outstanding.
pub fn command_gate(access: Access, state: &VehicleState, connected: bool) -> Option<&'static str> {
    if let Some(reason) = configure_blocked(access) {
        return Some(reason);
    }
    if !connected {
        return Some(NOT_CONNECTED);
    }
    if state
        .command
        .as_ref()
        .is_some_and(CommandRecord::outstanding)
    {
        return Some(COMMAND_RUNNING);
    }
    None
}

/// Why the cancel button is disabled; `None` exactly while a calibration is outstanding (and no
/// cancel already waits for its answer).
pub fn cancel_gate(access: Access, state: &VehicleState, connected: bool) -> Option<&'static str> {
    if let Some(reason) = configure_blocked(access) {
        return Some(reason);
    }
    if !connected {
        return Some(NOT_CONNECTED);
    }
    match &state.command {
        Some(r) if r.outstanding() && is_calibration(r) => {
            (r.cancel == Some(CancelStatus::Waiting)).then_some("the cancel was sent")
        }
        _ => Some("no calibration is running"),
    }
}

fn is_calibration(record: &CommandRecord) -> bool {
    record.command == 241 && record.params.iter().any(|p| *p != 0.0)
}

/// The names of the `MARV_DEVICE` ids (`docs/link-contract.md` section 8).
pub fn device_name(id: u32) -> Option<&'static str> {
    Some(match id {
        1 => "IMU1",
        2 => "IMU2",
        3 => "BARO",
        4 => "MAG",
        5 => "GNSS",
        6 => "BATT",
        7 => "RC",
        9 => "SD",
        10 => "PSRAM",
        11 => "LED",
        12 => "ESC",
        _ => return None,
    })
}

/// What a command is called in the status line.
pub fn command_label(record: &CommandRecord) -> String {
    let p = &record.params;
    match record.command {
        241 if p.iter().all(|v| *v == 0.0) => "Cancel calibration".into(),
        241 if p[0] == 1.0 => "Gyro calibration".into(),
        241 if p[4] == 4.0 => "Accelerometer calibration".into(),
        241 => "Calibration".into(),
        245 => match p[0] as u32 {
            0 => "Reload from flash".into(),
            1 => "Save to flash".into(),
            2 => "Reset all to defaults".into(),
            3 => "Reset calibration".into(),
            _ => "Storage".into(),
        },
        246 => "Reboot".into(),
        42_430 => {
            let device =
                device_name(p[0] as u32).map_or_else(|| format!("device {}", p[0]), str::to_string);
            match p[1] as u32 {
                1 => format!("Restart {device}"),
                2 => format!("Self-test {device}"),
                _ => format!("Device action on {device}"),
            }
        }
        n => record
            .name
            .clone()
            .unwrap_or_else(|| format!("Command {n}")),
    }
}

/// The status line of the last command: its words and colour.
pub fn command_line(record: &CommandRecord) -> (String, Color32) {
    let label = command_label(record);
    let (mut text, colour) = match &record.status {
        CommandStatus::Waiting if record.sends > 1 => (
            format!(
                "{label}: waiting for the vehicle (send {} of 3)",
                record.sends
            ),
            YELLOW,
        ),
        CommandStatus::Waiting => (format!("{label}: waiting for the vehicle"), YELLOW),
        CommandStatus::InProgress {
            percent: Some(percent),
        } => (format!("{label}: {percent} %"), YELLOW),
        CommandStatus::InProgress { percent: None } => (format!("{label}: in progress"), YELLOW),
        CommandStatus::Finished { text, accepted } => (
            format!("{label}: {text}"),
            if *accepted { GREEN } else { RED },
        ),
        CommandStatus::NoAnswer => (
            format!("{label}: no answer after {} sends", record.sends),
            RED,
        ),
        CommandStatus::NoResult => (
            format!("{label}: no result (the vehicle stopped reporting)"),
            RED,
        ),
        CommandStatus::Refused(why) => (format!("{label}: refused ({why})"), YELLOW),
    };
    match &record.cancel {
        Some(CancelStatus::Waiting) => text.push_str(" (cancel sent)"),
        Some(CancelStatus::Answered(answer)) => text.push_str(&format!(" (cancel {answer})")),
        Some(CancelStatus::NoAnswer) => text.push_str(" (cancel: no answer)"),
        None => {}
    }
    if let Some(refused) = &record.refused {
        text.push_str(&format!(
            " — {} other command(s) refused meanwhile",
            refused.count
        ));
    }
    (text, colour)
}

/// What is shown when there is no session.
pub fn not_connected() -> (String, Color32) {
    ("Not connected".to_string(), GREY)
}

/// The reason the session gives when its reader reaches the end (`gcs-mavlink`, `read_loop`); for a
/// replay that is the normal end.
const END_OF_INPUT: &str = "link closed by the other end";

/// Text and colour of the status. `now` is the time since the session started.
pub fn describe(link: &LinkState, now: Duration) -> (String, Color32) {
    describe_session(link, now, false)
}

/// As [`describe`]; `replaying` is true when the session replays a log file, whose end is not a
/// failure.
pub fn describe_session(link: &LinkState, now: Duration, replaying: bool) -> (String, Color32) {
    match &link.status {
        LinkStatus::Connecting => ("Connecting…".to_string(), YELLOW),
        LinkStatus::Connected => {
            let text = match &link.vehicle {
                Some(v) => format!(
                    "Connected — system {}, component {}, {}",
                    v.system_id, v.component_id, v.mav_type
                ),
                None => "Connected".to_string(),
            };
            (text, GREEN)
        }
        LinkStatus::Lost => {
            let silent = now.saturating_sub(link.last_heartbeat.unwrap_or(Duration::ZERO));
            (
                format!("Lost — no heartbeat for {:.1} s", silent.as_secs_f64()),
                RED,
            )
        }
        LinkStatus::Closed(reason) if replaying && reason == END_OF_INPUT => {
            ("Replay finished".to_string(), GREY)
        }
        LinkStatus::Closed(reason) => (format!("Closed — {reason}"), RED),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcs_mavlink::VehicleId;

    fn vehicle() -> VehicleId {
        VehicleId {
            system_id: 1,
            component_id: 1,
            mav_type: "MAV_TYPE_GENERIC".into(),
            autopilot: "MAV_AUTOPILOT_GENERIC".into(),
        }
    }

    #[test]
    fn a_role_has_a_name_and_only_operate_blocks_configure() {
        assert_eq!(role(Access::Operate), "Operate (read only)");
        assert_eq!(role(Access::Configure), "Configure");
        assert_eq!(configure_blocked(Access::Operate), Some(CONFIGURE_BLOCKED));
        assert_eq!(configure_blocked(Access::Configure), None);
    }

    #[test]
    fn connecting_is_yellow() {
        let link = LinkState::default();
        assert_eq!(
            describe(&link, Duration::from_secs(1)),
            ("Connecting…".to_string(), YELLOW)
        );
    }

    #[test]
    fn connected_is_green_and_names_the_vehicle() {
        let link = LinkState {
            status: LinkStatus::Connected,
            vehicle: Some(vehicle()),
            last_heartbeat: Some(Duration::from_secs(5)),
            ..LinkState::default()
        };
        assert_eq!(
            describe(&link, Duration::from_secs(6)),
            (
                "Connected — system 1, component 1, MAV_TYPE_GENERIC".to_string(),
                GREEN
            )
        );
    }

    #[test]
    fn lost_is_red_and_counts_from_the_last_heartbeat() {
        let link = LinkState {
            status: LinkStatus::Lost,
            vehicle: Some(vehicle()),
            last_heartbeat: Some(Duration::from_millis(10_000)),
            ..LinkState::default()
        };
        assert_eq!(
            describe(&link, Duration::from_millis(14_200)),
            ("Lost — no heartbeat for 4.2 s".to_string(), RED)
        );
    }

    #[test]
    fn closed_is_red_and_gives_the_reason() {
        let link = LinkState {
            status: LinkStatus::Closed("connection reset".into()),
            ..LinkState::default()
        };
        assert_eq!(
            describe(&link, Duration::from_secs(9)),
            ("Closed — connection reset".to_string(), RED)
        );
    }

    #[test]
    fn a_replay_that_reached_its_end_is_finished_not_red() {
        let link = LinkState {
            status: LinkStatus::Closed(END_OF_INPUT.into()),
            ..LinkState::default()
        };
        let now = Duration::from_secs(9);
        assert_eq!(
            describe_session(&link, now, true),
            ("Replay finished".to_string(), GREY)
        );
        assert_eq!(
            describe_session(&link, now, false),
            (format!("Closed — {END_OF_INPUT}"), RED)
        );
    }

    #[test]
    fn a_replay_that_failed_keeps_its_text() {
        let link = LinkState {
            status: LinkStatus::Closed("read failed: bad record".into()),
            ..LinkState::default()
        };
        assert_eq!(
            describe_session(&link, Duration::from_secs(9), true),
            ("Closed — read failed: bad record".to_string(), RED)
        );
    }
}
