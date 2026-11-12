//! The link status line: a pure function from the link state and the session time.

use std::time::Duration;

use eframe::egui::Color32;
use gcs_mavlink::{LinkState, LinkStatus};

pub const YELLOW: Color32 = Color32::from_rgb(230, 190, 40);
pub const GREEN: Color32 = Color32::from_rgb(70, 200, 100);
pub const RED: Color32 = Color32::from_rgb(235, 80, 80);
pub const GREY: Color32 = Color32::from_rgb(150, 150, 150);

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
