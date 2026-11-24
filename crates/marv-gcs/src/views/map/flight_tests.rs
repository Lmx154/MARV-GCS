use super::*;
use gcs_mavlink::{FlightOutcome, VehicleId};

fn state(status: LinkStatus, mode: Option<u32>) -> VehicleState {
    let mut state = VehicleState::default();
    state.link.status = status;
    state.link.vehicle = Some(VehicleId {
        system_id: 1,
        component_id: 1,
        mav_type: "MAV_TYPE_QUADROTOR".into(),
        autopilot: "MAV_AUTOPILOT_GENERIC".into(),
    });
    state.link.last_heartbeat = Some(Duration::from_secs(100));
    state.flight.mode_code = mode;
    state
}

fn secs(s: u64) -> Option<Duration> {
    Some(Duration::from_secs(s))
}

#[test]
fn the_buttons_are_enabled_only_with_the_link_up_and_nothing_waiting() {
    let up = state(LinkStatus::Connected, Some(3));
    assert_eq!(gate(&up.link, true, false), None);
    assert_eq!(gate(&up.link, true, true), Some(WAITING));
    assert_eq!(
        gate(&up.link, false, false),
        Some(crate::status::NOT_CONNECTED)
    );
    for status in [
        LinkStatus::Lost,
        LinkStatus::Connecting,
        LinkStatus::Closed("x".into()),
    ] {
        let down = state(status, Some(3));
        assert_eq!(
            gate(&down.link, true, false),
            Some(crate::status::NOT_CONNECTED)
        );
    }
    let mut none = up.clone();
    none.link.vehicle = None;
    assert_eq!(
        gate(&none.link, true, false),
        Some(crate::status::NOT_CONNECTED)
    );
}

#[test]
fn the_mode_is_shown_in_words_and_unknown_codes_say_so() {
    let mut flight = FlightState::default();
    assert_eq!(mode_line(&flight, true), "Mode: unknown (no vehicle)");
    flight.mode_code = Some(0);
    assert_eq!(
        mode_line(&flight, true),
        "Mode: Mode not reported by the vehicle"
    );
    flight.mode_code = Some(3);
    assert_eq!(mode_line(&flight, true), "Mode: Holding position");
    assert_eq!(mode_line(&flight, false), "Mode: unknown (no vehicle)");
    flight.mode_code = Some(8);
    assert!(mode_line(&flight, true).contains("no height estimate"));
    flight.mode_code = Some(12);
    assert_eq!(mode_line(&flight, true), "Mode: Unknown mode (12)");
}

fn denied(code: i32) -> FlightResult {
    FlightResult {
        command: FlightCommand::Start,
        outcome: FlightOutcome::Denied { code },
    }
}

#[test]
fn a_denial_is_an_error_with_the_reason_until_dismissed_or_replaced() {
    let last = denied(6);
    assert_eq!(
        error_line(Some(last), secs(1), None).as_deref(),
        Some("No waypoints are stored on the vehicle: upload a mission first: Start refused")
    );
    assert_eq!(error_line(Some(last), secs(1), Some((last, secs(1)))), None);
    // The same denial again is a new result: shown again.
    assert!(error_line(Some(last), secs(9), Some((last, secs(1)))).is_some());
    // An acceptance replaces it.
    let ok = FlightResult {
        command: FlightCommand::Start,
        outcome: FlightOutcome::Accepted,
    };
    assert_eq!(error_line(Some(ok), secs(9), Some((last, secs(1)))), None);
    assert_eq!(error_line(None, None, None), None);
}

#[test]
fn no_answer_is_an_error() {
    let r = FlightResult {
        command: FlightCommand::Abort,
        outcome: FlightOutcome::NoAnswer,
    };
    assert_eq!(
        error_line(Some(r), None, None).as_deref(),
        Some("Abort: no answer from the vehicle")
    );
}

#[test]
fn the_panel_stands_in_the_hands_off_modes_and_takeover_only() {
    for code in 0..=9 {
        let shown = panel(&state(LinkStatus::Connected, Some(code)), secs(101));
        let hands_off = [2, 3, 4, 5, 6, 8].contains(&code);
        assert_eq!(shown != Panel::Hidden, hands_off, "mode {code}: {shown:?}");
    }
    assert_eq!(
        panel(&state(LinkStatus::Connected, None), secs(101)),
        Panel::Hidden
    );
    assert_eq!(
        panel(&state(LinkStatus::Connected, Some(3)), secs(101)),
        Panel::Standing(FlightMode::Hold)
    );
}

#[test]
fn the_panel_goes_full_width_when_the_heartbeat_stops_after_a_hands_off_mode() {
    assert_eq!(
        panel(&state(LinkStatus::Lost, Some(4)), secs(112)),
        Panel::FullWidth(FlightMode::Auto, secs(12))
    );
    // The last mode was not hands-off: nothing to show; never connected: nothing.
    assert_eq!(
        panel(&state(LinkStatus::Lost, Some(1)), secs(112)),
        Panel::Hidden
    );
    assert_eq!(
        panel(&state(LinkStatus::Lost, Some(7)), secs(112)),
        Panel::Hidden
    );
    assert_eq!(
        panel(&state(LinkStatus::Connecting, Some(3)), secs(112)),
        Panel::Hidden
    );
}

#[test]
fn a_heartbeat_in_pilot_or_stop_clears_the_panel() {
    for code in [1, 7] {
        assert_eq!(
            panel(&state(LinkStatus::Connected, Some(code)), secs(101)),
            Panel::Hidden
        );
    }
}

#[test]
fn the_panel_wording_is_the_contracts_and_claims_nothing_more() {
    assert_eq!(
        TAKEOVER,
        "MISSION SWITCH ON: the laptop flies. TO TAKE CONTROL: mode switch POSITION, ARM ON, throttle in the middle (advice), then MISSION SWITCH OFF: you fly in Position at once. Not in Position: control stays locked until it is. TO LAND NOW: ARM OFF with the mission switch ON. TO STOP THE MOTORS: ARM OFF and the mission switch OFF (they stop at once, in the air too). Do NOT change the transmitter model in the air: the vehicle ignores it for the rest of the flight."
    );
    assert_eq!(LOCKED, "Control locked: mode switch to POSITION, ARM ON");
    assert_eq!(
        LINK_RETURNS,
        "When the link returns with the mission switch ON the vehicle carries on."
    );
    assert_eq!(
        panel_mode(panel(&state(LinkStatus::Connected, Some(2)), secs(101))),
        Some(FlightMode::Takeover)
    );
    assert_eq!(
        panel_mode(panel(&state(LinkStatus::Lost, Some(2)), secs(112))),
        Some(FlightMode::Takeover)
    );
    assert_eq!(
        panel_mode(panel(&state(LinkStatus::Connected, Some(3)), secs(101))),
        Some(FlightMode::Hold)
    );
    assert_eq!(
        panel_header(Panel::Standing(FlightMode::Hold)).as_deref(),
        Some("The vehicle reports: Holding position")
    );
    assert_eq!(
        panel_header(Panel::FullWidth(FlightMode::Auto, secs(12))).as_deref(),
        Some("The link to the vehicle stopped. Last reported 12 s ago: Flying the stored mission")
    );
    assert_eq!(panel_header(Panel::Hidden), None);
    for text in [TAKEOVER, NOT_CHANGED, LINK_RETURNS, LOCKED] {
        assert!(!text.contains("handed back") && !text.contains("is in TAKEOVER"));
    }
}
