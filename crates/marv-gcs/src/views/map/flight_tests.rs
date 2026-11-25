use super::*;
use gcs_mavlink::{FlightOutcome, FlightResult, VehicleId};

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

fn clock(t: Duration) -> String {
    format!("00:00:{:02}", t.as_secs())
}

fn answered(last: FlightResult, sent: u64, at: u64) -> FlightState {
    let mut flight = FlightState::default();
    flight.begin(last.command, Duration::from_secs(sent));
    flight.finish(last, Duration::from_secs(at));
    flight
}

#[test]
fn no_command_yet_shows_no_line() {
    assert_eq!(result_line(&FlightState::default(), None, clock), None);
}

#[test]
fn a_command_without_an_answer_says_it_is_waiting() {
    let mut flight = FlightState::default();
    flight.begin(FlightCommand::Start, Duration::from_secs(1));
    assert_eq!(
        result_line(&flight, None, clock),
        Some((
            Tone::Waiting,
            "Start: sent, waiting for the vehicle".to_string()
        ))
    );
}

#[test]
fn an_acceptance_is_stated_with_its_time_and_stays_until_dismissed() {
    let ok = FlightResult {
        command: FlightCommand::Start,
        outcome: FlightOutcome::Accepted,
    };
    let flight = answered(ok, 7, 9);
    let shown = (
        Tone::Accepted,
        "Start ACCEPTED by the vehicle at 00:00:09".to_string(),
    );
    assert_eq!(result_line(&flight, None, clock), Some(shown.clone()));
    // Still there on every later frame, whatever else is dismissed.
    assert_eq!(
        result_line(&flight, Some(Some(Duration::from_secs(3))), clock),
        Some(shown)
    );
    // Dismissed: gone.
    assert_eq!(result_line(&flight, Some(flight.sent), clock), None);
}

#[test]
fn a_refusal_names_the_time_and_the_reason_and_stays_until_dismissed() {
    let flight = answered(denied(6), 1, 2);
    assert_eq!(
        result_line(&flight, None, clock),
        Some((
            Tone::Failed,
            "Start REFUSED at 00:00:02: No waypoints are stored on the vehicle: upload a mission first"
                .to_string()
        ))
    );
    assert_eq!(result_line(&flight, Some(flight.sent), clock), None);
    let flight = answered(denied(20), 1, 2);
    assert!(
        result_line(&flight, None, clock)
            .unwrap()
            .1
            .ends_with("set GDN_SRC to 1 in Parameters, then Start again")
    );
    let abort = FlightResult {
        command: FlightCommand::Abort,
        outcome: FlightOutcome::Denied { code: 15 },
    };
    assert_eq!(
        result_line(&answered(abort, 1, 2), None, clock).unwrap().1,
        "Abort REFUSED at 00:00:02: The arm switch is OFF: the landing continues"
    );
}

#[test]
fn no_answer_says_when_it_was_sent() {
    let r = FlightResult {
        command: FlightCommand::Abort,
        outcome: FlightOutcome::NoAnswer,
    };
    assert_eq!(
        result_line(&answered(r, 5, 12), None, clock),
        Some((
            Tone::Failed,
            "Abort: NO ANSWER from the vehicle (sent 00:00:05)".to_string()
        ))
    );
}

#[test]
fn an_old_refusal_is_gone_when_a_new_command_is_sent() {
    let mut flight = answered(denied(6), 1, 2);
    // The press hides the old result at once, before the session has taken the command.
    let after_press = Some(flight.sent);
    assert_eq!(result_line(&flight, after_press, clock), None);
    // The session takes the command: the old text is replaced by the waiting line.
    flight.begin(FlightCommand::Start, Duration::from_secs(8));
    let line = result_line(&flight, after_press, clock).unwrap();
    assert_eq!(line.0, Tone::Waiting);
    assert!(!line.1.contains("REFUSED"));
    // Answered: the new result, not the old one.
    flight.finish(
        FlightResult {
            command: FlightCommand::Start,
            outcome: FlightOutcome::Accepted,
        },
        Duration::from_secs(9),
    );
    assert_eq!(
        result_line(&flight, after_press, clock).unwrap().1,
        "Start ACCEPTED by the vehicle at 00:00:09"
    );
}

#[test]
fn the_local_clock_reads_hh_mm_ss() {
    let text = local_clock(1_700_000_000);
    assert_eq!(text.len(), 8, "{text}");
    assert!(text.split(':').all(|p| p.parse::<u32>().is_ok()), "{text}");
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
    for text in [TAKEOVER, LINK_LOST, LINK_RETURNS, LOCKED] {
        assert!(!text.contains("handed back") && !text.contains("is in TAKEOVER"));
    }
}
