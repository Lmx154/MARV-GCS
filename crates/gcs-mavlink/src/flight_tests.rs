use super::*;

#[test]
fn the_five_commands_have_the_contracts_ids_and_parameters() {
    let one = |p2: f32| [1.0, p2, 0.0, 0.0, 0.0, 0.0, 0.0];
    assert_eq!(
        FlightCommand::Start.wire(),
        (MavCmd::MAV_CMD_MISSION_START, [0.0; 7])
    );
    assert_eq!(
        FlightCommand::Pause.wire(),
        (MavCmd::MAV_CMD_DO_PAUSE_CONTINUE, [0.0; 7])
    );
    assert_eq!(
        FlightCommand::Abort.wire(),
        (MavCmd::MAV_CMD_DO_SET_MODE, one(3.0))
    );
    assert_eq!(
        FlightCommand::Land.wire(),
        (MavCmd::MAV_CMD_DO_SET_MODE, one(6.0))
    );
    assert_eq!(
        FlightCommand::Return.wire(),
        (MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7])
    );
    let ids: Vec<u32> = FlightCommand::ALL
        .iter()
        .map(|f| f.wire().0 as u32)
        .collect();
    assert_eq!(ids, [300, 193, 176, 176, 20]);
}

#[test]
fn only_exactly_these_commands_are_flight_commands() {
    for f in FlightCommand::ALL {
        let (command, params) = f.wire();
        assert_eq!(FlightCommand::from_wire(command, &params), Some(f));
    }
    let mut other = FlightCommand::Land.wire().1;
    other[1] = 4.0;
    assert_eq!(
        FlightCommand::from_wire(MavCmd::MAV_CMD_DO_SET_MODE, &other),
        None
    );
    assert_eq!(
        FlightCommand::from_wire(MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, &[0.0; 7]),
        None
    );
}

#[test]
fn each_of_the_20_codes_and_0_has_its_text() {
    let texts: Vec<String> = (0..=20).map(deny_text).collect();
    assert_eq!(texts[0], "Refused, no reason given");
    assert_eq!(
        texts[1],
        "Not on the radio link: connect through the transmitter's WiFi backpack"
    );
    assert_eq!(
        texts[11],
        "A waypoint is more than 1 km from the take-off point"
    );
    assert_eq!(
        texts[18],
        "The link to this station was lost for more than 10 s"
    );
    assert_eq!(
        texts[2],
        "The transmitter is on the CRSF model: missions fly on the MAVLink model only"
    );
    assert_eq!(
        texts[3],
        "Hands-off flight is off: FS_HOLD_EN is 0, or this flight was armed on the CRSF model"
    );
    assert_eq!(texts[15], "The arm switch is OFF");
    assert_eq!(texts[16], "The mode switch is not in Position");
    assert_eq!(
        texts[19],
        "The mission switch (channel 8) is not confirmed ON: no live stick report with it ON"
    );
    assert_eq!(
        texts[20],
        "The vehicle holds a mission but is set to ignore it (GDN_SRC is 0): set GDN_SRC to 1 in Parameters, then Start again"
    );
    for (a, text) in texts.iter().enumerate() {
        assert!(!text.contains("not known"), "code {a}");
        assert_eq!(
            texts.iter().filter(|t| *t == text).count(),
            1,
            "code {a} is unique"
        );
    }
}

#[test]
fn an_unknown_code_is_shown_with_its_number() {
    assert_eq!(deny_text(21), "Refused, reason not known (code 21)");
    assert_eq!(deny_text(-4), "Refused, reason not known (code -4)");
}

#[test]
fn a_denied_start_reads_as_the_contracts_example() {
    let result = FlightResult {
        command: FlightCommand::Start,
        outcome: FlightOutcome::Denied { code: 11 },
    };
    assert_eq!(
        result.error().as_deref(),
        Some("A waypoint is more than 1 km from the take-off point: Start refused")
    );
}

#[test]
fn an_abort_denied_for_the_arm_switch_says_the_landing_continues() {
    let of = |command| {
        FlightResult {
            command,
            outcome: FlightOutcome::Denied { code: 15 },
        }
        .error()
    };
    assert_eq!(
        of(FlightCommand::Abort).as_deref(),
        Some("The arm switch is OFF: the landing continues: Abort refused")
    );
    assert_eq!(
        of(FlightCommand::Start).as_deref(),
        Some("The arm switch is OFF: Start refused")
    );
}

#[test]
fn only_accepted_is_not_an_error_and_no_answer_is_never_accepted() {
    let of = |outcome| {
        FlightResult {
            command: FlightCommand::Pause,
            outcome,
        }
        .error()
    };
    assert_eq!(of(FlightOutcome::Accepted), None);
    assert!(of(FlightOutcome::NoAnswer).is_some_and(|t| t.contains("no answer")));
    assert!(of(FlightOutcome::Other("unsupported")).is_some());
    assert_eq!(
        outcome_of(MavResult::MAV_RESULT_ACCEPTED, 0, "accepted"),
        FlightOutcome::Accepted
    );
    assert_eq!(
        outcome_of(MavResult::MAV_RESULT_DENIED, 2, "denied"),
        FlightOutcome::Denied { code: 2 }
    );
    assert_eq!(
        outcome_of(MavResult::MAV_RESULT_UNSUPPORTED, 0, "unsupported"),
        FlightOutcome::Other("unsupported")
    );
}

#[test]
fn the_mode_codes_decode_and_nine_is_unknown() {
    let modes: Vec<FlightMode> = (0..=9).map(FlightMode::from_code).collect();
    assert_eq!(modes[0], FlightMode::NotReported);
    assert_eq!(modes[2], FlightMode::Takeover);
    assert_eq!(modes[8], FlightMode::Blind);
    assert_eq!(modes[9], FlightMode::Unknown(9));
    assert_eq!(modes[9].text(), "Unknown mode (9)");
    let hands_off: Vec<u32> = (0..=9)
        .filter(|c| FlightMode::from_code(*c).hands_off())
        .collect();
    assert_eq!(hands_off, [3, 4, 5, 6, 8]);
}

#[test]
fn the_reason_is_the_text_without_the_button() {
    let denied = |command, code| FlightResult {
        command,
        outcome: FlightOutcome::Denied { code },
    };
    assert_eq!(
        denied(FlightCommand::Start, 20).reason().as_deref(),
        Some(deny_text(20).as_str())
    );
    assert_eq!(
        denied(FlightCommand::Abort, 15).reason().as_deref(),
        Some("The arm switch is OFF: the landing continues")
    );
    let accepted = FlightResult {
        command: FlightCommand::Start,
        outcome: FlightOutcome::Accepted,
    };
    assert_eq!(accepted.reason(), None);
}
