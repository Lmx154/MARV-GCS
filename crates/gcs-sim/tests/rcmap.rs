//! The RC mapping of `docs/link-contract.md` section 12: the parameter block, the worked-example tables
//! (the oracle: every row), the map example and the MANUAL_CONTROL stream of the simulated vehicle.

use gcs_sim::rcmap::{angle, channels_of, manual_control, throttle};
use gcs_sim::{ParamValue, SimConfig, SimVehicle, signals};
use mavlink::dialects::common::{
    MANUAL_CONTROL_DATA, MavMessage, MavParamType, MavSeverity, PARAM_SET_DATA,
};
use mavlink::types::CharArray;
use mavlink::{MAVLinkV2MessageRaw, MavHeader, MavlinkReader};

fn decode(bytes: &[u8]) -> Vec<MavMessage> {
    let mut reader = MavlinkReader::new(bytes);
    let mut out = Vec::new();
    while let Ok((_, message)) = reader.read_any_message::<MavMessage>() {
        out.push(message);
    }
    out
}

fn set_int(name: &str, value: i32) -> Vec<u8> {
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message(
        MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 0,
        },
        &MavMessage::PARAM_SET(PARAM_SET_DATA {
            param_value: f32::from_le_bytes(value.to_le_bytes()),
            target_system: 1,
            target_component: 1,
            param_id: CharArray::from(name),
            param_type: MavParamType::MAV_PARAM_TYPE_INT32,
        }),
    );
    raw.raw_bytes().to_vec()
}

/// The first MANUAL_CONTROL in the bytes.
fn manual_control_in(bytes: &[u8]) -> Option<MANUAL_CONTROL_DATA> {
    decode(bytes).into_iter().find_map(|m| match m {
        MavMessage::MANUAL_CONTROL(d) => Some(d),
        _ => None,
    })
}

/// Steps the vehicle 1 ms at a time and returns what it sent from `from_ms` to `to_ms`.
fn run(vehicle: &mut SimVehicle, from_ms: u64, to_ms: u64) -> Vec<u8> {
    (from_ms..=to_ms)
        .flat_map(|t| vehicle.step(t, &[]))
        .collect()
}

#[test]
fn angle_rows_of_the_worked_example_table() {
    // (MIN, TRIM, MAX, REV, DZ), then (pwm, output).
    #[allow(clippy::type_complexity)]
    let rows: &[((i32, i32, i32, bool, i32), &[(u16, i16)])] = &[
        (
            (1100, 1500, 1900, false, 0),
            &[
                (1500, 0),
                (1700, 500),
                (1300, -500),
                (1900, 1000),
                (2012, 1000),
                (1100, -1000),
                (988, -1000),
            ],
        ),
        (
            (988, 1500, 2012, false, 0),
            &[
                (2012, 1000),
                (1756, 500),
                (1244, -500),
                (1600, 195),
                (1400, -195),
                (988, -1000),
            ],
        ),
        (
            (988, 1500, 2012, false, 20),
            &[
                (1510, 0),
                (1520, 0),
                (1521, 2),
                (1766, 500),
                (1234, -500),
                (2012, 1000),
            ],
        ),
        (
            (988, 1500, 2012, true, 0),
            &[(1756, -500), (1244, 500), (2012, -1000)],
        ),
        (
            (1000, 1600, 2000, false, 0),
            &[(1800, 500), (1300, -500), (1000, -1000), (2000, 1000)],
        ),
        (
            (1100, 1000, 1900, false, 0),
            &[(1100, 111), (1550, 611), (1900, 1000)],
        ),
        (
            (1400, 1500, 1600, false, 200),
            &[(1400, 0), (1500, 0), (1600, 0)],
        ),
        (
            (1900, 1500, 1100, false, 0),
            &[(988, 0), (1500, 0), (2012, 0)],
        ),
    ];
    for ((min, trim, max, reversed, dz), cases) in rows {
        for (pwm, want) in *cases {
            assert_eq!(
                angle(*pwm, *min, *trim, *max, *reversed, *dz),
                *want,
                "angle MIN {min} TRIM {trim} MAX {max} REV {reversed} DZ {dz}, pwm {pwm}"
            );
        }
    }
}

#[test]
fn throttle_rows_of_the_worked_example_table() {
    // (MIN, MAX, REV, DZ), then (pwm, output).
    #[allow(clippy::type_complexity)]
    let rows: &[((i32, i32, bool, i32), &[(u16, i16)])] = &[
        (
            (1100, 1900, false, 0),
            &[(988, 0), (1100, 0), (1500, 500), (1900, 1000), (2012, 1000)],
        ),
        (
            (988, 2012, false, 0),
            &[(988, 0), (1244, 250), (1500, 500), (2012, 1000)],
        ),
        (
            (988, 2012, false, 30),
            &[(1018, 0), (1515, 500), (2012, 1000)],
        ),
        ((988, 2012, true, 0), &[(988, 1000), (1244, 750), (2012, 0)]),
        ((1400, 1600, false, 200), &[(1400, 0), (1500, 0), (1600, 0)]),
        ((1900, 1100, false, 0), &[(988, 0), (1500, 0), (2012, 0)]),
    ];
    for ((min, max, reversed, dz), cases) in rows {
        for (pwm, want) in *cases {
            assert_eq!(
                throttle(*pwm, *min, *max, *reversed, *dz),
                *want,
                "throttle MIN {min} MAX {max} REV {reversed} DZ {dz}, pwm {pwm}"
            );
        }
    }
}

/// MANUAL_CONTROL of channels 1 to 4 = `first_four` (the others 1500) with the vehicle's default RC
/// parameters, `changes` applied on top.
fn map_example(first_four: [u16; 4], changes: &[(&str, i32)]) -> MANUAL_CONTROL_DATA {
    let mut params = SimConfig::default().params;
    for (name, value) in changes {
        let p = params.iter_mut().find(|p| p.name == *name).unwrap();
        p.value = ParamValue::Int32(*value);
    }
    let mut channels = [1500u16; 16];
    channels[..4].copy_from_slice(&first_four);
    manual_control(1, &channels, |name| {
        params.iter().find(|p| p.name == name).map(|p| p.value)
    })
    .expect("the default vehicle has the RC block")
}

#[test]
fn the_map_example_of_the_contract() {
    let four = [1700, 1300, 1500, 1900];
    let m = map_example(four, &[]);
    assert_eq!(
        (m.y, m.x, m.z, m.r),
        (500, -500, 500, 1000),
        "roll pitch throttle yaw"
    );
    let m = map_example(four, &[("RCMAP_ROLL", 2), ("RCMAP_PITCH", 1)]);
    assert_eq!((m.y, m.x), (-500, 500), "roll and pitch exchanged");
    assert_eq!((m.z, m.r), (500, 1000));
    // A duplicate map is accepted and both functions read channel 1.
    let m = map_example(four, &[("RCMAP_ROLL", 1), ("RCMAP_YAW", 1)]);
    assert_eq!((m.y, m.r), (500, 500));
}

#[test]
fn the_rc_block_is_44_int32_parameters_after_the_others_in_the_contracts_order() {
    let params = SimConfig::default().params;
    let rc = &params[13..13 + 44];
    assert_eq!(rc.len(), 44);
    assert_eq!(
        params[12].name, "NAV_TEST_16CHARS",
        "no existing index moved"
    );
    let mut want: Vec<(String, i32)> = Vec::new();
    for n in 1..=8 {
        for (suffix, default) in [
            ("MIN", 1100),
            ("TRIM", 1500),
            ("MAX", 1900),
            ("REVERSED", 0),
            ("DZ", 0),
        ] {
            want.push((format!("RC{n}_{suffix}"), default));
        }
    }
    for (name, default) in [
        ("RCMAP_ROLL", 1),
        ("RCMAP_PITCH", 2),
        ("RCMAP_THROTTLE", 3),
        ("RCMAP_YAW", 4),
    ] {
        want.push((name.to_string(), default));
    }
    let got: Vec<_> = rc
        .iter()
        .map(|p| (p.name.clone(), p.value, p.read_only))
        .collect();
    let want: Vec<_> = want
        .into_iter()
        .map(|(n, v)| (n, ParamValue::Int32(v), false))
        .collect();
    assert_eq!(got, want);
}

#[test]
fn the_default_stream_is_the_formulas_applied_to_rc_channels_at_5_hz() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let mut frames = Vec::new();
    for t in 0..=2000u64 {
        for message in decode(&vehicle.step(t, &[])) {
            if let MavMessage::MANUAL_CONTROL(d) = message {
                frames.push((t, d));
            }
        }
    }
    assert_eq!(frames.len(), 11, "5 Hz, both ends of two seconds");
    for (t, d) in &frames {
        let channels = channels_of(&signals::rc_channels(*t));
        // Default parameters: roll channel 1, pitch 2, throttle 3, yaw 4; 1100 to 1900, trim 1500.
        assert_eq!(
            d.y,
            angle(channels[0], 1100, 1500, 1900, false, 0),
            "roll at {t}"
        );
        assert_eq!(
            d.x,
            angle(channels[1], 1100, 1500, 1900, false, 0),
            "pitch at {t}"
        );
        assert_eq!(
            d.z,
            throttle(channels[2], 1100, 1900, false, 0),
            "throttle at {t}"
        );
        assert_eq!(
            d.r,
            angle(channels[3], 1100, 1500, 1900, false, 0),
            "yaw at {t}"
        );
        assert_eq!((d.target, d.buttons), (1, 0));
        assert!((0..=1000).contains(&d.z));
    }
    assert!(
        frames.iter().any(|(_, d)| d.x != d.y && d.y != d.r),
        "the four values differ, so a swap would show"
    );
}

#[test]
fn a_written_map_changes_the_stream() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    vehicle.step(0, &[]);
    let echo = vehicle.step(1, &set_int("RCMAP_ROLL", 2));
    let echo = decode(&echo);
    assert!(
        echo.iter().all(|m| !matches!(m, MavMessage::STATUSTEXT(_))),
        "accepted"
    );
    vehicle.step(2, &set_int("RCMAP_PITCH", 1));
    let m = manual_control_in(&run(&mut vehicle, 3, 400)).expect("a MANUAL_CONTROL");
    let channels = channels_of(&signals::rc_channels(200));
    assert_eq!(m.y, angle(channels[1], 1100, 1500, 1900, false, 0));
    assert_eq!(m.x, angle(channels[0], 1100, 1500, 1900, false, 0));
}

#[test]
fn a_value_out_of_its_range_is_refused_and_a_value_at_its_limit_is_taken() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    vehicle.step(0, &[]);
    for (name, bad) in [
        ("RC1_MIN", 799),
        ("RC2_MAX", 2201),
        ("RC3_REVERSED", 2),
        ("RC4_DZ", 201),
        ("RCMAP_ROLL", 9),
        ("RCMAP_YAW", 0),
    ] {
        let was = vehicle.param(name);
        let out = decode(&vehicle.step(1, &set_int(name, bad)));
        assert_eq!(vehicle.param(name), was, "{name} {bad}");
        assert!(
            out.iter().any(|m| matches!(
                m,
                MavMessage::STATUSTEXT(t)
                    if t.severity == MavSeverity::MAV_SEVERITY_WARNING
                        && t.text.to_str().unwrap() == format!("{name}: out of range")
            )),
            "{name} {bad}: {out:?}"
        );
    }
    for (name, edge) in [("RC1_MIN", 800), ("RC2_MAX", 2200), ("RC4_DZ", 200)] {
        vehicle.step(2, &set_int(name, edge));
        assert_eq!(vehicle.param(name), Some(ParamValue::Int32(edge)));
    }
}

#[test]
fn a_vehicle_without_rc_parameters_sends_no_manual_control() {
    let mut config = SimConfig::default();
    config.params.truncate(13);
    let mut vehicle = SimVehicle::new(config);
    let bytes = run(&mut vehicle, 0, 1000);
    assert!(manual_control_in(&bytes).is_none());
    assert!(
        decode(&bytes)
            .iter()
            .any(|m| matches!(m, MavMessage::RC_CHANNELS(_))),
        "RC_CHANNELS carries on"
    );
}

#[test]
fn a_calibration_with_min_above_max_does_not_panic_in_the_stream() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    vehicle.step(0, &[]);
    vehicle.step(1, &set_int("RC1_MIN", 1900));
    vehicle.step(2, &set_int("RC1_MAX", 1100));
    vehicle.step(3, &set_int("RC3_MIN", 1900));
    vehicle.step(4, &set_int("RC3_MAX", 1100));
    let m = manual_control_in(&run(&mut vehicle, 5, 400)).expect("a MANUAL_CONTROL");
    assert_eq!((m.y, m.z), (0, 0), "MAX <= MIN reads 0");
}
