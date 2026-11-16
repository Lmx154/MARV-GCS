//! Section 2 of the link contract: the default streams, HIGHRES_IMU, MAV_CMD_REQUEST_MESSAGE and
//! the receive rules, driven with virtual time and read back with the mavlink library's own reader.

use std::collections::BTreeMap;

use gcs_sim::{ParamEncoding, SimConfig, SimVehicle, Stream};
use mavlink::dialects::common::{
    COMMAND_ACK_DATA, COMMAND_LONG_DATA, HEARTBEAT_DATA, HIGHRES_IMU_DATA, MavAutopilot, MavCmd,
    MavMessage, MavModeFlag, MavProtocolCapability, MavResult, MavState, MavType,
    PARAM_REQUEST_LIST_DATA,
};
use mavlink::{
    MAVLinkV1MessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkReader, Message, calculate_crc,
};

fn decode(bytes: &[u8]) -> Vec<(MavHeader, MavMessage)> {
    let mut reader = MavlinkReader::new(bytes);
    let mut out = Vec::new();
    while let Ok(item) = reader.read_any_message::<MavMessage>() {
        out.push(item);
    }
    out
}

fn encode_from(system_id: u8, component_id: u8, message: &MavMessage) -> Vec<u8> {
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message(
        MavHeader {
            system_id: u32::from(system_id),
            component_id,
            sequence: 0,
        },
        message,
    );
    raw.raw_bytes().to_vec()
}

fn command(target: (u8, u8), command: MavCmd, param1: f32) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1,
        command,
        target_system: target.0,
        target_component: target.1,
        ..COMMAND_LONG_DATA::default()
    })
}

/// A COMMAND_LONG from the ground station (255, 190) to the vehicle (1, 1).
fn request(command_id: MavCmd, param1: f32) -> Vec<u8> {
    encode_from(255, 190, &command((1, 1), command_id, param1))
}

fn request_message(message_id: u32) -> Vec<u8> {
    request(MavCmd::MAV_CMD_REQUEST_MESSAGE, message_id as f32)
}

/// The same frame with other incompatibility flags and a valid checksum; a frame with the signed
/// flag gets 13 signature bytes.
fn with_flags(mut frame: Vec<u8>, flags: u8) -> Vec<u8> {
    let len = usize::from(frame[1]);
    frame[2] = flags;
    let id = u32::from(frame[7]) | u32::from(frame[8]) << 8 | u32::from(frame[9]) << 16;
    let crc = calculate_crc(&frame[1..10 + len], MavMessage::extra_crc(id));
    frame[10 + len..12 + len].copy_from_slice(&crc.to_le_bytes());
    if flags & 1 != 0 {
        frame.extend_from_slice(&[0x55; 13]);
    }
    frame
}

fn booted(config: SimConfig) -> SimVehicle {
    let mut vehicle = SimVehicle::new(config);
    vehicle.step(0, &[]);
    vehicle
}

fn acks(bytes: &[u8]) -> Vec<COMMAND_ACK_DATA> {
    decode(bytes)
        .into_iter()
        .filter_map(|(_, m)| match m {
            MavMessage::COMMAND_ACK(d) => Some(d),
            _ => None,
        })
        .collect()
}

fn imu_frames(bytes: &[u8]) -> Vec<HIGHRES_IMU_DATA> {
    decode(bytes)
        .into_iter()
        .filter_map(|(_, m)| match m {
            MavMessage::HIGHRES_IMU(d) => Some(d),
            _ => None,
        })
        .collect()
}

#[test]
fn the_default_streams_are_the_real_firmwares_at_its_rates() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let mut bytes = Vec::new();
    for t in 0..=1000 {
        bytes.extend(vehicle.step(t, &[]));
    }
    let mut count: BTreeMap<u32, usize> = BTreeMap::new();
    for (_, m) in decode(&bytes) {
        *count.entry(m.message_id()).or_default() += 1;
    }
    // HEARTBEAT, SYS_STATUS and GPS_RAW_INT 1 Hz (both ends of the second included), the three sensor
    // values 25 Hz, RC_CHANNELS 5 Hz, and the boot STATUSTEXT.
    assert_eq!(
        count,
        BTreeMap::from([
            (0, 2),
            (1, 2),
            (24, 2),
            (29, 26),
            (65, 6),
            (105, 26),
            (116, 26),
            (253, 1)
        ])
    );
    let imu = imu_frames(&bytes);
    assert!(imu.iter().all(|d| d.id == 0), "IMU1 only");
    assert!(imu.iter().all(|d| d.time_usec % 40_000 == 0));
}

#[test]
fn highres_imu_on_the_wire_is_imu1_with_the_magnetometer() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let frames = imu_frames(&vehicle.step(0, &[]));
    assert_eq!(frames.len(), 1);
    let near = |a: f32, b: f32| (a - b).abs() < 2e-6;

    let a = &frames[0];
    assert_eq!((a.id, a.time_usec), (0, 0));
    assert!(near(a.xacc, 0.0) && near(a.yacc, 0.210_367_75) && near(a.zacc, -9.628_141));
    assert!(
        near(a.xgyro, 0.002_397_13) && near(a.ygyro, 0.003_989_98) && near(a.zgyro, 0.001_795_42)
    );
    assert_eq!(a.temperature, 25.0);
    assert_eq!((a.xmag, a.ymag, a.zmag), (0.21, -0.03, 0.42));
    assert!(
        [a.abs_pressure, a.diff_pressure, a.pressure_alt]
            .iter()
            .all(|v| v.is_nan())
    );
    assert_eq!(a.fields_updated.bits(), 0x11FF);

    // Later in time the IMU has moved and the time stamp follows.
    let later = imu_frames(&vehicle.step(40, &[]));
    assert_eq!(later[0].time_usec, 40_000);
    assert_ne!(later[0].yacc, a.yacc);
}

#[test]
fn heartbeat_is_the_contract_heartbeat_at_one_hertz() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let mut bytes = Vec::new();
    for t in 0..=10_000 {
        bytes.extend(vehicle.step(t, &[]));
    }
    let heartbeats: Vec<(MavHeader, HEARTBEAT_DATA)> = decode(&bytes)
        .into_iter()
        .filter_map(|(h, m)| match m {
            MavMessage::HEARTBEAT(d) => Some((h, d)),
            _ => None,
        })
        .collect();
    assert_eq!(heartbeats.len(), 11);
    for (h, d) in &heartbeats {
        assert_eq!((h.system_id, h.component_id), (1, 1));
        assert_eq!(d.mavtype, MavType::MAV_TYPE_GENERIC);
        assert_eq!(d.autopilot, MavAutopilot::MAV_AUTOPILOT_GENERIC);
        assert_eq!(d.base_mode, MavModeFlag::empty());
        assert_eq!(d.custom_mode, 0);
        assert_eq!(d.system_status, MavState::MAV_STATE_STANDBY);
        assert_eq!(d.mavlink_version, 3);
    }
}

#[test]
fn version_request_is_acked_then_answered_with_the_encoding_capability() {
    for (encoding, extra) in [
        (
            ParamEncoding::Bytewise,
            MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE,
        ),
        (
            ParamEncoding::CCast,
            MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST,
        ),
    ] {
        let mut vehicle = booted(SimConfig {
            param_encoding: encoding,
            ..SimConfig::default()
        });
        let messages = decode(&vehicle.step(1, &request_message(148)));
        assert_eq!(messages.len(), 2, "{messages:?}");
        let MavMessage::COMMAND_ACK(ack) = &messages[0].1 else {
            panic!("first is {:?}", messages[0].1)
        };
        assert_eq!(ack.command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!((ack.target_system, ack.target_component), (255, 190));
        let MavMessage::AUTOPILOT_VERSION(version) = &messages[1].1 else {
            panic!("second is {:?}", messages[1].1)
        };
        assert_eq!(
            version.capabilities,
            MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2 | extra
        );
        let want: u32 = if encoding == ParamEncoding::Bytewise {
            8192 + 16
        } else {
            8192 + 131_072
        };
        assert_eq!(version.capabilities.bits(), u64::from(want));
    }
}

#[test]
fn unanswered_version_request_sends_neither_ack_nor_message() {
    let mut vehicle = booted(SimConfig {
        answer_version_request: false,
        ..SimConfig::default()
    });
    assert!(vehicle.step(1, &request_message(148)).is_empty());
    assert!(SimConfig::default().answer_version_request);
    // Other requests are still answered.
    let messages = decode(&vehicle.step(2, &request_message(29)));
    assert_eq!(messages.len(), 2);
}

#[test]
fn requesting_a_stream_gives_an_ack_and_one_copy() {
    let mut vehicle = booted(SimConfig::default());
    let messages = decode(&vehicle.step(10, &request_message(105)));
    assert_eq!(messages.len(), 2, "{messages:?}");
    assert!(matches!(&messages[0].1, MavMessage::COMMAND_ACK(a)
        if a.result == MavResult::MAV_RESULT_ACCEPTED && a.command == MavCmd::MAV_CMD_REQUEST_MESSAGE));
    let copies: Vec<(u8, u64)> = messages[1..]
        .iter()
        .map(|(_, m)| match m {
            MavMessage::HIGHRES_IMU(d) => (d.id, d.time_usec),
            other => panic!("{other:?}"),
        })
        .collect();
    assert_eq!(copies, [(0, 10_000)]);

    for (t, (id, name)) in [
        (29, "SCALED_PRESSURE"),
        (116, "SCALED_IMU2"),
        (65, "RC_CHANNELS"),
        (1, "SYS_STATUS"),
        (24, "GPS_RAW_INT"),
        (0, "HEARTBEAT"),
    ]
    .into_iter()
    .enumerate()
    {
        let messages = decode(&vehicle.step(11 + t as u64, &request_message(id)));
        assert_eq!(messages.len(), 2, "{name}");
        assert!(
            matches!(&messages[0].1, MavMessage::COMMAND_ACK(a) if a.result == MavResult::MAV_RESULT_ACCEPTED)
        );
        assert_eq!(messages[1].1.message_id(), id, "{name}");
    }

    // A stream of a custom configuration can be requested too.
    let mut config = SimConfig::default();
    config.streams.push(Stream {
        message_id: 26,
        rate_hz: 1.0,
    });
    let mut vehicle = booted(config);
    let messages = decode(&vehicle.step(1, &request_message(26)));
    assert_eq!(messages.len(), 2);
    assert!(matches!(messages[1].1, MavMessage::SCALED_IMU(_)));
}

#[test]
fn every_other_command_is_unsupported() {
    let mut vehicle = booted(SimConfig::default());
    let cases = [
        // An id that is no stream: SYS_STATUS is not a default stream any more.
        (MavCmd::MAV_CMD_REQUEST_MESSAGE, 1.0),
        (MavCmd::MAV_CMD_REQUEST_MESSAGE, 9999.0),
        (MavCmd::MAV_CMD_REQUEST_MESSAGE, 29.5),
        (MavCmd::MAV_CMD_REQUEST_MESSAGE, -1.0),
        (MavCmd::MAV_CMD_REQUEST_MESSAGE, f32::NAN),
        (MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, 105.0),
        (MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0),
        (MavCmd::MAV_CMD_DO_SET_MODE, 1.0),
    ];
    for (i, (cmd, param1)) in cases.into_iter().enumerate() {
        let messages = decode(&vehicle.step(1 + i as u64, &request(cmd, param1)));
        assert_eq!(messages.len(), 1, "{cmd:?} {param1}: {messages:?}");
        let MavMessage::COMMAND_ACK(ack) = &messages[0].1 else {
            panic!("{:?}", messages[0].1)
        };
        assert_eq!(ack.command, cmd);
        assert_eq!(
            ack.result,
            MavResult::MAV_RESULT_UNSUPPORTED,
            "{cmd:?} {param1}"
        );
        assert_eq!((ack.target_system, ack.target_component), (255, 190));
    }
}

#[test]
fn commands_for_another_target_are_ignored() {
    for (system, component, expect) in [
        (1, 1, true),
        (0, 0, true),
        (1, 0, true),
        (0, 1, true),
        (2, 1, false),
        (1, 2, false),
        (2, 0, false),
        (0, 2, false),
    ] {
        let mut vehicle = booted(SimConfig::default());
        let frame = encode_from(
            255,
            190,
            &command((system, component), MavCmd::MAV_CMD_REQUEST_MESSAGE, 29.0),
        );
        let got = !acks(&vehicle.step(1, &frame)).is_empty();
        assert_eq!(got, expect, "{system}/{component}");
    }
}

#[test]
fn frames_with_any_incompat_flag_are_dropped_signed_ones_too() {
    let plain = request_message(29);
    // The flag helper must not break the frame by itself: no flags is the plain frame.
    assert_eq!(with_flags(plain.clone(), 0), plain);
    for flags in [0x01, 0x02, 0x03, 0x04, 0x80, 0xFF] {
        let mut vehicle = booted(SimConfig::default());
        let flagged = with_flags(plain.clone(), flags);
        let out = vehicle.step(1, &flagged);
        assert!(acks(&out).is_empty(), "flags {flags:#x} accepted");

        // A plain frame behind it is still handled: the dropped frame is skipped cleanly.
        let mut both = flagged;
        both.extend_from_slice(&plain);
        let out = vehicle.step(2, &both);
        assert_eq!(acks(&out).len(), 1, "flags {flags:#x}");
    }
    // A signed heartbeat is not counted either.
    let mut vehicle = booted(SimConfig::default());
    let heartbeat = encode_from(255, 190, &MavMessage::HEARTBEAT(HEARTBEAT_DATA::default()));
    vehicle.step(1, &with_flags(heartbeat.clone(), 1));
    assert_eq!(vehicle.heartbeats_received(), 0);
    vehicle.step(2, &heartbeat);
    assert_eq!(vehicle.heartbeats_received(), 1);
}

#[test]
fn mavlink_1_frames_are_dropped() {
    let mut vehicle = booted(SimConfig::default());
    let mut raw = MAVLinkV1MessageRaw::new();
    raw.serialize_message(
        MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 0,
        },
        &command((1, 1), MavCmd::MAV_CMD_REQUEST_MESSAGE, 29.0),
    );
    assert!(acks(&vehicle.step(1, raw.raw_bytes())).is_empty());
}

#[test]
fn frames_with_the_vehicles_own_system_id_are_ignored() {
    let mut vehicle = booted(SimConfig::default());
    let mut t = 0;
    for message in [
        command((1, 1), MavCmd::MAV_CMD_REQUEST_MESSAGE, 29.0),
        MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
        }),
    ] {
        let from_itself = encode_from(1, 190, &message);
        let mut bytes = Vec::new();
        for _ in 0..60 {
            t += 1;
            bytes.extend(vehicle.step(t, if t % 60 == 1 { &from_itself } else { &[] }));
        }
        assert!(
            !decode(&bytes)
                .iter()
                .any(|(_, m)| matches!(m, MavMessage::COMMAND_ACK(_) | MavMessage::PARAM_VALUE(_))),
            "answered a frame from its own system id"
        );
    }
    // The same command from another system is answered.
    let from_other = encode_from(
        2,
        190,
        &command((1, 1), MavCmd::MAV_CMD_REQUEST_MESSAGE, 29.0),
    );
    assert_eq!(acks(&vehicle.step(t + 1, &from_other)).len(), 1);
}
