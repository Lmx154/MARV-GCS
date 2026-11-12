use std::collections::HashMap;

use gcs_sim::{SimConfig, SimVehicle, signals};
use mavlink::dialects::common::{HEARTBEAT_DATA, MavAutopilot, MavMessage, MavState, MavType};
use mavlink::{MAVLinkV2MessageRaw, MavHeader, MavlinkReader, Message};

fn decode(bytes: &[u8]) -> Vec<(MavHeader, MavMessage)> {
    let mut reader = MavlinkReader::new(bytes);
    let mut out = Vec::new();
    while let Ok(item) = reader.read_any_message::<MavMessage>() {
        out.push(item);
    }
    out
}

fn encode(system_id: u8, component_id: u8, message: &MavMessage) -> Vec<u8> {
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

fn gcs_heartbeat(system_id: u8, component_id: u8) -> Vec<u8> {
    encode(
        system_id,
        component_id,
        &MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            mavtype: MavType::MAV_TYPE_GCS,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            mavlink_version: 3,
            ..HEARTBEAT_DATA::default()
        }),
    )
}

/// Runs the vehicle from `step_ms` to `end_ms` and counts messages by id.
fn counts(step_ms: u64, end_ms: u64) -> HashMap<u32, usize> {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let mut bytes = Vec::new();
    let mut t = step_ms;
    while t <= end_ms {
        bytes.extend(vehicle.step(t, &[]));
        t += step_ms;
    }
    let mut counts = HashMap::new();
    for (_, message) in decode(&bytes) {
        if matches!(message, MavMessage::STATUSTEXT(_)) {
            continue;
        }
        *counts.entry(message.message_id()).or_default() += 1;
    }
    counts
}

#[test]
fn default_config() {
    let config = SimConfig::default();
    assert_eq!((config.system_id, config.component_id), (1, 1));
    let streams: Vec<_> = config
        .streams
        .iter()
        .map(|s| (s.message_id, s.rate_hz))
        .collect();
    assert_eq!(streams, [(0, 1.0), (105, 50.0), (29, 10.0), (24, 5.0)]);
}

#[test]
fn rates_over_ten_seconds_1ms_and_20ms_steps() {
    // HIGHRES_IMU: 50 Hz periods of four frames each.
    let expected = [(0, 10), (105, 2000), (29, 100), (24, 50)];
    for step_ms in [1, 20] {
        let counts = counts(step_ms, 10_000);
        assert_eq!(
            counts.len(),
            expected.len(),
            "step {step_ms} ms: {counts:?}"
        );
        for (id, want) in expected {
            let got = counts[&id];
            let tolerance = if id == 105 { 4 } else { 1 };
            assert!(
                got.abs_diff(want) <= tolerance,
                "step {step_ms} ms, message {id}: {got} instead of {want}"
            );
        }
    }
}

#[test]
fn custom_streams_follow_their_rate() {
    let mut config = SimConfig::default();
    config.streams.retain(|s| s.message_id == 24);
    config.streams[0].rate_hz = 2.5;
    let mut vehicle = SimVehicle::new(config);
    let mut bytes = Vec::new();
    for t in 1..=10_000 {
        bytes.extend(vehicle.step(t, &[]));
    }
    let n = decode(&bytes).len();
    assert!(n.abs_diff(25) <= 1, "{n}");
}

#[test]
fn a_jump_in_time_is_not_made_up() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    for t in 0..=1000 {
        vehicle.step(t, &[]);
    }
    let jump = decode(&vehicle.step(61_000, &[]));
    // One message of each stream, four of them HIGHRES_IMU: 1 + 4 + 1 + 1.
    assert!(jump.len() <= 7, "burst of {} messages", jump.len());
    let after: usize = (61_001..=62_000)
        .map(|t| decode(&vehicle.step(t, &[])).len())
        .sum();
    // One second of the default streams: 1 + 4 * 50 + 10 + 5. HIGHRES_IMU arrives in fours.
    assert!(after.abs_diff(216) <= 8, "{after}");
}

#[test]
fn heartbeat_content() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let messages = decode(&vehicle.step(0, &[]));
    let heartbeat = messages
        .iter()
        .find_map(|(h, m)| match m {
            MavMessage::HEARTBEAT(d) => Some((h, d)),
            _ => None,
        })
        .expect("a heartbeat at time 0");
    assert_eq!((heartbeat.0.system_id, heartbeat.0.component_id), (1, 1));
    assert_eq!(heartbeat.1.mavtype, MavType::MAV_TYPE_GENERIC);
    assert_eq!(heartbeat.1.autopilot, MavAutopilot::MAV_AUTOPILOT_GENERIC);
    assert_eq!(heartbeat.1.system_status, MavState::MAV_STATE_STANDBY);
    assert_eq!(heartbeat.1.mavlink_version, 3);
}

#[test]
fn identity_comes_from_the_config() {
    let config = SimConfig {
        system_id: 7,
        component_id: 3,
        ..SimConfig::default()
    };
    let mut vehicle = SimVehicle::new(config);
    let messages = decode(&vehicle.step(0, &[]));
    assert!(!messages.is_empty());
    assert!(
        messages
            .iter()
            .all(|(h, _)| (h.system_id, h.component_id) == (7, 3))
    );
}

#[test]
fn emitted_message_is_the_signal_at_that_time() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let mut seen = 0;
    for t in 0..=3000u64 {
        for (_, message) in decode(&vehicle.step(t, &[])) {
            if matches!(message, MavMessage::STATUSTEXT(_)) {
                continue;
            }
            let want = signals::messages(message.message_id(), t);
            assert!(!want.is_empty(), "known message");
            // Debug text, because NaN fields make HIGHRES_IMU unequal to itself.
            assert!(
                want.iter()
                    .any(|w| format!("{w:?}") == format!("{message:?}")),
                "at {t} ms: {message:?}"
            );
            seen += 1;
        }
    }
    assert!(seen > 400);
}

#[test]
fn frames_are_mavlink2_with_one_consecutive_sequence() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let mut bytes = Vec::new();
    for t in 0..=5000 {
        bytes.extend(vehicle.step(t, &[]));
    }
    assert_eq!(bytes[0], 0xFD);
    let headers: Vec<_> = decode(&bytes).into_iter().map(|(h, _)| h).collect();
    assert!(headers.len() > 600);
    for (i, h) in headers.iter().enumerate() {
        assert_eq!(h.sequence, (i % 256) as u8, "frame {i}");
    }
    let all_v2 = {
        let mut rest = &bytes[..];
        let mut ok = true;
        while !rest.is_empty() {
            ok &= rest[0] == 0xFD;
            rest = &rest[12 + usize::from(rest[1])..];
        }
        ok
    };
    assert!(all_v2);
}

#[test]
fn same_steps_give_same_bytes() {
    let run = || {
        let mut vehicle = SimVehicle::new(SimConfig::default());
        let mut bytes = Vec::new();
        for t in (0..5000).step_by(7) {
            bytes.extend(vehicle.step(t, &gcs_heartbeat(255, 190)));
        }
        bytes
    };
    let a = run();
    assert!(!a.is_empty());
    assert_eq!(a, run());
}

#[test]
fn counts_a_heartbeat_split_over_two_steps_once() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let frame = gcs_heartbeat(255, 190);
    vehicle.step(0, &frame[..5]);
    assert_eq!(vehicle.heartbeats_received(), 0);
    vehicle.step(5, &frame[5..]);
    assert_eq!(vehicle.heartbeats_received(), 1);
    assert_eq!(vehicle.last_heartbeat_from(), Some((255, 190)));
    vehicle.step(10, &[]);
    assert_eq!(vehicle.heartbeats_received(), 1);
}

#[test]
fn byte_by_byte_delivery_counts_each_heartbeat_once() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    for b in gcs_heartbeat(255, 190)
        .into_iter()
        .chain(gcs_heartbeat(254, 1))
    {
        vehicle.step(0, &[b]);
    }
    assert_eq!(vehicle.heartbeats_received(), 2);
    assert_eq!(vehicle.last_heartbeat_from(), Some((254, 1)));
}

#[test]
fn counts_two_heartbeats_in_one_step() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let mut both = gcs_heartbeat(255, 190);
    both.extend(gcs_heartbeat(255, 190));
    vehicle.step(0, &both);
    assert_eq!(vehicle.heartbeats_received(), 2);
}

#[test]
fn garbage_before_a_heartbeat_does_not_hide_it() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let mut bytes = vec![0x00, 0x13, 0xFD, 0xFD, 0xFD, 0x02, 0xFF, 0x55];
    bytes.extend(gcs_heartbeat(255, 190));
    vehicle.step(0, &bytes);
    assert_eq!(vehicle.heartbeats_received(), 1);
    assert_eq!(vehicle.last_heartbeat_from(), Some((255, 190)));
}

#[test]
fn damaged_frames_and_other_messages_are_ignored() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let mut damaged = gcs_heartbeat(255, 190);
    let last = damaged.len() - 1;
    damaged[last] ^= 0xFF;
    vehicle.step(0, &damaged);
    let ping = encode(255, 190, &MavMessage::SYS_STATUS(signals::sys_status()));
    vehicle.step(10, &ping);
    assert_eq!(vehicle.heartbeats_received(), 0);
    assert_eq!(vehicle.last_heartbeat_from(), None);
    vehicle.step(20, &gcs_heartbeat(255, 190));
    assert_eq!(vehicle.heartbeats_received(), 1);
}

#[test]
fn own_system_heartbeats_are_not_counted() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    vehicle.step(0, &gcs_heartbeat(1, 1));
    assert_eq!(vehicle.heartbeats_received(), 0);
    vehicle.step(1, &gcs_heartbeat(2, 1));
    assert_eq!(vehicle.heartbeats_received(), 1);
    assert_eq!(vehicle.last_heartbeat_from(), Some((2, 1)));
}
