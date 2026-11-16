//! The simulated vehicle's log download (`docs/link-contract.md`, section 11), in virtual time.

use gcs_sim::{LogDataDrop, SimConfig, SimLog, SimVehicle, default_logs, log_bytes};
use mavlink::dialects::common::{
    HEARTBEAT_DATA, LOG_REQUEST_DATA_DATA, LOG_REQUEST_END_DATA, LOG_REQUEST_LIST_DATA,
    MavAutopilot, MavMessage, MavType,
};
use mavlink::{MAVLinkV2MessageRaw, MavHeader, MavlinkReader};

fn decode(bytes: &[u8]) -> Vec<MavMessage> {
    let mut reader = MavlinkReader::new(bytes);
    let mut out = Vec::new();
    while let Ok((_, message)) = reader.read_any_message::<MavMessage>() {
        out.push(message);
    }
    out
}

fn encode(message: &MavMessage) -> Vec<u8> {
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message(
        MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 0,
        },
        message,
    );
    raw.raw_bytes().to_vec()
}

fn heartbeat() -> Vec<u8> {
    encode(&MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        mavlink_version: 3,
        ..HEARTBEAT_DATA::default()
    }))
}

fn request_list(start: u16, end: u16) -> Vec<u8> {
    encode(&MavMessage::LOG_REQUEST_LIST(LOG_REQUEST_LIST_DATA {
        start,
        end,
        target_system: 1,
        target_component: 1,
    }))
}

fn request_data(id: u16, ofs: u32, count: u32) -> Vec<u8> {
    request_data_to(1, id, ofs, count)
}

fn request_data_to(system: u8, id: u16, ofs: u32, count: u32) -> Vec<u8> {
    encode(&MavMessage::LOG_REQUEST_DATA(LOG_REQUEST_DATA_DATA {
        ofs,
        count,
        id,
        target_system: system,
        target_component: 1,
    }))
}

fn request_end() -> Vec<u8> {
    encode(&MavMessage::LOG_REQUEST_END(LOG_REQUEST_END_DATA {
        target_system: 1,
        target_component: 1,
    }))
}

/// Steps the vehicle every millisecond from `from` to `to` inclusive (`input` goes in at `from`) and
/// returns what it sent, in order.
fn run(vehicle: &mut SimVehicle, from: u64, to: u64, input: &[u8]) -> Vec<MavMessage> {
    let mut bytes = vehicle.step(from, input);
    for t in from + 1..=to {
        bytes.extend(vehicle.step(t, &[]));
    }
    decode(&bytes)
}

/// (id, ofs, bytes) of every LOG_DATA, in order.
fn log_data(messages: &[MavMessage]) -> Vec<(u16, u32, Vec<u8>)> {
    messages
        .iter()
        .filter_map(|m| match m {
            MavMessage::LOG_DATA(d) => Some((d.id, d.ofs, d.data[..usize::from(d.count)].to_vec())),
            _ => None,
        })
        .collect()
}

/// (id, num_logs, last_log_num, size, time_utc) of every LOG_ENTRY.
fn entries(messages: &[MavMessage]) -> Vec<(u16, u16, u16, u32, u32)> {
    messages
        .iter()
        .filter_map(|m| match m {
            MavMessage::LOG_ENTRY(e) => {
                Some((e.id, e.num_logs, e.last_log_num, e.size, e.time_utc))
            }
            _ => None,
        })
        .collect()
}

fn vehicle() -> SimVehicle {
    // Past the boot text, with a heartbeat seen.
    let mut vehicle = SimVehicle::new(SimConfig::default());
    vehicle.step(0, &heartbeat());
    vehicle
}

#[test]
fn the_default_logs_have_gaps_an_odd_length_and_an_empty_one() {
    let logs = default_logs();
    let numbers: Vec<u16> = logs.iter().map(|l| l.number).collect();
    assert_eq!(numbers, [2, 3, 5, 8, 11]);
    assert!(logs.iter().any(|l| l.size == 0));
    assert!(logs.iter().any(|l| l.size % 90 != 0));
    assert!(logs.iter().any(|l| l.size > 46_080));
    assert_eq!(log_bytes(2, 12_345), log_bytes(2, 12_345));
    assert_ne!(log_bytes(2, 1000), log_bytes(5, 1000));
}

#[test]
fn a_listing_gives_one_entry_per_log_in_number_order_through_the_slot() {
    let mut vehicle = vehicle();
    // 4 frames per 5 ms: four entries in the first slot, the fifth in the next.
    let first = run(&mut vehicle, 100, 104, &request_list(0, 0xFFFF));
    assert_eq!(entries(&first).len(), 4, "{first:?}");
    let rest = run(&mut vehicle, 105, 120, &[]);
    let all: Vec<_> = entries(&first).into_iter().chain(entries(&rest)).collect();
    assert_eq!(
        all,
        vec![
            (2, 5, 11, 12_345, 0),
            (3, 5, 11, 0, 0),
            (5, 5, 11, 9_000, 0),
            (8, 5, 11, 60_001, 0),
            (11, 5, 11, 450, 0),
        ]
    );
}

#[test]
fn a_listing_follows_the_range() {
    let mut vehicle = vehicle();
    let got = entries(&run(&mut vehicle, 100, 130, &request_list(3, 8)));
    let ids: Vec<u16> = got.iter().map(|e| e.0).collect();
    assert_eq!(ids, [3, 5, 8]);
    // num_logs counts every log of the card, not the ones in the range.
    assert!(got.iter().all(|e| e.1 == 5 && e.2 == 11));
    // No log in the range: nothing.
    let none = run(&mut vehicle, 200, 400, &request_list(12, 20));
    assert!(entries(&none).is_empty(), "{none:?}");
}

#[test]
fn an_empty_card_answers_with_one_entry_with_id_0() {
    let mut vehicle = SimVehicle::new(SimConfig {
        logs: Vec::new(),
        ..SimConfig::default()
    });
    vehicle.step(0, &heartbeat());
    let got = entries(&run(&mut vehicle, 100, 200, &request_list(0, 0xFFFF)));
    assert_eq!(got, vec![(0, 0, 0, 0, 0)]);
}

#[test]
fn a_request_is_answered_with_frames_of_90_bytes_in_order_to_the_end() {
    let mut vehicle = vehicle();
    let got = log_data(&run(
        &mut vehicle,
        100,
        600,
        &request_data(2, 0, 0xFFFF_FFFF),
    ));
    assert_eq!(got.len(), 138, "12 345 bytes in frames of 90");
    let mut next = 0u32;
    let mut bytes = Vec::new();
    for (id, ofs, data) in &got {
        assert_eq!((*id, *ofs), (2, next));
        assert!(data.len() <= 90 && !data.is_empty());
        next += data.len() as u32;
        bytes.extend_from_slice(data);
    }
    assert_eq!(bytes, log_bytes(2, 12_345));
    assert_eq!(got.last().map(|g| g.2.len()), Some(12_345 % 90));
}

#[test]
fn the_pace_is_4_frames_per_5_ms() {
    let mut vehicle = SimVehicle::new(SimConfig {
        logs: vec![SimLog {
            number: 1,
            size: 2_000_000,
        }],
        ..SimConfig::default()
    });
    vehicle.step(0, &heartbeat());
    let got = log_data(&run(
        &mut vehicle,
        1000,
        1999,
        &request_data(1, 0, 0xFFFF_FFFF),
    ));
    // 200 slots of 5 ms, 4 frames each.
    assert!(got.len().abs_diff(800) <= 4, "{}", got.len());
}

#[test]
fn a_window_is_clamped_to_the_size_and_a_count_cuts_it() {
    let mut vehicle = vehicle();
    let got = log_data(&run(&mut vehicle, 100, 300, &request_data(5, 50, 100)));
    let lens: Vec<usize> = got.iter().map(|g| g.2.len()).collect();
    assert_eq!(lens, [90, 10]);
    assert_eq!(got[0].1, 50);
    assert_eq!(got[1].1, 140);
    let all = log_bytes(5, 9_000);
    assert_eq!(got[0].2, all[50..140]);
    // From near the end, more than is left: to the end of the log.
    let got = log_data(&run(&mut vehicle, 400, 500, &request_data(5, 8_950, 5_000)));
    let bytes: Vec<u8> = got.iter().flat_map(|g| g.2.clone()).collect();
    assert_eq!(bytes, all[8_950..]);
}

#[test]
fn at_or_past_the_end_is_one_zero_count_frame_and_a_missing_log_is_nothing() {
    let mut vehicle = vehicle();
    for (ofs, id) in [(9_000, 5), (50_000, 5), (0, 3)] {
        let got = log_data(&run(&mut vehicle, 100, 300, &request_data(id, ofs, 100)));
        assert_eq!(got, vec![(id, ofs, Vec::new())], "log {id} at {ofs}");
        vehicle.step(10_000, &heartbeat());
    }
    // A log that does not exist: no LOG_DATA at all, for as long as one looks.
    let none = run(&mut vehicle, 11_000, 14_000, &request_data(4, 0, 100));
    assert!(log_data(&none).is_empty(), "{none:?}");
}

#[test]
fn a_new_request_replaces_the_running_one_and_an_end_stops_it() {
    let mut vehicle = vehicle();
    let first = log_data(&run(
        &mut vehicle,
        100,
        120,
        &request_data(8, 0, 0xFFFF_FFFF),
    ));
    assert!(first.iter().all(|g| g.0 == 8) && !first.is_empty());
    let second = log_data(&run(&mut vehicle, 121, 400, &request_data(5, 0, 900)));
    let tail: Vec<_> = second.iter().filter(|g| g.0 == 8).collect();
    assert!(
        tail.is_empty(),
        "{} frames of the replaced transfer",
        tail.len()
    );
    assert_eq!(second.len(), 10);

    let started = log_data(&run(
        &mut vehicle,
        500,
        520,
        &request_data(8, 0, 0xFFFF_FFFF),
    ));
    assert!(!started.is_empty());
    let after = run(&mut vehicle, 521, 800, &request_end());
    assert!(
        log_data(&after).len() <= 4,
        "at most the frames of the step with the END"
    );
    let later = run(&mut vehicle, 801, 1200, &[]);
    assert!(log_data(&later).is_empty());
}

#[test]
fn a_request_for_another_system_is_not_answered() {
    let mut vehicle = vehicle();
    let got = run(&mut vehicle, 100, 300, &request_data_to(9, 5, 0, 900));
    assert!(log_data(&got).is_empty());
}

#[test]
fn a_transfer_stops_when_the_stations_heartbeat_has_not_been_seen_for_3_s() {
    let config = SimConfig {
        logs: vec![SimLog {
            number: 1,
            size: 2_000_000,
        }],
        ..SimConfig::default()
    };
    let mut vehicle = SimVehicle::new(config);
    vehicle.step(0, &heartbeat());
    let mut bytes = 0usize;
    // Heartbeats every second: the transfer runs for the 10 s looked at.
    let mut t = 100;
    let mut input = request_data(1, 0, 0xFFFF_FFFF);
    for _ in 0..10u64 {
        input.extend(heartbeat());
        for message in run(&mut vehicle, t, t + 999, &input) {
            if let MavMessage::LOG_DATA(d) = message {
                bytes += usize::from(d.count);
            }
        }
        input.clear();
        t += 1000;
    }
    assert!(bytes > 600_000, "{bytes}");
    // No more heartbeats: it stops about 3 s later.
    let mut late = 0usize;
    for message in run(&mut vehicle, t, t + 6_000, &[]) {
        if let MavMessage::LOG_DATA(d) = message {
            late += usize::from(d.count);
        }
    }
    assert!(
        late > 0 && late < 3_300 * 72,
        "{late} bytes after the last heartbeat"
    );
    let after = run(&mut vehicle, t + 6_001, t + 8_000, &[]);
    assert!(log_data(&after).is_empty());
}

#[test]
fn chosen_frames_are_lost_the_chosen_number_of_times() {
    let config = SimConfig {
        drop_log_data: vec![
            LogDataDrop { ofs: 900, times: 1 },
            LogDataDrop {
                ofs: 1800,
                times: u32::MAX,
            },
        ],
        ..SimConfig::default()
    };
    let mut vehicle = SimVehicle::new(config);
    vehicle.step(0, &heartbeat());
    let first = log_data(&run(&mut vehicle, 100, 300, &request_data(5, 0, 2_700)));
    let offsets: Vec<u32> = first.iter().map(|g| g.1).collect();
    assert_eq!(offsets.len(), 28, "30 frames, two lost");
    assert!(!offsets.contains(&900) && !offsets.contains(&1800));
    // Asked again, 900 comes, 1800 never does.
    let second = log_data(&run(&mut vehicle, 400, 600, &request_data(5, 900, 1_800)));
    let offsets: Vec<u32> = second.iter().map(|g| g.1).collect();
    assert!(
        offsets.contains(&900) && !offsets.contains(&1800),
        "{offsets:?}"
    );
}

#[test]
fn a_list_request_during_a_download_replaces_the_transfer() {
    let mut vehicle = vehicle();
    let first = log_data(&run(
        &mut vehicle,
        100,
        120,
        &request_data(8, 0, 0xFFFF_FFFF),
    ));
    assert!(!first.is_empty());
    let listed = run(&mut vehicle, 121, 400, &request_list(0, 0xFFFF));
    assert_eq!(entries(&listed).len(), 5, "{listed:?}");
    // At most the frames of the step that carried the request, none after it.
    assert!(log_data(&listed).len() <= 4, "{}", log_data(&listed).len());
    let later = run(&mut vehicle, 401, 800, &[]);
    assert!(log_data(&later).is_empty());
}
