use std::io::Cursor;

use crate::dialect::{
    GPS_RAW_INT_DATA, GpsFixType, HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavParamType,
    MavSeverity, MavState, MavType, PARAM_VALUE_DATA, SCALED_IMU_DATA, STATUSTEXT_DATA,
};
use mavlink::{MavHeader, MavlinkReader, MavlinkVersion};

use super::*;

/// Fixed-seed xorshift64: the only source of pseudo-random data in these tests.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn byte(&mut self) -> u8 {
        (self.next() >> 24) as u8
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() >> 16) as usize % n
    }
}

fn heartbeat() -> Message {
    Message::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0x0102_0304,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

fn param_value() -> Message {
    Message::PARAM_VALUE(PARAM_VALUE_DATA {
        param_value: 12.5,
        param_count: 900,
        param_index: 17,
        param_id: (*b"ABCDEFGHIJKLMNOP").into(),
        param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
    })
}

fn scaled_imu() -> Message {
    Message::SCALED_IMU(SCALED_IMU_DATA {
        time_boot_ms: 123_456,
        xacc: -1,
        yacc: 2,
        zacc: -981,
        xgyro: 4,
        ygyro: -5,
        zgyro: 6,
        xmag: 7,
        ymag: -8,
        zmag: 9,
        temperature: 0,
    })
}

/// Everything after `time_boot_ms` is zero, so the wire payload is cut to one byte.
fn scaled_imu_short() -> Message {
    Message::SCALED_IMU(SCALED_IMU_DATA {
        time_boot_ms: 1,
        ..Default::default()
    })
}

fn gps_raw_int() -> Message {
    Message::GPS_RAW_INT(GPS_RAW_INT_DATA {
        time_usec: 1_700_000_000_000_000,
        lat: 473_977_418,
        lon: 85_455_939,
        alt: 488_000,
        eph: 121,
        epv: 200,
        vel: 345,
        cog: 9000,
        fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
        satellites_visible: 11,
        alt_ellipsoid: 0,
        h_acc: 0,
        v_acc: 0,
        vel_acc: 0,
        hdg_acc: 0,
        yaw: 0,
    })
}

fn statustext() -> Message {
    Message::STATUSTEXT(STATUSTEXT_DATA {
        severity: MavSeverity::MAV_SEVERITY_WARNING,
        text: "PreArm: Compass not calibrated".into(),
        id: 0,
        chunk_seq: 0,
    })
}

fn statustext_short() -> Message {
    Message::STATUSTEXT(STATUSTEXT_DATA {
        severity: MavSeverity::MAV_SEVERITY_INFO,
        text: "hi".into(),
        id: 0,
        chunk_seq: 0,
    })
}

fn cross_check_messages() -> Vec<Message> {
    vec![
        heartbeat(),
        param_value(),
        scaled_imu(),
        scaled_imu_short(),
        gps_raw_int(),
        statustext(),
        statustext_short(),
    ]
}

fn encode(message: &Message) -> Vec<u8> {
    Encoder::new(1, 1).encode(message)
}

fn deframe(bytes: &[u8]) -> (Vec<Frame>, RxStats) {
    let mut deframer = Deframer::new();
    let mut out = Vec::new();
    deframer.push(bytes, &mut out);
    (out, deframer.stats())
}

fn known(frame: &Frame) -> &Message {
    match &frame.body {
        Body::Known(message) => message,
        other => panic!("expected Known, got {other:?}"),
    }
}

/// Build a frame by hand. `crc_extra` is used for the checksum, which covers the header and the
/// payload but never the signature.
fn build_frame(
    incompat: u8,
    sequence: u8,
    id: u32,
    payload: &[u8],
    crc_extra: u8,
    signature: &[u8],
) -> Vec<u8> {
    let mut frame = vec![STX, payload.len() as u8, incompat, 0, sequence, 9, 8];
    frame.extend_from_slice(&id.to_le_bytes()[..3]);
    frame.extend_from_slice(payload);
    let crc = mavlink::calculate_crc(&frame[1..], crc_extra);
    frame.extend_from_slice(&crc.to_le_bytes());
    frame.extend_from_slice(signature);
    frame
}

fn refresh_crc(frame: &mut [u8]) {
    let end = HEADER_LEN + usize::from(frame[1]);
    let crc = mavlink::calculate_crc(&frame[1..end], Message::extra_crc(message_id(frame)));
    frame[end..end + 2].copy_from_slice(&crc.to_le_bytes());
}

const UNKNOWN_ID: u32 = 42_000;

fn unknown_frame() -> Vec<u8> {
    assert!(Message::default_message_from_id(UNKNOWN_ID).is_none());
    build_frame(0, 5, UNKNOWN_ID, &[1, 2, 3, 4, 5], 77, &[])
}

fn three_frames() -> (Vec<u8>, Vec<Message>) {
    let messages = vec![heartbeat(), scaled_imu_short(), statustext()];
    let mut encoder = Encoder::new(42, 7);
    let stream = messages.iter().flat_map(|m| encoder.encode(m)).collect();
    (stream, messages)
}

/// Garbage that is certain not to start a frame: every 0xFD in it is followed, two bytes on, by
/// an incompat byte with a bit that is never valid.
fn garbage(rng: &mut Rng, len: usize) -> Vec<u8> {
    let mut bytes: Vec<u8> = (0..len)
        .map(|_| match rng.byte() {
            STX => 0xFC,
            b => b,
        })
        .collect();
    let mut at = 0;
    while at + 10 <= len {
        bytes[at] = STX;
        bytes[at + 2] |= 0x80;
        at += 11 + rng.below(20);
    }
    assert!(bytes.contains(&STX));
    bytes
}

#[test]
fn api_shape() {
    let _: fn() -> Deframer = Deframer::new;
    let _: fn(u8, u8) -> Encoder = Encoder::new;
    let _: fn(&mut Deframer, &[u8], &mut Vec<Frame>) = Deframer::push;
    let _: fn(&Deframer) -> RxStats = Deframer::stats;
    let _: fn(&mut Encoder, &Message) -> Vec<u8> = Encoder::encode;
    let stats = Deframer::new().stats();
    assert_eq!(
        stats,
        RxStats {
            frames: 0,
            crc_errors: 0,
            unknown: 0,
            undecodable: 0,
            discarded_bytes: 0
        }
    );
}

#[test]
fn round_trip_and_sequence_wrap() {
    let mut encoder = Encoder::new(42, 7);
    let mut deframer = Deframer::new();
    let messages = cross_check_messages();
    let mut all = Vec::new();
    for i in 0..300usize {
        let message = &messages[i % messages.len()];
        let bytes = encoder.encode(message);
        let mut out = Vec::new();
        deframer.push(&bytes, &mut out);
        assert_eq!(out.len(), 1, "frame {i}");
        let frame = &out[0];
        assert_eq!(known(frame), message);
        assert_eq!(frame.raw, bytes);
        assert_eq!(frame.system_id, 42);
        assert_eq!(frame.component_id, 7);
        assert_eq!(frame.sequence, (i % 256) as u8);
        assert_eq!(frame.message_id, message.message_id());
        all.push(frame.sequence);
    }
    assert_eq!(&all[254..258], &[254, 255, 0, 1]);
    let stats = deframer.stats();
    assert_eq!(stats.frames, 300);
    assert_eq!(stats.discarded_bytes, 0);
    assert_eq!(stats.crc_errors, 0);
}

#[test]
fn library_writes_we_read() {
    for message in cross_check_messages() {
        let mut bytes = Vec::new();
        let header = MavHeader {
            system_id: 1,
            component_id: 2,
            sequence: 3,
        };
        mavlink::write_v2_msg(&mut bytes, header, &message).unwrap();
        let (frames, stats) = deframe(&bytes);
        assert_eq!(frames.len(), 1, "{}", message.message_name());
        assert_eq!(known(&frames[0]), &message, "{}", message.message_name());
        assert_eq!(frames[0].raw, bytes);
        assert_eq!(
            (
                frames[0].system_id,
                frames[0].component_id,
                frames[0].sequence
            ),
            (1, 2, 3)
        );
        assert_eq!(stats.discarded_bytes, 0);
    }
}

#[test]
fn we_write_library_reads() {
    for message in cross_check_messages() {
        let mut encoder = Encoder::new(17, 18);
        let first = encoder.encode(&message);
        let second = encoder.encode(&message);
        let mut reader = MavlinkReader::new(Cursor::new([first, second].concat()));
        for sequence in 0..2 {
            let (header, read) = reader
                .read_message::<Message>(MavlinkVersion::V2)
                .unwrap_or_else(|e| panic!("{}: {e}", message.message_name()));
            assert_eq!(read, message);
            assert_eq!(
                (header.system_id, header.component_id, header.sequence),
                (17, 18, sequence)
            );
        }
    }
}

#[test]
fn trailing_zero_payload_is_truncated_on_the_wire() {
    for (message, full_len) in [(scaled_imu_short(), 22usize), (statustext_short(), 51)] {
        let bytes = encode(&message);
        let wire_len = usize::from(bytes[1]);
        assert!(wire_len < full_len, "{wire_len} vs {full_len}");
        assert_eq!(bytes.len(), HEADER_LEN + wire_len + CHECKSUM_LEN);
        let (frames, _) = deframe(&bytes);
        assert_eq!(known(&frames[0]), &message);
    }
    assert_eq!(encode(&scaled_imu_short())[1], 1);
}

#[test]
fn extension_field_is_sent_and_read() {
    let message = Message::SCALED_IMU(SCALED_IMU_DATA {
        time_boot_ms: 5,
        temperature: 2500,
        ..Default::default()
    });
    let mut bytes = Vec::new();
    let header = MavHeader {
        system_id: 1,
        component_id: 2,
        sequence: 3,
    };
    mavlink::write_v2_msg(&mut bytes, header, &message).unwrap();
    assert!(usize::from(bytes[1]) > 22, "payload {}", bytes[1]);
    let (frames, _) = deframe(&bytes);
    assert_eq!(frames.len(), 1);
    assert_eq!(known(&frames[0]), &message);
    match known(&frames[0]) {
        Message::SCALED_IMU(data) => assert_eq!(data.temperature, 2500),
        other => panic!("expected SCALED_IMU, got {other:?}"),
    }
}

#[test]
fn zero_extension_fields_are_truncated_on_the_wire() {
    let message = scaled_imu();
    let mut bytes = Vec::new();
    let header = MavHeader {
        system_id: 1,
        component_id: 2,
        sequence: 3,
    };
    mavlink::write_v2_msg(&mut bytes, header, &message).unwrap();
    assert!(usize::from(bytes[1]) <= 22, "payload {}", bytes[1]);
    let (frames, _) = deframe(&bytes);
    assert_eq!(frames.len(), 1);
    assert_eq!(known(&frames[0]), &message);
}

#[test]
fn chunking_does_not_change_the_result() {
    let (stream, messages) = three_frames();
    let (whole, whole_stats) = deframe(&stream);
    assert_eq!(whole.len(), 3);
    for (frame, message) in whole.iter().zip(&messages) {
        assert_eq!(known(frame), message);
    }

    let mut deframer = Deframer::new();
    let mut by_byte = Vec::new();
    for byte in &stream {
        deframer.push(std::slice::from_ref(byte), &mut by_byte);
    }
    assert_eq!(by_byte, whole);
    assert_eq!(deframer.stats(), whole_stats);

    for split in 0..=stream.len() {
        let mut deframer = Deframer::new();
        let mut out = Vec::new();
        deframer.push(&stream[..split], &mut out);
        deframer.push(&stream[split..], &mut out);
        assert_eq!(out, whole, "split at {split}");
        assert_eq!(deframer.stats(), whole_stats, "split at {split}");
    }
}

#[test]
fn garbage_before_between_and_after_frames() {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let (stream, messages) = three_frames();
    let frames: Vec<Vec<u8>> = {
        let mut encoder = Encoder::new(42, 7);
        messages.iter().map(|m| encoder.encode(m)).collect()
    };
    assert_eq!(frames.concat(), stream);

    let mut input = Vec::new();
    let mut garbage_len = 0;
    for frame in &frames {
        let len = 40 + rng.below(60);
        let g = garbage(&mut rng, len);
        garbage_len += g.len();
        input.extend_from_slice(&g);
        input.extend_from_slice(frame);
    }
    let g = garbage(&mut rng, 77);
    garbage_len += g.len();
    input.extend_from_slice(&g);

    for chunk in [input.len(), 1, 13] {
        let mut deframer = Deframer::new();
        let mut out = Vec::new();
        for piece in input.chunks(chunk) {
            deframer.push(piece, &mut out);
        }
        assert_eq!(out.len(), 3, "chunk {chunk}");
        for ((frame, message), raw) in out.iter().zip(&messages).zip(&frames) {
            assert_eq!(known(frame), message);
            assert_eq!(&frame.raw, raw);
        }
        let stats = deframer.stats();
        assert_eq!(stats.discarded_bytes, garbage_len as u64, "chunk {chunk}");
        assert_eq!(stats.frames, 3);
        assert_eq!(stats.crc_errors, 0);
    }
}

#[test]
fn mavlink1_frames_are_discarded_bytes() {
    let v1 = [0xFE, 3, 0, 1, 1, 0, 9, 9, 9, 0x12, 0x34];
    let good = encode(&heartbeat());
    let (frames, stats) = deframe(&[v1.as_slice(), good.as_slice()].concat());
    assert_eq!(frames.len(), 1);
    assert_eq!(stats.discarded_bytes, v1.len() as u64);
}

#[test]
fn flipped_payload_bit_is_a_crc_error_and_the_next_frame_survives() {
    let mut bad = encode(&param_value());
    bad[HEADER_LEN + 3] ^= 0x10;
    let good = encode(&gps_raw_int());
    let (frames, stats) = deframe(&[bad.clone(), good.clone()].concat());
    assert_eq!(frames.len(), 1);
    assert_eq!(known(&frames[0]), &gps_raw_int());
    assert_eq!(frames[0].raw, good);
    assert_eq!(stats.crc_errors, 1);
    assert_eq!(stats.frames, 1);
    assert_eq!(stats.discarded_bytes, bad.len() as u64);
}

#[test]
fn real_frame_inside_a_false_one_is_not_lost() {
    // A false frame that claims 200 bytes and swallows a real one.
    let good = encode(&gps_raw_int());
    let mut input = vec![STX, 200, 0, 0, 0, 1, 1, 0, 0, 0];
    input.extend_from_slice(&good);
    input.extend_from_slice(&[0; 200]);
    let (frames, stats) = deframe(&input);
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].raw, good);
    assert_eq!(stats.crc_errors, 1);
}

#[test]
fn cut_short_frame_followed_by_good_frame() {
    let full = encode(&param_value());
    let cut = &full[..full.len() - 5];
    let good = encode(&gps_raw_int());
    let (frames, stats) = deframe(&[cut, good.as_slice()].concat());
    assert_eq!(frames.len(), 1);
    assert_eq!(known(&frames[0]), &gps_raw_int());
    assert_eq!(stats.discarded_bytes, cut.len() as u64);
}

#[test]
fn lone_stx_before_a_real_frame() {
    // Heartbeat payload is 9 bytes, so the byte after the lone STX (the real frame's STX) is the
    // length 0xFD and the next one (9) is an invalid incompat byte.
    let good = encode(&heartbeat());
    assert_eq!(good[1], 9);
    let (frames, stats) = deframe(&[&[STX][..], good.as_slice()].concat());
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].raw, good);
    assert_eq!(stats.discarded_bytes, 1);
}

#[test]
fn unknown_id_followed_by_real_frame() {
    let unknown = unknown_frame();
    let good = encode(&heartbeat());
    let (frames, stats) = deframe(&[unknown.as_slice(), good.as_slice()].concat());
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].body, Body::Unknown);
    assert_eq!(frames[0].raw, unknown);
    assert_eq!(frames[0].message_id, UNKNOWN_ID);
    assert_eq!(frames[0].sequence, 5);
    assert_eq!(known(&frames[1]), &heartbeat());
    assert_eq!(stats.unknown, 1);
    assert_eq!(stats.frames, 2);
    assert_eq!(stats.discarded_bytes, 0);
}

#[test]
fn unknown_id_above_65535_followed_by_real_frame() {
    const ID: u32 = 0x01_2345;
    assert_eq!(ID, 74_565);
    assert!(Message::default_message_from_id(ID).is_none());
    let unknown = build_frame(0, 6, ID, &[9, 8, 7], 77, &[]);
    let good = encode(&heartbeat());
    let (frames, stats) = deframe(&[unknown.as_slice(), good.as_slice()].concat());
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].body, Body::Unknown);
    assert_eq!(frames[0].message_id, 74_565);
    assert_eq!(frames[0].raw, unknown);
    assert_eq!(known(&frames[1]), &heartbeat());
    assert_eq!(stats.unknown, 1);
    assert_eq!(stats.discarded_bytes, 0);
}

#[test]
fn unknown_id_stays_pending_until_the_next_byte_arrives() {
    let unknown = unknown_frame();
    let good = encode(&heartbeat());
    let mut deframer = Deframer::new();
    let mut out = Vec::new();
    deframer.push(&unknown, &mut out);
    assert!(out.is_empty());
    assert_eq!(deframer.buffered_len(), unknown.len());
    assert_eq!(deframer.stats().discarded_bytes, 0);
    deframer.push(&good, &mut out);
    assert_eq!(out.len(), 2);
    assert_eq!(out[0].body, Body::Unknown);
    assert_eq!(out[0].raw, unknown);
    assert_eq!(known(&out[1]), &heartbeat());
}

#[test]
fn unknown_id_followed_by_other_byte_is_discarded() {
    let unknown = unknown_frame();
    let good = encode(&heartbeat());
    let input = [unknown.as_slice(), &[0x55], good.as_slice()].concat();
    let (frames, stats) = deframe(&input);
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].raw, good);
    assert_eq!(stats.unknown, 0);
    assert_eq!(stats.discarded_bytes, unknown.len() as u64 + 1);
}

#[test]
fn undecodable_payload_with_good_checksum() {
    let mut bytes = encode(&heartbeat());
    assert_eq!(bytes[1], 9);
    bytes[HEADER_LEN + 7] = 200; // system_status: no such MAV_STATE
    refresh_crc(&mut bytes);
    let good = encode(&heartbeat());
    let (frames, stats) = deframe(&[bytes.clone(), good.clone()].concat());
    assert_eq!(frames.len(), 2);
    match &frames[0].body {
        Body::Undecodable(reason) => assert!(!reason.is_empty()),
        other => panic!("expected Undecodable, got {other:?}"),
    }
    assert_eq!(frames[0].raw, bytes);
    assert_eq!(frames[0].message_id, 0);
    assert_eq!(known(&frames[1]), &heartbeat());
    assert_eq!(stats.undecodable, 1);
    assert_eq!(stats.frames, 2);
    assert_eq!(stats.crc_errors, 0);
}

#[test]
fn signed_frame_is_delivered_with_its_signature() {
    let encoded = encode(&heartbeat());
    let payload = &encoded[HEADER_LEN..HEADER_LEN + 9];
    let signature: Vec<u8> = (0..13)
        .map(|i| if i == 4 { STX } else { 0xA0 + i })
        .collect();
    let signed = build_frame(1, 9, 0, payload, Message::extra_crc(0), &signature);
    assert_eq!(signed.len(), encoded.len() + 13);
    let next = encode(&gps_raw_int());
    let (frames, stats) = deframe(&[signed.clone(), next.clone()].concat());
    assert_eq!(frames.len(), 2);
    assert_eq!(known(&frames[0]), &heartbeat());
    assert_eq!(frames[0].raw, signed);
    assert_eq!(frames[0].sequence, 9);
    assert_eq!(known(&frames[1]), &gps_raw_int());
    assert_eq!(stats.discarded_bytes, 0);
}

#[test]
fn other_incompat_flags_are_not_frames() {
    let encoded = encode(&heartbeat());
    let payload = &encoded[HEADER_LEN..HEADER_LEN + 9];
    let next = encode(&gps_raw_int());
    for flags in [0x02u8, 0x03, 0x80] {
        let flagged = build_frame(flags, 9, 0, payload, Message::extra_crc(0), &[0; 13]);
        let (frames, stats) = deframe(&[flagged.clone(), next.clone()].concat());
        assert_eq!(frames.len(), 1, "flags {flags:#x}");
        assert_eq!(frames[0].raw, next);
        assert_eq!(
            stats.discarded_bytes,
            flagged.len() as u64,
            "flags {flags:#x}"
        );
    }
}

#[test]
fn random_noise_is_bounded_and_does_not_break_decoding() {
    let mut rng = Rng(0x0123_4567_89AB_CDEF);
    let mut deframer = Deframer::new();
    let mut out = Vec::new();
    let mut remaining = 1_000_000usize;
    while remaining > 0 {
        let size = (1 + rng.below(700)).min(remaining);
        let chunk: Vec<u8> = (0..size)
            .map(|_| if rng.below(40) == 0 { STX } else { rng.byte() })
            .collect();
        deframer.push(&chunk, &mut out);
        assert!(deframer.buffered_len() < 600, "{}", deframer.buffered_len());
        remaining -= size;
        out.clear();
    }
    assert!(deframer.peak < 600, "peak {}", deframer.peak);

    // A pending false candidate may hold back the first copies, never the later ones.
    let good = encode(&heartbeat());
    for _ in 0..40 {
        deframer.push(&good, &mut out);
    }
    let decoded = out.iter().filter(|f| f.raw == good).count();
    assert!(decoded >= 30, "decoded {decoded}");
    assert!(out.last().is_some_and(|f| f.raw == good));
    assert_eq!(known(out.last().unwrap()), &heartbeat());
    assert!(deframer.peak < 600);
}

const MARV_TEST_ID: u32 = 42_420;
const MARV_TEST_CRC_EXTRA: u8 = 100;

fn marv_test() -> Message {
    Message::MARV_TEST(crate::dialect::MARV_TEST_DATA {
        seq: 7,
        time_ms: 1_234,
        value: -5,
        tag: 1,
    })
}

/// `MARV_TEST` payload by hand: the fields in wire order (the wider ones first), little endian.
fn marv_test_payload() -> Vec<u8> {
    [
        7u32.to_le_bytes().as_slice(),
        1_234u32.to_le_bytes().as_slice(),
        (-5i32).to_le_bytes().as_slice(),
        &[1],
    ]
    .concat()
}

#[test]
fn marv_test_is_a_known_message_with_crc_extra_100() {
    assert_eq!(Message::extra_crc(MARV_TEST_ID), MARV_TEST_CRC_EXTRA);
    assert!(Message::default_message_from_id(MARV_TEST_ID).is_some());
    assert_eq!(marv_test().message_id(), MARV_TEST_ID);
    assert_eq!(marv_test().message_name(), "MARV_TEST");

    // Written by hand with CRC_EXTRA 100: known at once (no frame behind it to release it).
    let by_hand = build_frame(
        0,
        3,
        MARV_TEST_ID,
        &marv_test_payload(),
        MARV_TEST_CRC_EXTRA,
        &[],
    );
    let (frames, stats) = deframe(&by_hand);
    assert_eq!(frames.len(), 1);
    assert_eq!(known(&frames[0]), &marv_test());
    assert_eq!(frames[0].message_id, MARV_TEST_ID);
    assert_eq!(frames[0].raw, by_hand);
    assert_eq!(
        (
            stats.frames,
            stats.unknown,
            stats.undecodable,
            stats.crc_errors
        ),
        (1, 0, 0, 0)
    );

    // Our encoder writes the same payload and the same checksum.
    let encoded = encode(&marv_test());
    assert_eq!(
        encoded[HEADER_LEN..HEADER_LEN + 13],
        marv_test_payload()[..]
    );
    let crc = mavlink::calculate_crc(&encoded[1..HEADER_LEN + 13], MARV_TEST_CRC_EXTRA);
    assert_eq!(encoded[HEADER_LEN + 13..], crc.to_le_bytes());
}

#[test]
fn marv_test_with_a_wrong_checksum_is_a_crc_error_and_is_not_delivered() {
    // The same frame sealed with another CRC_EXTRA (77), as a dialect that lacks the message does.
    let wrong = build_frame(0, 3, MARV_TEST_ID, &marv_test_payload(), 77, &[]);
    let good = encode(&heartbeat());
    let (frames, stats) = deframe(&[wrong.clone(), good.clone()].concat());
    assert_eq!(frames.len(), 1);
    assert_eq!(known(&frames[0]), &heartbeat());
    assert_eq!(frames[0].raw, good);
    assert_eq!(stats.crc_errors, 1);
    assert_eq!(stats.unknown, 0);
    assert_eq!(stats.frames, 1);
    assert_eq!(stats.discarded_bytes, wrong.len() as u64);

    // A wrong checksum is not held back as an unknown frame either: alone, it is counted at once.
    let (frames, stats) = deframe(&wrong);
    assert!(frames.is_empty());
    assert_eq!(stats.crc_errors, 1);
}
