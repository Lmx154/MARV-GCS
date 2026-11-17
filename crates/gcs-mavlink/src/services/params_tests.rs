use super::*;
use crate::dialect::{MavParamType, MavProtocolCapability};
use crate::{VehicleId, VehicleState};
use std::collections::BTreeSet;

const GENERIC: &str = "MAV_AUTOPILOT_GENERIC";
const ARDUPILOT: &str = "MAV_AUTOPILOT_ARDUPILOTMEGA";

// ---------------------------------------------------------------- pure helpers

const ALL_TYPES: [ParamType; 7] = [
    ParamType::U8,
    ParamType::I8,
    ParamType::U16,
    ParamType::I16,
    ParamType::U32,
    ParamType::I32,
    ParamType::F32,
];

#[test]
fn bytewise_encoding_is_the_values_own_little_endian_bytes() {
    let cases: [(ParamValue, [u8; 4]); 10] = [
        (ParamValue::I32(-3), [0xFD, 0xFF, 0xFF, 0xFF]),
        (ParamValue::U8(200), [0xC8, 0x00, 0x00, 0x00]),
        (ParamValue::F32(1.5), [0x00, 0x00, 0xC0, 0x3F]),
        (ParamValue::I8(-1), [0xFF, 0x00, 0x00, 0x00]),
        (ParamValue::U16(0x1234), [0x34, 0x12, 0x00, 0x00]),
        (ParamValue::I16(-2), [0xFE, 0xFF, 0x00, 0x00]),
        (ParamValue::U32(3_735_928_559), [0xEF, 0xBE, 0xAD, 0xDE]),
        (ParamValue::I32(i32::MIN), [0x00, 0x00, 0x00, 0x80]),
        (ParamValue::U8(0), [0, 0, 0, 0]),
        (ParamValue::F32(-0.0), [0x00, 0x00, 0x00, 0x80]),
    ];
    for (value, bytes) in cases {
        assert_eq!(encode(value, ParamEncoding::Bytewise), bytes, "{value:?}");
        assert_eq!(
            decode(value.kind(), bytes, ParamEncoding::Bytewise),
            value,
            "{value:?}"
        );
    }
}

#[test]
fn c_cast_encoding_is_the_bytes_of_the_float() {
    let cases: [(ParamValue, [u8; 4]); 6] = [
        (ParamValue::I32(-3), (-3.0f32).to_le_bytes()),
        (ParamValue::U8(200), 200.0f32.to_le_bytes()),
        (ParamValue::F32(1.5), [0x00, 0x00, 0xC0, 0x3F]),
        (ParamValue::I16(-2), [0x00, 0x00, 0x00, 0xC0]),
        (ParamValue::U16(1), [0x00, 0x00, 0x80, 0x3F]),
        (ParamValue::I8(-1), [0x00, 0x00, 0x80, 0xBF]),
    ];
    for (value, bytes) in cases {
        assert_eq!(encode(value, ParamEncoding::CCast), bytes, "{value:?}");
        assert_eq!(
            decode(value.kind(), bytes, ParamEncoding::CCast),
            value,
            "{value:?}"
        );
    }
    // 200 as a float is 0x43480000, not the byte C8.
    assert_eq!(
        encode(ParamValue::U8(200), ParamEncoding::CCast),
        [0x00, 0x00, 0x48, 0x43]
    );
}

#[test]
fn a_32_bit_value_beyond_24_bits_is_exact_bytewise_and_rounded_by_the_float_cast() {
    let value = ParamValue::U32(3_735_928_559);
    let bytewise = encode(value, ParamEncoding::Bytewise);
    assert_eq!(
        decode(ParamType::U32, bytewise, ParamEncoding::Bytewise),
        value
    );
    // 3735928559 = 0xDEADBEEF; a 24-bit significand keeps 0xDEADBF00 = 3735928576.
    let cast = encode(value, ParamEncoding::CCast);
    assert_eq!(cast, [0xBF, 0xAD, 0x5E, 0x4F]);
    assert_eq!(
        decode(ParamType::U32, cast, ParamEncoding::CCast),
        ParamValue::U32(3_735_928_576)
    );
}

#[test]
fn encode_decode_round_trips_every_type_in_both_encodings() {
    let values = [
        ParamValue::U8(0),
        ParamValue::U8(255),
        ParamValue::I8(-128),
        ParamValue::I8(127),
        ParamValue::U16(0),
        ParamValue::U16(65535),
        ParamValue::I16(-32768),
        ParamValue::I16(32767),
        ParamValue::U32(0),
        ParamValue::U32(16_777_216),
        ParamValue::I32(-16_777_216),
        ParamValue::I32(-3),
        ParamValue::I32(i32::MIN),
        ParamValue::F32(0.0),
        ParamValue::F32(1.5),
        ParamValue::F32(-123456.78),
        ParamValue::F32(f32::MIN_POSITIVE),
        ParamValue::F32(f32::MAX),
    ];
    for encoding in [ParamEncoding::Bytewise, ParamEncoding::CCast] {
        for value in values {
            let raw = encode(value, encoding);
            assert_eq!(
                decode(value.kind(), raw, encoding),
                value,
                "{value:?} {encoding:?}"
            );
        }
    }
    for value in [
        ParamValue::U32(u32::MAX),
        ParamValue::U32(3_735_928_559),
        ParamValue::I32(i32::MAX),
        ParamValue::I32(2_000_000_001),
    ] {
        let raw = encode(value, ParamEncoding::Bytewise);
        assert_eq!(decode(value.kind(), raw, ParamEncoding::Bytewise), value);
    }
}

#[test]
fn bytewise_decoding_ignores_the_unused_bytes() {
    assert_eq!(
        decode(ParamType::U8, [7, 1, 2, 3], ParamEncoding::Bytewise),
        ParamValue::U8(7)
    );
    assert_eq!(
        decode(ParamType::I16, [0xFE, 0xFF, 9, 9], ParamEncoding::Bytewise),
        ParamValue::I16(-2)
    );
}

#[test]
fn value_from_accepts_the_boundaries_of_every_type() {
    let accepted: [(ParamType, f64, ParamValue); 15] = [
        (ParamType::U8, 0.0, ParamValue::U8(0)),
        (ParamType::U8, 255.0, ParamValue::U8(255)),
        (ParamType::I8, -128.0, ParamValue::I8(-128)),
        (ParamType::I8, 127.0, ParamValue::I8(127)),
        (ParamType::U16, 65535.0, ParamValue::U16(65535)),
        (ParamType::I16, -32768.0, ParamValue::I16(-32768)),
        (ParamType::I16, 32767.0, ParamValue::I16(32767)),
        (ParamType::U32, 4_294_967_295.0, ParamValue::U32(u32::MAX)),
        (ParamType::I32, -2_147_483_648.0, ParamValue::I32(i32::MIN)),
        (ParamType::I32, 2_147_483_647.0, ParamValue::I32(i32::MAX)),
        (ParamType::F32, 1.5, ParamValue::F32(1.5)),
        (
            ParamType::F32,
            f64::from(f32::MAX),
            ParamValue::F32(f32::MAX),
        ),
        (
            ParamType::F32,
            -f64::from(f32::MAX),
            ParamValue::F32(-f32::MAX),
        ),
        (ParamType::F32, 0.1, ParamValue::F32(0.1)),
        (ParamType::U8, -0.0, ParamValue::U8(0)),
    ];
    for (kind, number, value) in accepted {
        assert_eq!(kind.value_from(number), Some(value), "{kind:?} {number}");
    }
}

#[test]
fn value_from_refuses_out_of_range_fractions_nan_and_infinity() {
    let refused: [(ParamType, f64); 17] = [
        (ParamType::U8, 256.0),
        (ParamType::U8, -1.0),
        (ParamType::I8, -129.0),
        (ParamType::I8, 128.0),
        (ParamType::U16, 65536.0),
        (ParamType::I16, -32769.0),
        (ParamType::I16, 32768.0),
        (ParamType::U32, 4_294_967_296.0),
        (ParamType::U32, -1.0),
        (ParamType::I32, 2_147_483_648.0),
        (ParamType::I32, -2_147_483_649.0),
        (ParamType::I32, 1.5),
        (ParamType::U8, 0.5),
        (ParamType::F32, 3.5e38),
        (ParamType::F32, -3.5e38),
        (ParamType::U32, 4_294_967_295.5),
        (ParamType::I16, -0.25),
    ];
    for (kind, number) in refused {
        assert_eq!(kind.value_from(number), None, "{kind:?} {number}");
    }
    for kind in ALL_TYPES {
        for number in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(kind.value_from(number), None, "{kind:?} {number}");
        }
    }
}

#[test]
fn type_names_and_display() {
    assert_eq!(ParamType::U8.name(), "u8");
    assert_eq!(ParamType::F32.name(), "f32");
    assert_eq!(ParamValue::I32(-3).to_string(), "-3");
    assert_eq!(ParamValue::F32(1.5).to_string(), "1.5");
    assert_eq!(ParamValue::U8(200).as_f64(), 200.0);
    assert_eq!(ParamValue::F32(0.1).as_f64(), f64::from(0.1f32));
    let param = Param {
        name: "X".into(),
        index: 0,
        kind: ParamType::I32,
        raw: [0xFD, 0xFF, 0xFF, 0xFF],
    };
    assert_eq!(param.value(ParamEncoding::Bytewise), ParamValue::I32(-3));
    let cast = Param {
        raw: (-3.0f32).to_le_bytes(),
        ..param.clone()
    };
    assert_eq!(cast.value(ParamEncoding::CCast), ParamValue::I32(-3));
}

// ---------------------------------------------------------------- rig

struct Rig {
    service: Params,
    state: VehicleState,
    /// Every message the service sent, with the call's time in ms.
    log: Vec<(u64, Message)>,
    /// `Ctx::changed` after the last call.
    changed: bool,
}

impl Rig {
    fn new() -> Self {
        Self::with_vehicle(1, 1, GENERIC)
    }

    fn with_vehicle(system_id: u8, component_id: u8, autopilot: &str) -> Self {
        let mut state = VehicleState::default();
        state.link.vehicle = Some(VehicleId {
            system_id,
            component_id,
            mav_type: "MAV_TYPE_QUADROTOR".into(),
            autopilot: autopilot.into(),
        });
        Self {
            service: Params::new(&SessionConfig {
                access: crate::Access::Configure,
                ..SessionConfig::default()
            }),
            state,
            log: Vec::new(),
            changed: false,
        }
    }

    fn call(&mut self, now_ms: u64, f: impl FnOnce(&mut Params, &mut Ctx)) -> Vec<Message> {
        let mut out = Vec::new();
        let mut cx = Ctx {
            now: Duration::from_millis(now_ms),
            wall_time_us: 0,
            state: &mut self.state,
            out: &mut out,
            changed: false,
        };
        f(&mut self.service, &mut cx);
        self.changed = cx.changed;
        for message in &out {
            self.log.push((now_ms, message.clone()));
        }
        out
    }

    fn tick(&mut self, now_ms: u64) -> Vec<Message> {
        self.call(now_ms, |s, cx| s.on_tick(cx))
    }

    fn frame(&mut self, frame: &Frame, now_ms: u64) -> Vec<Message> {
        self.call(now_ms, |s, cx| s.on_frame(frame, cx))
    }

    fn deliver(&mut self, message: Message, now_ms: u64) -> Vec<Message> {
        self.frame(&vehicle_frame(message), now_ms)
    }

    fn command(&mut self, command: &Command, now_ms: u64) -> Vec<Message> {
        self.call(now_ms, |s, cx| s.on_command(command, cx))
    }

    /// The link becomes `Connected`; the first tick after it starts the protocol.
    fn connect(&mut self, now_ms: u64) -> Vec<Message> {
        self.state.link.status = LinkStatus::Connected;
        self.tick(now_ms)
    }

    fn table(&self) -> &ParamTable {
        &self.state.params
    }

    /// Ticks every 50 ms from `from` to `to` inclusive.
    fn tick_range(&mut self, from: u64, to: u64) {
        let mut t = from;
        while t <= to {
            self.tick(t);
            t += 50;
        }
    }

    fn times(&self, pick: impl Fn(&Message) -> bool) -> Vec<u64> {
        self.log
            .iter()
            .filter(|(_, m)| pick(m))
            .map(|(t, _)| *t)
            .collect()
    }

    fn list_times(&self) -> Vec<u64> {
        self.times(|m| matches!(m, Message::PARAM_REQUEST_LIST(_)))
    }

    fn version_times(&self) -> Vec<u64> {
        self.times(|m| matches!(m, Message::COMMAND_LONG(_)))
    }

    fn set_times(&self) -> Vec<u64> {
        self.times(|m| matches!(m, Message::PARAM_SET(_)))
    }

    /// Times of read requests, one entry per request message.
    fn read_times(&self) -> Vec<u64> {
        self.times(|m| matches!(m, Message::PARAM_REQUEST_READ(_)))
    }

    fn reads(&self) -> Vec<(u64, i16)> {
        self.log
            .iter()
            .filter_map(|(t, m)| match m {
                Message::PARAM_REQUEST_READ(r) => Some((*t, r.param_index)),
                _ => None,
            })
            .collect()
    }
}

fn vehicle_frame(message: Message) -> Frame {
    frame_from(1, message)
}

fn frame_from(system_id: u8, message: Message) -> Frame {
    Frame {
        system_id,
        component_id: 1,
        sequence: 0,
        message_id: 0,
        raw: Vec::new(),
        body: Body::Known(message),
    }
}

fn id16(name: &str) -> [u8; 16] {
    let mut id = [0u8; 16];
    id[..name.len()].copy_from_slice(name.as_bytes());
    id
}

fn pv(name: &str, index: u16, count: u16, ty: MavParamType, raw: [u8; 4]) -> Message {
    Message::PARAM_VALUE(PARAM_VALUE_DATA {
        param_value: f32::from_le_bytes(raw),
        param_count: count,
        param_index: index,
        param_id: id16(name).into(),
        param_type: ty,
    })
}

fn i32_pv(name: &str, index: u16, count: u16, value: i32) -> Message {
    pv(
        name,
        index,
        count,
        MavParamType::MAV_PARAM_TYPE_INT32,
        value.to_le_bytes(),
    )
}

fn version(bits: u64) -> Message {
    Message::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
        capabilities: MavProtocolCapability::from_bits_truncate(bits),
        ..AUTOPILOT_VERSION_DATA::default()
    })
}

const CAP_BYTEWISE: u64 = 16;
const CAP_C_CAST: u64 = 131_072;
const CAP_MAVLINK2: u64 = 8192;

fn commands(messages: &[Message]) -> Vec<&COMMAND_LONG_DATA> {
    messages
        .iter()
        .filter_map(|m| match m {
            Message::COMMAND_LONG(c) => Some(c),
            _ => None,
        })
        .collect()
}

fn sets(messages: &[Message]) -> Vec<&PARAM_SET_DATA> {
    messages
        .iter()
        .filter_map(|m| match m {
            Message::PARAM_SET(s) => Some(s),
            _ => None,
        })
        .collect()
}

/// A rig connected at t=0 with `count` I32 parameters P0.. delivered and the encoding settled
/// byte-wise. Returns with every send so far logged and the last call at t=100.
fn complete_rig(count: u16) -> Rig {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(version(CAP_BYTEWISE | CAP_MAVLINK2), 10);
    for i in 0..count {
        rig.deliver(i32_pv(&format!("P{i}"), i, count, -3), 20 + u64::from(i));
    }
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    rig.tick(100);
    rig
}

// ---------------------------------------------------------------- start, addressing, filtering

#[test]
fn nothing_is_sent_before_the_link_is_connected() {
    let mut rig = Rig::new();
    for status in [
        LinkStatus::Connecting,
        LinkStatus::Lost,
        LinkStatus::Closed("x".into()),
    ] {
        rig.state.link.status = status;
        for t in (0..5000).step_by(50) {
            assert!(rig.tick(t).is_empty());
            assert!(!rig.changed);
        }
        assert!(rig.command(&Command::RefreshParams, 5000).is_empty());
        assert!(!rig.changed);
        assert!(rig.deliver(i32_pv("A", 0, 1, 1), 5001).is_empty());
        assert!(rig.deliver(version(CAP_BYTEWISE), 5002).is_empty());
    }
    assert!(rig.log.is_empty());
    assert_eq!(*rig.table(), ParamTable::default());
    // And the protocol starts on the first Connected, not before.
    assert_eq!(rig.connect(6000).len(), 2);
}

#[test]
fn nothing_is_sent_without_a_vehicle_even_when_connected() {
    let mut rig = Rig::new();
    rig.state.link.vehicle = None;
    assert!(rig.connect(0).is_empty());
    assert!(rig.tick(1000).is_empty());
}

#[test]
fn first_connected_asks_for_the_encoding_and_the_list_addressed_to_the_vehicle() {
    let mut rig = Rig::with_vehicle(7, 3, GENERIC);
    let out = rig.connect(1234);
    assert!(rig.changed);
    assert_eq!(out.len(), 2);
    let command = commands(&out);
    assert_eq!(command.len(), 1);
    assert_eq!(command[0].command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
    assert_eq!(command[0].param1, 148.0);
    assert_eq!(
        (command[0].param2, command[0].param3, command[0].param7),
        (0.0, 0.0, 0.0)
    );
    assert_eq!(
        (command[0].target_system, command[0].target_component),
        (7, 3)
    );
    let Some(Message::PARAM_REQUEST_LIST(list)) = out
        .iter()
        .find(|m| matches!(m, Message::PARAM_REQUEST_LIST(_)))
    else {
        panic!("no list request in {out:?}");
    };
    assert_eq!((list.target_system, list.target_component), (7, 3));
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 0,
            expected: 0
        }
    );
    assert!(!rig.table().encoding_known);
    // The protocol starts once.
    assert!(rig.tick(1250).is_empty());
    assert!(!rig.changed);
}

#[test]
fn the_start_also_happens_on_the_heartbeat_frame_that_made_the_link_connected() {
    let mut rig = Rig::new();
    rig.state.link.status = LinkStatus::Connected;
    let out = rig.deliver(i32_pv("A", 0, 2, 5), 10);
    assert_eq!(commands(&out).len(), 1);
    assert_eq!(rig.list_times(), vec![10]);
}

#[test]
fn only_frames_from_the_vehicles_system_id_are_acted_on() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.changed = false;
    assert!(
        rig.frame(&frame_from(2, i32_pv("A", 0, 1, 1)), 10)
            .is_empty()
    );
    assert!(!rig.changed);
    rig.frame(&frame_from(2, version(CAP_C_CAST)), 11);
    assert!(!rig.changed);
    assert!(rig.table().params.is_empty());
    assert!(!rig.table().encoding_known);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 0,
            expected: 0
        }
    );
    rig.deliver(i32_pv("A", 0, 1, 1), 12);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
}

#[test]
fn frames_that_are_not_decoded_are_ignored() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.changed = false;
    for body in [Body::Unknown, Body::Undecodable("x".into())] {
        let mut frame = vehicle_frame(version(CAP_C_CAST));
        frame.body = body;
        rig.frame(&frame, 5);
    }
    assert!(!rig.changed);
    assert!(!rig.table().encoding_known);
}

// ---------------------------------------------------------------- encoding

#[test]
fn the_version_request_is_sent_three_times_500_ms_apart_then_the_fallback_applies() {
    let mut rig = Rig::with_vehicle(1, 1, ARDUPILOT);
    rig.connect(1000);
    rig.tick_range(1050, 1499);
    assert_eq!(rig.version_times(), vec![1000]);
    rig.tick(1500);
    rig.tick_range(1550, 1999);
    assert_eq!(rig.version_times(), vec![1000, 1500]);
    rig.tick(2000);
    assert_eq!(rig.version_times(), vec![1000, 1500, 2000]);
    rig.tick_range(2050, 2499);
    assert!(!rig.table().encoding_known);
    rig.tick(2500);
    assert!(rig.changed);
    assert!(rig.table().encoding_known);
    assert_eq!(rig.table().encoding, ParamEncoding::CCast);
    rig.tick_range(2550, 8000);
    assert_eq!(rig.version_times().len(), 3);
}

#[test]
fn without_a_reply_a_vehicle_that_is_not_ardupilot_is_bytewise() {
    let mut rig = Rig::with_vehicle(1, 1, GENERIC);
    rig.connect(0);
    rig.tick_range(50, 1450);
    assert!(!rig.table().encoding_known);
    rig.tick(1500);
    assert!(rig.table().encoding_known);
    assert_eq!(rig.table().encoding, ParamEncoding::Bytewise);
}

#[test]
fn the_bytewise_capability_bit_decides_and_stops_the_resends() {
    let mut rig = Rig::with_vehicle(1, 1, ARDUPILOT);
    rig.connect(0);
    rig.tick_range(50, 500);
    assert_eq!(rig.version_times(), vec![0, 500]);
    rig.deliver(version(CAP_BYTEWISE | CAP_MAVLINK2), 520);
    assert!(rig.changed);
    assert!(rig.table().encoding_known);
    assert_eq!(rig.table().encoding, ParamEncoding::Bytewise);
    rig.tick_range(550, 6000);
    assert_eq!(rig.version_times(), vec![0, 500]);
    assert_eq!(rig.table().encoding, ParamEncoding::Bytewise);
}

#[test]
fn the_c_cast_bit_decides_when_the_bytewise_bit_is_missing() {
    let mut rig = Rig::with_vehicle(1, 1, GENERIC);
    rig.connect(0);
    rig.deliver(version(CAP_C_CAST | CAP_MAVLINK2), 20);
    assert!(rig.table().encoding_known);
    assert_eq!(rig.table().encoding, ParamEncoding::CCast);
    rig.tick_range(50, 3000);
    assert_eq!(rig.version_times(), vec![0]);
}

#[test]
fn both_bits_set_means_bytewise() {
    let mut rig = Rig::with_vehicle(1, 1, ARDUPILOT);
    rig.connect(0);
    rig.deliver(version(CAP_BYTEWISE | CAP_C_CAST), 20);
    assert_eq!(rig.table().encoding, ParamEncoding::Bytewise);
}

#[test]
fn a_reply_with_neither_bit_applies_the_fallback_at_once_and_stops_the_resends() {
    for (autopilot, expected) in [
        (ARDUPILOT, ParamEncoding::CCast),
        (GENERIC, ParamEncoding::Bytewise),
    ] {
        let mut rig = Rig::with_vehicle(1, 1, autopilot);
        rig.connect(0);
        rig.deliver(version(CAP_MAVLINK2), 20);
        assert!(rig.changed);
        assert!(rig.table().encoding_known);
        assert_eq!(rig.table().encoding, expected);
        rig.tick_range(50, 3000);
        assert_eq!(rig.version_times(), vec![0]);
    }
}

#[test]
fn a_late_reply_with_a_bit_overrides_the_fallback() {
    let mut rig = Rig::with_vehicle(1, 1, ARDUPILOT);
    rig.connect(0);
    rig.tick_range(50, 2500);
    assert_eq!(rig.table().encoding, ParamEncoding::CCast);
    rig.deliver(version(CAP_BYTEWISE), 3000);
    assert!(rig.changed);
    assert_eq!(rig.table().encoding, ParamEncoding::Bytewise);
    // The same answer again changes nothing.
    rig.deliver(version(CAP_BYTEWISE), 3100);
    assert!(!rig.changed);
}

#[test]
fn the_list_does_not_wait_for_the_encoding_and_keeps_raw_bytes() {
    let mut rig = Rig::with_vehicle(1, 1, ARDUPILOT);
    rig.connect(0);
    rig.deliver(i32_pv("A", 0, 2, -3), 10);
    rig.deliver(
        pv(
            "B",
            1,
            2,
            MavParamType::MAV_PARAM_TYPE_REAL32,
            1.5f32.to_le_bytes(),
        ),
        11,
    );
    assert!(!rig.table().encoding_known);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    assert_eq!(rig.table().params["A"].raw, [0xFD, 0xFF, 0xFF, 0xFF]);
    // Decoded only on demand: with the encoding known later, the same raw bytes read differently.
    assert_eq!(rig.table().value("A"), Some(ParamValue::I32(-3)));
    rig.deliver(version(CAP_C_CAST), 20);
    assert_eq!(rig.table().params["A"].raw, [0xFD, 0xFF, 0xFF, 0xFF]);
    // FD FF FF FF read as a float is a NaN, which the C cast turns into 0.
    assert_eq!(rig.table().value("A"), Some(ParamValue::I32(0)));
    assert_eq!(rig.table().value("B"), Some(ParamValue::F32(1.5)));
    assert_eq!(rig.table().value("C"), None);
}

// ---------------------------------------------------------------- list

#[test]
fn each_value_is_stored_and_counted_by_distinct_index() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(i32_pv("A", 0, 4, 10), 10);
    assert!(rig.changed);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 1,
            expected: 4
        }
    );
    // Index 0 again, same bytes: no new entry, nothing changed.
    rig.deliver(i32_pv("A", 0, 4, 10), 11);
    assert!(!rig.changed);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 1,
            expected: 4
        }
    );
    rig.deliver(i32_pv("C", 2, 4, 30), 12);
    rig.deliver(i32_pv("B", 1, 4, 20), 13);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 3,
            expected: 4
        }
    );
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 3,
            expected: 4
        }
    );
    rig.deliver(i32_pv("D", 3, 4, 40), 14);
    assert!(rig.changed);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    let names: Vec<_> = rig.table().params.keys().cloned().collect();
    assert_eq!(names, ["A", "B", "C", "D"]);
    let c = &rig.table().params["C"];
    assert_eq!(
        (c.name.as_str(), c.index, c.kind, c.raw),
        ("C", 2, ParamType::I32, 30i32.to_le_bytes())
    );
}

#[test]
fn every_wire_type_is_stored_with_its_type_and_the_raw_bytes() {
    let mut rig = Rig::new();
    rig.connect(0);
    let raws = [
        [1, 2, 3, 4],
        [0xFF, 0, 0, 0],
        [0, 0, 0xC0, 0x7F],
        [9, 9, 9, 9],
    ];
    let types = [
        (MavParamType::MAV_PARAM_TYPE_UINT8, ParamType::U8),
        (MavParamType::MAV_PARAM_TYPE_INT8, ParamType::I8),
        (MavParamType::MAV_PARAM_TYPE_UINT16, ParamType::U16),
        (MavParamType::MAV_PARAM_TYPE_INT16, ParamType::I16),
        (MavParamType::MAV_PARAM_TYPE_UINT32, ParamType::U32),
        (MavParamType::MAV_PARAM_TYPE_INT32, ParamType::I32),
        (MavParamType::MAV_PARAM_TYPE_REAL32, ParamType::F32),
    ];
    for (i, (wire, kind)) in types.into_iter().enumerate() {
        let raw = raws[i % raws.len()];
        rig.deliver(pv(&format!("T{i}"), i as u16, 8, wire, raw), 10 + i as u64);
        let stored = &rig.table().params[&format!("T{i}")];
        assert_eq!((stored.kind, stored.raw), (kind, raw), "{wire:?}");
    }
}

#[test]
fn a_nan_bit_pattern_is_kept_exactly() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(
        pv(
            "N",
            0,
            1,
            MavParamType::MAV_PARAM_TYPE_REAL32,
            [0x01, 0x00, 0xC0, 0x7F],
        ),
        10,
    );
    assert_eq!(rig.table().params["N"].raw, [0x01, 0x00, 0xC0, 0x7F]);
}

#[test]
fn a_name_of_exactly_16_characters_has_no_terminator_and_shorter_ones_stop_at_zero() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(i32_pv("ABCDEFGHIJKLMNOP", 0, 2, 1), 10);
    rig.deliver(i32_pv("SHORT", 1, 2, 2), 11);
    let names: Vec<_> = rig.table().params.keys().cloned().collect();
    assert_eq!(names, ["ABCDEFGHIJKLMNOP", "SHORT"]);
    assert_eq!(rig.table().params["ABCDEFGHIJKLMNOP"].name.len(), 16);
}

#[test]
fn a_type_the_table_does_not_have_is_not_stored_but_its_index_counts() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(i32_pv("A", 0, 3, 1), 10);
    rig.deliver(
        pv(
            "WIDE",
            1,
            3,
            MavParamType::MAV_PARAM_TYPE_REAL64,
            [0, 0, 0, 0],
        ),
        11,
    );
    assert!(rig.changed);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 2,
            expected: 3
        }
    );
    rig.deliver(
        pv(
            "HUGE",
            2,
            3,
            MavParamType::MAV_PARAM_TYPE_UINT64,
            [0, 0, 0, 0],
        ),
        12,
    );
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    assert_eq!(rig.table().params.len(), 1);
}

#[test]
fn an_index_beyond_the_count_is_not_stored_or_counted() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(i32_pv("A", 0, 2, 1), 10);
    rig.deliver(i32_pv("Z", 7, 2, 1), 11);
    assert_eq!(rig.table().params.len(), 1);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 1,
            expected: 2
        }
    );
}

#[test]
fn no_answer_to_the_list_request_resends_it_twice_then_fails_after_one_more_second() {
    let mut rig = Rig::new();
    rig.connect(1000);
    rig.tick_range(1050, 3950);
    assert_eq!(rig.list_times(), vec![1000, 2000, 3000]);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 0,
            expected: 0
        }
    );
    rig.tick(4000);
    assert!(rig.changed);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Failed("no answer to the parameter request".into())
    );
    rig.tick_range(4050, 12000);
    assert_eq!(rig.list_times().len(), 3);
}

#[test]
fn refresh_empties_the_table_and_the_writes_and_asks_again() {
    let mut rig = complete_rig(3);
    rig.command(
        &Command::SetParam {
            name: "P1".into(),
            value: ParamValue::I32(5),
        },
        200,
    );
    assert_eq!(rig.table().writes.len(), 1);
    let out = rig.command(&Command::RefreshParams, 300);
    assert!(rig.changed);
    assert_eq!(out.len(), 1);
    assert!(matches!(out[0], Message::PARAM_REQUEST_LIST(_)));
    assert!(rig.table().params.is_empty());
    assert!(rig.table().writes.is_empty());
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 0,
            expected: 0
        }
    );
    // The abandoned write is not resent.
    rig.tick_range(350, 2000);
    assert_eq!(rig.set_times(), vec![200]);
    // The refresh has its own retry clock from 300.
    assert_eq!(rig.list_times(), vec![0, 300, 1300]);
    assert!(rig.table().encoding_known);
}

#[test]
fn a_gap_is_requested_by_index_after_one_second_of_silence() {
    let mut rig = Rig::new();
    rig.connect(0);
    for i in [0u16, 1, 2, 4, 5, 6, 8, 9] {
        rig.deliver(i32_pv(&format!("P{i}"), i, 10, 7), 100);
    }
    rig.tick_range(150, 1050);
    assert!(
        rig.reads().is_empty(),
        "silence is 1 s after the last value"
    );
    let out = rig.tick(1100);
    assert_eq!(out.len(), 2);
    for (message, wanted) in out.iter().zip([3i16, 7]) {
        let Message::PARAM_REQUEST_READ(read) = message else {
            panic!("{message:?}");
        };
        assert_eq!(read.param_index, wanted);
        assert_eq!((read.target_system, read.target_component), (1, 1));
    }
    assert_eq!(rig.table().params.len(), 8);
}

#[test]
fn a_value_arriving_late_restarts_the_silence_clock() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(i32_pv("P0", 0, 3, 7), 100);
    rig.deliver(i32_pv("P1", 1, 3, 7), 900);
    rig.tick_range(950, 1899);
    assert!(rig.reads().is_empty());
    rig.tick(1900);
    assert_eq!(rig.reads(), vec![(1900, 2)]);
}

#[test]
fn eight_failed_rounds_in_a_row_fail_the_fetch_and_keep_what_arrived() {
    let mut rig = Rig::new();
    rig.connect(0);
    for i in [0u16, 1, 2, 4, 5, 6, 8, 9] {
        rig.deliver(i32_pv(&format!("P{i}"), i, 10, 7), 100);
    }
    rig.tick_range(150, 5050);
    let expected_reads: Vec<(u64, i16)> = (0..8)
        .flat_map(|round| [(1100 + 500 * round, 3), (1100 + 500 * round, 7)])
        .collect();
    assert_eq!(rig.reads(), expected_reads);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 8,
            expected: 10
        }
    );
    rig.tick(5100);
    assert!(rig.changed);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Failed("2 of 10 parameters missing".into())
    );
    assert_eq!(rig.table().params.len(), 8);
    rig.tick_range(5150, 9000);
    assert_eq!(rig.reads().len(), 16);
}

#[test]
fn a_new_parameter_resets_the_failed_tries() {
    let mut rig = Rig::new();
    rig.connect(0);
    for i in [0u16, 1, 2, 4, 5, 6, 8, 9] {
        rig.deliver(i32_pv(&format!("P{i}"), i, 10, 7), 100);
    }
    rig.tick_range(150, 2150);
    // Rounds at 1100, 1600 and 2100 were two failures and a third round in flight.
    assert_eq!(rig.read_times().len(), 6);
    rig.deliver(i32_pv("P3", 3, 10, 7), 2200);
    rig.tick_range(2250, 2599);
    assert_eq!(rig.read_times().len(), 6);
    rig.tick(2600);
    assert_eq!(rig.reads()[6..].to_vec(), vec![(2600, 7)]);
    // Failures after the reset: one per round from 3100 on; the eighth at 6600 fails the fetch.
    rig.tick_range(2650, 6550);
    let later: Vec<(u64, i16)> = (1..8).map(|round| (2600 + 500 * round, 7)).collect();
    assert_eq!(rig.reads()[7..].to_vec(), later);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 9,
            expected: 10
        }
    );
    rig.tick(6600);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Failed("1 of 10 parameters missing".into())
    );
}

#[test]
fn at_most_50_indices_are_requested_per_round_and_the_next_round_follows_the_arrivals() {
    let mut rig = Rig::new();
    rig.connect(0);
    for i in 0..10u16 {
        rig.deliver(i32_pv(&format!("P{i}"), i, 130, 1), 100);
    }
    rig.tick_range(150, 1100);
    let first: Vec<i16> = rig.reads().iter().map(|(_, i)| *i).collect();
    assert_eq!(first, (10..60).collect::<Vec<i16>>());
    assert!(rig.reads().iter().all(|(t, _)| *t == 1100));
    for i in 10..60u16 {
        rig.deliver(i32_pv(&format!("P{i}"), i, 130, 1), 1150);
    }
    rig.tick(1150);
    let second: Vec<i16> = rig.reads()[50..].iter().map(|(_, i)| *i).collect();
    assert_eq!(second, (60..110).collect::<Vec<i16>>());
    assert!(rig.reads()[50..].iter().all(|(t, _)| *t == 1150));
    for i in 60..110u16 {
        rig.deliver(i32_pv(&format!("P{i}"), i, 130, 1), 1200);
    }
    rig.tick(1200);
    let third: Vec<i16> = rig.reads()[100..].iter().map(|(_, i)| *i).collect();
    assert_eq!(third, (110..130).collect::<Vec<i16>>());
    for i in 110..130u16 {
        rig.deliver(i32_pv(&format!("P{i}"), i, 130, 1), 1250);
    }
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    rig.tick_range(1300, 5000);
    assert_eq!(rig.reads().len(), 120);
}

#[test]
fn a_changed_count_restarts_the_fetch() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(i32_pv("A", 0, 10, 1), 10);
    rig.deliver(i32_pv("B", 1, 10, 1), 11);
    let out = rig.deliver(i32_pv("C", 2, 12, 1), 20);
    assert!(rig.changed);
    assert_eq!(out.len(), 1);
    assert!(matches!(out[0], Message::PARAM_REQUEST_LIST(_)));
    assert_eq!(rig.list_times(), vec![0, 20]);
    assert!(rig.table().params.is_empty());
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Fetching {
            received: 0,
            expected: 0
        }
    );
    for i in 0..12u16 {
        rig.deliver(i32_pv(&format!("P{i}"), i, 12, 1), 30);
    }
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    assert_eq!(rig.table().params.len(), 12);
}

#[test]
fn the_count_may_change_three_times_and_the_fourth_fails_the_fetch() {
    let mut rig = Rig::new();
    rig.connect(0);
    let counts = [10u16, 12, 14, 16, 18, 20, 22, 24];
    for (n, count) in counts.into_iter().enumerate() {
        rig.deliver(i32_pv("A", 0, count, 1), 10 + n as u64);
        // restarts happen on the 2nd, 4th, 6th value; the 8th is the fourth restart
        let expected_lists = 1 + n.div_ceil(2);
        if n < 7 {
            assert_eq!(
                rig.list_times().len(),
                expected_lists,
                "after count {count}"
            );
            assert!(matches!(rig.table().fetch, FetchStatus::Fetching { .. }));
        }
    }
    assert_eq!(rig.list_times().len(), 4);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Failed("parameter count keeps changing".into())
    );
    assert!(rig.changed);
    rig.tick_range(100, 6000);
    assert_eq!(rig.list_times().len(), 4);
}

#[test]
fn a_completed_fetch_resets_the_restart_budget() {
    let mut rig = Rig::new();
    rig.connect(0);
    for round in 0..5u64 {
        let t = 10 + round * 10;
        rig.deliver(i32_pv("A", 0, 3, 1), t);
        rig.deliver(i32_pv("B", 1, 4, 1), t + 1);
        assert!(
            matches!(rig.table().fetch, FetchStatus::Fetching { .. }),
            "round {round}"
        );
        for i in 0..4u16 {
            rig.deliver(i32_pv(&format!("P{i}"), i, 4, 1), t + 2);
        }
        assert_eq!(rig.table().fetch, FetchStatus::Complete, "round {round}");
        rig.command(&Command::RefreshParams, t + 3);
    }
}

#[test]
fn after_complete_a_value_updates_its_entry_by_name() {
    let mut rig = complete_rig(3);
    rig.deliver(i32_pv("P1", 1, 3, 77), 200);
    assert!(rig.changed);
    assert_eq!(rig.table().value("P1"), Some(ParamValue::I32(77)));
    assert_eq!(rig.table().params["P1"].index, 1);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    assert_eq!(rig.table().params.len(), 3);
    // Same bytes: nothing changed.
    rig.deliver(i32_pv("P1", 1, 3, 77), 210);
    assert!(!rig.changed);
    // A name not in the table is not added.
    rig.deliver(i32_pv("NEW", 2, 3, 5), 220);
    assert!(!rig.changed);
    assert_eq!(rig.table().params.len(), 3);
    // A type the table does not have does not touch the entry.
    rig.deliver(
        pv("P1", 1, 3, MavParamType::MAV_PARAM_TYPE_REAL64, [0; 4]),
        230,
    );
    assert!(!rig.changed);
    assert_eq!(rig.table().value("P1"), Some(ParamValue::I32(77)));
}

#[test]
fn after_complete_a_different_count_fetches_everything_again() {
    let mut rig = complete_rig(3);
    let out = rig.deliver(i32_pv("P1", 1, 4, 77), 200);
    assert_eq!(out.len(), 1);
    assert!(matches!(out[0], Message::PARAM_REQUEST_LIST(_)));
    assert!(rig.table().params.is_empty());
    assert!(matches!(rig.table().fetch, FetchStatus::Fetching { .. }));
}

#[test]
fn a_failed_fetch_ignores_a_count_change_but_still_updates_entries() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(i32_pv("A", 0, 2, 1), 10);
    rig.tick_range(50, 5100);
    assert!(matches!(rig.table().fetch, FetchStatus::Failed(_)));
    let lists = rig.list_times().len();
    rig.deliver(i32_pv("A", 0, 9, 5), 5200);
    assert_eq!(rig.list_times().len(), lists);
    assert_eq!(rig.table().value("A"), Some(ParamValue::I32(5)));
    assert!(matches!(rig.table().fetch, FetchStatus::Failed(_)));
}

// ---------------------------------------------------------------- write

fn set(name: &str, value: ParamValue) -> Command {
    Command::SetParam {
        name: name.into(),
        value,
    }
}

#[test]
fn a_write_sends_param_set_with_the_encoded_value_and_goes_pending() {
    let mut rig = complete_rig(2);
    let out = rig.command(&set("P1", ParamValue::I32(-5)), 200);
    assert!(rig.changed);
    let sent = sets(&out);
    assert_eq!(sent.len(), 1);
    assert_eq!(out.len(), 1);
    assert_eq!(sent[0].param_value.to_le_bytes(), [0xFB, 0xFF, 0xFF, 0xFF]);
    assert_eq!(sent[0].param_type, MavParamType::MAV_PARAM_TYPE_INT32);
    assert_eq!(sent[0].param_id, id16("P1").into());
    assert_eq!((sent[0].target_system, sent[0].target_component), (1, 1));
    assert_eq!(
        rig.table().writes["P1"],
        WriteStatus::Pending {
            value: ParamValue::I32(-5)
        }
    );
    // The table still shows the value in effect.
    assert_eq!(rig.table().value("P1"), Some(ParamValue::I32(-3)));
}

#[test]
fn a_write_is_encoded_c_cast_when_the_vehicle_is_c_cast() {
    let mut rig = Rig::with_vehicle(1, 1, ARDUPILOT);
    rig.connect(0);
    rig.deliver(version(CAP_C_CAST), 1);
    rig.deliver(
        pv(
            "F",
            0,
            1,
            MavParamType::MAV_PARAM_TYPE_UINT8,
            200.0f32.to_le_bytes(),
        ),
        2,
    );
    assert_eq!(rig.table().value("F"), Some(ParamValue::U8(200)));
    let out = rig.command(&set("F", ParamValue::U8(7)), 10);
    let sent = sets(&out);
    assert_eq!(sent[0].param_value.to_le_bytes(), [0x00, 0x00, 0xE0, 0x40]);
    assert_eq!(sent[0].param_type, MavParamType::MAV_PARAM_TYPE_UINT8);
}

#[test]
fn an_unanswered_write_is_sent_three_times_500_ms_apart_then_unanswered() {
    let mut rig = complete_rig(2);
    rig.command(&set("P0", ParamValue::I32(9)), 1000);
    rig.tick_range(1050, 1499);
    assert_eq!(rig.set_times(), vec![1000]);
    rig.tick(1500);
    assert_eq!(rig.set_times(), vec![1000, 1500]);
    rig.tick_range(1550, 1999);
    rig.tick(2000);
    assert_eq!(rig.set_times(), vec![1000, 1500, 2000]);
    rig.tick_range(2050, 2499);
    assert!(!rig.changed);
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Pending {
            value: ParamValue::I32(9)
        }
    );
    rig.tick(2500);
    assert!(rig.changed);
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Unanswered {
            value: ParamValue::I32(9)
        }
    );
    rig.tick_range(2550, 8000);
    assert_eq!(rig.set_times().len(), 3);
    assert_eq!(rig.table().value("P0"), Some(ParamValue::I32(-3)));
}

#[test]
fn an_echo_with_the_bytes_sent_confirms_the_write_and_updates_the_table() {
    let mut rig = complete_rig(2);
    rig.command(&set("P0", ParamValue::I32(9)), 1000);
    rig.deliver(i32_pv("P0", 0, 2, 9), 1200);
    assert!(rig.changed);
    assert!(rig.table().writes.is_empty());
    assert_eq!(rig.table().value("P0"), Some(ParamValue::I32(9)));
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    rig.tick_range(1250, 5000);
    assert_eq!(rig.set_times(), vec![1000]);
    assert!(rig.table().writes.is_empty());
}

#[test]
fn the_echo_is_matched_on_raw_bytes_not_on_decoded_numbers() {
    // Byte-wise U8 200 is C8 00 00 00; an echo with junk in the unused bytes is not the same bytes.
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(version(CAP_BYTEWISE), 1);
    rig.deliver(
        pv("U", 0, 1, MavParamType::MAV_PARAM_TYPE_UINT8, [1, 0, 0, 0]),
        2,
    );
    rig.command(&set("U", ParamValue::U8(200)), 10);
    rig.deliver(
        pv(
            "U",
            0,
            1,
            MavParamType::MAV_PARAM_TYPE_UINT8,
            [0xC8, 1, 0, 0],
        ),
        20,
    );
    assert!(matches!(
        rig.table().writes["U"],
        WriteStatus::Refused { .. }
    ));
    rig.tick_range(50, 1550);
    assert_eq!(rig.set_times(), vec![10]);
}

#[test]
fn a_new_set_for_the_same_name_replaces_the_old_one_and_restarts_its_clock() {
    let mut rig = complete_rig(2);
    rig.command(&set("P0", ParamValue::I32(9)), 1000);
    rig.command(&set("P0", ParamValue::I32(11)), 1100);
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Pending {
            value: ParamValue::I32(11)
        }
    );
    assert_eq!(rig.table().writes.len(), 1);
    rig.tick_range(1150, 2700);
    assert_eq!(rig.set_times(), vec![1000, 1100, 1600, 2100]);
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Unanswered {
            value: ParamValue::I32(11)
        }
    );
    // The resends carry the new value.
    let last = sets(&[rig
        .log
        .iter()
        .rev()
        .find(|(_, m)| matches!(m, Message::PARAM_SET(_)))
        .unwrap()
        .1
        .clone()])
    .into_iter()
    .map(|s| s.param_value.to_le_bytes())
    .collect::<Vec<_>>();
    assert_eq!(last, vec![11i32.to_le_bytes()]);
}

#[test]
fn a_refused_write_can_be_tried_again() {
    let mut rig = complete_rig(1);
    rig.command(&set("P0", ParamValue::I32(9)), 1000);
    rig.deliver(i32_pv("P0", 0, 1, -3), 1100);
    rig.tick_range(1150, 2500);
    assert!(matches!(
        rig.table().writes["P0"],
        WriteStatus::Refused { .. }
    ));
    rig.command(&set("P0", ParamValue::I32(9)), 2600);
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Pending {
            value: ParamValue::I32(9)
        }
    );
    rig.deliver(i32_pv("P0", 0, 1, 9), 2650);
    assert!(rig.table().writes.is_empty());
}

#[test]
fn writes_to_different_names_run_independently() {
    let mut rig = complete_rig(2);
    rig.command(&set("P0", ParamValue::I32(9)), 1000);
    rig.command(&set("P1", ParamValue::I32(8)), 1200);
    rig.deliver(i32_pv("P1", 1, 2, 8), 1300);
    assert!(matches!(
        rig.table().writes["P0"],
        WriteStatus::Pending { .. }
    ));
    assert!(!rig.table().writes.contains_key("P1"));
    rig.tick_range(1350, 1500);
    assert_eq!(rig.set_times(), vec![1000, 1200, 1500]);
}

#[test]
fn a_name_not_in_the_table_or_a_value_of_another_type_sends_nothing_and_changes_nothing() {
    let mut rig = complete_rig(2);
    let before = rig.table().clone();
    for command in [
        set("NOPE", ParamValue::I32(1)),
        set("P0", ParamValue::U8(1)),
        set("P0", ParamValue::F32(1.0)),
        set("p0", ParamValue::I32(1)),
    ] {
        let out = rig.command(&command, 500);
        assert!(out.is_empty(), "{command:?}");
        assert!(!rig.changed, "{command:?}");
        assert_eq!(*rig.table(), before, "{command:?}");
    }
    rig.tick_range(550, 4000);
    assert!(rig.set_times().is_empty());
}

fn read(name: &str) -> Command {
    Command::ReadParam { name: name.into() }
}

fn read_requests(messages: &[Message]) -> Vec<&PARAM_REQUEST_READ_DATA> {
    messages
        .iter()
        .filter_map(|m| match m {
            Message::PARAM_REQUEST_READ(r) => Some(r),
            _ => None,
        })
        .collect()
}

#[test]
fn a_read_sends_one_request_by_name_and_the_reply_lands_in_the_table() {
    let mut rig = complete_rig(2);
    let out = rig.command(&read("P1"), 200);
    let sent = read_requests(&out);
    assert_eq!(out.len(), 1);
    assert_eq!(sent.len(), 1);
    assert_eq!(sent[0].param_index, -1);
    assert_eq!(sent[0].param_id, id16("P1").into());
    assert_eq!((sent[0].target_system, sent[0].target_component), (1, 1));
    assert_eq!(rig.table().value("P1"), Some(ParamValue::I32(-3)));
    rig.deliver(i32_pv("P1", 1, 2, 41), 220);
    assert!(rig.changed);
    assert_eq!(rig.table().value("P1"), Some(ParamValue::I32(41)));
    assert_eq!(rig.table().value("P0"), Some(ParamValue::I32(-3)));
    rig.tick_range(250, 3000);
    assert_eq!(rig.read_times(), vec![200]);
    assert!(rig.table().writes.is_empty());
}

#[test]
fn a_read_nobody_answers_is_sent_three_times_500_ms_apart_then_dropped() {
    let mut rig = complete_rig(2);
    rig.command(&read("NO_SUCH_PARAM"), 1000);
    rig.tick_range(1050, 1499);
    assert_eq!(rig.read_times(), vec![1000]);
    rig.tick(1500);
    assert_eq!(rig.read_times(), vec![1000, 1500]);
    rig.tick_range(1550, 1999);
    rig.tick(2000);
    assert_eq!(rig.read_times(), vec![1000, 1500, 2000]);
    rig.tick_range(2050, 8000);
    assert_eq!(rig.read_times().len(), 3);
    assert_eq!(rig.table().params.len(), 2);
}

#[test]
fn an_answered_read_is_not_sent_again() {
    let mut rig = complete_rig(2);
    rig.command(&read("P0"), 1000);
    rig.tick(1200);
    rig.deliver(i32_pv("P0", 0, 2, 7), 1300);
    rig.tick_range(1350, 4000);
    assert_eq!(rig.read_times(), vec![1000]);
}

#[test]
fn unrelated_commands_are_ignored() {
    let mut rig = complete_rig(1);
    assert!(rig.command(&Command::StopLog, 500).is_empty());
    assert!(!rig.changed);
}

#[test]
fn a_differing_echo_after_the_send_is_the_refusal_at_once_and_nothing_is_sent_again() {
    let mut rig = complete_rig(2);
    rig.command(&set("P0", ParamValue::I32(9)), 1000);
    assert_eq!(rig.set_times(), vec![1000]);
    rig.deliver(i32_pv("P0", 0, 2, -3), 1010);
    assert!(rig.changed);
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Refused {
            value: ParamValue::I32(9),
            in_effect: ParamValue::I32(-3)
        }
    );
    assert_eq!(rig.table().value("P0"), Some(ParamValue::I32(-3)));
    rig.tick_range(1050, 8000);
    assert_eq!(rig.set_times(), vec![1000]);
    assert_eq!(
        sets(&rig.log.iter().map(|(_, m)| m.clone()).collect::<Vec<_>>()).len(),
        1
    );
    assert!(matches!(
        rig.table().writes["P0"],
        WriteStatus::Refused { .. }
    ));
}

#[test]
fn a_matching_echo_is_success_with_one_send() {
    let mut rig = complete_rig(2);
    rig.command(&set("P0", ParamValue::I32(9)), 1000);
    rig.deliver(i32_pv("P0", 0, 2, 9), 1010);
    assert!(rig.table().writes.is_empty());
    assert_eq!(rig.table().value("P0"), Some(ParamValue::I32(9)));
    rig.tick_range(1050, 8000);
    assert_eq!(rig.set_times(), vec![1000]);
    assert!(rig.table().writes.is_empty());
}

#[test]
fn a_value_for_another_parameter_changes_nothing_about_the_write() {
    let mut rig = complete_rig(2);
    rig.command(&set("P0", ParamValue::I32(9)), 1000);
    rig.deliver(i32_pv("P1", 1, 2, 77), 1010);
    assert_eq!(rig.table().value("P1"), Some(ParamValue::I32(77)));
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Pending {
            value: ParamValue::I32(9)
        }
    );
    rig.tick_range(1050, 1999);
    assert_eq!(rig.set_times(), vec![1000, 1500]);
    rig.deliver(i32_pv("P0", 0, 2, 9), 2000);
    assert!(rig.table().writes.is_empty());
}

#[test]
fn a_differing_value_before_the_param_set_was_sent_is_not_a_refusal() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(i32_pv("P0", 0, 1, -3), 5);
    rig.command(&set("P0", ParamValue::I32(9)), 10);
    assert!(rig.set_times().is_empty());
    rig.deliver(i32_pv("P0", 0, 1, -3), 20);
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Pending {
            value: ParamValue::I32(9)
        }
    );
    rig.deliver(version(CAP_BYTEWISE), 30);
    assert_eq!(rig.set_times(), vec![30]);
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Pending {
            value: ParamValue::I32(9)
        }
    );
}

#[test]
fn during_a_list_download_a_differing_value_is_not_a_refusal_and_the_resend_rule_decides() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(version(CAP_BYTEWISE | CAP_MAVLINK2), 10);
    rig.deliver(i32_pv("P0", 0, 3, -3), 20);
    assert!(matches!(
        rig.table().fetch,
        FetchStatus::Fetching { received: 1, .. }
    ));
    rig.command(&set("P0", ParamValue::I32(9)), 30);
    // A list frame carrying the old value.
    rig.deliver(i32_pv("P0", 0, 3, -3), 40);
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Pending {
            value: ParamValue::I32(9)
        }
    );
    rig.tick_range(50, 529);
    assert_eq!(rig.set_times(), vec![30]);
    rig.tick(530);
    rig.tick(1030);
    assert_eq!(rig.set_times(), vec![30, 530, 1030]);
    rig.tick(1530);
    assert_eq!(
        rig.table().writes["P0"],
        WriteStatus::Refused {
            value: ParamValue::I32(9),
            in_effect: ParamValue::I32(-3)
        }
    );
    assert_eq!(rig.set_times(), vec![30, 530, 1030]);
}

#[test]
fn during_a_list_download_the_equal_echo_still_confirms() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(version(CAP_BYTEWISE | CAP_MAVLINK2), 10);
    rig.deliver(i32_pv("P0", 0, 3, -3), 20);
    rig.command(&set("P0", ParamValue::I32(9)), 30);
    rig.deliver(i32_pv("P0", 0, 3, -3), 40);
    rig.deliver(i32_pv("P0", 0, 3, 9), 50);
    assert!(rig.table().writes.is_empty());
    assert_eq!(rig.set_times(), vec![30]);
}

#[test]
fn an_equal_echo_after_unanswered_or_refused_removes_the_entry() {
    let mut rig = complete_rig(2);
    rig.command(&set("P0", ParamValue::I32(9)), 1000);
    rig.tick_range(1050, 2500);
    assert!(matches!(
        rig.table().writes["P0"],
        WriteStatus::Unanswered { .. }
    ));
    rig.deliver(i32_pv("P0", 0, 2, 9), 3000);
    assert!(rig.changed);
    assert!(rig.table().writes.is_empty());
    assert_eq!(rig.table().value("P0"), Some(ParamValue::I32(9)));

    rig.command(&set("P1", ParamValue::I32(8)), 4000);
    rig.deliver(i32_pv("P1", 1, 2, 4), 4010);
    rig.tick_range(4050, 5500);
    assert!(matches!(
        rig.table().writes["P1"],
        WriteStatus::Refused { .. }
    ));
    // Another differing value does not remove it; the equal one does.
    rig.deliver(i32_pv("P1", 1, 2, 2), 6000);
    assert!(matches!(
        rig.table().writes["P1"],
        WriteStatus::Refused { .. }
    ));
    rig.deliver(i32_pv("P1", 1, 2, 8), 6100);
    assert!(rig.table().writes.is_empty());
}

#[test]
fn a_write_waits_for_the_encoding_and_is_then_sent_c_cast() {
    let mut rig = Rig::with_vehicle(1, 1, ARDUPILOT);
    rig.connect(0);
    rig.deliver(
        pv(
            "F",
            0,
            1,
            MavParamType::MAV_PARAM_TYPE_UINT8,
            200.0f32.to_le_bytes(),
        ),
        2,
    );
    assert!(!rig.table().encoding_known);
    let out = rig.command(&set("F", ParamValue::U8(7)), 50);
    assert!(sets(&out).is_empty());
    assert!(rig.changed);
    assert_eq!(
        rig.table().writes["F"],
        WriteStatus::Pending {
            value: ParamValue::U8(7)
        }
    );
    rig.tick_range(100, 400);
    assert!(rig.set_times().is_empty());
    let out = rig.deliver(version(CAP_C_CAST), 450);
    let sent = sets(&out);
    assert_eq!(sent.len(), 1);
    assert_eq!(sent[0].param_value.to_le_bytes(), [0x00, 0x00, 0xE0, 0x40]);
    assert_eq!(sent[0].param_type, MavParamType::MAV_PARAM_TYPE_UINT8);
    rig.tick_range(500, 1900);
    assert_eq!(rig.set_times(), vec![450, 950, 1450]);
    assert!(matches!(
        rig.table().writes["F"],
        WriteStatus::Pending { .. }
    ));
    rig.tick(1950);
    assert_eq!(
        rig.table().writes["F"],
        WriteStatus::Unanswered {
            value: ParamValue::U8(7)
        }
    );
}

#[test]
fn a_write_waiting_for_the_encoding_is_sent_when_the_fallback_applies() {
    let mut rig = Rig::with_vehicle(1, 1, ARDUPILOT);
    rig.connect(0);
    rig.deliver(
        pv(
            "F",
            0,
            1,
            MavParamType::MAV_PARAM_TYPE_UINT8,
            200.0f32.to_le_bytes(),
        ),
        2,
    );
    rig.command(&set("F", ParamValue::U8(7)), 50);
    rig.tick_range(100, 1450);
    assert!(rig.set_times().is_empty());
    rig.tick(1500);
    assert_eq!(rig.set_times(), vec![1500]);
    let sent: Vec<[u8; 4]> = rig
        .log
        .iter()
        .filter_map(|(_, m)| match m {
            Message::PARAM_SET(s) => Some(s.param_value.to_le_bytes()),
            _ => None,
        })
        .collect();
    assert_eq!(sent, vec![[0x00, 0x00, 0xE0, 0x40]]);
}

#[test]
fn a_vehicle_with_no_parameters_completes_with_an_empty_table() {
    let mut rig = Rig::new();
    rig.connect(0);
    rig.deliver(i32_pv("X", 0, 0, 1), 10);
    assert!(rig.changed);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    assert!(rig.table().params.is_empty());
    rig.tick_range(50, 5000);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    assert_eq!(rig.list_times().len(), 1);
    assert!(rig.read_times().is_empty());
}

// ---------------------------------------------------------------- scripted fake vehicle

#[derive(Clone)]
struct FakeParam {
    name: String,
    ty: MavParamType,
    /// The 4 bytes of the value field, kept exactly as the vehicle holds them.
    raw: [u8; 4],
}

enum Queued {
    Value(usize),
    Version,
}

/// A vehicle written from the specification. It answers requests one frame per 2 ms and keeps its
/// own values as 4 opaque bytes; it never uses the crate's `encode` / `decode`.
struct Fake {
    system_id: u8,
    params: Vec<FakeParam>,
    /// The count it reports.
    count: u16,
    /// Capability bits in the reply to the AUTOPILOT_VERSION request; `None`: no reply.
    capabilities: Option<u64>,
    silent: bool,
    ignore_sets: bool,
    refuse: Vec<String>,
    never: BTreeSet<u16>,
    /// Drops every fifth frame it would send.
    drop_fifth: bool,
    emitted: u32,
    dropped: u32,
    queue: Vec<(u64, Queued)>,
    next_free: u64,
    lists_seen: u32,
}

impl Fake {
    fn new(params: Vec<FakeParam>) -> Self {
        Self {
            system_id: 1,
            count: params.len() as u16,
            params,
            capabilities: Some(CAP_BYTEWISE | CAP_MAVLINK2),
            silent: false,
            ignore_sets: false,
            refuse: Vec::new(),
            never: BTreeSet::new(),
            drop_fifth: false,
            emitted: 0,
            dropped: 0,
            queue: Vec::new(),
            next_free: 0,
            lists_seen: 0,
        }
    }

    /// `n` parameters: I32, U8 and F32 in turn, each with its own byte pattern.
    fn numbered(n: usize) -> Self {
        let params = (0..n)
            .map(|i| match i % 3 {
                0 => FakeParam {
                    name: format!("I_{i}"),
                    ty: MavParamType::MAV_PARAM_TYPE_INT32,
                    raw: [i as u8, (i >> 8) as u8, 0xFF, 0xFF],
                },
                1 => FakeParam {
                    name: format!("B_{i}"),
                    ty: MavParamType::MAV_PARAM_TYPE_UINT8,
                    raw: [(i % 251) as u8, 0, 0, 0],
                },
                _ => FakeParam {
                    name: format!("F_{i}"),
                    ty: MavParamType::MAV_PARAM_TYPE_REAL32,
                    raw: (i as f32 * 0.5).to_le_bytes(),
                },
            })
            .collect();
        Self::new(params)
    }

    fn push(&mut self, now: u64, what: Queued) {
        let due = self.next_free.max(now + 2);
        self.next_free = due + 2;
        self.queue.push((due, what));
    }

    fn receive(&mut self, messages: &[Message], now: u64) {
        if self.silent {
            return;
        }
        for message in messages {
            match message {
                Message::PARAM_REQUEST_LIST(r) if r.target_system == self.system_id => {
                    self.lists_seen += 1;
                    for i in 0..self.params.len() {
                        if !self.never.contains(&(i as u16)) {
                            self.push(now, Queued::Value(i));
                        }
                    }
                }
                Message::PARAM_REQUEST_READ(r) if r.target_system == self.system_id => {
                    let index = r.param_index as usize;
                    if index < self.params.len() && !self.never.contains(&(index as u16)) {
                        self.push(now, Queued::Value(index));
                    }
                }
                Message::PARAM_SET(s) if s.target_system == self.system_id => {
                    if self.ignore_sets {
                        continue;
                    }
                    let id: [u8; 16] = *s.param_id;
                    let end = id.iter().position(|&b| b == 0).unwrap_or(16);
                    let Some(i) = self
                        .params
                        .iter()
                        .position(|p| p.name.as_bytes() == &id[..end])
                    else {
                        continue;
                    };
                    if !self.refuse.contains(&self.params[i].name) {
                        self.params[i].raw = s.param_value.to_le_bytes();
                    }
                    self.push(now, Queued::Value(i));
                }
                Message::COMMAND_LONG(c)
                    if c.target_system == self.system_id
                        && c.command == MavCmd::MAV_CMD_REQUEST_MESSAGE
                        && c.param1 == 148.0
                        && self.capabilities.is_some() =>
                {
                    self.push(now, Queued::Version);
                }
                _ => {}
            }
        }
    }

    fn due(&mut self, now: u64) -> Vec<Message> {
        let mut ready: Vec<(u64, Queued)> = Vec::new();
        let mut i = 0;
        while i < self.queue.len() {
            if self.queue[i].0 <= now {
                ready.push(self.queue.remove(i));
            } else {
                i += 1;
            }
        }
        ready.sort_by_key(|(due, _)| *due);
        let mut out = Vec::new();
        for (_, what) in ready {
            self.emitted += 1;
            if self.drop_fifth && self.emitted.is_multiple_of(5) {
                self.dropped += 1;
                continue;
            }
            out.push(match what {
                Queued::Version => version(self.capabilities.unwrap_or(0)),
                Queued::Value(i) => {
                    let p = &self.params[i];
                    pv(&p.name, i as u16, self.count, p.ty, p.raw)
                }
            });
        }
        out
    }
}

/// Runs the rig against the fake every 2 ms from `from` to `to` inclusive (both even).
fn run(rig: &mut Rig, fake: &mut Fake, from: u64, to: u64) {
    let mut t = from;
    while t <= to {
        for message in fake.due(t) {
            let sent = rig.deliver(message, t);
            fake.receive(&sent, t);
        }
        let sent = rig.tick(t);
        fake.receive(&sent, t);
        t += 2;
    }
}

fn start(rig: &mut Rig, fake: &mut Fake, t: u64) {
    let sent = rig.connect(t);
    fake.receive(&sent, t);
}

fn assert_table_matches(rig: &Rig, fake: &Fake) {
    let table = rig.table();
    assert_eq!(table.params.len(), fake.params.len());
    for (i, p) in fake.params.iter().enumerate() {
        let stored = table
            .params
            .get(&p.name)
            .unwrap_or_else(|| panic!("{} missing", p.name));
        assert_eq!(stored.raw, p.raw, "{}", p.name);
        assert_eq!(stored.index, i as u16, "{}", p.name);
    }
}

#[test]
fn fake_a_list_of_300_completes_while_a_fifth_of_the_vehicles_frames_are_dropped() {
    let mut rig = Rig::new();
    let mut fake = Fake::numbered(300);
    fake.drop_fifth = true;
    start(&mut rig, &mut fake, 0);
    run(&mut rig, &mut fake, 2, 40_000);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    assert_table_matches(&rig, &fake);
    assert!(fake.dropped >= 60, "dropped only {}", fake.dropped);
    assert!(
        !rig.read_times().is_empty(),
        "the gaps were never requested"
    );
    assert_eq!(rig.list_times(), vec![0]);
    assert_eq!(fake.lists_seen, 1);
    assert!(rig.table().encoding_known);
    assert_eq!(rig.table().encoding, ParamEncoding::Bytewise);
}

#[test]
fn fake_a_clean_list_of_300_needs_no_read_requests() {
    let mut rig = Rig::new();
    let mut fake = Fake::numbered(300);
    start(&mut rig, &mut fake, 0);
    run(&mut rig, &mut fake, 2, 3000);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    assert_table_matches(&rig, &fake);
    assert!(rig.read_times().is_empty());
}

#[test]
fn fake_nothing_answered_fails_after_three_sends_one_second_apart() {
    let mut rig = Rig::new();
    let mut fake = Fake::numbered(5);
    fake.silent = true;
    start(&mut rig, &mut fake, 0);
    run(&mut rig, &mut fake, 2, 2998);
    assert_eq!(rig.list_times(), vec![0, 1000, 2000]);
    assert!(matches!(rig.table().fetch, FetchStatus::Fetching { .. }));
    run(&mut rig, &mut fake, 3000, 3000);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Failed("no answer to the parameter request".into())
    );
    run(&mut rig, &mut fake, 3002, 10_000);
    assert_eq!(rig.list_times(), vec![0, 1000, 2000]);
}

#[test]
fn fake_indices_that_never_come_fail_with_the_count_and_keep_what_arrived() {
    let mut rig = Rig::new();
    let mut fake = Fake::numbered(40);
    fake.never = BTreeSet::from([5, 17]);
    start(&mut rig, &mut fake, 0);
    run(&mut rig, &mut fake, 2, 10_000);
    assert_eq!(
        rig.table().fetch,
        FetchStatus::Failed("2 of 40 parameters missing".into())
    );
    assert_eq!(rig.table().params.len(), 38);
    assert!(!rig.table().params.contains_key("F_5"));
    let reads = rig.reads();
    assert_eq!(reads.len(), 16);
    let indices: Vec<i16> = reads.iter().map(|(_, i)| *i).collect();
    assert_eq!(indices, [5, 17].repeat(8));
    // The list ends with the value at index 39, 2 ms after the request is answered; one second of
    // silence follows, then eight rounds 500 ms apart.
    let first = reads[0].0;
    assert!((1000..=1100).contains(&first), "first round at {first}");
    assert_eq!(reads[2].0, first + 500);
    assert_eq!(reads[4].0, first + 1000);
}

#[test]
fn fake_a_write_is_confirmed() {
    let mut rig = Rig::new();
    let mut fake = Fake::numbered(30);
    start(&mut rig, &mut fake, 0);
    run(&mut rig, &mut fake, 2, 400);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    let sent = rig.command(&set("I_3", ParamValue::I32(-5)), 402);
    fake.receive(&sent, 402);
    run(&mut rig, &mut fake, 404, 3000);
    assert!(rig.table().writes.is_empty());
    assert_eq!(rig.table().value("I_3"), Some(ParamValue::I32(-5)));
    assert_eq!(rig.set_times(), vec![402]);
    assert_eq!(fake.params[3].raw, [0xFB, 0xFF, 0xFF, 0xFF]);
}

#[test]
fn fake_a_refused_write_shows_the_value_the_vehicle_kept() {
    let mut rig = Rig::new();
    let mut fake = Fake::numbered(30);
    fake.refuse = vec!["B_4".into()];
    start(&mut rig, &mut fake, 0);
    run(&mut rig, &mut fake, 2, 400);
    let before = rig.table().value("B_4");
    assert_eq!(before, Some(ParamValue::U8(4)));
    let sent = rig.command(&set("B_4", ParamValue::U8(200)), 402);
    fake.receive(&sent, 402);
    run(&mut rig, &mut fake, 404, 3000);
    assert_eq!(
        rig.table().writes["B_4"],
        WriteStatus::Refused {
            value: ParamValue::U8(200),
            in_effect: ParamValue::U8(4)
        }
    );
    assert_eq!(rig.table().value("B_4"), Some(ParamValue::U8(4)));
    assert_eq!(rig.set_times(), vec![402]);
}

#[test]
fn fake_an_unanswered_write_is_unanswered_after_three_sends() {
    let mut rig = Rig::new();
    let mut fake = Fake::numbered(30);
    fake.ignore_sets = true;
    start(&mut rig, &mut fake, 0);
    run(&mut rig, &mut fake, 2, 400);
    let sent = rig.command(&set("F_5", ParamValue::F32(2.5)), 402);
    fake.receive(&sent, 402);
    run(&mut rig, &mut fake, 404, 3000);
    assert_eq!(
        rig.table().writes["F_5"],
        WriteStatus::Unanswered {
            value: ParamValue::F32(2.5)
        }
    );
    assert_eq!(rig.set_times(), vec![402, 902, 1402]);
    assert_eq!(rig.table().value("F_5"), Some(ParamValue::F32(2.5)));
}

#[test]
fn fake_a_16_character_name_lists_and_writes() {
    let mut rig = Rig::new();
    let mut fake = Fake::numbered(6);
    fake.params[2].name = "ABCDEFGHIJKLMNOP".into();
    fake.params[2].ty = MavParamType::MAV_PARAM_TYPE_INT32;
    fake.params[2].raw = [1, 0, 0, 0];
    start(&mut rig, &mut fake, 0);
    run(&mut rig, &mut fake, 2, 300);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    assert_eq!(
        rig.table().value("ABCDEFGHIJKLMNOP"),
        Some(ParamValue::I32(1))
    );
    let sent = rig.command(&set("ABCDEFGHIJKLMNOP", ParamValue::I32(2)), 302);
    let [set_message] = sets(&sent)[..] else {
        panic!("{sent:?}");
    };
    assert_eq!(*set_message.param_id, *b"ABCDEFGHIJKLMNOP");
    fake.receive(&sent, 302);
    run(&mut rig, &mut fake, 304, 1000);
    assert!(rig.table().writes.is_empty());
    assert_eq!(
        rig.table().value("ABCDEFGHIJKLMNOP"),
        Some(ParamValue::I32(2))
    );
    assert_eq!(fake.params[2].raw, [2, 0, 0, 0]);
}

#[test]
fn fake_a_changed_count_restarts_and_the_second_list_completes() {
    let mut rig = Rig::new();
    let mut fake = Fake::numbered(10);
    start(&mut rig, &mut fake, 0);
    run(&mut rig, &mut fake, 2, 10);
    assert!(matches!(rig.table().fetch, FetchStatus::Fetching { .. }));
    let extra = Fake::numbered(12);
    fake.params.extend_from_slice(&extra.params[10..]);
    fake.count = 12;
    run(&mut rig, &mut fake, 12, 3000);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    assert_table_matches(&rig, &fake);
    assert_eq!(rig.list_times().len(), 2);
    assert_eq!(fake.lists_seen, 2);
}

fn encoding_outcome(
    autopilot: &str,
    capabilities: Option<u64>,
    run_to: u64,
) -> (bool, ParamEncoding, usize) {
    let mut rig = Rig::with_vehicle(1, 1, autopilot);
    let mut fake = Fake::numbered(3);
    fake.capabilities = capabilities;
    start(&mut rig, &mut fake, 0);
    run(&mut rig, &mut fake, 2, run_to);
    assert_eq!(rig.table().fetch, FetchStatus::Complete);
    (
        rig.table().encoding_known,
        rig.table().encoding,
        rig.version_times().len(),
    )
}

#[test]
fn fake_the_four_encoding_outcomes() {
    // Capability bit byte-wise, from a vehicle that looks like ArduPilot: the bit wins.
    assert_eq!(
        encoding_outcome(ARDUPILOT, Some(CAP_BYTEWISE | CAP_MAVLINK2), 100),
        (true, ParamEncoding::Bytewise, 1)
    );
    // Capability bit C-cast only, from a vehicle that does not look like ArduPilot.
    assert_eq!(
        encoding_outcome(GENERIC, Some(CAP_C_CAST), 100),
        (true, ParamEncoding::CCast, 1)
    );
    // Neither bit: the autopilot decides, at once.
    assert_eq!(
        encoding_outcome(ARDUPILOT, Some(CAP_MAVLINK2), 100),
        (true, ParamEncoding::CCast, 1)
    );
    assert_eq!(
        encoding_outcome(GENERIC, Some(CAP_MAVLINK2), 100),
        (true, ParamEncoding::Bytewise, 1)
    );
    // No reply at all: not known at 1498 ms, fallback at 1500 after the third send.
    assert_eq!(
        encoding_outcome(ARDUPILOT, None, 1498),
        (false, ParamEncoding::Bytewise, 3)
    );
    assert_eq!(
        encoding_outcome(ARDUPILOT, None, 2500),
        (true, ParamEncoding::CCast, 3)
    );
    assert_eq!(
        encoding_outcome(GENERIC, None, 2500),
        (true, ParamEncoding::Bytewise, 3)
    );
}
