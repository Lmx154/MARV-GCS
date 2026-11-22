//! The parameter protocol, status texts, frame-loss injection and the silent switch, driven with
//! virtual time and read back with the mavlink library's own reader.

use gcs_sim::{ParamEncoding, ParamValue, SimConfig, SimParam, SimVehicle};
use mavlink::dialects::common::{
    HEARTBEAT_DATA, MavAutopilot, MavMessage, MavParamType, MavSeverity, MavType,
    PARAM_REQUEST_LIST_DATA, PARAM_REQUEST_READ_DATA, PARAM_SET_DATA, PARAM_VALUE_DATA,
};
use mavlink::types::CharArray;
use mavlink::{MAVLinkV2MessageRaw, MavHeader, MavlinkReader};

fn decode(bytes: &[u8]) -> Vec<(MavHeader, MavMessage)> {
    let mut reader = MavlinkReader::new(bytes);
    let mut out = Vec::new();
    while let Ok(item) = reader.read_any_message::<MavMessage>() {
        out.push(item);
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

fn request_list(target_system: u8, target_component: u8) -> Vec<u8> {
    encode(&MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
        target_system,
        target_component,
    }))
}

fn request_read(index: i16, name: &str) -> Vec<u8> {
    encode(&MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
        param_index: index,
        target_system: 1,
        target_component: 1,
        param_id: CharArray::from(name),
    }))
}

fn set(name: &str, field: f32, param_type: MavParamType) -> Vec<u8> {
    encode(&MavMessage::PARAM_SET(PARAM_SET_DATA {
        param_value: field,
        target_system: 1,
        target_component: 1,
        param_id: CharArray::from(name),
        param_type,
    }))
}

fn heartbeat() -> Vec<u8> {
    encode(&MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        mavlink_version: 3,
        ..HEARTBEAT_DATA::default()
    }))
}

fn values(bytes: &[u8]) -> Vec<PARAM_VALUE_DATA> {
    decode(bytes)
        .into_iter()
        .filter_map(|(_, m)| match m {
            MavMessage::PARAM_VALUE(d) => Some(d),
            _ => None,
        })
        .collect()
}

fn texts(bytes: &[u8]) -> Vec<(MavSeverity, String)> {
    decode(bytes)
        .into_iter()
        .filter_map(|(_, m)| match m {
            MavMessage::STATUSTEXT(d) => Some((d.severity, d.text.to_str().unwrap().to_string())),
            _ => None,
        })
        .collect()
}

/// The `param_id` as the receiver must read it: up to the first NUL, or all 16 bytes.
fn name(d: &PARAM_VALUE_DATA) -> String {
    let raw: &[u8; 16] = &d.param_id;
    let end = raw.iter().position(|&b| b == 0).unwrap_or(16);
    String::from_utf8(raw[..end].to_vec()).unwrap()
}

/// What the receiver reads from a PARAM_VALUE, by the encoding rules of the specification.
fn read_value(d: &PARAM_VALUE_DATA, encoding: ParamEncoding) -> ParamValue {
    let b = d.param_value.to_le_bytes();
    match (encoding, d.param_type) {
        (ParamEncoding::Bytewise, MavParamType::MAV_PARAM_TYPE_UINT8) => ParamValue::Uint8(b[0]),
        (ParamEncoding::Bytewise, MavParamType::MAV_PARAM_TYPE_INT8) => {
            ParamValue::Int8(b[0] as i8)
        }
        (ParamEncoding::Bytewise, MavParamType::MAV_PARAM_TYPE_UINT16) => {
            ParamValue::Uint16(u16::from_le_bytes([b[0], b[1]]))
        }
        (ParamEncoding::Bytewise, MavParamType::MAV_PARAM_TYPE_INT16) => {
            ParamValue::Int16(i16::from_le_bytes([b[0], b[1]]))
        }
        (ParamEncoding::Bytewise, MavParamType::MAV_PARAM_TYPE_UINT32) => {
            ParamValue::Uint32(u32::from_le_bytes(b))
        }
        (ParamEncoding::Bytewise, MavParamType::MAV_PARAM_TYPE_INT32) => {
            ParamValue::Int32(i32::from_le_bytes(b))
        }
        (ParamEncoding::CCast, MavParamType::MAV_PARAM_TYPE_UINT8) => {
            ParamValue::Uint8(d.param_value as u8)
        }
        (ParamEncoding::CCast, MavParamType::MAV_PARAM_TYPE_INT8) => {
            ParamValue::Int8(d.param_value as i8)
        }
        (ParamEncoding::CCast, MavParamType::MAV_PARAM_TYPE_UINT16) => {
            ParamValue::Uint16(d.param_value as u16)
        }
        (ParamEncoding::CCast, MavParamType::MAV_PARAM_TYPE_INT16) => {
            ParamValue::Int16(d.param_value as i16)
        }
        (ParamEncoding::CCast, MavParamType::MAV_PARAM_TYPE_UINT32) => {
            ParamValue::Uint32(d.param_value as u32)
        }
        (ParamEncoding::CCast, MavParamType::MAV_PARAM_TYPE_INT32) => {
            ParamValue::Int32(d.param_value as i32)
        }
        (_, MavParamType::MAV_PARAM_TYPE_REAL32) => ParamValue::Real32(d.param_value),
        other => panic!("unexpected type {other:?}"),
    }
}
/// The parameters of a default configuration: the 13 of the first block, the 44 of the RC block, the 13 of the sensor block and the 23 of indices 70 to 92 (`fwparams`). A listing takes 2 ms a parameter.
/// The parameters of a default configuration: the 13 of the first block, the 44 of the RC block and the 13 of the sensor block.
const DEFAULT_PARAMS: usize = 13 + 44 + 13 + 23;

fn vehicle_with(encoding: ParamEncoding) -> SimVehicle {
    SimVehicle::new(SimConfig {
        param_encoding: encoding,
        ..SimConfig::default()
    })
}

/// Steps 1 ms at a time from `from_ms` to `to_ms` (inclusive); `first` goes in the first step.
fn run(vehicle: &mut SimVehicle, from_ms: u64, to_ms: u64, first: &[u8]) -> Vec<u8> {
    let mut bytes = Vec::new();
    for t in from_ms..=to_ms {
        bytes.extend(vehicle.step(t, if t == from_ms { first } else { &[] }));
    }
    bytes
}

/// Boots the vehicle so that the boot text is out of the way.
fn booted(config: SimConfig) -> SimVehicle {
    let mut vehicle = SimVehicle::new(config);
    vehicle.step(0, &[]);
    vehicle
}

#[test]
fn default_parameters_are_int32_and_real32_with_the_special_cases() {
    let params = SimConfig::default().params;
    assert_eq!(params.len(), DEFAULT_PARAMS);
    assert!(
        params
            .iter()
            .all(|p| matches!(p.value, ParamValue::Int32(_) | ParamValue::Real32(_))),
        "the contract allows only INT32 and REAL32"
    );
    assert!(
        params
            .iter()
            .any(|p| matches!(p.value, ParamValue::Int32(_)))
    );
    assert!(
        params
            .iter()
            .any(|p| matches!(p.value, ParamValue::Real32(_)))
    );
    let uid: Vec<_> = params.iter().filter(|p| p.read_only).collect();
    assert_eq!(uid.len(), 1);
    assert_eq!(uid[0].name, "SYS_UID");
    assert_eq!(uid[0].value, ParamValue::Int32(1_592_460_852));
    assert!(params.iter().any(|p| p.name.len() == 16));
    assert!(
        params
            .iter()
            .all(|p| p.name.is_ascii() && p.name.len() <= 16)
    );
}

#[test]
fn with_every_param_type_adds_one_of_each_other_type() {
    let config = SimConfig::default().with_every_param_type();
    assert_eq!(config.params.len(), DEFAULT_PARAMS + 5);
    let added: Vec<_> = config.params[DEFAULT_PARAMS..]
        .iter()
        .map(|p| (p.name.as_str(), p.value, p.read_only))
        .collect();
    assert_eq!(
        added,
        [
            ("TYPE_U8", ParamValue::Uint8(200), false),
            ("TYPE_I8", ParamValue::Int8(-3), false),
            ("TYPE_U16", ParamValue::Uint16(50_000), false),
            ("TYPE_I16", ParamValue::Int16(-320), false),
            ("TYPE_U32", ParamValue::Uint32(3_735_928_559), false),
        ]
    );
    assert_eq!(
        config.params[..DEFAULT_PARAMS],
        SimConfig::default().params[..]
    );

    for encoding in [ParamEncoding::Bytewise, ParamEncoding::CCast] {
        let mut vehicle = SimVehicle::new(SimConfig {
            param_encoding: encoding,
            ..config.clone()
        });
        let got = values(&run(&mut vehicle, 0, 220, &request_list(1, 1)));
        assert_eq!(got.len(), DEFAULT_PARAMS + 5);
        for (d, want) in got.iter().zip(&config.params) {
            assert_eq!(name(d), want.name);
            if encoding == ParamEncoding::CCast && want.name == "SYS_UID" {
                continue;
            }
            if encoding == ParamEncoding::CCast && want.name == "TYPE_U32" {
                assert_eq!(
                    read_value(d, encoding),
                    ParamValue::Uint32(3_735_928_576),
                    "float cast"
                );
                continue;
            }
            assert_eq!(read_value(d, encoding), want.value, "{}", want.name);
        }
    }
}

#[test]
fn bytewise_values_decode_back_exactly() {
    let mut vehicle = vehicle_with(ParamEncoding::Bytewise);
    let bytes = run(&mut vehicle, 0, 220, &request_list(1, 1));
    let got = values(&bytes);
    let defaults = SimConfig::default().params;
    assert_eq!(got.len(), defaults.len());
    for (d, want) in got.iter().zip(&defaults) {
        assert_eq!(name(d), want.name);
        assert_eq!(
            read_value(d, ParamEncoding::Bytewise),
            want.value,
            "{}",
            want.name
        );
    }
    let field = |n: &str| got.iter().find(|d| name(d) == n).unwrap().param_value;
    // Own bytes in the low bytes, the rest zero: no sign extension.
    assert_eq!(field("SYS_TRIM").to_bits(), 0xFFFF_FFFD);
    assert_eq!(field("HIGHG_RANGE").to_bits(), 320);
    assert_eq!(field("SYS_UID").to_bits(), 1_592_460_852);
    assert_eq!(field("IMU_GYR_LPF_HZ"), 98.0);
}

#[test]
fn ccast_values_are_float_conversions() {
    let mut vehicle = vehicle_with(ParamEncoding::CCast);
    let bytes = run(&mut vehicle, 0, 220, &request_list(1, 1));
    let got = values(&bytes);
    let defaults = SimConfig::default().params;
    assert_eq!(got.len(), defaults.len());
    for (d, want) in got.iter().zip(&defaults) {
        if want.name == "SYS_UID" {
            // More than 24 bits: the float cast is what comes out.
            assert_eq!(d.param_value, 1_592_460_852i32 as f32);
            assert_eq!(d.param_value, 1_592_460_800.0);
            assert_eq!(
                read_value(d, ParamEncoding::CCast),
                ParamValue::Int32(1_592_460_800)
            );
        } else {
            assert_eq!(
                read_value(d, ParamEncoding::CCast),
                want.value,
                "{}",
                want.name
            );
        }
    }
    let field = |n: &str| got.iter().find(|d| name(d) == n).unwrap().param_value;
    assert_eq!(field("SYS_TRIM"), -3.0);
    assert_eq!(field("IMU_RATE_HZ"), 800.0);
    assert_eq!(field("NAV_TEST_16CHARS"), 1.5);
}

#[test]
fn param_type_always_states_the_real_type() {
    for encoding in [ParamEncoding::Bytewise, ParamEncoding::CCast] {
        let mut vehicle = SimVehicle::new(SimConfig {
            param_encoding: encoding,
            ..SimConfig::default().with_every_param_type()
        });
        let got = values(&run(&mut vehicle, 0, 220, &request_list(0, 0)));
        let ty = |n: &str| got.iter().find(|d| name(d) == n).unwrap().param_type;
        for n in [
            "IMU_RATE_HZ",
            "IMU_ACC_RANGE",
            "SYS_TRIM",
            "BARO_RATE_HZ",
            "HIGHG_RANGE",
            "SYS_UID",
        ] {
            assert_eq!(ty(n), MavParamType::MAV_PARAM_TYPE_INT32, "{n}");
        }
        assert_eq!(ty("BAT_LOW_V"), MavParamType::MAV_PARAM_TYPE_REAL32);
        assert_eq!(ty("TYPE_U8"), MavParamType::MAV_PARAM_TYPE_UINT8);
        assert_eq!(ty("TYPE_I8"), MavParamType::MAV_PARAM_TYPE_INT8);
        assert_eq!(ty("TYPE_U16"), MavParamType::MAV_PARAM_TYPE_UINT16);
        assert_eq!(ty("TYPE_I16"), MavParamType::MAV_PARAM_TYPE_INT16);
        assert_eq!(ty("TYPE_U32"), MavParamType::MAV_PARAM_TYPE_UINT32);
    }
}

#[test]
fn list_is_complete_in_order_and_paced_while_telemetry_flows() {
    let mut vehicle = booted(SimConfig::default().with_synthetic_params(500));
    let total = DEFAULT_PARAMS + 500;
    let mut per_step = Vec::new();
    let mut all = Vec::new();
    let mut telemetry = 0;
    for t in 1..=1400u64 {
        let input = if t == 100 {
            request_list(1, 1)
        } else {
            Vec::new()
        };
        let bytes = vehicle.step(t, &input);
        let n = values(&bytes).len();
        per_step.push(n);
        all.extend(values(&bytes));
        if t >= 100 && t < 100 + 2 * total as u64 {
            telemetry += decode(&bytes)
                .iter()
                .filter(|(_, m)| matches!(m, MavMessage::HIGHRES_IMU(d) if d.id == 0))
                .count();
        }
    }
    assert!(per_step.iter().all(|&n| n <= 1), "more than one per ms");
    assert_eq!(all.len(), total);
    for (i, d) in all.iter().enumerate() {
        assert_eq!(usize::from(d.param_index), i);
        assert_eq!(usize::from(d.param_count), total);
    }
    assert_eq!(name(&all[DEFAULT_PARAMS]), "P0000");
    assert_eq!(name(&all[total - 1]), "P0499");
    // About 1148 ms (574 parameters, one per 2 ms) at 25 Hz.
    assert!((26..=31).contains(&telemetry), "{telemetry} IMU messages");
}

#[test]
fn list_pace_holds_with_coarse_steps() {
    let mut vehicle = booted(SimConfig::default());
    let first = values(&vehicle.step(100, &request_list(1, 1)));
    assert_eq!(first.len(), 1);
    let next = values(&vehicle.step(105, &[]));
    assert_eq!(next.len(), 2, "one per 2 ms: due at 102 and 104");
    assert_eq!(next[0].param_index, 1);
}

#[test]
fn list_addressing() {
    for (system, component, expect) in [
        (1, 1, true),
        (0, 0, true),
        (1, 0, true),
        (0, 1, true),
        (2, 1, false),
        (1, 2, false),
        (2, 0, false),
    ] {
        let mut vehicle = booted(SimConfig::default());
        let bytes = run(&mut vehicle, 1, 40, &request_list(system, component));
        assert_eq!(!values(&bytes).is_empty(), expect, "{system}/{component}");
    }
}

#[test]
fn read_by_index_and_by_name_including_sixteen_characters() {
    for encoding in [ParamEncoding::Bytewise, ParamEncoding::CCast] {
        let mut vehicle = vehicle_with(encoding);
        vehicle.step(0, &[]);
        let by_index = values(&vehicle.step(1, &request_read(3, "")));
        assert_eq!(by_index.len(), 1);
        assert_eq!(name(&by_index[0]), "HIGHG_RANGE");
        assert_eq!(by_index[0].param_index, 3);
        assert_eq!(by_index[0].param_count, DEFAULT_PARAMS as u16);
        assert_eq!(read_value(&by_index[0], encoding), ParamValue::Int32(320));

        // The name is used only when the index is -1; an index wins otherwise.
        let index_wins = values(&vehicle.step(2, &request_read(0, "SYS_TRIM")));
        assert_eq!(name(&index_wins[0]), "IMU_RATE_HZ");

        let by_name = values(&vehicle.step(3, &request_read(-1, "NAV_TEST_16CHARS")));
        assert_eq!(by_name.len(), 1);
        assert_eq!(name(&by_name[0]), "NAV_TEST_16CHARS");
        assert_eq!(by_name[0].param_index, 12);
        assert_eq!(
            by_name[0].param_id[15], b'S',
            "no terminator at 16 characters"
        );
        assert_eq!(by_name[0].param_value, 1.5);

        let short = values(&vehicle.step(4, &request_read(-1, "SYS_TRIM")));
        assert_eq!(name(&short[0]), "SYS_TRIM");
        assert_eq!(short[0].param_id[8], 0);
        assert_eq!(read_value(&short[0], encoding), ParamValue::Int32(-3));
    }
}

#[test]
fn read_of_unknown_name_or_index_gets_no_reply() {
    let mut vehicle = booted(SimConfig::default());
    for (t, frame) in [
        request_read(-1, "NO_SUCH_PARAM"),
        request_read(-1, "SYS_TRI"),
        request_read(-1, "SYS_TRIMX"),
        request_read(DEFAULT_PARAMS as i16, ""),
        request_read(9999, ""),
        request_read(-2, ""),
        request_read(-1, ""),
    ]
    .iter()
    .enumerate()
    {
        let bytes = vehicle.step(1 + t as u64, frame);
        assert!(values(&bytes).is_empty(), "request {t}");
    }
    let last = values(&vehicle.step(20, &request_read(12, "")));
    assert_eq!(last.len(), 1);
}

#[test]
fn set_stores_the_value_and_replies_without_a_text() {
    for encoding in [ParamEncoding::Bytewise, ParamEncoding::CCast] {
        let mut vehicle = vehicle_with(encoding);
        vehicle.step(0, &[]);
        let field = ParamValue::Int32(1600).to_field_for_test(encoding);
        let bytes = vehicle.step(
            1,
            &set("IMU_RATE_HZ", field, MavParamType::MAV_PARAM_TYPE_INT32),
        );
        assert_eq!(vehicle.param("IMU_RATE_HZ"), Some(ParamValue::Int32(1600)));
        let got = values(&bytes);
        assert_eq!(got.len(), 1);
        assert_eq!(name(&got[0]), "IMU_RATE_HZ");
        assert_eq!(
            (got[0].param_index, got[0].param_count),
            (0, DEFAULT_PARAMS as u16)
        );
        assert_eq!(read_value(&got[0], encoding), ParamValue::Int32(1600));
        assert_eq!(got[0].param_type, MavParamType::MAV_PARAM_TYPE_INT32);
        assert!(texts(&bytes).is_empty(), "an accepted write sends no text");
        // The new value is what a later read returns.
        let again = values(&vehicle.step(2, &request_read(-1, "IMU_RATE_HZ")));
        assert_eq!(read_value(&again[0], encoding), ParamValue::Int32(1600));
    }
}

/// Test-side encoding of the values the tests send.
trait ToFieldForTest {
    fn to_field_for_test(self, encoding: ParamEncoding) -> f32;
}

impl ToFieldForTest for ParamValue {
    fn to_field_for_test(self, encoding: ParamEncoding) -> f32 {
        match (self, encoding) {
            (ParamValue::Int32(v), ParamEncoding::Bytewise) => f32::from_le_bytes(v.to_le_bytes()),
            (ParamValue::Int32(v), ParamEncoding::CCast) => v as f32,
            (ParamValue::Int8(v), ParamEncoding::Bytewise) => {
                f32::from_le_bytes([v.to_le_bytes()[0], 0, 0, 0])
            }
            (ParamValue::Int8(v), ParamEncoding::CCast) => f32::from(v),
            (ParamValue::Real32(v), _) => v,
            other => panic!("not needed by the tests: {other:?}"),
        }
    }
}

#[test]
fn set_with_a_16_character_name_and_other_types() {
    let mut vehicle = booted(SimConfig::default());
    let bytes = vehicle.step(
        1,
        &set(
            "NAV_TEST_16CHARS",
            2.25,
            MavParamType::MAV_PARAM_TYPE_REAL32,
        ),
    );
    assert_eq!(
        vehicle.param("NAV_TEST_16CHARS"),
        Some(ParamValue::Real32(2.25))
    );
    assert_eq!(values(&bytes)[0].param_value, 2.25);
    assert!(texts(&bytes).is_empty());

    let field = ParamValue::Int32(-100).to_field_for_test(ParamEncoding::Bytewise);
    let bytes = vehicle.step(
        2,
        &set("SYS_TRIM", field, MavParamType::MAV_PARAM_TYPE_INT32),
    );
    assert_eq!(vehicle.param("SYS_TRIM"), Some(ParamValue::Int32(-100)));
    assert_eq!(values(&bytes)[0].param_value.to_bits(), 0xFFFF_FF9C);

    let mut vehicle = booted(SimConfig::default().with_every_param_type());
    let field = ParamValue::Int8(-100).to_field_for_test(ParamEncoding::Bytewise);
    let bytes = vehicle.step(1, &set("TYPE_I8", field, MavParamType::MAV_PARAM_TYPE_INT8));
    assert_eq!(vehicle.param("TYPE_I8"), Some(ParamValue::Int8(-100)));
    assert_eq!(values(&bytes)[0].param_value.to_bits(), 0x9C);
}

#[test]
fn refused_sets_reply_with_the_current_value_and_a_warning_text() {
    let mut vehicle = booted(SimConfig::default());
    // Read-only.
    let bytes = vehicle.step(
        1,
        &set(
            "SYS_UID",
            f32::from_bits(1),
            MavParamType::MAV_PARAM_TYPE_INT32,
        ),
    );
    assert_eq!(
        vehicle.param("SYS_UID"),
        Some(ParamValue::Int32(1_592_460_852))
    );
    let got = values(&bytes);
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].param_value.to_bits(), 1_592_460_852);
    assert_eq!(got[0].param_index, 11);
    assert_eq!(
        texts(&bytes),
        [(
            MavSeverity::MAV_SEVERITY_WARNING,
            "SYS_UID: read only".to_string()
        )]
    );

    // Wrong type.
    let bytes = vehicle.step(
        2,
        &set("IMU_RATE_HZ", 5.0, MavParamType::MAV_PARAM_TYPE_REAL32),
    );
    assert_eq!(vehicle.param("IMU_RATE_HZ"), Some(ParamValue::Int32(800)));
    let got = values(&bytes);
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].param_value.to_bits(), 800);
    assert_eq!(got[0].param_type, MavParamType::MAV_PARAM_TYPE_INT32);
    assert_eq!(
        texts(&bytes),
        [(
            MavSeverity::MAV_SEVERITY_WARNING,
            "IMU_RATE_HZ: out of range".to_string()
        )]
    );
}

#[test]
fn the_first_echoes_can_be_dropped_while_the_writes_are_applied() {
    let mut vehicle = booted(SimConfig {
        drop_first_param_echoes: 2,
        ..SimConfig::default()
    });
    let write = |vehicle: &mut SimVehicle, t: u64| {
        let field = ParamValue::Int32(1600).to_field_for_test(ParamEncoding::Bytewise);
        vehicle.step(
            t,
            &set("IMU_RATE_HZ", field, MavParamType::MAV_PARAM_TYPE_INT32),
        )
    };
    assert!(values(&write(&mut vehicle, 1)).is_empty());
    assert_eq!(vehicle.param("IMU_RATE_HZ"), Some(ParamValue::Int32(1600)));
    assert!(values(&write(&mut vehicle, 2)).is_empty());
    assert_eq!(values(&write(&mut vehicle, 3)).len(), 1);
    assert_eq!(values(&write(&mut vehicle, 4)).len(), 1);
}

#[test]
fn set_of_an_unknown_name_gets_no_reply() {
    let mut vehicle = booted(SimConfig::default());
    let bytes = vehicle.step(1, &set("NOPE", 1.0, MavParamType::MAV_PARAM_TYPE_REAL32));
    assert!(values(&bytes).is_empty());
    assert!(texts(&bytes).is_empty());
}

#[test]
fn requests_for_another_system_are_ignored() {
    let mut vehicle = booted(SimConfig::default());
    let other = encode(&MavMessage::PARAM_SET(PARAM_SET_DATA {
        param_value: 1.0,
        target_system: 2,
        target_component: 1,
        param_id: CharArray::from("LOG_ENABLE"),
        param_type: MavParamType::MAV_PARAM_TYPE_INT32,
    }));
    let bytes = vehicle.step(1, &other);
    assert!(values(&bytes).is_empty());
    assert_eq!(vehicle.param("LOG_ENABLE"), Some(ParamValue::Int32(1)));
}

#[test]
fn boot_text_is_in_the_first_step_only() {
    let mut vehicle = SimVehicle::new(SimConfig::default());
    let first = texts(&vehicle.step(0, &[]));
    assert_eq!(
        first,
        [(
            MavSeverity::MAV_SEVERITY_INFO,
            "MARV simulator ready".to_string()
        )]
    );
    let rest = run(&mut vehicle, 1, 3000, &[]);
    assert!(texts(&rest).is_empty());
}

fn frame_count(config: SimConfig, steps: u64) -> (Vec<u8>, usize) {
    let mut vehicle = SimVehicle::new(config);
    let mut bytes = Vec::new();
    for i in 0..steps {
        bytes.extend(vehicle.step(i * 5, &[]));
    }
    let n = decode(&bytes).len();
    (bytes, n)
}

fn lossy(percent: u8, seed: u64) -> SimConfig {
    SimConfig {
        drop_tx_percent: percent,
        seed,
        ..SimConfig::default()
    }
}

#[test]
fn tx_loss_rate_is_close_to_the_setting() {
    let (_, sent) = frame_count(lossy(0, 7), 20_000);
    assert!(sent >= 5_000, "{sent}");
    let (_, arrived) = frame_count(lossy(20, 7), 20_000);
    let dropped = (sent - arrived) as f64 / sent as f64 * 100.0;
    assert!((18.0..=22.0).contains(&dropped), "{dropped} %");
}

#[test]
fn tx_loss_extremes_and_determinism() {
    let (plain, sent) = frame_count(lossy(0, 1), 4000);
    assert!(sent > 1000);
    let (none, _) = frame_count(lossy(0, 99), 4000);
    assert_eq!(plain, none, "p = 0 drops nothing whatever the seed");
    let (all, n) = frame_count(lossy(100, 1), 4000);
    assert!(all.is_empty() && n == 0, "p = 100 drops everything");

    let (a, _) = frame_count(lossy(30, 5), 4000);
    let (b, _) = frame_count(lossy(30, 5), 4000);
    let (c, _) = frame_count(lossy(30, 6), 4000);
    assert!(!a.is_empty());
    assert_eq!(a, b, "same seed, same bytes");
    assert_ne!(a, c, "another seed, other losses");
}

#[test]
fn tx_loss_leaves_gaps_in_the_sequence_numbers() {
    let (_, sent) = frame_count(lossy(0, 3), 4000);
    let (bytes, arrived) = frame_count(lossy(25, 3), 4000);
    let seqs: Vec<u8> = decode(&bytes).iter().map(|(h, _)| h.sequence).collect();
    let gaps: usize = seqs
        .windows(2)
        .map(|w| usize::from(w[1].wrapping_sub(w[0]) - 1))
        .sum();
    assert!(gaps > 100, "{gaps} missing sequence numbers");
    assert!(
        seqs.windows(2).any(|w| w[1].wrapping_sub(w[0]) > 1),
        "a visible gap"
    );
    // Every dropped frame is a missing number, apart from those before the first and after the
    // last frame that arrived.
    let dropped = sent - arrived;
    assert!(
        dropped - gaps < 40,
        "{dropped} dropped, {gaps} seen as gaps"
    );
}

fn rx_heartbeats(percent: u8, seed: u64) -> u32 {
    let mut vehicle = SimVehicle::new(SimConfig {
        drop_rx_percent: percent,
        seed,
        ..SimConfig::default()
    });
    for t in 0..10_000 {
        vehicle.step(t, &heartbeat());
    }
    vehicle.heartbeats_received()
}

#[test]
fn rx_loss_drops_incoming_frames_before_they_are_acted_on() {
    let kept = rx_heartbeats(20, 11);
    assert!((7800..=8200).contains(&kept), "{kept}");
    assert_eq!(rx_heartbeats(20, 11), kept, "same seed, same losses");
    assert_eq!(rx_heartbeats(0, 11), 10_000);
    assert_eq!(rx_heartbeats(100, 11), 0);

    let mut vehicle = SimVehicle::new(SimConfig {
        drop_rx_percent: 100,
        ..SimConfig::default()
    });
    vehicle.step(0, &[]);
    let bytes = vehicle.step(
        1,
        &set(
            "LOG_ENABLE",
            f32::from_bits(0),
            MavParamType::MAV_PARAM_TYPE_INT32,
        ),
    );
    assert!(values(&bytes).is_empty());
    assert_eq!(vehicle.param("LOG_ENABLE"), Some(ParamValue::Int32(1)));
}

#[test]
fn silent_vehicle_sends_nothing_but_still_acts() {
    let mut vehicle = booted(SimConfig::default());
    vehicle.set_silent(true);
    let mut bytes = run(&mut vehicle, 1, 2000, &[]);
    bytes.extend(vehicle.step(
        2001,
        &set(
            "LOG_ENABLE",
            f32::from_bits(0),
            MavParamType::MAV_PARAM_TYPE_INT32,
        ),
    ));
    bytes.extend(vehicle.step(2002, &request_list(1, 1)));
    bytes.extend(run(&mut vehicle, 2003, 2200, &[]));
    assert!(bytes.is_empty(), "{} bytes while silent", bytes.len());
    assert_eq!(vehicle.param("LOG_ENABLE"), Some(ParamValue::Int32(0)));

    vehicle.set_silent(false);
    let after = run(&mut vehicle, 2201, 3200, &[]);
    let frames = decode(&after);
    // About one second of the default streams (1 + 1 + 3 * 25 + 1 + 5 + 5 + 10 + 10 + 1), with no replay of what
    // was missed.
    assert!(frames.len().abs_diff(109) <= 6, "{}", frames.len());
    assert!(values(&after).is_empty());
}

#[test]
#[should_panic(expected = "TOO_LONG_PARAMETER_NAME")]
fn a_name_over_16_bytes_is_rejected() {
    let mut config = SimConfig::default();
    config.params.push(SimParam {
        name: "TOO_LONG_PARAMETER_NAME".to_string(),
        value: ParamValue::Uint8(0),
        read_only: false,
    });
    let _ = SimVehicle::new(config);
}

#[test]
#[should_panic(expected = "CAF\u{e9}")]
fn a_non_ascii_name_is_rejected() {
    let mut config = SimConfig::default();
    config.params.push(SimParam {
        name: "CAF\u{e9}".to_string(),
        value: ParamValue::Uint8(0),
        read_only: false,
    });
    let _ = SimVehicle::new(config);
}

#[test]
fn synthetic_parameters_are_appended() {
    let config = SimConfig::default().with_synthetic_params(3);
    let names: Vec<_> = config.params[DEFAULT_PARAMS..]
        .iter()
        .map(|p| p.name.as_str())
        .collect();
    assert_eq!(names, ["P0000", "P0001", "P0002"]);
    assert!(
        config.params[DEFAULT_PARAMS..]
            .iter()
            .all(|p| matches!(p.value, ParamValue::Real32(_)) && !p.read_only)
    );
    let vehicle = SimVehicle::new(config);
    assert!(vehicle.param("P0002").is_some());
    assert_eq!(vehicle.param("NOPE"), None);
}
