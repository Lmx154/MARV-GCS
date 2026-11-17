//! The sensor parameters on the wire: the firmware's defaults, range-checked writes and the reboot
//! notice. The vehicle knows no preset; the table is the station's (`gcs_mavlink::presets`) and the
//! simulator never depends on it: `crates/marv-gcs/tests/e2e_presets.rs` compares the two.

use gcs_sim::{ParamValue, SimConfig, SimVehicle};
use mavlink::dialects::common::{
    MavMessage, MavParamType, MavSeverity, PARAM_SET_DATA, PARAM_VALUE_DATA,
};
use mavlink::types::CharArray;
use mavlink::{MAVLinkV2MessageRaw, MavHeader, MavlinkReader};

type Echoes = Vec<(String, i32)>;
type Texts = Vec<(MavSeverity, String)>;

fn decode(bytes: &[u8]) -> Vec<MavMessage> {
    let mut reader = MavlinkReader::new(bytes);
    let mut out = Vec::new();
    while let Ok((_, message)) = reader.read_any_message::<MavMessage>() {
        out.push(message);
    }
    out
}

fn set(name: &str, value: i32) -> Vec<u8> {
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

fn echoes(bytes: &[u8]) -> Vec<(String, i32)> {
    decode(bytes)
        .into_iter()
        .filter_map(|m| match m {
            MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                param_id,
                param_value,
                ..
            }) => {
                let raw: &[u8; 16] = &param_id;
                let end = raw.iter().position(|&b| b == 0).unwrap_or(16);
                Some((
                    String::from_utf8(raw[..end].to_vec()).unwrap(),
                    i32::from_le_bytes(param_value.to_le_bytes()),
                ))
            }
            _ => None,
        })
        .collect()
}

fn texts(bytes: &[u8]) -> Vec<(MavSeverity, String)> {
    decode(bytes)
        .into_iter()
        .filter_map(|m| match m {
            MavMessage::STATUSTEXT(d) => Some((d.severity, d.text.to_str().unwrap().to_string())),
            _ => None,
        })
        .collect()
}

/// The vehicle after its boot text, and a step counter.
struct Rig {
    vehicle: SimVehicle,
    t: u64,
}

impl Rig {
    fn new() -> Self {
        let mut vehicle = SimVehicle::new(SimConfig::default());
        vehicle.step(0, &[]);
        Self { vehicle, t: 0 }
    }

    /// One PARAM_SET: the echoes and the texts the vehicle sent for it.
    fn write(&mut self, name: &str, value: i32) -> (Echoes, Texts) {
        self.t += 1;
        let bytes = self.vehicle.step(self.t, &set(name, value));
        (echoes(&bytes), texts(&bytes))
    }

    fn int(&self, name: &str) -> i32 {
        match self.vehicle.param(name) {
            Some(ParamValue::Int32(v)) => v,
            other => panic!("{name}: {other:?}"),
        }
    }
}

#[test]
fn the_thirteen_parameters_exist_with_the_firmwares_defaults_and_no_selector_does() {
    let rig = Rig::new();
    for (name, default) in [
        ("IMU1_ACC_FS", 16),
        ("IMU1_GYR_FS", 2000),
        ("IMU1_ODR_HZ", 800),
        ("IMU2_ACC_FS", 16),
        ("IMU2_GYR_FS", 2000),
        ("IMU2_ODR_HZ", 800),
        ("BARO_ODR_HZ", 50),
        ("BARO_OSR_P", 16),
        ("BARO_OSR_T", 2),
        ("BARO_IIR_P", 0),
        ("BARO_IIR_T", 0),
        ("MAG_AVG", 2),
    ] {
        assert_eq!(rig.int(name), default, "{name}");
    }
    assert_eq!(rig.vehicle.param("MAG_ODR_HZ"), Some(ParamValue::Real32(100.0)));
    for selector in ["IMU1_PRESET", "IMU2_PRESET", "BARO_PRESET", "MAG_PRESET"] {
        assert_eq!(rig.vehicle.param(selector), None, "{selector}");
    }
}

#[test]
fn a_baro_field_write_is_accepted_and_echoed_with_no_text() {
    let mut rig = Rig::new();
    let (echo, text) = rig.write("BARO_ODR_HZ", 25);
    assert_eq!(echo, [("BARO_ODR_HZ".to_string(), 25)]);
    assert!(text.is_empty(), "{text:?}");
    assert_eq!(rig.int("BARO_ODR_HZ"), 25);
    assert_eq!(rig.int("BARO_OSR_P"), 16, "the other fields are untouched");
}

#[test]
fn a_write_outside_the_range_is_refused_with_the_old_echo() {
    let mut rig = Rig::new();
    for bad in [0, 241, -1] {
        let (echo, text) = rig.write("BARO_ODR_HZ", bad);
        assert_eq!(echo, [("BARO_ODR_HZ".to_string(), 50)]);
        assert_eq!(text[0].1, "BARO_ODR_HZ: out of range");
    }
    assert_eq!(rig.int("BARO_ODR_HZ"), 50);
}

#[test]
fn an_imu_write_is_accepted_with_the_reboot_notice() {
    let mut rig = Rig::new();
    let (echo, text) = rig.write("IMU1_ODR_HZ", 400);
    assert_eq!(echo, [("IMU1_ODR_HZ".to_string(), 400)]);
    assert_eq!(
        text,
        [(
            MavSeverity::MAV_SEVERITY_INFO,
            "IMU1_ODR_HZ: applies after reboot".to_string()
        )]
    );
    assert_eq!(rig.int("IMU2_ODR_HZ"), 800, "the other IMU is untouched");
    let (_, text) = rig.write("IMU1_ODR_HZ", 400);
    assert!(text.is_empty(), "no change, no notice");
}

#[test]
fn a_mag_write_is_applied_at_once_with_no_notice() {
    let mut rig = Rig::new();
    let (echo, text) = rig.write("MAG_AVG", 8);
    assert_eq!(echo, [("MAG_AVG".to_string(), 8)]);
    assert!(text.is_empty());
}
