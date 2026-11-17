//! The sensor preset parameters on the wire: R-SEL, R-LOCK, R-CUSTOM and the reboot notice, with the
//! table of `marv-presets` as the oracle.

use gcs_sim::{ParamValue, SimConfig, SimVehicle};
use marv_presets::{BARO, IMU1, MAG, preset};
use mavlink::dialects::common::{
    MavMessage, MavParamType, MavSeverity, PARAM_SET_DATA, PARAM_VALUE_DATA,
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
    fn write(
        &mut self,
        name: &str,
        value: i32,
    ) -> (Vec<(String, i32)>, Vec<(MavSeverity, String)>) {
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

fn assert_fields_are_preset(rig: &Rig, sensor: &marv_presets::Sensor, id: i32) {
    let chosen = preset(sensor, id).unwrap();
    for (field, value) in sensor.fields.iter().zip(chosen.values) {
        match (rig.vehicle.param(field), value) {
            (Some(ParamValue::Int32(v)), marv_presets::Value::I32(w)) => {
                assert_eq!(v, *w, "{field}")
            }
            (Some(ParamValue::Real32(v)), marv_presets::Value::I32(w)) => {
                assert_eq!(v, *w as f32, "{field}")
            }
            other => panic!("{field}: {other:?}"),
        }
    }
}

#[test]
fn the_ten_parameters_exist_with_their_defaults_and_preset_1_values() {
    let rig = Rig::new();
    for (name, default) in [
        ("IMU1_PRESET", 1),
        ("IMU1_ACC_FS", 16),
        ("IMU1_GYR_FS", 2000),
        ("IMU1_ODR_HZ", 800),
        ("IMU2_PRESET", 1),
        ("IMU2_ACC_FS", 16),
        ("IMU2_GYR_FS", 2000),
        ("IMU2_ODR_HZ", 800),
        ("BARO_PRESET", 1),
        ("MAG_PRESET", 1),
    ] {
        assert_eq!(rig.int(name), default, "{name}");
    }
    for sensor in marv_presets::SENSORS {
        assert_fields_are_preset(&rig, sensor, 1);
    }
}

#[test]
fn baro_preset_3_sets_all_five_fields_in_one_write_and_echoes_only_the_selector() {
    let mut rig = Rig::new();
    let (echo, text) = rig.write("BARO_PRESET", 3);
    assert_eq!(echo, [("BARO_PRESET".to_string(), 3)]);
    assert!(text.is_empty(), "{text:?}");
    assert_fields_are_preset(&rig, &BARO, 3);
    assert_eq!(rig.int("BARO_ODR_HZ"), 25);
    assert_eq!(rig.int("BARO_IIR_P"), 3);
}

#[test]
fn a_field_write_under_preset_1_is_refused_with_the_text_and_the_old_echo() {
    let mut rig = Rig::new();
    let (echo, text) = rig.write("BARO_ODR_HZ", 25);
    assert_eq!(echo, [("BARO_ODR_HZ".to_string(), 50)]);
    assert_eq!(
        text,
        [(
            MavSeverity::MAV_SEVERITY_WARNING,
            "BARO_ODR_HZ: locked by BARO_PRESET (0 = custom)".to_string()
        )]
    );
    assert_eq!(rig.int("BARO_ODR_HZ"), 50);
    assert_eq!(
        rig.write("MAG_AVG", 8).1[0].1,
        "MAG_AVG: locked by MAG_PRESET (0 = custom)"
    );
}

#[test]
fn a_write_equal_to_the_stored_field_is_accepted_under_a_preset() {
    let mut rig = Rig::new();
    let (echo, text) = rig.write("BARO_ODR_HZ", 50);
    assert_eq!(echo, [("BARO_ODR_HZ".to_string(), 50)]);
    assert!(text.is_empty());
}

#[test]
fn preset_0_then_a_field_write_is_accepted_and_leaves_the_other_fields() {
    let mut rig = Rig::new();
    let (echo, text) = rig.write("BARO_PRESET", 0);
    assert_eq!(echo, [("BARO_PRESET".to_string(), 0)]);
    assert!(text.is_empty());
    assert_fields_are_preset(&rig, &BARO, 1);
    let (echo, text) = rig.write("BARO_ODR_HZ", 25);
    assert_eq!(echo, [("BARO_ODR_HZ".to_string(), 25)]);
    assert!(text.is_empty());
    assert_eq!(rig.int("BARO_ODR_HZ"), 25);
    assert_eq!(rig.int("BARO_OSR_P"), 16);
    let (echo, text) = rig.write("BARO_ODR_HZ", 241);
    assert_eq!(echo, [("BARO_ODR_HZ".to_string(), 25)]);
    assert_eq!(text[0].1, "BARO_ODR_HZ: out of range");
}

#[test]
fn an_unknown_preset_id_is_refused_and_nothing_changes() {
    let mut rig = Rig::new();
    for bad in [5, 9, -1] {
        let (echo, text) = rig.write("BARO_PRESET", bad);
        assert_eq!(echo, [("BARO_PRESET".to_string(), 1)]);
        assert_eq!(text[0].1, "BARO_PRESET: out of range");
    }
    assert_fields_are_preset(&rig, &BARO, 1);
}

#[test]
fn imu1_preset_4_is_accepted_with_the_reboot_notice() {
    let mut rig = Rig::new();
    let (echo, text) = rig.write("IMU1_PRESET", 4);
    assert_eq!(echo, [("IMU1_PRESET".to_string(), 4)]);
    assert_eq!(
        text,
        [(
            MavSeverity::MAV_SEVERITY_INFO,
            "IMU1_PRESET: applies after reboot".to_string()
        )]
    );
    assert_fields_are_preset(&rig, &IMU1, 4);
    assert_eq!(rig.int("IMU1_ODR_HZ"), 400);
    assert_eq!(rig.int("IMU2_ODR_HZ"), 800, "the other IMU is untouched");
}

#[test]
fn mag_preset_changes_are_applied_at_once_with_no_notice() {
    let mut rig = Rig::new();
    let (echo, text) = rig.write("MAG_PRESET", 4);
    assert_eq!(echo, [("MAG_PRESET".to_string(), 4)]);
    assert!(text.is_empty());
    assert_fields_are_preset(&rig, &MAG, 4);
}
