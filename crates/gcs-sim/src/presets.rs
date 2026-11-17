//! The sensor preset parameters of `docs/link-contract.md` section 2 and their rules as the simulated
//! vehicle keeps them: the four selectors and the governed fields (R-SEL, R-LOCK, R-CUSTOM, R-RESET,
//! R-BOOT of the design). This is the only place in this repository where these rules exist: the
//! ground station renders the table of `marv-presets` and writes the selector.
//!
//! The simulator does not reboot and does not use the values to shape its sensors. An IMU selector
//! or field is Reboot-class: a change is answered with the notice "<NAME>: applies after reboot".

use marv_presets::{SENSORS, Sensor, Value, preset};

use crate::params::{ParamValue, SimParam};

#[derive(Clone, Copy)]
enum Kind {
    I32,
    F32,
}

/// One parameter of the preset block or one governed field of BARO or MAG.
struct Spec {
    name: &'static str,
    kind: Kind,
    default: f32,
    low: f32,
    high: f32,
    /// Reboot-class: a change takes effect after save and reboot.
    reboot: bool,
}

const fn spec(
    name: &'static str,
    kind: Kind,
    default: f32,
    low: f32,
    high: f32,
    reboot: bool,
) -> Spec {
    Spec {
        name,
        kind,
        default,
        low,
        high,
        reboot,
    }
}

/// The ten parameters in the contract's order (indices 70 to 79 there), then the seven BARO and MAG
/// fields that already exist on the vehicle: they must exist for the lock to have something to lock.
/// Types, defaults and ranges of the last seven are those of the firmware's README (`MAG_ODR_HZ` is
/// a REAL32 there).
const SPECS: [Spec; 17] = [
    spec("IMU1_PRESET", Kind::I32, 1.0, 0.0, 4.0, true),
    spec("IMU1_ACC_FS", Kind::I32, 16.0, 2.0, 32.0, true),
    spec("IMU1_GYR_FS", Kind::I32, 2000.0, 125.0, 4000.0, true),
    spec("IMU1_ODR_HZ", Kind::I32, 800.0, 25.0, 800.0, true),
    spec("IMU2_PRESET", Kind::I32, 1.0, 0.0, 4.0, true),
    spec("IMU2_ACC_FS", Kind::I32, 16.0, 2.0, 16.0, true),
    spec("IMU2_GYR_FS", Kind::I32, 2000.0, 250.0, 4000.0, true),
    spec("IMU2_ODR_HZ", Kind::I32, 800.0, 25.0, 800.0, true),
    spec("BARO_PRESET", Kind::I32, 1.0, 0.0, 4.0, false),
    spec("MAG_PRESET", Kind::I32, 1.0, 0.0, 4.0, false),
    spec("BARO_ODR_HZ", Kind::I32, 50.0, 1.0, 240.0, false),
    spec("BARO_OSR_P", Kind::I32, 16.0, 1.0, 128.0, false),
    spec("BARO_OSR_T", Kind::I32, 2.0, 1.0, 128.0, false),
    spec("BARO_IIR_P", Kind::I32, 0.0, 0.0, 127.0, false),
    spec("BARO_IIR_T", Kind::I32, 0.0, 0.0, 127.0, false),
    spec("MAG_ODR_HZ", Kind::F32, 100.0, 1.5625, 400.0, false),
    spec("MAG_AVG", Kind::I32, 2.0, 1.0, 8.0, false),
];

fn spec_of(name: &str) -> Option<&'static Spec> {
    SPECS.iter().find(|s| s.name == name)
}

fn value_of(spec: &Spec, number: f32) -> ParamValue {
    match spec.kind {
        Kind::I32 => ParamValue::Int32(number as i32),
        Kind::F32 => ParamValue::Real32(number),
    }
}

/// The parameters of the preset block at their defaults.
pub(crate) fn preset_params() -> Vec<SimParam> {
    SPECS
        .iter()
        .map(|s| SimParam {
            name: s.name.to_string(),
            value: value_of(s, s.default),
            read_only: false,
        })
        .collect()
}

fn stored(params: &[SimParam], name: &str) -> Option<ParamValue> {
    params.iter().find(|p| p.name == name).map(|p| p.value)
}

fn number(value: ParamValue) -> Option<f64> {
    match value {
        ParamValue::Int32(v) => Some(f64::from(v)),
        ParamValue::Real32(v) => Some(f64::from(v)),
        _ => None,
    }
}

fn preset_value(spec: &Spec, value: Value) -> ParamValue {
    match value {
        Value::I32(v) => value_of(spec, v as f32),
        Value::F32(v) => value_of(spec, v),
    }
}

/// What the preset rules make of a PARAM_SET that passed the read-only and type checks.
#[derive(Debug, PartialEq)]
pub(crate) enum Verdict {
    /// Not a parameter of the preset block: the ordinary rules apply.
    Plain,
    /// Refused: the whole STATUSTEXT, the stored values stay and the echo is the old value.
    Refuse(String),
    /// Accepted: every value to store, at once (parameter index and value), and the notice to send
    /// after the echo, if any. The echo carries only the written name.
    Accept {
        changes: Vec<(usize, ParamValue)>,
        notice: Option<String>,
    },
}

/// The rules for a write of `new` to the parameter `index` of `params`.
pub(crate) fn judge(params: &[SimParam], index: usize, new: ParamValue) -> Verdict {
    let name = params[index].name.as_str();
    let Some(spec) = spec_of(name) else {
        return Verdict::Plain;
    };
    let in_range =
        number(new).is_some_and(|v| v >= f64::from(spec.low) && v <= f64::from(spec.high));
    if !in_range {
        return Verdict::Refuse(format!("{name}: out of range"));
    }
    let changed_notice = |any_change: bool| {
        (spec.reboot && any_change).then(|| format!("{name}: applies after reboot"))
    };
    if let Some(sensor) = SENSORS.iter().find(|s| s.selector == name) {
        return select(params, index, sensor, new, changed_notice);
    }
    let governor = SENSORS
        .iter()
        .find(|s| s.fields.contains(&name))
        .and_then(|s| stored(params, s.selector).map(|v| (s, v)));
    match governor {
        Some((sensor, ParamValue::Int32(sel))) if sel != 0 => {
            if stored(params, name) == Some(new) {
                Verdict::Accept {
                    changes: vec![(index, new)],
                    notice: None,
                }
            } else {
                Verdict::Refuse(format!(
                    "{name}: locked by {} (0 = custom)",
                    sensor.selector
                ))
            }
        }
        _ => {
            let any_change = stored(params, name) != Some(new);
            Verdict::Accept {
                changes: vec![(index, new)],
                notice: changed_notice(any_change),
            }
        }
    }
}

/// R-SEL: 0 stores 0 and leaves the fields; p > 0 stores the selector and every field of preset p
/// at once, if the crate lists that preset.
fn select(
    params: &[SimParam],
    index: usize,
    sensor: &Sensor,
    new: ParamValue,
    notice: impl Fn(bool) -> Option<String>,
) -> Verdict {
    let ParamValue::Int32(id) = new else {
        return Verdict::Refuse(format!("{}: out of range", sensor.selector));
    };
    let mut changes = vec![(index, new)];
    if id != 0 {
        let Some(chosen) = preset(sensor, id) else {
            return Verdict::Refuse(format!("{}: out of range", sensor.selector));
        };
        for (field, value) in sensor.fields.iter().zip(chosen.values) {
            let (Some(at), Some(spec)) =
                (params.iter().position(|p| p.name == *field), spec_of(field))
            else {
                return Verdict::Refuse(format!("{}: out of range", sensor.selector));
            };
            let value = preset_value(spec, *value);
            let ok = number(value)
                .is_some_and(|v| v >= f64::from(spec.low) && v <= f64::from(spec.high));
            if !ok {
                return Verdict::Refuse(format!("{field}: out of range"));
            }
            changes.push((at, value));
        }
    }
    let any_change = changes.iter().any(|(i, v)| params[*i].value != *v);
    Verdict::Accept {
        changes,
        notice: notice(any_change),
    }
}

/// R-RESET: every selector and field to its default; each field's default is preset 1's value.
pub fn reset_all(params: &mut [SimParam]) {
    for p in params.iter_mut() {
        if let Some(s) = spec_of(&p.name) {
            p.value = value_of(s, s.default);
        }
    }
}

/// R-BOOT: for every selector that is not 0 the fields take the preset's values. Returns the names
/// of the selectors whose fields differed (the vehicle sends one WARNING per device).
pub fn enforce_presets(params: &mut [SimParam]) -> Vec<&'static str> {
    let mut replaced = Vec::new();
    for sensor in SENSORS {
        let Some(ParamValue::Int32(id)) = stored(params, sensor.selector) else {
            continue;
        };
        let Some(chosen) = preset(sensor, id) else {
            continue;
        };
        let mut differed = false;
        for (field, value) in sensor.fields.iter().zip(chosen.values) {
            let Some(s) = spec_of(field) else {
                continue;
            };
            let value = preset_value(s, *value);
            if let Some(p) = params.iter_mut().find(|p| p.name == *field)
                && p.value != value
            {
                p.value = value;
                differed = true;
            }
        }
        if differed {
            replaced.push(sensor.selector);
        }
    }
    replaced
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> Vec<SimParam> {
        preset_params()
    }

    fn at(params: &[SimParam], name: &str) -> usize {
        params.iter().position(|p| p.name == name).unwrap()
    }

    fn get(params: &[SimParam], name: &str) -> ParamValue {
        params[at(params, name)].value
    }

    /// Applies an accepted verdict as the vehicle does.
    fn write(params: &mut [SimParam], name: &str, v: i32) -> Verdict {
        let i = at(params, name);
        let new = match params[i].value {
            ParamValue::Real32(_) => ParamValue::Real32(v as f32),
            _ => ParamValue::Int32(v),
        };
        let verdict = judge(params, i, new);
        if let Verdict::Accept { changes, .. } = &verdict {
            for (i, value) in changes {
                params[*i].value = *value;
            }
        }
        verdict
    }

    #[test]
    fn the_ten_parameters_have_the_contracts_names_types_defaults_and_ranges() {
        let table = [
            ("IMU1_PRESET", 1, 0, 4),
            ("IMU1_ACC_FS", 16, 2, 32),
            ("IMU1_GYR_FS", 2000, 125, 4000),
            ("IMU1_ODR_HZ", 800, 25, 800),
            ("IMU2_PRESET", 1, 0, 4),
            ("IMU2_ACC_FS", 16, 2, 16),
            ("IMU2_GYR_FS", 2000, 250, 4000),
            ("IMU2_ODR_HZ", 800, 25, 800),
            ("BARO_PRESET", 1, 0, 4),
            ("MAG_PRESET", 1, 0, 4),
        ];
        let p = params();
        for (i, (name, default, low, high)) in table.iter().enumerate() {
            assert_eq!(p[i].name, *name);
            assert_eq!(p[i].value, ParamValue::Int32(*default), "{name}");
            let s = spec_of(name).unwrap();
            assert_eq!((s.low, s.high), (*low as f32, *high as f32), "{name}");
            assert_eq!(s.reboot, name.starts_with("IMU"), "{name}");
        }
    }

    #[test]
    fn every_default_equals_preset_1_of_the_crate_and_every_preset_is_in_range() {
        let p = params();
        for sensor in SENSORS {
            assert_eq!(
                get(&p, sensor.selector),
                ParamValue::Int32(sensor.default_id)
            );
            let default = preset(sensor, sensor.default_id).unwrap();
            for (field, value) in sensor.fields.iter().zip(default.values) {
                assert_eq!(
                    get(&p, field),
                    preset_value(spec_of(field).unwrap(), *value),
                    "{field}"
                );
            }
            for chosen in sensor.presets {
                for (field, value) in sensor.fields.iter().zip(chosen.values) {
                    let s = spec_of(field).unwrap();
                    let v = number(preset_value(s, *value)).unwrap();
                    assert!(
                        v >= f64::from(s.low) && v <= f64::from(s.high),
                        "{field} of preset {}",
                        chosen.id
                    );
                }
            }
        }
    }

    #[test]
    fn a_selector_write_sets_every_field_of_the_preset_at_once_and_echoes_the_selector_only() {
        let mut p = params();
        let verdict = write(&mut p, "BARO_PRESET", 3);
        let Verdict::Accept { changes, notice } = verdict else {
            panic!("refused");
        };
        assert_eq!(notice, None);
        assert_eq!(changes.len(), 6);
        assert_eq!(changes[0].0, at(&p, "BARO_PRESET"));
        let chosen = preset(&marv_presets::BARO, 3).unwrap();
        for (field, value) in marv_presets::BARO.fields.iter().zip(chosen.values) {
            assert_eq!(
                get(&p, field),
                preset_value(spec_of(field).unwrap(), *value)
            );
        }
        assert_eq!(get(&p, "BARO_PRESET"), ParamValue::Int32(3));
        assert_eq!(get(&p, "BARO_ODR_HZ"), ParamValue::Int32(25));
    }

    #[test]
    fn the_mag_odr_field_is_real32_and_takes_the_presets_number() {
        let mut p = params();
        write(&mut p, "MAG_PRESET", 4);
        assert_eq!(get(&p, "MAG_ODR_HZ"), ParamValue::Real32(50.0));
        assert_eq!(get(&p, "MAG_AVG"), ParamValue::Int32(8));
    }

    #[test]
    fn a_field_write_under_a_preset_is_refused_with_the_text_and_nothing_changes() {
        let mut p = params();
        let before = p.clone();
        let verdict = write(&mut p, "BARO_ODR_HZ", 25);
        assert_eq!(
            verdict,
            Verdict::Refuse("BARO_ODR_HZ: locked by BARO_PRESET (0 = custom)".to_string())
        );
        assert_eq!(p, before);
        let Verdict::Refuse(text) = write(&mut p, "IMU2_GYR_FS", 4000) else {
            panic!("accepted");
        };
        assert_eq!(text, "IMU2_GYR_FS: locked by IMU2_PRESET (0 = custom)");
        assert!(text.len() <= 50);
    }

    #[test]
    fn a_field_write_equal_to_the_stored_value_is_accepted_under_a_preset() {
        let mut p = params();
        let verdict = write(&mut p, "BARO_ODR_HZ", 50);
        assert_eq!(
            verdict,
            Verdict::Accept {
                changes: vec![(at(&p, "BARO_ODR_HZ"), ParamValue::Int32(50))],
                notice: None
            }
        );
    }

    #[test]
    fn under_custom_a_field_write_is_checked_against_its_range_only() {
        let mut p = params();
        assert!(matches!(
            write(&mut p, "BARO_PRESET", 0),
            Verdict::Accept { .. }
        ));
        assert_eq!(get(&p, "BARO_ODR_HZ"), ParamValue::Int32(50));
        assert!(matches!(
            write(&mut p, "BARO_ODR_HZ", 25),
            Verdict::Accept { .. }
        ));
        assert_eq!(get(&p, "BARO_ODR_HZ"), ParamValue::Int32(25));
        for bad in [0, 241, -1] {
            assert_eq!(
                write(&mut p, "BARO_ODR_HZ", bad),
                Verdict::Refuse("BARO_ODR_HZ: out of range".to_string())
            );
        }
        assert_eq!(get(&p, "BARO_ODR_HZ"), ParamValue::Int32(25));
    }

    #[test]
    fn a_selector_outside_its_range_or_not_in_the_table_is_refused() {
        let mut p = params();
        let before = p.clone();
        for bad in [5, -1, 100] {
            assert_eq!(
                write(&mut p, "BARO_PRESET", bad),
                Verdict::Refuse("BARO_PRESET: out of range".to_string())
            );
        }
        assert_eq!(p, before);
    }

    #[test]
    fn choosing_a_preset_again_after_custom_replaces_the_edited_fields() {
        let mut p = params();
        write(&mut p, "BARO_PRESET", 0);
        write(&mut p, "BARO_IIR_T", 7);
        write(&mut p, "BARO_PRESET", 1);
        assert_eq!(get(&p, "BARO_IIR_T"), ParamValue::Int32(0));
        assert_eq!(get(&p, "BARO_PRESET"), ParamValue::Int32(1));
    }

    #[test]
    fn imu_selectors_and_fields_answer_with_the_reboot_notice_when_they_change() {
        let mut p = params();
        let Verdict::Accept { changes, notice } = write(&mut p, "IMU1_PRESET", 4) else {
            panic!("refused");
        };
        assert_eq!(notice.as_deref(), Some("IMU1_PRESET: applies after reboot"));
        assert_eq!(changes.len(), 4);
        assert_eq!(get(&p, "IMU1_ACC_FS"), ParamValue::Int32(8));
        assert_eq!(get(&p, "IMU1_GYR_FS"), ParamValue::Int32(1000));
        assert_eq!(get(&p, "IMU1_ODR_HZ"), ParamValue::Int32(400));
        let Verdict::Accept { notice, .. } = write(&mut p, "IMU1_PRESET", 4) else {
            panic!("refused");
        };
        assert_eq!(notice, None);
        write(&mut p, "IMU1_PRESET", 0);
        let Verdict::Accept { notice, .. } = write(&mut p, "IMU1_ACC_FS", 4) else {
            panic!("refused");
        };
        assert_eq!(notice.as_deref(), Some("IMU1_ACC_FS: applies after reboot"));
    }

    #[test]
    fn a_configure_class_selector_sends_no_notice() {
        let mut p = params();
        let Verdict::Accept { notice, .. } = write(&mut p, "MAG_PRESET", 2) else {
            panic!("refused");
        };
        assert_eq!(notice, None);
    }

    #[test]
    fn other_parameters_are_not_judged() {
        let p = vec![SimParam {
            name: "SYS_TRIM".to_string(),
            value: ParamValue::Int32(1),
            read_only: false,
        }];
        assert_eq!(judge(&p, 0, ParamValue::Int32(9)), Verdict::Plain);
    }

    #[test]
    fn reset_all_restores_the_selectors_and_their_fields_to_preset_1() {
        let mut p = params();
        write(&mut p, "BARO_PRESET", 3);
        write(&mut p, "IMU2_PRESET", 0);
        write(&mut p, "IMU2_GYR_FS", 500);
        write(&mut p, "MAG_PRESET", 4);
        reset_all(&mut p);
        assert_eq!(p, params());
    }

    #[test]
    fn boot_replaces_the_fields_of_a_selector_that_is_not_0_and_leaves_custom_alone() {
        let mut p = params();
        for (name, v) in [("BARO_ODR_HZ", 33), ("MAG_PRESET", 0), ("MAG_AVG", 5)] {
            let i = at(&p, name);
            p[i].value = ParamValue::Int32(v);
        }
        let replaced = enforce_presets(&mut p);
        assert_eq!(replaced, vec!["BARO_PRESET"]);
        assert_eq!(get(&p, "BARO_ODR_HZ"), ParamValue::Int32(50));
        assert_eq!(get(&p, "MAG_AVG"), ParamValue::Int32(5));
        assert!(enforce_presets(&mut p).is_empty());
    }
}
