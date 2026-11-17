//! The sensor parameters of `docs/link-contract.md` section 2 as the simulated vehicle keeps them: the
//! six IMU parameters and the BARO and MAG fields, with the firmware's defaults. The vehicle knows no
//! preset: a preset is the station writing several of these parameters (section 13). Every write is
//! checked against the parameter's range only.
//!
//! The simulator does not reboot and does not use the values to shape its sensors. An IMU parameter is
//! Reboot-class: a change is answered with the notice "<NAME>: applies after reboot".

use crate::params::{ParamValue, SimParam};

#[derive(Clone, Copy)]
enum Kind {
    I32,
    F32,
}

/// One IMU parameter, or one field of BARO or MAG that a station preset sets.
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

/// The six IMU parameters in the contract's order (indices 70 to 75 there), then the seven BARO and MAG
/// fields that already exist on the vehicle. Types, defaults and ranges are those of the firmware's
/// README (`MAG_ODR_HZ` is a REAL32 there); the defaults are the Flight / Balanced presets' values.
const SPECS: [Spec; 13] = [
    spec("IMU1_ACC_FS", Kind::I32, 16.0, 2.0, 32.0, true),
    spec("IMU1_GYR_FS", Kind::I32, 2000.0, 125.0, 4000.0, true),
    spec("IMU1_ODR_HZ", Kind::I32, 800.0, 25.0, 800.0, true),
    spec("IMU2_ACC_FS", Kind::I32, 16.0, 2.0, 16.0, true),
    spec("IMU2_GYR_FS", Kind::I32, 2000.0, 250.0, 4000.0, true),
    spec("IMU2_ODR_HZ", Kind::I32, 800.0, 25.0, 800.0, true),
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

/// The parameters of this block at their defaults.
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

fn number(value: ParamValue) -> Option<f64> {
    match value {
        ParamValue::Int32(v) => Some(f64::from(v)),
        ParamValue::Real32(v) => Some(f64::from(v)),
        _ => None,
    }
}

/// What the rules make of a PARAM_SET that passed the read-only and type checks.
#[derive(Debug, PartialEq)]
pub(crate) enum Verdict {
    /// Not a parameter of this block: the ordinary rules apply.
    Plain,
    /// Refused: the whole STATUSTEXT; the stored value stays and the echo is the old value.
    Refuse(String),
    /// Accepted: store the value; the notice to send after the echo, if any.
    Accept { notice: Option<String> },
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
    Verdict::Accept {
        notice: (spec.reboot && params[index].value != new)
            .then(|| format!("{name}: applies after reboot")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        if matches!(verdict, Verdict::Accept { .. }) {
            params[i].value = new;
        }
        verdict
    }

    #[test]
    fn the_thirteen_parameters_have_the_firmwares_names_types_defaults_and_ranges() {
        let table = [
            ("IMU1_ACC_FS", 16, 2, 32),
            ("IMU1_GYR_FS", 2000, 125, 4000),
            ("IMU1_ODR_HZ", 800, 25, 800),
            ("IMU2_ACC_FS", 16, 2, 16),
            ("IMU2_GYR_FS", 2000, 250, 4000),
            ("IMU2_ODR_HZ", 800, 25, 800),
            ("BARO_ODR_HZ", 50, 1, 240),
            ("BARO_OSR_P", 16, 1, 128),
            ("BARO_OSR_T", 2, 1, 128),
            ("BARO_IIR_P", 0, 0, 127),
            ("BARO_IIR_T", 0, 0, 127),
            ("MAG_AVG", 2, 1, 8),
        ];
        let p = preset_params();
        assert_eq!(p.len(), 13);
        for (name, default, low, high) in table {
            assert_eq!(get(&p, name), ParamValue::Int32(default), "{name}");
            let s = spec_of(name).unwrap();
            assert_eq!((s.low, s.high), (low as f32, high as f32), "{name}");
            assert_eq!(s.reboot, name.starts_with("IMU"), "{name}");
        }
        assert_eq!(get(&p, "MAG_ODR_HZ"), ParamValue::Real32(100.0));
        assert!(!p.iter().any(|q| q.name.ends_with("_PRESET")));
    }

    #[test]
    fn a_write_is_checked_against_the_range_only() {
        let mut p = preset_params();
        assert!(matches!(
            write(&mut p, "BARO_ODR_HZ", 25),
            Verdict::Accept { notice: None }
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
    fn the_mag_odr_field_is_real32_and_takes_a_number() {
        let mut p = preset_params();
        write(&mut p, "MAG_ODR_HZ", 50);
        assert_eq!(get(&p, "MAG_ODR_HZ"), ParamValue::Real32(50.0));
    }

    #[test]
    fn an_imu_parameter_that_changes_answers_with_the_reboot_notice() {
        let mut p = preset_params();
        let Verdict::Accept { notice } = write(&mut p, "IMU1_ACC_FS", 8) else {
            panic!("refused");
        };
        assert_eq!(notice.as_deref(), Some("IMU1_ACC_FS: applies after reboot"));
        let Verdict::Accept { notice } = write(&mut p, "IMU1_ACC_FS", 8) else {
            panic!("refused");
        };
        assert_eq!(notice, None);
        assert_eq!(
            write(&mut p, "IMU2_ACC_FS", 32),
            Verdict::Refuse("IMU2_ACC_FS: out of range".to_string())
        );
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
}
