//! The last 23 parameters of the simulated vehicle, indices 70 to 92, with the firmware's types,
//! defaults and ranges (`MARV-V2-firmware/crates/fw-settings/src/`, branch `compass-cal`; link contract
//! section 16 for the compass).
//!
//! Indices 70 to 79 are ten parameters of the firmware that the simulator did not have and that the
//! calibration needs or that fill the table up to the firmware's index 80: `MAG_OFFX/Y/Z`
//! (`mag.rs:41-43`), `BATT_A_SCALE` and `BATT_A_OFFSET` (`batt.rs:29-30`), `LOG_BACKEND_TYPE` (`sd.rs:16`),
//! `AHRS_ORIENTATION` (`imu.rs:121`), `MAG_ORIENT` (`mag.rs:53`), `MOT_POLES` (`esc.rs`) and `LED_BRIGHT`
//! (`led.rs`). Index 80 is `GPS_RATE_MS` (`gnss.rs:20`), 81 to 91 the compass block (`mag.rs:98-108`), 92 `GPS_GNSS` (the constellation set).
//! The simulator shapes no sensor from any of them except `MAG_OFF*` and 81 to 86 (the magnetometer,
//! [`crate::mag`]); `MAG_ORIENT` is not applied: the simulator's raw field is already in the body frame.
//!
//! A write is checked against the range only, except `GPS_RATE_MS`, which must also be 100, 200, 500 or
//! 1000 (`gnss.rs:36-41`) and 100 only while `GPS_GNSS` is 1 (a write of `GPS_GNSS` away from 1 is refused
//! while the rate is 100). Indices 81 to 89 and `MAG_OFF*` return to their defaults on the calibration
//! reset (command 245, param1 = 3).

use crate::params::{ParamValue, SimParam};
use crate::presets::Verdict;

#[derive(Clone, Copy)]
enum Kind {
    I32,
    F32,
}

struct Spec {
    name: &'static str,
    kind: Kind,
    default: f32,
    low: f32,
    high: f32,
    /// Reboot-class (`Apply::Reboot`): the vehicle answers a change with `<NAME>: applies after reboot`.
    reboot: bool,
    /// Returns to its default on the calibration reset.
    calibration: bool,
}

const fn s(
    name: &'static str,
    kind: Kind,
    default: f32,
    low: f32,
    high: f32,
    reboot: bool,
    calibration: bool,
) -> Spec {
    Spec {
        name,
        kind,
        default,
        low,
        high,
        reboot,
        calibration,
    }
}

/// Indices 70 to 92 in order.
const SPECS: [Spec; 23] = [
    s("MAG_OFFX", Kind::F32, 0.0, -200.0, 200.0, false, true),
    s("MAG_OFFY", Kind::F32, 0.0, -200.0, 200.0, false, true),
    s("MAG_OFFZ", Kind::F32, 0.0, -200.0, 200.0, false, true),
    s("BATT_A_SCALE", Kind::F32, 0.02, 0.0, 10.0, false, false),
    s("BATT_A_OFFSET", Kind::F32, 0.0, -3.3, 3.3, false, false),
    s("LOG_BACKEND_TYPE", Kind::I32, 1.0, 0.0, 1.0, true, false),
    s("AHRS_ORIENTATION", Kind::I32, 0.0, 0.0, 15.0, true, false),
    s("MAG_ORIENT", Kind::I32, 10.0, 0.0, 15.0, false, false),
    s("MOT_POLES", Kind::I32, 14.0, 2.0, 64.0, false, false),
    s("LED_BRIGHT", Kind::I32, 1.0, 0.0, 3.0, false, false),
    s("GPS_RATE_MS", Kind::I32, 200.0, 100.0, 1000.0, false, false),
    s("MAG_DIAGX", Kind::F32, 1.0, 0.2, 5.0, false, true),
    s("MAG_DIAGY", Kind::F32, 1.0, 0.2, 5.0, false, true),
    s("MAG_DIAGZ", Kind::F32, 1.0, 0.2, 5.0, false, true),
    s("MAG_ODIAGXY", Kind::F32, 0.0, -1.0, 1.0, false, true),
    s("MAG_ODIAGXZ", Kind::F32, 0.0, -1.0, 1.0, false, true),
    s("MAG_ODIAGYZ", Kind::F32, 0.0, -1.0, 1.0, false, true),
    s("MAG_CAL_TYPE", Kind::I32, 0.0, 0.0, 2.0, false, true),
    s("MAG_FLD_UT", Kind::F32, 0.0, 0.0, 95.0, false, true),
    s("MAG_FLD_INC", Kind::F32, 0.0, -90.0, 90.0, false, true),
    s("MAG_HLT_STR", Kind::F32, 10.0, 1.0, 50.0, false, false),
    s("MAG_HLT_INC", Kind::F32, 10.0, 1.0, 45.0, false, false),
    s("GPS_GNSS", Kind::I32, 0.0, 0.0, 2.0, false, false),
];

/// The text of the refusal of a `GPS_RATE_MS` inside 100..1000 that is not one of the four (`gnss.rs:36`).
pub const BAD_GPS_RATE: &str = "GPS_RATE_MS: not 100, 200, 500 or 1000";

/// The text of the refusal of a rate of 100 ms with a constellation set other than 1: it names both values
/// (a STATUSTEXT holds 50 characters).
fn rate_needs_gnss(gnss: i32) -> String {
    format!("GPS_RATE_MS 100 needs GPS_GNSS 1, is {gnss}")
}

/// The text of the refusal of a constellation set other than 1 while the rate is 100 ms.
fn gnss_blocked_by_rate(gnss: i32) -> String {
    format!("GPS_GNSS {gnss}: GPS_RATE_MS 100 needs 1")
}

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
pub(crate) fn fw_params() -> Vec<SimParam> {
    SPECS
        .iter()
        .map(|s| SimParam {
            name: s.name.to_string(),
            value: value_of(s, s.default),
            read_only: false,
        })
        .collect()
}

/// The calibration parameters (`MAG_OFF*` and 81 to 89) at their defaults: what the calibration reset
/// writes.
pub(crate) fn calibration_defaults() -> Vec<(&'static str, ParamValue)> {
    SPECS
        .iter()
        .filter(|s| s.calibration)
        .map(|s| (s.name, value_of(s, s.default)))
        .collect()
}

/// The rules for a write of `new` to the parameter `index` of `params`: the range, and the four values of
/// `GPS_RATE_MS` (100 only with `GPS_GNSS` 1). Not a parameter of this block: [`Verdict::Plain`].
pub(crate) fn judge(params: &[SimParam], index: usize, new: ParamValue) -> Verdict {
    let name = params[index].name.as_str();
    let Some(spec) = spec_of(name) else {
        return Verdict::Plain;
    };
    let number = match new {
        ParamValue::Int32(v) => f64::from(v),
        ParamValue::Real32(v) => f64::from(v),
        _ => f64::NAN,
    };
    if !(number >= f64::from(spec.low) && number <= f64::from(spec.high)) {
        return Verdict::Refuse(format!("{name}: out of range"));
    }
    if name == "GPS_RATE_MS" && ![100.0, 200.0, 500.0, 1000.0].contains(&number) {
        return Verdict::Refuse(BAD_GPS_RATE.to_string());
    }
    let other = |name: &str| {
        params
            .iter()
            .find(|p| p.name == name)
            .map(|p| match p.value {
                ParamValue::Int32(v) => v,
                _ => 0,
            })
    };
    if name == "GPS_RATE_MS" && number == 100.0 && other("GPS_GNSS") != Some(1) {
        return Verdict::Refuse(rate_needs_gnss(other("GPS_GNSS").unwrap_or(0)));
    }
    if name == "GPS_GNSS" && number != 1.0 && other("GPS_RATE_MS") == Some(100) {
        return Verdict::Refuse(gnss_blocked_by_rate(number as i32));
    }
    Verdict::Accept {
        notice: (spec.reboot && params[index].value != new)
            .then(|| format!("{name}: applies after reboot")),
    }
}
