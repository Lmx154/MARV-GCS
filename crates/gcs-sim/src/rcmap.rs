//! The RC mapping of `docs/link-contract.md` section 12 as the simulated vehicle does it: the 44 RC
//! parameters and MANUAL_CONTROL computed from RC_CHANNELS and those parameters with the two formulas
//! of the contract (an angle for roll, pitch and yaw, a range for throttle). This is the only place in
//! this repository where the formulas exist: the ground station only shows what MANUAL_CONTROL carries.

use mavlink::dialects::common::{MANUAL_CONTROL_DATA, RC_CHANNELS_DATA};

use crate::params::{ParamValue, SimParam};

/// The channels that have calibration parameters.
pub const MAPPED_CHANNELS: usize = 8;

/// The 44 parameters of the RC block in the contract's order: for n = 1 to 8 `RCn_MIN`, `RCn_TRIM`,
/// `RCn_MAX`, `RCn_REVERSED`, `RCn_DZ`; then `RCMAP_ROLL`, `RCMAP_PITCH`, `RCMAP_THROTTLE`, `RCMAP_YAW`.
/// Each is an INT32.
pub(crate) fn rc_params() -> Vec<SimParam> {
    let mut params = Vec::with_capacity(MAPPED_CHANNELS * 5 + 4);
    for n in 1..=MAPPED_CHANNELS {
        for suffix in ["MIN", "TRIM", "MAX", "REVERSED", "DZ"] {
            let name = format!("RC{n}_{suffix}");
            let (default, _) = default_and_range(&name).unwrap_or((0, (0, 0)));
            params.push(SimParam {
                name,
                value: ParamValue::Int32(default),
                read_only: false,
            });
        }
    }
    for name in ["RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW"] {
        let (default, _) = default_and_range(name).unwrap_or((0, (0, 0)));
        params.push(SimParam {
            name: name.to_string(),
            value: ParamValue::Int32(default),
            read_only: false,
        });
    }
    params
}

/// The default and the range (inclusive) of an RC parameter; `None` for any other name.
fn default_and_range(name: &str) -> Option<(i32, (i32, i32))> {
    if let Some(function) = name.strip_prefix("RCMAP_") {
        let default = match function {
            "ROLL" => 1,
            "PITCH" => 2,
            "THROTTLE" => 3,
            "YAW" => 4,
            _ => return None,
        };
        return Some((default, (1, MAPPED_CHANNELS as i32)));
    }
    let rest = name.strip_prefix("RC")?;
    let (channel, suffix) = rest.split_once('_')?;
    let channel: usize = channel.parse().ok()?;
    if !(1..=MAPPED_CHANNELS).contains(&channel) {
        return None;
    }
    match suffix {
        "MIN" => Some((1100, (800, 2200))),
        "TRIM" => Some((1500, (800, 2200))),
        "MAX" => Some((1900, (800, 2200))),
        "REVERSED" => Some((0, (0, 1))),
        "DZ" => Some((0, (0, 200))),
        _ => None,
    }
}

/// Whether `value` is outside the range of the RC parameter `name`. Any other parameter has no range
/// in the simulator.
pub(crate) fn out_of_range(name: &str, value: i32) -> bool {
    default_and_range(name).is_some_and(|(_, (low, high))| !(low..=high).contains(&value))
}

/// `round(1000 v)`, halves away from zero.
fn per_mille(v: f32) -> i16 {
    (v * 1000.0).round() as i16
}

/// `min(max(x, low), high)` for any `low` and `high`: no panic when `low > high`.
fn limit(x: i32, low: i32, high: i32) -> i32 {
    x.max(low).min(high)
}

/// Roll, pitch or yaw (ArduPilot `pwm_to_angle_dz_trim` divided by `high_in`): -1000 to 1000.
pub fn angle(pwm: u16, min: i32, trim: i32, max: i32, reversed: bool, dz: i32) -> i16 {
    if max <= min {
        return 0;
    }
    let r = limit(i32::from(pwm), min, max);
    let (hi, lo) = (trim + dz, trim - dz);
    let sign = if reversed { -1.0 } else { 1.0 };
    let v = if r > hi && max != hi {
        sign * (r - hi) as f32 / (max - hi) as f32
    } else if r < lo && lo != min {
        sign * (r - lo) as f32 / (lo - min) as f32
    } else {
        0.0
    };
    per_mille(v.clamp(-1.0, 1.0))
}

/// Throttle (ArduPilot `pwm_to_range_dz`, `TRIM` unused): 0 to 1000.
pub fn throttle(pwm: u16, min: i32, max: i32, reversed: bool, dz: i32) -> i16 {
    if max <= min {
        return 0;
    }
    let mut r = limit(i32::from(pwm), min, max);
    if reversed {
        r = max - (r - min);
    }
    let lo = min + dz;
    let v = if r > lo {
        (r - lo) as f32 / (max - lo) as f32
    } else {
        0.0
    };
    per_mille(v.clamp(0.0, 1.0))
}

/// The microseconds of channel 1 to 16 of an RC_CHANNELS.
pub fn channels_of(d: &RC_CHANNELS_DATA) -> [u16; 16] {
    [
        d.chan1_raw,
        d.chan2_raw,
        d.chan3_raw,
        d.chan4_raw,
        d.chan5_raw,
        d.chan6_raw,
        d.chan7_raw,
        d.chan8_raw,
        d.chan9_raw,
        d.chan10_raw,
        d.chan11_raw,
        d.chan12_raw,
        d.chan13_raw,
        d.chan14_raw,
        d.chan15_raw,
        d.chan16_raw,
    ]
}

/// The MANUAL_CONTROL of `channels` (microseconds of channel 1 to 16) with the vehicle's RC parameters
/// read through `param`. `None` when the vehicle has no RC parameters (no `RC1_MIN`) or a map or a
/// parameter is missing or not an INT32: the vehicle then sends no MANUAL_CONTROL.
pub fn manual_control(
    system_id: u8,
    channels: &[u16; 16],
    param: impl Fn(&str) -> Option<ParamValue>,
) -> Option<MANUAL_CONTROL_DATA> {
    let int = |name: &str| match param(name)? {
        ParamValue::Int32(v) => Some(v),
        _ => None,
    };
    int("RC1_MIN")?;
    let function = |name: &str| -> Option<(usize, [i32; 5])> {
        let channel = usize::try_from(int(name)?).ok()?;
        if !(1..=MAPPED_CHANNELS).contains(&channel) {
            return None;
        }
        let get = |suffix: &str| int(&format!("RC{channel}_{suffix}"));
        Some((
            channel,
            [
                get("MIN")?,
                get("TRIM")?,
                get("MAX")?,
                get("REVERSED")?,
                get("DZ")?,
            ],
        ))
    };
    let angle_of = |name: &str| -> Option<i16> {
        let (channel, [min, trim, max, reversed, dz]) = function(name)?;
        Some(angle(
            channels[channel - 1],
            min,
            trim,
            max,
            reversed != 0,
            dz,
        ))
    };
    let (channel, [min, _, max, reversed, dz]) = function("RCMAP_THROTTLE")?;
    Some(MANUAL_CONTROL_DATA {
        x: angle_of("RCMAP_PITCH")?,
        y: angle_of("RCMAP_ROLL")?,
        z: throttle(channels[channel - 1], min, max, reversed != 0, dz),
        r: angle_of("RCMAP_YAW")?,
        target: system_id,
        buttons: 0,
        ..MANUAL_CONTROL_DATA::default()
    })
}
