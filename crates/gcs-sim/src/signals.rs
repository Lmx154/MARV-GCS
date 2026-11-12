//! Sensor values of the simulated vehicle, each a pure function of the time in milliseconds.
//!
//! The simulated board lies still, with a small visible motion so plots are not flat lines.
//! `time_boot_ms` of every message is `now_ms` (GPS and HIGHRES_IMU `time_usec` is `now_ms * 1000`).
//!
//! The default streams are HEARTBEAT, HIGHRES_IMU (four instances, see [`highres_imu`]),
//! SCALED_PRESSURE and GPS_RAW_INT. [`scaled_imu`], [`scaled_imu2`] and [`sys_status`] are not
//! default streams any more (the link contract rejects SCALED_IMU: it stops at 32.7 g) but a
//! custom [`SimConfig`](crate::SimConfig) can still stream them.

use mavlink::MessageData;
use mavlink::dialects::common::{
    GPS_RAW_INT_DATA, GpsFixType, HEARTBEAT_DATA, HIGHRES_IMU_DATA, HighresImuUpdatedFlags,
    MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, SCALED_IMU_DATA, SCALED_IMU2_DATA,
    SCALED_PRESSURE_DATA, SYS_STATUS_DATA,
};

/// Mean barometric pressure in hPa.
pub const PRESSURE_MEAN_HPA: f32 = 1013.25;
/// Temperature of every sensor in centi-degrees Celsius.
pub const TEMPERATURE_CDEGC: i16 = 2500;
/// Latitude in degE7 (47.3977 deg).
pub const LATITUDE_E7: i32 = 473_977_000;
/// Longitude in degE7 (8.5456 deg).
pub const LONGITUDE_E7: i32 = 85_456_000;
/// Altitude in mm (488 m).
pub const ALTITUDE_MM: i32 = 488_000;
/// Battery voltage in mV.
pub const BATTERY_MV: u16 = 16_000;

/// Constant magnetometer reading in mGauss.
const MAG: [i16; 3] = [210, -30, 420];
/// Number of HIGHRES_IMU instances (`id` 0 to 3), all sent in each period of the stream.
pub const HIGHRES_IMU_INSTANCES: u8 = 4;
/// Standard gravity as the main IMUs measure it, in m/s^2 (z axis, board lying still).
pub const GRAVITY_MS2: f64 = 9.81;
/// Gravity as the high-g accelerometer measures it, in m/s^2 (z axis).
pub const HIGH_G_GRAVITY_MS2: f64 = 9.8;
/// Constant magnetic field of instance 3 in gauss.
pub const MAG_GAUSS: [f32; 3] = [0.21, -0.03, 0.42];
/// Temperature of the sensors that measure it, in degrees Celsius.
pub const TEMPERATURE_DEGC: f32 = 25.0;

fn seconds(now_ms: u64) -> f64 {
    now_ms as f64 / 1000.0
}

/// `amplitude * sin(2 pi freq_hz t + phase)`.
fn sine(now_ms: u64, amplitude: f64, freq_hz: f64, phase: f64) -> f64 {
    amplitude * (std::f64::consts::TAU * freq_hz * seconds(now_ms) + phase).sin()
}

/// `[xacc, yacc, zacc, xgyro, ygyro, zgyro]`: accel in mG, gyro in mrad/s.
///
/// Accel is (0, 0, -1000) plus sines of 30, 25 and 20 mG at 0.31, 0.47 and 0.23 Hz. Gyro is sines
/// of 5, 4 and 3 mrad/s at 0.20, 0.35 and 0.15 Hz. `phase` shifts every sine.
fn imu_motion(now_ms: u64, phase: f64) -> [i16; 6] {
    [
        sine(now_ms, 30.0, 0.31, phase).round() as i16,
        sine(now_ms, 25.0, 0.47, 1.0 + phase).round() as i16,
        (-1000.0 + sine(now_ms, 20.0, 0.23, 2.0 + phase)).round() as i16,
        sine(now_ms, 5.0, 0.20, 0.5 + phase).round() as i16,
        sine(now_ms, 4.0, 0.35, 1.5 + phase).round() as i16,
        sine(now_ms, 3.0, 0.15, 2.5 + phase).round() as i16,
    ]
}

/// `[xacc, yacc, zacc]` in m/s^2 and `[xgyro, ygyro, zgyro]` in rad/s of an IMU.
///
/// Accel is (0, 0, -9.81) plus sines of 0.3, 0.25 and 0.2 m/s^2 at 0.31, 0.47 and 0.23 Hz. Gyro is
/// sines of 5, 4 and 3 mrad/s (0.005, 0.004, 0.003 rad/s) at 0.20, 0.35 and 0.15 Hz. `phase` (rad)
/// shifts every sine.
fn imu_motion_si(now_ms: u64, phase: f64) -> [f64; 6] {
    [
        sine(now_ms, 0.3, 0.31, phase),
        sine(now_ms, 0.25, 0.47, 1.0 + phase),
        -GRAVITY_MS2 + sine(now_ms, 0.2, 0.23, 2.0 + phase),
        sine(now_ms, 0.005, 0.20, 0.5 + phase),
        sine(now_ms, 0.004, 0.35, 1.5 + phase),
        sine(now_ms, 0.003, 0.15, 2.5 + phase),
    ]
}

/// Deterministic noise in [-0.05, 0.05) m/s^2: a hash of the time and the axis, so it is a pure
/// function of time like everything else here.
fn high_g_noise(now_ms: u64, axis: u64) -> f64 {
    let mut x =
        now_ms.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ (axis + 1).wrapping_mul(0xD1B5_4A32_D192_ED03);
    x ^= x >> 32;
    x = x.wrapping_mul(0xD6E8_FEB8_6659_FD93);
    x ^= x >> 32;
    ((x >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 0.1
}

/// Acceleration of the high-g accelerometer in m/s^2: (0, 0, -9.8) plus sines of 0.5 m/s^2 at 1.7,
/// 2.3 and 1.1 Hz (phases 0, 1 and 2 rad) plus [`high_g_noise`], a coarser and noisier look than
/// the main IMUs.
fn high_g_motion(now_ms: u64) -> [f64; 3] {
    [
        sine(now_ms, 0.5, 1.7, 0.0) + high_g_noise(now_ms, 0),
        sine(now_ms, 0.5, 2.3, 1.0) + high_g_noise(now_ms, 1),
        -HIGH_G_GRAVITY_MS2 + sine(now_ms, 0.5, 1.1, 2.0) + high_g_noise(now_ms, 2),
    ]
}

/// HIGHRES_IMU instance `id` (0 to 3) at this time, or `None` for any other `id`. SI units;
/// `time_usec` is `now_ms * 1000`; a field the instance does not measure is NaN and its
/// `fields_updated` bit is clear; pressure fields are NaN in every instance.
///
/// - 0, main IMU: acceleration, gyro (see `imu_motion_si`: (0, 0, -9.81) m/s^2 plus sines up to
///   0.3 m/s^2, rotation sines of a few mrad/s) and temperature 25.0 degC.
/// - 1, second IMU: the same with every sine shifted by 1.3 rad.
/// - 2, high-g accelerometer: acceleration only, about (0, 0, -9.8) m/s^2 with sines of 0.5 m/s^2
///   and noise of up to 0.05 m/s^2.
/// - 3, magnetometer: field (0.21, -0.03, 0.42) gauss and temperature 25.0 degC.
pub fn highres_imu(id: u8, now_ms: u64) -> Option<HIGHRES_IMU_DATA> {
    use HighresImuUpdatedFlags as F;
    let nan = f32::NAN;
    let blank = HIGHRES_IMU_DATA {
        time_usec: now_ms * 1000,
        xacc: nan,
        yacc: nan,
        zacc: nan,
        xgyro: nan,
        ygyro: nan,
        zgyro: nan,
        xmag: nan,
        ymag: nan,
        zmag: nan,
        abs_pressure: nan,
        diff_pressure: nan,
        pressure_alt: nan,
        temperature: nan,
        fields_updated: F::empty(),
        id,
    };
    let imu = |phase: f64| {
        let m = imu_motion_si(now_ms, phase);
        HIGHRES_IMU_DATA {
            xacc: m[0] as f32,
            yacc: m[1] as f32,
            zacc: m[2] as f32,
            xgyro: m[3] as f32,
            ygyro: m[4] as f32,
            zgyro: m[5] as f32,
            temperature: TEMPERATURE_DEGC,
            fields_updated: F::HIGHRES_IMU_UPDATED_XACC
                | F::HIGHRES_IMU_UPDATED_YACC
                | F::HIGHRES_IMU_UPDATED_ZACC
                | F::HIGHRES_IMU_UPDATED_XGYRO
                | F::HIGHRES_IMU_UPDATED_YGYRO
                | F::HIGHRES_IMU_UPDATED_ZGYRO
                | F::HIGHRES_IMU_UPDATED_TEMPERATURE,
            ..blank.clone()
        }
    };
    Some(match id {
        0 => imu(0.0),
        1 => imu(1.3),
        2 => {
            let a = high_g_motion(now_ms);
            HIGHRES_IMU_DATA {
                xacc: a[0] as f32,
                yacc: a[1] as f32,
                zacc: a[2] as f32,
                fields_updated: F::HIGHRES_IMU_UPDATED_XACC
                    | F::HIGHRES_IMU_UPDATED_YACC
                    | F::HIGHRES_IMU_UPDATED_ZACC,
                ..blank
            }
        }
        3 => HIGHRES_IMU_DATA {
            xmag: MAG_GAUSS[0],
            ymag: MAG_GAUSS[1],
            zmag: MAG_GAUSS[2],
            temperature: TEMPERATURE_DEGC,
            fields_updated: F::HIGHRES_IMU_UPDATED_XMAG
                | F::HIGHRES_IMU_UPDATED_YMAG
                | F::HIGHRES_IMU_UPDATED_ZMAG
                | F::HIGHRES_IMU_UPDATED_TEMPERATURE,
            ..blank
        },
        _ => return None,
    })
}

/// HEARTBEAT: generic vehicle, generic autopilot, standby, MAVLink version 3.
pub fn heartbeat() -> HEARTBEAT_DATA {
    HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GENERIC,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    }
}

/// SYS_STATUS: battery 16000 mV, everything else unknown or zero.
pub fn sys_status() -> SYS_STATUS_DATA {
    SYS_STATUS_DATA {
        voltage_battery: BATTERY_MV,
        current_battery: -1,
        battery_remaining: -1,
        ..SYS_STATUS_DATA::default()
    }
}

/// SCALED_IMU: see [`imu_motion`]; mag is constant (210, -30, 420) mGauss; temperature 2500 cdegC.
pub fn scaled_imu(now_ms: u64) -> SCALED_IMU_DATA {
    let m = imu_motion(now_ms, 0.0);
    SCALED_IMU_DATA {
        time_boot_ms: now_ms as u32,
        xacc: m[0],
        yacc: m[1],
        zacc: m[2],
        xgyro: m[3],
        ygyro: m[4],
        zgyro: m[5],
        xmag: MAG[0],
        ymag: MAG[1],
        zmag: MAG[2],
        temperature: TEMPERATURE_CDEGC,
    }
}

/// SCALED_IMU2: the shape of [`scaled_imu`] with every sine shifted by 1.3 rad; temperature 2500 cdegC.
pub fn scaled_imu2(now_ms: u64) -> SCALED_IMU2_DATA {
    let m = imu_motion(now_ms, 1.3);
    SCALED_IMU2_DATA {
        time_boot_ms: now_ms as u32,
        xacc: m[0],
        yacc: m[1],
        zacc: m[2],
        xgyro: m[3],
        ygyro: m[4],
        zgyro: m[5],
        xmag: MAG[0],
        ymag: MAG[1],
        zmag: MAG[2],
        temperature: TEMPERATURE_CDEGC,
    }
}

/// SCALED_PRESSURE: 1013.25 hPa plus 0.5 hPa sine at 0.1 Hz, 25 degC, temperature_press_diff 0.
pub fn scaled_pressure(now_ms: u64) -> SCALED_PRESSURE_DATA {
    SCALED_PRESSURE_DATA {
        time_boot_ms: now_ms as u32,
        press_abs: PRESSURE_MEAN_HPA + sine(now_ms, 0.5, 0.1, 0.0) as f32,
        press_diff: 0.0,
        temperature: TEMPERATURE_CDEGC,
        temperature_press_diff: 0,
    }
}

/// GPS_RAW_INT: 3D fix at 47.3977 N, 8.5456 E, 488 m, 12 satellites, not moving.
///
/// Extension fields: `alt_ellipsoid` 488000 mm, `h_acc` 1500 mm, `v_acc` 2500 mm, `vel_acc`
/// 200 mm/s, `hdg_acc` 0, `yaw` 0.
pub fn gps_raw_int(now_ms: u64) -> GPS_RAW_INT_DATA {
    GPS_RAW_INT_DATA {
        time_usec: now_ms * 1000,
        lat: LATITUDE_E7,
        lon: LONGITUDE_E7,
        alt: ALTITUDE_MM,
        eph: 100,
        epv: 150,
        vel: 0,
        cog: 0,
        fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
        satellites_visible: 12,
        alt_ellipsoid: ALTITUDE_MM,
        h_acc: 1500,
        v_acc: 2500,
        vel_acc: 200,
        hdg_acc: 0,
        yaw: 0,
    }
}

/// The messages with this id at this time: one, except HIGHRES_IMU, which has one per instance
/// (ids 0 to 3, in that order). Empty when the simulator does not produce the message.
pub fn messages(message_id: u32, now_ms: u64) -> Vec<MavMessage> {
    match message_id {
        HEARTBEAT_DATA::ID => vec![MavMessage::HEARTBEAT(heartbeat())],
        SYS_STATUS_DATA::ID => vec![MavMessage::SYS_STATUS(sys_status())],
        SCALED_IMU_DATA::ID => vec![MavMessage::SCALED_IMU(scaled_imu(now_ms))],
        SCALED_IMU2_DATA::ID => vec![MavMessage::SCALED_IMU2(scaled_imu2(now_ms))],
        HIGHRES_IMU_DATA::ID => (0..HIGHRES_IMU_INSTANCES)
            .filter_map(|id| highres_imu(id, now_ms))
            .map(MavMessage::HIGHRES_IMU)
            .collect(),
        SCALED_PRESSURE_DATA::ID => vec![MavMessage::SCALED_PRESSURE(scaled_pressure(now_ms))],
        GPS_RAW_INT_DATA::ID => vec![MavMessage::GPS_RAW_INT(gps_raw_int(now_ms))],
        _ => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_at_time_zero() {
        let m = scaled_imu(0);
        assert_eq!((m.xacc, m.yacc, m.zacc), (0, 21, -982));
        assert_eq!((m.xgyro, m.ygyro, m.zgyro), (2, 4, 2));
        assert_eq!((m.xmag, m.ymag, m.zmag), (210, -30, 420));
        assert_eq!(m.time_boot_ms, 0);
        assert_eq!(m.temperature, 2500);
    }

    #[test]
    fn imu_at_known_peak() {
        // x accel is 30 sin(2 pi 0.31 t): the first peak is at t = 1/(4 * 0.31) s.
        let t = (1000.0_f64 / (4.0 * 0.31)).round() as u64;
        let m = scaled_imu(t);
        assert_eq!(m.xacc, 30);
        assert_eq!(m.time_boot_ms, t as u32);
    }

    #[test]
    fn imu_varies_and_stays_plausible() {
        let samples: Vec<_> = (0..20_000).step_by(10).map(scaled_imu).collect();
        for s in &samples {
            assert!(s.xacc.abs() <= 30 && s.yacc.abs() <= 25);
            assert!((-1020..=-980).contains(&s.zacc));
            assert!(s.xgyro.abs() <= 5 && s.ygyro.abs() <= 4 && s.zgyro.abs() <= 3);
        }
        assert!(samples.iter().map(|s| s.xacc).max() > Some(25));
        assert!(samples.iter().map(|s| s.xacc).min() < Some(-25));
        assert!(samples.iter().map(|s| s.zgyro).max() > Some(1));
    }

    #[test]
    fn imu2_differs_from_imu_in_phase_only() {
        let (a, b) = (scaled_imu(0), scaled_imu2(0));
        assert_eq!((b.xacc, b.yacc, b.zacc), (29, 19, -1003));
        assert_ne!(a.xacc, b.xacc);
        assert_eq!((a.xmag, a.ymag, a.zmag), (b.xmag, b.ymag, b.zmag));
        assert_eq!(b.time_boot_ms, 0);
        assert_eq!((a.temperature, b.temperature), (2500, 2500));
    }

    fn nan_fields(d: &HIGHRES_IMU_DATA) -> Vec<&'static str> {
        let all = [
            ("xacc", d.xacc),
            ("yacc", d.yacc),
            ("zacc", d.zacc),
            ("xgyro", d.xgyro),
            ("ygyro", d.ygyro),
            ("zgyro", d.zgyro),
            ("xmag", d.xmag),
            ("ymag", d.ymag),
            ("zmag", d.zmag),
            ("abs_pressure", d.abs_pressure),
            ("diff_pressure", d.diff_pressure),
            ("pressure_alt", d.pressure_alt),
            ("temperature", d.temperature),
        ];
        all.iter()
            .filter(|(_, v)| v.is_nan())
            .map(|(n, _)| *n)
            .collect()
    }

    fn assert_close(got: f32, want: f64) {
        assert!(
            (f64::from(got) - want).abs() < 2e-6,
            "{got} instead of {want}"
        );
    }

    #[test]
    fn highres_main_imu_at_time_zero() {
        use HighresImuUpdatedFlags as F;
        let d = highres_imu(0, 0).unwrap();
        assert_eq!((d.id, d.time_usec), (0, 0));
        assert_close(d.xacc, 0.0);
        assert_close(d.yacc, 0.210_367_746);
        assert_close(d.zacc, -9.628_140_515);
        assert_close(d.xgyro, 0.002_397_128);
        assert_close(d.ygyro, 0.003_989_980);
        assert_close(d.zgyro, 0.001_795_416);
        assert_eq!(d.temperature, 25.0);
        assert_eq!(
            nan_fields(&d),
            [
                "xmag",
                "ymag",
                "zmag",
                "abs_pressure",
                "diff_pressure",
                "pressure_alt"
            ]
        );
        assert_eq!(
            d.fields_updated,
            F::HIGHRES_IMU_UPDATED_XACC
                | F::HIGHRES_IMU_UPDATED_YACC
                | F::HIGHRES_IMU_UPDATED_ZACC
                | F::HIGHRES_IMU_UPDATED_XGYRO
                | F::HIGHRES_IMU_UPDATED_YGYRO
                | F::HIGHRES_IMU_UPDATED_ZGYRO
                | F::HIGHRES_IMU_UPDATED_TEMPERATURE
        );
        assert_eq!(d.fields_updated.bits(), 0x103F);
    }

    #[test]
    fn highres_second_imu_has_other_phases() {
        let (a, b) = (highres_imu(0, 0).unwrap(), highres_imu(1, 0).unwrap());
        assert_eq!(b.id, 1);
        assert_close(b.xacc, 0.289_067_456);
        assert_close(b.yacc, 0.186_426_303);
        assert_close(b.zacc, -9.841_549_139);
        assert_close(b.xgyro, 0.004_869_238);
        assert_close(b.ygyro, 0.001_339_953);
        assert_close(b.zgyro, -0.001_835_574);
        assert_ne!(a.xacc, b.xacc);
        assert_eq!(b.temperature, 25.0);
        assert_eq!(nan_fields(&a), nan_fields(&b));
        assert_eq!(a.fields_updated, b.fields_updated);
    }

    #[test]
    fn highres_main_imus_stay_within_their_amplitudes() {
        for id in [0, 1] {
            let samples: Vec<_> = (0..20_000)
                .step_by(10)
                .map(|t| highres_imu(id, t).unwrap())
                .collect();
            for s in &samples {
                assert!(s.xacc.abs() <= 0.3 + 1e-6 && s.yacc.abs() <= 0.25 + 1e-6);
                assert!((-10.01..=-9.609).contains(&s.zacc));
                assert!(s.xgyro.abs() <= 0.005 + 1e-6 && s.ygyro.abs() <= 0.004 + 1e-6);
                assert!(s.zgyro.abs() <= 0.003 + 1e-6);
            }
            assert!(samples.iter().any(|s| s.xacc > 0.25));
            assert!(samples.iter().any(|s| s.xacc < -0.25));
        }
    }

    #[test]
    fn highres_high_g_is_acceleration_only_and_noisy() {
        use HighresImuUpdatedFlags as F;
        let d = highres_imu(2, 0).unwrap();
        assert_eq!(d.id, 2);
        assert_close(d.xacc, -0.026_694_247);
        assert_close(d.yacc, 0.441_277_335);
        assert_close(d.zacc, -9.388_841_217);
        assert_eq!(
            nan_fields(&d),
            [
                "xgyro",
                "ygyro",
                "zgyro",
                "xmag",
                "ymag",
                "zmag",
                "abs_pressure",
                "diff_pressure",
                "pressure_alt",
                "temperature"
            ]
        );
        assert_eq!(
            d.fields_updated,
            F::HIGHRES_IMU_UPDATED_XACC | F::HIGHRES_IMU_UPDATED_YACC | F::HIGHRES_IMU_UPDATED_ZACC
        );
        let at_1s = highres_imu(2, 1000).unwrap();
        assert_close(at_1s.xacc, -0.493_777_498);
        // Noise: the residual of the pure sine is not zero, and never beyond 0.05 m/s^2.
        let mut residuals = Vec::new();
        for t in 0..2000 {
            let s = highres_imu(2, t).unwrap();
            let sine_x = sine(t, 0.5, 1.7, 0.0);
            let r = f64::from(s.xacc) - sine_x;
            assert!(r.abs() <= 0.0501, "{r} at {t}");
            assert!((f64::from(s.zacc) + 9.8).abs() <= 0.5501);
            residuals.push(r);
        }
        assert!(residuals.iter().any(|r| r.abs() > 0.04));
        assert!(residuals.windows(2).filter(|w| w[0] != w[1]).count() > 1900);
    }

    #[test]
    fn highres_magnetometer() {
        use HighresImuUpdatedFlags as F;
        for t in [0, 777, 123_456] {
            let d = highres_imu(3, t).unwrap();
            assert_eq!((d.id, d.time_usec), (3, t * 1000));
            assert_eq!((d.xmag, d.ymag, d.zmag), (0.21, -0.03, 0.42));
            assert_eq!(d.temperature, 25.0);
            assert_eq!(
                nan_fields(&d),
                [
                    "xacc",
                    "yacc",
                    "zacc",
                    "xgyro",
                    "ygyro",
                    "zgyro",
                    "abs_pressure",
                    "diff_pressure",
                    "pressure_alt"
                ]
            );
            assert_eq!(
                d.fields_updated,
                F::HIGHRES_IMU_UPDATED_XMAG
                    | F::HIGHRES_IMU_UPDATED_YMAG
                    | F::HIGHRES_IMU_UPDATED_ZMAG
                    | F::HIGHRES_IMU_UPDATED_TEMPERATURE
            );
        }
        assert!(highres_imu(4, 0).is_none());
    }

    #[test]
    fn pressure_samples() {
        let p0 = scaled_pressure(0);
        assert_eq!(p0.press_abs, 1013.25);
        assert_eq!(p0.temperature, 2500);
        assert_eq!(p0.temperature_press_diff, 0);
        let peak = scaled_pressure(2500);
        assert!((peak.press_abs - 1013.75).abs() < 1e-3);
        assert_eq!(peak.time_boot_ms, 2500);
        let trough = scaled_pressure(7500);
        assert!((trough.press_abs - 1012.75).abs() < 1e-3);
    }

    #[test]
    fn gps_values() {
        let g = gps_raw_int(1234);
        assert_eq!(g.time_usec, 1_234_000);
        assert_eq!((g.lat, g.lon, g.alt), (473_977_000, 85_456_000, 488_000));
        assert_eq!(g.fix_type, GpsFixType::GPS_FIX_TYPE_3D_FIX);
        assert_eq!(g.satellites_visible, 12);
        assert_eq!((g.alt_ellipsoid, g.h_acc, g.v_acc), (488_000, 1500, 2500));
        assert_eq!((g.vel_acc, g.hdg_acc, g.yaw), (200, 0, 0));
    }

    #[test]
    fn sys_status_battery() {
        assert_eq!(sys_status().voltage_battery, 16_000);
    }

    #[test]
    fn heartbeat_fields() {
        let h = heartbeat();
        assert_eq!(h.mavtype, MavType::MAV_TYPE_GENERIC);
        assert_eq!(h.autopilot, MavAutopilot::MAV_AUTOPILOT_GENERIC);
        assert_eq!(h.system_status, MavState::MAV_STATE_STANDBY);
        assert_eq!(h.mavlink_version, 3);
    }

    #[test]
    fn message_lookup() {
        assert!(messages(999, 0).is_empty());
        assert!(matches!(&messages(26, 5)[..], [MavMessage::SCALED_IMU(m)] if *m == scaled_imu(5)));
        let imu = messages(105, 5);
        assert_eq!(imu.len(), 4);
        for (i, m) in imu.iter().enumerate() {
            let MavMessage::HIGHRES_IMU(d) = m else {
                panic!("{m:?}")
            };
            assert_eq!(usize::from(d.id), i);
            assert_eq!(d.time_usec, 5000);
        }
    }
}
