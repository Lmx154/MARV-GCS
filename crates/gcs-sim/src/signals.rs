//! Sensor values of the simulated vehicle, each a pure function of the time in milliseconds.
//!
//! The simulated board lies still, with a small visible motion so plots are not flat lines.
//! `time_boot_ms` of every message is `now_ms` (GPS and HIGHRES_IMU `time_usec` is `now_ms * 1000`).
//!
//! The default streams are those of the real firmware on its USB port: HEARTBEAT, SYS_STATUS, GPS_RAW_INT
//! (1 Hz), HIGHRES_IMU (IMU1 as `id` 0, with the magnetometer in `xmag`/`ymag`/`zmag`), SCALED_IMU2 (IMU2, in mG
//! and mrad/s) and SCALED_PRESSURE (25 Hz), and RC_CHANNELS (5 Hz). [`scaled_imu`] is not a default stream
//! (the link contract rejects SCALED_IMU: it stops at 32.7 g) but a custom [`SimConfig`](crate::SimConfig)
//! can still stream it.

use mavlink::MessageData;
use mavlink::dialects::common::{
    GPS_RAW_INT_DATA, GpsFixType, HEARTBEAT_DATA, HIGHRES_IMU_DATA, HighresImuUpdatedFlags,
    MavAutopilot, MavMessage, MavModeFlag, MavState, MavSysStatusSensor,
    MavSysStatusSensorExtended, MavType, RC_CHANNELS_DATA, SCALED_IMU_DATA, SCALED_IMU2_DATA,
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
/// Battery current in cA (1.20 A).
pub const BATTERY_CA: i16 = 120;
/// RSSI of RC_CHANNELS (0 to 254; 255 is unknown).
pub const RC_RSSI: u8 = 200;

/// Constant magnetometer reading in mGauss.
const MAG: [i16; 3] = [210, -30, 420];
/// Number of HIGHRES_IMU instances: the real firmware sends IMU1 only, as `id` 0.
pub const HIGHRES_IMU_INSTANCES: u8 = 1;
/// Standard gravity as the main IMUs measure it, in m/s^2 (z axis, board lying still).
pub const GRAVITY_MS2: f64 = 9.81;
/// Constant magnetic field in gauss.
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

/// HIGHRES_IMU as the real firmware sends it: `id` 0 only (IMU1), anything else is `None`. SI units;
/// `time_usec` is `now_ms * 1000`; acceleration (see `imu_motion_si`: (0, 0, -9.81) m/s^2 plus sines up to
/// 0.3 m/s^2), rotation (sines of a few mrad/s), temperature 25.0 degC and the magnetic field
/// (0.21, -0.03, 0.42) gauss in `xmag`, `ymag`, `zmag`; the pressure fields are NaN and their
/// `fields_updated` bits clear.
pub fn highres_imu(id: u8, now_ms: u64) -> Option<HIGHRES_IMU_DATA> {
    use HighresImuUpdatedFlags as F;
    if id != 0 {
        return None;
    }
    let nan = f32::NAN;
    let m = imu_motion_si(now_ms, 0.0);
    Some(HIGHRES_IMU_DATA {
        time_usec: now_ms * 1000,
        xacc: m[0] as f32,
        yacc: m[1] as f32,
        zacc: m[2] as f32,
        xgyro: m[3] as f32,
        ygyro: m[4] as f32,
        zgyro: m[5] as f32,
        xmag: MAG_GAUSS[0],
        ymag: MAG_GAUSS[1],
        zmag: MAG_GAUSS[2],
        abs_pressure: nan,
        diff_pressure: nan,
        pressure_alt: nan,
        temperature: TEMPERATURE_DEGC,
        fields_updated: F::HIGHRES_IMU_UPDATED_XACC
            | F::HIGHRES_IMU_UPDATED_YACC
            | F::HIGHRES_IMU_UPDATED_ZACC
            | F::HIGHRES_IMU_UPDATED_XGYRO
            | F::HIGHRES_IMU_UPDATED_YGYRO
            | F::HIGHRES_IMU_UPDATED_ZGYRO
            | F::HIGHRES_IMU_UPDATED_XMAG
            | F::HIGHRES_IMU_UPDATED_YMAG
            | F::HIGHRES_IMU_UPDATED_ZMAG
            | F::HIGHRES_IMU_UPDATED_TEMPERATURE,
        id,
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

/// SYS_STATUS as the firmware's board sensor fills it: battery 16000 mV and 1.20 A (120 cA), the
/// remaining percentage unknown (-1), everything else zero.
pub fn sys_status() -> SYS_STATUS_DATA {
    SYS_STATUS_DATA {
        voltage_battery: BATTERY_MV,
        current_battery: BATTERY_CA,
        battery_remaining: -1,
        onboard_control_sensors_present: MavSysStatusSensor::empty(),
        onboard_control_sensors_enabled: MavSysStatusSensor::empty(),
        onboard_control_sensors_health: MavSysStatusSensor::empty(),
        onboard_control_sensors_present_extended: MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_enabled_extended: MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_health_extended: MavSysStatusSensorExtended::empty(),
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

/// SCALED_IMU2 as the firmware sends it (IMU2: acceleration in mG, rotation in mrad/s): the motion of
/// [`scaled_imu`] with every sine shifted by 1.3 rad; the magnetometer fields 0 (IMU2 has none);
/// temperature 2500 cdegC.
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
        xmag: 0,
        ymag: 0,
        zmag: 0,
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

/// RC_CHANNELS as the firmware sends it: 16 channels in microseconds (`chancount` 16, channels 17 and 18
/// 65535 = not used), `rssi` [`RC_RSSI`]. Channel `n` (1 to 16) is 1500 us plus a sine of up to 400 us
/// at (0.1 + 0.05 n) Hz, except channel 3 (throttle), which is 1000 us plus 0 to 1000 us.
pub fn rc_channels(now_ms: u64) -> RC_CHANNELS_DATA {
    let mut ch = [0u16; 16];
    for (i, slot) in ch.iter_mut().enumerate() {
        let n = (i + 1) as f64;
        *slot = if i == 2 {
            (1500.0 + sine(now_ms, 500.0, 0.1, 0.0)).round() as u16
        } else {
            (1500.0 + sine(now_ms, 400.0, 0.1 + 0.05 * n, n)).round() as u16
        };
    }
    RC_CHANNELS_DATA {
        time_boot_ms: now_ms as u32,
        chancount: 16,
        chan1_raw: ch[0],
        chan2_raw: ch[1],
        chan3_raw: ch[2],
        chan4_raw: ch[3],
        chan5_raw: ch[4],
        chan6_raw: ch[5],
        chan7_raw: ch[6],
        chan8_raw: ch[7],
        chan9_raw: ch[8],
        chan10_raw: ch[9],
        chan11_raw: ch[10],
        chan12_raw: ch[11],
        chan13_raw: ch[12],
        chan14_raw: ch[13],
        chan15_raw: ch[14],
        chan16_raw: ch[15],
        chan17_raw: u16::MAX,
        chan18_raw: u16::MAX,
        rssi: RC_RSSI,
    }
}

/// The messages with this id at this time: one (HIGHRES_IMU has one instance, `id` 0). Empty when the
/// simulator does not produce the message.
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
        RC_CHANNELS_DATA::ID => vec![MavMessage::RC_CHANNELS(rc_channels(now_ms))],
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
    fn imu2_differs_from_imu_in_phase_only_and_has_no_magnetometer() {
        let (a, b) = (scaled_imu(0), scaled_imu2(0));
        assert_eq!((b.xacc, b.yacc, b.zacc), (29, 19, -1003));
        assert_ne!(a.xacc, b.xacc);
        assert_eq!((b.xmag, b.ymag, b.zmag), (0, 0, 0));
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
    fn highres_is_imu1_with_the_magnetometer_as_the_firmware_sends_it() {
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
        assert_eq!((d.xmag, d.ymag, d.zmag), (0.21, -0.03, 0.42));
        assert_eq!(
            nan_fields(&d),
            ["abs_pressure", "diff_pressure", "pressure_alt"]
        );
        assert_eq!(
            d.fields_updated,
            F::HIGHRES_IMU_UPDATED_XACC
                | F::HIGHRES_IMU_UPDATED_YACC
                | F::HIGHRES_IMU_UPDATED_ZACC
                | F::HIGHRES_IMU_UPDATED_XGYRO
                | F::HIGHRES_IMU_UPDATED_YGYRO
                | F::HIGHRES_IMU_UPDATED_ZGYRO
                | F::HIGHRES_IMU_UPDATED_XMAG
                | F::HIGHRES_IMU_UPDATED_YMAG
                | F::HIGHRES_IMU_UPDATED_ZMAG
                | F::HIGHRES_IMU_UPDATED_TEMPERATURE
        );
        assert_eq!(d.fields_updated.bits(), 0x11FF);
        // Only IMU1 exists: no second instance, no high-g, no separate magnetometer.
        for id in 1..=4 {
            assert!(highres_imu(id, 0).is_none(), "id {id}");
        }
    }

    #[test]
    fn highres_stays_within_its_amplitudes() {
        let samples: Vec<_> = (0..20_000)
            .step_by(10)
            .map(|t| highres_imu(0, t).unwrap())
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

    #[test]
    fn rc_channels_are_sixteen_channels_in_microseconds() {
        for t in [0, 1234, 50_000] {
            let m = rc_channels(t);
            assert_eq!((m.chancount, m.rssi, m.time_boot_ms), (16, 200, t as u32));
            assert_eq!((m.chan17_raw, m.chan18_raw), (u16::MAX, u16::MAX));
            for us in [
                m.chan1_raw,
                m.chan2_raw,
                m.chan3_raw,
                m.chan4_raw,
                m.chan5_raw,
                m.chan6_raw,
                m.chan7_raw,
                m.chan8_raw,
                m.chan9_raw,
                m.chan10_raw,
                m.chan11_raw,
                m.chan12_raw,
                m.chan13_raw,
                m.chan14_raw,
                m.chan15_raw,
                m.chan16_raw,
            ] {
                assert!((1000..=2000).contains(&us), "{us} at {t} ms");
            }
        }
        assert_ne!(rc_channels(0).chan1_raw, rc_channels(1234).chan1_raw);
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
        let m = sys_status();
        assert_eq!((m.voltage_battery, m.current_battery), (16_000, 120));
        assert_eq!(m.battery_remaining, -1);
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
        assert_eq!(imu.len(), 1);
        let MavMessage::HIGHRES_IMU(d) = &imu[0] else {
            panic!("{:?}", imu[0])
        };
        assert_eq!((d.id, d.time_usec), (0, 5000));
        assert!(
            matches!(&messages(65, 7)[..], [MavMessage::RC_CHANNELS(m)] if *m == rc_channels(7))
        );
    }
}
