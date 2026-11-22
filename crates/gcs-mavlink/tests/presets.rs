//! Host tests of the preset tables: golden values, and every physical value and score recomputed
//! from datasheet constants written here with their datasheet file:line (files under
//! `_plan/datasheets-text/` of the drivers workspace).

use gcs_mavlink::presets::{
    BARO, GNSS, IMU1, IMU2, Kind, MAG, MARV_DEVICE_BARO, MARV_DEVICE_GNSS, MARV_DEVICE_IMU1,
    MARV_DEVICE_IMU2, MARV_DEVICE_MAG, SENSORS, Sensor, Value,
    Value::{F32, I32},
    preset,
};

fn all() -> [(&'static str, &'static Sensor); 5] {
    [
        ("IMU1", &IMU1),
        ("IMU2", &IMU2),
        ("BARO", &BARO),
        ("MAG", &MAG),
        ("GNSS", &GNSS),
    ]
}

/// The values as integers; an `F32` must be a whole number (the types are checked apart, in
/// `value_types_match_the_firmware_parameters`).
fn ints(values: &[Value]) -> Vec<i32> {
    values
        .iter()
        .map(|v| match v {
            Value::I32(x) => *x,
            Value::F32(x) => {
                assert_eq!(x.fract(), 0.0, "a whole number");
                *x as i32
            }
        })
        .collect()
}

// ---- structure -------------------------------------------------------------------------------

#[test]
fn devices_and_fields() {
    assert_eq!(IMU1.device, MARV_DEVICE_IMU1);
    assert_eq!(IMU2.device, MARV_DEVICE_IMU2);
    assert_eq!(BARO.device, MARV_DEVICE_BARO);
    assert_eq!(MAG.device, MARV_DEVICE_MAG);
    assert_eq!(GNSS.device, MARV_DEVICE_GNSS);
    assert_eq!(
        [
            MARV_DEVICE_IMU1,
            MARV_DEVICE_IMU2,
            MARV_DEVICE_BARO,
            MARV_DEVICE_MAG,
            MARV_DEVICE_GNSS
        ],
        [1, 2, 3, 4, 5]
    );
    assert_eq!(IMU1.fields, ["IMU1_ACC_FS", "IMU1_GYR_FS", "IMU1_ODR_HZ"]);
    assert_eq!(IMU2.fields, ["IMU2_ACC_FS", "IMU2_GYR_FS", "IMU2_ODR_HZ"]);
    assert_eq!(
        BARO.fields,
        [
            "BARO_ODR_HZ",
            "BARO_OSR_P",
            "BARO_OSR_T",
            "BARO_IIR_P",
            "BARO_IIR_T"
        ]
    );
    assert_eq!(MAG.fields, ["MAG_ODR_HZ", "MAG_AVG"]);
    assert_eq!(GNSS.fields, ["GPS_RATE_MS", "GPS_GNSS"]);
}

#[test]
fn sensors_list_is_the_five_in_device_order() {
    assert_eq!(SENSORS.len(), 5);
    let devices: Vec<u8> = SENSORS.iter().map(|s| s.device).collect();
    assert_eq!(
        devices,
        [
            MARV_DEVICE_IMU1,
            MARV_DEVICE_IMU2,
            MARV_DEVICE_BARO,
            MARV_DEVICE_MAG,
            MARV_DEVICE_GNSS
        ]
    );
    for (listed, (_, s)) in SENSORS.iter().zip(all()) {
        assert!(std::ptr::eq(*listed, s));
    }
}

#[test]
fn ids_are_contiguous_from_one_and_default_is_one() {
    for (name, s) in all() {
        let ids: Vec<i32> = s.presets.iter().map(|p| p.id).collect();
        let want: &[i32] = if s.device == MARV_DEVICE_GNSS {
            &[1, 2, 3]
        } else {
            &[1, 2, 3, 4]
        };
        assert_eq!(ids, want, "{name} ids");
        assert_eq!(s.default_id, 1, "{name} default id");
        assert!(preset(s, s.default_id).is_some(), "{name} default exists");
    }
}

#[test]
fn lookup_by_id() {
    for (name, s) in all() {
        for p in s.presets {
            assert_eq!(preset(s, p.id), Some(p), "{name} {}", p.id);
        }
        assert!(preset(s, 0).is_none(), "{name} Custom has no preset");
        assert!(preset(s, 5).is_none(), "{name} id 5");
        assert!(
            preset(s, 4).is_some() != (s.device == MARV_DEVICE_GNSS),
            "{name} id 4"
        );
        assert!(preset(s, -1).is_none(), "{name} id -1");
    }
}

#[test]
fn one_value_per_field_one_physical_and_score_per_axis() {
    for (name, s) in all() {
        for p in s.presets {
            assert_eq!(p.values.len(), s.fields.len(), "{name} {} values", p.id);
            assert_eq!(p.physical.len(), s.axes.len(), "{name} {} physical", p.id);
            assert_eq!(p.scores.len(), s.axes.len(), "{name} {} scores", p.id);
            assert_eq!(p.physical2.len(), s.axes.len(), "{name} {} physical2", p.id);
            for (k, a) in s.axes.iter().enumerate() {
                assert_eq!(
                    p.physical2[k].is_some(),
                    a.unit2.is_some(),
                    "{name} {} {} physical2 vs unit2",
                    p.id,
                    a.name
                );
            }
            assert!(
                p.scores.iter().all(|&x| x <= 100),
                "{name} {} score > 100",
                p.id
            );
            assert!(
                !p.name.is_empty() && !p.summary.is_empty(),
                "{name} {}",
                p.id
            );
            assert!(p.summary.is_ascii() && p.name.is_ascii());
        }
    }
}

#[test]
fn names_are_unique_per_sensor() {
    for (name, s) in all() {
        for (a, pa) in s.presets.iter().enumerate() {
            for pb in &s.presets[a + 1..] {
                assert_ne!(pa.name, pb.name, "{name}");
            }
        }
    }
}

#[test]
fn axes_names_units_ends_and_kinds() {
    use Kind::{Bound, Model, Plain};
    type AxisRow<'a> = (&'a str, &'a str, Option<&'a str>, f32, f32, Kind);
    let expect = |s: &Sensor, rows: &[AxisRow]| {
        assert_eq!(s.axes.len(), rows.len());
        for (a, r) in s.axes.iter().zip(rows) {
            assert_eq!((a.name, a.unit, a.unit2, a.x0, a.x100, a.kind), *r);
        }
    };
    let imu = [
        ("Accel range", "g", None, 2.0, 32.0, Plain),
        ("Gyro range", "dps", None, 125.0, 4000.0, Plain),
        ("Low noise", "dps", Some("mg"), 1.0, 0.01, Model),
        ("Bandwidth", "Hz", None, 10.0, 5000.0, Model),
        ("Light load", "B/s", None, 200_000.0, 200.0, Plain),
    ];
    expect(&IMU1, &imu);
    expect(&IMU2, &imu);
    expect(
        &BARO,
        &[
            ("Low noise", "Pa", Some("cm"), 1.0, 0.01, Plain),
            ("Fast response", "ms", None, 1000.0, 1.0, Plain),
            ("Update rate", "Hz", None, 1.0, 500.0, Plain),
            ("Light load", "B/s", None, 10_000.0, 10.0, Plain),
        ],
    );
    expect(
        &MAG,
        &[
            ("Low noise", "nT", Some("nT z"), 1000.0, 10.0, Model),
            ("Fast response", "ms", None, 1000.0, 1.0, Bound),
            ("Update rate", "Hz", None, 1.0, 500.0, Plain),
            ("Light load", "B/s", None, 10_000.0, 10.0, Plain),
        ],
    );
    expect(
        &GNSS,
        &[
            ("Update rate", "Hz", None, 1.0, 10.0, Plain),
            ("Fast response", "ms", None, 1000.0, 1.0, Bound),
            ("Light load", "B/s", None, 11_520.0, 100.0, Plain),
            ("Sky coverage", "systems", None, 1.0, 4.0, Plain),
        ],
    );
}

// ---- golden ids, names, values (ids and names never change meaning: invariant I10) -----------

#[test]
fn golden_ids_names_values_summaries() {
    type Row = (i32, &'static str, &'static str, &'static [Value]);
    let golden: [(&Sensor, &[Row]); 5] = [
        (
            &IMU1,
            &[
                (
                    1,
                    "Flight",
                    "+-16 g, +-2000 dps, 800 Hz",
                    &[I32(16), I32(2000), I32(800)],
                ),
                (
                    2,
                    "Bench",
                    "+-4 g, +-250 dps, 800 Hz",
                    &[I32(4), I32(250), I32(800)],
                ),
                (
                    3,
                    "Max range",
                    "+-32 g, +-4000 dps, 800 Hz",
                    &[I32(32), I32(4000), I32(800)],
                ),
                (
                    4,
                    "Smooth",
                    "+-8 g, +-1000 dps, 400 Hz",
                    &[I32(8), I32(1000), I32(400)],
                ),
            ],
        ),
        (
            &IMU2,
            &[
                (
                    1,
                    "Flight",
                    "+-16 g, +-2000 dps, 800 Hz",
                    &[I32(16), I32(2000), I32(800)],
                ),
                (
                    2,
                    "Bench",
                    "+-4 g, +-500 dps, 800 Hz",
                    &[I32(4), I32(500), I32(800)],
                ),
                (
                    3,
                    "Max range",
                    "+-16 g, +-4000 dps, 800 Hz",
                    &[I32(16), I32(4000), I32(800)],
                ),
                (
                    4,
                    "Smooth",
                    "+-8 g, +-1000 dps, 400 Hz",
                    &[I32(8), I32(1000), I32(400)],
                ),
            ],
        ),
        (
            &BARO,
            &[
                (
                    1,
                    "Balanced",
                    "50 Hz, x16/x2, no filter",
                    &[I32(50), I32(16), I32(2), I32(0), I32(0)],
                ),
                (
                    2,
                    "Fast",
                    "100 Hz, x8/x1, no filter",
                    &[I32(100), I32(8), I32(1), I32(0), I32(0)],
                ),
                (
                    3,
                    "Smooth",
                    "25 Hz, x32/x2, filter 3",
                    &[I32(25), I32(32), I32(2), I32(3), I32(0)],
                ),
                (
                    4,
                    "Precise",
                    "10 Hz, x128/x8, no filter",
                    &[I32(10), I32(128), I32(8), I32(0), I32(0)],
                ),
            ],
        ),
        (
            &MAG,
            &[
                (1, "Balanced", "100 Hz, average of 2", &[F32(100.0), I32(2)]),
                (2, "Fast", "200 Hz, average of 2", &[F32(200.0), I32(2)]),
                (
                    3,
                    "Low noise",
                    "100 Hz, average of 4",
                    &[F32(100.0), I32(4)],
                ),
                (
                    4,
                    "Ultra low noise",
                    "50 Hz, average of 8",
                    &[F32(50.0), I32(8)],
                ),
            ],
        ),
        (
            &GNSS,
            &[
                (
                    1,
                    "Flight",
                    "5 Hz, GPS+Galileo+BeiDou+QZSS",
                    &[I32(200), I32(0)],
                ),
                (2, "Fast", "10 Hz, GPS+Galileo", &[I32(100), I32(1)]),
                (
                    3,
                    "Max sky",
                    "5 Hz, GPS+Galileo+GLONASS",
                    &[I32(200), I32(2)],
                ),
            ],
        ),
    ];
    for (s, rows) in golden {
        assert_eq!(s.presets.len(), rows.len());
        for (p, r) in s.presets.iter().zip(rows) {
            assert_eq!(
                (p.id, p.name, p.summary, p.values),
                (r.0, r.1, r.2, r.3),
                "{}",
                s.fields[0]
            );
        }
    }
}

/// The governed fields the firmware declares as REAL32; every other governed field is I32.
/// `MAG_ODR_HZ`: fw-settings `mag.rs:32` (`ParamDef::f32(ODR_HZ, DEVICE, 100.0, 1.5625, 400.0, ..)`);
/// `MAG_AVG`: `mag.rs:33` (`ParamDef::i32`); `BARO_*`: `baro.rs:31-35` (`ParamDef::i32`);
/// `IMU1_*` and `IMU2_*` fields: `preset.rs:26-28, 30-32` (`ParamDef::i32`).
const FIRMWARE_F32_FIELDS: [&str; 1] = ["MAG_ODR_HZ"];

/// Every value has the type of its field's firmware parameter (a preset rule built from a wrong
/// type is refused by the firmware).
#[test]
fn value_types_match_the_firmware_parameters() {
    for (name, s) in all() {
        for p in s.presets {
            for (field, v) in s.fields.iter().zip(p.values) {
                match v {
                    F32(_) => assert!(
                        FIRMWARE_F32_FIELDS.contains(field),
                        "{name} {}: {field} must be I32",
                        p.id
                    ),
                    I32(_) => assert!(
                        !FIRMWARE_F32_FIELDS.contains(field),
                        "{name} {}: {field} must be F32",
                        p.id
                    ),
                }
            }
        }
    }
}

/// Preset 1 equals the firmware's field defaults (the IMU parameters 70 to 75 and the BARO and MAG
/// defaults).
#[test]
fn preset_one_equals_the_firmware_field_defaults() {
    let defaults: [(&Sensor, &[(&str, i32)]); 5] = [
        (
            &IMU1,
            &[
                ("IMU1_ACC_FS", 16),
                ("IMU1_GYR_FS", 2000),
                ("IMU1_ODR_HZ", 800),
            ],
        ),
        (
            &IMU2,
            &[
                ("IMU2_ACC_FS", 16),
                ("IMU2_GYR_FS", 2000),
                ("IMU2_ODR_HZ", 800),
            ],
        ),
        (
            &BARO,
            &[
                ("BARO_ODR_HZ", 50),
                ("BARO_OSR_P", 16),
                ("BARO_OSR_T", 2),
                ("BARO_IIR_P", 0),
                ("BARO_IIR_T", 0),
            ],
        ),
        (&MAG, &[("MAG_ODR_HZ", 100), ("MAG_AVG", 2)]),
        (&GNSS, &[("GPS_RATE_MS", 200), ("GPS_GNSS", 0)]),
    ];
    for (s, d) in defaults {
        let p = preset(s, s.default_id).unwrap();
        assert_eq!(s.fields.len(), d.len());
        for (i, (field, want)) in d.iter().enumerate() {
            assert_eq!(s.fields[i], *field);
            let want = if FIRMWARE_F32_FIELDS.contains(field) {
                F32(*want as f32)
            } else {
                I32(*want)
            };
            assert_eq!(p.values[i], want, "{field}");
        }
    }
}

// ---- recomputation from datasheet constants --------------------------------------------------

/// `clamp(ln(x / x0) / ln(x100 / x0), 0, 1)`.
fn fraction(x: f64, x0: f64, x100: f64) -> f64 {
    ((x / x0).ln() / (x100 / x0).ln()).clamp(0.0, 1.0)
}

/// `round_half_up(100 * f)`.
fn score(f: f64) -> u8 {
    (100.0 * f + 0.5).floor() as u8
}

/// Datasheet and driver constants of one IMU chip.
struct Chip {
    /// Gyro noise density, dps/rtHz.
    gyro_nd: f64,
    /// Accelerometer noise density, mg/rtHz, by full scale in g.
    accel_nd: fn(i32) -> f64,
    /// Gyro and accelerometer LSB (dps, mg), by full scale.
    gyro_lsb: fn(i32) -> f64,
    accel_lsb: fn(i32) -> f64,
    /// SPI bytes per sample (the address byte included).
    bytes: f64,
}

const ICM: Chip = Chip {
    // ICM-45686.txt:767 "Rate Noise Spectral Density ... 0.0038 dps/rtHz".
    gyro_nd: 0.0038,
    // ICM-45686.txt:824-826 "70 ug/rtHz up to 8 g, 80 ug/rtHz at 16 g, 110 ug/rtHz at 32 g".
    accel_nd: |g| match g {
        2..=8 => 0.070,
        16 => 0.080,
        32 => 0.110,
        _ => panic!("ICM accel FS {g}"),
    },
    // Driver: full scale / 32768 (icm45686 config.rs:45-48, 96-99); datasheet sensitivity
    // ICM-45686.txt:746-750 (gyro, LSB/dps) and :804-808 (accel, LSB/g) agree to rounding.
    gyro_lsb: |dps| f64::from(dps) / 32768.0,
    accel_lsb: |g| f64::from(g) * 1000.0 / 32768.0,
    // icm45686 fifo_io.rs:204-205 (INT1_STATUS0: 2 bytes with address) and lib.rs:429-433
    // (sample: 14 bytes with address) in the design's arithmetic.
    bytes: 18.0,
};

const LSM: Chip = Chip {
    // LSM6DSK320X.txt:655 "Rate noise density in high-performance mode 3.8 mdps/rtHz".
    gyro_nd: 0.0038,
    // LSM6DSK320X.txt:657 "60 ug/rtHz" for FS 2 to 16 g.
    accel_nd: |g| match g {
        2..=16 => 0.060,
        _ => panic!("LSM accel FS {g}"),
    },
    // LSM6DSK320X.txt:618-622 gyro sensitivity, mdps/LSB.
    gyro_lsb: |dps| {
        let mdps = match dps {
            250 => 8.75,
            500 => 17.5,
            1000 => 35.0,
            2000 => 70.0,
            4000 => 140.0,
            _ => panic!("LSM gyro FS {dps}"),
        };
        mdps / 1000.0
    },
    // LSM6DSK320X.txt:608-613 low-g sensitivity, mg/LSB.
    accel_lsb: |g| match g {
        2 => 0.061,
        4 => 0.122,
        8 => 0.244,
        16 => 0.488,
        _ => panic!("LSM accel FS {g}"),
    },
    // lsm6dsk320x lib.rs:494 `read_all` reads `RawAll::LEN` = 26 bytes (sample.rs:92, from
    // OUT_TEMP_L to UI_OUTZ_H_A_OIS_HG, asserted at sample.rs:172) plus the address byte, 27;
    // measured by F2's mock-bus test (the design's 21 was wrong).
    bytes: 27.0,
};

/// A recomputed preset: physical values and second physical values in axis order (IMU Low noise:
/// the gyro's rms in dps, then the accelerometer's in mg), and the scores.
struct Computed {
    physical: Vec<f64>,
    physical2: Vec<Option<f64>>,
    scores: Vec<u8>,
}

fn imu(chip: &Chip, p: &gcs_mavlink::presets::Preset) -> Computed {
    let v = ints(p.values);
    let (acc, gyr, odr) = (v[0], v[1], f64::from(v[2]));
    let bandwidth = odr / 2.0; // LSM6DSK320X.txt:722 "Noise RMS related to BW = ODR/2"
    let gyro = (chip.gyro_nd.powi(2) * bandwidth + (chip.gyro_lsb)(gyr).powi(2) / 12.0).sqrt();
    let accel =
        ((chip.accel_nd)(acc).powi(2) * bandwidth + (chip.accel_lsb)(acc).powi(2) / 12.0).sqrt();
    let load = odr * chip.bytes;
    // Axis ends of design 1.2.
    let noise = (fraction(gyro, 1.0, 0.01) + fraction(accel, 10.0, 0.1)) / 2.0;
    Computed {
        physical: vec![f64::from(acc), f64::from(gyr), gyro, bandwidth, load],
        physical2: vec![None, None, Some(accel), None, None],
        scores: vec![
            score(fraction(f64::from(acc), 2.0, 32.0)),
            score(fraction(f64::from(gyr), 125.0, 4000.0)),
            score(noise),
            score(fraction(bandwidth, 10.0, 5000.0)),
            score(fraction(load, 200_000.0, 200.0)),
        ],
    }
}

/// BMP581.txt:646-653 tconv_p typical, ms, by OSR_P.
fn tconv_p(osr: i32) -> f64 {
    match osr {
        1 => 1.0,
        2 => 1.7,
        4 => 2.9,
        8 => 5.4,
        16 => 10.4,
        32 => 20.4,
        64 => 40.4,
        128 => 80.4,
        _ => panic!("OSR_P {osr}"),
    }
}

/// BMP581.txt:654-661 tconv_t typical, ms, by OSR_T.
fn tconv_t(osr: i32) -> f64 {
    match osr {
        1 => 1.0,
        2 => 1.1,
        4 => 1.5,
        8 => 2.1,
        16 => 3.3,
        32 => 5.8,
        64 => 10.8,
        128 => 20.8,
        _ => panic!("OSR_T {osr}"),
    }
}

/// BMP581.txt:982-989 RMS noise at 100 kPa, Pa, by OSR_P.
fn sigma_t9(osr: i32) -> f64 {
    match osr {
        1 => 0.78,
        2 => 0.58,
        4 => 0.41,
        8 => 0.30,
        16 => 0.21,
        32 => 0.15,
        64 => 0.11,
        128 => 0.08,
        _ => panic!("OSR_P {osr}"),
    }
}

fn baro(p: &gcs_mavlink::presets::Preset) -> Computed {
    let v = ints(p.values);
    let (odr, osr_p, osr_t, iir_p) = (f64::from(v[0]), v[1], v[2], f64::from(v[3]));
    // The IIR of BMP581.txt:1003-1019 for independent samples: variance ratio 1 / (2c + 1).
    let noise = sigma_t9(osr_p) / (2.0 * iir_p + 1.0).sqrt();
    // tconv_p + tconv_t + c / ODR (the IIR's delay at zero frequency is c samples).
    let delay = tconv_p(osr_p) + tconv_t(osr_t) + iir_p / odr * 1000.0;
    // 13 bytes per sample: baro.rs:197-202, bmp581 lib.rs:438-442, 478.
    let load = odr * 13.0;
    Computed {
        physical: vec![noise, delay, odr, load],
        // Altitude noise, cm: 0.083 m/Pa (1 / (rho g), ISA sea level; design 1.2, display only).
        physical2: vec![Some(noise * 0.083 * 100.0), None, None, None],
        scores: vec![
            score(fraction(noise, 1.0, 0.01)),
            score(fraction(delay, 1000.0, 1.0)),
            score(fraction(odr, 1.0, 500.0)),
            score(fraction(load, 10_000.0, 10.0)),
        ],
    }
}

fn mag(p: &gcs_mavlink::presets::Preset) -> Computed {
    let v = ints(p.values);
    let (odr, avg) = (f64::from(v[0]), f64::from(v[1]));
    // BMM350.txt:492-497: 190 nT rms (x/y), ODR 100, averaging 2, BW = ODR/2; scaled by 1/sqrt(N).
    let noise = 190.0 * (2.0 / avg).sqrt();
    // Delay at most N x 2.5 ms: Table 5 fits N = 1 at 400 Hz, 2 at 200, 4 at 100, 8 at 50
    // (BMM350.txt:712-730).
    let delay = avg * 2.5;
    // 23 bytes per sample: bmm350 lib.rs:996-1016, 1025-1037, 1074-1076; mag.rs:193-194.
    let load = odr * 23.0;
    Computed {
        physical: vec![noise, delay, odr, load],
        // z channel: 450 nT at the datasheet point (BMM350.txt:495-496), same 1/sqrt(N) scaling.
        physical2: vec![Some(450.0 * (2.0 / avg).sqrt()), None, None, None],
        scores: vec![
            score(fraction(noise, 1000.0, 10.0)),
            score(fraction(delay, 1000.0, 1.0)),
            score(fraction(odr, 1.0, 500.0)),
            score(fraction(load, 10_000.0, 10.0)),
        ],
    }
}

fn gnss(p: &gcs_mavlink::presets::Preset) -> Computed {
    let v = ints(p.values);
    let (interval, set) = (f64::from(v[0]), v[1]);
    let rate = 1000.0 / interval;
    // A fix is at most one interval old at the receiver (latency excluded).
    let delay = interval;
    // One NAV-PVT frame is 6 B header + 92 B payload + 2 B checksum (u-blox M10 interface description).
    let load = rate * (6.0 + 92.0 + 2.0);
    // GPS_GNSS 0: GPS, Galileo, BeiDou B1C, QZSS (QZSS counted with GPS: 4 systems); 1: GPS, Galileo;
    // 2: GPS, Galileo, GLONASS.
    let systems = match set {
        0 => 4.0,
        1 => 2.0,
        2 => 3.0,
        _ => panic!("GPS_GNSS {set}"),
    };
    Computed {
        physical: vec![rate, delay, load, systems],
        physical2: vec![None; 4],
        scores: vec![
            score(fraction(rate, 1.0, 10.0)),
            score(fraction(delay, 1000.0, 1.0)),
            score(fraction(load, 11_520.0, 100.0)),
            score(fraction(systems, 1.0, 4.0)),
        ],
    }
}

fn compute(device: u8, p: &gcs_mavlink::presets::Preset) -> Computed {
    match device {
        MARV_DEVICE_IMU1 => imu(&ICM, p),
        MARV_DEVICE_IMU2 => imu(&LSM, p),
        MARV_DEVICE_BARO => baro(p),
        MARV_DEVICE_MAG => mag(p),
        MARV_DEVICE_GNSS => gnss(p),
        _ => unreachable!(),
    }
}

#[test]
fn every_physical_value_and_score_is_recomputed() {
    for (name, s) in all() {
        for p in s.presets {
            let c = compute(s.device, p);
            println!(
                "{name} {} {:<16} physical {:?} physical2 {:?} scores {:?} (stored {:?})",
                p.id, p.name, c.physical, c.physical2, c.scores, p.scores
            );
            for (k, axis) in s.axes.iter().enumerate() {
                let (stored, want) = (f64::from(p.physical[k]), c.physical[k]);
                assert!(
                    (stored - want).abs() <= 0.005 * want.abs(),
                    "{name} {} {}: stored {stored} vs recomputed {want}",
                    p.id,
                    axis.name
                );
                match (p.physical2[k], c.physical2[k]) {
                    (None, None) => {}
                    (Some(stored), Some(want)) => assert!(
                        (f64::from(stored) - want).abs() <= 0.005 * want.abs(),
                        "{name} {} {}: stored second {stored} vs recomputed {want}",
                        p.id,
                        axis.name
                    ),
                    (a, b) => panic!("{name} {} {}: second value {a:?} vs {b:?}", p.id, axis.name),
                }
                assert_eq!(
                    p.scores[k], c.scores[k],
                    "{name} {} {}: stored score vs recomputed",
                    p.id, axis.name
                );
            }
        }
    }
}

/// The scores of the design's section 2 (independent script), preset by preset.
#[test]
fn scores_equal_the_design_section_two() {
    let design: [(&Sensor, &[&[u8]]); 5] = [
        (
            &IMU1,
            &[
                &[75, 80, 48, 59, 38],
                &[25, 20, 49, 59, 38],
                &[100, 100, 43, 59, 38],
                &[50, 60, 57, 48, 48],
            ],
        ),
        (
            &IMU2,
            &[
                // amended 2026-11-17: 27 bytes measured (Light load 32 and 42, was 36 and 46)
                &[75, 80, 51, 59, 32],
                &[25, 40, 51, 59, 32],
                &[75, 100, 50, 59, 32],
                &[50, 60, 58, 48, 42],
            ],
        ),
        (
            &BARO,
            &[
                &[34, 65, 63, 40],
                &[26, 73, 74, 30],
                &[62, 28, 52, 50],
                &[55, 36, 37, 63],
            ],
        ),
        (
            &MAG,
            &[
                &[36, 77, 74, 21],
                &[36, 77, 85, 11],
                &[44, 67, 74, 21],
                &[51, 57, 63, 31],
            ],
        ),
        (
            &GNSS,
            &[
                // Update rate, Fast response, Light load, Sky coverage (independent: ln fractions of the ends)
                &[70, 23, 66, 100],
                &[100, 33, 51, 50],
                &[70, 23, 66, 79],
            ],
        ),
    ];
    for (s, rows) in design {
        assert_eq!(s.presets.len(), rows.len());
        for (p, row) in s.presets.iter().zip(rows.iter()) {
            assert_eq!(p.scores, *row, "{} {}", s.fields[0], p.id);
        }
    }
}

#[test]
fn range_axes_show_the_field_values_and_bounds_are_ordered() {
    for s in [&IMU1, &IMU2] {
        for p in s.presets {
            let v = ints(p.values);
            assert_eq!(p.physical[0], v[0] as f32);
            assert_eq!(p.physical[1], v[1] as f32);
        }
    }
    for (_, s) in all() {
        for a in s.axes {
            assert!(a.x0 > 0.0 && a.x100 > 0.0 && a.x0 != a.x100, "{}", a.name);
        }
    }
}
