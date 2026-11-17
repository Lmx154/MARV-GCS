use super::{
    Axis, Kind, MARV_DEVICE_BARO, MARV_DEVICE_IMU1, MARV_DEVICE_IMU2, MARV_DEVICE_MAG, Preset,
    Sensor, Value,
};

const fn i(v: i32) -> Value {
    Value::I32(v)
}

const fn f(v: f32) -> Value {
    Value::F32(v)
}

const fn axis(
    name: &'static str,
    unit: &'static str,
    unit2: Option<&'static str>,
    x0: f32,
    x100: f32,
    kind: Kind,
) -> Axis {
    Axis {
        name,
        unit,
        unit2,
        x0,
        x100,
        kind,
    }
}

const fn preset(
    id: i32,
    name: &'static str,
    summary: &'static str,
    values: &'static [Value],
    physical: &'static [f32],
    physical2: &'static [Option<f32>],
    scores: &'static [u8],
) -> Preset {
    Preset {
        id,
        name,
        summary,
        values,
        physical,
        physical2,
        scores,
    }
}

const IMU_AXES: &[Axis] = &[
    axis("Accel range", "g", None, 2.0, 32.0, Kind::Plain),
    axis("Gyro range", "dps", None, 125.0, 4000.0, Kind::Plain),
    axis("Low noise", "dps", Some("mg"), 1.0, 0.01, Kind::Model),
    axis("Bandwidth", "Hz", None, 10.0, 5000.0, Kind::Model),
    axis("Light load", "B/s", None, 200_000.0, 200.0, Kind::Plain),
];

const BARO_AXES: &[Axis] = &[
    axis("Low noise", "Pa", Some("cm"), 1.0, 0.01, Kind::Plain),
    axis("Fast response", "ms", None, 1000.0, 1.0, Kind::Plain),
    axis("Update rate", "Hz", None, 1.0, 500.0, Kind::Plain),
    axis("Light load", "B/s", None, 10_000.0, 10.0, Kind::Plain),
];

const MAG_AXES: &[Axis] = &[
    axis("Low noise", "nT", Some("nT z"), 1000.0, 10.0, Kind::Model),
    axis("Fast response", "ms", None, 1000.0, 1.0, Kind::Bound),
    axis("Update rate", "Hz", None, 1.0, 500.0, Kind::Plain),
    axis("Light load", "B/s", None, 10_000.0, 10.0, Kind::Plain),
];

/// IMU1, ICM-45686.
pub static IMU1: Sensor = Sensor {
    device: MARV_DEVICE_IMU1,
    fields: &["IMU1_ACC_FS", "IMU1_GYR_FS", "IMU1_ODR_HZ"],
    default_id: 1,
    axes: IMU_AXES,
    presets: &[
        preset(
            1,
            "Flight",
            "+-16 g, +-2000 dps, 800 Hz",
            &[i(16), i(2000), i(800)],
            &[16.0, 2000.0, 0.07802, 400.0, 14_400.0],
            &[None, None, Some(1.606), None, None],
            &[75, 80, 48, 59, 38],
        ),
        preset(
            2,
            "Bench",
            "+-4 g, +-250 dps, 800 Hz",
            &[i(4), i(250), i(800)],
            &[4.0, 250.0, 0.07603, 400.0, 14_400.0],
            &[None, None, Some(1.400), None, None],
            &[25, 20, 49, 59, 38],
        ),
        preset(
            3,
            "Max range",
            "+-32 g, +-4000 dps, 800 Hz",
            &[i(32), i(4000), i(800)],
            &[32.0, 4000.0, 0.08377, 400.0, 14_400.0],
            &[None, None, Some(2.218), None, None],
            &[100, 100, 43, 59, 38],
        ),
        preset(
            4,
            "Smooth",
            "+-8 g, +-1000 dps, 400 Hz",
            &[i(8), i(1000), i(400)],
            &[8.0, 1000.0, 0.05446, 200.0, 7_200.0],
            &[None, None, Some(0.9925), None, None],
            &[50, 60, 57, 48, 48],
        ),
    ],
};

/// IMU2, LSM6DSK320X.
pub static IMU2: Sensor = Sensor {
    device: MARV_DEVICE_IMU2,
    fields: &["IMU2_ACC_FS", "IMU2_GYR_FS", "IMU2_ODR_HZ"],
    default_id: 1,
    axes: IMU_AXES,
    presets: &[
        preset(
            1,
            "Flight",
            "+-16 g, +-2000 dps, 800 Hz",
            &[i(16), i(2000), i(800)],
            &[16.0, 2000.0, 0.07864, 400.0, 21_600.0],
            &[None, None, Some(1.208), None, None],
            &[75, 80, 51, 59, 32],
        ),
        preset(
            2,
            "Bench",
            "+-4 g, +-500 dps, 800 Hz",
            &[i(4), i(500), i(800)],
            &[4.0, 500.0, 0.07617, 400.0, 21_600.0],
            &[None, None, Some(1.201), None, None],
            &[25, 40, 51, 59, 32],
        ),
        preset(
            3,
            "Max range",
            "+-16 g, +-4000 dps, 800 Hz",
            &[i(16), i(4000), i(800)],
            &[16.0, 4000.0, 0.08608, 400.0, 21_600.0],
            &[None, None, Some(1.208), None, None],
            &[75, 100, 50, 59, 32],
        ),
        preset(
            4,
            "Smooth",
            "+-8 g, +-1000 dps, 400 Hz",
            &[i(8), i(1000), i(400)],
            &[8.0, 1000.0, 0.05468, 200.0, 10_800.0],
            &[None, None, Some(0.8514), None, None],
            &[50, 60, 58, 48, 42],
        ),
    ],
};

/// BARO, BMP581.
pub static BARO: Sensor = Sensor {
    device: MARV_DEVICE_BARO,
    fields: &[
        "BARO_ODR_HZ",
        "BARO_OSR_P",
        "BARO_OSR_T",
        "BARO_IIR_P",
        "BARO_IIR_T",
    ],
    default_id: 1,
    axes: BARO_AXES,
    presets: &[
        preset(
            1,
            "Balanced",
            "50 Hz, x16/x2, no filter",
            &[i(50), i(16), i(2), i(0), i(0)],
            &[0.21, 11.5, 50.0, 650.0],
            &[Some(1.743), None, None, None],
            &[34, 65, 63, 40],
        ),
        preset(
            2,
            "Fast",
            "100 Hz, x8/x1, no filter",
            &[i(100), i(8), i(1), i(0), i(0)],
            &[0.30, 6.4, 100.0, 1_300.0],
            &[Some(2.490), None, None, None],
            &[26, 73, 74, 30],
        ),
        preset(
            3,
            "Smooth",
            "25 Hz, x32/x2, filter 3",
            &[i(25), i(32), i(2), i(3), i(0)],
            &[0.05669, 141.5, 25.0, 325.0],
            &[Some(0.4706), None, None, None],
            &[62, 28, 52, 50],
        ),
        preset(
            4,
            "Precise",
            "10 Hz, x128/x8, no filter",
            &[i(10), i(128), i(8), i(0), i(0)],
            &[0.08, 82.5, 10.0, 130.0],
            &[Some(0.6640), None, None, None],
            &[55, 36, 37, 63],
        ),
    ],
};

/// MAG, BMM350.
pub static MAG: Sensor = Sensor {
    device: MARV_DEVICE_MAG,
    fields: &["MAG_ODR_HZ", "MAG_AVG"],
    default_id: 1,
    axes: MAG_AXES,
    presets: &[
        preset(
            1,
            "Balanced",
            "100 Hz, average of 2",
            &[f(100.0), i(2)],
            &[190.0, 5.0, 100.0, 2_300.0],
            &[Some(450.0), None, None, None],
            &[36, 77, 74, 21],
        ),
        preset(
            2,
            "Fast",
            "200 Hz, average of 2",
            &[f(200.0), i(2)],
            &[190.0, 5.0, 200.0, 4_600.0],
            &[Some(450.0), None, None, None],
            &[36, 77, 85, 11],
        ),
        preset(
            3,
            "Low noise",
            "100 Hz, average of 4",
            &[f(100.0), i(4)],
            &[134.4, 10.0, 100.0, 2_300.0],
            &[Some(318.2), None, None, None],
            &[44, 67, 74, 21],
        ),
        preset(
            4,
            "Ultra low noise",
            "50 Hz, average of 8",
            &[f(50.0), i(8)],
            &[95.0, 20.0, 50.0, 1_150.0],
            &[Some(225.0), None, None, None],
            &[51, 57, 63, 31],
        ),
    ],
};
