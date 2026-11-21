//! The simulator's compass: the parameter table (indices 80 to 91), the magnetometer model and the scripted
//! users of the guided calibration, the last end to end through `gcs_magcal::session::Session`.

use gcs_magcal::accept::Refusal;
use gcs_magcal::pose::Pose;
use gcs_magcal::session::{Ending, Gps, Outcome, ParamValue as PlanValue, Session, write_plan};
use gcs_magcal::undo::Calibration;
use gcs_magcal::wmm;
use gcs_sim::mag::{self, M3, MagEnvironment};
use gcs_sim::script::CalibrationUser;
use gcs_sim::{ParamValue, SimConfig, SimVehicle, default_params};
use mavlink::dialects::common::{
    COMMAND_LONG_DATA, GpsFixType, HEARTBEAT_DATA, MavCmd, MavMessage, MavModeFlag, MavParamType,
    PARAM_SET_DATA, PARAM_VALUE_DATA,
};
use mavlink::types::CharArray;
use mavlink::{MAVLinkV2MessageRaw, MavHeader, MavlinkReader};

// Scenario choices of this file (neither the decision, the contract nor the firmware gives them).
/// White noise on the raw field, uT per axis.
const NOISE_UT: f64 = 0.3;
/// Amplitude of the hand-held magnet, uT.
const MAGNET_UT: f64 = 6.0;
/// The vehicle's true soft iron (symmetric, positive definite) and hard iron, uT.
const S_TRUE: M3 = [
    [1.0625, 0.03125, -0.046875],
    [0.03125, 0.9375, 0.0625],
    [-0.046875, 0.0625, 0.78125],
];
const O_TRUE: [f64; 3] = [-14.2, -4.2, 1.5];

fn decode(bytes: &[u8]) -> Vec<MavMessage> {
    let mut reader = MavlinkReader::new(bytes);
    let mut out = Vec::new();
    while let Ok((_, m)) = reader.read_any_message::<MavMessage>() {
        out.push(m);
    }
    out
}

fn encode(message: &MavMessage) -> Vec<u8> {
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message(
        MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 0,
        },
        message,
    );
    raw.raw_bytes().to_vec()
}

fn set_frame(name: &str, field: f32, param_type: MavParamType) -> Vec<u8> {
    encode(&MavMessage::PARAM_SET(PARAM_SET_DATA {
        param_value: field,
        target_system: 1,
        target_component: 1,
        param_id: CharArray::from(name),
        param_type,
    }))
}

fn command_frame(command: MavCmd, param1: f32, param2: f32) -> Vec<u8> {
    encode(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1,
        param2,
        command,
        target_system: 1,
        target_component: 1,
        ..COMMAND_LONG_DATA::default()
    }))
}

fn real(v: &SimVehicle, name: &str) -> f32 {
    match v.param(name) {
        Some(ParamValue::Real32(x)) => x,
        other => panic!("{name}: {other:?}"),
    }
}

struct Rig {
    v: SimVehicle,
    now_ms: u64,
}

impl Rig {
    fn new(config: SimConfig) -> Self {
        let mut v = SimVehicle::new(config);
        v.step(0, &[]);
        Self { v, now_ms: 0 }
    }

    fn advance(&mut self, ms: u64, rx: &[u8]) -> Vec<u8> {
        self.now_ms += ms;
        self.v.step(self.now_ms, rx)
    }

    /// A write: the PARAM_VALUE answers and the STATUSTEXTs.
    fn write(&mut self, name: &str, value: PlanValue) -> (Vec<PARAM_VALUE_DATA>, Vec<String>) {
        let (field, ty) = match value {
            PlanValue::F32(v) => (v, MavParamType::MAV_PARAM_TYPE_REAL32),
            PlanValue::I32(v) => (
                f32::from_le_bytes(v.to_le_bytes()),
                MavParamType::MAV_PARAM_TYPE_INT32,
            ),
        };
        let bytes = self.advance(10, &set_frame(name, field, ty));
        let (mut values, mut texts) = (Vec::new(), Vec::new());
        for m in decode(&bytes) {
            match m {
                MavMessage::PARAM_VALUE(d) => values.push(d),
                MavMessage::STATUSTEXT(d) => texts.push(d.text.to_str().unwrap().to_string()),
                _ => {}
            }
        }
        (values, texts)
    }
}

#[test]
fn the_table_has_92_parameters_and_the_firmwares_80_to_91() {
    let p = default_params();
    assert_eq!(p.len(), 92);
    // No existing index moved: the first and last of the old blocks.
    assert_eq!(
        (p[0].name.as_str(), p[69].name.as_str()),
        ("IMU_RATE_HZ", "MAG_AVG")
    );
    use ParamValue::{Int32 as I, Real32 as F};
    let expected = [
        (80, "GPS_RATE_MS", I(200)),
        (81, "MAG_DIAGX", F(1.0)),
        (82, "MAG_DIAGY", F(1.0)),
        (83, "MAG_DIAGZ", F(1.0)),
        (84, "MAG_ODIAGXY", F(0.0)),
        (85, "MAG_ODIAGXZ", F(0.0)),
        (86, "MAG_ODIAGYZ", F(0.0)),
        (87, "MAG_CAL_TYPE", I(0)),
        (88, "MAG_FLD_UT", F(0.0)),
        (89, "MAG_FLD_INC", F(0.0)),
        (90, "MAG_HLT_STR", F(10.0)),
        (91, "MAG_HLT_INC", F(10.0)),
    ];
    for (i, name, default) in expected {
        assert_eq!(
            (p[i].name.as_str(), p[i].value, p[i].read_only),
            (name, default, false),
            "index {i}"
        );
    }
    let names: Vec<&str> = p[70..80].iter().map(|q| q.name.as_str()).collect();
    assert_eq!(
        names,
        [
            "MAG_OFFX",
            "MAG_OFFY",
            "MAG_OFFZ",
            "BATT_A_SCALE",
            "BATT_A_OFFSET",
            "LOG_BACKEND_TYPE",
            "AHRS_ORIENTATION",
            "MAG_ORIENT",
            "MOT_POLES",
            "LED_BRIGHT"
        ]
    );
}

#[test]
fn ranges_are_the_whole_check_and_a_refusal_keeps_the_value() {
    // name, minimum, maximum (decision section 1 and contract section 16; the firmware's mag.rs:98-108).
    let table: [(&str, f32, f32, bool); 11] = [
        ("MAG_DIAGX", 0.2, 5.0, false),
        ("MAG_DIAGY", 0.2, 5.0, false),
        ("MAG_DIAGZ", 0.2, 5.0, false),
        ("MAG_ODIAGXY", -1.0, 1.0, false),
        ("MAG_ODIAGXZ", -1.0, 1.0, false),
        ("MAG_ODIAGYZ", -1.0, 1.0, false),
        ("MAG_CAL_TYPE", 0.0, 2.0, true),
        ("MAG_FLD_UT", 0.0, 95.0, false),
        ("MAG_FLD_INC", -90.0, 90.0, false),
        ("MAG_HLT_STR", 1.0, 50.0, false),
        ("MAG_HLT_INC", 1.0, 45.0, false),
    ];
    let mut rig = Rig::new(SimConfig::default());
    let mk = |v: f32, int: bool| {
        if int {
            PlanValue::I32(v as i32)
        } else {
            PlanValue::F32(v)
        }
    };
    for (name, low, high, int) in table {
        for ok in [low, high] {
            let (values, texts) = rig.write(name, mk(ok, int));
            assert_eq!(values.len(), 1, "{name} {ok}");
            assert!(texts.is_empty(), "{name} {ok}: {texts:?}");
        }
        let before = rig.v.param(name);
        let step = if int { 1.0 } else { 0.01 };
        for bad in [low - step, high + step] {
            let (values, texts) = rig.write(name, mk(bad, int));
            assert_eq!(
                values.len(),
                1,
                "one PARAM_VALUE answers a refused write too"
            );
            assert_eq!(texts, [format!("{name}: out of range")], "{name} {bad}");
            assert_eq!(rig.v.param(name), before, "{name} keeps its value");
        }
    }
    let (_, texts) = rig.write("MAG_OFFX", PlanValue::F32(200.5));
    assert_eq!(texts, ["MAG_OFFX: out of range"]);
}

#[test]
fn gps_rate_ms_takes_100_200_500_1000_only() {
    let mut rig = Rig::new(SimConfig::default());
    for ok in [100, 500, 1000, 200] {
        let (_, texts) = rig.write("GPS_RATE_MS", PlanValue::I32(ok));
        assert!(texts.is_empty(), "{ok}: {texts:?}");
        assert_eq!(rig.v.param("GPS_RATE_MS"), Some(ParamValue::Int32(ok)));
    }
    for bad in [101, 150, 999] {
        let (_, texts) = rig.write("GPS_RATE_MS", PlanValue::I32(bad));
        assert_eq!(texts, ["GPS_RATE_MS: not 100, 200, 500 or 1000"], "{bad}");
        assert_eq!(rig.v.param("GPS_RATE_MS"), Some(ParamValue::Int32(200)));
    }
    for bad in [99, 1001, 0, -100] {
        let (_, texts) = rig.write("GPS_RATE_MS", PlanValue::I32(bad));
        assert_eq!(texts, ["GPS_RATE_MS: out of range"], "{bad}");
    }
}

#[test]
fn the_calibration_reset_returns_81_to_89_and_the_offsets_to_default_and_keeps_the_tolerances() {
    let mut rig = Rig::new(SimConfig::default());
    for (name, v) in [
        ("MAG_OFFX", 5.0),
        ("MAG_OFFZ", -7.0),
        ("MAG_DIAGX", 1.5),
        ("MAG_ODIAGYZ", 0.25),
        ("MAG_FLD_UT", 50.0),
        ("MAG_FLD_INC", 60.0),
        ("MAG_HLT_STR", 20.0),
        ("MAG_HLT_INC", 15.0),
    ] {
        rig.write(name, PlanValue::F32(v));
    }
    rig.write("MAG_CAL_TYPE", PlanValue::I32(2));
    rig.write("GPS_RATE_MS", PlanValue::I32(500));
    let _ = rig.advance(
        10,
        &command_frame(MavCmd::MAV_CMD_PREFLIGHT_STORAGE, 3.0, 0.0),
    );
    let _ = rig.advance(2_100, &[]);
    let defaults = default_params();
    for p in &defaults[70..92] {
        let kept = matches!(
            p.name.as_str(),
            "MAG_HLT_STR" | "MAG_HLT_INC" | "GPS_RATE_MS"
        );
        let now = rig.v.param(&p.name).unwrap();
        if kept {
            assert_ne!(now, p.value, "{} is not a calibration parameter", p.name);
        } else {
            assert_eq!(now, p.value, "{}", p.name);
        }
    }
}

fn near(a: &[f64; 3], b: &[f64; 3], tol: f64) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < tol)
}

/// Level, heading `psi` (rad): R = Rz(psi)^T.
fn level_heading(psi: f64) -> M3 {
    let (s, c) = psi.sin_cos();
    [[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]]
}

fn random_rotation(rng: &mut u64) -> M3 {
    let mut next = || {
        *rng ^= *rng << 13;
        *rng ^= *rng >> 7;
        *rng ^= *rng << 17;
        (*rng >> 11) as f64 / (1u64 << 53) as f64 - 0.5
    };
    // Gram-Schmidt of three random vectors.
    let a = [next(), next(), next()];
    let b = [next(), next(), next()];
    let n = |v: [f64; 3]| {
        let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        v.map(|x| x / l)
    };
    let x = n(a);
    let d = x[0] * b[0] + x[1] * b[1] + x[2] * b[2];
    let y = n([b[0] - d * x[0], b[1] - d * x[1], b[2] - d * x[2]]);
    let z = [
        x[1] * y[2] - x[2] * y[1],
        x[2] * y[0] - x[0] * y[2],
        x[0] * y[1] - x[1] * y[0],
    ];
    [x, y, z]
}

fn model_ned() -> [f64; 3] {
    wmm::field(47.3977, 8.5456, 488.0, mag::DECIMAL_YEAR)
        .unwrap()
        .ned_ut
}

#[test]
fn level_facing_north_reads_the_model_field_from_the_vehicle() {
    let mut rig = Rig::new(SimConfig::default());
    let bytes = rig.advance(100, &[]);
    let imu = decode(&bytes).into_iter().find_map(|m| match m {
        MavMessage::HIGHRES_IMU(d) => Some(d),
        _ => None,
    });
    let imu = imu.expect("a HIGHRES_IMU");
    let got = [imu.xmag, imu.ymag, imu.zmag].map(|g| f64::from(g) * 100.0);
    assert!(
        near(&got, &model_ned(), 1e-3),
        "{got:?} against {:?}",
        model_ned()
    );
}

#[test]
fn with_the_true_calibration_set_the_published_field_is_the_earth_field_in_the_body_frame() {
    let env = MagEnvironment {
        s_true: S_TRUE,
        o_true: O_TRUE,
        ..MagEnvironment::default()
    };
    let diag = [S_TRUE[0][0], S_TRUE[1][1], S_TRUE[2][2]].map(|v| v as f32);
    let off = [S_TRUE[0][1], S_TRUE[0][2], S_TRUE[1][2]].map(|v| v as f32);
    let offsets = O_TRUE.map(|v| v as f32);
    let mut rng = 0x1234_5678_9ABC_DEF1_u64;
    for k in 0..200 {
        let r = random_rotation(&mut rng);
        let raw = mag::raw_field_ut(&env, &r, k * 40, 1);
        let m = mag::published_gauss(raw, offsets, diag, off).map(|g| f64::from(g) * 100.0);
        let truth = mag::mul_vec(&r, model_ned());
        assert!(near(&m, &truth, 1e-3), "{m:?} against {truth:?}");
    }
    // And through the vehicle, with the nine parameters written, at times of the script (attitudes of the poses
    // and of their turns and transitions).
    let user = CalibrationUser::new(0);
    let mut rig = Rig::new(SimConfig {
        mag: env,
        user: Some(user.clone()),
        ..SimConfig::default()
    });
    for (name, v) in [
        ("MAG_OFFX", offsets[0]),
        ("MAG_OFFY", offsets[1]),
        ("MAG_OFFZ", offsets[2]),
        ("MAG_DIAGX", diag[0]),
        ("MAG_DIAGY", diag[1]),
        ("MAG_DIAGZ", diag[2]),
        ("MAG_ODIAGXY", off[0]),
        ("MAG_ODIAGXZ", off[1]),
        ("MAG_ODIAGYZ", off[2]),
    ] {
        let (_, texts) = rig.write(name, PlanValue::F32(v));
        assert!(texts.is_empty(), "{name}");
    }
    let mut seen = 0;
    while rig.now_ms < 60_000 {
        for m in decode(&rig.advance(40, &[])) {
            if let MavMessage::HIGHRES_IMU(d) = m {
                let t = d.time_usec / 1000;
                let r = user.motion(t).body_from_ned;
                let got = [d.xmag, d.ymag, d.zmag].map(|g| f64::from(g) * 100.0);
                let truth = mag::mul_vec(&r, model_ned());
                assert!(near(&got, &truth, 2e-3), "t={t}: {got:?} against {truth:?}");
                seen += 1;
            }
        }
    }
    assert!(seen > 1000);
}

#[test]
fn an_anomaly_site_shifts_the_inclination_and_keeps_the_strength_and_the_declination() {
    let f = wmm::field(47.3977, 8.5456, 488.0, mag::DECIMAL_YEAR).unwrap();
    let env = MagEnvironment {
        inclination_shift_deg: 20.0,
        ..MagEnvironment::default()
    };
    let b = mag::earth_ned_ut(&env);
    let strength = (b[0] * b[0] + b[1] * b[1] + b[2] * b[2]).sqrt();
    let inc = (b[2] / strength).asin().to_degrees();
    assert!((strength - f.intensity_ut).abs() < 1e-9);
    assert!((inc - (f.inclination_deg + 20.0)).abs() < 1e-9, "{inc}");
    assert!((b[1].atan2(b[0]).to_degrees() - f.declination_deg).abs() < 1e-9);
}

#[test]
fn the_noise_is_deterministic_and_has_the_chosen_deviation() {
    let env = MagEnvironment {
        noise_ut: NOISE_UT,
        ..MagEnvironment::default()
    };
    let clean = mag::raw_field_ut(&MagEnvironment::default(), &mag::IDENTITY, 0, 1);
    let mut sum2 = 0.0;
    let n = 3000;
    for k in 0..n {
        let a = mag::raw_field_ut(&env, &mag::IDENTITY, k * 40, 1);
        assert_eq!(a, mag::raw_field_ut(&env, &mag::IDENTITY, k * 40, 1));
        sum2 += (0..3).map(|i| (a[i] - clean[i]).powi(2)).sum::<f64>();
    }
    let sd = (sum2 / (3.0 * n as f64)).sqrt();
    assert!((sd - NOISE_UT).abs() < 0.03 * NOISE_UT * 3.0, "{sd}");
}

#[test]
fn the_quick_calibration_stores_the_raw_offsets_with_identity_and_type_1() {
    let env = MagEnvironment {
        s_true: S_TRUE,
        o_true: O_TRUE,
        ..MagEnvironment::default()
    };
    let mut rig = Rig::new(SimConfig {
        mag: env,
        ..SimConfig::default()
    });
    // A guided calibration in force (a non-identity matrix, type 2, a stored field) that the quick one replaces.
    for (name, v) in [
        ("MAG_DIAGX", 1.25),
        ("MAG_ODIAGXY", 0.125),
        ("MAG_FLD_UT", 48.0),
        ("MAG_FLD_INC", 63.0),
        ("MAG_OFFY", 9.0),
    ] {
        rig.write(name, PlanValue::F32(v));
    }
    rig.write("MAG_CAL_TYPE", PlanValue::I32(2));
    let _ = rig.advance(
        10,
        &command_frame(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 0.0, 1.0),
    );
    assert_eq!(
        rig.v.param("MAG_CAL_TYPE"),
        Some(ParamValue::Int32(2)),
        "nothing changes before it ends"
    );
    let _ = rig.advance(4_100, &[]);
    for (i, name) in ["MAG_OFFX", "MAG_OFFY", "MAG_OFFZ"].into_iter().enumerate() {
        assert_eq!(real(&rig.v, name), O_TRUE[i] as f32, "{name}");
    }
    for name in ["MAG_DIAGX", "MAG_DIAGY", "MAG_DIAGZ"] {
        assert_eq!(real(&rig.v, name), 1.0, "{name}");
    }
    for name in ["MAG_ODIAGXY", "MAG_ODIAGXZ", "MAG_ODIAGYZ", "MAG_FLD_UT"] {
        assert_eq!(real(&rig.v, name), 0.0, "{name}");
    }
    assert_eq!(rig.v.param("MAG_CAL_TYPE"), Some(ParamValue::Int32(1)));
    // The published field is now S_true^-1 R B: the raw field less the offsets.
    let imu = decode(&rig.advance(100, &[]))
        .into_iter()
        .find_map(|m| match m {
            MavMessage::HIGHRES_IMU(d) => Some(d),
            _ => None,
        });
    let imu = imu.unwrap();
    let got = [imu.xmag, imu.ymag, imu.zmag].map(|g| f64::from(g) * 100.0);
    let want = mag::mul_vec(&mag::invert(&S_TRUE).unwrap(), model_ned());
    assert!(near(&got, &want, 2e-3), "{got:?} against {want:?}");
}

#[test]
fn the_script_turns_about_the_vertical_at_the_chosen_rate_with_a_consistent_accelerometer() {
    let user = CalibrationUser::new(0);
    // Level pose: transition 1.5 s (a hold for the first pose), hold 0.5 s, then the turn.
    let m = user.motion(2_000 + 6_000);
    let down = mag::mul_vec(&m.body_from_ned, [0.0, 0.0, 1.0]);
    let rate = (m.gyro[0] * down[0] + m.gyro[1] * down[1] + m.gyro[2] * down[2]).abs();
    assert!((rate - std::f64::consts::TAU / 12.0).abs() < 1e-3, "{rate}");
    for (a, d) in m.accel.iter().zip(down) {
        assert!((a + 9.81 * d).abs() < 1e-9);
    }
    // Every pose is recognised from the script's accelerometer at its hold.
    for (i, pose) in Pose::ALL.iter().enumerate() {
        let t = 2_000 + i as u64 * 15_500 + 250;
        let a = user.motion(t).accel;
        assert!(pose.matches(a), "{pose}: {a:?}");
    }
    assert!((user.duration_s() - (6.0 * 14.5 - 1.5)).abs() < 1e-9);
}

/// What a station does with the stream: HIGHRES_IMU into the session, the first GPS_RAW_INT starts it, a
/// HEARTBEAT with the armed bit stops it.
struct Run {
    rig: Rig,
    session: Option<Session>,
    gps_seen: bool,
    bytes_after_silence: usize,
    silent: bool,
}

impl Run {
    fn new(config: SimConfig) -> Self {
        let mut rig = Rig::new(config);
        // A calibration in force that is far from identity (the contract's test vector).
        for (name, v) in [
            ("MAG_OFFX", -14.2),
            ("MAG_OFFY", -4.2),
            ("MAG_OFFZ", 1.5),
            ("MAG_DIAGX", 1.0625),
            ("MAG_DIAGY", 0.9375),
            ("MAG_DIAGZ", 0.78125),
            ("MAG_ODIAGXY", 0.03125),
            ("MAG_ODIAGXZ", -0.046875),
            ("MAG_ODIAGYZ", 0.0625),
        ] {
            rig.write(name, PlanValue::F32(v));
        }
        Self {
            rig,
            session: None,
            gps_seen: false,
            bytes_after_silence: 0,
            silent: false,
        }
    }

    fn calibration(&self) -> Calibration {
        let r = |n: &str| f64::from(real(&self.rig.v, n));
        Calibration {
            offsets: [r("MAG_OFFX"), r("MAG_OFFY"), r("MAG_OFFZ")],
            diag: [r("MAG_DIAGX"), r("MAG_DIAGY"), r("MAG_DIAGZ")],
            offdiag: [r("MAG_ODIAGXY"), r("MAG_ODIAGXZ"), r("MAG_ODIAGYZ")],
        }
    }

    /// Runs until `end_s` of the simulator's time in steps of 20 ms.
    fn until(&mut self, end_s: f64) {
        while (self.rig.now_ms as f64) < end_s * 1000.0 {
            let bytes = self.rig.advance(20, &[]);
            if self.silent {
                self.bytes_after_silence += bytes.len();
            }
            let now = self.rig.now_ms as f64 / 1000.0;
            for m in decode(&bytes) {
                match m {
                    MavMessage::GPS_RAW_INT(g)
                        if !self.gps_seen
                            && (g.fix_type as u8) >= GpsFixType::GPS_FIX_TYPE_3D_FIX as u8 =>
                    {
                        self.gps_seen = true;
                        let gps = Gps {
                            fix_type: g.fix_type as u8,
                            age_s: 0.0,
                            lat_deg: f64::from(g.lat) / 1e7,
                            lon_deg: f64::from(g.lon) / 1e7,
                            height_m: f64::from(g.alt) / 1e3,
                        };
                        self.session = Some(
                            Session::start(self.calibration(), Some(gps), mag::DECIMAL_YEAR, now)
                                .expect("starts"),
                        );
                    }
                    MavMessage::HEARTBEAT(HEARTBEAT_DATA { base_mode, .. })
                        if base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED) =>
                    {
                        if let Some(s) = &mut self.session {
                            s.armed();
                        }
                    }
                    MavMessage::HIGHRES_IMU(d) => {
                        if let Some(s) = &mut self.session {
                            s.sample(
                                d.time_usec,
                                [d.xacc, d.yacc, d.zacc],
                                [d.xgyro, d.ygyro, d.zgyro],
                                [d.xmag, d.ymag, d.zmag],
                            );
                        }
                    }
                    _ => {}
                }
            }
            if let Some(s) = &mut self.session {
                s.tick(now);
            }
        }
    }

    fn outcome(&self) -> &Outcome {
        self.session
            .as_ref()
            .expect("the session started")
            .outcome()
    }
}

fn config(user: CalibrationUser, extra: impl FnOnce(&mut MagEnvironment)) -> SimConfig {
    let mut env = MagEnvironment {
        s_true: S_TRUE,
        o_true: O_TRUE,
        noise_ut: NOISE_UT,
        ..MagEnvironment::default()
    };
    extra(&mut env);
    SimConfig {
        mag: env,
        user: Some(user),
        ..SimConfig::default()
    }
}

fn ending(run: &Run) -> Ending {
    match run.outcome() {
        Outcome::Ended(e) => e.clone(),
        other => panic!("expected an ending, got {other:?}"),
    }
}

#[test]
fn the_good_user_is_accepted_and_the_written_calibration_gives_the_earth_field() {
    let user = CalibrationUser::new(1_000);
    let mut run = Run::new(config(user.clone(), |_| {}));
    run.until(1.0 + user.duration_s() + 5.0);
    let Outcome::Accepted(done) = run.outcome().clone() else {
        panic!("not accepted: {:?}", run.outcome().message())
    };
    // Written back to a vehicle as the 13 parameters of the plan.
    let plan = write_plan(&done.accepted);
    assert_eq!(plan.len(), 13);
    for (name, value) in &plan {
        let (_, texts) = run.rig.write(name, *value);
        assert!(texts.is_empty(), "{name}: {texts:?}");
    }
    let cal = run.calibration();
    let model = wmm::field(47.3977, 8.5456, 488.0, mag::DECIMAL_YEAR).unwrap();
    let (mut worst_ut, mut worst_deg) = (0.0f64, 0.0f64);
    let f = |n: &str| real(&run.rig.v, n);
    let (offsets, diag, off) = (
        ["MAG_OFFX", "MAG_OFFY", "MAG_OFFZ"].map(f),
        ["MAG_DIAGX", "MAG_DIAGY", "MAG_DIAGZ"].map(f),
        ["MAG_ODIAGXY", "MAG_ODIAGXZ", "MAG_ODIAGYZ"].map(f),
    );
    let env = MagEnvironment {
        s_true: S_TRUE,
        o_true: O_TRUE,
        ..MagEnvironment::default()
    };
    let mut rng = 0xDEAD_BEEF_u64;
    let mut attitudes: Vec<M3> = (0..8)
        .map(|k| level_heading(f64::from(k) * std::f64::consts::FRAC_PI_4))
        .collect();
    attitudes.extend((0..50).map(|_| random_rotation(&mut rng)));
    for r in attitudes {
        let m = mag::published_gauss(mag::raw_field_ut(&env, &r, 0, 1), offsets, diag, off)
            .map(|g| f64::from(g) * 100.0);
        let truth = mag::mul_vec(&r, model.ned_ut);
        for i in 0..3 {
            worst_ut = worst_ut.max((m[i] - truth[i]).abs());
        }
        // Heading of a level vehicle: from the horizontal components.
        let (h, t) = (
            (-m[1]).atan2(m[0]).to_degrees(),
            (-truth[1]).atan2(truth[0]).to_degrees(),
        );
        if r[2][2] > 0.999 {
            worst_deg = worst_deg.max(((h - t + 540.0).rem_euclid(360.0) - 180.0).abs());
        }
    }
    eprintln!(
        "good user: fitness {:.3} uT, field error {worst_ut:.3} uT, heading error {worst_deg:.3} deg, offsets {:?}, diag {:?}",
        done.accepted.fitness_ut, cal.offsets, cal.diag
    );
    assert!(worst_ut < 1.0, "{worst_ut} uT");
    assert!(worst_deg < 2.0, "{worst_deg} deg");
    assert_eq!(run.rig.v.param("MAG_CAL_TYPE"), Some(ParamValue::Int32(2)));
}

#[test]
fn a_skipped_pose_times_out_naming_it() {
    let user = CalibrationUser::new(1_000).skipping(Pose::UpsideDown.index());
    let mut run = Run::new(config(user.clone(), |_| {}));
    run.until(1.0 + user.duration_s() + 100.0);
    let e = ending(&run);
    assert_eq!(e, Ending::TimedOut(Pose::UpsideDown));
    assert_eq!(
        e.message(),
        "Timed out waiting for upside down: start again"
    );
}

#[test]
fn a_magnet_nearby_is_refused_as_too_scattered() {
    let user = CalibrationUser::new(1_000);
    let mut run = Run::new(config(user.clone(), |e| e.magnet_ut = MAGNET_UT));
    run.until(1.0 + user.duration_s() + 5.0);
    let e = ending(&run);
    assert!(
        matches!(e, Ending::Refused(Refusal::TooScattered { .. })),
        "{e:?}"
    );
    assert!(
        e.message().contains("Readings too scattered"),
        "{}",
        e.message()
    );
    eprintln!("magnet: {}", e.message());
}

#[test]
fn an_anomaly_site_is_refused_as_a_disturbed_field() {
    let user = CalibrationUser::new(1_000);
    let mut run = Run::new(config(user.clone(), |e| e.inclination_shift_deg = 20.0));
    run.until(1.0 + user.duration_s() + 5.0);
    let e = ending(&run);
    assert!(
        e.message().contains("The field here is disturbed"),
        "{}",
        e.message()
    );
    eprintln!("anomaly: {}", e.message());
}

#[test]
fn arming_in_the_middle_stops_the_run_with_no_plan() {
    let user = CalibrationUser::new(1_000);
    let mut run = Run::new(config(user.clone(), |_| {}));
    run.until(40.0);
    assert_eq!(run.outcome(), &Outcome::Running);
    run.rig.v.set_armed(true);
    run.until(42.0);
    assert_eq!(ending(&run), Ending::Armed);
    assert!(!matches!(run.outcome(), Outcome::Accepted(_)));
    assert_eq!(
        ending(&run).message(),
        "The vehicle was armed: calibration stopped"
    );
}

#[test]
fn link_loss_in_the_middle_leaves_the_session_without_samples_and_it_times_out() {
    let user = CalibrationUser::new(1_000);
    let mut run = Run::new(config(user.clone(), |_| {}));
    run.until(40.0);
    assert_eq!(run.outcome(), &Outcome::Running);
    run.rig.v.set_silent(true);
    run.silent = true;
    // The last pose completed before the loss (level at about 16 s, upside down at about 31 s): 90 s later is
    // about 121 s, so at 100 s the session still waits.
    run.until(100.0);
    assert_eq!(
        run.outcome(),
        &Outcome::Running,
        "90 s have not passed since the last pose completed"
    );
    run.until(40.0 + 200.0);
    assert_eq!(run.bytes_after_silence, 0, "a silent vehicle sends nothing");
    assert!(
        matches!(ending(&run), Ending::TimedOut(_)),
        "{:?}",
        run.outcome()
    );
}
