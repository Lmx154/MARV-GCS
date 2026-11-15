use super::*;
use crate::VehicleState;
use crate::dialect::*;

struct Rig {
    service: Messages,
    state: VehicleState,
    out: Vec<Message>,
    /// `Ctx::changed` after the last call.
    changed: bool,
}

impl Rig {
    fn new() -> Self {
        Self {
            service: Messages::new(),
            state: VehicleState::default(),
            out: Vec::new(),
            changed: false,
        }
    }

    fn frame(&mut self, frame: &Frame, now_us: u64) {
        let mut cx = Ctx {
            now: Duration::from_micros(now_us),
            wall_time_us: 0,
            state: &mut self.state,
            out: &mut self.out,
            changed: false,
        };
        self.service.on_frame(frame, &mut cx);
        self.changed = cx.changed;
    }

    fn tick(&mut self, now_us: u64) {
        let mut cx = Ctx {
            now: Duration::from_micros(now_us),
            wall_time_us: 0,
            state: &mut self.state,
            out: &mut self.out,
            changed: false,
        };
        self.service.on_tick(&mut cx);
        self.changed = cx.changed;
    }

    fn store(&self) -> &MessageStore {
        &self.state.messages
    }

    fn rate(&self, message: &str, instance: u8) -> f32 {
        self.store().entry(message, instance).unwrap().rate_hz
    }
}

fn known(system_id: u8, component_id: u8, sequence: u8, message: Message) -> Frame {
    Frame {
        system_id,
        component_id,
        sequence,
        message_id: message.message_id(),
        raw: Vec::new(),
        body: Body::Known(message),
    }
}

fn heartbeat() -> Message {
    Message::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 3,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED
            | MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

fn imu(id: u8, xacc: f32) -> Message {
    Message::HIGHRES_IMU(HIGHRES_IMU_DATA {
        time_usec: 1_000_000,
        xacc,
        yacc: -2.5,
        zacc: 9.75,
        temperature: 21.5,
        fields_updated: HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XACC
            | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YACC,
        id,
        ..Default::default()
    })
}

fn num(name: &str, value: f64) -> Field {
    Field {
        name: name.to_owned(),
        value: FieldValue::Number(value),
    }
}

fn text(name: &str, value: &str) -> Field {
    Field {
        name: name.to_owned(),
        value: FieldValue::Text(value.to_owned()),
    }
}

/// Compare with NaN equal to NaN.
fn assert_fields(actual: &[Field], expected: &[Field]) {
    assert_eq!(actual.len(), expected.len(), "{actual:?}");
    for (a, e) in actual.iter().zip(expected) {
        assert_eq!(a.name, e.name);
        match (&a.value, &e.value) {
            (FieldValue::Number(x), FieldValue::Number(y)) if x.is_nan() && y.is_nan() => {}
            (x, y) => assert_eq!(x, y, "{}", a.name),
        }
    }
}

#[test]
fn fields_of_heartbeat_has_enums_and_flags_as_text() {
    assert_fields(
        &fields_of(&heartbeat()),
        &[
            num("custom_mode", 3.0),
            text("mavtype", "MAV_TYPE_QUADROTOR"),
            text("autopilot", "MAV_AUTOPILOT_ARDUPILOTMEGA"),
            text(
                "base_mode",
                "MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
            ),
            text("system_status", "MAV_STATE_STANDBY"),
            num("mavlink_version", 3.0),
        ],
    );
}

#[test]
fn fields_of_highres_imu_turns_nan_into_nan_numbers() {
    let nan = f64::NAN;
    let expected = [
        num("time_usec", 1_000_000.0),
        num("xacc", nan),
        num("yacc", -2.5),
        num("zacc", 9.75),
        num("xgyro", 0.0),
        num("ygyro", 0.0),
        num("zgyro", 0.0),
        num("xmag", nan),
        num("ymag", 0.0),
        num("zmag", 0.0),
        num("abs_pressure", 0.0),
        num("diff_pressure", 0.0),
        num("pressure_alt", 0.0),
        num("temperature", 21.5),
        text(
            "fields_updated",
            "HIGHRES_IMU_UPDATED_XACC | HIGHRES_IMU_UPDATED_YACC",
        ),
        num("id", 2.0),
    ];
    let Message::HIGHRES_IMU(mut data) = imu(2, f32::NAN) else {
        unreachable!()
    };
    data.xmag = f32::INFINITY;
    assert_fields(&fields_of(&Message::HIGHRES_IMU(data)), &expected);
}

#[test]
fn fields_of_gps_raw_int() {
    let message = Message::GPS_RAW_INT(GPS_RAW_INT_DATA {
        time_usec: 5,
        lat: 473_977_418,
        lon: -85_455_938,
        alt: 408_000,
        eph: 121,
        fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
        satellites_visible: 9,
        ..Default::default()
    });
    assert_fields(
        &fields_of(&message),
        &[
            num("time_usec", 5.0),
            num("lat", 473_977_418.0),
            num("lon", -85_455_938.0),
            num("alt", 408_000.0),
            num("eph", 121.0),
            num("epv", 0.0),
            num("vel", 0.0),
            num("cog", 0.0),
            text("fix_type", "GPS_FIX_TYPE_3D_FIX"),
            num("satellites_visible", 9.0),
            num("alt_ellipsoid", 0.0),
            num("h_acc", 0.0),
            num("v_acc", 0.0),
            num("vel_acc", 0.0),
            num("hdg_acc", 0.0),
            num("yaw", 0.0),
        ],
    );
}

#[test]
fn fields_of_statustext_and_param_value_have_char_arrays_as_text() {
    let status = Message::STATUSTEXT(STATUSTEXT_DATA {
        severity: MavSeverity::MAV_SEVERITY_WARNING,
        text: "low battery".into(),
        id: 7,
        chunk_seq: 1,
    });
    assert_fields(
        &fields_of(&status),
        &[
            text("severity", "MAV_SEVERITY_WARNING"),
            text("text", "low battery"),
            num("id", 7.0),
            num("chunk_seq", 1.0),
        ],
    );
    let param = Message::PARAM_VALUE(PARAM_VALUE_DATA {
        param_value: 1.5,
        param_count: 200,
        param_index: 12,
        param_id: "BAT_CAPACITY".into(),
        param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
    });
    assert_fields(
        &fields_of(&param),
        &[
            num("param_value", 1.5),
            num("param_count", 200.0),
            num("param_index", 12.0),
            text("param_id", "BAT_CAPACITY"),
            text("param_type", "MAV_PARAM_TYPE_REAL32"),
        ],
    );
}

#[test]
fn fields_of_arrays_split_when_short_and_summarise_when_long() {
    let message = Message::ATT_POS_MOCAP(ATT_POS_MOCAP_DATA {
        time_usec: 9,
        q: [1.0, 0.0, 0.5, f32::NAN],
        x: 1.0,
        y: 2.0,
        z: 3.0,
        covariance: [0.25; 21],
    });
    assert_fields(
        &fields_of(&message),
        &[
            num("time_usec", 9.0),
            num("q[0]", 1.0),
            num("q[1]", 0.0),
            num("q[2]", 0.5),
            num("q[3]", f64::NAN),
            num("x", 1.0),
            num("y", 2.0),
            num("z", 3.0),
            text(
                "covariance",
                "0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25 ... (21 values)",
            ),
        ],
    );
    let log = Message::LOG_DATA(LOG_DATA_DATA {
        ofs: 4,
        id: 1,
        count: 90,
        data: [7; 90],
    });
    assert_fields(
        &fields_of(&log),
        &[
            num("ofs", 4.0),
            num("id", 1.0),
            num("count", 90.0),
            text("data", "7, 7, 7, 7, 7, 7, 7, 7 ... (90 values)"),
        ],
    );
}

#[test]
fn fields_of_array_of_exactly_eight_is_split() {
    let message = Message::ATTITUDE_QUATERNION_COV(ATTITUDE_QUATERNION_COV_DATA {
        covariance: [1.0; 9],
        ..Default::default()
    });
    let fields = fields_of(&message);
    assert!(fields.iter().any(|f| f.name == "covariance"));
    let message = Message::SERVO_OUTPUT_RAW(SERVO_OUTPUT_RAW_DATA::default());
    assert!(fields_of(&message).iter().any(|f| f.name == "servo1_raw"));
}

#[test]
fn highres_imu_instances_get_separate_labelled_entries() {
    let mut rig = Rig::new();
    rig.frame(&known(1, 1, 0, imu(0, 1.0)), 1_000);
    rig.frame(&known(1, 1, 1, imu(2, 2.0)), 2_000);
    rig.frame(&known(1, 1, 2, imu(2, 3.0)), 3_000);
    assert!(rig.changed);
    let store = rig.store();
    assert_eq!(store.entries.len(), 2);
    let zero = store.entry("HIGHRES_IMU", 0).unwrap();
    let two = store.entry("HIGHRES_IMU", 2).unwrap();
    assert_eq!(zero.label, "HIGHRES_IMU[0]");
    assert_eq!(two.label, "HIGHRES_IMU[2]");
    assert_eq!(zero.name, "HIGHRES_IMU");
    assert_eq!((zero.count, two.count), (1, 2));
    assert_eq!(two.last_seen, Duration::from_micros(3_000));
    assert_eq!(two.number("xacc"), Some(3.0));
    assert_eq!(zero.number("xacc"), Some(1.0));
    assert!(store.entries.keys().all(|k| k.message_id == 105));
}

#[test]
fn message_without_instance_field_has_instance_zero_and_plain_label() {
    let mut rig = Rig::new();
    rig.frame(&known(1, 1, 0, heartbeat()), 0);
    let (key, entry) = rig.store().entries.iter().next().unwrap();
    assert_eq!(
        (
            key.system_id,
            key.component_id,
            key.message_id,
            key.instance
        ),
        (1, 1, 0, 0)
    );
    assert_eq!(
        (entry.name.as_str(), entry.label.as_str()),
        ("HEARTBEAT", "HEARTBEAT")
    );
    assert_eq!(entry.text("mavtype"), Some("MAV_TYPE_QUADROTOR"));
    assert_eq!(entry.number("custom_mode"), Some(3.0));
}

#[test]
fn unknown_and_undecodable_frames_get_entries_without_fields() {
    let mut rig = Rig::new();
    let unknown = Frame {
        system_id: 3,
        component_id: 4,
        sequence: 0,
        message_id: 42_000,
        raw: Vec::new(),
        body: Body::Unknown,
    };
    let undecodable = Frame {
        system_id: 3,
        component_id: 4,
        sequence: 1,
        message_id: 105,
        raw: Vec::new(),
        body: Body::Undecodable("bad".to_owned()),
    };
    rig.frame(&unknown, 10);
    assert!(rig.changed);
    rig.frame(&undecodable, 20);
    rig.frame(&unknown, 30);
    let store = rig.store();
    assert_eq!(store.entries.len(), 2);
    let unknown_entry = store.entry("UNKNOWN_42000", 0).unwrap();
    assert_eq!(unknown_entry.label, "UNKNOWN_42000");
    assert_eq!(unknown_entry.count, 2);
    assert!(unknown_entry.fields.is_empty());
    let undecodable_entry = store.entry("HIGHRES_IMU", 0).unwrap();
    assert_eq!(undecodable_entry.label, "HIGHRES_IMU[0]");
    assert!(undecodable_entry.fields.is_empty());
    assert!(store.series.is_empty());
}

#[test]
fn undecodable_message_with_id_outside_the_dialect_is_still_named() {
    let mut rig = Rig::new();
    let frame = Frame {
        system_id: 1,
        component_id: 1,
        sequence: 0,
        message_id: 9_999_999,
        raw: Vec::new(),
        body: Body::Undecodable("x".to_owned()),
    };
    rig.frame(&frame, 0);
    assert!(rig.store().entry("UNKNOWN_9999999", 0).is_some());
}

/// Feed `message` at `hz` from `from_us` for `seconds`; returns the time after the last one.
fn stream(rig: &mut Rig, message: &Message, hz: u64, from_us: u64, seconds: u64) -> u64 {
    let mut last = from_us;
    for i in 0..=hz * seconds {
        last = from_us + i * 1_000_000 / hz;
        rig.frame(&known(1, 1, i as u8, message.clone()), last);
    }
    last
}

#[test]
fn rate_of_a_steady_stream_is_within_two_percent() {
    for hz in [1u64, 5, 50, 500] {
        let mut rig = Rig::new();
        stream(&mut rig, &heartbeat(), hz, 0, 5);
        let rate = f64::from(rig.rate("HEARTBEAT", 0));
        assert!((rate - hz as f64).abs() <= hz as f64 * 0.02, "{hz}: {rate}");
    }
}

#[test]
fn rate_follows_a_change_of_speed() {
    let mut rig = Rig::new();
    let end = stream(&mut rig, &heartbeat(), 10, 0, 5);
    let end = stream(&mut rig, &heartbeat(), 100, end + 10_000, 5);
    let rate = f64::from(rig.rate("HEARTBEAT", 0));
    assert!((rate - 100.0).abs() <= 2.0, "{rate} at {end}");
}

#[test]
fn one_message_reads_zero_until_a_second_arrives() {
    let mut rig = Rig::new();
    rig.frame(&known(1, 1, 0, heartbeat()), 5_000_000);
    assert_eq!(rig.rate("HEARTBEAT", 0), 0.0);
    rig.tick(5_500_000);
    assert_eq!(rig.rate("HEARTBEAT", 0), 0.0);
    rig.frame(&known(1, 1, 1, heartbeat()), 5_500_000);
    assert!((rig.rate("HEARTBEAT", 0) - 2.0).abs() < 0.01);
}

#[test]
fn rate_of_a_stopped_stream_reaches_zero_in_time_and_tick_reports_it_once() {
    for (hz, silence_us) in [(500u64, 2_000_000u64), (50, 2_000_000), (1, 3_000_000)] {
        let mut rig = Rig::new();
        let end = stream(&mut rig, &heartbeat(), hz, 0, 5);
        assert!(rig.rate("HEARTBEAT", 0) > 0.0);
        rig.tick(end + silence_us - 1_000);
        assert!(rig.rate("HEARTBEAT", 0) > 0.0, "{hz} too early");
        assert!(!rig.changed);
        rig.tick(end + silence_us);
        assert_eq!(rig.rate("HEARTBEAT", 0), 0.0, "{hz}");
        assert!(rig.changed);
        rig.tick(end + silence_us + 50_000);
        assert!(!rig.changed, "{hz}: nothing changed on the second tick");
    }
}

#[test]
fn slow_stream_waits_three_periods_before_reading_zero() {
    let mut rig = Rig::new();
    let end = stream(&mut rig, &heartbeat(), 1, 0, 5);
    rig.tick(end + 2_500_000);
    assert!(rig.rate("HEARTBEAT", 0) > 0.0);
    rig.tick(end + 3_000_000);
    assert_eq!(rig.rate("HEARTBEAT", 0), 0.0);
}

#[test]
fn stream_that_resumes_after_silence_starts_afresh() {
    let mut rig = Rig::new();
    let end = stream(&mut rig, &heartbeat(), 10, 0, 5);
    rig.tick(end + 10_000_000);
    rig.frame(&known(1, 1, 0, heartbeat()), end + 20_000_000);
    assert_eq!(rig.rate("HEARTBEAT", 0), 0.0);
    rig.frame(&known(1, 1, 1, heartbeat()), end + 20_100_000);
    assert!((rig.rate("HEARTBEAT", 0) - 10.0).abs() < 0.1);
}

#[test]
fn tick_changes_nothing_when_no_rate_changed() {
    let mut rig = Rig::new();
    rig.tick(1_000);
    assert!(!rig.changed);
    rig.frame(&known(1, 1, 0, heartbeat()), 2_000);
    rig.tick(3_000);
    assert!(!rig.changed);
}

#[test]
fn every_number_field_gets_a_point_and_nan_gets_none() {
    let mut rig = Rig::new();
    rig.frame(&known(1, 1, 0, imu(2, f32::NAN)), 1_500_000);
    rig.frame(&known(1, 1, 1, imu(2, 4.0)), 2_500_000);
    let store = rig.store();
    let xacc = store.series("HIGHRES_IMU", 2, "xacc").unwrap();
    assert_eq!(xacc.points, VecDeque::from([[2.5, 4.0]]));
    let yacc = store.series("HIGHRES_IMU", 2, "yacc").unwrap();
    assert_eq!(yacc.points, VecDeque::from([[1.5, -2.5], [2.5, -2.5]]));
    assert!(store.series("HIGHRES_IMU", 2, "fields_updated").is_none());
    let count = store
        .series
        .keys()
        .filter(|k| k.message == "HIGHRES_IMU")
        .count();
    assert_eq!(count, 15, "16 fields, one of them text");
    let key = store.series.keys().find(|k| k.field == "yacc").unwrap();
    assert_eq!(
        key,
        &SeriesKey {
            system_id: 1,
            component_id: 1,
            message: "HIGHRES_IMU".to_owned(),
            instance: 2,
            field: "yacc".to_owned(),
        }
    );
}

#[test]
fn infinite_values_make_no_point() {
    let mut rig = Rig::new();
    rig.frame(&known(1, 1, 0, imu(0, f32::INFINITY)), 1_000_000);
    assert!(rig.store().series("HIGHRES_IMU", 0, "xacc").is_none());
    assert!(rig.store().series("HIGHRES_IMU", 0, "yacc").is_some());
}

#[test]
fn instances_have_separate_series() {
    let mut rig = Rig::new();
    rig.frame(&known(1, 1, 0, imu(0, 1.0)), 1_000_000);
    rig.frame(&known(1, 1, 1, imu(2, 2.0)), 2_000_000);
    rig.frame(&known(1, 1, 2, imu(2, 3.0)), 3_000_000);
    let store = rig.store();
    assert_eq!(
        store.series("HIGHRES_IMU", 0, "xacc").unwrap().points,
        VecDeque::from([[1.0, 1.0]])
    );
    assert_eq!(
        store.series("HIGHRES_IMU", 2, "xacc").unwrap().points,
        VecDeque::from([[2.0, 2.0], [3.0, 3.0]])
    );
    assert!(store.series("HIGHRES_IMU", 1, "xacc").is_none());
}

#[test]
fn series_drop_points_older_than_history() {
    let mut rig = Rig::new();
    for second in [0u64, 10, 59, 61, 120] {
        rig.frame(&known(1, 1, 0, imu(0, second as f32)), second * 1_000_000);
    }
    let points = &rig.store().series("HIGHRES_IMU", 0, "xacc").unwrap().points;
    assert_eq!(points, &VecDeque::from([[61.0, 61.0], [120.0, 120.0]]));
    rig.frame(&known(1, 1, 0, imu(0, 1.0)), 180_000_000);
    let points = &rig.store().series("HIGHRES_IMU", 0, "xacc").unwrap().points;
    assert_eq!(
        points.len(),
        2,
        "120 s is dropped from 180 s, 120 s itself is kept"
    );
}

#[test]
fn ten_minutes_at_500_hz_stay_within_both_limits() {
    let mut rig = Rig::new();
    let message = imu(0, 1.0);
    for i in 0..600u64 * 500 {
        rig.frame(&known(1, 1, i as u8, message.clone()), i * 2_000);
    }
    let store = rig.store();
    assert!(!store.series.is_empty());
    for (key, series) in &store.series {
        assert!(series.points.len() <= MAX_POINTS, "{key:?}");
        let newest = series.points.back().unwrap()[0];
        let oldest = series.points.front().unwrap()[0];
        assert!(newest - oldest <= HISTORY.as_secs_f64(), "{key:?}");
    }
    let xacc = store.series("HIGHRES_IMU", 0, "xacc").unwrap();
    assert_eq!(xacc.points.len(), MAX_POINTS);
    assert_eq!(store.entry("HIGHRES_IMU", 0).unwrap().count, 300_000);
}

#[test]
fn points_beyond_max_points_drop_the_oldest_first() {
    let mut rig = Rig::new();
    for i in 0..(MAX_POINTS as u64 + 10) {
        rig.frame(&known(1, 1, i as u8, imu(0, i as f32)), i * 100);
    }
    let points = &rig.store().series("HIGHRES_IMU", 0, "xacc").unwrap().points;
    assert_eq!(points.len(), MAX_POINTS);
    assert_eq!(points.front().unwrap()[1], 10.0);
    assert_eq!(points.back().unwrap()[1], (MAX_POINTS + 9) as f64);
}

#[test]
fn lost_frames_count_gaps_modulo_256() {
    let mut rig = Rig::new();
    let send = |rig: &mut Rig, sequence: u8| rig.frame(&known(1, 1, sequence, heartbeat()), 0);
    send(&mut rig, 200);
    assert_eq!(
        rig.store().lost_frames,
        0,
        "the first frame only starts the tracking"
    );
    send(&mut rig, 201);
    assert_eq!(rig.store().lost_frames, 0);
    send(&mut rig, 205);
    assert_eq!(rig.store().lost_frames, 3);
    send(&mut rig, 255);
    assert_eq!(rig.store().lost_frames, 3 + 49);
    send(&mut rig, 0);
    assert_eq!(rig.store().lost_frames, 52, "255 then 0 is no loss");
    send(&mut rig, 3);
    assert_eq!(rig.store().lost_frames, 54);
    send(&mut rig, 2);
    assert_eq!(
        rig.store().lost_frames,
        54 + 254,
        "going back wraps round the range"
    );
    send(&mut rig, 2);
    assert_eq!(
        rig.store().lost_frames,
        54 + 254,
        "a repeated number is no loss"
    );
}

#[test]
fn lost_frames_track_each_sender_apart() {
    let mut rig = Rig::new();
    rig.frame(&known(1, 1, 10, heartbeat()), 0);
    rig.frame(&known(1, 2, 100, heartbeat()), 0);
    rig.frame(&known(1, 1, 11, heartbeat()), 0);
    rig.frame(&known(1, 2, 101, heartbeat()), 0);
    rig.frame(&known(2, 1, 50, heartbeat()), 0);
    assert_eq!(rig.store().lost_frames, 0);
    rig.frame(&known(1, 1, 13, heartbeat()), 0);
    assert_eq!(rig.store().lost_frames, 1);
    rig.frame(&known(1, 2, 105, heartbeat()), 0);
    assert_eq!(rig.store().lost_frames, 4);
    rig.frame(&known(2, 1, 51, heartbeat()), 0);
    assert_eq!(rig.store().lost_frames, 4);
}

#[test]
fn lost_frames_count_unknown_frames_too() {
    let mut rig = Rig::new();
    let unknown = |sequence| Frame {
        system_id: 1,
        component_id: 1,
        sequence,
        message_id: 42_000,
        raw: Vec::new(),
        body: Body::Unknown,
    };
    rig.frame(&unknown(0), 0);
    rig.frame(&unknown(2), 0);
    assert_eq!(rig.store().lost_frames, 1);
}

#[test]
fn ten_thousand_frames_are_cheap() {
    let mut rig = Rig::new();
    let message = imu(0, 1.0);
    let started = std::time::Instant::now();
    for i in 0..10_000u64 {
        rig.frame(&known(1, 1, i as u8, message.clone()), i * 2_000);
    }
    let elapsed = started.elapsed();
    println!("10000 HIGHRES_IMU frames: {elapsed:?}");
    assert!(elapsed < Duration::from_secs(2), "{elapsed:?}");
    assert_eq!(rig.store().entry("HIGHRES_IMU", 0).unwrap().count, 10_000);
}

#[test]
fn fields_of_empty_flags_is_text() {
    let Message::HEARTBEAT(mut data) = heartbeat() else {
        unreachable!()
    };
    data.base_mode = MavModeFlag::empty();
    let fields = fields_of(&Message::HEARTBEAT(data));
    assert_eq!(fields[3], text("base_mode", ""));
}

#[test]
fn fields_of_marv_test_are_the_four_fields_of_the_message() {
    let message = Message::MARV_TEST(MARV_TEST_DATA {
        seq: 41,
        time_ms: 123_456,
        value: -7,
        tag: 2,
    });
    assert_fields(
        &fields_of(&message),
        &[
            num("seq", 41.0),
            num("time_ms", 123_456.0),
            num("value", -7.0),
            num("tag", 2.0),
        ],
    );
}

#[test]
fn marv_test_is_listed_by_name_with_its_fields_and_series() {
    let mut rig = Rig::new();
    for (seq, at_us) in [(0u32, 0), (1, 100_000)] {
        let message = Message::MARV_TEST(MARV_TEST_DATA {
            seq,
            time_ms: 1_000 + seq * 100,
            value: -3,
            tag: 1,
        });
        rig.frame(&known(1, 1, seq as u8, message), at_us);
    }
    let store = rig.store();
    assert_eq!(store.entries.len(), 1);
    let entry = store.entry("MARV_TEST", 0).unwrap();
    assert_eq!(entry.name, "MARV_TEST");
    assert_eq!(entry.label, "MARV_TEST");
    assert_eq!(entry.count, 2);
    assert_fields(
        &entry.fields,
        &[
            num("seq", 1.0),
            num("time_ms", 1_100.0),
            num("value", -3.0),
            num("tag", 1.0),
        ],
    );
    assert_eq!(entry.number("seq"), Some(1.0));
    let seq = store.series("MARV_TEST", 0, "seq").unwrap();
    assert_eq!(
        seq.points.iter().map(|p| p[1]).collect::<Vec<_>>(),
        [0.0, 1.0]
    );
    assert!(store.entry("UNKNOWN_42420", 0).is_none());
}

fn device_status(device: MarvDevice, samples: u32) -> Message {
    Message::MARV_DEVICE_STATUS(MARV_DEVICE_STATUS_DATA {
        time_boot_ms: 5_000,
        samples,
        errors: 1,
        last_sample_age_ms: 3,
        device_id: device,
        state: MarvDeviceState::MARV_DEVICE_STATE_OK,
        last_error: MarvDeviceError::MARV_DEVICE_ERROR_NONE,
        ..Default::default()
    })
}

#[test]
fn marv_device_status_keeps_one_row_per_device() {
    let mut rig = Rig::new();
    rig.frame(
        &known(1, 1, 0, device_status(MarvDevice::MARV_DEVICE_IMU1, 100)),
        1_000_000,
    );
    rig.frame(
        &known(1, 1, 1, device_status(MarvDevice::MARV_DEVICE_ESC, 7)),
        1_100_000,
    );
    let rows: Vec<_> = rig
        .store()
        .entries
        .iter()
        .filter(|(_, e)| e.name == "MARV_DEVICE_STATUS")
        .map(|(k, e)| (k.instance, e.number("samples"), e.count, e.last_seen))
        .collect();
    assert_eq!(
        rows,
        [
            (1, Some(100.0), 1, Duration::from_micros(1_000_000)),
            (12, Some(7.0), 1, Duration::from_micros(1_100_000)),
        ]
    );
    let imu = rig.store().entry("MARV_DEVICE_STATUS", 1).unwrap();
    assert_eq!(imu.label, "MARV_DEVICE_STATUS[1]");
    assert_eq!(imu.text("state"), Some("MARV_DEVICE_STATE_OK"));
}

#[test]
fn a_second_marv_device_status_of_one_device_replaces_its_row() {
    let mut rig = Rig::new();
    rig.frame(
        &known(1, 1, 0, device_status(MarvDevice::MARV_DEVICE_BARO, 100)),
        1_000_000,
    );
    rig.frame(
        &known(1, 1, 1, device_status(MarvDevice::MARV_DEVICE_MAG, 5)),
        1_500_000,
    );
    rig.frame(
        &known(1, 1, 2, device_status(MarvDevice::MARV_DEVICE_BARO, 740)),
        2_000_000,
    );
    let rows = rig
        .store()
        .entries
        .values()
        .filter(|e| e.name == "MARV_DEVICE_STATUS")
        .count();
    assert_eq!(rows, 2);
    let baro = rig.store().entry("MARV_DEVICE_STATUS", 3).unwrap();
    assert_eq!(baro.count, 2);
    assert_eq!(baro.number("samples"), Some(740.0));
    assert_eq!(baro.last_seen, Duration::from_secs(2));
    assert_eq!(
        rig.store()
            .entry("MARV_DEVICE_STATUS", 4)
            .unwrap()
            .number("samples"),
        Some(5.0)
    );
    // The history of the samples counter is kept per device.
    let series = rig
        .store()
        .series("MARV_DEVICE_STATUS", 3, "samples")
        .unwrap();
    assert_eq!(series.points, [[1.0, 100.0], [2.0, 740.0]]);
}

#[test]
fn another_message_type_still_has_one_row() {
    let mut rig = Rig::new();
    rig.frame(&known(1, 1, 0, heartbeat()), 0);
    rig.frame(&known(1, 1, 1, heartbeat()), 1_000_000);
    rig.frame(
        &known(1, 1, 2, device_status(MarvDevice::MARV_DEVICE_RC, 1)),
        1_000_000,
    );
    let heartbeats: Vec<_> = rig
        .store()
        .entries
        .iter()
        .filter(|(_, e)| e.name == "HEARTBEAT")
        .collect();
    assert_eq!(heartbeats.len(), 1);
    assert_eq!(heartbeats[0].1.count, 2);
    assert_eq!(heartbeats[0].1.label, "HEARTBEAT");
}

#[test]
fn an_undecodable_marv_device_status_is_a_row_without_fields_on_device_zero() {
    let mut rig = Rig::new();
    rig.frame(
        &Frame {
            system_id: 1,
            component_id: 1,
            sequence: 0,
            message_id: 42_421,
            raw: Vec::new(),
            body: Body::Undecodable("no such MARV_DEVICE".into()),
        },
        0,
    );
    let row = rig.store().entry("MARV_DEVICE_STATUS", 0).unwrap();
    assert!(row.fields.is_empty());
    assert_eq!(row.count, 1);
}
