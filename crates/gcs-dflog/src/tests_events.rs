//! The event rules, on synthetic logs in the firmware's table.

use std::io::Cursor;

use crate::tests::{V, fmt_record, int, record};
use crate::{Control, Event, EventSource, Reader, events, events_of, events_with};

const PARM: &str = "QNff";
const MSG: &str = "QZ";
const GPS: &str = "QBBIHBcLLeff";
const RCIN: &str = "QHHHHHHHHHHHHHH";
const DEV: &str = "QBBBIIIHHH";
const DSF: &str = "QIHIIII";

/// The FMT records of the firmware's table for the types the events read.
pub(crate) fn header() -> Vec<u8> {
    let mut f = Vec::new();
    f.extend(fmt_record(129, "PARM", PARM, "TimeUS,Name,Value,Default"));
    f.extend(fmt_record(130, "MSG", MSG, "TimeUS,Message"));
    f.extend(fmt_record(
        134,
        "GPS",
        GPS,
        "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs",
    ));
    f.extend(fmt_record(
        136,
        "RCIN",
        RCIN,
        "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14",
    ));
    f.extend(fmt_record(
        137,
        "DEV",
        DEV,
        "TimeUS,Id,State,Err,Smp,Errs,TOs,Rec,Att,Ovf",
    ));
    f.extend(fmt_record(
        138,
        "DSF",
        DSF,
        "TimeUS,Dp,Blk,Bytes,FMn,FMx,FAv",
    ));
    f
}

fn dev(t: i64, id: i64, state: i64, err: i64, errs: i64) -> Vec<u8> {
    record(
        137,
        DEV,
        &[
            int(t),
            int(id),
            int(state),
            int(err),
            int(1000),
            int(errs),
            int(0),
            int(0),
            int(0),
            int(0),
        ],
    )
}

fn msg(t: i64, text: &'static str) -> Vec<u8> {
    record(130, MSG, &[int(t), V::Text(text)])
}

fn parm(t: i64, name: &'static str, value: f64) -> Vec<u8> {
    record(
        129,
        PARM,
        &[int(t), V::Text(name), V::Float(value), V::Float(0.0)],
    )
}

/// A GPS record with a made-up, non-zero position.
fn gps(t: i64, status: i64) -> Vec<u8> {
    record(
        134,
        GPS,
        &[
            int(t),
            int(0),
            int(status),
            int(0),
            int(0),
            int(9),
            int(120),
            int(123_456_789),
            int(-987_654_321),
            int(5000),
            V::Float(1.0),
            V::Float(2.0),
        ],
    )
}

fn rcin(t: i64) -> Vec<u8> {
    let mut v = vec![int(t)];
    v.extend((0..14).map(|c| int(1000 + c)));
    record(136, RCIN, &v)
}

fn dsf(t: i64, dp: i64) -> Vec<u8> {
    record(
        138,
        DSF,
        &[int(t), int(dp), int(1), int(2), int(3), int(4), int(5)],
    )
}

fn run(records: &[Vec<u8>]) -> Vec<Event> {
    let mut file = header();
    for r in records {
        file.extend(r);
    }
    events_of(Cursor::new(file), None).unwrap()
}

fn of(events: &[Event], source: EventSource) -> Vec<(u64, &str)> {
    events
        .iter()
        .filter(|e| e.source == source)
        .map(|e| (e.time_us, e.text.as_str()))
        .collect()
}

#[test]
fn dev_changes_per_row_and_counter_growth_is_not_an_event() {
    let e = run(&[
        dev(1000, 1, 2, 0, 0),
        dev(2000, 1, 2, 0, 50), // counters grew: nothing
        dev(3000, 1, 3, 2, 51), // OK -> DEGRADED, TIMEOUT
        dev(4000, 1, 3, 1, 52), // same state, error changes
        dev(5000, 1, 3, 1, 90), // counters only
        dev(6000, 1, 2, 0, 90), // back to OK, error cleared
        dev(7000, 9, 2, 5, 0),  // SD, first row carries an error
        dev(8000, 12, 1, 0, 0), // an id the table does not name
    ]);
    assert_eq!(
        of(&e, EventSource::Dev),
        [
            (1000, "IMU1 OK"),
            (3000, "IMU1: OK -> DEGRADED (TIMEOUT)"),
            (4000, "IMU1: DEGRADED (error TIMEOUT -> BUS)"),
            (6000, "IMU1: DEGRADED -> OK (NONE)"),
            (7000, "SD OK (POR)"),
            (8000, "device 12 STARTING"),
        ]
    );
    assert_eq!(
        e.iter().map(|e| e.initial).collect::<Vec<_>>(),
        [true, false, false, false, true, true]
    );
}

#[test]
fn dev_rows_interleaved_are_tracked_apart() {
    // Compared with the previous DEV record of any row, almost every record here would look like a change.
    let e = run(&[
        dev(1000, 1, 2, 0, 0),
        dev(1001, 2, 2, 0, 0),
        dev(2000, 1, 2, 0, 5),
        dev(2001, 2, 5, 6, 5),
        dev(3000, 1, 2, 0, 9),
        dev(3001, 2, 5, 6, 9),
    ]);
    assert_eq!(
        of(&e, EventSource::Dev),
        [
            (1000, "IMU1 OK"),
            (1001, "IMU2 OK"),
            (2001, "IMU2: OK -> FAILED (STUCK)"),
        ]
    );
}

#[test]
fn dev_names_of_devices_states_and_errors() {
    let ids = [
        (1, "IMU1"),
        (2, "IMU2"),
        (3, "BARO"),
        (4, "MAG"),
        (5, "GNSS"),
        (6, "BATT"),
        (7, "RC"),
        (9, "SD"),
        (8, "device 8"),
        (10, "device 10"),
    ];
    let records: Vec<_> = ids.iter().map(|&(id, _)| dev(1000, id, 2, 0, 0)).collect();
    let e = run(&records);
    let got: Vec<_> = e.iter().map(|e| e.text.clone()).collect();
    let want: Vec<_> = ids.iter().map(|&(_, n)| format!("{n} OK")).collect();
    assert_eq!(got, want);

    let states = [
        "ABSENT",
        "STARTING",
        "OK",
        "DEGRADED",
        "RECOVERING",
        "FAILED",
    ];
    let records: Vec<_> = (0..6).map(|s| dev(1000 * (s + 1), 3, s, 0, 0)).collect();
    let e = run(&records);
    let got: Vec<_> = e.iter().map(|e| e.text.clone()).collect();
    let mut want = vec!["BARO ABSENT".to_owned()];
    want.extend(
        states
            .windows(2)
            .map(|w| format!("BARO: {} -> {}", w[0], w[1])),
    );
    assert_eq!(got, want);

    let errors = [
        "NONE",
        "BUS",
        "TIMEOUT",
        "ID_MISMATCH",
        "CONFIG_MISMATCH",
        "POR",
        "STUCK",
    ];
    let records: Vec<_> = (0..7).map(|k| dev(1000 * (k + 1), 4, 3, k, 0)).collect();
    let e = run(&records);
    let got: Vec<_> = e.iter().map(|e| e.text.clone()).collect();
    assert_eq!(got[0], "MAG DEGRADED");
    for k in 1..7 {
        assert_eq!(
            got[k],
            format!("MAG: DEGRADED (error {} -> {})", errors[k - 1], errors[k])
        );
    }
    // a state or error value the table does not name
    let e = run(&[dev(1000, 1, 2, 0, 0), dev(2000, 1, 9, 8, 0)]);
    assert_eq!(e[1].text, "IMU1: OK -> state 9 (error 8)");
}

#[test]
fn every_msg_text_is_an_event() {
    let e = run(&[
        msg(1000, "first text"),
        msg(2000, "first text"),
        msg(3000, "second"),
    ]);
    assert_eq!(
        of(&e, EventSource::Msg),
        [(1000, "first text"), (2000, "first text"), (3000, "second")]
    );
    assert!(e.iter().all(|e| !e.initial));
}

#[test]
fn parm_change_per_name_and_unchanged_repeat_is_no_event() {
    let e = run(&[
        parm(1000, "AAA", 1.0),
        parm(1001, "BBB", 0.1),
        parm(2000, "AAA", 1.0), // unchanged repeat
        parm(2001, "BBB", 0.1),
        parm(3000, "AAA", 2.5), // changed
        parm(4000, "BBB", 0.2),
        parm(5000, "AAA", 2.5), // unchanged again
        parm(6000, "AAA", 1.0),
    ]);
    assert_eq!(
        of(&e, EventSource::Parm),
        [
            (3000, "AAA: 1 -> 2.5"),
            (4000, "BBB: 0.1 -> 0.2"),
            (6000, "AAA: 2.5 -> 1"),
        ]
    );
}

#[test]
fn gps_status_changes_and_no_coordinate_in_any_text() {
    let e = run(&[
        gps(1000, 1),
        gps(2000, 1),
        gps(3000, 3),
        gps(4000, 3),
        gps(5000, 2),
        gps(6000, 0),
        gps(7000, 4),
    ]);
    assert_eq!(
        of(&e, EventSource::Gps),
        [
            (1000, "GPS no fix"),
            (3000, "GPS: no fix -> 3D"),
            (5000, "GPS: 3D -> 2D"),
            (6000, "GPS: 2D -> none"),
            (7000, "GPS: none -> status 4"),
        ]
    );
    for ev in &e {
        for needle in ["12.3", "98.7", "123456", "987654", "-9"] {
            assert!(!ev.text.contains(needle), "{}", ev.text);
        }
    }
    assert!(e[0].initial && !e[1].initial);
}

#[test]
fn rcin_gap_of_exactly_one_second_is_no_gap() {
    // 1.0 s between the first and the second: still present; the log ends with the sticks present
    let e = run(&[rcin(1_000_000), rcin(2_000_000), rcin(3_000_000)]);
    assert_eq!(of(&e, EventSource::Rcin), [(1_000_000, "sticks present")]);
}

#[test]
fn rcin_gap_just_over_one_second_ends_and_restarts_presence() {
    let e = run(&[
        rcin(1_000_000),
        rcin(1_500_000),
        rcin(2_500_001), // 1 000 001 us after the last
        rcin(2_600_000),
    ]);
    assert_eq!(
        of(&e, EventSource::Rcin),
        [
            (1_000_000, "sticks present"),
            (1_500_000, "sticks absent"),
            (2_500_001, "sticks present"),
        ]
    );
}

#[test]
fn rcin_presence_at_the_end_of_the_log() {
    // the log goes on for exactly 1 s after the last RCIN: present to the end
    let e = run(&[rcin(1_000_000), msg(2_000_000, "x")]);
    assert_eq!(of(&e, EventSource::Rcin), [(1_000_000, "sticks present")]);
    // more than 1 s: the sticks were gone from the last RCIN on
    let e = run(&[rcin(1_000_000), rcin(1_200_000), msg(2_200_001, "x")]);
    assert_eq!(
        of(&e, EventSource::Rcin),
        [(1_000_000, "sticks present"), (1_200_000, "sticks absent")]
    );
    // and the events are in time order although "absent" is known late
    let times: Vec<_> = e.iter().map(|e| e.time_us).collect();
    assert!(times.windows(2).all(|w| w[0] <= w[1]), "{times:?}");
}

#[test]
fn dsf_dropped_records() {
    let e = run(&[
        dsf(1000, 0),
        dsf(2000, 0),
        dsf(3000, 3),
        dsf(4000, 3),
        dsf(5000, 5),
    ]);
    assert_eq!(
        of(&e, EventSource::Dsf),
        [
            (1000, "logger: 0 records dropped so far"),
            (3000, "3 records dropped (3 in all)"),
            (5000, "2 records dropped (5 in all)"),
        ]
    );
}

#[test]
fn a_log_without_the_types_or_of_another_firmware_gives_no_events_and_no_panic() {
    // no record at all
    assert!(events_of(Cursor::new(Vec::new()), None).unwrap().is_empty());
    // only a type of another name
    let mut f = fmt_record(150, "TST", "QI", "TimeUS,Cnt");
    f.extend(record(150, "QI", &[int(1000), int(1)]));
    assert!(events_of(Cursor::new(f), None).unwrap().is_empty());
    // types of our names with the columns of another firmware: DEV as ArduPilot's device-info record, GPS
    // without Status, MSG without Message, PARM without Value, DSF without Dp, a text where a number is read
    let mut f = Vec::new();
    f.extend(fmt_record(137, "DEV", "QBBI", "TimeUS,Id,Bus,Address"));
    f.extend(fmt_record(134, "GPS", "QBf", "TimeUS,I,Spd"));
    f.extend(fmt_record(130, "MSG", "QB", "TimeUS,Level"));
    f.extend(fmt_record(129, "PARM", "QN", "TimeUS,Name"));
    f.extend(fmt_record(138, "DSF", "QZ", "TimeUS,Dp"));
    f.extend(fmt_record(136, "RCIN", "B", "Chan"));
    f.extend(record(137, "QBBI", &[int(1), int(1), int(2), int(3)]));
    f.extend(record(134, "QBf", &[int(1), int(0), V::Float(1.0)]));
    f.extend(record(130, "QB", &[int(1), int(1)]));
    f.extend(record(129, "QN", &[int(1), V::Text("A")]));
    f.extend(record(138, "QZ", &[int(1), V::Text("x")]));
    f.extend(record(136, "B", &[int(1)]));
    assert!(events_of(Cursor::new(f), None).unwrap().is_empty());
}

#[test]
fn source_names_filter() {
    for s in EventSource::ALL {
        assert_eq!(EventSource::from_name(s.name()), Some(s));
    }
    assert_eq!(EventSource::from_name("IMU"), None);
    let e = run(&[msg(1000, "m"), dsf(2000, 0), msg(3000, "n")]);
    let msgs: Vec<_> = e.iter().filter(|e| e.source == EventSource::Msg).collect();
    assert_eq!(msgs.len(), 2);
}

#[test]
fn progress_cancel_and_file_path() {
    let mut file = header();
    for i in 0..6000 {
        file.extend(msg(1000 * (i + 1), "x")); // 75 bytes each: 450 kB
    }
    let total = file.len() as u64;
    let mut reader = Reader::new(Cursor::new(file.clone()), Some(total));
    let mut reports = Vec::new();
    let e = events_with(&mut reader, |p| {
        reports.push(p);
        Control::Continue
    })
    .unwrap()
    .unwrap();
    assert_eq!(e.len(), 6000);
    assert!(reports.len() >= 2);
    assert_eq!(reports.last().unwrap().bytes, total);

    let mut reader = Reader::new(Cursor::new(file.clone()), Some(total));
    let mut n = 0;
    let out = events_with(&mut reader, |_| {
        n += 1;
        Control::Stop
    })
    .unwrap();
    assert!(out.is_none());
    assert!(reader.bytes_consumed() < total);

    let dir = std::env::temp_dir().join(format!("gcs-dflog-events-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("synthetic.BIN");
    std::fs::write(&path, file).unwrap();
    assert_eq!(events(&path).unwrap().len(), 6000);
    std::fs::remove_dir_all(&dir).unwrap();
    assert!(events(dir.join("missing.BIN")).is_err());
}
