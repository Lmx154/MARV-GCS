//! The condition language: parser, binding, and the pass that finds intervals.

use std::io::Cursor;

use crate::tests::{V, fmt_record, int, record};
use crate::{
    Condition, ConditionError, Control, FieldRange, Found, MAX_INTERVALS, Reader, Summary,
    find_with, summary_of,
};

const BAT: &str = "QBff";
const IMU: &str = "QBfffffff";
const GPS: &str = "QBBIHBcLLeff";
const DEV: &str = "QBBBIIIHHH";

fn header() -> Vec<u8> {
    let mut f = Vec::new();
    f.extend(fmt_record(130, "MSG", "QZ", "TimeUS,Message"));
    f.extend(fmt_record(
        131,
        "IMU",
        IMU,
        "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,T",
    ));
    f.extend(fmt_record(
        134,
        "GPS",
        GPS,
        "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs",
    ));
    f.extend(fmt_record(135, "BAT", BAT, "TimeUS,Inst,Volt,Curr"));
    f.extend(fmt_record(
        137,
        "DEV",
        DEV,
        "TimeUS,Id,State,Err,Smp,Errs,TOs,Rec,Att,Ovf",
    ));
    // a type without an instance field, one without TimeUS, and a plain one for the operators
    f.extend(fmt_record(150, "T", "Qfff", "TimeUS,A,B,C"));
    f.extend(fmt_record(151, "NOT", "ff", "A,B"));
    f
}

fn bat(t: i64, volt: f64, curr: f64) -> Vec<u8> {
    record(135, BAT, &[int(t), int(0), V::Float(volt), V::Float(curr)])
}

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
            int(0),
            int(0),
            int(0),
            V::Float(0.0),
            V::Float(0.0),
        ],
    )
}

fn imu(t: i64, inst: i64, acc_z: f64) -> Vec<u8> {
    let f = V::Float(0.0);
    record(
        131,
        IMU,
        &[
            int(t),
            int(inst),
            f.clone(),
            f.clone(),
            f.clone(),
            f.clone(),
            f.clone(),
            V::Float(acc_z),
            f,
        ],
    )
}

fn dev(t: i64, id: i64, errs: i64) -> Vec<u8> {
    record(
        137,
        DEV,
        &[
            int(t),
            int(id),
            int(2),
            int(0),
            int(0),
            int(errs),
            int(0),
            int(0),
            int(0),
            int(0),
        ],
    )
}

fn msg(t: i64) -> Vec<u8> {
    record(130, "QZ", &[int(t), V::Text("note")])
}

fn t_rec(t: i64, a: f64, b: f64, c: f64) -> Vec<u8> {
    record(
        150,
        "Qfff",
        &[int(t), V::Float(a), V::Float(b), V::Float(c)],
    )
}

fn file_of(records: &[Vec<u8>]) -> Vec<u8> {
    let mut f = header();
    for r in records {
        f.extend(r);
    }
    f
}

fn summary_and_file(records: &[Vec<u8>]) -> (Summary, Vec<u8>) {
    let f = file_of(records);
    (summary_of(Cursor::new(f.clone()), None).unwrap(), f)
}

fn find(text: &str, records: &[Vec<u8>]) -> Found {
    let (s, f) = summary_and_file(records);
    let bound = Condition::parse(text).unwrap().bind(&s.formats).unwrap();
    let mut reader = Reader::new(Cursor::new(f), None);
    find_with(&mut reader, &bound, |_| Control::Continue)
        .unwrap()
        .unwrap()
}

/// Does `text` hold over one record of `T` with these values (every value exactly representable in f32)?
fn holds(text: &str, a: f64, b: f64, c: f64) -> bool {
    !find(text, &[t_rec(1000, a, b, c)]).intervals.is_empty()
}

fn parse_err(text: &str) -> ConditionError {
    Condition::parse(text).unwrap_err()
}

fn bind_err(text: &str) -> ConditionError {
    let (s, _) = summary_and_file(&[]);
    Condition::parse(text)
        .unwrap()
        .bind(&s.formats)
        .err()
        .unwrap_or_else(|| panic!("{text} should not bind"))
}

#[test]
fn comparison_operators() {
    // A = 1, B = 2
    for (text, want) in [
        ("T.A < T.B", true),
        ("T.B < T.A", false),
        ("T.A < T.A", false),
        ("T.A <= T.B", true),
        ("T.A <= T.A", true),
        ("T.B <= T.A", false),
        ("T.B > T.A", true),
        ("T.A > T.A", false),
        ("T.B >= T.A", true),
        ("T.A >= T.A", true),
        ("T.A >= T.B", false),
        ("T.A == T.A", true),
        ("T.A == T.B", false),
        ("T.A != T.B", true),
        ("T.A != T.A", false),
    ] {
        assert_eq!(holds(text, 1.0, 2.0, 0.0), want, "{text}");
    }
}

#[test]
fn arithmetic_and_precedence() {
    // A = 12, B = 2, C = 3
    let h = |text: &str| holds(text, 12.0, 2.0, 3.0);
    assert!(h("T.A + T.B == 14"));
    assert!(h("T.A - T.B == 10"));
    assert!(h("T.A * T.B == 24"));
    assert!(h("T.A / T.B == 6"));
    assert!(h("T.A + T.B * T.C == 18")); // * over +
    assert!(h("(T.A + T.B) * T.C == 42"));
    assert!(h("T.A - T.B - T.C == 7")); // left to right
    assert!(h("T.A / T.B / T.C == 2"));
    assert!(h("T.A - T.B + T.C == 13"));
    assert!(h("T.A / T.B * T.C == 18"));
    assert!(h("T.A * 0 + 2 * 3 + 4 * 5 == 26"));
    // arithmetic over comparison: the whole sum is on each side
    assert!(h("T.A + T.B > T.C * 4"));
    assert!(!h("T.A + T.B < T.C * 4"));
}

#[test]
fn unary_minus_and_abs() {
    let h = |text: &str| holds(text, 12.0, 2.0, -3.0);
    assert!(h("-T.B == -2"));
    assert!(h("- -T.B == 2"));
    assert!(h("-T.B * T.C == 6")); // unary binds tighter than *
    assert!(h("T.A - -T.B == 14"));
    assert!(h("-(T.A - T.B) == -10"));
    assert!(h("abs(T.C) == 3"));
    assert!(h("abs(T.C) == abs(-3)"));
    assert!(h("abs(T.B - T.A) == 10"));
    assert!(h("abs(-T.A) + 1 == 13"));
    assert!(h("abs(abs(T.C)) == 3"));
    assert!(h("T.C < -2"));
    assert!(!h("T.C < -4"));
}

#[test]
fn numbers_integer_decimal_exponent() {
    let h = |text: &str| holds(text, 10.0, 0.25, 0.0);
    assert!(h("T.A == 10"));
    assert!(h("T.B == 0.25"));
    assert!(h("T.B == .25"));
    assert!(h("T.B == 2.5e-1"));
    assert!(h("T.B == 25E-2"));
    assert!(h("T.A == 1e1"));
    assert!(h("T.A == 1e+1"));
    assert!(h("T.A == 10."));
}

#[test]
fn logic_and_its_precedence() {
    // A = 1, B = 2
    let h = |text: &str| holds(text, 1.0, 2.0, 0.0);
    assert!(h("T.A < T.B and T.B > T.A"));
    assert!(!h("T.A < T.B and T.B < T.A"));
    assert!(h("T.B < T.A or T.A < T.B"));
    assert!(!h("T.B < T.A or T.B < T.A"));
    assert!(h("not T.A > T.B"));
    assert!(!h("not T.A < T.B"));
    assert!(h("not not T.A < T.B"));
    // and over or: true or (false and false), not (true or false) and false
    assert!(h("T.A < T.B or T.A > T.B and T.A > T.B"));
    // comparison over not: not applies to the whole comparison
    assert!(h("not T.A + 1 > T.B"));
    // not over and: (not true) or true
    assert!(h("not T.A < T.B or T.A < T.B"));
    assert!(!h("not (T.A < T.B or T.A < T.B)"));
    assert!(h("(T.A < T.B or T.A > T.B) and T.B == 2"));
    assert!(h("((T.A < T.B))"));
    // keywords are lower case words, not prefixes of names
    assert!(h("T.A<T.B and(T.B>T.A)"));
}

#[test]
fn references_with_and_without_an_instance() {
    for text in [
        "BAT.Volt < 14.8",
        "IMU[0].AccZ < -15",
        "DEV[7].Errs > 100 and GPS.Status < 3",
        "IMU [ 12 ] . AccZ < 1",
        "  BAT.Volt<14.8  ",
    ] {
        let (s, _) = summary_and_file(&[]);
        Condition::parse(text)
            .unwrap_or_else(|e| panic!("{text}: {e}"))
            .bind(&s.formats)
            .unwrap_or_else(|e| panic!("{text}: {e}"));
    }
}

#[test]
fn malformed_inputs_with_their_positions() {
    let at = |text: &str, position: usize, reason: &str| {
        let e = parse_err(text);
        assert_eq!(e.position, position, "{text}: {e}");
        assert!(e.reason.contains(reason), "{text}: {e}");
    };
    at("", 0, "empty");
    at("   ", 0, "empty");
    at("BAT.Volt <", 10, "ends too early");
    at("BAT.Volt < 14.8 and", 19, "ends too early");
    at("(BAT.Volt < 1", 13, "')'");
    at("BAT.Volt < 1)", 12, "unexpected ')'");
    at("BAT.Volt = 3", 9, "'=='");
    at("BAT.Volt < 1 < 2", 13, "chained");
    at("BAT.Volt", 0, "comparison is expected");
    at("BAT < 1", 4, "field name");
    at("BAT.Volt < 1 $", 13, "unexpected character '$'");
    at("BAT.Volt < 1 !", 13, "'!'");
    at("BAT.Volt < 1 2", 13, "unexpected the number 2");
    at("BAT.Volt < 1e", 12, "exponent");
    at("abs(BAT.Volt < 1) < 2", 4, "number is expected");
    at("foo(1) < 2", 0, "unknown function 'foo'");
    at("not BAT.Volt", 4, "comparison is expected");
    at("BAT.Volt and 1 < 2", 0, "comparison is expected");
    at("BAT.Volt + (1 < 2) < 3", 11, "number is expected");
    at("IMU[x].AccZ < 1", 4, "whole number");
    at("IMU[1.5].AccZ < 1", 4, "whole number");
    at("IMU[1.AccZ < 1", 4, "whole number");
    at("BAT.1 < 2", 3, "'.' and a field name");
    at("1 + < 2", 4, "unexpected '<'");
    at("and BAT.Volt < 1", 0, "unexpected 'and'");
    at("BAT.Volt < 1 and or BAT.Volt < 2", 17, "unexpected 'or'");
    at("abs() < 1", 4, "unexpected ')'");
    // a position is a character offset, not a byte offset: NBSP is two bytes
    at("BAT.Volt\u{a0}= 1", 9, "'=='");
}

#[test]
fn binding_errors_are_reported_before_any_pass() {
    let at = |text: &str, position: usize, reason: &str| {
        let e = bind_err(text);
        assert_eq!(e.position, position, "{text}: {e}");
        assert!(e.reason.contains(reason), "{text}: {e}");
    };
    at("FOO.Volt < 1", 0, "no record type 'FOO'");
    at("BAT.Volt < 1 and Nope.X > 1", 17, "no record type 'Nope'");
    at("BAT.Nope < 1", 4, "no field 'Nope'");
    at("BAT.Volt < 1 and GPS.Nope > 2", 21, "no field 'Nope'");
    at("MSG.Message < 1", 4, "text");
    at("T[1].A < 1", 1, "no instance field");
    at("MSG[0].TimeUS > 1", 3, "no instance field");
    at("NOT.A < 1", 0, "no TimeUS");
    at("1 < 2", 0, "names no record field");
    at("abs(-3) >= 3 or 1 == 1", 0, "names no record field");
    // the same names bind where the log has them
    let (s, _) = summary_and_file(&[]);
    assert!(
        Condition::parse("IMU[1].AccZ < 1")
            .unwrap()
            .bind(&s.formats)
            .is_ok()
    );
    // a log of another firmware: the type is there with other columns
    let mut f = fmt_record(135, "BAT", "QBf", "TimeUS,Inst,Volt");
    f.extend(record(135, "QBf", &[int(1), int(0), V::Float(1.0)]));
    let s = summary_of(Cursor::new(f), None).unwrap();
    let e = Condition::parse("BAT.Curr < 1")
        .unwrap()
        .bind(&s.formats)
        .err()
        .unwrap();
    assert_eq!(e.position, 4);
    assert!(
        Condition::parse("BAT.Volt < 1")
            .unwrap()
            .bind(&s.formats)
            .is_ok()
    );
}

fn r(field: &str, min: f64, max: f64) -> FieldRange {
    FieldRange {
        field: field.to_owned(),
        min,
        max,
    }
}

#[test]
fn latest_value_across_two_types() {
    let found = find(
        "BAT.Volt < 14 and GPS.Status < 3",
        &[
            bat(1000, 15.0, 0.0), // GPS not seen yet: false
            gps(2000, 1),         // seen, but the battery is fine: false
            bat(3000, 13.0, 0.0), // true: start
            imu(3500, 0, 0.0),    // a record of an unnamed type changes nothing
            bat(4000, 12.0, 0.0), // still true
            gps(5000, 3),         // false: the interval ends here
            msg(6000),
            gps(7000, 2), // true again, with the battery still at 12 V
            bat(8000, 20.0, 0.0),
        ],
    );
    assert_eq!(found.not_kept, 0);
    assert_eq!(found.intervals.len(), 2);
    let a = &found.intervals[0];
    assert_eq!(
        (a.start_us, a.end_us, a.duration_us),
        (3000, Some(5000), 2000)
    );
    // the record that ended the interval (GPS 3, BAT 20) is not part of its range
    assert_eq!(
        a.ranges,
        [r("BAT.Volt", 12.0, 13.0), r("GPS.Status", 1.0, 1.0)]
    );
    let b = &found.intervals[1];
    assert_eq!(
        (b.start_us, b.end_us, b.duration_us),
        (7000, Some(8000), 1000)
    );
    assert_eq!(
        b.ranges,
        [r("BAT.Volt", 12.0, 12.0), r("GPS.Status", 2.0, 2.0)]
    );
}

#[test]
fn nothing_is_true_until_every_named_type_has_a_record() {
    let found = find(
        "BAT.Volt < 14 and GPS.Status < 3",
        &[bat(1000, 1.0, 0.0), bat(2000, 1.0, 0.0), msg(3000)],
    );
    assert!(found.intervals.is_empty());
    assert_eq!(found.total(), 0);
    // a condition that is true by itself is still false while a named type is missing
    let found = find("BAT.Volt < 14 or GPS.Status < 3", &[bat(1000, 1.0, 0.0)]);
    assert!(found.intervals.is_empty());
}

#[test]
fn instance_picks_the_right_record_when_instances_interleave() {
    let records = [
        imu(1000, 0, -20.0),
        imu(2000, 1, 0.0),
        imu(3000, 0, -20.0),
        imu(4000, 1, -20.0), // instance 1 now low: true
        imu(5000, 0, 5.0),   // instance 0 recovers; instance 1 is still low
        imu(6000, 1, -16.0),
        imu(7000, 1, 0.0), // instance 1 recovers: false
    ];
    let found = find("IMU[1].AccZ < -15", &records);
    assert_eq!(found.intervals.len(), 1);
    let i = &found.intervals[0];
    assert_eq!((i.start_us, i.end_us), (4000, Some(7000)));
    assert_eq!(i.ranges, [r("IMU[1].AccZ", -20.0, -16.0)]);
    // cut after the instance-1 record at 6000: the interval is open and its range is -20 to -16
    let found = find("IMU[1].AccZ < -15", &records[..6]);
    assert_eq!(found.intervals[0].ranges, [r("IMU[1].AccZ", -20.0, -16.0)]);
    assert_eq!(found.intervals[0].end_us, None);

    // plain IMU.AccZ is the latest of any instance, which gives other intervals
    let any = find("IMU.AccZ < -15", &records);
    let spans: Vec<_> = any
        .intervals
        .iter()
        .map(|i| (i.start_us, i.end_us))
        .collect();
    assert_eq!(
        spans,
        [(1000, Some(2000)), (3000, Some(5000)), (6000, Some(7000))]
    );
    let zero = find("IMU[0].AccZ < -15", &records);
    let spans: Vec<_> = zero
        .intervals
        .iter()
        .map(|i| (i.start_us, i.end_us))
        .collect();
    assert_eq!(spans, [(1000, Some(5000))]);

    // an instance that never appears: false for the whole log
    assert!(find("IMU[2].AccZ < 100", &records).intervals.is_empty());

    // DEV[7].Errs with the rows of other devices in between
    let found = find(
        "DEV[7].Errs > 100",
        &[
            dev(1000, 7, 5),
            dev(2000, 3, 500),
            dev(3000, 7, 150),
            dev(4000, 3, 0),
            dev(5000, 7, 200),
            dev(6000, 7, 50),
        ],
    );
    assert_eq!(found.intervals.len(), 1);
    assert_eq!(found.intervals[0].start_us, 3000);
    assert_eq!(found.intervals[0].end_us, Some(6000));
    assert_eq!(found.intervals[0].ranges, [r("DEV[7].Errs", 150.0, 200.0)]);
}

#[test]
fn an_interval_still_open_at_the_end_of_the_log() {
    let found = find(
        "BAT.Volt < 14",
        &[
            bat(1000, 15.0, 0.0),
            bat(5000, 13.0, 0.0),
            bat(6000, 11.0, 0.0),
            msg(9000), // the last record of the log, of another type
        ],
    );
    assert_eq!(found.intervals.len(), 1);
    let i = &found.intervals[0];
    assert_eq!(i.start_us, 5000);
    assert_eq!(i.end_us, None);
    assert_eq!(i.duration_us, 4000); // to the log's end, 9000
    assert_eq!(i.ranges, [r("BAT.Volt", 11.0, 13.0)]);
}

#[test]
fn min_and_max_of_every_named_field() {
    let found = find(
        "BAT.Volt < 14 and BAT.Curr > 1 and abs(BAT.Volt - 12) < 5",
        &[
            bat(1000, 13.0, 2.0),
            bat(2000, 12.5, 7.0),
            bat(3000, 10.0, 3.0),
            bat(4000, 13.5, 1.5),
            bat(5000, 9.0, 0.5), // Curr low: ends
        ],
    );
    assert_eq!(found.intervals.len(), 1);
    assert_eq!(
        found.intervals[0].ranges,
        [r("BAT.Volt", 10.0, 13.5), r("BAT.Curr", 1.5, 7.0)]
    );
    // a field named twice is one range
    let found = find("BAT.Volt > 1 and BAT.Volt < 20", &[bat(1000, 5.0, 0.0)]);
    assert_eq!(found.intervals[0].ranges.len(), 1);
}

#[test]
fn at_most_ten_thousand_intervals_are_kept_and_the_rest_counted() {
    let n = MAX_INTERVALS as i64 + 5;
    let mut records = Vec::new();
    for k in 0..n {
        records.push(bat(2 * k + 1, 10.0, 0.0)); // true
        records.push(bat(2 * k + 2, 20.0, 0.0)); // false
    }
    let found = find("BAT.Volt < 14", &records);
    assert_eq!(found.intervals.len(), MAX_INTERVALS);
    assert_eq!(found.not_kept, 5);
    assert_eq!(found.total(), n as u64);
    assert_eq!(found.intervals[0].start_us, 1);
    assert_eq!(
        found.intervals.last().unwrap().start_us,
        2 * (MAX_INTERVALS as u64 - 1) + 1
    );

    // an interval still open at the end counts as well
    let mut records = Vec::new();
    for k in 0..MAX_INTERVALS as i64 {
        records.push(bat(2 * k + 1, 10.0, 0.0));
        records.push(bat(2 * k + 2, 20.0, 0.0));
    }
    records.push(bat(1_000_000, 10.0, 0.0));
    let found = find("BAT.Volt < 14", &records);
    assert_eq!(found.intervals.len(), MAX_INTERVALS);
    assert_eq!(found.not_kept, 1);
}

#[test]
fn division_by_zero_and_nan_compare_false() {
    // A = 1, B = 0
    let h = |text: &str| holds(text, 1.0, 0.0, 0.0);
    assert!(!h("T.A / T.B > 0"));
    assert!(!h("T.A / T.B < 0"));
    assert!(!h("T.A / T.B == T.A / T.B"));
    assert!(!h("T.A / T.B != 1"));
    assert!(!h("T.B / T.B >= 0"));
    assert!(h("not T.A / T.B > 0")); // the comparison is false, so its negation is true
    assert!(!h("T.A / T.B > 0 or T.A / T.B <= 0"));
    // a NaN in the log
    let nan = |text: &str| {
        !find(text, &[t_rec(1000, f64::NAN, 1.0, 1.0)])
            .intervals
            .is_empty()
    };
    assert!(!nan("T.A < 1"));
    assert!(!nan("T.A >= 1"));
    assert!(!nan("T.A == T.A"));
    assert!(!nan("T.A != 1"));
    assert!(nan("T.B == 1"));
}

#[test]
fn progress_and_cancel_pass_through() {
    let mut records = Vec::new();
    for k in 0..30_000 {
        records.push(bat(k + 1, 10.0, 0.0)); // 20 bytes each: 600 kB
    }
    let (s, f) = summary_and_file(&records);
    let bound = Condition::parse("BAT.Volt < 14")
        .unwrap()
        .bind(&s.formats)
        .unwrap();
    let total = f.len() as u64;
    let mut reader = Reader::new(Cursor::new(f.clone()), Some(total));
    let mut reports = 0;
    let found = find_with(&mut reader, &bound, |p| {
        assert_eq!(p.total, Some(total));
        reports += 1;
        Control::Continue
    })
    .unwrap()
    .unwrap();
    assert!(reports >= 2);
    assert_eq!(found.intervals.len(), 1);
    assert_eq!(found.intervals[0].end_us, None);
    assert_eq!(bound.fields().collect::<Vec<_>>(), ["BAT.Volt"]);

    let mut reader = Reader::new(Cursor::new(f), Some(total));
    let out = find_with(&mut reader, &bound, |_| Control::Stop).unwrap();
    assert!(out.is_none());
    assert!(reader.bytes_consumed() < total);
}
