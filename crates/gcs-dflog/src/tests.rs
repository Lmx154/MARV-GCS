//! Tests on synthetic bytes built here; no real log, no real position anywhere.

use std::io::{self, Cursor, Read};

use crate::{Control, Reader, Value, scan, summary, summary_of, summary_with};

/// A value to encode into one field of a test record.
#[derive(Clone, Debug)]
enum V {
    Int(i64),
    Float(f64),
    Text(&'static str),
    Array([i16; 32]),
    Raw(Vec<u8>),
}

fn char_size(c: char) -> usize {
    match c {
        'b' | 'B' | 'M' => 1,
        'h' | 'H' | 'g' | 'c' | 'C' => 2,
        'i' | 'I' | 'f' | 'e' | 'E' | 'L' | 'n' => 4,
        'q' | 'Q' | 'd' => 8,
        'N' => 16,
        'Z' | 'a' => 64,
        _ => panic!("test helper: unknown format character {c}"),
    }
}

fn record_len(format: &str) -> u8 {
    (3 + format.chars().map(char_size).sum::<usize>()) as u8
}

fn pad(text: &str, size: usize) -> Vec<u8> {
    let mut v = text.as_bytes().to_vec();
    assert!(v.len() <= size);
    v.resize(size, 0);
    v
}

/// The FMT record defining `type_id`.
fn fmt_record(type_id: u8, name: &str, format: &str, columns: &str) -> Vec<u8> {
    let mut r = vec![0xA3, 0x95, 128, type_id, record_len(format)];
    r.extend(pad(name, 4));
    r.extend(pad(format, 16));
    r.extend(pad(columns, 64));
    assert_eq!(r.len(), 89);
    r
}

/// A record of `type_id` whose fields are encoded by the characters of `format`.
fn record(type_id: u8, format: &str, values: &[V]) -> Vec<u8> {
    assert_eq!(format.chars().count(), values.len());
    let mut r = vec![0xA3, 0x95, type_id];
    for (c, v) in format.chars().zip(values) {
        let before = r.len();
        match (c, v) {
            (_, V::Raw(b)) => r.extend(b),
            ('b' | 'B' | 'M', V::Int(i)) => r.push(*i as u8),
            ('h' | 'H' | 'c' | 'C', V::Int(i)) => r.extend((*i as i16).to_le_bytes()),
            ('i' | 'I' | 'e' | 'E' | 'L', V::Int(i)) => r.extend((*i as i32).to_le_bytes()),
            ('q' | 'Q', V::Int(i)) => r.extend(i.to_le_bytes()),
            ('f', V::Float(x)) => r.extend((*x as f32).to_le_bytes()),
            ('d', V::Float(x)) => r.extend(x.to_le_bytes()),
            ('n', V::Text(s)) => r.extend(pad(s, 4)),
            ('N', V::Text(s)) => r.extend(pad(s, 16)),
            ('Z', V::Text(s)) => r.extend(pad(s, 64)),
            ('a', V::Array(a)) => r.extend(a.iter().flat_map(|x| x.to_le_bytes())),
            _ => panic!("test helper: {v:?} does not fit '{c}'"),
        }
        assert_eq!(r.len() - before, char_size(c));
    }
    r
}

fn int(i: i64) -> V {
    V::Int(i)
}

fn number(v: Option<Value<'_>>) -> f64 {
    match v {
        Some(Value::Number(x)) => x,
        other => panic!("not a number: {other:?}"),
    }
}

fn near(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-12 * b.abs().max(1.0), "{a} != {b}");
}

#[test]
fn every_format_character() {
    // the format string of a type is at most 16 characters: two types
    let format = "bBhHiIqQfdgcCeEL";
    let format2 = "MnNZa";
    // g: f16 bits 0x4900 = 10.0 (exponent 18 - 15 = 3, mantissa 0.25: 1.25 * 8)
    let mut arr = [0i16; 32];
    arr[0] = -5;
    arr[1] = 7;
    arr[31] = 32767;
    let values = vec![
        int(-100),
        int(200),
        int(-30000),
        int(60000),
        int(-2_000_000_000),
        int(4_000_000_000),
        int(-1 << 40),
        int(1 << 41),
        V::Float(1.5),
        V::Float(-2.25),
        V::Raw(0x4900u16.to_le_bytes().to_vec()),
        int(-1234),
        int(65000),
        int(-123_456_789),
        int(4_000_000_000),
        int(-351_234_567),
    ];
    let values2 = vec![
        int(7),
        V::Text("AbCd"),
        V::Text("sixteen chars ok"),
        V::Text("a short text"),
        V::Array(arr),
    ];
    let columns = "B1,B2,H1,H2,I1,I2,Q1,Q2,F,D,G,C1,C2,E1,E2,Lat";
    let mut file = fmt_record(150, "ALL", format, columns);
    file.extend(fmt_record(151, "TXT", format2, "Md,N4,N16,Z64,A"));
    file.extend(record(150, format, &values));
    file.extend(record(151, format2, &values2));

    let mut reader = Reader::new(Cursor::new(file), None);
    assert_eq!(reader.next_record().unwrap().unwrap().type_name(), "FMT");
    assert_eq!(reader.next_record().unwrap().unwrap().type_name(), "FMT");
    let r = reader.next_record().unwrap().unwrap();
    assert_eq!(r.type_name(), "ALL");
    assert_eq!(r.len(), 16);
    assert_eq!(r.columns().len(), 16);
    let n = |i| number(r.value(i));
    near(n(0), -100.0);
    near(n(1), 200.0);
    near(n(2), -30000.0);
    near(n(3), 60000.0);
    near(n(4), -2_000_000_000.0);
    near(n(5), 4_000_000_000.0);
    near(n(6), -((1u64 << 40) as f64));
    near(n(7), (1u64 << 41) as f64);
    near(n(8), 1.5);
    near(n(9), -2.25);
    near(n(10), 10.0);
    // scaled: c, C x0.01; e, E x0.01; L x1e-7
    near(n(11), -12.34);
    near(n(12), 650.0);
    near(n(13), -1_234_567.89);
    near(n(14), 40_000_000.0);
    near(n(15), -35.1234567);
    near(number(r.get("Lat")), -35.1234567);
    near(r.number_of("E1").unwrap(), -1_234_567.89);
    assert!(r.get("Nope").is_none());
    assert!(r.value(16).is_none());
    let r = reader.next_record().unwrap().unwrap();
    assert_eq!(r.type_name(), "TXT");
    near(number(r.value(0)), 7.0);
    assert_eq!(r.value(1), Some(Value::Text("AbCd".into())));
    assert_eq!(r.value(2), Some(Value::Text("sixteen chars ok".into())));
    assert_eq!(r.value(3), Some(Value::Text("a short text".into())));
    match r.value(4) {
        Some(Value::Array(a)) => {
            assert_eq!(a[0], -5);
            assert_eq!(a[1], 7);
            assert_eq!(a[2], 0);
            assert_eq!(a[31], 32767);
        }
        other => panic!("{other:?}"),
    }
    // strings and arrays are not numbers
    assert_eq!(r.number_of("N4"), None);
    assert_eq!(r.number_of("A"), None);
    assert_eq!(r.get("N16"), Some(Value::Text("sixteen chars ok".into())));
    assert!(reader.next_record().unwrap().is_none());
    assert_eq!(reader.bad_bytes(), 0);
    assert_eq!(reader.tail_bytes(), 0);
}

#[test]
fn half_precision_values() {
    let mut file = fmt_record(150, "H16", "g", "V");
    for bits in [
        0x0000u16, 0x3c00, 0xc000, 0x7bff, 0x0001, 0x7c00, 0xfc00, 0x7e00,
    ] {
        file.extend(record(150, "g", &[V::Raw(bits.to_le_bytes().to_vec())]));
    }
    let mut reader = Reader::new(Cursor::new(file), None);
    reader.next_record().unwrap().unwrap();
    let mut got = Vec::new();
    while let Some(r) = reader.next_record().unwrap() {
        got.push(number(r.value(0)));
    }
    assert_eq!(got[0], 0.0);
    assert_eq!(got[1], 1.0);
    assert_eq!(got[2], -2.0);
    assert_eq!(got[3], 65504.0);
    assert_eq!(got[4], 2f64.powi(-24));
    assert_eq!(got[5], f64::INFINITY);
    assert_eq!(got[6], f64::NEG_INFINITY);
    assert!(got[7].is_nan());
}

/// A small file: type 150 "TST" with TimeUS and a counter.
fn small_file(n: u64) -> Vec<u8> {
    let mut f = fmt_record(150, "TST", "QI", "TimeUS,Cnt");
    for i in 0..n {
        f.extend(record(
            150,
            "QI",
            &[int(1000 * (i as i64 + 1)), int(i as i64)],
        ));
    }
    f
}

#[test]
fn resynchronises_over_garbage() {
    let mut file = small_file(2);
    // planted garbage, with a false header (A3 95 then an unknown type) inside: 5 bytes
    file.extend([0x11, 0xA3, 0x95, 0x77, 0x22]);
    file.extend(record(150, "QI", &[int(3000), int(2)]));
    // and a lone A3 then 3 more: 4 bytes
    file.extend([0xA3, 0x00, 0x00, 0x00]);
    file.extend(record(150, "QI", &[int(4000), int(3)]));
    let mut reader = Reader::new(Cursor::new(file), None);
    let mut counters = Vec::new();
    while let Some(r) = reader.next_record().unwrap() {
        if r.type_name() == "TST" {
            counters.push(number(r.get("Cnt")));
        }
    }
    assert_eq!(counters, [0.0, 1.0, 2.0, 3.0]);
    assert_eq!(reader.bad_bytes(), 9);
    assert_eq!(reader.tail_bytes(), 0);
    assert_eq!(reader.records(), 5);
}

#[test]
fn truncated_last_record_is_tail_not_error() {
    let mut file = small_file(3);
    let whole = file.len();
    let extra = record(150, "QI", &[int(9000), int(9)]);
    file.extend(&extra[..7]);
    let mut reader = Reader::new(Cursor::new(file), None);
    let mut n = 0;
    while reader.next_record().unwrap().is_some() {
        n += 1;
    }
    assert_eq!(n, 4); // FMT + 3
    assert_eq!(reader.tail_bytes(), 7);
    assert_eq!(reader.bad_bytes(), 0);
    assert_eq!(reader.bytes_consumed(), whole as u64 + 7);

    // one or two stray bytes at the very end are tail as well
    let mut file = small_file(1);
    file.extend([0xA3, 0x95]);
    let s = summary_of(Cursor::new(file), None).unwrap();
    assert_eq!(s.tail_bytes, 2);
    assert_eq!(s.records, 2);
}

#[test]
fn unknown_type_is_bad_bytes_until_the_next_good_header() {
    let mut file = small_file(1);
    // type 151 has no FMT: its 3 + 12 bytes are bad
    let unknown = record(151, "QI", &[int(1), int(2)]);
    assert_eq!(unknown.len(), 15);
    file.extend(&unknown);
    file.extend(record(150, "QI", &[int(2000), int(1)]));
    let s = summary_of(Cursor::new(file), None).unwrap();
    assert_eq!(s.bad_bytes, 15);
    assert_eq!(s.records, 3);
    assert_eq!(s.counts.iter().find(|c| c.name == "TST").unwrap().count, 2);
}

#[test]
fn a_fmt_with_the_wrong_length_defines_nothing() {
    let mut bad = fmt_record(151, "BAD", "QI", "TimeUS,Cnt");
    bad[4] = 20; // should be 15
    let mut file = bad;
    file.extend(record(151, "QI", &[int(1), int(2)]));
    let s = summary_of(Cursor::new(file), None).unwrap();
    assert_eq!(s.records, 1);
    // 15 bytes of the undefined type: all but the last two are bad, those two are the tail
    assert_eq!((s.bad_bytes, s.tail_bytes), (13, 2));
    assert_eq!(s.formats.len(), 1);
}

/// A reader that gives one byte at a time, to meet every buffer boundary.
struct Dribble(Cursor<Vec<u8>>);

impl Read for Dribble {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let n = buf.len().min(1);
        self.0.read(&mut buf[..n])
    }
}

#[test]
fn same_result_whatever_the_read_sizes() {
    let mut file = small_file(50);
    file.extend([1, 2, 3]);
    file.extend(record(150, "QI", &[int(99_000), int(50)]));
    file.extend(&record(150, "QI", &[int(1), int(1)])[..10]);
    let a = summary_of(Cursor::new(file.clone()), None).unwrap();
    let b = summary_of(Dribble(Cursor::new(file)), None).unwrap();
    assert_eq!(a.records, 52);
    assert_eq!(a.bad_bytes, 3);
    assert_eq!(a.tail_bytes, 10);
    assert_eq!(
        (a.records, a.bad_bytes, a.tail_bytes),
        (b.records, b.bad_bytes, b.tail_bytes)
    );
    assert_eq!(a.last_time_us, b.last_time_us);
}

#[test]
fn a_file_larger_than_the_buffer() {
    // 20 000 records of 15 bytes = 300 kB > the 256 KiB buffer
    let file = small_file(20_000);
    let s = summary_of(Cursor::new(file.clone()), Some(file.len() as u64)).unwrap();
    assert_eq!(s.records, 20_001);
    assert_eq!(s.bytes, file.len() as u64);
    assert_eq!(s.first_time_us, Some(1000));
    assert_eq!(s.last_time_us, Some(20_000_000));
    assert_eq!(s.duration_us(), Some(19_999_000));
}

#[test]
fn summary_counts_times_and_formats() {
    let mut file = fmt_record(150, "TST", "QI", "TimeUS,Cnt");
    file.extend(fmt_record(151, "NOT", "BH", "A,B"));
    file.extend(record(150, "QI", &[int(5_000_000), int(0)]));
    file.extend(record(151, "BH", &[int(1), int(2)]));
    file.extend([0, 0, 0]);
    file.extend(record(150, "QI", &[int(7_500_000), int(1)]));
    file.extend(record(150, "QI", &[int(9_000_000), int(2)]));
    let s = summary_of(Cursor::new(file), None).unwrap();
    assert_eq!(s.records, 2 + 4);
    assert_eq!(s.first_time_us, Some(5_000_000));
    assert_eq!(s.last_time_us, Some(9_000_000));
    assert_eq!(s.duration_us(), Some(4_000_000));
    assert_eq!(s.bad_bytes, 3);
    assert_eq!(s.tail_bytes, 0);
    let counts: Vec<_> = s
        .counts
        .iter()
        .map(|c| (c.type_id, c.name.as_str(), c.count))
        .collect();
    assert_eq!(counts, [(128, "FMT", 2), (150, "TST", 3), (151, "NOT", 1)]);
    let names: Vec<_> = s.formats.iter().map(|f| f.name.as_str()).collect();
    assert_eq!(names, ["FMT", "TST", "NOT"]);
    let tst = &s.formats[1];
    assert_eq!(
        (tst.format.as_str(), tst.columns.as_slice()),
        ("QI", ["TimeUS".to_string(), "Cnt".to_string()].as_slice())
    );
    assert!(s.time_base_unix_us.is_none());
    assert!(s.start_unix_ms().is_none());
}

#[test]
fn gps_wall_clock() {
    let gps = "QIHL";
    let mut file = fmt_record(160, "GPS", gps, "TimeUS,GMS,GWk,Lat");
    file.extend(fmt_record(150, "TST", "QI", "TimeUS,Cnt"));
    file.extend(record(150, "QI", &[int(1_000_000), int(0)]));
    // no GPS time yet: GWk = 0 must be ignored
    file.extend(record(
        160,
        gps,
        &[int(2_000_000), int(111), int(0), int(0)],
    ));
    // GWk 2400, GMS 345 600 000 (4 days into the week), at TimeUS 5 000 000.
    // GPS epoch 1980-01-06 = Unix 315 964 800 s.
    // 2400 weeks * 604 800 s = 1 451 520 000 s; 345 600 000 ms = 345 600 s; leap seconds 18 s.
    // Unix s at that record = 315 964 800 + 1 451 520 000 + 345 600 - 18 = 1 767 830 382
    // (2026-01-07 23:59:42 UTC). The first TimeUS of the log is 1 000 000, 4 s earlier:
    // start = 1 767 830 378 s = 1 767 830 378 000 ms.
    file.extend(record(
        160,
        gps,
        &[int(5_000_000), int(345_600_000), int(2400), int(0)],
    ));
    // a later GPS record does not change the base
    file.extend(record(
        160,
        gps,
        &[int(8_000_000), int(348_600_000), int(2400), int(0)],
    ));
    let s = summary_of(Cursor::new(file), None).unwrap();
    assert_eq!(s.time_base_unix_us, Some(1_767_830_382_000_000 - 5_000_000));
    assert_eq!(s.first_time_us, Some(1_000_000));
    assert_eq!(s.start_unix_ms(), Some(1_767_830_378_000));
    assert_eq!(s.unix_ms_at(5_000_000), Some(1_767_830_382_000));
}

#[test]
fn no_gps_time_without_a_week() {
    let gps = "QIH";
    let mut file = fmt_record(160, "GPS", gps, "TimeUS,GMS,GWk");
    file.extend(record(
        160,
        gps,
        &[int(5_000_000), int(345_600_000), int(0)],
    ));
    let s = summary_of(Cursor::new(file), None).unwrap();
    assert!(s.time_base_unix_us.is_none());
}

#[test]
fn progress_and_cancellation() {
    let file = small_file(60_000); // 900 kB
    let total = file.len() as u64;
    let mut reader = Reader::new(Cursor::new(file.clone()), Some(total));
    let mut reports = Vec::new();
    let s = summary_with(&mut reader, |p| {
        reports.push(p);
        Control::Continue
    })
    .unwrap()
    .unwrap();
    assert_eq!(s.records, 60_001);
    assert!(reports.len() >= 3, "{}", reports.len());
    assert!(reports.windows(2).all(|w| w[0].bytes <= w[1].bytes));
    assert_eq!(reports.last().unwrap().bytes, total);
    assert!(reports.iter().all(|p| p.total == Some(total)));

    let mut reader = Reader::new(Cursor::new(file), Some(total));
    let mut seen = 0;
    let out = summary_with(&mut reader, |_| {
        seen += 1;
        if seen == 2 {
            Control::Stop
        } else {
            Control::Continue
        }
    })
    .unwrap();
    assert!(out.is_none());
    assert!(reader.bytes_consumed() < total);

    let mut reader = Reader::new(Cursor::new(small_file(10)), None);
    let mut n = 0;
    assert!(scan(&mut reader, |_| n += 1, |_| Control::Continue).unwrap());
    assert_eq!(n, 11);
}

#[test]
fn summary_of_a_file_path() {
    let dir = std::env::temp_dir().join(format!("gcs-dflog-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("synthetic.BIN");
    std::fs::write(&path, small_file(5)).unwrap();
    let s = summary(&path).unwrap();
    assert_eq!(s.records, 6);
    assert_eq!(s.duration_us(), Some(4000));
    std::fs::remove_dir_all(&dir).unwrap();
    assert!(summary(dir.join("missing.BIN")).is_err());
}

/// The eleven definitions of the firmware's table (`fw-core/src/dflog.rs`: names, formats, columns, type
/// numbers), read back from records encoded here by plain `to_le_bytes`, independent of the helper above.
#[test]
fn firmware_table_cross_check() {
    let table: [(u8, &str, &str, &str); 11] = [
        (128, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns"),
        (129, "PARM", "QNff", "TimeUS,Name,Value,Default"),
        (130, "MSG", "QZ", "TimeUS,Message"),
        (
            131,
            "IMU",
            "QBfffffff",
            "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,T",
        ),
        (132, "BARO", "QBfc", "TimeUS,I,Press,Temp"),
        (133, "MAG", "QBhhh", "TimeUS,I,MagX,MagY,MagZ"),
        (
            134,
            "GPS",
            "QBBIHBcLLeff",
            "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs",
        ),
        (135, "BAT", "QBff", "TimeUS,Inst,Volt,Curr"),
        (
            136,
            "RCIN",
            "QHHHHHHHHHHHHHH",
            "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14",
        ),
        (
            137,
            "DEV",
            "QBBBIIIHHH",
            "TimeUS,Id,State,Err,Smp,Errs,TOs,Rec,Att,Ovf",
        ),
        (138, "DSF", "QIHIIII", "TimeUS,Dp,Blk,Bytes,FMn,FMx,FAv"),
    ];
    let mut file = Vec::new();
    for (id, name, format, columns) in table {
        file.extend(fmt_record(id, name, format, columns));
    }
    let head = |t: u8| vec![0xA3u8, 0x95, t];
    // PARM
    let mut r = head(129);
    r.extend(1_000_000u64.to_le_bytes());
    r.extend(pad("SYSID_THISMAV", 16));
    r.extend(1.0f32.to_le_bytes());
    r.extend(0.5f32.to_le_bytes());
    file.extend(r);
    // MSG
    let mut r = head(130);
    r.extend(2_000_000u64.to_le_bytes());
    r.extend(pad("made-up text", 64));
    file.extend(r);
    // IMU
    let mut r = head(131);
    r.extend(3_000_000u64.to_le_bytes());
    r.push(1);
    for x in [0.25f32, -0.5, 1.0, 0.0, 0.0, 9.5, 21.5] {
        r.extend(x.to_le_bytes());
    }
    file.extend(r);
    // BARO: Temp is centi-degrees in an i16
    let mut r = head(132);
    r.extend(4_000_000u64.to_le_bytes());
    r.push(0);
    r.extend(101_325.0f32.to_le_bytes());
    r.extend(2150i16.to_le_bytes());
    file.extend(r);
    // MAG
    let mut r = head(133);
    r.extend(5_000_000u64.to_le_bytes());
    r.push(0);
    for x in [100i16, -200, 300] {
        r.extend(x.to_le_bytes());
    }
    file.extend(r);
    // GPS: made-up numbers only
    let mut r = head(134);
    r.extend(6_000_000u64.to_le_bytes());
    r.push(0);
    r.push(3);
    r.extend(12_345u32.to_le_bytes());
    r.extend(2400u16.to_le_bytes());
    r.push(9);
    r.extend(120i16.to_le_bytes());
    r.extend(100_000_000i32.to_le_bytes());
    r.extend((-200_000_000i32).to_le_bytes());
    r.extend(12_345i32.to_le_bytes());
    r.extend(2.5f32.to_le_bytes());
    r.extend(90.0f32.to_le_bytes());
    file.extend(r);
    // BAT
    let mut r = head(135);
    r.extend(7_000_000u64.to_le_bytes());
    r.push(0);
    r.extend(11.5f32.to_le_bytes());
    r.extend(0.75f32.to_le_bytes());
    file.extend(r);
    // RCIN
    let mut r = head(136);
    r.extend(8_000_000u64.to_le_bytes());
    for c in 0..14u16 {
        r.extend((1000 + c * 10).to_le_bytes());
    }
    file.extend(r);
    // DEV
    let mut r = head(137);
    r.extend(9_000_000u64.to_le_bytes());
    r.extend([5u8, 3, 2]);
    for x in [1000u32, 7, 8] {
        r.extend(x.to_le_bytes());
    }
    for x in [1u16, 2, 3] {
        r.extend(x.to_le_bytes());
    }
    file.extend(r);
    // DSF
    let mut r = head(138);
    r.extend(10_000_000u64.to_le_bytes());
    r.extend(4u32.to_le_bytes());
    r.extend(5u16.to_le_bytes());
    for x in [6u32, 7, 8, 9] {
        r.extend(x.to_le_bytes());
    }
    file.extend(r);

    let mut reader = Reader::new(Cursor::new(file), None);
    for (id, name, format, columns) in table {
        let r = reader.next_record().unwrap().unwrap();
        assert_eq!(r.type_name(), "FMT");
        assert_eq!(r.get("Type"), Some(Value::Number(f64::from(id))));
        assert_eq!(r.get("Name"), Some(Value::Text(name.into())));
        assert_eq!(r.get("Format"), Some(Value::Text(format.into())));
        assert_eq!(r.get("Columns"), Some(Value::Text(columns.into())));
    }
    fn next<'a>(reader: &'a mut Reader<Cursor<Vec<u8>>>, name: &str) -> crate::Record<'a> {
        let r = reader.next_record().unwrap().unwrap();
        assert_eq!(r.type_name(), name);
        r
    }
    let r = next(&mut reader, "PARM");
    assert_eq!(r.time_us(), Some(1_000_000));
    assert_eq!(r.get("Name"), Some(Value::Text("SYSID_THISMAV".into())));
    assert_eq!(r.number_of("Value"), Some(1.0));
    assert_eq!(r.number_of("Default"), Some(0.5));
    let r = next(&mut reader, "MSG");
    assert_eq!(r.time_us(), Some(2_000_000));
    assert_eq!(r.get("Message"), Some(Value::Text("made-up text".into())));
    let r = next(&mut reader, "IMU");
    assert_eq!(r.number_of("I"), Some(1.0));
    let vals: Vec<_> = ["GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ", "T"]
        .iter()
        .map(|c| r.number_of(c).unwrap())
        .collect();
    assert_eq!(vals, [0.25, -0.5, 1.0, 0.0, 0.0, 9.5, 21.5]);
    let r = next(&mut reader, "BARO");
    assert_eq!(r.number_of("Press"), Some(101_325.0));
    near(r.number_of("Temp").unwrap(), 21.5);
    let r = next(&mut reader, "MAG");
    assert_eq!(
        [
            r.number_of("MagX"),
            r.number_of("MagY"),
            r.number_of("MagZ")
        ],
        [Some(100.0), Some(-200.0), Some(300.0)]
    );
    let r = next(&mut reader, "GPS");
    assert_eq!(r.time_us(), Some(6_000_000));
    assert_eq!(r.number_of("Status"), Some(3.0));
    assert_eq!(r.number_of("GMS"), Some(12_345.0));
    assert_eq!(r.number_of("GWk"), Some(2400.0));
    assert_eq!(r.number_of("NSats"), Some(9.0));
    near(r.number_of("HDop").unwrap(), 1.2);
    near(r.number_of("Lat").unwrap(), 10.0);
    near(r.number_of("Lng").unwrap(), -20.0);
    near(r.number_of("Alt").unwrap(), 123.45);
    assert_eq!(r.number_of("Spd"), Some(2.5));
    assert_eq!(r.number_of("GCrs"), Some(90.0));
    let r = next(&mut reader, "BAT");
    assert_eq!(r.number_of("Volt"), Some(11.5));
    assert_eq!(r.number_of("Curr"), Some(0.75));
    let r = next(&mut reader, "RCIN");
    for c in 0..14 {
        assert_eq!(
            r.number_of(&format!("C{}", c + 1)),
            Some(f64::from(1000 + c * 10))
        );
    }
    let r = next(&mut reader, "DEV");
    assert_eq!(
        [
            "Id", "State", "Err", "Smp", "Errs", "TOs", "Rec", "Att", "Ovf"
        ]
        .map(|c| r.number_of(c).unwrap()),
        [5.0, 3.0, 2.0, 1000.0, 7.0, 8.0, 1.0, 2.0, 3.0]
    );
    let r = next(&mut reader, "DSF");
    assert_eq!(
        ["Dp", "Blk", "Bytes", "FMn", "FMx", "FAv"].map(|c| r.number_of(c).unwrap()),
        [4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
    );
    assert!(reader.next_record().unwrap().is_none());
    assert_eq!((reader.bad_bytes(), reader.tail_bytes()), (0, 0));
}

/// Real log from the path in `MARV_BIN`; prints counts only, never a field value.
/// `MARV_BIN=/path/file.BIN cargo test -p gcs-dflog --release -- --ignored --nocapture`
#[test]
#[ignore = "needs a real log: set MARV_BIN"]
fn real_log_counts() {
    let path = std::env::var("MARV_BIN").expect("set MARV_BIN to a .BIN file");
    let s = summary(&path).unwrap();
    println!("bytes {}", s.bytes);
    println!("records {}", s.records);
    for c in &s.counts {
        println!("  {:>5} {:<5} {}", c.type_id, c.name, c.count);
    }
    println!("duration_us {:?}", s.duration_us());
    println!("bad_bytes {}", s.bad_bytes);
    println!("tail_bytes {}", s.tail_bytes);
    println!("has_wall_clock {}", s.time_base_unix_us.is_some());
}

/// Time one summary pass over a synthetic file of about 100 MB. Reported, not asserted.
/// `MARV_PERF=dir cargo test -p gcs-dflog --release -- --ignored --nocapture synthetic_100mb`
#[test]
#[ignore = "timing only: set MARV_PERF to a directory"]
fn synthetic_100mb() {
    use std::io::Write;
    let dir =
        std::path::PathBuf::from(std::env::var("MARV_PERF").expect("set MARV_PERF to a directory"));
    let path = dir.join("synthetic-100mb.BIN");
    let imu = "QBfffffff";
    let mut head = fmt_record(131, "IMU", imu, "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,T");
    head.extend(fmt_record(
        137,
        "DEV",
        "QBBBIIIHHH",
        "TimeUS,Id,State,Err,Smp,Errs,TOs,Rec,Att,Ovf",
    ));
    let mut out = io::BufWriter::new(std::fs::File::create(&path).unwrap());
    out.write_all(&head).unwrap();
    let mut written = head.len() as u64;
    let mut i = 0i64;
    while written < 100_000_000 {
        let f = V::Float(0.5);
        let r = record(
            131,
            imu,
            &[
                int(i * 250),
                int(i % 2),
                f.clone(),
                f.clone(),
                f.clone(),
                f.clone(),
                f.clone(),
                f.clone(),
                f,
            ],
        );
        out.write_all(&r).unwrap();
        written += r.len() as u64;
        i += 1;
    }
    out.flush().unwrap();
    drop(out);
    let started = std::time::Instant::now();
    let s = summary(&path).unwrap();
    let took = started.elapsed();
    std::fs::remove_file(&path).unwrap();
    println!(
        "file {} bytes, {} records, summary pass {:?}",
        s.bytes, s.records, took
    );
    assert_eq!(s.records, i as u64 + 2);
}
