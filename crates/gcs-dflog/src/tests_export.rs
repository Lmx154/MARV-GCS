//! The CSV export, on synthetic logs, into a temporary folder of its own per test.

use std::fs;
use std::io::{self, Cursor};
use std::path::{Path, PathBuf};

use crate::tests::{V, fmt_record, int, record};
use crate::{
    Condition, Control, Export, ExportError, ExportRequest, Reader, export_with, summary_of,
};

const MSG: &str = "QZ";
const XYZ: &str = "QHcf";
const ARR: &str = "Qa";
const IMU: &str = "QBf";
const BAT: &str = "QBf";
const TV: &str = "Qf";
const NT: &str = "ff";

fn header() -> Vec<u8> {
    let mut f = Vec::new();
    f.extend(fmt_record(130, "MSG", MSG, "TimeUS,Message"));
    f.extend(fmt_record(140, "XYZ", XYZ, "TimeUS,Cnt,Sc,Fl"));
    f.extend(fmt_record(141, "ARR", ARR, "TimeUS,Arr"));
    f.extend(fmt_record(142, "IMU", IMU, "TimeUS,I,Acc"));
    f.extend(fmt_record(143, "BAT", BAT, "TimeUS,Inst,Volt"));
    f.extend(fmt_record(144, "TV", TV, "TimeUS,V"));
    f.extend(fmt_record(145, "NT", NT, "A,B"));
    f
}

fn msg(t: i64, text: &'static str) -> Vec<u8> {
    record(130, MSG, &[int(t), V::Text(text)])
}

fn imu(t: i64, inst: i64) -> Vec<u8> {
    record(142, IMU, &[int(t), int(inst), V::Float(1.5)])
}

fn bat(t: i64, volt: f64) -> Vec<u8> {
    record(143, BAT, &[int(t), int(0), V::Float(volt)])
}

fn tv(t: i64) -> Vec<u8> {
    record(144, TV, &[int(t), V::Float(0.0)])
}

fn nt(a: f64) -> Vec<u8> {
    record(145, NT, &[V::Float(a), V::Float(0.0)])
}

fn file_of(records: &[Vec<u8>]) -> Vec<u8> {
    let mut f = header();
    for r in records {
        f.extend(r);
    }
    f
}

/// A folder of this test that is removed when the test ends.
struct Temp(PathBuf);

impl Temp {
    fn new(test: &str) -> Temp {
        let path = std::env::temp_dir().join(format!(
            "gcs-dflog-export-test-{}-{test}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&path);
        Temp(path)
    }

    fn file(&self, name: &str) -> String {
        fs::read_to_string(self.0.join(name)).unwrap()
    }

    fn names(&self) -> Vec<String> {
        let mut v: Vec<String> = match fs::read_dir(&self.0) {
            Ok(d) => d
                .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
                .collect(),
            Err(_) => Vec::new(),
        };
        v.sort();
        v
    }
}

impl Drop for Temp {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn names(list: &[&str]) -> Vec<String> {
    list.iter().map(|s| (*s).to_owned()).collect()
}

fn run(
    file: &[u8],
    types: &[&str],
    rate_hz: Option<f64>,
    condition: Option<&str>,
    folder: &Path,
) -> Result<Export, ExportError> {
    let s = summary_of(Cursor::new(file.to_vec()), None).unwrap();
    let bound = condition.map(|c| Condition::parse(c).unwrap().bind(&s.formats).unwrap());
    let types = names(types);
    let request = ExportRequest {
        types: &types,
        rate_hz,
        condition: bound.as_ref(),
        folder,
        base: "log",
    };
    let mut reader = Reader::new(Cursor::new(file.to_vec()), None);
    export_with(&mut reader, &request, |_| Control::Continue)
}

fn counts(e: &Export) -> Vec<(&str, u64, u64, u64)> {
    e.types
        .iter()
        .map(|t| {
            (
                t.type_name.as_str(),
                t.written,
                t.skipped_by_rate,
                t.skipped_by_condition,
            )
        })
        .collect()
}

#[test]
fn exact_bytes_of_a_small_export() {
    let mut arr = [0i16; 32];
    arr[0] = -5;
    arr[1] = 7;
    arr[31] = 32767;
    let file = file_of(&[
        msg(1000, "a,\"b\""),
        record(140, XYZ, &[int(1000), int(7), int(250), V::Float(0.5)]),
        msg(2000, "ok"),
        record(
            140,
            XYZ,
            &[int(2000), int(65535), int(-100), V::Float(-0.25)],
        ),
        msg(3000, "x\ny"),
        record(141, ARR, &[int(1500), V::Array(arr)]),
        imu(1, 0),
    ]);
    let t = Temp::new("exact");
    let e = run(&file, &["MSG", "XYZ", "ARR", "NOPE"], None, None, &t.0).unwrap();
    assert!(!e.cancelled);
    assert_eq!(
        counts(&e),
        [
            ("MSG", 3, 0, 0),
            ("XYZ", 2, 0, 0),
            ("ARR", 1, 0, 0),
            ("NOPE", 0, 0, 0)
        ]
    );
    assert_eq!(e.types[0].path, Some(t.0.join("log_MSG.csv")));
    assert_eq!(e.types[3].path, None);
    assert_eq!(
        t.names(),
        ["log_ARR.csv", "log_MSG.csv", "log_XYZ.csv"],
        "the unchosen IMU and the absent NOPE have no file"
    );
    assert_eq!(
        t.file("log_MSG.csv"),
        "TimeUS,Message\n1000,\"a,\"\"b\"\"\"\n2000,ok\n3000,\"x\ny\"\n"
    );
    assert_eq!(
        t.file("log_XYZ.csv"),
        "TimeUS,Cnt,Sc,Fl\n1000,7,2.5,0.5\n2000,65535,-1,-0.25\n"
    );
    let mut items = vec!["0".to_owned(); 32];
    items[0] = "-5".to_owned();
    items[1] = "7".to_owned();
    items[31] = "32767".to_owned();
    assert_eq!(
        t.file("log_ARR.csv"),
        format!("TimeUS,Arr\n1500,\"{}\"\n", items.join(";"))
    );
}

#[test]
fn numbers_read_back_to_the_same_value() {
    let values = [0.1f32, 1.0 / 3.0, -2.5e-7, 123456.79, 3.0e10];
    let mut records = Vec::new();
    for (i, v) in values.iter().enumerate() {
        records.push(record(144, TV, &[int(i as i64), V::Float(f64::from(*v))]));
    }
    let t = Temp::new("roundtrip");
    run(&file_of(&records), &["TV"], None, None, &t.0).unwrap();
    let text = t.file("log_TV.csv");
    let back: Vec<f64> = text
        .lines()
        .skip(1)
        .map(|l| l.split(',').nth(1).unwrap().parse().unwrap())
        .collect();
    let want: Vec<f64> = values.iter().map(|&v| f64::from(v)).collect();
    assert_eq!(back, want);
}

#[test]
fn rate_limit_is_per_type_and_instance() {
    // 10 Hz: 100 000 us. Instance 0 and instance 1 are 30 000 us out of phase.
    let i0 = [0, 40_000, 100_000, 150_000, 200_000, 290_000, 300_000];
    let i1 = [30_000, 130_000, 160_000, 230_000, 235_000, 330_000];
    let mut all: Vec<(i64, i64)> = i0.iter().map(|&t| (t, 0)).collect();
    all.extend(i1.iter().map(|&t| (t, 1)));
    all.sort();
    let mut records: Vec<Vec<u8>> = all.iter().map(|&(t, i)| imu(t, i)).collect();
    // a type with no instance field: one instance; kept at 0, 100 000, 200 000 (149 999 is 49 999 after)
    for t in [0, 50_000, 100_000, 149_999, 200_000] {
        records.push(tv(t));
    }
    // a type without TimeUS is not limited
    for a in [1.0, 2.0, 3.0] {
        records.push(nt(a));
    }
    let file = file_of(&records);
    let t = Temp::new("rate");
    let e = run(&file, &["IMU", "TV", "NT"], Some(10.0), None, &t.0).unwrap();
    assert_eq!(
        counts(&e),
        [("IMU", 8, 5, 0), ("TV", 3, 2, 0), ("NT", 3, 0, 0)]
    );
    let imu_text = t.file("log_IMU.csv");
    let rows: Vec<(u64, u64)> = imu_text
        .lines()
        .skip(1)
        .map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            (f[0].parse().unwrap(), f[1].parse().unwrap())
        })
        .collect();
    assert_eq!(
        rows,
        [
            (0, 0),
            (30_000, 1),
            (100_000, 0),
            (130_000, 1),
            (200_000, 0),
            (230_000, 1),
            (300_000, 0),
            (330_000, 1)
        ]
    );
    assert_eq!(t.file("log_TV.csv"), "TimeUS,V\n0,0\n100000,0\n200000,0\n");
    assert_eq!(t.file("log_NT.csv"), "A,B\n1,0\n2,0\n3,0\n");

    // without a rate every record is written
    let t2 = Temp::new("rate-none");
    let e = run(&file, &["IMU"], None, None, &t2.0).unwrap();
    assert_eq!(counts(&e), [("IMU", 13, 0, 0)]);
}

#[test]
fn condition_writes_only_the_records_while_it_holds() {
    let file = file_of(&[
        msg(500, "m0"),  // the condition has not seen BAT: false
        bat(1000, 15.0), // false
        msg(1500, "m1"), // false
        bat(2000, 13.0), // makes it true: written
        msg(2500, "m2"), // true
        bat(3000, 12.0), // true
        bat(4000, 15.0), // makes it false: not written
        msg(4500, "m3"), // false
        bat(5000, 10.0), // true again
        msg(5500, "m4"), // true until the end
    ]);
    let t = Temp::new("condition");
    let e = run(&file, &["BAT", "MSG"], None, Some("BAT.Volt < 14"), &t.0).unwrap();
    assert_eq!(counts(&e), [("BAT", 3, 0, 2), ("MSG", 2, 0, 3)]);
    assert_eq!(
        t.file("log_BAT.csv"),
        "TimeUS,Inst,Volt\n2000,0,13\n3000,0,12\n5000,0,10\n"
    );
    assert_eq!(t.file("log_MSG.csv"), "TimeUS,Message\n2500,m2\n5500,m4\n");

    // the rate limit sees only what the condition let through: 1 Hz keeps the first of the three
    let t2 = Temp::new("condition-rate");
    let e = run(&file, &["BAT"], Some(1.0), Some("BAT.Volt < 14"), &t2.0).unwrap();
    assert_eq!(counts(&e), [("BAT", 1, 2, 2)]);
    assert_eq!(t2.file("log_BAT.csv"), "TimeUS,Inst,Volt\n2000,0,13\n");
}

#[test]
fn an_existing_target_file_is_refused_before_anything_is_written() {
    let file = file_of(&[msg(1, "a"), imu(1, 0)]);
    let t = Temp::new("exists");
    fs::create_dir_all(&t.0).unwrap();
    fs::write(t.0.join("log_IMU.csv"), "keep").unwrap();
    let err = run(&file, &["MSG", "IMU"], None, None, &t.0).unwrap_err();
    assert_eq!(err.error.kind(), io::ErrorKind::AlreadyExists);
    assert!(
        err.error.to_string().contains("log_IMU.csv"),
        "{}",
        err.error
    );
    assert!(err.files.is_empty());
    assert_eq!(t.names(), ["log_IMU.csv"], "no log_MSG.csv was made");
    assert_eq!(t.file("log_IMU.csv"), "keep");
}

#[test]
fn a_type_without_a_written_row_makes_no_file() {
    let file = file_of(&[msg(1, "a"), bat(2, 15.0), bat(3, 16.0), msg(4, "b")]);
    // none of the rows passes the condition; MSG is not in the condition and is chosen too
    let t = Temp::new("norows");
    let e = run(&file, &["BAT", "ABSENT"], None, Some("BAT.Volt < 0"), &t.0).unwrap();
    assert_eq!(counts(&e), [("BAT", 0, 0, 2), ("ABSENT", 0, 0, 0)]);
    assert!(e.types.iter().all(|t| t.path.is_none()));
    assert!(!t.0.exists(), "not even the folder is made");

    let t = Temp::new("norows-some");
    let e = run(&file, &["BAT", "MSG"], None, Some("BAT.Volt < 0"), &t.0).unwrap();
    assert_eq!(counts(&e), [("BAT", 0, 0, 2), ("MSG", 0, 0, 2)]);
    assert!(t.names().is_empty());
}

#[test]
fn cancel_leaves_the_files_written_so_far() {
    // 50 000 records of 16 bytes: 800 000 bytes, a progress report every 256 KiB
    let mut file = header();
    for i in 0..50_000 {
        file.extend(imu(i * 1000, 0));
    }
    let t = Temp::new("cancel");
    let s = summary_of(Cursor::new(file.clone()), None).unwrap();
    assert_eq!(s.records, 50_000 + 7);
    let types = names(&["IMU"]);
    let request = ExportRequest {
        types: &types,
        rate_hz: None,
        condition: None,
        folder: &t.0,
        base: "log",
    };
    let mut reader = Reader::new(Cursor::new(file), None);
    let mut reports = Vec::new();
    let e = export_with(&mut reader, &request, |p| {
        reports.push(p.bytes);
        Control::Stop
    })
    .unwrap();
    assert!(e.cancelled);
    assert_eq!(reports.len(), 1);
    let done = &e.types[0];
    assert!(
        done.written > 10_000 && done.written < 50_000,
        "{}",
        done.written
    );
    assert_eq!(done.path, Some(t.0.join("log_IMU.csv")));
    let text = t.file("log_IMU.csv");
    assert_eq!(text.lines().count() as u64, done.written + 1);
    assert!(text.ends_with('\n'), "flushed to the last whole row");
    assert_eq!(text.lines().next(), Some("TimeUS,I,Acc"));
}

#[test]
fn unusable_requests_and_failures_are_errors() {
    let file = file_of(&[msg(1, "a")]);
    let t = Temp::new("invalid");
    for rate in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let err = run(&file, &["MSG"], Some(rate), None, &t.0).unwrap_err();
        assert_eq!(err.error.kind(), io::ErrorKind::InvalidInput, "{rate}");
    }
    let err = run(&file, &["../MSG"], None, None, &t.0).unwrap_err();
    assert_eq!(err.error.kind(), io::ErrorKind::InvalidInput);
    assert!(!t.0.exists());

    // the folder is a file: the first file cannot be made, nothing is claimed as written
    fs::create_dir_all(&t.0).unwrap();
    let blocker = t.0.join("blocker");
    fs::write(&blocker, "x").unwrap();
    let err = run(&file, &["MSG"], None, None, &blocker).unwrap_err();
    assert!(err.files.is_empty());
    assert_eq!(t.names(), ["blocker"]);
}
