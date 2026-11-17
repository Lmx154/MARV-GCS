//! The Log tab's section "Review a log file", drawn by egui_kittest over synthetic `.BIN` files written to a
//! temporary folder (no real log, no real position anywhere). Waiting for a background pass is a bounded loop.

use std::cell::RefCell;
use std::fs;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use eframe::egui::accesskit::Role;
use eframe::egui::{Key, Modifiers};
use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::services::logs::{DownloadState, DownloadStatus, ListStatus, LogInfo, LogList};
use gcs_mavlink::{Access, Command, VehicleState};
use marv_gcs::logfile;
use marv_gcs::views::{self, ViewCtx};

type Commands = Rc<RefCell<Vec<Command>>>;

// ---- synthetic logs ------------------------------------------------------------------------------------------

fn fmt(type_id: u8, name: &str, format: &str, columns: &str, length: u8) -> Vec<u8> {
    let pad = |text: &str, size: usize| {
        let mut v = text.as_bytes().to_vec();
        v.resize(size, 0);
        v
    };
    let mut r = vec![0xA3, 0x95, 128, type_id, length];
    r.extend(pad(name, 4));
    r.extend(pad(format, 16));
    r.extend(pad(columns, 64));
    assert_eq!(r.len(), 89);
    r
}

fn header() -> Vec<u8> {
    let mut f = fmt(130, "MSG", "QZ", "TimeUS,Message", 75);
    f.extend(fmt(
        137,
        "DEV",
        "QBBBIIIHHH",
        "TimeUS,Id,State,Err,Smp,Errs,TOs,Rec,Att,Ovf",
        32,
    ));
    f.extend(fmt(140, "BAT", "Qf", "TimeUS,Volt", 15));
    f
}

fn us(seconds: f64) -> u64 {
    (seconds * 1e6).round() as u64
}

fn msg(t: f64, text: &str) -> Vec<u8> {
    let mut r = vec![0xA3, 0x95, 130];
    r.extend(us(t).to_le_bytes());
    let mut s = text.as_bytes().to_vec();
    s.resize(64, 0);
    r.extend(s);
    r
}

fn dev(t: f64, id: u8, state: u8, err: u8) -> Vec<u8> {
    let mut r = vec![0xA3, 0x95, 137];
    r.extend(us(t).to_le_bytes());
    r.extend([id, state, err]);
    r.extend([0u8; 12]);
    r.extend([0u8; 6]);
    r
}

fn bat(t: f64, volt: f32) -> Vec<u8> {
    let mut r = vec![0xA3, 0x95, 140];
    r.extend(us(t).to_le_bytes());
    r.extend(volt.to_le_bytes());
    r
}

/// Three messages, two device records (IMU1 goes from OK to DEGRADED for a timeout), six battery readings of
/// which two stretches are below 14.8 V; 6.000 s from the first to the last `TimeUS`.
fn small_log() -> Vec<u8> {
    let mut b = header();
    b.extend(dev(0.5, 1, 2, 0));
    b.extend(msg(1.1, "alpha ready"));
    b.extend(bat(1.5, 16.0));
    b.extend(msg(2.1, "bravo low"));
    b.extend(bat(2.25, 14.0));
    b.extend(dev(2.5, 1, 3, 2));
    b.extend(msg(3.1, "charlie done"));
    b.extend(bat(3.25, 14.0));
    b.extend(bat(4.25, 16.0));
    b.extend(bat(5.25, 13.0));
    b.extend(bat(6.5, 16.0));
    b
}

/// The small log with two bad bytes before it and an unfinished record after it.
fn damaged_log() -> Vec<u8> {
    let mut b = vec![0x01, 0x02];
    b.extend(small_log());
    b.extend([0xA3, 0x95, 140, 1, 2]);
    b
}

/// The small log and then a long run of battery readings, so that a pass over it is not instantaneous.
fn big_log() -> Vec<u8> {
    let mut b = small_log();
    for i in 0..2_000_000u32 {
        b.extend(bat(7.0 + f64::from(i) * 0.001, 12.0 + (i % 7) as f32));
    }
    b
}

// ---- the rig -------------------------------------------------------------------------------------------------

static COUNTER: AtomicUsize = AtomicUsize::new(0);

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "marv_review_{tag}_{}_{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    ));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir
}

struct Rig {
    h: Harness<'static>,
    commands: Commands,
    dir: PathBuf,
}

impl Drop for Rig {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.dir);
    }
}

fn rig(tag: &str) -> Rig {
    let dir = temp_dir(tag);
    let commands: Commands = Rc::default();
    let sink = commands.clone();
    let state = VehicleState::default();
    let mut view = views::all()
        .into_iter()
        .find(|v| v.name() == "Log")
        .expect("Log view is registered");
    let mut h = Harness::builder()
        .with_size(eframe::egui::vec2(1100.0, 4000.0))
        .build_ui(move |ui| {
            let mut out = sink.borrow_mut();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut out,
                now: None,
                access: Access::Operate,
            };
            view.ui(ui, &mut cx);
        });
    h.run();
    Rig { h, commands, dir }
}

/// Steps the harness until `done`, at most about twenty seconds.
fn wait(h: &mut Harness<'_>, what: &str, done: impl Fn(&Harness<'_>) -> bool) {
    for _ in 0..4000 {
        h.step();
        if done(h) {
            return;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("gave up waiting for {what}");
}

fn present(h: &Harness<'_>, text: &str) -> bool {
    h.query_by_label_contains(text).is_some()
}

fn type_into(h: &mut Harness<'_>, name: &str, text: &str) {
    h.get_by_label(name).focus();
    h.run();
    h.key_down_modifiers(Modifiers::COMMAND, Key::A);
    h.run();
    h.get_by_label(name).type_text(text);
    h.run();
}

fn checkbox(h: &Harness<'_>, label: &str) {
    h.get_by_role_and_label(Role::CheckBox, label).click();
}

impl Rig {
    fn write(&self, name: &str, bytes: &[u8]) -> PathBuf {
        let path = self.dir.join(name);
        fs::write(&path, bytes).unwrap();
        path
    }

    /// Types the path, presses Open and waits until the summary shows.
    fn open(&mut self, path: &Path) {
        type_into(&mut self.h, "review file", path.to_str().unwrap());
        self.h.get_by_label("Open").click();
        self.h.run();
        wait(&mut self.h, "the summary", |h| present(h, "File: "));
    }

    fn find(&mut self, condition: &str) {
        type_into(&mut self.h, "find condition", condition);
        self.h.get_by_label("Find").click();
        self.h.run();
    }

    fn no_commands(&self) {
        assert!(
            self.commands.borrow().is_empty(),
            "the section sent a command"
        );
    }
}

// ---- tests ---------------------------------------------------------------------------------------------------

#[test]
fn opening_a_file_shows_the_summary_and_the_events_and_a_source_box_hides_its_events() {
    let mut r = rig("open");
    let path = r.write("flight.BIN", &damaged_log());
    r.open(&path);
    let h = &mut r.h;
    h.get_by_label("File: flight.BIN");
    h.get_by_label_contains(&format!("Size: {} bytes", damaged_log().len()));
    h.get_by_label("Duration: 6.000 s");
    h.get_by_label_contains("Wall-clock start: not known");
    h.get_by_label("Bad bytes: 2");
    h.get_by_label("Tail bytes: 5");
    for counts in ["FMT: 3", "MSG: 3", "DEV: 2", "BAT: 6"] {
        h.get_by_label(counts);
    }
    h.get_by_label("5 of 5 events");
    for text in [
        "alpha ready",
        "bravo low",
        "charlie done",
        "IMU1: OK -> DEGRADED (TIMEOUT)",
    ] {
        h.get_by_label_contains(text);
    }
    h.get_by_label("0.500");
    h.get_by_label("initial");
    h.get_by_label_contains("IMU1 OK");

    checkbox(h, "MSG");
    h.run();
    assert!(!present(h, "alpha ready"));
    assert!(!present(h, "bravo low"));
    h.get_by_label_contains("IMU1: OK -> DEGRADED (TIMEOUT)");
    h.get_by_label("2 of 5 events");

    checkbox(h, "MSG");
    h.run();
    h.get_by_label_contains("alpha ready");
    h.get_by_label("5 of 5 events");
    checkbox(h, "DEV");
    h.run();
    assert!(!present(h, "IMU1"));
    h.get_by_label_contains("charlie done");
    r.no_commands();
}

#[test]
fn a_bad_condition_shows_its_position_and_reason_and_starts_no_pass() {
    let mut r = rig("bad");
    let path = r.write("flight.BIN", &small_log());
    r.open(&path);
    r.find("BAT.Nope < 3");
    let shown = |h: &Harness<'_>| -> String {
        h.get_by_label_contains("Condition error at character ")
            .accesskit_node()
            .value()
            .unwrap_or_default()
    };
    let error = shown(&r.h);
    assert!(
        error.contains("at character 4") && error.contains("Nope"),
        "{error}"
    );
    r.find("BAT.Volt <");
    let error = shown(&r.h);
    assert!(error.contains("at character 10"), "{error}");
    // A pass over this tiny file would have ended and shown its result well within this time.
    for _ in 0..40 {
        r.h.step();
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(r.h.query_by_label("Cancel").is_none());
    assert!(!present(&r.h, "interval(s)"));
    assert!(!present(&r.h, "never held"));
    assert!(!present(&r.h, "Find cancelled"));

    type_into(&mut r.h, "export condition", "DEV[9].Nope > 1");
    checkbox(&r.h, "BAT (6 records)");
    r.h.get_by_label("Export CSV").click();
    r.h.run();
    r.h.get_by_label("Condition error at character 7: 'DEV' has no field 'Nope'");
    assert!(r.h.query_by_label("Cancel").is_none());
    assert!(!r.dir.join("flight_BAT.csv").exists());
    r.no_commands();
}

#[test]
fn find_shows_the_intervals_with_the_extremes_and_an_open_end() {
    let mut r = rig("find");
    let path = r.write("flight.BIN", &small_log());
    r.open(&path);
    r.find("BAT.Volt < 14.8");
    wait(&mut r.h, "the intervals", |h| present(h, "interval(s)"));
    let h = &mut r.h;
    h.get_by_label("2 interval(s)");
    h.get_by_label("BAT.Volt min");
    h.get_by_label("BAT.Volt max");
    for (text, rows) in [
        ("2.250", 1),
        ("4.250", 1),
        ("2.000", 1),
        ("5.250", 1),
        ("6.500", 1),
        ("1.250", 1),
        ("14", 2),
        ("13", 2),
    ] {
        assert_eq!(h.query_all_by_label(text).count(), rows, "{text}");
    }
    assert!(!present(h, "more not kept"));
    assert_eq!(h.query_all_by_label("open").count(), 0);

    r.find("BAT.Volt > 0");
    wait(&mut r.h, "the open interval", |h| {
        present(h, "1 interval(s)")
    });
    let h = &mut r.h;
    h.get_by_label("open");
    h.get_by_label("1.500");
    h.get_by_label("5.000");
    h.get_by_label("16");
    assert!(h.query_by_label("2 interval(s)").is_none());

    r.find("BAT.Volt > 100");
    wait(&mut r.h, "the empty result", |h| present(h, "never held"));
    r.no_commands();
}

#[test]
fn export_writes_the_chosen_types_and_a_second_export_names_the_file_it_will_not_overwrite() {
    let mut r = rig("export");
    let path = r.write("flight.BIN", &small_log());
    r.open(&path);
    // The folder starts as the folder of the opened file.
    checkbox(&r.h, "MSG (3 records)");
    checkbox(&r.h, "BAT (6 records)");
    r.h.run();
    r.h.get_by_label("Export CSV").click();
    r.h.run();
    wait(&mut r.h, "the export", |h| present(h, "Export finished"));
    let h = &mut r.h;
    h.get_by_label_contains("MSG: 3 rows written to");
    h.get_by_label_contains("BAT: 6 rows written to");
    assert!(
        !present(h, "DEV: 2 rows"),
        "an unchosen type is not exported"
    );
    let msg_csv = fs::read_to_string(r.dir.join("flight_MSG.csv")).unwrap();
    assert!(
        msg_csv.starts_with("TimeUS,Message\n1100000,alpha ready\n"),
        "{msg_csv}"
    );
    let bat_csv = fs::read_to_string(r.dir.join("flight_BAT.csv")).unwrap();
    assert_eq!(bat_csv.lines().count(), 7);
    assert!(
        bat_csv.starts_with("TimeUS,Volt\n1500000,16\n"),
        "{bat_csv}"
    );
    assert!(!r.dir.join("flight_DEV.csv").exists());

    r.h.get_by_label("Export CSV").click();
    r.h.run();
    wait(&mut r.h, "the refusal", |h| present(h, "Export failed"));
    let failed =
        r.h.get_by_label_contains("Export failed")
            .accesskit_node()
            .value()
            .unwrap_or_default();
    assert!(
        failed.contains("flight_MSG.csv exists and is not overwritten"),
        "{failed}"
    );
    assert_eq!(
        fs::read_to_string(r.dir.join("flight_MSG.csv")).unwrap(),
        msg_csv
    );
    r.no_commands();
}

#[test]
fn export_with_a_rate_and_a_condition_counts_what_it_left_out() {
    let mut r = rig("rate");
    let path = r.write("flight.BIN", &small_log());
    r.open(&path);
    let out = r.dir.join("out");
    type_into(&mut r.h, "export folder", out.to_str().unwrap());
    checkbox(&r.h, "BAT (6 records)");
    type_into(&mut r.h, "export rate", "0.5");
    type_into(&mut r.h, "export condition", "BAT.Volt < 14.8");
    r.h.get_by_label("Export CSV").click();
    r.h.run();
    wait(&mut r.h, "the export", |h| present(h, "Export finished"));
    r.h.get_by_label_contains(
        "BAT: 2 rows written, 1 skipped by the rate, 3 skipped by the condition to",
    );
    let csv = fs::read_to_string(out.join("flight_BAT.csv")).unwrap();
    assert_eq!(csv, "TimeUS,Volt\n2250000,14\n5250000,13\n");

    // Nothing chosen, and a rate that is not a rate: said, and no pass.
    checkbox(&r.h, "BAT (6 records)");
    r.h.get_by_label("Export CSV").click();
    r.h.run();
    r.h.get_by_label("Choose at least one record type to export");
    checkbox(&r.h, "BAT (6 records)");
    type_into(&mut r.h, "export rate", "fast");
    r.h.get_by_label("Export CSV").click();
    r.h.run();
    r.h.get_by_label_contains("The rate must be a number of Hz above 0");
    assert!(r.h.query_by_label("Cancel").is_none());
    r.no_commands();
}

#[test]
fn cancel_during_a_pass_leaves_the_section_usable() {
    let mut r = rig("cancel");
    let big = r.write("big.BIN", &big_log());
    let small = r.write("small.BIN", &small_log());

    // Cancel while the file is being opened.
    type_into(&mut r.h, "review file", big.to_str().unwrap());
    r.h.get_by_label("Open").click();
    r.h.run();
    r.h.get_by_label_contains("pass 1 of 2");
    r.h.get_by_label("Cancel").click();
    wait(&mut r.h, "the cancel", |h| present(h, "Open cancelled"));
    r.h.run();
    assert!(!present(&r.h, "File: "));
    assert!(r.h.query_by_label("Cancel").is_none());

    // Another file opens at once.
    r.open(&small);
    r.h.get_by_label("File: small.BIN");

    // Opening a file while another is being read cancels the first pass: only the last file shows.
    type_into(&mut r.h, "review file", big.to_str().unwrap());
    r.h.get_by_label("Open").click();
    r.h.run();
    r.h.get_by_label_contains("pass 1 of 2");
    type_into(&mut r.h, "review file", small.to_str().unwrap());
    r.h.get_by_label("Open").click();
    r.h.run();
    wait(&mut r.h, "the second file", |h| {
        present(h, "File: small.BIN")
    });
    assert!(!present(&r.h, "File: big.BIN"));

    // Cancel a search and an export of the big file.
    r.open(&big);
    r.find("BAT.Volt < 14.8");
    r.h.get_by_label_contains("searching");
    r.h.get_by_label("Cancel").click();
    wait(&mut r.h, "the cancelled search", |h| {
        present(h, "Find cancelled")
    });
    r.h.run();
    assert!(r.h.query_by_label("Cancel").is_none());

    checkbox(&r.h, "BAT (2000006 records)");
    let out = r.dir.join("out");
    type_into(&mut r.h, "export folder", out.to_str().unwrap());
    r.h.get_by_label("Export CSV").click();
    r.h.run();
    r.h.get_by_label_contains("exporting");
    r.h.get_by_label("Cancel").click();
    wait(&mut r.h, "the cancelled export", |h| {
        present(h, "Export cancelled")
    });
    r.h.run();
    assert!(r.h.query_by_label("Cancel").is_none());

    // Still usable: a bad condition is said, a source box works, a new search runs to its end.
    r.find("BAT.Nope < 1");
    r.h.get_by_label_contains("Condition error at character ");
    checkbox(&r.h, "MSG");
    r.h.run();
    assert!(!present(&r.h, "alpha ready"));
    checkbox(&r.h, "MSG");
    r.h.run();
    r.h.get_by_label_contains("alpha ready");
    r.find("BAT.Volt > 1000");
    wait(&mut r.h, "the search to its end", |h| {
        present(h, "never held")
    });
    r.no_commands();
}

#[test]
fn the_section_sends_no_command_and_needs_no_session() {
    let mut r = rig("nocmd");
    let path = r.write("flight.BIN", &small_log());
    r.open(&path);
    r.find("BAT.Volt < 14.8");
    wait(&mut r.h, "the intervals", |h| present(h, "interval(s)"));
    checkbox(&r.h, "DEV (2 records)");
    r.h.get_by_label("Export CSV").click();
    r.h.run();
    wait(&mut r.h, "the export", |h| present(h, "Export finished"));
    r.no_commands();
    // Not a log at all.
    let text = r.write("notes.BIN", b"this is not a DataFlash log, only some text");
    r.open(&text);
    r.h.get_by_label_contains("No record found");
    // A file that is not there.
    let missing = r.dir.join("missing.BIN");
    type_into(&mut r.h, "review file", missing.to_str().unwrap());
    r.h.get_by_label("Open").click();
    r.h.run();
    wait(&mut r.h, "the error", |h| present(h, "cannot read"));
    r.no_commands();
}

#[test]
fn a_finished_download_offers_review_and_the_file_opens() {
    let dir = temp_dir("download");
    let commands: Commands = Rc::default();
    let sink = commands.clone();
    let mut state = VehicleState::default();
    state.logs.list = ListStatus::Done(LogList {
        logs: vec![LogInfo {
            number: 4,
            size: 2_097_152,
        }],
        last_log_num: 4,
    });
    state.logs.download = Some(DownloadState {
        number: 4,
        size: 2_000,
        done: 2_000,
        status: DownloadStatus::Done,
        requests: 1,
        started: Duration::from_secs(8),
        ended: Some(Duration::from_secs(9)),
    });
    let mut view = views::all()
        .into_iter()
        .find(|v| v.name() == "Log")
        .unwrap();
    let folder = dir.clone();
    let mut h = Harness::builder()
        .with_size(eframe::egui::vec2(1100.0, 4000.0))
        .build_ui(move |ui| {
            logfile::set_log_dir(ui.ctx(), &folder);
            let mut out = sink.borrow_mut();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut out,
                now: Some(Duration::from_secs(10)),
                access: Access::Configure,
            };
            view.ui(ui, &mut cx);
        });
    h.run();
    // Before a Download of that log there is no known target file, so no Review button.
    assert!(h.query_by_label("Review").is_none());
    h.get_by_label("Download").click();
    h.run();
    // The download wrote to the folder; the log's bytes arrive in it (here: the test writes them).
    let file = dir.join("00000004.BIN");
    assert!(file.is_file());
    fs::write(&file, small_log()).unwrap();
    h.get_by_label("Review").click();
    h.run();
    wait(&mut h, "the summary", |h| present(h, "File: 00000004.BIN"));
    h.get_by_label("MSG: 3");
    let _ = fs::remove_dir_all(&dir);
}
