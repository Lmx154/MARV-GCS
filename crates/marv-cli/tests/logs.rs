//! `marv-cli logs list` and `logs get` against the simulated vehicle over a real TCP socket.
//! Every wait is bounded by the command's own deadlines.

mod common;

use std::time::Instant;

use common::{Rig, TempDir};
use gcs_sim::{LogDataDrop, SimConfig, log_bytes};

#[test]
fn list_prints_one_line_per_log_and_the_last_number() {
    let rig = Rig::start();
    let r = rig.cli("logs list");
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(
        r.out, "2\t12345\n3\t0\n5\t9000\n8\t60001\n11\t450\nlast\t11\n",
        "{r:?}"
    );
    assert!(!r.err.contains("no logs"), "{r:?}");
}

#[test]
fn list_of_an_empty_card_is_success_with_a_line_on_stderr() {
    let rig = Rig::with(SimConfig {
        logs: Vec::new(),
        ..SimConfig::default()
    });
    let r = rig.cli("logs list");
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(r.out, "", "{r:?}");
    assert!(r.err.contains("marv-cli: no logs on the vehicle"), "{r:?}");
}

#[test]
fn get_writes_the_log_byte_for_byte() {
    let rig = Rig::start();
    let dir = TempDir::new("get");
    let file = dir.file("two.bin");
    let r = rig.cli(&format!("logs get 2 -o {}", file.display()));
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(
        r.out,
        format!("downloaded\t{}\t12345 bytes\n", file.display()),
        "{r:?}"
    );
    assert_eq!(
        std::fs::read(&file).expect("the file"),
        log_bytes(2, 12_345)
    );
}

#[test]
fn get_of_a_log_of_several_windows_reports_progress_at_most_twice_a_second() {
    let rig = Rig::start();
    let dir = TempDir::new("get-windows");
    let file = dir.file("eight.bin");
    let started = Instant::now();
    let r = rig.cli(&format!("logs get 8 -o {}", file.display()));
    let seconds = started.elapsed().as_secs_f64();
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(
        std::fs::read(&file).expect("the file"),
        log_bytes(8, 60_001)
    );
    let lines: Vec<&str> = r.err.lines().filter(|l| l.starts_with("log 8: ")).collect();
    assert!(!lines.is_empty(), "{r:?}");
    assert!(
        lines.iter().all(|l| l.contains(" of 60001 bytes (")),
        "{lines:?}"
    );
    assert!(
        (lines.len() as f64 - 1.0) * 0.5 <= seconds,
        "{} lines in {seconds} s: {lines:?}",
        lines.len()
    );
}

#[test]
fn get_of_an_empty_log_makes_an_empty_file() {
    let rig = Rig::start();
    let dir = TempDir::new("get-empty");
    let file = dir.file("three.bin");
    let r = rig.cli(&format!("logs get 3 -o {}", file.display()));
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(std::fs::read(&file).expect("the file"), Vec::<u8>::new());
}

#[test]
fn get_of_a_log_that_is_not_listed_is_a_failure_and_makes_no_file() {
    let rig = Rig::start();
    let dir = TempDir::new("get-missing");
    let file = dir.file("four.bin");
    let r = rig.cli(&format!("logs get 4 -o {}", file.display()));
    assert_eq!(r.code, 1, "{r:?}");
    assert!(
        r.err
            .contains("log 4 is not on the vehicle (logs: 2 3 5 8 11)"),
        "{r:?}"
    );
    assert!(!file.exists());
}

#[test]
fn get_does_not_overwrite_a_file_unless_forced() {
    let rig = Rig::start();
    let dir = TempDir::new("get-force");
    let file = dir.file("five.bin");
    std::fs::write(&file, b"keep me").expect("write");
    let r = rig.cli(&format!("logs get 5 -o {}", file.display()));
    assert_eq!(r.code, 1, "{r:?}");
    assert!(r.err.contains("--force"), "{r:?}");
    assert_eq!(std::fs::read(&file).expect("the file"), b"keep me");
    let r = rig.cli(&format!("logs get 5 -o {} --force", file.display()));
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(std::fs::read(&file).expect("the file"), log_bytes(5, 9_000));
}

#[test]
fn get_without_a_file_name_writes_nnnnnnnn_bin_in_the_current_directory() {
    // Log 11 is used by no other test, so the name cannot collide with a parallel one.
    struct Remove(std::path::PathBuf);
    impl Drop for Remove {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }
    let name = std::path::PathBuf::from("00000011.BIN");
    let _remove = Remove(name.clone());
    let _ = std::fs::remove_file(&name);
    let rig = Rig::start();
    let r = rig.cli("logs get 11");
    assert_eq!(r.code, 0, "{r:?}");
    assert!(
        r.out.starts_with("downloaded\t00000011.BIN\t450 bytes"),
        "{r:?}"
    );
    assert_eq!(std::fs::read(&name).expect("the file"), log_bytes(11, 450));
}

#[test]
fn lost_frames_are_asked_for_again_and_the_file_is_still_identical() {
    // One frame lost in the middle of the first window, and the last frame of that window.
    let rig = Rig::with(SimConfig {
        drop_log_data: vec![
            LogDataDrop { ofs: 900, times: 1 },
            LogDataDrop {
                ofs: 45_990,
                times: 1,
            },
        ],
        ..SimConfig::default()
    });
    let dir = TempDir::new("get-drops");
    let file = dir.file("eight.bin");
    let r = rig.cli(&format!("logs get 8 -o {}", file.display()));
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(
        std::fs::read(&file).expect("the file"),
        log_bytes(8, 60_001)
    );
}

#[test]
fn a_vehicle_that_stops_sending_is_a_failure_with_the_partial_file_kept() {
    // Every frame from offset 900 on is lost, always.
    let rig = Rig::with(SimConfig {
        drop_log_data: vec![LogDataDrop {
            ofs: 900,
            times: u32::MAX,
        }],
        ..SimConfig::default()
    });
    let dir = TempDir::new("get-stalled");
    let file = dir.file("five.bin");
    let r = rig.cli(&format!("logs get 5 -o {}", file.display()));
    assert_eq!(r.code, 1, "{r:?}");
    assert!(
        r.err
            .contains("no data from the vehicle at byte 900 after 5 new requests"),
        "{r:?}"
    );
    assert!(r.err.contains("900 of 9000 bytes are in"), "{r:?}");
    assert_eq!(
        std::fs::read(&file).expect("the file"),
        log_bytes(5, 9_000)[..900]
    );
}
