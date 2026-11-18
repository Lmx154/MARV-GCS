//! `marv-cli session export` and `session list`: a recording made against the simulator, then
//! converted; no vehicle is needed for the conversion itself.

mod common;

use common::{Rig, TempDir, run};

#[test]
fn export_writes_the_folder_and_list_shows_it() {
    let rig = Rig::start();
    let dir = TempDir::new("session");
    let tlog = dir.file("run.tlog");
    let r = rig.cli(&format!("record {} --for 1", tlog.display()));
    assert_eq!(r.code, 0, "{r:?}");

    let out = dir.file("out");
    let r = run(&format!(
        "session export {} --name my_run --out {}",
        tlog.display(),
        out.display()
    ));
    assert_eq!(r.code, 0, "{r:?}");
    let lines = r.lines();
    let folder_line = lines[0].strip_prefix("session\t").expect("folder line");
    let folder = std::path::PathBuf::from(folder_line);
    assert!(folder_line.ends_with("Z_my_run"), "{r:?}");
    assert!(folder.starts_with(&out), "{r:?}");
    assert!(
        lines
            .iter()
            .any(|l| l.starts_with("HEARTBEAT.csv\t") && l.ends_with(" rows")),
        "{r:?}"
    );
    assert!(lines.last().expect("last").starts_with("marks\t"), "{r:?}");
    for file in ["session.tlog", "session.json", "HEARTBEAT.csv"] {
        assert!(folder.join(file).is_file(), "{file}");
    }
    assert_eq!(
        std::fs::read(folder.join("session.tlog")).expect("copy"),
        std::fs::read(&tlog).expect("input")
    );
    let json = std::fs::read_to_string(folder.join("session.json")).expect("json");
    assert!(json.contains("\"name\": \"my_run\""), "{json}");
    assert!(
        !json.contains("\"lat\"") && !json.contains("\"lon\""),
        "{json}"
    );

    let again = run(&format!(
        "session export {} --name my_run --out {}",
        tlog.display(),
        out.display()
    ));
    assert_eq!(again.code, 1, "{again:?}");
    assert!(again.err.contains("never overwritten"), "{again:?}");
    assert_eq!(again.out, "", "{again:?}");

    let list = run(&format!("session list {}", out.display()));
    assert_eq!(list.code, 0, "{list:?}");
    let fields: Vec<&str> = list.out.trim_end().split('\t').collect();
    assert_eq!(fields.len(), 5, "{list:?}");
    assert!(folder_line.ends_with(fields[0]), "{list:?}");
    assert_eq!(fields[1], "my_run");
    assert!(
        fields[2].ends_with('Z') && fields[3].ends_with(" s"),
        "{list:?}"
    );
    assert!(fields[4].ends_with(" marks"), "{list:?}");
}

#[test]
fn the_defaults_are_the_files_own_name_and_a_sessions_folder_next_to_it() {
    let rig = Rig::start();
    let dir = TempDir::new("session-defaults");
    let tlog = dir.file("calibration_1.tlog");
    assert_eq!(
        rig.cli(&format!("record {} --for 1", tlog.display())).code,
        0
    );
    let r = run(&format!("session export {}", tlog.display()));
    assert_eq!(r.code, 0, "{r:?}");
    let folder = r.lines()[0].strip_prefix("session\t").expect("folder");
    assert!(
        folder.starts_with(&format!("{}/sessions/", dir.0.display()))
            && folder.ends_with("Z_calibration_1"),
        "{r:?}"
    );
}

#[test]
fn a_missing_file_fails_and_an_empty_list_is_success() {
    let dir = TempDir::new("session-missing");
    let r = run(&format!(
        "session export {}",
        dir.file("none.tlog").display()
    ));
    assert_eq!(r.code, 1, "{r:?}");
    assert!(r.err.contains("session export of"), "{r:?}");
    assert!(!dir.file("sessions").join("anything").exists());
    let r = run(&format!("session list {}", dir.file("nothing").display()));
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(r.out, "", "{r:?}");
    assert!(r.err.contains("no sessions in"), "{r:?}");
}
