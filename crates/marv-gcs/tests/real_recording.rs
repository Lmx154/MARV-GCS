//! The headless check against a recording of the real vehicle: `MARV_TLOG=<file.tlog> cargo test -p
//! marv-gcs --test real_recording -- --ignored --nocapture`. The recording is replayed through the
//! session, and every Sensors preset and the Radio tab's sources must find their message and fields in
//! it. RC_CHANNELS may be absent (transmitter off): it is reported, not failed. Prints message names and
//! counts only, never values.

use std::fs::File;
use std::time::{Duration, Instant};

use gcs_mavlink::tlog::{ReplaySpeed, replay};
use gcs_mavlink::{Access, LinkStatus, Session, SessionConfig};
use marv_gcs::views::radio::Sticks;
use marv_gcs::views::sensors::{battery_text, preset_sample_counts};

#[test]
#[ignore = "needs MARV_TLOG, a recording of the real vehicle"]
fn the_presets_and_the_radio_find_their_data_in_a_recording_of_the_real_vehicle() {
    let path = std::env::var("MARV_TLOG").expect("MARV_TLOG is the path of the recording");
    let (reader, writer) = replay(
        Box::new(File::open(&path).expect("open the recording")),
        ReplaySpeed::Fast,
    );
    let session = Session::start(
        reader,
        writer,
        SessionConfig {
            access: Access::Operate,
            ..SessionConfig::default()
        },
        Box::new(|| {}),
    );
    let end = Instant::now() + Duration::from_secs(60);
    while !matches!(session.state().link.status, LinkStatus::Closed(_)) {
        assert!(Instant::now() < end, "the replay did not end in 60 s");
        std::thread::sleep(Duration::from_millis(50));
    }
    let state = session.state().clone();
    let store = &state.messages;

    let mut counts: Vec<(String, u64)> = Vec::new();
    for entry in store.entries.values() {
        match counts.iter_mut().find(|(name, _)| *name == entry.name) {
            Some((_, n)) => *n += entry.count,
            None => counts.push((entry.name.clone(), entry.count)),
        }
    }
    println!("messages in the recording (name, count):");
    for (name, n) in &counts {
        println!("  {name}\t{n}");
    }

    let mut missing = Vec::new();
    for (title, n) in preset_sample_counts(store) {
        println!("preset\t{title}\tsamples {n}");
        if n == 0 {
            missing.push(title);
        }
    }
    println!(
        "{}",
        battery_text(store).replace(|c: char| c.is_ascii_digit(), "#")
    );
    assert!(
        store.entry("GPS_RAW_INT", 0).is_some(),
        "the GPS line has no GPS_RAW_INT"
    );
    println!("GPS_RAW_INT\tpresent");

    for device in [7u8, 9] {
        let present = store.entry("MARV_DEVICE_STATUS", device).is_some();
        println!(
            "MARV_DEVICE_STATUS id {device}\t{}",
            if present { "present" } else { "absent" }
        );
        assert!(present, "no MARV_DEVICE_STATUS row {device}");
    }
    match store.entry("RC_CHANNELS", 0) {
        Some(entry) => {
            let sticks = Sticks::of(store, None);
            let used = sticks.values.iter().filter(|v| v.is_some()).count();
            println!(
                "RC_CHANNELS\tpresent, count {}, chancount {}, {used} channels with a value",
                entry.count, sticks.count
            );
            assert!(used > 0);
        }
        None => println!("RC_CHANNELS\tabsent in this recording"),
    }
    println!(
        "RADIO_STATUS\t{}",
        if store.entry("RADIO_STATUS", 0).is_some() {
            "present"
        } else {
            "absent in this recording"
        }
    );
    assert!(missing.is_empty(), "presets with no samples: {missing:?}");
}
