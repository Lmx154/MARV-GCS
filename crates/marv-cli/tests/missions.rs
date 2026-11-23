//! `marv-cli mission upload | download | clear` against the simulated vehicle over a real TCP socket, in real
//! time (docs/link-contract.md, section 17). The station's mission client and the simulator's mission service were
//! written apart from the same text; these tests are where they meet. The files are written here by hand, row
//! by row, and the plan ids are computed here by a third CRC-32.

mod common;

use std::time::{Duration, Instant};

use common::{Output, Rig, TempDir};
use gcs_sim::SimConfig;

/// Waypoints of the contract's example 1 and small steps from them: (x, y, z) as the wire carries them.
fn points(n: usize) -> Vec<(i32, i32, f32)> {
    let example = [
        (100_000_000, -300_000_000, 10.0),
        (100_010_000, -300_000_000, 12.5),
        (100_010_000, -299_990_000, 15.0),
    ];
    (0..n)
        .map(|i| {
            example.get(i).copied().unwrap_or_else(|| {
                let k = i as i32;
                (
                    100_010_000 + 10_000 * (k - 2),
                    -299_990_000 + 10_000 * (k % 3),
                    15.0 + 0.25 * k as f32,
                )
            })
        })
        .collect()
}

fn degrees(e7: i32) -> String {
    format!("{:.8}", f64::from(e7) / 1e7)
}

/// The text of a QGC WPL 110 file: the home placeholder, then one row per point.
fn wpl(points: &[(i32, i32, f32)]) -> String {
    let mut text = String::from("QGC WPL 110\n");
    let (x, y) = points.first().map_or((0, 0), |p| (p.0, p.1));
    text += &format!(
        "0\t1\t0\t16\t0\t0\t0\t0\t{}\t{}\t0\t1\n",
        degrees(x),
        degrees(y)
    );
    for (i, (x, y, z)) in points.iter().enumerate() {
        text += &format!(
            "{}\t0\t3\t16\t0\t0\t0\t0\t{}\t{}\t{z}\t1\n",
            i + 1,
            degrees(*x),
            degrees(*y)
        );
    }
    text
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = u32::MAX;
    for &b in bytes {
        crc ^= u32::from(b);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xEDB8_8320 & (crc & 1).wrapping_neg());
        }
    }
    !crc
}

/// The contract's plan id: the CRC-32 of x, y and the bits of z of each point, little-endian.
fn plan_id(points: &[(i32, i32, f32)]) -> u32 {
    let mut bytes = Vec::new();
    for (x, y, z) in points {
        bytes.extend_from_slice(&x.to_le_bytes());
        bytes.extend_from_slice(&y.to_le_bytes());
        bytes.extend_from_slice(&z.to_bits().to_le_bytes());
    }
    match crc32(&bytes) {
        0 if !points.is_empty() => 1,
        id => id,
    }
}

/// A session on the rig's vehicle that has seen the heartbeat.
fn connected(rig: &Rig) -> gcs_mavlink::Session {
    let endpoint = format!("tcp:{}", rig.addr).parse().expect("endpoint");
    let session =
        gcs_link::start_session(&endpoint, gcs_mavlink::Access::Configure, Box::new(|| {}))
            .expect("open");
    let deadline = Instant::now() + Duration::from_secs(10);
    while session.state().link.vehicle.is_none() {
        assert!(Instant::now() < deadline, "never connected");
        std::thread::sleep(Duration::from_millis(10));
    }
    session
}

/// Runs the command line on a session that exists already.
fn on(session: &gcs_mavlink::Session, words: &str) -> Output {
    let args: Vec<String> = format!("--connect tcp:127.0.0.1:5760 {words}")
        .split_whitespace()
        .map(str::to_string)
        .collect();
    let Ok(marv_cli::args::Parsed::Run(invocation)) = marv_cli::args::parse(&args) else {
        panic!("{words}: does not parse");
    };
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = marv_cli::commands::execute(
        &invocation,
        session,
        gcs_mavlink::Access::Configure,
        &mut out,
        &mut err,
    );
    Output {
        code,
        out: String::from_utf8_lossy(&out).into_owned(),
        err: String::from_utf8_lossy(&err).into_owned(),
    }
}

fn config(rx: u32, tx: u32) -> SimConfig {
    SimConfig {
        drop_mission_rx_every: rx,
        drop_mission_tx_every: tx,
        ..SimConfig::default().with_device_status(1.0)
    }
}

/// What a download prints: the points (as the file would hold them) and the plan id.
fn downloaded(r: &Output) -> (Vec<(i32, i32, f32)>, String) {
    let mut lines = r.lines();
    if lines.last().is_some_and(|l| l.starts_with("written\t")) {
        lines.pop();
    }
    let (summary, rows) = lines.split_last().expect("a summary line");
    let points = rows
        .iter()
        .enumerate()
        .map(|(seq, line)| {
            let cells: Vec<&str> = line.split('\t').collect();
            assert_eq!(cells.len(), 4, "{line}");
            assert_eq!(cells[0], seq.to_string(), "{line}");
            let e7 = |text: &str| (text.parse::<f64>().expect("degrees") * 1e7).round() as i32;
            (
                e7(cells[1]),
                e7(cells[2]),
                cells[3].parse::<f32>().expect("altitude"),
            )
        })
        .collect();
    (points, (*summary).to_string())
}

/// Uploads `n` points, reads them back by `mission download`, and compares with what was written. Both the
/// printed list and the written file must say the same.
fn round_trip(rig: &Rig, n: usize, tag: &str) {
    let want = points(n);
    let dir = TempDir::new(tag);
    let (up, down) = (dir.file("up.waypoints"), dir.file("down.waypoints"));
    std::fs::write(&up, wpl(&want)).expect("write the file");
    let session = connected(rig);
    let id = plan_id(&want);

    let r = on(&session, &format!("mission upload {}", up.display()));
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(
        r.out,
        format!("verified\t{n} waypoints\tplan id {id:#010X}\n"),
        "{r:?}"
    );

    let r = on(&session, &format!("mission download {}", down.display()));
    assert_eq!(r.code, 0, "{r:?}");
    let (got, summary) = downloaded(&r);
    assert_eq!(got.len(), n, "{r:?}");
    for (seq, (a, b)) in got.iter().zip(&want).enumerate() {
        assert_eq!(
            (a.0, a.1, a.2.to_bits()),
            (b.0, b.1, b.2.to_bits()),
            "seq {seq}"
        );
    }
    assert_eq!(
        summary,
        format!("{n} waypoints\tplan id {id:#010X}"),
        "{r:?}"
    );
    assert!(
        r.out.ends_with(&format!("written\t{}\n", down.display())),
        "{r:?}"
    );
    let written = std::fs::read_to_string(&down).expect("the file");
    let back = marv_cli::waypoints::parse(&written).expect("the map tab's format");
    let sent = marv_cli::waypoints::parse(&wpl(&want)).expect("parse");
    assert_eq!(back, sent);
    assert_eq!(written.lines().count(), n + 2, "header, home, {n} rows");
}

#[test]
fn the_contract_vector_three_points_up_and_down() {
    let rig = Rig::start();
    round_trip(&rig, 3, "m-three");
    assert_eq!(plan_id(&points(3)), 0x99C3_BD49);
}

#[test]
fn sixteen_points_up_and_down() {
    let rig = Rig::start();
    round_trip(&rig, 16, "m-sixteen");
}

#[test]
fn three_points_with_every_third_frame_to_the_vehicle_lost() {
    round_trip(&Rig::with(config(3, 0)), 3, "m-three-rx");
}

#[test]
fn three_points_with_every_fourth_frame_from_the_vehicle_lost() {
    round_trip(&Rig::with(config(0, 4)), 3, "m-three-tx");
}

#[test]
fn three_points_with_both_losses() {
    round_trip(&Rig::with(config(3, 4)), 3, "m-three-both");
}

#[test]
fn sixteen_points_with_every_third_frame_to_the_vehicle_lost() {
    round_trip(&Rig::with(config(3, 0)), 16, "m-sixteen-rx");
}

#[test]
fn sixteen_points_with_every_fourth_frame_from_the_vehicle_lost() {
    round_trip(&Rig::with(config(0, 4)), 16, "m-sixteen-tx");
}

#[test]
fn sixteen_points_with_both_losses() {
    round_trip(&Rig::with(config(3, 4)), 16, "m-sixteen-both");
}

#[test]
fn seventeen_points_are_refused_with_no_space_and_the_list_is_unchanged() {
    let rig = Rig::start();
    let dir = TempDir::new("m-seventeen");
    let (three, many) = (dir.file("three.waypoints"), dir.file("many.waypoints"));
    std::fs::write(&three, wpl(&points(3))).expect("write");
    std::fs::write(&many, wpl(&points(17))).expect("write");
    let session = connected(&rig);

    let r = on(&session, &format!("mission upload {}", three.display()));
    assert_eq!(r.code, 0, "{r:?}");
    let r = on(&session, &format!("mission upload {}", many.display()));
    assert_eq!(r.code, 1, "{r:?}");
    assert_eq!(r.out, "", "{r:?}");
    assert!(
        r.err.contains(
            "mission upload failed: refused by the vehicle: MAV_MISSION_NO_SPACE (4): the vehicle holds at most 16 waypoints"
        ),
        "{r:?}"
    );
    assert!(
        r.err
            .contains("the vehicle holds 3 waypoints, plan id 0x99C3BD49"),
        "the read-back is shown: {r:?}"
    );
    let r = on(&session, "mission download");
    assert_eq!(r.code, 0, "{r:?}");
    let (got, summary) = downloaded(&r);
    let want = points(3);
    assert_eq!(got.len(), 3, "{r:?}");
    for (a, b) in got.iter().zip(&want) {
        assert_eq!((a.0, a.1, a.2.to_bits()), (b.0, b.1, b.2.to_bits()));
    }
    assert_eq!(summary, "3 waypoints\tplan id 0x99C3BD49", "{r:?}");
}

#[test]
fn clear_then_download_is_empty_with_plan_id_0() {
    let rig = Rig::start();
    let dir = TempDir::new("m-clear");
    let file = dir.file("three.waypoints");
    std::fs::write(&file, wpl(&points(3))).expect("write");
    let session = connected(&rig);

    assert_eq!(
        on(&session, &format!("mission upload {}", file.display())).code,
        0
    );
    let r = on(&session, "mission clear");
    assert_eq!(
        (r.code, r.out.as_str()),
        (0, "verified\t0 waypoints\tplan id 0x00000000\n"),
        "{r:?}"
    );
    let r = on(&session, "mission download");
    assert_eq!(
        (r.code, r.out.as_str()),
        (0, "0 waypoints\tplan id 0x00000000\n"),
        "{r:?}"
    );
}

#[test]
fn clear_with_losses_in_both_directions() {
    let rig = Rig::with(config(2, 2));
    let session = connected(&rig);
    let r = on(&session, "mission clear");
    assert_eq!(
        (r.code, r.out.as_str()),
        (0, "verified\t0 waypoints\tplan id 0x00000000\n"),
        "{r:?}"
    );
}

#[test]
fn an_upload_while_armed_is_refused_and_the_list_is_unchanged() {
    let rig = Rig::start();
    let dir = TempDir::new("m-armed");
    let (three, five) = (dir.file("three.waypoints"), dir.file("five.waypoints"));
    std::fs::write(&three, wpl(&points(3))).expect("write");
    std::fs::write(&five, wpl(&points(5))).expect("write");
    let session = connected(&rig);
    assert_eq!(
        on(&session, &format!("mission upload {}", three.display())).code,
        0
    );

    rig.vehicle.lock().expect("vehicle").set_armed(true);
    let r = on(&session, &format!("mission upload {}", five.display()));
    assert_eq!(r.code, 1, "{r:?}");
    assert!(
        r.err
            .contains("refused by the vehicle: MAV_MISSION_ERROR (1)")
            && r.err
                .contains("the vehicle holds 3 waypoints, plan id 0x99C3BD49"),
        "{r:?}"
    );
    let r = on(&session, "mission clear");
    assert_eq!(r.code, 1, "{r:?}");
    assert!(r.err.contains("MAV_MISSION_ERROR (1)"), "{r:?}");
    // A download is allowed while armed.
    let r = on(&session, "mission download");
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(downloaded(&r).0.len(), 3, "{r:?}");
    assert!(
        r.out.ends_with("3 waypoints\tplan id 0x99C3BD49\n"),
        "{r:?}"
    );

    rig.vehicle.lock().expect("vehicle").set_armed(false);
    let r = on(&session, &format!("mission upload {}", five.display()));
    assert_eq!(r.code, 0, "{r:?}");
}

#[test]
fn each_invocation_on_its_own_connection_sees_the_list_of_the_one_before() {
    let rig = Rig::with(config(3, 4));
    let dir = TempDir::new("m-binary");
    let file = dir.file("five.waypoints");
    std::fs::write(&file, wpl(&points(5))).expect("write");
    let id = plan_id(&points(5));
    let r = rig.cli(&format!("mission upload {}", file.display()));
    assert_eq!(
        (r.code, r.out.as_str()),
        (
            0,
            format!("verified\t5 waypoints\tplan id {id:#010X}\n").as_str()
        ),
        "{r:?}"
    );
    let r = rig.cli("mission download");
    assert_eq!(r.code, 0, "{r:?}");
    assert!(
        r.out
            .ends_with(&format!("5 waypoints\tplan id {id:#010X}\n")),
        "{r:?}"
    );
    let r = rig.cli("mission clear");
    assert_eq!(r.code, 0, "{r:?}");
    let r = rig.cli("mission download");
    assert_eq!(
        (r.code, r.out.as_str()),
        (0, "0 waypoints\tplan id 0x00000000\n"),
        "{r:?}"
    );
}

#[test]
fn a_download_keeps_an_existing_file_without_force() {
    let rig = Rig::start();
    let dir = TempDir::new("m-force");
    let file = dir.file("keep.waypoints");
    std::fs::write(&file, "mine").expect("write");
    let r = rig.cli(&format!("mission download {}", file.display()));
    assert_eq!(r.code, 1, "{r:?}");
    assert!(r.err.contains("exists (--force overwrites it)"), "{r:?}");
    assert_eq!(std::fs::read_to_string(&file).expect("read"), "mine");
    let r = rig.cli(&format!("mission download {} --force", file.display()));
    assert_eq!(r.code, 0, "{r:?}");
    let text = std::fs::read_to_string(&file).expect("read");
    assert_eq!(
        marv_cli::waypoints::parse(&text).expect("parse"),
        Vec::new()
    );
}

#[test]
fn a_bad_file_is_exit_1_and_sends_nothing() {
    let rig = Rig::start();
    let dir = TempDir::new("m-bad");
    let file = dir.file("bad.waypoints");
    std::fs::write(
        &file,
        "QGC WPL 110\n1\t0\t3\t22\t0\t0\t0\t0\t10.0\t-30.0\t5\t1\n",
    )
    .expect("write");
    let r = rig.cli(&format!("mission upload {}", file.display()));
    assert_eq!(r.code, 1, "{r:?}");
    assert!(r.err.contains("command 22 is not supported"), "{r:?}");
    let r = rig.cli(&format!(
        "mission upload {}",
        dir.file("missing.waypoints").display()
    ));
    assert_eq!(r.code, 1, "{r:?}");
    assert!(r.err.contains("cannot read"), "{r:?}");
}

#[test]
fn the_file_the_map_tab_writes_is_read_row_0_skipped() {
    // The map tab's own text for two waypoints (lat, lon with 8 decimals, the altitude as f32 prints).
    let text = "QGC WPL 110\n\
        0\t1\t0\t16\t0\t0\t0\t0\t10.00000000\t-30.00000000\t0\t1\n\
        1\t0\t3\t16\t0\t0\t0\t0\t10.00000000\t-30.00000000\t10\t1\n\
        2\t0\t3\t16\t0\t0\t0\t0\t10.00000010\t-29.99999990\t12.5\t1\n";
    let parsed = marv_cli::waypoints::parse(text).expect("parse");
    assert_eq!(parsed.len(), 2);
    assert_eq!(
        (parsed[0].lat_e7, parsed[0].lon_e7),
        (100_000_000, -300_000_000)
    );
    assert_eq!(
        (parsed[1].lat_e7, parsed[1].lon_e7),
        (100_000_001, -299_999_999)
    );
}

#[test]
fn a_file_with_only_the_home_row_uploads_nothing_and_verifies_an_empty_list() {
    let rig = Rig::start();
    let dir = TempDir::new("m-empty");
    let (three, none) = (dir.file("three.waypoints"), dir.file("none.waypoints"));
    std::fs::write(&three, wpl(&points(3))).expect("write");
    std::fs::write(&none, wpl(&[])).expect("write");
    let session = connected(&rig);
    assert_eq!(
        on(&session, &format!("mission upload {}", three.display())).code,
        0
    );
    let r = on(&session, &format!("mission upload {}", none.display()));
    assert_eq!(
        (r.code, r.out.as_str()),
        (0, "verified\t0 waypoints\tplan id 0x00000000\n"),
        "{r:?}"
    );
    let r = on(&session, "mission download");
    assert_eq!(r.out, "0 waypoints\tplan id 0x00000000\n", "{r:?}");
}

#[test]
fn a_vehicle_that_hears_nothing_is_no_answer_and_exit_1() {
    let rig = Rig::with(config(1, 0));
    let dir = TempDir::new("m-deaf");
    let file = dir.file("three.waypoints");
    std::fs::write(&file, wpl(&points(3))).expect("write");
    let r = on(
        &connected(&rig),
        &format!("mission upload {}", file.display()),
    );
    assert_eq!(r.code, 1, "{r:?}");
    assert_eq!(r.out, "", "{r:?}");
    assert!(
        r.err
            .contains("mission upload failed: no answer from the vehicle"),
        "{r:?}"
    );
}

#[test]
fn a_vehicle_that_cannot_be_heard_is_no_answer_and_exit_1() {
    let rig = Rig::with(config(0, 1));
    let session = connected(&rig);
    let r = on(&session, "mission clear");
    assert_eq!(r.code, 1, "{r:?}");
    assert!(
        r.err
            .contains("mission clear failed: no answer from the vehicle"),
        "{r:?}"
    );
}

/// Sends a mission command to the session itself and waits for the service to end it.
fn direct(
    session: &gcs_mavlink::Session,
    command: gcs_mavlink::Command,
) -> gcs_mavlink::services::missions::MissionsState {
    use gcs_mavlink::services::missions::MissionStatus;
    let before = session.state().missions.runs;
    session.send(command);
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        {
            let state = session.state();
            if state.missions.runs != before
                && matches!(
                    state.missions.status,
                    MissionStatus::Verified
                        | MissionStatus::Complete
                        | MissionStatus::Failed { .. }
                )
            {
                return state.missions.clone();
            }
        }
        assert!(Instant::now() < deadline, "the operation never ended");
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn a_waypoint_the_vehicle_would_refuse_is_refused_by_the_station_and_nothing_is_sent() {
    use gcs_mavlink::services::missions::{MissionFailure, MissionPoint, MissionStatus};
    let rig = Rig::start();
    let session = connected(&rig);
    let point = |x: i32, y: i32, z: f32| MissionPoint {
        lat_e7: x,
        lon_e7: y,
        z,
    };
    let good: Vec<MissionPoint> = points(3)
        .into_iter()
        .map(|(x, y, z)| point(x, y, z))
        .collect();
    let state = direct(
        &session,
        gcs_mavlink::Command::UploadMission {
            items: good.clone(),
        },
    );
    assert_eq!(state.status, MissionStatus::Verified, "{state:?}");
    for (items, problem) in [
        (
            vec![good[0], point(900_000_001, 0, 1.0)],
            "latitude out of range",
        ),
        (
            vec![point(-900_000_000, 1_800_000_001, 1.0)],
            "longitude out of range",
        ),
        (vec![point(0, 0, f32::NAN)], "altitude is not finite"),
    ] {
        let state = direct(&session, gcs_mavlink::Command::UploadMission { items });
        let MissionStatus::Failed { reason, .. } = &state.status else {
            panic!("{state:?}");
        };
        assert!(
            matches!(reason, MissionFailure::InvalidWaypoint { problem: p, .. } if *p == problem),
            "{state:?}"
        );
        assert!(state.vehicle.is_none(), "no read-back is made: {state:?}");
    }
    // The vehicle's list is what it was, and the limits themselves are sent and accepted.
    let state = direct(&session, gcs_mavlink::Command::DownloadMission);
    let held = state.vehicle.expect("the list");
    assert_eq!((held.items, held.plan_id), (good, 0x99C3_BD49));
    let edge = vec![
        point(900_000_000, 1_800_000_000, 1.0),
        point(-900_000_000, -1_800_000_000, -1.0),
    ];
    let state = direct(
        &session,
        gcs_mavlink::Command::UploadMission { items: edge },
    );
    assert_eq!(state.status, MissionStatus::Verified, "{state:?}");
}

#[test]
fn z_travels_as_its_four_bytes_whatever_they_are() {
    use gcs_mavlink::services::missions::{MissionPoint, MissionStatus};
    let rig = Rig::start();
    let session = connected(&rig);
    let zs = [
        f32::from_bits(0x8000_0000), // -0.0
        f32::MAX,
        f32::from_bits(0x0000_0001), // the smallest subnormal
        f32::from_bits(0xC2F6_E979), // -123.456
    ];
    let items: Vec<MissionPoint> = zs
        .iter()
        .map(|&z| MissionPoint {
            lat_e7: 100_000_000,
            lon_e7: -300_000_000,
            z,
        })
        .collect();
    let state = direct(
        &session,
        gcs_mavlink::Command::UploadMission {
            items: items.clone(),
        },
    );
    assert_eq!(state.status, MissionStatus::Verified, "{state:?}");
    let held = state.vehicle.expect("the read-back");
    let bits: Vec<u32> = held.items.iter().map(|p| p.z.to_bits()).collect();
    assert_eq!(bits, [0x8000_0000, 0x7F7F_FFFF, 1, 0xC2F6_E979]);
    let wanted: Vec<(i32, i32, f32)> = items.iter().map(|p| (p.lat_e7, p.lon_e7, p.z)).collect();
    assert_eq!(held.plan_id, plan_id(&wanted));
}
