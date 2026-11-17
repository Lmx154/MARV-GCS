//! The ground station against the real board over its USB port: the `mavlink_demo` program of the
//! `mavlink-device` repo must be running (`tools/board_check.sh` starts it). Every test is ignored
//! by default; run them one at a time:
//! `cargo test -p marv-gcs --test board -- --ignored --test-threads 1 --nocapture`.
//!
//! The tests are named `t1_`, `t2_`, ... because libtest runs them in name order and the first one
//! must be the first session after the board restarts: the boot text is sent once per power-up.
//! `listen_only` and `no_dtr` each need a freshly restarted board too (`tools/board_check.sh` restarts
//! the device program before each of the three phases and names the tests it runs in each).
//!
//! The numbers each test measured are printed as `BOARD key=value` lines.

use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use gcs_mavlink::dialect::{HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavState, MavType};
use gcs_mavlink::presets;
use gcs_mavlink::services::params::{FetchStatus, ParamEncoding, ParamValue, WriteStatus};
use gcs_mavlink::tlog::{LogSink, TlogReader};
use gcs_mavlink::{
    Access, Body, Command, Deframer, Encoder, Frame, LinkStatus, Message, Session, SessionConfig,
    VehicleState,
};
use gcs_transport::{Endpoint, list_serial_ports, open};

const VID: u16 = 0xc0de;
const PID: u16 = 0xcafe;
const PRODUCT: &str = "MARV V2 link demo";

/// The demonstration store of the board (`mavlink-device/hil/src/bin/mavlink_demo.rs`), in its
/// order, which is also the order of the simulator's defaults.
fn board_params() -> Vec<(&'static str, ParamValue)> {
    vec![
        ("IMU_RATE_HZ", ParamValue::I32(800)),
        ("IMU_ACC_RANGE", ParamValue::I32(16)),
        ("IMU_GYR_LPF_HZ", ParamValue::F32(98.0)),
        ("HIGHG_RANGE", ParamValue::I32(320)),
        ("BARO_RATE_HZ", ParamValue::I32(50)),
        ("GPS_RATE_HZ", ParamValue::I32(10)),
        ("LOG_ENABLE", ParamValue::I32(1)),
        ("LOG_RATE_HZ", ParamValue::I32(100)),
        ("BAT_V_SCALE", ParamValue::F32(11.0)),
        ("BAT_LOW_V", ParamValue::F32(14.0)),
        ("SYS_TRIM", ParamValue::I32(-3)),
        ("SYS_UID", ParamValue::I32(1_592_460_852)),
        ("NAV_TEST_16CHARS", ParamValue::F32(1.5)),
    ]
}

/// The port of the board, waiting up to 10 s for it to appear. A port with the right VID / PID
/// but another product string is another firmware: that is a failure, not a port to wait for.
fn find_port() -> String {
    let end = Instant::now() + Duration::from_secs(10);
    loop {
        let found = list_serial_ports()
            .into_iter()
            .find(|p| p.usb.as_ref().is_some_and(|u| u.vid == VID && u.pid == PID));
        if let Some(port) = found {
            let product = port.usb.as_ref().and_then(|u| u.product.clone());
            assert_eq!(
                product.as_deref(),
                Some(PRODUCT),
                "{} has VID {VID:#06x} / PID {PID:#06x} but another firmware: product {product:?}",
                port.path
            );
            return port.path;
        }
        assert!(
            Instant::now() < end,
            "no serial port with VID {VID:#06x} / PID {PID:#06x} within 10 s"
        );
        thread::sleep(Duration::from_millis(100));
    }
}

/// What a reader returned: the time since the port was opened and the bytes, one entry per read.
type Reads = Arc<Mutex<Vec<(Duration, Vec<u8>)>>>;

/// A thin reader in front of the real one: it records every byte and when it arrived.
struct Recorder {
    inner: Box<dyn Read + Send>,
    opened: Instant,
    reads: Reads,
}

impl Read for Recorder {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let n = self.inner.read(buf)?;
        let at = self.opened.elapsed();
        if let Ok(mut reads) = self.reads.lock() {
            reads.push((at, buf[..n].to_vec()));
        }
        Ok(n)
    }
}

/// Opens the port as the window does, with a recording reader; `opened` is taken when `open` returned.
fn open_recorded(path: &str) -> (gcs_transport::Link, Reads, Instant) {
    let endpoint = Endpoint::Serial {
        path: path.to_owned(),
        baud: gcs_transport::DEFAULT_BAUD,
    };
    let mut link = open(&endpoint).unwrap_or_else(|e| panic!("{e}"));
    let opened = Instant::now();
    let reads = Reads::default();
    link.reader = Box::new(Recorder {
        inner: link.reader,
        opened,
        reads: Arc::clone(&reads),
    });
    (link, reads, opened)
}

/// Everything the recorder saw, through the ground station's deframer.
fn deframe(reads: &Reads) -> (Vec<Frame>, gcs_mavlink::RxStats, Option<u8>) {
    let reads = reads.lock().expect("reads");
    let mut deframer = Deframer::new();
    let mut frames = Vec::new();
    for (_, bytes) in reads.iter() {
        deframer.push(bytes, &mut frames);
    }
    let first_byte = reads.first().and_then(|(_, b)| b.first().copied());
    (frames, deframer.stats(), first_byte)
}

fn is_heartbeat(frame: &Frame) -> bool {
    frame.message_id == 0 && matches!(frame.body, Body::Known(_))
}

fn has_ready_text(frames: &[Frame]) -> bool {
    frames.iter().any(|f| {
        f.message_id == 253
            && f.raw
                .windows(b"MARV link demo ready".len())
                .any(|w| w == b"MARV link demo ready")
    })
}

/// The sequence numbers of the frames are consecutive (mod 256), from the first.
fn assert_consecutive(frames: &[Frame]) {
    for pair in frames.windows(2) {
        assert_eq!(
            pair[1].sequence,
            pair[0].sequence.wrapping_add(1),
            "sequence {} then {}",
            pair[0].sequence,
            pair[1].sequence
        );
    }
}

/// An open session on the board's port, started as the window starts it.
struct Board {
    session: Option<Session>,
    opened: Instant,
    reads: Reads,
}

impl Board {
    fn open(path: &str) -> Board {
        let (link, reads, opened) = open_recorded(path);
        let session = Session::start(
            link.reader,
            link.writer,
            SessionConfig {
                access: Access::Configure,
                ..SessionConfig::default()
            },
            Box::new(|| {}),
        );
        Board {
            session: Some(session),
            opened,
            reads,
        }
    }

    fn session(&self) -> &Session {
        self.session.as_ref().expect("session")
    }

    fn state(&self) -> VehicleState {
        self.session().state().clone()
    }

    /// Polls every `poll` until `condition` holds; the time it took, or `None` at the deadline.
    fn wait_poll(
        &self,
        seconds: f64,
        poll: Duration,
        mut condition: impl FnMut(&VehicleState) -> bool,
    ) -> Option<Duration> {
        let begun = Instant::now();
        let end = begun + Duration::from_secs_f64(seconds);
        loop {
            if condition(&self.session().state()) {
                return Some(begun.elapsed());
            }
            if Instant::now() >= end {
                return None;
            }
            thread::sleep(poll);
        }
    }

    fn wait(&self, seconds: f64, condition: impl FnMut(&VehicleState) -> bool) -> Option<Duration> {
        self.wait_poll(seconds, Duration::from_millis(20), condition)
    }

    /// Waits for `Connected` within 2 s of the port being opened; the time it took.
    fn connected(&self) -> Duration {
        self.wait(2.0, |s| s.link.status == LinkStatus::Connected);
        let status = self.state().link.status;
        assert_eq!(status, LinkStatus::Connected, "not connected within 2 s");
        self.opened.elapsed()
    }

    fn complete(&self, seconds: f64) -> Duration {
        self.wait(seconds, |s| s.params.fetch == FetchStatus::Complete)
            .unwrap_or_else(|| {
                let s = self.state();
                panic!(
                    "no Complete within {seconds} s: {:?}, {} params",
                    s.params.fetch,
                    s.params.params.len()
                )
            })
    }

    /// Asks for the whole list again and waits until it is `Complete` again. The status must be
    /// seen to leave `Complete` (polled every 1 ms), so an old `Complete` is never mistaken for
    /// the new one.
    fn refresh(&self, seconds: f64) -> Duration {
        self.session().send(Command::RefreshParams);
        let begun = Instant::now();
        let left = self.wait_poll(2.0, Duration::from_millis(1), |s| {
            s.params.fetch != FetchStatus::Complete
        });
        assert!(left.is_some(), "the status never left Complete");
        self.complete(seconds);
        begun.elapsed()
    }
}

fn rate(state: &VehicleState, label: &str) -> f32 {
    state
        .messages
        .entries
        .values()
        .find(|entry| entry.label == label)
        .unwrap_or_else(|| panic!("no entry {label}"))
        .rate_hz
}

fn assert_rate(state: &VehicleState, label: &str, expected: f32, percent: f32) {
    let rate = rate(state, label);
    println!("BOARD rate {label}={rate:.3} Hz (want {expected} +-{percent} %)");
    assert!(
        (rate - expected).abs() <= expected * percent / 100.0,
        "{label}: {rate} Hz, expected {expected} Hz within {percent} %"
    );
}

fn assert_table_is_the_store(state: &VehicleState) {
    let want = board_params();
    assert_eq!(
        state.params.params.len(),
        want.len(),
        "number of parameters"
    );
    for (name, value) in want {
        let got = state.params.value(name);
        assert_eq!(got, Some(value), "{name}");
        assert_eq!(got.map(|v| v.kind()), Some(value.kind()), "{name}: type");
    }
}

#[test]
#[ignore = "needs the board running mavlink_demo"]
fn t1_connects_streams_at_the_contract_rates_and_says_ready() {
    let path = find_port();
    let board = Board::open(&path);
    let took = board.connected();
    println!("BOARD connected_ms={}", took.as_millis());
    let state = board.state();
    let vehicle = state.link.vehicle.expect("vehicle identity");
    assert_eq!(
        (vehicle.system_id, vehicle.component_id),
        (1, 1),
        "{vehicle:?}"
    );
    assert_eq!(vehicle.mav_type, "MAV_TYPE_GENERIC");

    // The text is sent once per power-up, on the first link after the board starts, and this is
    // the first session after it started (the start gate holds the board's first write until the
    // port is open).
    let texts = board.wait(2.0, |s| {
        s.texts
            .entries
            .iter()
            .any(|e| e.text == "MARV link demo ready")
    });
    println!(
        "BOARD ready_text_after_ms={:?} texts={:?}",
        texts.map(|d| d.as_millis()),
        board
            .state()
            .texts
            .entries
            .iter()
            .map(|e| &e.text)
            .collect::<Vec<_>>()
    );
    assert!(
        texts.is_some(),
        "the text MARV link demo ready did not arrive within 2 s"
    );

    let mut first_bad = None;
    let mut watch = |until: u64| {
        while board.opened.elapsed() < Duration::from_secs(until) {
            let rx = board.state().link.rx;
            if first_bad.is_none() && (rx.crc_errors > 0 || rx.discarded_bytes > 0) {
                first_bad = Some((board.opened.elapsed().as_millis(), rx));
            }
            thread::sleep(Duration::from_millis(20));
        }
    };
    watch(10);
    let state = board.state();
    for instance in 0..4 {
        assert_rate(&state, &format!("HIGHRES_IMU[{instance}]"), 50.0, 5.0);
    }
    assert_rate(&state, "SCALED_PRESSURE", 10.0, 5.0);
    assert_rate(&state, "GPS_RAW_INT", 5.0, 10.0);
    assert_rate(&state, "HEARTBEAT", 1.0, 5.0);

    let zacc = state
        .messages
        .series("HIGHRES_IMU", 0, "zacc")
        .expect("series HIGHRES_IMU[0].zacc");
    assert!(!zacc.points.is_empty(), "no zacc points");
    let (lo, hi) = zacc
        .points
        .iter()
        .fold((f64::MAX, f64::MIN), |(lo, hi), p| {
            (lo.min(p[1]), hi.max(p[1]))
        });
    println!(
        "BOARD zacc_min={lo:.3} zacc_max={hi:.3} points={}",
        zacc.points.len()
    );
    assert!(
        lo >= -10.5 && hi <= -9.0,
        "zacc {lo}..{hi} outside -10.5 to -9.0"
    );
    let xgyro2 = state.messages.series("HIGHRES_IMU", 2, "xgyro");
    assert!(
        xgyro2.is_none_or(|s| s.points.is_empty()),
        "HIGHRES_IMU[2].xgyro is NaN and must have no points"
    );
    let xmag3 = state.messages.series("HIGHRES_IMU", 3, "xmag");
    assert!(
        xmag3.is_some_and(|s| !s.points.is_empty()),
        "HIGHRES_IMU[3].xmag must have points"
    );

    watch(20);
    println!("BOARD first_bad_ms_and_stats={first_bad:?}");
    let state = board.state();
    println!(
        "BOARD at_20s frames={} crc_errors={} discarded_bytes={} unknown={} undecodable={} lost_frames={}",
        state.link.rx.frames,
        state.link.rx.crc_errors,
        state.link.rx.discarded_bytes,
        state.link.rx.unknown,
        state.link.rx.undecodable,
        state.messages.lost_frames
    );
    assert_eq!(state.link.rx.crc_errors, 0, "{:?}", state.link.rx);
    assert_eq!(state.link.rx.discarded_bytes, 0, "{:?}", state.link.rx);
    assert_eq!(state.messages.lost_frames, 0, "frames were lost");
    assert_eq!(state.link.status, LinkStatus::Connected);

    let (frames, stats, first_byte) = deframe(&board.reads);
    let first = frames.first().expect("a first frame");
    println!(
        "BOARD first_byte={first_byte:#04x?} first_frame id={} seq={} deframer_stats={stats:?}",
        first.message_id, first.sequence
    );
    assert_eq!(first_byte, Some(0xFD), "the first byte read");
    assert!(
        is_heartbeat(first),
        "the first frame is id {}",
        first.message_id
    );
    assert_eq!(first.sequence, 0, "the first frame's sequence number");
    assert_eq!(stats.discarded_bytes, 0, "{stats:?}");
    assert_eq!(stats.crc_errors, 0, "{stats:?}");
}

#[test]
#[ignore = "needs the board running mavlink_demo"]
fn t2_parameters_list_exactly_and_refresh_ten_times() {
    let board = Board::open(&find_port());
    board.connected();
    let took = board.complete(4.0);
    println!("BOARD first_list_ms={}", took.as_millis());
    let state = board.state();
    assert_table_is_the_store(&state);
    assert!(state.params.encoding_known, "encoding_known");
    assert_eq!(state.params.encoding, ParamEncoding::Bytewise);

    let mut times = Vec::new();
    for n in 1..=10 {
        let took = board.refresh(4.0);
        times.push(took.as_millis());
        let state = board.state();
        assert_eq!(state.params.params.len(), 13, "refresh {n}");
        assert_table_is_the_store(&state);
    }
    println!("BOARD refresh_ms={times:?}");
}

#[test]
#[ignore = "needs the board running mavlink_demo"]
fn t3_writes_are_confirmed_refused_and_clamped() {
    let board = Board::open(&find_port());
    board.connected();
    board.complete(4.0);
    assert_table_is_the_store(&board.state());

    let send = |name: &str, value: ParamValue| {
        board.session().send(Command::SetParam {
            name: name.to_owned(),
            value,
        });
    };

    // Confirmed writes: an INT32 and a REAL32.
    for (name, old, new) in [
        ("IMU_RATE_HZ", ParamValue::I32(800), ParamValue::I32(400)),
        ("BAT_LOW_V", ParamValue::F32(14.0), ParamValue::F32(13.5)),
    ] {
        send(name, new);
        let done = board.wait(3.0, |s| {
            s.params.value(name) == Some(new) && !s.params.writes.contains_key(name)
        });
        println!(
            "BOARD write {name} confirmed_ms={:?}",
            done.map(|d| d.as_millis())
        );
        assert!(
            done.is_some(),
            "{name}: {:?}",
            board.state().params.writes.get(name)
        );
        board.refresh(4.0);
        assert_eq!(
            board.state().params.value(name),
            Some(new),
            "{name} after a fresh list"
        );
        assert_eq!(board.state().params.fetch, FetchStatus::Complete);
        send(name, old);
        let back = board.wait(3.0, |s| {
            s.params.value(name) == Some(old) && !s.params.writes.contains_key(name)
        });
        assert!(back.is_some(), "{name} not restored");
    }

    // A read-only parameter: refused, the old value in effect.
    let uid = ParamValue::I32(1_592_460_852);
    send("SYS_UID", ParamValue::I32(5));
    let refused = board.wait(3.0, |s| {
        matches!(
            s.params.writes.get("SYS_UID"),
            Some(WriteStatus::Refused { .. })
        )
    });
    let state = board.state();
    println!(
        "BOARD SYS_UID writes={:?} value={:?}",
        state.params.writes.get("SYS_UID"),
        state.params.value("SYS_UID")
    );
    assert!(
        refused.is_some(),
        "no Refused: {:?}",
        state.params.writes.get("SYS_UID")
    );
    assert_eq!(
        state.params.writes.get("SYS_UID"),
        Some(&WriteStatus::Refused {
            value: ParamValue::I32(5),
            in_effect: uid
        })
    );
    assert_eq!(state.params.value("SYS_UID"), Some(uid));

    // A value out of range: the store clamps it to 1000 and the echo says so.
    send("LOG_RATE_HZ", ParamValue::I32(5000));
    let clamped = board.wait(3.0, |s| {
        matches!(
            s.params.writes.get("LOG_RATE_HZ"),
            Some(WriteStatus::Refused { .. })
        )
    });
    let state = board.state();
    println!(
        "BOARD LOG_RATE_HZ writes={:?} value={:?}",
        state.params.writes.get("LOG_RATE_HZ"),
        state.params.value("LOG_RATE_HZ")
    );
    assert!(
        clamped.is_some(),
        "no Refused: {:?}",
        state.params.writes.get("LOG_RATE_HZ")
    );
    assert_eq!(
        state.params.writes.get("LOG_RATE_HZ"),
        Some(&WriteStatus::Refused {
            value: ParamValue::I32(5000),
            in_effect: ParamValue::I32(1000)
        })
    );
    assert_eq!(
        state.params.value("LOG_RATE_HZ"),
        Some(ParamValue::I32(1000))
    );

    // Restore it, and check that the board is back to its defaults on a fresh list.
    send("LOG_RATE_HZ", ParamValue::I32(100));
    let back = board.wait(3.0, |s| {
        s.params.value("LOG_RATE_HZ") == Some(ParamValue::I32(100))
            && !s.params.writes.contains_key("LOG_RATE_HZ")
    });
    assert!(
        back.is_some(),
        "LOG_RATE_HZ not restored: {:?}",
        board.state().params.writes
    );
    board.refresh(4.0);
    assert_table_is_the_store(&board.state());
}

#[test]
#[ignore = "needs the board running mavlink_demo"]
fn t4_records_a_tlog_that_reads_back() {
    let board = Board::open(&find_port());
    board.connected();
    board.complete(4.0);

    let dir = std::env::var_os("BOARD_CHECK_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    let path = dir.join("board.tlog");
    let file = File::create(&path).expect("create the log file");
    board.session().send(Command::StartLog {
        sink: LogSink::new(file),
        label: path.display().to_string(),
    });
    assert!(
        board.wait(2.0, |s| s.log.recording.is_some()).is_some(),
        "recording did not start: {:?}",
        board.state().log
    );
    // A fresh list inside the recording, so the file holds PARAM_VALUE frames too.
    board.session().send(Command::RefreshParams);
    thread::sleep(Duration::from_secs(5));
    board.session().send(Command::StopLog);
    assert!(
        board.wait(2.0, |s| s.log.recording.is_none()).is_some(),
        "recording did not stop"
    );
    let log = board.state().log;
    assert_eq!(log.error, None);

    let records: Vec<_> = TlogReader::new(File::open(&path).expect("open the log"))
        .collect::<Result<_, _>>()
        .expect("the log reads without error");
    let from = |system: u8| records.iter().filter(|r| r.frame[5] == system).count();
    println!(
        "BOARD tlog path={} frames={} bytes={} records={} from_vehicle={} from_gcs={}",
        path.display(),
        log.frames,
        log.bytes,
        records.len(),
        from(1),
        from(255)
    );
    assert_eq!(
        records.len() as u64,
        log.frames,
        "records against log.frames"
    );
    assert_eq!(std::fs::metadata(&path).expect("metadata").len(), log.bytes);
    assert!(from(1) > 0, "no frame from the vehicle");
    assert!(from(255) > 0, "no frame from the ground station");
    assert_eq!(
        from(1) + from(255),
        records.len(),
        "frames from other systems"
    );
    assert!(records.windows(2).all(|w| w[0].time_us <= w[1].time_us));
}

#[test]
#[ignore = "needs the board running mavlink_demo"]
fn t5_reconnects_and_lists_again() {
    let path = find_port();
    let board = Board::open(&path);
    board.connected();
    board.complete(4.0);
    drop(board);
    thread::sleep(Duration::from_secs(1));

    let again = Board::open(&path);
    let took = again.connected();
    let list = again.complete(4.0);
    println!(
        "BOARD reconnect connected_ms={} list_ms={}",
        took.as_millis(),
        list.as_millis()
    );
    assert_table_is_the_store(&again.state());
}

/// The board with nobody speaking to it: after the open returned, 100 to 400 ms pass (DTR held for
/// 100 ms is the gate's way out), then a clean stream that starts with a whole HEARTBEAT, sequence 0.
/// Needs a freshly restarted board.
#[test]
#[ignore = "needs a freshly restarted board running mavlink_demo"]
fn listen_only() {
    let path = find_port();
    let (link, reads, opened) = open_recorded(&path);
    let _writer = link.writer;
    let mut reader = link.reader;
    let mut buf = [0u8; 4096];
    while opened.elapsed() < Duration::from_secs(3) {
        match reader.read(&mut buf) {
            Ok(0) => break,
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => panic!("read: {e}"),
        }
    }
    let first_at = reads.lock().expect("reads").first().map(|(t, _)| *t);
    let (frames, stats, first_byte) = deframe(&reads);
    println!(
        "BOARD listen_only first_byte_ms={:.1} first_byte={first_byte:#04x?} frames={} crc_errors={} discarded_bytes={} first_seq={:?} ready_text={}",
        first_at.map_or(f64::NAN, |t| t.as_secs_f64() * 1000.0),
        frames.len(),
        stats.crc_errors,
        stats.discarded_bytes,
        frames.first().map(|f| f.sequence),
        has_ready_text(&frames)
    );
    let first_at = first_at.expect("no byte in 3 s");
    assert_eq!(first_byte, Some(0xFD), "the first byte read");
    assert!(
        first_at >= Duration::from_millis(100) && first_at <= Duration::from_millis(400),
        "first byte {first_at:?} after the open returned, want 100 to 400 ms"
    );
    let first = frames.first().expect("a first frame");
    assert!(
        is_heartbeat(first),
        "the first frame is id {}",
        first.message_id
    );
    assert_eq!(first.sequence, 0, "the first frame's sequence number");
    assert_consecutive(&frames);
    assert_eq!(stats.crc_errors, 0, "{stats:?}");
    assert_eq!(stats.discarded_bytes, 0, "{stats:?}");
    assert!(
        has_ready_text(&frames),
        "the start-up text is not among the frames"
    );
}

/// A host that never raises DTR: the port is opened with DTR cleared (`dtr_on_open(false)`, and
/// cleared again right after the open; the kernel itself raises DTR inside `open()` and the
/// library drops it afterwards, so the line is high for a moment only). Nothing arrives until the
/// ground station's HEARTBEAT is sent. Needs a freshly restarted board.
#[test]
#[ignore = "needs a freshly restarted board running mavlink_demo"]
fn no_dtr() {
    let path = find_port();
    let mut port = serialport::new(&path, gcs_transport::DEFAULT_BAUD)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .timeout(Duration::from_millis(10))
        .dtr_on_open(false)
        .open()
        .expect("open");
    port.write_data_terminal_ready(false).expect("clear DTR");
    let opened = Instant::now();
    let mut buf = [0u8; 4096];

    let mut silent = 0usize;
    while opened.elapsed() < Duration::from_secs(1) {
        match port.read(&mut buf) {
            Ok(n) => silent += n,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => panic!("read: {e}"),
        }
    }
    println!("BOARD no_dtr bytes_in_silent_second={silent}");
    assert_eq!(silent, 0, "bytes arrived with DTR low and nothing sent");

    let heartbeat = Encoder::new(255, 190).encode(&Message::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    }));
    let mut deframer = Deframer::new();
    let mut frames = Vec::new();
    let sent = Instant::now();
    port.write_all(&heartbeat).expect("write");
    let mut first_frame_at = None;
    let mut first_byte = None;
    while sent.elapsed() < Duration::from_secs(3) {
        match port.read(&mut buf) {
            Ok(n) => {
                first_byte = first_byte.or(buf[..n].first().copied());
                deframer.push(&buf[..n], &mut frames);
                if first_frame_at.is_none() && !frames.is_empty() {
                    first_frame_at = Some(sent.elapsed());
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => panic!("read: {e}"),
        }
    }
    let stats = deframer.stats();
    println!(
        "BOARD no_dtr first_frame_ms={:.1} first_byte={first_byte:#04x?} frames={} crc_errors={} discarded_bytes={} first_seq={:?} ready_text={}",
        first_frame_at.map_or(f64::NAN, |t| t.as_secs_f64() * 1000.0),
        frames.len(),
        stats.crc_errors,
        stats.discarded_bytes,
        frames.first().map(|f| f.sequence),
        has_ready_text(&frames)
    );
    let first_frame_at = first_frame_at.expect("no frame in 3 s");
    assert!(
        first_frame_at <= Duration::from_millis(200),
        "first frame {first_frame_at:?} after the HEARTBEAT, want at most 200 ms"
    );
    assert_eq!(first_byte, Some(0xFD), "the first byte read");
    let first = frames.first().expect("a first frame");
    assert_eq!(first.sequence, 0, "the first frame's sequence number");
    assert_consecutive(&frames);
    assert_eq!(stats.crc_errors, 0, "{stats:?}");
    assert_eq!(stats.discarded_bytes, 0, "{stats:?}");
}

/// The card's apply logic against the board running the firmware: `write_plan` of the preset, one
/// `SetParam` each in that order, then every echo (the write leaves `params.writes` and the table holds
/// the value) compared with the table. RAM only: no command 245 is ever sent.
fn apply_preset(board: &Board, sensor: &presets::Sensor, id: i32) {
    use marv_gcs::views::presets::{fields_differ, write_plan};
    let chosen = presets::preset(sensor, id).expect("preset");
    let plan = write_plan(&board.state().params, sensor, chosen).expect("fields in the list");
    let names: Vec<String> = plan.iter().map(|(n, _)| (*n).to_owned()).collect();
    for (name, value) in plan {
        board.session().send(Command::SetParam {
            name: name.to_owned(),
            value,
        });
    }
    board
        .wait(5.0, |s| {
            names.iter().all(|n| {
                !matches!(
                    s.params.writes.get(n),
                    Some(WriteStatus::Pending { .. } | WriteStatus::Unanswered { .. })
                )
            })
        })
        .unwrap_or_else(|| panic!("{}: writes not answered: {names:?}", chosen.name));
    let state = board.state();
    for name in &names {
        assert!(
            !state.params.writes.contains_key(name),
            "{} -> {}: {name} was refused or unanswered: {:?}",
            sensor.fields[0],
            chosen.name,
            state.params.writes.get(name)
        );
    }
    assert_eq!(
        fields_differ(&state.params, sensor, id),
        Some(false),
        "{}: the echoes differ from the table",
        chosen.name
    );
}

#[test]
#[ignore = "needs the board running the firmware (tools/run.sh)"]
fn presets_every_baro_and_mag_transition_is_accepted_in_ram_and_ends_on_balanced() {
    let board = Board::open("/dev/ttyACM1");
    board.connected();
    board.complete(10.0);
    for sensor in [&presets::BARO, &presets::MAG] {
        for from in sensor.presets {
            for to in sensor.presets.iter().filter(|p| p.id != from.id) {
                apply_preset(&board, sensor, from.id);
                apply_preset(&board, sensor, to.id);
                println!("BOARD preset {} -> {} ok", from.name, to.name);
            }
        }
        apply_preset(&board, sensor, sensor.default_id);
        assert_eq!(
            presets::preset(sensor, sensor.default_id).map(|p| p.name),
            Some("Balanced")
        );
    }
    assert!(
        board
            .state()
            .command
            .as_ref()
            .is_none_or(|c| c.command != 245),
        "nothing was saved"
    );
}
