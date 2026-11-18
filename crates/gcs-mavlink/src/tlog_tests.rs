use std::io::Cursor;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use super::*;
use crate::dialect::{HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavState, MavType};
use crate::{Body, Deframer, Encoder, Message, VehicleState};

/// A writer the test can read back from and count flushes on.
#[derive(Clone, Default)]
struct Shared {
    data: Arc<Mutex<Vec<u8>>>,
    flushes: Arc<AtomicUsize>,
    dropped: Arc<AtomicUsize>,
}

impl Shared {
    fn bytes(&self) -> Vec<u8> {
        self.data.lock().unwrap().clone()
    }

    fn flushes(&self) -> usize {
        self.flushes.load(Ordering::SeqCst)
    }
}

impl Write for Shared {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.data.lock().unwrap().extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        self.flushes.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
}

impl Drop for Shared {
    fn drop(&mut self) {
        // Only the clone inside the recorder counts: the test keeps its own to the end.
        if Arc::strong_count(&self.data) > 1 {
            self.dropped.fetch_add(1, Ordering::SeqCst);
        }
    }
}

/// Fails every write after `ok` bytes went through.
struct Failing {
    ok: usize,
    dropped: Arc<AtomicUsize>,
}

impl Write for Failing {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.ok >= buf.len() {
            self.ok -= buf.len();
            Ok(buf.len())
        } else {
            Err(io::Error::other("disk full"))
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Drop for Failing {
    fn drop(&mut self) {
        self.dropped.fetch_add(1, Ordering::SeqCst);
    }
}

struct Rig {
    recorder: Recorder,
    state: VehicleState,
    out: Vec<Message>,
    changed: bool,
}

impl Rig {
    fn new() -> Self {
        Self {
            recorder: Recorder::new(),
            state: VehicleState::default(),
            out: Vec::new(),
            changed: false,
        }
    }

    fn run(&mut self, now_ms: u64, wall_us: u64, call: impl FnOnce(&mut Recorder, &mut Ctx)) {
        let mut cx = Ctx {
            now: Duration::from_millis(now_ms),
            wall_time_us: wall_us,
            state: &mut self.state,
            out: &mut self.out,
            changed: false,
        };
        call(&mut self.recorder, &mut cx);
        self.changed = cx.changed;
    }

    fn command(&mut self, command: &Command, now_ms: u64) {
        self.run(now_ms, 0, |recorder, cx| recorder.on_command(command, cx));
    }

    fn start(&mut self, writer: impl Write + Send + 'static, now_ms: u64) {
        let command = Command::StartLog {
            sink: LogSink::new(writer),
            label: "flight.tlog".into(),
        };
        self.command(&command, now_ms);
    }

    fn stop(&mut self, now_ms: u64) {
        self.command(&Command::StopLog, now_ms);
    }

    fn received(&mut self, raw: &[u8], now_ms: u64, wall_us: u64) {
        let frame = frame_of(raw);
        self.run(now_ms, wall_us, |recorder, cx| {
            recorder.on_frame(&frame, cx)
        });
    }

    fn sent(&mut self, raw: &[u8], now_ms: u64, wall_us: u64) {
        self.run(now_ms, wall_us, |recorder, cx| recorder.on_sent(raw, cx));
    }

    fn tick(&mut self, now_ms: u64) {
        self.run(now_ms, 0, |recorder, cx| recorder.on_tick(cx));
    }
}

fn frame_of(raw: &[u8]) -> Frame {
    Frame {
        system_id: raw[5],
        component_id: raw[6],
        sequence: raw[4],
        message_id: 0,
        raw: raw.to_vec(),
        body: Body::Unknown,
    }
}

fn heartbeat() -> Message {
    Message::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

/// A MAVLink 2 heartbeat frame with the given sequence number.
fn v2_frame(sequence: u8) -> Vec<u8> {
    let mut encoder = Encoder::new(1, 1);
    for _ in 0..sequence {
        encoder.encode(&heartbeat());
    }
    encoder.encode(&heartbeat())
}

fn v2_signed(payload: usize) -> Vec<u8> {
    let mut frame = vec![STX_V2, payload as u8, INCOMPAT_SIGNED, 0, 1, 1, 1, 0, 0, 0];
    frame.extend(std::iter::repeat_n(0xAB, payload + 2 + 13));
    frame
}

fn v1_frame(payload: usize) -> Vec<u8> {
    let mut frame = vec![STX_V1, payload as u8, 3, 1, 1, 0];
    frame.extend(std::iter::repeat_n(0xCD, payload + 2));
    frame
}

fn record(time_us: u64, frame: &[u8]) -> Vec<u8> {
    let mut out = time_us.to_be_bytes().to_vec();
    out.extend_from_slice(frame);
    out
}

fn read_all(bytes: Vec<u8>) -> Vec<io::Result<TlogRecord>> {
    TlogReader::new(Cursor::new(bytes)).collect()
}

// ---- the layout ----

#[test]
fn a_record_is_the_big_endian_time_then_the_frame() {
    let mut rig = Rig::new();
    let file = Shared::default();
    rig.start(file.clone(), 0);
    let frame = v2_frame(0);
    rig.received(&frame, 0, 0x0001_0203_0405_0607);
    let mut want = vec![0, 1, 2, 3, 4, 5, 6, 7];
    want.extend_from_slice(&frame);
    assert_eq!(file.bytes(), want);
}

// ---- the session recording ----

fn start_session(rig: &mut Rig, writer: impl Write + Send + 'static, label: &str) {
    rig.command(
        &Command::StartSessionLog {
            sink: LogSink::new(writer),
            label: label.into(),
        },
        0,
    );
}

#[test]
fn the_session_log_takes_both_directions_beside_the_automatic_one() {
    let mut rig = Rig::new();
    let auto = Shared::default();
    let session = Shared::default();
    rig.start(auto.clone(), 0);
    start_session(&mut rig, session.clone(), "s.tlog");
    assert_eq!(rig.state.session_log.recording.as_deref(), Some("s.tlog"));
    let frame = v2_frame(0);
    rig.received(&frame, 0, 5);
    rig.sent(&frame, 0, 6);
    let mut want = record(5, &frame);
    want.extend(record(6, &frame));
    assert_eq!(session.bytes(), want);
    assert_eq!(auto.bytes(), want);
    assert_eq!(rig.state.log.frames, 2);
    assert_eq!(rig.state.log.recording.as_deref(), Some("flight.tlog"));
}

#[test]
fn stopping_the_session_log_releases_the_file_and_names_it_closed() {
    let mut rig = Rig::new();
    let session = Shared::default();
    start_session(&mut rig, session.clone(), "s.tlog");
    rig.received(&v2_frame(0), 0, 5);
    rig.command(&Command::StopSessionLog, 10);
    assert_eq!(rig.state.session_log.recording, None);
    assert_eq!(rig.state.session_log.closed.as_deref(), Some("s.tlog"));
    assert_eq!(session.dropped.load(Ordering::SeqCst), 1);
    assert!(session.flushes() >= 1);
    let before = session.bytes();
    rig.received(&v2_frame(1), 20, 7);
    assert_eq!(session.bytes(), before);
}

#[test]
fn a_failing_session_log_closes_itself_and_leaves_the_automatic_one_alone() {
    let mut rig = Rig::new();
    let auto = Shared::default();
    rig.start(auto.clone(), 0);
    let dropped = Arc::new(AtomicUsize::new(0));
    start_session(
        &mut rig,
        Failing {
            ok: 0,
            dropped: Arc::clone(&dropped),
        },
        "s.tlog",
    );
    rig.received(&v2_frame(0), 0, 5);
    assert_eq!(dropped.load(Ordering::SeqCst), 1);
    assert_eq!(rig.state.session_log.recording, None);
    assert_eq!(rig.state.session_log.closed.as_deref(), Some("s.tlog"));
    assert_eq!(rig.state.session_log.error.as_deref(), Some("disk full"));
    assert_eq!(rig.state.log.recording.as_deref(), Some("flight.tlog"));
    assert_eq!(auto.bytes().len(), 8 + v2_frame(0).len());
}

// ---- the recorder ----

#[test]
fn start_sets_the_state() {
    let mut rig = Rig::new();
    rig.state.log = LogState {
        recording: None,
        frames: 9,
        bytes: 99,
        error: Some("old".into()),
    };
    rig.start(Shared::default(), 0);
    assert!(rig.changed);
    assert_eq!(
        rig.state.log,
        LogState {
            recording: Some("flight.tlog".into()),
            frames: 0,
            bytes: 0,
            error: None,
        }
    );
}

#[test]
fn a_sink_that_was_taken_gives_an_error_and_records_nothing() {
    let mut rig = Rig::new();
    let file = Shared::default();
    let sink = LogSink::new(file.clone());
    drop(sink.take());
    rig.command(
        &Command::StartLog {
            sink,
            label: "x".into(),
        },
        0,
    );
    assert!(rig.changed);
    assert_eq!(rig.state.log.recording, None);
    assert!(rig.state.log.error.is_some());
    rig.received(&v2_frame(0), 10, 1);
    assert!(file.bytes().is_empty());
    assert_eq!((rig.state.log.frames, rig.state.log.bytes), (0, 0));
}

#[test]
fn received_and_sent_frames_are_written_and_counted() {
    let mut rig = Rig::new();
    let file = Shared::default();
    rig.start(file.clone(), 0);
    let (a, b) = (v2_frame(0), v2_frame(1));
    rig.received(&a, 10, 111);
    assert!(rig.changed);
    rig.sent(&b, 20, 222);
    assert!(rig.changed);
    let mut want = record(111, &a);
    want.extend(record(222, &b));
    assert_eq!(file.bytes(), want);
    assert_eq!(rig.state.log.frames, 2);
    assert_eq!(rig.state.log.bytes, want.len() as u64);
}

#[test]
fn every_body_is_written_from_its_raw_bytes() {
    let mut rig = Rig::new();
    let file = Shared::default();
    rig.start(file.clone(), 0);
    let raw = v2_frame(0);
    for body in [
        Body::Known(heartbeat()),
        Body::Unknown,
        Body::Undecodable("x".into()),
    ] {
        let mut frame = frame_of(&raw);
        frame.body = body;
        rig.run(0, 5, |recorder, cx| recorder.on_frame(&frame, cx));
    }
    assert_eq!(file.bytes().len(), 3 * (8 + raw.len()));
}

#[test]
fn nothing_is_written_before_start_or_after_stop() {
    let mut rig = Rig::new();
    rig.received(&v2_frame(0), 0, 1);
    assert!(!rig.changed);
    let file = Shared::default();
    rig.start(file.clone(), 0);
    rig.received(&v2_frame(1), 1, 2);
    let written = file.bytes();
    rig.stop(2);
    assert!(rig.changed);
    assert_eq!(rig.state.log.recording, None);
    assert_eq!(rig.state.log.frames, 1, "counters kept");
    assert_eq!(rig.state.log.bytes, written.len() as u64);
    rig.received(&v2_frame(2), 3, 3);
    rig.sent(&v2_frame(3), 3, 3);
    assert!(!rig.changed);
    assert_eq!(file.bytes(), written);
}

#[test]
fn stop_without_a_recording_changes_nothing() {
    let mut rig = Rig::new();
    rig.stop(0);
    assert!(!rig.changed);
}

#[test]
fn a_write_error_ends_the_recording_and_drops_the_writer() {
    let mut rig = Rig::new();
    let dropped = Arc::new(AtomicUsize::new(0));
    let frame = v2_frame(0);
    rig.start(
        Failing {
            ok: 8 + frame.len(),
            dropped: dropped.clone(),
        },
        0,
    );
    rig.received(&frame, 0, 1);
    assert_eq!(rig.state.log.recording.as_deref(), Some("flight.tlog"));
    assert_eq!(dropped.load(Ordering::SeqCst), 0);
    rig.received(&frame, 1, 2);
    assert!(rig.changed);
    assert_eq!(rig.state.log.recording, None);
    assert!(
        rig.state
            .log
            .error
            .as_deref()
            .unwrap()
            .contains("disk full")
    );
    assert_eq!(dropped.load(Ordering::SeqCst), 1);
    assert_eq!(rig.state.log.frames, 1, "the failed one is not counted");
    rig.received(&frame, 2, 3);
    assert!(!rig.changed);
}

#[test]
fn flushes_once_a_second_when_something_was_written() {
    let mut rig = Rig::new();
    let file = Shared::default();
    rig.start(file.clone(), 5000);
    rig.tick(5500);
    rig.tick(6000);
    assert_eq!(file.flushes(), 0, "nothing written");
    rig.received(&v2_frame(0), 6100, 1);
    rig.tick(6200);
    assert_eq!(file.flushes(), 1, "a second since the start");
    rig.received(&v2_frame(5), 6300, 1);
    rig.tick(7199);
    assert_eq!(file.flushes(), 1);
    rig.tick(7200);
    assert_eq!(file.flushes(), 2);
    rig.tick(9000);
    assert_eq!(file.flushes(), 2, "nothing written since");
    rig.sent(&v2_frame(1), 9100, 2);
    rig.tick(9200);
    assert_eq!(file.flushes(), 3, "a second passed since the last flush");
    rig.sent(&v2_frame(2), 9300, 3);
    rig.tick(10_199);
    assert_eq!(file.flushes(), 3, "less than a second since the last flush");
    rig.tick(10_200);
    assert_eq!(file.flushes(), 4);
}

#[test]
fn stop_flushes_and_closes() {
    let mut rig = Rig::new();
    let file = Shared::default();
    rig.start(file.clone(), 0);
    rig.received(&v2_frame(0), 10, 1);
    assert_eq!(file.flushes(), 0);
    rig.stop(20);
    assert_eq!(file.flushes(), 1);
    assert_eq!(file.dropped.load(Ordering::SeqCst), 1, "writer dropped");
}

#[test]
fn a_new_start_flushes_and_replaces_the_old_one() {
    let mut rig = Rig::new();
    let first = Shared::default();
    let second = Shared::default();
    rig.start(first.clone(), 0);
    rig.received(&v2_frame(0), 10, 1);
    rig.start(second.clone(), 20);
    assert_eq!(first.flushes(), 1);
    assert_eq!(rig.state.log.frames, 0);
    rig.received(&v2_frame(1), 30, 2);
    assert_eq!(first.bytes().len(), 8 + v2_frame(0).len());
    assert_eq!(second.bytes().len(), 8 + v2_frame(1).len());
}

#[test]
fn dropping_the_recorder_flushes() {
    let mut rig = Rig::new();
    let file = Shared::default();
    rig.start(file.clone(), 0);
    rig.received(&v2_frame(0), 10, 1);
    assert_eq!(file.flushes(), 0);
    drop(rig);
    assert_eq!(file.flushes(), 1);
}

// ---- the reader ----

#[test]
fn reads_v2_v1_and_signed_records() {
    let frames = [
        v2_frame(0),
        v1_frame(0),
        v1_frame(9),
        v2_signed(0),
        v2_signed(17),
        v2_frame(1),
    ];
    let mut file = Vec::new();
    for (n, frame) in frames.iter().enumerate() {
        file.extend(record(1000 + n as u64, frame));
    }
    let got: Vec<TlogRecord> = read_all(file).into_iter().map(Result::unwrap).collect();
    let want: Vec<TlogRecord> = frames
        .iter()
        .enumerate()
        .map(|(n, frame)| TlogRecord {
            time_us: 1000 + n as u64,
            frame: frame.clone(),
        })
        .collect();
    assert_eq!(got, want);
}

#[test]
fn the_end_on_a_record_boundary_ends_quietly() {
    assert!(read_all(Vec::new()).is_empty());
    let mut reader = TlogReader::new(Cursor::new(record(1, &v2_frame(0))));
    assert!(reader.next().unwrap().is_ok());
    assert!(reader.next().is_none());
    assert!(reader.next().is_none());
}

#[test]
fn a_file_cut_inside_a_record_gives_one_unexpected_eof() {
    let whole = record(7, &v2_frame(0));
    let good = record(6, &v1_frame(2));
    // Every cut point inside the second record: in the time, right after it, in the header, in the body.
    for cut in 1..whole.len() {
        let mut file = good.clone();
        file.extend_from_slice(&whole[..cut]);
        let mut reader = TlogReader::new(Cursor::new(file));
        assert!(reader.next().unwrap().is_ok(), "cut {cut}");
        let error = reader.next().unwrap().unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::UnexpectedEof, "cut {cut}");
        assert!(reader.next().is_none(), "cut {cut}");
    }
}

#[test]
fn a_signed_record_cut_inside_the_signature_is_an_error() {
    let whole = record(7, &v2_signed(4));
    let mut reader = TlogReader::new(Cursor::new(whole[..whole.len() - 1].to_vec()));
    let error = reader.next().unwrap().unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::UnexpectedEof);
}

#[test]
fn a_wrong_first_byte_gives_one_invalid_data_with_the_offset() {
    let first = record(1, &v2_frame(0));
    let mut file = first.clone();
    file.extend(record(2, &[0x55, 1, 2, 3]));
    file.extend(record(3, &v2_frame(1)));
    let mut reader = TlogReader::new(Cursor::new(file));
    assert!(reader.next().unwrap().is_ok());
    let error = reader.next().unwrap().unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::InvalidData);
    let offset = first.len() + 8;
    assert!(
        error.to_string().contains(&offset.to_string()),
        "{error} should hold {offset}"
    );
    assert!(reader.next().is_none(), "no resynchronisation");
}

#[test]
fn a_source_error_is_returned_once() {
    struct Broken;
    impl Read for Broken {
        fn read(&mut self, _: &mut [u8]) -> io::Result<usize> {
            Err(io::Error::other("unplugged"))
        }
    }
    let mut reader = TlogReader::new(Broken);
    assert_eq!(reader.next().unwrap().unwrap_err().to_string(), "unplugged");
    assert!(reader.next().is_none());
}

#[test]
fn a_source_that_returns_small_pieces_reads_the_same() {
    struct OneByte(Cursor<Vec<u8>>);
    impl Read for OneByte {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            let n = 1.min(buf.len());
            self.0.read(&mut buf[..n])
        }
    }
    let mut file = record(1, &v2_frame(0));
    file.extend(record(2, &v1_frame(5)));
    let got: Vec<_> = TlogReader::new(OneByte(Cursor::new(file.clone())))
        .map(Result::unwrap)
        .collect();
    assert_eq!(got.len(), 2);
    assert_eq!(got[1].frame, v1_frame(5));
}

// ---- recorder and reader together ----

#[test]
fn what_was_recorded_is_read_back_in_order() {
    let mut rig = Rig::new();
    let file = Shared::default();
    rig.start(file.clone(), 0);
    let mut want = Vec::new();
    for n in 0..20u8 {
        let frame = v2_frame(n);
        let time = 1_700_000_000_000_000 + u64::from(n) * 1234;
        if n % 3 == 0 {
            rig.sent(&frame, 0, time);
        } else {
            rig.received(&frame, 0, time);
        }
        want.push(TlogRecord {
            time_us: time,
            frame,
        });
    }
    rig.stop(1);
    let got: Vec<TlogRecord> = read_all(file.bytes())
        .into_iter()
        .map(Result::unwrap)
        .collect();
    assert_eq!(got, want);
}

// ---- replay ----

fn file_of(times_us: &[u64]) -> Vec<u8> {
    let mut file = Vec::new();
    for (n, &time) in times_us.iter().enumerate() {
        file.extend(record(time, &v2_frame(n as u8)));
    }
    file
}

/// Reads to the end, with the given buffer size; the bytes and the number of `TimedOut` seen.
fn drain(reader: &mut dyn Read, buffer: usize) -> (Vec<u8>, usize) {
    let mut bytes = Vec::new();
    let mut timeouts = 0;
    let mut buf = vec![0u8; buffer];
    loop {
        match reader.read(&mut buf) {
            Ok(0) => return (bytes, timeouts),
            Ok(n) => bytes.extend_from_slice(&buf[..n]),
            Err(error) if error.kind() == io::ErrorKind::TimedOut => timeouts += 1,
            Err(error) => panic!("unexpected {error}"),
        }
    }
}

#[test]
fn fast_replay_gives_the_frames_through_the_deframer() {
    let (mut reader, _) = replay(
        Box::new(Cursor::new(file_of(&[0, 10_000_000, 20_000_000, 5]))),
        ReplaySpeed::Fast,
    );
    let started = Instant::now();
    let (bytes, timeouts) = drain(&mut reader, 4096);
    assert!(started.elapsed() < Duration::from_millis(100));
    assert_eq!(timeouts, 0);
    let mut deframer = Deframer::new();
    let mut frames = Vec::new();
    deframer.push(&bytes, &mut frames);
    let got: Vec<Vec<u8>> = frames.into_iter().map(|f| f.raw).collect();
    let want: Vec<Vec<u8>> = (0..4u8).map(v2_frame).collect();
    assert_eq!(got, want);
    assert_eq!(deframer.stats().discarded_bytes, 0);
}

#[test]
fn end_is_ok_zero_every_time() {
    let (mut reader, _) = replay(Box::new(Cursor::new(file_of(&[0]))), ReplaySpeed::Fast);
    let mut buf = [0u8; 100];
    assert!(reader.read(&mut buf).unwrap() > 0);
    for _ in 0..3 {
        assert_eq!(reader.read(&mut buf).unwrap(), 0);
    }
    let (mut reader, _) = replay(Box::new(Cursor::new(Vec::new())), ReplaySpeed::RealTime);
    for _ in 0..3 {
        assert_eq!(reader.read(&mut buf).unwrap(), 0);
    }
}

#[test]
fn a_small_buffer_loses_no_bytes() {
    for speed in [ReplaySpeed::Fast, ReplaySpeed::RealTime] {
        let (mut reader, _) = replay(Box::new(Cursor::new(file_of(&[0, 0, 0]))), speed);
        let (bytes, _) = drain(&mut reader, 7);
        let want: Vec<u8> = (0..3u8).flat_map(v2_frame).collect();
        assert_eq!(bytes, want, "{speed:?}");
    }
}

#[test]
fn real_time_replay_keeps_the_pace_and_times_out_while_waiting() {
    let (mut reader, _) = replay(
        Box::new(Cursor::new(file_of(&[
            1_700_000_000_000_000,
            1_700_000_000_200_000,
            1_700_000_000_400_000,
        ]))),
        ReplaySpeed::RealTime,
    );
    let started = Instant::now();
    let mut first = [0u8; 4096];
    assert!(reader.read(&mut first).unwrap() > 0);
    assert!(
        started.elapsed() < Duration::from_millis(50),
        "first is due at once"
    );
    let (bytes, timeouts) = drain(&mut reader, 4096);
    let elapsed = started.elapsed();
    assert!(
        elapsed >= Duration::from_millis(250) && elapsed <= Duration::from_millis(550),
        "took {elapsed:?}"
    );
    assert!(timeouts >= 1, "timeouts: {timeouts}");
    assert_eq!(bytes.len(), 2 * v2_frame(0).len());
}

#[test]
fn a_timestamp_that_goes_backwards_is_due_at_once() {
    let (mut reader, _) = replay(
        Box::new(Cursor::new(file_of(&[5_000_000, 1_000_000, 0]))),
        ReplaySpeed::RealTime,
    );
    let started = Instant::now();
    let (bytes, timeouts) = drain(&mut reader, 4096);
    assert!(started.elapsed() < Duration::from_millis(100));
    assert_eq!(timeouts, 0);
    assert_eq!(bytes.len(), 3 * v2_frame(0).len());
}

#[test]
fn a_frame_less_than_100_ms_away_is_waited_for_without_a_timeout() {
    let (mut reader, _) = replay(
        Box::new(Cursor::new(file_of(&[0, 60_000]))),
        ReplaySpeed::RealTime,
    );
    let mut buf = [0u8; 4096];
    assert!(reader.read(&mut buf).unwrap() > 0);
    let started = Instant::now();
    assert!(reader.read(&mut buf).unwrap() > 0);
    assert!(started.elapsed() >= Duration::from_millis(50));
}

#[test]
fn a_truncated_file_gives_the_frames_then_an_error_then_the_end() {
    let mut file = file_of(&[0, 0]);
    file.extend_from_slice(&record(0, &v2_frame(2))[..15]);
    let (mut reader, _) = replay(Box::new(Cursor::new(file)), ReplaySpeed::Fast);
    let mut buf = [0u8; 4096];
    let one = v2_frame(0).len();
    assert_eq!(reader.read(&mut buf).unwrap(), one);
    assert_eq!(reader.read(&mut buf).unwrap(), one);
    let error = reader.read(&mut buf).unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::UnexpectedEof);
    assert_eq!(reader.read(&mut buf).unwrap(), 0);
    assert_eq!(reader.read(&mut buf).unwrap(), 0);
}

#[test]
fn a_file_of_garbage_is_an_error() {
    let (mut reader, _) = replay(
        Box::new(Cursor::new(vec![0x11u8; 64])),
        ReplaySpeed::RealTime,
    );
    let mut buf = [0u8; 4096];
    let error = reader.read(&mut buf).unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::InvalidData);
    assert_eq!(reader.read(&mut buf).unwrap(), 0);
}

#[test]
fn the_writer_accepts_and_discards() {
    let (_, mut writer) = replay(Box::new(Cursor::new(Vec::new())), ReplaySpeed::Fast);
    assert_eq!(writer.write(&[1, 2, 3]).unwrap(), 3);
    writer.write_all(&[0; 1000]).unwrap();
    writer.flush().unwrap();
}
