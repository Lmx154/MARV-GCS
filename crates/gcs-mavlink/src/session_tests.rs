use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use mavlink::{MavHeader, MavlinkReader, MavlinkVersion};

use super::*;
use crate::dialect::{HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavState, MavType};
use crate::test_pipe::{DropFlag, PipeReader, PipeWriter, pipe};
use crate::{LinkStatus, Message};

fn config(timeout_ms: u64) -> SessionConfig {
    SessionConfig {
        heartbeat_timeout: Duration::from_millis(timeout_ms),
        access: Access::Configure,
        ..SessionConfig::default()
    }
}

fn nop() -> Box<dyn Fn() + Send + Sync> {
    Box::new(|| {})
}

fn wait_until(limit: Duration, mut condition: impl FnMut() -> bool) -> bool {
    let start = Instant::now();
    while start.elapsed() < limit {
        if condition() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    condition()
}

fn heartbeat(mavtype: MavType) -> Message {
    Message::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

/// A frame written by the `mavlink` crate itself, not by our encoder.
fn library_frame(system_id: u32, sequence: u8, message: &Message) -> Vec<u8> {
    let mut bytes = Vec::new();
    let header = MavHeader {
        system_id,
        component_id: 1,
        sequence,
    };
    mavlink::write_v2_msg(&mut bytes, header, message).expect("encode");
    bytes
}

type Received = Arc<Mutex<Vec<(MavHeader, Message)>>>;

/// A vehicle built from the `mavlink` crate: heartbeat every 100 ms, records what it receives.
struct FakeVehicle {
    stop: Arc<AtomicBool>,
    silent: Arc<AtomicBool>,
    received: Received,
    threads: Vec<JoinHandle<()>>,
}

impl FakeVehicle {
    fn spawn(to_session: PipeWriter, from_session: PipeReader) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let silent = Arc::new(AtomicBool::new(false));
        let received: Received = Arc::default();
        let sender = {
            let (stop, silent) = (Arc::clone(&stop), Arc::clone(&silent));
            std::thread::spawn(move || {
                let mut writer = to_session;
                let mut sequence = 0u8;
                while !stop.load(Ordering::Relaxed) {
                    if !silent.load(Ordering::Relaxed) {
                        let header = MavHeader {
                            system_id: 1,
                            component_id: 1,
                            sequence,
                        };
                        sequence = sequence.wrapping_add(1);
                        let message = heartbeat(MavType::MAV_TYPE_QUADROTOR);
                        if mavlink::write_v2_msg(&mut writer, header, &message).is_err() {
                            return;
                        }
                    }
                    std::thread::sleep(Duration::from_millis(100));
                }
            })
        };
        let receiver = {
            let (stop, received) = (Arc::clone(&stop), Arc::clone(&received));
            std::thread::spawn(move || {
                let mut reader = MavlinkReader::new(from_session);
                while !stop.load(Ordering::Relaxed) {
                    match reader.read_message::<Message>(MavlinkVersion::V2) {
                        Ok(item) => received.lock().unwrap().push(item),
                        Err(mavlink::error::MessageReadError::Io(error))
                            if error.kind() == io::ErrorKind::TimedOut => {}
                        Err(_) => return,
                    }
                }
            })
        };
        Self {
            stop,
            silent,
            received,
            threads: vec![sender, receiver],
        }
    }

    fn ground_station_heartbeats(&self) -> Vec<MavHeader> {
        self.received
            .lock()
            .unwrap()
            .iter()
            .filter(|(_, message)| {
                matches!(message, Message::HEARTBEAT(data) if data.mavtype == MavType::MAV_TYPE_GCS)
            })
            .map(|(header, _)| *header)
            .collect()
    }

    /// The header of every frame the session sent, in the order it arrived.
    fn ground_station_frames(&self) -> Vec<MavHeader> {
        self.received
            .lock()
            .unwrap()
            .iter()
            .map(|(header, _)| *header)
            .collect()
    }

    fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for thread in self.threads.drain(..) {
            thread.join().unwrap();
        }
    }
}

struct Rig {
    session: Session,
    vehicle: FakeVehicle,
}

fn rig(timeout_ms: u64) -> Rig {
    rig_with(config(timeout_ms))
}

fn rig_with(config: SessionConfig) -> Rig {
    let (vehicle_tx, session_rx) = pipe();
    let (session_tx, vehicle_rx) = pipe();
    let session = Session::start(Box::new(session_rx), Box::new(session_tx), config, nop());
    Rig {
        session,
        vehicle: FakeVehicle::spawn(vehicle_tx, vehicle_rx),
    }
}

#[test]
fn config_defaults() {
    let config = SessionConfig::default();
    assert_eq!(config.system_id, 255);
    assert_eq!(config.component_id, 190);
    assert_eq!(config.heartbeat_timeout, Duration::from_secs(5));
    assert_eq!(VehicleState::default().link.status, LinkStatus::Connecting);
}

#[test]
fn end_to_end_connects_and_exchanges_heartbeats() {
    let mut rig = rig(3000);
    let started = Instant::now();
    assert!(wait_until(Duration::from_millis(300), || {
        rig.session.state().link.status == LinkStatus::Connected
    }));
    let vehicle = rig.session.state().link.vehicle.clone().unwrap();
    assert_eq!(vehicle.system_id, 1);
    assert_eq!(vehicle.component_id, 1);
    assert_eq!(vehicle.mav_type, "MAV_TYPE_QUADROTOR");
    assert_eq!(vehicle.autopilot, "MAV_AUTOPILOT_ARDUPILOTMEGA");

    std::thread::sleep(Duration::from_millis(3200).saturating_sub(started.elapsed()));
    let seen = rig.vehicle.ground_station_heartbeats();
    assert!(
        (3..=4).contains(&seen.len()),
        "ground-station heartbeats in 3.2 s: {}",
        seen.len()
    );
    for header in &seen {
        assert_eq!(header.system_id, 255);
        assert_eq!(header.component_id, 190);
    }
    // One encoder numbers everything the session sends: heartbeats and what the other services
    // send (parameter requests) share one consecutive sequence.
    let all = rig.vehicle.ground_station_frames();
    assert!(all.len() >= seen.len());
    for pair in all.windows(2) {
        assert_eq!(pair[1].sequence, pair[0].sequence.wrapping_add(1));
    }
    let state = rig.session.state();
    assert!(state.link.tx_frames >= all.len() as u64);
    assert!(state.link.rx.frames >= 20, "rx: {:?}", state.link.rx);
    assert!(state.link.last_heartbeat.is_some());
    drop(state);
    rig.vehicle.stop();
}

#[test]
fn lost_after_timeout_and_connected_again() {
    let mut rig = rig(300);
    assert!(wait_until(Duration::from_millis(300), || {
        rig.session.state().link.status == LinkStatus::Connected
    }));
    rig.vehicle.silent.store(true, Ordering::Relaxed);
    let silent_since = Instant::now();
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(rig.session.state().link.status, LinkStatus::Connected);
    assert!(wait_until(Duration::from_millis(1000), || {
        rig.session.state().link.status == LinkStatus::Lost
    }));
    assert!(silent_since.elapsed() >= Duration::from_millis(200));
    assert!(rig.session.state().link.vehicle.is_some());

    rig.vehicle.silent.store(false, Ordering::Relaxed);
    assert!(wait_until(Duration::from_millis(500), || {
        rig.session.state().link.status == LinkStatus::Connected
    }));
    rig.vehicle.stop();
}

#[test]
fn closed_when_the_pipe_is_closed() {
    let mut rig = rig(3000);
    assert!(wait_until(Duration::from_millis(300), || {
        rig.session.state().link.status == LinkStatus::Connected
    }));
    rig.vehicle.stop();
    assert!(wait_until(Duration::from_millis(1000), || {
        matches!(rig.session.state().link.status, LinkStatus::Closed(_))
    }));
    let LinkStatus::Closed(text) = rig.session.state().link.status.clone() else {
        unreachable!()
    };
    assert!(text.contains("closed"), "text: {text}");
}

struct FailingReader;

impl Read for FailingReader {
    fn read(&mut self, _buf: &mut [u8]) -> io::Result<usize> {
        Err(io::Error::other("cable pulled"))
    }
}

#[test]
fn read_error_closes_with_the_reason() {
    let (writer, _keep) = pipe();
    let (writer, written_dropped) = DropFlag::new(writer);
    let session = Session::start(
        Box::new(FailingReader),
        Box::new(writer),
        config(3000),
        nop(),
    );
    assert!(wait_until(Duration::from_millis(500), || {
        matches!(session.state().link.status, LinkStatus::Closed(_))
    }));
    let LinkStatus::Closed(text) = session.state().link.status.clone() else {
        unreachable!()
    };
    assert!(text.contains("cable pulled"), "text: {text}");
    assert!(
        wait_until(Duration::from_millis(500), || written_dropped
            .load(Ordering::SeqCst)),
        "session thread ended and dropped the writer"
    );
}

struct FailingWriter;

impl Write for FailingWriter {
    fn write(&mut self, _buf: &[u8]) -> io::Result<usize> {
        Err(io::Error::other("port gone"))
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[test]
fn write_error_closes_and_ends_both_threads() {
    let (_keep, reader) = pipe();
    let (reader, reader_dropped) = DropFlag::new(reader);
    let session = Session::start(
        Box::new(reader),
        Box::new(FailingWriter),
        config(3000),
        nop(),
    );
    assert!(wait_until(Duration::from_millis(500), || {
        matches!(session.state().link.status, LinkStatus::Closed(_))
    }));
    let LinkStatus::Closed(text) = session.state().link.status.clone() else {
        unreachable!()
    };
    assert!(text.contains("port gone"), "text: {text}");
    assert!(
        wait_until(Duration::from_millis(500), || reader_dropped
            .load(Ordering::SeqCst)),
        "reader thread ended and dropped the reader"
    );
}

#[test]
fn drop_stops_threads_and_drops_both_ends() {
    let (_keep_tx, reader) = pipe();
    let (writer, _keep_rx) = pipe();
    let (reader, reader_dropped) = DropFlag::new(reader);
    let (writer, writer_dropped) = DropFlag::new(writer);
    let session = Session::start(Box::new(reader), Box::new(writer), config(3000), nop());
    std::thread::sleep(Duration::from_millis(150));
    assert!(!reader_dropped.load(Ordering::SeqCst));
    assert!(!writer_dropped.load(Ordering::SeqCst));
    let started = Instant::now();
    drop(session);
    assert!(
        started.elapsed() < Duration::from_millis(300),
        "drop took {:?}",
        started.elapsed()
    );
    assert!(reader_dropped.load(Ordering::SeqCst));
    assert!(writer_dropped.load(Ordering::SeqCst));
}

#[test]
fn rx_statistics_mirror_the_deframer() {
    let (mut to_session, reader) = pipe();
    let (writer, _keep_rx) = pipe();
    let session = Session::start(Box::new(reader), Box::new(writer), config(3000), nop());
    let mut bytes = vec![0x00, 0x01, 0x02];
    bytes.extend(library_frame(1, 0, &heartbeat(MavType::MAV_TYPE_QUADROTOR)));
    to_session.write_all(&bytes).unwrap();
    assert!(wait_until(Duration::from_millis(100), || {
        let rx = session.state().link.rx;
        rx.frames == 1 && rx.discarded_bytes == 3
    }));
}

#[test]
fn wake_follows_a_vehicle_heartbeat() {
    let (mut to_session, reader) = pipe();
    let (writer, _keep_rx) = pipe();
    let wakes = Arc::new(AtomicUsize::new(0));
    let counter = Arc::clone(&wakes);
    let session = Session::start(
        Box::new(reader),
        Box::new(writer),
        config(3000),
        Box::new(move || {
            counter.fetch_add(1, Ordering::SeqCst);
        }),
    );
    std::thread::sleep(Duration::from_millis(200));
    let before = wakes.load(Ordering::SeqCst);
    to_session
        .write_all(&library_frame(
            1,
            0,
            &heartbeat(MavType::MAV_TYPE_QUADROTOR),
        ))
        .unwrap();
    assert!(wait_until(Duration::from_millis(200), || {
        wakes.load(Ordering::SeqCst) > before
    }));
    assert_eq!(session.state().link.status, LinkStatus::Connected);
}

#[test]
fn wake_is_limited_under_load() {
    let (mut to_session, reader) = pipe();
    let (writer, _keep_rx) = pipe();
    let wakes = Arc::new(AtomicUsize::new(0));
    let counter = Arc::clone(&wakes);
    let session = Session::start(
        Box::new(reader),
        Box::new(writer),
        config(3000),
        Box::new(move || {
            counter.fetch_add(1, Ordering::SeqCst);
        }),
    );
    std::thread::sleep(Duration::from_millis(200));
    let before = wakes.load(Ordering::SeqCst);
    let frame = library_frame(1, 0, &heartbeat(MavType::MAV_TYPE_QUADROTOR));
    let batch: Vec<u8> = frame
        .iter()
        .copied()
        .cycle()
        .take(frame.len() * 10)
        .collect();
    let started = Instant::now();
    for step in 1..=100u32 {
        to_session.write_all(&batch).unwrap();
        std::thread::sleep((Duration::from_millis(10) * step).saturating_sub(started.elapsed()));
    }
    let woken = wakes.load(Ordering::SeqCst) - before;
    let elapsed = started.elapsed();
    assert!(elapsed < Duration::from_millis(1100), "took {elapsed:?}");
    assert!((15..=40).contains(&woken), "wakes in {elapsed:?}: {woken}");
    assert!(wait_until(Duration::from_millis(200), || {
        session.state().link.rx.frames >= 1000
    }));
}

#[test]
fn poisoned_state_can_still_be_read() {
    let (_keep_tx, reader) = pipe();
    let (writer, _keep_rx) = pipe();
    let session = Session::start(Box::new(reader), Box::new(writer), config(3000), nop());
    let state = Arc::clone(&session.state);
    let _ = std::thread::spawn(move || {
        let _guard = state.lock().unwrap();
        panic!("poison the mutex");
    })
    .join();
    assert!(session.state.is_poisoned());
    assert_eq!(session.state().link.status, LinkStatus::Connecting);
}

#[test]
fn threads_are_named() {
    let names = Arc::new(Mutex::new(Vec::<String>::new()));
    struct Spy(Arc<Mutex<Vec<String>>>);
    impl Read for Spy {
        fn read(&mut self, _buf: &mut [u8]) -> io::Result<usize> {
            let name = std::thread::current().name().unwrap_or("").to_owned();
            let mut names = self.0.lock().unwrap();
            names.push(name);
            if names.iter().any(|n| n == "gcs-session") {
                return Ok(0);
            }
            drop(names);
            std::thread::sleep(Duration::from_millis(10));
            Err(io::ErrorKind::TimedOut.into())
        }
    }
    impl Write for Spy {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            let name = std::thread::current().name().unwrap_or("").to_owned();
            self.0.lock().unwrap().push(name);
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    let session = Session::start(
        Box::new(Spy(Arc::clone(&names))),
        Box::new(Spy(Arc::clone(&names))),
        config(3000),
        nop(),
    );
    assert!(wait_until(Duration::from_millis(500), || {
        matches!(session.state().link.status, LinkStatus::Closed(_))
    }));
    let names = names.lock().unwrap();
    assert!(names.iter().any(|n| n == "gcs-reader"), "{names:?}");
    assert!(names.iter().any(|n| n == "gcs-session"), "{names:?}");
}

fn start_with(
    services: Vec<Box<dyn Service>>,
    reader: impl Read + Send + 'static,
    writer: impl Write + Send + 'static,
    wake: Box<dyn Fn() + Send + Sync>,
) -> Session {
    Session::start_with_services(
        Box::new(reader),
        Box::new(writer),
        config(3000),
        wake,
        services,
    )
}

#[test]
fn wake_without_any_input_after_the_first_heartbeat_was_sent() {
    let (_keep_tx, reader) = pipe();
    let (writer, _keep_rx) = pipe();
    let wakes = Arc::new(AtomicUsize::new(0));
    let counter = Arc::clone(&wakes);
    let session = Session::start(
        Box::new(reader),
        Box::new(writer),
        config(3000),
        Box::new(move || {
            counter.fetch_add(1, Ordering::SeqCst);
        }),
    );
    assert!(
        wait_until(Duration::from_millis(1500), || wakes.load(Ordering::SeqCst)
            >= 1),
        "no wake without input"
    );
    assert!(session.state().link.tx_frames >= 1);
}

struct PanicOnFrame;

impl Service for PanicOnFrame {
    fn on_frame(&mut self, _frame: &Frame, _cx: &mut Ctx) {
        panic!("boom in a service");
    }
}

#[test]
fn a_panicking_service_closes_the_link_visibly() {
    let (mut to_session, reader) = pipe();
    let (writer, _keep_rx) = pipe();
    let (reader, reader_dropped) = DropFlag::new(reader);
    let (writer, writer_dropped) = DropFlag::new(writer);
    let wakes = Arc::new(AtomicUsize::new(0));
    let counter = Arc::clone(&wakes);
    let session = start_with(
        vec![Box::new(PanicOnFrame)],
        reader,
        writer,
        Box::new(move || {
            counter.fetch_add(1, Ordering::SeqCst);
        }),
    );
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(wakes.load(Ordering::SeqCst), 0, "nothing changed yet");
    to_session
        .write_all(&library_frame(
            1,
            0,
            &heartbeat(MavType::MAV_TYPE_QUADROTOR),
        ))
        .unwrap();
    assert!(wait_until(Duration::from_millis(500), || {
        matches!(session.state().link.status, LinkStatus::Closed(_))
    }));
    let LinkStatus::Closed(text) = session.state().link.status.clone() else {
        unreachable!()
    };
    assert!(text.starts_with("internal error"), "text: {text}");
    assert!(text.contains("boom in a service"), "text: {text}");
    assert!(
        wait_until(Duration::from_millis(500), || wakes.load(Ordering::SeqCst)
            >= 1),
        "wake after the panic"
    );
    assert!(
        wait_until(Duration::from_millis(500), || writer_dropped
            .load(Ordering::SeqCst)),
        "session thread ended"
    );
    assert!(
        wait_until(Duration::from_millis(500), || reader_dropped
            .load(Ordering::SeqCst)),
        "reader thread ended"
    );
    let started = Instant::now();
    drop(session);
    assert!(
        started.elapsed() < Duration::from_millis(300),
        "drop took {:?}",
        started.elapsed()
    );
}

/// Queues two messages on each of its first five ticks and records what the session reports.
#[derive(Default)]
struct Recorder {
    ticks: usize,
    sent: Arc<Mutex<Vec<Vec<u8>>>>,
    times: Arc<Mutex<Vec<(Duration, u64)>>>,
}

impl Service for Recorder {
    fn on_tick(&mut self, cx: &mut Ctx) {
        self.times.lock().unwrap().push((cx.now, cx.wall_time_us));
        if self.ticks < 5 {
            cx.out.push(heartbeat(MavType::MAV_TYPE_GCS));
            cx.out.push(heartbeat(MavType::MAV_TYPE_ONBOARD_CONTROLLER));
        }
        self.ticks += 1;
    }

    fn on_sent(&mut self, raw: &[u8], _cx: &mut Ctx) {
        self.sent.lock().unwrap().push(raw.to_vec());
    }
}

#[test]
fn on_sent_receives_exactly_the_written_frames_in_order() {
    let (_keep_tx, reader) = pipe();
    let (writer, mut far_end) = pipe();
    let recorder = Recorder::default();
    let sent = Arc::clone(&recorder.sent);
    let session = start_with(vec![Box::new(recorder)], reader, writer, nop());
    assert!(wait_until(Duration::from_millis(1000), || {
        sent.lock().unwrap().len() >= 10
    }));
    std::thread::sleep(Duration::from_millis(100));
    let sent = sent.lock().unwrap().clone();
    assert_eq!(sent.len(), 10);
    for (index, raw) in sent.iter().enumerate() {
        let mut written = vec![0u8; raw.len()];
        far_end.read_exact(&mut written).unwrap();
        assert_eq!(&written, raw, "frame {index}");
    }
    assert_eq!(session.state().link.tx_frames, 10);
    assert!(matches!(
        far_end.read(&mut [0u8; 64]),
        Err(error) if error.kind() == io::ErrorKind::TimedOut
    ));
}

#[test]
fn ctx_times_are_session_relative_and_wall_clock() {
    let (_keep_tx, reader) = pipe();
    let (writer, _keep_rx) = pipe();
    let recorder = Recorder::default();
    let times = Arc::clone(&recorder.times);
    let session = start_with(vec![Box::new(recorder)], reader, writer, nop());
    assert!(wait_until(Duration::from_millis(500), || {
        times.lock().unwrap().len() >= 2
    }));
    std::thread::sleep(Duration::from_millis(150));
    let times = times.lock().unwrap().clone();
    let wall_now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64;
    assert!(times[0].0 < Duration::from_secs(1), "first: {:?}", times[0]);
    for pair in times.windows(2) {
        assert!(pair[1].0 >= pair[0].0);
    }
    let last = times.last().unwrap().0;
    assert!(
        last >= times[0].0 + Duration::from_millis(100),
        "now grows: {:?} -> {last:?}",
        times[0].0
    );
    assert!(last < Duration::from_secs(1));
    for (_, wall) in &times {
        assert!(
            wall.abs_diff(wall_now) < 5_000_000,
            "wall {wall} vs {wall_now}"
        );
    }
    drop(session);
}

#[test]
fn a_marv_test_frame_is_known_to_the_session_and_a_bad_checksum_is_counted() {
    let (mut to_session, reader) = pipe();
    let (writer, _keep_rx) = pipe();
    let session = Session::start(Box::new(reader), Box::new(writer), config(3000), nop());
    let message = Message::MARV_TEST(crate::dialect::MARV_TEST_DATA {
        seq: 5,
        time_ms: 900,
        value: 15,
        tag: 1,
    });
    let good = library_frame(1, 0, &message);
    let mut bad = library_frame(1, 1, &message);
    let last = bad.len() - 1;
    bad[last] ^= 0xFF;
    to_session
        .write_all(&[good.clone(), bad.clone()].concat())
        .unwrap();
    assert!(wait_until(Duration::from_millis(500), || {
        let rx = session.state().link.rx;
        rx.frames == 1 && rx.crc_errors == 1
    }));
    let state = session.state();
    assert_eq!(state.link.rx.unknown, 0);
    assert_eq!(state.link.rx.discarded_bytes, bad.len() as u64);
    let entry = state.messages.entry("MARV_TEST", 0).expect("listed");
    assert_eq!(entry.count, 1);
    assert_eq!(entry.number("seq"), Some(5.0));
    assert_eq!(entry.number("time_ms"), Some(900.0));
    assert_eq!(entry.number("value"), Some(15.0));
    assert_eq!(entry.number("tag"), Some(1.0));
}

#[test]
fn marv_device_status_gives_one_row_per_device_and_a_wrong_crc_extra_is_counted() {
    let (mut to_session, reader) = pipe();
    let (writer, _keep_rx) = pipe();
    let session = Session::start(Box::new(reader), Box::new(writer), config(3000), nop());
    let status = |device_id, samples| {
        Message::MARV_DEVICE_STATUS(crate::dialect::MARV_DEVICE_STATUS_DATA {
            time_boot_ms: 9_000,
            samples,
            device_id,
            ..Default::default()
        })
    };
    let imu1 = library_frame(
        1,
        0,
        &status(crate::dialect::MarvDevice::MARV_DEVICE_IMU1, 640),
    );
    let baro = library_frame(
        1,
        1,
        &status(crate::dialect::MarvDevice::MARV_DEVICE_BARO, 90),
    );
    let mut wrong = library_frame(
        1,
        2,
        &status(crate::dialect::MarvDevice::MARV_DEVICE_MAG, 5),
    );
    let end = wrong.len() - 2;
    let crc = mavlink::calculate_crc(&wrong[1..end], 230);
    wrong[end..].copy_from_slice(&crc.to_le_bytes());
    to_session
        .write_all(&[imu1, baro, wrong.clone()].concat())
        .unwrap();
    assert!(wait_until(Duration::from_millis(500), || {
        let rx = session.state().link.rx;
        rx.frames == 2 && rx.crc_errors == 1
    }));
    let state = session.state();
    assert_eq!(state.link.rx.undecodable, 0);
    assert_eq!(state.link.rx.discarded_bytes, wrong.len() as u64);
    let rows: Vec<_> = state
        .messages
        .entries
        .iter()
        .filter(|(_, e)| e.name == "MARV_DEVICE_STATUS")
        .map(|(k, e)| (k.instance, e.number("samples")))
        .collect();
    assert_eq!(rows, [(1, Some(640.0)), (3, Some(90.0))]);
}

mod access {
    use super::*;
    use crate::services::params::{Param, ParamType, ParamValue};
    use crate::{Access, Command, Refused};

    const LIMIT: Duration = Duration::from_secs(3);
    const QUIET: Duration = Duration::from_millis(500);

    fn rig_access(access: Access) -> Rig {
        let rig = rig_with(SessionConfig {
            access,
            ..config(5_000)
        });
        assert!(
            wait_until(LIMIT, || rig.session.state().link.status
                == LinkStatus::Connected),
            "the vehicle's heartbeat was not seen"
        );
        rig
    }

    /// One parameter known to the table, as after a download; the vehicle fake never serves one.
    fn put_param(rig: &Rig) {
        let mut state = rig.session.state();
        state.params.params.clear();
        state.params.params.insert(
            "AAA_ONE".into(),
            Param {
                name: "AAA_ONE".into(),
                index: 0,
                kind: ParamType::U8,
                raw: [5, 0, 0, 0],
            },
        );
        state.params.encoding_known = true;
    }

    fn set_one() -> Command {
        Command::SetParam {
            name: "AAA_ONE".into(),
            value: ParamValue::U8(7),
        }
    }

    fn save_to_flash() -> Command {
        Command::SendCommand {
            command: crate::dialect::MavCmd::MAV_CMD_PREFLIGHT_STORAGE,
            params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    }

    /// COMMAND_LONGs for the storage command that the vehicle fake received.
    fn storage_commands(rig: &Rig) -> usize {
        count(rig, |m| {
            matches!(m, Message::COMMAND_LONG(c)
                if c.command == crate::dialect::MavCmd::MAV_CMD_PREFLIGHT_STORAGE)
        })
    }

    fn count(rig: &Rig, wanted: fn(&Message) -> bool) -> usize {
        rig.vehicle
            .received
            .lock()
            .unwrap()
            .iter()
            .filter(|(_, message)| wanted(message))
            .count()
    }

    fn param_sets(rig: &Rig) -> usize {
        count(rig, |m| matches!(m, Message::PARAM_SET(_)))
    }

    fn list_requests(rig: &Rig) -> usize {
        count(rig, |m| matches!(m, Message::PARAM_REQUEST_LIST(_)))
    }

    #[test]
    fn default_config_is_operate_so_an_unconfigured_session_cannot_write() {
        assert_eq!(SessionConfig::default().access, Access::Operate);
    }

    #[test]
    fn an_operate_session_refuses_a_parameter_write_and_writes_nothing() {
        let mut rig = rig_access(Access::Operate);
        put_param(&rig);
        rig.session.send(set_one());
        std::thread::sleep(QUIET);
        assert_eq!(param_sets(&rig), 0);
        assert!(rig.session.state().params.writes.is_empty());
        assert_eq!(
            rig.session.state().link.refused,
            Some(Refused {
                command: "SetParam",
                count: 1
            })
        );
        rig.session.send(set_one());
        assert_eq!(rig.session.state().link.refused.as_ref().unwrap().count, 2);
        assert_eq!(param_sets(&rig), 0);
        rig.vehicle.stop();
    }

    #[test]
    fn negative_control_without_the_gate_the_write_goes_out_on_an_operate_session() {
        let mut rig = rig_access(Access::Operate);
        put_param(&rig);
        rig.session.send_ungated(set_one());
        assert!(
            wait_until(LIMIT, || param_sets(&rig) == 1),
            "the ungated write did not go out"
        );
        rig.session.send(set_one());
        std::thread::sleep(QUIET);
        assert_eq!(param_sets(&rig), 1, "the gated send must add nothing");
        assert_eq!(rig.session.state().link.refused.as_ref().unwrap().count, 1);
        rig.vehicle.stop();
    }

    #[test]
    fn an_operate_session_refuses_a_send_command_and_writes_nothing() {
        let mut rig = rig_access(Access::Operate);
        rig.session.send(save_to_flash());
        std::thread::sleep(QUIET);
        assert_eq!(storage_commands(&rig), 0);
        assert!(rig.session.state().command.is_none());
        assert_eq!(
            rig.session.state().link.refused,
            Some(Refused {
                command: "SendCommand",
                count: 1
            })
        );
        rig.vehicle.stop();
    }

    #[test]
    fn negative_control_without_the_gate_the_command_goes_out_on_an_operate_session() {
        let mut rig = rig_access(Access::Operate);
        rig.session.send_ungated(save_to_flash());
        assert!(
            wait_until(LIMIT, || storage_commands(&rig) >= 1),
            "the ungated command did not go out"
        );
        rig.vehicle.stop();
    }

    fn download_log() -> Command {
        Command::DownloadLog {
            number: 2,
            size: 1000,
            sink: crate::tlog::LogSink::new(Vec::new()),
        }
    }

    /// Log requests (list, data, end) that the vehicle fake received.
    fn log_requests(rig: &Rig) -> usize {
        count(rig, |m| {
            matches!(
                m,
                Message::LOG_REQUEST_LIST(_)
                    | Message::LOG_REQUEST_DATA(_)
                    | Message::LOG_REQUEST_END(_)
            )
        })
    }

    #[test]
    fn an_operate_session_refuses_a_log_download_and_a_cancel_and_writes_nothing() {
        let mut rig = rig_access(Access::Operate);
        rig.session.send(download_log());
        rig.session.send(Command::CancelLogDownload);
        std::thread::sleep(QUIET);
        assert_eq!(log_requests(&rig), 0);
        assert_eq!(rig.session.state().logs.downloads, 0);
        assert!(rig.session.state().logs.download.is_none());
        assert_eq!(
            rig.session.state().link.refused,
            Some(Refused {
                command: "CancelLogDownload",
                count: 2
            })
        );
        rig.vehicle.stop();
    }

    #[test]
    fn negative_control_without_the_gate_the_log_download_goes_out_on_an_operate_session() {
        let mut rig = rig_access(Access::Operate);
        rig.session.send_ungated(download_log());
        assert!(
            wait_until(LIMIT, || count(&rig, |m| matches!(
                m,
                Message::LOG_REQUEST_DATA(_)
            )) >= 1),
            "the ungated download did not go out"
        );
        rig.session.send_ungated(Command::CancelLogDownload);
        assert!(
            wait_until(LIMIT, || count(&rig, |m| matches!(
                m,
                Message::LOG_REQUEST_END(_)
            )) >= 1),
            "the ungated cancel did not go out"
        );
        rig.vehicle.stop();
    }

    #[test]
    fn a_configure_session_sends_the_command() {
        let mut rig = rig_access(Access::Configure);
        rig.session.send(save_to_flash());
        assert!(wait_until(LIMIT, || storage_commands(&rig) >= 1));
        assert_eq!(rig.session.state().link.refused, None);
        rig.vehicle.stop();
    }

    #[test]
    fn a_configure_session_writes_the_parameter() {
        let mut rig = rig_access(Access::Configure);
        put_param(&rig);
        rig.session.send(set_one());
        assert!(wait_until(LIMIT, || param_sets(&rig) >= 1));
        assert_eq!(rig.session.state().link.refused, None);
        rig.vehicle.stop();
    }

    #[test]
    fn an_operate_session_does_not_download_the_list_at_connect_but_a_refresh_does() {
        let mut rig = rig_access(Access::Operate);
        std::thread::sleep(Duration::from_millis(1_500));
        assert_eq!(list_requests(&rig), 0);
        assert_eq!(
            rig.session.state().params.fetch,
            crate::services::params::FetchStatus::Idle
        );
        rig.session.send(Command::RefreshParams);
        assert!(wait_until(LIMIT, || list_requests(&rig) >= 1));
        assert_eq!(rig.session.state().link.refused, None);
        rig.vehicle.stop();
    }

    #[test]
    fn a_configure_session_downloads_the_list_at_connect() {
        let mut rig = rig_access(Access::Configure);
        assert!(wait_until(LIMIT, || list_requests(&rig) >= 1));
        rig.vehicle.stop();
    }

    /// An index per variant, by a match with no wildcard arm: adding a variant to `Command` fails
    /// to compile here until it has an index, and `COUNT` then demands a row in the table.
    fn index(command: &Command) -> usize {
        match command {
            Command::RefreshParams => 0,
            Command::SetParam { .. } => 1,
            Command::StartLog { .. } => 2,
            Command::StopLog => 3,
            Command::SendCommand { .. } => 4,
            Command::ListLogs => 5,
            Command::DownloadLog { .. } => 6,
            Command::CancelLogDownload => 7,
            Command::StartSessionLog { .. } => 8,
            Command::StopSessionLog => 9,
        }
    }
    const COUNT: usize = 10;

    #[test]
    fn every_command_has_its_class_in_the_table() {
        let rows: Vec<(Command, Access, &str)> = vec![
            (Command::RefreshParams, Access::Operate, "RefreshParams"),
            (set_one(), Access::Configure, "SetParam"),
            (
                Command::StartLog {
                    sink: crate::tlog::LogSink::new(Vec::new()),
                    label: "x".into(),
                },
                Access::Operate,
                "StartLog",
            ),
            (Command::StopLog, Access::Operate, "StopLog"),
            (save_to_flash(), Access::Configure, "SendCommand"),
            (Command::ListLogs, Access::Configure, "ListLogs"),
            (
                Command::DownloadLog {
                    number: 1,
                    size: 0,
                    sink: crate::tlog::LogSink::new(Vec::new()),
                },
                Access::Configure,
                "DownloadLog",
            ),
            (
                Command::CancelLogDownload,
                Access::Configure,
                "CancelLogDownload",
            ),
            (
                Command::StartSessionLog {
                    sink: crate::tlog::LogSink::new(Vec::new()),
                    label: "x".into(),
                },
                Access::Operate,
                "StartSessionLog",
            ),
            (
                Command::StopSessionLog,
                Access::Operate,
                "StopSessionLog",
            ),
        ];
        let mut seen = [false; COUNT];
        for (command, access, name) in &rows {
            seen[index(command)] = true;
            assert_eq!(command.access(), *access, "{name}");
            assert_eq!(command.name(), *name);
        }
        assert!(
            seen.iter().all(|s| *s),
            "a Command variant has no row in the table"
        );
    }

    #[test]
    fn access_allows_follows_the_two_roles() {
        assert!(Access::Configure.allows(Access::Configure));
        assert!(Access::Configure.allows(Access::Operate));
        assert!(Access::Operate.allows(Access::Operate));
        assert!(!Access::Operate.allows(Access::Configure));
    }
}

mod replay {
    use std::io::Cursor;

    use super::*;
    use crate::dialect::{MavParamType, PARAM_VALUE_DATA};
    use crate::services::params::FetchStatus;
    use crate::tlog::{ReplaySpeed, replay};
    use crate::{Access, Body, Deframer, Encoder};

    const NAMES: [&str; 3] = ["AAA_ONE", "AAA_TWO", "BBB_ONE"];

    /// A recording as a vehicle's session would have made it: a heartbeat, then the whole list.
    fn recording() -> Vec<u8> {
        let mut encoder = Encoder::new(1, 1);
        let mut bytes = Vec::new();
        let mut time_us = 1_000_000u64;
        let mut push = |message: Message| {
            bytes.extend_from_slice(&time_us.to_be_bytes());
            bytes.extend_from_slice(&encoder.encode(&message));
            time_us += 1_000;
        };
        push(heartbeat(MavType::MAV_TYPE_QUADROTOR));
        for (index, name) in NAMES.iter().enumerate() {
            let mut id = [0u8; 16];
            id[..name.len()].copy_from_slice(name.as_bytes());
            push(Message::PARAM_VALUE(PARAM_VALUE_DATA {
                param_value: f32::from_le_bytes([index as u8 + 1, 0, 0, 0]),
                param_count: NAMES.len() as u16,
                param_index: index as u16,
                param_id: id.into(),
                param_type: MavParamType::MAV_PARAM_TYPE_INT32,
            }));
        }
        bytes
    }

    #[derive(Clone, Default)]
    struct Sink(Arc<Mutex<Vec<u8>>>);

    impl Write for Sink {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// A replay of a log: the table fills from the PARAM_VALUE frames the log carries; the session
    /// asks for no list, reads nothing by index and writes no parameter. The writer is a recording
    /// one in place of the replay's discarding writer, to see what would go out.
    #[test]
    fn a_replay_fills_the_parameter_table_from_the_log_and_requests_nothing() {
        let (reader, _discard) = replay(Box::new(Cursor::new(recording())), ReplaySpeed::Fast);
        let sink = Sink::default();
        let session = Session::start(
            reader,
            Box::new(sink.clone()),
            SessionConfig {
                access: Access::Operate,
                ..SessionConfig::default()
            },
            nop(),
        );
        assert!(
            wait_until(Duration::from_secs(3), || {
                session.state().params.fetch == FetchStatus::Complete
            }),
            "the replayed list did not fill the table: {:?}",
            session.state().params
        );
        {
            let state = session.state();
            assert_eq!(state.params.params.len(), NAMES.len());
            for name in NAMES {
                assert!(state.params.params.contains_key(name), "{name}");
            }
            assert_eq!(state.link.refused, None);
        }
        std::thread::sleep(Duration::from_millis(300));
        let mut frames = Vec::new();
        Deframer::new().push(&sink.0.lock().unwrap(), &mut frames);
        for frame in frames {
            assert!(
                !matches!(
                    frame.body,
                    Body::Known(
                        Message::PARAM_REQUEST_LIST(_)
                            | Message::PARAM_REQUEST_READ(_)
                            | Message::PARAM_SET(_)
                    )
                ),
                "a replay wrote {:?}",
                frame.body
            );
        }
    }
}
