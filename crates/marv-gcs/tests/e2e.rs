//! End to end: the real transport, MAVLink layer and session against the simulated vehicle, over
//! TCP and over a serial pseudo-port made by socat. A missing socat is a failure, not a skip.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::net::{SocketAddr, TcpListener};
use std::path::{Path, PathBuf};
use std::process::{Child, Command as Process, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use gcs_mavlink::services::texts::Severity;
use gcs_mavlink::tlog::{LogSink, ReplaySpeed, TlogReader, replay};
use gcs_mavlink::{Access, Command, LinkStatus, Session, SessionConfig, VehicleState};
use gcs_sim::{SimConfig, SimVehicle, serve_tcp};
use gcs_transport::{Endpoint, Link, open};

mod support {
    use super::*;

    /// A bound listener whose server thread has not been started yet.
    pub struct Bound {
        listener: TcpListener,
        pub addr: SocketAddr,
    }

    /// The simulator on a TCP socket. The thread is stopped and joined on drop.
    pub struct Server {
        pub vehicle: Arc<Mutex<SimVehicle>>,
        pub addr: SocketAddr,
        stop: Arc<AtomicBool>,
        thread: Option<JoinHandle<()>>,
    }

    impl Bound {
        pub fn new() -> Bound {
            let listener = TcpListener::bind("127.0.0.1:0").expect("bind 127.0.0.1:0");
            let addr = listener.local_addr().expect("local address");
            Bound { listener, addr }
        }

        pub fn serve(self, config: SimConfig) -> Server {
            let vehicle = Arc::new(Mutex::new(SimVehicle::new(config)));
            let stop = Arc::new(AtomicBool::new(false));
            let thread = {
                let vehicle = Arc::clone(&vehicle);
                let stop = Arc::clone(&stop);
                thread::Builder::new()
                    .name("e2e-sim-server".into())
                    .spawn(move || serve_tcp(self.listener, vehicle, stop))
                    .expect("spawn the simulator thread")
            };
            Server {
                vehicle,
                addr: self.addr,
                stop,
                thread: Some(thread),
            }
        }
    }

    impl Server {
        pub fn start(config: SimConfig) -> Server {
            Bound::new().serve(config)
        }

        /// Stops the thread and waits for it; the socket is closed when this returns.
        pub fn stop(&mut self) {
            self.stop.store(true, Ordering::Relaxed);
            if let Some(thread) = self.thread.take() {
                thread.join().expect("the simulator thread panicked");
            }
        }

        pub fn endpoint(&self) -> Endpoint {
            Endpoint::Tcp {
                addr: self.addr.to_string(),
            }
        }

        pub fn with_vehicle<T>(&self, f: impl FnOnce(&mut SimVehicle) -> T) -> T {
            f(&mut self.vehicle.lock().unwrap_or_else(|e| e.into_inner()))
        }
    }

    impl Drop for Server {
        fn drop(&mut self) {
            self.stop.store(true, Ordering::Relaxed);
            if let Some(thread) = self.thread.take() {
                let _ = thread.join();
            }
        }
    }

    /// A session and the instant it was started at (session times are measured from it).
    pub struct Client {
        pub session: Session,
        pub started: Instant,
    }

    impl Client {
        pub fn start(link: Link) -> Client {
            Self::start_with(link.reader, link.writer)
        }

        pub fn start_with(
            reader: Box<dyn std::io::Read + Send>,
            writer: Box<dyn std::io::Write + Send>,
        ) -> Client {
            let started = Instant::now();
            let session = Session::start(
                reader,
                writer,
                SessionConfig {
                    access: Access::Configure,
                    ..SessionConfig::default()
                },
                Box::new(|| {}),
            );
            Client { session, started }
        }

        pub fn connect(endpoint: &Endpoint) -> Client {
            Self::start(open(endpoint).unwrap_or_else(|e| panic!("{e}")))
        }

        pub fn state(&self) -> VehicleState {
            self.session.state().clone()
        }

        pub fn status(&self) -> LinkStatus {
            self.session.state().link.status.clone()
        }

        pub fn since_start(&self) -> Duration {
            self.started.elapsed()
        }
    }

    /// Polls `condition` every 20 ms; true as soon as it holds, false at the deadline.
    pub fn wait_for(deadline: Duration, mut condition: impl FnMut() -> bool) -> bool {
        let end = Instant::now() + deadline;
        loop {
            if condition() {
                return true;
            }
            if Instant::now() >= end {
                return false;
            }
            thread::sleep(Duration::from_millis(20));
        }
    }

    /// Lets the vehicle run for `duration`: the duration itself is what is measured.
    pub fn run_for(duration: Duration) {
        thread::sleep(duration);
    }

    pub fn secs(n: u64) -> Duration {
        Duration::from_secs(n)
    }

    pub fn within_percent(value: f32, expected: f32, percent: f32) -> bool {
        (value - expected).abs() <= expected * percent / 100.0
    }

    /// A fresh directory under the system temporary directory, removed on drop.
    pub struct TempDir(pub PathBuf);

    impl TempDir {
        pub fn new() -> TempDir {
            static COUNTER: AtomicU32 = AtomicU32::new(0);
            let nanos = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_nanos());
            let dir = std::env::temp_dir().join(format!(
                "marv-e2e-{}-{nanos}-{}",
                std::process::id(),
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir(&dir).unwrap_or_else(|e| panic!("create {}: {e}", dir.display()));
            TempDir(dir)
        }

        pub fn path(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    pub const SOCAT: &str = "/usr/bin/socat";

    /// A socat child that bridges a pseudo serial port to a TCP address; killed on drop.
    pub struct Socat {
        child: Child,
        stderr: BufReader<std::process::ChildStderr>,
        pub pty: String,
    }

    impl Socat {
        pub fn bridge(addr: SocketAddr) -> Socat {
            let mut child = Process::new(SOCAT)
                .args(["-d", "-d", "pty,raw,echo=0", &format!("tcp:{addr}")])
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::piped())
                .spawn()
                .unwrap_or_else(|e| {
                    panic!("cannot run {SOCAT}: socat must be installed for the serial test: {e}")
                });
            let stderr = BufReader::new(child.stderr.take().expect("socat stderr"));
            let mut socat = Socat {
                child,
                stderr,
                pty: String::new(),
            };
            let deadline = Instant::now() + secs(10);
            let mut line = String::new();
            loop {
                assert!(
                    Instant::now() < deadline,
                    "socat did not report its pty in 10 s"
                );
                line.clear();
                let n = socat
                    .stderr
                    .read_line(&mut line)
                    .expect("read socat stderr");
                assert!(n > 0, "socat exited before it reported its pty");
                if let Some(i) = line.find("PTY is ") {
                    socat.pty = line[i + 7..].trim().to_string();
                    return socat;
                }
            }
        }

        pub fn kill(&mut self) {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }

    impl Drop for Socat {
        fn drop(&mut self) {
            self.kill();
        }
    }
}

use support::*;

fn connected(client: &Client) -> bool {
    client.status() == LinkStatus::Connected
}

/// The rate of the entry with this label, or `None` when there is no such entry.
fn rate(state: &VehicleState, label: &str) -> Option<f32> {
    state
        .messages
        .entries
        .values()
        .find(|entry| entry.label == label)
        .map(|entry| entry.rate_hz)
}

fn assert_rate(state: &VehicleState, label: &str, expected: f32, percent: f32) {
    let rate = rate(state, label).unwrap_or_else(|| panic!("no entry {label}"));
    assert!(
        within_percent(rate, expected, percent),
        "{label}: {rate} Hz, expected {expected} Hz within {percent} %"
    );
}

#[test]
fn connect_identifies_the_vehicle_and_exchanges_heartbeats() {
    let server = Server::start(SimConfig::default());
    let client = Client::connect(&server.endpoint());

    assert!(
        wait_for(secs(2), || connected(&client)),
        "not connected within 2 s: {:?}",
        client.status()
    );
    let state = client.state();
    let vehicle = state.link.vehicle.expect("vehicle identity");
    assert_eq!(
        (vehicle.system_id, vehicle.component_id),
        (1, 1),
        "{vehicle:?}"
    );
    assert_eq!(vehicle.mav_type, "MAV_TYPE_GENERIC");

    let before = server.with_vehicle(|v| v.heartbeats_received());
    run_for(secs(4));
    let after = server.with_vehicle(|v| v.heartbeats_received());
    let grown = after - before;
    assert!(
        (3..=5).contains(&grown),
        "the simulator received {grown} heartbeats in 4 s, expected 3 to 5"
    );
    assert_eq!(
        server.with_vehicle(|v| v.last_heartbeat_from()),
        Some((255, 190))
    );
}

#[test]
fn silence_is_lost_after_five_seconds_and_the_link_recovers() {
    let server = Server::start(SimConfig::default());
    let client = Client::connect(&server.endpoint());
    assert!(wait_for(secs(2), || connected(&client)), "not connected");

    // Silence starts right after a heartbeat arrived, so the 5 s timeout runs from now.
    let last = client.state().link.last_heartbeat;
    assert!(
        wait_for(secs(3), || client.state().link.last_heartbeat != last),
        "no new heartbeat in 3 s"
    );
    server.with_vehicle(|v| v.set_silent(true));
    let silent_at = Instant::now();

    assert!(
        wait_for(secs(7), || client.status() == LinkStatus::Lost),
        "not Lost within 7 s of silence: {:?}",
        client.status()
    );
    let after = silent_at.elapsed();
    assert!(
        after >= Duration::from_millis(4500),
        "Lost after only {after:?} of silence, the timeout is 5 s"
    );

    server.with_vehicle(|v| v.set_silent(false));
    assert!(
        wait_for(secs(2), || connected(&client)),
        "not Connected again within 2 s: {:?}",
        client.status()
    );
}

#[test]
fn closing_the_server_closes_the_link() {
    let mut server = Server::start(SimConfig::default());
    let client = Client::connect(&server.endpoint());
    assert!(wait_for(secs(2), || connected(&client)), "not connected");

    server.stop();
    assert!(
        wait_for(secs(2), || matches!(client.status(), LinkStatus::Closed(_))),
        "not Closed within 2 s: {:?}",
        client.status()
    );
}

#[test]
fn live_data_has_the_contract_rates_and_values() {
    let server = Server::start(SimConfig::default());
    let client = Client::connect(&server.endpoint());
    assert!(wait_for(secs(2), || connected(&client)), "not connected");

    run_for(secs(6));
    let state = client.state();
    for label in ["HIGHRES_IMU[0]", "SCALED_IMU2", "SCALED_PRESSURE"] {
        assert_rate(&state, label, 25.0, 5.0);
    }
    assert_rate(&state, "RC_CHANNELS", 5.0, 10.0);
    assert_rate(&state, "GPS_RAW_INT", 1.0, 10.0);
    assert_rate(&state, "SYS_STATUS", 1.0, 10.0);
    assert_rate(&state, "HEARTBEAT", 1.0, 10.0);
    assert!(rate(&state, "HIGHRES_IMU[1]").is_none(), "IMU1 only");

    let zacc = state
        .messages
        .series("HIGHRES_IMU", 0, "zacc")
        .expect("series HIGHRES_IMU[0].zacc");
    assert!(!zacc.points.is_empty(), "no zacc points");
    for [time, value] in &zacc.points {
        assert!(
            (-10.5..=-9.0).contains(value),
            "zacc {value} at {time} s is outside -10.5 to -9.0"
        );
    }
    let newest = zacc.points.back().expect("a newest point")[0];
    let age = client.since_start().as_secs_f64() - newest;
    assert!(age < 0.3, "the newest zacc point is {age} s old");

    let xgyro = state.messages.series("HIGHRES_IMU", 0, "xgyro");
    assert!(
        xgyro.is_some_and(|s| !s.points.is_empty()),
        "instance 0 has gyro data, so its series must have points"
    );
    let pressure = state.messages.series("HIGHRES_IMU", 0, "abs_pressure");
    assert!(
        pressure.is_none_or(|s| s.points.is_empty()),
        "HIGHRES_IMU.abs_pressure is NaN and must have no points: {:?}",
        pressure.map(|s| s.points.len())
    );
    let xmag = state.messages.series("HIGHRES_IMU", 0, "xmag");
    assert!(
        xmag.is_some_and(|s| !s.points.is_empty()),
        "the magnetometer is in HIGHRES_IMU's xmag"
    );

    assert_eq!(state.messages.lost_frames, 0, "frames were lost");
    assert_eq!(state.link.rx.crc_errors, 0, "{:?}", state.link.rx);
    assert_eq!(state.link.rx.discarded_bytes, 0, "{:?}", state.link.rx);
}

#[test]
fn device_status_gives_one_row_per_device_against_the_simulator() {
    let server = Server::start(SimConfig::default().with_device_status(1.0));
    let client = Client::connect(&server.endpoint());
    assert!(wait_for(secs(2), || connected(&client)), "not connected");

    run_for(secs(3));
    let state = client.state();
    let rows: Vec<_> = state
        .messages
        .entries
        .iter()
        .filter(|(_, entry)| entry.name == "MARV_DEVICE_STATUS")
        .collect();
    assert_eq!(rows.len(), 8, "{rows:?}");
    let instances: Vec<u8> = rows.iter().map(|(key, _)| key.instance).collect();
    assert_eq!(instances, [1, 2, 3, 4, 5, 6, 7, 9]);
    for (_, entry) in &rows {
        assert!(entry.count >= 3, "{entry:?}");
        assert_eq!(entry.text("state"), Some("MARV_DEVICE_STATE_OK"));
    }
    let imu1 = state.messages.entry("MARV_DEVICE_STATUS", 1).unwrap();
    assert_eq!(imu1.text("device_id"), Some("MARV_DEVICE_IMU1"));
    let imu2 = state.messages.entry("MARV_DEVICE_STATUS", 2).unwrap();
    assert!(imu2.number("samples").unwrap() >= 640.0 * 2.0);
    assert_eq!(state.link.rx.crc_errors, 0, "{:?}", state.link.rx);
    assert_eq!(state.link.rx.undecodable, 0, "{:?}", state.link.rx);
    assert_eq!(state.link.rx.unknown, 0, "{:?}", state.link.rx);
}

#[test]
fn dropped_frames_are_counted_as_lost() {
    let config = SimConfig {
        drop_tx_percent: 10,
        ..SimConfig::default()
    };
    let server = Server::start(config);
    let client = Client::connect(&server.endpoint());
    assert!(wait_for(secs(6), || connected(&client)), "not connected");

    run_for(secs(6));
    let state = client.state();
    let lost = state.messages.lost_frames as f64;
    let received = state.link.rx.frames as f64;
    let share = lost / (received + lost);
    assert!(
        (0.05..=0.15).contains(&share),
        "lost {lost} of {} frames: {:.1} %, expected 5 to 15 % (10 % dropped)",
        received + lost,
        share * 100.0
    );
}

#[test]
fn boot_text_reaches_the_text_log() {
    // The vehicle sends its boot text on its first step. The connection is made first (it waits
    // in the listener's queue), then the server starts, so the first step has a client to send to.
    let bound = Bound::new();
    let client = Client::connect(&Endpoint::Tcp {
        addr: bound.addr.to_string(),
    });
    let _server = bound.serve(SimConfig::default());

    assert!(
        wait_for(secs(3), || !client.state().texts.entries.is_empty()),
        "no text within 3 s"
    );
    let state = client.state();
    let entry = state
        .texts
        .entries
        .iter()
        .find(|e| e.text == "MARV simulator ready")
        .unwrap_or_else(|| panic!("boot text missing: {:?}", state.texts.entries));
    assert_eq!(entry.severity, Severity::Info);
    assert_eq!((entry.system_id, entry.component_id), (1, 1));
}

/// The HIGHRES_IMU instance of a raw MAVLink 2 frame, if it is one (message id 105).
fn highres_imu_instance(frame: &[u8]) -> Option<u8> {
    const STX_V2: u8 = 0xFD;
    if frame.len() < 12 || frame[0] != STX_V2 {
        return None;
    }
    let message_id = u32::from(frame[7]) | u32::from(frame[8]) << 8 | u32::from(frame[9]) << 16;
    if message_id != 105 {
        return None;
    }
    let payload = &frame[10..10 + usize::from(frame[1])];
    // `id` is byte 62 of the full payload; a payload cut short by trailing zeros has id 0.
    Some(payload.get(62).copied().unwrap_or(0))
}

#[test]
fn a_recording_replays_to_the_same_counts() {
    let server = Server::start(SimConfig::default());
    let client = Client::connect(&server.endpoint());
    assert!(wait_for(secs(2), || connected(&client)), "not connected");

    let dir = TempDir::new();
    let path = dir.path().join("e2e.tlog");
    let file = File::create(&path).expect("create the log file");
    client.session.send(Command::StartLog {
        sink: LogSink::new(file),
        label: path.display().to_string(),
    });
    assert!(
        wait_for(secs(2), || client.state().log.recording.is_some()),
        "recording did not start: {:?}",
        client.state().log
    );
    run_for(secs(3));
    client.session.send(Command::StopLog);
    assert!(
        wait_for(secs(2), || client.state().log.recording.is_none()),
        "recording did not stop"
    );
    let log = client.state().log;
    assert_eq!(log.error, None);

    let records: Vec<_> = TlogReader::new(File::open(&path).expect("open the log"))
        .collect::<Result<_, _>>()
        .expect("the log reads without error");
    assert_eq!(
        records.len() as u64,
        log.frames,
        "records against log.frames"
    );
    assert!(records.len() > 200, "only {} records in 3 s", records.len());
    assert_eq!(
        std::fs::metadata(&path).expect("log metadata").len(),
        log.bytes,
        "file size against log.bytes"
    );
    assert!(
        records.windows(2).all(|w| w[0].time_us <= w[1].time_us),
        "timestamps decrease"
    );
    let from = |system: u8| records.iter().filter(|r| r.frame[5] == system).count();
    assert!(from(1) > 0, "no frame from the vehicle (system 1)");
    assert!(
        from(255) > 0,
        "no frame from the ground station (system 255)"
    );

    let mut in_file = [0u64; 4];
    for record in &records {
        if let Some(instance) = highres_imu_instance(&record.frame) {
            in_file[usize::from(instance)] += 1;
        }
    }
    assert!(in_file[0] > 50, "{in_file:?}");
    assert_eq!(in_file[1..], [0, 0, 0], "IMU1 only: {in_file:?}");

    let (reader, writer) = replay(
        Box::new(File::open(&path).expect("open the log to replay")),
        ReplaySpeed::Fast,
    );
    let replayed = Client::start_with(reader, writer);
    assert!(
        wait_for(secs(10), || matches!(
            replayed.status(),
            LinkStatus::Closed(_)
        )),
        "the replay did not reach Closed: {:?}",
        replayed.status()
    );
    let state = replayed.state();
    let entry = state
        .messages
        .entry("HIGHRES_IMU", 0)
        .expect("no HIGHRES_IMU[0] after the replay");
    assert_eq!(
        entry.count, in_file[0],
        "HIGHRES_IMU[0]: replayed against recorded"
    );
    assert_eq!(state.link.rx.crc_errors, 0, "{:?}", state.link.rx);
    assert_eq!(state.link.rx.discarded_bytes, 0, "{:?}", state.link.rx);
}

#[test]
fn serial_pseudo_port_carries_the_whole_stack() {
    // Order matters for a clean start: the listener queues socat's connection, the port is
    // opened, and only then does the simulator start sending.
    let bound = Bound::new();
    let mut socat = Socat::bridge(bound.addr);
    let link = open(&Endpoint::Serial {
        path: socat.pty.clone(),
        baud: 115_200,
    })
    .unwrap_or_else(|e| panic!("{e}"));
    let client = Client::start(link);
    let _server = bound.serve(SimConfig::default());

    assert!(
        wait_for(secs(3), || connected(&client)),
        "not Connected within 3 s: {:?}",
        client.status()
    );
    run_for(secs(5));
    let state = client.state();
    assert_rate(&state, "HIGHRES_IMU[0]", 25.0, 5.0);
    assert_eq!(state.link.rx.discarded_bytes, 0, "{:?}", state.link.rx);

    socat.kill();
    assert!(
        wait_for(secs(2), || matches!(client.status(), LinkStatus::Closed(_))),
        "not Closed within 2 s of killing socat: {:?}",
        client.status()
    );
}
