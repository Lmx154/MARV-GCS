//! End-to-end tests of the commands (docs/link-contract.md, section 10): the station's session
//! against the simulated vehicle over a real TCP socket, in real time. Nothing is mocked.

use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use gcs_mavlink::dialect::MavCmd;
use gcs_mavlink::{
    Access, CancelStatus, Command, CommandRecord, CommandStatus, LinkStatus, Session,
    SessionConfig, VehicleState,
};
use gcs_sim::{SimConfig, SimVehicle};
use gcs_transport::Endpoint;

const POLL: Duration = Duration::from_millis(10);

struct Rig {
    stop: Arc<AtomicBool>,
    server: Option<JoinHandle<()>>,
    session: Option<Session>,
}

impl Rig {
    fn start(config: SimConfig, access: Access) -> Rig {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let addr = listener.local_addr().expect("address").to_string();
        let vehicle = Arc::new(Mutex::new(SimVehicle::new(config)));
        let stop = Arc::new(AtomicBool::new(false));
        let server = {
            let stop = Arc::clone(&stop);
            std::thread::spawn(move || gcs_sim::serve_tcp(listener, vehicle, stop))
        };
        let link = gcs_transport::open(&Endpoint::Tcp { addr }).expect("open the link");
        let session = Session::start(
            link.reader,
            link.writer,
            SessionConfig {
                access,
                ..SessionConfig::default()
            },
            Box::new(|| {}),
        );
        let rig = Rig {
            stop,
            server: Some(server),
            session: Some(session),
        };
        assert!(
            rig.wait(10.0, |s| s.link.status == LinkStatus::Connected)
                .is_some(),
            "never connected"
        );
        rig
    }

    fn configure(config: SimConfig) -> Rig {
        Rig::start(config, Access::Configure)
    }

    fn state(&self) -> MutexGuard<'_, VehicleState> {
        self.session.as_ref().expect("session").state()
    }

    fn send(&self, command: MavCmd, params: [f32; 7]) {
        self.session
            .as_ref()
            .expect("session")
            .send(Command::SendCommand { command, params });
    }

    fn record(&self) -> CommandRecord {
        self.state().command.clone().expect("a command record")
    }

    /// Polls every 10 ms until `condition` holds, for at most `seconds`. Bounded: it returns
    /// `None` at the deadline.
    fn wait(
        &self,
        seconds: f64,
        mut condition: impl FnMut(&VehicleState) -> bool,
    ) -> Option<Duration> {
        let begun = Instant::now();
        let deadline = begun + Duration::from_secs_f64(seconds);
        while Instant::now() < deadline {
            if condition(&self.state()) {
                return Some(begun.elapsed());
            }
            std::thread::sleep(POLL);
        }
        None
    }

    /// Waits until the command has ended and collects every percentage seen on the way.
    fn run_to_end(&self, seconds: f64) -> Vec<u8> {
        let mut seen = Vec::new();
        let ended = self.wait(seconds, |s| {
            if let Some(CommandStatus::InProgress { percent: Some(p) }) =
                s.command.as_ref().map(|r| r.status.clone())
                && seen.last() != Some(&p)
            {
                seen.push(p);
            }
            s.command.as_ref().is_some_and(|r| !r.outstanding())
        });
        assert!(
            ended.is_some(),
            "no end within {seconds} s: {:?}",
            self.state().command
        );
        seen
    }
}

impl Drop for Rig {
    fn drop(&mut self) {
        self.session = None;
        self.stop.store(true, Ordering::Relaxed);
        if let Some(server) = self.server.take() {
            let _ = server.join();
        }
    }
}

fn p(index: usize, value: f32) -> [f32; 7] {
    let mut a = [0.0; 7];
    a[index] = value;
    a
}

fn finished(text: &'static str, accepted: bool) -> CommandStatus {
    CommandStatus::Finished { text, accepted }
}

const STORAGE: MavCmd = MavCmd::MAV_CMD_PREFLIGHT_STORAGE;
const CAL: MavCmd = MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION;

#[test]
fn save_to_flash_is_accepted_and_its_progress_is_seen() {
    let rig = Rig::configure(SimConfig::default());
    rig.send(STORAGE, p(0, 1.0));
    let seen = rig.run_to_end(6.0);
    let record = rig.record();
    assert_eq!(record.status, finished("accepted", true));
    assert_eq!(record.sends, 1);
    assert!(
        seen.iter().any(|p| (20..100).contains(p)),
        "the progress was not seen: {seen:?}"
    );
    let took = record.ended.expect("ended") - record.started;
    assert!(
        (Duration::from_millis(1_800)..Duration::from_millis(3_000)).contains(&took),
        "took {took:?}"
    );
}

#[test]
fn a_gyro_calibration_reaches_accepted_after_3_s_with_progress() {
    let rig = Rig::configure(SimConfig::default());
    rig.send(CAL, p(0, 1.0));
    let seen = rig.run_to_end(8.0);
    assert_eq!(rig.record().status, finished("accepted", true));
    assert!(seen.iter().any(|p| *p >= 50), "{seen:?}");
    let record = rig.record();
    assert!(record.ended.expect("ended") - record.started >= Duration::from_millis(2_800));
}

#[test]
fn a_cancel_ends_the_calibration_cancelled_and_the_cancels_own_ack_is_kept_apart() {
    let rig = Rig::configure(SimConfig::default());
    rig.send(CAL, p(0, 1.0));
    assert!(
        rig.wait(3.0, |s| matches!(
            s.command.as_ref().map(|r| &r.status),
            Some(CommandStatus::InProgress { .. })
        ))
        .is_some()
    );
    rig.send(CAL, [0.0; 7]);
    rig.run_to_end(3.0);
    let record = rig.record();
    assert_eq!(record.status, finished("cancelled", false));
    assert_eq!(record.cancel, Some(CancelStatus::Answered("accepted")));
    assert!(record.ended.expect("ended") - record.started < Duration::from_millis(2_500));
}

#[test]
fn the_vehicle_dropping_the_first_two_commands_the_third_send_succeeds() {
    let rig = Rig::configure(SimConfig {
        drop_first_commands: 2,
        ..SimConfig::default()
    });
    rig.send(STORAGE, p(0, 1.0));
    rig.run_to_end(10.0);
    let record = rig.record();
    assert_eq!(record.status, finished("accepted", true));
    assert_eq!(record.sends, 3);
}

#[test]
fn the_vehicle_dropping_three_commands_the_result_is_no_answer_after_3_sends() {
    let rig = Rig::configure(SimConfig {
        drop_first_commands: 3,
        ..SimConfig::default()
    });
    rig.send(STORAGE, p(0, 1.0));
    rig.run_to_end(8.0);
    let record = rig.record();
    assert_eq!(record.status, CommandStatus::NoAnswer);
    assert_eq!(record.sends, 3);
    let took = record.ended.expect("ended") - record.started;
    assert!(
        (Duration::from_millis(4_400)..Duration::from_millis(5_000)).contains(&took),
        "{took:?}"
    );
}

#[test]
fn a_second_command_while_one_runs_is_refused_and_the_first_goes_on() {
    let rig = Rig::configure(SimConfig::default());
    rig.send(CAL, p(0, 1.0));
    rig.send(STORAGE, p(0, 1.0));
    assert!(
        rig.wait(2.0, |s| s
            .command
            .as_ref()
            .is_some_and(|r| r.refused.is_some()))
            .is_some()
    );
    rig.run_to_end(8.0);
    let record = rig.record();
    assert_eq!(record.command, 241);
    assert_eq!(record.status, finished("accepted", true));
    assert_eq!(record.refused.map(|r| (r.command, r.count)), Some((245, 1)));
}

#[test]
fn denied_unsupported_and_device_commands_get_the_vehicles_answers() {
    let rig = Rig::configure(SimConfig::default());
    let cases = [
        (STORAGE, p(0, 5.0), finished("denied", false)),
        (
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            [0.0; 7],
            finished("unsupported", false),
        ),
        (
            MavCmd::MAV_CMD_MARV_DEVICE,
            [3.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            finished("denied", false),
        ),
        (
            MavCmd::MAV_CMD_MARV_DEVICE,
            [3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            finished("accepted", true),
        ),
        (
            MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            p(0, 1.0),
            finished("accepted", true),
        ),
    ];
    for (command, params, expected) in cases {
        rig.send(command, params);
        rig.run_to_end(6.0);
        assert_eq!(rig.record().status, expected, "{command:?}");
        assert_eq!(rig.record().command, command as u32);
    }
}

#[test]
fn in_progress_then_no_final_ack_ends_as_no_result_after_20_s() {
    let rig = Rig::configure(SimConfig {
        never_final_ack: true,
        ..SimConfig::default()
    });
    rig.send(STORAGE, p(0, 1.0));
    rig.run_to_end(30.0);
    let record = rig.record();
    assert_eq!(record.status, CommandStatus::NoResult);
    // The last progress ACK is at 1.5 s; 20 s later the command is given up.
    let took = record.ended.expect("ended") - record.started;
    assert!(
        (Duration::from_millis(21_000)..Duration::from_millis(22_500)).contains(&took),
        "{took:?}"
    );
}

#[test]
fn an_operate_session_sends_nothing_to_the_vehicle() {
    let rig = Rig::start(SimConfig::default(), Access::Operate);
    rig.send(STORAGE, p(0, 1.0));
    std::thread::sleep(Duration::from_millis(500));
    assert!(rig.state().command.is_none());
    assert_eq!(rig.state().link.refused.as_ref().map(|r| r.count), Some(1));
}
