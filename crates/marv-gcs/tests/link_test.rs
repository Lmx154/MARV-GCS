//! `marv-link-test` over UDP on localhost: a peer plays the flight controller (heartbeats and
//! `MARV_TEST` with tag 1, two of them skipped, and a gap in the header sequence) and collects what
//! the tool sends. The tool runs as a library call; one test runs the real binary for its exit code.

use std::net::{SocketAddr, UdpSocket};
use std::process::Command;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use gcs_mavlink::{Deframer, Frame};
use gcs_transport::Endpoint;
use marv_dialect::marv::{
    HEARTBEAT_DATA, MARV_TEST_DATA, MavAutopilot, MavModeFlag, MavState, MavType,
};
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, MessageData};
use marv_gcs::link_test::{self, Args, Parsed, Report};

const HEARTBEAT_ID: u32 = 0;

/// What the peer collected from the tool.
#[derive(Default)]
struct Collected {
    /// (system, component, header sequence) of every heartbeat.
    heartbeats: Vec<(u8, u8, u8)>,
    /// (header sequence, content) of every MARV_TEST.
    tests: Vec<(u8, MARV_TEST_DATA)>,
}

struct Peer {
    socket: UdpSocket,
    target: SocketAddr,
    sequence: u8,
}

impl Peer {
    fn send<D: MessageData>(&mut self, data: &D) {
        let header = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message_data(header, data);
        self.socket.send_to(raw.raw_bytes(), self.target).unwrap();
    }

    /// A frame that is lost on the way: only the header sequence moves on.
    fn skip(&mut self, frames: u8) {
        self.sequence = self.sequence.wrapping_add(frames);
    }

    fn heartbeat(&mut self) {
        self.send(&HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        });
    }

    fn test(&mut self, seq: u32) {
        self.send(&MARV_TEST_DATA {
            seq,
            time_ms: seq * 10,
            value: -(seq as i32),
            tag: 1,
        });
    }

    fn collect(&mut self, deframer: &mut Deframer, collected: &mut Collected) {
        let mut buf = [0u8; 2048];
        if let Ok((n, _)) = self.socket.recv_from(&mut buf) {
            let mut frames: Vec<Frame> = Vec::new();
            deframer.push(&buf[..n], &mut frames);
            for frame in frames {
                if frame.message_id == HEARTBEAT_ID {
                    collected.heartbeats.push((
                        frame.system_id,
                        frame.component_id,
                        frame.sequence,
                    ));
                } else if frame.message_id == MARV_TEST_DATA::ID {
                    let payload = &frame.raw[10..10 + usize::from(frame.raw[1])];
                    let data = MARV_TEST_DATA::deser(MavlinkVersion::V2, payload).unwrap();
                    collected.tests.push((frame.sequence, data));
                }
            }
        }
    }
}

/// Sets the flag when dropped, so the peer loop also sees a tool thread that panicked.
struct SetOnDrop(Arc<AtomicBool>);

impl Drop for SetOnDrop {
    fn drop(&mut self) {
        self.0.store(true, Ordering::Relaxed);
    }
}

/// The longest the peer plays on: the tool normally ends after about 3 s.
const PEER_DEADLINE: Duration = Duration::from_secs(30);

fn free_port() -> SocketAddr {
    UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
}

/// Runs the tool for `duration_s` against the peer's script; returns the tool's report, its output
/// and what the peer collected.
fn run_against_peer(count: u32, rate_hz: f64, duration_s: f64) -> (Report, String, Collected) {
    let bind = free_port();
    let args = Args {
        endpoint: Endpoint::Udp { bind },
        count,
        rate_hz,
        duration_s,
        quiet_start_s: 0.5,
    };
    let done = Arc::new(AtomicBool::new(false));
    let tool = thread::spawn({
        let done = Arc::clone(&done);
        move || {
            let _done = SetOnDrop(done);
            let mut out = Vec::new();
            let report = link_test::run(&args, &mut out).expect("run");
            (report, String::from_utf8(out).unwrap())
        }
    });

    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    socket
        .set_read_timeout(Some(Duration::from_millis(5)))
        .unwrap();
    let mut peer = Peer {
        socket,
        target: bind,
        sequence: 0,
    };
    let mut deframer = Deframer::new();
    let mut collected = Collected::default();
    let listen = |peer: &mut Peer, deframer: &mut Deframer, collected: &mut Collected, ms: u32| {
        for _ in 0..ms / 5 {
            peer.collect(deframer, collected);
        }
    };

    // Heartbeats first: the tool learns where to send from what arrives.
    for _ in 0..12 {
        peer.heartbeat();
        listen(&mut peer, &mut deframer, &mut collected, 50);
    }
    // 50 MARV_TEST, seq 7 and 31 lost on the way, and a gap of 3 frames in the header sequence.
    for seq in 0..50 {
        match seq {
            7 | 31 => peer.skip(1),
            20 => {
                peer.skip(3);
                peer.test(seq);
            }
            _ => peer.test(seq),
        }
        if seq % 10 == 9 {
            peer.heartbeat();
        }
        listen(&mut peer, &mut deframer, &mut collected, 10);
    }
    let begun = Instant::now();
    while !done.load(Ordering::Relaxed) {
        assert!(
            begun.elapsed() < PEER_DEADLINE,
            "the tool did not finish within {PEER_DEADLINE:?} of the peer's script"
        );
        peer.heartbeat();
        listen(&mut peer, &mut deframer, &mut collected, 50);
    }
    // Whatever is still in flight.
    listen(&mut peer, &mut deframer, &mut collected, 50);
    let (report, output) = tool.join().unwrap_or_else(|payload| {
        let why = payload
            .downcast_ref::<String>()
            .map(String::as_str)
            .or_else(|| payload.downcast_ref::<&str>().copied())
            .unwrap_or("not a string");
        panic!("the tool thread panicked: {why}")
    });
    (report, output, collected)
}

#[test]
fn counts_what_the_peer_sent_and_sends_what_it_promised() {
    let (report, output, collected) = run_against_peer(20, 50.0, 3.0);

    // The tool counted the vehicle's MARV_TEST: 48 arrived, 2 lost.
    let marv = &report.marv_test;
    assert_eq!(marv.received, 48, "{output}");
    assert_eq!(marv.lost, 2, "{output}");
    assert_eq!((marv.first_seq, marv.last_seq), (Some(0), Some(49)));
    assert_eq!((marv.duplicates, marv.restarts, marv.other_tag), (0, 0, 0));
    assert_eq!(marv.bad_frames, 0);
    assert_eq!(report.first_tests.len(), 3);
    assert!(
        report.first_tests[0].contains("seq: 0"),
        "{:?}",
        report.first_tests
    );

    // Header sequence: 2 skipped MARV_TEST and a gap of 3 are 5 lost frames of sender 1/1.
    let gaps = report.senders[&(1, 1)];
    assert_eq!(gaps.lost, 5, "{output}");
    assert!(gaps.loss_percent() > 0.0 && gaps.loss_percent() < 10.0);
    assert_eq!(report.messages[&(1, 1, MARV_TEST_DATA::ID)], 48);
    assert_eq!(report.rx.crc_errors, 0);
    assert_eq!(report.radio_status, 0);
    assert!(report.link_error.is_none());
    assert!(output.contains("== summary"), "{output}");
    assert!(output.contains("loss"), "{output}");

    // What the tool sent: exactly 20 MARV_TEST, tag 2, seq 0..19 in order, value = seq * 3.
    let seqs: Vec<u32> = collected.tests.iter().map(|(_, t)| t.seq).collect();
    assert_eq!(seqs, (0..20).collect::<Vec<u32>>(), "{output}");
    for (_, test) in &collected.tests {
        assert_eq!(test.tag, 2);
        assert_eq!(test.value, test.seq as i32 * 3);
    }
    let times: Vec<u32> = collected.tests.iter().map(|(_, t)| t.time_ms).collect();
    assert!(times.windows(2).all(|w| w[0] <= w[1]), "{times:?}");
    assert!(times[0] >= 500 && times[19] >= times[0] + 300, "{times:?}");
    assert_eq!(report.sent_tests, 20);

    // Heartbeats from 255/190, one per second.
    assert!(
        collected.heartbeats.len() >= 2,
        "{:?}",
        collected.heartbeats
    );
    assert!(
        collected
            .heartbeats
            .iter()
            .all(|h| (h.0, h.1) == (255, 190))
    );

    // One frame sequence over all frames sent: every step is +1, wrapping, in the order sent.
    // Heartbeats and MARV_TEST are collected apart, so merge them by sequence number.
    let mut sequences: Vec<u8> = collected.heartbeats.iter().map(|h| h.2).collect();
    sequences.extend(collected.tests.iter().map(|(s, _)| *s));
    sequences.sort_unstable();
    assert!(
        sequences.windows(2).all(|w| w[1] == w[0] + 1),
        "{sequences:?}"
    );
}

#[test]
fn argument_errors_are_refused_and_the_binary_exits_with_2() {
    let parse = |args: &[&str]| link_test::parse(args.iter().map(|s| s.to_string()));
    let err = |args: &[&str]| parse(args).unwrap_err();

    assert!(err(&[]).contains("--connect"));
    assert!(err(&["--count", "5"]).contains("--connect"));
    assert!(err(&["--connect"]).contains("needs a value"));
    assert!(err(&["--connect", "carrier-pigeon:1"]).contains("invalid endpoint"));
    assert!(err(&["--connect", "udp:0.0.0.0:0"]).contains("port"));
    assert!(err(&["--connect", "udp:0.0.0.0:14550", "--count", "many"]).contains("--count"));
    assert!(err(&["--connect", "udp:0.0.0.0:14550", "--rate", "0"]).contains("--rate"));
    assert!(err(&["--connect", "udp:0.0.0.0:14550", "--rate", "fast"]).contains("--rate"));
    assert!(err(&["--connect", "udp:0.0.0.0:14550", "--duration", "-1"]).contains("--duration"));
    assert!(
        err(&["--connect", "udp:0.0.0.0:14550", "--quiet-start", "nan"]).contains("--quiet-start")
    );
    assert!(err(&["--connect", "udp:0.0.0.0:14550", "--bogus"]).contains("unknown argument"));
    assert_eq!(parse(&["--help"]), Ok(Parsed::Help));

    // The defaults of the contract.
    let Ok(Parsed::Run(args)) = parse(&["--connect", "udp:0.0.0.0:14550"]) else {
        panic!("the defaults must parse");
    };
    assert_eq!(
        (
            args.count,
            args.rate_hz,
            args.duration_s,
            args.quiet_start_s
        ),
        (100, 10.0, 30.0, 2.0)
    );

    let output = Command::new(env!("CARGO_BIN_EXE_marv-link-test"))
        .args(["--rate", "0"])
        .output()
        .unwrap();
    assert_eq!(output.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&output.stderr).contains("usage:"));
}
