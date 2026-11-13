//! The link test tool (`marv-link-test`): no window, a plain read and write loop over a transport.
//!
//! It sends the ground station's heartbeat and, after a quiet start, a counted series of `MARV_TEST`
//! messages, and reports what comes back: frames per sender and message, the `MARV_TEST` sequence
//! numbers of the flight controller, the header sequence gaps of every sender, bytes per second.
//! It makes no judgement; a human reads the percentages.
//!
//! The station's MAVLink layer speaks the `marv` dialect, so the deframer knows `MARV_TEST`: it
//! checks its checksum (a frame with a wrong one is counted in the deframer's `crc_errors` and never
//! reaches this tool) and delivers it decoded.

use std::collections::BTreeMap;
use std::io::Write;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::thread;
use std::time::{Duration, Instant};

use gcs_mavlink::{Body, Deframer, Frame, RxStats, SessionConfig};
use gcs_transport::{Endpoint, open};
use marv_dialect::marv::{HEARTBEAT_DATA, MARV_TEST_DATA, MavMessage, RADIO_STATUS_DATA};
use marv_dialect::marv::{MavAutopilot, MavModeFlag, MavState, MavType};
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message, MessageData};

pub const USAGE: &str = "usage: marv-link-test --connect <endpoint> [--count <n>] [--rate <hz>]
                      [--duration <s>] [--quiet-start <s>]

  --connect <endpoint>   tcp:HOST:PORT, serial:PATH[:BAUD] or udp:ADDR:PORT (the local address to
                         listen on); required
  --count <n>            MARV_TEST messages to send (default 100)
  --rate <hz>            their rate (default 10)
  --duration <s>         how long to run in total (default 30)
  --quiet-start <s>      listen this long before sending MARV_TEST (default 2)
  --help                 print this text

Sends the ground station's HEARTBEAT (system 255, component 190) once per second for the whole run.
MARV_TEST messages carry tag 2, seq 0, 1, 2, ..., time_ms since the start and value = seq * 3.
Prints one line per second and a summary at the end; it does not judge the result.";

pub const DEFAULT_COUNT: u32 = 100;
pub const DEFAULT_RATE_HZ: f64 = 10.0;
pub const DEFAULT_DURATION_S: f64 = 30.0;
pub const DEFAULT_QUIET_START_S: f64 = 2.0;

/// `MARV_TEST.tag` of the flight controller and of the ground station.
pub const TAG_FLIGHT_CONTROLLER: u8 = 1;
pub const TAG_GROUND_STATION: u8 = 2;

const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(1);
const REPORT_INTERVAL: Duration = Duration::from_secs(1);
const READ_BUFFER: usize = 4096;
/// Messages printed in full as they arrive: the first `MARV_TEST` of the flight controller.
const FULL_PRINTS: usize = 3;
const HEADER_LEN: usize = 10;

#[derive(Debug, Clone, PartialEq)]
pub struct Args {
    pub endpoint: Endpoint,
    pub count: u32,
    pub rate_hz: f64,
    pub duration_s: f64,
    pub quiet_start_s: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Parsed {
    Run(Args),
    Help,
}

/// Parse the arguments after the program name. The error text is one line, without the usage.
pub fn parse<I: IntoIterator<Item = String>>(args: I) -> Result<Parsed, String> {
    let mut args = args.into_iter();
    let mut endpoint = None;
    let mut count = DEFAULT_COUNT;
    let mut rate_hz = DEFAULT_RATE_HZ;
    let mut duration_s = DEFAULT_DURATION_S;
    let mut quiet_start_s = DEFAULT_QUIET_START_S;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => return Ok(Parsed::Help),
            "--connect" => {
                let text = value(&mut args, &arg)?;
                endpoint = Some(text.parse::<Endpoint>().map_err(|e| e.to_string())?);
            }
            "--count" => {
                let text = value(&mut args, &arg)?;
                count = text
                    .parse::<u32>()
                    .map_err(|_| format!("invalid --count {text:?}: expected a whole number"))?;
            }
            "--rate" => {
                let text = value(&mut args, &arg)?;
                rate_hz = match text.parse::<f64>() {
                    Ok(v) if v.is_finite() && v > 0.0 && v <= 10_000.0 => v,
                    _ => {
                        return Err(format!(
                            "invalid --rate {text:?}: expected a rate in Hz above 0 and up to 10000"
                        ));
                    }
                };
            }
            "--duration" => {
                let text = value(&mut args, &arg)?;
                duration_s = seconds(&text, &arg)?;
            }
            "--quiet-start" => {
                let text = value(&mut args, &arg)?;
                quiet_start_s = seconds(&text, &arg)?;
            }
            _ => return Err(format!("unknown argument {arg:?}")),
        }
    }
    let endpoint = endpoint.ok_or_else(|| "--connect <endpoint> is required".to_string())?;
    Ok(Parsed::Run(Args {
        endpoint,
        count,
        rate_hz,
        duration_s,
        quiet_start_s,
    }))
}

fn value(args: &mut impl Iterator<Item = String>, option: &str) -> Result<String, String> {
    args.next().ok_or_else(|| format!("{option} needs a value"))
}

fn seconds(text: &str, option: &str) -> Result<f64, String> {
    match text.parse::<f64>() {
        Ok(v) if v.is_finite() && (0.0..=86_400.0 * 30.0).contains(&v) => Ok(v),
        _ => Err(format!(
            "invalid {option} {text:?}: expected seconds, 0 or more"
        )),
    }
}

/// Lost frames by the 8-bit header sequence of one sender.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SeqGaps {
    pub received: u64,
    pub lost: u64,
    last: Option<u8>,
}

impl SeqGaps {
    fn push(&mut self, sequence: u8) {
        if let Some(last) = self.last {
            self.lost += u64::from(sequence.wrapping_sub(last).wrapping_sub(1));
        }
        self.last = Some(sequence);
        self.received += 1;
    }

    /// Lost frames as a percentage of the frames expected (received plus lost).
    pub fn loss_percent(&self) -> f64 {
        percent(self.lost, self.received + self.lost)
    }
}

/// `MARV_TEST` messages with `tag` 1 (the flight controller), counted by their `seq`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MarvTestStats {
    pub received: u64,
    pub first_seq: Option<u32>,
    pub last_seq: Option<u32>,
    /// Gaps in `seq`: numbers skipped between two messages that arrived in order.
    pub lost: u64,
    /// The same `seq` as the message before it.
    pub duplicates: u64,
    /// A `seq` below the one before it (not equal): the sender restarted, or messages are reordered.
    pub restarts: u64,
    /// `MARV_TEST` with a tag other than 1.
    pub other_tag: u64,
    /// `MARV_TEST` whose payload would not decode (a wrong checksum never gets this far: the deframer
    /// counts it in `RxStats::crc_errors`).
    pub bad_frames: u64,
    next_expected: Option<u32>,
}

impl MarvTestStats {
    fn push(&mut self, seq: u32) {
        self.received += 1;
        if self.first_seq.is_none() {
            self.first_seq = Some(seq);
        }
        match self.next_expected {
            Some(expected) if seq > expected => self.lost += u64::from(seq - expected),
            Some(expected) if seq == expected => {}
            Some(_) if Some(seq) == self.last_seq => {
                self.duplicates += 1;
                return;
            }
            Some(_) => self.restarts += 1,
            None => {}
        }
        self.last_seq = Some(seq);
        self.next_expected = Some(seq.wrapping_add(1));
    }

    /// Lost as a percentage of the messages expected (received plus lost).
    pub fn loss_percent(&self) -> f64 {
        percent(self.lost, self.received + self.lost)
    }
}

fn percent(part: u64, whole: u64) -> f64 {
    if whole == 0 {
        0.0
    } else {
        part as f64 * 100.0 / whole as f64
    }
}

/// Everything counted from the frames that arrived.
#[derive(Debug, Clone, Default)]
pub struct Report {
    pub elapsed: Duration,
    pub sent_heartbeats: u32,
    pub sent_tests: u32,
    pub frames: u64,
    pub bytes: u64,
    /// Frames per (system id, component id, message id).
    pub messages: BTreeMap<(u8, u8, u32), u64>,
    /// Header sequence gaps per (system id, component id).
    pub senders: BTreeMap<(u8, u8), SeqGaps>,
    pub marv_test: MarvTestStats,
    /// The first `MARV_TEST` of the flight controller, in full.
    pub first_tests: Vec<String>,
    pub radio_status: u64,
    /// The deframer's own counters at the end.
    pub rx: RxStats,
    /// Set when the link ended the run before its time.
    pub link_error: Option<String>,
}

impl Report {
    /// Counts one frame; returns the text of a message that is to be printed in full right away.
    fn on_frame(&mut self, frame: &Frame) -> Option<String> {
        self.frames += 1;
        *self
            .messages
            .entry((frame.system_id, frame.component_id, frame.message_id))
            .or_default() += 1;
        self.senders
            .entry((frame.system_id, frame.component_id))
            .or_default()
            .push(frame.sequence);
        let from = format!(
            "{}/{} header seq {}",
            frame.system_id, frame.component_id, frame.sequence
        );
        if frame.message_id == MARV_TEST_DATA::ID {
            return self.on_marv_test(frame, &from);
        }
        if frame.message_id == RADIO_STATUS_DATA::ID {
            self.radio_status += 1;
            return Some(
                match RADIO_STATUS_DATA::deser(MavlinkVersion::V2, payload(frame)) {
                    Ok(data) => format!("RADIO_STATUS from {from}: {data:?}"),
                    Err(e) => format!("RADIO_STATUS from {from}: payload refused: {e:?}"),
                },
            );
        }
        None
    }

    fn on_marv_test(&mut self, frame: &Frame, from: &str) -> Option<String> {
        let Body::Known(MavMessage::MARV_TEST(data)) = &frame.body else {
            self.marv_test.bad_frames += 1;
            return None;
        };
        if data.tag != TAG_FLIGHT_CONTROLLER {
            self.marv_test.other_tag += 1;
            return None;
        }
        self.marv_test.push(data.seq);
        if self.first_tests.len() < FULL_PRINTS {
            let text = format!("MARV_TEST from {from}: {data:?}");
            self.first_tests.push(text.clone());
            return Some(text);
        }
        None
    }
}

fn payload(frame: &Frame) -> &[u8] {
    &frame.raw[HEADER_LEN..HEADER_LEN + usize::from(frame.raw[1])]
}

/// The name of a message id in the `marv` dialect.
fn name_of(id: u32) -> String {
    MavMessage::default_message_from_id(id)
        .map_or_else(|| format!("UNKNOWN_{id}"), |m| m.message_name().to_string())
}

/// One MAVLink 2 frame from the ground station; the sequence number counts every frame sent.
struct Sender {
    system_id: u8,
    component_id: u8,
    sequence: u8,
}

impl Sender {
    fn frame<D: MessageData>(&mut self, data: &D) -> Vec<u8> {
        let header = MavHeader {
            system_id: u32::from(self.system_id),
            component_id: self.component_id,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message_data(header, data);
        raw.raw_bytes().to_vec()
    }
}

fn ground_station_heartbeat() -> HEARTBEAT_DATA {
    HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    }
}

/// What the reader thread passes on.
enum Input {
    Received {
        bytes: usize,
        frames: Vec<Frame>,
    },
    /// The reader ended: the link is down (or the run is over, with the last frames).
    Ended {
        frames: Vec<Frame>,
        stats: RxStats,
        error: Option<String>,
    },
}

fn read_loop(
    mut reader: Box<dyn std::io::Read + Send>,
    stop: Arc<AtomicBool>,
    tx: mpsc::Sender<Input>,
) {
    let mut deframer = Deframer::new();
    let mut buf = [0u8; READ_BUFFER];
    let mut error = None;
    while !stop.load(Ordering::Relaxed) {
        match reader.read(&mut buf) {
            Ok(0) => {
                error = Some("the link was closed".to_string());
                break;
            }
            Ok(n) => {
                let mut frames = Vec::new();
                deframer.push(&buf[..n], &mut frames);
                if tx.send(Input::Received { bytes: n, frames }).is_err() {
                    return;
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => {
                error = Some(format!("the link failed: {e}"));
                break;
            }
        }
    }
    let _ = tx.send(Input::Ended {
        frames: Vec::new(),
        stats: deframer.stats(),
        error,
    });
}

/// Runs the test: opens the endpoint, sends and listens for `args.duration_s`, prints a line per
/// second and the summary to `out`, and returns the counts.
pub fn run(args: &Args, out: &mut dyn Write) -> Result<Report, String> {
    let link = open(&args.endpoint).map_err(|e| e.to_string())?;
    let mut writer = link.writer;
    let (tx, rx) = mpsc::channel();
    let stop = Arc::new(AtomicBool::new(false));
    let reader = thread::Builder::new()
        .name("link-test-read".into())
        .spawn({
            let stop = Arc::clone(&stop);
            move || read_loop(link.reader, stop, tx)
        })
        .map_err(|e| format!("cannot start the reader thread: {e}"))?;

    let config = SessionConfig::default();
    let mut sender = Sender {
        system_id: config.system_id,
        component_id: config.component_id,
        sequence: 0,
    };
    let duration = Duration::from_secs_f64(args.duration_s);
    let quiet = Duration::from_secs_f64(args.quiet_start_s);
    let period = Duration::from_secs_f64(1.0 / args.rate_hz);
    let test_at = |k: u32| quiet + period.mul_f64(f64::from(k));
    let _ = writeln!(
        out,
        "marv-link-test on {}: heartbeat from {}/{} every second; {} MARV_TEST at {} Hz after {} s; {} s in all",
        args.endpoint,
        config.system_id,
        config.component_id,
        args.count,
        args.rate_hz,
        args.quiet_start_s,
        args.duration_s
    );
    if args.count > 0 && test_at(args.count - 1) > duration {
        let _ = writeln!(
            out,
            "note: --duration ends the run before the last MARV_TEST is due"
        );
    }

    let mut report = Report::default();
    let start = Instant::now();
    let mut next_heartbeat = Duration::ZERO;
    let mut next_report = REPORT_INTERVAL;
    let mut sent_at_report = (0u32, 0u32);
    let mut counted_at_report = (0u64, 0u64);
    let mut ended = false;
    let mut write_error: Option<String> = None;

    'run: loop {
        let now = start.elapsed();
        if now >= duration || ended || write_error.is_some() {
            break;
        }
        while next_heartbeat <= now {
            let bytes = sender.frame(&ground_station_heartbeat());
            if let Err(e) = writer.write_all(&bytes).and_then(|()| writer.flush()) {
                write_error = Some(format!("the link failed on write: {e}"));
                continue 'run;
            }
            report.sent_heartbeats += 1;
            next_heartbeat += HEARTBEAT_INTERVAL;
        }
        while report.sent_tests < args.count && test_at(report.sent_tests) <= now {
            let seq = report.sent_tests;
            let data = MARV_TEST_DATA {
                seq,
                time_ms: now.as_millis() as u32,
                value: (seq as i32).wrapping_mul(3),
                tag: TAG_GROUND_STATION,
            };
            let bytes = sender.frame(&data);
            if let Err(e) = writer.write_all(&bytes).and_then(|()| writer.flush()) {
                write_error = Some(format!("the link failed on write: {e}"));
                continue 'run;
            }
            report.sent_tests += 1;
        }
        if next_report <= now {
            write_status_line(
                out,
                next_report,
                &report,
                &sent_at_report,
                &counted_at_report,
            );
            sent_at_report = (report.sent_heartbeats, report.sent_tests);
            counted_at_report = (report.frames, report.bytes);
            next_report += REPORT_INTERVAL;
        }
        let mut due = next_heartbeat.min(next_report).min(duration);
        if report.sent_tests < args.count {
            due = due.min(test_at(report.sent_tests));
        }
        let wait = due.saturating_sub(start.elapsed());
        match rx.recv_timeout(wait) {
            Ok(input) => ended = absorb(input, &mut report, out),
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => ended = true,
        }
        while let Ok(input) = rx.try_recv() {
            ended |= absorb(input, &mut report, out);
        }
    }

    stop.store(true, Ordering::Relaxed);
    let _ = reader.join();
    for input in rx.try_iter() {
        absorb(input, &mut report, out);
    }
    report.elapsed = start.elapsed();
    if report.link_error.is_none() {
        report.link_error = write_error;
    }
    write_summary(out, args, &report);
    Ok(report)
}

/// Takes what the reader thread passed on; true when the reader has ended.
fn absorb(input: Input, report: &mut Report, out: &mut dyn Write) -> bool {
    let (frames, ended) = match input {
        Input::Received { bytes, frames } => {
            report.bytes += bytes as u64;
            (frames, false)
        }
        Input::Ended {
            frames,
            stats,
            error,
        } => {
            report.rx = stats;
            if error.is_some() {
                report.link_error = error;
            }
            (frames, true)
        }
    };
    for frame in &frames {
        if let Some(text) = report.on_frame(frame) {
            let _ = writeln!(out, "{text}");
        }
    }
    ended
}

fn write_status_line(
    out: &mut dyn Write,
    at: Duration,
    report: &Report,
    sent_before: &(u32, u32),
    counted_before: &(u64, u64),
) {
    let marv = &report.marv_test;
    let hdr_lost: u64 = report.senders.values().map(|s| s.lost).sum();
    let hdr_seen: u64 = report.senders.values().map(|s| s.received + s.lost).sum();
    let _ = writeln!(
        out,
        "t={:>4} s  sent {} (+{} heartbeat, +{} MARV_TEST)  rx {} frames {} B/s | MARV_TEST tag 1: n={} last_seq={} lost={} | header seq lost {} of {} ({:.2} %)",
        at.as_secs(),
        report.sent_heartbeats + report.sent_tests,
        report.sent_heartbeats - sent_before.0,
        report.sent_tests - sent_before.1,
        report.frames - counted_before.0,
        report.bytes - counted_before.1,
        marv.received,
        marv.last_seq.map_or("-".to_string(), |s| s.to_string()),
        marv.lost,
        hdr_lost,
        hdr_seen,
        percent(hdr_lost, hdr_seen),
    );
}

fn write_summary(out: &mut dyn Write, args: &Args, report: &Report) {
    let secs = report.elapsed.as_secs_f64();
    let _ = writeln!(out, "\n== summary: {secs:.1} s on {} ==", args.endpoint);
    let _ = writeln!(
        out,
        "sent: {} HEARTBEAT, {} MARV_TEST (tag 2, seq 0..{}); {} frames",
        report.sent_heartbeats,
        report.sent_tests,
        report.sent_tests.saturating_sub(1),
        report.sent_heartbeats + report.sent_tests
    );
    let _ = writeln!(
        out,
        "received: {} frames, {} bytes ({:.1} B/s on average)",
        report.frames,
        report.bytes,
        if secs > 0.0 {
            report.bytes as f64 / secs
        } else {
            0.0
        }
    );
    let _ = writeln!(out, "frames by sender, component and message:");
    if report.messages.is_empty() {
        let _ = writeln!(out, "  (none)");
    }
    for ((system, component, id), count) in &report.messages {
        let _ = writeln!(
            out,
            "  system {system:>3} component {component:>3}  {} ({id}): {count}",
            name_of(*id)
        );
    }
    let _ = writeln!(
        out,
        "header sequence gaps (lost by the 8-bit sequence, wrapping; a restart counts as a loss):"
    );
    for ((system, component), gaps) in &report.senders {
        let _ = writeln!(
            out,
            "  system {system:>3} component {component:>3}: received {}, lost {}, expected {}, loss {:.2} %",
            gaps.received,
            gaps.lost,
            gaps.received + gaps.lost,
            gaps.loss_percent()
        );
    }
    let marv = &report.marv_test;
    let seq = |s: Option<u32>| s.map_or("-".to_string(), |s| s.to_string());
    let _ = writeln!(
        out,
        "MARV_TEST from the flight controller (tag 1): received {}, first seq {}, last seq {}, lost {} ({:.2} %), duplicates {}, restarts {}",
        marv.received,
        seq(marv.first_seq),
        seq(marv.last_seq),
        marv.lost,
        marv.loss_percent(),
        marv.duplicates,
        marv.restarts
    );
    let _ = writeln!(
        out,
        "  other: {} with another tag, {} with a payload that would not decode",
        marv.other_tag, marv.bad_frames
    );
    for text in &report.first_tests {
        let _ = writeln!(out, "  first received: {text}");
    }
    let _ = writeln!(
        out,
        "deframer: {} checksum failures, {} frames with an id the station's dialect (marv) does not have, {} undecodable, {} discarded bytes",
        report.rx.crc_errors, report.rx.unknown, report.rx.undecodable, report.rx.discarded_bytes
    );
    let _ = writeln!(out, "RADIO_STATUS received: {}", report.radio_status);
    if let Some(error) = &report.link_error {
        let _ = writeln!(out, "the run ended early: {error}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_sequence_gaps_wrap_at_256() {
        let mut gaps = SeqGaps::default();
        for s in [254, 255, 0, 1] {
            gaps.push(s);
        }
        assert_eq!((gaps.received, gaps.lost), (4, 0));
        gaps.push(5);
        assert_eq!((gaps.received, gaps.lost), (5, 3));
        gaps.push(3);
        assert_eq!(gaps.lost, 3 + 253);
    }

    #[test]
    fn marv_test_sequence_counts_gaps_duplicates_and_restarts() {
        let mut stats = MarvTestStats::default();
        for seq in [0, 1, 2, 5, 6, 6, 7, 2, 3] {
            stats.push(seq);
        }
        assert_eq!(stats.received, 9);
        assert_eq!((stats.first_seq, stats.last_seq), (Some(0), Some(3)));
        assert_eq!((stats.lost, stats.duplicates, stats.restarts), (2, 1, 1));
    }
}
