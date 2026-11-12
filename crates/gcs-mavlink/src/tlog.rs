//! Telemetry log: recording every frame to a `.tlog`, reading one back, replaying one as a link.
//!
//! Record layout (the one Mission Planner, QGroundControl and pymavlink use): 8 bytes, big-endian,
//! microseconds of Unix time, then one raw MAVLink frame.

use std::fmt;
use std::io::{self, Read, Write};
use std::sync::{Mutex, PoisonError};
use std::thread;
use std::time::{Duration, Instant};

use crate::services::{Ctx, Service};
use crate::{Command, Frame};

const STX_V1: u8 = 0xFE;
const STX_V2: u8 = 0xFD;
const INCOMPAT_SIGNED: u8 = 0x01;
const TIME_LEN: usize = 8;
/// How long the writer may hold written data before it is flushed.
const FLUSH_INTERVAL: Duration = Duration::from_secs(1);
/// The longest one `replay` read waits for a frame that is not due.
const REPLAY_WAIT: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Default, PartialEq)]
pub struct LogState {
    /// The label of the log being written (the file path), or `None`.
    pub recording: Option<String>,
    pub frames: u64,
    pub bytes: u64,
    /// Why recording stopped by itself.
    pub error: Option<String>,
}

/// Where the recorder writes. The window opens the file and hands it over in `Command::StartLog`,
/// so the service itself does no file handling.
pub struct LogSink(Mutex<Option<Box<dyn Write + Send>>>);

impl LogSink {
    pub fn new(writer: impl Write + Send + 'static) -> Self {
        Self(Mutex::new(Some(Box::new(writer))))
    }

    /// The writer; `None` when it was taken before.
    pub fn take(&self) -> Option<Box<dyn Write + Send>> {
        self.0.lock().unwrap_or_else(PoisonError::into_inner).take()
    }
}

impl fmt::Debug for LogSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("LogSink")
    }
}

/// The service that writes the log.
#[derive(Default)]
pub struct Recorder {
    writer: Option<Box<dyn Write + Send>>,
    /// Something was written since the last flush.
    dirty: bool,
    /// Session time of the last flush (or of the start).
    last_flush: Duration,
}

impl Recorder {
    pub fn new() -> Self {
        Self::default()
    }

    fn write(&mut self, raw: &[u8], cx: &mut Ctx) {
        let Some(writer) = self.writer.as_mut() else {
            return;
        };
        let mut record = Vec::with_capacity(TIME_LEN + raw.len());
        record.extend_from_slice(&cx.wall_time_us.to_be_bytes());
        record.extend_from_slice(raw);
        match writer.write_all(&record) {
            Ok(()) => {
                self.dirty = true;
                cx.state.log.frames += 1;
                cx.state.log.bytes += record.len() as u64;
            }
            Err(error) => self.fail(&error, cx),
        }
        cx.changed = true;
    }

    fn flush(&mut self, now: Duration) -> io::Result<()> {
        self.last_flush = now;
        if !self.dirty {
            return Ok(());
        }
        self.dirty = false;
        match self.writer.as_mut() {
            Some(writer) => writer.flush(),
            None => Ok(()),
        }
    }

    fn fail(&mut self, error: &io::Error, cx: &mut Ctx) {
        self.writer = None;
        self.dirty = false;
        cx.state.log.recording = None;
        cx.state.log.error = Some(error.to_string());
        cx.changed = true;
    }

    /// Flush and drop the writer, if any. A flush error ends up in `error`.
    fn close(&mut self, cx: &mut Ctx) {
        if let Err(error) = self.flush(cx.now) {
            self.fail(&error, cx);
        }
        self.writer = None;
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        if let Some(writer) = self.writer.as_mut() {
            let _ = writer.flush();
        }
    }
}

impl Service for Recorder {
    fn on_frame(&mut self, frame: &Frame, cx: &mut Ctx) {
        self.write(&frame.raw, cx);
    }

    fn on_sent(&mut self, raw: &[u8], cx: &mut Ctx) {
        self.write(raw, cx);
    }

    fn on_tick(&mut self, cx: &mut Ctx) {
        if self.dirty
            && cx.now.saturating_sub(self.last_flush) >= FLUSH_INTERVAL
            && let Err(error) = self.flush(cx.now)
        {
            self.fail(&error, cx);
        }
    }

    fn on_command(&mut self, command: &Command, cx: &mut Ctx) {
        match command {
            Command::StartLog { sink, label } => {
                self.close(cx);
                match sink.take() {
                    Some(writer) => {
                        self.writer = Some(writer);
                        self.dirty = false;
                        self.last_flush = cx.now;
                        cx.state.log = LogState {
                            recording: Some(label.clone()),
                            frames: 0,
                            bytes: 0,
                            error: None,
                        };
                    }
                    None => {
                        cx.state.log = LogState {
                            error: Some("the log file is not available (already taken)".into()),
                            ..LogState::default()
                        };
                    }
                }
                cx.changed = true;
            }
            Command::StopLog if self.writer.is_some() => {
                self.close(cx);
                cx.state.log.recording = None;
                cx.changed = true;
            }
            _ => {}
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TlogRecord {
    /// Unix time in microseconds.
    pub time_us: u64,
    /// One raw MAVLink frame.
    pub frame: Vec<u8>,
}

/// Reads the records of a `.tlog`.
pub struct TlogReader<R> {
    source: R,
    /// Bytes consumed so far.
    offset: u64,
    /// After the end or an error.
    done: bool,
}

impl<R: Read> TlogReader<R> {
    pub fn new(source: R) -> Self {
        Self {
            source,
            offset: 0,
            done: false,
        }
    }

    /// Fill `buf` as far as the source goes; the number of bytes read, short only at the end.
    fn fill(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut filled = 0;
        while filled < buf.len() {
            match self.source.read(&mut buf[filled..]) {
                Ok(0) => break,
                Ok(n) => filled += n,
                Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                Err(error) => return Err(error),
            }
        }
        self.offset += filled as u64;
        Ok(filled)
    }

    fn fill_exact(&mut self, buf: &mut [u8]) -> io::Result<()> {
        if self.fill(buf)? < buf.len() {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                format!("the log ends inside a record, at offset {}", self.offset),
            ));
        }
        Ok(())
    }

    /// `None` at the end of the file on a record boundary.
    fn record(&mut self) -> io::Result<Option<TlogRecord>> {
        let mut time = [0u8; TIME_LEN];
        match self.fill(&mut time)? {
            0 => return Ok(None),
            TIME_LEN => {}
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    format!("the log ends inside a record, at offset {}", self.offset),
                ));
            }
        }
        let start = self.offset;
        let mut head = [0u8; 3];
        self.fill_exact(&mut head[..1])?;
        let (known, total) = match head[0] {
            STX_V2 => {
                self.fill_exact(&mut head[1..])?;
                let signature = if head[2] & INCOMPAT_SIGNED != 0 {
                    13
                } else {
                    0
                };
                (3, 12 + usize::from(head[1]) + signature)
            }
            STX_V1 => {
                self.fill_exact(&mut head[1..2])?;
                (2, 8 + usize::from(head[1]))
            }
            other => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("byte 0x{other:02x} at offset {start} does not start a MAVLink frame"),
                ));
            }
        };
        let mut frame = vec![0u8; total];
        frame[..known].copy_from_slice(&head[..known]);
        self.fill_exact(&mut frame[known..])?;
        Ok(Some(TlogRecord {
            time_us: u64::from_be_bytes(time),
            frame,
        }))
    }
}

impl<R: Read> Iterator for TlogReader<R> {
    type Item = io::Result<TlogRecord>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        match self.record() {
            Ok(Some(record)) => Some(Ok(record)),
            Ok(None) => {
                self.done = true;
                None
            }
            Err(error) => {
                self.done = true;
                Some(Err(error))
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplaySpeed {
    /// At the pace it was recorded.
    RealTime,
    /// As fast as it is read.
    Fast,
}

/// A link made from a log. The reader gives the recorded frames and follows the transport reader
/// contract (`TimedOut` while the next frame is not due, `Ok(0)` at the end); the writer discards.
pub fn replay(
    source: Box<dyn Read + Send>,
    speed: ReplaySpeed,
) -> (Box<dyn Read + Send>, Box<dyn Write + Send>) {
    let reader = ReplayReader {
        records: TlogReader::new(source),
        speed,
        clock: None,
        held: None,
        pending: Vec::new(),
        taken: 0,
    };
    (Box::new(reader), Box::new(Discard))
}

/// Where the replay clock starts: the first read, and the timestamp of the first record.
struct Clock {
    start: Instant,
    first_us: u64,
}

struct ReplayReader {
    records: TlogReader<Box<dyn Read + Send>>,
    speed: ReplaySpeed,
    clock: Option<Clock>,
    /// A record that was read but is not due yet.
    held: Option<TlogRecord>,
    /// The frame being handed out, and how much of it has gone.
    pending: Vec<u8>,
    taken: usize,
}

impl ReplayReader {
    /// Waits for the record's time, at most `REPLAY_WAIT`; whether it is due now.
    fn wait_until_due(&mut self, record: &TlogRecord) -> bool {
        if self.speed == ReplaySpeed::Fast {
            return true;
        }
        let clock = self.clock.get_or_insert_with(|| Clock {
            start: Instant::now(),
            first_us: record.time_us,
        });
        let after = Duration::from_micros(record.time_us.saturating_sub(clock.first_us));
        let due = clock.start + after;
        let now = Instant::now();
        if now >= due {
            return true;
        }
        let wait = due - now;
        thread::sleep(wait.min(REPLAY_WAIT));
        Instant::now() >= due
    }
}

impl Read for ReplayReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        if self.taken == self.pending.len() {
            let record = match self.held.take() {
                Some(record) => record,
                None => match self.records.next() {
                    Some(Ok(record)) => record,
                    Some(Err(error)) => return Err(error),
                    None => return Ok(0),
                },
            };
            if !self.wait_until_due(&record) {
                self.held = Some(record);
                return Err(io::ErrorKind::TimedOut.into());
            }
            self.pending = record.frame;
            self.taken = 0;
        }
        let n = buf.len().min(self.pending.len() - self.taken);
        buf[..n].copy_from_slice(&self.pending[self.taken..self.taken + n]);
        self.taken += n;
        Ok(n)
    }
}

/// The writer half of a replay link.
struct Discard;

impl Write for Discard {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
#[path = "tlog_tests.rs"]
mod tests;
