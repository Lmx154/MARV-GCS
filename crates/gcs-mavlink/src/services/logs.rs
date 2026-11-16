//! Log download service: lists the logs on the vehicle's card and pulls one of them, in windows, to a
//! writer (docs/link-contract.md, section 11). Sans-IO: the time comes from `cx.now`.

use std::collections::BTreeMap;
use std::fmt;
use std::io::Write;
use std::time::Duration;

use super::{Ctx, Service};
use crate::dialect::{
    LOG_DATA_DATA, LOG_ENTRY_DATA, LOG_REQUEST_DATA_DATA, LOG_REQUEST_END_DATA,
    LOG_REQUEST_LIST_DATA,
};
use crate::{Body, Command, Frame, LinkStatus, Message};

/// The most bytes one LOG_DATA carries.
pub const FRAME_BYTES: u32 = 90;
/// The station asks for this many frames at a time.
pub const WINDOW_FRAMES: u32 = 512;
/// The bytes of one window.
pub const WINDOW_BYTES: u32 = WINDOW_FRAMES * FRAME_BYTES;
/// A window that has not moved the first missing offset for this long is asked for again.
pub const RETRY_AFTER: Duration = Duration::from_secs(1);
/// Fruitless re-requests of the same first missing offset before the download is given up.
pub const RETRIES: u8 = 5;
/// A list with no new LOG_ENTRY for this long is asked for again.
pub const LIST_QUIET: Duration = Duration::from_secs(1);
/// LIST requests in all, the first included (the first, then three more: as the parameter list).
pub const LIST_ASKS: u8 = 4;
/// The last log number of a request for all of them.
const ALL_LOGS: u16 = 0xFFFF;

/// One log on the card.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogInfo {
    /// Its number on the card (`LOG_ENTRY.id`); numbers have gaps.
    pub number: u16,
    /// Bytes; for the log being written, as of its last flush.
    pub size: u32,
}

/// What a listing found: the logs in ascending number order, and the number of the log being written.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LogList {
    pub logs: Vec<LogInfo>,
    pub last_log_num: u16,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ListError {
    /// Not asked: no vehicle is connected.
    NoVehicle,
    /// Not one LOG_ENTRY after every ask.
    NoAnswer,
    /// Some entries arrived, not as many as `num_logs` said, after every ask.
    Incomplete { received: usize, expected: usize },
}

impl fmt::Display for ListError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ListError::NoVehicle => f.write_str("the vehicle is not connected"),
            ListError::NoAnswer => f.write_str("the vehicle did not answer the request"),
            ListError::Incomplete { received, expected } => {
                write!(f, "only {received} of {expected} log entries arrived")
            }
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum ListStatus {
    /// Nothing asked yet.
    #[default]
    Idle,
    /// Asked; `expected` is `num_logs` of the first entry that arrived.
    Listing {
        received: usize,
        expected: Option<usize>,
    },
    Done(LogList),
    Failed(ListError),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DownloadError {
    /// Not started: no vehicle is connected.
    NoVehicle,
    /// The first missing byte did not arrive after [`RETRIES`] re-requests. The vehicle is silent, or
    /// the log does not exist (it answers a request for a log it does not have with nothing at all).
    Stalled { offset: u32 },
    /// The writer failed; the text is its error.
    Sink(String),
}

impl fmt::Display for DownloadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DownloadError::NoVehicle => f.write_str("the vehicle is not connected"),
            DownloadError::Stalled { offset } => write!(
                f,
                "no data from the vehicle at byte {offset} after {RETRIES} new requests"
            ),
            DownloadError::Sink(why) => write!(f, "cannot write the log: {why}"),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DownloadStatus {
    Running,
    /// Every byte up to the size is in the writer, and the writer was flushed.
    Done,
    Failed(DownloadError),
    /// `Command::CancelLogDownload`.
    Cancelled,
}

/// The current or last download. `done` of `size` bytes is the progress.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DownloadState {
    pub number: u16,
    /// The size the download was started with; it is not changed if the log grows.
    pub size: u32,
    /// Bytes handed to the writer, in order: the first missing offset.
    pub done: u32,
    pub status: DownloadStatus,
    /// LOG_REQUEST_DATA sent, the retries included.
    pub requests: u32,
    pub started: Duration,
    /// Session time at which it ended; `None` while it runs.
    pub ended: Option<Duration>,
}

/// What the window shows of this service.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LogsState {
    pub list: ListStatus,
    pub download: Option<DownloadState>,
    /// Requests refused because a listing or a download was running.
    pub refused: u32,
}

struct ListTrack {
    target: (u8, u8),
    entries: BTreeMap<u16, u32>,
    expected: Option<usize>,
    last_log_num: u16,
    asks: u8,
    /// Session time of the last ask or the last new entry.
    since: Duration,
}

struct Transfer {
    target: (u8, u8),
    number: u16,
    size: u32,
    /// The first missing offset: every byte below it is in the writer.
    next: u32,
    /// The end of the window asked for last.
    window_end: u32,
    /// Frames received beyond `next`, inside the window, by offset.
    pending: BTreeMap<u32, Vec<u8>>,
    writer: Box<dyn Write + Send>,
    /// Session time of the last ask, or of the last move of `next`.
    since: Duration,
    /// Re-requests since `next` last moved.
    retries: u8,
}

#[derive(Default)]
pub struct Logs {
    list: Option<ListTrack>,
    transfer: Option<Transfer>,
}

impl Logs {
    pub fn new() -> Self {
        Self::default()
    }

    fn target(cx: &Ctx) -> Option<(u8, u8)> {
        let vehicle = cx.state.link.vehicle.as_ref()?;
        (cx.state.link.status == LinkStatus::Connected)
            .then_some((vehicle.system_id, vehicle.component_id))
    }

    fn ask_list(target: (u8, u8), cx: &mut Ctx) {
        cx.out
            .push(Message::LOG_REQUEST_LIST(LOG_REQUEST_LIST_DATA {
                start: 0,
                end: ALL_LOGS,
                target_system: target.0,
                target_component: target.1,
            }));
    }

    fn start_list(&mut self, cx: &mut Ctx) {
        if self.list.is_some() || self.transfer.is_some() {
            cx.state.logs.refused += 1;
            cx.changed = true;
            return;
        }
        cx.changed = true;
        let Some(target) = Self::target(cx) else {
            cx.state.logs.list = ListStatus::Failed(ListError::NoVehicle);
            return;
        };
        Self::ask_list(target, cx);
        cx.state.logs.list = ListStatus::Listing {
            received: 0,
            expected: None,
        };
        self.list = Some(ListTrack {
            target,
            entries: BTreeMap::new(),
            expected: None,
            last_log_num: 0,
            asks: 1,
            since: cx.now,
        });
    }

    fn on_entry(&mut self, entry: &LOG_ENTRY_DATA, cx: &mut Ctx) {
        let Some(track) = self.list.as_mut() else {
            return;
        };
        if entry.id == 0 && entry.num_logs == 0 {
            // The vehicle's "no logs": an empty list, not an error.
            let last_log_num = entry.last_log_num;
            self.finish_list(
                Ok(LogList {
                    logs: Vec::new(),
                    last_log_num,
                }),
                cx,
            );
            return;
        }
        let expected = usize::from(entry.num_logs);
        if track.entries.insert(entry.id, entry.size).is_none() {
            track.since = cx.now;
        }
        track.expected = Some(expected);
        track.last_log_num = entry.last_log_num;
        let received = track.entries.len();
        if received >= expected {
            let list = LogList {
                logs: track
                    .entries
                    .iter()
                    .map(|(&number, &size)| LogInfo { number, size })
                    .collect(),
                last_log_num: track.last_log_num,
            };
            self.finish_list(Ok(list), cx);
        } else {
            cx.state.logs.list = ListStatus::Listing {
                received,
                expected: Some(expected),
            };
            cx.changed = true;
        }
    }

    fn finish_list(&mut self, result: Result<LogList, ListError>, cx: &mut Ctx) {
        self.list = None;
        cx.state.logs.list = match result {
            Ok(list) => ListStatus::Done(list),
            Err(error) => ListStatus::Failed(error),
        };
        cx.changed = true;
    }

    fn tick_list(&mut self, cx: &mut Ctx) {
        let Some(track) = self.list.as_mut() else {
            return;
        };
        if cx.now.saturating_sub(track.since) < LIST_QUIET {
            return;
        }
        if track.asks < LIST_ASKS {
            Self::ask_list(track.target, cx);
            track.asks += 1;
            track.since = cx.now;
            return;
        }
        let error = match track.expected {
            None => ListError::NoAnswer,
            Some(expected) => ListError::Incomplete {
                received: track.entries.len(),
                expected,
            },
        };
        self.finish_list(Err(error), cx);
    }

    fn start_download(
        &mut self,
        number: u16,
        size: u32,
        writer: Option<Box<dyn Write + Send>>,
        cx: &mut Ctx,
    ) {
        if self.list.is_some() || self.transfer.is_some() {
            cx.state.logs.refused += 1;
            cx.changed = true;
            return;
        }
        let mut state = DownloadState {
            number,
            size,
            done: 0,
            status: DownloadStatus::Running,
            requests: 0,
            started: cx.now,
            ended: None,
        };
        cx.changed = true;
        let target = Self::target(cx);
        let failure = match (&target, &writer) {
            (None, _) => Some(DownloadError::NoVehicle),
            (_, None) => Some(DownloadError::Sink(
                "the writer was taken before".to_string(),
            )),
            _ => None,
        };
        if let Some(error) = failure {
            state.status = DownloadStatus::Failed(error);
            state.ended = Some(cx.now);
            cx.state.logs.download = Some(state);
            return;
        }
        let (Some(target), Some(writer)) = (target, writer) else {
            return;
        };
        cx.state.logs.download = Some(state);
        let transfer = Transfer {
            target,
            number,
            size,
            next: 0,
            window_end: 0,
            pending: BTreeMap::new(),
            writer,
            since: cx.now,
            retries: 0,
        };
        if size == 0 {
            // Nothing to ask for.
            self.transfer = Some(transfer);
            self.complete(cx);
            return;
        }
        self.transfer = Some(transfer);
        self.request(cx);
    }

    /// Asks for the window that starts at the first missing offset.
    fn request(&mut self, cx: &mut Ctx) {
        let Some(transfer) = self.transfer.as_mut() else {
            return;
        };
        transfer.window_end = transfer
            .size
            .min(transfer.next.saturating_add(WINDOW_BYTES));
        transfer.since = cx.now;
        cx.out
            .push(Message::LOG_REQUEST_DATA(LOG_REQUEST_DATA_DATA {
                ofs: transfer.next,
                count: transfer.window_end - transfer.next,
                id: transfer.number,
                target_system: transfer.target.0,
                target_component: transfer.target.1,
            }));
        if let Some(download) = cx.state.logs.download.as_mut() {
            download.requests += 1;
        }
    }

    /// Ends the transfer with a failure or a cancel: tells the vehicle to stop, records the end.
    fn abandon(&mut self, status: DownloadStatus, cx: &mut Ctx) {
        let Some(mut transfer) = self.transfer.take() else {
            return;
        };
        let _ = transfer.writer.flush();
        cx.out.push(Message::LOG_REQUEST_END(LOG_REQUEST_END_DATA {
            target_system: transfer.target.0,
            target_component: transfer.target.1,
        }));
        if let Some(download) = cx.state.logs.download.as_mut() {
            download.status = status;
            download.ended = Some(cx.now);
        }
        cx.changed = true;
    }

    fn complete(&mut self, cx: &mut Ctx) {
        let Some(mut transfer) = self.transfer.take() else {
            return;
        };
        let status = match transfer.writer.flush() {
            Ok(()) => DownloadStatus::Done,
            Err(error) => DownloadStatus::Failed(DownloadError::Sink(error.to_string())),
        };
        if let Some(download) = cx.state.logs.download.as_mut() {
            download.status = status;
            download.ended = Some(cx.now);
        }
        cx.changed = true;
    }

    fn on_data(&mut self, data: &LOG_DATA_DATA, cx: &mut Ctx) {
        let Some(transfer) = self.transfer.as_mut() else {
            return;
        };
        let count = usize::from(data.count).min(data.data.len());
        if data.id != transfer.number || count == 0 {
            return;
        }
        let start = u64::from(data.ofs);
        let end = (start + count as u64).min(u64::from(transfer.window_end));
        // Beyond the window, or nothing new: ignored.
        if start >= u64::from(transfer.window_end) || end <= u64::from(transfer.next) {
            return;
        }
        let from = start.max(u64::from(transfer.next));
        let bytes = &data.data[(from - start) as usize..(end - start) as usize];
        let from = from as u32;
        if from > transfer.next {
            let keep = transfer
                .pending
                .get(&from)
                .is_none_or(|old| old.len() < bytes.len());
            if keep {
                transfer.pending.insert(from, bytes.to_vec());
            }
            return;
        }
        let before = transfer.next;
        let mut failure = Self::write(transfer, bytes);
        while failure.is_none()
            && let Some(entry) = transfer.pending.first_entry()
            && *entry.key() <= transfer.next
        {
            let (at, held) = entry.remove_entry();
            let skip = (transfer.next - at) as usize;
            if skip < held.len() {
                failure = Self::write(transfer, &held[skip..]);
            }
        }
        let (next, window_end, size) = (transfer.next, transfer.window_end, transfer.size);
        if next > before {
            transfer.retries = 0;
            transfer.since = cx.now;
        }
        if let Some(download) = cx.state.logs.download.as_mut() {
            download.done = next;
        }
        cx.changed = true;
        if let Some(error) = failure {
            self.abandon(DownloadStatus::Failed(DownloadError::Sink(error)), cx);
        } else if next >= size {
            self.complete(cx);
        } else if next >= window_end {
            self.request(cx);
        }
    }

    /// Hands `bytes` to the writer and moves `next`; the writer's error as text.
    fn write(transfer: &mut Transfer, bytes: &[u8]) -> Option<String> {
        match transfer.writer.write_all(bytes) {
            Ok(()) => {
                transfer.next += bytes.len() as u32;
                None
            }
            Err(error) => Some(error.to_string()),
        }
    }

    fn tick_transfer(&mut self, cx: &mut Ctx) {
        let Some(transfer) = self.transfer.as_mut() else {
            return;
        };
        if cx.now.saturating_sub(transfer.since) < RETRY_AFTER {
            return;
        }
        if transfer.retries >= RETRIES {
            let offset = transfer.next;
            self.abandon(
                DownloadStatus::Failed(DownloadError::Stalled { offset }),
                cx,
            );
            return;
        }
        transfer.retries += 1;
        self.request(cx);
        cx.changed = true;
    }
}

impl Service for Logs {
    fn on_frame(&mut self, frame: &Frame, cx: &mut Ctx) {
        let Body::Known(message) = &frame.body else {
            return;
        };
        let from_vehicle = |target: (u8, u8)| frame.system_id == target.0;
        match message {
            Message::LOG_ENTRY(entry)
                if self.list.as_ref().is_some_and(|t| from_vehicle(t.target)) =>
            {
                self.on_entry(entry, cx);
            }
            Message::LOG_DATA(data)
                if self
                    .transfer
                    .as_ref()
                    .is_some_and(|t| from_vehicle(t.target)) =>
            {
                self.on_data(data, cx);
            }
            _ => {}
        }
    }

    fn on_tick(&mut self, cx: &mut Ctx) {
        self.tick_list(cx);
        self.tick_transfer(cx);
    }

    fn on_command(&mut self, command: &Command, cx: &mut Ctx) {
        match command {
            Command::ListLogs => self.start_list(cx),
            Command::DownloadLog { number, size, sink } => {
                self.start_download(*number, *size, sink.take(), cx)
            }
            Command::CancelLogDownload => self.abandon(DownloadStatus::Cancelled, cx),
            _ => {}
        }
    }
}

#[cfg(test)]
#[path = "logs_tests.rs"]
mod tests;
