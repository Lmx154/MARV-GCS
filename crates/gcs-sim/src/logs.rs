//! The logs on the simulated card, and how the vehicle serves them (`docs/link-contract.md`,
//! section 11): LOG_REQUEST_LIST, LOG_REQUEST_DATA and LOG_REQUEST_END.
//!
//! The bytes of a log are a pure function of its number and offset, so a test needs no copy of them:
//! [`log_bytes`].

use std::collections::VecDeque;

/// The vehicle sends this many LOG_ENTRY or LOG_DATA frames per [`SLOT_MS`] (the firmware's stream
/// slot: about 72 kB/s of log data).
pub(crate) const SLOT_FRAMES: usize = 4;
pub(crate) const SLOT_MS: u64 = 5;
/// A transfer stops when the station's HEARTBEAT has not been seen for this long (ArduPilot's rule).
pub(crate) const HEARTBEAT_LOST_MS: u64 = 3000;
/// The most bytes of one LOG_DATA.
pub(crate) const FRAME_BYTES: u32 = 90;

/// One log on the simulated card.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimLog {
    /// Its number on the card; numbers have gaps.
    pub number: u16,
    pub size: u32,
}

/// Loses the LOG_DATA frame whose offset is `ofs`, the first `times` times it would be sent (any log).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogDataDrop {
    pub ofs: u32,
    pub times: u32,
}

/// Five logs with gaps in their numbers: 2 of 12 345 bytes (not a multiple of 90), 3 empty, 5 of 9 000,
/// 8 of 60 001 (more than one window of 46 080) and 11, the one being written, of 450.
pub fn default_logs() -> Vec<SimLog> {
    [(2, 12_345), (3, 0), (5, 9_000), (8, 60_001), (11, 450)]
        .into_iter()
        .map(|(number, size)| SimLog { number, size })
        .collect()
}

/// The byte at `ofs` of log `number`.
fn log_byte(number: u16, ofs: u32) -> u8 {
    (ofs.wrapping_mul(131) ^ (ofs >> 8) ^ (u32::from(number) * 17)) as u8
}

/// The bytes of log `number`, `size` of them: the same for every call.
pub fn log_bytes(number: u16, size: u32) -> Vec<u8> {
    log_bytes_range(number, size, 0, size)
}

/// `count` bytes of log `number` from `ofs`, as far as the log of `size` bytes goes.
pub(crate) fn log_bytes_range(number: u16, size: u32, ofs: u32, count: u32) -> Vec<u8> {
    (ofs..ofs.saturating_add(count).min(size))
        .map(|i| log_byte(number, i))
        .collect()
}

/// The transfer that runs: LOG_DATA from `next` up to `end`.
pub(crate) struct Transfer {
    pub number: u16,
    pub next: u32,
    pub end: u32,
    pub started_ms: u64,
}

/// A frame waiting for a place in a slot.
pub(crate) enum Queued {
    Entry {
        id: u16,
        num_logs: u16,
        last_log_num: u16,
        size: u32,
    },
    /// LOG_DATA with count 0, the answer to a request at or past the end.
    EndMarker { id: u16, ofs: u32 },
}

/// What the vehicle has to send through its slot.
#[derive(Default)]
pub(crate) struct Outbox {
    pub queue: VecDeque<Queued>,
    pub transfer: Option<Transfer>,
    /// When the next slot is due.
    pub due_ms: u64,
    /// The drops still to make.
    pub drops: Vec<LogDataDrop>,
}

impl Outbox {
    /// Whether the frame of a LOG_DATA at `ofs` is lost this time.
    pub fn drops_frame(&mut self, ofs: u32) -> bool {
        match self.drops.iter_mut().find(|d| d.ofs == ofs && d.times > 0) {
            Some(drop) => {
                drop.times -= 1;
                true
            }
            None => false,
        }
    }
}
