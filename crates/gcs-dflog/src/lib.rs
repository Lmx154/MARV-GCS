//! A generic, streaming reader of ArduPilot DataFlash `.BIN` logs, and a one-pass summary.
//!
//! The file is a sequence of records `A3 95 <type> <payload>`, little-endian. Type 128 is FMT
//! (89 bytes: Type u8, Length u8 (the whole record), Name char[4], Format char[16], Columns char[64]); it
//! defines every other type, so the reader knows nothing about any particular record. It follows
//! ArduPilot's own reader (pymavlink `DFReader.py`): a record starts at `A3 95` followed by a type whose FMT
//! has been seen, anything else is skipped one byte at a time and counted as bad bytes, and an incomplete
//! last record is not an error: its bytes are counted as the tail.
//!
//! One pass, constant memory: [`Reader::next_record`] hands out a [`Record`] that borrows the reader's
//! buffer and decodes a field only when asked, so a 100 MB log is never held decoded. [`scan`] runs a
//! pass with progress and cancellation; [`summary`] is one such pass.
//!
//! This crate has no GUI and no dependency; it is the station's own reader, not the firmware's `fw-core`
//! (which only writes our fixed table).

mod format;
mod reader;
mod summary;

#[cfg(test)]
mod tests;

pub use format::{Format, Value};
pub use reader::{Control, Progress, Reader, Record, scan};
pub use summary::{Summary, TypeCount, summary, summary_of, summary_with};
