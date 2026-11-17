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
//! On top of [`scan`] there are three more one-pass operations, each with progress and cancellation:
//!
//! - [`events_with`] (and [`events`], [`events_of`]): the event timeline of a log of the MARV firmware (device
//!   state changes, status texts, parameter changes, GPS fix changes, sticks present, dropped records); the rules
//!   are in the documentation of [`events_with`].
//! - [`Condition`]: a small language such as `BAT.Volt < 14.8`, `IMU[0].AccZ < -15` or
//!   `DEV[7].Errs > 100 and GPS.Status < 3`, parsed with [`Condition::parse`], bound to the log's record types
//!   with [`Condition::bind`] (so a wrong name is an error before the pass) and run with [`find_with`], which
//!   returns the intervals where it held, with the smallest and largest value of each field named. The syntax
//!   and the semantics (pymavlink's: evaluated over the latest record of each named type) are in the
//!   documentation of [`Condition`].
//! - [`export_with`]: the records of chosen types written as CSV files, one per type, optionally at a limited
//!   rate per type and instance and only where a condition holds; the file names and the format are in its
//!   documentation.
//!
//! This crate has no GUI and no dependency; it is the station's own reader, not the firmware's `fw-core`
//! (which only writes our fixed table).

mod condition;
mod events;
mod export;
mod format;
mod reader;
mod summary;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_conditions;
#[cfg(test)]
mod tests_events;
#[cfg(test)]
mod tests_export;

pub use condition::{
    BoundCondition, Condition, ConditionError, FieldRange, Found, Interval, MAX_INTERVALS,
    find_with,
};
pub use events::{Event, EventSource, events, events_of, events_with};
pub use export::{Export, ExportError, ExportRequest, TypeExport, export_with};
pub use format::{Format, Value};
pub use reader::{Control, Progress, Reader, Record, scan};
pub use summary::{Summary, TypeCount, summary, summary_of, summary_with};
