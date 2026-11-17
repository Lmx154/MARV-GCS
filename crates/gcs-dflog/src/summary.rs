//! The one-pass summary of a log.

use std::io::{self, Read};
use std::path::Path;

use crate::format::Format;
use crate::reader::{Control, Progress, Reader, scan};

/// Unix time of the GPS epoch, 1980-01-06 00:00:00 UTC, in seconds.
const GPS_EPOCH_UNIX_S: i64 = 315_964_800;
const SECONDS_PER_WEEK: i64 = 604_800;
/// GPS time is ahead of UTC by this many seconds (as ArduPilot's readers assume).
const LEAP_SECONDS: i64 = 18;

/// How many records of one type.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TypeCount {
    pub type_id: u8,
    pub name: String,
    pub count: u64,
}

/// What one pass over a log found.
#[derive(Clone, Debug)]
pub struct Summary {
    /// Bytes consumed: records, bad bytes and tail.
    pub bytes: u64,
    /// Records of every type, FMT included.
    pub records: u64,
    /// Records per type that has at least one, by type number.
    pub counts: Vec<TypeCount>,
    /// `TimeUS` of the first record that has one, in file order.
    pub first_time_us: Option<u64>,
    /// `TimeUS` of the last record that has one, in file order.
    pub last_time_us: Option<u64>,
    /// Bytes skipped because no known record started there.
    pub bad_bytes: u64,
    /// Bytes of an incomplete last record.
    pub tail_bytes: u64,
    /// Every type the file defines, by type number (FMT included).
    pub formats: Vec<Format>,
    /// Unix time in microseconds at `TimeUS` = 0, from the first `GPS` record with `GWk` > 0.
    pub time_base_unix_us: Option<i64>,
}

impl Summary {
    /// Last minus first `TimeUS`, in microseconds.
    pub fn duration_us(&self) -> Option<u64> {
        Some(self.last_time_us?.saturating_sub(self.first_time_us?))
    }

    /// Unix time in milliseconds of a `TimeUS`, if the log has a GPS time base.
    pub fn unix_ms_at(&self, time_us: u64) -> Option<i64> {
        Some((self.time_base_unix_us? + time_us as i64).div_euclid(1000))
    }

    /// Unix time in milliseconds of the log's first `TimeUS`.
    pub fn start_unix_ms(&self) -> Option<i64> {
        self.unix_ms_at(self.first_time_us?)
    }
}

/// Unix microseconds of a GPS week and millisecond of the week, minus the leap seconds.
fn gps_to_unix_us(week: i64, ms: i64) -> i64 {
    (GPS_EPOCH_UNIX_S + SECONDS_PER_WEEK * week - LEAP_SECONDS) * 1_000_000 + ms * 1000
}

/// One pass over `reader` with progress and cancellation; `None` if `on_progress` answered `Stop`.
pub fn summary_with<R: Read>(
    reader: &mut Reader<R>,
    on_progress: impl FnMut(Progress) -> Control,
) -> io::Result<Option<Summary>> {
    let mut per_type = [0u64; 256];
    let mut first = None;
    let mut last = None;
    let mut base = None;
    let done = scan(
        reader,
        |r| {
            per_type[usize::from(r.type_id())] += 1;
            if let Some(t) = r.time_us() {
                first.get_or_insert(t);
                last = Some(t);
                if base.is_none()
                    && r.type_name() == "GPS"
                    && let (Some(wk), Some(ms)) = (r.number_of("GWk"), r.number_of("GMS"))
                    && wk > 0.0
                {
                    base = Some(gps_to_unix_us(wk as i64, ms as i64) - t as i64);
                }
            }
        },
        on_progress,
    )?;
    if !done {
        return Ok(None);
    }
    let formats: Vec<Format> = reader.formats().cloned().collect();
    let counts = formats
        .iter()
        .filter(|f| per_type[usize::from(f.type_id)] > 0)
        .map(|f| TypeCount {
            type_id: f.type_id,
            name: f.name.clone(),
            count: per_type[usize::from(f.type_id)],
        })
        .collect();
    Ok(Some(Summary {
        bytes: reader.bytes_consumed(),
        records: reader.records(),
        counts,
        first_time_us: first,
        last_time_us: last,
        bad_bytes: reader.bad_bytes(),
        tail_bytes: reader.tail_bytes(),
        formats,
        time_base_unix_us: base,
    }))
}

/// The summary of any stream; `total` is its size when known.
pub fn summary_of<R: Read>(source: R, total: Option<u64>) -> io::Result<Summary> {
    let mut reader = Reader::new(source, total);
    Ok(summary_with(&mut reader, |_| Control::Continue)?
        .expect("a pass that is never stopped completes"))
}

/// The summary of a `.BIN` file.
pub fn summary(path: impl AsRef<Path>) -> io::Result<Summary> {
    let mut reader = Reader::open(path)?;
    Ok(summary_with(&mut reader, |_| Control::Continue)?
        .expect("a pass that is never stopped completes"))
}
