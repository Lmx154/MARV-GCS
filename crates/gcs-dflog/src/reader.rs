//! The streaming reader: resynchronisation, truncated tail, progress and cancellation.

use std::fs::File;
use std::io::{self, Read};
use std::path::Path;

use crate::format::{Format, Value};

const HEAD1: u8 = 0xA3;
const HEAD2: u8 = 0x95;
const FMT_TYPE: u8 = 128;
const BUFFER: usize = 256 * 1024;
/// The longest record a FMT can describe (its Length is a u8).
const MAX_RECORD: usize = 255;
/// A progress report is made after at least this many bytes since the last one.
const PROGRESS_STEP: u64 = 256 * 1024;

/// How far a pass has got.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Progress {
    /// Bytes of the file consumed so far (records, bad bytes and tail).
    pub bytes: u64,
    /// The size of the file when known.
    pub total: Option<u64>,
}

/// What a progress callback answers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Control {
    Continue,
    Stop,
}

/// One record: a borrowed view of the reader's buffer, valid until the next call of
/// [`Reader::next_record`]. Fields are decoded only when asked for.
#[derive(Clone, Copy)]
pub struct Record<'a> {
    format: &'a Format,
    payload: &'a [u8],
    offset: u64,
}

impl<'a> Record<'a> {
    pub fn type_id(&self) -> u8 {
        self.format.type_id
    }

    pub fn type_name(&self) -> &'a str {
        &self.format.name
    }

    pub fn format(&self) -> &'a Format {
        self.format
    }

    /// The column names, as the FMT gave them.
    pub fn columns(&self) -> &'a [String] {
        &self.format.columns
    }

    /// The number of fields.
    pub fn len(&self) -> usize {
        self.format.field_count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Offset of the record's first byte (`A3`) in the file.
    pub fn offset(&self) -> u64 {
        self.offset
    }

    /// The field at `index`.
    pub fn value(&self, index: usize) -> Option<Value<'a>> {
        self.format.value(self.payload, index)
    }

    /// The field in the column called `name`.
    pub fn get(&self, name: &str) -> Option<Value<'a>> {
        self.value(self.format.column_index(name)?)
    }

    /// The numeric field at `index` after scaling; `None` for a string, an array or no such field.
    pub fn number(&self, index: usize) -> Option<f64> {
        self.format.number(self.payload, index)
    }

    /// The numeric field in the column called `name`.
    pub fn number_of(&self, name: &str) -> Option<f64> {
        self.number(self.format.column_index(name)?)
    }

    /// The `TimeUS` column, in microseconds, if the type has a numeric one.
    pub fn time_us(&self) -> Option<u64> {
        let t = self.number(self.format.time_index()?)?;
        (t >= 0.0).then_some(t as u64)
    }
}

/// Reads records from any `Read`, one at a time, in constant memory.
pub struct Reader<R> {
    inner: R,
    buf: Vec<u8>,
    pos: usize,
    end: usize,
    eof: bool,
    consumed: u64,
    total: Option<u64>,
    formats: Vec<Option<Format>>,
    records: u64,
    bad_bytes: u64,
    tail_bytes: u64,
}

impl Reader<File> {
    /// Opens a file; the progress total is its size.
    pub fn open(path: impl AsRef<Path>) -> io::Result<Reader<File>> {
        let file = File::open(path)?;
        let total = file.metadata()?.len();
        Ok(Reader::new(file, Some(total)))
    }
}

impl<R: Read> Reader<R> {
    /// `total` is the size of the stream when known (for progress).
    pub fn new(inner: R, total: Option<u64>) -> Reader<R> {
        let mut formats: Vec<Option<Format>> = (0..256).map(|_| None).collect();
        formats[usize::from(FMT_TYPE)] = Some(Format::fmt_of_fmt());
        Reader {
            inner,
            buf: vec![0; BUFFER],
            pos: 0,
            end: 0,
            eof: false,
            consumed: 0,
            total,
            formats,
            records: 0,
            bad_bytes: 0,
            tail_bytes: 0,
        }
    }

    /// Bytes of the stream consumed so far.
    pub fn bytes_consumed(&self) -> u64 {
        self.consumed
    }

    pub fn progress(&self) -> Progress {
        Progress {
            bytes: self.consumed,
            total: self.total,
        }
    }

    /// Records returned so far.
    pub fn records(&self) -> u64 {
        self.records
    }

    /// Bytes skipped so far because no known record started there.
    pub fn bad_bytes(&self) -> u64 {
        self.bad_bytes
    }

    /// Bytes of an incomplete last record (at the end of the stream, once reached).
    pub fn tail_bytes(&self) -> u64 {
        self.tail_bytes
    }

    /// The record types defined so far, by type number.
    pub fn formats(&self) -> impl Iterator<Item = &Format> {
        self.formats.iter().flatten()
    }

    /// The definition of a type, if its FMT has been seen.
    pub fn format_of(&self, type_id: u8) -> Option<&Format> {
        self.formats[usize::from(type_id)].as_ref()
    }

    /// Makes at least `need` bytes available if the stream has them; returns how many are.
    fn ensure(&mut self, need: usize) -> io::Result<usize> {
        if self.end - self.pos < need && !self.eof {
            if self.buf.len() - self.end < BUFFER / 4 {
                self.buf.copy_within(self.pos..self.end, 0);
                self.end -= self.pos;
                self.pos = 0;
            }
            while self.end - self.pos < need {
                match self.inner.read(&mut self.buf[self.end..]) {
                    Ok(0) => {
                        self.eof = true;
                        break;
                    }
                    Ok(n) => self.end += n,
                    Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                    Err(e) => return Err(e),
                }
            }
        }
        Ok(self.end - self.pos)
    }

    fn consume(&mut self, n: usize) {
        self.pos += n;
        self.consumed += n as u64;
    }

    /// Finds the next record; returns its type and where it starts in the buffer, and consumes it.
    fn advance(&mut self) -> io::Result<Option<(u8, usize, usize)>> {
        loop {
            let avail = self.ensure(3)?;
            if avail < 3 {
                self.tail_bytes += avail as u64;
                self.consume(avail);
                return Ok(None);
            }
            let (a, b, t) = (
                self.buf[self.pos],
                self.buf[self.pos + 1],
                self.buf[self.pos + 2],
            );
            if a == HEAD1 && b == HEAD2 {
                if let Some(len) = self.formats[usize::from(t)]
                    .as_ref()
                    .map(|f| usize::from(f.length))
                {
                    let avail = self.ensure(len.min(MAX_RECORD))?;
                    if avail < len {
                        self.tail_bytes += avail as u64;
                        self.consume(avail);
                        return Ok(None);
                    }
                    let start = self.pos;
                    self.consume(len);
                    return Ok(Some((t, start, len)));
                }
            }
            self.bad_bytes += 1;
            self.consume(1);
        }
    }

    /// The next record, or `None` at the end of the stream. A FMT record is returned like any other,
    /// after it has defined its type.
    pub fn next_record(&mut self) -> io::Result<Option<Record<'_>>> {
        let Some((t, start, len)) = self.advance()? else {
            return Ok(None);
        };
        let payload_start = start + 3;
        if t == FMT_TYPE {
            if let Some(f) = Format::from_fmt_payload(&self.buf[payload_start..start + len]) {
                // A FMT does not redefine FMT itself.
                if f.type_id != FMT_TYPE {
                    self.formats[usize::from(f.type_id)] = Some(f);
                }
            }
        }
        self.records += 1;
        match self.formats[usize::from(t)].as_ref() {
            Some(format) => Ok(Some(Record {
                format,
                payload: &self.buf[payload_start..start + len],
                offset: self.consumed - len as u64,
            })),
            None => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "record type without a format",
            )),
        }
    }
}

/// Runs one pass: `on_record` for every record, `on_progress` after at least 256 KiB since the last
/// report and once at the end. Returns `false` if `on_progress` answered `Stop` (the pass is then
/// incomplete), `true` when the stream was read to its end.
pub fn scan<R: Read>(
    reader: &mut Reader<R>,
    mut on_record: impl FnMut(&Record<'_>),
    mut on_progress: impl FnMut(Progress) -> Control,
) -> io::Result<bool> {
    let mut reported = reader.bytes_consumed();
    loop {
        match reader.next_record()? {
            Some(record) => on_record(&record),
            None => {
                on_progress(reader.progress());
                return Ok(true);
            }
        }
        if reader.bytes_consumed() - reported >= PROGRESS_STEP {
            reported = reader.bytes_consumed();
            if on_progress(reader.progress()) == Control::Stop {
                return Ok(false);
            }
        }
    }
}
