//! The event timeline: see [`events_with`].

use std::collections::HashMap;
use std::io::{self, Read};
use std::path::Path;

use crate::format::Value;
use crate::reader::{Control, Progress, Reader, Record, scan};

/// A gap between two `RCIN` records longer than this ends the presence of the sticks (microseconds).
const RCIN_GAP_US: u64 = 1_000_000;

/// Where an event comes from: the record type, usable as a filter.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EventSource {
    Dev,
    Msg,
    Parm,
    Gps,
    Rcin,
    Dsf,
}

impl EventSource {
    pub const ALL: [EventSource; 6] = [
        EventSource::Dev,
        EventSource::Msg,
        EventSource::Parm,
        EventSource::Gps,
        EventSource::Rcin,
        EventSource::Dsf,
    ];

    /// The name of the record type: `DEV`, `MSG`, `PARM`, `GPS`, `RCIN`, `DSF`.
    pub fn name(self) -> &'static str {
        match self {
            EventSource::Dev => "DEV",
            EventSource::Msg => "MSG",
            EventSource::Parm => "PARM",
            EventSource::Gps => "GPS",
            EventSource::Rcin => "RCIN",
            EventSource::Dsf => "DSF",
        }
    }

    /// The source called `name` (as [`EventSource::name`] gives it).
    pub fn from_name(name: &str) -> Option<EventSource> {
        EventSource::ALL.into_iter().find(|s| s.name() == name)
    }
}

/// One thing that happened.
#[derive(Clone, Debug, PartialEq)]
pub struct Event {
    pub time_us: u64,
    pub source: EventSource,
    pub text: String,
    /// The first record of a source (or of a `DEV` row): the state the log starts in, not a change.
    pub initial: bool,
}

fn device_name(id: u32) -> String {
    match id {
        1 => "IMU1".to_owned(),
        2 => "IMU2".to_owned(),
        3 => "BARO".to_owned(),
        4 => "MAG".to_owned(),
        5 => "GNSS".to_owned(),
        6 => "BATT".to_owned(),
        7 => "RC".to_owned(),
        9 => "SD".to_owned(),
        n => format!("device {n}"),
    }
}

fn state_name(s: u32) -> String {
    match s {
        0 => "ABSENT".to_owned(),
        1 => "STARTING".to_owned(),
        2 => "OK".to_owned(),
        3 => "DEGRADED".to_owned(),
        4 => "RECOVERING".to_owned(),
        5 => "FAILED".to_owned(),
        n => format!("state {n}"),
    }
}

fn error_name(e: u32) -> String {
    match e {
        0 => "NONE".to_owned(),
        1 => "BUS".to_owned(),
        2 => "TIMEOUT".to_owned(),
        3 => "ID_MISMATCH".to_owned(),
        4 => "CONFIG_MISMATCH".to_owned(),
        5 => "POR".to_owned(),
        6 => "STUCK".to_owned(),
        n => format!("error {n}"),
    }
}

fn gps_status_name(s: u32) -> String {
    match s {
        0 => "none".to_owned(),
        1 => "no fix".to_owned(),
        2 => "2D".to_owned(),
        3 => "3D".to_owned(),
        n => format!("status {n}"),
    }
}

/// A parameter value for the eye: the shortest text of its single-precision value (the log stores `f32`).
fn param_text(v: f64) -> String {
    format!("{}", v as f32)
}

fn differs(a: f64, b: f64) -> bool {
    a != b && !(a.is_nan() && b.is_nan())
}

#[derive(Default)]
struct Tracker {
    events: Vec<Event>,
    /// Per device id: its last State and Err.
    dev: HashMap<u32, (u32, u32)>,
    params: HashMap<String, f64>,
    gps: Option<u32>,
    /// `TimeUS` of the last `RCIN` record, once there was one.
    rcin_last: Option<u64>,
    dsf: Option<f64>,
    /// `TimeUS` of the last record that has one.
    last_time: Option<u64>,
}

impl Tracker {
    fn push(&mut self, time_us: u64, source: EventSource, text: String, initial: bool) {
        self.events.push(Event {
            time_us,
            source,
            text,
            initial,
        });
    }

    fn feed(&mut self, r: &Record<'_>) {
        let Some(t) = r.time_us() else { return };
        self.last_time = Some(t);
        match r.type_name() {
            "DEV" => self.dev(r, t),
            "MSG" => {
                if let Some(Value::Text(text)) = r.get("Message") {
                    self.push(t, EventSource::Msg, text.into_owned(), false);
                }
            }
            "PARM" => self.parm(r, t),
            "GPS" => self.gps(r, t),
            "RCIN" => self.rcin(t),
            "DSF" => self.dsf(r, t),
            _ => {}
        }
    }

    fn dev(&mut self, r: &Record<'_>, t: u64) {
        let (Some(id), Some(state), Some(err)) =
            (r.number_of("Id"), r.number_of("State"), r.number_of("Err"))
        else {
            return;
        };
        let (id, state, err) = (id as u32, state as u32, err as u32);
        let name = device_name(id);
        match self.dev.insert(id, (state, err)) {
            None => {
                let mut text = format!("{name} {}", state_name(state));
                if err != 0 {
                    text.push_str(&format!(" ({})", error_name(err)));
                }
                self.push(t, EventSource::Dev, text, true);
            }
            Some((old_state, old_err)) if old_state != state => {
                let mut text =
                    format!("{name}: {} -> {}", state_name(old_state), state_name(state));
                if err != 0 || err != old_err {
                    text.push_str(&format!(" ({})", error_name(err)));
                }
                self.push(t, EventSource::Dev, text, false);
            }
            Some((_, old_err)) if old_err != err => {
                let text = format!(
                    "{name}: {} (error {} -> {})",
                    state_name(state),
                    error_name(old_err),
                    error_name(err)
                );
                self.push(t, EventSource::Dev, text, false);
            }
            Some(_) => {}
        }
    }

    fn parm(&mut self, r: &Record<'_>, t: u64) {
        let (Some(Value::Text(name)), Some(value)) = (r.get("Name"), r.number_of("Value")) else {
            return;
        };
        match self.params.get_mut(name.as_ref()) {
            None => {
                self.params.insert(name.into_owned(), value);
            }
            Some(old) => {
                if differs(*old, value) {
                    let text = format!("{name}: {} -> {}", param_text(*old), param_text(value));
                    *old = value;
                    self.push(t, EventSource::Parm, text, false);
                }
            }
        }
    }

    fn gps(&mut self, r: &Record<'_>, t: u64) {
        let Some(status) = r.number_of("Status") else {
            return;
        };
        let status = status as u32;
        match self.gps.replace(status) {
            None => self.push(
                t,
                EventSource::Gps,
                format!("GPS {}", gps_status_name(status)),
                true,
            ),
            Some(old) if old != status => self.push(
                t,
                EventSource::Gps,
                format!(
                    "GPS: {} -> {}",
                    gps_status_name(old),
                    gps_status_name(status)
                ),
                false,
            ),
            Some(_) => {}
        }
    }

    fn rcin(&mut self, t: u64) {
        match self.rcin_last.replace(t) {
            None => self.push(t, EventSource::Rcin, "sticks present".to_owned(), false),
            Some(last) if t.saturating_sub(last) > RCIN_GAP_US => {
                self.push(last, EventSource::Rcin, "sticks absent".to_owned(), false);
                self.push(t, EventSource::Rcin, "sticks present".to_owned(), false);
            }
            Some(_) => {}
        }
    }

    fn dsf(&mut self, r: &Record<'_>, t: u64) {
        let Some(dp) = r.number_of("Dp") else { return };
        match self.dsf.replace(dp) {
            None => self.push(
                t,
                EventSource::Dsf,
                format!("logger: {} records dropped so far", dp as u64),
                true,
            ),
            Some(old) if dp > old => self.push(
                t,
                EventSource::Dsf,
                format!(
                    "{} records dropped ({} in all)",
                    (dp - old) as u64,
                    dp as u64
                ),
                false,
            ),
            Some(_) => {}
        }
    }

    fn finish(mut self) -> Vec<Event> {
        if let (Some(last), Some(end)) = (self.rcin_last, self.last_time)
            && end.saturating_sub(last) > RCIN_GAP_US
        {
            self.push(last, EventSource::Rcin, "sticks absent".to_owned(), false);
        }
        self.events.sort_by_key(|e| e.time_us);
        self.events
    }
}

/// firmware's table (`fw-core/src/dflog.rs`; the meanings of the numbers are in `marv.xml`).
///
/// The rules, each on the record type of that name (a log without the type, or of another firmware whose
/// type of that name lacks the columns, simply gives no events of that source):
///
/// - `DEV`: per row (`Id`), a `State` or `Err` that differs from that row's previous record. Counters
///   growing is not an event. The row's first record gives one "initial" event (`IMU1 OK`).
/// - `MSG`: every text.
/// - `PARM`: a `Value` that differs from the last one seen for that `Name`. A name's first record is its
///   starting value, not an event (there is no initial event per parameter).
/// - `GPS`: `Status` differs from the previous `GPS` record (0 none, 1 no fix, 2 2D, 3 3D). The first
///   record gives an initial event (`GPS no fix`).
/// - `RCIN`: "sticks present" at the first record, or at the first after a gap of more than 1 s (the
///   firmware's freshness limit; a gap of exactly 1 s is none); "sticks absent" at the last record before such
///   a gap, or at the last record before the log's end when the log goes on for more than 1 s after it.
/// - `DSF`: `Dp` greater than in the previous record: "N records dropped (M in all)". The first record gives an
///   initial event with the count so far.
///
/// No coordinate or any other `GPS` field but `Status` appears in an event. Events are in order of `TimeUS`
/// (a stable sort: equal times keep the order of the file).
///
/// One pass over `reader` with progress and cancellation; `None` if `on_progress` answered `Stop`.
pub fn events_with<R: Read>(
    reader: &mut Reader<R>,
    on_progress: impl FnMut(Progress) -> Control,
) -> io::Result<Option<Vec<Event>>> {
    let mut tracker = Tracker::default();
    let done = scan(reader, |r| tracker.feed(r), on_progress)?;
    Ok(done.then(|| tracker.finish()))
}

/// The events of any stream; `total` is its size when known.
pub fn events_of<R: Read>(source: R, total: Option<u64>) -> io::Result<Vec<Event>> {
    let mut reader = Reader::new(source, total);
    Ok(events_with(&mut reader, |_| Control::Continue)?
        .expect("a pass that is never stopped completes"))
}

/// The events of a `.BIN` file.
pub fn events(path: impl AsRef<Path>) -> io::Result<Vec<Event>> {
    let mut reader = Reader::open(path)?;
    Ok(events_with(&mut reader, |_| Control::Continue)?
        .expect("a pass that is never stopped completes"))
}
