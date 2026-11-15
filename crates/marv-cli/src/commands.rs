//! The commands, run on a started session. Every wait is bounded by a deadline.

use std::fs::OpenOptions;
use std::io::{self, Write};
use std::time::{Duration, Instant};

use gcs_mavlink::services::messages::{FieldValue, MessageEntry};
use gcs_mavlink::services::params::FetchStatus;
use gcs_mavlink::services::texts::TextEntry;
use gcs_mavlink::tlog::LogSink;
use gcs_mavlink::{
    Access, Command as SessionCommand, CommandRecord, CommandStatus, LinkStatus, Refused, Session,
    VehicleState,
};

use crate::args::{
    Calibration, Command, DEFAULT_DEVICES, DEFAULT_RECORD, DEFAULT_TEXTS, DEFAULT_WATCH,
    Invocation, mav_command,
};
use crate::exit;

const POLL: Duration = Duration::from_millis(10);
/// How long `param set` waits for the vehicle's reason after it saw the refusal.
const REASON_GRACE: Duration = Duration::from_millis(300);
/// How long the parameter list may take.
const LIST_DEADLINE: Duration = Duration::from_secs(30);
/// How long a parameter write waits for the vehicle's echo (the session retries three times).
const WRITE_DEADLINE: Duration = Duration::from_secs(8);
/// The longest a command is followed (the compass calibration takes up to 40 s on the vehicle); the
/// session itself ends one by 20 s of silence.
const COMMAND_DEADLINE: Duration = Duration::from_secs(90);
const DEVICE_MESSAGE: &str = "MARV_DEVICE_STATUS";
/// The last texts `status` shows.
const STATUS_TEXTS: usize = 3;

/// Why a command ended without success: the exit status and what to say on stderr.
#[derive(Debug)]
pub struct Fail {
    pub code: i32,
    pub message: String,
}

impl Fail {
    fn new(code: i32, message: impl Into<String>) -> Self {
        Fail {
            code,
            message: message.into(),
        }
    }
}

impl From<io::Error> for Fail {
    /// A closed pipe (`| head`) ends the command quietly; any other write error is a failure.
    fn from(error: io::Error) -> Self {
        if error.kind() == io::ErrorKind::BrokenPipe {
            Fail::new(exit::OK, "")
        } else {
            Fail::new(exit::FAILED, format!("cannot write the output: {error}"))
        }
    }
}

type Outcome = Result<(), Fail>;

/// Runs the invocation's command on `session`, whose role is `access`; returns the exit status.
pub fn execute(
    invocation: &Invocation,
    session: &Session,
    access: Access,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> i32 {
    let on = !matches!(
        invocation.command,
        Command::Texts | Command::Watch | Command::Status
    );
    let mut echo = Echo::new(err, on);
    let result = run_command(invocation, session, access, out, &mut echo);
    echo.pump(session);
    let err = echo.err;
    let code = match result {
        Ok(()) => exit::OK,
        Err(fail) => {
            if !fail.message.is_empty() {
                let _ = writeln!(err, "marv-cli: {}", fail.message);
            }
            fail.code
        }
    };
    let _ = out.flush();
    code
}

fn run_command(
    invocation: &Invocation,
    session: &Session,
    access: Access,
    out: &mut dyn Write,
    echo: &mut Echo,
) -> Outcome {
    wait_for_vehicle(session, invocation.timeout, echo)?;
    let duration = |default| invocation.duration.unwrap_or(default);
    match &invocation.command {
        Command::Status => status(session, access, duration(DEFAULT_DEVICES), out, echo),
        Command::Devices => devices(session, duration(DEFAULT_DEVICES), out, echo),
        Command::Texts => texts(session, duration(DEFAULT_TEXTS), out),
        Command::Watch => watch(session, duration(DEFAULT_WATCH), out),
        Command::ParamList => {
            wait_for_params(session, access, echo)?;
            let state = session.state();
            for param in state.params.params.values() {
                writeln!(
                    out,
                    "{}\t{}\t{}",
                    param.name,
                    param.value(state.params.encoding),
                    param.kind.name()
                )?;
            }
            Ok(())
        }
        Command::ParamGet(name) => {
            wait_for_params(session, access, echo)?;
            let state = session.state();
            let Some(param) = state.params.params.get(name) else {
                return Err(Fail::new(exit::FAILED, format!("unknown parameter {name}")));
            };
            writeln!(
                out,
                "{}\t{}\t{}",
                param.name,
                param.value(state.params.encoding),
                param.kind.name()
            )?;
            Ok(())
        }
        Command::ParamSet(name, number) => {
            wait_for_params(session, access, echo)?;
            param_set(session, name, *number, out, echo)
        }
        Command::Record(path) => record(session, path, duration(DEFAULT_RECORD), out, echo),
        command => {
            let Some((label, mav, params)) = mav_command(command) else {
                return Err(Fail::new(exit::FAILED, "internal error: no such command"));
            };
            send(
                session,
                SessionCommand::SendCommand {
                    command: mav,
                    params,
                },
            )?;
            if matches!(command, Command::Calibrate(Calibration::Compass)) {
                writeln!(
                    out,
                    "{label}: rotate the vehicle through all orientations for 30 s"
                )?;
            }
            follow_command(session, &label, out, echo)
        }
    }
}

/// Polls `check` every 10 ms until it returns `Some` or `deadline` has passed. Bounded.
fn wait<T>(deadline: Duration, check: impl FnMut() -> Option<T>) -> Option<T> {
    wait_with(deadline, check, &mut || {})
}

/// Like [`wait`]; `between` runs after every `check`, also the one that finds the result.
fn wait_with<T>(
    deadline: Duration,
    mut check: impl FnMut() -> Option<T>,
    between: &mut dyn FnMut(),
) -> Option<T> {
    let end = Instant::now() + deadline;
    loop {
        let found = check();
        between();
        if let Some(found) = found {
            return Some(found);
        }
        if Instant::now() >= end {
            return None;
        }
        std::thread::sleep(POLL);
    }
}

fn closed(state: &VehicleState) -> Option<Fail> {
    match &state.link.status {
        LinkStatus::Closed(reason) => Some(Fail::new(
            exit::NO_VEHICLE,
            format!("the link closed: {reason}"),
        )),
        _ => None,
    }
}

/// Waits for the vehicle's heartbeat.
fn wait_for_vehicle(session: &Session, timeout: Duration, echo: &mut Echo) -> Outcome {
    let seen = wait_with(
        timeout,
        || {
            let state = session.state();
            if let Some(fail) = closed(&state) {
                return Some(Err(fail));
            }
            (state.link.status == LinkStatus::Connected && state.link.vehicle.is_some())
                .then_some(Ok(()))
        },
        &mut || echo.pump(session),
    );
    seen.unwrap_or_else(|| {
        Err(Fail::new(
            exit::NO_VEHICLE,
            format!(
                "no heartbeat from a vehicle within {} s",
                timeout.as_secs_f64()
            ),
        ))
    })
}

/// Sends through the session's gate; a refusal is the failure the gate recorded.
fn send(session: &Session, command: SessionCommand) -> Outcome {
    let before = session.state().link.refused.as_ref().map_or(0, |r| r.count);
    let name = command.name();
    session.send(command);
    match &session.state().link.refused {
        Some(refused) if refused.count > before => Err(Fail::new(
            exit::REFUSED,
            format!("{} ({name})", Refused::TEXT),
        )),
        _ => Ok(()),
    }
}

/// Waits until the parameter list is complete; on an operate link it asks for it first (a read).
fn wait_for_params(session: &Session, access: Access, echo: &mut Echo) -> Outcome {
    if access == Access::Operate {
        send(session, SessionCommand::RefreshParams)?;
    }
    let result = wait_with(
        LIST_DEADLINE,
        || {
            let state = session.state();
            if let Some(fail) = closed(&state) {
                return Some(Err(fail));
            }
            match &state.params.fetch {
                FetchStatus::Complete => Some(Ok(())),
                FetchStatus::Failed(why) => Some(Err(Fail::new(
                    exit::FAILED,
                    format!("the parameter list failed: {why}"),
                ))),
                _ => None,
            }
        },
        &mut || echo.pump(session),
    );
    result.unwrap_or_else(|| {
        let state = session.state();
        let how = match &state.params.fetch {
            FetchStatus::Fetching { received, expected } => {
                format!("{received} of {expected} received")
            }
            _ => "none received".to_string(),
        };
        Err(Fail::new(
            exit::FAILED,
            format!("the parameter list did not arrive in time ({how})"),
        ))
    })
}

fn param_set(
    session: &Session,
    name: &str,
    number: f64,
    out: &mut dyn Write,
    echo: &mut Echo,
) -> Outcome {
    let (value, before_echoes) = {
        let state = session.state();
        let Some(param) = state.params.params.get(name) else {
            return Err(Fail::new(exit::FAILED, format!("unknown parameter {name}")));
        };
        let Some(value) = param.kind.value_from(number) else {
            return Err(Fail::new(
                exit::FAILED,
                format!(
                    "{number} does not fit the type {} of {name}",
                    param.kind.name()
                ),
            ));
        };
        (value, echo_count(&state))
    };
    send(
        session,
        SessionCommand::SetParam {
            name: name.to_string(),
            value,
        },
    )?;
    use gcs_mavlink::services::params::WriteStatus;
    let result = wait_with(
        WRITE_DEADLINE,
        || {
            let state = session.state();
            if let Some(fail) = closed(&state) {
                return Some(Err(fail));
            }
            match state.params.writes.get(name) {
                Some(WriteStatus::Pending { .. }) => None,
                Some(WriteStatus::Refused { in_effect, .. }) => Some(Err(Fail::new(
                    exit::FAILED,
                    format!("{name}: asked {value}, the vehicle keeps {in_effect}"),
                ))),
                Some(WriteStatus::Unanswered { .. }) => Some(Err(Fail::new(
                    exit::FAILED,
                    format!("{name}: no echo from the vehicle"),
                ))),
                None if echo_count(&state) > before_echoes => Some(Ok(())),
                None => None,
            }
        },
        &mut || echo.pump(session),
    );
    let result = result.unwrap_or_else(|| {
        Err(Fail::new(
            exit::FAILED,
            format!("{name}: no echo from the vehicle in time"),
        ))
    });
    let refused = matches!(
        session.state().params.writes.get(name),
        Some(WriteStatus::Refused { .. })
    );
    if refused {
        // The vehicle's reason follows its echo by a frame; give it a moment.
        let printed = echo.printed;
        let end = Instant::now() + REASON_GRACE;
        while echo.printed == printed && Instant::now() < end {
            echo.pump(session);
            std::thread::sleep(POLL);
        }
    }
    // What is in effect now, whatever the outcome (the table follows every PARAM_VALUE).
    let state = session.state();
    if let Some(param) = state.params.params.get(name) {
        writeln!(
            out,
            "{}\t{}\t{}",
            param.name,
            param.value(state.params.encoding),
            param.kind.name()
        )?;
    }
    result
}

/// How many PARAM_VALUE the session has received.
fn echo_count(state: &VehicleState) -> u64 {
    state
        .messages
        .entry("PARAM_VALUE", 0)
        .map_or(0, |entry| entry.count)
}

/// Follows the commands service's record to its end, printing a line when it changes.
fn follow_command(session: &Session, label: &str, out: &mut dyn Write, echo: &mut Echo) -> Outcome {
    let mut last = String::new();
    let end = wait_with(
        COMMAND_DEADLINE,
        || {
            let record = {
                let state = session.state();
                if let Some(fail) = closed(&state) {
                    return Some(Err(fail));
                }
                state.command.clone()
            };
            let record = record?;
            let line = describe(&record);
            if line != last {
                if let Err(error) = writeln!(out, "{label}: {line}") {
                    return Some(Err(error.into()));
                }
                last = line;
            }
            if record.outstanding() {
                return None;
            }
            Some(match &record.status {
                CommandStatus::Finished { accepted: true, .. } => Ok(()),
                _ => Err(Fail::new(exit::FAILED, format!("{label}: {last}"))),
            })
        },
        &mut || echo.pump(session),
    );
    end.unwrap_or_else(|| Err(Fail::new(exit::FAILED, format!("{label}: gave up waiting"))))
}

fn describe(record: &CommandRecord) -> String {
    match &record.status {
        CommandStatus::Waiting if record.sends > 1 => {
            format!("waiting for the vehicle (send {} of 3)", record.sends)
        }
        CommandStatus::Waiting => "waiting for the vehicle".to_string(),
        CommandStatus::InProgress {
            percent: Some(percent),
        } => format!("{percent} %"),
        CommandStatus::InProgress { percent: None } => "in progress".to_string(),
        CommandStatus::Finished { text, .. } => (*text).to_string(),
        CommandStatus::NoAnswer => format!("no answer after {} sends", record.sends),
        CommandStatus::NoResult => "no result (the vehicle stopped reporting)".to_string(),
        CommandStatus::Refused(why) => format!("refused ({why})"),
    }
}

/// One row of the device table.
struct DeviceRow {
    device: u8,
    sender: (u8, u8),
    cells: [String; 8],
}

fn word(entry: &MessageEntry, field: &str, prefix: &str) -> String {
    entry.text(field).map_or_else(String::new, |t| {
        t.strip_prefix(prefix).unwrap_or(t).to_string()
    })
}

fn count(entry: &MessageEntry, field: &str) -> String {
    entry
        .number(field)
        .map_or_else(String::new, |n| format!("{}", n as u64))
}

/// NAME, STATE, LAST_ERROR, SAMPLES, ERRORS, TIMEOUTS, RECOVERIES, LAST_SAMPLE_AGE_MS
/// (`never` when the device has had no sample: 65535).
fn device_rows(state: &VehicleState) -> Vec<DeviceRow> {
    let mut rows: Vec<DeviceRow> = state
        .messages
        .entries
        .iter()
        .filter(|(_, entry)| entry.name == DEVICE_MESSAGE && !entry.fields.is_empty())
        .map(|(key, entry)| DeviceRow {
            device: key.instance,
            sender: (key.system_id, key.component_id),
            cells: [
                word(entry, "device_id", "MARV_DEVICE_"),
                word(entry, "state", "MARV_DEVICE_STATE_"),
                word(entry, "last_error", "MARV_DEVICE_ERROR_"),
                count(entry, "samples"),
                count(entry, "errors"),
                count(entry, "timeouts"),
                count(entry, "recoveries"),
                match entry.number("last_sample_age_ms") {
                    Some(ms) if ms as u16 == u16::MAX => "never".to_string(),
                    Some(ms) => format!("{}", ms as u16),
                    None => String::new(),
                },
            ],
        })
        .collect();
    rows.sort_by_key(|row| (row.device, row.sender));
    rows
}

/// The cells that change when something happens to a device; samples and the sample age change
/// all the time and do not count for `watch`.
fn events_of(row: &DeviceRow) -> Vec<String> {
    let c = &row.cells;
    [&c[0], &c[1], &c[2], &c[4], &c[5]]
        .iter()
        .map(|cell| (*cell).clone())
        .collect()
}

fn device_line(row: &DeviceRow) -> String {
    row.cells.join("\t")
}

fn collect(session: &Session, duration: Duration, echo: &mut Echo) -> Outcome {
    let end = Instant::now() + duration;
    let result = wait_with(
        duration + POLL,
        || {
            let state = session.state();
            if let Some(fail) = closed(&state) {
                return Some(Err(fail));
            }
            (Instant::now() >= end).then_some(Ok(()))
        },
        &mut || echo.pump(session),
    );
    result.unwrap_or(Ok(()))
}

fn devices(session: &Session, duration: Duration, out: &mut dyn Write, echo: &mut Echo) -> Outcome {
    collect(session, duration, echo)?;
    let state = session.state();
    for row in device_rows(&state) {
        writeln!(out, "{}", device_line(&row))?;
    }
    Ok(())
}

fn text_line(entry: &TextEntry) -> String {
    format!(
        "{:.3}\t{}\t{}",
        entry.time.as_secs_f64(),
        entry.severity.name(),
        entry.text.replace(['\n', '\t'], " ")
    )
}

/// Prints the vehicle's STATUSTEXTs on stderr as they arrive, whatever command runs: a text belongs
/// to the session, not to the command, so those that came before the command are shown too.
struct Echo<'a> {
    err: &'a mut dyn Write,
    cursor: TextCursor,
    /// Off for `texts`, `watch` and `status`, which print texts on stdout.
    on: bool,
    printed: usize,
}

impl<'a> Echo<'a> {
    fn new(err: &'a mut dyn Write, on: bool) -> Self {
        Echo {
            err,
            cursor: TextCursor::default(),
            on,
            printed: 0,
        }
    }

    fn pump(&mut self, session: &Session) {
        if !self.on {
            return;
        }
        let state = session.state();
        for entry in self.cursor.fresh(&state) {
            let _ = writeln!(
                self.err,
                "text\t{}\t{}",
                entry.severity.name(),
                entry.text.replace(['\n', '\t'], " ")
            );
            self.printed += 1;
        }
        let _ = self.err.flush();
    }
}

/// Tracks which texts were printed: by arrival time, and how many of that time.
#[derive(Default)]
struct TextCursor {
    time: Duration,
    seen_at_time: usize,
}

impl TextCursor {
    fn fresh<'a>(&mut self, state: &'a VehicleState) -> Vec<&'a TextEntry> {
        let mut fresh = Vec::new();
        let mut skipped = 0;
        for entry in &state.texts.entries {
            if entry.time < self.time {
                continue;
            }
            if entry.time == self.time && skipped < self.seen_at_time {
                skipped += 1;
                continue;
            }
            fresh.push(entry);
        }
        if let Some(last) = fresh.last() {
            let same = fresh.iter().filter(|e| e.time == last.time).count();
            if last.time == self.time {
                self.seen_at_time += same;
            } else {
                self.time = last.time;
                self.seen_at_time = same;
            }
        }
        fresh
    }
}

fn texts(session: &Session, duration: Duration, out: &mut dyn Write) -> Outcome {
    let end = Instant::now() + duration;
    let mut cursor = TextCursor::default();
    let result = wait(duration + POLL, || {
        let state = session.state();
        if let Some(fail) = closed(&state) {
            return Some(Err(fail));
        }
        for entry in cursor.fresh(&state) {
            if let Err(error) = writeln!(out, "{}", text_line(entry)) {
                return Some(Err(error.into()));
            }
        }
        (Instant::now() >= end).then_some(Ok(()))
    });
    result.unwrap_or(Ok(()))
}

fn watch(session: &Session, duration: Duration, out: &mut dyn Write) -> Outcome {
    let end = Instant::now() + duration;
    let mut cursor = TextCursor::default();
    let mut shown: std::collections::BTreeMap<(u8, (u8, u8)), Vec<String>> = Default::default();
    let result = wait(duration + POLL, || {
        let state = session.state();
        if let Some(fail) = closed(&state) {
            return Some(Err(fail));
        }
        for row in device_rows(&state) {
            let events = events_of(&row);
            if shown.get(&(row.device, row.sender)) != Some(&events) {
                if let Err(error) = writeln!(out, "device\t{}", device_line(&row)) {
                    return Some(Err(error.into()));
                }
                shown.insert((row.device, row.sender), events);
            }
        }
        for entry in cursor.fresh(&state) {
            if let Err(error) = writeln!(out, "text\t{}", text_line(entry)) {
                return Some(Err(error.into()));
            }
        }
        (Instant::now() >= end).then_some(Ok(()))
    });
    result.unwrap_or(Ok(()))
}

/// The sensors SYS_STATUS reports healthy, in words; `None` before the first SYS_STATUS.
fn health(state: &VehicleState) -> Option<String> {
    let entry = state.messages.entry("SYS_STATUS", 0)?;
    let field = entry
        .fields
        .iter()
        .find(|f| f.name == "onboard_control_sensors_health")?;
    Some(match &field.value {
        FieldValue::Text(text) if text.trim().is_empty() => "none".to_string(),
        FieldValue::Text(text) => text.clone(),
        FieldValue::Number(n) if *n == 0.0 => "none".to_string(),
        FieldValue::Number(n) => format!("0x{:x}", *n as u64),
    })
}

fn status(
    session: &Session,
    access: Access,
    duration: Duration,
    out: &mut dyn Write,
    echo: &mut Echo,
) -> Outcome {
    collect(session, duration, echo)?;
    let state = session.state();
    let Some(vehicle) = &state.link.vehicle else {
        return Err(Fail::new(exit::NO_VEHICLE, "no vehicle"));
    };
    writeln!(
        out,
        "vehicle\tsystem={} component={} type={}",
        vehicle.system_id, vehicle.component_id, vehicle.mav_type
    )?;
    let role = match access {
        Access::Operate => "Operate",
        Access::Configure => "Configure",
    };
    writeln!(out, "role\t{role}")?;
    writeln!(
        out,
        "health\t{}",
        health(&state).unwrap_or_else(|| "not received".to_string())
    )?;
    writeln!(out, "devices\t{}", device_rows(&state).len())?;
    let skip = state.texts.entries.len().saturating_sub(STATUS_TEXTS);
    for entry in state.texts.entries.iter().skip(skip) {
        writeln!(out, "text\t{}", text_line(entry))?;
    }
    Ok(())
}

fn record(
    session: &Session,
    path: &std::path::Path,
    duration: Duration,
    out: &mut dyn Write,
    echo: &mut Echo,
) -> Outcome {
    let file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|e| {
            Fail::new(
                exit::FAILED,
                format!(
                    "cannot create {} (an existing file is never overwritten): {e}",
                    path.display()
                ),
            )
        })?;
    let label = path.display().to_string();
    send(
        session,
        SessionCommand::StartLog {
            sink: LogSink::new(file),
            label,
        },
    )?;
    let started = wait_with(
        Duration::from_secs(2),
        || {
            let state = session.state();
            (state.log.recording.is_some() || state.log.error.is_some()).then_some(())
        },
        &mut || echo.pump(session),
    );
    if started.is_none() || session.state().log.error.is_some() {
        let why = session.state().log.error.clone().unwrap_or_default();
        return Err(Fail::new(
            exit::FAILED,
            format!("the recording did not start {why}"),
        ));
    }
    let end = Instant::now() + duration;
    let result = wait_with(
        duration + POLL,
        || {
            let state = session.state();
            if let Some(fail) = closed(&state) {
                return Some(Err(fail));
            }
            if let Some(why) = &state.log.error {
                return Some(Err(Fail::new(
                    exit::FAILED,
                    format!("the recording stopped: {why}"),
                )));
            }
            (Instant::now() >= end).then_some(Ok(()))
        },
        &mut || echo.pump(session),
    );
    let result = result.unwrap_or(Ok(()));
    let _ = send(session, SessionCommand::StopLog);
    let _ = wait_with(
        Duration::from_secs(2),
        || session.state().log.recording.is_none().then_some(()),
        &mut || echo.pump(session),
    );
    let state = session.state();
    writeln!(
        out,
        "recorded\t{}\t{} frames\t{} bytes",
        path.display(),
        state.log.frames,
        state.log.bytes
    )?;
    result
}
