//! Parameter service: the vehicle's parameter list, reads and writes (docs/link-contract.md,
//! "Parameters"). The table keeps the 4 raw bytes of every value and decodes them only to show or
//! to set, so the list never waits for the encoding to be known.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::time::Duration;

use super::{Ctx, Service};
use crate::dialect::{
    AUTOPILOT_VERSION_DATA, COMMAND_LONG_DATA, MavAutopilot, MavCmd, MavParamType,
    MavProtocolCapability, PARAM_REQUEST_LIST_DATA, PARAM_REQUEST_READ_DATA, PARAM_SET_DATA,
    PARAM_VALUE_DATA,
};
use crate::session::SessionConfig;
use crate::state::{CommandStatus, UnsavedCalibration};
use crate::{Access, Body, Command, Frame, LinkStatus, Message};

/// How a vehicle puts a value into the 4-byte value field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ParamEncoding {
    /// The value's own little-endian bytes, the rest zero.
    #[default]
    Bytewise,
    /// The number converted to a 32-bit float.
    CCast,
}

/// The parameter types that fit the 4-byte field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamType {
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    F32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParamValue {
    U8(u8),
    I8(i8),
    U16(u16),
    I16(i16),
    U32(u32),
    I32(i32),
    F32(f32),
}

impl ParamType {
    pub fn name(self) -> &'static str {
        match self {
            ParamType::U8 => "u8",
            ParamType::I8 => "i8",
            ParamType::U16 => "u16",
            ParamType::I16 => "i16",
            ParamType::U32 => "u32",
            ParamType::I32 => "i32",
            ParamType::F32 => "f32",
        }
    }

    /// A value of this type holding `number`; `None` when it does not fit (not finite, out of
    /// range, or a fraction for an integer type).
    pub fn value_from(self, number: f64) -> Option<ParamValue> {
        if !number.is_finite() {
            return None;
        }
        let whole = |min: f64, max: f64| number.fract() == 0.0 && number >= min && number <= max;
        match self {
            ParamType::U8 => whole(0.0, f64::from(u8::MAX)).then_some(ParamValue::U8(number as u8)),
            ParamType::I8 => whole(f64::from(i8::MIN), f64::from(i8::MAX))
                .then_some(ParamValue::I8(number as i8)),
            ParamType::U16 => {
                whole(0.0, f64::from(u16::MAX)).then_some(ParamValue::U16(number as u16))
            }
            ParamType::I16 => whole(f64::from(i16::MIN), f64::from(i16::MAX))
                .then_some(ParamValue::I16(number as i16)),
            ParamType::U32 => {
                whole(0.0, f64::from(u32::MAX)).then_some(ParamValue::U32(number as u32))
            }
            ParamType::I32 => whole(f64::from(i32::MIN), f64::from(i32::MAX))
                .then_some(ParamValue::I32(number as i32)),
            ParamType::F32 => {
                let narrowed = number as f32;
                narrowed.is_finite().then_some(ParamValue::F32(narrowed))
            }
        }
    }
}

impl ParamValue {
    pub fn kind(self) -> ParamType {
        match self {
            ParamValue::U8(_) => ParamType::U8,
            ParamValue::I8(_) => ParamType::I8,
            ParamValue::U16(_) => ParamType::U16,
            ParamValue::I16(_) => ParamType::I16,
            ParamValue::U32(_) => ParamType::U32,
            ParamValue::I32(_) => ParamType::I32,
            ParamValue::F32(_) => ParamType::F32,
        }
    }

    pub fn as_f64(self) -> f64 {
        match self {
            ParamValue::U8(v) => f64::from(v),
            ParamValue::I8(v) => f64::from(v),
            ParamValue::U16(v) => f64::from(v),
            ParamValue::I16(v) => f64::from(v),
            ParamValue::U32(v) => f64::from(v),
            ParamValue::I32(v) => f64::from(v),
            ParamValue::F32(v) => f64::from(v),
        }
    }
}

impl fmt::Display for ParamValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParamValue::U8(v) => write!(f, "{v}"),
            ParamValue::I8(v) => write!(f, "{v}"),
            ParamValue::U16(v) => write!(f, "{v}"),
            ParamValue::I16(v) => write!(f, "{v}"),
            ParamValue::U32(v) => write!(f, "{v}"),
            ParamValue::I32(v) => write!(f, "{v}"),
            ParamValue::F32(v) => write!(f, "{v}"),
        }
    }
}

/// The value field's 4 bytes (little-endian order of the wire float) for a value.
pub fn encode(value: ParamValue, encoding: ParamEncoding) -> [u8; 4] {
    match encoding {
        ParamEncoding::CCast => (value.as_f64() as f32).to_le_bytes(),
        ParamEncoding::Bytewise => match value {
            ParamValue::U8(v) => [v, 0, 0, 0],
            ParamValue::I8(v) => [v as u8, 0, 0, 0],
            ParamValue::U16(v) => {
                let b = v.to_le_bytes();
                [b[0], b[1], 0, 0]
            }
            ParamValue::I16(v) => {
                let b = v.to_le_bytes();
                [b[0], b[1], 0, 0]
            }
            ParamValue::U32(v) => v.to_le_bytes(),
            ParamValue::I32(v) => v.to_le_bytes(),
            ParamValue::F32(v) => v.to_le_bytes(),
        },
    }
}

/// The value the 4 bytes of the value field hold.
pub fn decode(kind: ParamType, raw: [u8; 4], encoding: ParamEncoding) -> ParamValue {
    match encoding {
        ParamEncoding::CCast => {
            let number = f32::from_le_bytes(raw);
            match kind {
                ParamType::U8 => ParamValue::U8(number as u8),
                ParamType::I8 => ParamValue::I8(number as i8),
                ParamType::U16 => ParamValue::U16(number as u16),
                ParamType::I16 => ParamValue::I16(number as i16),
                ParamType::U32 => ParamValue::U32(number as u32),
                ParamType::I32 => ParamValue::I32(number as i32),
                ParamType::F32 => ParamValue::F32(number),
            }
        }
        ParamEncoding::Bytewise => match kind {
            ParamType::U8 => ParamValue::U8(raw[0]),
            ParamType::I8 => ParamValue::I8(raw[0] as i8),
            ParamType::U16 => ParamValue::U16(u16::from_le_bytes([raw[0], raw[1]])),
            ParamType::I16 => ParamValue::I16(i16::from_le_bytes([raw[0], raw[1]])),
            ParamType::U32 => ParamValue::U32(u32::from_le_bytes(raw)),
            ParamType::I32 => ParamValue::I32(i32::from_le_bytes(raw)),
            ParamType::F32 => ParamValue::F32(f32::from_le_bytes(raw)),
        },
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Param {
    pub name: String,
    pub index: u16,
    pub kind: ParamType,
    /// The 4 bytes of the value field exactly as the vehicle sent them.
    pub raw: [u8; 4],
}

impl Param {
    pub fn value(&self, encoding: ParamEncoding) -> ParamValue {
        decode(self.kind, self.raw, encoding)
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub enum FetchStatus {
    /// Nothing asked yet.
    #[default]
    Idle,
    Fetching {
        received: u16,
        expected: u16,
    },
    Complete,
    /// The text says what is missing; the table keeps what did arrive.
    Failed(String),
}

#[derive(Debug, Clone, PartialEq)]
pub enum WriteStatus {
    /// Sent, waiting for the vehicle's echo.
    Pending { value: ParamValue },
    /// The vehicle answered with another value: it refused, and `in_effect` is what it uses.
    Refused {
        value: ParamValue,
        in_effect: ParamValue,
    },
    /// No echo after the retries.
    Unanswered { value: ParamValue },
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ParamTable {
    /// By name.
    pub params: BTreeMap<String, Param>,
    /// How this vehicle encodes values.
    pub encoding: ParamEncoding,
    /// False until the vehicle said how it encodes values or the fallback rule was applied.
    pub encoding_known: bool,
    pub fetch: FetchStatus,
    /// Writes under way, refused or unanswered, by name. A confirmed write is removed.
    pub writes: BTreeMap<String, WriteStatus>,
}

impl ParamTable {
    pub fn value(&self, name: &str) -> Option<ParamValue> {
        self.params.get(name).map(|p| p.value(self.encoding))
    }
}

const AUTOPILOT_VERSION_ID: f32 = 148.0;
const ENCODING_RESEND: Duration = Duration::from_millis(500);
const ENCODING_SENDS: u8 = 3;
const LIST_SILENCE: Duration = Duration::from_secs(1);
const LIST_SENDS: u8 = 3;
const READ_ROUND: Duration = Duration::from_millis(500);
const READ_ROUND_SIZE: usize = 50;
const READ_FAILED_TRIES: u8 = 8;
const MAX_RESTARTS: u8 = 3;
const WRITE_RESEND: Duration = Duration::from_millis(500);
const WRITE_SENDS: u8 = 3;

fn kind_from_wire(wire: MavParamType) -> Option<ParamType> {
    match wire {
        MavParamType::MAV_PARAM_TYPE_UINT8 => Some(ParamType::U8),
        MavParamType::MAV_PARAM_TYPE_INT8 => Some(ParamType::I8),
        MavParamType::MAV_PARAM_TYPE_UINT16 => Some(ParamType::U16),
        MavParamType::MAV_PARAM_TYPE_INT16 => Some(ParamType::I16),
        MavParamType::MAV_PARAM_TYPE_UINT32 => Some(ParamType::U32),
        MavParamType::MAV_PARAM_TYPE_INT32 => Some(ParamType::I32),
        MavParamType::MAV_PARAM_TYPE_REAL32 => Some(ParamType::F32),
        _ => None,
    }
}

fn kind_to_wire(kind: ParamType) -> MavParamType {
    match kind {
        ParamType::U8 => MavParamType::MAV_PARAM_TYPE_UINT8,
        ParamType::I8 => MavParamType::MAV_PARAM_TYPE_INT8,
        ParamType::U16 => MavParamType::MAV_PARAM_TYPE_UINT16,
        ParamType::I16 => MavParamType::MAV_PARAM_TYPE_INT16,
        ParamType::U32 => MavParamType::MAV_PARAM_TYPE_UINT32,
        ParamType::I32 => MavParamType::MAV_PARAM_TYPE_INT32,
        ParamType::F32 => MavParamType::MAV_PARAM_TYPE_REAL32,
    }
}

/// The name in a 16-byte id field: up to the first zero byte, or all 16 bytes.
fn name_from_wire(id: &[u8; 16]) -> String {
    let end = id.iter().position(|&b| b == 0).unwrap_or(id.len());
    String::from_utf8_lossy(&id[..end]).into_owned()
}

fn name_to_wire(name: &str) -> [u8; 16] {
    let mut id = [0u8; 16];
    let bytes = name.as_bytes();
    let len = bytes.len().min(16);
    id[..len].copy_from_slice(&bytes[..len]);
    id
}

#[derive(Debug)]
enum EncodingPhase {
    /// Not connected yet.
    Idle,
    Asking {
        sends: u8,
        last_send: Duration,
    },
    Done,
}

#[derive(Debug)]
struct Round {
    requested: Vec<u16>,
    started: Duration,
    progress: bool,
}

#[derive(Debug)]
enum Fetch {
    /// Not connected yet.
    Idle,
    /// The request is out and no PARAM_VALUE has come.
    Requesting {
        sends: u8,
        last_send: Duration,
    },
    Receiving {
        expected: u16,
        got: BTreeSet<u16>,
        last_rx: Duration,
        round: Option<Round>,
        failed: u8,
    },
    Complete {
        expected: u16,
    },
    Failed,
}

#[derive(Debug)]
struct WriteTrack {
    /// The bytes in the PARAM_SET; `None` while the encoding is not known and nothing was sent.
    sent: Option<[u8; 4]>,
    value: ParamValue,
    sends: u8,
    last_send: Duration,
    /// The last value a differing PARAM_VALUE reported since the first send.
    differing: Option<ParamValue>,
    /// The write ended as refused or unanswered; only an equal echo can still change it.
    ended: bool,
}

/// The service that runs the parameter protocol.
#[derive(Debug)]
pub struct Params {
    access: Access,
    started: bool,
    encoding: EncodingPhase,
    fetch: Fetch,
    restarts: u8,
    writes: BTreeMap<String, WriteTrack>,
    /// The command whose end already caused the offset reads (its start and end time).
    offsets_read_for: Option<(Duration, Duration)>,
}

impl Params {
    pub fn new(config: &SessionConfig) -> Self {
        Self {
            access: config.access,
            started: false,
            encoding: EncodingPhase::Idle,
            fetch: Fetch::Idle,
            restarts: 0,
            writes: BTreeMap::new(),
            offsets_read_for: None,
        }
    }

    /// The vehicle's system and component id, once the link has been `Connected`; starts the
    /// protocol on the first time.
    fn target(&mut self, cx: &mut Ctx) -> Option<(u8, u8)> {
        let vehicle = cx.state.link.vehicle.as_ref()?;
        let target = (vehicle.system_id, vehicle.component_id);
        if !self.started {
            if cx.state.link.status != LinkStatus::Connected {
                return None;
            }
            self.started = true;
            self.encoding = EncodingPhase::Asking {
                sends: 1,
                last_send: cx.now,
            };
            cx.out.push(request_autopilot_version(target));
            if self.access == Access::Configure {
                self.begin_fetch(target, cx);
            }
        }
        Some(target)
    }

    fn begin_fetch(&mut self, target: (u8, u8), cx: &mut Ctx) {
        let table = &mut cx.state.params;
        table.params.clear();
        table.writes.clear();
        table.fetch = FetchStatus::Fetching {
            received: 0,
            expected: 0,
        };
        cx.changed = true;
        self.writes.clear();
        self.fetch = Fetch::Requesting {
            sends: 1,
            last_send: cx.now,
        };
        cx.out
            .push(Message::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
                target_system: target.0,
                target_component: target.1,
            }));
    }

    fn fail_fetch(&mut self, text: String, cx: &mut Ctx) {
        self.fetch = Fetch::Failed;
        cx.state.params.fetch = FetchStatus::Failed(text);
        cx.changed = true;
    }

    fn settle_encoding(&mut self, target: (u8, u8), encoding: ParamEncoding, cx: &mut Ctx) {
        self.encoding = EncodingPhase::Done;
        let table = &mut cx.state.params;
        if table.encoding != encoding || !table.encoding_known {
            table.encoding = encoding;
            table.encoding_known = true;
            cx.changed = true;
        }
        self.send_waiting_writes(target, cx);
    }

    /// The first PARAM_SET of every write that waited for the encoding.
    fn send_waiting_writes(&mut self, target: (u8, u8), cx: &mut Ctx) {
        for (name, track) in &mut self.writes {
            if track.sent.is_some() {
                continue;
            }
            let Some(param) = cx.state.params.params.get(name) else {
                continue;
            };
            let sent = encode(track.value, cx.state.params.encoding);
            track.sent = Some(sent);
            track.sends = 1;
            track.last_send = cx.now;
            cx.out.push(param_set(target, name, sent, param.kind));
        }
    }

    fn fallback_encoding(&self, cx: &Ctx) -> ParamEncoding {
        let ardupilot = cx.state.link.vehicle.as_ref().is_some_and(|v| {
            v.autopilot == format!("{:?}", MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA)
        });
        if ardupilot {
            ParamEncoding::CCast
        } else {
            ParamEncoding::Bytewise
        }
    }

    fn on_autopilot_version(
        &mut self,
        target: (u8, u8),
        version: &AUTOPILOT_VERSION_DATA,
        cx: &mut Ctx,
    ) {
        let caps = version.capabilities;
        let decided = if caps
            .contains(MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)
        {
            Some(ParamEncoding::Bytewise)
        } else if caps.contains(MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST)
        {
            Some(ParamEncoding::CCast)
        } else {
            None
        };
        match (decided, &self.encoding) {
            (Some(encoding), _) => self.settle_encoding(target, encoding, cx),
            (None, EncodingPhase::Asking { .. }) => {
                let encoding = self.fallback_encoding(cx);
                self.settle_encoding(target, encoding, cx);
            }
            (None, _) => {}
        }
    }

    fn tick_encoding(&mut self, target: (u8, u8), cx: &mut Ctx) {
        let EncodingPhase::Asking { sends, last_send } = &mut self.encoding else {
            return;
        };
        if cx.now.saturating_sub(*last_send) < ENCODING_RESEND {
            return;
        }
        if *sends < ENCODING_SENDS {
            *sends += 1;
            *last_send = cx.now;
            cx.out.push(request_autopilot_version(target));
        } else {
            let encoding = self.fallback_encoding(cx);
            self.settle_encoding(target, encoding, cx);
        }
    }

    fn tick_fetch(&mut self, target: (u8, u8), cx: &mut Ctx) {
        match &mut self.fetch {
            Fetch::Requesting { sends, last_send } => {
                if cx.now.saturating_sub(*last_send) < LIST_SILENCE {
                    return;
                }
                if *sends < LIST_SENDS {
                    *sends += 1;
                    *last_send = cx.now;
                    cx.out
                        .push(Message::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
                            target_system: target.0,
                            target_component: target.1,
                        }));
                } else {
                    self.fail_fetch("no answer to the parameter request".to_owned(), cx);
                }
            }
            Fetch::Receiving {
                expected,
                got,
                last_rx,
                round,
                failed,
            } => {
                let Some(current) = round else {
                    if cx.now.saturating_sub(*last_rx) >= LIST_SILENCE {
                        *round = Some(read_missing(target, *expected, got, cx));
                    }
                    return;
                };
                let all_in = current.requested.iter().all(|index| got.contains(index));
                if !all_in && cx.now.saturating_sub(current.started) < READ_ROUND {
                    return;
                }
                if current.progress {
                    *failed = 0;
                } else {
                    *failed += 1;
                }
                if *failed >= READ_FAILED_TRIES {
                    let text = format!(
                        "{} of {} parameters missing",
                        usize::from(*expected) - got.len(),
                        expected
                    );
                    self.fail_fetch(text, cx);
                } else {
                    *round = Some(read_missing(target, *expected, got, cx));
                }
            }
            Fetch::Idle | Fetch::Complete { .. } | Fetch::Failed => {}
        }
    }

    fn tick_writes(&mut self, target: (u8, u8), cx: &mut Ctx) {
        for (name, track) in &mut self.writes {
            let Some(sent) = track.sent else {
                continue;
            };
            if track.ended || cx.now.saturating_sub(track.last_send) < WRITE_RESEND {
                continue;
            }
            if track.sends < WRITE_SENDS {
                track.sends += 1;
                track.last_send = cx.now;
                if let Some(param) = cx.state.params.params.get(name) {
                    cx.out.push(param_set(target, name, sent, param.kind));
                }
            } else {
                track.ended = true;
                let status = match track.differing {
                    Some(in_effect) => WriteStatus::Refused {
                        value: track.value,
                        in_effect,
                    },
                    None => WriteStatus::Unanswered { value: track.value },
                };
                cx.state.params.writes.insert(name.clone(), status);
                cx.changed = true;
            }
        }
    }

    /// The vehicle sends no PARAM_VALUE for the offsets a calibration writes or a reload restores, so
    /// after an accepted calibration the station asks for its offsets by name, and after an accepted
    /// reload from flash (245 param1 = 0, which can change any parameter) it fetches the whole list again,
    /// as the Refresh button does. A name with a write of its own under way is left out: its echo
    /// rule would take the reply for the write's answer.
    fn read_changed_offsets(&mut self, target: (u8, u8), cx: &mut Ctx) {
        if self.access != Access::Configure {
            return;
        }
        let Some(record) = cx.state.command.as_ref() else {
            return;
        };
        let (CommandStatus::Finished { accepted: true, .. }, Some(ended)) =
            (&record.status, record.ended)
        else {
            return;
        };
        if self.offsets_read_for == Some((record.started, ended)) {
            return;
        }
        self.offsets_read_for = Some((record.started, ended));
        let p = record.params;
        let names = if record.command == MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION as u32 {
            UnsavedCalibration::offsets(p[0] == 1.0, p[4] == 4.0, p[1] == 1.0)
        } else if record.command == MavCmd::MAV_CMD_PREFLIGHT_STORAGE as u32 && p[0] == 0.0 {
            self.restarts = 0;
            self.begin_fetch(target, cx);
            return;
        } else {
            return;
        };
        for name in names {
            if self.writes.contains_key(name) {
                continue;
            }
            cx.out
                .push(Message::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
                    param_index: -1,
                    target_system: target.0,
                    target_component: target.1,
                    param_id: name_to_wire(name).into(),
                }));
        }
    }

    fn set_param(&mut self, target: (u8, u8), name: &str, value: ParamValue, cx: &mut Ctx) {
        let table = &cx.state.params;
        let Some(param) = table.params.get(name) else {
            return;
        };
        if param.kind != value.kind() {
            return;
        }
        let sent = table.encoding_known.then(|| encode(value, table.encoding));
        if let Some(sent) = sent {
            cx.out.push(param_set(target, name, sent, param.kind));
        }
        self.writes.insert(
            name.to_owned(),
            WriteTrack {
                sent,
                value,
                sends: u8::from(sent.is_some()),
                last_send: cx.now,
                differing: None,
                ended: false,
            },
        );
        cx.state
            .params
            .writes
            .insert(name.to_owned(), WriteStatus::Pending { value });
        cx.changed = true;
    }

    fn on_param_value(&mut self, target: (u8, u8), v: &PARAM_VALUE_DATA, cx: &mut Ctx) {
        let name = name_from_wire(&v.param_id);
        let raw = v.param_value.to_le_bytes();
        let kind = kind_from_wire(v.param_type);
        let (index, count) = (v.param_index, v.param_count);
        let downloading = matches!(
            self.fetch,
            Fetch::Requesting { .. } | Fetch::Receiving { .. }
        );
        match &mut self.fetch {
            Fetch::Idle => {
                if self.access == Access::Operate {
                    observe(
                        &mut cx.state.params,
                        &name,
                        (index, count),
                        kind,
                        raw,
                        &mut cx.changed,
                    );
                }
                return;
            }
            Fetch::Requesting { .. } if count == 0 => {
                self.restarts = 0;
                self.fetch = Fetch::Complete { expected: 0 };
                cx.state.params.fetch = FetchStatus::Complete;
                cx.changed = true;
            }
            Fetch::Requesting { .. } => {
                self.fetch = Fetch::Receiving {
                    expected: count,
                    got: BTreeSet::new(),
                    last_rx: cx.now,
                    round: None,
                    failed: 0,
                };
            }
            Fetch::Receiving { expected, .. } | Fetch::Complete { expected } => {
                if *expected != count {
                    self.restarts += 1;
                    if self.restarts > MAX_RESTARTS {
                        self.fail_fetch("parameter count keeps changing".to_owned(), cx);
                    } else {
                        self.begin_fetch(target, cx);
                    }
                    return;
                }
            }
            Fetch::Failed => {}
        }
        match &mut self.fetch {
            Fetch::Receiving {
                expected,
                got,
                last_rx,
                round,
                ..
            } => {
                *last_rx = cx.now;
                if index >= *expected {
                    return;
                }
                if let Some(kind) = kind {
                    store(
                        &mut cx.state.params,
                        name.clone(),
                        index,
                        kind,
                        raw,
                        &mut cx.changed,
                    );
                }
                if got.insert(index) {
                    if let Some(round) = round {
                        round.progress = true;
                    }
                    let received = u16::try_from(got.len()).unwrap_or(u16::MAX);
                    if received == *expected {
                        let expected = *expected;
                        self.restarts = 0;
                        self.fetch = Fetch::Complete { expected };
                        cx.state.params.fetch = FetchStatus::Complete;
                    } else {
                        cx.state.params.fetch = FetchStatus::Fetching {
                            received,
                            expected: *expected,
                        };
                    }
                    cx.changed = true;
                }
            }
            _ => {
                if let (Some(kind), Some(param)) = (kind, cx.state.params.params.get_mut(&name))
                    && (param.raw != raw || param.kind != kind)
                {
                    param.raw = raw;
                    param.kind = kind;
                    cx.changed = true;
                }
            }
        }
        self.resolve_write(&name, raw, kind, downloading, cx);
    }

    /// An echo for a written name. Equal bytes confirm the write. A differing value after the
    /// PARAM_SET went out is the vehicle's refusal (it answers every PARAM_SET with exactly one
    /// PARAM_VALUE): the write ends at once and nothing is sent again. While a list download is
    /// under way its frames are not answers, so the value is only noted and the resend rule decides.
    fn resolve_write(
        &mut self,
        name: &str,
        raw: [u8; 4],
        kind: Option<ParamType>,
        downloading: bool,
        cx: &mut Ctx,
    ) {
        let (Some(kind), Some(track)) = (kind, self.writes.get_mut(name)) else {
            return;
        };
        let Some(sent) = track.sent else {
            return;
        };
        if !cx.state.params.params.contains_key(name) {
            return;
        }
        if raw == sent {
            cx.state.params.writes.remove(name);
            self.writes.remove(name);
            cx.changed = true;
        } else {
            let in_effect = decode(kind, raw, cx.state.params.encoding);
            track.differing = Some(in_effect);
            if !downloading && !track.ended {
                track.ended = true;
                cx.state.params.writes.insert(
                    name.to_owned(),
                    WriteStatus::Refused {
                        value: track.value,
                        in_effect,
                    },
                );
                cx.changed = true;
            }
        }
    }
}

fn store(
    table: &mut ParamTable,
    name: String,
    index: u16,
    kind: ParamType,
    raw: [u8; 4],
    changed: &mut bool,
) {
    let param = Param {
        name: name.clone(),
        index,
        kind,
        raw,
    };
    if table.params.get(&name) != Some(&param) {
        table.params.insert(name, param);
        *changed = true;
    }
}

/// What an operate-only session does with a PARAM_VALUE while no download is under way (a replayed log,
/// or a list another station asked for): it shows the entry, and asks for nothing and repairs nothing.
fn observe(
    table: &mut ParamTable,
    name: &str,
    (index, count): (u16, u16),
    kind: Option<ParamType>,
    raw: [u8; 4],
    changed: &mut bool,
) {
    let Some(kind) = kind else {
        return;
    };
    if count == 0 {
        if table.fetch != FetchStatus::Complete {
            table.fetch = FetchStatus::Complete;
            *changed = true;
        }
        return;
    }
    if index >= count {
        return;
    }
    store(table, name.to_owned(), index, kind, raw, changed);
    let received = u16::try_from(table.params.len()).unwrap_or(u16::MAX);
    let fetch = if received >= count {
        FetchStatus::Complete
    } else {
        FetchStatus::Fetching {
            received,
            expected: count,
        }
    };
    if table.fetch != fetch {
        table.fetch = fetch;
        *changed = true;
    }
}

/// Requests up to `READ_ROUND_SIZE` missing indices.
fn read_missing(target: (u8, u8), expected: u16, got: &BTreeSet<u16>, cx: &mut Ctx) -> Round {
    let requested: Vec<u16> = (0..expected)
        .filter(|index| !got.contains(index))
        .take(READ_ROUND_SIZE)
        .collect();
    for &index in &requested {
        if let Ok(wire_index) = i16::try_from(index) {
            cx.out
                .push(Message::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
                    param_index: wire_index,
                    target_system: target.0,
                    target_component: target.1,
                    param_id: [0u8; 16].into(),
                }));
        }
    }
    Round {
        requested,
        started: cx.now,
        progress: false,
    }
}

fn request_autopilot_version(target: (u8, u8)) -> Message {
    Message::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: AUTOPILOT_VERSION_ID,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
        target_system: target.0,
        target_component: target.1,
        confirmation: 0,
    })
}

fn param_set(target: (u8, u8), name: &str, raw: [u8; 4], kind: ParamType) -> Message {
    Message::PARAM_SET(PARAM_SET_DATA {
        param_value: f32::from_le_bytes(raw),
        target_system: target.0,
        target_component: target.1,
        param_id: name_to_wire(name).into(),
        param_type: kind_to_wire(kind),
    })
}

impl Service for Params {
    fn on_frame(&mut self, frame: &Frame, cx: &mut Ctx) {
        let Some(target) = self.target(cx) else {
            return;
        };
        if frame.system_id != target.0 {
            return;
        }
        let Body::Known(message) = &frame.body else {
            return;
        };
        match message {
            Message::AUTOPILOT_VERSION(version) => self.on_autopilot_version(target, version, cx),
            Message::PARAM_VALUE(value) => self.on_param_value(target, value, cx),
            _ => {}
        }
    }

    fn on_tick(&mut self, cx: &mut Ctx) {
        let Some(target) = self.target(cx) else {
            return;
        };
        self.tick_encoding(target, cx);
        self.tick_fetch(target, cx);
        self.tick_writes(target, cx);
        self.read_changed_offsets(target, cx);
    }

    fn on_command(&mut self, command: &Command, cx: &mut Ctx) {
        let Some(target) = self.target(cx) else {
            return;
        };
        match command {
            Command::RefreshParams => {
                self.restarts = 0;
                self.begin_fetch(target, cx);
            }
            Command::SetParam { name, value } => self.set_param(target, name, *value, cx),
            _ => {}
        }
    }
}

#[cfg(test)]
#[path = "params_tests.rs"]
mod tests;
