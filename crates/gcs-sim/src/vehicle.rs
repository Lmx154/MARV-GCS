//! The sans-IO vehicle: virtual time and received bytes in, bytes to send out.
//!
//! It behaves as section 2 of the link contract (`docs/link-contract.md`) says the real vehicle
//! will: MAVLink 2 only; frames with any incompatibility flag (signed ones too), frames with the
//! vehicle's own system id and frames addressed to another system or component are dropped; a
//! COMMAND_LONG for the vehicle gets a COMMAND_ACK.
//!
//! Commands: MAV_CMD_REQUEST_MESSAGE (512) is answered for AUTOPILOT_VERSION (148, unless
//! [`SimConfig::answer_version_request`] is off) and for the id of any configured stream. Every
//! other command, MAV_CMD_SET_MESSAGE_INTERVAL (511) included, gets UNSUPPORTED:
//! MAV_CMD_SET_MESSAGE_INTERVAL is not implemented in the simulator yet, although the real
//! vehicle will implement it.

use mavlink::dialects::common::{
    AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA, COMMAND_LONG_DATA, GPS_RAW_INT_DATA, HEARTBEAT_DATA,
    HIGHRES_IMU_DATA, MavCmd, MavMessage, MavProtocolCapability, MavResult, MavSeverity,
    PARAM_VALUE_DATA, SCALED_PRESSURE_DATA, STATUSTEXT_DATA,
};
use mavlink::types::CharArray;
use mavlink::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message, MessageData};

use crate::params::{ParamEncoding, ParamValue, SimParam, default_params, other_type_params};
use crate::signals;

/// MAVLink 2 start-of-frame marker.
const STX_V2: u8 = 0xFD;
/// Bytes before the payload of a MAVLink 2 frame (marker and header).
const V2_PREFIX: usize = 10;
const V2_CHECKSUM: usize = 2;
const V2_SIGNATURE: usize = 13;
/// Incompatibility flag of a signed frame: it makes the frame 13 bytes longer.
const IFLAG_SIGNED: u8 = 0x01;
/// The message AUTOPILOT_VERSION, requested with MAV_CMD_REQUEST_MESSAGE.
const AUTOPILOT_VERSION_ID: u32 = 148;
/// A step further ahead than this does not catch up: the vehicle carries on from the new time.
const MAX_CATCH_UP_MS: u64 = 1000;
/// Longest parameter name: `param_id` has 16 bytes.
const PARAM_NAME_LEN: usize = 16;
/// One parameter is sent per this many milliseconds of virtual time after PARAM_REQUEST_LIST.
const PARAM_LIST_PERIOD_MS: u64 = 2;
const BOOT_TEXT: &str = "MARV simulator ready";

/// One message the vehicle sends, and how often.
#[derive(Debug, Clone, PartialEq)]
pub struct Stream {
    pub message_id: u32,
    pub rate_hz: f32,
}

/// What the simulated vehicle is and sends.
#[derive(Debug, Clone, PartialEq)]
pub struct SimConfig {
    pub system_id: u8,
    pub component_id: u8,
    pub streams: Vec<Stream>,
    pub params: Vec<SimParam>,
    pub param_encoding: ParamEncoding,
    /// Percent (0 to 100) of outgoing frames that are dropped.
    pub drop_tx_percent: u8,
    /// Percent (0 to 100) of incoming frames that are dropped before they are acted on.
    pub drop_rx_percent: u8,
    /// Seed of the frame-loss generators.
    pub seed: u64,
    /// Whether MAV_CMD_REQUEST_MESSAGE for AUTOPILOT_VERSION is answered. When false the vehicle
    /// sends neither the COMMAND_ACK nor the message, to test a ground station's fallback.
    pub answer_version_request: bool,
}

impl SimConfig {
    /// Adds `n` parameters named `P0000`, `P0001`, ... of type Real32, for load tests.
    #[must_use]
    pub fn with_synthetic_params(mut self, n: usize) -> Self {
        let first = self.params.len();
        self.params.extend((0..n).map(|i| SimParam {
            name: format!("P{i:04}"),
            value: ParamValue::Real32((first + i) as f32),
            read_only: false,
        }));
        self
    }

    /// Adds one parameter of each type the contract does not use, for testing a ground station's
    /// general decoding: `TYPE_U8` 200, `TYPE_I8` -3, `TYPE_U16` 50000, `TYPE_I16` -320 and
    /// `TYPE_U32` 3735928559.
    #[must_use]
    pub fn with_every_param_type(mut self) -> Self {
        self.params.extend(other_type_params());
        self
    }
}

impl Default for SimConfig {
    fn default() -> Self {
        let stream = |message_id, rate_hz| Stream {
            message_id,
            rate_hz,
        };
        Self {
            system_id: 1,
            component_id: 1,
            streams: vec![
                stream(HEARTBEAT_DATA::ID, 1.0),
                stream(HIGHRES_IMU_DATA::ID, 50.0),
                stream(SCALED_PRESSURE_DATA::ID, 10.0),
                stream(GPS_RAW_INT_DATA::ID, 5.0),
            ],
            params: default_params(),
            param_encoding: ParamEncoding::default(),
            drop_tx_percent: 0,
            drop_rx_percent: 0,
            seed: 1,
            answer_version_request: true,
        }
    }
}

/// When one stream is next due: message `count` is due at `anchor_ms + count * period_ms`.
struct Schedule {
    message_id: u32,
    period_ms: f64,
    anchor_ms: f64,
    count: u64,
}

impl Schedule {
    fn due_ms(&self) -> f64 {
        self.anchor_ms + self.count as f64 * self.period_ms
    }
}

/// Decides which frames are lost: a xorshift64 generator, so the same seed gives the same losses.
struct LossGate {
    state: u64,
    percent: u8,
}

impl LossGate {
    fn new(seed: u64, percent: u8) -> Self {
        Self {
            state: if seed == 0 {
                0x9E37_79B9_7F4A_7C15
            } else {
                seed
            },
            percent,
        }
    }

    fn drops(&mut self) -> bool {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        (x >> 32) % 100 < u64::from(self.percent)
    }
}

/// A simulated MAVLink vehicle. It has no clock or thread, and its only randomness is the seeded
/// frame loss: the same sequence of [`step`](Self::step) calls gives the same bytes.
pub struct SimVehicle {
    config: SimConfig,
    schedules: Vec<Schedule>,
    last_now_ms: Option<u64>,
    sequence: u8,
    rx: Vec<u8>,
    heartbeats_received: u32,
    last_heartbeat_from: Option<(u8, u8)>,
    now_ms: u64,
    booted: bool,
    silent: bool,
    tx_loss: LossGate,
    rx_loss: LossGate,
    /// Next parameter index to send after PARAM_REQUEST_LIST, while a list is being sent.
    list_next: Option<usize>,
    list_due_ms: u64,
}

impl SimVehicle {
    /// # Panics
    ///
    /// If a parameter name is longer than 16 bytes or not ASCII, or there are more than 65535
    /// parameters: a wrong configuration of a test double must be loud.
    pub fn new(config: SimConfig) -> Self {
        for p in &config.params {
            assert!(
                p.name.len() <= PARAM_NAME_LEN && p.name.is_ascii(),
                "simulator parameter {:?}: the name must be ASCII and at most {PARAM_NAME_LEN} bytes",
                p.name
            );
        }
        assert!(
            u16::try_from(config.params.len()).is_ok(),
            "simulator has {} parameters, at most 65535 are possible",
            config.params.len()
        );
        let schedules = config
            .streams
            .iter()
            .filter(|s| s.rate_hz.is_finite() && s.rate_hz > 0.0)
            .filter(|s| !signals::messages(s.message_id, 0).is_empty())
            .map(|s| Schedule {
                message_id: s.message_id,
                period_ms: 1000.0 / f64::from(s.rate_hz),
                anchor_ms: 0.0,
                count: 0,
            })
            .collect();
        Self {
            tx_loss: LossGate::new(config.seed, config.drop_tx_percent),
            rx_loss: LossGate::new(config.seed ^ 0xD1B5_4A32_D192_ED03, config.drop_rx_percent),
            config,
            schedules,
            last_now_ms: None,
            sequence: 0,
            rx: Vec::new(),
            heartbeats_received: 0,
            last_heartbeat_from: None,
            now_ms: 0,
            booted: false,
            silent: false,
            list_next: None,
            list_due_ms: 0,
        }
    }

    /// Current value of the parameter `name`.
    pub fn param(&self, name: &str) -> Option<ParamValue> {
        self.config
            .params
            .iter()
            .find(|p| p.name == name)
            .map(|p| p.value)
    }

    /// While silent the vehicle sends nothing (the time still passes and the streams carry on
    /// unseen), but it still receives and acts: a PARAM_SET changes the value.
    pub fn set_silent(&mut self, silent: bool) {
        self.silent = silent;
    }

    /// Give it the bytes received since the last call and the time; get the bytes to send.
    ///
    /// Each message is stamped with the time it was due, which is `now_ms` unless the step is
    /// longer than the message period. A jump of more than one second (or backwards) is not made
    /// up: every stream restarts at the new time.
    pub fn step(&mut self, now_ms: u64, received: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        self.now_ms = now_ms;
        if !self.booted {
            self.booted = true;
            self.push_text(BOOT_TEXT, &mut out);
        }
        self.ingest(received, &mut out);
        self.emit_params(&mut out);
        self.emit_due(now_ms, &mut out);
        out
    }

    /// Number of HEARTBEAT frames received from a system other than this vehicle.
    pub fn heartbeats_received(&self) -> u32 {
        self.heartbeats_received
    }

    /// System and component id of the sender of the last counted HEARTBEAT.
    pub fn last_heartbeat_from(&self) -> Option<(u8, u8)> {
        self.last_heartbeat_from
    }

    fn emit_due(&mut self, now_ms: u64, out: &mut Vec<u8>) {
        let jumped = self
            .last_now_ms
            .is_none_or(|last| now_ms < last || now_ms - last > MAX_CATCH_UP_MS);
        if jumped {
            for s in &mut self.schedules {
                s.anchor_ms = now_ms as f64;
                s.count = 0;
            }
        }
        self.last_now_ms = Some(now_ms);

        let now = now_ms as f64;
        loop {
            let next = self
                .schedules
                .iter()
                .enumerate()
                .filter(|(_, s)| s.due_ms() <= now)
                .min_by(|(_, a), (_, b)| a.due_ms().total_cmp(&b.due_ms()))
                .map(|(i, _)| i);
            let Some(i) = next else { break };
            let stamp_ms = self.schedules[i].due_ms().floor() as u64;
            self.schedules[i].count += 1;
            for message in signals::messages(self.schedules[i].message_id, stamp_ms) {
                self.push_frame(&message, out);
            }
        }
    }

    /// Sends the parameters of a PARAM_REQUEST_LIST in index order, one per millisecond.
    fn emit_params(&mut self, out: &mut Vec<u8>) {
        while let Some(index) = self.list_next {
            if index >= self.config.params.len() {
                self.list_next = None;
            } else if self.list_due_ms <= self.now_ms {
                self.list_next = Some(index + 1);
                self.list_due_ms += PARAM_LIST_PERIOD_MS;
                self.push_param_value(index, out);
            } else {
                break;
            }
        }
    }

    fn push_param_value(&mut self, index: usize, out: &mut Vec<u8>) {
        let param = &self.config.params[index];
        let message = MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_value: param.value.to_field(self.config.param_encoding),
            param_count: self.config.params.len() as u16,
            param_index: index as u16,
            param_id: CharArray::from(param.name.as_str()),
            param_type: param.value.param_type(),
        });
        self.push_frame(&message, out);
    }

    fn push_text(&mut self, text: &str, out: &mut Vec<u8>) {
        let message = MavMessage::STATUSTEXT(STATUSTEXT_DATA {
            severity: MavSeverity::MAV_SEVERITY_INFO,
            text: CharArray::from(text),
            ..STATUSTEXT_DATA::default()
        });
        self.push_frame(&message, out);
    }

    fn push_frame(&mut self, message: &MavMessage, out: &mut Vec<u8>) {
        if self.silent {
            return;
        }
        let header = MavHeader {
            system_id: u32::from(self.config.system_id),
            component_id: self.config.component_id,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message(header, message);
        if !self.tx_loss.drops() {
            out.extend_from_slice(raw.raw_bytes());
        }
    }

    /// Appends received bytes and consumes every complete MAVLink 2 frame in them. Bytes that do
    /// not start a valid frame are skipped; an incomplete frame waits for the next call, unless a
    /// valid frame follows it (then the incomplete one was a false start).
    fn ingest(&mut self, received: &[u8], out: &mut Vec<u8>) {
        self.rx.extend_from_slice(received);
        let mut pos = 0;
        loop {
            let Some(offset) = self.rx[pos..].iter().position(|&b| b == STX_V2) else {
                pos = self.rx.len();
                break;
            };
            pos += offset;
            let rest = &self.rx[pos..];
            match frame_len(rest) {
                Some(total) if rest.len() >= total => match decode_frame(&rest[..total]) {
                    Some((system_id, component_id, message)) => {
                        pos += total;
                        if !self.rx_loss.drops() {
                            self.handle(system_id, component_id, &message, out);
                        }
                    }
                    None => pos += 1,
                },
                _ => match later_valid_frame(rest) {
                    Some(start) => pos += start,
                    None => break,
                },
            }
        }
        self.rx.drain(..pos);
    }

    fn handle(&mut self, system_id: u8, component_id: u8, message: &MavMessage, out: &mut Vec<u8>) {
        if system_id == self.config.system_id {
            return;
        }
        match message {
            MavMessage::HEARTBEAT(_) => {
                self.heartbeats_received = self.heartbeats_received.saturating_add(1);
                self.last_heartbeat_from = Some((system_id, component_id));
            }
            MavMessage::PARAM_REQUEST_LIST(d)
                if self.is_addressed(d.target_system, d.target_component) =>
            {
                self.list_next = Some(0);
                self.list_due_ms = self.now_ms;
            }
            MavMessage::PARAM_REQUEST_READ(d)
                if self.is_addressed(d.target_system, d.target_component) =>
            {
                let index = if d.param_index == -1 {
                    self.find_param(&d.param_id[..])
                } else {
                    usize::try_from(d.param_index).ok()
                };
                if let Some(index) = index.filter(|&i| i < self.config.params.len()) {
                    self.push_param_value(index, out);
                }
            }
            MavMessage::PARAM_SET(d) if self.is_addressed(d.target_system, d.target_component) => {
                let Some(index) = self.find_param(&d.param_id[..]) else {
                    return;
                };
                let param = &mut self.config.params[index];
                if !param.read_only && d.param_type == param.value.param_type() {
                    param.value = param
                        .value
                        .same_type_from_field(d.param_value, self.config.param_encoding);
                    let text = format!("{} set", param.name);
                    self.push_param_value(index, out);
                    self.push_text(&text, out);
                } else {
                    self.push_param_value(index, out);
                }
            }
            MavMessage::COMMAND_LONG(d)
                if self.is_addressed(d.target_system, d.target_component) =>
            {
                self.handle_command(d, (system_id, component_id), out);
            }
            _ => {}
        }
    }

    /// Answers a COMMAND_LONG addressed to this vehicle. `from` is the sender's system and
    /// component, to which the COMMAND_ACK goes.
    fn handle_command(&mut self, d: &COMMAND_LONG_DATA, from: (u8, u8), out: &mut Vec<u8>) {
        if d.command != MavCmd::MAV_CMD_REQUEST_MESSAGE {
            self.push_ack(d.command, MavResult::MAV_RESULT_UNSUPPORTED, from, out);
            return;
        }
        let requested = Some(d.param1)
            .filter(|v| v.is_finite() && *v >= 0.0 && v.fract() == 0.0 && *v < u32::MAX as f32)
            .map(|v| v as u32);
        let messages = match requested {
            Some(AUTOPILOT_VERSION_ID) if !self.config.answer_version_request => return,
            Some(AUTOPILOT_VERSION_ID) => vec![MavMessage::AUTOPILOT_VERSION(self.version())],
            Some(id) if self.config.streams.iter().any(|s| s.message_id == id) => {
                signals::messages(id, self.now_ms)
            }
            _ => Vec::new(),
        };
        if messages.is_empty() {
            self.push_ack(d.command, MavResult::MAV_RESULT_UNSUPPORTED, from, out);
            return;
        }
        self.push_ack(d.command, MavResult::MAV_RESULT_ACCEPTED, from, out);
        for message in &messages {
            self.push_frame(message, out);
        }
    }

    /// AUTOPILOT_VERSION: MAVLINK2 and the parameter encoding of this vehicle, nothing else set.
    fn version(&self) -> AUTOPILOT_VERSION_DATA {
        let encoding = match self.config.param_encoding {
            ParamEncoding::Bytewise => {
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
            }
            ParamEncoding::CCast => {
                MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST
            }
        };
        AUTOPILOT_VERSION_DATA {
            capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2 | encoding,
            ..AUTOPILOT_VERSION_DATA::default()
        }
    }

    fn push_ack(&mut self, command: MavCmd, result: MavResult, to: (u8, u8), out: &mut Vec<u8>) {
        let message = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: to.0,
            target_component: to.1,
        });
        self.push_frame(&message, out);
    }

    /// Whether a target system and component (0 is everyone) include this vehicle.
    fn is_addressed(&self, target_system: u8, target_component: u8) -> bool {
        (target_system == 0 || target_system == self.config.system_id)
            && (target_component == 0 || target_component == self.config.component_id)
    }

    /// Index of the parameter named in a `param_id`: the bytes up to the first NUL, or all 16.
    fn find_param(&self, param_id: &[u8]) -> Option<usize> {
        let end = param_id
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(param_id.len());
        let name = &param_id[..end];
        self.config
            .params
            .iter()
            .position(|p| p.name.as_bytes() == name)
    }
}

/// Length of the frame starting at `rest[0]`, once its first three bytes are there.
fn frame_len(rest: &[u8]) -> Option<usize> {
    let flags = *rest.get(2)?;
    let signature = if flags & IFLAG_SIGNED != 0 {
        V2_SIGNATURE
    } else {
        0
    };
    Some(V2_PREFIX + usize::from(rest[1]) + V2_CHECKSUM + signature)
}

/// Offset of the first complete, valid frame that starts after `rest[0]`.
fn later_valid_frame(rest: &[u8]) -> Option<usize> {
    (1..rest.len()).filter(|&i| rest[i] == STX_V2).find(|&i| {
        frame_len(&rest[i..]).is_some_and(|total| {
            rest.len() - i >= total && decode_frame(&rest[i..i + total]).is_some()
        })
    })
}

/// Checks one complete MAVLink 2 frame (flags, checksum, payload) and returns sender and message.
fn decode_frame(frame: &[u8]) -> Option<(u8, u8, MavMessage)> {
    if frame[2] != 0 {
        return None;
    }
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.as_mut_slice()
        .get_mut(..frame.len())?
        .copy_from_slice(frame);
    if !raw.has_valid_crc::<MavMessage>() {
        return None;
    }
    let message = MavMessage::parse(MavlinkVersion::V2, raw.message_id(), raw.payload()).ok()?;
    let system_id = u8::try_from(raw.system_id()).ok()?;
    Some((system_id, raw.component_id(), message))
}
