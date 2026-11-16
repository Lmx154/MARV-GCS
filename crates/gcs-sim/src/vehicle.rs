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
//! vehicle will implement it. The actions of `docs/link-contract.md` section 10 (241, 245, 246 and
//! 42430) are answered as the vehicle does: see [`SimVehicle`] and the [`actions`] module. A
//! COMMAND_LONG is read by hand from its bytes (its `command` may be a number the `common` dialect
//! does not have, 42430).
//!
//! Logs: LOG_REQUEST_LIST, LOG_REQUEST_DATA and LOG_REQUEST_END are answered as `docs/link-contract.md`
//! section 11 says the real vehicle will answer them: see [`crate::SimLog`] and [`crate::LogDataDrop`].

use mavlink::dialects::common::{
    AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA, GPS_RAW_INT_DATA, HEARTBEAT_DATA, HIGHRES_IMU_DATA,
    LOG_DATA_DATA, LOG_ENTRY_DATA, MANUAL_CONTROL_DATA, MavMessage, MavProtocolCapability,
    MavResult, MavSeverity, PARAM_VALUE_DATA, RC_CHANNELS_DATA, SCALED_IMU2_DATA,
    SCALED_PRESSURE_DATA, STATUSTEXT_DATA, SYS_STATUS_DATA,
};
use mavlink::types::CharArray;
use mavlink::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message, MessageData};

use crate::actions::{self, Action, Admission};
use crate::devices;
use crate::logs::{
    self, FRAME_BYTES, HEARTBEAT_LOST_MS, LogDataDrop, Outbox, Queued, SLOT_FRAMES, SLOT_MS,
    SimLog, Transfer,
};
use crate::params::{ParamEncoding, ParamValue, SimParam, default_params, other_type_params};
use crate::{rcmap, signals};

/// MAVLink 2 start-of-frame marker.
const STX_V2: u8 = 0xFD;
/// Bytes before the payload of a MAVLink 2 frame (marker and header).
const V2_PREFIX: usize = 10;
const V2_CHECKSUM: usize = 2;
const V2_SIGNATURE: usize = 13;
/// Incompatibility flag of a signed frame: it makes the frame 13 bytes longer.
const IFLAG_SIGNED: u8 = 0x01;
/// MAV_CMD_REQUEST_MESSAGE.
const MAV_CMD_REQUEST_MESSAGE: u16 = 512;
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
    /// The first this many COMMAND_LONGs other than MAV_CMD_REQUEST_MESSAGE are dropped unanswered,
    /// to test a ground station's retransmission. 0 is none.
    pub drop_first_commands: u32,
    /// An admitted action gets its IN_PROGRESS ACKs but never its final ACK (it still ends on the
    /// vehicle), to test a ground station's "no result".
    pub never_final_ack: bool,
    /// The first this many PARAM_VALUE echoes of a PARAM_SET are not sent (the write itself is
    /// applied, and a refusal's STATUSTEXT is still sent), to test a ground station's resend of a
    /// write whose echo was lost. 0 is none.
    pub drop_first_param_echoes: u32,
    /// When not 0, an accepted PARAM_SET is first answered with a PARAM_VALUE carrying the OLD
    /// value (a stale copy in flight) and the real echo with the new value follows this many
    /// milliseconds later, to test a ground station that must not read the stale copy as a
    /// refusal. 0 is off.
    pub stale_echo_delay_ms: u64,
    /// The logs on the card (`docs/link-contract.md` section 11): none is an empty card.
    pub logs: Vec<SimLog>,
    /// LOG_DATA frames to lose, by offset, to test a ground station's re-requests.
    pub drop_log_data: Vec<LogDataDrop>,
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

impl SimConfig {
    /// Adds the stream of `MARV_DEVICE_STATUS` (`docs/link-contract.md`, section 8): at `rate_hz`
    /// one message for each of IMU1, IMU2, BARO, MAG, GNSS, BATT and RC. Off in the default
    /// configuration, which is the streams of section 2.
    #[must_use]
    pub fn with_device_status(mut self, rate_hz: f32) -> Self {
        self.streams.push(Stream {
            message_id: devices::MESSAGE_ID,
            rate_hz,
        });
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
                // The streams of the real firmware on its USB port.
                stream(HEARTBEAT_DATA::ID, 1.0),
                stream(SYS_STATUS_DATA::ID, 1.0),
                stream(HIGHRES_IMU_DATA::ID, 25.0),
                stream(SCALED_IMU2_DATA::ID, 25.0),
                stream(SCALED_PRESSURE_DATA::ID, 25.0),
                stream(GPS_RAW_INT_DATA::ID, 1.0),
                stream(RC_CHANNELS_DATA::ID, 5.0),
                stream(MANUAL_CONTROL_DATA::ID, 5.0),
            ],
            params: default_params(),
            param_encoding: ParamEncoding::default(),
            drop_tx_percent: 0,
            drop_rx_percent: 0,
            seed: 1,
            answer_version_request: true,
            drop_first_commands: 0,
            never_final_ack: false,
            drop_first_param_echoes: 0,
            stale_echo_delay_ms: 0,
            logs: logs::default_logs(),
            drop_log_data: Vec::new(),
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
    /// The one action that runs, if any.
    action: Option<Action>,
    /// COMMAND_LONGs dropped so far for `drop_first_commands`.
    commands_dropped: u32,
    /// PARAM_VALUE echoes dropped so far for `drop_first_param_echoes`.
    echoes_dropped: u32,
    /// The real echo held back by `stale_echo_delay_ms`: parameter index and when it is due.
    held_echo: Option<(usize, u64)>,
    /// PARAM_SET frames addressed to this vehicle so far.
    param_sets_received: u32,
    /// What the vehicle has queued to send for the log download.
    logs_out: Outbox,
    /// Time of the last HEARTBEAT from another system.
    last_heartbeat_ms: Option<u64>,
}

/// A COMMAND_LONG read from its bytes: the command is a number, not a dialect enum.
struct RawCommand {
    command: u16,
    params: [f32; 7],
    target_system: u8,
    target_component: u8,
}

/// What `decode_frame` makes of a valid frame.
enum Incoming {
    Message(Box<MavMessage>),
    Command(RawCommand),
}

/// MAVLink message id of COMMAND_LONG.
const COMMAND_LONG_ID: u32 = 76;
/// Payload length of a COMMAND_LONG with no trailing zero cut.
const COMMAND_LONG_LEN: usize = 33;

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
            .filter(|s| {
                s.message_id == devices::MESSAGE_ID
                    || s.message_id == MANUAL_CONTROL_DATA::ID
                    || !signals::messages(s.message_id, 0).is_empty()
            })
            .map(|s| Schedule {
                message_id: s.message_id,
                period_ms: 1000.0 / f64::from(s.rate_hz),
                anchor_ms: 0.0,
                count: 0,
            })
            .collect();
        let config_drops = config.drop_log_data.clone();
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
            action: None,
            commands_dropped: 0,
            echoes_dropped: 0,
            held_echo: None,
            param_sets_received: 0,
            logs_out: Outbox {
                drops: config_drops,
                ..Outbox::default()
            },
            last_heartbeat_ms: None,
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
        self.emit_held_echo(&mut out);
        self.emit_action(now_ms, &mut out);
        self.emit_logs(now_ms, &mut out);
        self.emit_due(now_ms, &mut out);
        out
    }

    /// Number of HEARTBEAT frames received from a system other than this vehicle.
    pub fn heartbeats_received(&self) -> u32 {
        self.heartbeats_received
    }

    /// Number of PARAM_SET frames addressed to this vehicle that arrived, for a known name or not.
    pub fn param_sets_received(&self) -> u32 {
        self.param_sets_received
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
            if self.schedules[i].message_id == devices::MESSAGE_ID {
                for payload in devices::payloads(stamp_ms) {
                    self.push_raw(devices::MESSAGE_ID, devices::CRC_EXTRA, &payload, out);
                }
                continue;
            }
            for message in self.stream_messages(self.schedules[i].message_id, stamp_ms) {
                self.push_frame(&message, out);
            }
        }
    }

    /// The messages of a stream at `now_ms`. MANUAL_CONTROL follows the RC parameters now in effect
    /// (`docs/link-contract.md` section 12); it is empty for a vehicle that has none.
    fn stream_messages(&self, message_id: u32, now_ms: u64) -> Vec<MavMessage> {
        if message_id != MANUAL_CONTROL_DATA::ID {
            return signals::messages(message_id, now_ms);
        }
        let channels = rcmap::channels_of(&signals::rc_channels(now_ms));
        rcmap::manual_control(self.config.system_id, &channels, |name| self.param(name))
            .map(MavMessage::MANUAL_CONTROL)
            .into_iter()
            .collect()
    }

    /// Sends the parameters of a PARAM_REQUEST_LIST in index order, one per millisecond.
    fn emit_held_echo(&mut self, out: &mut Vec<u8>) {
        if let Some((index, due_ms)) = self.held_echo
            && due_ms <= self.now_ms
        {
            self.held_echo = None;
            self.push_param_value(index, out);
        }
    }

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
        self.push_text_with(MavSeverity::MAV_SEVERITY_INFO, text, out);
    }

    fn push_text_with(&mut self, severity: MavSeverity, text: &str, out: &mut Vec<u8>) {
        let message = MavMessage::STATUSTEXT(STATUSTEXT_DATA {
            severity,
            text: CharArray::from(text),
            ..STATUSTEXT_DATA::default()
        });
        self.push_frame(&message, out);
    }

    /// A frame of a message the `common` dialect does not have, built by hand from its id, its
    /// CRC_EXTRA and its payload; the trailing zero bytes of the payload are cut, but one (MAVLink 2).
    fn push_raw(&mut self, id: u32, crc_extra: u8, payload: &[u8], out: &mut Vec<u8>) {
        if self.silent {
            return;
        }
        let used = payload.iter().rposition(|&b| b != 0).map_or(1, |i| i + 1);
        let mut frame = vec![
            STX_V2,
            used as u8,
            0,
            0,
            self.sequence,
            self.config.system_id,
            self.config.component_id,
        ];
        frame.extend_from_slice(&id.to_le_bytes()[..3]);
        frame.extend_from_slice(&payload[..used]);
        let crc = mavlink::calculate_crc(&frame[1..], crc_extra);
        frame.extend_from_slice(&crc.to_le_bytes());
        self.sequence = self.sequence.wrapping_add(1);
        if !self.tx_loss.drops() {
            out.extend_from_slice(&frame);
        }
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
                    Some((system_id, component_id, incoming)) => {
                        pos += total;
                        if !self.rx_loss.drops() {
                            match incoming {
                                Incoming::Message(message) => {
                                    self.handle(system_id, component_id, &message, out);
                                }
                                Incoming::Command(command) => {
                                    self.handle_long(system_id, component_id, &command, out);
                                }
                            }
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
                self.last_heartbeat_ms = Some(self.now_ms);
            }
            MavMessage::LOG_REQUEST_LIST(d)
                if self.is_addressed(d.target_system, d.target_component) =>
            {
                self.list_logs(d.start, d.end);
            }
            MavMessage::LOG_REQUEST_DATA(d)
                if self.is_addressed(d.target_system, d.target_component) =>
            {
                self.serve_log(d.id, d.ofs, d.count);
            }
            MavMessage::LOG_REQUEST_END(d)
                if self.is_addressed(d.target_system, d.target_component) =>
            {
                self.logs_out.transfer = None;
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
                self.param_sets_received += 1;
                let Some(index) = self.find_param(&d.param_id[..]) else {
                    return;
                };
                let param = &mut self.config.params[index];
                let reason = if param.read_only {
                    Some("read only")
                } else if d.param_type != param.value.param_type() {
                    Some("out of range")
                } else if matches!(
                    param.value.same_type_from_field(d.param_value, self.config.param_encoding),
                    ParamValue::Int32(v) if rcmap::out_of_range(&param.name, v)
                ) {
                    Some("out of range")
                } else {
                    None
                };
                let refusal = reason.map(|reason| format!("{}: {reason}", param.name));
                let stale = reason.is_none() && self.config.stale_echo_delay_ms > 0;
                if stale {
                    self.push_param_value(index, out);
                    self.held_echo = Some((index, self.now_ms + self.config.stale_echo_delay_ms));
                }
                let param = &mut self.config.params[index];
                if reason.is_none() {
                    param.value = param
                        .value
                        .same_type_from_field(d.param_value, self.config.param_encoding);
                }
                if stale {
                } else if self.echoes_dropped < self.config.drop_first_param_echoes {
                    self.echoes_dropped += 1;
                } else {
                    self.push_param_value(index, out);
                }
                if let Some(text) = refusal {
                    self.push_text_with(MavSeverity::MAV_SEVERITY_WARNING, &text, out);
                }
            }
            _ => {}
        }
    }

    /// Starts a slot clock when the vehicle had nothing to send.
    fn wake_logs(&mut self) {
        if self.logs_out.queue.is_empty() && self.logs_out.transfer.is_none() {
            self.logs_out.due_ms = self.now_ms;
        }
    }

    /// LOG_REQUEST_LIST: one LOG_ENTRY per log in `start..=end`, in number order; one with id 0 when the
    /// card has no log. A new list replaces the entries not yet sent and a running transfer (one
    /// activity at a time).
    fn list_logs(&mut self, start: u16, end: u16) {
        let mut logs = self.config.logs.clone();
        logs.sort_by_key(|l| l.number);
        let last_log_num = logs.last().map_or(0, |l| l.number);
        let num_logs = logs.len() as u16;
        self.wake_logs();
        self.logs_out.transfer = None;
        self.logs_out
            .queue
            .retain(|q| !matches!(q, Queued::Entry { .. }));
        if logs.is_empty() {
            self.logs_out.queue.push_back(Queued::Entry {
                id: 0,
                num_logs: 0,
                last_log_num: 0,
                size: 0,
            });
        }
        for log in logs.iter().filter(|l| (start..=end).contains(&l.number)) {
            self.logs_out.queue.push_back(Queued::Entry {
                id: log.number,
                num_logs,
                last_log_num,
                size: log.size,
            });
        }
    }

    /// LOG_REQUEST_DATA: replaces the running transfer. Nothing at all for a log that does not exist.
    fn serve_log(&mut self, id: u16, ofs: u32, count: u32) {
        let Some(log) = self.config.logs.iter().find(|l| l.number == id) else {
            return;
        };
        let size = log.size;
        self.wake_logs();
        self.logs_out.transfer = None;
        if ofs >= size {
            self.logs_out.queue.push_back(Queued::EndMarker { id, ofs });
            return;
        }
        self.logs_out.transfer = Some(Transfer {
            number: id,
            next: ofs,
            end: ofs.saturating_add(count).min(size),
            started_ms: self.now_ms,
        });
    }

    /// Through the slot: 4 frames per 5 ms of virtual time, entries and markers first.
    fn emit_logs(&mut self, now_ms: u64, out: &mut Vec<u8>) {
        if let Some(transfer) = &self.logs_out.transfer {
            let seen = self.last_heartbeat_ms.unwrap_or(transfer.started_ms);
            if now_ms.saturating_sub(seen) > HEARTBEAT_LOST_MS {
                self.logs_out.transfer = None;
            }
        }
        if self.logs_out.queue.is_empty() && self.logs_out.transfer.is_none() {
            return;
        }
        if now_ms.saturating_sub(self.logs_out.due_ms) > MAX_CATCH_UP_MS {
            self.logs_out.due_ms = now_ms;
        }
        while self.logs_out.due_ms <= now_ms
            && (!self.logs_out.queue.is_empty() || self.logs_out.transfer.is_some())
        {
            self.logs_out.due_ms += SLOT_MS;
            for _ in 0..SLOT_FRAMES {
                if let Some(queued) = self.logs_out.queue.pop_front() {
                    let message = match queued {
                        Queued::Entry {
                            id,
                            num_logs,
                            last_log_num,
                            size,
                        } => MavMessage::LOG_ENTRY(LOG_ENTRY_DATA {
                            time_utc: 0,
                            size,
                            id,
                            num_logs,
                            last_log_num,
                        }),
                        Queued::EndMarker { id, ofs } => MavMessage::LOG_DATA(LOG_DATA_DATA {
                            ofs,
                            id,
                            count: 0,
                            data: [0; 90],
                        }),
                    };
                    self.push_frame(&message, out);
                } else if let Some(frame) = self.next_log_frame() {
                    self.push_frame(&frame, out);
                }
            }
        }
    }

    /// The next LOG_DATA of the transfer, or `None` when it ended or the frame is one that is lost.
    fn next_log_frame(&mut self) -> Option<MavMessage> {
        let transfer = self.logs_out.transfer.as_mut()?;
        let (number, ofs) = (transfer.number, transfer.next);
        let count = (transfer.end - ofs).min(FRAME_BYTES);
        transfer.next += count;
        let finished = transfer.next >= transfer.end;
        let size = self.config.logs.iter().find(|l| l.number == number)?.size;
        if finished {
            self.logs_out.transfer = None;
        }
        if self.logs_out.drops_frame(ofs) {
            return None;
        }
        let bytes = logs::log_bytes_range(number, size, ofs, count);
        let mut data = [0u8; 90];
        data[..bytes.len()].copy_from_slice(&bytes);
        Some(MavMessage::LOG_DATA(LOG_DATA_DATA {
            ofs,
            id: number,
            count: count as u8,
            data,
        }))
    }

    /// A COMMAND_LONG from another system: the ones for this vehicle are answered.
    fn handle_long(&mut self, system_id: u8, component_id: u8, d: &RawCommand, out: &mut Vec<u8>) {
        if system_id == self.config.system_id
            || !self.is_addressed(d.target_system, d.target_component)
        {
            return;
        }
        let from = (system_id, component_id);
        if d.command == MAV_CMD_REQUEST_MESSAGE {
            self.handle_request_message(d, from, out);
            return;
        }
        if self.commands_dropped < self.config.drop_first_commands {
            self.commands_dropped += 1;
            return;
        }
        self.handle_action_command(d, from, out);
    }

    /// MAV_CMD_REQUEST_MESSAGE: the message and an ACK; anything not served is UNSUPPORTED.
    fn handle_request_message(&mut self, d: &RawCommand, from: (u8, u8), out: &mut Vec<u8>) {
        let requested = Some(d.params[0])
            .filter(|v| v.is_finite() && *v >= 0.0 && v.fract() == 0.0 && *v < u32::MAX as f32)
            .map(|v| v as u32);
        let messages = match requested {
            Some(AUTOPILOT_VERSION_ID) if !self.config.answer_version_request => return,
            Some(AUTOPILOT_VERSION_ID) => vec![MavMessage::AUTOPILOT_VERSION(self.version())],
            Some(id) if self.config.streams.iter().any(|s| s.message_id == id) => {
                self.stream_messages(id, self.now_ms)
            }
            _ => Vec::new(),
        };
        if messages.is_empty() {
            self.push_ack(d.command, MavResult::MAV_RESULT_UNSUPPORTED, 0, from, out);
            return;
        }
        self.push_ack(d.command, MavResult::MAV_RESULT_ACCEPTED, 0, from, out);
        for message in &messages {
            self.push_frame(message, out);
        }
    }

    /// The commands of `docs/link-contract.md` section 10, in the vehicle's admission order:
    /// UNSUPPORTED or DENIED by command and parameters, then the cancel, then TEMPORARILY_REJECTED
    /// when an action runs, then admitted.
    fn handle_action_command(&mut self, d: &RawCommand, from: (u8, u8), out: &mut Vec<u8>) {
        match actions::admit(d.command, &d.params) {
            Admission::Unsupported => {
                self.push_ack(d.command, MavResult::MAV_RESULT_UNSUPPORTED, 0, from, out);
            }
            Admission::Denied(reason) => {
                self.push_text(reason, out);
                self.push_ack(d.command, MavResult::MAV_RESULT_DENIED, 0, from, out);
            }
            Admission::Cancel => {
                let running = self.action.as_ref().filter(|a| a.is_calibration());
                let Some(running) = running.cloned() else {
                    self.push_text("no calibration is running", out);
                    self.push_ack(d.command, MavResult::MAV_RESULT_DENIED, 0, from, out);
                    return;
                };
                self.push_ack(d.command, MavResult::MAV_RESULT_ACCEPTED, 0, from, out);
                self.action = None;
                if !self.config.never_final_ack {
                    self.push_ack(
                        running.command,
                        MavResult::MAV_RESULT_CANCELLED,
                        0,
                        running.from,
                        out,
                    );
                }
            }
            Admission::Run { duration_ms } => {
                if let Some(running) = &self.action {
                    if running.command == d.command && running.params == d.params {
                        let percent = running.percent(self.now_ms);
                        self.push_ack(
                            d.command,
                            MavResult::MAV_RESULT_IN_PROGRESS,
                            percent,
                            from,
                            out,
                        );
                    } else {
                        self.push_text("another action is running", out);
                        self.push_ack(
                            d.command,
                            MavResult::MAV_RESULT_TEMPORARILY_REJECTED,
                            0,
                            from,
                            out,
                        );
                    }
                    return;
                }
                self.action = Some(Action::new(
                    d.command,
                    d.params,
                    from,
                    self.now_ms,
                    duration_ms,
                ));
                self.push_ack(d.command, MavResult::MAV_RESULT_IN_PROGRESS, 0, from, out);
            }
        }
    }

    /// Progress ACKs every 500 ms of the running action, then its final ACK.
    fn emit_action(&mut self, now_ms: u64, out: &mut Vec<u8>) {
        while let Some(action) = &mut self.action {
            let Some(due) = action.next_due(now_ms) else {
                break;
            };
            let (command, from) = (action.command, action.from);
            match due {
                actions::Due::Progress(percent) => {
                    self.push_ack(
                        command,
                        MavResult::MAV_RESULT_IN_PROGRESS,
                        percent,
                        from,
                        out,
                    );
                }
                actions::Due::Final => {
                    self.action = None;
                    if !self.config.never_final_ack {
                        self.push_ack(command, MavResult::MAV_RESULT_ACCEPTED, 0, from, out);
                    }
                }
            }
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

    /// A COMMAND_ACK to `to`, built by hand (the command may be a number `common` does not have).
    fn push_ack(
        &mut self,
        command: u16,
        result: MavResult,
        progress: u8,
        to: (u8, u8),
        out: &mut Vec<u8>,
    ) {
        let mut payload = Vec::with_capacity(10);
        payload.extend_from_slice(&command.to_le_bytes());
        payload.push(result as u8);
        payload.push(progress);
        payload.extend_from_slice(&0i32.to_le_bytes());
        payload.push(to.0);
        payload.push(to.1);
        self.push_raw(
            COMMAND_ACK_DATA::ID,
            COMMAND_ACK_DATA::EXTRA_CRC,
            &payload,
            out,
        );
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
/// A COMMAND_LONG is read by hand, whatever its command number.
fn decode_frame(frame: &[u8]) -> Option<(u8, u8, Incoming)> {
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
    let system_id = u8::try_from(raw.system_id()).ok()?;
    if raw.message_id() == COMMAND_LONG_ID {
        let mut payload = [0u8; COMMAND_LONG_LEN];
        let given = raw.payload();
        payload.get_mut(..given.len())?.copy_from_slice(given);
        let mut params = [0f32; 7];
        for (i, param) in params.iter_mut().enumerate() {
            *param = f32::from_le_bytes(payload[4 * i..4 * i + 4].try_into().ok()?);
        }
        let command = RawCommand {
            command: u16::from_le_bytes([payload[28], payload[29]]),
            params,
            target_system: payload[30],
            target_component: payload[31],
        };
        return Some((system_id, raw.component_id(), Incoming::Command(command)));
    }
    let message = MavMessage::parse(MavlinkVersion::V2, raw.message_id(), raw.payload()).ok()?;
    Some((
        system_id,
        raw.component_id(),
        Incoming::Message(Box::new(message)),
    ))
}
