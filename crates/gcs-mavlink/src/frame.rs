//! MAVLink 2 framing: bytes in, frames out, and the other way round. No I/O, no clock.

use mavlink::{MavHeader, MavlinkVersion, Message as _};

use crate::Message;

const STX: u8 = 0xFD;
const HEADER_LEN: usize = 10;
const CHECKSUM_LEN: usize = 2;
const SIGNATURE_LEN: usize = 13;
const INCOMPAT_SIGNED: u8 = 0x01;
const MAX_PAYLOAD: usize = 255;
const MAX_FRAME: usize = HEADER_LEN + MAX_PAYLOAD + CHECKSUM_LEN + SIGNATURE_LEN;
/// Input is appended in slices of this size, so the buffer never exceeds `MAX_FRAME` + `FEED_CHUNK`.
const FEED_CHUNK: usize = MAX_FRAME;

/// One frame as received: header fields, the exact wire bytes and the decoded body.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub system_id: u8,
    pub component_id: u8,
    pub sequence: u8,
    pub message_id: u32,
    /// The whole frame exactly as it was on the wire.
    pub raw: Vec<u8>,
    pub body: Body,
}

#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::large_enum_variant)] // `Known(Message)` is the specified shape of the public API
pub enum Body {
    /// Id in the dialect, checksum good, decoded.
    Known(Message),
    /// Id in the dialect, checksum good, payload refused by the decoder.
    Undecodable(String),
    /// Id not in the dialect: the checksum cannot be checked.
    Unknown,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RxStats {
    pub frames: u64,
    pub crc_errors: u64,
    pub unknown: u64,
    pub undecodable: u64,
    pub discarded_bytes: u64,
}

/// Sans-IO MAVLink 2 deframer.
#[derive(Debug, Default)]
pub struct Deframer {
    buf: Vec<u8>,
    stats: RxStats,
    #[cfg(test)]
    peak: usize,
}

#[allow(clippy::large_enum_variant)] // short-lived return value, never stored
enum Step {
    /// Not enough bytes yet to decide about the candidate at the start of the buffer.
    Wait,
    /// Not a frame: drop the first byte and search again.
    Drop,
    /// A frame of this many bytes.
    Deliver(usize, Body),
}

impl Deframer {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, bytes: &[u8], out: &mut Vec<Frame>) {
        for chunk in bytes.chunks(FEED_CHUNK) {
            self.buf.extend_from_slice(chunk);
            #[cfg(test)]
            {
                self.peak = self.peak.max(self.buf.len());
            }
            self.scan(out);
        }
    }

    pub fn stats(&self) -> RxStats {
        self.stats
    }

    #[cfg(test)]
    fn buffered_len(&self) -> usize {
        self.buf.len()
    }

    fn scan(&mut self, out: &mut Vec<Frame>) {
        let mut pos = 0;
        while pos < self.buf.len() {
            match self.buf[pos..].iter().position(|&b| b == STX) {
                Some(skip) => {
                    self.stats.discarded_bytes += skip as u64;
                    pos += skip;
                }
                None => {
                    self.stats.discarded_bytes += (self.buf.len() - pos) as u64;
                    pos = self.buf.len();
                    break;
                }
            }
            match examine(&mut self.stats, &self.buf[pos..]) {
                Step::Wait => break,
                Step::Drop => {
                    self.stats.discarded_bytes += 1;
                    pos += 1;
                }
                Step::Deliver(len, body) => {
                    let raw = self.buf[pos..pos + len].to_vec();
                    self.stats.frames += 1;
                    match body {
                        Body::Unknown => self.stats.unknown += 1,
                        Body::Undecodable(_) => self.stats.undecodable += 1,
                        Body::Known(_) => {}
                    }
                    out.push(Frame {
                        system_id: raw[5],
                        component_id: raw[6],
                        sequence: raw[4],
                        message_id: message_id(&raw),
                        raw,
                        body,
                    });
                    pos += len;
                }
            }
        }
        self.buf.drain(..pos);
    }
}

/// Decide about the candidate frame starting at `data[0]` (which is `STX`).
fn examine(stats: &mut RxStats, data: &[u8]) -> Step {
    if data.len() < HEADER_LEN {
        return Step::Wait;
    }
    let payload_len = usize::from(data[1]);
    let incompat = data[2];
    if incompat & !INCOMPAT_SIGNED != 0 {
        return Step::Drop;
    }
    let signature_len = if incompat & INCOMPAT_SIGNED != 0 {
        SIGNATURE_LEN
    } else {
        0
    };
    let checksum_at = HEADER_LEN + payload_len;
    let total = checksum_at + CHECKSUM_LEN + signature_len;
    if data.len() < total {
        return Step::Wait;
    }
    let id = message_id(data);
    if Message::default_message_from_id(id).is_none() {
        return match data.get(total) {
            None => Step::Wait,
            Some(&STX) => Step::Deliver(total, Body::Unknown),
            Some(_) => Step::Drop,
        };
    }
    let expected = mavlink::calculate_crc(&data[1..checksum_at], Message::extra_crc(id));
    let received = u16::from_le_bytes([data[checksum_at], data[checksum_at + 1]]);
    if expected != received {
        stats.crc_errors += 1;
        return Step::Drop;
    }
    let body = match Message::parse(MavlinkVersion::V2, id, &data[HEADER_LEN..checksum_at]) {
        Ok(message) => Body::Known(message),
        Err(error) => Body::Undecodable(error.to_string()),
    };
    Step::Deliver(total, body)
}

fn message_id(frame: &[u8]) -> u32 {
    u32::from(frame[7]) | u32::from(frame[8]) << 8 | u32::from(frame[9]) << 16
}

/// MAVLink 2 encoder with its own sequence counter.
#[derive(Debug)]
pub struct Encoder {
    system_id: u8,
    component_id: u8,
    sequence: u8,
}

impl Encoder {
    pub fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            system_id,
            component_id,
            sequence: 0,
        }
    }

    pub fn encode(&mut self, message: &Message) -> Vec<u8> {
        let header = MavHeader {
            system_id: u32::from(self.system_id),
            component_id: self.component_id,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        let mut raw = mavlink::MAVLinkV2MessageRaw::new();
        raw.serialize_message(header, message);
        raw.raw_bytes().to_vec()
    }
}

#[cfg(test)]
#[path = "frame_tests.rs"]
mod tests;
