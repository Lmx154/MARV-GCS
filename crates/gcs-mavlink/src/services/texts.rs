//! Text messages from the vehicle (STATUSTEXT).

use std::collections::{HashMap, VecDeque};
use std::time::Duration;

use super::{Ctx, Service};
use crate::dialect::MavSeverity;
use crate::{Body, Frame, Message};

/// The most entries the log keeps; the oldest go first.
pub const MAX_TEXTS: usize = 1000;

/// MAV_SEVERITY, most severe first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Emergency,
    Alert,
    Critical,
    Error,
    Warning,
    Notice,
    Info,
    Debug,
}

impl Severity {
    pub fn name(self) -> &'static str {
        match self {
            Severity::Emergency => "EMERGENCY",
            Severity::Alert => "ALERT",
            Severity::Critical => "CRITICAL",
            Severity::Error => "ERROR",
            Severity::Warning => "WARNING",
            Severity::Notice => "NOTICE",
            Severity::Info => "INFO",
            Severity::Debug => "DEBUG",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct TextEntry {
    /// Session time it arrived.
    pub time: Duration,
    pub severity: Severity,
    pub text: String,
    pub system_id: u8,
    pub component_id: u8,
}

/// Oldest first.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TextLog {
    pub entries: VecDeque<TextEntry>,
}

/// The chunks of one long text seen so far from one sender.
#[derive(Debug)]
struct Chain {
    id: u16,
    chunk_seq: u8,
    /// The bytes of all chunks, so a character split between chunks decodes whole.
    bytes: Vec<u8>,
}

/// The service that fills the log.
#[derive(Debug, Default)]
pub struct Texts {
    /// Per sender, the long text its newest entry belongs to.
    chains: HashMap<(u8, u8), Chain>,
}

impl Texts {
    pub fn new() -> Self {
        Self::default()
    }
}

fn severity(value: MavSeverity) -> Severity {
    match value {
        MavSeverity::MAV_SEVERITY_EMERGENCY => Severity::Emergency,
        MavSeverity::MAV_SEVERITY_ALERT => Severity::Alert,
        MavSeverity::MAV_SEVERITY_CRITICAL => Severity::Critical,
        MavSeverity::MAV_SEVERITY_ERROR => Severity::Error,
        MavSeverity::MAV_SEVERITY_WARNING => Severity::Warning,
        MavSeverity::MAV_SEVERITY_NOTICE => Severity::Notice,
        MavSeverity::MAV_SEVERITY_INFO => Severity::Info,
        MavSeverity::MAV_SEVERITY_DEBUG => Severity::Debug,
    }
}

impl Service for Texts {
    fn on_frame(&mut self, frame: &Frame, cx: &mut Ctx) {
        let Body::Known(Message::STATUSTEXT(data)) = &frame.body else {
            return;
        };
        let sender = (frame.system_id, frame.component_id);
        let bytes = data
            .text
            .iter()
            .position(|&b| b == 0)
            .map_or(&data.text[..], |end| &data.text[..end]);
        let log = &mut cx.state.texts.entries;

        if data.id != 0
            && let Some(chain) = self.chains.get_mut(&sender)
            && chain.id == data.id
            && chain.chunk_seq.checked_add(1) == Some(data.chunk_seq)
            && let Some(entry) = log
                .iter_mut()
                .rev()
                .find(|e| (e.system_id, e.component_id) == sender)
        {
            chain.chunk_seq = data.chunk_seq;
            chain.bytes.extend_from_slice(bytes);
            entry.text = String::from_utf8_lossy(&chain.bytes).into_owned();
            cx.changed = true;
            return;
        }

        if data.id == 0 {
            self.chains.remove(&sender);
        } else {
            self.chains.insert(
                sender,
                Chain {
                    id: data.id,
                    chunk_seq: data.chunk_seq,
                    bytes: bytes.to_vec(),
                },
            );
        }
        if log.len() >= MAX_TEXTS {
            log.pop_front();
        }
        log.push_back(TextEntry {
            time: cx.now,
            severity: severity(data.severity),
            text: String::from_utf8_lossy(bytes).into_owned(),
            system_id: frame.system_id,
            component_id: frame.component_id,
        });
        cx.changed = true;
    }
}

#[cfg(test)]
#[path = "texts_tests.rs"]
mod tests;
