//! Message store: every message that arrives, its rate, its decoded fields and the history of every
//! numeric field. Nothing here is specific to one message, except the table of instance fields.

use std::collections::{BTreeMap, VecDeque};
use std::time::Duration;

use mavlink::Message as _;
use serde_json::Value;

use super::{Ctx, Service};
use crate::{Body, Frame, Message};

/// How much history a series keeps.
pub const HISTORY: Duration = Duration::from_secs(60);
/// The most points one series keeps.
pub const MAX_POINTS: usize = 30_000;
/// Messages that carry several instances, and the field that says which one (link contract).
/// A message that is not listed here has instance 0.
pub const INSTANCE_FIELDS: &[(&str, &str)] = &[("HIGHRES_IMU", "id")];

/// One kind of message from one sender.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MessageKey {
    pub system_id: u8,
    pub component_id: u8,
    pub message_id: u32,
    pub instance: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MessageEntry {
    /// `HIGHRES_IMU`; `UNKNOWN_42000` for an id the dialect does not have.
    pub name: String,
    /// The name, with the instance when the message has one: `HIGHRES_IMU[2]`.
    pub label: String,
    pub count: u64,
    pub rate_hz: f32,
    /// Session time of the latest one.
    pub last_seen: Duration,
    /// Decoded fields of the latest one, in message order; empty when it could not be decoded.
    pub fields: Vec<Field>,
}

impl MessageEntry {
    pub fn number(&self, field: &str) -> Option<f64> {
        self.fields.iter().find_map(|f| match &f.value {
            FieldValue::Number(n) if f.name == field => Some(*n),
            _ => None,
        })
    }

    pub fn text(&self, field: &str) -> Option<&str> {
        self.fields.iter().find_map(|f| match &f.value {
            FieldValue::Text(t) if f.name == field => Some(t.as_str()),
            _ => None,
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub name: String,
    pub value: FieldValue,
}

#[derive(Debug, Clone, PartialEq)]
pub enum FieldValue {
    Number(f64),
    Text(String),
}

/// One numeric field of one kind of message from one sender.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SeriesKey {
    pub system_id: u8,
    pub component_id: u8,
    pub message: String,
    pub instance: u8,
    pub field: String,
}

/// `[session time in seconds, value]`, oldest first.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Series {
    pub points: VecDeque<[f64; 2]>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct MessageStore {
    pub entries: BTreeMap<MessageKey, MessageEntry>,
    /// History of every numeric field of every decoded message.
    pub series: BTreeMap<SeriesKey, Series>,
    /// Frames the senders numbered but that never arrived (gaps in the sequence numbers).
    pub lost_frames: u64,
}

impl MessageStore {
    /// The latest message of that name and instance, from the first sender that has one.
    pub fn entry(&self, message: &str, instance: u8) -> Option<&MessageEntry> {
        self.entries
            .iter()
            .find(|(key, entry)| key.instance == instance && entry.name == message)
            .map(|(_, entry)| entry)
    }

    /// The history of one field, from the first sender that has it.
    pub fn series(&self, message: &str, instance: u8, field: &str) -> Option<&Series> {
        self.series
            .iter()
            .find(|(key, _)| {
                key.instance == instance && key.message == message && key.field == field
            })
            .map(|(_, series)| series)
    }
}

/// Arrays of numbers up to this length become one field per element.
const ARRAY_SPLIT: usize = 8;
/// How far back the rate looks.
const RATE_WINDOW: Duration = Duration::from_secs(2);
/// A stream that has been silent for this long reads 0 Hz (or three of its periods, if longer).
const RATE_STALE: Duration = Duration::from_secs(2);

/// The fields of a decoded message, in the message's own order (through its serde form).
///
/// The `type` tag is skipped; numbers, booleans and `null` (NaN or infinity) become `Number`;
/// strings, enums (`{"type": "NAME"}`) and bit flags (`"A | B"`) become `Text`; a short array of
/// numbers becomes one `Number` per element named `name[i]`; a longer array becomes one `Text`.
pub fn fields_of(message: &Message) -> Vec<Field> {
    let Ok(serde_json::Value::Object(map)) = serde_json::to_value(message) else {
        return Vec::new();
    };
    let mut fields = Vec::with_capacity(map.len());
    for (name, value) in map {
        if name != "type" {
            push_field(&mut fields, name, value);
        }
    }
    fields
}

fn push_field(fields: &mut Vec<Field>, name: String, value: Value) {
    let number = |value: &Value| match value {
        Value::Number(n) => Some(n.as_f64().unwrap_or(f64::NAN)),
        Value::Null => Some(f64::NAN),
        _ => None,
    };
    let value = match value {
        Value::Bool(b) => FieldValue::Number(f64::from(u8::from(b))),
        Value::String(text) => FieldValue::Text(text),
        Value::Array(items)
            if (1..=ARRAY_SPLIT).contains(&items.len())
                && items.iter().all(|item| number(item).is_some()) =>
        {
            for (index, item) in items.iter().enumerate() {
                fields.push(Field {
                    name: format!("{name}[{index}]"),
                    value: FieldValue::Number(number(item).unwrap_or(f64::NAN)),
                });
            }
            return;
        }
        Value::Array(items) if items.len() > ARRAY_SPLIT => {
            let head: Vec<String> = items
                .iter()
                .take(ARRAY_SPLIT)
                .map(Value::to_string)
                .collect();
            FieldValue::Text(format!("{} ... ({} values)", head.join(", "), items.len()))
        }
        Value::Object(map) if map.len() == 1 => match map.get("type") {
            Some(Value::String(variant)) => FieldValue::Text(variant.clone()),
            _ => FieldValue::Text(Value::Object(map).to_string()),
        },
        other => match number(&other) {
            Some(n) => FieldValue::Number(n),
            None => FieldValue::Text(other.to_string()),
        },
    };
    fields.push(Field { name, value });
}

/// What the service remembers about one kind of message from one sender.
#[derive(Debug, Default)]
struct Tracker {
    /// Arrival times inside the rate window (at least the last two, however old).
    times: VecDeque<Duration>,
    /// Series keys of the numeric fields, in field order, so a frame allocates no key.
    series_keys: Vec<SeriesKey>,
}

impl Tracker {
    /// Seconds between messages in the window; `None` with fewer than two.
    fn period(&self) -> Option<Duration> {
        let (first, last) = (self.times.front()?, self.times.back()?);
        let gaps = u32::try_from(self.times.len() - 1)
            .ok()
            .filter(|&n| n > 0)?;
        Some((*last - *first) / gaps)
    }

    fn is_stale(&self, now: Duration) -> bool {
        let Some(&last) = self.times.back() else {
            return false;
        };
        let limit = self
            .period()
            .map_or(RATE_STALE, |p| (p * 3).max(RATE_STALE));
        now.saturating_sub(last) >= limit
    }

    /// Record an arrival and return the rate it implies.
    fn arrived(&mut self, now: Duration) -> f32 {
        if self.is_stale(now) {
            self.times.clear();
        }
        self.times.push_back(now);
        while self.times.len() > 2 && self.times[0] + RATE_WINDOW < now {
            self.times.pop_front();
        }
        self.period()
            .filter(|period| !period.is_zero())
            .map_or(0.0, |period| (1.0 / period.as_secs_f64()) as f32)
    }
}

/// The service that fills the store.
#[derive(Debug, Default)]
pub struct Messages {
    trackers: BTreeMap<MessageKey, Tracker>,
    /// Last sequence number seen from each (system id, component id).
    sequences: BTreeMap<(u8, u8), u8>,
}

impl Messages {
    pub fn new() -> Self {
        Self::default()
    }

    fn count_lost(&mut self, frame: &Frame, store: &mut MessageStore) {
        let sender = (frame.system_id, frame.component_id);
        if let Some(previous) = self.sequences.insert(sender, frame.sequence) {
            let step = frame.sequence.wrapping_sub(previous);
            if step > 1 {
                store.lost_frames += u64::from(step - 1);
            }
        }
    }
}

fn append_point(series: &mut Series, time: f64, value: f64) {
    series.points.push_back([time, value]);
    let oldest = time - HISTORY.as_secs_f64();
    while series.points.front().is_some_and(|point| point[0] < oldest) {
        series.points.pop_front();
    }
    while series.points.len() > MAX_POINTS {
        series.points.pop_front();
    }
}

impl Service for Messages {
    fn on_frame(&mut self, frame: &Frame, cx: &mut Ctx) {
        let store = &mut cx.state.messages;
        self.count_lost(frame, store);

        let (name, fields) = match &frame.body {
            Body::Known(message) => (message.message_name(), fields_of(message)),
            Body::Undecodable(_) => (
                Message::default_message_from_id(frame.message_id)
                    .map_or("", |message| message.message_name()),
                Vec::new(),
            ),
            Body::Unknown => ("", Vec::new()),
        };
        let name = if name.is_empty() {
            format!("UNKNOWN_{}", frame.message_id)
        } else {
            name.to_owned()
        };
        let instance_field = INSTANCE_FIELDS
            .iter()
            .find(|(message, _)| *message == name)
            .map(|(_, field)| *field);
        let instance = instance_field
            .and_then(|wanted| {
                fields.iter().find_map(|f| match &f.value {
                    FieldValue::Number(n) if f.name == wanted => Some(*n as u8),
                    _ => None,
                })
            })
            .unwrap_or(0);
        let key = MessageKey {
            system_id: frame.system_id,
            component_id: frame.component_id,
            message_id: frame.message_id,
            instance,
        };

        let tracker = self.trackers.entry(key.clone()).or_default();
        let rate_hz = tracker.arrived(cx.now);

        if matches!(frame.body, Body::Known(_)) {
            let time = cx.now.as_secs_f64();
            let mut slot = 0;
            for field in &fields {
                let FieldValue::Number(value) = field.value else {
                    continue;
                };
                if tracker
                    .series_keys
                    .get(slot)
                    .is_none_or(|k| k.field != field.name)
                {
                    let series_key = SeriesKey {
                        system_id: key.system_id,
                        component_id: key.component_id,
                        message: name.clone(),
                        instance,
                        field: field.name.clone(),
                    };
                    if slot < tracker.series_keys.len() {
                        tracker.series_keys[slot] = series_key;
                    } else {
                        tracker.series_keys.push(series_key);
                    }
                }
                if value.is_finite() {
                    let series_key = &tracker.series_keys[slot];
                    if !store.series.contains_key(series_key) {
                        store.series.insert(series_key.clone(), Series::default());
                    }
                    if let Some(series) = store.series.get_mut(series_key) {
                        append_point(series, time, value);
                    }
                }
                slot += 1;
            }
        }

        let entry = store.entries.entry(key).or_insert_with(|| MessageEntry {
            label: if instance_field.is_some() {
                format!("{name}[{instance}]")
            } else {
                name.clone()
            },
            name,
            count: 0,
            rate_hz: 0.0,
            last_seen: Duration::ZERO,
            fields: Vec::new(),
        });
        entry.count += 1;
        entry.rate_hz = rate_hz;
        entry.last_seen = cx.now;
        entry.fields = fields;
        cx.changed = true;
    }

    fn on_tick(&mut self, cx: &mut Ctx) {
        for (key, tracker) in &self.trackers {
            if !tracker.is_stale(cx.now) {
                continue;
            }
            if let Some(entry) = cx.state.messages.entries.get_mut(key)
                && entry.rate_hz != 0.0
            {
                entry.rate_hz = 0.0;
                cx.changed = true;
            }
        }
    }
}

#[cfg(test)]
#[path = "messages_tests.rs"]
mod tests;
