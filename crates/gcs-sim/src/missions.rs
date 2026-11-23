//! The vehicle's side of the mission service (`docs/link-contract.md`, section 17), written from that text
//! alone: the stored list, the staging copy, the refusals and their order, the plan id, the timers and the reply
//! slot. Sans-IO: virtual time and decoded messages in, messages to hand to the slot out.
//!
//! The six messages are read and written by hand from their bytes (fields in wire order, with the extension
//! fields last, trailing zero bytes cut): a station may send a `frame`, `command` or `mission_type` that is no
//! value of the dialect's enums, and the contract refuses it with a result instead of dropping the frame.
//!
//! Readings where the text is open (all in the test file `tests/missions.rs` too):
//! - The slot sends on a fixed 5 ms grid of the simulator's clock, at the first multiple of 5 ms after the time
//!   the message was handed to it (example 1: handed at t 0, sent at t 40).
//! - A retry's 1500 ms run from the time the message was handed to the slot (rule of the slot, example 1 steps 4
//!   to 6), not from its send.
//! - A re-request because of a wrong item restarts the 1500 ms timer and keeps the retry count.
//! - The 9000 ms of a download run from the last mission message of the owner (the MISSION_REQUEST_LIST first).
//! - There is one link, so an owner is (system id, component id).

/// Waypoints the vehicle holds.
pub(crate) const CAPACITY: u16 = 16;
/// A retry of the vehicle's MISSION_REQUEST_INT, and the wait after the last send.
const RETRY_MS: u64 = 1500;
/// Retries after the first send (6 sends in all).
const RETRIES: u32 = 5;
/// A download ends after this long without a mission message from its owner.
const DOWNLOAD_IDLE_MS: u64 = 9000;
/// One mission message per this many milliseconds.
pub(crate) const SLOT_MS: u64 = 5;

pub(crate) const ID_REQUEST_LIST: u32 = 43;
pub(crate) const ID_COUNT: u32 = 44;
pub(crate) const ID_CLEAR_ALL: u32 = 45;
pub(crate) const ID_ACK: u32 = 47;
pub(crate) const ID_REQUEST_INT: u32 = 51;
pub(crate) const ID_ITEM_INT: u32 = 73;

const MAV_CMD_NAV_WAYPOINT: u16 = 16;
const FRAME_GLOBAL_RELATIVE_ALT: u8 = 3;
const FRAME_GLOBAL_RELATIVE_ALT_INT: u8 = 6;
const MAX_LAT: i32 = 900_000_000;
const MAX_LON: i32 = 1_800_000_000;

/// `MAV_MISSION_RESULT` values the vehicle sends.
pub(crate) mod result {
    pub const ACCEPTED: u8 = 0;
    pub const ERROR: u8 = 1;
    pub const UNSUPPORTED_FRAME: u8 = 2;
    pub const UNSUPPORTED: u8 = 3;
    pub const NO_SPACE: u8 = 4;
    pub const INVALID_PARAM5_X: u8 = 10;
    pub const INVALID_PARAM6_Y: u8 = 11;
    pub const INVALID_SEQUENCE: u8 = 13;
    pub const DENIED: u8 = 14;
    pub const OPERATION_CANCELLED: u8 = 15;
}

/// (id, CRC_EXTRA, payload length with extensions) of the six messages.
const MESSAGES: [(u32, u8, usize); 6] = [
    (ID_REQUEST_LIST, 132, 3),
    (ID_COUNT, 221, 9),
    (ID_CLEAR_ALL, 232, 3),
    (ID_ACK, 153, 8),
    (ID_REQUEST_INT, 196, 5),
    (ID_ITEM_INT, 38, 38),
];

/// CRC_EXTRA of a mission message.
pub(crate) fn crc_extra(id: u32) -> Option<u8> {
    MESSAGES.iter().find(|m| m.0 == id).map(|m| m.1)
}

/// (system, component) of a sender or a destination.
pub(crate) type Id = (u8, u8);

/// A waypoint item as the station sends it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ItemIn {
    pub seq: u16,
    pub command: u16,
    pub frame: u8,
    pub x: i32,
    pub y: i32,
    pub z_bits: u32,
    pub mission_type: u8,
}

/// A mission message from the station, as far as the vehicle looks at it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum In {
    RequestList {
        mission_type: u8,
    },
    Count {
        count: u16,
        mission_type: u8,
    },
    ClearAll {
        mission_type: u8,
    },
    /// Every `type` is treated alike.
    Ack,
    RequestInt {
        seq: u16,
        mission_type: u8,
    },
    Item(ItemIn),
}

fn u16_at(p: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([p[at], p[at + 1]])
}

fn u32_at(p: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([p[at], p[at + 1], p[at + 2], p[at + 3]])
}

/// Reads a mission message of id `id` from its (possibly truncated) payload: the target system and component
/// and the message. `None` for any other id. A missing byte reads as 0.
pub(crate) fn parse(id: u32, payload: &[u8]) -> Option<(Id, In)> {
    let len = MESSAGES.iter().find(|m| m.0 == id)?.2;
    let mut p = vec![0u8; len];
    let n = payload.len().min(len);
    p[..n].copy_from_slice(&payload[..n]);
    Some(match id {
        ID_REQUEST_LIST => ((p[0], p[1]), In::RequestList { mission_type: p[2] }),
        ID_COUNT => (
            (p[2], p[3]),
            In::Count {
                count: u16_at(&p, 0),
                mission_type: p[4],
            },
        ),
        ID_CLEAR_ALL => ((p[0], p[1]), In::ClearAll { mission_type: p[2] }),
        ID_ACK => ((p[0], p[1]), In::Ack),
        ID_REQUEST_INT => (
            (p[2], p[3]),
            In::RequestInt {
                seq: u16_at(&p, 0),
                mission_type: p[4],
            },
        ),
        _ => (
            (p[32], p[33]),
            In::Item(ItemIn {
                x: u32_at(&p, 16) as i32,
                y: u32_at(&p, 20) as i32,
                z_bits: u32_at(&p, 24),
                seq: u16_at(&p, 28),
                command: u16_at(&p, 30),
                frame: p[34],
                mission_type: p[37],
            }),
        ),
    })
}

/// A message the vehicle sends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Out {
    Count {
        count: u16,
        opaque_id: u32,
    },
    Ack {
        result: u8,
        mission_type: u8,
        opaque_id: u32,
    },
    RequestInt {
        seq: u16,
    },
    Item {
        seq: u16,
        x: i32,
        y: i32,
        z_bits: u32,
    },
}

impl Out {
    /// (message id, payload) of this message addressed to `to`; the payload is not yet cut.
    pub fn encode(&self, to: Id) -> (u32, Vec<u8>) {
        match *self {
            Self::Count { count, opaque_id } => {
                let mut p = Vec::with_capacity(9);
                p.extend_from_slice(&count.to_le_bytes());
                p.extend_from_slice(&[to.0, to.1, 0]);
                p.extend_from_slice(&opaque_id.to_le_bytes());
                (ID_COUNT, p)
            }
            Self::Ack {
                result,
                mission_type,
                opaque_id,
            } => {
                let mut p = vec![to.0, to.1, result, mission_type];
                p.extend_from_slice(&opaque_id.to_le_bytes());
                (ID_ACK, p)
            }
            Self::RequestInt { seq } => {
                let mut p = Vec::with_capacity(5);
                p.extend_from_slice(&seq.to_le_bytes());
                p.extend_from_slice(&[to.0, to.1, 0]);
                (ID_REQUEST_INT, p)
            }
            Self::Item { seq, x, y, z_bits } => {
                let mut p = Vec::with_capacity(38);
                p.extend_from_slice(&[0u8; 16]);
                p.extend_from_slice(&x.to_le_bytes());
                p.extend_from_slice(&y.to_le_bytes());
                p.extend_from_slice(&z_bits.to_le_bytes());
                p.extend_from_slice(&seq.to_le_bytes());
                p.extend_from_slice(&MAV_CMD_NAV_WAYPOINT.to_le_bytes());
                p.extend_from_slice(&[to.0, to.1, FRAME_GLOBAL_RELATIVE_ALT, 0, 1, 0]);
                (ID_ITEM_INT, p)
            }
        }
    }
}

/// A message and where it goes.
pub(crate) type Reply = (Id, Out);

/// What the vehicle keeps of one waypoint: `x`, `y` and the 4 bytes of `z`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Waypoint {
    x: i32,
    y: i32,
    z_bits: u32,
}

/// IEEE CRC-32: reflected polynomial 0xEDB88320, initial value and final XOR 0xFFFFFFFF.
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in bytes {
        crc ^= u32::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// The plan id of a list: the CRC-32 of 12 bytes per waypoint (`x`, `y`, `z` bits, little-endian); 0 for the
/// empty list; 1 when a non-empty list comes out 0.
fn plan_id(list: &[Waypoint]) -> u32 {
    if list.is_empty() {
        return 0;
    }
    let mut bytes = Vec::with_capacity(12 * list.len());
    for w in list {
        bytes.extend_from_slice(&w.x.to_le_bytes());
        bytes.extend_from_slice(&w.y.to_le_bytes());
        bytes.extend_from_slice(&w.z_bits.to_le_bytes());
    }
    match crc32(&bytes) {
        0 => 1,
        id => id,
    }
}

enum Kind {
    Upload {
        count: u16,
        staging: Vec<Waypoint>,
        /// The `seq` asked for and expected.
        expecting: u16,
    },
    Download,
}

/// The one transaction that runs.
struct Txn {
    owner: Id,
    kind: Kind,
    /// When its timer runs out.
    deadline_ms: u64,
    /// Retries made of the request of an upload.
    retries: u32,
}

/// The last upload that ended at an item: it lets a lost final ACK be answered again.
struct LastUpload {
    owner: Id,
    seq: u16,
    ack: Out,
}

/// The stored list, the transaction and the memory of the last upload.
#[derive(Default)]
pub(crate) struct Missions {
    list: Vec<Waypoint>,
    plan_id: u32,
    txn: Option<Txn>,
    last: Option<LastUpload>,
}

fn ack(to: Id, result: u8, mission_type: u8, opaque_id: u32) -> Reply {
    (
        to,
        Out::Ack {
            result,
            mission_type,
            opaque_id,
        },
    )
}

impl Missions {
    fn clear(&mut self) {
        self.list.clear();
        self.plan_id = 0;
    }

    /// The time the running transaction's timer runs out.
    pub fn due_ms(&self) -> Option<u64> {
        self.txn.as_ref().map(|t| t.deadline_ms)
    }

    /// The timer ran out: a retry of the request, or giving up (an upload after the sixth send, a download
    /// after 9000 ms of silence): MISSION_ACK OPERATION_CANCELLED to the owner, the list unchanged.
    pub fn fire(&mut self) -> Vec<Reply> {
        let Some(txn) = self.txn.as_mut() else {
            return Vec::new();
        };
        if let Kind::Upload { expecting, .. } = &txn.kind
            && txn.retries < RETRIES
        {
            txn.retries += 1;
            txn.deadline_ms += RETRY_MS;
            return vec![(txn.owner, Out::RequestInt { seq: *expecting })];
        }
        let owner = txn.owner;
        self.txn = None;
        vec![ack(owner, result::OPERATION_CANCELLED, 0, 0)]
    }

    /// The owner's link went down: the transaction ends silently, staging discarded, and the last upload is
    /// forgotten. The stored list stays.
    pub fn link_down(&mut self) {
        self.txn = None;
        self.last = None;
    }

    /// A mission message from `from`, addressed to the vehicle, at time `now_ms`.
    pub fn handle(&mut self, now_ms: u64, from: Id, msg: In, armed: bool) -> Vec<Reply> {
        let starts = matches!(
            msg,
            In::Count { .. } | In::RequestList { .. } | In::ClearAll { .. }
        );
        if let Some(txn) = &self.txn {
            if txn.owner != from {
                let mission_type = match msg {
                    In::Ack => return Vec::new(),
                    In::RequestList { mission_type }
                    | In::Count { mission_type, .. }
                    | In::ClearAll { mission_type }
                    | In::RequestInt { mission_type, .. } => mission_type,
                    In::Item(item) => item.mission_type,
                };
                return vec![ack(from, result::DENIED, mission_type, 0)];
            }
            if starts {
                self.txn = None;
            }
        }
        if starts {
            self.last = None;
        }
        if self.txn.is_some() {
            self.in_transaction(now_ms, from, msg, armed)
        } else {
            self.idle(now_ms, from, msg, armed)
        }
    }

    /// Rule 3: no transaction runs.
    fn idle(&mut self, now_ms: u64, from: Id, msg: In, armed: bool) -> Vec<Reply> {
        match msg {
            In::Count {
                count,
                mission_type,
            } => {
                if mission_type != 0 {
                    return vec![ack(from, result::UNSUPPORTED, mission_type, 0)];
                }
                if armed {
                    return vec![ack(from, result::ERROR, mission_type, 0)];
                }
                if count > CAPACITY {
                    return vec![ack(from, result::NO_SPACE, mission_type, 0)];
                }
                if count == 0 {
                    self.clear();
                    return vec![ack(from, result::ACCEPTED, 0, 0)];
                }
                self.txn = Some(Txn {
                    owner: from,
                    kind: Kind::Upload {
                        count,
                        staging: Vec::new(),
                        expecting: 0,
                    },
                    deadline_ms: now_ms + RETRY_MS,
                    retries: 0,
                });
                vec![(from, Out::RequestInt { seq: 0 })]
            }
            In::ClearAll { mission_type } => {
                if mission_type != 0 {
                    return vec![ack(from, result::UNSUPPORTED, mission_type, 0)];
                }
                if armed {
                    return vec![ack(from, result::ERROR, mission_type, 0)];
                }
                self.clear();
                vec![ack(from, result::ACCEPTED, 0, 0)]
            }
            In::RequestList { mission_type } => {
                if mission_type != 0 {
                    return vec![ack(from, result::UNSUPPORTED, mission_type, 0)];
                }
                self.txn = Some(Txn {
                    owner: from,
                    kind: Kind::Download,
                    deadline_ms: now_ms + DOWNLOAD_IDLE_MS,
                    retries: 0,
                });
                vec![(
                    from,
                    Out::Count {
                        count: self.list.len() as u16,
                        opaque_id: self.plan_id,
                    },
                )]
            }
            In::Item(item) => match &self.last {
                Some(last) if last.owner == from && last.seq == item.seq => {
                    vec![(from, last.ack)]
                }
                _ => vec![ack(from, result::ERROR, item.mission_type, 0)],
            },
            In::RequestInt { mission_type, .. } => {
                vec![ack(from, result::ERROR, mission_type, 0)]
            }
            In::Ack => Vec::new(),
        }
    }

    /// Rules 4 and 5: the owner's upload or download, for a message that is not a start message.
    fn in_transaction(&mut self, now_ms: u64, from: Id, msg: In, armed: bool) -> Vec<Reply> {
        let Some(txn) = self.txn.as_mut() else {
            return Vec::new();
        };
        match &mut txn.kind {
            Kind::Upload {
                count,
                staging,
                expecting,
            } => match msg {
                In::Item(item) => {
                    let k = *expecting;
                    if item.seq != k {
                        txn.deadline_ms = now_ms + RETRY_MS;
                        return vec![(from, Out::RequestInt { seq: k })];
                    }
                    if let Some(refusal) = refuse_item(&item, armed) {
                        let reply = ack(from, refusal, item.mission_type, 0);
                        self.last = Some(LastUpload {
                            owner: from,
                            seq: k,
                            ack: reply.1,
                        });
                        self.txn = None;
                        return vec![reply];
                    }
                    staging.push(Waypoint {
                        x: item.x,
                        y: item.y,
                        z_bits: item.z_bits,
                    });
                    if k + 1 < *count {
                        *expecting = k + 1;
                        txn.deadline_ms = now_ms + RETRY_MS;
                        txn.retries = 0;
                        return vec![(from, Out::RequestInt { seq: k + 1 })];
                    }
                    self.list = std::mem::take(staging);
                    self.plan_id = plan_id(&self.list);
                    let reply = ack(from, result::ACCEPTED, 0, self.plan_id);
                    self.last = Some(LastUpload {
                        owner: from,
                        seq: k,
                        ack: reply.1,
                    });
                    self.txn = None;
                    vec![reply]
                }
                In::Ack => {
                    self.txn = None;
                    Vec::new()
                }
                _ => Vec::new(),
            },
            Kind::Download => {
                txn.deadline_ms = now_ms + DOWNLOAD_IDLE_MS;
                match msg {
                    In::RequestInt { seq, mission_type } => {
                        if mission_type != 0 {
                            self.txn = None;
                            return vec![ack(from, result::UNSUPPORTED, mission_type, 0)];
                        }
                        match self.list.get(usize::from(seq)) {
                            Some(w) => vec![(
                                from,
                                Out::Item {
                                    seq,
                                    x: w.x,
                                    y: w.y,
                                    z_bits: w.z_bits,
                                },
                            )],
                            None => {
                                self.txn = None;
                                vec![ack(from, result::INVALID_SEQUENCE, mission_type, 0)]
                            }
                        }
                    }
                    In::Ack => {
                        self.txn = None;
                        Vec::new()
                    }
                    _ => Vec::new(),
                }
            }
        }
    }
}

/// The checks of an item of the expected `seq`, in the contract's order: the result that refuses it, if any.
fn refuse_item(item: &ItemIn, armed: bool) -> Option<u8> {
    if item.mission_type != 0 {
        Some(result::UNSUPPORTED)
    } else if armed {
        Some(result::ERROR)
    } else if item.command != MAV_CMD_NAV_WAYPOINT {
        Some(result::UNSUPPORTED)
    } else if item.frame != FRAME_GLOBAL_RELATIVE_ALT && item.frame != FRAME_GLOBAL_RELATIVE_ALT_INT
    {
        Some(result::UNSUPPORTED_FRAME)
    } else if !(-MAX_LAT..=MAX_LAT).contains(&item.x) {
        Some(result::INVALID_PARAM5_X)
    } else if !(-MAX_LON..=MAX_LON).contains(&item.y) {
        Some(result::INVALID_PARAM6_Y)
    } else {
        None
    }
}

/// The reply slot of the link: one waiting message, sent at the next multiple of [`SLOT_MS`] after it was handed
/// over.
#[derive(Default)]
pub(crate) struct Slot {
    waiting: Option<(Id, Out, u64)>,
}

impl Slot {
    /// Hands a message to the slot at `now_ms`. It replaces the waiting message to the same destination and is
    /// dropped while a message to another destination waits.
    pub fn hand_over(&mut self, now_ms: u64, to: Id, msg: Out) {
        match &mut self.waiting {
            None => self.waiting = Some((to, msg, (now_ms / SLOT_MS + 1) * SLOT_MS)),
            Some((dest, waiting, _)) if *dest == to => *waiting = msg,
            Some(_) => {}
        }
    }

    /// The waiting message when its slot has come by `now_ms`.
    pub fn take_due(&mut self, now_ms: u64) -> Option<Reply> {
        match self.waiting {
            Some((to, msg, at_ms)) if at_ms <= now_ms => {
                self.waiting = None;
                Some((to, msg))
            }
            _ => None,
        }
    }

    pub fn clear(&mut self) {
        self.waiting = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_of_the_check_string() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn an_empty_list_has_plan_id_0() {
        assert_eq!(plan_id(&[]), 0);
    }
}
