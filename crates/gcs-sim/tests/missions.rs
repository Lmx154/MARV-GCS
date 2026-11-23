//! The simulated vehicle's mission service (`docs/link-contract.md`, section 17), driven message by message in
//! virtual time. The frames are built and read by hand from the contract's field tables, with the contract's
//! CRC_EXTRA values, and the plan ids are computed here by a second CRC-32, so these tests do not use the
//! `mavlink` crate's mission messages (a frame may carry a `frame` or `command` that is no enum value).
//!
//! The 5 ms slot sends at the first multiple of 5 ms after the time a message was handed to it (a step at 100
//! ms hands a reply over, the frame comes out of the step at 105 ms).

use gcs_sim::{SimConfig, SimVehicle};
use mavlink::calculate_crc;

const GCS: (u8, u8) = (255, 190);
const OTHER: (u8, u8) = (255, 191);
const VEHICLE: (u8, u8) = (1, 1);

const ACCEPTED: u8 = 0;
const ERROR: u8 = 1;
const UNSUPPORTED_FRAME: u8 = 2;
const UNSUPPORTED: u8 = 3;
const NO_SPACE: u8 = 4;
const INVALID_PARAM5_X: u8 = 10;
const INVALID_PARAM6_Y: u8 = 11;
const INVALID_SEQUENCE: u8 = 13;
const DENIED: u8 = 14;
const CANCELLED: u8 = 15;

/// (id, CRC_EXTRA, length with extensions) from the contract's table.
const MESSAGES: [(u32, u8, usize); 6] = [
    (43, 132, 3),
    (44, 221, 9),
    (45, 232, 3),
    (47, 153, 8),
    (51, 196, 5),
    (73, 38, 38),
];

/// The three points of example 1.
const P0: Pt = Pt {
    x: 100_000_000,
    y: -300_000_000,
    z_bits: 0x4120_0000,
};
const P1: Pt = Pt {
    x: 100_010_000,
    y: -300_000_000,
    z_bits: 0x4148_0000,
};
const P2: Pt = Pt {
    x: 100_010_000,
    y: -299_990_000,
    z_bits: 0x4170_0000,
};
const EXAMPLE_PLAN_ID: u32 = 0x99C3_BD49;

#[derive(Debug, Clone, Copy, PartialEq)]
struct Pt {
    x: i32,
    y: i32,
    z_bits: u32,
}

// ---- the station's frames -------------------------------------------------------------------------------

/// A mission message from the station: the payload at full length, with the target at `tgt`.
#[derive(Clone)]
struct Raw {
    from: (u8, u8),
    id: u32,
    p: Vec<u8>,
    tgt: usize,
}

impl Raw {
    fn from(mut self, from: (u8, u8)) -> Self {
        self.from = from;
        self
    }

    fn to(mut self, to: (u8, u8)) -> Self {
        self.p[self.tgt] = to.0;
        self.p[self.tgt + 1] = to.1;
        self
    }

    /// The MAVLink 2 frame, with the trailing zero bytes of the payload cut (one byte stays).
    fn bytes(&self) -> Vec<u8> {
        let used = self.p.iter().rposition(|&b| b != 0).map_or(1, |i| i + 1);
        let extra = MESSAGES.iter().find(|m| m.0 == self.id).unwrap().1;
        let mut f = vec![0xFD, used as u8, 0, 0, 0, self.from.0, self.from.1];
        f.extend_from_slice(&self.id.to_le_bytes()[..3]);
        f.extend_from_slice(&self.p[..used]);
        let crc = calculate_crc(&f[1..], extra);
        f.extend_from_slice(&crc.to_le_bytes());
        f
    }
}

fn s_count_full(n: u16, mission_type: u8, opaque: u32) -> Raw {
    let mut p = n.to_le_bytes().to_vec();
    p.extend_from_slice(&[1, 1, mission_type]);
    p.extend_from_slice(&opaque.to_le_bytes());
    Raw {
        from: GCS,
        id: 44,
        p,
        tgt: 2,
    }
}

fn s_count(n: u16) -> Raw {
    s_count_full(n, 0, 0)
}

fn s_list_mt(mission_type: u8) -> Raw {
    Raw {
        from: GCS,
        id: 43,
        p: vec![1, 1, mission_type],
        tgt: 0,
    }
}

fn s_list() -> Raw {
    s_list_mt(0)
}

fn s_clear_mt(mission_type: u8) -> Raw {
    Raw {
        from: GCS,
        id: 45,
        p: vec![1, 1, mission_type],
        tgt: 0,
    }
}

fn s_clear() -> Raw {
    s_clear_mt(0)
}

fn s_ack(result: u8) -> Raw {
    Raw {
        from: GCS,
        id: 47,
        p: vec![1, 1, result, 0, 0, 0, 0, 0],
        tgt: 0,
    }
}

fn s_req_mt(seq: u16, mission_type: u8) -> Raw {
    let mut p = seq.to_le_bytes().to_vec();
    p.extend_from_slice(&[1, 1, mission_type]);
    Raw {
        from: GCS,
        id: 51,
        p,
        tgt: 2,
    }
}

fn s_req(seq: u16) -> Raw {
    s_req_mt(seq, 0)
}

/// The fields of a MISSION_ITEM_INT; `new` is what the station sends.
#[derive(Clone, Copy)]
struct Spec {
    seq: u16,
    frame: u8,
    command: u16,
    mission_type: u8,
    pt: Pt,
    current: u8,
    autocontinue: u8,
    params: [f32; 4],
}

impl Spec {
    fn new(seq: u16, pt: Pt) -> Self {
        Self {
            seq,
            frame: 3,
            command: 16,
            mission_type: 0,
            pt,
            current: 0,
            autocontinue: 1,
            params: [0.0; 4],
        }
    }

    fn raw(&self) -> Raw {
        let mut p = Vec::with_capacity(38);
        for v in self.params {
            p.extend_from_slice(&v.to_le_bytes());
        }
        p.extend_from_slice(&self.pt.x.to_le_bytes());
        p.extend_from_slice(&self.pt.y.to_le_bytes());
        p.extend_from_slice(&self.pt.z_bits.to_le_bytes());
        p.extend_from_slice(&self.seq.to_le_bytes());
        p.extend_from_slice(&self.command.to_le_bytes());
        p.extend_from_slice(&[
            1,
            1,
            self.frame,
            self.current,
            self.autocontinue,
            self.mission_type,
        ]);
        Raw {
            from: GCS,
            id: 73,
            p,
            tgt: 32,
        }
    }
}

fn s_item(seq: u16, pt: Pt) -> Raw {
    Spec::new(seq, pt).raw()
}

// ---- the vehicle's frames --------------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
enum Got {
    Count {
        to: (u8, u8),
        count: u16,
        mt: u8,
        opaque: u32,
    },
    Ack {
        to: (u8, u8),
        ty: u8,
        mt: u8,
        opaque: u32,
    },
    Req {
        to: (u8, u8),
        seq: u16,
        mt: u8,
    },
    Item {
        to: (u8, u8),
        seq: u16,
        pt: Pt,
        frame: u8,
        command: u16,
        current: u8,
        autocontinue: u8,
        params: [u32; 4],
        mt: u8,
    },
}

fn v_ack(ty: u8) -> Got {
    Got::Ack {
        to: GCS,
        ty,
        mt: 0,
        opaque: 0,
    }
}

fn v_ack_plan(plan: u32) -> Got {
    Got::Ack {
        to: GCS,
        ty: ACCEPTED,
        mt: 0,
        opaque: plan,
    }
}

fn v_req(seq: u16) -> Got {
    Got::Req {
        to: GCS,
        seq,
        mt: 0,
    }
}

fn v_count(count: u16, opaque: u32) -> Got {
    Got::Count {
        to: GCS,
        count,
        mt: 0,
        opaque,
    }
}

fn v_item(seq: u16, pt: Pt) -> Got {
    Got::Item {
        to: GCS,
        seq,
        pt,
        frame: 3,
        command: 16,
        current: 0,
        autocontinue: 1,
        params: [0; 4],
        mt: 0,
    }
}

/// Every mission frame in `bytes` as (header sequence, message), after checking what the contract fixes about the
/// wire: MAVLink 2, sent by the vehicle (1, 1), CRC_EXTRA, no trailing zero byte (but one), at most the full length.
fn parse_out(bytes: &[u8]) -> Vec<(u8, Got)> {
    let mut out = Vec::new();
    let mut at = 0;
    while at < bytes.len() {
        assert_eq!(bytes[at], 0xFD, "frame start at {at}");
        let len = usize::from(bytes[at + 1]);
        let frame = &bytes[at..at + 12 + len];
        at += 12 + len;
        assert_eq!(frame[2], 0, "incompatibility flags");
        assert_eq!((frame[5], frame[6]), VEHICLE);
        let id = u32::from_le_bytes([frame[7], frame[8], frame[9], 0]);
        let Some(&(_, extra, full)) = MESSAGES.iter().find(|m| m.0 == id) else {
            continue;
        };
        let crc = u16::from_le_bytes([frame[10 + len], frame[11 + len]]);
        assert_eq!(
            calculate_crc(&frame[1..10 + len], extra),
            crc,
            "CRC of {id}"
        );
        assert!(len <= full, "payload of {id} is {len} bytes");
        assert!(len == 1 || frame[9 + len] != 0, "trailing zero in {id}");
        let mut p = vec![0u8; full];
        p[..len].copy_from_slice(&frame[10..10 + len]);
        let u16_at = |i: usize| u16::from_le_bytes([p[i], p[i + 1]]);
        let u32_at = |i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        let got = match id {
            44 => Got::Count {
                to: (p[2], p[3]),
                count: u16_at(0),
                mt: p[4],
                opaque: u32_at(5),
            },
            47 => Got::Ack {
                to: (p[0], p[1]),
                ty: p[2],
                mt: p[3],
                opaque: u32_at(4),
            },
            51 => Got::Req {
                to: (p[2], p[3]),
                seq: u16_at(0),
                mt: p[4],
            },
            73 => Got::Item {
                to: (p[32], p[33]),
                seq: u16_at(28),
                pt: Pt {
                    x: u32_at(16) as i32,
                    y: u32_at(20) as i32,
                    z_bits: u32_at(24),
                },
                frame: p[34],
                command: u16_at(30),
                current: p[35],
                autocontinue: p[36],
                params: [u32_at(0), u32_at(4), u32_at(8), u32_at(12)],
                mt: p[37],
            },
            other => panic!("the vehicle sent message {other}"),
        };
        out.push((frame[4], got));
    }
    out
}

// ---- the rig ---------------------------------------------------------------------------------------------

/// A vehicle stepped every millisecond, and what it sent since the last `take`.
struct Rig {
    v: SimVehicle,
    now: u64,
    seen: Vec<(u64, u8, Got)>,
    /// The payloads of the mission frames, as sent.
    wire: Vec<Vec<u8>>,
}

impl Rig {
    fn new() -> Self {
        Self::with(SimConfig {
            streams: Vec::new(),
            ..SimConfig::default()
        })
    }

    fn with(config: SimConfig) -> Self {
        let mut rig = Self {
            v: SimVehicle::new(config),
            now: 0,
            seen: Vec::new(),
            wire: Vec::new(),
        };
        rig.step(0, &[]);
        rig
    }

    fn step(&mut self, t: u64, input: &[u8]) {
        let bytes = self.v.step(t, input);
        self.now = t;
        // Only mission frames matter; the boot STATUSTEXT and anything else is skipped by id.
        let mut at = 0;
        let mut mission = Vec::new();
        while at < bytes.len() {
            let len = usize::from(bytes[at + 1]);
            let id = u32::from_le_bytes([bytes[at + 7], bytes[at + 8], bytes[at + 9], 0]);
            if MESSAGES.iter().any(|m| m.0 == id) {
                mission.extend_from_slice(&bytes[at..at + 12 + len]);
                self.wire.push(bytes[at + 10..at + 10 + len].to_vec());
            }
            at += 12 + len;
        }
        for (seq, got) in parse_out(&mission) {
            self.seen.push((t, seq, got));
        }
    }

    /// Steps every millisecond up to and including `t`.
    fn run_to(&mut self, t: u64) {
        for at in self.now + 1..=t {
            self.step(at, &[]);
        }
    }

    fn run(&mut self, ms: u64) {
        self.run_to(self.now + ms);
    }

    /// The frame goes in at the step of time `t`.
    fn send_at(&mut self, t: u64, raw: &Raw) {
        assert!(t > self.now, "t {t} is not after {}", self.now);
        self.run_to(t - 1);
        self.step(t, &raw.bytes());
    }

    fn send(&mut self, raw: &Raw) {
        self.send_at(self.now + 1, raw);
    }

    fn take(&mut self) -> Vec<(u64, Got)> {
        std::mem::take(&mut self.seen)
            .into_iter()
            .map(|(t, _, g)| (t, g))
            .collect()
    }

    /// What the vehicle sent after the frame, once the slot has passed.
    fn ask(&mut self, raw: &Raw) -> Vec<Got> {
        self.send(raw);
        self.run(10);
        self.take().into_iter().map(|(_, g)| g).collect()
    }

    fn quiet_for(&mut self, ms: u64) {
        self.run(ms);
        assert_eq!(self.take(), Vec::new());
    }
}

/// The station's upload at its own pace: it answers every request; returns the vehicle's last message.
fn upload(r: &mut Rig, pts: &[Pt]) -> Got {
    r.send(&s_count(pts.len() as u16));
    for _ in 0..=pts.len() + 2 {
        r.run(10);
        for (_, got) in r.take() {
            match got {
                Got::Req { seq, .. } => r.send(&s_item(seq, pts[usize::from(seq)])),
                Got::Ack { .. } => return got,
                other => panic!("{other:?}"),
            }
        }
    }
    panic!("the upload did not end");
}

/// The station's download: the count, the plan id and the items (each checked to be what the vehicle sends: frame
/// 3, command 16, current 0, autocontinue 1, parameters 0.0).
fn download(r: &mut Rig) -> (u16, u32, Vec<Pt>) {
    let first = r.ask(&s_list());
    let [
        Got::Count {
            to: GCS,
            count,
            mt: 0,
            opaque,
        },
    ] = first[..]
    else {
        panic!("{first:?}");
    };
    let mut pts = Vec::new();
    for seq in 0..count {
        let got = r.ask(&s_req(seq));
        let [Got::Item { pt, .. }] = got[..] else {
            panic!("{got:?}");
        };
        assert_eq!(got, [v_item(seq, pt)]);
        pts.push(pt);
    }
    assert_eq!(r.ask(&s_ack(ACCEPTED)), []);
    (count, opaque, pts)
}

fn holds(r: &mut Rig, pts: &[Pt], plan: u32) {
    assert_eq!(download(r), (pts.len() as u16, plan, pts.to_vec()));
}

fn three(r: &mut Rig) {
    assert_eq!(upload(r, &[P0, P1, P2]), v_ack_plan(EXAMPLE_PLAN_ID));
}

// ---- a second CRC-32, for the plan ids ----------------------------------------------------------------------

fn crc_table() -> [u32; 256] {
    std::array::from_fn(|i| {
        (0..8).fold(i as u32, |c, _| {
            if c & 1 != 0 {
                (c >> 1) ^ 0xEDB8_8320
            } else {
                c >> 1
            }
        })
    })
}

/// The register after `bytes`, before the final XOR.
fn crc_reg(bytes: &[u8]) -> u32 {
    let t = crc_table();
    bytes.iter().fold(!0u32, |r, &b| {
        t[((r ^ u32::from(b)) & 0xFF) as usize] ^ (r >> 8)
    })
}

fn plan_of(pts: &[Pt]) -> u32 {
    let mut bytes = Vec::new();
    for p in pts {
        bytes.extend_from_slice(&p.x.to_le_bytes());
        bytes.extend_from_slice(&p.y.to_le_bytes());
        bytes.extend_from_slice(&p.z_bits.to_le_bytes());
    }
    match !crc_reg(&bytes) {
        0 if !pts.is_empty() => 1,
        crc => crc,
    }
}

/// Four bytes that, appended to `prefix`, make the CRC-32 of the whole 0.
fn forge_to_zero(prefix: &[u8]) -> [u8; 4] {
    let t = crc_table();
    let mut index = [0usize; 4];
    let mut want = 0xFFFF_FFFFu32;
    for j in (0..4).rev() {
        let i = (0..256).find(|&i| t[i] >> 24 == want >> 24).unwrap();
        index[j] = i;
        want = (want ^ t[i]) << 8;
    }
    let mut reg = crc_reg(prefix);
    let mut out = [0u8; 4];
    for j in 0..4 {
        out[j] = ((reg ^ index[j] as u32) & 0xFF) as u8;
        reg = t[index[j]] ^ (reg >> 8);
    }
    out
}

// ---- the plan id --------------------------------------------------------------------------------------------

#[test]
fn the_second_crc_agrees_with_the_check_string_and_the_test_vector() {
    assert_eq!(!crc_reg(b"123456789"), 0xCBF4_3926);
    assert_eq!(plan_of(&[P0, P1, P2]), EXAMPLE_PLAN_ID);
    assert_eq!(plan_of(&[]), 0);
}

#[test]
fn three_points_give_plan_id_99c3bd49_on_the_wire_as_49_bd_c3_99() {
    let mut r = Rig::new();
    assert_eq!(upload(&mut r, &[P0, P1, P2]), v_ack_plan(0x99C3_BD49));
    let ack = r.wire.last().unwrap();
    assert_eq!(*ack, [255, 190, 0, 0, 0x49, 0xBD, 0xC3, 0x99]);
    // The same id in every download's MISSION_COUNT.
    assert_eq!(r.ask(&s_list()), [v_count(3, 2_579_742_025)]);
}

#[test]
fn the_empty_list_has_plan_id_0() {
    let mut r = Rig::new();
    assert_eq!(r.ask(&s_list()), [v_count(0, 0)]);
    three(&mut r);
    assert_eq!(r.ask(&s_clear()), [v_ack(ACCEPTED)]);
    assert_eq!(r.ask(&s_list()), [v_count(0, 0)]);
}

#[test]
fn a_non_empty_list_whose_crc_is_zero_has_plan_id_1() {
    let mut prefix = Vec::new();
    prefix.extend_from_slice(&P0.x.to_le_bytes());
    prefix.extend_from_slice(&P0.y.to_le_bytes());
    let z = forge_to_zero(&prefix);
    let pt = Pt {
        z_bits: u32::from_le_bytes(z),
        ..P0
    };
    assert_eq!(!crc_reg(&[prefix, z.to_vec()].concat()), 0);
    let mut r = Rig::new();
    assert_eq!(upload(&mut r, &[pt]), v_ack_plan(1));
    holds(&mut r, &[pt], 1);
}

#[test]
fn sixteen_points_are_accepted_and_come_back_with_their_plan_id() {
    let pts: Vec<Pt> = (0..16)
        .map(|i| Pt {
            x: 100_000_000 + i * 10_000,
            y: -300_000_000 + i * 1_000,
            z_bits: (10.0 + i as f32).to_bits(),
        })
        .collect();
    let mut r = Rig::new();
    let plan = plan_of(&pts);
    assert_eq!(upload(&mut r, &pts), v_ack_plan(plan));
    holds(&mut r, &pts, plan);
}

// ---- the worked examples ---------------------------------------------------------------------------------

#[test]
fn worked_example_1_a_lost_item_then_the_read_back() {
    let mut r = Rig::new();
    // The slot is 5 ms here: a reply handed over at t comes out at the next multiple of 5 ms after t.
    r.send_at(100, &s_count(3));
    r.run_to(106);
    assert_eq!(r.take(), [(105, v_req(0))]);
    r.send_at(107, &s_item(0, P0));
    r.run_to(112);
    assert_eq!(r.take(), [(110, v_req(1))]);
    // Item 1 is lost on the link: the station does not send it. The vehicle's timer (from 107) runs out at 1607.
    r.run_to(1606);
    assert_eq!(r.take(), []);
    r.run_to(1611);
    assert_eq!(r.take(), [(1610, v_req(1))]);
    r.send_at(1612, &s_item(1, P1));
    r.run_to(1616);
    assert_eq!(r.take(), [(1615, v_req(2))]);
    r.send_at(1617, &s_item(2, P2));
    r.run_to(1621);
    assert_eq!(r.take(), [(1620, v_ack_plan(EXAMPLE_PLAN_ID))]);
    // The read-back.
    r.send_at(1622, &s_list());
    r.run_to(1626);
    assert_eq!(r.take(), [(1625, v_count(3, 2_579_742_025))]);
    let mut t = 1627;
    for (seq, pt) in [P0, P1, P2].into_iter().enumerate() {
        r.send_at(t, &s_req(seq as u16));
        r.run_to(t + 4);
        assert_eq!(r.take(), [(t / 5 * 5 + 5, v_item(seq as u16, pt))]);
        t += 5;
    }
    r.send_at(t, &s_ack(ACCEPTED));
    // Nothing more is sent after the download's ACK, not even a timer's message.
    r.quiet_for(10_000);
}

#[test]
fn worked_example_2_a_refused_17_point_upload_leaves_the_list_alone() {
    let mut r = Rig::new();
    three(&mut r);
    r.send(&s_count(17));
    r.run(10);
    let got = r.take();
    assert_eq!(got.len(), 1, "{got:?}");
    assert_eq!(got[0].1, v_ack(NO_SPACE));
    // No item is asked for, even after a retry time; no transaction stays open.
    r.quiet_for(10_000);
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
    let mut other = Rig::new();
    assert_eq!(other.ask(&s_count(65_535)), [v_ack(NO_SPACE)]);
}

// ---- the refusals, each from its own test ---------------------------------------------------------------------

/// A vehicle that holds the three points, then an upload of `n` whose item 0 was accepted: the item `seq` 1 to send
/// is checked in the test.
fn mid_upload(n: u16) -> Rig {
    let mut r = Rig::new();
    three(&mut r);
    assert_eq!(r.ask(&s_count(n)), [v_req(0)]);
    assert_eq!(r.ask(&s_item(0, P1)), [v_req(1)]);
    r
}

/// The item `seq` 1 of `spec` is refused with `ty`, and the stored list is still the three points.
fn item_refused(spec: Spec, ty: u8) {
    let mut r = mid_upload(3);
    assert_eq!(
        r.ask(&spec.raw()),
        [Got::Ack {
            to: GCS,
            ty,
            mt: spec.mission_type,
            opaque: 0
        }]
    );
    // Nothing more is asked for or sent, and the list is as it was.
    r.quiet_for(10_000);
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
}

#[test]
fn an_item_with_mission_type_not_0_is_unsupported() {
    for mt in [1, 2, 255] {
        item_refused(
            Spec {
                mission_type: mt,
                ..Spec::new(1, P1)
            },
            UNSUPPORTED,
        );
    }
}

#[test]
fn an_item_with_command_not_16_is_unsupported() {
    for command in [0, 17, 22, 42_430, 65_535] {
        item_refused(
            Spec {
                command,
                ..Spec::new(1, P1)
            },
            UNSUPPORTED,
        );
    }
}

#[test]
fn an_item_with_a_frame_not_3_or_6_is_unsupported_frame() {
    // 0 and 2 are frames of the dialect, 99 and 255 are none of its values.
    for frame in [0, 1, 2, 4, 5, 7, 99, 255] {
        item_refused(
            Spec {
                frame,
                ..Spec::new(1, P1)
            },
            UNSUPPORTED_FRAME,
        );
    }
}

#[test]
fn a_latitude_out_of_range_is_invalid_param5_x_and_the_limits_are_in() {
    for x in [900_000_001, -900_000_001, i32::MAX, i32::MIN] {
        item_refused(Spec::new(1, Pt { x, ..P1 }), INVALID_PARAM5_X);
    }
    let mut r = Rig::new();
    let limits = [
        Pt {
            x: 900_000_000,
            ..P0
        },
        Pt {
            x: -900_000_000,
            ..P1
        },
    ];
    assert_eq!(upload(&mut r, &limits), v_ack_plan(plan_of(&limits)));
}

#[test]
fn a_longitude_out_of_range_is_invalid_param6_y_and_the_limits_are_in() {
    for y in [1_800_000_001, -1_800_000_001, i32::MAX, i32::MIN] {
        item_refused(Spec::new(1, Pt { y, ..P1 }), INVALID_PARAM6_Y);
    }
    let mut r = Rig::new();
    let limits = [
        Pt {
            y: 1_800_000_000,
            ..P0
        },
        Pt {
            y: -1_800_000_000,
            ..P1
        },
    ];
    assert_eq!(upload(&mut r, &limits), v_ack_plan(plan_of(&limits)));
}

#[test]
fn an_item_while_armed_is_an_error() {
    let mut r = mid_upload(3);
    r.v.set_armed(true);
    assert_eq!(r.ask(&s_item(1, P1)), [v_ack(ERROR)]);
    r.quiet_for(10_000);
    r.v.set_armed(false);
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
}

#[test]
fn the_checks_of_an_item_run_in_the_contracts_order() {
    // Everything wrong at once; each step repairs the check that fired.
    let mut spec = Spec {
        mission_type: 1,
        command: 17,
        frame: 99,
        ..Spec::new(
            1,
            Pt {
                x: i32::MAX,
                y: i32::MAX,
                ..P1
            },
        )
    };
    type Repair<'a> = (&'a dyn Fn(&mut Spec), u8);
    let steps: [Repair; 5] = [
        (&|s: &mut Spec| s.mission_type = 0, UNSUPPORTED),
        (&|_: &mut Spec| (), ERROR),
        (&|s: &mut Spec| s.command = 16, UNSUPPORTED),
        (&|s: &mut Spec| s.frame = 3, UNSUPPORTED_FRAME),
        (&|s: &mut Spec| s.pt.x = P1.x, INVALID_PARAM5_X),
    ];
    for (i, (repair, expect)) in steps.iter().enumerate() {
        let mut r = mid_upload(3);
        // Armed only in step 1: the check after the mission type.
        r.v.set_armed(i == 1);
        let got = r.ask(&spec.raw());
        let Got::Ack { ty, .. } = got[0] else {
            panic!("{got:?}");
        };
        assert_eq!(ty, *expect, "step {i}");
        repair(&mut spec);
    }
    // With x repaired the longitude is the last check.
    let mut r = mid_upload(3);
    let got = r.ask(&spec.raw());
    assert!(
        matches!(
            got[0],
            Got::Ack {
                ty: INVALID_PARAM6_Y,
                ..
            }
        ),
        "{got:?}"
    );
}

#[test]
fn a_count_with_mission_type_not_0_is_unsupported() {
    for mt in [1, 2, 255] {
        let mut r = Rig::new();
        assert_eq!(
            r.ask(&s_count_full(3, mt, 0)),
            [Got::Ack {
                to: GCS,
                ty: UNSUPPORTED,
                mt,
                opaque: 0
            }]
        );
        r.quiet_for(100);
    }
}

#[test]
fn a_count_while_armed_is_an_error_count_0_too() {
    for n in [3, 0, 17] {
        let mut r = Rig::new();
        three(&mut r);
        r.v.set_armed(true);
        assert_eq!(r.ask(&s_count(n)), [v_ack(ERROR)], "count {n}");
        r.v.set_armed(false);
        holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
    }
}

#[test]
fn mission_type_is_checked_before_armed_and_before_the_count() {
    let mut r = Rig::new();
    r.v.set_armed(true);
    assert!(matches!(
        r.ask(&s_count_full(17, 1, 0))[..],
        [Got::Ack {
            ty: UNSUPPORTED,
            mt: 1,
            ..
        }]
    ));
    // Armed is checked before the count.
    assert_eq!(r.ask(&s_count(17)), [v_ack(ERROR)]);
    r.v.set_armed(false);
    assert_eq!(r.ask(&s_count(17)), [v_ack(NO_SPACE)]);
}

#[test]
fn a_count_of_17_or_more_is_no_space() {
    for n in [17, 18, 255, 256, 65_535] {
        let mut r = Rig::new();
        assert_eq!(r.ask(&s_count(n)), [v_ack(NO_SPACE)], "count {n}");
        r.quiet_for(10_000);
    }
}

#[test]
fn a_clear_with_mission_type_not_0_is_unsupported_255_included() {
    for mt in [1, 2, 255] {
        let mut r = Rig::new();
        three(&mut r);
        assert_eq!(
            r.ask(&s_clear_mt(mt)),
            [Got::Ack {
                to: GCS,
                ty: UNSUPPORTED,
                mt,
                opaque: 0
            }]
        );
        holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
    }
}

#[test]
fn a_clear_while_armed_is_an_error_and_unsupported_comes_first() {
    let mut r = Rig::new();
    three(&mut r);
    r.v.set_armed(true);
    assert_eq!(r.ask(&s_clear()), [v_ack(ERROR)]);
    assert!(matches!(
        r.ask(&s_clear_mt(255))[..],
        [Got::Ack {
            ty: UNSUPPORTED,
            mt: 255,
            ..
        }]
    ));
    r.v.set_armed(false);
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
}

#[test]
fn a_clear_empties_the_list_also_when_it_was_empty() {
    let mut r = Rig::new();
    assert_eq!(r.ask(&s_clear()), [v_ack(ACCEPTED)]);
    three(&mut r);
    assert_eq!(r.ask(&s_clear()), [v_ack(ACCEPTED)]);
    holds(&mut r, &[], 0);
    // No transaction stays: another sender can start one.
    assert_eq!(
        r.ask(&s_count(1).from(OTHER)),
        [Got::Req {
            to: OTHER,
            seq: 0,
            mt: 0
        }]
    );
}

#[test]
fn a_request_list_with_mission_type_not_0_is_unsupported() {
    for mt in [1, 255] {
        let mut r = Rig::new();
        assert_eq!(
            r.ask(&s_list_mt(mt)),
            [Got::Ack {
                to: GCS,
                ty: UNSUPPORTED,
                mt,
                opaque: 0
            }]
        );
        // It started nothing.
        assert_eq!(
            r.ask(&s_count(1).from(OTHER)),
            [Got::Req {
                to: OTHER,
                seq: 0,
                mt: 0
            }]
        );
    }
}

#[test]
fn a_download_is_allowed_while_armed() {
    let mut r = Rig::new();
    three(&mut r);
    r.v.set_armed(true);
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
}

#[test]
fn an_item_or_a_request_with_no_transaction_is_an_error_and_an_ack_is_ignored() {
    let mut r = Rig::new();
    assert_eq!(r.ask(&s_item(0, P0)), [v_ack(ERROR)]);
    assert_eq!(r.ask(&s_req(0)), [v_ack(ERROR)]);
    assert_eq!(
        r.ask(&s_req_mt(0, 1)),
        [Got::Ack {
            to: GCS,
            ty: ERROR,
            mt: 1,
            opaque: 0
        }]
    );
    for ty in [ACCEPTED, ERROR, CANCELLED] {
        assert_eq!(r.ask(&s_ack(ty)), []);
    }
}

#[test]
fn a_request_in_a_download_at_or_above_the_count_is_invalid_sequence_and_ends_it() {
    for seq in [3, 4, 65_535] {
        let mut r = Rig::new();
        three(&mut r);
        assert_eq!(r.ask(&s_list()), [v_count(3, EXAMPLE_PLAN_ID)]);
        assert_eq!(r.ask(&s_req(seq)), [v_ack(INVALID_SEQUENCE)]);
        // It ended: no transaction, so a request is an error now.
        assert_eq!(r.ask(&s_req(0)), [v_ack(ERROR)]);
    }
    // Seq 2 is the last one that is served.
    let mut r = Rig::new();
    three(&mut r);
    r.ask(&s_list());
    assert_eq!(r.ask(&s_req(2)), [v_item(2, P2)]);
}

#[test]
fn a_request_in_a_download_with_mission_type_not_0_is_unsupported_and_ends_it() {
    let mut r = Rig::new();
    three(&mut r);
    r.ask(&s_list());
    assert_eq!(
        r.ask(&s_req_mt(0, 1)),
        [Got::Ack {
            to: GCS,
            ty: UNSUPPORTED,
            mt: 1,
            opaque: 0
        }]
    );
    assert_eq!(r.ask(&s_req(0)), [v_ack(ERROR)]);
}

// ---- atomic commit ------------------------------------------------------------------------------------------

#[test]
fn an_upload_that_ends_any_other_way_than_its_last_item_leaves_the_list_alone() {
    // A cancel by the station's ACK (any type), a new start message of the owner, the link going down.
    for ty in [ACCEPTED, CANCELLED, ERROR] {
        let mut r = mid_upload(2);
        assert_eq!(r.ask(&s_ack(ty)), []);
        r.quiet_for(10_000);
        holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
    }
    let mut r = mid_upload(2);
    assert_eq!(r.ask(&s_list()), [v_count(3, EXAMPLE_PLAN_ID)]);
    r.ask(&s_ack(ACCEPTED));
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
    let mut r = mid_upload(2);
    r.v.link_down();
    r.quiet_for(10_000);
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
}

#[test]
fn an_upload_replaces_the_list_in_one_step_with_the_new_plan_id() {
    let mut r = Rig::new();
    three(&mut r);
    let new = [P2, P0];
    assert_eq!(upload(&mut r, &new), v_ack_plan(plan_of(&new)));
    holds(&mut r, &new, plan_of(&new));
}

#[test]
fn a_count_of_0_clears_the_list_answers_at_once_and_leaves_no_transaction() {
    let mut r = Rig::new();
    three(&mut r);
    assert_eq!(r.ask(&s_count(0)), [v_ack(ACCEPTED)]);
    holds(&mut r, &[], 0);
    three(&mut r);
    assert_eq!(r.ask(&s_count(0)), [v_ack(ACCEPTED)]);
    assert_eq!(
        r.ask(&s_count(1).from(OTHER)),
        [Got::Req {
            to: OTHER,
            seq: 0,
            mt: 0
        }]
    );
}

#[test]
fn a_download_returns_exactly_what_was_committed() {
    let nan = Pt {
        z_bits: 0x7FC0_0001,
        ..P0
    };
    let neg_zero = Pt {
        z_bits: 0x8000_0000,
        ..P1
    };
    let mut r = Rig::new();
    r.send(&s_count(3));
    r.run(10);
    r.take();
    // Frame 6 is stored as 3, `current`, `autocontinue` and the parameters (NaN too) are ignored.
    let odd = |seq, pt, frame| Spec {
        frame,
        current: 1,
        autocontinue: 0,
        params: [f32::NAN, -1.0, 1e30, 7.0],
        ..Spec::new(seq, pt)
    };
    r.send(&odd(0, nan, 6).raw());
    r.run(10);
    r.send(&odd(1, neg_zero, 3).raw());
    r.run(10);
    r.send(&odd(2, P2, 6).raw());
    r.run(10);
    let pts = [nan, neg_zero, P2];
    let got = r.take();
    assert_eq!(got.last().unwrap().1, v_ack_plan(plan_of(&pts)));
    holds(&mut r, &pts, plan_of(&pts));
    // In any order and as often as asked.
    r.ask(&s_list());
    for seq in [2, 0, 2, 1, 0] {
        assert_eq!(r.ask(&s_req(seq)), [v_item(seq, pts[usize::from(seq)])]);
    }
}

// ---- timers ---------------------------------------------------------------------------------------------------

#[test]
fn an_upload_asks_6_times_1500_ms_apart_then_gives_up_with_the_list_unchanged() {
    let mut r = Rig::new();
    three(&mut r);
    let t0 = r.now + 500;
    r.send_at(t0, &s_count(2));
    r.run_to(t0 + 9_200);
    let got = r.take();
    let slot = |t: u64| t / 5 * 5 + 5;
    let mut expect: Vec<(u64, Got)> = (0..6).map(|k| (slot(t0 + 1500 * k), v_req(0))).collect();
    expect.push((slot(t0 + 9000), v_ack(CANCELLED)));
    assert_eq!(got, expect);
    // The upload is over: the owner can start a new one, and the list is as it was.
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
    assert_eq!(upload(&mut r, &[P1]), v_ack_plan(plan_of(&[P1])));
}

#[test]
fn every_request_of_an_upload_gets_its_own_retries_and_the_count_starts_again() {
    let mut r = Rig::new();
    r.send_at(100, &s_count(2));
    r.run_to(105);
    r.take();
    // Item 0 after 2 retries of its request.
    r.run_to(3_110);
    let got = r.take();
    assert_eq!(got, [(1605, v_req(0)), (3105, v_req(0))]);
    r.send_at(3_111, &s_item(0, P0));
    r.run_to(3_115);
    assert_eq!(r.take(), [(3115, v_req(1))]);
    // Request 1 starts at 0 retries: 6 sends, then the give-up 9000 ms after the first.
    r.run_to(3_111 + 9_100);
    let got = r.take();
    let times: Vec<u64> = got.iter().map(|g| g.0).collect();
    assert_eq!(times, [4_615, 6_115, 7_615, 9_115, 10_615, 12_115]);
    assert!(got[..5].iter().all(|g| g.1 == v_req(1)), "{got:?}");
    assert_eq!(got[5].1, v_ack(CANCELLED));
}

#[test]
fn a_wrong_item_is_dropped_unchecked_the_request_is_repeated_and_its_timer_restarts() {
    let mut r = Rig::new();
    r.send_at(100, &s_count(3));
    r.send_at(107, &s_item(0, P0));
    r.run_to(112);
    assert_eq!(r.take(), [(105, v_req(0)), (110, v_req(1))]);
    // A repeat of item 0, and item 2 with a bad frame: both only get the request again, at the next slot.
    r.send_at(1000, &s_item(0, P0));
    r.run_to(1005);
    assert_eq!(r.take(), [(1005, v_req(1))]);
    let bad = Spec {
        frame: 99,
        mission_type: 1,
        ..Spec::new(2, P2)
    };
    r.send_at(1100, &bad.raw());
    r.run_to(1105);
    assert_eq!(r.take(), [(1105, v_req(1))]);
    // The timer restarted at 1100: no retry at 1607, the first one at 2600.
    r.run_to(2_599);
    assert_eq!(r.take(), []);
    r.run_to(2_605);
    assert_eq!(r.take(), [(2605, v_req(1))]);
    // And the upload goes on.
    assert_eq!(r.ask(&s_item(1, P1)), [v_req(2)]);
    assert_eq!(r.ask(&s_item(2, P2)), [v_ack_plan(EXAMPLE_PLAN_ID)]);
}

#[test]
fn a_download_ends_after_9000_ms_without_a_message_from_its_owner() {
    let mut r = Rig::new();
    three(&mut r);
    let t0 = r.now + 500;
    r.send_at(t0, &s_list());
    r.run_to(t0 + 9_200);
    let slot = |t: u64| t / 5 * 5 + 5;
    assert_eq!(
        r.take(),
        [
            (slot(t0), v_count(3, EXAMPLE_PLAN_ID)),
            (slot(t0 + 9000), v_ack(CANCELLED))
        ]
    );
    // It is over: a request is an error and the list is as it was.
    assert_eq!(r.ask(&s_req(0)), [v_ack(ERROR)]);
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
}

#[test]
fn every_message_of_the_owner_restarts_the_9000_ms_of_a_download() {
    let mut r = Rig::new();
    three(&mut r);
    r.send_at(1_000, &s_list());
    r.send_at(8_000, &s_req(1));
    r.run_to(8_010);
    r.take();
    r.run_to(17_000);
    assert_eq!(r.take(), []);
    r.run_to(17_010);
    assert_eq!(r.take(), [(17_005, v_ack(CANCELLED))]);
}

// ---- owners ------------------------------------------------------------------------------------------------

#[test]
fn a_second_owner_is_denied_and_the_running_upload_is_not_changed() {
    let other_ids = [OTHER, (254, 190), (255, 1)];
    for who in other_ids {
        let mut r = mid_upload(3);
        let denied = |mt: u8| Got::Ack {
            to: who,
            ty: DENIED,
            mt,
            opaque: 0,
        };
        assert_eq!(r.ask(&s_count(2).from(who)), [denied(0)]);
        assert_eq!(r.ask(&s_list().from(who)), [denied(0)]);
        assert_eq!(r.ask(&s_clear().from(who)), [denied(0)]);
        assert_eq!(r.ask(&s_item(1, P1).from(who)), [denied(0)]);
        assert_eq!(r.ask(&s_req(0).from(who)), [denied(0)]);
        assert_eq!(r.ask(&s_count_full(2, 1, 0).from(who)), [denied(1)]);
        // An ACK is ignored, even a cancel.
        assert_eq!(r.ask(&s_ack(CANCELLED).from(who)), []);
        // The owner goes on: item 1, item 2, and the stored list is the new one.
        assert_eq!(r.ask(&s_item(1, P1)), [v_req(2)]);
        assert_eq!(r.ask(&s_item(2, P2)), [v_ack_plan(plan_of(&[P1, P1, P2]))]);
        // After the end, the other sender may start.
        assert_eq!(
            r.ask(&s_count(1).from(who)),
            [Got::Req {
                to: who,
                seq: 0,
                mt: 0
            }]
        );
    }
}

#[test]
fn a_second_owner_is_denied_during_a_download_too_and_a_denial_ends_nothing() {
    let mut r = Rig::new();
    three(&mut r);
    assert_eq!(r.ask(&s_list()), [v_count(3, EXAMPLE_PLAN_ID)]);
    for raw in [s_count(0), s_clear(), s_list(), s_req(0), s_item(0, P0)] {
        assert_eq!(
            r.ask(&raw.from(OTHER)),
            [Got::Ack {
                to: OTHER,
                ty: DENIED,
                mt: 0,
                opaque: 0
            }]
        );
    }
    assert_eq!(r.ask(&s_req(1)), [v_item(1, P1)]);
    r.ask(&s_ack(ACCEPTED));
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
}

#[test]
fn the_owner_is_system_and_component_together() {
    // Same system, other component: denied; the other system with the same component: denied.
    let mut r = Rig::new();
    assert_eq!(r.ask(&s_list()), [v_count(0, 0)]);
    for who in [(255, 191), (254, 190)] {
        assert!(matches!(
            r.ask(&s_list().from(who))[..],
            [Got::Ack { ty: DENIED, .. }]
        ));
    }
}

// ---- restart and the lost final ACK -------------------------------------------------------------------------------

#[test]
fn a_repeated_count_or_request_list_of_the_owner_starts_again_from_the_beginning() {
    let mut r = Rig::new();
    assert_eq!(r.ask(&s_count(3)), [v_req(0)]);
    assert_eq!(r.ask(&s_item(0, P0)), [v_req(1)]);
    assert_eq!(r.ask(&s_item(1, P1)), [v_req(2)]);
    // The count again: staging is gone, item 0 is asked for.
    assert_eq!(r.ask(&s_count(2)), [v_req(0)]);
    assert_eq!(r.ask(&s_item(0, P2)), [v_req(1)]);
    assert_eq!(r.ask(&s_item(1, P0)), [v_ack_plan(plan_of(&[P2, P0]))]);
    holds(&mut r, &[P2, P0], plan_of(&[P2, P0]));
    // A repeated request list answers with the count again; item requests go on from there.
    assert_eq!(r.ask(&s_list()), [v_count(2, plan_of(&[P2, P0]))]);
    assert_eq!(r.ask(&s_req(0)), [v_item(0, P2)]);
    assert_eq!(r.ask(&s_list()), [v_count(2, plan_of(&[P2, P0]))]);
    // A clear from the owner in the middle of an upload ends it.
    assert_eq!(r.ask(&s_count(3)), [v_req(0)]);
    assert_eq!(r.ask(&s_clear()), [v_ack(ACCEPTED)]);
    r.quiet_for(10_000);
    holds(&mut r, &[], 0);
}

#[test]
fn a_repeat_of_the_last_item_after_a_commit_gets_the_same_ack_and_changes_nothing() {
    let mut r = Rig::new();
    assert_eq!(upload(&mut r, &[P0, P1, P2]), v_ack_plan(EXAMPLE_PLAN_ID));
    // Any content: only the owner and the seq count.
    for _ in 0..2 {
        assert_eq!(r.ask(&s_item(2, P2)), [v_ack_plan(EXAMPLE_PLAN_ID)]);
        assert_eq!(r.ask(&s_item(2, P0)), [v_ack_plan(EXAMPLE_PLAN_ID)]);
    }
    // Another seq, or another sender: an error.
    assert_eq!(r.ask(&s_item(1, P1)), [v_ack(ERROR)]);
    assert_eq!(
        r.ask(&s_item(2, P2).from(OTHER)),
        [Got::Ack {
            to: OTHER,
            ty: ERROR,
            mt: 0,
            opaque: 0
        }]
    );
    r.quiet_for(10_000);
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
    // Any start message makes it forget (the download above was one).
    assert_eq!(r.ask(&s_item(2, P2)), [v_ack(ERROR)]);
}

#[test]
fn a_refusal_at_an_item_is_repeated_for_the_same_item_until_a_start_message() {
    let mut r = mid_upload(3);
    let bad = Spec {
        frame: 2,
        ..Spec::new(1, P1)
    };
    assert_eq!(r.ask(&bad.raw()), [v_ack(UNSUPPORTED_FRAME)]);
    assert_eq!(r.ask(&bad.raw()), [v_ack(UNSUPPORTED_FRAME)]);
    assert_eq!(r.ask(&s_item(1, P1)), [v_ack(UNSUPPORTED_FRAME)]);
    assert_eq!(r.ask(&s_item(0, P0)), [v_ack(ERROR)]);
    // A start message that is itself refused still makes it forget.
    assert!(matches!(
        r.ask(&s_list_mt(1))[..],
        [Got::Ack {
            ty: UNSUPPORTED,
            ..
        }]
    ));
    assert_eq!(r.ask(&bad.raw()), [v_ack(ERROR)]);
}

#[test]
fn the_memory_of_the_last_upload_is_not_made_by_a_timeout_or_a_cancel() {
    let mut r = Rig::new();
    assert_eq!(r.ask(&s_count(2)), [v_req(0)]);
    r.ask(&s_ack(CANCELLED));
    assert_eq!(r.ask(&s_item(0, P0)), [v_ack(ERROR)]);
    assert_eq!(r.ask(&s_count(2)), [v_req(0)]);
    r.run(9_100);
    assert!(r.take().iter().any(|g| g.1 == v_ack(CANCELLED)));
    assert_eq!(r.ask(&s_item(0, P0)), [v_ack(ERROR)]);
}

// ---- link down -----------------------------------------------------------------------------------------------

#[test]
fn link_down_ends_the_transaction_silently_drops_the_waiting_reply_and_forgets_the_last_upload() {
    let mut r = mid_upload(3);
    // A reply is waiting in the slot when the link goes down.
    r.send(&s_item(1, P1));
    r.v.link_down();
    r.quiet_for(10_000);
    // The transaction is gone: another sender can start, and the list is as it was.
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
    let mut r = Rig::new();
    three(&mut r);
    r.v.link_down();
    assert_eq!(r.ask(&s_item(2, P2)), [v_ack(ERROR)]);
    holds(&mut r, &[P0, P1, P2], EXAMPLE_PLAN_ID);
    let mut r = mid_upload(3);
    r.v.link_down();
    assert_eq!(
        r.ask(&s_count(1).from(OTHER)),
        [Got::Req {
            to: OTHER,
            seq: 0,
            mt: 0
        }]
    );
}

// ---- the reply slot -------------------------------------------------------------------------------------------

#[test]
fn a_new_reply_to_the_same_destination_replaces_the_waiting_one() {
    let mut r = Rig::new();
    three(&mut r);
    r.ask(&s_list());
    r.send_at(1201, &s_req(0));
    r.send_at(1203, &s_req(2));
    r.run_to(1215);
    assert_eq!(r.take(), [(1205, v_item(2, P2))]);
}

#[test]
fn a_reply_to_another_destination_is_dropped_while_one_waits() {
    let mut r = Rig::new();
    three(&mut r);
    r.ask(&s_list());
    r.send_at(2001, &s_req(0));
    r.send_at(2003, &s_count(1).from(OTHER));
    r.run_to(10_000);
    // The denial is not sent later; the owner's item is.
    assert_eq!(r.take(), [(2005, v_item(0, P0))]);
}

#[test]
fn at_most_one_mission_message_leaves_per_5_ms() {
    let mut r = Rig::new();
    three(&mut r);
    r.ask(&s_list());
    for (i, t) in [3001u64, 3006, 3011].into_iter().enumerate() {
        r.send_at(t, &s_req(i as u16));
    }
    r.run_to(3030);
    let times: Vec<u64> = r.take().into_iter().map(|g| g.0).collect();
    assert_eq!(times, [3005, 3010, 3015]);
}

// ---- addressing -------------------------------------------------------------------------------------------------

#[test]
fn a_message_for_another_system_or_component_is_dropped_and_not_answered() {
    let mut r = Rig::new();
    for to in [(2, 1), (1, 2), (255, 190), (0, 2), (2, 0)] {
        assert_eq!(r.ask(&s_count(17).to(to)), [], "{to:?}");
        assert_eq!(r.ask(&s_clear().to(to)), []);
    }
    // 0 means everyone; a frame cut before its target component reads 0 there.
    for to in [(0, 0), (0, 1), (1, 0)] {
        assert_eq!(r.ask(&s_count(17).to(to)), [v_ack(NO_SPACE)], "{to:?}");
    }
    let mut cut = s_list();
    cut.p = vec![1];
    assert_eq!(r.ask(&cut), [v_count(0, 0)]);
}

#[test]
fn a_mission_message_from_the_vehicles_own_system_is_ignored() {
    let mut r = Rig::new();
    assert_eq!(r.ask(&s_count(17).from((1, 1))), []);
}

// ---- fault injection ------------------------------------------------------------------------------------------

#[test]
fn every_nth_mission_frame_to_the_vehicle_can_be_lost() {
    let mut r = Rig::with(SimConfig {
        streams: Vec::new(),
        drop_mission_rx_every: 3,
        ..SimConfig::default()
    });
    let mut answered = Vec::new();
    for i in 1..=7u64 {
        r.send_at(i * 100, &s_clear());
        r.run_to(i * 100 + 20);
        answered.push(!r.take().is_empty());
        // A frame for another vehicle is not counted.
        r.send_at(i * 100 + 50, &s_clear().to((2, 1)));
        r.run_to(i * 100 + 70);
        assert_eq!(r.take(), []);
    }
    assert_eq!(answered, [true, true, false, true, true, false, true]);
}

#[test]
fn every_nth_mission_frame_from_the_vehicle_can_be_lost_and_still_takes_a_sequence_number() {
    let mut r = Rig::with(SimConfig {
        streams: Vec::new(),
        drop_mission_tx_every: 2,
        ..SimConfig::default()
    });
    let mut seen = Vec::new();
    for i in 1..=6u64 {
        r.send_at(i * 100, &s_clear());
        r.run_to(i * 100 + 20);
    }
    seen.extend(std::mem::take(&mut r.seen));
    assert_eq!(seen.len(), 3);
    // The boot text had sequence 0; the ACKs 1, 3, 5 were sent and 2, 4, 6 were lost.
    assert_eq!(seen.iter().map(|s| s.1).collect::<Vec<_>>(), [1, 3, 5]);
}

#[test]
fn a_lost_request_starts_its_timer_all_the_same() {
    // The 2nd frame the vehicle sends is lost: request 1. Its retry comes 1500 ms after the hand-over.
    let mut r = Rig::with(SimConfig {
        streams: Vec::new(),
        drop_mission_tx_every: 2,
        ..SimConfig::default()
    });
    r.send_at(100, &s_count(3));
    r.send_at(107, &s_item(0, P0));
    r.run_to(1_600);
    assert_eq!(r.take(), [(105, v_req(0))]);
    r.run_to(1_611);
    assert_eq!(r.take(), [(1610, v_req(1))]);
}

#[test]
fn armed_is_off_until_a_test_sets_it() {
    let mut r = Rig::new();
    assert_eq!(upload(&mut r, &[P0]), v_ack_plan(plan_of(&[P0])));
    r.v.set_armed(true);
    assert_eq!(r.ask(&s_count(1)), [v_ack(ERROR)]);
}
