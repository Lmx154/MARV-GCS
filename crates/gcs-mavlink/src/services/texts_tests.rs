use super::*;
use crate::VehicleState;
use crate::dialect::{HEARTBEAT_DATA, STATUSTEXT_DATA};

struct Rig {
    service: Texts,
    state: VehicleState,
    out: Vec<Message>,
    changed: bool,
}

impl Rig {
    fn new() -> Self {
        Self {
            service: Texts::new(),
            state: VehicleState::default(),
            out: Vec::new(),
            changed: false,
        }
    }

    fn frame(&mut self, frame: &Frame, now_ms: u64) {
        let mut cx = Ctx {
            now: Duration::from_millis(now_ms),
            wall_time_us: 0,
            state: &mut self.state,
            out: &mut self.out,
            changed: false,
        };
        self.service.on_frame(frame, &mut cx);
        self.changed = cx.changed;
    }

    fn entries(&self) -> Vec<TextEntry> {
        self.state.texts.entries.iter().cloned().collect()
    }
}

fn text_frame(
    sender: (u8, u8),
    severity: MavSeverity,
    bytes: &[u8],
    id: u16,
    chunk_seq: u8,
) -> Frame {
    let mut text = [0u8; 50];
    text[..bytes.len()].copy_from_slice(bytes);
    Frame {
        system_id: sender.0,
        component_id: sender.1,
        sequence: 0,
        message_id: 253,
        raw: Vec::new(),
        body: Body::Known(Message::STATUSTEXT(STATUSTEXT_DATA {
            severity,
            text: text.into(),
            id,
            chunk_seq,
        })),
    }
}

fn plain(sender: (u8, u8), text: &str) -> Frame {
    text_frame(
        sender,
        MavSeverity::MAV_SEVERITY_INFO,
        text.as_bytes(),
        0,
        0,
    )
}

fn chunk(sender: (u8, u8), bytes: &[u8], id: u16, chunk_seq: u8) -> Frame {
    text_frame(
        sender,
        MavSeverity::MAV_SEVERITY_WARNING,
        bytes,
        id,
        chunk_seq,
    )
}

const VEHICLE: (u8, u8) = (1, 1);

#[test]
fn a_text_becomes_an_entry() {
    let mut rig = Rig::new();
    let frame = text_frame(
        (7, 9),
        MavSeverity::MAV_SEVERITY_CRITICAL,
        b"PreArm: Compass",
        0,
        0,
    );
    rig.frame(&frame, 1234);
    assert!(rig.changed);
    assert_eq!(
        rig.entries(),
        vec![TextEntry {
            time: Duration::from_millis(1234),
            severity: Severity::Critical,
            text: "PreArm: Compass".into(),
            system_id: 7,
            component_id: 9,
        }]
    );
}

#[test]
fn every_severity_maps() {
    let all = [
        (MavSeverity::MAV_SEVERITY_EMERGENCY, Severity::Emergency),
        (MavSeverity::MAV_SEVERITY_ALERT, Severity::Alert),
        (MavSeverity::MAV_SEVERITY_CRITICAL, Severity::Critical),
        (MavSeverity::MAV_SEVERITY_ERROR, Severity::Error),
        (MavSeverity::MAV_SEVERITY_WARNING, Severity::Warning),
        (MavSeverity::MAV_SEVERITY_NOTICE, Severity::Notice),
        (MavSeverity::MAV_SEVERITY_INFO, Severity::Info),
        (MavSeverity::MAV_SEVERITY_DEBUG, Severity::Debug),
    ];
    let mut rig = Rig::new();
    for (wire, _) in all {
        rig.frame(&text_frame(VEHICLE, wire, b"x", 0, 0), 0);
    }
    let got: Vec<Severity> = rig.entries().iter().map(|e| e.severity).collect();
    let want: Vec<Severity> = all.iter().map(|&(_, s)| s).collect();
    assert_eq!(got, want);
}

#[test]
fn text_stops_at_the_first_nul_and_is_lossy() {
    let mut rig = Rig::new();
    rig.frame(&plain(VEHICLE, "abc\0def"), 0);
    rig.frame(
        &text_frame(VEHICLE, MavSeverity::MAV_SEVERITY_INFO, b"ab\xFFcd", 0, 0),
        0,
    );
    rig.frame(&plain(VEHICLE, ""), 0);
    let texts: Vec<String> = rig.entries().into_iter().map(|e| e.text).collect();
    assert_eq!(texts, ["abc", "ab\u{FFFD}cd", ""]);
}

#[test]
fn a_full_50_byte_text_without_nul_is_kept_whole() {
    let mut rig = Rig::new();
    let bytes = [b'z'; 50];
    rig.frame(
        &text_frame(VEHICLE, MavSeverity::MAV_SEVERITY_INFO, &bytes, 0, 0),
        0,
    );
    assert_eq!(rig.entries()[0].text, "z".repeat(50));
}

#[test]
fn other_bodies_are_ignored() {
    let mut rig = Rig::new();
    let mut frame = plain(VEHICLE, "x");
    frame.body = Body::Unknown;
    rig.frame(&frame, 0);
    frame.body = Body::Undecodable("x".into());
    rig.frame(&frame, 0);
    frame.body = Body::Known(Message::HEARTBEAT(HEARTBEAT_DATA::default()));
    rig.frame(&frame, 0);
    assert!(rig.entries().is_empty());
    assert!(!rig.changed);
}

#[test]
fn keeps_the_newest_max_texts() {
    let mut rig = Rig::new();
    for n in 0..MAX_TEXTS + 5 {
        rig.frame(&plain(VEHICLE, &format!("t{n}")), n as u64);
    }
    let entries = rig.entries();
    assert_eq!(entries.len(), MAX_TEXTS);
    assert_eq!(entries[0].text, "t5");
    assert_eq!(entries[MAX_TEXTS - 1].text, format!("t{}", MAX_TEXTS + 4));
}

#[test]
fn chunks_with_consecutive_numbers_make_one_entry() {
    let mut rig = Rig::new();
    rig.frame(&chunk(VEHICLE, b"first ", 42, 0), 100);
    rig.frame(&chunk(VEHICLE, b"second ", 42, 1), 200);
    assert!(rig.changed);
    rig.frame(&chunk(VEHICLE, b"third", 42, 2), 300);
    assert!(rig.changed);
    let entries = rig.entries();
    assert_eq!(entries.len(), 1);
    assert_eq!(entries[0].text, "first second third");
    assert_eq!(
        entries[0].time,
        Duration::from_millis(100),
        "arrival of the first chunk"
    );
    assert_eq!(entries[0].severity, Severity::Warning);
}

#[test]
fn a_character_split_between_chunks_decodes_whole() {
    let mut rig = Rig::new();
    let euro = "\u{20AC}".as_bytes();
    rig.frame(&chunk(VEHICLE, &euro[..2], 9, 0), 0);
    rig.frame(&chunk(VEHICLE, &euro[2..], 9, 1), 0);
    assert_eq!(rig.entries()[0].text, "\u{20AC}");
}

#[test]
fn chunks_that_do_not_continue_start_a_new_entry() {
    let mut rig = Rig::new();
    rig.frame(&chunk(VEHICLE, b"a", 5, 0), 0);
    rig.frame(&chunk(VEHICLE, b"b", 5, 2), 0);
    rig.frame(&chunk(VEHICLE, b"c", 6, 3), 0);
    rig.frame(&chunk(VEHICLE, b"d", 6, 3), 0);
    rig.frame(&chunk(VEHICLE, b"e", 6, 0), 0);
    let texts: Vec<String> = rig.entries().into_iter().map(|e| e.text).collect();
    assert_eq!(texts, ["a", "b", "c", "d", "e"]);
}

#[test]
fn id_zero_never_continues() {
    let mut rig = Rig::new();
    rig.frame(&chunk(VEHICLE, b"a", 0, 0), 0);
    rig.frame(&chunk(VEHICLE, b"b", 0, 1), 0);
    assert_eq!(rig.entries().len(), 2);
}

#[test]
fn an_other_text_in_between_breaks_the_chain() {
    let mut rig = Rig::new();
    rig.frame(&chunk(VEHICLE, b"a", 5, 0), 0);
    rig.frame(&plain(VEHICLE, "interruption"), 0);
    rig.frame(&chunk(VEHICLE, b"b", 5, 1), 0);
    let texts: Vec<String> = rig.entries().into_iter().map(|e| e.text).collect();
    assert_eq!(texts, ["a", "interruption", "b"]);
}

#[test]
fn chains_are_per_sender() {
    let mut rig = Rig::new();
    let other = (1, 2);
    rig.frame(&chunk(VEHICLE, b"a1 ", 5, 0), 0);
    rig.frame(&chunk(other, b"b1 ", 5, 0), 0);
    rig.frame(&chunk(VEHICLE, b"a2", 5, 1), 0);
    rig.frame(&chunk(other, b"b2", 5, 1), 0);
    let entries = rig.entries();
    assert_eq!(entries.len(), 2);
    assert_eq!(entries[0].text, "a1 a2");
    assert_eq!((entries[0].system_id, entries[0].component_id), VEHICLE);
    assert_eq!(entries[1].text, "b1 b2");
    assert_eq!((entries[1].system_id, entries[1].component_id), other);
}

#[test]
fn another_senders_chunk_with_the_same_id_does_not_continue() {
    let mut rig = Rig::new();
    rig.frame(&chunk(VEHICLE, b"a", 5, 0), 0);
    rig.frame(&chunk((2, 1), b"b", 5, 1), 0);
    assert_eq!(rig.entries().len(), 2);
}

#[test]
fn chunk_sequence_at_255_does_not_overflow() {
    let mut rig = Rig::new();
    rig.frame(&chunk(VEHICLE, b"a", 5, 255), 0);
    rig.frame(&chunk(VEHICLE, b"b", 5, 0), 0);
    assert_eq!(rig.entries().len(), 2);
}
