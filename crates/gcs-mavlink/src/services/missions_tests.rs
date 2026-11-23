use super::*;
use crate::{VehicleId, VehicleState};

use MavMissionResult as R;

/// The contract's example points (worked example 1): made-up points in the open ocean.
fn example() -> Vec<MissionPoint> {
    vec![
        MissionPoint {
            lat_e7: 100_000_000,
            lon_e7: -300_000_000,
            z: 10.0,
        },
        MissionPoint {
            lat_e7: 100_010_000,
            lon_e7: -300_000_000,
            z: 12.5,
        },
        MissionPoint {
            lat_e7: 100_010_000,
            lon_e7: -299_990_000,
            z: 15.0,
        },
    ]
}

const EXAMPLE_ID: u32 = 0x99C3_BD49;

/// What the vehicle sends to the station: target 255, 190.
fn v_request(seq: u16) -> Message {
    Message::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
        seq,
        target_system: 255,
        target_component: 190,
        mission_type: PLAN,
    })
}

fn v_ack(result: MavMissionResult, opaque_id: u32) -> Message {
    Message::MISSION_ACK(MISSION_ACK_DATA {
        target_system: 255,
        target_component: 190,
        mavtype: result,
        mission_type: PLAN,
        opaque_id,
    })
}

fn v_count(count: u16, opaque_id: u32) -> Message {
    Message::MISSION_COUNT(MISSION_COUNT_DATA {
        count,
        target_system: 255,
        target_component: 190,
        mission_type: PLAN,
        opaque_id,
    })
}

fn v_item_with(seq: u16, point: MissionPoint, frame: MavFrame) -> Message {
    Message::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: point.lat_e7,
        y: point.lon_e7,
        z: point.z,
        seq,
        command: MavCmd::MAV_CMD_NAV_WAYPOINT,
        target_system: 255,
        target_component: 190,
        frame,
        current: 0,
        autocontinue: 1,
        mission_type: PLAN,
    })
}

fn v_item(seq: u16, point: MissionPoint) -> Message {
    v_item_with(seq, point, MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT)
}

/// What the station sends to the vehicle: target 1, 1; every field written out.
fn s_count(count: u16) -> Message {
    Message::MISSION_COUNT(MISSION_COUNT_DATA {
        count,
        target_system: 1,
        target_component: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        opaque_id: 0,
    })
}

fn s_item(seq: u16, point: MissionPoint) -> Message {
    Message::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: point.lat_e7,
        y: point.lon_e7,
        z: point.z,
        seq,
        command: MavCmd::MAV_CMD_NAV_WAYPOINT,
        target_system: 1,
        target_component: 1,
        frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        current: 0,
        autocontinue: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
    })
}

fn s_list() -> Message {
    Message::MISSION_REQUEST_LIST(MISSION_REQUEST_LIST_DATA {
        target_system: 1,
        target_component: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
    })
}

fn s_clear() -> Message {
    Message::MISSION_CLEAR_ALL(MISSION_CLEAR_ALL_DATA {
        target_system: 1,
        target_component: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
    })
}

fn s_request(seq: u16) -> Message {
    Message::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
        seq,
        target_system: 1,
        target_component: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
    })
}

fn s_ack(result: MavMissionResult) -> Message {
    Message::MISSION_ACK(MISSION_ACK_DATA {
        target_system: 1,
        target_component: 1,
        mavtype: result,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        opaque_id: 0,
    })
}

/// A short name of a station message, for timelines.
fn tag(message: &Message) -> String {
    match message {
        Message::MISSION_COUNT(m) => format!("COUNT {}", m.count),
        Message::MISSION_CLEAR_ALL(_) => "CLEAR".into(),
        Message::MISSION_REQUEST_LIST(_) => "LIST".into(),
        Message::MISSION_REQUEST_INT(m) => format!("REQ {}", m.seq),
        Message::MISSION_ITEM_INT(m) => format!("ITEM {}", m.seq),
        Message::MISSION_ACK(m) => format!("ACK {:?}", m.mavtype),
        other => format!("other {other:?}"),
    }
}

struct Rig {
    service: Missions,
    state: VehicleState,
    /// Every message sent, with the call's time in ms.
    log: Vec<(u64, Message)>,
    /// The first message of `log` the vehicle fake has not seen.
    cursor: usize,
}

impl Rig {
    fn new() -> Self {
        let mut state = VehicleState::default();
        state.link.status = LinkStatus::Connected;
        state.link.vehicle = Some(VehicleId {
            system_id: 1,
            component_id: 1,
            mav_type: "MAV_TYPE_GENERIC".into(),
            autopilot: "MAV_AUTOPILOT_GENERIC".into(),
        });
        Self {
            service: Missions::new(&SessionConfig::default()),
            state,
            log: Vec::new(),
            cursor: 0,
        }
    }

    fn call(&mut self, now_ms: u64, f: impl FnOnce(&mut Missions, &mut Ctx)) {
        let mut out = Vec::new();
        let mut cx = Ctx {
            now: Duration::from_millis(now_ms),
            wall_time_us: 0,
            state: &mut self.state,
            out: &mut out,
            changed: false,
        };
        f(&mut self.service, &mut cx);
        for message in out {
            self.log.push((now_ms, message));
        }
    }

    fn command(&mut self, command: Command, now_ms: u64) {
        self.call(now_ms, |s, cx| s.on_command(&command, cx));
    }

    fn upload(&mut self, items: &[MissionPoint], now_ms: u64) {
        self.command(
            Command::UploadMission {
                items: items.to_vec(),
            },
            now_ms,
        );
    }

    fn clear(&mut self, now_ms: u64) {
        self.command(Command::ClearMission, now_ms);
    }

    fn download(&mut self, now_ms: u64) {
        self.command(Command::DownloadMission, now_ms);
    }

    fn tick(&mut self, now_ms: u64) {
        self.call(now_ms, |s, cx| s.on_tick(cx));
    }

    /// Ticks every millisecond from `from` to `to` inclusive.
    fn tick_range(&mut self, from: u64, to: u64) {
        for t in from..=to {
            self.tick(t);
        }
    }

    fn close(&mut self, now_ms: u64) {
        self.state.link.status = LinkStatus::Closed("unplugged".into());
        self.call(now_ms, |s, cx| s.on_close(cx));
    }

    fn deliver_from(&mut self, system: u8, component: u8, message: Message, now_ms: u64) {
        let frame = Frame {
            system_id: system,
            component_id: component,
            sequence: 0,
            message_id: 0,
            raw: Vec::new(),
            body: Body::Known(message),
        };
        self.call(now_ms, |s, cx| s.on_frame(&frame, cx));
    }

    /// A message from the vehicle (system 1, component 1).
    fn deliver(&mut self, message: Message, now_ms: u64) {
        self.deliver_from(1, 1, message, now_ms);
    }

    /// What was sent, in order.
    fn sent(&self) -> Vec<Message> {
        self.log.iter().map(|(_, m)| m.clone()).collect()
    }

    /// What was sent, with times, as short names.
    fn timeline(&self) -> Vec<(u64, String)> {
        self.log.iter().map(|(t, m)| (*t, tag(m))).collect()
    }

    /// The times at which a message with this short name was sent.
    fn times(&self, name: &str) -> Vec<u64> {
        self.log
            .iter()
            .filter(|(_, m)| tag(m) == name)
            .map(|(t, _)| *t)
            .collect()
    }

    fn status(&self) -> MissionStatus {
        self.state.missions.status.clone()
    }

    fn failed(&self) -> (MissionFailure, MissionPhase) {
        match self.status() {
            MissionStatus::Failed { reason, phase } => (reason, phase),
            other => panic!("not failed: {other:?}"),
        }
    }

    /// Runs what the station sent since the last call through the vehicle fake, at `now_ms`, until
    /// nothing new is sent. The messages `lose` returns true for do not reach the vehicle.
    fn exchange(
        &mut self,
        vehicle: &mut Vehicle,
        now_ms: u64,
        lose: &mut dyn FnMut(&Message) -> bool,
    ) {
        while self.cursor < self.log.len() {
            let message = self.log[self.cursor].1.clone();
            self.cursor += 1;
            if lose(&message) {
                continue;
            }
            for reply in vehicle.handle(&message) {
                self.deliver(reply, now_ms);
            }
        }
    }

    fn run(&mut self, vehicle: &mut Vehicle, now_ms: u64) {
        self.exchange(vehicle, now_ms, &mut |_| false);
    }
}

/// A vehicle written from the contract's rules for one owner, without timers and without a reply slot.
#[derive(Default)]
struct Vehicle {
    list: Vec<MissionPoint>,
    armed: bool,
    /// An upload running: the count and the items staged.
    staging: Option<(u16, Vec<MissionPoint>)>,
    downloading: bool,
    /// The item of this `seq` is sent with `z` changed, in a download.
    tamper: Option<u16>,
    /// A download says this plan id instead of the true one.
    lie_plan: Option<u32>,
    /// A download says one waypoint fewer than it holds.
    short_by_one: bool,
    /// The item of this `seq` is sent with frame 6, in a download.
    odd_frame: Option<u16>,
}

#[allow(deprecated)] // the frame 6 of the contract is `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`
impl Vehicle {
    fn holding(list: Vec<MissionPoint>) -> Self {
        Self {
            list,
            ..Self::default()
        }
    }

    fn handle(&mut self, message: &Message) -> Vec<Message> {
        match message {
            Message::MISSION_COUNT(m) => {
                self.staging = None;
                self.downloading = false;
                if self.armed {
                    return vec![v_ack(R::MAV_MISSION_ERROR, 0)];
                }
                if m.count > 16 {
                    return vec![v_ack(R::MAV_MISSION_NO_SPACE, 0)];
                }
                if m.count == 0 {
                    self.list.clear();
                    return vec![v_ack(R::MAV_MISSION_ACCEPTED, 0)];
                }
                self.staging = Some((m.count, Vec::new()));
                vec![v_request(0)]
            }
            Message::MISSION_CLEAR_ALL(_) => {
                self.staging = None;
                self.downloading = false;
                if self.armed {
                    return vec![v_ack(R::MAV_MISSION_ERROR, 0)];
                }
                self.list.clear();
                vec![v_ack(R::MAV_MISSION_ACCEPTED, 0)]
            }
            Message::MISSION_REQUEST_LIST(_) => {
                self.staging = None;
                self.downloading = true;
                let held = u16::try_from(self.list.len()).unwrap();
                let count = held - u16::from(self.short_by_one && held > 0);
                vec![v_count(
                    count,
                    self.lie_plan.unwrap_or_else(|| plan_id(&self.list)),
                )]
            }
            Message::MISSION_ITEM_INT(m) => {
                let Some((count, items)) = self.staging.as_mut() else {
                    return vec![v_ack(R::MAV_MISSION_ERROR, 0)];
                };
                let expected = u16::try_from(items.len()).unwrap();
                if m.seq != expected {
                    return vec![v_request(expected)];
                }
                let refusal = if self.armed {
                    Some(R::MAV_MISSION_ERROR)
                } else if m.command != MavCmd::MAV_CMD_NAV_WAYPOINT {
                    Some(R::MAV_MISSION_UNSUPPORTED)
                } else if m.frame != MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
                    && m.frame != MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                {
                    Some(R::MAV_MISSION_UNSUPPORTED_FRAME)
                } else if !(-900_000_000..=900_000_000).contains(&m.x) {
                    Some(R::MAV_MISSION_INVALID_PARAM5_X)
                } else if !(-1_800_000_000..=1_800_000_000).contains(&m.y) {
                    Some(R::MAV_MISSION_INVALID_PARAM6_Y)
                } else {
                    None
                };
                if let Some(result) = refusal {
                    self.staging = None;
                    return vec![v_ack(result, 0)];
                }
                items.push(MissionPoint {
                    lat_e7: m.x,
                    lon_e7: m.y,
                    z: m.z,
                });
                if m.seq + 1 == *count {
                    self.list = std::mem::take(items);
                    self.staging = None;
                    vec![v_ack(R::MAV_MISSION_ACCEPTED, plan_id(&self.list))]
                } else {
                    vec![v_request(m.seq + 1)]
                }
            }
            Message::MISSION_REQUEST_INT(m) => {
                let at = usize::from(m.seq);
                if !self.downloading || at >= self.list.len() {
                    return vec![v_ack(R::MAV_MISSION_INVALID_SEQUENCE, 0)];
                }
                let mut point = self.list[at];
                if self.tamper == Some(m.seq) {
                    point.z += 1.0;
                }
                let frame = if self.odd_frame == Some(m.seq) {
                    MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                } else {
                    MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT
                };
                vec![v_item_with(m.seq, point, frame)]
            }
            Message::MISSION_ACK(_) => {
                self.staging = None;
                self.downloading = false;
                Vec::new()
            }
            _ => Vec::new(),
        }
    }
}

// ---- plan id and degrees

#[test]
fn the_crc_check_value_is_the_ieee_one() {
    assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
}

#[test]
fn the_test_vector_of_the_contract_gives_the_plan_id_of_example_1() {
    assert_eq!(plan_id(&example()), 0x99C3_BD49);
    assert_eq!(plan_id(&example()), 2_579_742_025);
}

#[test]
fn the_36_bytes_of_the_test_vector_are_x_y_z_little_endian_per_waypoint() {
    let mut bytes = Vec::new();
    for point in example() {
        bytes.extend_from_slice(&point.lat_e7.to_le_bytes());
        bytes.extend_from_slice(&point.lon_e7.to_le_bytes());
        bytes.extend_from_slice(&point.z.to_bits().to_le_bytes());
    }
    let contract: [u8; 36] = [
        0x00, 0xE1, 0xF5, 0x05, 0x00, 0x5D, 0x1E, 0xEE, 0x00, 0x00, 0x20, 0x41, //
        0x10, 0x08, 0xF6, 0x05, 0x00, 0x5D, 0x1E, 0xEE, 0x00, 0x00, 0x48, 0x41, //
        0x10, 0x08, 0xF6, 0x05, 0x10, 0x84, 0x1E, 0xEE, 0x00, 0x00, 0x70, 0x41,
    ];
    assert_eq!(bytes, contract);
    assert_eq!(crc32(&bytes), 0x99C3_BD49);
}

#[test]
fn the_empty_list_has_plan_id_0() {
    assert_eq!(plan_id(&[]), 0x0000_0000);
}

#[test]
fn a_non_empty_list_whose_crc_is_0_has_plan_id_1() {
    // z is chosen (its 4 bytes are free) so that the CRC of the 12 bytes is 0.
    let point = MissionPoint {
        lat_e7: 100_000_000,
        lon_e7: -300_000_000,
        z: f32::from_bits(0xD704_32CE),
    };
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&point.lat_e7.to_le_bytes());
    bytes.extend_from_slice(&point.lon_e7.to_le_bytes());
    bytes.extend_from_slice(&point.z.to_bits().to_le_bytes());
    assert_eq!(crc32(&bytes), 0);
    assert_eq!(plan_id(&[point]), 1);
}

#[test]
fn z_is_compared_as_4_bytes() {
    let at = |z: f32| MissionPoint {
        lat_e7: 1,
        lon_e7: 2,
        z,
    };
    assert_ne!(at(0.0), at(-0.0));
    assert_eq!(at(f32::NAN), at(f32::NAN));
    assert_eq!(at(10.0), at(10.0));
}

#[test]
fn degrees_become_e7_rounded_to_nearest_in_f64() {
    assert_eq!(degrees_to_e7(10.0), Some(100_000_000));
    assert_eq!(degrees_to_e7(-30.0), Some(-300_000_000));
    assert_eq!(degrees_to_e7(10.001), Some(100_010_000));
    assert_eq!(degrees_to_e7(-29.999), Some(-299_990_000));
    assert_eq!(degrees_to_e7(0.0), Some(0));
    assert_eq!(degrees_to_e7(90.0), Some(900_000_000));
    assert_eq!(degrees_to_e7(-180.0), Some(-1_800_000_000));
}

#[test]
fn degrees_round_halves_away_from_zero() {
    // 1/256 degree is exact in binary and x 10^7 is 39062.5 exactly.
    assert_eq!(1.0f64 / 256.0 * 1e7, 39_062.5);
    assert_eq!(degrees_to_e7(1.0 / 256.0), Some(39_063));
    assert_eq!(degrees_to_e7(-1.0 / 256.0), Some(-39_063));
    // Below the half rounds down, above it up.
    assert_eq!(degrees_to_e7(0.4e-7), Some(0));
    assert_eq!(degrees_to_e7(0.6e-7), Some(1));
    assert_eq!(degrees_to_e7(-0.6e-7), Some(-1));
}

#[test]
fn degrees_that_are_not_a_number_or_do_not_fit_an_i32_give_nothing() {
    assert_eq!(degrees_to_e7(f64::NAN), None);
    assert_eq!(degrees_to_e7(f64::INFINITY), None);
    assert_eq!(degrees_to_e7(f64::NEG_INFINITY), None);
    assert_eq!(degrees_to_e7(300.0), None);
    assert_eq!(degrees_to_e7(-300.0), None);
    assert_eq!(degrees_to_e7(214.7483647), Some(i32::MAX));
}

// ---- worked examples

#[test]
fn worked_example_1_three_points_one_item_lost_then_the_read_back() {
    let mut rig = Rig::new();
    let points = example();

    // 1. t 0: MISSION_COUNT 3
    rig.upload(&points, 0);
    assert_eq!(rig.sent(), vec![s_count(3)]);
    assert_eq!(rig.status(), MissionStatus::Uploading { sent: 0, total: 3 });

    // 2, 3. t 40: REQUEST_INT 0; the station answers with item 0
    rig.deliver(v_request(0), 40);
    // 4, 5. t 80: REQUEST_INT 1; the item 1 the station sends is lost on the link
    rig.deliver(v_request(1), 80);
    assert_eq!(
        rig.sent(),
        vec![s_count(3), s_item(0, points[0]), s_item(1, points[1])]
    );
    // The wire values of the table: x 100000000, y -300000000, z 10.0 / 12.5.
    let Message::MISSION_ITEM_INT(item) = &rig.sent()[1] else {
        panic!()
    };
    assert_eq!((item.x, item.y, item.z), (100_000_000, -300_000_000, 10.0));
    let Message::MISSION_ITEM_INT(item) = &rig.sent()[2] else {
        panic!()
    };
    assert_eq!((item.x, item.y, item.z), (100_010_000, -300_000_000, 12.5));

    // The station does not resend item 1 (not the last item): its 9000 ms wait runs.
    rig.tick_range(81, 1559);
    assert_eq!(rig.log.len(), 3, "{:?}", rig.timeline());

    // 6, 7. t 1560: REQUEST_INT 1 again; the same item 1
    rig.deliver(v_request(1), 1560);
    assert_eq!(rig.sent()[3], s_item(1, points[1]));
    // 8, 9. t 1600: REQUEST_INT 2; item 2, the last
    rig.deliver(v_request(2), 1600);
    assert_eq!(rig.sent()[4], s_item(2, points[2]));
    let Message::MISSION_ITEM_INT(item) = &rig.sent()[4] else {
        panic!()
    };
    assert_eq!((item.x, item.y, item.z), (100_010_000, -299_990_000, 15.0));
    assert_eq!(rig.status(), MissionStatus::Uploading { sent: 3, total: 3 });

    // 10, 11. t 1640: MISSION_ACK ACCEPTED, opaque_id 2579742025: the upload ends accepted; the
    // station asks for the list at once.
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, 2_579_742_025), 1640);
    assert_eq!(rig.sent()[5], s_list());
    assert_eq!(rig.log[5].0, 1640);
    assert_eq!(
        rig.status(),
        MissionStatus::ReadingBack {
            received: 0,
            total: None
        }
    );

    // 12, 13. t 1680: MISSION_COUNT 3 with the plan id; one request and one item per slot
    rig.deliver(v_count(3, 2_579_742_025), 1680);
    assert_eq!(rig.sent()[6], s_request(0));
    rig.deliver(v_item(0, points[0]), 1720);
    assert_eq!(rig.sent()[7], s_request(1));
    rig.deliver(v_item(1, points[1]), 1760);
    assert_eq!(rig.sent()[8], s_request(2));
    rig.deliver(v_item(2, points[2]), 1800);

    // 14. MISSION_ACK ACCEPTED, opaque_id 0. Verdict: done.
    assert_eq!(rig.sent()[9], s_ack(R::MAV_MISSION_ACCEPTED));
    assert_eq!(rig.log.len(), 10);
    assert_eq!(rig.status(), MissionStatus::Verified);
    assert_eq!(
        rig.state.missions.vehicle,
        Some(VehicleList {
            plan_id: EXAMPLE_ID,
            items: points
        })
    );
    assert_eq!(rig.state.missions.ended, Some(Duration::from_millis(1800)));
    // Nothing more is ever sent.
    rig.tick_range(1801, 20_000);
    assert_eq!(rig.log.len(), 10);
}

#[test]
fn worked_example_1_with_the_vehicle_fake_and_the_item_lost() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle::default();
    rig.upload(&example(), 0);
    let mut lost = false;
    rig.exchange(&mut vehicle, 40, &mut |m| {
        // The second item is lost once.
        let is_item_1 = matches!(m, Message::MISSION_ITEM_INT(i) if i.seq == 1);
        let lose = is_item_1 && !lost;
        lost |= is_item_1;
        lose
    });
    assert_eq!(rig.status(), MissionStatus::Uploading { sent: 2, total: 3 });
    assert!(vehicle.list.is_empty(), "the list is not committed yet");
    // The vehicle's retry: REQUEST_INT 1 again.
    rig.deliver(v_request(1), 1560);
    rig.run(&mut vehicle, 1560);
    assert_eq!(rig.status(), MissionStatus::Verified);
    assert_eq!(vehicle.list, example());
    assert_eq!(
        rig.times("LIST").len() + rig.times("ACK MAV_MISSION_ACCEPTED").len(),
        2
    );
}

#[test]
fn worked_example_2_a_17_point_upload_is_refused_and_the_list_is_read_back() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle::holding(example());
    let seventeen: Vec<MissionPoint> = (0..17).map(|i| example()[i % 3]).collect();

    // 1. MISSION_COUNT 17
    rig.upload(&seventeen, 0);
    assert_eq!(rig.sent(), vec![s_count(17)]);
    // 2. MISSION_ACK NO_SPACE: no item is asked for
    // 3. The upload ends refused; the station reads back: MISSION_REQUEST_LIST, MISSION_COUNT 3 with the
    // plan id, the three items, MISSION_ACK ACCEPTED.
    rig.run(&mut vehicle, 40);
    assert_eq!(
        rig.sent(),
        vec![
            s_count(17),
            s_list(),
            s_request(0),
            s_request(1),
            s_request(2),
            s_ack(R::MAV_MISSION_ACCEPTED)
        ]
    );
    assert_eq!(
        rig.failed(),
        (
            MissionFailure::Refused(R::MAV_MISSION_NO_SPACE),
            MissionPhase::Transfer
        )
    );
    assert_eq!(
        MissionFailure::Refused(R::MAV_MISSION_NO_SPACE).to_string(),
        "refused: the vehicle holds at most 16 waypoints"
    );
    // The vehicle's list is unchanged and shown as what the vehicle holds.
    assert_eq!(vehicle.list, example());
    assert_eq!(
        rig.state.missions.vehicle,
        Some(VehicleList {
            plan_id: EXAMPLE_ID,
            items: example()
        })
    );
    assert_eq!(rig.state.missions.sent, seventeen);
}

// ---- upload, with the vehicle fake

#[test]
fn an_upload_with_a_read_back_that_matches_ends_verified() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle::default();
    rig.upload(&example(), 0);
    assert_eq!(rig.state.missions.runs, 1);
    assert_eq!(rig.state.missions.kind, Some(MissionKind::Upload));
    rig.run(&mut vehicle, 100);
    assert_eq!(rig.status(), MissionStatus::Verified);
    assert_eq!(vehicle.list, example());
    assert_eq!(
        rig.timeline()
            .iter()
            .map(|(_, t)| t.as_str())
            .collect::<Vec<_>>(),
        [
            "COUNT 3",
            "ITEM 0",
            "ITEM 1",
            "ITEM 2",
            "LIST",
            "REQ 0",
            "REQ 1",
            "REQ 2",
            "ACK MAV_MISSION_ACCEPTED"
        ]
    );
    assert_eq!(rig.state.missions.ended, Some(Duration::from_millis(100)));
}

#[test]
fn an_upload_of_the_most_the_vehicle_holds_ends_verified() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle::default();
    let sixteen: Vec<MissionPoint> = (0..16).map(|i| example()[i % 3]).collect();
    rig.upload(&sixteen, 0);
    rig.run(&mut vehicle, 1);
    assert_eq!(rig.status(), MissionStatus::Verified);
    assert_eq!(vehicle.list, sixteen);
}

#[test]
fn a_read_back_that_differs_in_one_item_is_not_verified() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle {
        tamper: Some(1),
        ..Vehicle::default()
    };
    rig.upload(&example(), 0);
    rig.run(&mut vehicle, 1);
    assert_eq!(
        rig.failed(),
        (
            MissionFailure::ItemDiffers { seq: 1 },
            MissionPhase::ReadBack
        )
    );
    // The read-back list is shown as what the vehicle holds.
    let held = rig.state.missions.vehicle.clone().expect("the list");
    assert_eq!(held.items[1].z, 13.5);
    assert_eq!(held.items[0], example()[0]);
}

#[test]
fn the_first_item_that_differs_is_named() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle {
        tamper: Some(2),
        ..Vehicle::default()
    };
    rig.upload(&example(), 0);
    rig.run(&mut vehicle, 1);
    assert_eq!(rig.failed().0, MissionFailure::ItemDiffers { seq: 2 });
}

#[test]
fn a_read_back_with_another_plan_id_is_not_verified() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle {
        lie_plan: Some(7),
        ..Vehicle::default()
    };
    rig.upload(&example(), 0);
    rig.run(&mut vehicle, 1);
    assert_eq!(
        rig.failed(),
        (
            MissionFailure::PlanIdDiffers {
                sent: EXAMPLE_ID,
                found: 7
            },
            MissionPhase::ReadBack
        )
    );
    assert_eq!(
        rig.state.missions.vehicle.as_ref().map(|v| v.items.clone()),
        Some(example())
    );
}

#[test]
fn a_read_back_with_another_count_is_not_verified() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle {
        short_by_one: true,
        ..Vehicle::default()
    };
    rig.upload(&example(), 0);
    rig.run(&mut vehicle, 1);
    assert_eq!(
        rig.failed(),
        (
            MissionFailure::CountDiffers { sent: 3, found: 2 },
            MissionPhase::ReadBack
        )
    );
}

#[test]
fn a_read_back_item_with_frame_6_is_not_what_was_sent() {
    // The vehicle always says frame 3 (section 17); the verdict compares the frame.
    let mut rig = Rig::new();
    let mut vehicle = Vehicle {
        odd_frame: Some(2),
        ..Vehicle::default()
    };
    rig.upload(&example(), 0);
    rig.run(&mut vehicle, 1);
    assert_eq!(rig.failed().0, MissionFailure::ItemDiffers { seq: 2 });
}

#[test]
fn a_refused_upload_is_not_done_even_when_the_vehicle_holds_the_list() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle::holding(example());
    vehicle.armed = true;
    rig.upload(&example(), 0);
    rig.run(&mut vehicle, 1);
    assert_eq!(
        rig.failed(),
        (
            MissionFailure::Refused(R::MAV_MISSION_ERROR),
            MissionPhase::Transfer
        )
    );
    assert_eq!(
        rig.state.missions.vehicle.as_ref().map(|v| v.items.clone()),
        Some(example())
    );
}

#[test]
fn a_latitude_or_longitude_out_of_range_is_refused_by_the_vehicle_not_the_station() {
    for (lat, lon, code) in [
        (900_000_001, 0, R::MAV_MISSION_INVALID_PARAM5_X),
        (-900_000_001, 0, R::MAV_MISSION_INVALID_PARAM5_X),
        (0, 1_800_000_001, R::MAV_MISSION_INVALID_PARAM6_Y),
        (0, -1_800_000_001, R::MAV_MISSION_INVALID_PARAM6_Y),
    ] {
        let mut rig = Rig::new();
        let mut vehicle = Vehicle::default();
        let bad = MissionPoint {
            lat_e7: lat,
            lon_e7: lon,
            z: 1.0,
        };
        rig.upload(&[example()[0], bad], 0);
        rig.run(&mut vehicle, 1);
        // The station sent both items: it checks nothing itself.
        assert_eq!(rig.times("ITEM 1").len(), 1, "{lat} {lon}");
        assert_eq!(
            rig.failed(),
            (MissionFailure::Refused(code), MissionPhase::Transfer)
        );
        assert!(vehicle.list.is_empty());
    }
}

#[test]
fn the_limits_of_the_range_are_sent_and_accepted() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle::default();
    let edge = MissionPoint {
        lat_e7: 900_000_000,
        lon_e7: -1_800_000_000,
        z: -5.0,
    };
    rig.upload(&[edge], 0);
    rig.run(&mut vehicle, 1);
    assert_eq!(rig.status(), MissionStatus::Verified);
}

// ---- refusal codes

#[test]
fn each_refusal_code_of_the_contract_is_surfaced_with_its_meaning() {
    let codes = [
        (R::MAV_MISSION_ERROR, "armed"),
        (R::MAV_MISSION_UNSUPPORTED_FRAME, "frames 3 and 6"),
        (
            R::MAV_MISSION_UNSUPPORTED,
            "mission type 0 and the command 16",
        ),
        (R::MAV_MISSION_NO_SPACE, "at most 16 waypoints"),
        (R::MAV_MISSION_INVALID_PARAM5_X, "latitude"),
        (R::MAV_MISSION_INVALID_PARAM6_Y, "longitude"),
        (R::MAV_MISSION_INVALID_SEQUENCE, "at or above the count"),
        (R::MAV_MISSION_DENIED, "another owner"),
        (R::MAV_MISSION_OPERATION_CANCELLED, "timeout"),
    ];
    let mut meanings = Vec::new();
    for (code, word) in codes {
        let mut rig = Rig::new();
        rig.upload(&example(), 0);
        rig.deliver(v_ack(code, 0), 10);
        // The read-back is on its way; the vehicle's list is empty.
        assert_eq!(rig.sent()[1], s_list(), "{code:?}");
        rig.deliver(v_count(0, 0), 20);
        assert_eq!(
            rig.failed(),
            (MissionFailure::Refused(code), MissionPhase::Transfer),
            "{code:?}"
        );
        let meaning = result_meaning(code);
        assert!(meaning.contains(word), "{code:?}: {meaning}");
        assert!(
            MissionFailure::Refused(code).to_string().contains(meaning),
            "{code:?}"
        );
        meanings.push(meaning);
    }
    meanings.sort_unstable();
    meanings.dedup();
    assert_eq!(meanings.len(), codes.len(), "each code has its own words");
}

#[test]
fn a_refusal_during_a_clear_ends_it_refused_and_reads_back() {
    let mut rig = Rig::new();
    rig.clear(0);
    rig.deliver(v_ack(R::MAV_MISSION_ERROR, 0), 10);
    assert_eq!(rig.sent()[1], s_list());
    rig.deliver(v_count(0, 0), 20);
    assert_eq!(
        rig.failed(),
        (
            MissionFailure::Refused(R::MAV_MISSION_ERROR),
            MissionPhase::Transfer
        )
    );
}

#[test]
fn a_refusal_during_a_download_ends_it_failed_with_no_read_back() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.deliver(v_ack(R::MAV_MISSION_DENIED, 0), 10);
    assert_eq!(
        rig.failed(),
        (
            MissionFailure::Refused(R::MAV_MISSION_DENIED),
            MissionPhase::Transfer
        )
    );
    assert_eq!(rig.sent(), vec![s_list()]);
    rig.tick_range(11, 20_000);
    assert_eq!(rig.log.len(), 1);
}

#[test]
fn a_refusal_in_the_read_back_ends_it_failed_and_nothing_is_read_back_again() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.deliver(v_request(0), 1);
    rig.deliver(v_request(1), 2);
    rig.deliver(v_request(2), 3);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, EXAMPLE_ID), 4);
    assert_eq!(rig.sent().last(), Some(&s_list()));
    rig.deliver(v_ack(R::MAV_MISSION_DENIED, 0), 5);
    assert_eq!(
        rig.failed(),
        (
            MissionFailure::Refused(R::MAV_MISSION_DENIED),
            MissionPhase::ReadBack
        )
    );
    assert_eq!(rig.times("LIST").len(), 1);
}

#[test]
fn a_cancel_from_the_vehicle_ends_the_upload_refused_with_its_code() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.deliver(v_request(0), 10);
    rig.deliver(v_ack(R::MAV_MISSION_OPERATION_CANCELLED, 0), 20);
    assert_eq!(rig.sent().last(), Some(&s_list()));
    rig.deliver(v_count(0, 0), 30);
    assert_eq!(
        rig.failed().0,
        MissionFailure::Refused(R::MAV_MISSION_OPERATION_CANCELLED)
    );
}

#[test]
fn a_request_beyond_the_list_is_answered_invalid_sequence_and_the_upload_fails() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.deliver(v_request(3), 10);
    assert_eq!(
        rig.sent(),
        vec![s_count(3), s_ack(R::MAV_MISSION_INVALID_SEQUENCE), s_list()]
    );
    rig.deliver(v_count(0, 0), 20);
    assert_eq!(
        rig.failed(),
        (
            MissionFailure::InvalidSequence { seq: 3 },
            MissionPhase::Transfer
        )
    );
}

#[test]
fn an_item_is_sent_again_every_time_it_is_asked() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    for t in [10, 20, 30] {
        rig.deliver(v_request(0), t);
    }
    rig.deliver(v_request(1), 40);
    rig.deliver(v_request(0), 50);
    assert_eq!(rig.times("ITEM 0"), [10, 20, 30, 50]);
    assert_eq!(rig.times("ITEM 1"), [40]);
    assert_eq!(rig.status(), MissionStatus::Uploading { sent: 2, total: 3 });
}

// ---- ACCEPTED that does not end an upload

#[test]
fn an_accepted_ack_before_the_last_item_was_sent_does_not_end_the_upload() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    // Before any item.
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, EXAMPLE_ID), 10);
    assert_eq!(rig.status(), MissionStatus::Uploading { sent: 0, total: 3 });
    assert_eq!(rig.log.len(), 1);
    // It stops no timer: the MISSION_COUNT is sent again at 1500.
    rig.tick_range(11, 1500);
    assert_eq!(rig.times("COUNT 3"), [0, 1500]);
    // After items 0 and 1, not the last.
    rig.deliver(v_request(0), 1600);
    rig.deliver(v_request(1), 1610);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, EXAMPLE_ID), 1620);
    assert_eq!(rig.status(), MissionStatus::Uploading { sent: 2, total: 3 });
    assert!(rig.times("LIST").is_empty());
    // And it does not restart the wait of 9000 ms after item 1 (sent at 1610).
    rig.tick_range(1621, 10_609);
    assert!(rig.times("LIST").is_empty());
    rig.tick(10_610);
    assert_eq!(rig.times("LIST"), [10_610]);
}

#[test]
fn an_accepted_ack_with_another_opaque_id_does_not_end_the_upload() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.deliver(v_request(0), 10);
    rig.deliver(v_request(1), 20);
    rig.deliver(v_request(2), 30);
    for wrong in [0, EXAMPLE_ID + 1, 1] {
        rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, wrong), 40);
        assert_eq!(
            rig.status(),
            MissionStatus::Uploading { sent: 3, total: 3 },
            "{wrong}"
        );
    }
    assert!(rig.times("LIST").is_empty());
    // It stops no timer either: the last item is sent again 1500 ms after its send.
    rig.tick_range(41, 1530);
    assert_eq!(rig.times("ITEM 2"), [30, 1530]);
    // The right one ends it.
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, EXAMPLE_ID), 1540);
    assert_eq!(rig.times("LIST"), [1540]);
}

#[test]
fn an_accepted_ack_during_a_download_is_ignored_and_stops_no_timer() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, 0), 100);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, EXAMPLE_ID), 100);
    assert_eq!(
        rig.status(),
        MissionStatus::Downloading {
            received: 0,
            total: None
        }
    );
    rig.tick_range(101, 1500);
    assert_eq!(rig.times("LIST"), [0, 1500]);
    // Also after the count.
    rig.deliver(v_count(2, EXAMPLE_ID), 1600);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, 0), 1700);
    assert_eq!(
        rig.status(),
        MissionStatus::Downloading {
            received: 0,
            total: Some(2)
        }
    );
    rig.tick_range(1701, 3100);
    assert_eq!(rig.times("REQ 0"), [1600, 3100]);
}

#[test]
fn an_accepted_ack_during_the_read_back_is_ignored() {
    let mut rig = Rig::new();
    rig.clear(0);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, 0), 10);
    assert_eq!(rig.times("LIST"), [10]);
    // A late copy of the answer to the clear.
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, 0), 20);
    assert_eq!(
        rig.status(),
        MissionStatus::ReadingBack {
            received: 0,
            total: None
        }
    );
    rig.tick_range(21, 1510);
    assert_eq!(rig.times("LIST"), [10, 1510]);
}

// ---- count 0 and clear

#[test]
fn an_upload_of_no_waypoints_sends_count_0_and_is_a_clear() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle::holding(example());
    rig.upload(&[], 0);
    assert_eq!(rig.sent(), vec![s_count(0)]);
    // The vehicle's answer to count 0: ACCEPTED, opaque_id 0 = the plan id of the empty list.
    rig.run(&mut vehicle, 10);
    assert!(vehicle.list.is_empty());
    assert_eq!(rig.status(), MissionStatus::Verified);
    assert_eq!(
        rig.sent(),
        vec![s_count(0), s_list(), s_ack(R::MAV_MISSION_ACCEPTED)]
    );
    assert_eq!(
        rig.state.missions.vehicle,
        Some(VehicleList {
            plan_id: 0,
            items: Vec::new()
        })
    );
}

#[test]
fn an_upload_of_no_waypoints_ends_on_accepted_after_the_count_and_only_with_plan_id_0() {
    let mut rig = Rig::new();
    rig.upload(&[], 0);
    // ACCEPTED with another opaque_id is a late copy of an earlier answer.
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, EXAMPLE_ID), 10);
    assert_eq!(rig.log.len(), 1);
    // With plan id 0 it ends the upload of the empty list.
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, 0), 20);
    assert_eq!(rig.times("LIST"), [20]);
}

#[test]
fn a_clear_sends_clear_all_and_ends_verified_after_the_read_back() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle::holding(example());
    rig.clear(0);
    assert_eq!(rig.sent(), vec![s_clear()]);
    assert_eq!(rig.status(), MissionStatus::Clearing);
    assert_eq!(rig.state.missions.kind, Some(MissionKind::Clear));
    rig.run(&mut vehicle, 10);
    assert!(vehicle.list.is_empty());
    assert_eq!(rig.status(), MissionStatus::Verified);
    assert_eq!(
        rig.sent(),
        vec![s_clear(), s_list(), s_ack(R::MAV_MISSION_ACCEPTED)]
    );
}

#[test]
fn a_clear_whose_read_back_still_shows_waypoints_is_not_verified() {
    let mut rig = Rig::new();
    rig.clear(0);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, 0), 10);
    rig.deliver(v_count(1, 5), 20);
    rig.deliver(v_item(0, example()[0]), 30);
    assert_eq!(
        rig.failed(),
        (
            MissionFailure::CountDiffers { sent: 0, found: 1 },
            MissionPhase::ReadBack
        )
    );
}

#[test]
fn a_clear_ignores_an_accepted_ack_with_a_plan_id() {
    let mut rig = Rig::new();
    rig.clear(0);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, EXAMPLE_ID), 10);
    assert_eq!(rig.status(), MissionStatus::Clearing);
    assert_eq!(rig.log.len(), 1);
}

// ---- download

#[test]
fn a_download_asks_for_the_list_and_every_item_and_ends_with_accepted() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle::holding(example());
    rig.download(0);
    assert_eq!(rig.sent(), vec![s_list()]);
    assert_eq!(rig.state.missions.kind, Some(MissionKind::Download));
    rig.run(&mut vehicle, 50);
    assert_eq!(
        rig.sent(),
        vec![
            s_list(),
            s_request(0),
            s_request(1),
            s_request(2),
            s_ack(R::MAV_MISSION_ACCEPTED)
        ]
    );
    assert_eq!(rig.status(), MissionStatus::Complete);
    assert_eq!(
        rig.state.missions.vehicle,
        Some(VehicleList {
            plan_id: EXAMPLE_ID,
            items: example()
        })
    );
}

#[test]
fn a_download_shows_its_progress() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.deliver(v_count(3, EXAMPLE_ID), 10);
    assert_eq!(
        rig.status(),
        MissionStatus::Downloading {
            received: 0,
            total: Some(3)
        }
    );
    rig.deliver(v_item(0, example()[0]), 20);
    assert_eq!(
        rig.status(),
        MissionStatus::Downloading {
            received: 1,
            total: Some(3)
        }
    );
}

#[test]
fn a_download_of_an_empty_list_ends_at_once_after_the_count() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.deliver(v_count(0, 0), 10);
    assert_eq!(rig.sent(), vec![s_list(), s_ack(R::MAV_MISSION_ACCEPTED)]);
    assert_eq!(rig.status(), MissionStatus::Complete);
    assert_eq!(
        rig.state.missions.vehicle,
        Some(VehicleList {
            plan_id: 0,
            items: Vec::new()
        })
    );
}

#[test]
fn a_wrong_item_is_dropped_and_the_expected_one_is_asked_for_again_at_once() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.deliver(v_count(3, EXAMPLE_ID), 10);
    rig.deliver(v_item(0, example()[0]), 20);
    // Item 2 when 1 was asked for, and a repeat of 0.
    rig.deliver(v_item(2, example()[2]), 30);
    rig.deliver(v_item(0, example()[0]), 40);
    assert_eq!(
        rig.timeline(),
        [
            (0, "LIST".to_string()),
            (10, "REQ 0".to_string()),
            (20, "REQ 1".to_string()),
            (30, "REQ 1".to_string()),
            (40, "REQ 1".to_string())
        ]
    );
    rig.deliver(v_item(1, example()[1]), 50);
    rig.deliver(v_item(2, example()[2]), 60);
    assert_eq!(rig.status(), MissionStatus::Complete);
    assert_eq!(
        rig.state.missions.vehicle.as_ref().map(|v| v.items.clone()),
        Some(example())
    );
}

#[test]
fn a_second_count_in_the_same_download_is_ignored() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.deliver(v_count(3, EXAMPLE_ID), 10);
    rig.deliver(v_count(1, 9), 20);
    assert_eq!(rig.sent(), vec![s_list(), s_request(0)]);
    rig.deliver(v_item(0, example()[0]), 30);
    rig.deliver(v_count(1, 9), 40);
    rig.deliver(v_item(1, example()[1]), 50);
    rig.deliver(v_item(2, example()[2]), 60);
    assert_eq!(
        rig.state.missions.vehicle,
        Some(VehicleList {
            plan_id: EXAMPLE_ID,
            items: example()
        })
    );
}

#[test]
fn an_item_before_the_count_is_ignored() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.deliver(v_item(0, example()[0]), 10);
    assert_eq!(rig.sent(), vec![s_list()]);
}

// ---- timeouts and retries, with a fake clock

#[test]
fn the_count_is_sent_six_times_1500_ms_apart_then_given_up_and_the_read_back_is_tried() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.tick_range(1, 8999);
    assert_eq!(rig.times("COUNT 3"), [0, 1500, 3000, 4500, 6000, 7500]);
    assert_eq!(rig.log.len(), 6);
    rig.tick(9000);
    // The station gives up: MISSION_ACK CANCELLED, then the read-back at once.
    assert_eq!(
        rig.timeline()[6..],
        [
            (9000, "ACK MAV_MISSION_OPERATION_CANCELLED".to_string()),
            (9000, "LIST".to_string())
        ]
    );
    assert_eq!(rig.sent()[6], s_ack(R::MAV_MISSION_OPERATION_CANCELLED));
    assert_eq!(
        rig.status(),
        MissionStatus::ReadingBack {
            received: 0,
            total: None
        }
    );
    // The read-back gets no answer either: six sends of the request, then the cancel, no more.
    rig.tick_range(9001, 30_000);
    assert_eq!(
        rig.times("LIST"),
        [9000, 10_500, 12_000, 13_500, 15_000, 16_500]
    );
    assert_eq!(
        rig.times("ACK MAV_MISSION_OPERATION_CANCELLED"),
        [9000, 18_000]
    );
    assert_eq!(rig.log.len(), 14);
    assert_eq!(
        rig.failed(),
        (MissionFailure::TimedOut, MissionPhase::Transfer)
    );
    assert_eq!(
        rig.state.missions.ended,
        Some(Duration::from_millis(18_000))
    );
    assert_eq!(rig.state.missions.vehicle, None);
}

#[test]
fn a_timed_out_upload_whose_read_back_shows_the_list_is_still_not_done() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.tick_range(1, 9000);
    rig.deliver(v_count(3, EXAMPLE_ID), 9010);
    for (seq, point) in example().into_iter().enumerate() {
        rig.deliver(v_item(seq as u16, point), 9020 + seq as u64);
    }
    assert_eq!(
        rig.failed(),
        (MissionFailure::TimedOut, MissionPhase::Transfer)
    );
    assert_eq!(
        rig.state.missions.vehicle.as_ref().map(|v| v.items.clone()),
        Some(example())
    );
}

#[test]
fn a_request_from_the_vehicle_stops_the_count_timer() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.tick_range(1, 1499);
    rig.deliver(v_request(0), 1499);
    // The wait is now the item wait: 9000 ms after item 0, not 1500 after the count.
    rig.tick_range(1500, 10_498);
    assert_eq!(rig.times("COUNT 3"), [0]);
    assert!(rig.times("LIST").is_empty());
    rig.tick(10_499);
    assert_eq!(rig.times("LIST"), [10_499]);
}

#[test]
fn an_item_other_than_the_last_is_not_resent_and_its_wait_is_9000_ms() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.deliver(v_request(0), 40);
    rig.deliver(v_request(1), 80);
    rig.tick_range(81, 9079);
    assert_eq!(rig.times("ITEM 0"), [40]);
    assert_eq!(rig.times("ITEM 1"), [80]);
    assert!(rig.times("LIST").is_empty());
    rig.tick(9080);
    assert_eq!(
        rig.timeline()[3..],
        [
            (9080, "ACK MAV_MISSION_OPERATION_CANCELLED".to_string()),
            (9080, "LIST".to_string())
        ]
    );
}

#[test]
fn a_request_restarts_the_9000_ms_wait() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.deliver(v_request(0), 40);
    rig.tick_range(41, 5000);
    rig.deliver(v_request(0), 5000);
    rig.tick_range(5001, 13_999);
    assert!(rig.times("LIST").is_empty());
    rig.tick(14_000);
    assert_eq!(rig.times("LIST"), [14_000]);
}

#[test]
fn the_last_item_is_sent_six_times_1500_ms_apart_then_given_up() {
    let mut rig = Rig::new();
    rig.upload(&example()[..1], 0);
    rig.deliver(v_request(0), 10);
    rig.tick_range(11, 9009);
    assert_eq!(
        rig.times("ITEM 0"),
        [10, 1510, 3010, 4510, 6010, 7510],
        "{:?}",
        rig.timeline()
    );
    assert_eq!(rig.sent()[1], rig.sent()[2], "the same message again");
    assert!(rig.times("LIST").is_empty());
    rig.tick(9010);
    assert_eq!(rig.times("ACK MAV_MISSION_OPERATION_CANCELLED"), [9010]);
    assert_eq!(rig.times("LIST"), [9010]);
    assert_eq!(
        rig.status(),
        MissionStatus::ReadingBack {
            received: 0,
            total: None
        }
    );
}

#[test]
fn a_request_for_the_last_item_stops_its_timer_and_the_answer_starts_a_new_one() {
    let mut rig = Rig::new();
    rig.upload(&example()[..1], 0);
    rig.deliver(v_request(0), 10);
    rig.tick_range(11, 1509);
    rig.deliver(v_request(0), 1509);
    rig.tick_range(1510, 3008);
    assert_eq!(rig.times("ITEM 0"), [10, 1509]);
    rig.tick(3009);
    assert_eq!(rig.times("ITEM 0"), [10, 1509, 3009]);
}

#[test]
fn a_clear_is_sent_six_times_1500_ms_apart_then_given_up_and_read_back() {
    let mut rig = Rig::new();
    rig.clear(100);
    rig.tick_range(101, 9099);
    assert_eq!(rig.times("CLEAR"), [100, 1600, 3100, 4600, 6100, 7600]);
    rig.tick(9100);
    assert_eq!(rig.times("ACK MAV_MISSION_OPERATION_CANCELLED"), [9100]);
    assert_eq!(rig.times("LIST"), [9100]);
}

#[test]
fn a_clear_answered_in_time_is_not_sent_again() {
    let mut rig = Rig::new();
    rig.clear(0);
    rig.tick_range(1, 1499);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, 0), 1499);
    rig.tick_range(1500, 3000);
    assert_eq!(rig.times("CLEAR"), [0]);
}

#[test]
fn the_list_request_is_sent_six_times_then_given_up_with_no_read_back() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.tick_range(1, 8999);
    assert_eq!(rig.times("LIST"), [0, 1500, 3000, 4500, 6000, 7500]);
    rig.tick(9000);
    assert_eq!(rig.times("ACK MAV_MISSION_OPERATION_CANCELLED"), [9000]);
    assert_eq!(
        rig.failed(),
        (MissionFailure::TimedOut, MissionPhase::Transfer)
    );
    rig.tick_range(9001, 30_000);
    assert_eq!(rig.log.len(), 7);
    assert_eq!(rig.state.missions.ended, Some(Duration::from_millis(9000)));
}

#[test]
fn an_item_request_is_sent_six_times_then_given_up_and_a_count_stops_the_list_timer() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.tick_range(1, 99);
    rig.deliver(v_count(2, 5), 100);
    rig.tick_range(101, 9099);
    assert_eq!(rig.times("LIST"), [0]);
    assert_eq!(rig.times("REQ 0"), [100, 1600, 3100, 4600, 6100, 7600]);
    rig.tick(9100);
    assert_eq!(rig.times("ACK MAV_MISSION_OPERATION_CANCELLED"), [9100]);
    assert_eq!(
        rig.failed(),
        (MissionFailure::TimedOut, MissionPhase::Transfer)
    );
}

#[test]
fn the_retry_count_starts_at_0_for_every_new_message() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.deliver(v_count(2, 5), 100);
    // One retry of REQ 0 at 1600, then the item.
    rig.tick_range(101, 1700);
    rig.deliver(v_item(0, example()[0]), 1700);
    rig.tick_range(1701, 10_699);
    assert_eq!(rig.times("REQ 1"), [1700, 3200, 4700, 6200, 7700, 9200]);
    assert!(rig.times("ACK MAV_MISSION_OPERATION_CANCELLED").is_empty());
    rig.tick(10_700);
    assert_eq!(rig.times("ACK MAV_MISSION_OPERATION_CANCELLED"), [10_700]);
}

#[test]
fn a_re_request_restarts_the_timer_and_is_not_counted_as_a_retry() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.deliver(v_count(2, 5), 100);
    // A wrong item at 1000: REQ 0 again at once, the timer restarts.
    rig.tick_range(101, 1000);
    rig.deliver(v_item(1, example()[1]), 1000);
    assert_eq!(rig.times("REQ 0"), [100, 1000]);
    rig.tick_range(1001, 9999);
    // Five retries after the re-request: 2500 ... 8500; not 1600 (100 + 1500).
    assert_eq!(
        rig.times("REQ 0"),
        [100, 1000, 2500, 4000, 5500, 7000, 8500]
    );
    assert!(rig.times("ACK MAV_MISSION_OPERATION_CANCELLED").is_empty());
    rig.tick(10_000);
    assert_eq!(rig.times("ACK MAV_MISSION_OPERATION_CANCELLED"), [10_000]);
}

#[test]
fn a_late_answer_after_giving_up_is_ignored() {
    let mut rig = Rig::new();
    rig.download(0);
    rig.tick_range(1, 9000);
    let sent = rig.log.len();
    rig.deliver(v_count(0, 0), 9001);
    assert_eq!(rig.log.len(), sent);
    assert_eq!(rig.failed().0, MissionFailure::TimedOut);
}

// ---- link lost

#[test]
fn link_loss_ends_an_upload_at_once_and_nothing_is_sent() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.deliver(v_request(0), 10);
    let sent = rig.log.len();
    rig.close(500);
    assert_eq!(
        rig.failed(),
        (MissionFailure::LinkLost, MissionPhase::Transfer)
    );
    assert_eq!(rig.state.missions.ended, Some(Duration::from_millis(500)));
    assert_eq!(rig.log.len(), sent, "no cancel, no read-back");
    rig.tick_range(501, 30_000);
    rig.deliver(v_request(1), 600);
    assert_eq!(rig.log.len(), sent);
}

#[test]
fn link_loss_during_the_read_back_ends_it_at_once_with_no_new_read_back() {
    let mut rig = Rig::new();
    rig.clear(0);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, 0), 10);
    rig.deliver(v_count(1, 5), 20);
    let sent = rig.log.len();
    rig.close(30);
    assert_eq!(
        rig.failed(),
        (MissionFailure::LinkLost, MissionPhase::ReadBack)
    );
    assert_eq!(rig.log.len(), sent);
    rig.tick_range(31, 30_000);
    assert_eq!(rig.log.len(), sent);
}

#[test]
fn link_loss_ends_a_download_and_a_clear_at_once() {
    for start in [Rig::download, Rig::clear] {
        let mut rig = Rig::new();
        start(&mut rig, 0);
        rig.close(5);
        assert_eq!(
            rig.failed(),
            (MissionFailure::LinkLost, MissionPhase::Transfer)
        );
        assert_eq!(rig.log.len(), 1);
    }
}

#[test]
fn link_loss_with_nothing_running_changes_nothing() {
    let mut rig = Rig::new();
    rig.close(5);
    assert_eq!(rig.status(), MissionStatus::Idle);
    assert_eq!(rig.state.missions, MissionsState::default());
}

// ---- one operation at a time, start conditions

#[test]
fn a_second_command_while_one_runs_is_refused_and_nothing_is_sent() {
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.deliver(v_request(0), 10);
    let before = rig.log.len();
    let status = rig.status();
    rig.download(20);
    rig.clear(21);
    rig.upload(&example()[..1], 22);
    assert_eq!(rig.log.len(), before);
    assert_eq!(rig.state.missions.refused, 3);
    assert_eq!(rig.state.missions.runs, 1);
    assert_eq!(rig.status(), status);
    assert_eq!(rig.state.missions.sent, example());
}

#[test]
fn a_command_is_refused_during_the_read_back_too() {
    let mut rig = Rig::new();
    rig.clear(0);
    rig.deliver(v_ack(R::MAV_MISSION_ACCEPTED, 0), 10);
    let before = rig.log.len();
    rig.download(20);
    assert_eq!(rig.log.len(), before);
    assert_eq!(rig.state.missions.refused, 1);
}

#[test]
fn a_new_command_is_accepted_after_the_last_one_ended() {
    let mut rig = Rig::new();
    let mut vehicle = Vehicle::default();
    rig.upload(&example(), 0);
    rig.run(&mut vehicle, 10);
    assert_eq!(rig.status(), MissionStatus::Verified);
    rig.download(20);
    rig.run(&mut vehicle, 30);
    assert_eq!(rig.status(), MissionStatus::Complete);
    assert_eq!(rig.state.missions.runs, 2);
    assert_eq!(rig.state.missions.refused, 0);
    assert_eq!(rig.state.missions.started, Some(Duration::from_millis(20)));
    assert_eq!(rig.state.missions.sent, Vec::new());
}

#[test]
fn without_a_vehicle_nothing_is_sent_and_the_start_fails() {
    let mut rig = Rig::new();
    rig.state.link.vehicle = None;
    rig.upload(&example(), 0);
    assert!(rig.log.is_empty());
    assert_eq!(
        rig.failed(),
        (MissionFailure::NoVehicle, MissionPhase::Transfer)
    );
    assert_eq!(rig.state.missions.runs, 1);
    // A failed start leaves no operation running.
    rig.state.link.vehicle = Some(VehicleId {
        system_id: 1,
        component_id: 1,
        mav_type: "MAV_TYPE_GENERIC".into(),
        autopilot: "MAV_AUTOPILOT_GENERIC".into(),
    });
    rig.download(10);
    assert_eq!(rig.sent(), vec![s_list()]);
}

#[test]
fn a_list_too_long_for_a_count_is_not_sent() {
    let mut rig = Rig::new();
    let points = vec![example()[0]; usize::from(u16::MAX) + 1];
    rig.upload(&points, 0);
    assert!(rig.log.is_empty());
    assert_eq!(rig.failed().0, MissionFailure::CountTooLarge);
}

// ---- addressing

#[test]
fn a_message_not_from_the_vehicle_or_not_for_the_station_is_dropped() {
    let to = |system: u8, component: u8| {
        Message::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
            seq: 0,
            target_system: system,
            target_component: component,
            mission_type: PLAN,
        })
    };
    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    // Another system, another component.
    rig.deliver_from(2, 1, v_request(0), 10);
    rig.deliver_from(1, 2, v_request(0), 10);
    // Other targets.
    rig.deliver(to(7, 190), 10);
    rig.deliver(to(255, 7), 10);
    rig.deliver(to(1, 1), 10);
    // Another plan type.
    rig.deliver(
        Message::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
            seq: 0,
            target_system: 255,
            target_component: 190,
            mission_type: MavMissionType::MAV_MISSION_TYPE_FENCE,
        }),
        10,
    );
    // Not a decoded message.
    let frame = Frame {
        system_id: 1,
        component_id: 1,
        sequence: 0,
        message_id: 51,
        raw: Vec::new(),
        body: Body::Unknown,
    };
    rig.call(10, |s, cx| s.on_frame(&frame, cx));
    assert_eq!(rig.sent(), vec![s_count(3)]);
    // Target 0 and 0 is for everybody.
    rig.deliver(to(0, 0), 20);
    rig.deliver(to(255, 0), 21);
    rig.deliver(to(0, 190), 22);
    assert_eq!(rig.times("ITEM 0"), [20, 21, 22]);
}

#[test]
fn a_mission_message_with_no_operation_running_is_ignored() {
    let mut rig = Rig::new();
    rig.deliver(v_request(0), 0);
    rig.deliver(v_count(3, 1), 0);
    rig.deliver(v_ack(R::MAV_MISSION_DENIED, 0), 0);
    rig.deliver(v_item(0, example()[0]), 0);
    rig.tick_range(1, 10_000);
    assert!(rig.log.is_empty());
    assert_eq!(rig.state.missions, MissionsState::default());
}

#[test]
fn messages_that_belong_to_no_step_of_the_operation_are_ignored() {
    // A clear takes no request; an upload takes no count or item; a download takes no request.
    let mut rig = Rig::new();
    rig.clear(0);
    rig.deliver(v_request(0), 1);
    rig.deliver(v_count(3, 1), 1);
    rig.deliver(v_item(0, example()[0]), 1);
    assert_eq!(rig.sent(), vec![s_clear()]);

    let mut rig = Rig::new();
    rig.upload(&example(), 0);
    rig.deliver(v_count(3, 1), 1);
    rig.deliver(v_item(0, example()[0]), 1);
    assert_eq!(rig.sent(), vec![s_count(3)]);

    let mut rig = Rig::new();
    rig.download(0);
    rig.deliver(v_request(0), 1);
    assert_eq!(rig.sent(), vec![s_list()]);
}

#[test]
fn the_service_is_registered_and_the_commands_are_not_for_the_other_services() {
    let services = crate::services::default_services(&SessionConfig::default());
    assert!(!services.is_empty());
    let mut rig = Rig::new();
    rig.command(Command::RefreshParams, 0);
    rig.command(Command::StopLog, 0);
    assert!(rig.log.is_empty());
    assert_eq!(rig.status(), MissionStatus::Idle);
}
