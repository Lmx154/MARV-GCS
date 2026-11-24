use super::*;
use crate::VehicleState;
use crate::dialect::HEARTBEAT_DATA;

const OWN: u8 = 255;

struct Rig {
    service: Heartbeat,
    state: VehicleState,
    out: Vec<Message>,
    /// `Ctx::changed` after the last call.
    changed: bool,
}

impl Rig {
    fn new(timeout: Duration) -> Self {
        let config = SessionConfig {
            system_id: OWN,
            heartbeat_timeout: timeout,
            ..SessionConfig::default()
        };
        Self {
            service: Heartbeat::new(&config),
            state: VehicleState::default(),
            out: Vec::new(),
            changed: false,
        }
    }

    fn tick(&mut self, now_ms: u64) {
        let mut cx = Ctx {
            now: Duration::from_millis(now_ms),
            wall_time_us: 0,
            state: &mut self.state,
            out: &mut self.out,
            changed: false,
        };
        self.service.on_tick(&mut cx);
        self.changed = cx.changed;
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

    fn status(&self) -> &LinkStatus {
        &self.state.link.status
    }
}

fn heartbeat_frame(system_id: u8, component_id: u8, mavtype: MavType) -> Frame {
    Frame {
        system_id,
        component_id,
        sequence: 0,
        message_id: 0,
        raw: Vec::new(),
        body: Body::Known(Message::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype,
            autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        })),
    }
}

fn vehicle_frame(system_id: u8) -> Frame {
    heartbeat_frame(system_id, 1, MavType::MAV_TYPE_QUADROTOR)
}

#[test]
fn sends_first_on_first_tick_then_every_second() {
    let mut rig = Rig::new(Duration::from_secs(3));
    let mut sent_at = Vec::new();
    for step in 0..=500u64 {
        let now = step * 20;
        let before = rig.out.len();
        rig.tick(now);
        for _ in before..rig.out.len() {
            sent_at.push(now);
        }
    }
    assert!(
        (10..=11).contains(&sent_at.len()),
        "heartbeats: {}",
        sent_at.len()
    );
    assert_eq!(sent_at[0], 0, "first one on the first tick");
    for pair in sent_at.windows(2) {
        assert_eq!(pair[1] - pair[0], 1000);
    }
}

#[test]
fn heartbeat_content() {
    let mut rig = Rig::new(Duration::from_secs(3));
    rig.tick(12345);
    assert_eq!(rig.out.len(), 1);
    let Message::HEARTBEAT(data) = &rig.out[0] else {
        panic!("not a heartbeat: {:?}", rig.out[0]);
    };
    assert_eq!(data.mavtype, MavType::MAV_TYPE_GCS);
    assert_eq!(data.autopilot, MavAutopilot::MAV_AUTOPILOT_INVALID);
    assert_eq!(data.base_mode, MavModeFlag::empty());
    assert_eq!(data.system_status, MavState::MAV_STATE_ACTIVE);
    assert_eq!(data.mavlink_version, 3);
}

#[test]
fn late_ticks_do_not_burst() {
    let mut rig = Rig::new(Duration::from_secs(3));
    rig.tick(0);
    rig.tick(5000);
    rig.tick(5020);
    assert_eq!(
        rig.out.len(),
        2,
        "one per call at most, none for missed seconds"
    );
    rig.tick(5999);
    assert_eq!(rig.out.len(), 2);
    rig.tick(6000);
    assert_eq!(rig.out.len(), 3);
}

#[test]
fn starts_connecting() {
    let rig = Rig::new(Duration::from_secs(3));
    assert_eq!(rig.status(), &LinkStatus::Connecting);
    assert_eq!(rig.state.link.vehicle, None);
}

#[test]
fn vehicle_heartbeat_connects_and_fills_vehicle() {
    let mut rig = Rig::new(Duration::from_secs(3));
    let frame = heartbeat_frame(7, 9, MavType::MAV_TYPE_GENERIC);
    rig.frame(&frame, 1500);
    assert_eq!(rig.status(), &LinkStatus::Connected);
    assert_eq!(
        rig.state.link.vehicle,
        Some(VehicleId {
            system_id: 7,
            component_id: 9,
            mav_type: "MAV_TYPE_GENERIC".into(),
            autopilot: "MAV_AUTOPILOT_ARDUPILOTMEGA".into(),
        })
    );
    assert_eq!(
        rig.state.link.last_heartbeat,
        Some(Duration::from_millis(1500))
    );
}

#[test]
fn same_system_refreshes_other_systems_and_ground_stations_do_not_count() {
    let mut rig = Rig::new(Duration::from_secs(3));
    rig.frame(&vehicle_frame(1), 100);
    let vehicle = rig.state.link.vehicle.clone();

    rig.frame(&heartbeat_frame(1, 5, MavType::MAV_TYPE_CAMERA), 200);
    assert_eq!(
        rig.state.link.last_heartbeat,
        Some(Duration::from_millis(200))
    );
    assert_eq!(
        rig.state.link.vehicle, vehicle,
        "other component of the vehicle"
    );

    rig.frame(&vehicle_frame(2), 300);
    rig.frame(&heartbeat_frame(1, 1, MavType::MAV_TYPE_GCS), 400);
    rig.frame(&vehicle_frame(OWN), 500);
    assert_eq!(
        rig.state.link.last_heartbeat,
        Some(Duration::from_millis(200))
    );
    assert_eq!(rig.state.link.vehicle, vehicle);
}

#[test]
fn own_and_ground_station_heartbeats_do_not_connect() {
    let mut rig = Rig::new(Duration::from_secs(3));
    rig.frame(&vehicle_frame(OWN), 100);
    rig.frame(&heartbeat_frame(3, 1, MavType::MAV_TYPE_GCS), 200);
    assert_eq!(rig.status(), &LinkStatus::Connecting);
    assert_eq!(rig.state.link.vehicle, None);
    assert_eq!(rig.state.link.last_heartbeat, None);
}

#[test]
fn other_bodies_are_ignored() {
    let mut rig = Rig::new(Duration::from_secs(3));
    let mut frame = vehicle_frame(1);
    frame.body = Body::Unknown;
    rig.frame(&frame, 100);
    frame.body = Body::Undecodable("x".into());
    rig.frame(&frame, 100);
    assert_eq!(rig.status(), &LinkStatus::Connecting);
}

#[test]
fn lost_after_timeout_and_connected_again() {
    let mut rig = Rig::new(Duration::from_secs(3));
    rig.frame(&vehicle_frame(1), 1000);
    rig.tick(4000);
    assert_eq!(
        rig.status(),
        &LinkStatus::Connected,
        "exactly the timeout is not over it"
    );
    rig.tick(4001);
    assert_eq!(rig.status(), &LinkStatus::Lost);
    assert!(rig.state.link.vehicle.is_some(), "vehicle kept");
    rig.frame(&vehicle_frame(1), 5000);
    assert_eq!(rig.status(), &LinkStatus::Connected);
    assert_eq!(
        rig.state.link.last_heartbeat,
        Some(Duration::from_millis(5000))
    );
    rig.tick(7999);
    assert_eq!(rig.status(), &LinkStatus::Connected);
}

#[test]
fn never_changes_closed() {
    let mut rig = Rig::new(Duration::from_secs(3));
    rig.frame(&vehicle_frame(1), 1000);
    let closed = LinkStatus::Closed("cable pulled".into());
    rig.state.link.status = closed.clone();
    rig.frame(&vehicle_frame(1), 1100);
    assert_eq!(rig.status(), &closed);
    rig.tick(60_000);
    assert_eq!(rig.status(), &closed);
}

#[test]
fn changed_is_false_when_a_tick_changes_nothing() {
    let mut rig = Rig::new(Duration::from_secs(3));
    rig.tick(0);
    assert!(!rig.changed, "sending the heartbeat is not a state change");
    rig.tick(20);
    assert!(!rig.changed);
    rig.frame(&vehicle_frame(1), 100);
    rig.tick(200);
    assert!(!rig.changed, "connected and within the timeout");
}

#[test]
fn changed_is_set_when_the_status_turns_lost() {
    let mut rig = Rig::new(Duration::from_secs(3));
    rig.frame(&vehicle_frame(1), 1000);
    rig.tick(4000);
    assert!(!rig.changed);
    rig.tick(4001);
    assert_eq!(rig.status(), &LinkStatus::Lost);
    assert!(rig.changed);
    rig.tick(4021);
    assert!(!rig.changed, "already lost");
}

#[test]
fn changed_is_set_by_a_vehicle_heartbeat_and_not_by_an_ignored_one() {
    let mut rig = Rig::new(Duration::from_secs(3));
    rig.frame(&vehicle_frame(1), 100);
    assert!(rig.changed, "vehicle filled, connected");
    rig.frame(&vehicle_frame(1), 200);
    assert!(rig.changed, "last_heartbeat updated");

    for (frame, name) in [
        (vehicle_frame(2), "other system"),
        (
            heartbeat_frame(1, 1, MavType::MAV_TYPE_GCS),
            "ground station",
        ),
        (vehicle_frame(OWN), "own system id"),
    ] {
        rig.frame(&frame, 300);
        assert!(!rig.changed, "{name}");
    }
    let mut other = vehicle_frame(1);
    other.body = Body::Unknown;
    rig.frame(&other, 400);
    assert!(!rig.changed, "not a heartbeat");

    rig.state.link.status = LinkStatus::Closed("x".into());
    rig.frame(&vehicle_frame(1), 500);
    assert!(!rig.changed, "closed");
}

#[test]
fn the_vehicles_custom_mode_is_kept_and_a_ground_station_heartbeat_does_not_set_it() {
    let mut rig = Rig::new(Duration::from_secs(3));
    assert_eq!(rig.state.flight.mode_code, None);
    let Frame {
        body: Body::Known(Message::HEARTBEAT(mut data)),
        ..
    } = vehicle_frame(1)
    else {
        panic!("a heartbeat");
    };
    data.custom_mode = 3;
    let mut frame = vehicle_frame(1);
    frame.body = Body::Known(Message::HEARTBEAT(data));
    rig.frame(&frame, 100);
    assert_eq!(rig.state.flight.mode_code, Some(3));
    rig.frame(&heartbeat_frame(1, 1, MavType::MAV_TYPE_GCS), 200);
    assert_eq!(rig.state.flight.mode_code, Some(3));
}
