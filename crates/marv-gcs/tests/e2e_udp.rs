//! End to end over the UDP link: the real transport, MAVLink layer and session against a peer that
//! plays a vehicle with hand-built heartbeat frames. `gcs-sim` only listens on TCP and is not used.

use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use gcs_mavlink::dialect::{HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavState, MavType};
use gcs_mavlink::{Body, Deframer, Encoder, LinkStatus, Session, SessionConfig, dialect};
use gcs_transport::{Endpoint, open};

struct Rig {
    session: Session,
    bind: SocketAddr,
    vehicle: UdpSocket,
    encoder: Encoder,
    deframer: Deframer,
    /// Systems and components of every heartbeat that reached the vehicle.
    heartbeats: Vec<(u8, u8)>,
}

impl Rig {
    fn start() -> Rig {
        let bind = UdpSocket::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap();
        let link = open(&Endpoint::Udp { bind }).expect("open the UDP link");
        let session = Session::start(
            link.reader,
            link.writer,
            SessionConfig::default(),
            Box::new(|| {}),
        );
        let vehicle = UdpSocket::bind("127.0.0.1:0").unwrap();
        vehicle
            .set_read_timeout(Some(Duration::from_millis(50)))
            .unwrap();
        Rig {
            session,
            bind,
            vehicle,
            encoder: Encoder::new(1, 1),
            deframer: Deframer::new(),
            heartbeats: Vec::new(),
        }
    }

    fn status(&self) -> LinkStatus {
        self.session.state().link.status.clone()
    }

    fn send_heartbeat(&mut self) {
        let message = dialect::MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        });
        let bytes = self.encoder.encode(&message);
        self.vehicle.send_to(&bytes, self.bind).unwrap();
    }

    /// Collects what the station sent to the vehicle for up to `span`.
    fn listen(&mut self, span: Duration) {
        let end = Instant::now() + span;
        let mut buf = [0u8; 2048];
        while Instant::now() < end {
            if let Ok((n, _)) = self.vehicle.recv_from(&mut buf) {
                let mut frames = Vec::new();
                self.deframer.push(&buf[..n], &mut frames);
                for frame in frames {
                    if matches!(frame.body, Body::Known(dialect::MavMessage::HEARTBEAT(_))) {
                        self.heartbeats.push((frame.system_id, frame.component_id));
                    }
                }
            }
        }
    }

    /// Sends a vehicle heartbeat every 100 ms until `done` holds or `limit` passes.
    fn until(&mut self, limit: Duration, done: impl Fn(&Rig) -> bool) -> bool {
        let end = Instant::now() + limit;
        while Instant::now() < end {
            self.send_heartbeat();
            self.listen(Duration::from_millis(100));
            if done(self) {
                return true;
            }
        }
        false
    }
}

#[test]
fn the_station_heartbeats_reach_the_peer_and_the_vehicle_is_connected() {
    let mut rig = Rig::start();
    assert!(
        rig.until(Duration::from_secs(5), |r| r.status()
            == LinkStatus::Connected
            && !r.heartbeats.is_empty()),
        "status {:?}, heartbeats seen by the vehicle {:?}",
        rig.status(),
        rig.heartbeats
    );
    let vehicle = rig
        .session
        .state()
        .link
        .vehicle
        .clone()
        .expect("vehicle identity");
    assert_eq!((vehicle.system_id, vehicle.component_id), (1, 1));
    assert_eq!(vehicle.mav_type, "MAV_TYPE_QUADROTOR");
    assert_eq!(rig.heartbeats[0], (255, 190));
}

#[test]
fn nothing_is_sent_to_a_vehicle_that_has_not_spoken_yet() {
    let mut rig = Rig::start();
    rig.listen(Duration::from_millis(1500));
    assert!(rig.heartbeats.is_empty(), "{:?}", rig.heartbeats);
    assert_eq!(rig.status(), LinkStatus::Connecting);
}

/// Negative control: if the reader returned `Ok(0)` for an empty datagram the session would close.
#[test]
fn empty_datagrams_do_not_close_the_session() {
    let mut rig = Rig::start();
    for _ in 0..3 {
        rig.vehicle.send_to(&[], rig.bind).unwrap();
    }
    std::thread::sleep(Duration::from_millis(300));
    assert!(
        !matches!(rig.status(), LinkStatus::Closed(_)),
        "{:?}",
        rig.status()
    );
    assert!(
        rig.until(Duration::from_secs(5), |r| r.status()
            == LinkStatus::Connected),
        "status {:?}",
        rig.status()
    );
    rig.vehicle.send_to(&[], rig.bind).unwrap();
    rig.listen(Duration::from_millis(300));
    assert_eq!(rig.status(), LinkStatus::Connected);
}
