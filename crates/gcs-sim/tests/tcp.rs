use std::io::Write;
use std::net::{Shutdown, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use gcs_sim::{SimConfig, SimVehicle, serve_tcp};
use mavlink::dialects::common::{HEARTBEAT_DATA, MavMessage, MavType};
use mavlink::{MAVLinkV2MessageRaw, MavHeader, MavlinkReader};

struct Server {
    addr: std::net::SocketAddr,
    vehicle: Arc<Mutex<SimVehicle>>,
    stop: Arc<AtomicBool>,
    thread: JoinHandle<()>,
}

fn start() -> Server {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    let vehicle = Arc::new(Mutex::new(SimVehicle::new(SimConfig::default())));
    let stop = Arc::new(AtomicBool::new(false));
    let thread = {
        let (vehicle, stop) = (vehicle.clone(), stop.clone());
        thread::spawn(move || serve_tcp(listener, vehicle, stop))
    };
    Server {
        addr,
        vehicle,
        stop,
        thread,
    }
}

/// Counts of HEARTBEAT and HIGHRES_IMU messages (25 Hz) decoded from a connection so far.
#[derive(Default)]
struct Seen {
    heartbeats: usize,
    imu: usize,
}

fn read_in_background(stream: &TcpStream) -> (Arc<Mutex<Seen>>, JoinHandle<()>) {
    let seen = Arc::new(Mutex::new(Seen::default()));
    let mut reader = MavlinkReader::new(stream.try_clone().unwrap());
    let thread = {
        let seen = seen.clone();
        thread::spawn(move || {
            while let Ok((_, message)) = reader.read_any_message::<MavMessage>() {
                let mut seen = seen.lock().unwrap();
                match message {
                    MavMessage::HEARTBEAT(_) => seen.heartbeats += 1,
                    MavMessage::HIGHRES_IMU(_) => seen.imu += 1,
                    _ => {}
                }
            }
        })
    };
    (seen, thread)
}

fn gcs_heartbeat() -> Vec<u8> {
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message(
        MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 0,
        },
        &MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            mavtype: MavType::MAV_TYPE_GCS,
            mavlink_version: 3,
            ..HEARTBEAT_DATA::default()
        }),
    );
    raw.raw_bytes().to_vec()
}

fn wait_until(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn streams_telemetry_in_real_time() {
    let server = start();
    let mut stream = TcpStream::connect(server.addr).unwrap();
    let (seen, reader) = read_in_background(&stream);
    thread::sleep(Duration::from_millis(2500));
    {
        let seen = seen.lock().unwrap();
        assert!(seen.heartbeats >= 2, "heartbeats: {}", seen.heartbeats);
        assert!(
            (56..=69).contains(&seen.imu),
            "HIGHRES_IMU: {} instead of about 62",
            seen.imu
        );
    }

    stream.write_all(&gcs_heartbeat()).unwrap();
    wait_until("the heartbeat to be counted", || {
        server.vehicle.lock().unwrap().heartbeats_received() == 1
    });
    assert_eq!(
        server.vehicle.lock().unwrap().last_heartbeat_from(),
        Some((255, 190))
    );

    stream.shutdown(Shutdown::Both).unwrap();
    reader.join().unwrap();
    server.stop.store(true, Ordering::Relaxed);
    server.thread.join().unwrap();
}

#[test]
fn next_client_gets_data_and_vehicle_keeps_state() {
    let server = start();

    let first = TcpStream::connect(server.addr).unwrap();
    let (seen, reader) = read_in_background(&first);
    wait_until("data for the first client", || seen.lock().unwrap().imu > 5);
    (&first).write_all(&gcs_heartbeat()).unwrap();
    wait_until("first heartbeat", || {
        server.vehicle.lock().unwrap().heartbeats_received() == 1
    });
    first.shutdown(Shutdown::Both).unwrap();
    reader.join().unwrap();

    let second = TcpStream::connect(server.addr).unwrap();
    let (seen, reader) = read_in_background(&second);
    wait_until("data for the second client", || {
        let seen = seen.lock().unwrap();
        seen.imu > 5 && seen.heartbeats >= 1
    });
    (&second).write_all(&gcs_heartbeat()).unwrap();
    wait_until("second heartbeat", || {
        server.vehicle.lock().unwrap().heartbeats_received() == 2
    });
    second.shutdown(Shutdown::Both).unwrap();
    reader.join().unwrap();

    server.stop.store(true, Ordering::Relaxed);
    server.thread.join().unwrap();
}

#[test]
fn stops_promptly_with_and_without_a_client() {
    let server = start();
    thread::sleep(Duration::from_millis(100));
    let before = Instant::now();
    server.stop.store(true, Ordering::Relaxed);
    server.thread.join().unwrap();
    assert!(before.elapsed() < Duration::from_millis(200));

    let server = start();
    let stream = TcpStream::connect(server.addr).unwrap();
    let (seen, reader) = read_in_background(&stream);
    wait_until("data", || seen.lock().unwrap().imu > 0);
    let before = Instant::now();
    server.stop.store(true, Ordering::Relaxed);
    server.thread.join().unwrap();
    assert!(before.elapsed() < Duration::from_millis(200));
    drop(stream);
    reader.join().unwrap();
}
