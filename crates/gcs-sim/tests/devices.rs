//! `MARV_DEVICE_STATUS` of the simulated vehicle, read back with a hand-written frame reader (the
//! message is defined in the `marv-dialect` README; this crate does not use that crate).

use gcs_sim::{SimConfig, SimVehicle};

const ID: u32 = 42_421;
const CRC_EXTRA: u8 = 231;

struct Frame {
    id: u32,
    sequence: u8,
    /// The payload, padded with zeros to 30 bytes when the id is `ID`.
    payload: Vec<u8>,
    trimmed: usize,
    crc_ok: bool,
}

fn frames(bytes: &[u8]) -> Vec<Frame> {
    let mut out = Vec::new();
    let mut at = 0;
    while at < bytes.len() {
        assert_eq!(bytes[at], 0xFD, "frame start at {at}");
        let len = usize::from(bytes[at + 1]);
        let end = at + 10 + len;
        let id = u32::from_le_bytes([bytes[at + 7], bytes[at + 8], bytes[at + 9], 0]);
        let extra = if id == ID { CRC_EXTRA } else { 0 };
        let crc = u16::from_le_bytes([bytes[end], bytes[end + 1]]);
        let mut payload = bytes[at + 10..end].to_vec();
        let trimmed = payload.len();
        if id == ID {
            payload.resize(30, 0);
        }
        out.push(Frame {
            id,
            sequence: bytes[at + 4],
            payload,
            trimmed,
            crc_ok: id != ID || mavlink::calculate_crc(&bytes[at + 1..end], extra) == crc,
        });
        at = end + 2;
    }
    out
}

fn u32_at(f: &Frame, offset: usize) -> u32 {
    u32::from_le_bytes(f.payload[offset..offset + 4].try_into().unwrap())
}

fn run(config: SimConfig, seconds: u64) -> Vec<Frame> {
    let mut vehicle = SimVehicle::new(config);
    let mut bytes = Vec::new();
    for t in (0..=seconds * 1000).step_by(10) {
        bytes.extend(vehicle.step(t, &[]));
    }
    frames(&bytes)
}

#[test]
fn the_default_configuration_sends_no_device_status() {
    assert!(
        run(SimConfig::default(), 3)
            .iter()
            .all(|frame| frame.id != ID)
    );
}

#[test]
fn seven_devices_once_a_second_with_advancing_counters() {
    let frames = run(SimConfig::default().with_device_status(1.0), 3);
    let rows: Vec<_> = frames.iter().filter(|f| f.id == ID).collect();
    // Seconds 0, 1, 2 and 3: seven devices each.
    assert_eq!(rows.len(), 28);
    assert!(rows.iter().all(|f| f.crc_ok));
    let device = |f: &Frame| f.payload[26];
    assert_eq!(
        rows[..7].iter().map(|f| device(f)).collect::<Vec<_>>(),
        [1, 2, 3, 4, 5, 6, 7]
    );
    for f in &rows {
        assert_eq!(f.payload[27], 2, "state OK");
        assert_eq!(f.payload[28], 0, "no error");
        assert_eq!(f.payload[29], 0, "no flags");
        assert!(f.trimmed <= 30);
    }
    // IMU1: 1000 samples per second, IMU2: 640, read from time_boot_ms and samples.
    for (device_id, rate) in [(1u8, 1000u32), (2, 640), (3, 50), (7, 100)] {
        let own: Vec<_> = rows.iter().filter(|f| device(f) == device_id).collect();
        assert_eq!(own.len(), 4);
        for pair in own.windows(2) {
            assert_eq!(u32_at(pair[1], 0) - u32_at(pair[0], 0), 1000);
            assert_eq!(
                u32_at(pair[1], 4) - u32_at(pair[0], 4),
                rate,
                "device {device_id}"
            );
        }
    }
    // Sequence numbers run on across the messages of the vehicle.
    let sequences: Vec<u8> = frames.iter().map(|f| f.sequence).collect();
    assert!(sequences.windows(2).all(|w| w[1] == w[0].wrapping_add(1)));
}

#[test]
fn a_silent_vehicle_sends_no_device_status() {
    let mut vehicle = SimVehicle::new(SimConfig::default().with_device_status(1.0));
    vehicle.set_silent(true);
    let mut bytes = Vec::new();
    for t in (0..=2000).step_by(10) {
        bytes.extend(vehicle.step(t, &[]));
    }
    assert!(bytes.is_empty());
}
