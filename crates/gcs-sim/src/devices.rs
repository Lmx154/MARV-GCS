//! `MARV_DEVICE_STATUS` of the simulated vehicle (`docs/link-contract.md`, section 8), written from
//! the message's definition in the `marv-dialect` README and not from the crate: id 42421, CRC_EXTRA
//! 231, a payload of 30 bytes (`u32` x 4, `u16` x 5, `u8` x 4, little endian, in that order).
//!
//! One message per device, once per second of the stream, with counters that advance: `samples` is
//! the device's rate times the time since boot, so a station that computes samples per second from
//! two rows reads that rate back.

/// The message id.
pub const MESSAGE_ID: u32 = 42_421;
/// The checksum seed of the message.
pub const CRC_EXTRA: u8 = 231;

/// `MARV_DEVICE` values.
const IMU1: u8 = 1;
const IMU2: u8 = 2;
const BARO: u8 = 3;
const MAG: u8 = 4;
const GNSS: u8 = 5;
const BATT: u8 = 6;
const RC: u8 = 7;
const SD: u8 = 9;
const ESC: u8 = 12;
/// `MARV_DEVICE_STATE_OK` and `MARV_DEVICE_ERROR_NONE`.
const STATE_OK: u8 = 2;
const ERROR_NONE: u8 = 0;

/// The devices of the simulated vehicle and the samples per second each delivers.
pub const DEVICES: [(u8, u32); 9] = [
    (IMU1, 1000),
    (IMU2, 640),
    (BARO, 50),
    (MAG, 100),
    (GNSS, 10),
    (BATT, 10),
    (RC, 100),
    (SD, 1),
    (ESC, 1000),
];

/// The payload of one device at `now_ms` after boot (not yet cut of its trailing zeros).
pub fn payload(device: u8, samples_per_second: u32, now_ms: u64) -> [u8; 30] {
    let samples = now_ms * u64::from(samples_per_second) / 1000;
    // The last sample is about half a period old.
    let age_ms = 500 / samples_per_second;
    let mut out = [0u8; 30];
    out[0..4].copy_from_slice(&(now_ms as u32).to_le_bytes());
    out[4..8].copy_from_slice(&(samples as u32).to_le_bytes());
    // errors, timeouts: 0 (bytes 8..16)
    out[16..18].copy_from_slice(&(age_ms as u16).to_le_bytes());
    // recoveries, attempts, overflows, config_restored: 0 (bytes 18..26)
    out[26] = device;
    out[27] = STATE_OK;
    out[28] = ERROR_NONE;
    // flags: 0 (byte 29)
    out
}

/// The payloads of every device at `now_ms`, in `device_id` order.
pub fn payloads(now_ms: u64) -> Vec<[u8; 30]> {
    DEVICES
        .iter()
        .map(|&(device, rate)| payload(device, rate, now_ms))
        .collect()
}
