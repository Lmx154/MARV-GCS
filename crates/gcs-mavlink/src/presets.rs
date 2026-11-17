//! Sensor presets of the MARV V2 flight controller: a station feature (`docs/link-contract.md`
//! section 13). A preset is the station setting several parameters at once, as a verified
//! combination; the firmware knows no preset.
//!
//! One table per sensor (IMU1, IMU2, BARO, MAG): the parameters a preset sets and, for each preset,
//! its values, its summary, and the physical values and 0 to 100 scores of its radar chart axes.
//! Data only: nothing here maps values to a driver configuration or checks them.

mod tables;

pub use tables::{BARO, IMU1, IMU2, MAG};

/// The four sensors in device order (IMU1, IMU2, BARO, MAG).
pub static SENSORS: [&Sensor; 4] = [&IMU1, &IMU2, &BARO, &MAG];

/// `MARV_DEVICE_IMU1` of the `marv` dialect.
pub const MARV_DEVICE_IMU1: u8 = 1;
/// `MARV_DEVICE_IMU2` of the `marv` dialect.
pub const MARV_DEVICE_IMU2: u8 = 2;
/// `MARV_DEVICE_BARO` of the `marv` dialect.
pub const MARV_DEVICE_BARO: u8 = 3;
/// `MARV_DEVICE_MAG` of the `marv` dialect.
pub const MARV_DEVICE_MAG: u8 = 4;

/// A parameter value, typed as the firmware's parameters are.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Value {
    I32(i32),
    F32(f32),
}

/// How an axis value was obtained.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Computed from datasheet and driver numbers.
    Plain,
    /// Scaled from a datasheet point by a stated model; shown as MODEL.
    Model,
    /// An upper bound, not a measurement; shown as BOUND.
    Bound,
}

/// One axis of the radar chart. The score is
/// `round_half_up(100 * clamp(ln(x / x0) / ln(x100 / x0), 0, 1))`: x0 scores 0, x100 scores 100.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Axis {
    pub name: &'static str,
    pub unit: &'static str,
    /// Unit of the second value of this axis (`Preset::physical2`), if it has one.
    pub unit2: Option<&'static str>,
    pub x0: f32,
    pub x100: f32,
    pub kind: Kind,
}

/// One preset. `values` follow `Sensor::fields`; `physical` and `scores` follow `Sensor::axes`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Preset {
    pub id: i32,
    pub name: &'static str,
    pub summary: &'static str,
    pub values: &'static [Value],
    pub physical: &'static [f32],
    /// Second physical value per axis, `Some` exactly where `Axis::unit2` is `Some`.
    pub physical2: &'static [Option<f32>],
    pub scores: &'static [u8],
}

/// The presets of one sensor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sensor {
    /// `MARV_DEVICE_*` number of the device.
    pub device: u8,
    /// Names of the parameters a preset sets, in the order of `Preset::values`.
    pub fields: &'static [&'static str],
    pub default_id: i32,
    pub axes: &'static [Axis],
    pub presets: &'static [Preset],
}

/// The preset of `sensor` with this id, if there is one.
pub fn preset(sensor: &Sensor, id: i32) -> Option<&Preset> {
    sensor.presets.iter().find(|p| p.id == id)
}
