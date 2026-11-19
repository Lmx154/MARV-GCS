//! What each parameter of the MARV V2 firmware means: a one-line description, the unit and the
//! allowed values in words. A station feature like `presets`: the firmware's parameter definitions
//! carry no description and MAVLink's PARAM_VALUE no metadata (`docs/link-contract.md`), so the
//! station holds this table, keyed by name. A name that is not here has no entry: callers show
//! nothing extra for it.
//!
//! Source: the firmware's `fw-settings` tables (names, ranges) and its README's parameter tables
//! (descriptions, units); `AHRS_ORIENTATION` and `MAG_ORIENT` from the orientation decision, `MOT_POLES` from the ESC telemetry decision.

/// What the station knows about one parameter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParamInfo {
    pub name: &'static str,
    /// One line, in plain words.
    pub description: &'static str,
    /// The unit of the value; none for a count, a choice or a gain.
    pub unit: Option<&'static str>,
    /// The allowed values or range, in words.
    pub range: &'static str,
}

/// The number of parameters of the firmware (`fw_settings::COUNT`).
pub const COUNT: usize = 79;

const fn p(
    name: &'static str,
    description: &'static str,
    unit: &'static str,
    range: &'static str,
) -> ParamInfo {
    ParamInfo {
        name,
        description,
        unit: if unit.is_empty() { None } else { Some(unit) },
        range,
    }
}

const ROTATION: &str = "0 to 15, ArduPilot's rotations: 0 none, 1 to 7 yaw 45 to 315 degrees in steps of 45, 8 roll 180, 9 to 11 roll 180 with yaw 45, 90 or 135, 12 pitch 180, 13 to 15 roll 180 with yaw 225, 270 or 315";
const GYR_OFF_RANGE: &str = "-34.9066 to 34.9066 (the gyros' full scale, 2000 dps)";
const ACC_OFF_RANGE: &str = "-9.80665 to 9.80665 (one gravity)";
const MAG_OFF_RANGE: &str = "-200 to 200";
const IMU_ODR_RANGE: &str = "25, 50, 100, 200, 400 or 800";
const OSR_RANGE: &str = "1, 2, 4, 8, 16, 32, 64 or 128";
const IIR_RANGE: &str = "0 (off), 1, 3, 7, 15, 31, 63 or 127";

/// Every parameter except the RC channels' (see [`RC`]), in the firmware's order.
pub static GENERAL: [ParamInfo; 39] = [
    p(
        "BARO_ODR_HZ",
        "Barometer output rate; the chip runs at its nearest rate (50 is 50.056 Hz).",
        "Hz",
        "1 to 240: 1, 2, 3, 4, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 100, 120, 130, 140, 160, 180, 200, 220 or 240",
    ),
    p(
        "BARO_OSR_P",
        "Barometer pressure oversampling: more samples per reading, less noise.",
        "",
        OSR_RANGE,
    ),
    p(
        "BARO_OSR_T",
        "Barometer temperature oversampling: more samples per reading, less noise.",
        "",
        OSR_RANGE,
    ),
    p(
        "BARO_IIR_P",
        "Barometer pressure IIR filter coefficient; 0 is no filter.",
        "",
        IIR_RANGE,
    ),
    p(
        "BARO_IIR_T",
        "Barometer temperature IIR filter coefficient; 0 is no filter.",
        "",
        IIR_RANGE,
    ),
    p(
        "MAG_ODR_HZ",
        "Compass output rate.",
        "Hz",
        "400, 200, 100, 50, 25, 12.5, 6.25, 3.125 or 1.5625",
    ),
    p(
        "MAG_AVG",
        "Compass samples averaged into each output sample.",
        "",
        "1, 2, 4 or 8; the rate limits it: at most 1 at 400 Hz, 2 at 200 Hz, 4 at 100 Hz, 8 below",
    ),
    p(
        "MAG_OFFX",
        "Compass offset of the X axis, subtracted from every sample in the body frame; set by the compass calibration.",
        "uT",
        MAG_OFF_RANGE,
    ),
    p(
        "MAG_OFFY",
        "Compass offset of the Y axis, subtracted from every sample in the body frame; set by the compass calibration.",
        "uT",
        MAG_OFF_RANGE,
    ),
    p(
        "MAG_OFFZ",
        "Compass offset of the Z axis, subtracted from every sample in the body frame; set by the compass calibration.",
        "uT",
        MAG_OFF_RANGE,
    ),
    p(
        "BATT_V_SCALE",
        "Gain on the battery voltage: 1.0 is the divider's nominal ratio, 2.0 doubles the reported voltage.",
        "",
        "0.1 to 10",
    ),
    p(
        "BATT_A_SCALE",
        "Volts at the ADC pin per ampere of the ESC's current output; the default is a bench placeholder, not calibrated; 0 reads 0 A.",
        "V/A",
        "0 to 10",
    ),
    p(
        "BATT_A_OFFSET",
        "Volts at the ADC pin with no current, subtracted before the scale.",
        "V",
        "-3.3 to 3.3",
    ),
    p(
        "IMU1_GYR_OFFX",
        "IMU1 gyro offset of the X axis, subtracted from every sample; set by the gyro calibration.",
        "rad/s",
        GYR_OFF_RANGE,
    ),
    p(
        "IMU1_GYR_OFFY",
        "IMU1 gyro offset of the Y axis, subtracted from every sample; set by the gyro calibration.",
        "rad/s",
        GYR_OFF_RANGE,
    ),
    p(
        "IMU1_GYR_OFFZ",
        "IMU1 gyro offset of the Z axis, subtracted from every sample; set by the gyro calibration.",
        "rad/s",
        GYR_OFF_RANGE,
    ),
    p(
        "IMU1_ACC_OFFX",
        "IMU1 accelerometer offset of the X axis, subtracted from every sample; set by the accelerometer calibration.",
        "m/s^2",
        ACC_OFF_RANGE,
    ),
    p(
        "IMU1_ACC_OFFY",
        "IMU1 accelerometer offset of the Y axis, subtracted from every sample; set by the accelerometer calibration.",
        "m/s^2",
        ACC_OFF_RANGE,
    ),
    p(
        "IMU1_ACC_OFFZ",
        "IMU1 accelerometer offset of the Z axis, subtracted from every sample; set by the accelerometer calibration.",
        "m/s^2",
        ACC_OFF_RANGE,
    ),
    p(
        "IMU2_GYR_OFFX",
        "IMU2 gyro offset of the X axis, subtracted from every sample; set by the gyro calibration.",
        "rad/s",
        GYR_OFF_RANGE,
    ),
    p(
        "IMU2_GYR_OFFY",
        "IMU2 gyro offset of the Y axis, subtracted from every sample; set by the gyro calibration.",
        "rad/s",
        GYR_OFF_RANGE,
    ),
    p(
        "IMU2_GYR_OFFZ",
        "IMU2 gyro offset of the Z axis, subtracted from every sample; set by the gyro calibration.",
        "rad/s",
        GYR_OFF_RANGE,
    ),
    p(
        "IMU2_ACC_OFFX",
        "IMU2 accelerometer offset of the X axis, subtracted from every sample; set by the accelerometer calibration.",
        "m/s^2",
        ACC_OFF_RANGE,
    ),
    p(
        "IMU2_ACC_OFFY",
        "IMU2 accelerometer offset of the Y axis, subtracted from every sample; set by the accelerometer calibration.",
        "m/s^2",
        ACC_OFF_RANGE,
    ),
    p(
        "IMU2_ACC_OFFZ",
        "IMU2 accelerometer offset of the Z axis, subtracted from every sample; set by the accelerometer calibration.",
        "m/s^2",
        ACC_OFF_RANGE,
    ),
    p(
        "LOG_BACKEND_TYPE",
        "Where the flight log is written; read at boot, so a change applies after save and reboot.",
        "",
        "0 no logging (the SD card is not touched), 1 a file on the SD card",
    ),
    p("RCMAP_ROLL", "The RC channel that is roll.", "", "1 to 8"),
    p("RCMAP_PITCH", "The RC channel that is pitch.", "", "1 to 8"),
    p(
        "RCMAP_THROTTLE",
        "The RC channel that is throttle.",
        "",
        "1 to 8",
    ),
    p("RCMAP_YAW", "The RC channel that is yaw.", "", "1 to 8"),
    p(
        "IMU1_ACC_FS",
        "IMU1 accelerometer full scale; applies after save and reboot.",
        "g",
        "2, 4, 8, 16 or 32",
    ),
    p(
        "IMU1_GYR_FS",
        "IMU1 gyroscope full scale; applies after save and reboot.",
        "dps",
        "125, 250, 500, 1000, 2000 or 4000",
    ),
    p(
        "IMU1_ODR_HZ",
        "IMU1 output rate of both sensors; applies after save and reboot.",
        "Hz",
        IMU_ODR_RANGE,
    ),
    p(
        "IMU2_ACC_FS",
        "IMU2 low-g accelerometer full scale; applies after save and reboot.",
        "g",
        "2, 4, 8 or 16",
    ),
    p(
        "IMU2_GYR_FS",
        "IMU2 gyroscope full scale; applies after save and reboot.",
        "dps",
        "250, 500, 1000, 2000 or 4000",
    ),
    p(
        "IMU2_ODR_HZ",
        "IMU2 output rate of both sensors; applies after save and reboot.",
        "Hz",
        IMU_ODR_RANGE,
    ),
    p(
        "AHRS_ORIENTATION",
        "Board orientation: the rotation from the board's frame to the vehicle's, applied to both IMUs after their offsets; applies after save and reboot.",
        "",
        ROTATION,
    ),
    p(
        "MAG_ORIENT",
        "Compass orientation: the rotation from the compass chip's axes to the vehicle's, applied before the offsets; 10 is the BMM350 mounted +X right, +Y forward, +Z up.",
        "",
        ROTATION,
    ),
    p(
        "MOT_POLES",
        "Magnets per motor, used to turn the ESC's electrical speed into rpm.",
        "",
        "2 to 64",
    ),
];

/// The five parameters of each RC channel 1 to 8: low end, centre, high end, reversed, dead zone.
macro_rules! rc_channels {
    ($($n:literal),*) => {
        [$(
            p(
                concat!("RC", $n, "_MIN"),
                concat!("RC channel ", $n, " value at the low end of its travel."),
                "us",
                "800 to 2200",
            ),
            p(
                concat!("RC", $n, "_TRIM"),
                concat!("RC channel ", $n, " centre."),
                "us",
                "800 to 2200",
            ),
            p(
                concat!("RC", $n, "_MAX"),
                concat!("RC channel ", $n, " value at the high end of its travel."),
                "us",
                "800 to 2200",
            ),
            p(
                concat!("RC", $n, "_REVERSED"),
                concat!("Reverses RC channel ", $n, ": the angle is negated, the throttle mirrored between MIN and MAX."),
                "",
                "0 normal, 1 reversed",
            ),
            p(
                concat!("RC", $n, "_DZ"),
                concat!("Dead zone of RC channel ", $n, ": around TRIM for roll, pitch and yaw, above MIN for throttle."),
                "us",
                "0 to 200",
            ),
        )*]
    };
}

/// The RC channels' parameters, in the firmware's order.
pub static RC: [ParamInfo; 40] = rc_channels!("1", "2", "3", "4", "5", "6", "7", "8");

/// Every entry, [`GENERAL`] then [`RC`].
pub fn entries() -> impl Iterator<Item = &'static ParamInfo> {
    GENERAL.iter().chain(RC.iter())
}

/// The entry of a parameter, or `None` for a name the table does not know.
pub fn lookup(name: &str) -> Option<&'static ParamInfo> {
    entries().find(|info| info.name == name)
}

#[cfg(test)]
#[path = "param_info_tests.rs"]
mod tests;
