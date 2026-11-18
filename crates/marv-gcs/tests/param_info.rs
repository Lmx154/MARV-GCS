//! The station's parameter table against the simulated vehicle.

use gcs_mavlink::param_info;

/// The simulator's own parameters that are not the firmware's: the link contract's mock set
/// (`gcs_sim::default_params`), which no firmware parameter is called.
const SIM_ONLY: [&str; 13] = [
    "IMU_RATE_HZ",
    "IMU_ACC_RANGE",
    "IMU_GYR_LPF_HZ",
    "HIGHG_RANGE",
    "BARO_RATE_HZ",
    "GPS_RATE_HZ",
    "LOG_ENABLE",
    "LOG_RATE_HZ",
    "BAT_V_SCALE",
    "BAT_LOW_V",
    "SYS_TRIM",
    "SYS_UID",
    "NAV_TEST_16CHARS",
];

#[test]
fn every_firmware_parameter_the_simulator_reports_has_an_entry() {
    let params = gcs_sim::default_params();
    let mut known = 0;
    for p in &params {
        if SIM_ONLY.contains(&p.name.as_str()) {
            assert!(param_info::lookup(&p.name).is_none(), "{}", p.name);
        } else {
            assert!(param_info::lookup(&p.name).is_some(), "{}", p.name);
            known += 1;
        }
    }
    assert_eq!(params.len(), SIM_ONLY.len() + known);
    assert_eq!(known, 44 + 13);
}
