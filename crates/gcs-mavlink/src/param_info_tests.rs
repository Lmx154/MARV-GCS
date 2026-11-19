use super::*;

/// `fw_settings::TABLE` of the firmware, in its order. A new firmware parameter is a name to add
/// here and an entry to add to the table.
const FIRMWARE_NAMES: [&str; 79] = [
    "BARO_ODR_HZ",
    "BARO_OSR_P",
    "BARO_OSR_T",
    "BARO_IIR_P",
    "BARO_IIR_T",
    "MAG_ODR_HZ",
    "MAG_AVG",
    "MAG_OFFX",
    "MAG_OFFY",
    "MAG_OFFZ",
    "BATT_V_SCALE",
    "BATT_A_SCALE",
    "BATT_A_OFFSET",
    "IMU1_GYR_OFFX",
    "IMU1_GYR_OFFY",
    "IMU1_GYR_OFFZ",
    "IMU1_ACC_OFFX",
    "IMU1_ACC_OFFY",
    "IMU1_ACC_OFFZ",
    "IMU2_GYR_OFFX",
    "IMU2_GYR_OFFY",
    "IMU2_GYR_OFFZ",
    "IMU2_ACC_OFFX",
    "IMU2_ACC_OFFY",
    "IMU2_ACC_OFFZ",
    "LOG_BACKEND_TYPE",
    "RC1_MIN",
    "RC1_TRIM",
    "RC1_MAX",
    "RC1_REVERSED",
    "RC1_DZ",
    "RC2_MIN",
    "RC2_TRIM",
    "RC2_MAX",
    "RC2_REVERSED",
    "RC2_DZ",
    "RC3_MIN",
    "RC3_TRIM",
    "RC3_MAX",
    "RC3_REVERSED",
    "RC3_DZ",
    "RC4_MIN",
    "RC4_TRIM",
    "RC4_MAX",
    "RC4_REVERSED",
    "RC4_DZ",
    "RC5_MIN",
    "RC5_TRIM",
    "RC5_MAX",
    "RC5_REVERSED",
    "RC5_DZ",
    "RC6_MIN",
    "RC6_TRIM",
    "RC6_MAX",
    "RC6_REVERSED",
    "RC6_DZ",
    "RC7_MIN",
    "RC7_TRIM",
    "RC7_MAX",
    "RC7_REVERSED",
    "RC7_DZ",
    "RC8_MIN",
    "RC8_TRIM",
    "RC8_MAX",
    "RC8_REVERSED",
    "RC8_DZ",
    "RCMAP_ROLL",
    "RCMAP_PITCH",
    "RCMAP_THROTTLE",
    "RCMAP_YAW",
    "IMU1_ACC_FS",
    "IMU1_GYR_FS",
    "IMU1_ODR_HZ",
    "IMU2_ACC_FS",
    "IMU2_GYR_FS",
    "IMU2_ODR_HZ",
    "AHRS_ORIENTATION",
    "MAG_ORIENT",
    "MOT_POLES",
];

#[test]
fn the_table_has_exactly_the_firmwares_79_names() {
    assert_eq!(COUNT, 79);
    assert_eq!(FIRMWARE_NAMES.len(), COUNT);
    let mut have: Vec<&str> = entries().map(|i| i.name).collect();
    assert_eq!(have.len(), COUNT);
    let mut want = FIRMWARE_NAMES.to_vec();
    have.sort_unstable();
    want.sort_unstable();
    assert_eq!(have, want);
    want.dedup();
    assert_eq!(want.len(), COUNT, "no name twice");
}

#[test]
fn every_entry_has_a_one_line_description_and_a_range() {
    for info in entries() {
        assert!(info.name.len() <= 16, "{}", info.name);
        assert!(!info.description.is_empty(), "{}", info.name);
        assert!(!info.range.is_empty(), "{}", info.name);
        for text in [info.description, info.range] {
            assert!(!text.contains('\n'), "{}", info.name);
        }
        assert_ne!(info.unit, Some(""), "{}", info.name);
    }
}

#[test]
fn lookup_finds_an_entry_by_name_and_knows_no_other() {
    let baro = lookup("BARO_ODR_HZ").expect("known");
    assert_eq!(baro.unit, Some("Hz"));
    let rc = lookup("RC7_DZ").expect("known");
    assert_eq!(rc.unit, Some("us"));
    assert!(rc.description.contains('7'));
    assert_eq!(lookup("BARO_OSR_P").expect("known").unit, None);
    assert!(lookup("NOT_A_PARAM").is_none());
    assert!(lookup("").is_none());
}

#[test]
fn units_follow_the_firmware_readme() {
    let unit = |name: &str| lookup(name).and_then(|i| i.unit);
    assert_eq!(unit("MAG_OFFZ"), Some("uT"));
    assert_eq!(unit("IMU2_GYR_OFFX"), Some("rad/s"));
    assert_eq!(unit("IMU1_ACC_OFFY"), Some("m/s^2"));
    assert_eq!(unit("IMU1_ACC_FS"), Some("g"));
    assert_eq!(unit("IMU2_GYR_FS"), Some("dps"));
    assert_eq!(unit("BATT_A_SCALE"), Some("V/A"));
    assert_eq!(unit("BATT_A_OFFSET"), Some("V"));
    assert_eq!(unit("RC3_TRIM"), Some("us"));
    for name in [
        "AHRS_ORIENTATION",
        "MAG_ORIENT",
        "LOG_BACKEND_TYPE",
        "RCMAP_YAW",
        "RC2_REVERSED",
        "MAG_AVG",
        "BATT_V_SCALE",
    ] {
        assert_eq!(unit(name), None, "{name}");
    }
}
