//! The write order of an Apply (`docs/link-contract.md` section 13): every field whose value goes down
//! first, then every field whose value goes up. For every pair of presets of BARO and of MAG, every
//! intermediate combination in that order is valid, judged by an oracle written here from the
//! datasheets (`_plan/facts-sensor-presets.md`, which cites them):
//! BMP581 Table 7 (datasheet text lines 849-865), the highest ODR for each OSR_P and OSR_T;
//! BMM350 Table 5 (datasheet text lines 712-730), the highest averaging for each ODR.

use std::collections::BTreeMap;

use gcs_mavlink::presets::{BARO, MAG, Sensor, Value};
use gcs_mavlink::services::params::{Param, ParamTable, ParamType};
use marv_gcs::views::presets::write_plan;

const OSR: [i32; 8] = [1, 2, 4, 8, 16, 32, 64, 128];

/// BMP581 Table 7: rows OSR_P x1 .. x128, columns OSR_T x1 .. x128, the highest ODR in Hz.
const BMP581_MAX_ODR: [[i32; 8]; 8] = [
    [240, 240, 240, 240, 200, 130, 80, 40],
    [240, 240, 240, 220, 180, 120, 70, 40],
    [220, 220, 200, 180, 140, 100, 70, 40],
    [140, 140, 130, 120, 100, 80, 50, 35],
    [80, 80, 80, 70, 70, 50, 45, 30],
    [45, 45, 40, 40, 40, 35, 30, 20],
    [20, 20, 20, 20, 20, 20, 15, 15],
    [10, 10, 10, 10, 10, 10, 10, 5],
];

/// IIR 0, 1, 3, 7, 15, 31, 63, 127 are all valid with any OSR and ODR.
fn baro_valid(v: &BTreeMap<String, f64>) -> bool {
    let at = |name: &str| {
        OSR.iter()
            .position(|o| f64::from(*o) == v[name])
            .expect(name)
    };
    v["BARO_ODR_HZ"] <= f64::from(BMP581_MAX_ODR[at("BARO_OSR_P")][at("BARO_OSR_T")])
}

/// BMM350 Table 5: ODR 400 allows averaging 1, 200 allows 2, 100 allows 4, 50 and below allow 8.
fn mag_valid(v: &BTreeMap<String, f64>) -> bool {
    let odr = v["MAG_ODR_HZ"];
    let most = if odr >= 400.0 {
        1.0
    } else if odr >= 200.0 {
        2.0
    } else if odr >= 100.0 {
        4.0
    } else {
        8.0
    };
    [1.0, 2.0, 4.0, 8.0].contains(&v["MAG_AVG"]) && v["MAG_AVG"] <= most
}

fn values(sensor: &Sensor, id: i32) -> BTreeMap<String, f64> {
    let p = sensor.presets.iter().find(|p| p.id == id).expect("preset");
    sensor
        .fields
        .iter()
        .zip(p.values)
        .map(|(f, v)| {
            (
                (*f).to_string(),
                match v {
                    Value::I32(x) => f64::from(*x),
                    Value::F32(x) => f64::from(*x),
                },
            )
        })
        .collect()
}

fn table_of(v: &BTreeMap<String, f64>) -> ParamTable {
    let mut table = ParamTable {
        encoding_known: true,
        ..ParamTable::default()
    };
    for (i, (name, n)) in v.iter().enumerate() {
        let (kind, raw) = if name == "MAG_ODR_HZ" {
            (ParamType::F32, (*n as f32).to_le_bytes())
        } else {
            (ParamType::I32, (*n as i32).to_le_bytes())
        };
        table.params.insert(
            name.clone(),
            Param {
                name: name.clone(),
                index: i as u16,
                kind,
                raw,
            },
        );
    }
    table
}

/// Every ordered pair of presets: the plan from one to the other, every intermediate combination
/// valid, the end equal to the target, and only the changed fields written.
fn check(sensor: &Sensor, valid: fn(&BTreeMap<String, f64>) -> bool) -> usize {
    let mut intermediates = 0;
    for from in sensor.presets {
        for to in sensor.presets {
            let start = values(sensor, from.id);
            let end = values(sensor, to.id);
            assert!(valid(&start) && valid(&end), "{} -> {}", from.name, to.name);
            let plan = write_plan(&table_of(&start), sensor, to).expect("plan");
            let changed = start.iter().filter(|(k, v)| end[*k] != **v).count();
            assert_eq!(
                plan.len(),
                changed,
                "{} -> {}: only changes",
                from.name,
                to.name
            );
            let mut now = start.clone();
            let mut seen_up = false;
            for (name, value) in &plan {
                let new = value.as_f64();
                let up = new > now[*name];
                assert!(
                    up || !seen_up,
                    "{} -> {}: a lowering after a raising",
                    from.name,
                    to.name
                );
                seen_up |= up;
                now.insert((*name).to_string(), new);
                assert!(
                    valid(&now),
                    "{} -> {}: invalid after writing {name}: {now:?}",
                    from.name,
                    to.name
                );
                intermediates += 1;
            }
            assert_eq!(now, end, "{} -> {}", from.name, to.name);
        }
    }
    intermediates
}

#[test]
fn every_intermediate_combination_of_every_baro_transition_is_valid() {
    assert!(check(&BARO, baro_valid) > 0);
}

#[test]
fn every_intermediate_combination_of_every_mag_transition_is_valid() {
    assert!(check(&MAG, mag_valid) > 0);
}

#[test]
fn raising_first_would_break_the_rule_so_the_order_matters() {
    // Balanced (50 Hz, x16/x2) to Precise (10 Hz, x128/x8): the OSR written before the ODR is invalid.
    let start = values(&BARO, 1);
    let mut now = start.clone();
    now.insert("BARO_OSR_P".to_string(), 128.0);
    assert!(!baro_valid(&now), "x128 at 50 Hz is over Table 7's 10 Hz");
    // Balanced (100 Hz, avg 2) to Ultra low noise (50 Hz, avg 8): averaging 8 at 100 Hz is over Table 5.
    let mut now = values(&MAG, 1);
    now.insert("MAG_AVG".to_string(), 8.0);
    assert!(!mag_valid(&now));
    assert!(baro_valid(&start));
}
