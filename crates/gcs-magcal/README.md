# gcs-magcal

The numerical core of the guided compass calibration (`_plan/decision-compass-cal-20261120.md`, link contract
section 16). Pure: no I/O, no clock, no dependency beyond `std`. Units are microtesla and degrees.

- `wmm`: the World Magnetic Model 2025, degree 12, from NOAA's `WMM.COF` (public domain, embedded from
  `data/WMM.COF`), valid 2025.0 to 2030.0. `cargo run -p gcs-magcal --example wmm -- LAT LON HEIGHT_M YEAR`.
- `undo`: the calibration in force, m = S (r - o); `Calibration::undo` refuses a matrix that is not positive
  definite; `gauss_to_ut` converts HIGHRES_IMU's gauss.
- `fit`: PX4's Levenberg-Marquardt sphere fit then ellipsoid fit, ported from `lm_fit.cpp` (BSD-3-Clause; PX4's
  notice is kept in `src/fit.rs`). `cargo run -p gcs-magcal --example fit_csv -- FILE.csv [RADIUS_UT]` fits a CSV
  of raw x,y,z in uT.
- `accept`: the checks of the decision's section 4 in its order, each with its message; on acceptance S is scaled so
  the mean corrected strength equals the model's. Coverage (all six poses turned) is an input, `incomplete_poses`.

Tests: `tests/wmm.rs` (NOAA's two WMM2025 test tables, `tests/data/`), `tests/undo.rs` (the contract's test vector
and a round-trip property), `tests/fit.rs` (synthetic rings, a thin band, one test per acceptance row).
