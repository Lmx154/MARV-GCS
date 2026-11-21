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
  the mean corrected strength equals the model's, and the scaled terms are checked again against the same limits.
  Coverage (all six poses turned) is an input, `incomplete_poses`. Acceptance without it is NOT a guarantee against
  poor coverage: data with one direction missing can pass every numeric row; the coverage row is the guard.

The procedure on top of it, still pure (no UI, no link code, no clock, no async; time is passed in):

- `pose`: the six poses (level, upside down, nose down, nose up, left side down, right side down) recognised from one
  accelerometer sample with PX4's rule (each axis within 5 m/s^2 of the expected reading; FRD, specific force: level
  reads (0, 0, -g)). The cited constants and file:line are in the module text.
- `turn`: the gyro integrated about the gravity axis while a pose is held; a ring of 12 sectors of 30 deg per pose,
  complete when every sector was visited (either direction). An incomplete ring is cleared when ANOTHER pose is
  recognised, not when the reading falls to "none"; a complete ring stays. A step of 0.5 s or more, or not forward in
  time, is not integrated (display-only safeguard).
- `collect`: raw samples (the calibration in force undone) with the unit down vector, thinned by PX4's spacing rule
  (mag_calibration.cpp:147-149: 5.4 R / sqrt(240) / 3, R the model's field strength).
- `session`: the state machine. `Session::start(calibration, gps, decimal_year, now)` refuses an invalid stored matrix or a
  missing/old/poor GPS fix; `sample`, `param_changed`, `armed`, `cancel`, `tick(now)` drive it; `status(now)` is the
  snapshot for a view; the outcome is `Accepted` with the ordered `write_plan` (13 writes) or an ending with its
  message. It writes nothing. `verify_read_back` compares a read-back set with the plan (equal as f32).
- `heading`: the heading check over recorded data: tilt-compensated compass heading plus declination against GPS
  course, median difference and median absolute deviation, "within 10 deg" or "off by X deg: ...".

Tests: `tests/pose.rs`, `tests/turn.rs`, `tests/collect.rs`, `tests/session.rs` (a scripted user at 25 Hz),
`tests/heading.rs`; `tests/wmm.rs` (NOAA's two WMM2025 test tables, `tests/data/`), `tests/undo.rs` (the contract's test vector
and a round-trip property), `tests/fit.rs` (synthetic rings, a thin band, one test per acceptance row).
