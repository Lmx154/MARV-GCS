# Link contract: sensor presets

Part of [MARV-GCS](../README.md) · [link-contract](link-contract.md), section 13

A preset is the station setting several parameters at once as a verified, sane combination. It is a station feature: the
firmware has no preset, no selector and no preset rule. The module `gcs_mavlink::presets` (`crates/gcs-mavlink/src/presets/
tables.rs`) is the normative table: ids, names, summaries, field values, axes, physical values and scores; the window,
`marv-cli` and the tests use it, the simulator does not. The tables below are its content (its tests, `crates/gcs-mavlink/
tests/presets.rs`, are the oracle). Ids 1 to 4 and their names do not change meaning or number; a new preset is appended
with the next id. The card behaviour is in [ui-radio-devices](ui-radio-devices.md); the parameters in
[link-parameters](link-parameters.md).

## What a preset changes

| Sensor | Fields |
|---|---|
| IMU1, IMU2 | Accelerometer range, gyro range, output data rate (both sensors of a chip at one rate). Everything else is the same in every preset: internal clock, INT1 latched data-ready, single-sample reads, no FIFO, chip filters off |
| BARO | Rate, pressure and temperature oversampling, pressure and temperature IIR |
| MAG | Rate and averaging |
| GNSS | The fix interval `GPS_RATE_MS` and the constellation set `GPS_GNSS` |
| BATT | None |

## The chart

Axes, more is better on every axis. IMU1 and IMU2 (pentagon): Accel range, Gyro range, Low noise, Bandwidth, Light load.
BARO and MAG (diamond): Low noise, Fast response, Update rate, Light load. GNSS (diamond): Update rate, Fast response,
Light load, Sky coverage.

Score of an axis: `s = round_half_up(100 * clamp(ln(x / x0) / ln(x100 / x0), 0, 1))`, a log scale between two fixed ends x0
(score 0) and x100 (score 100), the same ends for both IMUs. IMU Low noise is the mean of two fractions:
`s = round_half_up(100 * (fg + fa) / 2)`, fg and fa the clamped fractions of gyro and accelerometer noise.

| Sensor | Axis | Quantity x | x0 (0) | x100 (100) |
|---|---|---|---|---|
| IMU | Accel range | full scale, g | 2 | 32 |
| IMU | Gyro range | full scale, dps | 125 | 4000 |
| IMU | Low noise | gyro rms dps; accel rms mg | 1 dps; 10 mg | 0.01 dps; 0.1 mg |
| IMU | Bandwidth | B = ODR / 2, Hz | 10 | 5000 |
| IMU | Light load | SPI bytes per second | 200 000 | 200 |
| BARO | Low noise | pressure rms, Pa | 1 | 0.01 |
| BARO, MAG | Fast response | delay, ms | 1000 | 1 |
| BARO, MAG | Update rate | ODR, Hz | 1 | 500 |
| BARO, MAG | Light load | I2C bytes per second | 10 000 | 10 |
| MAG | Low noise | x/y field rms, nT | 1000 | 10 |
| GNSS | Update rate | fixes per second | 1 | 10 |
| GNSS | Fast response | interval, ms | 1000 | 1 |
| GNSS | Light load | UART bytes per second | 11 520 (115200 8N1) | 100 (one frame per second) |
| GNSS | Sky coverage | constellations | 1 | 4 |

## Physical values

From datasheet and driver numbers; MODEL and BOUND are shown as such on the label.

- IMU Bandwidth: B = ODR / 2 (MODEL; the LSM6DSK320X datasheet's convention, and the upper bound for the ICM-45686 with its
  UI filter bypassed).
- IMU noise: `sigma = sqrt(ND^2 * B + LSB^2 / 12)`, gyro and accelerometer separately; ND the datasheet's noise density (ICM
  gyro 0.0038 dps/rtHz, accel 70 / 80 / 110 ug/rtHz up to 8 g / 16 g / 32 g; LSM gyro 3.8 mdps/rtHz, accel 60 ug/rtHz), LSB
  the driver's scale (full scale / 32768 for the ICM; the sensitivity of the range for the LSM).
- IMU Light load: ODR x bytes per sample: ICM 18, LSM 27 (the driver's `read_all`: 26 bytes + address).
- BARO Low noise: `sigma_T9(OSR_P) / sqrt(2c + 1)`, sigma_T9 the datasheet's Table 9 value for the pressure oversampling, c
  the pressure IIR coefficient. Altitude shown beside it: 0.083 m per Pa (display only).
- BARO Fast response: `tconv_p(OSR_P) + tconv_t(OSR_T) + c / ODR`, the typical conversion times of the datasheet's Table 4.
- BARO Light load: ODR x 13 bytes.
- MAG Low noise: `190 nT * sqrt(2 / N)` for x/y and `450 nT * sqrt(2 / N)` for z, N the averaging (MODEL). Fast response:
  delay at most N x 2.5 ms (BOUND). Light load: ODR x 23 bytes.
- GNSS: Update rate = 1000 / `GPS_RATE_MS`, Hz. Fast response = `GPS_RATE_MS`, ms, BOUND (a fix is at most one interval old at
  the receiver; latency excluded). Light load = 100 B per NAV-PVT frame (6 B header + 92 B payload + 2 B checksum, u-blox M10)
  x rate, receiver to board. Sky coverage = the number of constellations enabled (QZSS counted with GPS): 4, 2, 3.
- Not shown on purpose: supply current, chip filter delay, IMU latency.

## Tables

Scores 0 to 100; the physical value in brackets.

**IMU1, ICM-45686** (low-noise mode, internal clock, UI LPF bypass, FIR off, INT1 latched, no FIFO).

| Id | Name | ACC_FS | GYR_FS | ODR_HZ | Accel range | Gyro range | Low noise | Bandwidth | Light load |
|---|---|---|---|---|---|---|---|---|---|
| 1 | Flight (default) | 16 | 2000 | 800 | 75 | 80 | 48 (0.078 dps, 1.61 mg) | 59 (400 Hz) | 38 (14 400 B/s) |
| 2 | Bench (clips in flight) | 4 | 250 | 800 | 25 | 20 | 49 (0.076 dps, 1.40 mg) | 59 (400 Hz) | 38 (14 400 B/s) |
| 3 | Max range | 32 | 4000 | 800 | 100 | 100 | 43 (0.084 dps, 2.22 mg) | 59 (400 Hz) | 38 (14 400 B/s) |
| 4 | Smooth | 8 | 1000 | 400 | 50 | 60 | 57 (0.054 dps, 0.99 mg) | 48 (200 Hz) | 48 (7 200 B/s) |

**IMU2, LSM6DSK320X** (high-accuracy ODR set 800: code 960 = 800 Hz, 480 = 400 Hz; LPF1 and LPF2 off; high-g channel off;
INT1 low-g data-ready latched; no FIFO).

| Id | Name | ACC_FS | GYR_FS | ODR_HZ | Accel range | Gyro range | Low noise | Bandwidth | Light load |
|---|---|---|---|---|---|---|---|---|---|
| 1 | Flight (default) | 16 | 2000 | 800 | 75 | 80 | 51 (0.079 dps, 1.21 mg) | 59 (400 Hz) | 32 (21 600 B/s) |
| 2 | Bench (clips in flight) | 4 | 500 | 800 | 25 | 40 | 51 (0.076 dps, 1.20 mg) | 59 (400 Hz) | 32 (21 600 B/s) |
| 3 | Max range (low-g limit 16 g) | 16 | 4000 | 800 | 75 | 100 | 50 (0.086 dps, 1.21 mg) | 59 (400 Hz) | 32 (21 600 B/s) |
| 4 | Smooth | 8 | 1000 | 400 | 50 | 60 | 58 (0.055 dps, 0.85 mg) | 48 (200 Hz) | 42 (10 800 B/s) |

**BARO, BMP581** (normal mode, FIFO off, data-ready pulsed).

| Id | Name | ODR_HZ | OSR_P | OSR_T | IIR_P | IIR_T | Low noise | Fast response | Update rate | Light load |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Balanced (default) | 50 | 16 | 2 | 0 | 0 | 34 (0.21 Pa, about 1.7 cm) | 65 (11.5 ms) | 63 | 40 (650 B/s) |
| 2 | Fast | 100 | 8 | 1 | 0 | 0 | 26 (0.30 Pa, 2.5 cm) | 73 (6.4 ms) | 74 | 30 (1 300 B/s) |
| 3 | Smooth | 25 | 32 | 2 | 3 | 0 | 62 (0.057 Pa, 0.5 cm) | 28 (141.5 ms) | 52 | 50 (325 B/s) |
| 4 | Precise (Bosch "Highest resolution") | 10 | 128 | 8 | 0 | 0 | 55 (0.08 Pa, 0.7 cm) | 36 (82.5 ms) | 37 | 63 (130 B/s) |

**MAG, BMM350** (normal mode, all axes, data-ready pulsed).

| Id | Name | ODR_HZ | AVG | Low noise (MODEL) | Fast response (BOUND) | Update rate | Light load |
|---|---|---|---|---|---|---|---|
| 1 | Balanced (default) | 100 | 2 | 36 (190 nT x/y, 450 z) | 77 (<= 5 ms) | 74 | 21 (2 300 B/s) |
| 2 | Fast | 200 | 2 | 36 (190 nT, 450 z) | 77 (<= 5 ms) | 85 | 11 (4 600 B/s) |
| 3 | Low noise | 100 | 4 | 44 (134 nT, 318 z) | 67 (<= 10 ms) | 74 | 21 (2 300 B/s) |
| 4 | Ultra low noise | 50 | 8 | 51 (95 nT, 225 z) | 57 (<= 20 ms) | 63 | 31 (1 150 B/s) |

**GNSS** (device 5, u-blox SAM-M10Q; default preset 1 = the firmware defaults 200 / 0). Three presets; the other sensors
have four.

| Id | Name | GPS_RATE_MS | GPS_GNSS | Update rate | Fast response (BOUND) | Light load | Sky coverage |
|---|---|---|---|---|---|---|---|
| 1 | Flight (default) | 200 | 0 | 70 (5 Hz) | 23 (<= 200 ms) | 66 (500 B/s) | 100 (4) |
| 2 | Fast | 100 | 1 | 100 (10 Hz) | 33 (<= 100 ms) | 51 (1 000 B/s) | 50 (2) |
| 3 | Max sky | 200 | 2 | 70 (5 Hz) | 23 (<= 200 ms) | 66 (500 B/s) | 79 (3) |

Summaries for the cards: IMU "+-16 g, +-2000 dps, 800 Hz"; BARO "50 Hz, x16/x2, no filter"; MAG "100 Hz, average of 2"; GNSS
"5 Hz, GPS+Galileo+BeiDou+QZSS", "10 Hz, GPS+Galileo", "5 Hz, GPS+Galileo+GLONASS".

## Write order

The active preset is the one whose field values all equal the vehicle's (a REAL32 compared numerically), else "Custom". Apply
writes the preset's fields as ordinary `PARAM_SET`s: first every field whose value goes DOWN, then every field whose value
goes UP, each group in the table's field order; a field that does not change is not written. Every intermediate combination
stays valid because the limits are downward closed: BMP581 Table 7 (the highest ODR falls as OSR_P or OSR_T rises) and BMM350
Table 5 (the highest averaging falls as ODR rises) only forbid raising an OSR or an averaging above what the ODR allows, or
raising an ODR above what the OSR allows; the IIR and IMU fields are unconstrained. GNSS is the exception because its fields
are tied (100 ms needs `GPS_GNSS` 1): the card writes `GPS_GNSS` before a faster rate and after a slower one.

| Change | Writes, in order |
|---|---|
| Flight to Fast | `GPS_GNSS` 1, `GPS_RATE_MS` 100 |
| Fast to Flight | `GPS_RATE_MS` 200, `GPS_GNSS` 0 |
| Max sky to Fast | `GPS_GNSS` 1, `GPS_RATE_MS` 100 |
| Fast to Max sky | `GPS_RATE_MS` 200, `GPS_GNSS` 2 |
| Flight and Max sky | `GPS_GNSS` only |

Accepted costs: another tool can write raw values, and a link drop in the middle of an Apply leaves the sensor between two
presets ("Custom") until it is applied again. The vehicle validates every write regardless. The simulator enforces the GNSS
cross rule in both directions with a text that names both values, has the six IMU parameters (Reboot class, the "applies
after reboot" text) and the BARO and MAG fields at the Flight / Balanced values, range-checked only; it does not use the
table.

## Related

[ui-radio-devices](ui-radio-devices.md) · [link-parameters](link-parameters.md) · [marv-cli](marv-cli.md) (`--debug` gate)
