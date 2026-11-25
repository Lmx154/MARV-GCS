# Flight report

Part of [MARV-GCS](../README.md)

`marv-cli log report` reads one DataFlash log written by the MARV firmware and prints its health, armed windows, flights,
motor identification, vibration, attitude tracking and a closing block of the vehicle values the log supports. The
analysis is `gcs_dflog::flight_report_with`; `marv-cli` only parses arguments and prints. Record names and columns come
from the log's own FMT records, never from offsets; a record type the log lacks makes the sections that need it say "not
in this log" and the report goes on.

```
marv-cli log report flight.BIN --mass-kg 1.2 --kf 2.8e-5 [--poles 14] [--accel-lpf-hz 20 ...] [--level-range MIN MAX] [--json report.json]
```

| Input | Rule |
|---|---|
| `--mass-kg`, `--kf` | Required: no airframe is built in. `kf` in N/(rad/s)², one motor |
| `--poles` | Rotor speed is `eRPM / (poles / 2) x 2 pi / 60` rad/s. Default: the log's `MOT_POLES`. With neither, every section that needs the rotor speed says "give --poles: the motor's magnet count" and the rest is still produced |
| `--accel-lpf-hz` | Repeatable. With none, the vibration section's 1 g window has only its raw share (the firmware's filter setting is not known to the tool) |
| `--level-range min max` | Ends of the motor command range (min below max; else exit 2); the closing block then prints the motor line's speed at both |
| `--json FILE` | The same content as JSON (written with `serde_json`); an existing file is never overwritten |

Memory is one small accumulator per half-second bin of armed time plus the buffers of the interval being read; a 100 MB
log is never held decoded. The vibration, tracking and ground sections need the flights, known only after the first pass,
so the stream is read a second time (`IMU`, `NAV` and `SETP` records; the ground check `NAV` and `IMU`).

## Sections

| Section | What it holds |
|---|---|
| Header | File name, bytes, bad and tail bytes, duration, the inputs and where the pole count came from |
| Health | Dropped records (last `FDS.Dp`, `DSF.Dp` and their sum), least `FMn` of each ring, largest `TimeUS` spacing of consecutive `NAV` records inside one armed window (us), `DMD` records with `Sat` != 0 inside the flights, highest `ESC` temperature per motor, lowest `BAT.Volt` |
| Armed windows | From `MOT.Arm`: start, end, duration of each stretch with `Arm` = 1 (a log that ends armed ends the window at its last `MOT` record and says so) |
| Flights | Defined below; a window with none is "armed, not airborne" |
| Per flight | Duration; mean throttle stick `RCS.T`; mean `SETP.Thr` in N and as a fraction of mass x g; mean rpm per motor; mean of kf x sum of speed² over mass x g; battery volts (minimum) and amps (mean, maximum); mean `DMD` `TX`, `TY`, `TZ` |
| Motor identification | Over all armed time, per motor a least-squares line speed = a + b x level (rad/s; levels as in `MOT.A0..A3`) with rms residual, bins, level range and mean battery voltage; the same line pooled over four motors; the thrust constant |
| Vibration | Per flight and per ground run from `IMU` records with `I` = 0 |
| Tracking | Per flight, how well the `NAV` attitude followed the `SETP` attitude setpoint |
| Ground check of the tilt estimate | Per ground run, whether the estimated tilt moved while the vehicle did not |
| Values this log supports | The closing block |

The firmware writes one step's records in the order `IMU`, `NAV`, `SETP`, `DMD`, `MOT`, every step while armed and every
8th while disarmed. The armed state of a `NAV` is therefore that of the `MOT` record with the same `TimeUS`, and a spacing
counts only when both `NAV` records lie in the same armed window; the long spacing across an arm or disarm edge is not a
logging gap.

## Flight definition

Within an armed window, time is cut in bins of 0.5 s from the window's start (a partial last bin is left out). For a bin,
r = kf x (mean over its `ESC` records of the sum over four motors of speed²) / (mass x 9.80665 N); a bin with no `ESC`
record has r = 0.

| Constant | Value | Meaning |
|---|---|---|
| `FLIGHT_START_RATIO` | 0.85 | A flight starts at the first bin with r at or above this |
| `FLIGHT_END_R` | 0.5 | It ends before the first later bin with r below this, or at the end of the window; the last bin is dropped |
| `MIN_FLIGHT_BINS` | 6 | Minimum bins after dropping |

Every per-flight figure and the saturation count use the flight's bins (means of per-bin means, so a busier record type
does not weigh more). Why 0.85: a run-up below 85 % of the weight is not a flight, and a hover sits at r = 1 within a few
per cent, so a start at 1.0 would miss it whenever the given kf is a few per cent high. Round 2 (below) computes r with
the fitted constant, which makes the threshold 85 % of the mean flight thrust. Why the end at 0.5: one bin just under the
start line used to split a flight in two, and keeping only high-thrust bins biased the thrust constant upward. If round 1
finds no flight in a log with armed bins at r >= 0.5, the report says "no flight found with the given kf; if the vehicle
flew, kf is probably too low (the largest r seen was X)".

## Thrust constant and identification

From lift-off to touchdown the mean thrust equals the weight, so for a flight k = mass x g / (time-mean of the sum of
speed² over the flight); pooled over flights, the mean is over all their bins. The bounds depend on the given kf, so it is
done twice: bounds with the given kf give k1; bounds again with k1 give k2. The flights reported are those of round 2 and
both constants are printed. If a bound moved by more than one bin between the rounds (or the number of flights changed)
the report says so.

Identification, per motor: a bin counts when it has at least two `MOT` and one `ESC` record, the population standard
deviation of that motor's level in it is below 12 levels (`STEADY_LEVEL_STD`), and its mean level is above the idle plateau
(the lowest level sent while armed) plus 4 (`IDLE_MARGIN`). The fit uses bin means and needs three such bins at two or more
levels, else "not enough". A fit whose level range is under 25 % of its mean level (`NARROW_LEVEL_RANGE`) is printed
"ill-conditioned: narrow level range". The line and k are valid only for the level range and battery voltage seen; k is
compared with the given kf and the ratio printed.

## Vibration

Intervals are each flight and each ground run: armed, not in a flight (a flight's dropped last bin counts as in it),
consecutive bins whose mean level of the four motors is at least 20 levels (`GROUND_RUN_MARGIN`) above the mean of their
idle plateaus, at least 4 bins (`MIN_GROUND_RUN_BINS`). For each, from `IMU` records of instance 0 (`GyrX..Z` rad/s,
`AccX..Z` m/s²):

- the sample rate (median `TimeUS` spacing); below 400 Hz (`MIN_SPECTRUM_RATE_HZ`) the report prints it and "too slow for
  spectra" and still gives the other figures;
- the standard deviation per axis of gyro and accelerometer, and of the accelerometer magnitude;
- the Welch spectrum per axis: segments of 1024 samples, Hann window, 50 % overlap, mean removed per segment, averaged
  (a radix-2 FFT, `fft_in_place`), scaled so its sum is the variance. For each of the six axes the three largest peaks:
  frequency, and amplitude as the rms within ±2 bins of the peak (a sine of amplitude A reads A / sqrt 2);
- the rotor band: lowest and highest over the four motors of the mean rotor frequency (rpm / 60), and per axis the share
  of total variance within that band widened by 10 % each side and within twice the band. A band above half the sample
  rate is seen folded: the report says "folded" and prints the ends as `|f - fs x round(f / fs)|`;
- the 1 g window: the share of raw accelerometer samples with 0.9 g <= |a| <= 1.1 g, and the same after a first-order
  low-pass `y += (x - y) x dt / (dt + 1/(2 pi fc))` (seeded with the first sample) for each `--accel-lpf-hz`.

Reading it: a rotor-band share near 1 on an axis says that axis's vibration is the rotors' (their speed or twice it); a
large peak elsewhere is a structural resonance or a neighbouring source; a ground run next to a flight shows what is the
motors and what is the air; a raw 1 g share well below 1 with a filtered share near 1 means the filter at that cut-off
removes the vibration.

## Tracking

From `NAV` (`Q1..Q4`, the estimated attitude quaternion [w, x, y, z] body to NED; `GX`, `GY` body rates, rad/s) and `SETP`
inside each flight. Only a `SETP` with `K` = 1 (angle mode) is used, paired with the `NAV` of the same `TimeUS`; others are
counted ("N records with another setpoint kind, not analysed"). Angles are roll, pitch, yaw (ZYX) in degrees; the error is
always setpoint minus estimate, yaw wrapped to ±180. Definitions are `pub const` in `tracking.rs`.

| Figure | Definition |
|---|---|
| Per flight and axis | Min and max of the setpoint (yaw as the raw angle in (-180, 180]); mean and rms of the error; the largest-magnitude (signed) error with its time |
| Excursion (roll, pitch) | At each record, the min and max of that axis's setpoint over the preceding `EXCURSION_WINDOW_S` = 0.5 s; the excursion is how far the estimate lies outside that range (0 inside). Per flight: the largest, its time, the estimate, the end of the range passed and the body rate in deg/s. Whole log: the 3 largest events per axis, taken greedily; a local maximum less than `EXCURSION_SEPARATION_S` = 1.0 s from a larger one already taken is skipped |
| Holds (roll, pitch) | A run starts at a record and goes on while the setpoint stays within `HOLD_TOL_DEG` = 1.0 of its first value. It is a hold when it lasts at least `HOLD_MIN_S` = 0.5 s and the setpoint is at least `HOLD_MIN_DEG` = 10.0 away from 0. Settled figures are means from `HOLD_SETTLE_S` = 0.3 s after its start: setpoint, estimate, settled error. Per axis: number of holds, median and largest settled error; the text lists up to 10 holds per axis (largest first), the JSON all in time order |

## Ground check of the tilt estimate

With the motors running on the ground the vehicle does not rotate, so the estimated tilt must not move. Figures only, no
verdict. One block per ground run from `NAV` and `IMU` (`I` = 0) in the second pass.

- Reference: the `GROUND_REF_S` = 1.0 s that ends at the run's start: mean estimated roll and pitch, gyro bias (mean `GyrX`,
  `GyrY`), accelerometer tilt. With fewer than `GROUND_REF_MIN_RECORDS` = 20 `NAV` or `IMU` records it says so and gives
  no figures.
- Per 0.5 s bin: mean estimated roll and pitch; gyro-integrated rotation about body x and y since the run start (small-angle
  sum of (gyro minus bias) x dt); the tilt implied by the mean acceleration (roll = atan2(-AccY, -AccZ), pitch =
  atan2(AccX, hypot(AccY, AccZ))).
- Per run: reference roll and pitch; largest estimate shift per axis (signed, with its bin time); gyro rotation at the
  last sample; largest unexplained shift per axis (the bin's estimate shift minus the mean gyro-integrated rotation over its
  samples, roll with x, pitch with y); largest shift of the accelerometer tilt from its reference value. "Largest" is the
  largest magnitude, kept signed. A log without `NAV` or `IMU` says "not in this log"; without a ground run, "no ground
  run".

## Values this log supports

The last block gathers the vehicle values the log gives and the range each is valid for; it computes nothing a second way
and applies nothing. Its title says the values are valid only for the levels, speeds and battery voltage seen.

| Value | Content |
|---|---|
| Thrust constant | Round-2 value, smallest and largest per-flight value, number of flights |
| Motor line | The pooled fit, its level range, rms residual, mean battery voltage, "ill-conditioned" when so; with `--level-range`, the speed at `min` and `max`, each marked "extrapolated" outside the fitted range |
| Hover | Over flights weighted by duration, the mean throttle stick and the mean commanded collective as a fraction of mass x g, each with per-flight min and max, and the lowest and highest of the flights' lowest battery voltage |
| Accelerometer low-pass | Share of samples inside the 1 g window over all flights weighted by sample count, raw and per `--accel-lpf-hz` |

A part whose input is missing prints the usual line ("give --poles", "not in this log (...)" or "no flight").

## Privacy

The report reads no position. The analysis never opens the fields of `GPS`, `GPS2`, `GPA`, `POS` or `ORGN` records (not
even their time), and the report has no field that could hold one. A test puts a GPS record with a known position in a
synthetic log and checks that nothing of it reaches the report; the command-line test checks the text and JSON too.

## Code

`crates/gcs-dflog/src/`: `flight_report.rs` (flights, identification, health), `vibration.rs`, `tracking.rs`, `ground.rs`,
`values.rs`; `crates/marv-cli/src/log_report.rs` prints it. Tests: `tests_report.rs`.

## Related

[dflog](dflog.md) · [marv-cli](marv-cli.md) · [sessions-and-logs](sessions-and-logs.md)
