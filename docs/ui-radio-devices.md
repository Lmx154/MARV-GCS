# Radio and Devices tabs

Part of [MARV-GCS](../README.md)

The Radio tab shows the receiver's channels and the vehicle's mapped sticks, and calibrates and maps them. The Devices tab
shows the health of every device, runs calibrations, holds the guided compass calibration and the sensor-preset cards.
Everything that writes is Configure-only and shown disabled with its reason on an Operate link.

## Radio

| Part | Behaviour |
|---|---|
| Channel bars | 16 bars from `RC_CHANNELS` (800 to 2200 us, marks at 1000, 1500, 2000, the number beside each, `chancount` respected, 65535 unused); link quality (`rssi`, 255 unknown); age of the last message ("no stick data" when older than 2 s or never seen, bars empty); the RC row of `MARV_DEVICE_STATUS` (id 7); `RADIO_STATUS` when it arrives |
| Function bars | From `MANUAL_CONTROL`, which the vehicle computes from the mapping ([link-telemetry](link-telemetry.md)): Roll (`y`), Pitch (`x`), Yaw (`r`) on -1000..1000, Throttle (`z`) on 0..1000, each with its expected direction ("stick right: +", "stick forward: +", "yaw right: +", "throttle up: 1000"); "no data" when none or older than 2 s. The station computes no value |
| Marks | On channels 1 to 8, once the parameter list is complete: marks at that channel's `MIN`, `TRIM`, `MAX` and its function (`RCMAP_*`). A vehicle without `RC1_MIN` gets "this firmware has no RC mapping parameters" |

### Radio calibration (Configure)

"Calibrate radio" is disabled, with its reason, on an Operate session, while the sticks are stale and until the parameter
list is complete; the text beside it says: transmitter on in its CRSF model; battery unplugged, no propellers.

1. **Capture.** The running minimum and maximum of each of the 16 channels is drawn on its bar while the user moves every
   stick and switch to both ends; then Done.
2. **Centre.** "Centre the sticks, throttle down, then OK" takes the current values as the trims.
3. **Review.** Channels 1 to 8 as current -> captured `MIN`/`TRIM`/`MAX`. A channel whose range does not reach below 1300
   and above 1700 is "not moved" and is not written; `TRIM` is clamped into [`MIN`, `MAX`]. Channels 9 to 16 are shown,
   never written.
4. **Apply** sends one `PARAM_SET` per value that differs (at most 24); **Cancel**, at any step, sends nothing.
5. After Apply: "Applied in RAM; not saved to flash" and a "Save to flash" button (command 245 param1 = 1) until a save
   is accepted. Nothing is saved automatically, and the station sends no radio-calibration command.

### Mapping (Configure)

Four drop-downs Roll / Pitch / Throttle / Yaw -> channel 1 to 8 (`RCMAP_*`); per function "Reversed" (`RCn_REVERSED`) and
"Dead zone (us)" (`RCn_DZ`, 0 to 200) of the channel the function is mapped to in the map being edited. Edits are staged
(a "Staged: ..." line lists them); "Apply mapping" sends one `PARAM_SET` per staged value that differs and "Revert" drops
them. The same "not saved" line and "Save to flash" button follow. Two functions on one channel show a warning and do not
block Apply. The section is disabled on Operate, until the list is complete, on a vehicle without `RC1_MIN`, and during a
calibration; "Calibrate radio" is disabled while mapping edits are staged. Reversal is never inferred.

## Devices

| Part | Behaviour |
|---|---|
| Health table | One row per `MARV_DEVICE_STATUS` device in `device_id` order: state (coloured), last error, samples and samples per second (from two successive rows), age of the last sample (`never` for 65535), errors, timeouts, recoveries, attempts, overflows, configuration restores, flags (HIL, self-test), time since the row arrived (older than 15 s is stale, not removed). Read only |
| Buttons | Calibrate gyro, Calibrate accelerometer (level), Cancel calibration; per row Restart (IMU1, IMU2, BARO, MAG, GNSS, BATT, SD) and Self-test (IMU1, IMU2, MAG). All Configure-only |
| Unsaved bar | After an accepted calibration: "Gyro, accel calibrated: in use now, not saved to flash" with **Save to flash** (245 param1 = 1; one save covers every calibration so far) and **Discard** (245 param1 = 0: reload from flash, dropping every unsaved change). It stays until a save or reload is accepted from any tab or the vehicle reboots. After a calibration the station reads that calibration's offsets back; after an accepted reload it fetches the whole parameter list again |
| After a reboot | On USB a reboot starts a new session, so the bar is cleared; on a link that survives it (UDP) the line "unsaved calibration lost at reboot" stays until the next calibration or accepted 245 |

The real board sends rows for ids 1 to 7 and 9; the simulator sends IMU1, IMU2, BARO, MAG, GNSS, BATT and RC.

### Compass panel (guided calibration)

The line "Compass calibration: ..." says what the vehicle's parameters hold once loaded: "Not calibrated" (`MAG_CAL_TYPE` 0
and offsets zero), "Best effort" (type 1; type 0 with offsets reads "best effort (older calibration)"), "Guided" (type 2
with `MAG_FLD_UT` above 0), with the stored field and inclination. The maths and the procedure are in
[gcs-magcal](../crates/gcs-magcal/README.md); the wire side in [link-compass](link-compass.md).

| Part | Behaviour |
|---|---|
| Main button | "Calibrate compass (guided)": six poses (level, upside down, nose down, nose up, left side down, right side down) in any order, and in each one slow turn about the vertical. There is no button per pose: the accelerometer recognises the pose and the turn completes it |
| While running | Left: a 3D quad drawn with the egui painter (grey ghost in the pose to take, the live quad at the tilt of the `HIGHRES_IMU` accelerometer, because the firmware sends no `ATTITUDE`). Right: six tiles, each with a ring of 12 sectors of 30 degrees that fills as the turn progresses. Below: the instruction, the seconds left for the pose (90 s each), the samples kept, "Cancel compass calibration" |
| Inputs | The calibration in force (`MAG_OFF*`, `MAG_DIAG*`, `MAG_ODIAG*`) and the latest `GPS_RAW_INT` at start; every `HIGHRES_IMU` id 0 message, the armed state, any change of those nine parameters (the run is abandoned) and the time on every frame; a lost link ends in "Timed out waiting for <pose>: start again" |
| Refusals and endings | No GPS fix of type 2 or better younger than 100 s, an invalid stored matrix, a pose that never completes, scattered readings, implausible field strength, distortion, a disturbed field, armed, cancelled: the session's message is shown and nothing is written |
| Commit | Only an accepted run writes, one `PARAM_SET` at a time, each judged by its echo: `MAG_CAL_TYPE` 0, `MAG_OFFX/Y/Z`, `MAG_DIAGX/Y/Z`, `MAG_ODIAGXY/XZ/YZ`, `MAG_FLD_UT`, `MAG_FLD_INC`, `MAG_CAL_TYPE` 2 (thirteen writes). A refused or unanswered write stops the sequence and names the parameter |
| Verification | The station fetches the list again and compares all thirteen; only then "Compass calibrated (guided): in use now, not saved to flash" with fitness, offsets and measured against model inclination, and Save to flash / Discard. A failed write offers no Save (the vehicle is marked type 0; Discard or a reboot restores flash). A read-back that differs or takes over 30 s makes the station write `MAG_CAL_TYPE` 0 once more and say so; if that fails it warns of an unverified calibration |
| Gates | Configure-only; disabled on an Operate link and while armed. The tab must stay showing until the result is committed |
| Quick calibration | "Quick calibration (best effort)": command 241 param2 = 1, 30 s of rotating through all orientations. Heading errors of 15 degrees and more were measured with this kind of calibration |
| Heading check | "Start heading check", walk a straight line, "Stop heading check": the tilt-compensated compass heading plus the model declination against the GPS course (ground speed 1 m/s or more, at least 10 records); it informs and never blocks ("within 10 deg" or "off by 14 deg: calibrate again or check what is mounted near the compass"). The model date is the computer's clock; the model is valid 2025.0 to 2030.0 |

### Sensor presets

A preset is the station setting several parameters at once as a verified combination; the firmware knows no preset. The
table, scores and the write-order rule are in [link-presets](link-presets.md). Five cards (IMU1, IMU2, BARO, MAG, GNSS),
each with a picker (the presets of `gcs_mavlink::presets`; "Custom (debug)" only with the debug switch on), a radar chart
(rings at 25, 50, 75, 100 %, the chosen preset filled, the one under the mouse outlined), the summary, the physical value
of every axis with its unit and a `model` or `bound` tag, and a status line.

| Behaviour | Detail |
|---|---|
| Active preset | The one whose fields all equal the vehicle's (a REAL32 compared numerically), else "Custom" (read only; a preset can still be chosen); "parameters not loaded" before the list is in |
| Apply | Configure-only. Writes the chosen preset's fields as `PARAM_SET`s, falling fields first then rising ones; a field that does not change is not written; GNSS orders `GPS_GNSS` around `GPS_RATE_MS` |
| Result | Judged from each write's echo. BARO, MAG, GNSS: "Applied in RAM; not saved to flash" with Save to flash (the line goes only when a save started after the Apply is accepted). A refused write names the field ("refused: BARO_OSR_P (vehicle uses 16)") and the card shows what the values now match |
| IMU1, IMU2 | The vehicle answers each write with the INFO text `<NAME>: applies after reboot`; the card says "Pending: takes effect after Save and reboot" with one button, Save and reboot (second click): command 245 param1 = 1 and, only when that is ACCEPTED, 246 param1 = 1. A refused or unanswered save sends no 246. Pending belongs to the session |
| Debug switch | "Debug: raw sensor settings", station-local, off at every start, never saved. On: "Custom (debug)" is offered, the card's fields become values to type ("Apply fields": one `PARAM_SET` per changed field, the vehicle validates and its refusal shows beside the field) |

The chart is 180 px tall (radius 70, smaller as needed to keep every axis label inside the card).

## Code

`crates/marv-gcs/src/views/`: `radio.rs`, `devices.rs`, `compass.rs`, `compass_quad.rs`, `presets.rs`; the presets table
in `crates/gcs-mavlink/src/presets/tables.rs`; the compass session in `crates/gcs-magcal/src/session.rs`.

## Related

[ui-tabs](ui-tabs.md) · [link-telemetry](link-telemetry.md) · [link-presets](link-presets.md) ·
[link-compass](link-compass.md) · [link-commands](link-commands.md)
