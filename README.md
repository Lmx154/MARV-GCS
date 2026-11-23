# MARV ground control station

A desktop ground station for the MARV flight controller, in Rust, speaking MAVLink 2. It is an MVP that
does three things: live sensor data, parameters, and logging. The station speaks the project's own `marv`
dialect (`marv_dialect::marv`: the `common` messages plus the project's own, so far `MARV_TEST`, a link test
message, and `MARV_DEVICE_STATUS`, the health of one device of the flight controller), so both appear in the
Messages tab like any other message, with all their fields. It is built so that the firmware can add parameters, settings and messages
without the ground station being reworked.

The flight-controller end of the link is three small crates, each in its own repo:
[`usb-cdc`](https://github.com/Lmx154/usb-cdc) (bytes over USB),
[`marv-dialect`](https://github.com/Lmx154/marv-dialect) (the message definitions) and
[`mavlink-device`](https://github.com/Lmx154/mavlink-device) (the MAVLink layer, on any byte stream). The
ground station has been run against the MARV V2 board's real firmware (the `MARV-V2-firmware` repository)
over its USB port and over the radio: it runs eight device rows (IMU1, IMU2, BARO, MAG, GNSS, BATT, the radio receiver RC and the SD card),
the parameters, the commands, the calibrations, the self-test and the log download. The automated board
check (`tools/board_check.sh`) still uses the small demonstration program of `mavlink-device`. Without a board it runs against the simulated vehicle in this repo, which sends the real firmware's streams (below) and behaves as
`docs/link-contract.md` says the real one does.

## Quick start

```
cargo run -p gcs-sim                                      # simulated vehicle on 127.0.0.1:5760
cargo run -p marv-gcs -- --connect tcp:127.0.0.1:5760      # the window, connected to it
```

With a board: plug in its USB port, choose `Serial` in the bar at the top, pick the port (it is listed
with its product name and USB ids) and press Connect, or start with `--connect serial:/dev/ttyACM1`. The
baud rate does not matter for a USB port. On Linux the user must be in the `dialout` group.

With an ExpressLRS radio: join the WiFi network of the radio's backpack (the radio is typically at
`10.0.0.1`, the laptop gets `10.0.0.x`), choose `UDP` in the bar (or start with
`--connect udp:0.0.0.0:14550`) and press Connect. The address is the local one to listen on; the radio
sends MAVLink to port 14550, and the ground station answers to whoever sent last. A USB transmitter
module is expected to use the serial link instead (`--connect serial:PATH:460800`).

To test the radio link with the project's own test message, without a window:

```
cargo run --release --bin marv-link-test -- --connect udp:0.0.0.0:14550
```

It sends the station's heartbeat (system 255, component 190) every second and, after a quiet start,
`MARV_TEST` messages (tag 2, `seq` 0, 1, 2, ..., `value` = `seq * 3`), and prints once per second and at the
end what came back: frames per sender and message, the flight controller's `MARV_TEST` (tag 1) counted by
`seq` (received, first, last, lost, duplicates, restarts), the lost frames by the 8-bit header sequence of
every sender as a count and a percentage, bytes per second, checksum failures, and `RADIO_STATUS` if any
arrives. Options: `--count <n>` (default 100), `--rate <hz>` (10), `--duration <s>` (30), `--quiet-start <s>`
(2). It makes no pass or fail claim; the percentages are read by a person. Over UDP the first frames
are discarded until the peer has sent something (the peer is whoever sent last).

```
marv-gcs [--connect <endpoint>] [--log-dir <dir>] [--no-log] [--view <name>]
         [--review <file.BIN>] [--screenshot <file.png>] [--after <seconds>]
```

`<endpoint>` is `tcp:HOST:PORT`, `serial:PATH[:BAUD]`, `udp:ADDR:PORT`, or a recorded log to replay: `tlog:PATH` (at the
recorded pace) or `tlogfast:PATH`. `--view` picks the tab shown at start. `--review` opens a DataFlash `.BIN` log in the Log tab's "Review a log file" section and starts on that tab (an explicit `--view` wins). `--screenshot` saves a picture
of the window after `--after` seconds and exits; it is how the pictures in a report are made.

Building needs Rust 1.99 or newer (the `rust-version` of `marv-dialect`). On Linux the serial-port crate needs the libudev development package
(`libudev-dev` on Ubuntu).
Debug builds keep line tables only (`[profile.dev]` in `Cargo.toml`): backtraces still name file and line, and the
test binaries (each carries the Map tab's HTTP stack) link with less memory. Under a 4 GB memory cap build with
at most 4 jobs (`cargo test --workspace -j 4`): with 16 parallel jobs the compilers or linkers together exceeded
4 GB and were killed (2026-11-17); without a cap the default is fine.

AppImage: `tools/appimage.sh` builds `target/appimage/MARV-GCS-x86_64.AppImage`, the window as one file (about 8 MB)
that starts with a double click or by its name, no flags. It is the AppImage project's type-2 runtime (release
20251108, checked by SHA-256) followed by a squashfs of `usr/bin/marv-gcs`, `packaging/marv-gcs.desktop`, the icon
`packaging/marv-gcs.svg` and an `AppRun` that starts the window in `~/Documents/MARV-GCS`, so recordings and downloads
go to `~/Documents/MARV-GCS/logs` from wherever it is started. Nothing else is bundled: libc, libudev, libcap and the
display libraries come from the host. It runs on x86_64 Linux with glibc 2.39 or newer (built on Ubuntu 24.04) and
needs FUSE (`fuse3`); not for the Pi (ARM). Checked on 2026-11-17: started from another folder, the window opened,
listed the board's port and connected over USB (all device rows live).

## What it does

**HUD** (top left of the window, on every tab; 320 px wide at first, dragged between 240 and 480). A flight instrument drawn from the messages the vehicle sends; the station computes nothing from them except unit conversions. From `ATTITUDE` (30): an artificial horizon (roll and pitch; roll positive is right wing down, pitch positive is nose up) with a pitch ladder (a line every 5 degrees, a label every 10), a roll arc from -60 to +60 degrees with ticks at 10, 20, 30, 45 and 60 and a pointer, and a heading strip (`yaw`, 0 to 359, N/E/S/W and a tick every 10 degrees) with a small mark for the GPS course over ground; under the drawing the numbers `Roll`, `Pitch` and `Heading`. The drawing is a square as wide as the panel and everything in it, fonts included, scales with that width. From `GPS_RAW_INT`: ground speed (`vel`, cm/s shown in m/s; 65535 is unknown) in a box on the left, altitude (`alt`, mm shown in m, labelled "GPS alt MSL") in a box on the right, the course mark (`cog`, 65535 is unknown), and a line with the fix type and the number of satellites (255 is unknown). Under the numbers, small sections, scrolling when the window is short: Link status (connected, lost or not connected, heartbeat age, packet loss, `RADIO_STATUS` rssi when it arrives, RC link from `RC_CHANNELS`), GPS (fix, satellites, position), Battery (voltage and current from `SYS_STATUS`, charge), Vehicle (armed from `HEARTBEAT`; motors as "4/4 ESCs online" when `ESC_INFO` says all are, else only the offline ones), Sensor status (one green line "All sensors OK (N)" when every `MARV_DEVICE_STATUS` row and every enabled `SYS_STATUS` sensor is healthy, else only the ones that are not; grey "no sensor data" with no session or old data) and Recent messages (the last 3 `STATUSTEXT`, warnings and errors coloured). No `ATTITUDE` yet, or the last one older than 2 s: the horizon is level and grey with the text "no attitude from the vehicle". GPS values unknown or older than 3 s show `--`. Not connected: the drawing is grey. The real firmware does not send `ATTITUDE` yet (an estimator is being designed), so on the real board the HUD shows the GPS and battery values and "no attitude from the vehicle"; the simulator sends `ATTITUDE` at 10 Hz (a test double: slowly varying roll up to 20 degrees, pitch up to 10 degrees, yaw turning once in about 63 s, rates consistent with them). The HUD sends no command.

| Tab | What it shows |
|---|---|
| Sensors | Live plots, a table of presets (one row adds a plot), each a message and fields the real firmware sends: IMU1 acceleration and rotation and the magnetic field (gauss) from `HIGHRES_IMU` (`id` 0; the magnetometer is in its `xmag`/`ymag`/`zmag`), IMU2 acceleration and rotation from `SCALED_IMU2` (mG and mrad/s, converted to m/s² and rad/s as the titles say), pressure from `SCALED_PRESSURE`, battery voltage (V) and current (A) from `SYS_STATUS` (65535 mV and -1 cA mean "not available" and are left out); one line of GPS and one of the battery. While the pointer is over a plot, next to egui_plot's crosshair a small popup shows the time at the pointer and, for every series of that plot, its colour swatch, name (as in the legend) and the value of the sample nearest in time (not interpolated) with the plot's unit, highest value first as the lines are drawn; a series whose line is within 6 px of the pointer (vertically, at that time) is in bold with an outlined swatch, so overlapping lines are told apart. It works the same live and replaying a `.tlog`. There is no high-g plot: the firmware sends nothing of that sensor. At the top, the session bar (see "Sessions"): a name field and Start; while a session runs the name, the elapsed time, a note field with Mark (Enter in the field marks too) and Stop, and one line with the marks so far; after Stop `Saved: <folder>` or the error. Start is disabled with the reason next to it when not connected, replaying a `.tlog` ("replaying a recording") or run with `--no-log` ("recording is off (--no-log)"); a session sends nothing, so Operate and Configure both can start one. |
| Radio | 16 channel bars from `RC_CHANNELS` (microseconds, scale 800 to 2200 with marks at 1000, 1500 and 2000, the number beside each bar, `chancount` respected, 65535 = unused), the link quality (`rssi`, 255 = unknown), the age of the last `RC_CHANNELS` ("no stick data" when it is older than 2 s or was never seen: the bars are then empty), the RC row of `MARV_DEVICE_STATUS` (id 7) and `RADIO_STATUS` when that message arrives. Four function bars from `MANUAL_CONTROL`, which the vehicle computes from the mapping (`docs/link-contract.md` section 12): Roll (`y`) and Pitch (`x`) and Yaw (`r`) on -1000 to 1000, Throttle (`z`) on 0 to 1000, each with its expected direction ("stick right: +", "stick forward: +", "yaw right: +", "throttle up: 1000"), and "no data" when none arrived or the last is older than 2 s; the station computes no value of its own. On channels 1 to 8, once the parameter list is complete, marks at that channel's `MIN`, `TRIM` and `MAX` and its function (from `RCMAP_*`); a vehicle without `RC1_MIN` gets the line "this firmware has no RC mapping parameters" instead. Calibration (Configure sessions only, `docs/link-contract.md` section 12): "Calibrate radio" is disabled, with its reason, on an Operate session, while the sticks are stale and until the parameter list is complete; beside it the text "transmitter on in its CRSF model; battery unplugged, no propellers". Capture: the running minimum and maximum of each of the 16 channels is drawn on its bar while the user moves every stick and switch to both ends, then Done; centre: "Centre the sticks, throttle down, then OK" takes the current values as the trims; review: channels 1 to 8 as current -> captured `MIN`/`TRIM`/`MAX` (a channel whose range does not reach below 1300 and above 1700 is "not moved" and is not written; `TRIM` is clamped into [`MIN`, `MAX`]), channels 9 to 16 shown and never written; Apply sends one `PARAM_SET` per value that differs from the vehicle's (at most 24) and Cancel, in any step, sends nothing. After Apply the line "Applied in RAM; not saved to flash" stays, with a "Save to flash" button (command 245 with param1 = 1, the Parameters tab's gate), until a save is accepted; nothing is saved automatically. The station computes no normalised value and sends no radio-calibration command. Mapping (Configure sessions only, same section of the contract): four drop-downs Roll / Pitch / Throttle / Yaw -> channel 1 to 8 (`RCMAP_*`) and, per function, "Reversed" (`RCn_REVERSED`) and "Dead zone (us)" (`RCn_DZ`, 0 to 200) of the channel the function is mapped to in the map being edited (stage Roll -> channel 5 and its Reversed box edits `RC5_REVERSED`). Edits are only staged (a "Staged: ..." line lists them); "Apply mapping" sends one `PARAM_SET` per staged value that differs from the vehicle's and "Revert" drops them and sends nothing; the same "not saved" line and "Save to flash" button as the calibration follow. Two functions on one channel show a warning and do not block Apply. The section is disabled, with its reason, on an Operate session, until the parameter list is complete, on a vehicle without `RC1_MIN`, and during a calibration; "Calibrate radio" is disabled while mapping edits are staged. Reversal is never inferred. |
| Messages | Every message that arrives: rate, count, age and all its fields. Any numeric field can be plotted. Nothing here knows a particular message. |
| Devices | The health of each device of the flight controller (`MARV_DEVICE_STATUS`, `docs/link-contract.md` section 8): one row per device in `device_id` order with its state (coloured), last error, samples and samples per second (computed from two successive rows), age of the last sample, errors, timeouts, recoveries, attempts, overflows, configuration restores, flags (HIL, self-test) and how long ago the row arrived (a row older than 15 s is shown as stale, not removed). The table is read only; above it the buttons Calibrate gyro, Calibrate accelerometer (level) and Cancel calibration, a **Compass** panel (below) and, after a calibration that ended accepted, a bar "Gyro, accel calibrated: in use now, not saved to flash" (the calibrations done so far) with Save to flash (command 245, param1 = 1) and Discard (245, param1 = 0: returns to the saved settings and drops every unsaved change) that stays until a save or a reload is accepted, from any tab, or the vehicle reboots (on USB a reboot starts a new session, so the bar is simply cleared; on a link that survives the restart the line "unsaved calibration lost at reboot" stays); after a calibration the station reads that calibration's offsets back, and after an accepted reload from flash (Discard, or the Parameters tab) it fetches the whole parameter list again, so that the Parameters tab shows the real values; and per row Restart (IMU1, IMU2, BARO, MAG, GNSS, BATT, SD) and Self-test (IMU1, IMU2, MAG), all Configure-only (section "Commands" below). **Compass panel** (decision `_plan/decision-compass-cal-20261120.md`, `docs/link-contract.md` section 16; the maths is `crates/gcs-magcal`, the panel is the view and the wiring): the line "Compass calibration: ..." says what the vehicle's parameters hold once they are loaded: "Not calibrated" (`MAG_CAL_TYPE` 0 and the offsets all zero), "Best effort" (type 1; type 0 with offsets reads "best effort (older calibration)"), "Guided" (type 2 with `MAG_FLD_UT` above 0), with the stored expected field and inclination when present. The main button is "Calibrate compass (guided)": six poses (level, upside down, nose down, nose up, left side down, right side down) in any order, and in each one slow turn about the vertical; there is no button per pose, the accelerometer recognises the pose and the turn completes it. While it runs the panel shows, on the left, a 3D quad drawn with the egui painter (two arms, four rotor 16-gons, a red nose marker, a curved arrow for the turn; a grey ghost in the pose to take, which is the one being held or else the next incomplete one, and the live quad at the tilt of the HIGHRES_IMU accelerometer, because the real firmware sends no `ATTITUDE`) and, on the right, six tiles, one per pose, each with a ring of 12 sectors of 30 deg that fills as the turn progresses; below, the instruction ("hold it as shown", "turn it slowly once round about the vertical"), the seconds left for the pose (90 s each) and the samples kept; "Cancel compass calibration" stops the run. The view reads the calibration in force (`MAG_OFF*`, `MAG_DIAG*`, `MAG_ODIAG*`) and the latest `GPS_RAW_INT` when it starts, feeds the session every HIGHRES_IMU `id` 0 message, the armed state of HEARTBEAT, any change of those nine parameters (the run is abandoned) and the time on every frame, so a lost link ends in "Timed out waiting for <pose>: start again" after 90 s. A refusal to start (no GPS fix of type 2 or better younger than 100 s, an invalid stored matrix) and every ending (a pose that never completes, readings too scattered, an implausible field strength, distortion, a disturbed field, armed, cancelled) show the session's message as it is, and nothing is written. Only an accepted run writes, one `PARAM_SET` at a time, each judged by its echo: `MAG_CAL_TYPE` 0, then `MAG_OFFX/Y/Z`, `MAG_DIAGX/Y/Z`, `MAG_ODIAGXY/XZ/YZ`, `MAG_FLD_UT`, `MAG_FLD_INC`, then `MAG_CAL_TYPE` 2 (thirteen writes); a refused or unanswered write stops the sequence and names the parameter. After the last write the station fetches the parameter list again and compares all thirteen with what it wrote; only after that does the panel say "Compass calibrated (guided): in use now, not saved to flash" with the fitness, the offsets and the measured inclination against the model's, and the bar offers Save to flash (command 245, param1 = 1) and Discard until a save or a reload is accepted. After a failed write there is no Save: the panel says the vehicle's compass is marked not calibrated (type 0) and that Discard or a reboot restores the saved values. When the read-back differs from what was written, or does not complete within 30 s, the station writes `MAG_CAL_TYPE` 0 once more (judged by its echo like the others, so that type 2 stands only for a set that was read back equal) and then says the same; if that write is refused or unanswered it says instead that the vehicle may be using an unverified compass calibration and that Discard or a reboot restores the saved values. Save is never offered after a failure; Discard is. Configure-only: the buttons are disabled with their reason on an Operate link and while the vehicle is armed. The secondary button "Quick calibration (best effort)" (with the line "Heading errors of 15 deg and more were measured with this kind of calibration") is the old command 241 (param2 = 1), unchanged. The heading check (it informs, never blocks): "Start heading check", walk a straight line with the vehicle, "Stop heading check"; the panel compares the tilt-compensated compass heading plus the model declination at the GPS position with the GPS course (ground speed of 1 m/s or more, at least 10 records) and shows the difference, e.g. "within 10 deg" or "off by 14 deg: calibrate again or check what is mounted near the compass". The model date is the computer's clock (the model is valid from 2025.0 to 2030.0). The real board sends rows for ids 1 to 7 and 9; the simulator sends the same eight. Above the table, "Sensor presets" (`docs/link-contract.md` section 13), a station feature: a preset is the station setting several parameters at once, as a verified combination; the firmware knows no preset. Five cards, IMU1, IMU2, BARO, MAG and GNSS, each with a picker of the presets of the table in `gcs_mavlink::presets` (the station computes no score; "Custom (debug)" is offered only with the debug switch on), a radar chart (rings at 25, 50, 75 and 100 %, one spoke per axis, the chosen preset filled orange, the preset under the mouse in the open picker outlined for comparison), the summary, the physical value of every axis with its unit (and the second value, with `model` or `bound` where the number is a model or an upper bound) and one status line. The active preset is the one whose field values all equal the vehicle's (a REAL32 numerically), else "Custom" (read-only: a preset can still be chosen), once the parameter list is loaded ("parameters not loaded" before). Apply writes the chosen preset's fields as ordinary `PARAM_SET` writes (Configure only, disabled with its reason on Operate and while a command runs): first every field whose value goes down, then every field whose value goes up, a field that does not change is not written; the order keeps every intermediate combination valid for BARO (BMP581 Table 7) and MAG (BMM350 Table 5); GNSS writes `GPS_GNSS` before a faster `GPS_RATE_MS` and after a slower one (100 ms needs `GPS_GNSS` 1). The card judges the result from each write's echo: BARO, MAG and GNSS say "Applied in RAM; not saved to flash" only when every echo is back and the values equal the table, with Save to flash (command 245, param1 = 1; the line goes only when a save started after the Apply is accepted); a refused write names the field ("refused: BARO_OSR_P (vehicle uses 16)") and the card then shows what the values now match, often "Custom"; a write with no answer is named too, and values that never arrive give "the vehicle's values differ from the preset table". IMU1 and IMU2: the same writes (the vehicle answers each with the INFO text `<NAME>: applies after reboot`), then the card says "Pending: takes effect after Save and reboot" and has one button, Save and reboot (it asks for a second click, as the Link tab's Reboot vehicle does): it sends command 245 param1 = 1 and, only when that save is ACCEPTED, command 246 param1 = 1; a refused or unanswered save sends no 246 and the card says so ("save refused: ...", "save unanswered", "the vehicle was not rebooted") and offers the button again. Pending belongs to the session (a new session starts without it; the vehicle's running IMU configuration is not on the link). Debug switch: the checkbox "Debug: raw sensor settings" in the header of the section, station-local, off at every start and never saved (a gate of this window only: the firmware knows nothing of it). Off: "Custom (debug)" is not offered in the pickers and nothing raw is editable. On: it is offered in every picker; choosing it shows the card's fields as values to type, the IMU ones marked "applies after reboot", with "Apply fields": one `PARAM_SET` per changed field; the vehicle validates and what it refuses is shown beside the field. Apply, Save to flash, Save and reboot and Apply fields are Configure-only, shown disabled with the reason on Operate. The chart is 180 px tall (radius 70, smaller only as far as needed to keep every axis label inside the card) and the value lines use the normal text size. |
| Parameters | Whatever parameters the vehicle reports: search, a collapsible tree by scope (Flight controller, RC controller, ESC, each with its count) and, inside it, by group (the name's prefix before the first `_`, in natural order, so RC2 before RC10; a name without `_` is in "(other)"); the sections start open and the groups closed, and a search shows only the matching rows with their groups opened and the counts as "matches of total"; edit, write with confirmation (a refused or unanswered write says so), save to and load from a file; the storage buttons Save to flash, Reload from flash, Reset all to defaults and Reset calibration (the two resets ask for a second click). The tab knows the group prefixes of the link contract (`RC<n>` and `RCMAP` are the RC controller, `ESC`, `MOT` and `DSHOT` the ESC, all others the flight controller) and, for the sensor presets, the fields of the table in `gcs_mavlink::presets`: while the Devices tab's debug switch ("Debug: raw sensor settings", one switch for the whole station) is off, every field a preset sets is shown read only with the note "set in Devices > Sensor presets" (a file load that stages such a field does not send it either); with the switch on they are edited like any other parameter (the IMU ones marked "applies after reboot"); a refused write shows as for any parameter. On an operate-only link the list is not downloaded at connect (press Refresh) and the editing controls are shown disabled with the reason. A Unit column and, on hovering a name, a one-line description and the allowed values come from the station's own table of the firmware's 80 parameters, `gcs_mavlink::param_info` (the link carries no metadata, `docs/link-contract.md` section 2); a name that is not in the table shows no unit and no hover. |
| Log | Two kinds of log, in two sections. "Station recording (.tlog)": every session is recorded automatically to `logs/<UTC time>.tlog`, the telemetry-log format Mission Planner, QGroundControl and pymavlink read; a log can be opened and replayed through the same tabs. Under it "Sessions": the folders of `logs/sessions/` newest first (folder, name, start, duration, marks), read when the tab opens and after a session was saved, each with Replay, which opens its `session.tlog` as a replay. "On-board logs (SD card)": the SD row (state, last error, blocks written, age) or "no SD row yet"; Restart SD; Refresh list and a table of the card's logs (number, bytes, MiB, the last one marked "being written"); per row Download to `<folder>/NNNNNNNN.BIN` (the folder is a text field, default the window's log directory; an existing file is never overwritten, the reason is shown), one download at a time with a progress bar (bytes, kB/s), Cancel, the line "Keep the USB cable connected until the download finishes.", and the result or error in words. The file is written as `NNNNNNNN.BIN.part` and renamed to `.BIN` only when every byte arrived; a download that fails (the link closed, e.g. the cable pulled: "Download of log N failed: link lost at X of Y bytes; reconnect and download again"; or no data for 6 s) or is cancelled deletes the `.part` file, so a half file never looks like a log and the log can be downloaded again at once (the review drop-down lists `.BIN` files only). All Configure-only. "Review a log file" (a local file: it works with or without a vehicle, sends nothing and needs no Configure access): a path field with Open and Browse… (the desktop's own file dialog, starting in the log directory, showing `.BIN` files of any letter case, with "All files" as the other choice; choosing a file opens it as Open does, Cancel does nothing; without a desktop file-chooser portal a one-line red reason is shown instead), a drop-down of the `.BIN` files in the window's log directory (newest first), a Review button next to a finished download, or `--review`; a background pass with a progress bar and Cancel reads the file twice (summary, then events; the window never waits for it). It shows the summary (name, size, duration, wall-clock start when the log has a GPS time, records per type, bad and tail bytes), the Events table (time since boot, wall clock, source, text, with a checkbox per source and the first record of a source marked "initial"), Find (a condition such as `BAT.Volt < 14.8`: a mistake is shown with its position and reason before any pass; then the intervals where it held, with the smallest and largest value of each field named, and "N more not kept" past 10 000), and Export CSV (a checkbox per record type, an optional rate in Hz, an optional condition, a folder that starts as the file's own; per type the rows written and skipped and the file, or the error: an existing file is never overwritten and is named). No coordinates are shown. Below, the vehicle's text messages (in a box of their own; the tab scrolls). |
| Map | The vehicle's GPS position and this session's track on a dark map background. The marker is the latest `GPS_RAW_INT` (`lat` and `lon` in degE7) when its `fix_type` is 2D or better and it is younger than 3 s, with an arrow that turns by the `yaw` of `ATTITUDE` (0 = north, clockwise) while that message is younger than 2 s (the HUD's rule), else along the course over ground (`cog`), else a dot (`cog` unknown, 65535). The firmware sends no `ATTITUDE` yet, so on the real board the arrow still follows `cog`; the simulator sends one. A small compass rose sits in the bottom-right corner of the map, just above the map credit; the map is always north up, so north on it points straight up. No marker before the first fix (the world view and "waiting for a GPS fix"); afterwards, when the message is older than 3 s or has no fix, the last known position stays in grey with "no position" or "no GPS fix". The track is every position of the session (at most 10 000, the oldest dropped), drawn as a line, kept while another tab is open and cleared when a new session starts. Controls: "Follow vehicle" (on at the start: the map stays centred on the vehicle; dragging the map turns it off), "Centre on vehicle" (one jump back, while not following), + and - buttons, the mouse wheel to zoom, a drag to pan; the coordinates are written beside them. **Waypoints:** a "Waypoints" button opens a panel at the right of the map (the map is the same without it). A left click on the map adds a waypoint to the end of the current list (while "Add on click" is on, with the panel's default altitude) and a right click within 10 pixels of one removes it; the points are drawn as numbered amber circles joined in order. The panel has a list selector with New, Rename, Duplicate and Delete (Delete and Clear ask first), a table of the points (latitude, longitude, altitude above home, move up, move down, remove), "Add by coordinates" (a pasted `lat, lon` in the latitude box fills both), the count and length of the path, "Centre on waypoints", and a line saying "saved" or why not. Every change is saved at once, one file per list, in QGC WPL 110 text (`<name>.waypoints`, altitude relative to home) in `~/.local/share/marv-gcs/missions` (or `$XDG_DATA_HOME/marv-gcs/missions`); the lists stay when a new session starts. Under "On the vehicle", "Upload to vehicle", "Read from vehicle" and "Clear on vehicle" send the current list to a connected vehicle, read the vehicle's list back and clear it (link contract section 17); an upload or clear counts only after the read-back matches, the vehicle's list is drawn as blue dashed rings beside the edited list's amber discs, and "Copy to a new list" is the only way the vehicle's list reaches a file. The vehicle only stores the list; nothing here makes it follow the list. The editor works with or without a vehicle and over every link, and writes no position of the vehicle to any file or log. **Tiles and privacy:** the background is drawn by the station from OpenFreeMap's vector tiles (<https://openfreemap.org>: no key, no account, no request limit; OpenMapTiles schema, OpenStreetMap data), in a dark style: grey land, lighter roads (motorways the lightest), buildings a step above the land, muted water, light grey labels. The keyless raster maps were not sharp enough or not keyless: Esri's dark map has no tile past zoom 16, CARTO's needs an API key since September 2026 and Stadia's answers 401 without one. The tiles are fetched over HTTPS (`https://tiles.openfreemap.org`) only for the part of the map on the screen, with the User-Agent `MARV-GCS/<version> (MARV V2 ground station)`, and kept on this computer in `~/.cache/marv-gcs/tiles` (or `$XDG_CACHE_HOME/marv-gcs/tiles`) so a viewed area is not fetched again while the server's cache rules allow it. In plain words: the tile server learns which area you view (and your computer's address); nothing else leaves the computer, and the vehicle's position is never sent, only the tile numbers of the area on the screen. The station never downloads an area in advance and has no "offline map"; without a network, only tiles already in the cache show, and the position and track are still drawn. "OpenFreeMap © OpenMapTiles © OpenStreetMap contributors" is always drawn on the map. The tiles hold data up to zoom 14; closer (the map zooms in to 26), the areas, lines and labels of the zoom 14 tile are drawn larger, so the map stays sharp, and a road keeps its width on the ground. The tiles come from the `walkers` crate (MIT) with its `mvt` feature (reading vector tiles), which brings an HTTP client (`reqwest`, `tokio`, `rustls` with `aws-lc-rs`) and, with `mvt`, `egui-wgpu` and `wgpu`: walkers draws vector tiles only with wgpu and the window uses glow, so those are compiled but not used, and the station paints the tiles with egui. Only `marv-gcs` depends on it. |
| Motors | Read-only ESC telemetry from `ESC_STATUS` and `ESC_INFO` (`docs/link-contract.md` section 14): a strip with "ESCs online n/4", the ESC device row's state and the link type, four boxes (Motor 1 to 4) with rpm, voltage, temperature, current and the error percentage (100 x change of `error_count` / change of `counter` between the two latest `ESC_INFO`; blank when they are more than 60 s apart), "OFFLINE" in the title and a red frame when the ESC's `info` bit is 0, values in grey when the message is older than 2 s, "No ESC telemetry from the vehicle" when none ever arrived, and a plot of the four rpm series. Above the boxes, the strip "Motor test (propellers off)" (`docs/link-contract.md` section 15): a switch "Propellers are removed and I accept the risk" (off at every connect and at every new session), a throttle field in percent (0.05 to 5.00, default 5.00) and a duration field in seconds (0.1 to 3.0, default 2.0; a value outside is brought to the nearest limit, and text that is not a number to the minimum), a Spin button in each box (command 209 with that motor, the two fields, param2 0, param5 1, param6 2) and one Stop button (209 with throttle 0 for the motor last spun, motor 1 if none). Spin needs the switch and an open command gate (Configure session, connected, no command outstanding); Stop needs only the gate; a disabled control shows why. A refused test shows the result and the vehicle's STATUSTEXT in the strip. Nothing is sent while a field is edited, and a spin is never repeated. Without a click the tab sends no command. |
| Link | Connection state and counters: frames received and sent, checksum errors, discarded bytes; the session's role (Access) and the last command it refused; the Reboot vehicle button (asks for a second click). |

What the real firmware sends on its USB port, and so what the tabs and the simulator's default streams show: HEARTBEAT and
SYS_STATUS 1 Hz, `MARV_DEVICE_STATUS` (every row at least once a second), GPS_RAW_INT 1 Hz, HIGHRES_IMU, SCALED_IMU2 and
SCALED_PRESSURE 25 Hz each, RC_CHANNELS 5 Hz while the receiver delivers sticks, MANUAL_CONTROL 5 Hz while a stick report is
younger than 1 s (the sticks as the vehicle maps them; the simulator always sends it, from its own RC_CHANNELS and its 44 RC
parameters; as decided in the link contract, not yet seen from the real board), STATUSTEXT. (An older demonstration program
sent four HIGHRES_IMU instances at 50 Hz; the window no longer looks for them.) The simulator's default streams are these plus ESC_STATUS at 10 Hz (rpm 0 on all four motors, as on the real board, until a motor test spins one: command 209 is answered by the table of section 15, ACCEPTED at once, rpm = level x 18.6 on that motor for the duration) and ESC_INFO at 1 Hz (the Motors tab) and ATTITUDE at 10 Hz, which the real firmware does not send yet (a test double for the HUD).

The simulator's compass (`gcs-sim`, modules `mag` and `script`; for testing the guided compass calibration without a board). HIGHRES_IMU's `xmag`, `ymag`, `zmag` are no longer a constant: m = S (r - o) / 100 in gauss, with S and o from `MAG_DIAG*`, `MAG_ODIAG*` and `MAG_OFF*` (single precision, the contract's order) and r = S_true^-1 (R B) + o_true + noise + magnet, where B is the World Magnetic Model (`gcs-magcal`) at the simulator's position and date (2026.9), R the attitude (body from NED) and `SimConfig::mag` holds `S_true` (default identity), `o_true` (0), the noise, a magnet and an anomaly site (inclination shift). Level, facing north, with the default calibration it reads the model field here (about 21.5, 1.4, 43.3 uT, so no longer the constant (0.21, -0.03, 0.42) gauss). `SimConfig::user` is a scripted user who holds each of the six poses and makes one turn of 12 s about the vertical, with the accelerometer and gyro of HIGHRES_IMU consistent with the motion; `skipping(pose)` leaves one pose out; `SimVehicle::set_silent` (link loss) and `SimVehicle::set_armed` (HEARTBEAT armed) do the rest. The quick compass calibration (command 241, param2 = 1) ends by writing the raw-field offsets (`o_true`), S = identity, `MAG_CAL_TYPE` 1 and `MAG_FLD_UT` 0. Tests: `crates/gcs-sim/tests/compass.rs`, six end-to-end runs through `gcs_magcal::session::Session`.

The simulator's parameters (`gcs-sim`, a test double of the vehicle): 93 in all: 13 of its own, the 44 RC parameters, the 13 of the sensor block of `docs/link-contract.md` section 2 (below), then the 23 of indices 70 to 92 with the firmware's types, defaults and ranges: `MAG_OFFX/Y/Z`, `BATT_A_SCALE`, `BATT_A_OFFSET`, `LOG_BACKEND_TYPE`, `AHRS_ORIENTATION`, `MAG_ORIENT`, `MOT_POLES`, `LED_BRIGHT` (70 to 79), `GPS_RATE_MS` (80: INT32, default 200, only 100, 200, 500 or 1000; another value from 100 to 1000 is refused with `GPS_RATE_MS: not 100, 200, 500 or 1000`) and the compass block 81 to 91 (contract section 16; 81 to 89 and `MAG_OFF*` return to their defaults when command 245 with param1 = 3 ends) and `GPS_GNSS` (92: INT32, default 0, 0 to 2; a `GPS_RATE_MS` of 100 is refused unless `GPS_GNSS` is 1, and `GPS_GNSS` cannot leave 1 while the rate is 100). The sensor block: the six IMU parameters `IMU1_ACC_FS`, `IMU1_GYR_FS`, `IMU1_ODR_HZ`, `IMU2_ACC_FS`, `IMU2_GYR_FS` and `IMU2_ODR_HZ` (INT32, the Flight values by default, Reboot-class) and the seven BARO and MAG fields (`BARO_ODR_HZ`, `BARO_OSR_P`, `BARO_OSR_T`, `BARO_IIR_P`, `BARO_IIR_T`, `MAG_ODR_HZ` as a REAL32, `MAG_AVG`, the Balanced values by default). The simulator knows no preset and does not use the table: a write is checked against the parameter's range only, and an IMU parameter that changes is answered with an INFO text `<NAME>: applies after reboot` (the simulator does not reboot, and shapes none of its sensors from these values).

The status at the top right says `Connecting`, `Connected` (with the vehicle's ids), `Lost` (no heartbeat
for 5 s) or `Closed` (with the reason), then the role of the session.

## Commands

Every button above is a MAVLink command to the flight controller (`docs/link-contract.md` section 10): the
`commands` service in `gcs-mavlink` sends one `COMMAND_LONG` at a time (3 sends, 1.5 s apart, `confirmation`
0, 1, 2), follows `COMMAND_ACK` (IN_PROGRESS with a percentage, then the final result) and gives up after
20 s of silence. All are Configure-class: on an Operate link they are shown disabled with the reason, and
while a command is running every command button is disabled ("a command is running"), except Cancel
calibration during a calibration. The Devices tab has Calibrate gyro, Calibrate accelerometer (level) and
the Compass panel (the guided calibration, and the quick best-effort one, which is command 241 with param2 = 1; an accepted calibration is in use at once but saved only by Save to flash, which the Devices tab offers in a bar until it is done), and the Log tab Restart SD; while the quick compass calibration waits or runs it shows "rotate the vehicle through all
orientations" next to the status line. The guided calibration must be finished with the Devices tab showing: if the tab is hidden when the result is ready, the commit fails with its message and nothing is written. The last command, in words ("Gyro calibration: 40 %", "Compass
calibration: accepted", "Reboot: no answer after 3 sends"), is shown under the buttons of the Parameters, Devices and Link tabs and is
`VehicleState.command`. The simulator answers them as the vehicle will, and the firmware answers them too.

## The command line (`marv-cli`)

For a companion computer with no display (a Raspberry Pi cabled to the flight controller's USB port): the
same station without a window. One command per invocation, on one link, plain text on stdout, errors on
stderr. It depends on no GUI crate (`cargo tree -p marv-cli -e normal | grep -ciE 'eframe|egui|winit|wgpu|glow'`
is 0) and on no third-party crate beyond what the station already uses. It presents itself as a ground
station (system 255, component 190), speaks through the same session as the window, and sends every write
through `Session::send`, so the role of the link applies as in the window (below): on a `udp:` link every
command that writes prints `marv-cli: refused: this link is operate-only (...)` and exits 4, having written
nothing. `serial:` and `tcp:` links are Configure.

```
marv-cli --connect <endpoint> [--timeout <s>] <command> [args]      # endpoint: serial:PATH[:BAUD], tcp:HOST:PORT, udp:ADDR:PORT
marv-cli --connect serial:/dev/ttyACM0 status
marv-cli --connect serial:/dev/ttyACM0 param set LOG_RATE_HZ 200
marv-cli --connect serial:/dev/ttyACM0 calibrate gyro
```

Every command first waits for the vehicle's heartbeat (`--timeout`, default 5 s) and fails with exit status 3
and `no heartbeat from a vehicle within N s` without it. Options may stand anywhere on the line.

Every command also prints, on stderr, each STATUSTEXT the vehicle sends while it runs, one line each as it arrives:
`text<TAB>SEVERITY<TAB>TEXT` (so `param set` shows the firmware's reason for a refusal, and `device restart` what the
vehicle says). stdout is not changed by it. `texts`, `watch` and `status` print texts on stdout as before, with no second copy on
stderr (a command whose own stdout shows texts does not echo them). A text belongs to the session, not to the command: one that arrived before the command's own request was sent
(a boot text, an earlier text of the same session) is shown too.

| Command | What it does and prints |
|---|---|
| `status [--for s]` | collects for `--for` (default 2 s), then prints `vehicle<TAB>system=1 component=1 type=...`, `role<TAB>Operate` or `Configure`, `health<TAB>` the sensors SYS_STATUS reports healthy in words (`none`, or `not received`), `devices<TAB>N` (the number of device rows), and the last 3 texts as `text<TAB>TIME<TAB>SEVERITY<TAB>TEXT` |
| `devices [--for s]` | collects for `--for` (default 2 s), then one line per `MARV_DEVICE_STATUS` device: `NAME<TAB>STATE<TAB>LAST_ERROR<TAB>SAMPLES<TAB>ERRORS<TAB>TIMEOUTS<TAB>RECOVERIES<TAB>LAST_SAMPLE_AGE_MS<TAB>FLAGS` (`never` when the device has had no sample; FLAGS as in the Devices tab: `-` for none, `HIL`, `self-test`, `bit N` for an unknown bit, joined by `, `) |
| `texts [--for s]` | STATUSTEXTs as they arrive, until the time is over (default 10 s): `TIME<TAB>SEVERITY<TAB>TEXT`, TIME in session seconds |
| `watch [--for s]` | until the time is over (default 30 s): `device<TAB>` + a device line when a device appears or its state, last error, errors or timeouts change (samples and the age change all the time and do not count), and `text<TAB>` + a text line |
| `param list` | `NAME<TAB>VALUE<TAB>TYPE`, one per parameter, by name; waits for the whole list (on a `udp:` link it asks for it: a read) |
| `param get NAME` | the same line for one parameter; exit 1 for an unknown name |
| `param set NAME VALUE` | waits for the vehicle's echo and prints the same line with the value now in effect; if the echo differs from what was asked (the vehicle refused, which ends the write at once, with one `PARAM_SET` on the link) the line is printed, the message and the vehicle's own `text` line go to stderr and the exit status is 1; a value that does not fit the parameter's type is not sent |
| `param set NAME VALUE --debug` | the gate for raw sensor settings, which is the station's own (the firmware knows nothing of it): without `--debug`, `param set` of a field a preset sets (the table in `gcs_mavlink::presets`: `IMUn_ACC_FS`, `IMUn_GYR_FS`, `IMUn_ODR_HZ`, `BARO_ODR_HZ`, `BARO_OSR_P`, `BARO_OSR_T`, `BARO_IIR_P`, `BARO_IIR_T`, `MAG_ODR_HZ`, `MAG_AVG`) refuses with one line naming `--debug` (exit 2) before connecting, so nothing is sent; with `--debug` it is sent. `--debug` applies to `param set` only |
| `param save` / `param reload` | `MAV_CMD_PREFLIGHT_STORAGE` with param1 = 1 / 0 |
| `param reset-all --yes` / `param reset-calibration --yes` | param1 = 2 / 3; without `--yes` they refuse (exit 2) before connecting |
| `calibrate gyro` / `accel` / `compass` / `cancel` | `MAV_CMD_PREFLIGHT_CALIBRATION` (gyro param1 = 1; accelerometer param5 = 4, one position, level; compass param2 = 1, the quick best-effort method: 30 s of rotating the vehicle through all orientations, the CLI prints that as its first line (the guided calibration is the Devices tab's Compass panel only); cancel all zeros) |
| `device restart NAME` / `device selftest NAME` | `MAV_CMD_MARV_DEVICE`; NAME is IMU1, IMU2, BARO, MAG, GNSS, BATT, RC, SD, PSRAM, LED or ESC (any case); every name is sent and the vehicle decides (it denies a restart of RC; the window's buttons offer only the ones it accepts) |
| `reboot --yes` | `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` param1 = 1; without `--yes` it refuses (exit 2) before connecting |
| `logs list` | the logs on the vehicle's card, one line each, `NUMBER<TAB>SIZE` (bytes), then `last<TAB>N`: the number of the log being written, or the highest present when none is open; numbers have gaps; an empty card prints nothing on stdout and `marv-cli: no logs on the vehicle` on stderr, exit 0. Configure links only (`serial:`, `tcp:`) |
| `logs get N [-o FILE] [--force]` | lists, then downloads log N to FILE (default `NNNNNNNN.BIN`, the card's name, in the current directory); prints `downloaded<TAB>FILE<TAB>SIZE bytes`. Progress on stderr, at most twice a second: `log N: DONE of SIZE bytes (P %)`. An existing file is refused (exit 1) unless `--force`, which truncates it when the download starts, so a failed download leaves the partial new data in its place; a log that is not in the list is exit 1 and creates no file; a download that ends without all its bytes is exit 1 and keeps the partial file, saying how many bytes are in it. Configure links only. See "Logs" below |
| `mission upload FILE.waypoints` | sends the waypoints of a QGC WPL 110 file (the file the window's map tab writes: row 0, the home placeholder, is skipped; rows 1 to n are `MAV_CMD_NAV_WAYPOINT` in frame 3; a row off the globe or with a non-finite altitude is exit 1 naming its line, nothing sent), then reads the list back and prints one line `verified<TAB>N waypoints<TAB>plan id 0xHHHHHHHH` (exit 0) only when the vehicle accepted it and the read-back equals what was sent. Otherwise exit 1 and one line on stderr with the reason: the vehicle's refusal as `refused by the vehicle: MAV_MISSION_NO_SPACE (4): the vehicle holds at most 16 waypoints`, `no answer from the vehicle`, a count, waypoint or plan id that differs; followed by `; the vehicle holds N waypoints, plan id 0x...` when the read-back completed (exit 3 when the link went down). Slow on a lossy link: every lost frame costs 1.5 s (docs/link-contract.md, section 17). Works on every link (Operate class) |
| `mission download [FILE.waypoints] [--force]` | the vehicle's list: `SEQ<TAB>LATITUDE<TAB>LONGITUDE<TAB>ALTITUDE` per waypoint (`SEQ` from 0, degrees with 7 decimals, metres above home), then `N waypoints<TAB>plan id 0xHHHHHHHH`. With a file name it also writes the list as a `.waypoints` file the map tab opens (row 0 = the first waypoint's position) and prints `written<TAB>FILE`; an existing file is refused (exit 1, before anything is sent) unless `--force` |
| `mission clear` | empties the vehicle's list and reads it back: `verified<TAB>0 waypoints<TAB>plan id 0x00000000`, exit 0 only when the read-back is empty; a refusal is exit 1 as for `upload` (the vehicle refuses while armed: `MAV_MISSION_ERROR (1)`) |
| `record FILE [--for s]` | records the link to a `.tlog` (the session's own recording) for `--for` (default 60 s), then prints `recorded<TAB>FILE<TAB>N frames<TAB>N bytes`; an existing file is never overwritten (exit 1) |
| `session export FILE.tlog [--name N] [--out DIR]` | no `--connect`, no vehicle: converts a recording into a session folder (see "Sessions"). `--name` defaults to the file's name without `.tlog`; `--out` defaults to `sessions/` next to the file. Prints `session<TAB>FOLDER`, then `FILE.csv<TAB>N rows` for each CSV, `skipped<TAB>MESSAGE<TAB>WHY` for messages not written, and `marks<TAB>N`. An existing folder is never overwritten (exit 1); a file with no frames or one that cannot be read is exit 1 |
| `session list [DIR]` | no `--connect`: the sessions in DIR (default `logs/sessions`, where the window puts them), one line each: `FOLDER<TAB>NAME<TAB>START<TAB>D s<TAB>N marks`; none (or no such folder) is exit 0 with a line on stderr |
| `log report FILE.BIN --mass-kg KG --kf K [--poles N] [--accel-lpf-hz HZ]... [--json FILE]` | no `--connect`, no vehicle: the flight report of a DataFlash log the firmware wrote (see "Flight report" below); text on stdout, and with `--json` the same content in FILE (an existing file is never overwritten, exit 1). `--mass-kg` (kg) and `--kf` (N/(rad/s)^2, one motor) are required: the tool has no airframe of its own. `--poles` defaults to the log's `MOT_POLES` parameter; with neither, every section that needs the rotor speed says "give --poles: the motor's magnet count" and the rest of the report is still made. `--accel-lpf-hz` (repeatable, Hz above 0) adds a low-passed share to the vibration section's 1 g window. A missing or bad argument is exit 2 with the reason; an unreadable file is exit 1 |

Output is one record per line; fields are separated by a TAB, so `cut -f`, `awk -F'\t'` and `grep` do. A
command that sends a MAVLink command follows the commands service's record to its end and prints a line each
time it changes: `gyro calibration: waiting for the vehicle`, `gyro calibration: 40 %`, then the result
(`gyro calibration: accepted`; `restart RC: denied`; `... no answer after 3 sends`). One command at a time, as
in the window. Ctrl-C ends the process at once; a recording keeps everything written so far (the file is not
buffered by the CLI), and a calibration the vehicle has begun goes on, so `calibrate cancel` from the next
invocation stops it (the vehicle serves one station at a time on its USB port, so the next invocation can only
connect after the first has let go).

| Exit status | Meaning |
|---|---|
| 0 | success (a command: the vehicle's result is accepted) |
| 1 | the command failed: not accepted (denied, failed, cancelled, no answer, ...), a parameter echo that differs, an unknown parameter, a recording that cannot start; for `logs`: the list did not arrive, the log is not listed, the file exists or cannot be written, the download is incomplete |
| 2 | usage error (unknown command, a missing argument, a bad number, `--yes` or `--debug` missing); nothing is opened |
| 3 | no link, no heartbeat within `--timeout`, or the link went down |
| 4 | refused: the link is operate-only and the command writes (`marv-cli: refused: this link is operate-only (SetParam)`) |

### Logs

`logs list` and `logs get` download the `.BIN` logs the firmware writes to its SD card, over the same USB
cable, with ArduPilot's MAVLink log download (`docs/link-contract.md` section 11). The file is the card's file,
byte for byte, and opens in MAVExplorer, UAV Log Viewer and Mission Planner. Log numbers are the card's, with
gaps; the log being written can be downloaded too, up to what the vehicle has flushed (at most about 1 s
behind), and logging goes on meanwhile.

- Speed: the vehicle sends 4 frames of 90 bytes per 5 ms, so about 70 kB/s at best: a log of 10 MB takes
  about 2.5 minutes and one of 161 MB about 37 minutes. The command prints its progress and does not hurry.
- The station asks for 512 frames at a time and asks again from the first missing byte when a second passes
  without news; after 5 fruitless tries at the same byte it stops with
  `no data from the vehicle at byte N after 5 new requests` (exit 1) and keeps what it has in the file.
  A vehicle that does not have the log gives the same silence, so `logs get` lists first and refuses a number
  that is not listed. Ctrl-C ends the process; the vehicle stops sending 3 s after the last heartbeat.
- Only on the USB cable (`serial:`) and TCP (the simulator): the radio's node does not serve these messages, and
  on a `udp:` link both commands are refused with exit 4, nothing sent. The window's Log tab does the same with buttons (see "What it does").
- Run against the real board on 2026-11-16 (by the firmware's implementer): `logs list` matched the card; `logs get` of
  two closed logs (1.26 MB and 1.17 MB) was byte-identical (sha256) to the card-reader copies, at about 70 kB/s; the
  log being written downloads up to its flushed size. A USB drop in the middle of a transfer was seen
  on 2026-11-18 (the window's Log tab, cable pulled at about 50 %) to leave the download running for ever and a half file
  named like a log; both are fixed (the session now ends the download when the link closes, and the window's download is
  written to a `.part` file), the fix is tested and was seen on the board the same day (the link-lost line within seconds, the download repeated to the end). `marv-cli logs get` keeps the partial file.
  Not seen yet: a card with hundreds of logs. Lost frames and a silent vehicle are tested against the simulator (`gcs-sim`) only.

There is no JSON or other structured mode: the tab-separated lines are what the session holds, and a structured
form would need a new dependency.

Build for a Raspberry Pi: `cargo build --release -p marv-cli` on the Pi itself, or cross-compiled with
`cargo build --release -p marv-cli --target aarch64-unknown-linux-gnu` (64-bit Raspberry Pi OS; the 32-bit one
is `armv7-unknown-linux-gnueabihf`). It was NOT built or run for the Pi here: only on this Linux x86-64
computer, against the simulator. The user needs the `dialout` group for `/dev/ttyACM*`. Against the simulator:

```
cargo run -p gcs-sim &                                          # on 127.0.0.1:5760
cargo run -p marv-cli -- --connect tcp:127.0.0.1:5760 devices
```

### Sessions

A session is a folder an agent (or a person) reads with Python's `csv` and `json`, `awk` and `grep`: no
MAVLink library. `marv-cli session export FILE.tlog` makes one from any recording (the window's automatic
recordings too); the converter is `gcs_link::session_folder::export_session` (a `.tlog`, an optional parameter
snapshot, optional user marks and a name in; a folder out). Written by hand to the layout below; nothing is
read from the vehicle.

```
logs/sessions/2026-11-17_17-47-08Z_calibration_1/     <UTC time of the first frame>_<name, file-safe> (the time alone when the name is that time)
  session.tlog          the frames (the export of a whole recording is a copy of it): the window's Log tab replays it
  session.json          what this session was: read it first (below)
  HIGHRES_IMU.csv       one file per message type that is in the recording
  SCALED_PRESSURE.csv   ...
```

A folder that exists is never overwritten. The vehicle's own frames only go to the CSVs and the summaries;
the station's own (system 255: its commands, heartbeats, parameter requests) stay in `session.tlog` and are
counted in `station_frames`.

**CSV.** A header row, then one row per message. The first column is `t_s`, seconds since the session's start
(the receive time of the first frame, so a session starts at 0; 6 decimals); the second is the message's own time
(`time_usec` or `time_boot_ms`) when it has one; then the other fields in the order of the `mavlink` crate's
structs, which is the wire order (large fields first), not the order of the XML definition. An array field is
flat: `name_0`, `name_1`, ... An enum is its name (`MAV_STATE_ACTIVE`); a bit-flag field (`HIGHRES_IMU.fields_updated`, the `SYS_STATUS` sensor masks, `HEARTBEAT.base_mode`, ...) is its integer value, so `awk` and Python can mask and compare it (and it has no mean in the summary); an infinite or not-a-number value `nan`;
a text containing a comma is in double quotes. Messages of different instances (`HIGHRES_IMU` `id`,
`MARV_DEVICE_STATUS` `device_id`) are in the same file; the `id` or `device_id` column tells them apart.

**Privacy.** No latitude or longitude is in any CSV or in `session.json`; only `session.tlog` holds them. Every field
named `lat`, `lon`, `latitude`, `longitude`, `*_lat`, `*_lon`, `lat_*` or `lon_*` is dropped (so `GPS_RAW_INT.csv`
has no `lat` and no `lon` column, `GLOBAL_POSITION_INT.csv` keeps `alt`, `relative_alt`, `vx`, `vy`, `vz`, `hdg`); the
messages that are only positions (`GPS_GLOBAL_ORIGIN`, `SET_GPS_GLOBAL_ORIGIN`, `HOME_POSITION`, `SET_HOME_POSITION`,
`MISSION_ITEM`, `MISSION_ITEM_INT`, `COMMAND_INT`) are not written and are listed in `skipped`. Tested with a `GPS_RAW_INT` recording.

**`session.json`** (`schema_version` 1; `t_s` is always seconds since the start):

| Key | Type | Meaning |
|---|---|---|
| `schema_version` | number | 1 |
| `name` | string | the name as given |
| `start_utc`, `stop_utc` | string | `2026-11-17T17:47:08.697Z`; the first and the last frame |
| `duration_s` | number | `stop - start` |
| `source` | string | the input file's name, no directory |
| `vehicle` | object | `system_id` (the sender of the first heartbeat of an autopilot, else of the first frame; null if none) and `firmware` (from `AUTOPILOT_VERSION`: `major.minor.patch`, `-dev`, `-alpha`, `-beta` or `-rc` for an unofficial one, and the 8 custom bytes in hex in brackets when not all zero; null when the recording has none) |
| `station_frames` | number | frames the station sent (in `session.tlog` only) |
| `parameters.at_start` | object | name -> value: the snapshot when one was given, else the first `PARAM_VALUE` of each name |
| `parameters.changes` | list | `{t_s, name, old, new}` for each later `PARAM_VALUE` whose value differs from the one before (a name not in the snapshot starts from its first value) |
| `marks` | list | `{t_s, kind, text}` by time, `kind` is one of: `user` (given by the user), `command` (every `COMMAND_ACK`: `MAV_CMD_PREFLIGHT_CALIBRATION (gyro): ACCEPTED`), `text` (every `STATUSTEXT`: `INFO: ...`), `link` (`link lost` at the last frame before a silence of more than 3 s and `link back` at the first after it) |
| `files` | object | CSV file name -> number of rows |
| `skipped` | list | `{message, why}`: frames not written (positions only, not in the dialect, refused by the decoder) with their count |
| `units` | object | `{MESSAGE: {column: unit}}` for the columns that have one (below) |
| `summary.messages` | object | by message (`NAME`, or `NAME[instance]` for `HIGHRES_IMU` and `MARV_DEVICE_STATUS`): `count`, `rate_hz` (`count - 1` over the time from the first to the last; null with one message) and `fields`: for every numeric column except the time column, `{mean, sd, min, max}` (6 significant digits; `sd` is the sample standard deviation, 0 for one value) |
| `summary.intervals` | list | empty without user marks; else one entry per interval between consecutive user marks, the start and the stop: `{from, to, start_s, end_s, messages}` (`from` is `start` or a mark's text, `to` is `stop` or a mark's text; `messages` as above, over that interval only; a mark belongs to the interval it starts). "Before" and "after" a mark are two entries to compare. |
| `warnings` | list | text: the recording ends inside a record (the frames before it are converted) |

**What a command's mark says.** The `.tlog` records the frames the station sends as well as the ones it receives
(`Recorder::on_sent`), so a `MAV_CMD_PREFLIGHT_CALIBRATION` acknowledgement names the calibration of the station's
latest `COMMAND_LONG` for that command: `gyro`, `compass`, `accelerometer`, `ground pressure`, `radio`, `cancel`. A
recording without the station's frame gives `MAV_CMD_PREFLIGHT_CALIBRATION: ACCEPTED`. An acknowledgement of a command
the dialect does not know gives `command N: RESULT` (read from the payload).

**Units.** The `mavlink` crate does not expose the definitions' units at run time (they are only in its doc comments), so
`units` comes from a table in `crates/gcs-link/src/session_folder.rs` copied from the MAVLink and marv-dialect
definitions, for `HIGHRES_IMU`, `SCALED_IMU2`, `SCALED_PRESSURE`, `SYS_STATUS`, `GPS_RAW_INT`, `RC_CHANNELS` (`chanN_raw`: us),
`MANUAL_CONTROL` (the definition has none) and `MARV_DEVICE_STATUS`. A message that is not in the table has no `units`
entry; add its fields to the table.

**From the window.** On the Sensors tab the bar starts a named session (an empty name is the time of its first
frame), drops marks with an optional note, and stops. While it runs, every frame, received and sent, is also written
to `<log dir>/sessions/.recording-<UTC time>.tlog` (a second file beside the automatic recording, which goes on
unchanged; the recorder takes it with `Command::StartSessionLog` / `StopSessionLog`). At Stop the window calls
`export_session` on that file with the name, the parameter table as it was at the start (when the whole list had been
read; otherwise the first `PARAM_VALUE` of each name stands for it) and the marks, each stamped with the Unix time of
the click. The export runs on a thread of its own: the window keeps drawing and shows "Saving…". On success the
folder is `<log dir>/sessions/<UTC time>_<name>/`, the bar says `Saved: <folder>` and the `.recording-` file is
deleted; on failure the bar gives the error and the path of the `.recording-` file, which is kept (convert it later
with `marv-cli session export`). A session also ends and is written on Disconnect, when another connection or a
replay replaces the link, and when the window closes; closing waits for the export to finish (a large recording
delays the exit by that long; an export that fails is reported on stderr and its `.recording-` file stays). A session
that recorded no frame is not written. The Log tab's "Sessions" list replays any of them. `--no-log` also turns
sessions off.

Not built: the cut of a longer recording to a time range. The converter derives its marks from the frames: the
acknowledgements of commands (calibration, reboot and the others), text messages and gaps in the link; the parameter
values it sees are kept as events in the table, not as marks.

## Two roles: Operate and Configure

The station is either only a telemetry display and controller (Operate) or also the flight controller's
configurator (Configure). The ground station enforces it, never the firmware (`docs/link-contract.md`,
section 9). The link gives the role:

| Link | Role |
|---|---|
| Serial | Configure |
| TCP (today: the simulator) | Configure |
| UDP | Operate |
| Log-file replay | Operate |

On an Operate session `Session::send` refuses every configure-class command (parameter writes,
sending a MAVLink command, storage, calibration, device actions, reboot, log download): nothing is written, and the Link
view shows the refusal. The parameter list is not downloaded at connect (Refresh asks for it), and the
configure controls are shown disabled with the reason, not hidden. A session whose access was not set is Operate: the gate fails closed. A new `Command` variant is classified
in `Command::access` (`crates/gcs-mavlink/src/state.rs`); the code does not compile until it is.

## How it is built

Seven crates; the dependency graph enforces the layers (`docs/architecture.md`).

| Crate | What it is |
|---|---|
| `gcs-transport` | Bytes only: USB serial port, TCP and UDP. It never names MAVLink. |
| `gcs-mavlink` | MAVLink 2 on any reader and writer: framing, one small state machine per sub-protocol (heartbeat, messages, parameters, commands, text, recorder, logs), the vehicle state, the log reader and replay. No transport, no window. |
| `gcs-sim` | The simulated vehicle (`marv-sim`). Written from the specification and never using `gcs-mavlink`, so the two check each other. |
| `gcs-link` | The join, with no GUI: the role an endpoint gives the station (`access_of`, the one mapping) and the start of a session on it, and `session_folder`, the converter of a `.tlog` into a session folder (file writing, so it is not in `gcs-mavlink`). Used by both applications. |
| `gcs-dflog` | Reads ArduPilot DataFlash `.BIN` logs, generically, from the file's own FMT records (`A3 95 <type> <payload>`; every format character decoded and scaled as pymavlink does; resynchronisation over bad bytes and a truncated last record counted, not an error), one record at a time in constant memory, with progress and cancellation; and a one-pass summary (records per type, first and last `TimeUS`, bad and tail bytes, the FMT list, the wall-clock start from a `GPS` record with `GWk` > 0). The event timeline (device state changes, status texts, parameter changes, GPS fix changes, sticks present, dropped records) and the condition search (below), each one pass with progress and cancellation. The CSV export (below), one pass with progress and cancellation. The flight report (below), one pass with progress and cancellation. No GUI, no dependency; the Log tab's "Review a log file" section of `marv-gcs` uses it. |
| `marv-cli` | The command line (`marv-cli`), for a computer with no display: see below. |
| `marv-gcs` | The window: the connection bar and one file per tab; also the link test tool `marv-link-test` (`src/link_test.rs`, `src/bin/marv_link_test.rs`), which builds `MARV_TEST` with `marv-dialect`. |

### Flight report (`gcs-dflog`, `marv-cli log report`)

```
marv-cli log report flight.BIN --mass-kg 1.2 --kf 2.8e-5 [--poles 14] [--accel-lpf-hz 20 ...] [--level-range MIN MAX] [--json report.json]
```

`flight_report_with` (also `flight_report(path, &ReportInputs)` and `flight_report_of`) reads one log of the MARV
firmware, with progress and cancellation, and returns a plain `FlightReport`; `marv-cli` only parses the
arguments and prints it (the text is the report, `--json` the same content as JSON, written with `serde_json`, which
the workspace already uses; `gcs-dflog` itself still has no dependency). Record names and columns come from the
log's own FMT records, never from offsets; a record type the log does not define makes the sections that need it say
"not in this log" and the report goes on. Memory is one small accumulator per half-second bin of armed time, plus
(vibration) one 1024-sample buffer and the accumulators of the interval being read, (tracking) the setpoint of the
last 0.5 s and the accumulators of the flight being read, plus one small entry per hold and per local maximum of the
excursion, never the records (a 100 MB log is never held decoded). The vibration and tracking sections need the
flights, which are known only when the first pass is done, so the stream is read a second time, `IMU`, `NAV` and
`SETP` records only (the ground check: `NAV` and `IMU`): the stream must be seekable (`Read + Seek`; a file or a
`Cursor`), and progress runs from 0 again for it.

Inputs: `--mass-kg` and `--kf` are required (no vehicle number is built in); rotor speed is
`eRPM / (poles / 2) x 2 pi / 60` rad/s, with `--poles`, else the log's `MOT_POLES`. With neither there is no default:
every section that needs the rotor speed (flights, the saturation count, the motor identification, the thrust
constant, the vibration section) says "give --poles: the motor's magnet count", and the rest is still produced.
`--accel-lpf-hz <Hz>` may be given several times; with none, the 1 g window of the vibration section has only its raw
share (the tool does not know the firmware's filter setting, so there is no default).
`--level-range <min> <max>` (two numbers, min below max; anything else is a usage error, exit 2) gives the ends of
the motor command range: the closing block then prints the motor line's speed at both.

| Section | What it holds |
|---|---|
| Header | file name, bytes, bad and tail bytes, duration (first to last `TimeUS`), the inputs and where the pole count came from |
| Health | dropped records (the last `FDS.Dp` and `DSF.Dp`, and their sum), the least `FMn` of each ring, the largest `TimeUS` spacing of consecutive `NAV` records inside one armed window (us), the `DMD` records with `Sat` != 0 inside the flights, the highest `ESC` temperature per motor, the lowest `BAT.Volt` (the first battery instance seen) |
| Armed windows | from `MOT.Arm`: start, end and duration of each stretch with `Arm` = 1 (a log that ends armed ends the window at its last `MOT` record and says so) |
| Flights | see below; a window with none is "armed, not airborne" |
| Per flight | duration; mean throttle stick `RCS.T`; mean `SETP.Thr` in N and as a fraction of mass x g; mean rpm per motor; mean of kf x sum of speed^2 over mass x g; battery volts (minimum) and amps (mean, maximum); mean `DMD` `TX`, `TY`, `TZ` |
| Motor identification | over all armed time, per motor a least-squares line speed = a + b x level (rad/s, levels as sent in `MOT.A0..A3`) with its rms residual, the number of bins, the level range and the mean battery voltage of the bins; the same line fitted to the bins of all four motors together ("pooled"); and the thrust constant (below) |
| Vibration | per flight and per ground run, from the `IMU` records with `I` = 0 (below) |
| Tracking | per flight, how well the `NAV` attitude followed the `SETP` attitude setpoint (below) |
| Ground check of the tilt estimate | per ground run, whether the estimated tilt moved while the vehicle did not (below) |
| Values this log supports | the last block: the vehicle values the log gives, each with the range it is valid for (below) |

**The NAV spacing:** the firmware writes the records of one step in the order `IMU`, `NAV`, `SETP`, `DMD`, `MOT`, and
logs every step while armed but only every 8th while disarmed. The armed state of a `NAV` is therefore that of the
`MOT` record with the same `TimeUS`, and a spacing is counted only when both `NAV` records lie in the same armed
window; the long spacing across an arm or disarm edge is not a logging gap.

**The flight definition** (deliberate, fixed): within an armed window, time is cut in bins of 0.5 s from the window's
start (a partial last bin is left out). For a bin, r = kf x (the mean over its `ESC` records of the sum over the four
motors of speed^2) / (mass x 9.80665 N); a bin with no `ESC` record has r = 0. A flight starts at the first bin with
r >= 0.85 (`FLIGHT_START_RATIO`) and ends before the first later bin with r < 0.5, or at the end of the armed window;
its last bin is dropped; it needs at least 6 bins (after dropping). Every figure "per flight" and the saturation
count use its bins (means of per-bin means, so a busier record type does not weigh more). Why 0.85: on the ground a
run-up below 85 % of the weight is not a flight; in the air the thrust stays near the weight, so a steady hover sits
at r = 1 within a few per cent and a start at 1.0 misses it whenever the given kf is a few per cent high; the few
run-up bins between 0.85 and 1.0 that are included are short against a flight. Round 2 (below) computes r with k1,
which makes the threshold 85 % of the mean flight thrust, nearly independent of the given kf. Why the end at 0.5: a
single bin just under the start line used to split one flight into two, and keeping only the high-thrust bins
biased the thrust constant upward; the hysteresis keeps the whole of a flight, including its slow bins. If round 1
finds no flight in a log that has armed bins with r >= 0.5, the report says "no flight found with the given kf; if
the vehicle flew, kf is probably too low (the largest r seen was X)". Between the
bounds nothing about the thrust is assumed except the one physical fact below.

**The thrust constant:** from lift-off to touchdown the mean thrust equals the weight, so for a flight
k = mass x g / (the time-mean of the sum of speed^2 over the whole flight); pooled over all flights, the time-mean is
taken over all their bins (a weighting by duration). The bounds of a flight depend on the given kf, so it is done twice:
bounds with the given kf give k1; bounds again with k1 give k2. The flights reported are those of the second round, and
the report prints both constants. When a flight bound moved by more than one bin between the rounds (or the number of
flights changed) it says so: the constant then depends on the kf given, and the bounds deserve a look.

**Identification:** per motor, a bin counts when it has at least two `MOT` and one `ESC` record, the standard
deviation (population) of that motor's level in it is below 12 levels, and its mean level is above the idle plateau,
the lowest level that motor was sent while armed, plus 4. The fit uses the bin means and needs three such bins at two
or more levels, else "not enough". The pooled fit uses the points of the four motors (its "bins" are points). A fit
whose level range (max minus min of the bin means) is under 25 % of its mean level is printed with "ill-conditioned:
narrow level range" (text and JSON): the slope is then poorly determined. The line and k are **valid only for the
level range and the battery voltage seen** in those bins (the report says so next to them); k compares with the
given kf, the report prints the ratio.

**Vibration:** the intervals are each flight and each ground run: armed, not in a flight (the dropped last bin of a
flight counts as in it), consecutive bins whose mean level of the four motors is at least 20 levels above the mean of
the motors' idle plateaus, at least 4 bins. For each, from the `IMU` records of instance 0 (`GyrX..Z` in rad/s,
`AccX..Z` in m/s^2) inside the interval:

- the sample rate, the median of the `TimeUS` spacing; below 400 Hz the report prints the rate and "too slow for
  spectra" and gives no spectra (the other figures are still given);
- the standard deviation per axis of gyro and accelerometer, and of the accelerometer magnitude;
- the Welch spectrum per axis: segments of 1024 samples, Hann window, 50 % overlap, mean removed per segment,
  averaged over the segments (a radix-2 FFT in `gcs-dflog`, `fft_in_place`). The spectrum is scaled so that its
  sum is the variance. For each of the six axes the three largest peaks (local maxima of the averaged spectrum): the
  frequency, and the amplitude as the rms of the signal within +-2 bins of the peak (a sine of amplitude A reads
  A / sqrt 2);
- the rotor band of the interval: the lowest and highest, over the four motors, of the mean rotor frequency
  (rpm / 60, over the interval's bins), and per axis the share of the total variance within that band widened by 10 %
  on each side, and within twice the band (widened the same way). A band reaching above half the sample rate is
  seen folded: the report says "folded" and prints the ends as `|f - fs x round(f / fs)|`; the share then counts
  every spectrum bin at which some frequency of the band shows;
- the 1 g window: the share of the raw accelerometer samples with 0.9 g <= |a| <= 1.1 g, and the same after a
  first-order low-pass `y += (x - y) x dt / (dt + 1/(2 pi fc))` (seeded with the interval's first sample, `dt` the
  spacing of the samples) for each `--accel-lpf-hz` given. It mirrors what the firmware's estimator does; the
  firmware's setting is not known to the tool.

How to read it: a rotor-band share near 1 on an axis says the vibration of that axis is the rotors' (their speed or
twice it, when the blades or the motors are unbalanced); a large peak elsewhere is a structural resonance or a
neighbouring source; a ground run next to a flight shows what is the motors and what is the air. A raw 1 g share
well below 1 with a filtered share near 1 means the vibration is fast enough for the filter at that cut-off to remove.

**Tracking:** from the `NAV` (`Q1..Q4`, the estimated attitude quaternion [w, x, y, z], body to NED; `GX`, `GY` body
rates in rad/s) and `SETP` records inside each flight. Only a `SETP` with `K` = 1 (angle mode: `Q1..Q4` the attitude
setpoint) is used, paired with the `NAV` of the same `TimeUS`; the `SETP` records inside a flight with another `K` are
counted and printed ("N records with another setpoint kind, not analysed"), nothing else is read of them. The
quaternions are turned into roll, pitch and yaw (ZYX) and printed in degrees. The error is always **setpoint minus
estimate**, the yaw difference wrapped to +-180. Without `NAV` or `SETP` in the log the section says "not in this
log"; without a pole count it says "give --poles", since the flights need the rotor speed. The analysis definitions
below are `pub const` in `gcs-dflog` (`tracking.rs`); they are not vehicle numbers and the tool has none.

- Per flight and axis (roll, pitch, yaw): the minimum and maximum of the setpoint (yaw as the raw angle in
  (-180, 180]), the mean and the rms of the error, and the error of largest magnitude (signed) with its time.
- Excursion (roll and pitch): at each record, the minimum and maximum of that axis's setpoint over the preceding
  `EXCURSION_WINDOW_S` = 0.5 s (the current record included, kept in a ring buffer; at the first record of a flight
  the window is that record alone). The excursion is how far the estimate lies outside [minimum, maximum] (0 when
  inside), so with a constant setpoint any offset of the estimate is an excursion. Per flight and axis: the largest
  excursion, its time, the estimate, the end of the range it passed, and the body rate of that axis (`GX` for roll,
  `GY` for pitch) in deg/s at that record ("none" when the estimate never left the range). For the whole log: the 3
  largest excursion events per axis, taken greedily from the largest: events are the local maxima above 0 of the
  excursion within a flight (the first record of a plateau), and a local maximum less than
  `EXCURSION_SEPARATION_S` = 1.0 s from a larger one already taken is skipped.
- Holds (roll and pitch): the records of a flight are cut into runs: a run starts at a record and goes on while the
  axis setpoint stays within `HOLD_TOL_DEG` = 1.0 deg of its value at the run's first record; the first record that
  leaves starts the next run. A run is a hold when it lasts at least `HOLD_MIN_S` = 0.5 s (first to last record) and
  the setpoint at its first record is at least `HOLD_MIN_DEG` = 10.0 deg away from 0 (either way). Its settled
  figures are means over the records from `HOLD_SETTLE_S` = 0.3 s after its start to its end: mean setpoint, mean
  estimate, and the settled error (setpoint minus estimate). Per axis for the whole log: the number of holds and the
  median and the largest magnitude of the settled error (the median of an even count is the mean of the two middle
  values); the text lists up to 10 holds per axis, largest magnitude first (start, duration, mean setpoint, mean
  estimate, settled error), the JSON all of them in time order.

**Ground check of the tilt estimate:** with the motors running on the ground the vehicle does not rotate, so the
estimated tilt must not move; a filter fault shows as a tilt shift that the gyro does not confirm. One block per
ground run (the vibration section's definition), from the `NAV` records and the `IMU` records with `I` = 0, in the
second pass. Figures only: there is no verdict and no threshold. Roll and pitch come from the `NAV` quaternion with
the same formulas as the tracking section, degrees.

- Reference interval: the `GROUND_REF_S` = 1.0 s that ends at the run's start. Reference tilt: the mean estimated roll
  and pitch over it; gyro bias: the mean `GyrX` and `GyrY` over it; accelerometer reference tilt: the tilt implied by
  the mean acceleration over it. With fewer than `GROUND_REF_MIN_RECORDS` = 20 `NAV` records or 20 `IMU` records in
  it the block says so (with the counts) and gives no figures. Both constants are `pub const` in `gcs-dflog`
  (`ground.rs`).
- During the run, per bin of 0.5 s ([`BIN_US`], counted from the run's start, its time the start of the bin): the
  mean estimated roll and pitch; the gyro-integrated rotation since the run's start about body x and y, the sum of
  (gyro minus bias) x dt over the instance-0 samples in degrees (a small-angle sum: no rotation composition; dt is
  the time since the previous sample, the first one's since the run's start, so a sample's rate counts for the
  interval ending at it); and the tilt implied by the bin's mean acceleration (roll = atan2(-AccY, -AccZ), pitch =
  atan2(AccX, hypot(AccY, AccZ))).
- Per ground run: the reference roll and pitch; the largest estimate shift from the reference per axis (signed, with
  its bin time); the gyro-integrated rotation at the last sample of the run (x, y); the largest unexplained shift per
  axis with its bin time, where the unexplained shift of a bin is (the bin's mean estimate minus the reference) minus
  (the mean, over the bin's instance-0 samples, of the gyro-integrated rotation up to each sample), roll with x and
  pitch with y, so that like is compared with like; and the largest shift
  of the accelerometer-implied tilt of a bin from its value over the reference interval, per axis. "Largest" is the
  largest magnitude, kept signed. A log without `NAV` or `IMU` says "not in this log"; a
  log without a ground run says "no ground run".

**Values this log supports:** the last block of the report. It gathers, in one place, the vehicle values the log
gives and the range each is valid for; it computes nothing a second way (every figure is one of the sections above)
and applies nothing anywhere. Its title says the values are valid only for the levels, speeds and battery voltage seen
in this log.

- Thrust constant: the round-2 value, the smallest and largest per-flight value, the number of flights.
- Motor line: the pooled fit (speed = a + b x level, rad/s), its level range, rms residual and mean battery voltage,
  "ill-conditioned" when the fit says so. With `--level-range <min> <max>` also the line's speed at `min` and at
  `max`, each marked "extrapolated" when it lies outside the fitted level range.
- Hover: over all flights weighted by duration, the mean throttle stick (`RCS.T`) and the mean commanded collective
  as a fraction of mass x g, each with the per-flight smallest and largest, and the lowest and highest of the flights'
  lowest battery voltage (the hover stick depends on the voltage).
- Accelerometer low-pass: the share of samples inside the 1 g window over all flights, weighted by sample count, raw,
  and for each `--accel-lpf-hz` given after that low-pass; with none, only the raw share.
- A part whose input is missing prints the usual line: "give --poles", "not in this log (...)" or "no flight".

**Privacy:** the report reads no position. The analysis never opens the fields of `GPS`, `GPS2`, `GPA`, `POS` or
`ORGN` records (not even their time), and the report has no field that could hold one. A test puts a GPS record with a
known position in a synthetic log and checks that nothing of it, not even its late timestamp, reaches the report; the
command-line test checks the text and the JSON for it too.

### Searching a log with a condition (`gcs-dflog`)

A condition is a small expression over the fields of a log's records; the result is the list of stretches of
the log where it held (start and end `TimeUS`, duration, and the smallest and largest value of each field the
condition names; at most 10 000 are kept, the number of further ones is reported). Names are those of the log's
own FMT records.

| Write | Meaning |
|---|---|
| `TYPE.Field` | the field of the latest record of that type (of any instance) |
| `TYPE[n].Field` | the same, of the latest record whose instance field (the first column after `TimeUS` named `I`, `Inst`, `Instance` or `Id`) is `n` |
| `15`, `14.8`, `1e-3` | numbers |
| `+ - * /`, `-x`, `abs(x)` | arithmetic (division by zero is NaN) |
| `< <= > >= == !=` | comparisons, not chained; any comparison with a NaN is false |
| `not`, `and`, `or`, `( )` | logic, in that order of precedence, weakest last |

As in pymavlink, the condition is evaluated after each record of a type it names, over the latest record of each
named type, and it is false until every named type has had a record. Examples: `BAT.Volt < 14.8` (low battery),
`IMU[0].AccZ < -15` (a hard knock on the first IMU), `DEV[7].Errs > 100 and GPS.Status < 3` (the radio
receiver's error counter is high while the GPS has no 3D fix). A wrong name, a text field used as a number or
`[n]` on a type without an instance field is reported with its position before the pass starts.

### Exporting a log to CSV (`gcs-dflog`)

`export_with` writes the chosen record types of a log as CSV files in one pass, with progress and cancellation,
optionally at a limited rate and only where a condition holds. It is for the post-processing that other tools
do better than the station (a spreadsheet, a plotting script).

- **Files**: one per chosen type that has at least one row written, `<log name without extension>_<TYPE>.csv`
  (`00000039_IMU.csv`) in the chosen folder, which is created if needed. A file is created new and never
  overwritten: if any target file exists the export is refused before anything is written, naming the file (as
  the log download does). A type that is not in the log, or whose rows were all left out, makes no file. Writes
  are buffered. If the pass is cancelled, or fails, the files written so far are left as they are, each with
  its header and the rows up to that point, and the result (or the error) lists them.
- **Format**: the first line is the type's FMT column names; then one line per record, fields as decoded
  (after scaling), separated by `,`, lines ended by `\n`. Numbers are the shortest text that reads back to the
  same value (integers without a decimal point, never an exponent). Text is quoted with `"` when it holds a
  comma, a quote or a line break, with quotes doubled (RFC 4180). An `a` array is one quoted field of its 32
  values joined with `;`.
- **Rate limit** (Hz): per type AND instance (the instance field as in the conditions; a type without one is a
  single instance), a record is kept when none of its type and instance has been kept yet, or when its `TimeUS`
  is at least `1e6 / rate` microseconds after the last kept one. A type without `TimeUS` is not limited.
  (mavlogdump's `--reduce-rate` mixes the instances; this does not.)
- **Condition**: only the records at which the condition holds are written, with the semantics of the search
  above (latest value of each named type; a record that makes it true is written, a record that makes it false
  is not; a record of another type is written while the condition holds). The condition is applied first, then
  the rate limit.
- **Result**: per type, the rows written and the records skipped by the rate limit and by the condition, and
  the file; and whether the pass was cancelled.

- `docs/link-contract.md`: what the flight controller and the ground station must do on the wire (the
  device diagnostics message and the Devices view are its section 8), and the design of the two crates on
  the flight-controller side.
- `docs/extending.md`: how to add a tab, a service, a transport, a preset plot. A new parameter or a new
  standard message needs no change here.

## Tests

```
cargo test --workspace          # 962 tests (11 more are ignored: they need a board or a recording, or only time a pass), about 60 s; needs /usr/bin/socat for the serial tests
cargo clippy --workspace --all-targets -- -D warnings
tools/board_check.sh <dir>      # with the board on its debug probe and its USB port cabled: about 3 minutes
```

`tools/board_check.sh` flashes the demonstration firmware from a checkout of
<https://github.com/Lmx154/mavlink-device> (its path in `MAVLINK_DEVICE_REPO`, default
`~/Documents/MARV-V2-drivers/mavlink-device`; that repo fetches the other two by itself), runs the board tests
(`crates/marv-gcs/tests/board.rs`, ignored by `cargo test`), reads the recording with pymavlink when a
Python that has it is given in `CHECK_PYTHON`, and takes four pictures of the window. It restarts the
board three times: a normal session, a program that only listens, and one that keeps DTR low.

What the tests stand on:
- Each protocol state machine is tested with plain calls and a made-up clock: retries, timeouts and
  frame loss are exact, not timing dependent.
- End to end (`crates/marv-gcs/tests/e2e*.rs`): the real transport, MAVLink layer and session against the
  simulated vehicle over TCP and over a pseudo serial port (and, with a hand-played vehicle, over UDP): connect, loss of the vehicle and recovery,
  message rates and values, the full parameter list with 20 % of frames dropped in each direction, writes
  confirmed, refused and unanswered, both parameter encodings, recording and replay.
- The command line (`crates/marv-cli/tests`, 58 tests): every command and option and every error of the arguments, `session export` and `session list` (a recording made against the simulator, converted, refused when the folder exists), `log report` (a synthetic log, the JSON file, the arguments); every command against the simulator over TCP, the log download included (byte for byte, with lost frames, a stalled vehicle, an existing file); no vehicle (nothing listening, a silent peer); the negative control, where a counting writer on an Operate session sees no `PARAM_SET`, no log message and no `COMMAND_LONG` but the capability request while `devices`, `status` and `param list` work; and the dependency rule (no GUI crate in `cargo tree -p marv-cli`).
- The views are drawn and clicked without a window (`egui_kittest`). The HUD was looked at on 2026-11-17 in `--screenshot` pictures against the simulator (moving attitude) and the real board (no attitude: grey horizon and the text); one fault found and fixed (the text lines were drawn over the drawing). The Log, Sensors, Radio and Devices tabs were looked at on 2026-11-16 in pictures taken with `--screenshot` against the real board (one fault found and fixed: the Radio bars of channels 10 to 16 were shifted); the Radio tab's function bars, `MIN`/`TRIM`/`MAX` marks and the mapping section at rest were looked at the same way against the simulator and against the board running the firmware's RC-mapping build (one fault found and fixed: the mapping rows did not line up). Besides that they are checked by tests of the views' logic and by one ignored test that replays a recording of the real vehicle (`MARV_TLOG=file.tlog cargo test -p marv-gcs --test real_recording -- --ignored --nocapture`; 2026-11-16: every Sensors preset, RC_CHANNELS, and the SD and RC rows found their data in a 20 s recording; RADIO_STATUS is not sent on USB). The Devices tab's unsaved-calibration bar was used on 2026-11-18 by the owner against the real board (gyro, accelerometer and compass calibrated, Discard and Save to flash, offsets kept after a reboot). Seen by the owner on the real board, 2026-11-18: the Map tab (tiles, marker, track line, buttons, compass rose); in the Log tab the table of on-board logs, the download's progress bar, the keep-the-cable-connected line, the red "link lost" line of a failed download and the re-download after it, Browse… and its file dialog with the `.BIN` filter, and in the "Review a log file" section Find and Export, the progress bars and the scrolling of the tab; the two battery plots; the Radio tab's calibration screens and the mapping editor in use with a real transmitter; the Devices tab's Sensor presets in use (BARO Smooth and Balanced; IMU1 Smooth, then Flight, with Save and reboot). Not yet seen on a screen: the arrow turned by `ATTITUDE` yaw on the Map tab (the firmware sends no `ATTITUDE`; drawn headless and in a `--screenshot` against the simulator only), the Sensors tab's session bar (idle, running with marks, the saved and error lines, the reason next to a disabled Start) and the Log tab's Sessions list (checked headless only, against the simulator and a temporary log directory), the Sensors plots' hover readout (the popup, its placement near the pointer, the bold and outlined marks for lines under the pointer; checked headless only), and the Parameters tab's read-only rows with their note, its Unit column and the hover on a name.
- Checked with independent implementations: framing against the `mavlink` crate's own reader and writer
  for all 234 message types; the simulator's output with a hand-written decoder; a recorded log read with
  pymavlink (every frame decoded, same counts, both directions).
- On the MARV V2 board over USB (`tools/board_check.sh`, 2026-11-12, one Linux computer): connected within
  20 ms of the open; first byte of the first session after a restart is the start of frame 0, and the
  first 20 s carry 4343 frames with 0 checksum errors, 0 discarded bytes and 0 lost frames; rates 50.000 to
  50.002 Hz per IMU instance, 10.000 Hz pressure, 5.000 Hz GPS, 1.000 Hz heartbeat (that was the demonstration program, not the real firmware's rates above); 13 parameters listed
  in 20 ms; writes confirmed in 20 ms, a read-only one refused, an out-of-range one answered with the
  clamped value; a recording of 1100 frames decoded completely by pymavlink; reconnect in 20 ms.

## Not covered, and known limits

- A motor test is built (propellers off, one motor, 5 % at most, 3 s at most; Motors tab) and has been run against the simulator only, not on the board. Not built: flying a mission (the Map tab stores a waypoint list on the vehicle, reads it back and clears it; nothing starts it), joystick input,
  firmware upload, changing message rates from the ground station. (On-board log download is built, in the window and in `marv-cli`.)
- `marv-link-test` has been run on localhost and, once (2026-11-15, 30 s), over a real radio; the UDP link
  has not been run for longer than that against a real radio. The deframer releases a frame of an id the dialect does not have only when the next frame
  starts, so a count of such frames can be one frame late until the next frame arrives (`MARV_TEST` is
  known and is delivered at once); the tool does not flush the last one at the end of the run.
- Run against one board (its real firmware, and the demonstration program for `tools/board_check.sh`) on one
  Linux computer. Not run: Windows, macOS, a USB-serial adapter between the board and the computer, sessions longer than a minute.
- After the board enumerates, Linux hands its port over in terminal mode for about a millisecond of every
  first open; the board therefore keeps quiet until the port is really open (`docs/link-contract.md`,
  "The start gate"). A device that does not do this shows a few discarded bytes at the first connect; the
  ground station carries on.
- MAVLink 2 only. Signed frames are passed through without checking the signature. One vehicle at a time.
- Parameters: types of up to 32 bits; no automatic re-fetch after the vehicle was lost and came back
  (press Refresh).
- File paths can be typed. The one file dialog is the Log tab's "Browse…" (below); it needs the desktop's file-chooser portal and a D-Bus session, and without them says so and you type the path.
- Opening a TCP address that does not answer blocks the window for up to 2 s.
- The UDP link has been tested on localhost and briefly against a real radio backpack (2026-11-15). Its peer is
  whoever sent last (no authentication, as usual for MAVLink over UDP). A write while the network is
  unreachable (the WiFi dropped) is discarded and the link stays up; any other error ends the link.
  A USB transmitter module is expected to use the serial link at 460800 baud; that is untested.
- A real-time replay waits out the gaps of the recording, however long.
- Unbounded on a hostile or very noisy link: the number of distinct message kinds tracked, and data
  queued while a TCP peer stops reading.

## License

MIT.

The Log tab's file dialog uses the `rfd` crate (0.17.2, MIT) through the XDG desktop portal: no GTK is linked, and the portal client loads the host's `libdbus-1` at run time, so the AppImage still bundles no library.
