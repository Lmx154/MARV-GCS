# marv-cli

Part of [MARV-GCS](../README.md)

The station without a window, for a companion computer with no display (a Raspberry Pi cabled to the flight controller's
USB port) or a script. One command per invocation, on one link; plain text on stdout, errors on stderr. It depends on no
GUI crate (`cargo tree -p marv-cli -e normal | grep -ciE 'eframe|egui|winit|wgpu|glow'` is 0). It presents itself as a
ground station (system 255, component 190), speaks through the same session as the window, and sends every write
through `Session::send`, so the role of the link applies: on a `udp:` link every command that writes prints
`marv-cli: refused: this link is operate-only (...)` and exits 4 having written nothing. `serial:` and `tcp:` links are
Configure ([link-access](link-access.md)).

```
marv-cli --connect <endpoint> [--timeout <s>] <command> [args]     # endpoint: serial:PATH[:BAUD], tcp:HOST:PORT, udp:ADDR:PORT
marv-cli --connect serial:/dev/ttyACM0 status
marv-cli --connect serial:/dev/ttyACM0 param set LOG_RATE_HZ 200
marv-cli --connect serial:/dev/ttyACM0 calibrate gyro
```

Every command first waits for the vehicle's heartbeat (`--timeout`, default 5 s) and otherwise fails with exit 3 and
`no heartbeat from a vehicle within N s`. Options may stand anywhere. Every command also prints each `STATUSTEXT` the
vehicle sends while it runs on stderr as `text<TAB>SEVERITY<TAB>TEXT` (a text belongs to the session, so one that arrived
before the command's request is shown too); `texts`, `watch` and `status` print texts on stdout with no copy on stderr.

## Commands with a vehicle

| Command | What it does and prints |
|---|---|
| `status [--for s]` | Collects `--for` (default 2 s), then `vehicle<TAB>system=1 component=1 type=...`, `role<TAB>Operate` or `Configure`, `health<TAB>` the sensors `SYS_STATUS` reports healthy (`none`, or `not received`), `devices<TAB>N`, and the last 3 texts as `text<TAB>TIME<TAB>SEVERITY<TAB>TEXT` |
| `devices [--for s]` | One line per `MARV_DEVICE_STATUS` device: `NAME<TAB>STATE<TAB>LAST_ERROR<TAB>SAMPLES<TAB>ERRORS<TAB>TIMEOUTS<TAB>RECOVERIES<TAB>LAST_SAMPLE_AGE_MS<TAB>FLAGS` (`never` for no sample; FLAGS `-`, `HIL`, `self-test`, `bit N`, joined by `, `) |
| `texts [--for s]` | `STATUSTEXT`s as they arrive until the time is over (default 10 s): `TIME<TAB>SEVERITY<TAB>TEXT`, TIME in session seconds |
| `watch [--for s]` | Default 30 s: `device<TAB>` + a device line when a device appears or its state, last error, errors or timeouts change, and `text<TAB>` + a text line |
| `param list` | `NAME<TAB>VALUE<TAB>TYPE` per parameter, by name; waits for the whole list (on `udp:` it asks for it: a read) |
| `param get NAME` | The same line for one parameter; exit 1 for an unknown name |
| `param set NAME VALUE` | Waits for the echo and prints the line with the value now in effect. If the echo differs (the vehicle refused) the line is printed, the message and the vehicle's `text` go to stderr and the exit is 1; a value that does not fit the type is not sent. A refusal is reported after 300 ms of waiting for a late echo |
| `param set NAME VALUE --debug` | The station's gate for raw sensor settings. Without `--debug`, `param set` of a field a preset sets (`IMUn_ACC_FS`, `IMUn_GYR_FS`, `IMUn_ODR_HZ`, `BARO_ODR_HZ`, `BARO_OSR_P`, `BARO_OSR_T`, `BARO_IIR_P`, `BARO_IIR_T`, `MAG_ODR_HZ`, `MAG_AVG`) refuses with a line naming `--debug` (exit 2) before connecting |
| `param save` / `param reload` | `MAV_CMD_PREFLIGHT_STORAGE` param1 = 1 / 0 |
| `param reset-all --yes` / `param reset-calibration --yes` | param1 = 2 / 3; without `--yes` they refuse (exit 2) before connecting |
| `calibrate gyro` / `accel` / `compass` / `cancel` | `MAV_CMD_PREFLIGHT_CALIBRATION`: gyro param1 = 1; accelerometer param5 = 4 (level); compass param2 = 1, the quick best-effort method (the CLI prints "rotate the vehicle through all orientations for 30 s" first; the guided calibration is the Devices tab only); cancel all zeros |
| `device restart NAME` / `device selftest NAME` | `MAV_CMD_MARV_DEVICE`; NAME is IMU1, IMU2, BARO, MAG, GNSS, BATT, RC, SD, PSRAM, LED or ESC (any case); every name is sent and the vehicle decides |
| `reboot --yes` | `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` param1 = 1; without `--yes` it refuses (exit 2) before connecting |
| `record FILE [--for s]` | Records the link to a `.tlog` for `--for` (default 60 s), then `recorded<TAB>FILE<TAB>N frames<TAB>N bytes`; an existing file is never overwritten (exit 1) |
| `logs list` | `NUMBER<TAB>SIZE` per log on the card, then `last<TAB>N`; an empty card prints `marv-cli: no logs on the vehicle` on stderr, exit 0. Configure links only |
| `logs get N [-o FILE] [--force]` | Lists, then downloads log N (default `NNNNNNNN.BIN` in the current directory) and prints `downloaded<TAB>FILE<TAB>SIZE bytes`. See [Log download](#log-download) |
| `mission upload FILE.waypoints [--return-height m]` | Sends the waypoints of a QGC WPL 110 file (row 0, the home placeholder, is skipped) and the return item (the file's last row, command 20, its altitude the return height; `--return-height` supplies it for a file without one, otherwise the command refuses). Reads the list back and prints `verified<TAB>N waypoints<TAB>return height H m<TAB>plan id 0xHHHHHHHH` (exit 0) only when the vehicle accepted it and the read-back equals what was sent; otherwise exit 1 with one stderr line (the vehicle's refusal as `refused by the vehicle: MAV_MISSION_NO_SPACE (4): ...`, `no answer from the vehicle`, a count, waypoint or plan id that differs; exit 3 when the link went down). A row off the globe or with a non-finite altitude is exit 1 naming its line, nothing sent. Works on every link; slow on a lossy link (every lost frame costs 1.5 s) |
| `mission download [FILE.waypoints] [--force]` | `SEQ<TAB>LATITUDE<TAB>LONGITUDE<TAB>ALTITUDE` per waypoint (degrees with 7 decimals, metres above home), then `return height<TAB>H m` and `N waypoints<TAB>plan id 0x...`. With a file name it also writes the list as a `.waypoints` file whose last row is the return height and prints `written<TAB>FILE`; an existing file is refused (exit 1) without `--force` |
| `mission clear` | Empties the vehicle's list and reads it back: `verified<TAB>0 waypoints<TAB>plan id 0x00000000`, exit 0 only when the read-back is empty; a refusal is exit 1 |
| `mission start` / `pause` / `abort` / `land` / `return` | A flight command over the radio link ([link-flight](link-flight.md)): prints `<Name>: accepted`, or the vehicle's reason text; exit 1 when refused or unanswered. Abort drops the mission and holds position. No command arms, disarms or stops the motors |

Output is one record per line; fields are separated by a TAB, so `cut -f`, `awk -F'\t'` and `grep` work. A command that
sends a MAVLink command follows its record to the end and prints a line each time it changes (`gyro calibration: waiting
for the vehicle`, `gyro calibration: 40 %`, then `gyro calibration: accepted`, `restart RC: denied`, `... no answer after 3
sends`). One command at a time, as in the window. Ctrl-C ends the process at once; a recording keeps everything written,
and a calibration the vehicle began goes on, so `calibrate cancel` from the next invocation stops it (the vehicle serves
one station at a time on its USB port). There is no JSON mode for these commands.

## Commands on files

No `--connect`, no vehicle.

| Command | What it does |
|---|---|
| `session export FILE.tlog [--name N] [--out DIR]` | Converts a recording into a session folder ([sessions-and-logs](sessions-and-logs.md)). `--name` defaults to the file's name without `.tlog`; `--out` to `sessions/` next to the file. Prints `session<TAB>FOLDER`, then `FILE.csv<TAB>N rows` per CSV, `skipped<TAB>MESSAGE<TAB>WHY` and `marks<TAB>N`. An existing folder is never overwritten (exit 1); a file with no frames is exit 1 |
| `session list [DIR]` | The sessions in DIR (default `logs/sessions`): `FOLDER<TAB>NAME<TAB>START<TAB>D s<TAB>N marks`; none is exit 0 with a stderr line |
| `log report FILE.BIN --mass-kg KG --kf K [--poles N] [--accel-lpf-hz HZ]... [--level-range MIN MAX] [--json FILE]` | The flight report ([flight-report](flight-report.md)). `--mass-kg` and `--kf` (N/(rad/s)², one motor) are required; `--poles` defaults to the log's `MOT_POLES`; `--json` writes the same content to FILE (never overwrites, exit 1). A bad argument is exit 2; an unreadable file exit 1 |
| `log tune FILE.BIN [--json] [--from S] [--to S] [--CONSTANT V]...` | Post-flight tuning analysis of a log with the firmware's `PIDT` and `NTCH` records, per armed span that was airborne: context (modes, gains from `PARM`, notch, log health), stick-driven events in the rate target, delay, rise, overshoot and settling of the achieved rate against the target, frequency response where coherent, P, I, D terms and D torque above 50 Hz, gyro noise against the rotor frequencies, and a list of measured facts. It proposes no gain. Every threshold and window is a named constant printed in the header and overridable by a flag of its name. A log without the records gets its context and lines saying what needs newer firmware. `--json` prints JSON on stdout. No position is read |
| `log survey FILE.BIN [--name LIST] [--alt M] [--channel N] [--force]` | Survey mode: each press of RC channel 9 (`--channel`, 1 to 14; `RCIN` going from below to above the midpoint of its own range and staying there at least 2 samples; a range under 400 raw units is "never moved", no file) becomes one waypoint at the median position of the receiver's 3D-fix `GPS` records within the press, widened 0.3 s each side (a press with none is skipped and named). Saved as `<LIST>.waypoints` in the window's store (default name `survey-<date of the log>-<file name>`; an existing list is kept without `--force`), every point `--alt` metres (default 5) above take-off, at most 16 points. The output lists time, hold time, fix count, satellites, HDop and distance from the previous point, never a coordinate |

## Exit status

| Code | Meaning |
|---|---|
| 0 | Success (a command: the vehicle's result is accepted) |
| 1 | The command failed: not accepted (denied, failed, cancelled, no answer), a parameter echo that differs, an unknown parameter, a recording that cannot start; for `logs` and `mission`: the list did not arrive, the log is not listed, the file exists or cannot be written, the download is incomplete, the read-back differs |
| 2 | Usage error (unknown command, missing argument, bad number, `--yes` or `--debug` missing); nothing is opened |
| 3 | No link, no heartbeat within `--timeout`, or the link went down |
| 4 | Refused: the link is operate-only and the command writes |

## Log download

`logs list` and `logs get` download the `.BIN` logs the firmware writes to its SD card over the same USB cable, with
ArduPilot's MAVLink log download ([link-logs](link-logs.md)). The file is the card's file byte for byte and opens in
MAVExplorer, UAV Log Viewer and Mission Planner. Log numbers are the card's, with gaps; the log being written can be
downloaded too, up to what the vehicle has flushed (at most about 1 s behind), and logging goes on.

| Fact | Detail |
|---|---|
| Speed | About 70 kB/s at best: 4 frames of 90 bytes per 5 ms. A 10 MB log takes about 2.5 minutes, 161 MB about 37 minutes. Progress goes to stderr at most twice a second: `log N: DONE of SIZE bytes (P %)` |
| Requests | 512 frames at a time; asks again from the first missing byte when a second passes without news; after 5 fruitless tries it stops with `no data from the vehicle at byte N after 5 new requests` (exit 1) and keeps what it has |
| Existing file | Refused (exit 1) unless `--force`, which truncates it when the download starts. A log not in the list is exit 1 and creates no file. An incomplete download is exit 1 and keeps the partial file, saying how many bytes are in it |
| Links | USB (`serial:`) and TCP (the simulator) only: on `udp:` both commands are refused with exit 4, nothing sent |

## Raspberry Pi

Build with `cargo build --release -p marv-cli` on the Pi, or cross-compile with `--target aarch64-unknown-linux-gnu`
(64-bit Raspberry Pi OS; the 32-bit target is `armv7-unknown-linux-gnueabihf`). The user needs the `dialout` group for
`/dev/ttyACM*`. Against the simulator:

```
cargo run -p gcs-sim &                                          # on 127.0.0.1:5760
cargo run -p marv-cli -- --connect tcp:127.0.0.1:5760 devices
```

## Tests

`crates/marv-cli/tests`: every command and option and every argument error; `session export` and `session list`;
`log report` on a synthetic log; every command against the simulator over TCP, the log download included (byte for byte,
with lost frames, a stalled vehicle, an existing file); no vehicle (nothing listening, a silent peer); the negative control
in `tests/gate.rs` (a counting writer on an Operate session sees no `PARAM_SET`, no log message and no `COMMAND_LONG` but
the capability request, while `devices`, `status` and `param list` work); and the dependency rule.

## Related

[ui](ui.md) · [sessions-and-logs](sessions-and-logs.md) · [flight-report](flight-report.md) · [extending](extending.md) ·
[crate folder](../crates/marv-cli/README.md)
