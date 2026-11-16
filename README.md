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
over its USB port and over the radio: it runs seven device rows (IMU1, IMU2, BARO, MAG, GNSS, BATT and the
radio receiver RC), the parameters, the commands, the calibrations and the self-test. The automated board
check (`tools/board_check.sh`) still uses the small demonstration program of `mavlink-device`. Without a board it runs against the simulated vehicle in this repo, which behaves as
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
         [--screenshot <file.png>] [--after <seconds>]
```

`<endpoint>` is `tcp:HOST:PORT`, `serial:PATH[:BAUD]`, `udp:ADDR:PORT`, or a recorded log to replay: `tlog:PATH` (at the
recorded pace) or `tlogfast:PATH`. `--view` picks the tab shown at start. `--screenshot` saves a picture
of the window after `--after` seconds and exits; it is how the pictures in a report are made.

Building needs Rust 1.99 or newer (the `rust-version` of `marv-dialect`). On Linux the serial-port crate needs the libudev development package
(`libudev-dev` on Ubuntu).

## What it does

| Tab | What it shows |
|---|---|
| Sensors | Live plots of both IMUs, the high-g accelerometer, the magnetometer and the barometer, and one line of GPS. The plots are a table of presets; one row adds a plot. |
| Messages | Every message that arrives: rate, count, age and all its fields. Any numeric field can be plotted. Nothing here knows a particular message. |
| Devices | The health of each device of the flight controller (`MARV_DEVICE_STATUS`, `docs/link-contract.md` section 8): one row per device in `device_id` order with its state (coloured), last error, samples and samples per second (computed from two successive rows), age of the last sample, errors, timeouts, recoveries, attempts, overflows, configuration restores, flags (HIL, self-test) and how long ago the row arrived (a row older than 15 s is shown as stale, not removed). The table is read only; above it the buttons Calibrate gyro, Calibrate accelerometer (level) and Cancel calibration, and per row Restart (IMU1, IMU2, BARO, MAG, GNSS, BATT) and Self-test (IMU1, IMU2, MAG), all Configure-only (section "Commands" below). The simulator sends seven devices; no firmware sends the message yet, so it has not been seen from a board. |
| Parameters | Whatever parameters the vehicle reports: search, group by name prefix, edit, write with confirmation (a refused or unanswered write says so), save to and load from a file; the storage buttons Save to flash, Reload from flash, Reset all to defaults and Reset calibration (the two resets ask for a second click). Nothing here knows a particular parameter. On an operate-only link the list is not downloaded at connect (press Refresh) and the editing controls are shown disabled with the reason. |
| Log | The vehicle's text messages, and the recording. Every session is recorded automatically to `logs/<UTC time>.tlog`, the telemetry-log format Mission Planner, QGroundControl and pymavlink read. A log can be opened and replayed through the same tabs. |
| Link | Connection state and counters: frames received and sent, checksum errors, discarded bytes; the session's role (Access) and the last command it refused; the Reboot vehicle button (asks for a second click). |

The status at the top right says `Connecting`, `Connected` (with the vehicle's ids), `Lost` (no heartbeat
for 5 s) or `Closed` (with the reason), then the role of the session.

## Commands

Every button above is a MAVLink command to the flight controller (`docs/link-contract.md` section 10): the
`commands` service in `gcs-mavlink` sends one `COMMAND_LONG` at a time (3 sends, 1.5 s apart, `confirmation`
0, 1, 2), follows `COMMAND_ACK` (IN_PROGRESS with a percentage, then the final result) and gives up after
20 s of silence. All are Configure-class: on an Operate link they are shown disabled with the reason, and
while a command is running every command button is disabled ("a command is running"), except Cancel
calibration during a calibration. The Devices tab has Calibrate gyro, Calibrate accelerometer (level) and
Calibrate compass; while the compass calibration waits or runs it shows "rotate the vehicle through all
orientations" next to the status line. The last command, in words ("Gyro calibration: 40 %", "Compass
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
| `devices [--for s]` | collects for `--for` (default 2 s), then one line per `MARV_DEVICE_STATUS` device: `NAME<TAB>STATE<TAB>LAST_ERROR<TAB>SAMPLES<TAB>ERRORS<TAB>TIMEOUTS<TAB>RECOVERIES<TAB>LAST_SAMPLE_AGE_MS` (`never` when the device has had no sample) |
| `texts [--for s]` | STATUSTEXTs as they arrive, until the time is over (default 10 s): `TIME<TAB>SEVERITY<TAB>TEXT`, TIME in session seconds |
| `watch [--for s]` | until the time is over (default 30 s): `device<TAB>` + a device line when a device appears or its state, last error, errors or timeouts change (samples and the age change all the time and do not count), and `text<TAB>` + a text line |
| `param list` | `NAME<TAB>VALUE<TAB>TYPE`, one per parameter, by name; waits for the whole list (on a `udp:` link it asks for it: a read) |
| `param get NAME` | the same line for one parameter; exit 1 for an unknown name |
| `param set NAME VALUE` | waits for the vehicle's echo and prints the same line with the value now in effect; if the echo differs from what was asked (the vehicle refused, which ends the write at once, with one `PARAM_SET` on the link) the line is printed, the message and the vehicle's own `text` line go to stderr and the exit status is 1; a value that does not fit the parameter's type is not sent |
| `param save` / `param reload` | `MAV_CMD_PREFLIGHT_STORAGE` with param1 = 1 / 0 |
| `param reset-all --yes` / `param reset-calibration --yes` | param1 = 2 / 3; without `--yes` they refuse (exit 2) before connecting |
| `calibrate gyro` / `accel` / `compass` / `cancel` | `MAV_CMD_PREFLIGHT_CALIBRATION` (gyro param1 = 1; accelerometer param5 = 4, one position, level; compass param2 = 1, 30 s of rotating the vehicle through all orientations, the CLI prints that as its first line; cancel all zeros) |
| `device restart NAME` / `device selftest NAME` | `MAV_CMD_MARV_DEVICE`; NAME is IMU1, IMU2, BARO, MAG, GNSS, BATT, RC, SD, PSRAM, LED or ESC (any case); every name is sent and the vehicle decides (it denies a restart of RC; the window's buttons offer only the ones it accepts) |
| `reboot --yes` | `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` param1 = 1; without `--yes` it refuses (exit 2) before connecting |
| `logs list` | the logs on the vehicle's card, one line each, `NUMBER<TAB>SIZE` (bytes), then `last<TAB>N`: the number of the log being written, or the highest present when none is open; numbers have gaps; an empty card prints nothing on stdout and `marv-cli: no logs on the vehicle` on stderr, exit 0. Configure links only (`serial:`, `tcp:`) |
| `logs get N [-o FILE] [--force]` | lists, then downloads log N to FILE (default `NNNNNNNN.BIN`, the card's name, in the current directory); prints `downloaded<TAB>FILE<TAB>SIZE bytes`. Progress on stderr, at most twice a second: `log N: DONE of SIZE bytes (P %)`. An existing file is refused (exit 1) unless `--force`; a log that is not in the list is exit 1 and creates no file; a download that ends without all its bytes is exit 1 and keeps the partial file, saying how many bytes are in it. Configure links only. See "Logs" below |
| `record FILE [--for s]` | records the link to a `.tlog` (the session's own recording) for `--for` (default 60 s), then prints `recorded<TAB>FILE<TAB>N frames<TAB>N bytes`; an existing file is never overwritten (exit 1) |

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
| 2 | usage error (unknown command, a missing argument, a bad number, `--yes` missing); nothing is opened |
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
  on a `udp:` link both commands are refused with exit 4, nothing sent. The window (`marv-gcs`) has no logs tab yet.
- Not verified against a board: the firmware side does not exist yet. It has been run against the SIMULATOR
  only (`gcs-sim`, which follows the contract, including lost frames).

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

On an Operate session `Session::send` refuses every configure-class command (parameter writes, and later
sending a MAVLink command, storage, calibration, device actions, reboot, log download): nothing is written, and the Link
view shows the refusal. The parameter list is not downloaded at connect (Refresh asks for it), and the
configure controls are shown disabled with the reason, not hidden. A session whose access was not set is Operate: the gate fails closed. A new `Command` variant is classified
in `Command::access` (`crates/gcs-mavlink/src/state.rs`); the code does not compile until it is.

## How it is built

Six crates; the dependency graph enforces the layers (`docs/architecture.md`).

| Crate | What it is |
|---|---|
| `gcs-transport` | Bytes only: USB serial port, TCP and UDP. It never names MAVLink. |
| `gcs-mavlink` | MAVLink 2 on any reader and writer: framing, one small state machine per sub-protocol (heartbeat, messages, parameters, commands, text, recorder, logs), the vehicle state, the log reader and replay. No transport, no window. |
| `gcs-sim` | The simulated vehicle (`marv-sim`). Written from the specification and never using `gcs-mavlink`, so the two check each other. |
| `gcs-link` | The join, with no GUI: the role an endpoint gives the station (`access_of`, the one mapping) and the start of a session on it. Used by both applications. |
| `marv-cli` | The command line (`marv-cli`), for a computer with no display: see below. |
| `marv-gcs` | The window: the connection bar and one file per tab; also the link test tool `marv-link-test` (`src/link_test.rs`, `src/bin/marv_link_test.rs`), which builds `MARV_TEST` with `marv-dialect`. |

- `docs/link-contract.md`: what the flight controller and the ground station must do on the wire (the
  device diagnostics message and the Devices view are its section 8), and the design of the two crates on
  the flight-controller side.
- `docs/extending.md`: how to add a tab, a service, a transport, a preset plot. A new parameter or a new
  standard message needs no change here.

## Tests

```
cargo test --workspace          # 561 tests, 55 s; needs /usr/bin/socat for the serial tests
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
- The command line (`crates/marv-cli/tests`, 30 tests): every command and option and every error of the arguments; every command against the simulator over TCP; no vehicle (nothing listening, a silent peer); the negative control, where a counting writer on an Operate session sees no `PARAM_SET` and no `COMMAND_LONG` but the capability request while `devices`, `status` and `param list` work; and the dependency rule (no GUI crate in `cargo tree -p marv-cli`).
- The views are drawn and clicked without a window (`egui_kittest`).
- Checked with independent implementations: framing against the `mavlink` crate's own reader and writer
  for all 234 message types; the simulator's output with a hand-written decoder; a recorded log read with
  pymavlink (every frame decoded, same counts, both directions).
- On the MARV V2 board over USB (`tools/board_check.sh`, 2026-11-12, one Linux computer): connected within
  20 ms of the open; first byte of the first session after a restart is the start of frame 0, and the
  first 20 s carry 4343 frames with 0 checksum errors, 0 discarded bytes and 0 lost frames; rates 50.000 to
  50.002 Hz per IMU instance, 10.000 Hz pressure, 5.000 Hz GPS, 1.000 Hz heartbeat; 13 parameters listed
  in 20 ms; writes confirmed in 20 ms, a read-only one refused, an out-of-range one answered with the
  clamped value; a recording of 1100 frames decoded completely by pymavlink; reconnect in 20 ms.

## Not covered, and known limits

- Not built: missions, a map, joystick input, firmware upload, a logs tab in the window (the command line downloads the logs: see "Logs"),
  changing message rates from the ground station. None is needed by the firmware yet.
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
- File paths are typed into a text field; there is no file dialog.
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
