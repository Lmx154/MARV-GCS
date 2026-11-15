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
ground station has been run against the MARV V2 board over its real USB port with a
demonstration firmware (values computed on the board, parameters in RAM); the real sensors are not wired
to it yet. Without a board it runs against the simulated vehicle in this repo, which behaves as
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
calibration during a calibration. The last command, in words ("Gyro calibration: 40 %", "Reboot: no answer
after 3 sends"), is shown under the buttons of the Parameters, Devices and Link tabs and is
`VehicleState.command`. The simulator answers them as the vehicle will; no firmware does yet.

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
sending a MAVLink command, storage, calibration, device actions, reboot): nothing is written, and the Link
view shows the refusal. The parameter list is not downloaded at connect (Refresh asks for it), and the
configure controls are shown disabled with the reason, not hidden. A session whose access was not set is Operate: the gate fails closed. A new `Command` variant is classified
in `Command::access` (`crates/gcs-mavlink/src/state.rs`); the code does not compile until it is.

## How it is built

Four crates; the dependency graph enforces the layers (`docs/architecture.md`).

| Crate | What it is |
|---|---|
| `gcs-transport` | Bytes only: USB serial port, TCP and UDP. It never names MAVLink. |
| `gcs-mavlink` | MAVLink 2 on any reader and writer: framing, one small state machine per sub-protocol (heartbeat, messages, parameters, commands, text, recorder), the vehicle state, the log reader and replay. No transport, no window. |
| `gcs-sim` | The simulated vehicle (`marv-sim`). Written from the specification and never using `gcs-mavlink`, so the two check each other. |
| `marv-gcs` | The window: the connection bar and one file per tab; also the link test tool `marv-link-test` (`src/link_test.rs`, `src/bin/marv_link_test.rs`), which builds `MARV_TEST` with `marv-dialect`. |

- `docs/link-contract.md`: what the flight controller and the ground station must do on the wire (the
  device diagnostics message and the Devices view are its section 8), and the design of the two crates on
  the flight-controller side.
- `docs/extending.md`: how to add a tab, a service, a transport, a preset plot. A new parameter or a new
  standard message needs no change here.

## Tests

```
cargo test --workspace          # 462 tests, 25 s; needs /usr/bin/socat for the serial tests
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

- Not built: missions, a map, joystick input, firmware upload, download of logs stored on the vehicle,
  changing message rates from the ground station. None is needed by the firmware yet.
- `marv-link-test` has been tested on localhost only; neither it nor the UDP link has been run against
  a real radio. The deframer releases a frame of an id the dialect does not have only when the next frame
  starts, so a count of such frames can be one frame late until the next frame arrives (`MARV_TEST` is
  known and is delivered at once); the tool does not flush the last one at the end of the run.
- Run against one board with a demonstration firmware on one Linux computer. Not run: Windows, macOS, a
  USB-serial adapter or radio in between, sessions longer than a minute, real sensor data.
- After the board enumerates, Linux hands its port over in terminal mode for about a millisecond of every
  first open; the board therefore keeps quiet until the port is really open (`docs/link-contract.md`,
  "The start gate"). A device that does not do this shows a few discarded bytes at the first connect; the
  ground station carries on.
- MAVLink 2 only. Signed frames are passed through without checking the signature. One vehicle at a time.
- Parameters: types of up to 32 bits; no automatic re-fetch after the vehicle was lost and came back
  (press Refresh).
- File paths are typed into a text field; there is no file dialog.
- Opening a TCP address that does not answer blocks the window for up to 2 s.
- The UDP link has been tested on localhost only, not against a real radio or backpack. Its peer is
  whoever sent last (no authentication, as usual for MAVLink over UDP). A write while the network is
  unreachable (the WiFi dropped) is discarded and the link stays up; any other error ends the link.
  A USB transmitter module is expected to use the serial link at 460800 baud; that is untested.
- A real-time replay waits out the gaps of the recording, however long.
- Unbounded on a hostile or very noisy link: the number of distinct message kinds tracked, and data
  queued while a TCP peer stops reading.

## License

MIT.
