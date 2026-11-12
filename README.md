# MARV ground control station

A desktop ground station for the MARV flight controller, in Rust, speaking MAVLink 2. It is an MVP that
does three things: live sensor data, parameters, and logging. It is built so that the firmware can add
parameters, settings and messages without the ground station being reworked.

The flight-controller end of the link (a USB crate and a MAVLink device crate, each in its own repo) is
not built yet. Until it is, the ground station runs against the simulated vehicle in this repo, which
behaves as `docs/link-contract.md` says the real one will.

## Quick start

```
cargo run -p gcs-sim                                      # simulated vehicle on 127.0.0.1:5760
cargo run -p marv-gcs -- --connect tcp:127.0.0.1:5760      # the window, connected to it
```

With a board: plug in its USB port, choose `Serial` in the bar at the top, pick the port and press
Connect, or start with `--connect serial:/dev/ttyACM1`. The baud rate does not matter for a USB port.

```
marv-gcs [--connect <endpoint>] [--log-dir <dir>] [--no-log] [--view <name>]
         [--screenshot <file.png>] [--after <seconds>]
```

`<endpoint>` is `tcp:HOST:PORT`, `serial:PATH[:BAUD]`, or a recorded log to replay: `tlog:PATH` (at the
recorded pace) or `tlogfast:PATH`. `--view` picks the tab shown at start. `--screenshot` saves a picture
of the window after `--after` seconds and exits; it is how the pictures in a report are made.

Building needs Rust 1.95 or newer. On Linux the serial-port crate needs the libudev development package
(`libudev-dev` on Ubuntu).

## What it does

| Tab | What it shows |
|---|---|
| Sensors | Live plots of both IMUs, the high-g accelerometer, the magnetometer and the barometer, and one line of GPS. The plots are a table of presets; one row adds a plot. |
| Messages | Every message that arrives: rate, count, age and all its fields. Any numeric field can be plotted. Nothing here knows a particular message. |
| Parameters | Whatever parameters the vehicle reports: search, group by name prefix, edit, write with confirmation (a refused or unanswered write says so), save to and load from a file. Nothing here knows a particular parameter. |
| Log | The vehicle's text messages, and the recording. Every session is recorded automatically to `logs/<UTC time>.tlog`, the telemetry-log format Mission Planner, QGroundControl and pymavlink read. A log can be opened and replayed through the same tabs. |
| Link | Connection state and counters: frames received and sent, checksum errors, discarded bytes. |

The status at the top right says `Connecting`, `Connected` (with the vehicle's ids), `Lost` (no heartbeat
for 5 s) or `Closed` (with the reason).

## How it is built

Four crates; the dependency graph enforces the layers (`docs/architecture.md`).

| Crate | What it is |
|---|---|
| `gcs-transport` | Bytes only: USB serial port and TCP. It never names MAVLink. |
| `gcs-mavlink` | MAVLink 2 on any reader and writer: framing, one small state machine per sub-protocol (heartbeat, messages, parameters, text, recorder), the vehicle state, the log reader and replay. No transport, no window. |
| `gcs-sim` | The simulated vehicle (`marv-sim`). Written from the specification and never using `gcs-mavlink`, so the two check each other. |
| `marv-gcs` | The window: the connection bar and one file per tab. |

- `docs/link-contract.md`: what the flight controller and the ground station must do on the wire, and the
  design of the two crates on the flight-controller side.
- `docs/extending.md`: how to add a tab, a service, a transport, a preset plot. A new parameter or a new
  standard message needs no change here.

## Tests

```
cargo test --workspace          # about 370 tests, 30 s; needs /usr/bin/socat for the serial tests
cargo clippy --workspace --all-targets -- -D warnings
```

What the tests stand on:
- Each protocol state machine is tested with plain calls and a made-up clock: retries, timeouts and
  frame loss are exact, not timing dependent.
- End to end (`crates/marv-gcs/tests/e2e*.rs`): the real transport, MAVLink layer and session against the
  simulated vehicle over TCP and over a pseudo serial port: connect, loss of the vehicle and recovery,
  message rates and values, the full parameter list with 20 % of frames dropped in each direction, writes
  confirmed, refused and unanswered, both parameter encodings, recording and replay.
- The views are drawn and clicked without a window (`egui_kittest`).
- Checked with independent implementations: framing against the `mavlink` crate's own reader and writer
  for all 234 message types; the simulator's output with a hand-written decoder; a recorded log read with
  pymavlink (every frame decoded, same counts, both directions).

## Not covered, and known limits

- Not built: missions, a map, joystick input, firmware upload, download of logs stored on the vehicle,
  changing message rates from the ground station. None is needed by the firmware yet.
- Not yet run against a real board: only against the simulator and a pseudo serial port. The serial line
  settings and DTR are set but unproven on a device.
- MAVLink 2 only. Signed frames are passed through without checking the signature. One vehicle at a time.
- Parameters: types of up to 32 bits; no automatic re-fetch after the vehicle was lost and came back
  (press Refresh).
- File paths are typed into a text field; there is no file dialog.
- Opening a TCP address that does not answer blocks the window for up to 2 s.
- A real-time replay waits out the gaps of the recording, however long.
- Unbounded on a hostile or very noisy link: the number of distinct message kinds tracked, and data
  queued while a TCP peer stops reading.

## License

MIT.
