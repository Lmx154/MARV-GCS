# Extending the ground station

The ground station is built so that the firmware can grow without the ground station being reworked.
Most additions need no change here at all; the rest are one file plus one or two registry lines.
`docs/architecture.md` has the layer rules these recipes rely on.

## What needs no change here

| The firmware adds | What happens |
|---|---|
| A parameter or setting | It shows up in the Parameters tab: the list is whatever the vehicle reports. Grouping comes from the name's prefix before the first `_`. No name, unit or range is stored in the ground station. |
| A message from the standard `common` set | It shows up in the Messages tab with its rate and every field, and any numeric field can be plotted. It is also recorded in the telemetry log. |
| A text message (STATUSTEXT) | It shows up in the Log tab. |

## One-line additions

| To add | Where |
|---|---|
| A preset plot on the Sensors tab | One row in `PRESETS` in `crates/marv-gcs/src/views/sensors.rs`: title, message name, instance, field names, a `scale` (the unit conversion, 1.0 for none) and the raw values that mean "not available". The message and fields must be ones the real firmware sends. |
| A message that carries several instances (one per sensor) | One row in `INSTANCE_FIELDS` in `crates/gcs-mavlink/src/services/messages.rs`: message name and the field that says which instance. The row gives the instance its `[n]` label (`HIGHRES_IMU[2]`) and, for a number-valued field, the instance itself. If the field is an enum (the store shows it as text), the number cannot be read from the row: add also one arm to `enum_instance` in the same file, which returns it; such a message needs both. Its instances then get separate rows and separate plot series. |

## A new tab (view)

1. Create `crates/marv-gcs/src/views/<name>.rs` with a type that implements `View`:
   `name()` is the tab title, `ui()` draws from `cx.state` (read-only, locked for the frame) and pushes
   `Command`s into `cx.commands` for anything the user asks for. `cx.now` is the session time, or `None`
   when nothing is connected. `cx.access` is the session's role: a control that sends a configure-class
   `Command` is shown disabled with `status::CONFIGURE_BLOCKED` as the reason when `cx.access` is
   `Operate` (use `status::configure_blocked`), never hidden.
2. In `crates/marv-gcs/src/views/mod.rs` add `mod <name>;` and one `Box::new(<name>::<Type>),` line in `all()`.

Nothing else changes: the tab row, the command line option `--view` and the tests that draw every view
all come from that registry. Test a view without a window with `egui_kittest` (see `tests/views_*.rs`);
give every input widget an accessible name.

## A new protocol service (missions, rate control, ...)

1. Create `crates/gcs-mavlink/src/services/<name>.rs` with a type that implements `Service`. It is a
   state machine: `on_frame` for what arrives, `on_tick` for timers (at least every 50 ms), `on_command`
   for what the user asked, `on_sent` if it needs to see what went out. It sends by pushing messages into
   `cx.out`, reads the time from `cx.now`, and sets `cx.changed = true` when it changed the state. No
   socket, thread, clock or file inside it.
2. If it has something to show, add a section for it to `VehicleState` in `crates/gcs-mavlink/src/state.rs`.
3. If the user can ask it for something, add a variant to `Command` in the same file and classify it in
   `Command::access` (and `Command::name`) there: `Configure` for anything that writes to the vehicle (a
   parameter write, a MAVLink command, storage, calibration, a device action, reboot) or that the vehicle
   offers on a cable only (log download), `Operate` for what only reads or only concerns the station. Both
   are matches with no wildcard arm, so the code does not compile until the variant has its class; add a row for it to the table test in
   `crates/gcs-mavlink/src/session_tests.rs` (`every_command_has_its_class_in_the_table`). `Session::send`
   refuses a configure-class command on an Operate session; a service does not check it.
4. In `crates/gcs-mavlink/src/services/mod.rs` add `pub mod <name>;` and one line in `default_services`.

Test it with plain calls and a made-up `now` (see `services/params_tests.rs`), then end to end against the
simulator (`crates/marv-gcs/tests/e2e*.rs`, or through the command line, `crates/marv-cli/tests/logs.rs`; every loop that waits for a state or drains an API is bounded). When the firmware side of the feature is specified, write it
into `docs/link-contract.md` first and teach the simulator (`crates/gcs-sim`) to behave that way, so the
ground station is developed against the real behaviour.

The log service (`services/logs.rs`, contract in `docs/link-contract.md` section 11) is the worked example of a
service that moves a lot of data: it never holds it. `Command::DownloadLog` carries a writer the caller opened
(`LogSink`, the one the recorder takes); the service hands the bytes to it in order as they arrive and keeps
in `VehicleState.logs` only the progress (`done` of `size`), the status and a counter (`downloads`) that a caller
compares before and after sending, so that it never reads the result of an earlier request. Retries are
counted from `cx.now`, so its tests run in made-up time (`services/logs_tests.rs`).

The command service (`services/commands.rs`) is the worked example of a service that follows a request to
its result: `Command::SendCommand { command, params }` starts it, `VehicleState.command` shows it. A new
command button needs no change in the service: the view pushes `Command::SendCommand` (gate it with
`status::command_gate`, draw it with `views/actions.rs::button` or `Confirm::button` when it is hard to undo),
`status::command_label` gives it its words in the status line, and the simulator's `actions::admit` its
answer. The vehicle's side is `docs/link-contract.md` section 10.

## A new transport (a telemetry radio; UDP is the worked example)

1. Add a module to `crates/gcs-transport/src/` that opens the link and returns a reader and a writer that
   follow the reader contract in `docs/architecture.md` (return as soon as a byte is there; `TimedOut`
   after 100 ms without data; `Ok(0)` or another error when the link is down). `udp.rs` is the example
   for a link that is not a stream: it keeps the part of a datagram that did not fit the caller's buffer,
   skips empty datagrams instead of returning `Ok(0)`, and says in its module doc which error kinds are
   "nothing yet" and which end the link.
2. Add a variant to `Endpoint` (with its text form, `udp:ADDR:PORT`) and an arm in `open`. Give the link its
   role in `access_of` in `crates/gcs-link/src/lib.rs` (Operate or Configure, `docs/link-contract.md`
   section 9): it is a match over `Endpoint` with no wildcard arm, so the application does not compile
   until you do. The mapping does not live in `gcs-transport`, and it is the one mapping for the window and for
   `marv-cli` (it needs no change there).
3. Add the choice to the connection bar in `crates/marv-gcs/src/connection.rs` (UDP: one text field, the
   local address to listen on) and its parse test in `cli.rs`.

Test it against a plain peer socket (`crates/gcs-transport/tests/udp.rs`), then through a session
(`crates/marv-gcs/tests/e2e_udp.rs`). The MAVLink layer does not change: it takes any reader and writer.
This crate never names MAVLink.

## A command for the command line (`marv-cli`)

1. If it is a new MAVLink command or parameter action, first do the service and the `Command` variant as above
   (with its access class); the command line never decides access, `Session::send` does.
2. Add the word to `command` in `crates/marv-cli/src/args.rs` (and to `mav_command` when it sends a
   `COMMAND_LONG`: the label of its progress lines, the `MAV_CMD`, the seven parameters), and the usage text.
3. A command that is not a `COMMAND_LONG` gets a function (the logs ones, `logs_list` and `logs_get`, are the example
   of one that waits for a service's result and writes a file) in `crates/marv-cli/src/commands.rs` and an arm in
   `run_command`. Every wait in it goes through `wait`, which has a deadline. Its output is tab-separated
   lines, errors on stderr, and an exit status from `exit` in `lib.rs`.
4. Test the parsing in `tests/args.rs` and the behaviour against the simulator in `tests/sim.rs` (or a file of its own, `tests/logs.rs`); if it writes,
   add it to the list in `tests/gate.rs` (the negative control: an Operate session must refuse it with 0 frames).

## A message outside the standard set

The project's own messages live in `marv-dialect` (own repo, https://github.com/Lmx154/marv-dialect): its
module `marv_dialect::marv` is `common` plus the messages of `definitions/marv.xml`, so far `MARV_TEST`
(id 42420, CRC_EXTRA 100, fields `seq`, `time_ms`, `value`, `tag`; a link test message) and
`MARV_DEVICE_STATUS` (id 42421, CRC_EXTRA 231; one message per device; `docs/link-contract.md` section 8).
`MARV_DEVICE_STATUS` is the one message the station names: its `device_id` is an enum, so it has a row in
`INSTANCE_FIELDS` (the `[n]` label) and an arm in `enum_instance` (the instance number, one row per device) in
`services/messages.rs`, and the Devices view (`views/devices.rs`) reads its fields.

The station speaks that dialect: `crates/gcs-mavlink/src/lib.rs` has `pub use marv_dialect::marv as
dialect;`, and `marv-dialect` is a git dependency at one rev, with `features = ["std", "serde"]`, in the
workspace `Cargo.toml` (`serde` is what the Messages tab needs: `services/messages.rs` reads the fields of
any message through `serde_json::to_value(message)`). So a message of the dialect is known to the
deframer (its checksum is checked with its CRC_EXTRA; a wrong one is counted as a checksum failure), is
decoded, shows in the Messages tab by name with all its fields, can be plotted and is recorded in the
telemetry log, with no code in the station that names it. `marv-link-test` is the one place that builds
a `MARV_TEST` (to send it). `gcs-sim` stays on `mavlink::dialects::common` (wire-compatible for the
common messages); it builds `MARV_DEVICE_STATUS` by hand from its definition (`crates/gcs-sim/src/devices.rs`). `marv-dialect` asks for Rust 1.99 or newer, which is the station's minimum too.

To add the next message:

1. Add it to `definitions/marv.xml` in `marv-dialect` (see its README), with the lock-step test, and commit
   it there.
2. Here, change the one `rev` of `marv-dialect` in the workspace `Cargo.toml` (one rev in the whole
   workspace: `cargo tree -i marv-dialect` lists one) and run the tests. Nothing else changes for it to
   appear in the Messages tab.
3. Add a test in the pattern of the `MARV_TEST` ones: a frame with the right checksum is known
   (`frame_tests.rs`), one with a wrong checksum is a checksum failure, and the Messages service shows its
   fields (`services/messages_tests.rs`).

## What to keep true

- `cargo tree -p gcs-transport | grep -cE 'mavlink|marv-dialect'` is 0.
- `cargo tree -p gcs-mavlink | grep -cE 'serialport|egui|eframe|gcs-transport'` is 0.
- `cargo tree -p gcs-sim | grep -c gcs-mavlink` is 0 (the simulator is written from the specification, so
  the two check each other).
- `cargo tree -p gcs-link -e normal | grep -ciE 'eframe|egui|winit|wgpu|glow'` and the same for `marv-cli` are 0.
- No parameter name and no message-specific code outside a view that is about that message (the one
  exception is `enum_instance` in the message store, for a message whose instance field is an enum: it
  supplies the instance number, and the message also needs its row in `INSTANCE_FIELDS`, which gives the label).
