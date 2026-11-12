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
| A preset plot on the Sensors tab | One row in `PRESETS` in `crates/marv-gcs/src/views/sensors.rs`: title, message name, instance, field names. |
| A message that carries several instances (one per sensor) | One row in `INSTANCE_FIELDS` in `crates/gcs-mavlink/src/services/messages.rs`: message name and the field that says which instance. Its instances then get separate rows and separate plot series. |

## A new tab (view)

1. Create `crates/marv-gcs/src/views/<name>.rs` with a type that implements `View`:
   `name()` is the tab title, `ui()` draws from `cx.state` (read-only, locked for the frame) and pushes
   `Command`s into `cx.commands` for anything the user asks for. `cx.now` is the session time, or `None`
   when nothing is connected.
2. In `crates/marv-gcs/src/views/mod.rs` add `mod <name>;` and one `Box::new(<name>::<Type>),` line in `all()`.

Nothing else changes: the tab row, the command line option `--view` and the tests that draw every view
all come from that registry. Test a view without a window with `egui_kittest` (see `tests/views_*.rs`);
give every input widget an accessible name.

## A new protocol service (missions, log download, rate control, ...)

1. Create `crates/gcs-mavlink/src/services/<name>.rs` with a type that implements `Service`. It is a
   state machine: `on_frame` for what arrives, `on_tick` for timers (at least every 50 ms), `on_command`
   for what the user asked, `on_sent` if it needs to see what went out. It sends by pushing messages into
   `cx.out`, reads the time from `cx.now`, and sets `cx.changed = true` when it changed the state. No
   socket, thread, clock or file inside it.
2. If it has something to show, add a section for it to `VehicleState` in `crates/gcs-mavlink/src/state.rs`.
3. If the user can ask it for something, add a variant to `Command` in the same file.
4. In `crates/gcs-mavlink/src/services/mod.rs` add `pub mod <name>;` and one line in `default_services`.

Test it with plain calls and a made-up `now` (see `services/params_tests.rs`), then end to end against the
simulator (`crates/marv-gcs/tests/e2e*.rs`). When the firmware side of the feature is specified, write it
into `docs/link-contract.md` first and teach the simulator (`crates/gcs-sim`) to behave that way, so the
ground station is developed against the real behaviour.

## A new transport (UDP, a telemetry radio)

1. Add a module to `crates/gcs-transport/src/` that opens the link and returns a reader and a writer that
   follow the reader contract in `docs/architecture.md` (return as soon as a byte is there; `TimedOut`
   after 100 ms without data; `Ok(0)` or another error when the link is down).
2. Add a variant to `Endpoint` (with its text form) and an arm in `open`.
3. Add the choice to the connection bar in `crates/marv-gcs/src/connection.rs`.

The MAVLink layer does not change: it takes any reader and writer. This crate never names MAVLink.

## A message outside the standard set

Not needed yet. When the first one appears it becomes one small repo (`marv-dialect`: an XML file that
includes `common.xml`, plus a build script), and the alias `dialect` in `crates/gcs-mavlink/src/lib.rs`
changes in one line, with the same one-line change in the device crate. See `docs/link-contract.md`.

## What to keep true

- `cargo tree -p gcs-transport | grep -c mavlink` is 0.
- `cargo tree -p gcs-mavlink | grep -cE 'serialport|egui|eframe|gcs-transport'` is 0.
- `cargo tree -p gcs-sim | grep -c gcs-mavlink` is 0 (the simulator is written from the specification, so
  the two check each other).
- No parameter name and no message-specific code outside a view that is about that message.
