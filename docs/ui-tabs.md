# Sensors, Messages, Parameters, Link and Motors tabs

Part of [MARV-GCS](../README.md)

Five of the window's tabs. Each reads the shared vehicle state and sends commands only on a click. Configure-only
controls are shown disabled with the reason on an Operate link ([link-access](link-access.md)). The other tabs:
[Radio and Devices](ui-radio-devices.md), [Map](ui-map.md), [Log](sessions-and-logs.md).

## Sensors

Live plots from a table of presets; one row adds a plot. Each is a message and fields the real firmware sends.

| Preset plot | Source |
|---|---|
| IMU1 acceleration and rotation, magnetic field (gauss) | `HIGHRES_IMU` `id` 0 (the magnetometer is in `xmag`/`ymag`/`zmag`) |
| IMU2 acceleration and rotation | `SCALED_IMU2` (mG and mrad/s, converted to m/s² and rad/s as the titles say) |
| Pressure | `SCALED_PRESSURE` |
| Battery voltage (V) and current (A) | `SYS_STATUS` (65535 mV and -1 cA mean not available and are left out) |

While the pointer is over a plot, a popup next to the crosshair gives the time and, for every series, its swatch, name
and the sample nearest in time (not interpolated) with the unit, highest first. A series within 6 px of the pointer is
bold, so overlapping lines are told apart. It works the same live and replaying a `.tlog`. There is no high-g plot: the
firmware sends nothing of that sensor.

At the top is the session bar: a name field and Start; while a session runs, the name, elapsed time, a note field with
Mark (Enter marks too) and Stop, and one line with the marks so far; after Stop `Saved: <folder>` or the error. Start is
disabled with its reason when not connected, replaying, or run with `--no-log`. A session sends nothing, so Operate and
Configure both can start one. What a session writes: [sessions-and-logs](sessions-and-logs.md).

## Messages

Every message that arrives: rate, count, age and all its fields; any numeric field can be plotted. Nothing here knows a
particular message. Messages with several instances (`HIGHRES_IMU` by `id`, `MARV_DEVICE_STATUS` by `device_id`) get one
row and one plot series per instance. A frame whose payload the dialect refuses (an unknown enum value) is counted in the
Link tab's "Undecodable frames" and listed here with no fields.

## Parameters

Whatever parameters the vehicle reports ([link-parameters](link-parameters.md)).

| Feature | Behaviour |
|---|---|
| Tree | Search, then a collapsible tree by scope (Flight controller, RC controller, ESC, each with its count) and, inside it, by group (the name's prefix before the first `_`, natural order so RC2 precedes RC10; a name without `_` is in "(other)"). Sections start open, groups closed; a search shows only matching rows with groups opened and counts as "matches of total" |
| Edit | Write with confirmation; a refused or unanswered write says so. Save to and load from a file (`NAME,VALUE` per line, `#` comments; the layout Mission Planner reads) |
| Storage buttons | Save to flash, Reload from flash, Reset all to defaults, Reset calibration (the two resets ask for a second click) |
| Metadata | A Unit column, and on hovering a name a one-line description and the allowed values, from the station's own table `gcs_mavlink::param_info` (the link carries no metadata); a name not in the table shows no unit and no hover |
| Preset fields | While the Devices tab's debug switch is off, every field a sensor preset sets is read only with "set in Devices > Sensor presets" (a file load that stages such a field does not send it); with the switch on they are edited like any parameter (IMU ones marked "applies after reboot") |
| Operate link | The list is not downloaded at connect (press Refresh); editing controls are shown disabled with the reason |

Group prefixes: `RC<n>` and `RCMAP` are the RC controller; `ESC`, `MOT` and `DSHOT` are the ESC; all others are the
flight controller.

## Link

Connection state and counters: frames received and sent, checksum errors, undecodable frames, discarded bytes; the
session's role (Access) and the last command it refused; the "Reboot vehicle" button (asks for a second click).

## Motors

Read-only ESC telemetry from `ESC_STATUS` and `ESC_INFO` ([link-telemetry](link-telemetry.md)) plus the motor test.

| Part | Behaviour |
|---|---|
| Strip | "ESCs online n/4", the ESC device row's state and the link type |
| Four boxes | Motor 1 to 4: rpm, voltage, temperature, current, error percentage (100 x change of `error_count` / change of `counter` between the two latest `ESC_INFO`; blank when they are more than 60 s apart). "OFFLINE" in the title and a red frame when the ESC's `info` bit is 0; grey values when the message is older than 2 s; "No ESC telemetry from the vehicle" when none ever arrived |
| Plot | The four rpm series |
| Motor test strip | "Motor test (propellers off)": a switch "Propellers are removed and I accept the risk" (off at every connect and new session), a throttle field in percent (0.05 to 5.00, default 5.00), a duration in seconds (0.1 to 3.0, default 2.0; out-of-range values go to the nearest limit, text that is not a number to the minimum), a Spin button per box and one Stop button |

Spin sends command 209 for that motor with the two fields, param2 0, param5 1, param6 2; Stop sends 209 with throttle 0
for the motor last spun (motor 1 if none). Spin needs the switch and an open command gate (Configure session, connected,
no command outstanding); Stop needs only the gate; a disabled control shows why. A refused test shows the result and the
vehicle's `STATUSTEXT`. Nothing is sent while a field is edited, a spin is never repeated, and without a click the tab
sends no command. Wire rules: [link-commands](link-commands.md).

## Code

`crates/marv-gcs/src/views/`: `sensors.rs`, `plot.rs`, `messages.rs`, `params.rs`, `link.rs`, `motors.rs`; shared button
helpers in `actions.rs`.

## Related

[ui](ui.md) · [ui-radio-devices](ui-radio-devices.md) · [link-parameters](link-parameters.md) ·
[sessions-and-logs](sessions-and-logs.md)
