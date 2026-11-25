# gcs-mavlink

Part of [MARV-GCS](../../README.md) · [architecture](../../docs/architecture.md)

MAVLink 2 on any `Read` / `Write` pair: framing, one sans-IO service per sub-protocol, the vehicle state the window reads,
`.tlog` recording and replay, the sensor-preset table and the parameter metadata table. No transport, no window.

| Module | Contents | Node |
|---|---|---|
| `frame.rs` | `Deframer`, `Encoder`, `Frame`, `Body`, `RxStats` | [architecture](../../docs/architecture.md#gcs-mavlink) |
| `session.rs` | `Session`: threads, clock, `send` (the access gate) | [link-access](../../docs/link-access.md) |
| `state.rs` | `VehicleState`, `Command` and its `Access` class | [link-access](../../docs/link-access.md) |
| `services/heartbeat.rs` | Station heartbeat, vehicle link state | [link-contract](../../docs/link-contract.md) |
| `services/params.rs` | Parameter list, reads, writes, retries | [link-parameters](../../docs/link-parameters.md) |
| `services/commands.rs` | One `COMMAND_LONG` at a time, ACK following, the calibration cancel | [link-commands](../../docs/link-commands.md) |
| `services/logs.rs` | On-board log listing and download | [link-logs](../../docs/link-logs.md) |
| `services/missions.rs` | Mission upload, read-back, clear, verdict | [link-missions](../../docs/link-missions.md) |
| `services/messages.rs` | Message store, instance rows, field history | [ui-tabs](../../docs/ui-tabs.md) |
| `services/texts.rs` | `STATUSTEXT` log | |
| `flight.rs` | The five flight commands, deny texts, the effective mode | [link-flight](../../docs/link-flight.md) |
| `presets.rs`, `presets/tables.rs` | Sensor presets (station feature) | [link-presets](../../docs/link-presets.md) |
| `param_info.rs` | Description, unit and range of the firmware's parameters | [ui-tabs](../../docs/ui-tabs.md) |
| `param_file.rs` | `NAME,VALUE` parameter files | [ui-tabs](../../docs/ui-tabs.md) |
| `tlog.rs` | `.tlog` record, read, replay as a link | [sessions-and-logs](../../docs/sessions-and-logs.md) |
| `test_pipe.rs` | In-memory byte pipe for tests | |

Tests sit beside each module (`*_tests.rs`) and use plain calls with a made-up clock; `tests/presets.rs` checks the preset
table against the contract's formulas.
