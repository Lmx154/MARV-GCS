# gcs-dflog

Part of [MARV-GCS](../../README.md) · [architecture](../../docs/architecture.md)

A generic, streaming reader of ArduPilot DataFlash `.BIN` logs, driven by the file's own FMT records, with one-pass
operations on top: summary, event timeline, condition search, CSV export and the flight report. No GUI and no dependency.

| Module | Contents | Node |
|---|---|---|
| `reader.rs`, `format.rs` | `Reader`, `scan`, record decoding, progress and cancellation | [dflog](../../docs/dflog.md) |
| `summary.rs` | One-pass summary | [dflog](../../docs/dflog.md) |
| `events.rs` | Event timeline | [dflog](../../docs/dflog.md) |
| `condition.rs` | Condition language and `find_with` | [dflog](../../docs/dflog.md) |
| `export.rs` | `export_with`, CSV files | [dflog](../../docs/dflog.md) |
| `flight_report.rs` | Flights, thrust constant, identification, health | [flight-report](../../docs/flight-report.md) |
| `vibration.rs`, `tracking.rs`, `ground.rs`, `values.rs` | Spectra, attitude tracking, ground tilt check, closing values block | [flight-report](../../docs/flight-report.md) |

Tests: `src/tests*.rs` (synthetic logs). The analysis never opens a position record.
