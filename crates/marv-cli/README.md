# marv-cli

Part of [MARV-GCS](../../README.md) · [marv-cli](../../docs/marv-cli.md)

The command line: one command per invocation on one link, or a command on a file with no link. It depends on no GUI crate
and sends every write through `Session::send`.

| Module | Contents | Node |
|---|---|---|
| `main.rs`, `lib.rs` | `run`, exit statuses | [marv-cli](../../docs/marv-cli.md) |
| `args.rs` | Hand-written parser and the usage text | [marv-cli](../../docs/marv-cli.md) |
| `commands.rs` | Commands on a started session: status, param, calibrate, logs, mission, flight | [marv-cli](../../docs/marv-cli.md) |
| `waypoints.rs` | The `.waypoints` (QGC WPL 110) file with its return row | [link-missions](../../docs/link-missions.md) |
| `session.rs` | `session export` and `session list` | [sessions-and-logs](../../docs/sessions-and-logs.md) |
| `log_report.rs` | `log report` | [flight-report](../../docs/flight-report.md) |
| `log_tune.rs`, `log_tune/` | `log tune`: events, response, spectra, rendering | [marv-cli](../../docs/marv-cli.md) |
| `log_survey.rs` | `log survey` | [marv-cli](../../docs/marv-cli.md) |

Tests: `tests/` (arguments, every command against `gcs-sim`, the operate-gate negative control in `tests/gate.rs`, and
`tests/dependencies.rs`).
