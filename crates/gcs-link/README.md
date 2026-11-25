# gcs-link

Part of [MARV-GCS](../../README.md) · [architecture](../../docs/architecture.md)

The small join with no GUI that both applications use: the one mapping from an endpoint to the role it gives the station,
the start of a session on an endpoint, and the converter from a `.tlog` recording to a session folder.

| Module | Contents | Node |
|---|---|---|
| `lib.rs` | `access_of` (Serial, TCP = Configure; UDP = Operate), `start_session` | [link-access](../../docs/link-access.md) |
| `session_folder.rs` | `export_session`, `list_sessions`, the CSV and `session.json` writer, the units table | [sessions-and-logs](../../docs/sessions-and-logs.md) |

Tests: `src/session_folder_tests.rs`. The crate adds `serde_json` and no GUI crate or HTTP client.
