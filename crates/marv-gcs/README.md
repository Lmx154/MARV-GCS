# marv-gcs

Part of [MARV-GCS](../../README.md) · [ui](../../docs/ui.md)

The desktop window: the connection bar, the HUD and status cards, and one file per tab. Also `marv-link-test`, a window-less
radio link test that builds `MARV_TEST` with `marv-dialect`. Only this crate depends on `eframe`, `walkers` and `rfd`.

| Module | Contents | Node |
|---|---|---|
| `app.rs`, `main.rs`, `cli.rs` | Window, tab row, session and screenshot plumbing, options | [ui](../../docs/ui.md) |
| `connection.rs`, `status.rs` | Connection bar, link status line | [ui](../../docs/ui.md) |
| `theme.rs`, `screenshot.rs` | Look and drawing helpers; `--screenshot` | [ui](../../docs/ui.md) |
| `views/hud.rs`, `views/sidebar.rs` | HUD and status cards | [ui](../../docs/ui.md) |
| `views/sensors.rs`, `plot.rs`, `messages.rs`, `params.rs`, `link.rs`, `motors.rs` | Tabs | [ui-tabs](../../docs/ui-tabs.md) |
| `views/radio.rs`, `devices.rs`, `compass.rs`, `compass_quad.rs`, `presets.rs` | Radio and Devices tabs | [ui-radio-devices](../../docs/ui-radio-devices.md) |
| `views/map.rs`, `views/map/mission.rs`, `views/map/flight.rs`, `missions.rs` | Map, waypoint lists, flight buttons | [ui-map](../../docs/ui-map.md) |
| `views/log.rs`, `views/log_review.rs`, `sessions.rs`, `logfile.rs` | Log tab, sessions, recording files | [sessions-and-logs](../../docs/sessions-and-logs.md) |
| `views/actions.rs`, `views/mod.rs` | Button helpers; the view registry | [extending](../../docs/extending.md) |
| `link_test.rs`, `bin/marv_link_test.rs` | The link test tool | [ui](../../docs/ui.md#link-test-tool) |

Tests: `tests/e2e*.rs` (real transport, MAVLink layer and session against `gcs-sim` over TCP, a pseudo serial port and UDP),
`tests/views_*.rs` (views drawn and clicked headless with `egui_kittest`), `tests/board.rs` (ignored: needs a board),
`tests/real_recording.rs` (ignored: replays a recording given by `MARV_TLOG`).
