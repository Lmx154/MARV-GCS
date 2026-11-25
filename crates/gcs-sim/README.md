# gcs-sim

Part of [MARV-GCS](../../README.md) · [architecture](../../docs/architecture.md)

The simulated vehicle (`marv-sim`), for developing and testing the station without a flight controller. It is a sans-IO
`SimVehicle` (virtual time, bytes in, bytes out) plus a real-time TCP server. It is written from the
[link contract](../../docs/link-contract.md) and the MAVLink specification and never uses `gcs-mavlink`, so the two check
each other; it may use `gcs-magcal` for the Earth's field.

```
cargo run -p gcs-sim                      # marv-sim on 127.0.0.1:5760
```

| Module | Contents | Node |
|---|---|---|
| `vehicle.rs` | The vehicle: heartbeat, streams, parameters, dispatch | [link-contract](../../docs/link-contract.md) |
| `server.rs`, `main.rs` | TCP server, one client at a time; the `marv-sim` binary | [ui](../../docs/ui.md) |
| `params.rs`, `fwparams.rs`, `presets.rs` | Parameters and their encoding; the firmware's later indices; sensor fields | [link-parameters](../../docs/link-parameters.md) |
| `actions.rs` | Commands and the one running action | [link-commands](../../docs/link-commands.md) |
| `devices.rs` | `MARV_DEVICE_STATUS` rows | [link-telemetry](../../docs/link-telemetry.md) |
| `rcmap.rs` | RC parameters and `MANUAL_CONTROL` | [link-telemetry](../../docs/link-telemetry.md) |
| `logs.rs` | The logs on the simulated card | [link-logs](../../docs/link-logs.md) |
| `missions.rs` | The vehicle's side of the mission service | [link-missions](../../docs/link-missions.md) |
| `mag.rs`, `script.rs` | Simulated compass and the scripted calibration user | [link-compass](../../docs/link-compass.md) |
| `signals.rs` | Sensor values as functions of time | |

Tests: `tests/*.rs` check it against the contract's tables (RC examples, mission vector, log download, compass runs).
It is a test double, not the firmware: it sends `ATTITUDE` at 10 Hz for the HUD, does not reboot, and runs disarmed.
