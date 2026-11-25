# gcs-transport

Part of [MARV-GCS](../../README.md) · [architecture](../../docs/architecture.md)

Bytes only: opens a USB serial port, a TCP connection or a UDP socket from an `Endpoint` and returns a reader and a writer.
It never names MAVLink and never depends on `mavlink` or `gcs-mavlink`. Every transport follows one reader contract (return
as soon as a byte is there, `TimedOut` after 100 ms without data, `Ok(0)` or another error means the link is down).

| Module | Contents | Node |
|---|---|---|
| `endpoint.rs` | `Endpoint` (`serial:PATH[:BAUD]`, `tcp:HOST:PORT`, `udp:ADDR:PORT`), parse and display round trip | [architecture](../../docs/architecture.md#gcs-transport) |
| `serial.rs` | Serial port open and `list_serial_ports` (path, USB ids, product) | [ui](../../docs/ui.md) |
| `tcp.rs` | TCP client (the simulator) | [architecture](../../docs/architecture.md) |
| `udp.rs` | UDP: peer is the latest sender, datagram handling, which errors keep the link up | [architecture](../../docs/architecture.md) |
| `lib.rs` | `open`, `Link`, `READ_TIMEOUT` | |

Tests: `tests/tcp.rs`, `tests/udp.rs` (a plain peer socket), `tests/serial_pty.rs` (a pseudo serial port; needs
`/usr/bin/socat`).
