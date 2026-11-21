# Architecture

The ground station is eight crates. The dependency graph enforces the layers, so a rule that is broken
does not build or shows up in `cargo tree`.

```
        marv-gcs (window)     marv-cli (command line, no display)
                 \             /
                  gcs-link  (endpoint -> role, start a session)
                 /       \
      gcs-transport     gcs-mavlink            gcs-sim (simulated vehicle; dev and tests only)
      bytes only        MAVLink on any         uses the `mavlink` crate directly,
      serial, TCP, UDP  Read / Write pair      never gcs-mavlink

      gcs-dflog (reads DataFlash .BIN logs from a file; depends on no other crate and no external crate)
      gcs-magcal (compass calibration maths: field model, undo, fit, acceptance; depends on no other crate
                  and no external crate)
```

## Layer rules

1. **USB lives alone.** `gcs-transport` moves bytes. It never depends on `mavlink` or `gcs-mavlink`.
2. **MAVLink is a layer on top.** `gcs-mavlink` takes any `std::io::Read` / `std::io::Write` pair. It
   never depends on `gcs-transport`, `serialport` or a GUI crate.
3. **The join is small and has no GUI.** Only `gcs-link` and the applications (`marv-gcs`, `marv-cli`) depend
   on both `gcs-transport` and `gcs-mavlink`. `gcs-link` holds exactly what every application needs to turn an
   endpoint into a running session: `access_of` (the role a link gives the station) and `start_session`; it also
   holds `session_folder`, the converter of a `.tlog` into a session folder (it writes files, which `gcs-mavlink`
   never does, and both applications call it; it adds `serde_json` and no GUI crate). Only
   `marv-gcs` depends on `eframe`, on `walkers` (the Map tab's tiles; it brings an HTTP client) and on `rfd` (the file dialog); `gcs-link`
   and `marv-cli` never depend on a GUI crate or an HTTP client.
4. **The simulator is independent.** `gcs-sim` never depends on `gcs-mavlink`; it is written from the
   MAVLink specification so the two check each other.
5. **Protocol logic is sans-IO.** A service is a state machine: messages and the time go in, messages to
   send and events come out. No socket, thread, clock or GUI inside it, so it is tested with plain calls.
6. **Nothing is specific to one parameter or one message** unless a view is about that message. The
   parameter list is whatever the vehicle reports. Every received message is listed and plottable by
   generic field access.
7. **The log reader is independent.** `gcs-dflog` reads a file, not a link: it depends on no crate of the
   workspace and on no external crate (the applications use it for the Log tab's review section), and has no GUI.
8. **The compass calibration maths is pure.** `gcs-magcal` is numbers in, numbers out: no I/O, no clock, no link,
   no GUI; it depends on no crate of the workspace and on no external crate (std only, its linear algebra is its own).

Check: `cargo tree -p gcs-transport | grep -c mavlink` is 0; `cargo tree -p gcs-mavlink | grep -cE 'serialport|egui|eframe'` is 0; `cargo tree -p gcs-sim | grep -c gcs-mavlink` is 0; `cargo tree -p gcs-dflog -e normal | wc -l` is 1 (the crate has no dependency: only its own line is printed); `cargo tree -p gcs-magcal -e normal | wc -l` is 1; `cargo tree -p gcs-link -e normal | grep -ciE 'eframe|egui|winit|wgpu|glow'` is 0; `cargo tree -p marv-cli -e normal | grep -ciE 'eframe|egui|winit|wgpu|glow'` is 0 (`crates/marv-cli/tests/dependencies.rs` runs the last two).

## How to extend (kept true as the code grows; details in `docs/extending.md`)

| To add | Change |
|---|---|
| A parameter or setting | Nothing here. The firmware declares it; the parameter view lists what the vehicle reports. |
| A telemetry message from the standard set | Nothing here. It appears in the message list and can be plotted. |
| A custom message | The dialect: add it in `marv-dialect`, bump its rev here; the station reads it through one alias in `gcs-mavlink` (`dialect`, which is `marv_dialect::marv`), see `docs/extending.md`. |
| A view (tab) | One file in `crates/marv-gcs/src/views/`, plus its `mod` line and its registry line in `views/mod.rs`. |
| A transport (radio, ...) | One module in `gcs-transport` and one `Endpoint` variant. |
| A sensor preset (a checked combination of existing parameters) | One entry in `gcs-mavlink/src/presets/tables.rs` (`gcs_mavlink::presets`, used by the Devices cards and marv-cli's `--debug` gate); the firmware needs nothing. |
| A protocol service (missions, rate control; log download is the worked example, `services/logs.rs`) | One sans-IO module in `gcs-mavlink/src/services/`, plus its `mod` line and its registry line in `services/mod.rs`; a section in `VehicleState` and a `Command` variant if it shows or takes something. |

## Interfaces between the crates

### `gcs-transport`

```rust
pub enum Endpoint {
    Serial { path: String, baud: u32 },   // "serial:/dev/ttyACM0:115200"
    Tcp { addr: String },                 // "tcp:127.0.0.1:5760"
    Udp { bind: std::net::SocketAddr },   // "udp:0.0.0.0:14550", the local address to listen on
}
// Endpoint: FromStr + Display, round-trip exact.

pub struct Link {
    pub reader: Box<dyn std::io::Read + Send>,
    pub writer: Box<dyn std::io::Write + Send>,
}
pub fn open(endpoint: &Endpoint) -> Result<Link, OpenError>;

pub const READ_TIMEOUT: std::time::Duration; // 100 ms

pub struct PortInfo { pub path: String, pub usb: Option<UsbInfo> }
pub struct UsbInfo { pub vid: u16, pub pid: u16, pub manufacturer: Option<String>, pub product: Option<String>, pub serial_number: Option<String> }
pub fn list_serial_ports() -> Vec<PortInfo>;
```

What the layer above relies on, for every transport:
- `reader.read(buf)` returns as soon as at least one byte is there (`Ok(n)`, `n >= 1`); it does not wait
  to fill `buf`.
- With no data for `READ_TIMEOUT` it returns `Err` of kind `TimedOut` (never `WouldBlock`), so the reading
  thread can look at its stop flag. A timeout loses no bytes.
- `Ok(0)` or any other error means the link is down for good (peer closed, cable pulled).
- The two halves may be used from two threads at once. Dropping both closes the link.

UDP specifics (`crates/gcs-transport/src/udp.rs`): the peer is the source of the latest datagram; before
one arrives a write succeeds and discards the bytes; each write is one datagram; the reader returns a
datagram over several reads when the buffer is small and skips empty datagrams; `ConnectionRefused`,
`ConnectionReset`, `NetworkUnreachable`, `HostUnreachable` and `NetworkDown` (a peer that went away or a
network that dropped) are not link-down, on read or on write: a write is discarded and the link stays up.

### `gcs-link`

```rust
/// The one mapping from a link to a role: Serial and Tcp are Configure, Udp is Operate. Exhaustive over `Endpoint`.
pub fn access_of(endpoint: &gcs_transport::Endpoint) -> gcs_mavlink::Access;
/// Open the endpoint and start a session on it with `access`; `wake` is called when the state changed.
pub fn start_session(endpoint: &Endpoint, access: Access, wake: Box<dyn Fn() + Send + Sync>)
    -> Result<gcs_mavlink::Session, gcs_transport::OpenError>;

// gcs_link::session_folder (README, "Sessions"): a recording into <out_dir>/<UTC start>_<name>/, never overwriting.
pub fn export_session(tlog: &Path, out_dir: &Path, options: &SessionOptions) -> io::Result<Exported>;
pub struct SessionOptions { pub name: String, pub parameters: Option<Vec<(String, f64)>>, pub marks: Vec<UserMark> }
pub struct UserMark { pub time_us: u64 /* Unix microseconds */, pub text: String }
pub fn list_sessions(dir: &Path) -> io::Result<Vec<SessionInfo>>;
```

### `gcs-mavlink`: framing

```rust
pub use marv_dialect::marv as dialect;   // the one place that names the dialect: `common` plus the project's own messages
pub type Message = dialect::MavMessage;

pub struct Frame {
    pub system_id: u8,
    pub component_id: u8,
    pub sequence: u8,
    pub message_id: u32,
    pub raw: Vec<u8>,      // the whole frame exactly as it was on the wire
    pub body: Body,
}
pub enum Body {
    Known(Message),            // id in the dialect, checksum good, decoded
    Undecodable(String),       // id in the dialect, checksum good, payload refused by the decoder
    Unknown,                   // id not in the dialect: checksum cannot be checked
}

pub struct Deframer;           // sans-IO: bytes in, frames out
impl Deframer {
    pub fn new() -> Self;
    pub fn push(&mut self, bytes: &[u8], out: &mut Vec<Frame>);
    pub fn stats(&self) -> RxStats;
}
pub struct RxStats { pub frames: u64, pub crc_errors: u64, pub unknown: u64, pub undecodable: u64, pub discarded_bytes: u64 }

pub struct Encoder;            // MAVLink 2, own sequence counter
impl Encoder {
    pub fn new(system_id: u8, component_id: u8) -> Self;
    pub fn encode(&mut self, message: &Message) -> Vec<u8>;
}
```

### `gcs-dflog`

```rust
// Any stream or file; every pass reports progress and can be cancelled (on_progress returns Control::Stop).
pub struct Reader<R>;                       // Reader::open(path), Reader::new(read, total)
pub fn scan(reader, on_record, on_progress) -> io::Result<bool>;
pub fn summary(path) -> io::Result<Summary>;                       // counts, times, FMT list, wall clock
pub fn events(path) -> io::Result<Vec<Event>>;                     // the event timeline (events_with: progress)
pub struct Event { pub time_us: u64, pub source: EventSource, pub text: String, pub initial: bool }
pub fn Condition::parse(text) -> Result<Condition, ConditionError>;       // syntax: see the README
pub fn Condition::bind(&self, formats: &[Format]) -> Result<BoundCondition, ConditionError>;
pub fn find_with(reader, &BoundCondition, on_progress) -> io::Result<Option<Found>>; // the intervals
pub fn export_with(reader, &ExportRequest, on_progress) -> Result<Export, ExportError>; // CSV files, one per type
pub fn flight_report_with(reader /* Read + Seek */, &ReportInputs, on_progress) -> io::Result<Option<FlightReport>>; // health, flights, motor fit, vibration (second pass over the IMU)
```

### `gcs-magcal`

```rust
// Units: uT and degrees. Decision `_plan/decision-compass-cal-20261120.md`, link contract section 16.
pub fn wmm::field(lat_deg, lon_deg, height_m, decimal_year) -> Result<wmm::Field, wmm::WmmError>;  // WMM2025, NOAA's WMM.COF
pub struct wmm::Field { pub intensity_ut, pub inclination_deg, pub declination_deg, pub horizontal_ut, pub ned_ut: [f64; 3] }
pub fn undo::gauss_to_ut(gauss: [f32; 3]) -> [f64; 3];                       // HIGHRES_IMU gauss -> uT
pub struct undo::Calibration { pub offsets, pub diag, pub offdiag }          // MAG_OFF*, MAG_DIAG*, MAG_ODIAGXY/XZ/YZ
impl Calibration { fn matrix(&self); fn apply(&self, raw) -> m; fn undo(&self) -> Result<Undo, InvalidMatrix>; }
impl Undo { fn raw(&self, published_ut) -> [f64; 3]; }                       // r = S^-1 m + o
pub fn fit::fit(samples_ut: &[[f64; 3]], initial_radius_ut: f64) -> fit::Fit; // PX4 LM: sphere, then ellipsoid
pub fn accept::accept(&accept::Inputs) -> Result<accept::Accepted, accept::Refusal>;  // section 4, in order
```

### `gcs-sim`

```rust
pub struct SimVehicle;         // sans-IO, virtual time
impl SimVehicle {
    pub fn new(config: SimConfig) -> Self;
    /// Give it the bytes received since the last call and the time; get the bytes to send.
    pub fn step(&mut self, now_ms: u64, received: &[u8]) -> Vec<u8>;
}
/// One vehicle that keeps running across connections; tests keep a handle to look inside it.
pub fn serve_tcp(
    listener: std::net::TcpListener,
    vehicle: std::sync::Arc<std::sync::Mutex<SimVehicle>>,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
);
// binary: marv-sim --tcp 127.0.0.1:5760
```

### `gcs-mavlink`: services, state, session

A service is one sub-protocol as a state machine (rule 5). The session owns the threads and the clock
and calls the services; the window only reads `VehicleState` and sends `Command`s.

```rust
/// Everything the window shows. One section per service; a new service adds a section.
pub struct VehicleState {
    pub link: LinkState,          // heartbeat service
    pub messages: MessageStore,   // every message: rate, fields, history of numeric fields
    pub params: ParamTable,       // the vehicle's parameters, fetch and write status
    pub texts: TextLog,           // STATUSTEXT
    pub log: LogState,            // the recording
    pub logs: LogsState,          // the vehicle's logs: the last listing, the current download (service `logs`)
    pub command: Option<CommandRecord>, // the current or last MAVLink command (service `commands`)
}

pub enum LinkStatus {
    Connecting,          // link open, no vehicle heartbeat yet
    Connected,           // vehicle heartbeat seen within the timeout
    Lost,                // link open, heartbeats stopped
    Closed(String),      // transport is down; the text says why
}
pub struct LinkState {
    pub status: LinkStatus,
    pub vehicle: Option<VehicleId>,          // from the vehicle's HEARTBEAT
    pub last_heartbeat: Option<Duration>,    // session time of the last one
    pub rx: RxStats,
    pub tx_frames: u64,
    pub refused: Option<Refused>,            // the last command `Session::send` refused, and how many
}
pub struct Refused { pub command: &'static str, pub count: u64 }
pub enum Access { Operate, Configure }       // fixed per session; `Command::access` says what a command needs
pub struct VehicleId { pub system_id: u8, pub component_id: u8, pub mav_type: String, pub autopilot: String }

/// What the window can ask for. One variant per user action; grows with the features.
pub enum Command {
    RefreshParams,
    SetParam { name: String, value: ParamValue },
    StartLog { sink: LogSink, label: String },   // the window opens the file and hands it over
    StopLog,
    StartSessionLog { sink: LogSink, label: String },  // a second recording, for a monitoring session; `session_log` says when it is closed
    StopSessionLog,
    SendCommand { command: MavCmd, params: [f32; 7] },          // Configure
    ListLogs,                                                    // Configure: the three log commands
    DownloadLog { number: u16, size: u32, sink: LogSink },      // (docs/link-contract.md section 11):
    CancelLogDownload,                                           // the sink is a writer the caller opened
}

pub struct Ctx<'a> {
    pub now: Duration,                 // time since the session started
    pub wall_time_us: u64,             // Unix time, for the log
    pub state: &'a mut VehicleState,
    pub out: &'a mut Vec<Message>,     // messages to send
    pub changed: bool,                 // a service sets this when it changed `state`; the window is then woken
}
pub trait Service: Send {
    fn on_frame(&mut self, _frame: &Frame, _cx: &mut Ctx) {}
    fn on_sent(&mut self, _raw: &[u8], _cx: &mut Ctx) {}
    fn on_tick(&mut self, _cx: &mut Ctx) {}        // at least every 50 ms
    fn on_command(&mut self, _command: &Command, _cx: &mut Ctx) {}
}
/// The registry: the one line to touch when a service is added.
pub fn default_services(config: &SessionConfig) -> Vec<Box<dyn Service>>;

pub struct SessionConfig { pub system_id: u8 /* 255 */, pub component_id: u8 /* 190 */, pub heartbeat_timeout: Duration /* 5 s */, pub access: Access /* Operate */ }

pub struct Session;
impl Session {
    /// Starts the threads. `wake` is called (at most about 30 times a second) when the state changed.
    pub fn start(
        reader: Box<dyn Read + Send>,
        writer: Box<dyn Write + Send>,
        config: SessionConfig,
        wake: Box<dyn Fn() + Send + Sync>,
    ) -> Session;
    pub fn state(&self) -> std::sync::MutexGuard<'_, VehicleState>;
    /// The one choke point: a command that needs more `Access` than the session has is refused,
    /// nothing is written and `link.refused` records it.
    pub fn send(&self, command: Command);
    // Drop stops the threads and closes the link (within 300 ms).
}
```

The reader half must follow the transport contract above (`TimedOut` = no data yet; `Ok(0)` or another
error = link down). When the link goes down the session sets `LinkStatus::Closed` and stops.

### `marv-gcs`: views

```rust
pub struct ViewCtx<'a> {
    pub state: &'a VehicleState,          // locked for the frame
    pub commands: &'a mut Vec<Command>,   // sent after the frame
    pub now: Option<Duration>,            // session time; `None` when there is no session
    pub access: Access,                   // the session's role; configure controls are shown disabled on Operate
}
pub trait View {
    fn name(&self) -> &'static str;       // tab title
    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx);
}
/// The registry in `views/mod.rs`: the one line to touch when a view is added.
pub fn all() -> Vec<Box<dyn View>>;
```

### `marv-cli`

```rust
pub fn run(args: &[String], out: &mut dyn Write, err: &mut dyn Write) -> i32;   // parse, open, execute; the exit status
pub mod args { pub fn parse(args: &[String]) -> Result<Parsed, String>; }       // by hand, no dependency
pub mod commands { pub fn execute(invocation, session, access, out, err) -> i32; }  // on a session that exists
```

One command per invocation, on one link; the usage, the output format and the exit statuses are in
`README.md`. It sends everything through `Session::send` and has no gate of its own.
