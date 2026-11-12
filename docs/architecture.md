# Architecture

The ground station is four crates. The dependency graph enforces the layers, so a rule that is broken
does not build or shows up in `cargo tree`.

```
                 marv-gcs  (window: views, wiring)
                 /       \
      gcs-transport     gcs-mavlink            gcs-sim (simulated vehicle; dev and tests only)
      bytes only        MAVLink on any         uses the `mavlink` crate directly,
      USB serial, TCP   Read / Write pair      never gcs-mavlink
```

## Layer rules

1. **USB lives alone.** `gcs-transport` moves bytes. It never depends on `mavlink` or `gcs-mavlink`.
2. **MAVLink is a layer on top.** `gcs-mavlink` takes any `std::io::Read` / `std::io::Write` pair. It
   never depends on `gcs-transport`, `serialport` or a GUI crate.
3. **The window joins them.** Only `marv-gcs` depends on both, and on `eframe`.
4. **The simulator is independent.** `gcs-sim` never depends on `gcs-mavlink`; it is written from the
   MAVLink specification so the two check each other.
5. **Protocol logic is sans-IO.** A service is a state machine: messages and the time go in, messages to
   send and events come out. No socket, thread, clock or GUI inside it, so it is tested with plain calls.
6. **Nothing is specific to one parameter or one message** unless a view is about that message. The
   parameter list is whatever the vehicle reports. Every received message is listed and plottable by
   generic field access.

Check: `cargo tree -p gcs-transport | grep -c mavlink` is 0; `cargo tree -p gcs-mavlink | grep -cE 'serialport|egui|eframe'` is 0; `cargo tree -p gcs-sim | grep -c gcs-mavlink` is 0.

## How to extend (kept true as the code grows; details in `docs/extending.md`)

| To add | Change |
|---|---|
| A parameter or setting | Nothing here. The firmware declares it; the parameter view lists what the vehicle reports. |
| A telemetry message from the standard set | Nothing here. It appears in the message list and can be plotted. |
| A custom message | The dialect: one alias in `gcs-mavlink` (`dialect`). |
| A view (tab) | One file in `crates/marv-gcs/src/views/`, plus its `mod` line and its registry line in `views/mod.rs`. |
| A transport (UDP, radio) | One module in `gcs-transport` and one `Endpoint` variant. |
| A protocol service (missions, log download) | One sans-IO module in `gcs-mavlink/src/services/`, plus its `mod` line and its registry line in `services/mod.rs`; a section in `VehicleState` and a `Command` variant if it shows or takes something. |

## Interfaces between the crates

### `gcs-transport`

```rust
pub enum Endpoint {
    Serial { path: String, baud: u32 },   // "serial:/dev/ttyACM0:115200"
    Tcp { addr: String },                 // "tcp:127.0.0.1:5760"
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

### `gcs-mavlink`: framing

```rust
pub use mavlink::dialects::common as dialect;   // the one place that names the dialect
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
}
pub struct VehicleId { pub system_id: u8, pub component_id: u8, pub mav_type: String, pub autopilot: String }

/// What the window can ask for. One variant per user action; grows with the features.
pub enum Command {
    RefreshParams,
    SetParam { name: String, value: ParamValue },
    StartLog { sink: LogSink, label: String },   // the window opens the file and hands it over
    StopLog,
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

pub struct SessionConfig { pub system_id: u8 /* 255 */, pub component_id: u8 /* 190 */, pub heartbeat_timeout: Duration /* 5 s */ }

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
}
pub trait View {
    fn name(&self) -> &'static str;       // tab title
    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx);
}
/// The registry in `views/mod.rs`: the one line to touch when a view is added.
pub fn all() -> Vec<Box<dyn View>>;
```
