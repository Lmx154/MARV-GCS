# Link contract: flight controller and ground station

Decision of 2026-11-12 (architect; recorded by the lead). It binds both ends. It follows the owner's rule:
"do not combine usb with mavlink. usb lives alone mavlink is a layer on top of it that utilizes usb".
Evidence paths: `EU` = embassy-usb 0.6.0 source, `ERP` = embassy-rp 0.10.0 source, `MC` = mavlink-core
0.21.0 source, `MX` = the MAVLink XML shipped inside the `mavlink` 0.21.0 crate.

## 1. The layers

```
computer                                    flight controller
--------                                    -----------------
marv-gcs (window)                           firmware (owns identity, tasks, parameters, sensors)
gcs-mavlink   <---- MAVLink 2 frames ---->  mavlink-device   (any embedded-io-async Read / Write)
gcs-transport <---- bytes ---------------->  usb-cdc          (bytes only; never names MAVLink)
serial port of the operating system         embassy-usb CDC-ACM
```

The computer needs no driver: Linux, Windows and macOS show CDC-ACM as a serial port.

## 2. Wire contract (what both ends must do)

Identity and heartbeat
- Vehicle: system 1, component 1. Ground station: system 255, component 190.
- Vehicle HEARTBEAT at 1 Hz, not changeable: type GENERIC, autopilot GENERIC, base_mode 0, custom_mode 0,
  system_status set by the firmware (BOOT, then STANDBY), mavlink_version 3.
- Ground station HEARTBEAT at 1 Hz: type GCS, autopilot INVALID, base_mode 0, status ACTIVE. The first is
  sent at once after the port is open and configured.
- Each end calls the other lost after 5.0 s without its HEARTBEAT. A transport error is lost at once.
- MAVLink 2 only. The vehicle drops MAVLink 1 frames, frames with any incompat flag, frames with its own
  system id and frames addressed to another target. No signing.

Parameters
- Encoding is byte-wise: the value's own little-endian bytes in the 4-byte value field. The firmware uses
  only INT32 and REAL32 (a bool or an enum is an INT32).
- The vehicle answers MAV_CMD_REQUEST_MESSAGE (512) for AUTOPILOT_VERSION (148) with capabilities
  MAVLINK2 (8192) | PARAM_ENCODE_BYTEWISE (16).
- Ground station: ask for AUTOPILOT_VERSION on connect and use the capability bit. With no reply after
  3 x 500 ms, or neither bit set: C-cast when HEARTBEAT.autopilot is ARDUPILOTMEGA, otherwise byte-wise.
  It stores the raw 4 bytes and the type and decodes only to show or to set, so the list never waits for
  the encoding.
- Vehicle: list paced at one PARAM_VALUE per 2 ms; read by index, or by name when the index is -1; a
  PARAM_SET is always answered with a PARAM_VALUE carrying the value now in effect, also when refused;
  an unknown name or index gets no reply; the set, its order and its count are fixed for one boot;
  names are at most 16 characters of [A-Z0-9_], unique, stable across versions. No PARAM_ERROR (marked
  work in progress in `MX/common.xml`).
- Ground-station timing (the specification gives none): list: no PARAM_VALUE within 1 s, resend the
  request, up to 3 times; 1 s of silence with entries missing, request each missing index (500 ms, 3
  tries), then report it missing. Read: 500 ms x 3. Set: 3 sends, 500 ms apart; an echo equal in raw
  bytes confirms at any moment; 500 ms after the third send the write is refused if the vehicle answered
  with another value (the value in effect is shown), unanswered if it did not answer. A change of
  param_count means fetch everything again.
- Amendments by the lead after review (2026-11-12). (1) A different echo does not end a write at once:
  a stale copy of the old value still in flight would read as a refusal that is false. (2) A write waits
  until the encoding is known, so it is never sent in the wrong encoding. (3) A vehicle with no parameters
  answers the list request with one PARAM_VALUE whose param_count is 0; the ground station shows an empty,
  complete list. The device layer must do this: silence would read as "no answer".

Sensors (default rate, fastest allowed)
- HIGHRES_IMU (105), SI units, one instance per sensor in its `id` field, 50 Hz per instance (200 Hz):
  id 0 ICM-45686 acceleration, gyro, temperature; id 1 LSM6DSK320X low-g acceleration, gyro, temperature;
  id 2 high-g acceleration (+-320 g); id 3 BMM350 magnetic field, temperature. A field the sensor does not
  measure is NaN with its `fields_updated` bit clear. `time_usec` is the sample's time since boot. Axes
  are the board frame until the mounting is known. (SCALED_IMU was rejected: it stops at 32.7 g.)
- SCALED_PRESSURE (29) for the BMP581, 10 Hz (50 Hz). GPS_RAW_INT (24), 5 Hz (10 Hz). STATUSTEXT (253).
- Load: about 15.6 kB/s at the defaults, about 62 kB/s at the fastest settings.
- Ground station consequence: plot series are keyed by (message, instance), at least `HIGHRES_IMU.id`,
  and NaN points are skipped.

Rates and commands
- Sent unasked: HEARTBEAT, the default streams and STATUSTEXT, from the moment the link task lets the
  vehicle transmit: from boot on a link without an open (UART, radio), on USB from the gate in section 3.
- MAV_CMD_SET_MESSAGE_INTERVAL (511) as COMMAND_LONG: -1 off, 0 default; faster than the stream's minimum,
  slower than 10 s, an id not in the table, HEARTBEAT or param3 != 0 get DENIED. Rates live in RAM and
  return to defaults when the link goes down. MAV_CMD_REQUEST_MESSAGE (512): ACK, then one copy.
- Every other command gets UNSUPPORTED. The device layer never arms, never changes a mode and does nothing
  when the ground station is lost.

Both ends build the messages from byte-identical MAVLink XML with generator 0.21.0 and extension fields
on: the device and the ground station both through `marv-dialect` (see section 4; the station also keeps
`mavlink = "=0.21.0"` with `mav2-message-extensions` for its framing helpers, and its simulated vehicle
is written on that crate's `common`). `marv-dialect` holds `common` and, as a second module, `marv`
(`common` plus `marv.xml`: so far the link test message `MARV_TEST`, id 42420, CRC_EXTRA 100). The ground
station's `dialect` alias is `marv_dialect::marv` (with the crate's `serde` feature, which the Messages tab
reads fields through), so it knows `MARV_TEST`: it checks its checksum and shows it with its fields; see
`docs/extending.md`.

Not now, each with its trigger: SYS_STATUS and battery (battery data exists); ATTITUDE (an estimator);
TIMESYNC (timestamps must line up with the computer); parameter metadata (descriptions or ranges wanted);
parameter persistence (the first parameter that must survive a reboot; firmware scope); signing (a radio
link); more parameter types (a value that fits neither INT32 nor REAL32); per-instance rates.

## 3. The USB crate (`usb-cdc`, own repo: https://github.com/Lmx154/usb-cdc)

A thin crate on embassy-usb 0.6 `CdcAcmClass`: a byte pipe through `embedded-io-async` 0.7 plus a link
state. No queue, no task, no MAVLink. It exists because the bare class has three traps on this HAL:
1. `Sender::write` can end a transfer on a full packet and `flush` does nothing (`EU/src/class/cdc_acm.rs:90-93, 451-463`): the computer may hold the data back.
2. A zero-length packet from the computer comes out of `BufferedReceiver::read` as `Ok(0)` (`cdc_acm.rs:597-603`), which `embedded-io-async` defines as end of file.
3. On embassy-rp 0.10.0 endpoint reads and writes never return `Disabled` (`ERP/src/usb.rs:577-586, 621-630`) and a bus reset does not fail pending futures (`usb.rs:393-412`): a write in flight when the cable is pulled hangs until the next enumeration.

Required behaviour
- "Up" = configured and not suspended, from the crate's own `Handler`. Down on reset, deconfigure, suspend, disable.
- While down, `read` and `write` return `Err(NotConnected)` at once.
- While up with a computer that is not reading, `write` waits; only the calling task waits; a dropped write has sent nothing.
- No IN packet is longer than 63 bytes (max packet size - 1), so no transfer ever needs a zero-length packet.
- Zero-length packets from the computer are skipped and counted; `read` never returns `Ok(0)` for a non-empty buffer.
- A partly read packet is thrown away when the link goes down. DTR is reported; the crate never gates data on it.
- The firmware builds the `embassy_usb::Builder` with its own identity and runs `UsbDevice::run()` in a task it never cancels; the crate adds the CDC function and one handler.
- **The start gate (architect, 2026-11-12, after the diagnosis on the board).** The firmware's USB link
  task writes nothing after each link-up until DTR has held true for 100 ms (checked at most 5 ms apart)
  or a ground-station HEARTBEAT has arrived, whichever comes first; meanwhile it reads and passes bytes
  to `Node::receive`. Then it runs the node as before, also into a closed port, until the link goes down.
  It never falls silent because the ground station is gone. The gate lives in the link task only, built
  from public pieces (`Link::dtr()`, `Node::receive`, `Node::gcs_present`); neither crate changes.
  Reason, measured: after every enumeration the computer's tty starts in cooked mode (canonical, echo,
  signal characters), and cdc_acm starts reading inside `open()`, about 1 ms before the opening program
  can set raw mode. A board that is already writing has its first bytes (about 117) edited, partly
  dropped and echoed back to it: the ground station saw 32 to 95 discarded bytes and sometimes one
  checksum error in its first read, and the start-up text (second frame; its length byte 0x15 is the
  kill-line character, and the HEARTBEAT before it ends in 0x03, the interrupt character) never arrived.
  Raw mode lands 0.31 to 0.35 ms after DTR; a ground station's first HEARTBEAT is sent after its `open()`
  returned. Nothing on the computer removes the window without a kernel change. A later open within the
  same enumeration is already raw, so there is no new gate then; such a session may begin with the tail
  of one frame, and the ground station resynchronises. Rejected: DTR alone (silent for a host that never
  raises it), a received frame alone (silent for a program that only listens), a timeout fallback (brings
  the damage back when the opener is late), a gate inside the crates.

```rust
pub struct State<'d>;
#[non_exhaustive] pub struct Config { pub max_packet_size: u16 }           // 64
pub struct UsbCdc<'d, D: embassy_usb::driver::Driver<'d>>;
impl<'d, D: Driver<'d>> UsbCdc<'d, D> {
    pub fn new(builder: &mut embassy_usb::Builder<'d, D>, state: &'d mut State<'d>, config: Config) -> Self;
    pub fn split(self) -> (Tx<'d, D>, Rx<'d, D>, Link<'d>);                 // Tx: Write, Rx: Read
}
impl Link<'_> { pub fn is_up(&self) -> bool; pub fn dtr(&self) -> bool; pub async fn wait_up(&self); pub async fn wait_down(&self); pub fn stats(&self) -> Stats; }
#[non_exhaustive] pub enum Error { NotConnected }
```

Dependencies: embassy-usb 0.6 (`default-features = false`), embedded-io-async 0.7, optional defmt. It works
with any HAL that implements embassy-usb-driver and with any executor; it does not work on the
`usb-device` stack and has no blocking front-end (a USB device must be serviced all the time). Those are
named exceptions to the drivers' CONTRACT.md (A1, A5, A11).

Two more named exceptions, recorded by the lead after the code review of 2026-11-12:
- A5 (Send): none of `Tx`, `Rx`, `Link` is `Send`, because embassy-usb's own control state is not `Sync`.
  They may be used from different executors on the same core (the shared state is lock-protected), never
  across cores.
- A8 (interrupt masking): the link state sits behind embassy-sync's critical-section mutex, because the
  `Handler` callbacks run in the `UsbDevice::run()` task, which a firmware may put on another executor
  than the link task. The section holds a few compares, counter updates and at most six wakes (an
  embassy wake only enqueues). Its duration is to be measured on the board. embassy-usb's own CDC state
  does the same.

Acceptance
- Host tests against a fake driver: every IN packet at most 63 bytes for write lengths 0-4096 and the bytes unchanged; zero-length packets never give `Ok(0)`; `NotConnected` at once while down and for a pending call on reset, deconfigure or suspend; cancel at every await; partial packet dropped on link down.
- `cargo tree -e normal` shows no mavlink, embassy-rp or embassy-executor.
- On the board with a host tool on `serialport`: enumerates within 5 s; 1 MiB echo in random write sizes (forcing 63, 64, 65, 128, 4096) with 0 mismatches; device-to-host throughput at least 250 kB/s; bursts of exactly 64, 128 and 4096 bytes arrive complete within 50 ms, with a positive control (a test build that sends full 64-byte packets must fail this check); port open but not read for 10 s, then closed for 10 s: a 1 kHz tick task misses 0 ticks; 20 close and reopen cycles with 0 errors; 10 soft detaches with a write pending give `NotConnected` within 50 ms and the link comes back; one physical unplug by the owner.

## 4. The MAVLink device layer (`mavlink-device`, own repo: https://github.com/Lmx154/mavlink-device)

A sans-IO core (`Node`: bytes and time in, frames out) and one thin `async run` over any
`embedded_io_async::{Read, Write}`. No USB, transport or executor dependency.

- Codec (amended 2026-11-12 after the lead's build experiments; architect's second decision): the message
  types come from `marv-dialect`, a small crate in its own repo (https://github.com/Lmx154/marv-dialect)
  that runs `mavlink-bindgen` =0.21.0 as a BUILD-dependency (with `mav2-message-extensions`) over
  `common.xml`, `standard.xml` and `minimal.xml` copied byte-identical from the `mavlink` 0.21.0 package,
  and depends at run time on `mavlink-core` =0.21.0 (`embedded`; `std` as an additive feature),
  num-traits, num-derive and bitflags. Why not the stock `mavlink` crate: with `mav2-message-extensions`
  and no std it does not build, because that feature switches on the optional normal dependency
  `mavlink-bindgen` (regex, quick-xml; `mavlink-0.21.0/Cargo.toml:129, :251`); without the feature
  `HIGHRES_IMU` has no `id`, and COMMAND_ACK, GPS_RAW_INT, SCALED_PRESSURE and AUTOPILOT_VERSION lose
  their extension fields too. The XML is MIT-licensed (https://mavlink.io/en/#license): the repo carries
  `definitions/LICENSE-MAVLINK`.
- The device layer uses the same `mavlink-core` items as planned, at the message-struct level only
  (`serialize_message_data`, `D::deser`, `calculate_crc` with `D::EXTRA_CRC`;
  `MC/src/lib.rs:211-226, 459, 1155`), with its own deframer. Never the `MavMessage` enum's `ser` /
  `parse` or the `write_*` helpers: they are the source of the roughly 70 kB flash warning
  (`MC/src/lib.rs:1412-1414`). Rejected: the stock crate with extension bytes appended by hand (trailing
  zeros are stripped from the base payload, so the byte lands at the wrong offset); a patched or forked
  `mavlink`; mavio (a second code generator); a hand-written codec.
- Lock-step check, in `marv-dialect` (`cargo test --test same_wire`, against the std `mavlink` crate as a
  dev-dependency): the sha256 of each vendored file; for every message id the name, EXTRA_CRC, ENCODED_LEN
  and the bytes of a NON-ZERO payload after a round trip (an all-zero payload proves nothing: MAVLink 2
  strips trailing zeros and the CRC seed leaves extension fields out); and a positive control showing the
  comparison fails against a `mavlink` built without extension fields.
- The layer owns: framing, sequence numbers, CRC; HEARTBEAT and presence of the ground station; the
  parameter protocol; commands 511 and 512 and UNSUPPORTED for the rest; the stream scheduler; a
  STATUSTEXT queue of 4.
- The firmware passes in: parameter storage, validation and persistence (`ParamStore`); sensor values,
  sampled when due and never queued (`Telemetry`); the stream table; time; the task; the byte stream.

```rust
pub trait ParamStore { fn count(&self) -> u16; fn name(&self, i: u16) -> Option<ParamName>; fn get(&self, i: u16) -> Option<ParamValue>;
    fn find(&self, n: &ParamName) -> Option<u16>; fn set(&mut self, i: u16, v: ParamValue) -> Result<ParamValue, ParamError>; }
pub trait Telemetry { fn sample(&mut self, message_id: u32, now_us: u64, out: &mut Outgoing<'_>) -> bool; }
impl<'a> Node<'a> {
    pub fn new(config: Config, streams: &'a mut [StreamSlot]) -> Self;
    pub fn receive(&mut self, bytes: &[u8], now_us: u64, params: &mut impl ParamStore);
    pub fn poll_transmit(&mut self, now_us: u64, params: &impl ParamStore, tel: &mut impl Telemetry, frame: &mut [u8; 280]) -> Option<usize>;
    pub fn next_deadline_us(&self) -> u64;
    pub fn on_link_down(&mut self);
    pub async fn run<R: Read, W: Write, D: DelayNs>(&mut self, rx: &mut R, tx: &mut W, delay: &mut D, now_us: impl FnMut() -> u64,
        params: &mut impl ParamStore, tel: &mut impl Telemetry) -> RunError<R::Error, W::Error>;
}
```

Dependencies: `marv-dialect`, embedded-io-async 0.7, embedded-hal-async 1.0, optional defmt; the std
`mavlink` crate only as a dev-dependency. For the target, `cargo tree -e normal | grep -cE 'mavlink-bindgen|quick-xml|regex|mavlink v'`
must be 0 (the failed build turned into a check).

Acceptance: host tests (every emitted frame decodes with the std `mavlink` crate; random split points;
CRC errors counted; a full parameter list through a fake that drops 20 % of frames; set echo, refusal,
silence for an unknown name; every command case gets its result; HEARTBEAT never delayed by more than one
frame; defaults return after link down); `cargo tree -e normal | grep -cE 'usb|serialport|embassy'` is 0;
on the board through the ground station: HEARTBEAT at 1 Hz +-5 % within 2 s, 10 full lists, INT32 and
REAL32 sets echoed, rates within 5 %, a `.tlog` written, and the ELF has no `MavMessage` ser / parse symbol.

## 5. Invariants

1. A lower layer never names a higher one (`cargo tree` checks at both ends).
2. No USB IN packet of 64 bytes or more; `read` never returns `Ok(0)` for a non-empty buffer; no `usb-cdc` call waits while the link is down.
3. On the flight controller only the link task awaits a `usb-cdc` or `mavlink-device` future. Sensor and control tasks hand values over through non-blocking latest-value cells.
4. Both ends generate from byte-identical MAVLink XML with generator 0.21.0 and extension fields on (the
   lock-step test in `marv-dialect`; `cargo tree -e features -i mavlink` in the ground station shows
   v0.21.0 with `mav2-message-extensions`). A version bump is one change across both, gated by that test.
5. Only INT32 and REAL32 parameters; the set is fixed per boot.
6. The device layer never arms, never changes a mode, has no failsafe.
7. Default streams at their fastest stay within 25 % of the measured USB throughput.
8. On USB nothing is written between link-up and the start gate; after an enumeration the first byte a
   reader gets is the 0xFD of a whole frame. This is a rule of the USB link task, not of the wire: a
   ground station must still resynchronise at the start of any session (a USB-serial adapter on the
   computer has the same cooked window and the vehicle cannot see it).

## 6. Not known yet, and what settles it

- Whether this computer holds back a transfer that ends on a full packet: the positive control on the board.
- Settled 2026-11-12: ModemManager did not open the port on this computer (the board recorded no control-line activity before the ground station's own open). Port permissions were no problem for a user in `dialout`.
- Settled 2026-11-12: the stock `mavlink` crate does not build for thumbv8m with extension fields (see section 4). Still open: whether the `MavMessage` enum's `ser` / `parse` stay out of a linked firmware binary: the first firmware link and `nm`.
- Real throughput with 63-byte packets: measured on the board; it sets invariant 7.
- Settled 2026-11-12: a cable pull with the board on battery power gives a suspend 3.2 ms after the last good write, and the pending write ends 38 us later (one physical unplug; `usb-cdc` README, check F).

## 7. The owner's to choose

- USB vendor and product id. The bring-up test values (0xc0de:0xcafe) are for the bench only. Before a
  board leaves the bench: pid.codes (0x1209; needs open hardware and software) or a Raspberry Pi
  allocation (0x2E8A; an application). Strings by default: manufacturer "MARV", product "MARV V2", serial
  number from the chip's unique id.
- The vehicle type in HEARTBEAT (GENERIC until the airframe is decided).
- Whether Mission Planner or QGroundControl must also be able to connect to the board (default: no).
