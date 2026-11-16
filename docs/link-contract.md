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
  tries), then report it missing. Read: 500 ms x 3. Set: a PARAM_VALUE for the written name that arrives
  after the PARAM_SET went out ends the write at once: equal in raw bytes it confirms, any other value is the
  vehicle's refusal (the value in effect is shown, nothing is sent again); caveat: a stale PARAM_VALUE of the old value
  that was already in flight and arrives after the send is read as a refusal too, the accepting echo that follows clears
  it (the station's views heal), and the command line waits 300 ms for exactly this before it reports a refusal. Only when no PARAM_VALUE for that
  name arrives at all (a lost frame) it sends again: 3 sends, 500 ms apart, then unanswered 500 ms after the
  third. While a list download is under way its PARAM_VALUE frames are not answers: a differing value then
  only notes the value in effect, an equal one still confirms, and the 3 sends / 500 ms rule decides (refused
  if some differing value was seen, otherwise unanswered). A change of param_count means fetch everything again.
- Texts: on a refusal (read only, out of range, a setting the device cannot take) the vehicle also sends one
  STATUSTEXT with the reason, "NAME: reason" (WARNING in the simulator), right after the PARAM_VALUE. An accepted
  write sends none. A text belongs to the session, not to the command: the command line prints every text that
  arrives, also one that came before its request.
- Amendments by the lead after review (2026-11-12). (1) Withdrawn 2026-11-15: a different echo used to be
  retried (a stale copy of the old value in flight would read as a refusal); the vehicle answers every PARAM_SET
  with exactly one PARAM_VALUE, so a differing one after the send is the refusal. (2) A write waits
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
(`common` plus `marv.xml`: so far the link test message `MARV_TEST`, id 42420, CRC_EXTRA 100, and the device
diagnostics message `MARV_DEVICE_STATUS`, section 8). The ground
station's `dialect` alias is `marv_dialect::marv` (with the crate's `serde` feature, which the Messages tab
reads fields through), so it knows `MARV_TEST`: it checks its checksum and shows it with its fields; see
`docs/extending.md`.

Log download (LOG_REQUEST_LIST, LOG_ENTRY, LOG_REQUEST_DATA, LOG_DATA, LOG_REQUEST_END) is on the vehicle's USB
link only; the contract for it is section 11.

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

## 8. Device diagnostics: `MARV_DEVICE_STATUS`

Written before the code (the design of 2026-11-13, section 8 of `decision-foundation-20261113.md`). The
message is defined in `marv-dialect` (its README, section `MARV_DEVICE_STATUS`): id 42421, CRC_EXTRA 231,
a fixed payload of 30 bytes. The station reads it through the dialect only: it names no field or id of
the message outside the Devices view and the one instance rule of the message store.

What the station expects from the vehicle
- One message per device, carrying that device's `device_id` (`MARV_DEVICE`: 1 IMU1, 2 IMU2, 3 BARO,
  4 MAG, 5 GNSS, 6 BATT, 7 RC, 9 SD, 10 PSRAM, 11 LED, 12 ESC). The firmware sends it unasked.
- Rate: on USB every device at least once a second; on the radio one device per second in turn, so with
  N devices each device is refreshed every N seconds. The station relies on no exact rate.
- Sender: the vehicle (system 1, component 1). The station keeps rows per sender, as for any message.
- Every counter (`samples`, `errors`, `timeouts`, `recoveries`, `attempts`, `overflows`,
  `config_restored`) counts since boot and wraps. `time_boot_ms` is the vehicle's own clock.

What the station does with it
- The message store keeps the latest message per (sender, `device_id`): one row per device, each with the
  time it was received, its count and its rate. A second message of the same device replaces its row; a
  device is never overwritten by another device. Every other message keeps one row per instance as before
  (`HIGHRES_IMU` by its `id`, any other message one row).
- Devices view: one row per device seen, in `device_id` order (the order of the table above), read only.
  - State, last error and flags are shown as words (`OK`, `DEGRADED`, ...; flags `HIL` and `self-test`,
    `-` when none).
  - `last_sample_age_ms` 65535 means no sample yet and is shown as `never`.
  - Samples per second is computed by the station from the two latest rows of the same device:
    (`samples` now - `samples` before) / (`time_boot_ms` now - `time_boot_ms` before) x 1000, from the
    vehicle's own clock, so a late or bunched arrival does not disturb it. It is blank until two rows
    exist, and blank once when `samples` or `time_boot_ms` went backwards (the vehicle restarted, or a
    counter wrapped, which at 4.29 x 10^9 samples is not expected in a session). It is also blank when
    the two rows carry the same `time_boot_ms`.
  - A row is stale when it was received more than 15 s ago (15.000 s exactly is not stale); the
    threshold is one constant in the view, the same on every link, and it is above the 11 s cycle of
    11 devices at one row per second on the radio. A stale row is shown as stale and kept, never removed;
    the Received column says how old the row really is.
- An unknown value in `device_id` (0, 8 or above 12), `state` (above 5) or `last_error` (above 6) makes
  the frame undecodable with this dialect version: the generated enums refuse a value that is not an
  entry. The checksum is right (the id is known), so the frame is counted in "Undecodable frames" of the
  Link view (`RxStats.undecodable`), not as a checksum error. The Messages view lists it as
  `MARV_DEVICE_STATUS[0]` with no fields (the instance cannot be read), and the Devices view does not
  show it: it shows only rows that decoded. A vehicle with a newer dialect than the station therefore
  shows its new devices or states as missing, and the Link view counts them.
- A frame of id 42421 with another CRC_EXTRA than 231 is a checksum failure ("Checksum errors" of the
  Link view); it is not delivered.

The Devices view also has the restart and self-test buttons and the calibration buttons of section 10; the
table itself is read only. The access gate of section 9 is built.

The simulator (`gcs-sim`) sends `MARV_DEVICE_STATUS` for IMU1, IMU2, BARO, MAG, GNSS, BATT and RC, each
once per second, with advancing counters (`samples` at the device's nominal rate), state OK, error NONE and
no flags. It builds the frame from this section, not from `marv-dialect`.

## 9. Access: Operate and Configure

The owner's rule, verbatim: "i don't want the firmware to enforce it. things should be enforced in the GCS
since that is where we expose the firmware's endpoints for settings and stuff so they can be changed".
The vehicle does not enforce anything here: it answers every ground station the same way on every link,
and nothing in the firmware, the wire or this contract changes. The gate is the ground station's alone.

Roles. A session has one `Access`, fixed when it starts (`SessionConfig.access` in `gcs-mavlink`):
- `Operate`: the station is a telemetry display and controller. It reads and it concerns itself.
- `Configure`: the station is also the flight controller's configurator.

Which link gives which role (one function in `gcs-link`, `gcs_link::access_of`, used by the window and by the
command line, exhaustive over `gcs_transport::Endpoint`; the transport knows only bytes and does not hold the
mapping, and `gcs-mavlink` does not know the transport):

| Link | Access | Why |
|---|---|---|
| Serial | Configure | a cable to the board: the bench |
| TCP | Configure | today TCP is the simulator; the default, the owner was asked |
| UDP | Operate | a radio or a network: the field |
| Log-file replay | Operate | a recording has no vehicle to configure |

A session built without an explicit access is `Operate` (`SessionConfig::default()`): the gate fails closed,
so a session that someone forgot to configure cannot write. Whoever needs to write says `Configure`.

Enforcement. `Session::send` is the one choke point, for the window and for the command line (`marv-cli`). Every
`Command` has a class (`Command::access`, one exhaustive `match` with no wildcard arm, so a new variant
cannot be added without a class). A configure-class command on an Operate session is refused: nothing is
written to the link (0 bytes) and the refusal is recorded in `VehicleState.link.refused` (what was asked,
how many times), from where a view or a command line shows "refused: this link is operate-only". Configure
sessions pass every command.

Classes.
- Configure-class: every parameter write (`Command::SetParam`); and sending a MAVLink command to the vehicle
  (`Command::SendCommand`: ALL of them, an unknown `MAV_CMD` id counts as Configure, so storage actions,
  calibration, device actions (restart, self-test) and reboot are all Configure; section 10); and the three
  commands of log download (`Command::ListLogs`, `Command::DownloadLog`, `Command::CancelLogDownload`; section
  11), because the vehicle offers log download on its USB link only and the station must not ask for it on a
  radio or a network link, where the vehicle does not answer.
- Operate-class: everything that only reads or only concerns the station: the heartbeat (a service, not a
  command), a parameter list request (`Command::RefreshParams`), a single parameter read (reserved: no such `Command` exists yet), log recording
  (`Command::StartLog`, `Command::StopLog`), link management (connect, disconnect, replay).

Services that run by themselves are not commands and are not gated. They only read: the capability
request (`MAV_CMD_REQUEST_MESSAGE` for `AUTOPILOT_VERSION`) at connect, the repair reads of a list that is
being downloaded, and the heartbeat.

Parameters on an Operate session: the list is not downloaded at connect; the Parameters view says so, and
a Refresh the user asks for is allowed (reads are operate-class). On a Configure session the list is
downloaded at connect as before. An Operate session
still shows a `PARAM_VALUE` it did not ask for, so a replayed log fills the table as before: the entry
is stored and the status follows the count, but no list is requested and no missing entry is read by index.

Views. Every configure control is SHOWN DISABLED, with the reason next to it or as hover text ("Configure
needs a serial or TCP link; this link is operate-only"), never hidden. The Link view and the connection bar
say which role the session has.

## 10. Commands

Written before the code (the design of 2026-11-13, `decision-foundation-20261113.md`, sections 5 and 8). The
station sends a MAVLink command only when the user presses a button; it never sends one by itself (the
capability request of section 9 is a read, not a command of this section). Every `Command::SendCommand`
is Configure-class: on an Operate session it writes 0 bytes and is recorded in `link.refused`.

What the vehicle answers (binding for the station and for the simulator)
- A `COMMAND_LONG` it does not know: `COMMAND_ACK` UNSUPPORTED. Refused by its parameters: DENIED (a
  STATUSTEXT gives the reason). Armed, or another action running: TEMPORARILY_REJECTED.
- An admitted action: `COMMAND_ACK` IN_PROGRESS with `progress` 0 at once, then IN_PROGRESS with a
  percentage about twice a second, then ONE final `COMMAND_ACK` (ACCEPTED, FAILED or CANCELLED). Every ACK
  is addressed to the sender: `target_system` and `target_component` are the station's ids.
- The same command sent again while it runs (a retransmission) gets IN_PROGRESS again, never a rejection.
- Longest durations on the vehicle: calibration 15 s (the compass calibration 40 s), storage 3 s, device action 5 s, reboot 3 s.

The commands of the first build (`COMMAND_LONG`, `confirmation` as below, parameters not named are 0)
| Command | Parameters |
|---|---|
| 241 `MAV_CMD_PREFLIGHT_CALIBRATION` | param1 = 1: gyro; param5 = 4: accelerometer, one position, level; param2 = 1 and all others 0: compass (the user rotates the vehicle through all orientations for 30 s; progress about twice a second; FAILED when an axis saw too little movement); all seven 0: cancel the running calibration |
| 245 `MAV_CMD_PREFLIGHT_STORAGE` | param1: 0 reload from flash, 1 save, 2 all parameters to defaults, 3 calibration parameters to defaults |
| 246 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` | param1 = 1: reboot |
| 42430 `MAV_CMD_MARV_DEVICE` | param1 = device id (`MARV_DEVICE`), param2 = 1 restart, 2 self-test; restart for IMU1, IMU2, BARO, MAG, GNSS, BATT, self-test for IMU1, IMU2, MAG |

What the station does (`gcs-mavlink`, service `commands`; state in `VehicleState.command`)
- One outstanding command at a time. A second `SendCommand` while one is outstanding is refused: nothing is
  sent and the outstanding command's record counts it (`refused`: the refused command number and how many);
  the outstanding command goes on. The one exception is the cancel below. A `SendCommand` while no vehicle
  is connected is refused too (the record's state is `Refused` with the reason).
- Send: `COMMAND_LONG` to the vehicle's system and component, `confirmation` 0. Without any ACK for that
  command it is sent again after 1.5 s with `confirmation` 1, and after another 1.5 s with 2: three sends
  in all, at 0, 1.5 s and 3.0 s. 1.5 s after the third send, with still no ACK, the result is "no answer"
  (at 4.5 s) and nothing more is sent.
- An ACK counts when its `command` is the outstanding command's number and it is addressed to the station
  (`target_system` 0 or the station's id, `target_component` 0 or the station's), from the vehicle's
  system. Any other ACK is ignored: it changes nothing.
- IN_PROGRESS stops the retransmission. The state then follows `progress` (0 to 100; 255 is unknown). If no
  further ACK arrives for 20 s (counted from the last one) the result is "no result (the vehicle stopped
  reporting)". An IN_PROGRESS never ends a command.
- Any other result ends the command with that result: ACCEPTED, TEMPORARILY_REJECTED, DENIED, UNSUPPORTED,
  FAILED, CANCELLED, COMMAND_LONG_ONLY, COMMAND_INT_ONLY, COMMAND_UNSUPPORTED_MAV_FRAME, NOT_IN_CONTROL;
  each has its own text. An ACK after the end is ignored. The last command stays in `VehicleState.command`
  (number, name, parameters, state, number of sends, when it started and ended) until the next one.

The cancel (the one exception to "one outstanding command")
- While a calibration (241 with a non-zero parameter) is outstanding, a `SendCommand` of 241 with all seven
  parameters 0 is sent although the calibration is outstanding. It has its own send count and timers (the
  same 1.5 s, at most 3 sends) and its own state in the calibration's record (`cancel`: waiting, answered
  with a text, no answer). Only one cancel is in play at a time; another one meanwhile is refused as any
  other command.
- Both are command 241, so the number cannot tell their ACKs apart. The vehicle answers the cancel at once
  with ACCEPTED (DENIED when no calibration runs) and ends a cancelled calibration with a final CANCELLED.
  The station therefore reads them in order, by this rule:
  1. From the moment a cancel is sent, the calibration command is never sent again, whatever was or was not
     acknowledged before (the silence rule of 20 s then runs from the send of the cancel at the latest).
     Nothing is sent after that except the cancel's own sends (at most 3).
  2. While a cancel is in play, ACKs for 241 are read in order. IN_PROGRESS is the calibration's progress.
     CANCELLED means the calibration was cancelled: the record ends "cancelled". The first other result R1
     is held as "the answer so far" (it is also the cancel's state, `answered`), and the cancel is not sent
     again. A second other result R2 decides: if R2 is DENIED, the calibration had already finished by
     itself with R1 (the vehicle had nothing to cancel), and the record ends with R1's text and the note
     "finished before the cancel" (for example "accepted (finished before the cancel)"); any other R2 ends
     the record with R2's text.
  3. If only R1 arrived: 3 s after it, with no CANCELLED and no second result, the record ends. R1 ACCEPTED:
     "cancel accepted, no final result"; R1 DENIED: "nothing to cancel" (the calibration had ended and its
     result was lost or came before the cancel); any other R1: R1's text. Progress ACKs do not extend the 3 s.
  4. If the cancel gets no ACK at all after its 3 sends, its state is "no answer" and the calibration goes
     on being followed as before (its 20 s rule), still without being sent again; from then on its ACKs are
     read as without a cancel.
  5. In every case the record ends and the buttons come back. When the calibration ends, a cancel still
     waiting is dropped.

The simulator (`gcs-sim`) behaves as the table above: admitted commands get IN_PROGRESS 0 at once,
IN_PROGRESS with the percentage every 500 ms and ACCEPTED after 2 s (a calibration after 3 s, the compass
calibration after 4 s; a cancel ends it CANCELLED); one action at a time (another command: TEMPORARILY_REJECTED, the same again: IN_PROGRESS);
245 with param1 other than 0 to 4, 246 with param1 other than 1, 241 with other parameters (param1 = 1 and param2 = 1 together too), 42430 with a
device or action not listed: DENIED; any other command except 512 (`MAV_CMD_REQUEST_MESSAGE`, as before): UNSUPPORTED. Two switches of its
configuration, off by default: drop the first N `COMMAND_LONG`s that are not `MAV_CMD_REQUEST_MESSAGE`, and
never send the final ACK. The simulator reads and writes 42430 by hand, as it does `MARV_DEVICE_STATUS`.

Not in this feature: `COMMAND_INT`, a queue of commands, arming, flight commands, automatic commands.

## 11. Log download

Decision of 2026-11-16 (the lead, within the architect's Q6 of `decision-sd-logging-20261115.md`). It binds
both ends: the firmware serves it, the station (`gcs-mavlink` service `logs`, `marv-cli logs list` and
`logs get`) and the simulator (`gcs-sim`) follow it. It is ArduPilot's MAVLink log download, with the
messages of `common` unchanged.

Messages: LOG_REQUEST_LIST 117, LOG_ENTRY 118, LOG_REQUEST_DATA 119, LOG_DATA 120, LOG_REQUEST_END 122.
LOG_ERASE 121 is not served. The three requests carry `target_system` and `target_component`: the vehicle's
ids (1 and 1), as the station learned them from its HEARTBEAT.

Where. On the vehicle's USB link only. The radio's node does not list these ids, so a request there is
dropped. The station treats log download as Configure-class (section 9): it asks for it on a serial or a TCP
link and refuses it on UDP with its usual message and exit status 4, with nothing written. The vehicle does
not check anything: the station enforces, as everywhere else.

Listing. The station sends LOG_REQUEST_LIST with `start` 0 and `end` 0xFFFF (all). The vehicle answers one
LOG_ENTRY per existing log whose number lies in `start..=end`, in ascending order of number:
- `id` is the log's NUMBER on the card, 1 to 500. Numbers have gaps (a deleted log leaves one); the station
  never assumes they are contiguous.
- `num_logs` is how many logs exist on the card (not how many are in the range); `last_log_num` is the number
  of the log being written, or the highest present when none is open.
- `time_utc` is 0 (unavailable: the card has no clock; the wall-clock time is inside the log, in its GPS
  records).
- `size` is in bytes. For the log being written it is the size as of its last flush: it grows, and is at most
  about 1 s behind.
- No logs, or no card: ONE LOG_ENTRY with `id` 0, `num_logs` 0, `last_log_num` 0, `size` 0. The station reads
  it as an empty list, not as an error.
- A range with no log in it, on a card that has logs: nothing is sent (the station asks only for all).

Download. The station sends LOG_REQUEST_DATA (`id`, `ofs`, `count`). The vehicle answers with LOG_DATA (`id`,
`ofs`, `count` at most 90, `data`) in ascending offset order for the bytes `ofs .. ofs + count`, clamped to the
log's size at the time of the request; a `count` of 0xFFFFFFFF, or one that goes past the end, means "to the
end".
- One transfer at a time: a LOG_REQUEST_DATA for an existing log replaces the running transfer.
- `ofs` at or past the end of an existing log: ONE LOG_DATA with `count` 0 (ArduPilot's end marker). The
  station never waits for it and does not count it as the end of a download.
- A log that does not exist: NO LOG_DATA at all, not even a marker. For the station this is the same as a
  silent vehicle, and it ends the same way (below).
- LOG_REQUEST_END stops the running transfer. A transfer also stops when the station's HEARTBEAT has not been
  seen for 3 s (ArduPilot's rule), or when the link goes down. The station sends its HEARTBEAT at 1 Hz
  (section 2).
- MAVLink 2 cuts the trailing zero bytes of a payload: the station reads `count`, never the length of the
  payload, and treats what was cut as zeros.

Pace. The vehicle sends LOG_DATA through a stream slot of its node: at most 4 frames per 5 ms, about 72 kB/s of
log data (a log of 161 MB takes about 37 minutes at best). The station paces itself:
- One outstanding request. It asks for a window of 512 frames (46 080 bytes) from the first missing offset, or
  for what is left of the size when that is less. When every byte of the window is in, it asks for the next.
- Bytes are taken as they come: a frame that repeats what is in, or that is out of order inside the window, is
  tolerated (held until its turn); a frame of another log id, from another system, with count 0, or beyond
  the window is ignored.
- Progress is the first missing offset moving. When it has not moved for 1 s after the last request or move,
  the station asks again from the first missing offset (a "retry"). After 5 retries of the same first missing
  offset, one second after the fifth, it gives up (6 s of silence from the first request): the error "no data
  from the vehicle at byte N after 5 new requests", and it sends LOG_REQUEST_END. A move of the first missing
  offset starts the count again.
- The download is complete when every byte up to the `size` of the LOG_ENTRY it started from has arrived,
  never by waiting for a zero-count LOG_DATA. A log that grew after the listing is not a failure: the station
  ends at the size it started with. A log of size 0 is complete at once and nothing is requested.
- LOG_REQUEST_END is sent when a download is given up or cancelled, and when the file cannot be written.
- A listing waits 1 s for a new LOG_ENTRY; it then asks again for all logs, 4 requests in all, the entries
  already in are kept; it ends when `num_logs` different entries are in, or fails ("only N of M log entries
  arrived", or no answer at all) one second after the fourth request.

Logging continues during a download. The log being written can be listed and downloaded up to its flushed
size. Not offered: erase; a download while a log clean-up or a card recovery is running (the request is simply
not answered then; the station's retries cover a short one). No new parameter, no new MARV message, no dialect
change.

What the station does (`gcs-mavlink`, service `logs`; state in `VehicleState.logs`)
- `Command::ListLogs` starts a listing; the result is `logs.list` (`Done` with the logs and `last_log_num`, or
  `Failed`). `Command::DownloadLog { number, size, sink }` starts the download of one log, `size` taken from a
  listing, to a writer the caller opened (`LogSink`): the bytes are handed to it in order as they come, so a
  log is never held in memory; progress is `logs.download.done` of `size`. `Command::CancelLogDownload` ends
  the running download. One listing or download at a time: a second request is refused and counted
  (`logs.refused`). A request with no vehicle connected fails at once, sending nothing.
- `marv-cli logs list` prints `NUMBER<TAB>SIZE` per log and then `last<TAB>N`; an empty card prints nothing
  but `marv-cli: no logs on the vehicle` on stderr; exit 0. `marv-cli logs get N [-o FILE] [--force]` lists,
  checks that N is there (exit 1 if not), opens the file (default `NNNNNNNN.BIN` in the current directory;
  an existing file is refused unless `--force`), downloads with a progress line on stderr at most twice a
  second, and prints `downloaded<TAB>FILE<TAB>SIZE bytes`. See the README for the exit statuses.

The simulator (`gcs-sim`) behaves as above, in virtual time: five logs with numbers 2, 3, 5, 8 and 11 (sizes
12 345, 0, 9 000, 60 001 and 450), answered through a slot of 4 frames per 5 ms, a request replacing the running
one, a zero-count LOG_DATA at or past the end, nothing for a missing log, the 3 s heartbeat rule. Its
configuration can drop chosen LOG_DATA frames by offset (`drop_log_data`) and replace the logs (`logs`).
It has been the only vehicle this was run against: the firmware side does not exist yet.
