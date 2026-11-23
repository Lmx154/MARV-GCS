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
  Amendment 2026-11-23 (approved, not built: stage 2): custom_mode carries the vehicle's effective mode, section 10;
  0 stays the value of every firmware before it.
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
  request, up to 3 times; 1 s of silence with entries missing, re-request the missing indices in rounds of 500 ms;
  the download is reported failed, with the number missing, after 8 consecutive rounds in which no missing index arrived
  (amendment 2026-11-17: was 3; at 20 % loss each way 3 rounds left about 1 download in 20 failed). Read: 500 ms x 3. Set: a PARAM_VALUE for the written name that arrives
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
- Naming (amendment 2026-11-17): a parameter's name begins with its group (the device or function) followed
  by `_`; a name without `_` is in the group "(other)". The station shows its Parameters tab as a tree: the
  groups `RC<n>` (n digits) and `RCMAP` under "RC controller", `ESC`, `MOT` and `DSHOT` under "ESC", all others
  under "Flight controller"; nothing but the group prefix decides, the link carries no other metadata.
- The RC block (amendment 2026-11-16): 44 parameters of device `MARV_DEVICE_RC` (7), appended after all others,
  so no existing parameter changes its index. All INT32, all applied live (no reboot, no save: the value is in
  effect at once and goes to flash only on command 245 with param1 = 1). Command 245 with param1 = 3 (MAVLink's
  `PARAM_RESET_SENSOR_DEFAULT`, sensor calibrations only) does NOT reset them; param1 = 2 (all parameters) does
  (amendment 2026-11-17, owner's decision). n is 1 to 8. Their meaning is section 12.

  | Name | Unit | Default | Range |
  |---|---|---|---|
  | `RCn_MIN` | us | 1100 | 800..2200 |
  | `RCn_TRIM` | us | 1500 | 800..2200 |
  | `RCn_MAX` | us | 1900 | 800..2200 |
  | `RCn_REVERSED` | 0/1 | 0 | 0..1 |
  | `RCn_DZ` | us | 0 | 0..200 |
  | `RCMAP_ROLL` | channel | 1 | 1..8 |
  | `RCMAP_PITCH` | channel | 2 | 1..8 |
  | `RCMAP_THROTTLE` | channel | 3 | 1..8 |
  | `RCMAP_YAW` | channel | 4 | 1..8 |

  Order: for n = 1 to 8 the five `RCn_*` as MIN, TRIM, MAX, REVERSED, DZ; then `RCMAP_ROLL`, `RCMAP_PITCH`,
  `RCMAP_THROTTLE`, `RCMAP_YAW`.
- The IMU parameters (amendment 2026-11-17; presets are a station feature, section 13): 6 parameters appended after
  RC, so the first 70 indices do not change and the count goes from 70 to 76. All INT32. Class Reboot: the chip is
  configured once, at boot, from these values; a write changes RAM, the vehicle answers with the INFO text
  "<NAME>: applies after reboot", and the value takes effect after save (command 245 with param1 = 1) and reboot.
  The defaults are the Flight preset's values (section 13). The vehicle has no preset selector and enforces no
  preset: every write is checked against the parameter's own range (and, for BARO and MAG, the driver's rules), as
  any parameter.

  | Index | Name | Type, unit | Default | Range | Class | Device |
  |---|---|---|---|---|---|---|
  | 70 | `IMU1_ACC_FS` | I32, g | 16 | 2, 4, 8, 16, 32 | Reboot | IMU1 |
  | 71 | `IMU1_GYR_FS` | I32, dps | 2000 | 125..4000 | Reboot | IMU1 |
  | 72 | `IMU1_ODR_HZ` | I32, Hz | 800 | 25..800 | Reboot | IMU1 |
  | 73 | `IMU2_ACC_FS` | I32, g | 16 | 2, 4, 8, 16 | Reboot | IMU2 |
  | 74 | `IMU2_GYR_FS` | I32, dps | 2000 | 250..4000 | Reboot | IMU2 |
  | 75 | `IMU2_ODR_HZ` | I32, Hz | 800 | 25..800 | Reboot | IMU2 |

  The BARO and MAG fields keep their indices, types and defaults (`BARO_ODR_HZ` 50, `BARO_OSR_P` 16, `BARO_OSR_T` 2,
  `BARO_IIR_P` 0, `BARO_IIR_T` 0, `MAG_ODR_HZ` 100 as a REAL32, `MAG_AVG` 2: the Balanced values). The fields a
  preset sets are these 13; the calibration offsets are not among them.
- The orientation parameters (amendment 2026-11-17; `_plan/decision-orientation-20261117.md`): 2 parameters appended
  after the IMU parameters, so the count goes from 76 to 78. Values are ArduPilot's rotation numbers 0..15 (0 none,
  1..7 yaw 45..315, 8 roll 180, 9..11 roll 180 + yaw 45..135, 12 pitch 180, 13..15 roll 180 + yaw 225..315).

  | Index | Name | Type | Default | Range | Class | Device | Meaning |
  |---|---|---|---|---|---|---|---|
  | 76 | `AHRS_ORIENTATION` | I32 | 0 | 0..15 | Reboot | 0 (system) | the board's rotation in the vehicle (both IMUs) |
  | 77 | `MAG_ORIENT` | I32 | 10 | 0..15 | Live | MAG | the external compass's rotation in the vehicle |
- The ESC parameter (amendment 2026-11-18, section 14): 1 parameter appended after the orientation ones, so the count
  goes from 78 to 79.

  | Index | Name | Type | Default | Range | Class | Device | Meaning |
  |---|---|---|---|---|---|---|---|
  | 78 | `MOT_POLES` | I32 | 14 | 2..64 | Live | ESC (12) | magnets per motor, for rpm |
  | 79 | `LED_BRIGHT` | I32 | 1 | 0..3 | Live | 0 (system) | status LED brightness: 0 off, 1 dim, 2 medium, 3 bright |

  The group `MOT` is under "ESC" in the station's tree. Saved only on command 245 param1 = 1, like every parameter.
- The mission source parameter (amendment proposed 2026-11-23, pending the owner's approval;
  `_plan/decision-mavlink-mode-20261123.md`, section 1 C; firmware `crates/fw-settings/src/flight.rs`, `GDN_SRC_AT`): 1
  parameter appended as the last, so the count goes from 119 to 120. Indices 93 to 118 (guidance and gain parameters)
  are not listed in this contract.

  | Index | Name | Type | Default | Range | Class | Device | Meaning |
  |---|---|---|---|---|---|---|---|
  | 119 | `GDN_SRC` | I32 | 0 | 0..1 | Live | 0 (system) | where the shadow guidance takes its targets: 0 the `GDN_*` parameters, 1 the committed mission of section 17 (up to 16 waypoints; the radius stays `GDN_RADIUS`) |
- Amendments by the lead after review (2026-11-12). (1) Withdrawn 2026-11-15: a different echo used to be
  retried (a stale copy of the old value in flight would read as a refusal); the vehicle answers every PARAM_SET
  with exactly one PARAM_VALUE, so a differing one after the send is the refusal. (2) A write waits
  until the encoding is known, so it is never sent in the wrong encoding. (3) A vehicle with no parameters
  answers the list request with one PARAM_VALUE whose param_count is 0; the ground station shows an empty,
  complete list. The device layer must do this: silence would read as "no answer".

Sensors (default rate, fastest allowed). Amendment 2026-11-16: the contract is what the real firmware
(`MARV-V2-firmware`) sends on its USB port, in the first list below. The second list is the demonstration
program of `mavlink-device`, kept for what still uses it.
What the real firmware sends on its USB port (the simulator's default streams are the same; the window's Sensors and Radio tabs read them):
- HIGHRES_IMU (105) `id` 0 only: IMU1, with the magnetometer in `xmag`/`ymag`/`zmag`, in gauss.
  SCALED_IMU2: IMU2, in mG and mrad/s. SCALED_PRESSURE. All three at 25 Hz.
- Amendment 2026-11-17: SCALED_IMU2's gyro fields are int16 mrad/s and saturate at +-32.767 rad/s (about 1877 dps).
  In IMU2's Flight and Max range presets (section 13) a faster rate shows as that limit. HIGHRES_IMU (floats) and the
  log carry the full range.
- Amendment 2026-11-17 (orientation): every IMU and magnetometer field is in the vehicle's body frame, FRD (x forward,
  y right, z down), after the firmware's rotation (the parameters above), with the calibration offsets removed. Level
  and at rest the accelerometer's z reads about -9.81 m/s^2 (SCALED_IMU2 about -1000 mG). Firmware before this
  amendment sent the chips' own axes.
- SYS_STATUS and GPS_RAW_INT at 1 Hz. RC_CHANNELS at 5 Hz.
- ESC_STATUS (291) at 10 Hz and ESC_INFO (290) at 1 Hz, USB only, for four ESCs (amendment 2026-11-18: section 14).
- MANUAL_CONTROL (69) at 5 Hz (fastest 50 Hz), USB only, sent only while a CRSF stick report is younger than
  1 s. Its fields and scaling are in section 12.

The demonstration program of `mavlink-device` (not the contract; amendment 2026-11-16: it is still what `tools/board_check.sh` and `tests/board.rs` use):
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
link only; the contract for it is section 11. Missions (MISSION_REQUEST_LIST, MISSION_COUNT, MISSION_CLEAR_ALL,
MISSION_ACK, MISSION_REQUEST_INT, MISSION_ITEM_INT) are on both of the vehicle's links, USB and radio; the contract for
them is section 17.

The station shows ATTITUDE in its HUD when the vehicle sends it; the real firmware does not send it yet (an
estimator is being designed), and the simulator sends it at 10 Hz as a test double.

Not now, each with its trigger:
TIMESYNC (timestamps must line up with the computer); parameter metadata on the link (the station holds the table of the firmware's parameters itself, `gcs_mavlink::param_info`: description, unit and allowed values by name; the link still carries none, and the trigger is a vehicle that must describe its own parameters);
signing (a radio
link); more parameter types (a value that fits neither INT32 nor REAL32); per-instance rates; RC channels 9 to 16 calibration (trigger: a use for them);
chip filters in the sensor presets (trigger: the control loop's filter design); IMU rates above 800 Hz (trigger: FIFO
reads); the vehicle sending PARAM_VALUE for values it changes itself, as ArduPilot does (trigger: more vehicle-side
changes); the running IMU configuration on the link (trigger: a pending reboot must be visible to a second station).

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
9. MANUAL_CONTROL from the vehicle is a report: the firmware acts on no inbound MANUAL_CONTROL or
   RC_CHANNELS_OVERRIDE, and nothing in the firmware reads the normalised sticks except its encoder
   (amendment 2026-11-16).
10. Only CRSF stick reports feed MANUAL_CONTROL (amendment 2026-11-16).

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
- The numbers of the mission service (section 17; `_plan/decision-mavlink-mode-20261123.md`, section 4), proposed
  2026-11-23 and built to these defaults until he decides: 16 waypoints (4, a number he has already approved, if he
  wants no new one); timeout 1500 ms with 5 retries; one reply slot per 40 ms on the radio link and per 5 ms on USB.
  Also awaiting him: the amendment of section 9 that makes the mission commands Operate-class. Decided 2026-11-23
  (the decision's sections 8 and 10): all approved, the section 9 amendment too; 16 waypoints plus a return item, and
  the lowest and highest height as parameters, 5 m and 50 m by default (section 17, Amendment A).

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

After a calibration ends with an accepted final ACK, the Devices view shows a bar until it is resolved, for
example "Gyro, accel calibrated: in use now, not saved to flash". The vehicle applies the offsets at once and
writes them to flash only on command 245 with param1 = 1, so a reboot loses them. The bar has **Save to
flash** (245 param1 = 1; one save covers every calibration done so far) and **Discard** (245 param1 = 0, the
vehicle's reload from flash: it returns to the saved settings and drops every unsaved change, not only the
calibrations). Calibrations in a row add up in one bar; a failed or cancelled one adds nothing. The bar goes
when a 245 with param1 = 1 or 0 ends accepted (from any tab), or when the vehicle restarts. On USB a reboot closes the link and the station reconnects with a new session
(a new recording), which starts with no bar; on a link that survives the restart (UDP) it is seen as the
`time_boot_ms` of `MARV_DEVICE_STATUS` going back by more than 1 s; after a restart
with calibrations unsaved the view keeps the line "unsaved calibration lost at reboot" until the next
calibration or accepted 245 param1 = 0 or 1. Save and Discard use the gate of the other command buttons. The
vehicle sends no `PARAM_VALUE` for values it changes itself, so the station asks for them: after an accepted
calibration, `PARAM_REQUEST_READ` by name for its offsets (gyro `IMU1_GYR_OFFX/Y/Z` and `IMU2_GYR_OFFX/Y/Z`,
accelerometer `IMU1_ACC_OFFX/Y/Z` and `IMU2_ACC_OFFX/Y/Z`, compass `MAG_OFFX/Y/Z`), and after an accepted 245
param1 = 0 (from any tab; it can change any parameter) the whole list again, as the Refresh button does
(Configure sessions only; a name with a write of its own under way is left out of the offset reads; one
request each, not repeated).

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
  (`Command::SendCommand`: ALL of them but the four flight commands of the last bullet below, an unknown `MAV_CMD` id
  counts as Configure, so storage actions,
  calibration, device actions (restart, self-test) and reboot are all Configure; section 10); and the three
  commands of log download (`Command::ListLogs`, `Command::DownloadLog`, `Command::CancelLogDownload`; section
  11), because the vehicle offers log download on its USB link only and the station must not ask for it on a
  radio or a network link, where the vehicle does not answer.
- Operate-class: everything that only reads or only concerns the station: the heartbeat (a service, not a
  command), a parameter list request (`Command::RefreshParams`), log recording
  (`Command::StartLog`, `Command::StopLog`, `Command::StartSessionLog`, `Command::StopSessionLog`), link management (connect, disconnect, replay).
- Mission commands, amendment 2026-11-23 (`_plan/decision-mavlink-mode-20261123.md`, sections 1 C and 5), approved by
  the owner: "1. Yes" (its section 8) to "Missions may be uploaded, read and cleared over the backpack link" (its
  question 1). Built: the three commands of the mission service
  (`Command::UploadMission`, `Command::DownloadMission`, `Command::ClearMission`; section 17) are Operate-class, so they
  are allowed on a UDP session, and their controls are enabled on Operate and Configure alike. Reason: the owner wants
  the waypoints made on the map tab sent to the vehicle, and in the field the laptop reaches the vehicle through the
  transmitter's WiFi backpack, which is the UDP link; and a mission write changes no setting of the vehicle: the
  vehicle refuses it while armed, keeps the list in RAM only (gone at the next boot), does nothing autonomous with it
  in this stage, and the station proves every result by reading the list back. No other rule changes: every
  parameter write and every `Command::SendCommand` stay Configure-class; the mission messages are not MAVLink
  commands (`COMMAND_LONG`). With Amendment A of section 17 built (approved, not built) the vehicle also takes a list
  while armed and flies it in AUTO: an upload in flight redirects the vehicle, as the owner asked ("We must be able to
  abort the mission whenever and redirect the drone"); the class does not change.
- Flight commands, amendment 2026-11-23 (the decision's section 9, A10), approved by the owner: "6. yes" (its section
  10) to "Start, Pause, Land and Abort work over the WiFi backpack link" (its question Q-A6). Approved, not built. The
  four flight commands of section 10 are Operate-class, so they are allowed on a UDP session and their controls are
  enabled on Operate and Configure alike: Start (`MAV_CMD_MISSION_START`), Pause (`MAV_CMD_DO_PAUSE_CONTINUE` with
  param1 0), Abort and Land (`MAV_CMD_DO_SET_MODE` with param1 1 and custom mode HOLD or LAND). Only these: the same
  commands with other parameters, and every other `MAV_CMD`, stay Configure-class. Whether the station gives them
  `Command` variants of their own or classes `Command::SendCommand` by its id and parameters is the station's (packet
  H4); the class match stays exhaustive. Reason: in the field the laptop reaches the vehicle only through the backpack;
  none of the four arms, disarms or kills, and the vehicle refuses each of them while a CRSF report is fresh (section
  10).

Services that run by themselves are not commands and are not gated. They only read: the capability
request (`MAV_CMD_REQUEST_MESSAGE` for `AUTOPILOT_VERSION`) at connect, the repair reads of a list that is
being downloaded, and the heartbeat.

Parameters on an Operate session: the list is not downloaded at connect; the Parameters view says so, and
a Refresh the user asks for is allowed (reads are operate-class). On a Configure session the list is
downloaded at connect as before. An Operate session
still shows a `PARAM_VALUE` it did not ask for, so a replayed log fills the table as before: the entry
is stored and the status follows the count, but no list is requested and no missing entry is read by index.

The Radio tab (section 12): Calibrate, Apply, the mapping edits and Save to flash are Configure controls. No rule
changes: `Command::SetParam` and `Command::SendCommand` are already Configure-class (amendment 2026-11-16).

The Sensor presets section of the Devices tab (section 13), amendment 2026-11-17: Apply, Save, Save and reboot and
the Custom edits are Configure controls (`Command::SetParam`, `Command::SendCommand`), no rule changes. The debug
switch ("Debug: raw sensor settings") is the station's own gate, in the owner's rule above: station-local, off at every
start, and the firmware knows nothing of it.

Views. Every configure control is SHOWN DISABLED, with the reason next to it or as hover text ("Configure
needs a serial or TCP link; this link is operate-only"), never hidden. The Link view and the connection bar
say which role the session has.

## 10. Commands

Written before the code (the design of 2026-11-13, `decision-foundation-20261113.md`, sections 5 and 8). The
station sends a MAVLink command only when the user presses a button; it never sends one by itself (the
capability request of section 9 is a read, not a command of this section). Every `Command::SendCommand`
is Configure-class: on an Operate session it writes 0 bytes and is recorded in `link.refused` (except the four flight
commands at the end of this section, section 9).

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
| 241 `MAV_CMD_PREFLIGHT_CALIBRATION` | param1 = 1: gyro; param5 = 4: accelerometer, one position, level; param2 = 1 and all others 0: compass (the user rotates the vehicle through all orientations for 30 s; progress about twice a second; FAILED when an axis saw too little movement); all seven 0: cancel the running calibration; the offsets apply at once and are saved only by 245 param1 = 1 |
| 245 `MAV_CMD_PREFLIGHT_STORAGE` | param1: 0 reload from flash (the vehicle sends no `PARAM_VALUE` afterwards), 1 save, 2 all parameters to defaults, 3 sensor calibration parameters to defaults (not the RC end points) |
| 246 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` | param1 = 1: reboot |
| 42430 `MAV_CMD_MARV_DEVICE` | param1 = device id (`MARV_DEVICE`), param2 = 1 restart, 2 self-test; restart for IMU1, IMU2, BARO, MAG, GNSS, BATT, self-test for IMU1, IMU2, MAG |
| 209 `MAV_CMD_DO_MOTOR_TEST` | see section 15; answered ACCEPTED at once (no IN_PROGRESS) |

What the station does (`gcs-mavlink`, service `commands`; state in `VehicleState.command`)
- One outstanding command at a time. A second `SendCommand` while one is outstanding is refused: nothing is
  sent and the outstanding command's record counts it (`refused`: the refused command number and how many);
  the outstanding command goes on. The one exception is the cancel below. A `SendCommand` while no vehicle
  is connected is refused too (the record's state is `Refused` with the reason).
- Send: `COMMAND_LONG` to the vehicle's system and component, `confirmation` 0. Without any ACK for that
  command it is sent again after 1.5 s with `confirmation` 1, and after another 1.5 s with 2: three sends
  in all, at 0, 1.5 s and 3.0 s. 1.5 s after the third send, with still no ACK, the result is "no answer"
  (at 4.5 s) and nothing more is sent. The one exception is a motor test spin (209 with param3 above 0): it
  is sent once, never again, and with no ACK the result is "no answer" 1.5 s after the send; a stop (209 with
  param3 0) is re-sent like any command.
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

Not in this feature: `COMMAND_INT`, a queue of commands, arming, flight commands (until the amendment below), automatic
commands.

Flight commands and the effective mode (amendment 2026-11-23; `_plan/decision-mavlink-mode-20261123.md` of
`MARV-V2-drivers`, section 9: A1, A9, A10 and its state table; approved by the owner, its section 10, with A2' and
A10' there). APPROVED, NOT BUILT. Built today (firmware 72de0d4): the vehicle
answers 300, 193 and 176 UNSUPPORTED (section 2) and sends custom_mode 0. The codes below come with stage 2, the states
and the commands with packet H2 (stage 3, `FS_HOLD_EN` 1); the station's buttons and `gcs-sim` with packet H4. It binds
the firmware, the station and `gcs-sim`.

The effective mode, in HEARTBEAT `custom_mode` (a value, not a bitfield), on every link:

| Code | Mode | The vehicle (the decision's state table) |
|---|---|---|
| 0 | not reported | every firmware before stage 2, and `gcs-sim` today: the one value section 2 assigned, kept; the station shows no mode |
| 1 | PILOT | a fresh CRSF report and the pilot latch set: today's flight modes |
| 2 | TAKEOVER | a fresh CRSF report in HOLD, AUTO, RETURN or LAND: the ladder as if ch6 were High, centred sticks, real sticks and ch6 ignored; it stays TAKEOVER until a report has shown switch6 High and a later one shows it not High (A2'), which gives PILOT |
| 3 | HOLD | hands-off Position hold, centred sticks |
| 4 | AUTO | flying the stored list (section 17) |
| 5 | RETURN | climbing to the return height, then a straight leg home |
| 6 | LAND | descending to touchdown, then disarm |
| 7 | STOP | disarmed, motors stopped |

Codes 8 and above are not assigned (a station shows "unknown" and the number). With `FS_HOLD_EN` 0 only PILOT and STOP
are sent.

The commands: `COMMAND_LONG`, on the radio link (on the station's side the UDP link through the backpack, section 9);
parameters not named are 0.

| Button | Command | Parameters | Taken in | Then |
|---|---|---|---|---|
| Start | 300 `MAV_CMD_MISSION_START` | all 0; the vehicle ignores them | HOLD, with every Start condition below | AUTO: from the kept place, else from `seq` 0 |
| Pause | 193 `MAV_CMD_DO_PAUSE_CONTINUE` | param1 0 | AUTO; RETURN | HOLD; from AUTO the place is kept |
| Abort | 176 `MAV_CMD_DO_SET_MODE` | param1 1 (`MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`), param2 3 (HOLD) | AUTO; RETURN; LAND on T3 only | HOLD, the place dropped |
| Land | 176 `MAV_CMD_DO_SET_MODE` | param1 1, param2 6 (LAND) | HOLD; AUTO; RETURN | LAND |

What the vehicle answers
- One `COMMAND_ACK` at once, addressed to the sender as every ACK: ACCEPTED when it takes the command (the state changes
  at the step it is taken), otherwise DENIED. Never IN_PROGRESS, TEMPORARILY_REJECTED or FAILED.
- DENIED, for each of the four, when any of these holds:
  1. a CRSF stick report is fresh (younger than 1.0 s, the gate's rule): in PILOT and TAKEOVER always;
  2. the vehicle is in a state the table does not give for that command: PILOT, TAKEOVER and STOP for all four, and
     every state with `FS_HOLD_EN` 0. The cases where the vehicle is already in the command's result (Start in AUTO,
     Pause or Abort in HOLD, Land in LAND) are an open point (section 17);
  3. the parameters differ from the table: Pause with param1 other than 0 (continue is not served: Start resumes);
     `MAV_CMD_DO_SET_MODE` with param1 other than 1, or param2 other than 3 or 6.
- Start is DENIED, also in HOLD, unless all of these hold (A10, A9):
  1. the station is present on the radio link: a station HEARTBEAT received on the radio link within the last 5.0 s
     (the decision's section 1 B; a USB heartbeat does not count, A1);
  2. a list with waypoints is stored: `count` 2 or more (`count` 1 is the return item alone; section 17);
  3. home is set (A6: its height at the arming step, its position at the first armed step with a position, before
     take-off; cleared at disarm);
  4. the vehicle and every stored waypoint are at most 1000 m horizontally from home (A9; a waypoint at 1000.5 m or a
     vehicle at 1001 m is refused). Checked at every Start and nowhere else: not at upload, not in flight;
  5. position tier T3 (A3: the position is covered);
  6. the heading monitor reports OK (`nav-eskf`:1774; designed separately);
  7. the battery is not critical;
  8. the last CRSF report before the sticks were lost had the mode switch in Position (switch6 High; arm on holds by
     construction) (A10').
- No command arms, disarms or kills, whatever its parameters. The one kill stays the arm switch on CRSF (A2).

The station sends them as every command above (one outstanding, three sends 1.5 s apart, the ACK rule); the first
ACCEPTED or DENIED ends one. It shows the effective mode from the HEARTBEAT, not from the ACK.

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
- A mounted card with no logs: ONE LOG_ENTRY with `id` 0, `num_logs` 0, `last_log_num` 0, `size` 0. The
  station reads it as an empty list, not as an error.
- No card, or the vehicle's SD row not OK (starting, recovering, clean-up before a log opens, logging off):
  the vehicle does NOT answer (amended 2026-11-16 after the firmware was built: an "empty list" would be a
  wrong answer while logs exist but cannot be reached). The station's listing then ends with "no answer"; the
  Devices view (`devices`) says why.
- A LOG_REQUEST_LIST during a download replaces the transfer (one activity at a time, as ArduPilot).
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
- When the link closes while a download runs (the cable is pulled, a read or write fails), the session ends the
  download at once as failed ("link lost at byte N"; nothing is sent, the link being gone), and does not wait
  for the retries above. Without that the state stayed `Running` for ever (seen on the real board, 2026-11-18).
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
  the running download. The station drops the writer when the download ends, whatever the end: the window's
  writer fills `NNNNNNNN.BIN.part` and renames it to `.BIN` only when exactly `size` bytes were written and
  flushed, otherwise deletes it (`marv-cli` writes its file directly and keeps a partial one). One listing or download at a time: a second request is refused and counted
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
The firmware side was run against this client on the real board on 2026-11-16 (its implementer): `logs list`
matched the card; `logs get` of two closed logs (1.26 MB and 1.17 MB) was byte-identical (sha256) to the
card-reader copies, at about 70 kB/s; the log being written downloads up to its flushed size. Not seen yet: a
card with hundreds of logs. A USB drop in the middle of a transfer (2026-11-18) showed the download never ending; fixed as above, tested, and seen on the board the same day (the link-lost line within seconds, the re-download completed).

## 12. RC mapping

Decision of 2026-11-16 (the lead, from the architect's consult). It binds both ends: the firmware computes and
reports, the station (the Radio tab) displays and writes the parameters, the simulator (`gcs-sim`) follows it.
The owner's rules: "I want to keep logic in firmware."; "settings should be saved when the user saves them via
apply or save button."; "mavlink is telemetry only. crsf is for sticks." The mapped sticks are reported, not
used: no arming, failsafe, flight mode, control loop, motor or actuator output. Parameters: section 2,
Parameters (the RC block). References: ArduPilot's `RC_Channel.cpp` (`AP`, line numbers of the copy kept with
the decision).

Input. The 16 microsecond values (880 to 2159 us) of the latest CRSF stick report. For each function f
(roll, pitch, throttle, yaw) the channel is c = `RCMAP_f`; `MIN`, `TRIM`, `MAX`, `REV` (`RCc_REVERSED`) and
`DZ` are those of channel c. Arithmetic on integers, the division in f32. Which function is which type is fixed
by the function (throttle is "range", the others are "angle"), not by a parameter; `DZ` means "around trim"
for an angle and "above the bottom" for a range.

Roll, pitch, yaw (angle; ArduPilot `pwm_to_angle_dz_trim`, `AP` 345-362, divided by `high_in`):
```
if MAX <= MIN: v = 0                                   (AP 462-464)
r  = min(max(pwm, MIN), MAX)                           (AP 353; not a clamp that panics when MIN > MAX)
hi = TRIM + DZ;  lo = TRIM - DZ;  s = REV ? -1 : 1     (AP 347-350)
if   r > hi and MAX != hi:  v = s * (r - hi) / (MAX - hi)   (AP 355-356)
elif r < lo and lo != MIN:  v = s * (r - lo) / (lo - MIN)   (AP 357-358)
else:                       v = 0                            (AP 359-360)
v = min(max(v, -1), 1)                                 (AP 454)
```
Throttle (range; ArduPilot `pwm_to_range_dz`, `AP` 387-401, divided by `high_in`; TRIM is not used):
```
if MAX <= MIN: v = 0
r  = min(max(pwm, MIN), MAX)                           (AP 389)
if REV: r = MAX - (r - MIN)                            (AP 391-393)
lo = MIN + DZ                                          (AP 395)
v  = (r > lo) ? (r - lo) / (MAX - lo) : 0              (AP 397-400)
v  = min(max(v, 0), 1)
```
No cross-parameter check exists (neither MIN < TRIM < MAX nor a unique map): the formulas are defined for every
in-range value, and no division by zero is reachable. A map with two functions on one channel is accepted; both
read that channel.

Sign convention (that of MANUAL_CONTROL and of PX4). Reported pitch is positive for stick FORWARD, roll positive
for stick RIGHT, yaw positive for CLOCKWISE (right), throttle 0 at the bottom. The firmware applies no sign
change beyond `REV`; the user sets `RCn_REVERSED` so that the station shows these directions.

Stale or absent. Without a CRSF stick report younger than 1 s (strictly less) there are no values and no
message. No neutral or held values are invented. Only stick reports whose source is CRSF feed the
normalisation: MAVLink RC_CHANNELS_OVERRIDE never does. RC_CHANNELS (raw) keeps its behaviour.

What is reported. MANUAL_CONTROL (69), on the firmware's USB link only, no dialect change. Stream: default
interval 200 000 us (5 Hz), fastest 20 000 us (50 Hz), changeable by SET_MESSAGE_INTERVAL like every stream.
Fields:
- `target` = 1 (the vehicle's own system id).
- `x` = round(1000 pitch); `y` = round(1000 roll); `z` = round(1000 throttle), 0 to 1000; `r` = round(1000 yaw).
- `buttons` = 0; the extension fields 0 (`enabled_extensions` = 0).
- Rounding to nearest, halves away from zero.
RC_CHANNELS (raw, 5 Hz) stays as it is: the station needs it for the calibration capture.

Worked examples (computed with an independent script from the formulas above; output = round(1000 v), halves
away from zero). They are binding for the firmware, the simulator and the station's tests.

| Angle: MIN, TRIM, MAX, REV, DZ | pwm -> output |
|---|---|
| 1100, 1500, 1900, 0, 0 (defaults) | 1500->0, 1700->500, 1300->-500, 1900->1000, 2012->1000, 1100->-1000, 988->-1000 |
| 988, 1500, 2012, 0, 0 | 2012->1000, 1756->500, 1244->-500, 1600->195, 1400->-195, 988->-1000 |
| 988, 1500, 2012, 0, 20 | 1510->0, 1520->0, 1521->2, 1766->500, 1234->-500, 2012->1000 |
| 988, 1500, 2012, 1, 0 | 1756->-500, 1244->500, 2012->-1000 |
| 1000, 1600, 2000, 0, 0 | 1800->500, 1300->-500, 1000->-1000, 2000->1000 |
| 1100, 1000, 1900, 0, 0 (TRIM below MIN) | 1100->111, 1550->611, 1900->1000 |
| 1400, 1500, 1600, 0, 200 (DZ wider than sides) | 1400->0, 1500->0, 1600->0 |
| 1900, 1500, 1100, 0, 0 (MIN above MAX) | 988->0, 1500->0, 2012->0 |

| Throttle: MIN, MAX, REV, DZ | pwm -> output |
|---|---|
| 1100, 1900, 0, 0 (defaults) | 988->0, 1100->0, 1500->500, 1900->1000, 2012->1000 |
| 988, 2012, 0, 0 | 988->0, 1244->250, 1500->500, 2012->1000 |
| 988, 2012, 0, 30 | 1018->0, 1515->500, 2012->1000 |
| 988, 2012, 1, 0 | 988->1000, 1244->750, 2012->0 |
| 1400, 1600, 0, 200 (DZ >= span) | 1400->0, 1500->0, 1600->0 |
| 1900, 1100, 0, 0 (MIN above MAX) | 988->0, 1500->0, 2012->0 |

Map example (defaults): channels 1..4 = 1700, 1300, 1500, 1900 give roll 500, pitch -500, throttle 500, yaw
1000; with RCMAP_ROLL = 2 and RCMAP_PITCH = 1: roll -500, pitch 500. A duplicate map (RCMAP_ROLL = RCMAP_YAW =
1) is accepted and both read channel 1, as in ArduPilot and PX4 (neither checks).

The station, and where the logic lives
- Firmware: all normalisation (map, reversal, end points, trim, dead zone, clamp), the freshness and source
  rule, the report. The station computes no normalised value: the four function bars show MANUAL_CONTROL.
- Station: during a calibration it captures per-channel extremes and centre from RC_CHANNELS, checks that a
  channel was moved, stages the values, writes them with PARAM_SET when the user presses Apply, and sends
  command 245 with param1 = 1 when the user presses Save to flash. Nothing is saved automatically.
- The station sends no radio-calibration command.

Display (Radio tab, read only, Operate and Configure)
- The 16 raw bars stay. On channels 1 to 8, once the parameter list is complete, markers at that channel's
  `MIN`, `TRIM` and `MAX`, and the channel tagged with its function (from `RCMAP_*`).
- Four function bars from MANUAL_CONTROL: Roll (`y`), Pitch (`x`), Yaw (`r`) on -1000..1000, Throttle (`z`) on
  0..1000, each with its expected direction ("stick forward: +", "stick right: +", "yaw right: +", "throttle
  up: 1000"). Stale after 2 s, as RC_CHANNELS; "no data" otherwise.
- A vehicle with no `RC1_MIN`: the mapping section says "this firmware has no RC mapping parameters".

Calibration (Configure sessions only)
1. "Calibrate radio": disabled, with its reason, on an Operate session (section 9) and while the sticks are
   stale. The text says: transmitter on in its CRSF model; battery unplugged, no propellers.
2. Capturing: per channel 1 to 16 the running minimum and maximum of RC_CHANNELS are drawn on the bars. The user
   moves every stick and switch slowly to both ends and holds each end briefly (5 Hz report), then "Done".
3. Centre: "Centre the sticks, throttle down, then OK"; OK takes the current value of each channel as its trim.
4. Review: a table for channels 1 to 8 (current `MIN`/`TRIM`/`MAX` -> captured). A channel whose captured range
   does not reach below 1300 AND above 1700 is "not moved" and is not written. `TRIM` is clamped into
   [`MIN`, `MAX`]. Channels 9 to 16 are shown, never written (they have no parameters).
5. Apply: one PARAM_SET per value that differs (at most 24), through the existing write path and its
   confirmation rules (section 2, Parameters); or Cancel: nothing is sent.
6. After Apply the tab shows "Applied in RAM; not saved to flash" with a "Save to flash" button (command 245
   with param1 = 1, the same command and gate as the Parameters tab), until a save is ACCEPTED in this session.
- Reversal is not inferred during the capture: it is an explicit checkbox in the mapping section, judged against
  the function bars.

Mapping (Configure sessions only; staged edits, then Apply, then Save to flash)
- Four drop-downs Roll / Pitch / Throttle / Yaw -> channel 1 to 8 (`RCMAP_*`).
- Per function: "Reversed" (`RCn_REVERSED` of the channel it maps to) and "Dead zone (us)" (`RCn_DZ`, 0 to 200).
- A duplicate map shows a warning line; it does not block.
- The same "Apply" / "Save to flash" pair and the same "not saved" line as the calibration.

The simulator (`gcs-sim`) has the 44 parameters with these defaults and ranges, and a MANUAL_CONTROL stream at
5 Hz computed from its own RC_CHANNELS and parameters with the formulas above (a test double: it reproduces the
worked-example tables).

### 14. ESC telemetry (amendment 2026-11-18, `_plan/decision-esc-telemetry-20261118.md`)
The firmware streams DShot300 at 1 kHz to the four ESCs from boot (Stop frames, and throttle on one motor during a motor test, section 15; no arming) and reads their bidirectional replies (eRPM, and extended telemetry after command 13).
**ESC_STATUS (291)**, USB, default 10 Hz (100 ms), fastest 50 Hz (20 ms). `index` 0. `time_usec` = board time of the latest exchange (since boot). Motor order: element 0 = M1 (GPIO39), 1 = M2 (GPIO38), 2 = M3 (GPIO37), 3 = M4 (GPIO36).
- `rpm[i]`: mechanical rpm = eRPM / (`MOT_POLES` / 2), rounded down, from the latest eRPM reply if it is younger than 100 ms; else 0. 0 when the motor reports stopped, and 0 when the ESC is not answering (see `ESC_INFO.info`).
- `voltage[i]`: the ESC's own extended-telemetry voltage (0.25 V steps) when received within the last 1 s, else NaN.
- `current[i]`: always NaN in this version (the AM32 ESCs on the bench report 153-160 A at rest: not a current).

**ESC_INFO (290)**, USB, default 1 Hz, fastest 10 Hz. `index` 0, `count` 4, `connection_type` 5 (DSHOT), `time_usec` as above.
- `counter`: DShot exchanges since boot, modulo 65536. Every exchange carries one frame to each of the four ESCs.
- `error_count[i]`: exchanges since boot in which ESC i gave no valid reply (silent, undecodable, or foreign signal), wrapping.
- Error percentage, computed by the station from two consecutive ESC_INFO: 100 x Δerror_count[i] / Δcounter (modulo 65536); blank when the two are more than 60 s apart, or when `time_usec` went back.
- `info`: bit i set when ESC i gave a valid reply within the last 100 ms (online).
- `failure_flags[i]`: 0 (AM32 reports no failure states).
- `temperature[i]`: extended-telemetry temperature x 100 (cdegC) when received within the last 1 s, else INT16_MAX.

Neither message is sent when the flight core's ESC data is older than 1 s. Not on the radio link in this version (trigger: the owner wants motor data in flight; budget about 270 B/s today).

Device row 12 (ESC), report-only: STARTING / ABSENT (no ESC has answered this boot: no battery) / OK (all four online) / DEGRADED (1-3 online, error TIMEOUT) / FAILED (none online after answering, error TIMEOUT; or the ESC task stopped, error STUCK). samples = valid replies, errors = invalid or missing replies, timeouts = drop-outs, attempts = stream starts.

The arming-tune silence shows as about 30 % errors for one second after each ESC start (expected).

The station's Motors tab shows this (and has the motor test controls, section 15); the simulator (`gcs-sim`) sends both messages at these rates, with row 12, as a test double.

### 15. Motor test (amendment 2026-11-18, `_plan/decision-motor-test-20261118.md`, amendment A)
Propellers off only. One motor at a time, no mixer, no arming. `COMMAND_LONG` 209 `MAV_CMD_DO_MOTOR_TEST`:
param1 motor 1-4 (M1-M4, the ESC_STATUS order of section 14); param2 0 (percent); param3 throttle 0 to 5.0 %
(0 = stop now); param4 duration above 0 up to 3.0 s (ignored for a stop); param5 0 or 1; param6 0 or 2 (board
order); param7 0. The level sent to the ESC is round(param3 x 20) of 2000, at least 1.
Answers: ACCEPTED at once, and the spin runs for param4 (never more than 3 s from its first frame). A spin
command while one runs replaces it (the earlier motor gets Stop). A stop is always ACCEPTED and ends the spin
within one exchange (1 ms). DENIED (with a STATUSTEXT): a parameter outside the above. TEMPORARILY_REJECTED:
another action running. While a spin runs, every other command that starts an action is TEMPORARILY_REJECTED.
The firmware does not check the ESCs or the battery before a spin and does not end a spin on telemetry: its
limits are the 5 % ceiling and the duration, as in Betaflight and ArduPilot. There is no repeat-to-hold: if the
link drops, the spin runs to its end. The other three motors get Stop throughout. The firmware answers 209 on
every link (section 9); the station sends it only on a Configure session, only from the Motors tab with the
"propellers are removed" switch on; Stop needs no switch. The simulator answers 209 by this table and shows rpm
on the tested motor for the spin.
The station sends a spin once and does not re-send it when no answer comes; a stop is re-sent like any command.

### 16. Compass calibration (amendment 2026-11-20, `_plan/decision-compass-cal-20261120.md`)
**Parameters.** 11 parameters appended after index 79, so no existing index moves; with index 80 (`GPS_RATE_MS`, I32,
default 200, allowed 100, 200, 500, 1000, firmware branch `gnss-presets`; its section is not written here) the count
goes to 92. Amendment 2026-11-22 (u-blox SAM-M10Q, firmware branch `gnss-m10`, `_plan/decision-gnss-m10-20261122.md`):
index 92 `GPS_GNSS` (I32, default 0, range 0..2, Configure: the receiver is re-configured on the write, no reboot; 0 = GPS +
Galileo + BeiDou B1C + QZSS, 1 = GPS + Galileo, 2 = GPS + Galileo + GLONASS) takes the count to 93. The firmware refuses
a `GPS_RATE_MS` faster than the constellation set allows: 100 ms only with `GPS_GNSS` 1, with a text that names both
values. Index 92 matches the firmware (gnss-m10 16b782e). Indices 81-89 are "calibration" parameters: command 245 with param1 = 3 returns each of them to its default.
All are Live (applied at once, saved only by 245 param1 = 1).

| Index | Name | Type, unit | Default | Range | Reset by calibration reset |
|---|---|---|---|---|---|
| 81 | `MAG_DIAGX` | F32 | 1.0 | 0.2..5.0 | yes |
| 82 | `MAG_DIAGY` | F32 | 1.0 | 0.2..5.0 | yes |
| 83 | `MAG_DIAGZ` | F32 | 1.0 | 0.2..5.0 | yes |
| 84 | `MAG_ODIAGXY` | F32 | 0.0 | -1.0..1.0 | yes |
| 85 | `MAG_ODIAGXZ` | F32 | 0.0 | -1.0..1.0 | yes |
| 86 | `MAG_ODIAGYZ` | F32 | 0.0 | -1.0..1.0 | yes |
| 87 | `MAG_CAL_TYPE` | I32 | 0 | 0..2 (0 none or unknown, 1 quick, 2 guided) | yes |
| 88 | `MAG_FLD_UT` | F32, uT | 0.0 | 0..95 (0 = not set) | yes |
| 89 | `MAG_FLD_INC` | F32, deg | 0.0 | -90..90 (positive down; meaningful only when `MAG_FLD_UT` > 0) | yes |
| 90 | `MAG_HLT_STR` | F32, % | 10 | 1..50 | no |
| 91 | `MAG_HLT_INC` | F32, deg | 10 | 1..45 | no |

`MAG_OFFX/Y/Z` keep their index, range (-200..200 uT) and meaning.

**Model.** m = S (r - o), all in microtesla. r is the magnetometer's field in the body frame (FRD) after `MAG_ORIENT`;
o = (`MAG_OFFX`, `MAG_OFFY`, `MAG_OFFZ`); S is symmetric:

    [[DIAGX,   ODIAGXY, ODIAGXZ],
     [ODIAGXY, DIAGY,   ODIAGYZ],
     [ODIAGXZ, ODIAGYZ, DIAGZ  ]]

In single precision, in this order: d = r - o; m_i = (S_i0 d_0 + S_i1 d_1) + S_i2 d_2, with no fused multiply-add.
With the defaults (S = identity) the result equals r - o. The sign convention is PX4's (offsets are subtracted);
ArduPilot adds its offsets, so an ArduPilot offset must be negated when ported.

**Test vector.** Both ends test against it: the firmware bit for bit, the station's undo within 1e-4 uT.
- S: DIAGX 1.0625, DIAGY 0.9375, DIAGZ 0.78125; ODIAGXY 0.03125, ODIAGXZ -0.046875, ODIAGYZ 0.0625.
- o = (-14.2, -4.2, 1.5) as f32 = 0xC1633333, 0xC0866666, 0x3FC00000.
- r = (20.5, -3.25, 38.0) = 0x41A40000, 0xC0500000, 0x42180000.
- d = 0x420ACCCD, 0x3F733330, 0x42120000.
- m = (35.1875, 4.2562499..., 26.9484367...) = 0x420CC000, 0x40883333, 0x41D79666.

**What the vehicle publishes.** The magnetometer fields of HIGHRES_IMU `id` 0 carry m: the field with the calibration
in force applied (the offsets and S). The unit stays the one of the existing text (gauss; 1 gauss = 100 uT).

**The station's undo.** For each sample, r = S0^-1 m + o0, with S0 and o0 read at the start of the run.
- If S0 is not positive definite, the station refuses to start.
- If any of `MAG_OFF*` or 81-86 changes during the run (a PARAM_VALUE is seen), the run is abandoned.
- The station refuses to start, or stops, when HEARTBEAT shows armed.

**Write order (the commit).**
1. `MAG_CAL_TYPE` = 0.
2. `MAG_OFFX/Y/Z`, `MAG_DIAGX/Y/Z`, `MAG_ODIAGXY/XZ/YZ`, `MAG_FLD_UT`, `MAG_FLD_INC`.
3. `MAG_CAL_TYPE` = 2.
4. Read all of them back and compare.

Save (245 param1 = 1) only after a good read-back. If the link drops during the write, the vehicle holds a mixed set
marked type 0, in RAM only: Discard (245 param1 = 0) or a reboot restores flash.

**Calibration state**, as both ends read it:
- Not calibrated: `MAG_CAL_TYPE` 0 and `MAG_OFFX/Y/Z` all zero.
- Best effort: type 1, or type 0 with non-zero offsets.
- Guided: type 2 with `MAG_FLD_UT` > 0.

**The quick method** (command 241 with param2 = 1, section 10): from this amendment it measures the body-frame raw
field, not the field with the calibration in force. On acceptance it writes, together, `MAG_OFFX/Y/Z`, S = identity,
`MAG_CAL_TYPE` = 1 and `MAG_FLD_UT` = 0.

**Health-test data.** Field strength is compared with `MAG_FLD_UT` x (1 +- `MAG_HLT_STR` / 100), the inclination
(positive down) with `MAG_FLD_INC` +- `MAG_HLT_INC`. With `MAG_FLD_UT` = 0 the test is unavailable. What the flight
code does with this is not part of this amendment.

The guided procedure (poses, fit, acceptance) is a station feature; another station needs only the parameters and the
model, test vector, publication, undo, write order and state above.

## 13. Sensor presets

Decision of 2026-11-17, amended the same day on the owner's redirect: "preset configs should basically be us setting
several params at once as a verified/sane combo. not some special vip feature inside the driver/firmware". A preset is
a station feature: the firmware has no preset, no selector and no preset rule. The owner's earlier rules still hold:
default IMU preset Flight (+-16 g / +-2000 dps / 800 Hz); "do not over engineer"; "settings should be saved when the
user saves them via apply or save button."; the user does not type a manual rate "only if it's for debugging
purposes". The parameters: section 2, Parameters (the IMU parameters and the BARO and MAG fields).

One source. The module `gcs_mavlink::presets` of this workspace is the normative table: ids, names, summaries, field
values, axes, physical values and scores; the window, `marv-cli` and the tests use it, the simulator does not. The
tables below are its content. Ids 1 to 4 and their names do not change meaning or number; a new preset is appended with
the next id. Nothing is added to the link: parameter metadata stays "Not now" (section 2).

What a preset changes. IMU presets vary only the accelerometer range, the gyro range and the output data rate (both
sensors of a chip at one rate); everything else is as before in every preset (internal clock, INT1 latched data-ready,
single-sample reads, no FIFO, chip filters off). BARO presets: rate, pressure and temperature oversampling, pressure and
temperature IIR. MAG presets: rate and averaging. GNSS presets (amendment 2026-11-22, owner's choice "Rate +
constellations"; it lifts the 2026-11-17 rule that no preset sets the constellations): the fix interval `GPS_RATE_MS`
and the constellation set `GPS_GNSS`. None for BATT.

The chart (radar). Axes, more is better on every axis. IMU1 and IMU2 (pentagon): Accel range, Gyro range, Low noise,
Bandwidth, Light load. BARO and MAG (diamond): Low noise, Fast response, Update rate, Light load. GNSS (diamond):
Update rate, Fast response, Light load, Sky coverage.

Score of an axis: `s = round_half_up(100 * clamp(ln(x / x0) / ln(x100 / x0), 0, 1))`, a log scale between two fixed
ends x0 (score 0) and x100 (score 100), the same ends for both IMUs. IMU Low noise is the mean of two fractions:
`s = round_half_up(100 * (fg + fa) / 2)`, fg and fa the clamped fractions of the gyro and accelerometer noise.

| Sensor | Axis | Quantity x | x0 (0) | x100 (100) |
|---|---|---|---|---|
| IMU | Accel range | full scale, g | 2 | 32 |
| IMU | Gyro range | full scale, dps | 125 | 4000 |
| IMU | Low noise | gyro rms dps; accel rms mg | 1 dps; 10 mg | 0.01 dps; 0.1 mg |
| IMU | Bandwidth | B = ODR / 2, Hz | 10 | 5000 |
| IMU | Light load | SPI bytes per second | 200 000 | 200 |
| BARO | Low noise | pressure rms, Pa | 1 | 0.01 |
| BARO, MAG | Fast response | delay, ms | 1000 | 1 |
| BARO, MAG | Update rate | ODR, Hz | 1 | 500 |
| BARO, MAG | Light load | I2C bytes per second | 10 000 | 10 |
| MAG | Low noise | x/y field rms, nT | 1000 | 10 |

Physical values (from datasheet and driver numbers; MODEL and BOUND are shown as such on the label):
- IMU Bandwidth: B = ODR / 2 (MODEL; the LSM6DSK320X datasheet's own convention, and the upper bound for the ICM-45686
  with its UI filter bypassed).
- IMU noise: `sigma = sqrt(ND^2 * B + LSB^2 / 12)`, gyro and accelerometer separately; ND the datasheet's noise density
  (ICM gyro 0.0038 dps/rtHz, accel 70 / 80 / 110 ug/rtHz up to 8 g / 16 g / 32 g; LSM gyro 3.8 mdps/rtHz, accel
  60 ug/rtHz), LSB the driver's scale (full scale / 32768 for the ICM; the sensitivity of the range for the LSM).
- IMU Light load: ODR x bytes per sample: ICM 18, LSM 27 (the driver's `read_all`: 26 bytes + address; amendment 2026-11-17, measured: the design said 21).
- BARO Low noise: `sigma_T9(OSR_P) / sqrt(2c + 1)`, sigma_T9 the datasheet's Table 9 value for the pressure
  oversampling, c the pressure IIR coefficient. Altitude shown beside it: 0.083 m per Pa (display only).
- BARO Fast response: `tconv_p(OSR_P) + tconv_t(OSR_T) + c / ODR`, the typical conversion times of the datasheet's
  Table 4.
- BARO Light load: ODR x 13 bytes.
- MAG Low noise: `190 nT * sqrt(2 / N)` for x/y and `450 nT * sqrt(2 / N)` for z, N the averaging (MODEL).
- MAG Fast response: delay at most N x 2.5 ms (BOUND).
- MAG Light load: ODR x 23 bytes.
- Not shown on purpose: supply current, chip filter delay, IMU latency.

Scores 0 to 100; the physical value in brackets. Computed with an independent script from the formulas above; the
module's tests (`crates/gcs-mavlink/tests/presets.rs`) are the oracle.

IMU1, ICM-45686 (low-noise mode, internal clock, UI LPF bypass, FIR off, INT1 latched, no FIFO).

| Id | Name | ACC_FS | GYR_FS | ODR_HZ | Accel range | Gyro range | Low noise | Bandwidth | Light load |
|---|---|---|---|---|---|---|---|---|---|
| 1 | Flight (default) | 16 | 2000 | 800 | 75 | 80 | 48 (0.078 dps, 1.61 mg) | 59 (400 Hz) | 38 (14 400 B/s) |
| 2 | Bench (today; clips in flight) | 4 | 250 | 800 | 25 | 20 | 49 (0.076 dps, 1.40 mg) | 59 (400 Hz) | 38 (14 400 B/s) |
| 3 | Max range | 32 | 4000 | 800 | 100 | 100 | 43 (0.084 dps, 2.22 mg) | 59 (400 Hz) | 38 (14 400 B/s) |
| 4 | Smooth | 8 | 1000 | 400 | 50 | 60 | 57 (0.054 dps, 0.99 mg) | 48 (200 Hz) | 48 (7 200 B/s) |

IMU2, LSM6DSK320X (high-accuracy ODR set 800: code 960 = 800 Hz, 480 = 400 Hz; LPF1 and LPF2 off; high-g channel off;
INT1 low-g data-ready latched; no FIFO).

| Id | Name | ACC_FS | GYR_FS | ODR_HZ | Accel range | Gyro range | Low noise | Bandwidth | Light load |
|---|---|---|---|---|---|---|---|---|---|
| 1 | Flight (default) | 16 | 2000 | 800 | 75 | 80 | 51 (0.079 dps, 1.21 mg) | 59 (400 Hz) | 32 (21 600 B/s) |
| 2 | Bench (today; clips in flight) | 4 | 500 | 800 | 25 | 40 | 51 (0.076 dps, 1.20 mg) | 59 (400 Hz) | 32 (21 600 B/s) |
| 3 | Max range (low-g limit 16 g) | 16 | 4000 | 800 | 75 | 100 | 50 (0.086 dps, 1.21 mg) | 59 (400 Hz) | 32 (21 600 B/s) |
| 4 | Smooth | 8 | 1000 | 400 | 50 | 60 | 58 (0.055 dps, 0.85 mg) | 48 (200 Hz) | 42 (10 800 B/s) |

BARO, BMP581 (normal mode, FIFO off, data-ready pulsed).

| Id | Name | ODR_HZ | OSR_P | OSR_T | IIR_P | IIR_T | Low noise | Fast response | Update rate | Light load |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Balanced (default; today) | 50 | 16 | 2 | 0 | 0 | 34 (0.21 Pa, about 1.7 cm) | 65 (11.5 ms) | 63 | 40 (650 B/s) |
| 2 | Fast | 100 | 8 | 1 | 0 | 0 | 26 (0.30 Pa, 2.5 cm) | 73 (6.4 ms) | 74 | 30 (1 300 B/s) |
| 3 | Smooth | 25 | 32 | 2 | 3 | 0 | 62 (0.057 Pa, 0.5 cm) | 28 (141.5 ms) | 52 | 50 (325 B/s) |
| 4 | Precise (Bosch "Highest resolution") | 10 | 128 | 8 | 0 | 0 | 55 (0.08 Pa, 0.7 cm) | 36 (82.5 ms) | 37 | 63 (130 B/s) |

MAG, BMM350 (normal mode, all axes, data-ready pulsed).

| Id | Name | ODR_HZ | AVG | Low noise (MODEL) | Fast response (BOUND) | Update rate | Light load |
|---|---|---|---|---|---|---|---|
| 1 | Balanced (default; today) | 100 | 2 | 36 (190 nT x/y, 450 z) | 77 (<= 5 ms) | 74 | 21 (2 300 B/s) |
| 2 | Fast | 200 | 2 | 36 (190 nT, 450 z) | 77 (<= 5 ms) | 85 | 11 (4 600 B/s) |
| 3 | Low noise | 100 | 4 | 44 (134 nT, 318 z) | 67 (<= 10 ms) | 74 | 21 (2 300 B/s) |
| 4 | Ultra low noise | 50 | 8 | 51 (95 nT, 225 z) | 57 (<= 20 ms) | 63 | 31 (1 150 B/s) |

GNSS (device 5, u-blox SAM-M10Q; fields `GPS_RATE_MS`, `GPS_GNSS`; default preset 1 = the firmware defaults 200 / 0).
Physical values: Update rate = 1000 / `GPS_RATE_MS`, Hz. Fast response = `GPS_RATE_MS`, ms, BOUND (a fix is at most one
interval old at the receiver; latency excluded). Light load = 100 B per NAV-PVT frame (6 B header + 92 B payload + 2 B
checksum, u-blox M10 interface description) x rate, B/s, receiver to board. Sky coverage = the number of constellations
enabled (QZSS counted with GPS, not on its own): 4, 2, 3.

| Sensor | Axis | Quantity x | x0 (0) | x100 (100) |
|---|---|---|---|---|
| GNSS | Update rate | fixes per second | 1 | 10 |
| GNSS | Fast response | interval, ms | 1000 | 1 |
| GNSS | Light load | UART bytes per second | 11 520 (115200 8N1) | 100 (one frame per second) |
| GNSS | Sky coverage | constellations | 1 | 4 |

| Id | Name | GPS_RATE_MS | GPS_GNSS | Update rate | Fast response (BOUND) | Light load | Sky coverage |
|---|---|---|---|---|---|---|---|
| 1 | Flight (default) | 200 | 0 | 70 (5 Hz) | 23 (<= 200 ms) | 66 (500 B/s) | 100 (4) |
| 2 | Fast | 100 | 1 | 100 (10 Hz) | 33 (<= 100 ms) | 51 (1 000 B/s) | 50 (2) |
| 3 | Max sky | 200 | 2 | 70 (5 Hz) | 23 (<= 200 ms) | 66 (500 B/s) | 79 (3) |

GNSS summaries: "5 Hz, GPS+Galileo+BeiDou+QZSS", "10 Hz, GPS+Galileo", "5 Hz, GPS+Galileo+GLONASS". GNSS has three
presets (ids 1 to 3); the other sensors have four.

Summaries for the cards (crate `summary`): IMU "+-16 g, +-2000 dps, 800 Hz" etc.; BARO "50 Hz, x16/x2, no filter"; MAG
"100 Hz, average of 2".

The station (Devices tab, a "Sensor presets" section above the health table; five cards IMU1, IMU2, BARO, MAG, GNSS)
- Each card: the preset picker (names from the table), the chart, the summary line, the physical value of every axis
  with its unit and its MODEL or BOUND tag, one status line. Cards show on Operate when the parameter list is present,
  otherwise "parameters not loaded".
- Active preset: the preset whose field values all equal the vehicle's (a REAL32 compared numerically), else "Custom".
  There is no selector to read and no version to compare.
- Apply writes the chosen preset's fields as ordinary `PARAM_SET` writes: first every field whose value goes DOWN, then
  every field whose value goes UP, each group in the table's field order; a field whose value does not change is not
  written. Every intermediate combination stays valid because the limits are downward closed: BMP581 Table 7 (the
  highest ODR falls as OSR_P or OSR_T rises) and BMM350 Table 5 (the highest averaging falls as ODR rises) only
  forbid raising an OSR or an averaging above what the ODR allows, or raising an ODR above what the OSR allows; the
  IIR and the IMU fields are unconstrained. Lowering an ODR, an OSR or an averaging therefore never makes a valid
  combination invalid, and the first group never does; the second group ends on a valid combination. GNSS is the one
  exception, because its fields are tied: the firmware refuses 100 ms unless `GPS_GNSS` is 1. The card therefore writes
  `GPS_GNSS` before a faster rate (a lower `GPS_RATE_MS`) and after a slower one: Flight to Fast writes `GPS_GNSS` 1
  then `GPS_RATE_MS` 100; Fast to Flight writes `GPS_RATE_MS` 200 then `GPS_GNSS` 0; Max sky to Fast writes `GPS_GNSS`
  1 then `GPS_RATE_MS` 100; Fast to Max sky writes `GPS_RATE_MS` 200 then `GPS_GNSS` 2; Flight and Max sky differ in
  `GPS_GNSS` only. Every intermediate combination is valid. BARO, MAG and GNSS show "Applied in RAM; not saved".
- The card judges the result from each write's echo (the vehicle's PARAM_VALUE after the write; the vehicle answers
  every PARAM_SET with exactly one, section 2) compared with the table. Every write answered and every field equal:
  BARO and MAG show "Applied in RAM; not saved", and Save to flash (command 245 with param1 = 1) as on the Radio tab.
  A refused write (a differing echo): the card names the field and shows what the values now match, often "Custom".
  A write with no answer: the same, naming the field.
- IMUs: the same writes (the vehicle answers each with "<NAME>: applies after reboot"); the card then shows "Pending:
  takes effect after Save and reboot" with one button "Save and reboot": command 245 with param1 = 1 and, only on its
  ACCEPTED, command 246 with param1 = 1. Pending is session-local (the running configuration is not on the link). A
  reboot without a save drops the preset.
- Accepted costs: another tool can write raw values, and a link drop in the middle of an Apply leaves the sensor
  between two presets ("Custom") until it is applied again. The vehicle validates every write regardless.
- Debug switch: station-local, off at every start, "Debug: raw sensor settings" in the section header. Off: "Custom
  (debug)" is not offered; the Parameters tab shows the fields a preset sets read-only with "set in Devices > Sensor
  presets" (a vehicle whose values match no preset is shown as "Custom", read-only, and a preset can be chosen). On:
  "Custom (debug)" is offered; choosing it shows the card's fields as values to type, and the Parameters tab edits them
  (IMU fields marked "applies after reboot"). `marv-cli param set` of a field a preset sets needs `--debug`. It is a gate
  in the station only (section 9).
- Operate and Configure: Apply, Save, Save and reboot and the Custom edits are Configure controls, shown disabled with
  the reason on Operate (section 9); Apply is also disabled while a calibration runs.

The simulator (`gcs-sim`) enforces the GNSS cross rule (`GPS_RATE_MS` 100 needs `GPS_GNSS` 1, in both directions, with a
text that names both values) and has the six IMU parameters (Reboot class, the "applies after reboot" text) and the BARO and MAG fields at the Flight / Balanced values, range-checked only (a test double); it does not use the table.

## 17. Missions

Proposed 2026-11-23 (`_plan/decision-mavlink-mode-20261123.md` of `MARV-V2-drivers`: section 1 C, section 3 stage 1,
sections 4 and 5). Approved by the owner 2026-11-23 (its section 8: "1. Yes", "2. Yes", and "Yes" to 1500 ms / 5
retries and the 40 ms / 5 ms reply slots), except two rules that Amendment A below replaces: the refusal while armed and
the unused `z` (his "3. No"). BUILT: every paragraph from here to "Amendment A" (stage 1: firmware 72de0d4, the station,
`gcs-sim`). APPROVED, NOT BUILT: the paragraphs under "Amendment A" (the decision's section 9, approved in its section
10). It binds the vehicle (the firmware), the station (`gcs-mavlink`, the map tab,
`marv-cli`) and the station's simulated vehicle (`gcs-sim`); each is built from this text alone. Scope (stage 1):
upload, read back, clear and store a list of waypoints. Nothing here flies a list (Amendment A: a list flies in AUTO).
It is MAVLink's mission protocol
(`MM` = https://mavlink.io/en/services/mission.html, read 2026-11-23) cut down to the subset below, with the messages
of `common` unchanged: no dialect change, no new MARV message. `CX` = `definitions/common.xml` of `marv-dialect`
(sha256 f157929c...). Where `MM` leaves a point open, the rule here settles it.

Messages (id, CRC_EXTRA and length with extensions as generated by `marv-dialect` for `common`):

| Message | Id | CRC_EXTRA | Length | Station sends | Vehicle sends |
|---|---|---|---|---|---|
| MISSION_REQUEST_LIST | 43 | 132 | 3 | to start a download | never |
| MISSION_COUNT | 44 | 221 | 9 | to start an upload | to answer MISSION_REQUEST_LIST |
| MISSION_CLEAR_ALL | 45 | 232 | 3 | to clear | never |
| MISSION_ACK | 47 | 153 | 8 | to end a download; to give up | to end an upload or a clear; every refusal |
| MISSION_REQUEST_INT | 51 | 196 | 5 | in a download | in an upload |
| MISSION_ITEM_INT | 73 | 38 | 38 | in an upload | in a download |

- This revision of `CX` has the extension field `mission_type` in all six messages and `opaque_id` (uint32) in
  MISSION_COUNT and MISSION_ACK (`CX`:5863-5864, :5889-5890). MAVLink 2 cuts trailing zero bytes from a payload: a
  receiver reads every missing byte as 0, so a frame that ends before `mission_type` or `opaque_id` carries 0 there.
- Not served: MISSION_ITEM (39) and MISSION_REQUEST (40) (deprecated; `MM` asks implementations to support them too, the
  decision's subset leaves them out: they are on no accept list of the vehicle, which drops them, and the station
  never sends them); MISSION_WRITE_PARTIAL_LIST, MISSION_SET_CURRENT, MISSION_CURRENT, MISSION_ITEM_REACHED; the fence
  and rally plan types.

Where. On both of the vehicle's links, USB and radio, identically: the same messages, values, rules and timers; only the
reply pacing differs (below). One service on the vehicle serves both links. The station uses it on every session
(serial, TCP, UDP: section 9).

Addressing
- The station sends every mission message with `target_system` and `target_component` = the vehicle's system and
  component as learned from its HEARTBEAT (1 and 1).
- The vehicle handles a mission message only when `target_system` is 0 or 1 and `target_component` is 0 or 1 (the rule
  of the firmware's log download, `src/logdl.rs`); any other is dropped and not answered.
- The vehicle addresses every mission message it sends to the system and component that sent the message it answers
  (the station: 255 and 190), on the link that message came from.
- The station takes a mission message only from the vehicle's system and component, with `target_system` 0 or its own
  id and `target_component` 0 or its own id; it drops any other.

Field values. "Sends" is what the sender puts in the field; "accepts" is the receiver's check. A field marked ignored is
neither checked nor stored. Waypoint fields of MISSION_ITEM_INT:

| Field | Station sends (upload) | Vehicle accepts | Vehicle sends (download) | Station accepts |
|---|---|---|---|---|
| `seq` | the seq requested | the expected seq only (others: below) | the seq requested | the expected seq only |
| `frame` | 3 (`MAV_FRAME_GLOBAL_RELATIVE_ALT`) | 3 or 6 (`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`, the same frame: `MM`); else refused | 3, whichever of 3 or 6 was uploaded (the frame is not stored, below) | compared (read-back) |
| `command` | 16 (`MAV_CMD_NAV_WAYPOINT`) | 16; else refused | 16 | compared |
| `current` | 0 | ignored | 0 | ignored |
| `autocontinue` | 1 | ignored | 1 | ignored |
| `param1` to `param4` | 0.0 | ignored (any value, NaN too) | 0.0 | ignored |
| `x` | latitude in degrees x 10^7 | -900000000 to 900000000, both included; else refused | as stored | compared |
| `y` | longitude in degrees x 10^7 | -1800000000 to 1800000000, both included; else refused | as stored | compared |
| `z` | altitude in m above home, finite | any value: its 4 bytes are stored unchanged, never used | the stored 4 bytes | compared as 4 bytes |
| `mission_type` | 0 | 0; else refused | 0 | 0; else dropped |

The other messages:
- MISSION_COUNT from the station: `count` = n, the number of waypoints (0 clears); `mission_type` 0; `opaque_id` 0
  (`CX`: "0 on upload"). The vehicle checks `count` and `mission_type` and ignores `opaque_id`.
- MISSION_COUNT from the vehicle: `count` = the number stored; `mission_type` 0; `opaque_id` = the plan id of the stored
  list.
- MISSION_REQUEST_LIST and MISSION_CLEAR_ALL: `mission_type` 0. MISSION_REQUEST_INT: `seq`, `mission_type` 0.
- MISSION_ACK from the vehicle: `type` from the table of results below; `mission_type` = that of the message it
  answers, for every `type`, MAV_MISSION_DENIED included (0 when the vehicle gives up by itself); `opaque_id` = the new plan id when it accepts an upload (0 for an
  upload of count 0), 0 in every other MISSION_ACK, an accepted clear included.
- MISSION_ACK from the station: `type` MAV_MISSION_ACCEPTED (end of a download), MAV_MISSION_OPERATION_CANCELLED (it
  gives up) or MAV_MISSION_INVALID_SEQUENCE (below); `mission_type` 0; `opaque_id` 0 (`CX`: "0 on download").

Waypoints and the station's file. `seq` 0 is the first waypoint (`MM`: `seq` is "indexed from 0"; ArduPilot's home
position at `seq` 0 is not followed). The station's file (QGC WPL 110, `crates/marv-gcs/src/missions.rs`) holds a home
placeholder in row 0 and the waypoints in rows 1 to n: row i is sent as `seq` i - 1, and row 0 is never sent. `x` and
`y` are the row's latitude and longitude times 10^7, computed in f64 and rounded to the nearest integer, halves away
from zero; `z` is the row's altitude as f32. A list read back from the vehicle is written with row i = `seq` i - 1 and
row 0 made by the file's own rule.

Capacity. 16 waypoints. A `count` of 1 to 16 is an upload; a `count` of 0 is valid and clears the list (`MM`: "Uploading
an empty mission (MISSION_COUNT is 0) has the same effect as clearing the mission"); a `count` of 17 or more is refused.
The station does not check the count itself: the vehicle decides.

Storage. The list lives in the vehicle's RAM only: it is empty after every boot (plan id 0) and is never written to
flash. A link going down does not touch it. Per waypoint the vehicle keeps `x`, `y` and the 4 bytes of `z` (12 bytes,
192 bytes for 16); `frame` and `command` are not stored (always 3 and 16 on download), so a list uploaded with frame 6
reads back with frame 3.

Staging and commit. An upload writes into a staging copy. When the last item (`seq` n - 1) has passed every check, the
staging copy replaces the stored list in one step, the plan id is computed, and only then is MISSION_ACK
MAV_MISSION_ACCEPTED sent. Whatever reads the stored list sees the old list or the new one, never a mix. Every other
end of an upload (a refusal, a cancel, a timeout, the link going down, a new start message from the owner, a reboot)
discards the staging copy: the stored list and its plan id stay as they were (`MM`: after an error "the current state
of the mission on the vehicle should be unaltered").

Plan id
- Algorithm: the IEEE CRC-32 of the firmware's `crates/fw-core/src/record.rs` (`crc32`): reflected polynomial
  0xEDB88320, initial value 0xFFFFFFFF, final XOR 0xFFFFFFFF; the CRC of the ASCII bytes "123456789" is 0xCBF43926.
- Bytes covered, for a list of n waypoints: 12 x n bytes, for `seq` = 0, 1, ..., n - 1 in that order: `x` as int32
  little-endian, `y` as int32 little-endian, `z` as its IEEE-754 single-precision bits, little-endian. Nothing else
  (no count, `seq`, `frame` or `command`).
- The empty list (n = 0) covers no bytes: its plan id is 0x00000000. If the CRC of a non-empty list comes out
  0x00000000, its plan id is 0x00000001 (0 means "no plan" in `opaque_id`, `CX`).
- Where it travels: MISSION_ACK `opaque_id` when an upload is accepted; MISSION_COUNT `opaque_id` in every download
  (`MM`, "Detecting Plan Changes"). The station computes it from the waypoints it sent. Test vector: example 1.

Transactions
- One at a time for the whole vehicle, across both links. An upload and a download are transactions; a clear (and an
  upload of count 0) is answered at once and leaves no transaction running. The start messages are MISSION_COUNT,
  MISSION_REQUEST_LIST and MISSION_CLEAR_ALL.
- The owner of a transaction is (link, system id, component id) of the frame that started it (from the frame header).
  The same station on the other link is another owner.

What the vehicle does with each mission message that passed the addressing check:
1. While a transaction runs, a message from anyone but its owner: MISSION_COUNT, MISSION_REQUEST_LIST,
   MISSION_CLEAR_ALL, MISSION_ITEM_INT and MISSION_REQUEST_INT get MISSION_ACK MAV_MISSION_DENIED; a MISSION_ACK is
   ignored. The running transaction is not changed.
2. A start message from the owner ends the owner's running transaction (as a cancel: staging discarded, nothing sent)
   and is then handled as in 3. A repeated MISSION_COUNT or MISSION_REQUEST_LIST therefore starts its transaction
   again from the beginning.
3. No transaction running:
   - MISSION_COUNT, checked in this order: `mission_type` not 0: MAV_MISSION_UNSUPPORTED; armed: MAV_MISSION_ERROR;
     `count` above 16: MAV_MISSION_NO_SPACE. `count` 0: the list is emptied and MISSION_ACK MAV_MISSION_ACCEPTED is
     sent (`opaque_id` 0). `count` 1 to 16: an upload starts with MISSION_REQUEST_INT `seq` 0.
   - MISSION_CLEAR_ALL, in this order: `mission_type` not 0 (255, `MAV_MISSION_TYPE_ALL`, included):
     MAV_MISSION_UNSUPPORTED; armed: MAV_MISSION_ERROR. Otherwise the list is emptied (also when it was empty) and
     MISSION_ACK MAV_MISSION_ACCEPTED is sent (`opaque_id` 0).
   - MISSION_REQUEST_LIST: `mission_type` not 0: MAV_MISSION_UNSUPPORTED. Otherwise a download starts with
     MISSION_COUNT (`count`, plan id). Allowed while armed.
   - MISSION_ITEM_INT: if it comes from the owner of the last upload and carries the `seq` of the item that ended that
     upload, the MISSION_ACK that ended it is sent again, unchanged, and nothing else happens (a lost final ACK: the
     station sends its last item again, below). Any other: MISSION_ACK MAV_MISSION_ERROR.
   - MISSION_REQUEST_INT: MISSION_ACK MAV_MISSION_ERROR. MISSION_ACK: ignored.
   The vehicle remembers the last upload that ended at an item (accepted, or refused at an item): its owner, that
   item's `seq` and the MISSION_ACK it sent. It forgets it when it handles any start message from anyone, accepted or
   refused, when that owner's link goes down, and at reboot.
4. In the owner's upload, expecting `seq` k:
   - MISSION_ITEM_INT, in this order: `seq` not k (a repeat or out of order): dropped without any other check, and
     MISSION_REQUEST_INT `seq` k is sent again (`MM`: "the out-of-sequence item is dropped"; not an error);
     `mission_type` not 0: MAV_MISSION_UNSUPPORTED; armed: MAV_MISSION_ERROR; `command` not 16:
     MAV_MISSION_UNSUPPORTED; `frame` not 3 or 6: MAV_MISSION_UNSUPPORTED_FRAME; `x` out of range:
     MAV_MISSION_INVALID_PARAM5_X; `y` out of range: MAV_MISSION_INVALID_PARAM6_Y. An item that passes is stored in the
     staging copy at k; then MISSION_REQUEST_INT `seq` k + 1, or, when k = n - 1, the commit and MISSION_ACK
     MAV_MISSION_ACCEPTED with the new plan id.
   - MISSION_ACK, any `type`: the upload ends as cancelled, staging discarded; no reply.
   - MISSION_REQUEST_INT: dropped.
5. In the owner's download of n waypoints:
   - MISSION_REQUEST_INT: `mission_type` not 0: MAV_MISSION_UNSUPPORTED; `seq` n or above:
     MAV_MISSION_INVALID_SEQUENCE. Otherwise MISSION_ITEM_INT of that `seq`, for any `seq` below n, as often as asked.
   - MISSION_ACK, any `type`: the download ends; no reply.
   - MISSION_ITEM_INT: dropped.
6. Every MISSION_ACK the vehicle sends with a `type` other than MAV_MISSION_ACCEPTED ends the transaction it answers,
   if one runs, with the stored list unchanged; a MAV_MISSION_DENIED to another sender ends nothing.

Results (`MAV_MISSION_RESULT`, `CX`):

| Case | `type` |
|---|---|
| Accepted: an upload committed, a clear, an upload of count 0, the end of a download (from the station) | MAV_MISSION_ACCEPTED (0) |
| Armed: MISSION_COUNT, MISSION_CLEAR_ALL, or an item of an upload | MAV_MISSION_ERROR (1) |
| A MISSION_ITEM_INT (other than a lost final ACK's repeat) or a MISSION_REQUEST_INT with no transaction running | MAV_MISSION_ERROR (1) |
| `count` above 16 | MAV_MISSION_NO_SPACE (4) |
| `mission_type` not 0 (ArduPilot's choice for an unsupported plan type, `GCS_Common.cpp`) | MAV_MISSION_UNSUPPORTED (3) |
| `command` not 16 | MAV_MISSION_UNSUPPORTED (3) |
| `frame` not 3 or 6 | MAV_MISSION_UNSUPPORTED_FRAME (2) |
| latitude out of range | MAV_MISSION_INVALID_PARAM5_X (10) |
| longitude out of range | MAV_MISSION_INVALID_PARAM6_Y (11) |
| A MISSION_REQUEST_INT for a `seq` at or above the count (either side) | MAV_MISSION_INVALID_SEQUENCE (13) |
| Another owner's transaction is running | MAV_MISSION_DENIED (14) |
| Timeout: the sender gave up after its retries (either side) | MAV_MISSION_OPERATION_CANCELLED (15) |

Sequences (S = station, V = vehicle)

Upload of n = 1 to 16 waypoints (`MM`, "Upload a Plan to the Vehicle"):
1. S sends MISSION_COUNT n; its timer starts.
2. V checks it and sends MISSION_REQUEST_INT `seq` 0; its timer starts.
3. S sends MISSION_ITEM_INT `seq` 0.
4. V checks the item, stores it in staging and sends MISSION_REQUEST_INT `seq` 1; steps 3 and 4 repeat for each `seq`
   up to n - 1.
5. V, when item n - 1 passes, commits and sends MISSION_ACK MAV_MISSION_ACCEPTED with the new plan id.
6. S takes the ACK and reads the list back (the verdict, below).

Clear (`MM`, "Clear Plan"):
1. S sends MISSION_CLEAR_ALL for `Command::ClearMission`, or MISSION_COUNT with `count` 0 for `Command::UploadMission`
   of an empty list; its timer starts.
2. V empties the list and sends MISSION_ACK MAV_MISSION_ACCEPTED, `opaque_id` 0.
3. S reads the list back.

Download (`MM`, "Download a Plan from the Vehicle"):
1. S sends MISSION_REQUEST_LIST; its timer starts.
2. V sends MISSION_COUNT n with the plan id.
3. S sends MISSION_REQUEST_INT `seq` 0 (n at least 1); its timer starts.
4. V sends MISSION_ITEM_INT `seq` 0; steps 3 and 4 repeat for each `seq` up to n - 1.
5. S, with every item in (with n = 0 at once after step 2), sends MISSION_ACK MAV_MISSION_ACCEPTED.
6. V ends the download on that ACK.

What the station sends (checked before anything goes out)
- A list of 1 to 65535 waypoints is sent as it is, also above 16: the vehicle decides and refuses with
  MAV_MISSION_NO_SPACE (example 2). A list of more than 65535 waypoints (`count` is a uint16) is refused by the station
  itself.
- Every waypoint is checked before anything is sent: latitude and longitude must be finite and, converted as above,
  inside the vehicle's ranges (`x` -900000000 to 900000000, `y` -1800000000 to 1800000000, both included); the altitude
  must be finite. Any failure refuses the whole operation in the station: nothing is sent, no read-back is made, and the
  reason names the row (the same test as `Waypoint::is_valid` in `crates/marv-gcs/src/missions.rs`).

What the station does with the vehicle's messages
- In an upload: it answers every MISSION_REQUEST_INT with `seq` below n at once with MISSION_ITEM_INT of that `seq`,
  every time it is asked. A MISSION_REQUEST_INT with `seq` n or above (in an upload of `count` 0, every
  MISSION_REQUEST_INT): it sends MISSION_ACK MAV_MISSION_INVALID_SEQUENCE and the upload fails. A MISSION_ACK
  MAV_MISSION_ACCEPTED ends the upload only after the station has sent item n - 1 (for n = 0: after MISSION_COUNT) and
  only when its `opaque_id` equals the plan id the station computed (0 for n = 0); any other ACCEPTED is ignored (a late
  copy of an earlier answer). A MISSION_ACK of any other `type` ends the upload as refused with that `type`.
- In a download: it takes MISSION_ITEM_INT `seq` k when it asked for k; an item of another `seq` is dropped and
  MISSION_REQUEST_INT `seq` k is sent again at once (`MM`: "the expected item should be re-requested by the GCS"). A
  second MISSION_COUNT in the same download is ignored. A MISSION_ACK MAV_MISSION_ACCEPTED is ignored (the vehicle sends
  none in a download, so it is a late copy); any other `type` ends the download as failed with that `type`. The station
  sets no cap on the vehicle's `count`: it asks for every `seq` below it, also above 16, and the verdict compares.
- In a clear: MISSION_ACK MAV_MISSION_ACCEPTED with `opaque_id` 0 ends it accepted; an ACCEPTED with any other
  `opaque_id` is ignored (a late copy of an upload's answer; the empty list's plan id is 0); any other `type` ends it
  refused.
- Every mission message that belongs to no operation of the station is ignored. The station runs one mission
  operation at a time; a second command meanwhile is refused and nothing is sent.

Timeouts and retries (`MM`, "Timeouts and Retries": 1500 ms, 5 retries; `MM`'s 250 ms for plan items is not used here:
1500 ms holds for every message)
- Hand-over: the vehicle hands a message to its link's reply slot (below); the station sends at once, so for the
  station hand-over and send are the same moment. Every timer of either side starts at the hand-over, whatever then
  happens to the message (sent, replaced or dropped in the slot).
- A retry is the same message handed over again when 1500 ms have passed since its last hand-over without its
  response. At most 5 retries (6 hand-overs of that message); 1500 ms after the sixth without the response, the sender
  gives up.
- A new message starts with a retry count of 0 and its own timer. Every message a side sends in answer to a request is
  a new message: in particular the station's MISSION_ITEM_INT `seq` n - 1 sent again because the vehicle asked for it
  again starts a new count of 6 hand-overs (9000 ms in all).
- A re-request (a request sent again because a wrong item arrived) is a hand-over: it restarts that request's 1500 ms
  timer and leaves its retry count as it was (neither counted nor reset).

| Who | Retries | Response that stops it | On giving up |
|---|---|---|---|
| S | MISSION_COUNT | a MISSION_REQUEST_INT or a MISSION_ACK | sends MISSION_ACK MAV_MISSION_OPERATION_CANCELLED; failed, "no answer" |
| S | MISSION_CLEAR_ALL | a MISSION_ACK | the same |
| S | MISSION_ITEM_INT `seq` n - 1 (the last item only) | a MISSION_REQUEST_INT or a MISSION_ACK | the same |
| S | MISSION_REQUEST_LIST | MISSION_COUNT, or a MISSION_ACK other than ACCEPTED | the same |
| S | MISSION_REQUEST_INT `seq` k | MISSION_ITEM_INT `seq` k, or a MISSION_ACK other than ACCEPTED | the same |
| V | MISSION_REQUEST_INT `seq` k (upload) | MISSION_ITEM_INT `seq` k from the owner | sends MISSION_ACK MAV_MISSION_OPERATION_CANCELLED to the owner; the upload ends, the list unchanged |

- A MISSION_ACK the station ignores (above) is no response: it stops no timer.
- Not retried by its sender: an item other than the last (the vehicle asks for it again), MISSION_ACK, and the
  vehicle's MISSION_COUNT and MISSION_ITEM_INT (the station asks again).
- After sending an item other than the last, the station waits for the vehicle: when 9000 ms (6 x 1500 ms, the
  vehicle's whole retry span) pass without a MISSION_REQUEST_INT or a MISSION_ACK that it takes, it gives up as in the
  table.
- In a download the vehicle asks for nothing: when 9000 ms pass after the last mission message from the owner that
  passed the addressing check (the starting MISSION_REQUEST_LIST, every MISSION_REQUEST_INT, a dropped
  MISSION_ITEM_INT, any other), it ends the download and sends MISSION_ACK MAV_MISSION_OPERATION_CANCELLED to the
  owner. Its own sends do not count.
- A frame dropped by the addressing check refreshes no timer of either side.
- A side that receives MISSION_ACK MAV_MISSION_OPERATION_CANCELLED from the other ends the operation (`MM`, "Canceling
  Operations").

Reply pacing
- The vehicle sends its mission messages through one reply slot per link, with an interval of 40 ms on the radio link
  (the radio node's 40 ms parameter-list interval, `_plan/decision-rc-link-20261113.md`:159) and 5 ms on USB (the log
  download slot, the firmware's `src/logdl.rs`:37). On the radio link the slot is a stream of the radio node, so the
  radio's flow control lengthens its interval as it does every stream's, by at most 2040 ms
  (`_plan/decision-rc-link-20261113.md`:155-158); the timers do not change for it.
- When a handed-over message goes out is bounded, not fixed: two mission messages on one link go out at least one
  interval apart, and a handed-over message that is not replaced or dropped goes out no later than one interval after
  its hand-over or after the previous mission message on that link, whichever is later. Sending at once when the slot
  is idle and sending at the first tick of a fixed grid after the hand-over are both legal. No side may depend on the
  choice: every rule here holds for any timing inside these bounds.
- A slot holds one waiting message. A new message to the same destination (system, component) replaces the one
  waiting; a new message to another destination is dropped while one waits (its receiver's retry recovers it). A
  re-request equal to the message waiting therefore adds nothing.
- The station sends at once, without pacing.

Link down
- Vehicle: when the owner's link goes down (USB: down as in section 3; radio: the port leaves its MAVLink lock, where
  the radio node's `on_link_down` runs), the transaction ends at once as cancelled: staging discarded, the stored list
  unchanged, nothing sent. The message waiting in that link's slot is dropped, and the vehicle forgets the last upload
  of an owner on that link (rule 3). A transaction owned on the other link goes on.
- Station: when the session's link closes, a running mission operation ends at once as failed ("link lost"); nothing is
  sent and no read-back is made (as log download, section 11).

The station's verdict
- After an upload or a clear ends in any way other than "link lost" or the station's own refusal before sending
  (above), that is accepted, refused by the vehicle, or given up, the station reads the list back with one download,
  at once.
- An upload or a clear is done only when all hold: the vehicle accepted it (MISSION_ACK MAV_MISSION_ACCEPTED with the
  plan id the station computed, 0 for a clear); the read-back completed; its `count` equals n; its items equal what was
  sent, `seq` by `seq` (`frame` 3, `command` 16, `x`, `y`, and `z` compared as 4 bytes); and its `opaque_id` equals the
  station's plan id. Otherwise it failed, and the read-back list, when one completed, is shown as what the vehicle
  holds.
- The reason shown is the first of these that applies:
  1. "link lost", at any point, the read-back included;
  2. the upload or the clear failed: the vehicle's refusal `type`, "no answer" (given up), or the station's own
     MAV_MISSION_INVALID_SEQUENCE;
  3. the read-back failed: its refusal `type` or "no answer";
  4. the count differs: "the vehicle holds m waypoints, n were sent";
  5. the first `seq` whose item differs;
  6. the plan id differs.
- A download asked for by itself shows the vehicle's list and plan id: complete or failed, no other verdict.

The station's commands are `Command::UploadMission`, `Command::DownloadMission` and `Command::ClearMission`
(Operate-class, section 9); `marv-cli mission upload FILE`, `mission download` and `mission clear` use them (exit
statuses: the README). The simulator (`gcs-sim`) serves this section as the vehicle does, written from this text, with
a reply slot of 5 ms on its link. It runs disarmed: its armed refusals are exercised only by tests calling its library,
with no configuration switch and nothing on the link that arms it. Its configuration can drop chosen mission frames in
either direction.

Worked example 1: a three-point upload with one MISSION_ITEM_INT lost, then the read-back. On the radio link, the
vehicle's list empty before (plan id 0). Times in ms, approximate, and only one legal timing among others (reply
pacing, above): the vehicle's slot on a fixed grid at multiples of 40 ms, the link's own delay left out.
Made-up points in the open ocean:

| `seq` | latitude | longitude | `x` | `y` | `z` (m) |
|---|---|---|---|---|---|
| 0 | 10.0000000 | -30.0000000 | 100000000 | -300000000 | 10.0 |
| 1 | 10.0010000 | -30.0000000 | 100010000 | -300000000 | 12.5 |
| 2 | 10.0010000 | -29.9990000 | 100010000 | -299990000 | 15.0 |

1. t 0, S -> V: MISSION_COUNT `count` 3, `mission_type` 0, `opaque_id` 0; target 1, 1.
2. t 40, V -> S: MISSION_REQUEST_INT `seq` 0; target 255, 190.
3. S -> V: MISSION_ITEM_INT `seq` 0, `frame` 3, `command` 16, `current` 0, `autocontinue` 1, `param1`-`param4` 0.0,
   `x` 100000000, `y` -300000000, `z` 10.0, `mission_type` 0.
4. t 80, V -> S: MISSION_REQUEST_INT `seq` 1 (handed to the slot when item 0 arrived, at about t 45: the vehicle's
   timer starts then).
5. S -> V: MISSION_ITEM_INT `seq` 1 (`x` 100010000, `y` -300000000, `z` 12.5): lost on the link. The station does not
   resend it (not the last item); its 9000 ms wait runs.
6. t 1560, V -> S: MISSION_REQUEST_INT `seq` 1, retry 1 of 5 (the timer of step 4 ran out at about t 1545).
7. S -> V: MISSION_ITEM_INT `seq` 1, the same as in step 5.
8. t 1600, V -> S: MISSION_REQUEST_INT `seq` 2.
9. S -> V: MISSION_ITEM_INT `seq` 2 (`x` 100010000, `y` -299990000, `z` 15.0); the station's timer for the last item
   starts.
10. The vehicle commits the three points; plan id 0x99C3BD49. t 1640, V -> S: MISSION_ACK `type`
    MAV_MISSION_ACCEPTED (0), `mission_type` 0, `opaque_id` 2579742025 (0x99C3BD49; on the wire 49 BD C3 99). It
    equals the station's own plan id, so the upload ends accepted.
11. S -> V: MISSION_REQUEST_LIST `mission_type` 0.
12. t 1680, V -> S: MISSION_COUNT `count` 3, `opaque_id` 2579742025.
13. S and V: MISSION_REQUEST_INT and MISSION_ITEM_INT for `seq` 0, 1, 2 (one per slot, t 1720 to 1800); each item
    with `frame` 3, `command` 16, `current` 0, `autocontinue` 1, `param1`-`param4` 0.0 and the `x`, `y`, `z` above.
14. S -> V: MISSION_ACK MAV_MISSION_ACCEPTED, `opaque_id` 0. Verdict: done (count, items and plan id equal).

The test vector, computed with Python's `zlib.crc32` and with a copy of `crc32` from `record.rs` (both agree):
36 bytes covered, `seq` 0, 1, 2, each `x`, `y`, `z` little-endian:
```
00 E1 F5 05  00 5D 1E EE  00 00 20 41
10 08 F6 05  00 5D 1E EE  00 00 48 41
10 08 F6 05  10 84 1E EE  00 00 70 41
```
CRC-32 = 0x99C3BD49 = plan id. The empty list: 0x00000000.

Worked example 2: a refused 17-point upload. The vehicle holds the three points of example 1 (plan id 0x99C3BD49).
1. S -> V: MISSION_COUNT `count` 17, `mission_type` 0, `opaque_id` 0.
2. V -> S: MISSION_ACK `type` MAV_MISSION_NO_SPACE (4), `mission_type` 0, `opaque_id` 0. No item is asked for; no
   staging is used; no transaction stays open.
3. The upload ends refused ("the vehicle holds at most 16 waypoints"). The station reads back: MISSION_REQUEST_LIST,
   MISSION_COUNT `count` 3 with `opaque_id` 2579742025, the three items of example 1, MISSION_ACK
   MAV_MISSION_ACCEPTED. Verdict: failed, and the vehicle's list is unchanged.

Amendment A: the return item, the height limits and the flown list (2026-11-23; the decision's section 9, A4 to A6, A9,
I6 and I7; approved by the owner in its section 10). APPROVED, NOT BUILT. Firmware 72de0d4 and the station at this
revision do the paragraphs above. Packet H1 builds the vehicle's checks below (`fw-core`; the armed refusal stays),
packet H2 lifts the armed refusal, packet H4 builds the station's side and `gcs-sim`. Each paragraph below replaces, once
built, the one above that it names; the rest of this section holds unchanged. Mixing the two revisions fails closed: a
vehicle of stage 1 refuses the return item (`command` not 16: MAV_MISSION_UNSUPPORTED), and a vehicle with this
amendment refuses a list of stage 1 (its last item is not `command` 20: MAV_MISSION_INVALID). Worked examples 1 and 2
are stage 1; example 3 is this amendment.

The return item
- The last item of every non-empty upload, `seq` n - 1, is the return item: `command` 20
  (`MAV_CMD_NAV_RETURN_TO_LAUNCH`), `frame` 3 or 6, `x` 0, `y` 0, `z` = the return height in m above home; `current` 0,
  `autocontinue` 1, `param1` to `param4` 0.0 (ignored), `mission_type` 0. The owner: "This should be a mission input.
  it may differ from mission to mission."
- Deviation from `CX`: MAV_CMD_NAV_RETURN_TO_LAUNCH has every parameter "Empty" (`CX`:1171-1179); this contract puts the
  return height in its `z`.
- The vehicle never flies it as a leg: after the last waypoint AUTO goes to HOLD at its height, the place dropped (A6).
  Its `z` is used only by RETURN: the height flown home is the larger of it and the present height, latched at entry
  (A7). With no list stored (after a clear or a boot) RETURN flies at the present height (A5; the owner's "2. yes" to
  Q-A2).

Count (replaces "Capacity")
- `count` = n = the number of waypoints + 1. A `count` of 1 to 17 is an upload: up to 16 waypoints at `seq` 0 to n - 2,
  then the return item. `count` 1 is the return item alone: it sets the return height, and Start is refused (section 10).
  A `count` of 18 or more: MAV_MISSION_NO_SPACE. A `count` of 0 still clears the list, as above. The station does not
  check the count itself.

Height (replaces the `z` row of the field table and the rule "never used")
- Frames 3 and 6 both mean `z` in m above home (A6). Home's height is the steering estimate's height at the arming step,
  so a list stored on the ground gives heights above the place of the next arming.
- Every `z`, of the waypoints and of the return item, must be finite and within [`GDN_ALT_MIN`, `GDN_ALT_MAX`], both
  included; otherwise MAV_MISSION_INVALID_PARAM7. The vehicle checks it at upload only, when it checks that item (rule
  4), against the parameters' values in force then (A5). It does not check it again at commit, at Start or in flight.
  It stores the 4 bytes as before and flies them as heights (A6). What a stored list becomes when a limit parameter is
  changed afterwards is an open point (below).
- The two parameters, proposed names (the owner: "lowest 5 meters highest 50 meters (make these parameters)"); group
  `GDN`, with `GDN_SRC` and `GDN_RADIUS`; appended after `GDN_SRC` when built, so no index moves (section 2):

  | Name | Type, unit | Default | Range | Class | Device | Meaning |
  |---|---|---|---|---|---|---|
  | `GDN_ALT_MIN` | F32, m | 5.0 | open point | Live | 0 (system) | the lowest `z` above home of a waypoint or the return item |
  | `GDN_ALT_MAX` | F32, m | 50.0 | open point | Live | 0 (system) | the highest `z` above home of a waypoint or the return item |

  The 5 m floor covers the barometer's sink at speed (2.7 m measured at 6.8 m/s, A6), with 2.3 m to spare.

Checks (replaces rule 3's MISSION_COUNT checks and rule 4's MISSION_ITEM_INT checks after the `seq` check)
- MISSION_COUNT, in this order: `mission_type` not 0: MAV_MISSION_UNSUPPORTED; armed (until H2): MAV_MISSION_ERROR;
  `count` above 17: MAV_MISSION_NO_SPACE. `count` 0 and `count` 1 to 17 as above.
- MISSION_ITEM_INT `seq` k of n, in this order:
  1. `mission_type` not 0: MAV_MISSION_UNSUPPORTED;
  2. armed (until H2): MAV_MISSION_ERROR;
  3. `command`: at k = n - 1, not 20: MAV_MISSION_INVALID; at any other k, 20: MAV_MISSION_INVALID, and neither 16 nor
     20: MAV_MISSION_UNSUPPORTED;
  4. `frame` not 3 or 6: MAV_MISSION_UNSUPPORTED_FRAME;
  5. `x`: a waypoint's out of range, or the return item's not 0: MAV_MISSION_INVALID_PARAM5_X;
  6. `y`: a waypoint's out of range, or the return item's not 0: MAV_MISSION_INVALID_PARAM6_Y;
  7. `z` not finite, below `GDN_ALT_MIN` or above `GDN_ALT_MAX`: MAV_MISSION_INVALID_PARAM7.
  An item that passes is stored in staging as before; the commit on item n - 1 is unchanged.
- Results, in place of the matching rows of the table above (the other rows hold):

  | Case | `type` |
  |---|---|
  | Armed (until H2): MISSION_COUNT, MISSION_CLEAR_ALL, or an item of an upload | MAV_MISSION_ERROR (1) |
  | `count` above 17 | MAV_MISSION_NO_SPACE (4) |
  | A waypoint's `command` neither 16 nor 20 | MAV_MISSION_UNSUPPORTED (3) |
  | The last item's `command` not 20, or `command` 20 at any other `seq` | MAV_MISSION_INVALID (5) |
  | A waypoint's latitude out of range, or the return item's `x` not 0 | MAV_MISSION_INVALID_PARAM5_X (10) |
  | A waypoint's longitude out of range, or the return item's `y` not 0 | MAV_MISSION_INVALID_PARAM6_Y (11) |
  | `z` not finite, or outside [`GDN_ALT_MIN`, `GDN_ALT_MAX`] | MAV_MISSION_INVALID_PARAM7 (12) |

Storage and download (replaces "Storage" and the vehicle's download column)
- Up to 17 items of 12 bytes (204 bytes), in RAM only as before; the return item's `x` and `y` are 0 by rule.
- In a download MISSION_ITEM_INT `seq` n - 1 goes with `command` 20, `frame` 3, `x` 0, `y` 0, `z` the stored 4 bytes,
  `current` 0, `autocontinue` 1, `param1` to `param4` 0.0, `mission_type` 0; every other `seq` as before (`command`
  16, `frame` 3).
- The station's verdict compares it like any item (`command` 20 at `seq` n - 1, 16 elsewhere) and shows the return
  height.

Plan id (replaces "Bytes covered")
- The same algorithm, over all n items, the return item included: 12 x n bytes, `seq` 0 to n - 1, each `x`, `y`, `z`
  little-endian as before (the return item: 8 bytes of 0, then its `z`). The empty list's plan id is still 0. Test
  vector: example 3.

The station's file (adds to "Waypoints and the station's file")
- Row 0 the home placeholder, rows 1 to n - 1 the waypoints, row n the return item: frame 3, command 20, latitude and
  longitude 0, altitude = the return height. Row i is still sent as `seq` i - 1. For three waypoints and a return
  height of 20 m, the last row (fields separated by tabs, as every row):
  ```
  4	0	3	20	0	0	0	0	0.00000000	0.00000000	20	1
  ```
- The station sends n = the number of waypoints + 1 items; its own checks before sending are as above (finite values);
  the height limits are the vehicle's to check.

Armed (A4; packet H2; replaces "armed: MAV_MISSION_ERROR" in rules 3 and 4)
- MISSION_COUNT, MISSION_CLEAR_ALL and the items of an upload are no longer refused while armed: an upload in flight
  redirects the vehicle. The flight core takes only a whole committed list (staging, the atomic commit, then the
  whole-set triple buffer), at its next step; a partly received list never flies (I6).
- A new plan id (an accepted upload or a clear): in AUTO, HOLD with the place dropped; in HOLD, the place dropped, so
  Start begins at `seq` 0; in RETURN and LAND nothing changes (the return height was latched at entry); in PILOT and
  TAKEOVER the shadow guidance starts over.

Worked example 3: the three points of example 1 with a return height of 20.0 m, uploaded to a vehicle with this
amendment and the default limits (5.0 and 50.0). The sequence is example 1's with `count` 4; `seq` 3 is the return item.

| `seq` | `command` | `x` | `y` | `z` (m) |
|---|---|---|---|---|
| 0 | 16 | 100000000 | -300000000 | 10.0 |
| 1 | 16 | 100010000 | -300000000 | 12.5 |
| 2 | 16 | 100010000 | -299990000 | 15.0 |
| 3 | 20 | 0 | 0 | 20.0 |

48 bytes covered:
```
00 E1 F5 05  00 5D 1E EE  00 00 20 41
10 08 F6 05  00 5D 1E EE  00 00 48 41
10 08 F6 05  10 84 1E EE  00 00 70 41
00 00 00 00  00 00 00 00  00 00 A0 41
```
CRC-32 = 0x1862F4A2 = plan id: MISSION_ACK and the read-back's MISSION_COUNT carry `opaque_id` 409138338 (on the wire
A2 F4 62 18). Computed with Python's `zlib.crc32` and with a bitwise copy of the algorithm above (both agree; the same
script gives example 1's 0x99C3BD49). With a return item `z` of 4.0 instead, the upload ends at `seq` 3 with
MAV_MISSION_INVALID_PARAM7 and the vehicle's list is unchanged.

Open points (each would otherwise be guessed by the firmware, the station or `gcs-sim`)
1. A limit parameter changed after a list is stored: the list stays and flies, Start refuses it, or the change clears
   it (A5 checks only at upload).
2. The ranges of `GDN_ALT_MIN` and `GDN_ALT_MAX`, and what a write that makes the lowest above the highest gets.
3. `count` 0: kept here as a clear (A5 gives 1 to 17 for an upload and says nothing of 0).
4. A command whose result is the present state (Start in AUTO, Pause or Abort in HOLD, Land in LAND): ACCEPTED or
   DENIED; it matters because the station sends again after 1.5 s when an ACK is lost; and whether Abort in HOLD drops
   a place kept by Pause.
5. The four flight commands arriving on USB (A10 says the radio link): taken, DENIED, or UNSUPPORTED.
6. The reason for a DENIED flight command: a STATUSTEXT naming the first failed condition (as section 10 does for a
   refusal by parameters), or nothing.
7. BLIND (tier T1, U-1): reported as LAND (6) or with a code of its own.
8. HEARTBEAT base_mode: stays 0, or sets MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1) and MAV_MODE_FLAG_SAFETY_ARMED (128)
   once `custom_mode` carries the mode (the station must also show armed, the decision's section 1 D).
9. What Start reads for "heading monitor OK" and "battery not critical" until the monitor and BATT exist (deny, or
   skip).
10. The station's files of stage 1 (no return row) and where the map tab takes the return height (a default, or none).
11. `gcs-sim`: how it reaches HOLD without CRSF, and which Start conditions (home, T3, heading, battery, switch6 in the
    last CRSF report) it models or sets by configuration.
