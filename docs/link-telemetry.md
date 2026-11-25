# Link contract: device status, RC mapping, ESC telemetry

Part of [MARV-GCS](../README.md) · [link-contract](link-contract.md), sections 8, 12 and 14

Three vehicle reports the station only displays: the health of each device, the sticks as the vehicle maps them, and the
four ESCs. The firmware computes; the station never computes a normalised stick value or an ESC figure beyond the error
percentage.

## 8. `MARV_DEVICE_STATUS`

Defined in `marv-dialect`: id 42421, CRC_EXTRA 231, a fixed payload of 30 bytes (`u32` x 4, `u16` x 5, `u8` x 4, little
endian). The station reads it through the dialect only and names no field or id of it outside the Devices view and the one
instance rule of the message store.

| Rule | Detail |
|---|---|
| One message per device | Carries that device's `device_id` (`MARV_DEVICE`: 1 IMU1, 2 IMU2, 3 BARO, 4 MAG, 5 GNSS, 6 BATT, 7 RC, 9 SD, 10 PSRAM, 11 LED, 12 ESC). Sent unasked by the vehicle (system 1, component 1) |
| Rate | On USB every device at least once a second; on the radio one device per second in turn (N devices: each refreshed every N seconds). The station relies on no exact rate |
| Counters | `samples`, `errors`, `timeouts`, `recoveries`, `attempts`, `overflows`, `config_restored` count since boot and wrap; `time_boot_ms` is the vehicle's clock |

What the station does:

- The message store keeps the latest message per (sender, `device_id`): one row per device with its receive time, count and
  rate. A device is never overwritten by another.
- Devices tab: one row per device seen, in `device_id` order, read only. State, last error and flags as words (`OK`,
  `DEGRADED`, ...; flags `HIL`, `self-test`, `-` for none); `last_sample_age_ms` 65535 is `never`.
- Samples per second = (`samples` now - before) / (`time_boot_ms` now - before) x 1000 from the two latest rows of the same
  device; blank until two rows exist, once after `samples` or `time_boot_ms` went backwards (restart or wrap), and when both
  rows carry the same `time_boot_ms`.
- A row older than 15 s is stale (exactly 15.000 s is not); it is shown stale and kept. The threshold is one constant in the
  view, above the 11 s cycle of 11 devices at one row per second on the radio.
- An unknown `device_id` (0, 8 or above 12), `state` (above 5) or `last_error` (above 6) makes the frame undecodable with
  this dialect version: the checksum is right so it is counted in "Undecodable frames" of the Link tab, listed in Messages as
  `MARV_DEVICE_STATUS[0]` with no fields, and not shown in Devices. A vehicle with a newer dialect shows its new devices as
  missing. A frame of id 42421 with another CRC_EXTRA is a checksum failure and not delivered.
- After an accepted calibration, the vehicle applies the offsets at once and saves only on 245 param1 = 1. It sends no
  `PARAM_VALUE` for values it changes itself, so the station reads them: `PARAM_REQUEST_READ` by name for the offsets (gyro
  `IMU1_GYR_OFFX/Y/Z`, `IMU2_GYR_OFFX/Y/Z`; accelerometer `IMU1_ACC_OFFX/Y/Z`, `IMU2_ACC_OFFX/Y/Z`; compass `MAG_OFFX/Y/Z`),
  and after an accepted 245 param1 = 0 the whole list again (Configure only; a name with a write of its own under way is
  left out; one request each, not repeated). The unsaved bar: [ui-radio-devices](ui-radio-devices.md).

The simulator sends rows for IMU1, IMU2, BARO, MAG, GNSS, BATT and RC, each once per second, state OK, error NONE, counters
advancing (`samples` at the device's nominal rate); it builds the frame from this section, not from `marv-dialect`.

## 12. RC mapping and `MANUAL_CONTROL`

The firmware computes and reports; the station (Radio tab) displays and writes the parameters ([link-parameters](link-parameters.md)).
The mapped sticks are reported, not used: no arming, failsafe, flight mode, control loop, motor or actuator output.
MAVLink is telemetry only; CRSF is for sticks.

**Input.** The 16 microsecond values (880 to 2159 us) of the latest CRSF stick report. For each function f (roll, pitch,
throttle, yaw) the channel is c = `RCMAP_f`; `MIN`, `TRIM`, `MAX`, `REV` (`RCc_REVERSED`) and `DZ` are those of channel c.
Arithmetic on integers, the division in f32. Throttle is a "range" function, the others "angle": fixed by the function.

Roll, pitch, yaw (angle; ArduPilot `pwm_to_angle_dz_trim`, divided by `high_in`):

```
if MAX <= MIN: v = 0
r  = min(max(pwm, MIN), MAX)
hi = TRIM + DZ;  lo = TRIM - DZ;  s = REV ? -1 : 1
if   r > hi and MAX != hi:  v = s * (r - hi) / (MAX - hi)
elif r < lo and lo != MIN:  v = s * (r - lo) / (lo - MIN)
else:                       v = 0
v = min(max(v, -1), 1)
```

Throttle (range; ArduPilot `pwm_to_range_dz`; TRIM is not used):

```
if MAX <= MIN: v = 0
r  = min(max(pwm, MIN), MAX)
if REV: r = MAX - (r - MIN)
lo = MIN + DZ
v  = (r > lo) ? (r - lo) / (MAX - lo) : 0
v  = min(max(v, 0), 1)
```

No cross-parameter check exists (neither MIN < TRIM < MAX nor a unique map): the formulas are defined for every in-range
value and no division by zero is reachable. Two functions on one channel are accepted; both read that channel (as ArduPilot
and PX4).

**Sign convention** (that of `MANUAL_CONTROL` and PX4): pitch positive for stick FORWARD, roll positive for stick RIGHT, yaw
positive for CLOCKWISE (right), throttle 0 at the bottom. The firmware applies no sign change beyond `REV`; the user sets
`RCn_REVERSED` so the station shows these directions.

**Stale or absent.** Without a CRSF stick report younger than 1 s (strictly less) there are no values and no message; no
neutral or held values are invented. Only CRSF reports feed the normalisation; `RC_CHANNELS_OVERRIDE` never does.
`RC_CHANNELS` (raw, 5 Hz) keeps its behaviour: the station needs it for the calibration capture.

**`MANUAL_CONTROL` (69)**, USB only, no dialect change. Default interval 200 000 us (5 Hz), fastest 20 000 us (50 Hz),
changeable by `SET_MESSAGE_INTERVAL`.

| Field | Value |
|---|---|
| `target` | 1 (the vehicle's system id) |
| `x`, `y`, `r` | round(1000 pitch), round(1000 roll), round(1000 yaw) |
| `z` | round(1000 throttle), 0 to 1000 |
| `buttons`, extensions | 0 (`enabled_extensions` = 0) |

Rounding is to nearest, halves away from zero.

**Worked examples** (binding for the firmware, the simulator and the station's tests; output = round(1000 v)):

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

Map example (defaults): channels 1..4 = 1700, 1300, 1500, 1900 give roll 500, pitch -500, throttle 500, yaw 1000; with
`RCMAP_ROLL` = 2 and `RCMAP_PITCH` = 1: roll -500, pitch 500. The simulator has the 44 parameters with these defaults and a
`MANUAL_CONTROL` stream at 5 Hz computed from its own `RC_CHANNELS` and parameters (it reproduces the tables).

## 14. ESC telemetry

The firmware streams DShot300 at 1 kHz to the four ESCs from boot (Stop frames, and throttle on one motor during a motor
test; no arming) and reads their bidirectional replies (eRPM, and extended telemetry after command 13). Neither message is
sent when the flight core's ESC data is older than 1 s, and neither goes on the radio link. Motor order: element 0 = M1
(GPIO39), 1 = M2 (GPIO38), 2 = M3 (GPIO37), 3 = M4 (GPIO36).

**`ESC_STATUS` (291)**, USB, default 10 Hz (100 ms), fastest 50 Hz. `index` 0; `time_usec` = board time of the latest exchange.

| Field | Value |
|---|---|
| `rpm[i]` | Mechanical rpm = eRPM / (`MOT_POLES` / 2), rounded down, from the latest eRPM reply if younger than 100 ms; else 0. 0 also when the motor reports stopped or the ESC is not answering |
| `voltage[i]` | The ESC's extended-telemetry voltage (0.25 V steps) when received within 1 s, else NaN |
| `current[i]` | Always NaN (the AM32 ESCs report values at rest that are not a current) |

**`ESC_INFO` (290)**, USB, default 1 Hz, fastest 10 Hz. `index` 0, `count` 4, `connection_type` 5 (DSHOT), `time_usec` as above.

| Field | Value |
|---|---|
| `counter` | DShot exchanges since boot, modulo 65536; every exchange carries one frame to each ESC |
| `error_count[i]` | Exchanges since boot in which ESC i gave no valid reply (silent, undecodable, foreign signal), wrapping |
| `info` | Bit i set when ESC i gave a valid reply within the last 100 ms (online) |
| `failure_flags[i]` | 0 (AM32 reports no failure states) |
| `temperature[i]` | Extended-telemetry temperature x 100 (cdegC) when received within 1 s, else INT16_MAX |

The error percentage is computed by the station from two consecutive `ESC_INFO`: 100 x Δ`error_count[i]` / Δ`counter` (modulo
65536); blank when they are more than 60 s apart or `time_usec` went back. The arming-tune silence shows as about 30 % errors
for one second after each ESC start.

Device row 12 (ESC), report only: STARTING; ABSENT (no ESC has answered this boot: no battery); OK (all four online);
DEGRADED (1 to 3 online, error TIMEOUT); FAILED (none online after answering, error TIMEOUT; or the ESC task stopped, error
STUCK). `samples` = valid replies, `errors` = invalid or missing, `timeouts` = drop-outs, `attempts` = stream starts. The
simulator sends both messages at these rates, with row 12, as a test double.

## Related

[link-parameters](link-parameters.md) · [ui-radio-devices](ui-radio-devices.md) · [ui-tabs](ui-tabs.md) (Motors) ·
[link-commands](link-commands.md)
