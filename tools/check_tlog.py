"""Read a .tlog with pymavlink (an independent MAVLink implementation) and with a raw byte parser,
and print what each found, so the two can be compared with what the ground station reported."""
import os, struct, sys, collections
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

path = sys.argv[1]
raw = open(path, "rb").read()

# 1. raw walk by the file-format rule only: 8-byte big-endian time, then one frame
pos, raw_ids, raw_dir, times = 0, collections.Counter(), collections.Counter(), []
while pos < len(raw):
    (t,) = struct.unpack(">Q", raw[pos:pos + 8]); pos += 8
    assert raw[pos] == 0xFD, f"not a MAVLink 2 frame at {pos}"
    ln, incompat = raw[pos + 1], raw[pos + 2]
    msgid = raw[pos + 7] | raw[pos + 8] << 8 | raw[pos + 9] << 16
    raw_ids[msgid] += 1; raw_dir[raw[pos + 5]] += 1; times.append(t)
    pos += 12 + ln + (13 if incompat & 1 else 0)
print(f"raw walk: {len(times)} records, ends on a record boundary: {pos == len(raw)}, "
      f"time never decreases: {all(a <= b for a, b in zip(times, times[1:]))}, "
      f"span {(times[-1] - times[0]) / 1e6:.2f} s")
print("raw walk, frames by sending system:", dict(sorted(raw_dir.items())))

# 2. pymavlink: its own framing, CRC table and field decoding
m = mavutil.mavlink_connection(path, dialect="common", robust_parsing=False)
names, by_sys, bad = collections.Counter(), collections.Counter(), 0
imu = collections.defaultdict(list); first = {}
while True:
    msg = m.recv_match()
    if msg is None:
        break
    t = msg.get_type()
    if t == "BAD_DATA":
        bad += 1; continue
    names[t] += 1; by_sys[msg.get_srcSystem()] += 1
    first.setdefault((msg.get_srcSystem(), t), msg)
    if t == "HIGHRES_IMU":
        imu[msg.id].append(msg)
print(f"pymavlink: {sum(names.values())} messages decoded, BAD_DATA {bad}")
print("pymavlink, by sending system:", dict(sorted(by_sys.items())))
print("pymavlink, by type:", dict(sorted(names.items())))
ids = {0: "HEARTBEAT", 24: "GPS_RAW_INT", 29: "SCALED_PRESSURE", 105: "HIGHRES_IMU", 253: "STATUSTEXT",
       22: "PARAM_VALUE", 21: "PARAM_REQUEST_LIST", 20: "PARAM_REQUEST_READ", 23: "PARAM_SET",
       76: "COMMAND_LONG", 77: "COMMAND_ACK", 148: "AUTOPILOT_VERSION"}
raw_named = collections.Counter({ids.get(k, f"id{k}"): v for k, v in raw_ids.items()})
print("same counts as the raw walk:", dict(raw_named) == dict(names))

import math
for i in sorted(imu):
    a = imu[i][len(imu[i]) // 2]
    f = lambda x: "NaN" if math.isnan(x) else f"{x:.3f}"
    print(f"HIGHRES_IMU id {i}: {len(imu[i])} msgs  acc=({f(a.xacc)},{f(a.yacc)},{f(a.zacc)})  "
          f"gyro=({f(a.xgyro)},{f(a.ygyro)},{f(a.zgyro)})  mag=({f(a.xmag)},{f(a.ymag)},{f(a.zmag)})  "
          f"temp={f(a.temperature)}  fields_updated=0x{a.fields_updated:04x}")
for (sysid, t), msg in sorted(first.items(), key=lambda kv: (kv[0][0], kv[0][1])):
    if t in ("HEARTBEAT", "COMMAND_LONG", "PARAM_REQUEST_LIST", "AUTOPILOT_VERSION", "COMMAND_ACK", "STATUSTEXT", "GPS_RAW_INT", "SCALED_PRESSURE"):
        d = msg.to_dict(); d.pop("mavpackettype", None)
        keep = {k: v for k, v in d.items() if k in ("type", "autopilot", "base_mode", "system_status", "mavlink_version",
                "command", "param1", "target_system", "target_component", "capabilities", "result", "severity", "text",
                "fix_type", "lat", "lon", "alt", "satellites_visible", "press_abs", "temperature")}
        print(f"first {t} from system {sysid} comp {msg.get_srcComponent()}: {keep}")
pv = [mm for (s, t), mm in first.items() if t == "PARAM_VALUE"]
if pv:
    print("first PARAM_VALUE:", pv[0].param_id, pv[0].param_type, pv[0].param_count, pv[0].param_index,
          "raw bytes", struct.pack("<f", pv[0].param_value).hex())
