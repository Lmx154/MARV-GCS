#!/usr/bin/env bash
# The ground station against the real board, in three phases. Each phase starts the board's
# `mavlink_demo` program afresh through the mavlink-device repo's tools/run.sh (in the background,
# in its own process group): a fresh start is a board reset and a new USB enumeration. It waits for
# the new USB port, runs, and stops the device program.
#   phase 1: board tests t1..t5 of crates/marv-gcs/tests/board.rs, the recorded .tlog checked with
#            pymavlink, four pictures of the real window;
#   phase 2: the test `listen_only` (a program that only listens);
#   phase 3: the test `no_dtr` (a program that never raises DTR).
# Exit status 0 only if every phase passed. No process is left behind on any exit path.
#
# Usage: tools/board_check.sh <output directory>
# Environment: MAVLINK_DEVICE_REPO (default $HOME/Documents/MARV-V2-drivers/mavlink-device),
#   CHECK_PYTHON (a Python that has pymavlink; default python3; without it that one check is skipped),
#   CHECK_TLOG (the checker script; default tools/check_tlog.py), DISPLAY (default :1).
set -u
out="${1:?usage: tools/board_check.sh <output directory>}"
mkdir -p "$out" || exit 1
out="$(cd "$out" && pwd)"
repo="${MAVLINK_DEVICE_REPO:-$HOME/Documents/MARV-V2-drivers/mavlink-device}"
# The independent reader of the recording: a Python that has pymavlink (CHECK_PYTHON, default python3)
# and the script next to this one. Without pymavlink the check is skipped and said so.
py="${CHECK_PYTHON:-python3}"
checker="${CHECK_TLOG:-$(dirname "$0")/check_tlog.py}"
export DISPLAY="${DISPLAY:-:1}"
cd "$(dirname "$0")/.." || exit 1
rm -f "$out"/*.png "$out"/board.tlog "$out"/*.out "$out"/device-*.log
rm -rf "$out/windows"

fail=0
note() { echo "== $*"; }
bad() { echo "FAIL: $*"; fail=1; }

note "build the tests and the window"
cargo test -p marv-gcs --test board --no-run >"$out/build.out" 2>&1 || { cat "$out/build.out"; exit 1; }
cargo build -q -p marv-gcs >>"$out/build.out" 2>&1 || { cat "$out/build.out"; exit 1; }

# The build folder is shared by the driver repos: make the crates rebuild from their sources.
touch "$repo/src/lib.rs" "$repo/hil/src/lib.rs" "$repo/hil/src/bin/mavlink_demo.rs"
[ -f "$repo/../usb-cdc/src/lib.rs" ] && touch "$repo/../usb-cdc/src/lib.rs"


dev_pid=""
descendants() {
  local c
  for c in $(pgrep -P "$1" 2>/dev/null); do echo "$c"; descendants "$c"; done
}
# tools/run.sh runs probe-rs under `timeout`, which moves itself to a process group of its own: the
# group kill alone would leave it (and the probe lock it holds) behind, so every descendant is
# collected first and stopped too.
stop_device() {
  local pids p n
  if [ -z "$dev_pid" ] || ! kill -0 "$dev_pid" 2>/dev/null; then
    dev_pid=""
    return 1
  fi
  pids="$dev_pid $(descendants "$dev_pid")"
  kill -TERM -- "-$dev_pid" 2>/dev/null
  kill -TERM $pids 2>/dev/null
  for n in $(seq 1 50); do
    p=""
    for pid in $pids; do kill -0 "$pid" 2>/dev/null && p="$p $pid"; done
    [ -z "$p" ] && break
    sleep 0.1
  done
  [ -z "$p" ] || kill -KILL $p 2>/dev/null
  wait "$dev_pid" 2>/dev/null
  dev_pid=""
  return 0
}
trap 'stop_device' EXIT
trap 'stop_device; exit 130' INT TERM HUP

port=""
cur_log=""
# start_device <phase name>: touches the sources, starts the device program afresh, waits for its
# start-up line and for a USB port created after the start. Sets $port; returns 1 on failure.
start_device() {
  local phase="$1" stamp="$out/stamp-$1" log="" p props
  # The build folder is shared by the driver repos: make the crates rebuild from their sources.
  touch "$repo/src/lib.rs" "$repo/hil/src/lib.rs" "$repo/hil/src/bin/mavlink_demo.rs"
  [ -f "$repo/../usb-cdc/src/lib.rs" ] && touch "$repo/../usb-cdc/src/lib.rs"
  note "$phase: start the device program"
  mkdir -p "$repo/hil/logs"
  touch "$stamp"
  sleep 1
  setsid "$repo/tools/run.sh" mavlink_demo 300 >"$out/device-$phase.out" 2>&1 &
  dev_pid=$!
  for _ in $(seq 1 1500); do
    log="$(find "$repo/hil/logs" -name 'mavlink_demo-*.log' -newer "$stamp" 2>/dev/null | sort | tail -n 1)"
    if [ -n "$log" ] && grep -q "size_of run future" "$log" 2>/dev/null; then break; fi
    if ! kill -0 "$dev_pid" 2>/dev/null; then break; fi
    log=""
    sleep 0.1
  done
  if [ -z "$log" ]; then
    echo "the device program did not start:"; cat "$out/device-$phase.out"; return 1
  fi
  cur_log="$log"
  cp "$log" "$out/device-$phase-start.log"
  port=""
  for _ in $(seq 1 100); do
    for p in /dev/ttyACM*; do
      [ -e "$p" ] || continue
      [ "$(stat -c %Z "$p")" -ge "$(stat -c %Z "$stamp")" ] || continue
      props="$(udevadm info -q property -n "$p" 2>/dev/null)"
      if grep -q '^ID_VENDOR_ID=c0de$' <<<"$props" && grep -q '^ID_MODEL_ID=cafe$' <<<"$props" \
         && grep -q '^ID_USB_MODEL=MARV_V2_link_demo$' <<<"$props"; then port="$p"; fi
    done
    [ -n "$port" ] && break
    sleep 0.1
  done
  [ -n "$port" ] || { echo "no new USB port with VID c0de / PID cafe / product MARV V2 link demo"; return 1; }
  note "$phase: port $port"
}

# stop_device_phase <phase name>: stops the device program and checks its log.
stop_phase() {
  local phase="$1"
  note "$phase: stop the device program"
  if ! stop_device; then
    bad "$phase: the device program had ended before the checks were done"
  fi
  cp "$cur_log" "$out/device-$phase-end.log" 2>/dev/null
  grep -E 'gate open' "$out/device-$phase-end.log" 2>/dev/null | sed "s/^/DEVICE $phase: /"
  if grep -qE 'panicked|ERROR' "$out/device-$phase-end.log" 2>/dev/null; then bad "$phase: the device log has a panic or an error"; fi
}

run_tests() { # <phase> <test args...>
  local phase="$1"; shift
  BOARD_CHECK_DIR="$out" cargo test -p marv-gcs --test board -- --ignored --test-threads 1 --nocapture "$@" >"$out/board_tests-$phase.out" 2>&1
  local status=$?
  grep -E '^(BOARD|test |test result)' "$out/board_tests-$phase.out"
  [ "$status" -eq 0 ] || { bad "$phase: board tests exit $status"; grep -B2 -A12 "panicked" "$out/board_tests-$phase.out" | head -60; }
}

# ---- phase 1
if start_device phase1; then
  note "phase1: board tests"
  run_tests phase1 t1_ t2_ t3_ t4_ t5_

  note "phase1: pymavlink on the recording"
  if [ ! -s "$out/board.tlog" ]; then
    bad "no board.tlog"
  elif ! "$py" -c 'import pymavlink' 2>/dev/null; then
    echo "[SKIP] the recording was not read by pymavlink: $py has no pymavlink (set CHECK_PYTHON to one that has)"
  else
    "$py" "$checker" "$out/board.tlog" >"$out/pycheck.out" 2>&1
    py_status=$?
    cat "$out/pycheck.out"
    [ "$py_status" -eq 0 ] || bad "pymavlink check exit $py_status"
    gcs_records="$(sed -n 's/.*BOARD tlog .* records=\([0-9]*\) .*/\1/p' "$out/board_tests-phase1.out" | tail -n 1)"
    py_records="$(sed -n 's/^pymavlink: \([0-9]*\) messages decoded, BAD_DATA \([0-9]*\)$/\1/p' "$out/pycheck.out")"
    py_bad="$(sed -n 's/^pymavlink: \([0-9]*\) messages decoded, BAD_DATA \([0-9]*\)$/\2/p' "$out/pycheck.out")"
    echo "records: ground station $gcs_records, pymavlink $py_records, BAD_DATA $py_bad"
    [ -n "$gcs_records" ] && [ "$gcs_records" = "$py_records" ] || bad "record counts differ"
    [ "$py_bad" = "0" ] || bad "BAD_DATA is not 0"
    for name in HEARTBEAT HIGHRES_IMU PARAM_VALUE; do
      grep -q "$name" "$out/pycheck.out" || bad "$name not in the pymavlink output"
    done
  fi

  note "phase1: pictures of the window"
  mkdir -p "$out/windows"
  for view in sensors messages parameters link; do
    cargo run -q -p marv-gcs -- --connect "serial:$port" --view "$view" --log-dir "$out/windows" \
      --screenshot "$out/$view.png" --after 6 >"$out/window-$view.out" 2>&1
    status=$?
    if [ "$status" -ne 0 ] || [ ! -s "$out/$view.png" ]; then
      bad "window $view: exit $status"; cat "$out/window-$view.out"
    else
      echo "picture $out/$view.png"
    fi
  done
  stop_phase phase1
else
  bad "phase1: the device program did not start"; stop_device
fi

# ---- phase 2
if start_device phase2; then
  note "phase2: listen_only"
  run_tests phase2 --exact listen_only
  stop_phase phase2
else
  bad "phase2: the device program did not start"; stop_device
fi

# ---- phase 3
if start_device phase3; then
  note "phase3: no_dtr"
  run_tests phase3 --exact no_dtr
  stop_phase phase3
else
  bad "phase3: the device program did not start"; stop_device
fi

trap - EXIT INT TERM HUP
if [ "$fail" -eq 0 ]; then echo "board_check: PASS"; else echo "board_check: FAIL"; fi
exit "$fail"
