#!/usr/bin/env bash
set -eo pipefail

DURATION_S="${1:-60}"
BLUEBOAT_WORKSPACE="${BLUEBOAT_WORKSPACE:-$HOME/blueboat_ws}"

if ! [[ "$DURATION_S" =~ ^[0-9]+$ ]] || (( DURATION_S < 10 || DURATION_S > 600 )); then
  echo "Usage: $0 [duration_seconds: 10..600]" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$BLUEBOAT_WORKSPACE/install/setup.bash"

STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="${BLUEBOAT_HARDWARE_EVIDENCE:-$PWD/artifacts/hardware-telemetry-$STAMP}"
mkdir -p "$OUT"

topics=(
  /mavros/imu/data_raw
  /mavros/imu/data
  /mavros/local_position/odom
  /mavros/rc/out
  /mavros/state
)

printf '\n=== READ-ONLY HARDWARE PREFLIGHT ===\n'
state="$(timeout 8 ros2 topic echo --once /mavros/state 2>/dev/null || true)"
override_info="$(ros2 topic info /mavros/rc/override 2>/dev/null || true)"
printf '%s\n%s\n' "$state" "$override_info"
printf '%s\n' "$state" > "$OUT/mavros-state-before.txt"
printf '%s\n' "$override_info" > "$OUT/override-info-before.txt"

if ! grep -q 'connected: true' <<<"$state"; then
  echo 'STOP: MAVROS is not connected.' >&2
  exit 1
fi
if ! grep -q 'armed: false' <<<"$state"; then
  echo 'STOP: this read-only characterization requires the boat to be disarmed.' >&2
  exit 1
fi
if ! grep -q 'Publisher count: 0' <<<"$override_info"; then
  echo 'STOP: an RC override publisher exists.' >&2
  exit 1
fi

{
  printf 'schema=1\n'
  printf 'target=hardware\n'
  printf 'duration_s=%s\n' "$DURATION_S"
  printf 'started_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf 'hostname=%s\n' "$(hostname)"
  printf 'kernel=%s\n' "$(uname -srmo)"
  printf 'ros_distro=%s\n' "${ROS_DISTRO:-unknown}"
} > "$OUT/session.txt"

ros2 node list > "$OUT/ros-nodes.txt"
ros2 topic list -t > "$OUT/ros-topics.txt"

printf '\n=== ENDPOINT DETAILS ===\n'
for topic in "${topics[@]}" /mavros/rc/override; do
  safe="${topic#/}"
  safe="${safe//\//_}"
  ros2 topic info -v "$topic" > "$OUT/info-$safe.txt" 2>&1 || true
  printf 'captured=%s\n' "$topic"
done

printf '\n=== SIMULTANEOUS RATE/GAP CAPTURE (%ss) ===\n' "$DURATION_S"
pids=()
for topic in "${topics[@]}"; do
  safe="${topic#/}"
  safe="${safe//\//_}"
  timeout "$DURATION_S" ros2 topic hz "$topic" \
    > "$OUT/rate-$safe.txt" 2>&1 &
  pids+=("$!")
done

for pid in "${pids[@]}"; do
  status=0
  wait "$pid" || status=$?
  if (( status != 0 && status != 124 )); then
    echo "STOP: rate monitor pid=$pid exited with status=$status" >&2
    exit 1
  fi
done

python3 - "$OUT" <<'PY'
import json
import re
import sys
from pathlib import Path

directory = Path(sys.argv[1])
summary = {"schema": 1, "target": "hardware", "streams": {}}

for path in sorted(directory.glob("rate-*.txt")):
    text = path.read_text(errors="replace")
    rates = [float(value) for value in re.findall(r"average rate:\s*([0-9.]+)", text)]
    gaps = [float(value) for value in re.findall(r"max:\s*([0-9.]+)s", text)]
    summary["streams"][path.stem.removeprefix("rate-")] = {
        "last_average_rate_hz": rates[-1] if rates else None,
        "minimum_reported_rate_hz": min(rates) if rates else None,
        "maximum_observed_gap_s": max(gaps) if gaps else None,
        "rate_reports": len(rates),
    }

(directory / "summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY

printf '\n=== FINAL SAFETY ===\n'
state_after="$(timeout 8 ros2 topic echo --once /mavros/state 2>/dev/null || true)"
override_after="$(ros2 topic info /mavros/rc/override 2>/dev/null || true)"
printf '%s\n%s\n' "$state_after" "$override_after"
printf '%s\n' "$state_after" > "$OUT/mavros-state-after.txt"
printf '%s\n' "$override_after" > "$OUT/override-info-after.txt"

if ! grep -q 'armed: false' <<<"$state_after"; then
  echo 'STOP: final disarmed state was not confirmed.' >&2
  exit 1
fi
if ! grep -q 'Publisher count: 0' <<<"$override_after"; then
  echo 'STOP: an RC override publisher appeared.' >&2
  exit 1
fi

find "$OUT" -maxdepth 1 -type f ! -name SHA256SUMS -print0 \
  | sort -z \
  | xargs -0 sha256sum > "$OUT/SHA256SUMS"

mkdir -p "$PWD/artifacts"
printf '%s\n' "$OUT" > "$PWD/artifacts/latest_hardware_telemetry.txt"

printf '\nPASS: read-only hardware telemetry evidence captured.\n'
printf 'evidence_directory=%s\n' "$OUT"
printf 'No arming, mode change, parameter write, or actuator command was issued.\n'
