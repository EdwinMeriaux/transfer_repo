#!/usr/bin/env bash
set -eo pipefail

DURATION_S="${1:-60}"
BLUEBOAT_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BLUEBOAT_WORKSPACE="${BLUEBOAT_WORKSPACE:-$HOME/blueboat_ws}"
PROFILE="$HOME/.config/blueboat/alberto/controller.yaml"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$BLUEBOAT_REPO/artifacts/lab-shadow-$STAMP"
CONTROLLER_PID=""
SOURCES_PID=""
CLEANED=false

if ! [[ "$DURATION_S" =~ ^[0-9]+$ ]] || (( DURATION_S < 20 || DURATION_S > 300 )); then
  echo "Usage: $0 [duration_seconds: 20..300]" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$BLUEBOAT_REPO/.venv/bin/activate"
source "$BLUEBOAT_WORKSPACE/install/setup.bash"

stop_process_group() {
  local pid="$1"
  [ -n "$pid" ] || return
  kill -0 -- "-$pid" 2>/dev/null || {
    wait "$pid" 2>/dev/null || true
    return
  }

  kill -INT -- "-$pid" 2>/dev/null || true
  for attempt in $(seq 1 10); do
    kill -0 -- "-$pid" 2>/dev/null || break
    sleep 0.5
  done
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -KILL -- "-$pid" 2>/dev/null || true
    sleep 1
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  [ "$CLEANED" = true ] && return
  CLEANED=true
  set +e

  stop_process_group "$CONTROLLER_PID"
  stop_process_group "$SOURCES_PID"
}
trap cleanup EXIT INT TERM

mkdir -p "$OUT/controller"
touch "$OUT/controller-start.marker"

printf '\n=== LAB SHADOW SAFETY PREFLIGHT ===\n'
printf 'gps_injection=false\nsynthetic_ros_odometry=/blueboat/lab/odometry\nactuation_enabled=false\n'

STATE="$(timeout 8 ros2 topic echo --once /mavros/state 2>/dev/null || true)"
OVERRIDE="$(ros2 topic info /mavros/rc/override 2>/dev/null || true)"
printf '%s\n%s\n' "$STATE" "$OVERRIDE"

printf '%s\n' "$STATE" | grep -q 'connected: true' || {
  echo 'STOP: MAVROS is not connected.' >&2
  exit 1
}
printf '%s\n' "$STATE" | grep -q 'armed: false' || {
  echo 'STOP: the physical boat must remain disarmed.' >&2
  exit 1
}
printf '%s\n' "$STATE" | grep -q 'mode: MANUAL' || {
  echo 'STOP: MANUAL mode was not confirmed.' >&2
  exit 1
}
printf '%s\n' "$OVERRIDE" | grep -q 'Publisher count: 0' || {
  echo 'STOP: an RC override publisher already exists.' >&2
  exit 1
}

if pgrep -af 'blueboat_ppo_controller/ppo_node' > "$OUT/existing-controller.txt"; then
  cat "$OUT/existing-controller.txt"
  echo 'STOP: a PPO controller process already exists.' >&2
  exit 1
fi

for topic in /blueboat/lab/odometry /blueboat/cmd_vel; do
  INFO="$(ros2 topic info "$topic" 2>/dev/null || true)"
  if printf '%s\n' "$INFO" | grep -Eq 'Publisher count: [1-9]'; then
    printf '%s\n' "$INFO"
    echo "STOP: an existing publisher owns $topic." >&2
    exit 1
  fi
done

IMU="$(timeout 8 ros2 topic echo --once /mavros/imu/data 2>/dev/null || true)"
printf '%s\n' "$IMU" > "$OUT/imu-before.txt"
printf '%s\n' "$IMU" | grep -q 'frame_id: base_link' || {
  echo 'STOP: live IMU frame base_link was not confirmed.' >&2
  exit 1
}

printf '\n=== START ISOLATED ZERO-COMMAND SOURCES ===\n'
setsid python "$BLUEBOAT_REPO/scripts/lab_shadow_sources.py" \
  > "$OUT/sources.log" 2>&1 &
SOURCES_PID=$!

ODOM_READY=false
for attempt in $(seq 1 20); do
  sleep 0.25
  if timeout 2 ros2 topic echo --once /blueboat/lab/odometry \
      > "$OUT/synthetic-odometry-sample.txt" 2>/dev/null; then
    ODOM_READY=true
    break
  fi
done
[ "$ODOM_READY" = true ] || {
  echo 'STOP: isolated synthetic odometry did not start.' >&2
  exit 1
}

printf '\n=== START E274 SHADOW CONTROLLER ===\n'
setsid ros2 run blueboat_ppo_controller ppo_node --ros-args \
  --params-file "$PROFILE" \
  -p mode:=shadow \
  -p enable_actuators:=false \
  -p deployment_target:=hardware \
  -p odometry_topic:=/blueboat/lab/odometry \
  -p log_directory:="$OUT/controller" \
  > "$OUT/controller-console.log" 2>&1 &
CONTROLLER_PID=$!

sleep 3
kill -0 "$CONTROLLER_PID" 2>/dev/null || {
  cat "$OUT/controller-console.log"
  echo 'STOP: shadow controller exited during startup.' >&2
  exit 1
}

printf '\n=== MONITOR NON-ACTUATING SHADOW (%ss) ===\n' "$DURATION_S"
for elapsed in $(seq 5 5 "$DURATION_S"); do
  sleep 5

  STATE_NOW="$(timeout 5 ros2 topic echo --once /mavros/state 2>/dev/null || true)"
  OVERRIDE_NOW="$(ros2 topic info /mavros/rc/override 2>/dev/null || true)"
  STATUS="$(timeout 5 ros2 topic echo --once /blueboat/diagnostics --field data 2>/dev/null | tail -1 || true)"
  printf 'elapsed=%3ss status=%s\n' "$elapsed" "${STATUS:-unavailable}"

  printf '%s\n' "$STATE_NOW" | grep -q 'armed: false' || {
    echo 'STOP: disarmed state was lost.' >&2
    exit 1
  }
  printf '%s\n' "$OVERRIDE_NOW" | grep -q 'Publisher count: 0' || {
    echo 'STOP: an RC override publisher appeared.' >&2
    exit 1
  }
  kill -0 "$CONTROLLER_PID" 2>/dev/null || {
    echo 'STOP: shadow controller exited early.' >&2
    exit 1
  }
done

printf '\n=== CONTROLLED SHUTDOWN ===\n'
cleanup
trap - EXIT INT TERM

EVIDENCE="$(find "$OUT/controller" -maxdepth 1 -type f -name 'controller-*.jsonl' -newer "$OUT/controller-start.marker" -print | head -1)"
[ -n "$EVIDENCE" ] || {
  echo 'STOP: controller evidence file was not produced.' >&2
  exit 1
}

printf '\n=== SHADOW EVIDENCE ANALYSIS ===\n'
python - "$EVIDENCE" "$OUT/summary.json" <<'PY'
import collections
import json
import math
import sys
from pathlib import Path

evidence = Path(sys.argv[1])
summary_path = Path(sys.argv[2])
rows = []
for line in evidence.read_text().splitlines():
    try:
        rows.append(json.loads(line))
    except json.JSONDecodeError:
        pass

operational = [row for row in rows if row.get('status') != 'shutdown']
shadow = [row for row in operational if row.get('status') == 'shadow']
if operational:
    end = max(float(row['monotonic_s']) for row in operational)
    tail = [row for row in operational if float(row['monotonic_s']) >= end - 20.0]
else:
    tail = []

non_shadow_tail = collections.Counter(
    row.get('status', 'missing') for row in tail if row.get('status') != 'shadow'
)
inference = sorted(
    float(row['inference_s']) for row in shadow
    if math.isfinite(float(row.get('inference_s', float('nan'))))
)
p99 = inference[min(len(inference) - 1, int(0.99 * len(inference)))] if inference else None
summary = {
    'mode': 'shadow',
    'policy_backend': 'e274_6_5b',
    'gps_injected': False,
    'synthetic_ros_odometry': True,
    'physical_hardware_actuated': False,
    'operational_records': len(operational),
    'shadow_records': len(shadow),
    'last_20s_records': len(tail),
    'last_20s_non_shadow_statuses': dict(non_shadow_tail),
    'inference_p99_ms': None if p99 is None else 1000.0 * p99,
}
summary['passed'] = (
    len(tail) >= 500
    and not non_shadow_tail
    and p99 is not None
    and p99 < 0.020
)
summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
print(json.dumps(summary, indent=2, sort_keys=True))
if not summary['passed']:
    raise SystemExit('STOP: lab shadow acceptance gates failed.')
PY

printf '\n=== FINAL SAFETY ===\n'
STATE_AFTER="$(timeout 8 ros2 topic echo --once /mavros/state 2>/dev/null || true)"
OVERRIDE_AFTER="$(ros2 topic info /mavros/rc/override 2>/dev/null || true)"
printf '%s\n%s\n' "$STATE_AFTER" "$OVERRIDE_AFTER"
printf '%s\n' "$STATE_AFTER" | grep -q 'armed: false'
printf '%s\n' "$OVERRIDE_AFTER" | grep -q 'Publisher count: 0'

REMAINING="$(pgrep -af 'blueboat_ppo_controller/ppo_node|blueboat_lab_shadow_sources' || true)"
if [ -n "$REMAINING" ]; then
  printf '%s\n' "$REMAINING"
  echo 'STOP: a lab-shadow process remains after controlled cleanup.' >&2
  exit 1
fi

for topic in /blueboat/lab/odometry /blueboat/cmd_vel; do
  INFO="$(ros2 topic info "$topic" 2>/dev/null || true)"
  if printf '%s\n' "$INFO" | grep -Eq 'Publisher count: [1-9]'; then
    printf '%s\n' "$INFO"
    echo "STOP: a test publisher remains on $topic." >&2
    exit 1
  fi
done

find "$OUT" -type f ! -name SHA256SUMS -print0 \
  | sort -z \
  | xargs -0 sha256sum > "$OUT/SHA256SUMS"

printf '\nPASS: indoor e274 lab shadow completed without GPS injection or actuation.\n'
printf 'evidence_directory=%s\n' "$OUT"

