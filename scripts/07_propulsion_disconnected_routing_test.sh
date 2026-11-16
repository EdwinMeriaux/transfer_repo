#!/usr/bin/env bash
set -eo pipefail

BLUEBOAT_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BLUEBOAT_WORKSPACE="${BLUEBOAT_WORKSPACE:-$HOME/blueboat_ws}"
PROFILE="$HOME/.config/blueboat/alberto/motor_interface.json"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$BLUEBOAT_REPO/artifacts/propulsion-disconnected-routing-$STAMP"
CLEANED=false

source /opt/ros/humble/setup.bash
source "$BLUEBOAT_REPO/.venv/bin/activate"
source "$BLUEBOAT_WORKSPACE/install/setup.bash"

cleanup() {
  [ "$CLEANED" = true ] && return
  CLEANED=true
  set +e
  timeout 3 ros2 topic pub --once \
    /mavros/rc/override mavros_msgs/msg/OverrideRCIn \
    '{channels: [0, 65535, 0, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535]}' \
    >/dev/null 2>&1
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

printf '\n=== PHYSICAL SAFETY CONFIRMATION ===\n'
printf 'This test publishes bounded RC overrides and requires manual arming.\n'
printf 'The propulsion battery/ESC power must be physically disconnected.\n'
read -r -p 'Type PROPULSION_DISCONNECTED to continue: ' CONFIRMATION
[ "$CONFIRMATION" = PROPULSION_DISCONNECTED ] || {
  echo 'STOP: physical disconnection was not confirmed.' >&2
  exit 1
}

mkdir -p "$OUT"

printf '\n=== LIVE PARAMETER RECAPTURE ===\n'
"$BLUEBOAT_REPO/scripts/alberto.sh" capture | tee "$OUT/capture-output.txt"
LATEST_CAPTURE="$(find "$HOME/.config/blueboat/alberto" -maxdepth 1 -type f -name 'live-capture-*.json' -print | sort | tail -1)"
python - "$LATEST_CAPTURE" <<'PY'
import json
import sys

capture = json.load(open(sys.argv[1]))
actual = capture['actual_parameters']
expected = {
    'RC_OVERRIDE_TIME': 0.5,
    'FS_ACTION': 2,
    'MOT_THST_ASYM': 1,
}
selected = {name: actual.get(name) for name in expected}
print({'live_safeguards': selected})
if capture.get('parameter_differences'):
    raise SystemExit('STOP: live parameters differ from the imported export.')
if selected != expected:
    raise SystemExit(f'STOP: required safeguards are not live: {selected!r}')
PY

printf '\n=== DISARMED PREFLIGHT ===\n'
STATE="$(timeout 8 ros2 topic echo --once /mavros/state 2>/dev/null || true)"
OVERRIDE="$(ros2 topic info /mavros/rc/override 2>/dev/null || true)"
printf '%s\n%s\n' "$STATE" "$OVERRIDE"
printf '%s\n' "$STATE" | grep -q 'connected: true'
printf '%s\n' "$STATE" | grep -q 'armed: false'
printf '%s\n' "$STATE" | grep -q 'mode: MANUAL'
printf '%s\n' "$OVERRIDE" | grep -q 'Publisher count: 0'

printf '\n=== MANUAL ARM REQUIRED ===\n'
printf 'Alberto: arm the Pixhawk using the normal approved operator control.\n'
printf 'Propulsion must remain physically disconnected. Waiting up to 45 seconds.\n'
ARMED=false
for attempt in $(seq 1 45); do
  sleep 1
  STATE="$(timeout 3 ros2 topic echo --once /mavros/state 2>/dev/null || true)"
  if printf '%s\n' "$STATE" | grep -q 'armed: true' \
      && printf '%s\n' "$STATE" | grep -q 'mode: MANUAL'; then
    ARMED=true
    break
  fi
done
[ "$ARMED" = true ] || {
  echo 'STOP: armed MANUAL state was not confirmed.' >&2
  exit 1
}

printf '\n=== RUN BOUNDED ±20 PWM ROUTING PROBE ===\n'
set +e
python "$BLUEBOAT_REPO/scripts/propulsion_disconnected_routing_probe.py" \
  --interface "$PROFILE" \
  --output-directory "$OUT/probe" \
  --delta-pwm 20 \
  2>&1 | tee "$OUT/probe-console.log"
PROBE_STATUS=${PIPESTATUS[0]}
set -e

printf '\n=== MANUAL DISARM REQUIRED ===\n'
printf 'Alberto: disarm using the normal approved operator control.\n'
DISARMED=false
for attempt in $(seq 1 45); do
  sleep 1
  STATE="$(timeout 3 ros2 topic echo --once /mavros/state 2>/dev/null || true)"
  if printf '%s\n' "$STATE" | grep -q 'armed: false'; then
    DISARMED=true
    break
  fi
done
[ "$DISARMED" = true ] || {
  echo 'STOP: final disarmed state was not confirmed.' >&2
  exit 1
}

printf '\n=== FINAL SAFETY ===\n'
OVERRIDE="$(ros2 topic info /mavros/rc/override 2>/dev/null || true)"
printf '%s\n%s\n' "$STATE" "$OVERRIDE"
printf '%s\n' "$OVERRIDE" | grep -q 'Publisher count: 0'

find "$OUT" -type f ! -name SHA256SUMS -print0 \
  | sort -z \
  | xargs -0 sha256sum > "$OUT/SHA256SUMS"

[ "$PROBE_STATUS" -eq 0 ] || {
  echo 'STOP: routing probe failed; review evidence before any further test.' >&2
  exit "$PROBE_STATUS"
}

printf '\nPASS: propulsion-disconnected routing probe completed.\n'
printf 'physical_motor_direction_tested=false\n'
printf 'evidence_directory=%s\n' "$OUT"

