#!/usr/bin/env bash
set -eo pipefail

BLUEBOAT_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BLUEBOAT_CONFIG="${BLUEBOAT_CONFIG:-$HOME/.config/blueboat/alberto}"
BLUEBOAT_WORKSPACE="${BLUEBOAT_WORKSPACE:-$HOME/blueboat_ws}"
MASS_STD_KG="${MASS_STD_KG:-2.0}"
MAX_FORCE_N="${MAX_FORCE_N:-3.0}"

LATEST_CSV=""
for BLUEBOAT_CSV in "$BLUEBOAT_CONFIG"/runs/calibration/calibration-*.csv; do
  [[ -e "$BLUEBOAT_CSV" ]] || continue
  if [[ -z "$LATEST_CSV" || "$BLUEBOAT_CSV" -nt "$LATEST_CSV" ]]; then
    LATEST_CSV="$BLUEBOAT_CSV"
  fi
done
if [[ -z "$LATEST_CSV" ]]; then
  echo "No calibration CSV found under $BLUEBOAT_CONFIG/runs/calibration" >&2
  exit 1
fi

LATEST_JSON="${LATEST_CSV%.csv}.json"
LATEST_EVENTS="${LATEST_CSV%.csv}.events.jsonl"
for BLUEBOAT_FILE in "$LATEST_JSON" "$LATEST_EVENTS"; do
  if [[ ! -f "$BLUEBOAT_FILE" ]]; then
    echo "Missing calibration file: $BLUEBOAT_FILE" >&2
    exit 1
  fi
done

if ! grep -q '"status": "complete"' "$LATEST_EVENTS"; then
  echo "Latest calibration did not complete: $LATEST_EVENTS" >&2
  exit 1
fi

ANALYSIS="$BLUEBOAT_CONFIG/analysis.json"
cp "$ANALYSIS" "$ANALYSIS.before-field-analysis-$(date +%Y%m%d-%H%M%S)"
jq --argjson mass_std "$MASS_STD_KG" '
  .mass_std_kg = $mass_std |
  .imu_offset_flu_m = [0.0, 0.0, 0.0]
' "$ANALYSIS" > "$ANALYSIS.tmp"
mv "$ANALYSIS.tmp" "$ANALYSIS"

REPORT="$BLUEBOAT_CONFIG/runs/calibration/report-$(date +%Y%m%d-%H%M%S)"
"$BLUEBOAT_REPO/scripts/alberto.sh" analyze "$LATEST_CSV" --output "$REPORT"

echo
echo "Review:"
echo "  $REPORT/trials.json"
echo "  $REPORT/lookup.json"
echo "  $REPORT/lookup.png"
echo "  $REPORT/timeseries.png"
echo
read -r -p "After Alberto reviews these results, type APPROVE: " BLUEBOAT_APPROVAL
if [[ "$BLUEBOAT_APPROVAL" != "APPROVE" ]]; then
  echo "Not approved. Draft remains at $REPORT/ppo_calibration.draft.json"
  exit 1
fi

cd "$REPORT"
jq -e --argjson cap "$MAX_FORCE_N" '
  .paired_validation.passed == true and
  (.unmeasured | length == 0) and
  ([
    .curves.left_forward,
    .curves.left_reverse,
    .curves.right_forward,
    .curves.right_reverse
  ] | all(
    (.measured_force_domain_N[0] <= $cap) and
    (.measured_force_domain_N[1] >= $cap)
  ))
' lookup.json >/dev/null

jq '
  .verified = true |
  .independent_mapping_reviewed = true
' lookup.json > lookup.tmp
mv lookup.tmp lookup.json

LOOKUP_SHA="$(sha256sum lookup.json | awk '{print $1}')"
jq --arg sha "$LOOKUP_SHA" --argjson cap "$MAX_FORCE_N" '
  .verified = true |
  .effective_mapping_accepted = true |
  .independent_effective_mapping_verified = true |
  .maximum_abs_force_N = $cap |
  .effective_lookup_sha256 = $sha
' ppo_calibration.draft.json > ppo_calibration.json

source /opt/ros/humble/setup.bash
source "$BLUEBOAT_REPO/.venv/bin/activate"
source "$BLUEBOAT_WORKSPACE/install/setup.bash"

python3 - "$REPORT/ppo_calibration.json" "$MAX_FORCE_N" <<'PY'
import sys
from blueboat_ppo_controller.motor_adapter import MotorAdapter

path = sys.argv[1]
limit = float(sys.argv[2])
adapter = MotorAdapter(path, "hardware")
for forces in ([0.0, 0.0], [limit, limit], [-limit, -limit],
               [limit, -limit], [-limit, limit]):
    adapter.convert(forces)
print("FINAL PPO CALIBRATION VALID")
PY

printf '%s\n' "$REPORT/ppo_calibration.json" > "$BLUEBOAT_CONFIG/latest_ppo_calibration.txt"
echo "FINAL PPO CALIBRATION: $REPORT/ppo_calibration.json"
