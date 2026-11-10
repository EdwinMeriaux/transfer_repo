#!/usr/bin/env bash
set -euo pipefail

BLUEBOAT_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BLUEBOAT_WORKSPACE="${BLUEBOAT_WORKSPACE:-$HOME/blueboat_ws}"

source /opt/ros/humble/setup.bash
if [[ -f "$BLUEBOAT_REPO/.venv/bin/activate" ]]; then
  source "$BLUEBOAT_REPO/.venv/bin/activate"
fi
if [[ -f "$BLUEBOAT_WORKSPACE/install/setup.bash" ]]; then
  source "$BLUEBOAT_WORKSPACE/install/setup.bash"
fi
export PYTHONPATH="$BLUEBOAT_REPO/ros2_ws/src/blueboat_ppo_controller${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m blueboat_ppo_controller.mission_node \
  --mission center-circle --center-distance 25 --diameter 12 "$@"
