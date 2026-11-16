#!/usr/bin/env bash
set -eo pipefail
BLUEBOAT_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BLUEBOAT_WORKSPACE="${BLUEBOAT_WORKSPACE:-$HOME/blueboat_ws}"
for BLUEBOAT_SETUP in /opt/ros/humble/setup.bash "$BLUEBOAT_REPO/.venv/bin/activate" "$BLUEBOAT_WORKSPACE/install/setup.bash"; do
  if [[ ! -f "$BLUEBOAT_SETUP" ]]; then
    echo "Missing $BLUEBOAT_SETUP; run scripts/install_alberto.sh after installing Humble prerequisites." >&2
    exit 1
  fi
  source "$BLUEBOAT_SETUP"
done
exec ros2 run blueboat_ppo_controller alberto_boat "$@"
