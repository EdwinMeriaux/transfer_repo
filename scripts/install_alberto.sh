#!/usr/bin/env bash
set -eo pipefail
BLUEBOAT_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BLUEBOAT_WORKSPACE="${1:-$HOME/blueboat_ws}"
BLUEBOAT_PACKAGE="$BLUEBOAT_REPO/ros2_ws/src/blueboat_ppo_controller"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble must already be installed at /opt/ros/humble.' >&2
  exit 1
fi
/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 10), "Use Ubuntu 22.04 system Python 3.10 for Humble"'
mkdir -p "$BLUEBOAT_WORKSPACE/src"
if [[ -e "$BLUEBOAT_WORKSPACE/src/blueboat_ppo_controller" || -L "$BLUEBOAT_WORKSPACE/src/blueboat_ppo_controller" ]]; then
  if [[ "$(readlink -f "$BLUEBOAT_WORKSPACE/src/blueboat_ppo_controller")" != "$BLUEBOAT_PACKAGE" ]]; then
    echo 'Workspace already contains another blueboat_ppo_controller; resolve it without overwriting your work.' >&2
    exit 1
  fi
else
  ln -s "$BLUEBOAT_PACKAGE" "$BLUEBOAT_WORKSPACE/src/blueboat_ppo_controller"
fi
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages "$BLUEBOAT_REPO/.venv"
source "$BLUEBOAT_REPO/.venv/bin/activate"
python -m pip install -r "$BLUEBOAT_REPO/requirements.txt"
python -m pip install "$BLUEBOAT_PACKAGE[calibration,handoff]"
python -c 'import rclpy, mavros_msgs.msg, onnxruntime'
cd "$BLUEBOAT_WORKSPACE"
python -m colcon build --symlink-install --packages-select blueboat_ppo_controller
source install/setup.bash
inference_test
if [[ ! -e "$HOME/.config/blueboat/alberto" ]]; then
  alberto_boat init
else
  echo 'Existing local Alberto configuration preserved.'
fi
echo 'Installed. Start with scripts/alberto.sh status and scripts/alberto.sh run telemetry.'
