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
snapshot_system_python() {
  /usr/bin/python3 - <<'PY'
from importlib.metadata import PackageNotFoundError, version

names = ('numpy', 'scipy', 'onnxruntime', 'PyYAML', 'matplotlib', 'setuptools')
for name in names:
    try:
        value = version(name)
    except PackageNotFoundError:
        value = 'NOT_INSTALLED'
    print(f'{name}={value}')
PY
}
SYSTEM_PYTHON_BEFORE="$(snapshot_system_python)"
printf 'System Python before isolated install:\n%s\n' "$SYSTEM_PYTHON_BEFORE"
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
if [[ -f "$BLUEBOAT_REPO/.venv/pyvenv.cfg" ]]; then
  if grep -Eiq '^include-system-site-packages[[:space:]]*=[[:space:]]*true' "$BLUEBOAT_REPO/.venv/pyvenv.cfg"; then
    echo 'Existing .venv exposes system packages; remove it manually after review and rerun.' >&2
    exit 1
  fi
else
  /usr/bin/python3 -m venv "$BLUEBOAT_REPO/.venv"
fi
source "$BLUEBOAT_REPO/.venv/bin/activate"
python -m pip install -r "$BLUEBOAT_REPO/requirements-jetson.txt"
python -m pip install --no-deps "$BLUEBOAT_PACKAGE"
python -c 'import rclpy, mavros_msgs.msg, onnxruntime'
python - <<'PY'
import numpy
import onnxruntime
import scipy
import yaml

expected = {
    'numpy': '1.26.4',
    'scipy': '1.11.4',
    'onnxruntime': '1.18.1',
    'PyYAML': '6.0.3',
}
actual = {
    'numpy': numpy.__version__,
    'scipy': scipy.__version__,
    'onnxruntime': onnxruntime.__version__,
    'PyYAML': yaml.__version__,
}
print({'isolated_environment': actual})
if actual != expected:
    raise SystemExit(f'Unexpected isolated dependency versions: {actual!r}')
PY
SYSTEM_PYTHON_AFTER="$(snapshot_system_python)"
printf 'System Python after isolated install:\n%s\n' "$SYSTEM_PYTHON_AFTER"
if [[ "$SYSTEM_PYTHON_AFTER" != "$SYSTEM_PYTHON_BEFORE" ]]; then
  echo 'STOP: system Python package metadata changed unexpectedly.' >&2
  exit 1
fi
cd "$BLUEBOAT_WORKSPACE"
python -m colcon build --symlink-install --packages-select blueboat_ppo_controller
source install/setup.bash
inference_test
if [[ ! -e "$HOME/.config/blueboat/alberto" ]]; then
  alberto_boat init
else
  echo 'Existing local Alberto configuration preserved.'
fi
echo 'Installed in an isolated virtual environment. System Python, ROS, MAVROS and Pixhawk settings were not modified.'
echo 'Start with scripts/alberto.sh status and scripts/alberto.sh run telemetry.'
