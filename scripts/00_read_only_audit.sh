#!/usr/bin/env bash
set -eo pipefail

printf '\n=== BLUEBOAT READ-ONLY AUDIT ===\n'
printf 'timestamp=%s\n' "$(date --iso-8601=seconds 2>/dev/null || date)"
printf 'user=%s\n' "$(id -un)"
printf 'host=%s\n' "$(hostname)"
printf 'kernel=%s\n' "$(uname -a)"

printf '\n=== OS AND ARCHITECTURE ===\n'
cat /etc/os-release 2>/dev/null || true
printf 'architecture=%s\n' "$(uname -m)"
printf 'python3=%s\n' "$(/usr/bin/python3 --version 2>&1 || true)"
printf 'disk_root=%s\n' "$(df -h / | tail -1)"

printf '\n=== JETSON / NVIDIA ===\n'
cat /etc/nv_tegra_release 2>/dev/null || printf 'nv_tegra_release=unavailable\n'
command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi || true

printf '\n=== EXPECTED ROS INSTALLATION ===\n'
if [[ -f /opt/ros/humble/setup.bash ]]; then
  printf 'ros_humble=present\n'
  source /opt/ros/humble/setup.bash
  printf 'ros_distro=%s\n' "${ROS_DISTRO:-unknown}"
  printf 'ros2=%s\n' "$(command -v ros2)"
  printf 'colcon=%s\n' "$(command -v colcon || true)"
  if /usr/bin/python3 -c 'import rclpy, mavros_msgs.msg' 2>/dev/null; then
    printf 'ros_python_dependencies=present\n'
  else
    printf 'ros_python_dependencies=missing_or_incompatible\n'
  fi
else
  printf 'ros_humble=missing\n'
fi

printf '\n=== RELEVANT PROCESSES ===\n'
pgrep -af 'mavros_node|sim_vehicle.py|/ardurover|ArduRover|mavproxy.py|blueboat_ppo_controller/ppo_node|ros2 launch.*blueboat' || printf 'none\n'

printf '\n=== SERIAL DEVICES ===\n'
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' -o -name 'serial*' \) -print 2>/dev/null | sort || true

printf '\n=== NETWORK ADDRESSES (READ ONLY) ===\n'
if command -v ip >/dev/null 2>&1; then
  ip -brief address 2>/dev/null || true
else
  ifconfig 2>/dev/null || true
fi

if command -v ros2 >/dev/null 2>&1; then
  printf '\n=== ROS TOPICS OF INTEREST ===\n'
  ros2 topic list 2>/dev/null | grep -E '^/mavros/|^/blueboat/' || printf 'none\n'

  printf '\n=== MAVROS STATE (READ ONLY) ===\n'
  timeout 5 ros2 topic echo --once /mavros/state 2>/dev/null || printf 'mavros_state=unavailable\n'

  printf '\n=== RC OVERRIDE ENDPOINT (READ ONLY) ===\n'
  ros2 topic info /mavros/rc/override 2>/dev/null || printf 'rc_override_endpoint=unavailable\n'

  printf '\n=== TELEMETRY ENDPOINTS (READ ONLY) ===\n'
  for topic in /mavros/local_position/odom /mavros/imu/data /mavros/rc/in /mavros/rc/out; do
    printf 'topic=%s\n' "$topic"
    ros2 topic info "$topic" 2>/dev/null || printf 'unavailable\n'
  done
fi

printf '\nAUDIT COMPLETE — no nodes were started, no messages were published, no parameters were written, and no actuator command was sent.\n'

