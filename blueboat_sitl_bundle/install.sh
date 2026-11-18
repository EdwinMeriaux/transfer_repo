#!/usr/bin/env bash
# Install the u990 model, the thrust check and the PPO gap handling into the SITL container's package.
# A file the container already has is replaced ONLY if it is unchanged from the known original
# (sha256); modified files are skipped and reported, never overwritten. Backups are kept.
#   bash install.sh [PACKAGE_DIR]   (default: ~/gz_ws/gazebo_blue_boat/ros2_ws/src/blueboat_ppo_controller)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="${1:-$HOME/gz_ws/gazebo_blue_boat/ros2_ws/src/blueboat_ppo_controller}"
MOD="$PKG/blueboat_ppo_controller"
[ -d "$MOD" ] || { echo "No package at $MOD" >&2; exit 1; }
BACKUP="$HOME/blueboat_bundle_backup_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP"
SKIPPED=""
while read -r name original; do
  target="$MOD/$name"
  if [ -f "$target" ]; then
    cp "$target" "$BACKUP/$name"
    if [ "$(sha256sum "$target" | cut -d' ' -f1)" != "$original" ]; then
      SKIPPED="$SKIPPED $name"; continue
    fi
  fi
  cp "$HERE/files/blueboat_ppo_controller/$name" "$target"; echo "replaced  $name"
done <<'LIST'
generalist_policy_runtime.py c884f4ab3b05ac6aaafeb996f8248dea909500dba20727bba1da3b5c0a847b27
ppo_node.py e89cd88e567b0b4975ae746b8a16ae8248ee146798d66123fba045e8fc081b4c
safety.py 9542d233d148a06fe9a04704a339792e506dc015a99399167652ea977b66bdf1
LIST
for name in thrust_check.py force_curve.py telemetry_health.py async_writer.py gap_injector.py; do
  if [ -f "$MOD/$name" ]; then cp "$MOD/$name" "$BACKUP/$name"; fi
  cp "$HERE/files/blueboat_ppo_controller/$name" "$MOD/$name"; echo "added     $name"
done
mkdir -p "$MOD/assets" "$MOD/data" "$PKG/test"
cp -R "$HERE/files/blueboat_ppo_controller/assets/blueboat_u990" "$MOD/assets/"; echo "added     assets/blueboat_u990"
cp "$HERE/files/blueboat_ppo_controller/data/t200_thrust_N.csv" "$MOD/data/"; echo "added     data/t200_thrust_N.csv"
TESTS="test/test_thrust_check.py test/test_force_curve.py test/test_gap_injector.py"
case " $SKIPPED " in *" generalist_policy_runtime.py "*) ;; *) TESTS="$TESTS test/test_policy_models.py" ;; esac
case " $SKIPPED " in *" ppo_node.py "*|*" safety.py "*) ;; *) TESTS="$TESTS test/test_ppo_gaps.py" ;; esac
for name in ros_harness.py test_policy_models.py test_thrust_check.py test_force_curve.py test_gap_injector.py test_ppo_gaps.py; do
  cp "$HERE/files/test/$name" "$PKG/test/$name"
done
echo "backups in $BACKUP"
if [ -n "$SKIPPED" ]; then
  echo
  echo "SKIPPED (the container has local changes; not overwritten):$SKIPPED"
  echo "Send me:  cd $PKG && git diff --$(for n in $SKIPPED; do printf ' blueboat_ppo_controller/%s' "$n"; done)"
fi
echo
cd "$PKG"
# shellcheck disable=SC2086
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PKG${PYTHONPATH:+:$PYTHONPATH}" python -m pytest -q $TESTS
