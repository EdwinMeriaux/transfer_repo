#!/usr/bin/env bash
# Put the new PPO code (packet-gap handling, new safety) and the u990 model on the real boat.
# A file the boat already has is replaced ONLY if it is unchanged from the original (sha256);
# a locally modified file is skipped and reported, never overwritten. Every replaced file is backed up.
# Changes no autopilot parameter, does not arm, does not touch ~/.config/blueboat (boat_config.py writes a NEW yaml there).
#   bash install.sh [REPO_DIR]   (default: ~/blueboat-hardware-deployment)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${1:-$HOME/blueboat-hardware-deployment}"
PKG="$REPO/ros2_ws/src/blueboat_ppo_controller"
MOD="$PKG/blueboat_ppo_controller"
MODEL="$HERE/../blueboat_sitl_bundle/files/blueboat_ppo_controller/assets/blueboat_u990"
[ -d "$MOD" ] || { echo "No package at $MOD" >&2; exit 1; }
[ -f "$MODEL/manifest.json" ] || { echo "No u990 model at $MODEL (git pull transfer_repo)" >&2; exit 1; }
BACKUP="$HOME/blueboat_boat_backup_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP"
SKIPPED=""
while read -r name original; do
  target="$MOD/$name"
  if [ -f "$target" ]; then
    cp "$target" "$BACKUP/$name"
    if cmp -s "$target" "$HERE/files/blueboat_ppo_controller/$name"; then echo "up to date $name"; continue; fi
    if [ "$(sha256sum "$target" | cut -d' ' -f1)" != "$original" ]; then
      SKIPPED="$SKIPPED $name"; continue
    fi
  fi
  cp "$HERE/files/blueboat_ppo_controller/$name" "$target"; echo "replaced  $name"
done <<'LIST'
ppo_node.py e89cd88e567b0b4975ae746b8a16ae8248ee146798d66123fba045e8fc081b4c
safety.py 9542d233d148a06fe9a04704a339792e506dc015a99399167652ea977b66bdf1
generalist_policy_runtime.py c884f4ab3b05ac6aaafeb996f8248dea909500dba20727bba1da3b5c0a847b27
LIST
for name in telemetry_health.py async_writer.py; do
  if [ -f "$MOD/$name" ]; then cp "$MOD/$name" "$BACKUP/$name"; fi
  cp "$HERE/files/blueboat_ppo_controller/$name" "$MOD/$name"; echo "added     $name"
done
mkdir -p "$MOD/assets" "$PKG/test"
cp -R "$MODEL" "$MOD/assets/"; echo "added     assets/blueboat_u990"
for name in ros_harness.py test_ppo_gaps.py test_policy_models.py; do
  if [ -f "$PKG/test/$name" ]; then cp "$PKG/test/$name" "$BACKUP/$name"; fi
  cp "$HERE/files/test/$name" "$PKG/test/$name"
done
echo "backups in $BACKUP"
if [ -n "$SKIPPED" ]; then
  echo
  echo "SKIPPED (the boat has local changes; not overwritten):$SKIPPED"
  echo "STOP and send me:  cd $REPO && git --no-pager diff --$(for n in $SKIPPED; do printf ' ros2_ws/src/blueboat_ppo_controller/blueboat_ppo_controller/%s' "$n"; done)"
  exit 3
fi
echo
echo "== which copy does ros2 launch import? (must be under $PKG)"
(cd / && python3 -c "import importlib.util as u; s = u.find_spec('blueboat_ppo_controller'); print(list(s.submodule_search_locations) if s else 'NOT FOUND: source ~/blueboat_ws/install/setup.bash first')") || true
echo
echo "== tests"
cd "$PKG"
if python3 -c "import pytest" 2>/dev/null; then
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PKG${PYTHONPATH:+:$PYTHONPATH}" python3 -m pytest -q \
    test/test_ppo_gaps.py test/test_policy_models.py
else
  echo "pytest not installed; skipped"
fi
