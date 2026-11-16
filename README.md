# BlueBoat e274 hardware deployment

This branch contains only the files needed to audit, install, and validate the guarded e274 controller on Alberto's boat computer. It intentionally excludes Gazebo, SITL, recorded artifacts, temporary models, and simulation workspaces.

Source controller revision: `fb083f8d19959bf66f12dedde10860859065e691`  
Policy: `e274_6_5b`  
Requested hardware force limits: `-7 N` reverse and `+20 N` forward  
Physical hardware validation completed: **no**

## Safety state

The installed profile starts in `shadow` with `enable_actuators: false`. There is no measured boat calibration in this repository. Powered PPO must remain blocked until the actual boat has passed the documented parameter, authority, neutral/sign, failsafe, restrained-motor, and calibration checks.

AnyDesk is only a remote desktop connection. A person at the boat must retain physical control and an independent emergency stop for every powered step. Do not rely on AnyDesk or its network link as the safety mechanism.

Known items that must be resolved against the real Pixhawk rather than guessed:

- `RC_OVERRIDE_TIME=3` in the supplied observation must be tested/configured to a positive value no greater than `0.5 s`.
- `FS_ACTION=4` is not accepted for direct actuation; a tested Hold or Terminate fallback is required.
- `MOT_THST_ASYM=1.6` is incompatible with the independent measured-curve adapter unless the coupled mapping is characterized.
- Port/starboard neutral values and forward signs are still measurements, not assumptions.
- No hardware calibration or powered-test approval flags are shipped.

## 1. Clone and verify

Clone into a new directory; do not overwrite an existing boat workspace.

```bash
cd "$HOME"
git clone \
  --branch blueboat-hardware-deployment \
  --single-branch \
  https://github.com/EdwinMeriaux/transfer_repo.git \
  blueboat-hardware-deployment

cd "$HOME/blueboat-hardware-deployment"
git status --short
cat SOURCE_REVISION
sha256sum \
  ros2_ws/src/blueboat_ppo_controller/blueboat_ppo_controller/assets/e274_6_5b/e274_6.5B.onnx
```

The worktree must be clean. The expected source revision is the value above, and the expected policy SHA-256 is:

```text
9ded67458fdd177405b89f12f4790a3b43cc08dc4fa377cf3841d6ce39cc6bf5
```

## 2. Run the read-only audit first

This step does not install, start, publish, arm, or write anything.

```bash
cd "$HOME/blueboat-hardware-deployment"
chmod +x scripts/*.sh
./scripts/00_read_only_audit.sh | tee "audit-$(date +%Y%m%d-%H%M%S).txt"
```

Send the complete audit file back for review before installation. If the boat connection is difficult, stop here; this audit is useful precisely because it identifies the actual computer, ROS/MAVROS state, serial devices, and endpoints without changing them.

## 3. Install and validate without actuation

Only after reviewing the audit. The installer creates a private `.venv` without access to system site packages. It installs exact Python 3.10/aarch64-compatible versions (`numpy 1.26.4`, `scipy 1.11.4`, `onnxruntime 1.18.1`, `PyYAML 6.0.3`, and `colcon-common-extensions 0.3.0`) inside that directory only. It does not use `sudo`, apt, firmware tools, or a parameter-writing interface, and it does not alter the boat's system Python, ROS, MAVROS, or Pixhawk versions.

```bash
cd "$HOME/blueboat-hardware-deployment"
./scripts/install_alberto.sh

source /opt/ros/humble/setup.bash
source "$HOME/blueboat-hardware-deployment/.venv/bin/activate"
source "$HOME/blueboat_ws/install/setup.bash"

inference_test
./scripts/alberto.sh status || true
```

`status` is expected to report blockers at this stage. That is a successful fail-closed result, not permission to bypass a check.

## 4. Non-actuating live checks

With MAVROS connected to the real Pixhawk, but with the boat disarmed and propulsion physically made safe:

```bash
cd "$HOME/blueboat-hardware-deployment"
./scripts/alberto.sh capture
./scripts/alberto.sh status || true
./scripts/alberto.sh run telemetry
```

Stop telemetry with `Ctrl-C`. Then characterize real message rates while still disarmed:

```bash
./scripts/05_hardware_telemetry_characterization.sh
```

Shadow mode performs inference but creates no RC-override publisher:

```bash
./scripts/alberto.sh run shadow
```

While it runs, verify from another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /mavros/rc/override
```

Publisher count must remain `0`. Stop shadow mode with `Ctrl-C`.

## Not authorized by this bundle

Do not run `powered`, `ppo`, calibration power, arm, mode-change, or parameter-write commands from this initial handoff. Those require the physical operator, completed measurements, reviewed audit evidence, and a separate step-by-step test plan.

Detailed references are in `docs/ALBERTO_BOAT.md`, `docs/REAL_BOAT_SAFETY.md`, and `docs/THRUST_CALIBRATION.md`.
