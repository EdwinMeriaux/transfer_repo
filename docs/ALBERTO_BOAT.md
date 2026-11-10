# Alberto's BlueBoat: prefilled hardware bundle and field commands

This is the short path for the actual boat described on **2026-11-09**. All local configuration files are kept together; no repeated copying of individual fields from the parameter export is needed. It builds on the existing [setup guide](ALBERTO_SETUP.md) and [effective thrust calibration procedure](THRUST_CALIBRATION.md).

**Available now:** install, offline inference, telemetry recording, dry-run, shadow, read-only identity/parameter capture, command-line mass/measurement updates, and analysis using recorded session mass. **Powered calibration/PPO remain blocked** until the current vehicle settings and missing measurements/tests are resolved. The export is not a thrust calibration or a record of completed safety tests. No command in this workflow changes flight-controller parameters, mode or arming.

## Exactly what was populated

Source file: `Surface Boat-4.6.2-STABLE-20261109-183907-paddle.params`, SHA-256 `a6005b927404b6fa17edfee31ccb66c8afd878cbe5bf8444ff1ead30d351cda4`. The repository includes 66 relevant control/failsafe parameters, not the entire hardware export or unrelated sensor identifiers. Operator report: approximately **23 kg**, **right motor on servo output 1**, **left motor on servo output 3**, neutral **1500**, forward above neutral.

| Item | Left/port | Right/starboard |
|---|---:|---:|
| PPO action/force index | 0 | 1 |
| **RC override input** | **1** (`RCMAP_ROLL`) | **3** (`RCMAP_THROTTLE`) |
| **Physical servo output** | **3**, function 73 | **1**, function 74 |
| Reported RC-input neutral | 1500 | 1500 |
| Reported input forward polarity | +1 | +1 |
| Exported servo trim | 1510 | 1510 |
| Exported servo reversal | 1 | 0 |
| Measured actual servo neutral/polarity | Pending | Pending |

Do not swap the PPO action order or RC inputs to match servo numbers. In the source-reviewed **Rover 4.6.2 MANUAL two-paddle path**, left comes from the steering/roll RC channel and right from throttle; output functions select the physical connectors. Sources: [Rover input mixing](https://github.com/ArduPilot/ardupilot/blob/Rover-4.6.2/Rover/mode.cpp), [RC channel selection](https://github.com/ArduPilot/ardupilot/blob/Rover-4.6.2/Rover/radio.cpp). This establishes the software interpretation; actual paired response still needs verification.

The 1500 report and 1510 trims are kept distinct. Neither the output-neutral measurement nor its direction is silently inferred from a trim or reverse bit. `servo_neutral_pwm` and `servo_forward_pwm_sign` remain null until supplied from observation. `AHRS_ORIENTATION` is managed by the flight controller; this profile does not add another guessed IMU rotation. Validate MAVROS FLU/ENU conventions as described in the calibration guide.

## All files that go on the boat

The actual deployment package remains [`ros2_ws/src/blueboat_ppo_controller/`](../ros2_ws/src/blueboat_ppo_controller/). Its bundled profile is [`blueboat_ppo_controller/profiles/alberto/`](../ros2_ws/src/blueboat_ppo_controller/blueboat_ppo_controller/profiles/alberto/), included even in a wheel or a copied ROS package. Run `alberto_boat init` once to copy these files together into **`~/.config/blueboat/alberto/`**:

| File | Purpose |
|---|---|
| `boat.json` | 23 kg reported mass, provenance and outstanding measurements |
| `observed_parameters.json` | Relevant actual exported values, unchanged; not an FC parameter-upload file |
| `motor_interface.json` | Correct input/output routing and RC neutral; hardware identity, output measurements, evidence and approvals remain pending |
| `thrust_calibration.yaml` | 23 kg, default dry-run, original bounded pulse plan and telemetry limits |
| `controller.yaml` | Default PPO shadow mode, exact bundled model |
| `analysis.json` | Analysis options; mass uncertainty and COM→IMU offset must be supplied, not guessed |
| `README.md` | Profile summary and known blockers |

Run directories are placed under that same local directory's `runs/calibration/` and `runs/ppo/`. Session CSV/JSON/events belong together; analysis output directories must be new. The generic SITL templates remain separate: **do not use this hardware UID/profile for SITL**. Do not commit live captures, operator evidence or generated configurations back into the repository.

## Install once on the Jetson

Assumes Ubuntu 22.04, ROS 2 Humble, MAVROS and your existing simulator/vehicle connection are already installed. Use system Python 3.10. For a new checkout:

```bash
cd ~
git clone --branch main https://github.com/EdwinMeriaux/gazebo_blue_boat.git
cd ~/gazebo_blue_boat
sudo apt-get install python3-venv python3-pip python3-colcon-common-extensions
./scripts/install_alberto.sh
```

For an existing checkout, first save your work and `git pull --ff-only` on main, then run the installer. It creates a venv, installs inference/analysis dependencies, links the package into `~/blueboat_ws/src`, builds with colcon, runs offline inference and initializes the local bundle. An existing different package is refused; existing local configuration is preserved. It does not launch powered control. A custom workspace can be supplied as the install script's first argument; set `BLUEBOAT_WORKSPACE=/your/workspace` when using the wrapper afterward.

The wrapper sources Humble, the venv and the workspace automatically in each terminal. From the repository root:

```bash
./scripts/alberto.sh status
./scripts/alberto.sh run telemetry
```

`status` intentionally exits **2** while there are blockers. This does not prevent telemetry/dry-run/shadow. Stop a running node with Ctrl+C before starting another. The wrapper accepts `--directory /another/bundle` **before** its subcommand for a separate local profile. `init` refuses to overwrite an existing directory.

If you copy only the ROS package into an existing workspace, build it as in [ALBERTO_SETUP.md](ALBERTO_SETUP.md), source ROS/venv/workspace once and run `alberto_boat init`, `alberto_boat status`, etc. The repository wrapper scripts are conveniences, not runtime dependencies. All profile files are installed with the package.

## Confirm mass and capture the current vehicle

The shipped 23 kg is an approximate operator report, not a measurement made by this software. It affects calibration/analysis, not the trained policy or MJX model. Change it with one command when total wet mass including payload is known:

```bash
./scripts/alberto.sh configure --mass-kg 23
./scripts/alberto.sh capture
./scripts/alberto.sh status
```

`capture` requires connected MAVROS and reads only `vehicle_info_get`, `param/pull`, and `param/get_parameters`. It records actual sysid/compid/encoded firmware version/hardware UID and all pinned parameters, reports differences, and fills the live identity fields. It refuses a missing hardware UID or unexpected system ID. It does **not** adopt changed parameter values as approved expectations, set any FC parameter, or mark tests passed. Captures use unique filenames. Stop running controller/calibration nodes before changing or capturing configuration; they do not hot-reload local files. Running capture/configure clears local interface approval and verification flags; do them before the final evidence review.

Default MAVROS topics/services match the existing package. If the namespace differs, edit the two local YAML files once, including `vehicle_info_service`, `param_pull_service`, `param_get_service` when needed. Topic names cannot be inferred from an ArduPilot parameter export. Hardware launches use wall clock by default. All required timestamps, frames, sensor rates and estimator flags are still checked.

## Non-actuating field commands

With existing MAVROS telemetry active, run one mode at a time:

```bash
./scripts/alberto.sh run telemetry
# After stopping telemetry with Ctrl+C:
./scripts/alberto.sh run dry_run
# After stopping dry-run:
./scripts/alberto.sh run shadow
```

Telemetry logs sensor streams. Dry-run previews bounded pulses without a motor publisher. Shadow runs PPO feedback and publishes requested forces without commanding motors. In a second **ROS-sourced** terminal during shadow, a bounded command is:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --rate 10 /blueboat/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Record source data in another ROS-sourced terminal, choosing a new bag directory each time:

```bash
mkdir -p ~/.config/blueboat/alberto/bags
ros2 bag record -o ~/.config/blueboat/alberto/bags/telemetry-first \
  /mavros/imu/data_raw /mavros/imu/data /mavros/local_position/odom \
  /mavros/rc/in /mavros/rc/out /mavros/state /mavros/estimator_status \
  /mavros/rc/override /blueboat/calibration/status /blueboat/calibration/enable \
  /blueboat/cmd_vel /blueboat/observation /blueboat/requested_forces /blueboat/diagnostics
```

The current adapter needs full motion estimates; IMU acceleration alone does not supply ground-relative velocity. No sensor quantity is filled with zeros to clear a gate.

## Current parameter differences to resolve with the operator

These are findings, **not commands to apply blindly**. The software preserves every observed value and never changes the boat remotely.

| Exported value | Why it blocks this integration / required review |
|---|---|
| `RC_OPTIONS=37` | Bits 0 and 2 ignore the physical receiver and receiver failsafe flags. This conflicts with the existing receiver takeover design. Clarify whether takeover is physical RC or joystick/GCS over MAVLink; the latter needs a separately designed/tested authority path, not a bypass flag. |
| All `RCn_OPTION=0` | No separate override-enable switch (`46`) is configured. Choose an actual available receiver channel, distinct from RC1/RC3 and mode channel 8, then test it. No channel is fabricated in this profile. |
| `RC_OVERRIDE_TIME=3` | Existing controller requires a tested positive timeout ≤0.5 s. Measure real fallback behavior after companion/network failure. |
| `FS_THR_ENABLE=0` | Receiver failsafe is disabled; the existing guard requires it enabled and tested. |
| `MOT_THST_ASYM=1.6` | The independent per-motor curve adapter currently requires 1. The mixer can couple commands; review/validate this mapping or implement a characterized coupled mapping before powered use. Do not just bypass the check. |
| `SERVO1/3_TRIM=1510`, `SERVO3_REVERSED=1` | Preserve them and measure actual neutral/sign separately from the reported input-neutral 1500. |

`FS_GCS_ENABLE=2`, GCS/receiver timing/actions, motor slew 200, steering/throttle mix 0.6, and mode channel 8 are retained for matching and review. Their presence is not proof that fallback behavior has been tested. The added runtime check rejects ignored receiver/failsafe bits even if someone manually changes `verified` to true. Source for those bits: [Rover 4.6.2 RC options](https://github.com/ArduPilot/ardupilot/blob/Rover-4.6.2/libraries/RC_Channel/RC_Channel.h); [failsafe parameters](https://github.com/ArduPilot/ardupilot/blob/Rover-4.6.2/Rover/Parameters.cpp).

After the operator deliberately changes and tests vehicle parameters, export them again, then import the export with one command:

```bash
./scripts/alberto.sh refresh --params '/absolute/path/to/new-boat.params'
./scripts/alberto.sh capture
./scripts/alberto.sh status
```

Refresh preserves a timestamped backup of the old interface and repopulates the mapping and expected values. It clears identity, actual servo measurements and approvals because old evidence may no longer apply. It never writes the export back to the FC. Record original and changed parameter dumps with the test evidence.

## Enter remaining measurements once

Use `configure` options rather than copying fields between files:

```text
./scripts/alberto.sh configure --mass-kg MEASURED_TOTAL_KG --mass-std-kg UNCERTAINTY_KG \
  --imu-offset X_METRES Y_METRES Z_METRES \
  --left-servo-neutral MEASURED_PWM --right-servo-neutral MEASURED_PWM \
  --left-servo-forward-sign SIGN --right-servo-forward-sign SIGN \
  --enable-rc-channel VERIFIED_CHANNEL
```

Replace uppercase tokens with actual values; signs are `1` or `-1`. COM→IMU is in body FLU metres. A physically reversed left motor may have a negative servo PWM sign even though its RC input forward sign is positive; confirm both. Supplying a channel does not set its FC auxiliary option, and supplying measurements does not approve the interface.

The only remaining manual review file is the local `motor_interface.json`: provide the actual evidence path and SHA, and mark only tests actually completed. Existing requirements include interface direction/neutral/paired-output tests, telemetry and IMU placement, manual takeover/receiver/GCS/override timeout tests, SITL control and SITL calibration acceptance, and restrained control tests. Set `verified` only after review. [The full calibration guide](THRUST_CALIBRATION.md) defines those tests. No executable here auto-approves them.

## Powered collection and analysis after those checks

These commands intentionally fail with the shipped profile. They are supplied so no extra field editing is needed after a verified interface exists:

```bash
./scripts/alberto.sh run powered --enable-actuators
```

The operator selects MANUAL and arms through the existing tested workflow, enables the separate receiver switch, and starts the explicit software heartbeat in another ROS-sourced terminal only when ready:

```bash
ros2 topic pub --rate 10 /blueboat/calibration/enable std_msgs/msg/Bool '{data: true}'
```

Do not run PPO or another motor publisher simultaneously. For IMU effective-force estimation, use the controlled free-hull procedure; restrained control tests establish wiring/fallback, not valid free-hull acceleration. All original rest, rate, tilt, feedback, watchdog and pulse limits remain active. A 23 kg mass entry does not make `F=ma` exact; added mass, drag and identification limitations remain as described in the calibration guide.

After stopping collection, use the actual CSV filename and a new report directory:

```bash
./scripts/alberto.sh analyze /absolute/path/to/calibration-RUN.csv \
  --output /absolute/path/to/new-report
```

The sibling session JSON is loaded automatically, neutral values come from this profile, and **mass comes from the recorded session**, not a later changed configuration. Actual session motor mapping/neutrals must agree with the selected profile. The command refuses unknown mass uncertainty/IMU offset. It produces plots, rejection records, lookup and an unapproved PPO draft where supported; it never marks a fit accepted.

After independent review of a sufficiently covered measured mapping, PPO actuation uses that reviewed file:

```bash
./scripts/alberto.sh run ppo --enable-actuators \
  --calibration /absolute/path/to/reviewed-ppo-calibration.json
```

The normal PPO enable heartbeat `/blueboat/enable`, fresh desired Twist, valid live telemetry, identity/parameters, receiver enable and armed MANUAL gates still apply. See [ALBERTO_SETUP.md](ALBERTO_SETUP.md) for that sequence. The trained model, observation layout, action scaling and force envelope have not changed.

## Stop

Use the tested physical takeover procedure first. Stop the true software heartbeat publisher, then send false on the active mode's enable topic:

```bash
ros2 topic pub --once /blueboat/calibration/enable std_msgs/msg/Bool '{data: false}'
# For PPO instead:
ros2 topic pub --once /blueboat/enable std_msgs/msg/Bool '{data: false}'
```

Stop the node and bag with Ctrl+C, and verify actual neutral/authority through the operator workflow. Release returns control to the configured receiver/FC; it is not guaranteed braking. Process/network failures rely on tested independent FC fallback. Linux/Jetson installation, live MAVROS capture, SITL and physical execution of this new convenience workflow remain untested here; local software validation is recorded separately.
