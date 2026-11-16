# Alberto's BlueBoat hardware profile

This profile describes the boat reported on 2026-11-09. It uses the Jetson's existing MAVROS UDP connection directly to the Pixhawk for PPO and calibration output. QGroundControl remains on the separate laptop → Wi-Fi → Raspberry Pi/BlueOS operator path. This repository adds no gateway, ground-side key or ownership-grant process.

During powered PPO or calibration, stop QGC joystick motor transmission and every other motor publisher. The ROS graph check can enforce one `/mavros/rc/override` publisher but cannot detect a separate MAVLink sender outside ROS.

## Populated configuration

Source export: `Surface Boat-4.6.2-STABLE-20261109-183907-paddle.params`, SHA-256 `a6005b927404b6fa17edfee31ccb66c8afd878cbe5bf8444ff1ead30d351cda4`.

Operator report: approximately 23 kg, right motor on servo output 1, left motor on servo output 3, RC-input neutral 1500, and forward above neutral.

| Item | Left/port | Right/starboard |
|---|---:|---:|
| PPO action/force index | 0 | 1 |
| MAVLink RC override input | 1 (`RCMAP_ROLL`) | 3 (`RCMAP_THROTTLE`) |
| Physical servo output | 3, function 73 | 1, function 74 |
| Reported input neutral | 1500 | 1500 |
| Reported input forward polarity | +1 | +1 |
| Exported servo trim | 1510 | 1510 |
| Exported servo reversal | 1 | 0 |
| Measured actual servo neutral/polarity | Pending | Pending |

Do not swap PPO action order or RC inputs to match servo numbers. Rover's MANUAL two-paddle path obtains left from steering/roll and right from throttle; the output functions select physical connectors. Actual paired response still requires a restrained test.

Input neutral 1500 and servo trim 1510 are deliberately distinct. Actual output neutral and direction are not inferred from trim/reversal. `servo_neutral_pwm` and `servo_forward_pwm_sign` remain unset until observed.

## Local bundle

Installation copies the packaged profile to `~/.config/blueboat/alberto/`:

| File | Purpose |
|---|---|
| `boat.json` | Reported mass, provenance and control architecture |
| `observed_parameters.json` | Read-only observed parameter subset; not an upload file |
| `motor_interface.json` | Direct-MAVROS routing, identity pins, mapping and evidence flags |
| `thrust_calibration.yaml` | Default dry-run and bounded pulse plan |
| `controller.yaml` | Default PPO shadow mode and model settings |
| `analysis.json` | Analysis settings requiring measured uncertainty/IMU offset |

Generated sessions live under `runs/calibration/` and `runs/ppo/`. Do not commit captures, local evidence or generated configuration.

## Upgrade from the removed gateway build

After pulling and reinstalling, migrate the preserved local profile once:

```bash
./scripts/alberto.sh configure --control-authority direct_mavros
```

This deletes stale UID and gateway/handoff fields, clears approvals and sets `enable_rc_channel` to null. It does not write any flight-controller setting.

## Read-only capture

With MAVROS connected:

```bash
./scripts/alberto.sh capture
./scripts/alberto.sh status
```

Capture calls `vehicle_info_get`, `param/pull` and `param/get_parameters`. It requires the expected ArduPilot rover/boat and pins sysid, compid, encoded firmware version, vehicle type and all expected parameters. Hardware UID is completely outside the identity and capture path: UID zero, missing UID, or a different UID does not decide capture or runtime acceptance.

Capture does not adopt live differences as new expected values, write parameters, arm, change mode or mark evidence complete. It creates a timestamped snapshot and resets approval flags. Stop all controller/calibration nodes first because they do not hot-reload files.

## Direct command path and gates

The actuator and reviewed powered calibration publish `mavros_msgs/msg/OverrideRCIn` directly on `/mavros/rc/override`. The runtime still requires:

- exactly one ROS override publisher;
- armed MANUAL state;
- fresh explicit software enable heartbeat;
- fresh command, odometry and IMU streams with advancing, aligned source timestamps and exact frames;
- matching sysid, compid, exact firmware, vehicle type and parameter pins;
- reviewed motor mapping and accepted evidence;
- bounded force/PWM, rate, tilt, feedback and watchdog checks;
- configured flight-controller command-loss fallback.

Neither capture nor the controller arms or changes flight mode. A software heartbeat does not make unresolved physical configuration safe.

ArduRover publishes `EKF_STATUS_REPORT` (193) and [documents common `ESTIMATOR_STATUS` (230) as unsupported](https://ardupilot.org/rover/docs/Rover_MAVLink_Messages.html); message 230 is the source of MAVROS `/mavros/estimator_status`. The Alberto profile explicitly uses `require_estimator_status: false`; this removes only that impossible topic dependency. Connected/healthy FC state and the live IMU/odometry validity, frame, timestamp, rate, skew and limit gates remain mandatory. Generic profiles default to requiring estimator status.

## Current blockers

The supplied profile intentionally has `powered_ready=false`:

| Exported value/state | Required resolution |
|---|---|
| `RC_OVERRIDE_TIME=3` | Test and configure a positive override timeout no greater than 0.5 s. |
| `FS_ACTION=4` | Direct mode requires a tested Hold (2) or Terminate (5) fallback; RTL/SmartRTL may move the boat after loss. |
| `MOT_THST_ASYM=1.6` | The independent-curve adapter requires 1 or a separately characterized coupled mapping. |
| Servo neutral/sign unset | Measure actual neutral and forward polarity independently on both sides. |
| Interface/evidence unverified | Complete takeover, timeout, paired-output, telemetry, IMU, SITL, calibration and restrained-motor reviews. |

`RC_OPTIONS=37`, `FS_THR_ENABLE=0` and no `RCn_OPTION=46` are retained for this receiver-less direct-MAVROS architecture; the software no longer pretends a physical receiver switch exists. That increases the importance of an exclusive command source, the explicit ROS enable, operator takeover, and tested MAVLink/flight-controller timeouts.

After deliberate flight-controller changes, export the new values and run:

```bash
./scripts/alberto.sh refresh --params /absolute/path/to/new-boat.params
./scripts/alberto.sh capture
./scripts/alberto.sh status
```

Refresh keeps a timestamped old-interface backup, rebuilds the expected pins, removes UID, and clears live identity, output measurements and approvals. It never uploads parameters.

## Measurements and calibration

Enter only measured values:

```bash
./scripts/alberto.sh configure \
  --mass-kg MEASURED_TOTAL_KG \
  --mass-std-kg UNCERTAINTY_KG \
  --imu-offset X_METRES Y_METRES Z_METRES \
  --left-servo-neutral MEASURED_PWM \
  --right-servo-neutral MEASURED_PWM \
  --left-servo-forward-sign SIGN \
  --right-servo-forward-sign SIGN
```

Signs are `1` or `-1`; COM→IMU offset uses body FLU metres. Supplying values does not approve them. Attach the reviewed evidence file/hash and set flags only for tests actually completed.

The reviewed powered sequence and PPO launch sequence are in the repository [README](../README.md). The unchecked PWM script is only a restrained wiring aid; it is not effective-force calibration.

## Validation boundary

No direct Jetson→Pixhawk actuation, QGC coexistence, actual boat capture or physical motor test was performed by the repository tests. Gazebo validates the relative route controller and ONNX-to-force simulation path only. Hardware deployment still requires the operator's staged tests and recovery plan.
