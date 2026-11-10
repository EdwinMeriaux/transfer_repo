# Alberto BlueBoat PPO controller and relative-distance routes

This branch provides a ROS 2 Humble controller for Alberto's BlueBoat, read-only vehicle capture, bounded motor calibration, and two relative-distance missions. The field controller uses **direct MAVROS RC override** from the Jetson to the Pixhawk. There is no custom gateway, ground key, or ownership-grant process.

The intended connection is:

```text
QGroundControl laptop ── Wi-Fi ── Raspberry Pi / BlueOS ── Pixhawk
Jetson / ROS 2 / MAVROS ───── existing direct UDP connection ─────┘
```

QGroundControl remains the operator/manual-control path. During PPO or calibration, QGC joystick transmission and every other motor publisher must be stopped so that MAVROS is the only command source. The software does not arm the boat, select MANUAL, write flight-controller parameters, or prove that a physical installation is safe.

## What is ready

- Deterministic ONNX PPO inference, telemetry, dry-run and shadow modes.
- Direct actuator output through `/mavros/rc/override` after all runtime gates pass.
- Read-only capture of system ID, component ID, firmware and pinned parameters. **UID is neither read nor required.**
- A deliberately unchecked, restrained PWM wiring test.
- A 20 m outbound, 180° turn, 18 m return mission.
- A 25 m outbound, 12 m-diameter circle, return-to-2-m-offset mission.
- Calm-water Gazebo validation of both relative-distance routes.

The supplied hardware profile intentionally remains blocked for powered operation until the actual neutral/sign measurements, parameter changes and test evidence are reviewed. `status` returning exit code 2 means those blockers still exist; it does not block telemetry, dry-run or shadow mode.

## Install on Ubuntu 22.04

ROS 2 Humble, MAVROS and the existing Pixhawk connection must already be installed. Use system Python 3.10.

```bash
cd ~/gazebo_blue_boat
git fetch origin
git switch alberto/simple-field-scripts
git pull --ff-only origin alberto/simple-field-scripts
sudo apt-get install python3-venv python3-pip python3-colcon-common-extensions
./scripts/install_alberto.sh
```

The installer creates `.venv`, links this package into `~/blueboat_ws/src`, builds it, runs an offline inference check and creates `~/.config/blueboat/alberto/` only when that directory does not already exist.

### Migrate an existing local profile

The installer preserves existing local configuration. Run this once after updating from the older gateway build:

```bash
./scripts/alberto.sh configure --control-authority direct_mavros
```

This removes old UID and gateway/handoff fields, sets the direct-MAVROS authority, and clears prior approvals. It does not modify the Pixhawk.

## Connect and capture

Start the vehicle's normal MAVROS connection to the Pixhawk, then verify that the required services and topics exist:

```bash
source /opt/ros/humble/setup.bash
source ~/blueboat_ws/install/setup.bash
ros2 service list | grep /mavros
ros2 topic echo --once /mavros/state
```

From the repository:

```bash
./scripts/alberto.sh configure --control-authority direct_mavros
./scripts/alberto.sh capture
./scripts/alberto.sh status
```

`capture` is read-only. It requires exactly one expected ArduPilot rover/boat and records sysid, compid, encoded firmware version and all configured parameter pins. It does not use the hardware UID, adopt changed parameters as approved values, arm, change mode, or command a motor.

Stop controller/calibration nodes before capture or configuration because running nodes do not hot-reload the files.

## Safe non-actuating checks

Run one at a time:

```bash
./scripts/alberto.sh run telemetry
./scripts/alberto.sh run dry_run
./scripts/alberto.sh run shadow
```

For shadow mode, publish a bounded raw command from a second ROS-sourced terminal:

```bash
ros2 topic pub --rate 10 /blueboat/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

These are raw normalized control requests interpreted by the PPO/controller path; they are not ArduPilot position, heading, or depth setpoints.

ArduRover does not emit common MAVLink `ESTIMATOR_STATUS` (message 230). Alberto's profile therefore does not wait on the empty MAVROS `/mavros/estimator_status` topic; it retains the connected/healthy flight-controller state, exact frame, advancing source timestamp, freshness, skew, finite-value, quaternion, IMU and odometry checks. Generic non-Rover profiles still require estimator status by default.

## Restrained PWM wiring test

Only use this with propulsion physically restrained or otherwise made safe, QGC joystick output stopped, exactly one ROS publisher on `/mavros/rc/override`, and an operator ready for manual recovery:

```bash
./scripts/01_calibrate_unchecked.sh
```

The script waits five seconds, sends bounded left/right/symmetric PWM pulses, requests neutral on Ctrl+C, then releases the channels. It intentionally bypasses vehicle-readiness, sensor, identity, parameter and evidence checks. Its log is a wiring/PWM record, not a thrust calibration and not authorization for a field run.

## Reviewed powered calibration

This remains blocked until `./scripts/alberto.sh status` reports `"powered_ready": true` and the physical test plan has been completed.

Terminal 1:

```bash
./scripts/alberto.sh run powered --enable-actuators
```

Terminal 2, only when the operator has selected MANUAL, armed using the established workflow, disabled competing motor transmission and is ready:

```bash
source /opt/ros/humble/setup.bash
source ~/blueboat_ws/install/setup.bash
ros2 topic pub --rate 10 /blueboat/calibration/enable std_msgs/msg/Bool '{data: true}'
```

Stop the heartbeat first and send false before stopping the node:

```bash
ros2 topic pub --once /blueboat/calibration/enable std_msgs/msg/Bool '{data: false}'
```

See [thrust calibration](docs/THRUST_CALIBRATION.md) for the measurement and acceptance requirements.

## PPO field routes

After a reviewed physical calibration JSON exists, use three terminals. There is no gateway process or QGC grant command.

Terminal 1 — PPO actuator:

```bash
./scripts/alberto.sh run ppo --enable-actuators \
  --calibration /absolute/path/to/reviewed-ppo-calibration.json
```

Terminal 2 — explicit enable heartbeat:

```bash
source /opt/ros/humble/setup.bash
source ~/blueboat_ws/install/setup.bash
ros2 topic pub --rate 10 /blueboat/enable std_msgs/msg/Bool '{data: true}'
```

Terminal 3 — run exactly one relative-distance route:

```bash
./scripts/02_out_20m_turn_return.sh
# or
./scripts/03_forward_25m_circle_12m.sh
```

Both routes default to 0.4 m/s, limit yaw rate to 0.3 rad/s and use local odometry rather than GPS coordinates. The first valid odometry pose defines the origin and starting heading. The first returns approximately 2 m short of the start; the circle route also returns to a point 2 m ahead of the start. Details and command overrides are in [simple field runs](docs/SIMPLE_FIELD_RUNS.md).

To stop, terminate the route, publish `false` on `/blueboat/enable`, stop the heartbeat, then Ctrl+C the actuator. Confirm actual neutral/release and restore the established manual-control path. A released MAVLink override is not guaranteed braking; the tested flight-controller timeout/failsafe is the independent fallback.

## Known hardware blockers

The current captured/exported values include important unresolved issues:

- `RC_OVERRIDE_TIME=3`; this controller requires a tested positive timeout no greater than 0.5 s.
- `FS_ACTION=4`; direct MAVROS control requires a tested Hold (2) or Terminate (5) fallback because RTL/SmartRTL may move the boat after link loss.
- `MOT_THST_ASYM=1.6`; the independent per-motor calibration adapter currently requires 1 or a separately characterized coupled mapping.
- Actual servo neutral and forward polarity are still unmeasured.
- Required takeover, timeout, telemetry, IMU, paired-motor, SITL, calibration and restrained-motor evidence is incomplete.

These are findings, not instructions to blindly change parameters. Export, review and test every intentional flight-controller change, then import the new export with:

```bash
./scripts/alberto.sh refresh --params /absolute/path/to/new-boat.params
./scripts/alberto.sh capture
./scripts/alberto.sh status
```

## Validation boundary

The route controllers were tested in calm-water Gazebo Harmonic using the bundled ONNX PPO and upstream BlueBoat marine model. The direct-force `ONNX → Newton forces → Gazebo` tests validate route/policy behavior, not MAVROS serialization, ArduRover mixing, the Jetson/Pixhawk UDP path, QGC coexistence, physical PWM conversion or real-water stopping. Route results are recorded in [simple_field_missions.json](docs/validation/simple_field_missions.json); the direct-MAVROS software test boundary is recorded in [direct_mavros_tests.json](docs/validation/direct_mavros_tests.json).

Additional references:

- [Alberto boat profile](docs/ALBERTO_BOAT.md)
- [ROS interfaces](docs/ROS_INTERFACES.md)
- [Real-boat safety](docs/REAL_BOAT_SAFETY.md)
- [Simulation validation](docs/SIMULATION_VALIDATION.md)
