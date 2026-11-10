# Alberto BlueBoat PPO: start-to-finish operating guide

This branch contains the trained PPO controller and three field scripts:

1. An explicitly unchecked PWM pulse sequence.
2. A 20 m outbound route, 180-degree turn and 18 m return, ending about 2 m ahead of the start.
3. A 25 m approach to the center of a 12 m-diameter circle, one clockwise circle and a return to a point 2 m ahead of the start.

The route scripts publish relative speed and yaw-rate requests. They are not standalone motor controllers. A working PPO actuator, telemetry, local odometry, enable heartbeat, reviewed motor calibration and command-ownership gateway are also required.

## Current blockers before physical PPO operation

- Use Ubuntu 22.04, ROS 2 Humble and system Python 3.10. The installer rejects Ubuntu 20.04.
- `/mavros/local_position/odom` must provide a valid local position. The routes do not use latitude/longitude waypoints, but they still need odometry to measure distance.
- No reviewed real-boat PPO calibration JSON is committed to this repository.
- `01_calibrate_unchecked.sh` sends PWM pulses and records only the commanded PWM. It does not measure thrust or generate the calibration JSON required by PPO.
- The protected gateway intentionally drops MAVROS motor overrides. The unchecked MAVROS publisher is therefore an isolated wiring/PWM tool, not part of the protected field-control path. Do not create a command bypass merely to make it run.

See [Alberto's boat configuration](docs/ALBERTO_BOAT.md), [gateway handoff](docs/GCS_HANDOFF.md), [calibration](docs/THRUST_CALIBRATION.md) and [field routes](docs/SIMPLE_FIELD_RUNS.md) for the detailed contracts and evidence requirements.

## 1. Install the correct branch

The target computer must already have ROS 2 Humble and MAVROS installed.

For a new checkout:

```bash
cd ~

git clone --branch alberto/simple-field-scripts \
  https://github.com/EdwinMeriaux/gazebo_blue_boat.git

cd ~/gazebo_blue_boat

sudo apt-get update
sudo apt-get install -y \
  python3-venv python3-pip python3-colcon-common-extensions

./scripts/install_alberto.sh
```

For an existing checkout:

```bash
cd ~/gazebo_blue_boat
git fetch origin
git switch alberto/simple-field-scripts
git pull --ff-only
./scripts/install_alberto.sh
```

Confirm that the bundled model works offline:

```bash
source /opt/ros/humble/setup.bash
source ~/gazebo_blue_boat/.venv/bin/activate
source ~/blueboat_ws/install/setup.bash

inference_test
```

The inference test publishes no motor commands.

## 2. Connect the existing boat stack

Start Alberto's known working components:

1. Flight controller.
2. MAVProxy using its existing flight-controller connection.
3. MAVROS.
4. QGroundControl.

Do not guess or replace the working flight-controller connection string.

Configure the exclusive gateway once, substituting the real addresses:

```bash
cd ~/gazebo_blue_boat

./scripts/alberto.sh gateway-configure \
  --qgc-ip QGC_COMPUTER_IP \
  --mavproxy-ip 127.0.0.1
```

Use the companion computer's address instead of `127.0.0.1` if MAVProxy runs on another host.

Securely copy `ros2_ws/src/blueboat_ppo_controller/blueboat_ppo_controller/ground_owner.py` and the generated `~/.config/blueboat/alberto/ground.key` to the QGC computer. Keep the key private, do not commit it and retain file mode `600`.

Configure these routes as described in [GCS_HANDOFF.md](docs/GCS_HANDOFF.md):

- MAVProxy has one output to gateway port `14600`.
- QGroundControl connects through gateway port `14601`.
- MAVROS uses `udp://127.0.0.1:14602@127.0.0.1:14603`.
- Old or direct QGC, MAVROS and other command routes are removed.

Initially run the gateway in monitor-only mode:

```bash
./scripts/alberto.sh gateway
```

## 3. Verify telemetry and local odometry

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/gazebo_blue_boat/.venv/bin/activate
source ~/blueboat_ws/install/setup.bash

ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/odom --once
ros2 topic echo /mavros/imu/data --once
ros2 topic echo /mavros/estimator_status --once

ros2 topic hz /mavros/local_position/odom
ros2 topic hz /mavros/imu/data
```

Before either route can run, verify all of the following:

- Odometry pose frame is `map`.
- Odometry body frame is `base_link`.
- Position changes in the correct direction during a known motion.
- Heading changes in the correct direction during a known turn.
- Motion measurements are fresh and preferably at least 50 Hz.
- Position and orientation are finite and do not freeze.

If there is no GPS and no alternative localization source producing valid local odometry, the distance missions cannot determine how far the boat has travelled.

## 4. Capture the physical boat configuration

Enter the measured total wet mass, including payload:

```bash
cd ~/gazebo_blue_boat

./scripts/alberto.sh configure --mass-kg MEASURED_TOTAL_KG
./scripts/alberto.sh capture
./scripts/alberto.sh status
```

`status` intentionally exits with code 2 while powered-operation evidence is missing. This is a blocker report, not a crash.

Alberto's flight controller reports `uid=0`, and this MAVROS release has no `uid2` field. Capture records that value as "UID unavailable" instead of rejecting the vehicle. The remaining identity gate still pins system ID, component ID, exact firmware, vehicle type, gateway target and the complete reviewed parameter set.

The reviewed configuration must confirm:

- Flight-controller identity and firmware.
- Left/right channel and physical-output mapping.
- Actual neutral PWM and forward polarity for each motor.
- Compatible pinned parameters and override timeout.
- Manual takeover, GCS loss and independent fallback behavior.

## 5. Run the non-actuating checks

Run one mode at a time and stop it with Ctrl+C before starting the next:

```bash
./scripts/alberto.sh run telemetry
```

```bash
./scripts/alberto.sh run dry_run
```

```bash
./scripts/alberto.sh run shadow
```

During shadow mode, publish a small request from another ROS-sourced terminal:

```bash
ros2 topic pub --rate 10 /blueboat/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Inspect the model output and diagnostics:

```bash
ros2 topic echo /blueboat/diagnostics
ros2 topic echo /blueboat/requested_forces
```

Shadow mode must not publish motor overrides.

## 6. Optional unchecked PWM pulse test

The requested unchecked pulse command is:

```bash
./scripts/01_calibrate_unchecked.sh
```

A smaller initial sequence is:

```bash
./scripts/01_calibrate_unchecked.sh \
  --delta -40 40 \
  --repeats 1
```

This command deliberately skips vehicle, GPS, odometry, IMU, estimator, parameter, gateway and evidence checks. It directly publishes RC-input PWM requests for individual left, individual right and symmetric pulses.

It must have restrained or otherwise safely isolated propulsion, one motor publisher, an operator with tested manual takeover and a verified direct MAVROS override route. The protected gateway's read-only MAVROS path will reject it. Its log is not a PPO force calibration and cannot authorize either field route.

## 7. Produce and review the actual PPO calibration

After the interface, routing and acceptance evidence have been completed, start the guarded powered calibration:

```bash
./scripts/alberto.sh run powered --enable-actuators
```

In another ROS-sourced terminal, start the calibration enable heartbeat only when the operator is ready:

```bash
ros2 topic pub --rate 10 \
  /blueboat/calibration/enable \
  std_msgs/msg/Bool \
  '{data: true}'
```

The operator must also:

1. Select MANUAL through QGroundControl.
2. Arm through the existing tested workflow.
3. Disable QGC joystick transmission and wait at least one second.
4. Grant autonomous ownership from the QGC computer using the procedure below.

After collecting powered calibration data, stop collection and analyze its actual CSV into a new directory:

```bash
./scripts/alberto.sh analyze \
  /absolute/path/to/calibration-RUN.csv \
  --output /absolute/path/to/new-report
```

The generated PPO calibration is a draft. A reviewer must confirm its evidence, measured range, motor mapping, assumptions and checksums before marking it verified. Synthetic results or commanded PWM alone cannot be used as physical calibration evidence.

## 8. Start a field mission

Once a reviewed physical-boat calibration file exists, use separate terminals for the gateway, PPO actuator, enable heartbeat and one route.

### Terminal 1: protected gateway

```bash
cd ~/gazebo_blue_boat
./scripts/alberto.sh gateway --enable-forwarding
```

### Terminal 2: PPO actuator

```bash
cd ~/gazebo_blue_boat

./scripts/alberto.sh run ppo --enable-actuators \
  --calibration /absolute/path/to/reviewed-ppo-calibration.json
```

### Terminal 3: PPO enable heartbeat

```bash
source /opt/ros/humble/setup.bash
source ~/gazebo_blue_boat/.venv/bin/activate
source ~/blueboat_ws/install/setup.bash

ros2 topic pub --rate 10 \
  /blueboat/enable \
  std_msgs/msg/Bool \
  '{data: true}'
```

### Terminal 4: exactly one route

For the 20 m outbound route, 180-degree turn and 18 m return:

```bash
cd ~/gazebo_blue_boat
./scripts/02_out_20m_turn_return.sh
```

For the 25 m approach, 12 m-diameter circle and return to 2 m ahead of the start:

```bash
cd ~/gazebo_blue_boat
./scripts/03_forward_25m_circle_12m.sh
```

Never run both route scripts simultaneously. Both use a target cruise speed of 0.4 m/s and a maximum yaw-rate request of 0.3 rad/s. The PPO interface rejects forward-speed requests outside ±0.8 m/s and yaw-rate requests outside ±0.4 rad/s.

### QGroundControl computer: grant ownership

After MANUAL is selected, the vehicle is armed and joystick transmission has stopped:

```bash
python3 ground_owner.py auto \
  --gateway GATEWAY_IP:14604 \
  --key-file /path/to/ground.key
```

Keep that process running for the entire mission. The gateway and controller do not arm the vehicle or select MANUAL automatically.

## 9. Stop after every mission

First request an autonomous stop from the QGC computer:

```bash
python3 ground_owner.py stop \
  --gateway GATEWAY_IP:14604 \
  --key-file /path/to/ground.key
```

Stop the PPO heartbeat:

```bash
ros2 topic pub --once \
  /blueboat/enable \
  std_msgs/msg/Bool \
  '{data: false}'
```

After QGC confirms HOLD or disarmed state and the physical outputs are neutral, deliberately request manual ownership:

```bash
python3 ground_owner.py manual \
  --gateway GATEWAY_IP:14604 \
  --key-file /path/to/ground.key
```

Then:

1. Stop the route, heartbeat and PPO processes with Ctrl+C.
2. Verify actual neutral output and stopped propulsion.
3. Re-enable the QGC joystick deliberately.
4. Preserve and inspect the controller, gateway and ROS bag logs.
5. Fully reset and inspect the system before attempting the other route.

Release of command ownership is not guaranteed braking. Process or network failure relies on the independently tested flight-controller fallback.

## Validated simulation results

The same deterministic ONNX PPO checkpoint was tested through independent left/right Newton-force commands in calm-water Gazebo Harmonic:

| Mission | Target | Gazebo result |
|---|---:|---:|
| Outbound/return cruise | 0.400 m/s | 0.380 m/s mean, 0.404 m/s peak |
| 20 m route endpoint | 2.000 m ahead of start | 2.034 m, 0.037 m target error |
| Circle-route cruise | 0.400 m/s | 0.378 m/s mean, 0.409 m/s peak |
| Circle radius | 6.000 m | 6.086 m mean, 0.168 m RMSE |
| Circle-route endpoint | 2.000 m ahead of start | 2.038 m, 0.039 m target error |

These results validate the relative path controller and the PPO-to-Gazebo force pathway. They do not validate Alberto's MAVROS serialization, gateway routing, ArduRover mixing, physical PWM conversion, sensors or real-water stopping behavior. The machine-readable record is in [docs/validation/simple_field_missions.json](docs/validation/simple_field_missions.json).

## Developer checks

```bash
cd ~/gazebo_blue_boat
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
python -m pytest -q
```

The deployment package is [ros2_ws/src/blueboat_ppo_controller](ros2_ws/src/blueboat_ppo_controller). The original PPO weights, observation contract and action scaling are unchanged by the field scripts.
