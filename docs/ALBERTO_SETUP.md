# Alberto: install and test the BlueBoat PPO controller

Target: Ubuntu 22.04, ROS 2 Humble, Jetson AGX Orin. This guide assumes your existing MAVROS, ArduRover SITL and Gazebo vehicle already work. It does not install a second flight controller, replace your world or change your ArduPilot parameters.

We provide the trained model, exact observation adapter, ONNX CPU inference, ROS node, calibration gate, logs and optional direct-force simulation tools. The ROS/SITL path has **not run on the development Mac**. Start with inference and shadow mode. Your installed firmware, mappings, frames, message rates and actual thrust conversion need the acceptance tests below.

## THE CODE ALBERTO NEEDS TO INSTALL

**[`ros2_ws/src/blueboat_ppo_controller/`](../ros2_ws/src/blueboat_ppo_controller/)** is the actual standalone ROS 2 deployment package. It contains all inference assets in `blueboat_ppo_controller/assets/`. Copying or symlinking this entire directory into your existing workspace is sufficient. The research repository, original training environment, JAX, MuJoCo and optional simulation scripts are not deployment dependencies.

## 1. Clone and install inference dependencies

Commands below use bash and place the checkout at `~/gazebo_blue_boat`; use that location consistently or substitute your chosen path.

```bash
cd ~
git clone --branch main https://github.com/EdwinMeriaux/gazebo_blue_boat.git
cd ~/gazebo_blue_boat
source /opt/ros/humble/setup.bash
sudo apt-get install python3-venv python3-pip python3-colcon-common-extensions
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install ./ros2_ws/src/blueboat_ppo_controller
inference_test
```

Use Ubuntu's Python 3.10 with Humble, not a separate Conda Python. System site packages make the apt-installed ROS Python bindings available. ONNX Runtime uses its CPU provider; CUDA/TensorRT and JetPack-specific GPU inference are not required. Let pip select a compatible CPython 3.10/aarch64 wheel. Installation and timing on Jetson remain to be tested; an unavailable wheel or import error is a dependency issue to resolve before proceeding, not a reason to substitute a different model.

The inference test checks the bundled hashes, runs a known observation 1,000 times and prints left/right raw actions, forces and timing. Expected raw action approximately `[0.639906, 1.226293]`, forces `[15.99765, 25.0]` N. These are offline numbers and publish nothing. The package loads bundled ONNX automatically; no checkpoint conversion is needed. `model_directory` may point to another copy of the **same** assets including the manifest, but must not point to the ZIP checkpoint.

## 2. Add to your existing ROS workspace and build

Example workspace `~/blueboat_ws`; replace with yours. Do not overwrite an existing package of the same name.

```bash
source /opt/ros/humble/setup.bash
source ~/gazebo_blue_boat/.venv/bin/activate
mkdir -p ~/blueboat_ws/src
ln -s ~/gazebo_blue_boat/ros2_ws/src/blueboat_ppo_controller ~/blueboat_ws/src/blueboat_ppo_controller
cd ~/blueboat_ws
python -m colcon build --symlink-install --packages-select blueboat_ppo_controller
source install/setup.bash
ros2 pkg executables blueboat_ppo_controller
ros2 run blueboat_ppo_controller inference_test
```

If the package is copied instead, keep the complete directory including `assets`, `config`, `launch`, `resource`, `package.xml`, `setup.py` and `setup.cfg`. Do not copy just `ppo_node.py`. Existing MAVROS installations must expose `mavros_msgs`; check `python -c 'import rclpy, mavros_msgs.msg, onnxruntime'`. Install missing ROS dependencies through your normal Humble/rosdep workflow; the repository does not vendor ROS messages.

In **each new terminal**:

```bash
source /opt/ros/humble/setup.bash
source ~/gazebo_blue_boat/.venv/bin/activate
source ~/blueboat_ws/install/setup.bash
```

## 3. Inspect your existing SITL/MAVROS graph and configure names

Launch your existing Gazebo + ArduRover SITL + MAVROS setup using its known working commands. World/plugin/connection details are installation-specific and are deliberately not guessed here. Do not run the direct-force controller simultaneously; each pathway must have one actuator authority.

```bash
ros2 topic list -t
ros2 service list -t
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/odom --once
ros2 topic echo /mavros/imu/data --once
ros2 topic echo /mavros/estimator_status --once
ros2 topic hz /mavros/local_position/odom
```

Check IMU rate similarly. Verify measured body-frame velocity, positive yaw convention, timestamps and estimator flags using known simulated motions. The adapter expects ENU/FLU, map/base_link, and every velocity component. Frame names alone do not prove the values have the correct convention. Aim for ≥50 Hz fresh motion measurements; never relabel or zero missing fields just to clear a check.

```bash
mkdir -p ~/gazebo_blue_boat/artifacts/config
cp ~/gazebo_blue_boat/ros2_ws/src/blueboat_ppo_controller/config/controller.yaml ~/gazebo_blue_boat/artifacts/config/controller.yaml
```

Edit that copy if MAVROS runs under another namespace. All topic/service parameters and their types are listed in [ROS_INTERFACES.md](ROS_INTERFACES.md). Leave `model_directory` empty for bundled assets. `world_frame`/`body_frame` must describe actual validated data. Keep timeouts initially unchanged. Use `use_sim_time:=true` only when `/clock` is present and agrees with MAVROS header timestamps; otherwise use the default wall clock. Simulation pause will intentionally stop actuation.

## 4. Start shadow mode

```bash
ros2 launch blueboat_ppo_controller controller.launch.py \
  config:=$HOME/gazebo_blue_boat/artifacts/config/controller.yaml \
  mode:=shadow
```

In another sourced terminal, start a bounded test request:

```bash
ros2 topic pub --rate 10 /blueboat/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

A subsequent turning test can use `linear.x: 0.2, angular.z: 0.1`. Stop the first publisher before starting another. Supported requests are ±0.8 m/s and ±0.4 rad/s; other Twist components must be zero. Out-of-range commands are rejected rather than silently clipped.

Inspect in separate terminals:

```bash
ros2 topic echo /blueboat/diagnostics
ros2 topic echo /blueboat/requested_forces
ros2 topic echo /blueboat/observation --once
ros2 node info /blueboat_ppo_controller
```

Confirm node info lists **no publisher** on `/mavros/rc/override`. The boat will not follow PPO requests in shadow mode. Shadow validates observation/inference/timing, while your existing controller/manual test supplies any motion. Do not interpret shadow tracking error as PPO closed-loop performance.

## 5. Record and analyze

```bash
mkdir -p ~/gazebo_blue_boat/artifacts/bags
ros2 bag record -o ~/gazebo_blue_boat/artifacts/bags/first-shadow \
  /blueboat/cmd_vel /blueboat/enable /blueboat/observation \
  /blueboat/requested_forces /blueboat/diagnostics \
  /mavros/local_position/odom /mavros/imu/data /mavros/estimator_status \
  /mavros/state /mavros/rc/in /mavros/rc/out /mavros/rc/override
```

Add `/clock` for simulation time. Change the output directory for each run. Stop rosbag with Ctrl+C and preserve the JSONL log under `~/.ros/blueboat/`:

```bash
ls -lt ~/.ros/blueboat
python ~/gazebo_blue_boat/scripts/analyze_log.py ~/.ros/blueboat/controller-REPLACE_WITH_RUN_ID.jsonl
```

Replace the filename with the actual log. The analyzer reports valid-sample speed/yaw RMSE, inference p99 and status counts. For closed-loop acceptance, run fixed-duration commands from rest, retain all start/stop/fault intervals, record the force cap and compare target/actual using the same metrics. A fault-free fast inference test alone does not verify end-to-end latency or thrust delivery.

## 6. Complete mapping/calibration before SITL actuation

The new [IMU calibration guide](THRUST_CALIBRATION.md) provides telemetry-only recording, dry-run pulses, synthetic analysis/plots and opt-in live SITL acceptance. It separates verified motor-interface evidence from measured force curves. Its effective surge estimates require additional assumptions/review before optional integration through `MotorAdapter`; no PPO model or observation/action definition changes are needed.

Read and perform [REAL_BOAT_SAFETY.md](REAL_BOAT_SAFETY.md). Copy `calibration.template.json` into `artifacts/calibration/`, measure the actual force-versus-RC-input mapping through your simulator/ArduPilot chain, and record identity, parameters and takeover/failsafe evidence. The template is not runnable actuation configuration. The synthetic pytest calibration is **never** a vehicle calibration.

Useful read-only checks:

```bash
ros2 service call /mavros/vehicle_info_get mavros_msgs/srv/VehicleInfoGet \
  '{sysid: 0, compid: 0, get_all: false}'
ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull '{force_pull: true}'
ros2 service call /mavros/param/get_parameters rcl_interfaces/srv/GetParameters \
  '{names: [PILOT_STEER_TYPE, RCMAP_ROLL, RCMAP_THROTTLE, RC_OVERRIDE_TIME]}'
```

Names may need your namespace. Existing vehicle parameters are never set by this package. RC override requires a working accepted MAVLink source identity and independently tested physical/simulated receiver enable switch. MANUAL two-paddle mapping is source-reviewed for Rover 4.5.7; recheck your version and test paired outputs.

## 7. Launch calibrated SITL actuator mode

Only after the previous tests and evidence are complete, stop the shadow node and use the actual verified calibration file:

```bash
ros2 launch blueboat_ppo_controller controller.launch.py \
  config:=$HOME/gazebo_blue_boat/artifacts/config/controller.yaml \
  mode:=actuator enable_actuators:=true deployment_target:=sitl \
  calibration_file:=$HOME/gazebo_blue_boat/artifacts/calibration/sitl.json
```

The operator must explicitly select MANUAL, arm through the existing tested workflow, and enable the separate RC switch; this node does none of those actions. Start with a small deployment cap and a small speed request. In another sourced terminal send the software enable heartbeat:

```bash
ros2 topic pub --rate 10 /blueboat/enable std_msgs/msg/Bool '{data: true}'
```

Continue the command publisher and record the same bag/logs. Confirm individual and combined output signs, actual applied Gazebo forces, measured speed/yaw, delays and saturation. Perform command-loss, telemetry-loss, process-kill, MAVROS-loss, receiver/enable loss and manual takeover tests. An active fault latches and requires a deliberate restart after resolving the cause. Do not tune away a fault to make a video.

## 8. Stop safely

First use the tested physical/simulated takeover procedure. Stop the true enable heartbeat publisher, then request software release:

```bash
ros2 topic pub --once /blueboat/enable std_msgs/msg/Bool '{data: false}'
```

Stop the command publisher and controller with Ctrl+C, and stop rosbag. Verify receiver/flight-controller authority and actual neutral/stop through the existing operator workflow. Ctrl+C attempts measured neutral then channel release; release itself is not guaranteed braking. Process/network failure relies on the flight controller's tested timeout/failsafes.

## 9. Tests on your machine and hardware progression

```bash
cd ~/gazebo_blue_boat
python -m pip install -r requirements-dev.txt
python -m pytest -q
```

Here the real ROS message test should run rather than skip. Also run the actual colcon build, shadow node and full SITL experiment; mocked tests cannot establish middleware behavior. Measure sustained 50 Hz loop timing on the Jetson under the intended load, test logs/disk-full behavior, and record dependency/firmware versions.

Only after successful SITL and restrained hardware calibration use a **separate** hardware calibration with `deployment_target:=hardware`. Hardware mode requires its own UID/evidence plus completed SITL and restrained-motor flags. Real-water validation remains outstanding, including sensors in the pool, motor dynamics, manual takeover, payload/current/wave differences and safe recovery. See [validation status](SIMULATION_VALIDATION.md) for the exact evidence already available.
