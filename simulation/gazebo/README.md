# Gazebo pathways

## A. Original direct-force test (reproduced locally)

`run_direct.py` loads the bundled ONNX actor and advances native Gazebo with `TestFixture`, using 0.005 s physics and four steps per policy action. `bridge.py` is extracted from the validated research adapter. It waits for both thruster subscriptions and reads state directly from the simulated link. ArduPilot is removed from the generated **direct-force** world so there is one control authority.

Prerequisites: Gazebo Harmonic with Python `gz.sim8`, `gz.math7`, `gz.msgs10`, `gz.transport13`, plus the `gz-waves1-waves-model-system` and `gz-waves1-hydrodynamics-system` plugins. Use a Python interpreter with compatible Gazebo bindings, and install this repository's inference package into its isolated environment. These optional dependencies are not needed on the deployment controller.

External sources used in the validated installation:

- [ArduPilot/SITL_Models](https://github.com/ArduPilot/SITL_Models), commit `25bc38ed8c6c0345840159a8cbc0b02781d52f3c`.
- [srmainwaring/asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim), commit `ca8629df4e191235753dfae92ef725d30b923364`.

Obtain/build those packages using their upstream instructions and license terms or use your existing matching installation. No upstream CAD meshes, plugin binaries or source trees are redistributed here. The Mac validation used an existing headless plugin build; its build-only changes disabled optional GUI/render/gnuplot targets and fixed Eigen include handling/rpath. Hydrodynamic source was unchanged. Linux users can use the upstream full build. A Gazebo Classic or Fortress installation is not interchangeable with these Harmonic Python bindings.

From this repository's root, set the two external paths to your installed assets/plugins:

```bash
export BLUEBOAT_MODEL_DIR=/path/to/SITL_Models/Gazebo/models/blueboat
export BLUEBOAT_WAVES_LIB=/path/to/asv_wave_sim/install/lib
export GZ_SIM_SYSTEM_PLUGIN_PATH="$BLUEBOAT_WAVES_LIB${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_IP=127.0.0.1
export GZ_PARTITION=blueboat_direct_test
python -m pip install ./ros2_ws/src/blueboat_ppo_controller
python simulation/gazebo/prepare_world.py \
  --blueboat-model "$BLUEBOAT_MODEL_DIR" --output artifacts/worlds/calm.sdf
python simulation/gazebo/run_direct.py \
  --world artifacts/worlds/calm.sdf --reference positive_yaw \
  --seconds 24 --output artifacts/direct-positive-yaw
```

The external path placeholders must be replaced; no machine-specific environment file is shipped. A Conda-native Gazebo installation may additionally need its own `GZ_CONFIG_PATH` and built-in plugin path, as required by that installation. The test runs headless. Outputs are `trajectory.csv` and `summary.json`; output directories must be new so an earlier result is not overwritten.

For the six historical references:

```bash
for reference in constant_speed changing_speed positive_yaw negative_yaw sinusoidal_yaw combined; do
  python simulation/gazebo/run_direct.py --world artifacts/worlds/calm.sdf \
    --reference "$reference" --output "artifacts/benchmark-$reference"
done
```

Every episode settles unpowered for 12 simulated seconds then runs for 24 seconds. `combined` includes yaw requests up to 0.45 rad/s, outside the ±0.4 training/deployment command envelope; it is preserved solely to reproduce the historical benchmark. Do not send that reference unchanged to the ROS node.

`prepare_world.py --waves moderate` or `--waves strong` produces sinusoidal wave physics with amplitudes 0.05 or 0.10 m (crest-to-trough 0.10 or 0.20 m), periods 2.5 or 3 s. Choose a different output world path. These wave choices are separate robustness experiments, not part of the six calm-water reproduction results. The water visual is a flat plane; it does not render the physical wave elevation. Never infer wave physics from this surface graphic.

## B. ArduPilot SITL / MAVROS (implemented, execution pending)

Use Alberto's existing vehicle/world with the ArduPilot plugin and marine physics. **Do not use the generated direct-force world above:** it intentionally removes the ArduPilot plugin. Do not run `run_direct.py` alongside SITL actuation.

The complete integration is:

```text
ROS Twist → exact ONNX policy → left/right Newton requests → measured RC-input conversion
→ MAVROS OverrideRCIn → ArduRover MANUAL/two-paddle mapping → existing Gazebo motor plugin
→ MAVROS body odometry + IMU + estimator status → policy observations
```

The controller package and configuration templates implement this path. The simulator's ESC/plugin conversion, installed firmware and measured paired-thruster behavior must be verified; an RC value is not a force measurement. See [Alberto's step-by-step guide](../../docs/ALBERTO_SETUP.md), [mapping/calibration](../../docs/REAL_BOAT_SAFETY.md), and [topics](../../docs/ROS_INTERFACES.md). Start in shadow mode; actuator launch requires verified calibration and active failsafes. No SITL performance result is claimed in this repository yet.
