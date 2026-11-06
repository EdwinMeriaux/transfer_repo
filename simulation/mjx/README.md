# Corrected planar MJX reproduction

This optional simulation contains the small BlueBoat model, original procedural hull visual and calibrated resistance/wave helper functions from the source research commit. It does not include the training framework. The boat has world-X/Y slide joints and yaw; vertical motion/roll/pitch are constrained. Drag includes nonlinear surge, sway/yaw coupling and forward-speed-dependent yaw resistance calibrated against Gazebo. The two sites apply Newton-valued forces with asymmetric −15…+25 N limits.

In a separate compatible environment (the validation used Python 3.11, JAX 0.7.2 and MuJoCo 3.7.0):

```bash
python -m pip install 'jax==0.7.2' 'mujoco==3.7.0' 'mujoco-mjx==3.7.0'
python -m pip install ./ros2_ws/src/blueboat_ppo_controller
JAX_PLATFORMS=cpu python simulation/mjx/run_mjx.py \
  --speed 0.4 --yaw-rate 0.4 --seconds 24 \
  --output artifacts/mjx-positive-yaw.json
```

Run from the repository root. Use a new output filename. The script drives the copied MJX dynamics with the **same ONNX actor and exact observation adapter** used in deployment, runs 1,200 control frames, and reports speed/yaw RMSE. It is a calm-water reproduction; the included `waves.py` supports the source forcing model but this CLI does not enable waves. The helper also retains current-relative resistance support; the CLI uses zero current.

MJX's planar state supplies exactly zero constrained vertical/roll/pitch quantities. This is a property of this simulator, not permission to replace missing real measurements with zeros. Gazebo and the ROS node use full measured motion. This reproduction is separate from the original full research training/evaluation environment.
