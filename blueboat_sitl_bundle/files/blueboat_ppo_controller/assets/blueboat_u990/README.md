# BlueBoat: a policy trained on your identified boat

A specialist policy for the BlueBoat (`blueboat`), trained on our model of the boat only: ArduPilot's SITL model with your Gazebo-matched resistance (your Gazebo-matched coupled v2 resistance with the coupled yaw term, on our main at 19fcd78a7). Trained for 519 million steps (990 updates), training commit `19fcd78a7` of our repository, with our full randomisation (hidden draws of mass, size and every water coefficient, thrust lag of 0 to 150 ms, action delays, pushes, waves and currents).

Our model follows your v2 model open-loop (thrust held from rest, steady state after 40 s; surge in m/s, yaw rate in rad/s):

| thrust left, right (N) | your v2 model | our model |
|---|---|---|
| 10, 10 | 1.427, 0.000 | 1.406, 0.000 |
| 25, 25 | 2.448, 0.000 | 2.409, 0.000 |
| -15, -15 | -1.820, 0.000 | -1.793, 0.000 |
| 0, 10 | 0.922, 0.047 | 0.910, 0.048 |
| 0, 25 | 1.631, 0.103 | 1.607, 0.104 |
| -7.5, 15 | 0.755, 0.197 | 0.746, 0.200 |
| -15, 25 | 0.887, 0.449 | 0.874, 0.458 |
| -10, 10 | -0.021, 0.591 | -0.021, 0.590 |
| -15, 15 | -0.035, 0.831 | -0.035, 0.830 |
| -15, 0 | -1.194, 0.065 | -1.177, 0.066 |

The remaining 1 to 2 % is the air drag our simulator adds.

The policy runs at 50 Hz. It reads the boat's velocity, the IMU's angular velocity and the gravity direction, and returns the thrust of each thruster in N.

## Files

- `inference.py`: the runtime with JAX (`policies/blueboat_blueboat_u990.model`); `inference_onnx.py`: the same with ONNX Runtime (`policies/blueboat_blueboat_u990.onnx`), which needs only NumPy and ONNX Runtime.
- `policy_inputs.py`, `policy.py`, `inference_torch.py`: input construction, the network, and its PyTorch version used for the ONNX export.
- `deploy_config.json`, `descriptions/`: the description of the boat that the policy reads. Keep them next to the runtime.
- `model/plane_phys.xml`: the MuJoCo model the policy was trained on.
- `requirements.txt`: Python packages of the JAX runtime.

## Interface

```python
import numpy as np
from inference import DeploymentPolicy  # or: from inference_onnx import DeploymentPolicy

policy = DeploymentPolicy()
policy.reset_history()
previous_action = np.zeros(2, dtype=np.float32)

# every 20 ms:
command = policy.fix_command((vx, 0.0, 0.0, yaw_rate))
action = policy.act(policy.nominal_joint_positions, np.zeros(2), previous_action, gyro, gravity, command, velocity, command)
previous_action = action
port, starboard = policy.actuator_controls(action)
```

Call `reset_history()` and set `previous_action` to zero whenever control starts or restarts, and run the step every 20 ms. All vectors are in the IMU frame, x forward, y left, z up (FLU): `velocity` the boat's velocity over ground in m/s (GPS or the autopilot's velocity estimate, turned into the boat's frame), `gyro` in rad/s, `gravity` the unit vector of gravity (about (0, 0, -1) when level). From ArduPilot's body frame (x forward, y right, z down), negate y and z.

`actuator_controls` returns the thrusts of the port and the starboard thruster in N, positive forward: 7.5 N per unit of the policy's action, clipped to -15 to 25 N. Send them to the ESCs through the thrust curve of your thrusters. In training each thrust followed its command with a lag of 75 ms (drawn from 0 to 150 ms); the runtime does not filter.

## Commands

- **Forward and backward**: -1 to 1 m/s. There is no sideways command; pass 0.
- **Yaw rate**: up to 0.98 rad/s either way; positive turns left (counter-clockwise seen from above). On the spot the thrusters reach about 0.83 rad/s.
- Commands below 10 % of these limits (0.1 m/s, 0.098 rad/s) count as zero (`fix_command`). A zero command asks for zero velocity over ground.

## Simulation checks

Our training environment with this policy, 10 s per command from rest, mean over 3 to 10 s (forward and sideways speed in m/s, yaw rate in rad/s):

| command | forward, yaw rate | measured: forward, sideways, yaw rate |
|---|---|---|
| hold | 0, 0 | -0.00, 0.00, 0.00 |
| forward | 0.25, 0 | 0.24, 0.00, 0.00 |
| forward | 0.5, 0 | 0.49, -0.00, -0.00 |
| forward | 0.8, 0 | 0.77, 0.00, -0.00 |
| backward | -0.25, 0 | -0.25, 0.00, 0.00 |
| backward | -0.5, 0 | -0.51, -0.00, -0.00 |
| turn left on the spot | 0, 0.4 | -0.03, 0.02, 0.44 |
| turn right on the spot | 0, -0.4 | -0.03, -0.02, -0.44 |
| turn left on the spot | 0, 0.8 | -0.05, 0.04, 0.81 |
| turn right on the spot | 0, -0.8 | -0.05, -0.04, -0.81 |
| forward, turn left | 0.5, 0.4 | 0.46, 0.00, 0.41 |
| forward, turn right | 0.5, -0.4 | 0.46, -0.00, -0.41 |
| full forward | 1, 0 | 0.96, 0.00, -0.00 |
| backward | -0.8, 0 | -0.81, 0.00, 0.00 |
| full backward | -1, 0 | -1.01, 0.00, 0.00 |
| full turn left on the spot | 0, 0.98 | -0.05, 0.04, 0.83 |
| full turn right on the spot | 0, -0.98 | -0.05, -0.04, -0.83 |
| slow forward | 0.1, 0 | 0.10, 0.00, 0.00 |
| slow forward | 0.15, 0 | 0.15, 0.00, 0.00 |
| slow forward | 0.2, 0 | 0.20, -0.00, -0.00 |
| slow backward | -0.15, 0 | -0.15, -0.00, -0.00 |
| slow turn left on the spot | 0, 0.15 | -0.01, 0.01, 0.18 |
| fast forward, turn left | 0.8, 0.6 | 0.74, -0.01, 0.53 |
| backward, turn left | -0.5, 0.4 | -0.52, 0.04, 0.38 |

The runtime in this folder gives the training environment's actions within 9.54e-07 (`validate_equivalence.py`). These are simulation results; the first water test decides.

## Provenance

- Run: blueboatx blueboat, ias job 211712, 990 updates; training commit `19fcd78a7a0d86a2421d531a8a5e576c1411ac6b`.
- Bundle `blueboat_blueboat_u990.model`: SHA256 `415350f4b737aa562a53beeea7f96c42eb2b97baf5cabbf2d7cbe38f0aae7c11`, actor SHA256 `6a6635d3599ce75e55f865b624ee7d9a954a36453088b7c08b90e5f6d1579be7`, checkpoint SHA256 `dd30988873a39fe41a0442ce9fb55f17181c4bfdf9805d59ae161691843cf9c6`.

## Your deployment code

This package has the policy and the runtime that turns the boat's measurements into the policy's inputs. Please write the deployment code for the boat (the node that reads the autopilot's state, calls the policy every 20 ms and sends the two thrusts) in our repository under `deployment/blueboat/` and open a pull request.
