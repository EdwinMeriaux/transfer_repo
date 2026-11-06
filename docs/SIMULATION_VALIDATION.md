# Validation status

## What actually ran

Development validation was performed on macOS arm64. This is a simulation and software validation report, not a real-boat claim.

| Check | Result |
|---|---|
| Original checkpoint SHA and ZIP integrity | Passed; exact requested 7,141,893-byte archive |
| Original checkpoint loading | Original JAX/Flax actor loaded during extraction; 1,024 reference outputs captured |
| ONNX vs original JAX means | 1,024 samples, max absolute error 2.116e-6; tolerance atol/rtol 2e-5 |
| Exact original observation adapter equivalence | 1,024 randomized orientations, world velocities, angular rates, commands and previous actions |
| Unit and transport-mocked integration tests | **28 passed, 1 skipped** |
| Wheel build and fresh isolated install | Passed; inference executed from outside repository with bundled assets |
| Native Gazebo direct-force benchmark | Six 24 s episodes, 1,200 controls each, 12 s unpowered settling; all stable |
| Standalone corrected MJX positive turn | 24 s / 1,200 controls, finite state; original result reproduced |
| Real ROS message serialization | **Skipped:** rclpy/ROS 2 unavailable on this Mac |
| Humble colcon build, live ROS graph and MAVROS | **Not run** |
| ArduRover SITL closed loop | **Not run**; calibration and environment checks required |
| Jetson timing and physical BlueBoat | **Not run** |

The local tests cover frame/normalization/previous-raw-action handling, action clipping, model integrity, left/right ordering, 18-channel override/release values, calibration identity and parameter gates, telemetry/command timeout, default shadow behavior, neutral/release and fault latching, independent watchdog, late inference, shutdown and logging failure. `test_node_transport_mock.py` executes the real controller class with mocked ROS transport; it does not validate DDS, MAVROS, flight firmware or real ROS messages. `test_ros_messages.py` uses actual ROS serialization when available and explicitly skips otherwise.

A wheel-only inference smoke test ran 1,000 iterations with CPU ONNX Runtime 1.31.1: median **0.0653 ms**, p99 **0.0832 ms** on this Mac. This does not measure the whole ROS loop, logging/middleware latency, Jetson scheduling or actuator delay. The timing unit test requires p99 inference under 20 ms on the development host; repeat under the intended Jetson workload.

## Reproduced direct-force results

The new standalone ONNX controller uses the original six reference schedules. RMSE includes the complete 24 s tracking interval after settling. Units are m/s and rad/s respectively.

| Reference | Speed RMSE | Yaw-rate RMSE |
|---|---:|---:|
| constant_speed | 0.04872460 | 0.00661912 |
| changing_speed | 0.15255403 | 0.00637229 |
| positive_yaw | 0.03589684 | 0.08938543 |
| negative_yaw | 0.06723302 | 0.03442993 |
| sinusoidal_yaw | 0.04767702 | 0.10567740 |
| combined | 0.06472558 | 0.14881716 |

Mean of episode RMSE: **0.06946851 m/s, 0.06521689 rad/s**. The largest per-episode difference from the source JAX Gazebo result is **2.78e-7 m/s and 1.26e-7 rad/s**. This confirms the extraction preserved the validated direct-force behavior to numerical tolerance in these tests. It is not an independent marine-physics accuracy measurement.

Standalone MJX positive-turn RMSE is **0.03479604 m/s, 0.07351325 rad/s**, versus original JAX **0.03479543 m/s, 0.07351258 rad/s**. This simplified reproduction uses the corrected planar physics and exact actor inputs. Gazebo is full marine rigid-body simulation; MJX does not model vertical/roll/pitch motion in this model.

Machine-readable results and source comparison: [`validation/direct_force_results.json`](validation/direct_force_results.json). Small summary reports are committed; generated worlds, trajectories, environments and binary simulator dependencies stay in ignored `artifacts/` or external installations. Reproduction commands are in [Gazebo](../simulation/gazebo/README.md) and [MJX](../simulation/mjx/README.md).

## Preserved historical research result

The same checkpoint was trained for **4,194,304 steps** in corrected MJX. The historical six-episode mean calm-water errors were:

| Controller/backend | Speed RMSE (m/s) | Yaw-rate RMSE (rad/s) |
|---|---:|---:|
| Retrained policy / MJX | 0.0670 | 0.0617 |
| Retrained policy / Gazebo | 0.0695 | 0.0652 |
| Older policy / Gazebo | 0.0612 | 0.1825 |

These numbers describe simulation-to-simulation transfer after fitting MJX's drag/coupling to Gazebo. They do not establish pool performance. The combined schedule reaches 0.45 rad/s, outside the new training/deployment yaw-request range. The original training included wave forcing, but the table and new reproduction are **calm-water** tests. Wave-enabled world generation is provided separately; no new wave/SITL/hardware result is inferred from the calm test.

## Acceptance still required with Alberto

Run real Humble serialization and colcon installation, verify frames/timestamps/rates and estimator flags in shadow mode, measure Jetson full-loop timing, identify and calibrate each motor through the complete ArduRover/Gazebo path, verify paired motor forces and delay, and inject all timeout/takeover/failsafe conditions. Only then evaluate closed-loop SITL tracking. Hardware requires a separate measured curve and restrained tests before any real-water experiment. Follow [Alberto's guide](ALBERTO_SETUP.md) and [real-boat gates](REAL_BOAT_SAFETY.md).
