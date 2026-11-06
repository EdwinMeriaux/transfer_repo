# Controller and observation contract

The controller is a low-level feedback policy: desired forward speed `u*` and counterclockwise yaw rate `r*` plus measured motion produce independent port/left and starboard/right forces. A separate planner would turn a destination into these requests. There is no waypoint planner, obstacle avoidance or position-hold guarantee here; a zero velocity request is not an emergency motor stop.

## Exact trained policy

Source: `loco_mjx` commit `05ffc1b9aa0f016253dff255ad37d7882310f36b`, branch `blueboat/currents-and-comparison-videos`. The checkpoint contains the `urma2.mjx_split` PPO actor and training data. It was trained for **4,194,304 steps**, one seed, 128 parallel environments, corrected `plane.xml`, 200 Hz physics and 50 Hz policy control. Training speed/yaw targets were within ±0.8 m/s and ±0.4 rad/s, with randomized wave forcing, zero current, and no mass/drag domain randomization.

The deterministic actor uses the learned mean; it does not sample Gaussian exploration noise. Network width multiplier is 0.5. Two actuator tokens each contain 51 description fields and four state fields. The presence flag (description index 50) is excluded from the 54-input encoder and masks the state latent. The description/state encoder is 54→128→128 with LayerNorm and GELU; a separate 4→8 GELU state encoder feeds 128 routing groups. Learned temperatures and masked aggregation give 128×8=1024 features. Concatenating 34 general features yields 1058 inputs to the weight-normalized core: 1058→256→128→128→128→128, LayerNorm after the first layer and GELU between hidden layers. Routing decodes two actuator means, clipped to ±10 as in the source actor.

There is no recurrent state, observation history, privileged actor observation or symmetric actor in this checkpoint. The `history_routing` configuration flag is inactive because history inputs are disabled. The only feedback memory supplied here is the **previous raw actor mean**. The stochastic log-standard-deviation weights are preserved in the extracted archive but not used for deterministic inference.

The ONNX graph uses opset 17 / IR 9, float32, input `observation` shaped `[batch,289]`, output `actions` shaped `[batch,2]`. It implements source GELU, LayerNorm fast variance/epsilon and weight normalization directly, without changing the architecture. In deployment the batch is one. Export tests compared 1,024 randomized six-degree-of-freedom inputs against the original JAX actor: maximum absolute raw-action discrepancy **2.116e-6**, tolerance `atol=rtol=2e-5`. This is numerical equivalence over tested inputs, not a proof over all possible states.

## Observations

The canonical complete specification is the bundled `assets/observation_contract.json`: normalized template, per-element divisors, offsets, clipping bounds, group indices and actor configuration. The actor reads the first 110 token features plus these 34 general indices:

```text
220,221,222,223,227,228,229,233,234,235,236,241,242,243,244,
249,250,253,254,255,261,263,264,265,269,270,271,272,273,274,
275,283,285,286
```

| Dynamic values | Policy indices (zero-based) | Physical input | Normalization |
|---|---|---|---|
| Linear velocity | 220–222 | FLU body `[u,v,w]`, m/s | divide by 10 |
| Angular velocity | 227–229 | FLU body `[p,q,r]`, rad/s | divide by 50 |
| Requested velocity | 233–236 | `[u*,0,0,r*]` | divisor 1 |
| Gravity direction | 253–255 | `R(body→ENU)^T [0,0,-1]` | divisor 1 |
| Previous actor actions | 53,108 | raw left/right actor mean | divide by 10 |

The adapter also populates the corresponding critic slots (224–226, 230–232, 237–240, 256–258, 163/218), preserving the original 289-element interface; the actor does not consume those slots. Group updates use `clip(value/divisor - offset, low, high)`, followed by global clipping to ±10. Static morphology, actuator descriptors and non-moving thruster joint features come from the original frozen template. These are fixed model properties, not invented replacements for missing motion measurements.

Absolute heading and position are not dynamic actor inputs. The quaternion is needed to rotate gravity and, for the native Gazebo adapter, world velocities. Heading is implicitly used in that transformation, rather than inserted as an extra yaw feature.

## Frames, measurements and timing

ROS conventions are ENU world and FLU body: x forward, y left, z up. Positive yaw is counterclockwise about up. MAVROS converts the flight controller's NED/FRD conventions. This package expects that conversion already applied; it must not be applied a second time.

`/mavros/local_position/odom` supplies **body-frame linear velocity** (`child_frame_id=base_link`, pose frame `map`); `/mavros/imu/data` supplies body angular velocity and body-to-ENU quaternion (`frame_id=base_link`). This matches the [MAVROS local-position implementation](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/local_position.cpp). Verify actual frame IDs and conventions in the installed version. A sensor-mounted IMU in another frame must be transformed upstream with a validated transform, not merely relabeled.

All three components of velocity and angular velocity are required. Missing telemetry, bad frames, unknown IMU orientation/angular covariance flags, nonfinite values, quaternion norm error >0.02, or component magnitudes >5 m/s or 5 rad/s prevent output. Estimator attitude, horizontal and vertical velocity status must be valid, with no constant-position fallback, GPS glitch or acceleration error. No fallback zero heave/sway/roll/pitch values are used in ROS.

Odometry and IMU require increasing source timestamps, ROS-clock age ≤0.2 s, monotonic receipt age ≤0.2 s and timestamp skew ≤0.1 s. These are upper bounds, not a claim that 200 ms is acceptable control latency: aim for fresh ≥50 Hz measurements and measure the actual delay. A steady-clock 50 Hz control timer and independent watchdog use wall time, even when `use_sim_time` is true. Commands expire after 0.5 s. Incoming Twist has no timestamp, so command freshness is reception-based: avoid queued/replayed command streams.

MAVROS ground-referenced velocity is the intended observation. It is **not water-relative velocity**. Currents change the hydrodynamic load; they cannot be corrected by blindly subtracting a guessed current from the observation. Training used zero current. Real GPS/estimator noise, delay, roll/heave, sensor alignment, payload, thrust dynamics and currents are remaining transfer differences.

## Actions and actuation

For each raw actor output `a`, compute `b=clip(a,-1,1)` then `F=25*b` if positive, else `F=15*b`. Hence forces lie in −15…+25 N. Next-step history uses raw `a`, not `b`, force, deployment-capped force or PWM. History resets at startup or a fault/stop. Applying a conservative deployment force cap changes the closed-loop plant seen by the policy; matching historical RMSE is not expected under a low cap.

Direct-force Gazebo consumes Newton-valued port/starboard requests. The ROS path uses measured per-motor curves to convert force to RC-input PWM-equivalent values, then ArduRover's verified MANUAL/two-paddle path. Motor calibration is essential: RC input is not direct force or necessarily the same as servo-output PWM. No MAVROS velocity setpoint, automatic arming, mode switching or parameter writing occurs in the node. See [safety and calibration](REAL_BOAT_SAFETY.md).
