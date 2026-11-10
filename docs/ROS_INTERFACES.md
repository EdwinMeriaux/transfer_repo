# ROS 2 interfaces

Node/package: `blueboat_ppo_controller`. Executables: `ppo_node`, `inference_test`. The latter has no ROS imports. The node uses a single ROS executor plus an independent watchdog thread. ROS 2 Humble message serialization still requires a Linux/ROS execution test; transport-mocked tests do not replace that test.

The separate `thrust_calibration` executable and offline `calibration_analyze` / `calibration_synthetic` tools are described in [THRUST_CALIBRATION.md](THRUST_CALIBRATION.md). Calibration defaults to dry-run, adds `/mavros/imu/data_raw` and `/mavros/rc/out` subscriptions, configurable odometry/body/world velocity selection, `/blueboat/calibration/enable` heartbeat and JSON `/blueboat/calibration/status`. It creates an RC override publisher only in explicitly enabled powered mode with a verified motor interface and requires a sustained rest baseline before excitation. Its telemetry checks do not establish physical frame/mapping correctness automatically.

## Default subscriptions

| Topic | Type | Contract |
|---|---|---|
| `/blueboat/cmd_vel` | `geometry_msgs/msg/Twist` | Only `linear.x` (m/s) and `angular.z` (rad/s); body FLU. Others must be zero. Finite, absolute limits 0.8 and 0.4. Publish continuously, e.g. 10 Hz. |
| `/blueboat/enable` | `std_msgs/msg/Bool` | Actuator enable heartbeat; true must arrive within 0.5 s. False releases owned channels. Not needed in shadow. |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | `map` pose frame, `base_link` twist; all three body linear components. |
| `/mavros/imu/data` | `sensor_msgs/msg/Imu` | `base_link`, quaternion and all three angular rates; known-valid measurements. |
| `/mavros/state` | `mavros_msgs/msg/State` | Connected and system status standby/active, receipt ≤2 s. Actuator requires armed MANUAL. |
| `/mavros/estimator_status` | `mavros_msgs/msg/EstimatorStatus` | Valid attitude and horizontal/vertical velocity, no glitch/error/fallback; receipt ≤2 s. |
| `/mavros/rc/in` | `mavros_msgs/msg/RCIn` | Actuator only: separate receiver enable switch high (1800–2200), receipt ≤0.5 s. |

Motion subscriptions use sensor-data QoS (best effort, volatile); command and enable use reliable depth 1. Publisher QoS compatibility must be checked in the actual ROS graph. No `/clock` source is provided. Use simulation time only if MAVROS timestamps and Gazebo's bridged clock share the same time base. Pausing simulation makes monotonic freshness expire and stops actuation.

## Publications

| Topic | Type | Contract |
|---|---|---|
| `/blueboat/requested_forces` | `std_msgs/msg/Float64MultiArray` | `[left_N,right_N]` after policy clipping/scaling, before deployment cap; valid control frames only. |
| `/blueboat/observation` | `std_msgs/msg/Float64MultiArray` | Exact 289 normalized floats; valid control frames only. |
| `/blueboat/diagnostics` | `std_msgs/msg/String` | JSON status/fault records; successful rows include command, quaternion, body velocity/rates, raw actions, requested/capped forces, RC PWM and timing. |
| `/mavros/rc/override` | `mavros_msgs/msg/OverrideRCIn` | **Actuator mode only**. Exactly 18 uint16 entries. Two owned channels in 1–8; all others 65535 (ignore). Release uses 0 on owned channels only. |

No RC override publisher exists in shadow mode. Forces and observations are monitoring outputs, not ROS actuator topics. Do not bridge `/blueboat/requested_forces` to physical motors while claiming to run shadow mode. The separate native Gazebo script publishes `gz.msgs.Double` Newtons to `/model/blueboat/joint/motor_port_joint/cmd_thrust` and `motor_stbd_joint/cmd_thrust`.

The [MAVROS message definition](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/OverrideRCIn.msg) specifies the 18-entry array. The package deliberately owns only channels 1–8 because the MAVLink release semantics differ for channels 9–18. A release hands control back to the receiver/flight controller; it does not guarantee that the boat stops.

## Read-only service checks in actuator mode

Every 5 s, subject to service completion:

1. `/mavros/vehicle_info_get`, `mavros_msgs/srv/VehicleInfoGet`: require one matching ArduPilot rover/boat and exact sysid/compid/firmware/recorded UID fields from calibration. A recorded zero UID means the FC did not provide one; the remaining identity and pinned-parameter checks still apply.
2. `/mavros/param/pull`, `mavros_msgs/srv/ParamPull`, `force_pull: true`: refresh flight-controller parameters.
3. `/mavros/param/get_parameters`, `rcl_interfaces/srv/GetParameters`: compare all calibration-pinned values.

Checks expire after 15 s. Unavailable services, mismatches or hung queries prevent/stop actuation; restarting is required after an active fault. The node never writes parameters, arms or sets a mode. These service names are configurable because namespaces/installations differ. Inspect them with `ros2 service list -t`. Source contracts: [MAVROS parameter plugin](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/param.cpp), [VehicleInfoGet](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/srv/VehicleInfoGet.srv).

## Parameters and logs

Start with `config/controller.yaml`. Every topic in this document is configurable using the corresponding `*_topic` node parameter; vehicle/parameter services use `vehicle_info_service`, `param_pull_service`, `param_get_service`. Diagnostics/forces/observation names can be added to the YAML. Other parameters are `world_frame`, `body_frame`, the three safety timeouts, and `log_directory`.

`model_directory` defaults to bundled assets; an override must contain the model, contract and matching manifest. `log_directory` defaults to `~/.ros/blueboat`, with one unique JSONL file per run. Save this file alongside the ROS bag and calibration/evidence. `scripts/analyze_log.py` computes sample-based speed/yaw RMSE and inference p99 over valid rows; it excludes fault rows and does not imply those intervals tracked correctly. Record the full test duration and fault counts when reporting results.

Launch arguments `mode`, `enable_actuators`, `deployment_target`, `calibration_file` and `use_sim_time` explicitly override the YAML. Their safe defaults remain shadow, false, sitl, empty and false, even if a copied YAML contains actuator settings.
