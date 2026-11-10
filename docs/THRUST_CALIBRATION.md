# Alberto: IMU-based effective thrust calibration

**Alberto's installation:** use [ALBERTO_BOAT.md](ALBERTO_BOAT.md). Its `direct_mavros` path publishes directly from the Jetson to `/mavros/rc/override`; there is no custom gateway or grant process. The receiver/switch requirements below apply only to the generic `rc_receiver` path. Direct MAVROS instead requires one command publisher, stopped QGC joystick transmission, explicit ROS enable, tested operator takeover and independent flight-controller timeout/failsafe behavior. All measurement, telemetry, evidence and calibration limits still apply.

This module measures the **initial effective surge response** to bounded RC-input PWM pulses. It does not measure static propeller thrust. Total wet boat mass, including battery and payload, is supplied as the ROS `mass_kg` parameter. The node, YAML and launch file default to **dry-run** with no motor publisher. The shipped mass is `0.0`: dry-run/telemetry are allowed, powered operation is refused.

Local status: **84 tests passed, 4 skipped** on macOS; see [the validation record](validation/thrust_calibration_tests.json). ROS 2 Humble serialization and the three live ArduPilot SITL tests require Alberto's Linux installation. **No SITL or physical-boat calibration has been completed by this change.** Both templates remain unverified. The model, observation adapter, action scaling and training environment are unchanged.

## Files and the PPO pipeline

All deployment code is inside the existing `ros2_ws/src/blueboat_ppo_controller/` package. It needs no MJX/JAX/MuJoCo checkout.

| File in `blueboat_ppo_controller/` | Role |
|---|---|
| `calibration_node.py` | ROS telemetry, source-time/rate/frame checks, operator gates, finite PWM procedure, watchdog and CSV/events. Separate executable; does not run PPO. |
| `thrust_calibration.py` | ROS-independent pulse scheduler, sustained-rest gate, clock/rate checks and CSV schema. |
| `calibration_analysis.py` | Offline gravity/bias/lever-arm correction, rejection, effective-force estimates, monotonic fits, uncertainty, plots and a PPO calibration draft. |
| `calibration_synthetic.py` | Reproducible fake data; never actuation evidence. |
| `thrust_lookup.py` | Inverse effective Newtons → input PWM, bounded to measured regions. |
| `motor_interface.py` | Existing identity/mapping/failsafe checks extracted from `MotorAdapter`, without requiring a force curve. Shared by calibration and PPO. |
| `motor_adapter.py` | PPO integration point. Existing force curves still work; optional reviewed effective lookup is guarded by additional checks. |

PPO remains `policy_runtime.py → requested left/right N → MotorAdapter.convert() → existing rc_override() → MAVROS/ArduRover MANUAL two-paddle inputs`. No new subscriber changes PPO's observations or actions. The calibration node and PPO actuator node must never control the vehicle simultaneously. The calibrator refuses another publisher on its RC override topic; also inspect the FC/GCS for other MAVLink control sources, which ROS discovery cannot see.

Implementation audit: `test_callbacks_csv_estimator_and_ppo_inverse_end_to_end` in `tests/test_calibration_transport_mock.py` exercises production IMU/attitude/velocity/output callbacks, sustained rest, all 36 default excitation trials, the actual CSV writer, mass inherited from the session, effective-force fitting, generated PPO draft rejection and a reviewer-accepted temporary inverse mapping. It checks known synthetic forces and unchanged Newton-valued requests. This is transport-mocked evidence, not a ROS/SITL or physical test. Separate tests exercise the real default constructor without overriding its mode, baseline disturbances, hardware refusal without completed SITL acceptance, watchdogs and inverse bounds.

## Install and run the offline checks first

Use the checkout containing this change. Follow [ALBERTO_SETUP.md](ALBERTO_SETUP.md) for the existing Humble workspace and MAVROS/SITL installation. These commands use bash, Ubuntu's Python 3.10, and `~/gazebo_blue_boat`:

```bash
cd ~/gazebo_blue_boat
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
python -m pip install './ros2_ws/src/blueboat_ppo_controller[calibration]'
python -m pytest -q
cd ~/blueboat_ws
python -m colcon build --symlink-install --packages-select blueboat_ppo_controller
source install/setup.bash
ros2 pkg executables blueboat_ppo_controller
```

The plotting extra installs Matplotlib. NumPy/SciPy are already package dependencies. The three new executables are `thrust_calibration`, `calibration_synthetic` and `calibration_analyze`; only the first imports ROS.

Exercise the complete offline path on any supported Python installation, without ROS:

```bash
cd ~/gazebo_blue_boat
mkdir -p artifacts/calibration-demo
calibration_synthetic artifacts/calibration-demo/synthetic.csv --mass-kg 42.5
calibration_analyze artifacts/calibration-demo/synthetic.csv \
  --mass-kg 42.5 --left-neutral 1500 --right-neutral 1500 \
  --metadata artifacts/calibration-demo/synthetic.json \
  --output artifacts/calibration-demo/report
```

Here **42.5 kg is only a synthetic test input**. Supply the actual mass for every real analysis. Output directories must be new, preserving previous reports. Expect 36 accepted fake trials and six fitted branches. Inspect `timeseries.png`, `lookup.png`, `trials.json` and `lookup.json`. The lookup is marked unverified and not reviewed for independent motors; the PPO adapter rejects synthetic-session metadata.

## Verify the actual MAVROS graph in telemetry mode

Start the existing working Gazebo + ArduRover SITL + MAVROS stack using its installation-specific launch commands. This repository does not supply that stack. Start with propulsion isolated for real-hardware telemetry checks. In sourced terminals:

```bash
ros2 topic list -t
ros2 service list -t
ros2 topic info /mavros/imu/data_raw --verbose
ros2 topic info /mavros/imu/data --verbose
ros2 topic info /mavros/local_position/odom --verbose
ros2 topic info /mavros/rc/out --verbose
ros2 topic echo /mavros/imu/data_raw --once
ros2 topic echo /mavros/imu/data --once
ros2 topic echo /mavros/local_position/odom --once
ros2 topic echo /mavros/rc/out --once
ros2 topic hz /mavros/imu/data_raw
```

Run `ros2 topic hz` separately for attitude, velocity and RC output; each runs until Ctrl+C. Save the installed MAVROS version/revision and firmware identity. Topic existence alone is insufficient. The ROS 2 [MAVROS IMU plugin](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/imu.cpp) declares raw orientation unavailable and transforms gyro/acceleration into FLU; the [local-position plugin](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/local_position.cpp) publishes body-frame twist in odometry and separate body/world velocity topics. Check your installed version and live messages; the `ros2` source branch is not proof of your installation's behavior.

```bash
mkdir -p ~/gazebo_blue_boat/artifacts/calibration
cp ~/gazebo_blue_boat/ros2_ws/src/blueboat_ppo_controller/config/thrust_calibration.yaml \
  ~/gazebo_blue_boat/artifacts/calibration/run.yaml
ros2 launch blueboat_ppo_controller calibration.launch.py \
  config:=$HOME/gazebo_blue_boat/artifacts/calibration/run.yaml mode:=telemetry
```

In another terminal:

```bash
ros2 topic echo /blueboat/calibration/status
ros2 node info /blueboat_thrust_calibration
```

Telemetry and dry-run modes create **no RC override publisher**. Status contains the discovered publisher names/types/QoS, rolling receipt rates and largest gaps, source timestamps, source skew, frame IDs, rest verification and CSV path. Rates are measured over a rolling two-second window; powered preflight needs at least one second of motion samples. Defaults require raw ≥40 Hz, attitude ≥20 Hz, velocity ≥10 Hz, output ≥5 Hz; source age ≤0.2 s, cross-stream skew ≤0.05 s, largest motion receipt gap ≤0.15 s. These are necessary gates, not a promise that low-rate telemetry passes: aim for **50 Hz or more on all motion streams** to satisfy skew and cover the short initial window. Offline analysis also rejects raw source gaps >0.06 s. Do not relax gates to mask missing data.

Verify with known motions and rest measurements:

- Raw IMU and attitude are aligned with the configured body FLU axes: x forward, y port, z up. Attitude is a body-to-ENU quaternion. Near level at rest, a specific-force accelerometer normally reads about **+9.80665 m/s² on z**, not zero. Inspect tilt responses to establish gravity sign, scaling and mounting orientation. Do not use `/imu/euler` as an ENU quaternion.
- Default velocity is `nav_msgs/Odometry`: `header.frame_id=map`, `child_frame_id=base_link`, twist in FLU. Alternatives are `velocity_type: body_twist` with `/mavros/local_position/velocity_body`, or `world_twist` with `/mavros/local_position/velocity_local`, both `geometry_msgs/TwistStamped`. World twist is rotated into FLU with the checked attitude. Select an actual available estimate; the node does not fabricate missing velocity.
- Exact configured frames must match; the module does not apply TF or relabel a differently aligned IMU. A validated upstream transform is needed for another sensor orientation. Fused velocity/attitude can share IMU errors, so their agreement is a consistency check, not independent ground truth.
- Header timestamps must advance, be neither future nor stale, and share the node's ROS clock. ROS time is used for source checks; monotonic time governs scheduling, heartbeat expiry and watchdogs. Latest attitude/velocity within the skew bound are joined to each raw sample; there is no extrapolation. Preserve original streams in a bag to investigate synchronization errors.
- Use `use_sim_time:=true` only if `/clock`, MAVROS headers and node time have been verified to agree. A simulation pause/clock reset stops powered operation. Do not switch clocks to clear a timestamp fault.

Every raw callback records CSV, including invalid rows and their rejection reason. CSV contains source/receipt/ROS/command timestamps, expected trial end, trial/phase/mode, planned and actually published RC-input PWM, complete RCOut channels plus mapped motor outputs when powered, acceleration, angular velocity, quaternion and body velocity. The `.json` session sidecar stores parameters and the verified interface profile; `.events.jsonl` records phase changes and periodic diagnostics. `/mavros/rc/out` (`mavros_msgs/msg/RCOut`) is FC-reported servo PWM, **not an ESC or shaft-force sensor**. The [MAVROS RC plugin](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/rc_io.cpp) synchronizes its timestamp from `SERVO_OUTPUT_RAW`; verify that time base alongside the motion streams. Zero/unused servo slots are preserved. Telemetry-only mode retains the full output array because motor wiring has not yet been accepted.

Record original topics as well:

```bash
ros2 bag record -o ~/gazebo_blue_boat/artifacts/calibration/telemetry-bag \
  /mavros/imu/data_raw /mavros/imu/data /mavros/local_position/odom \
  /mavros/state /mavros/estimator_status /mavros/rc/in /mavros/rc/out \
  /mavros/rc/override /blueboat/calibration/enable /blueboat/calibration/status
```

Use a new bag directory per run; substitute configured topic names, add `/clock` when appropriate. Preserve CSV, both sidecars, bag, firmware/plugin versions, parameter dump, battery conditions, payload and weather/current notes together.

## Establish a verified motor interface before powered calibration

The existing two-paddle RC route is **source-reviewed, not SITL-verified in this checkout**. Read [REAL_BOAT_SAFETY.md](REAL_BOAT_SAFETY.md), including its parameter list and manual takeover procedure. Copy `config/motor_interface.template.json` into `artifacts/calibration/`. It deliberately contains no force curves, no measured neutral, and no verified flags. This separates motor-interface evidence from force estimation and avoids a circular requirement for an already calibrated force curve.

Use the established operator workflow with propulsion safely isolated/restrained as appropriate to establish left/right wiring, input channels, actual servo outputs, forward signs, neutral, and independent/paired PWM response. This restrained test verifies motor control; **do not use restrained-hull acceleration as a free-hull thrust trial**. Simulator plugins, firmware mixing, RC normalization and ESC deadband can make commanded input PWM differ from actual output PWM.

Populate the profile with measured identity (`sysid`, `compid`, exact `flight_sw_version`, version label), all `expected_parameters`, `rc_channel` and `servo_output` for each side, measured input `neutral_pwm`, measured actual `servo_neutral_pwm`, and both `forward_pwm_sign` and `servo_forward_pwm_sign` (`1` or `-1`, independently measured). Hardware UID is not part of capture or runtime identity. A separate RC enable channel is required only for `rc_receiver`; `direct_mavros` uses no invented receiver switch. The numbers 1/3/7 in the template are candidate locations only. Pin the full RC/servo/motor/mixing/mode/failsafe parameter set from the existing safety guide, plus installation-specific authorization/GCS and failsafe settings. Example read-only queries:

```bash
ros2 service call /mavros/vehicle_info_get mavros_msgs/srv/VehicleInfoGet \
  '{sysid: 0, compid: 0, get_all: false}'
ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull '{force_pull: true}'
ros2 service call /mavros/param/get_parameters rcl_interfaces/srv/GetParameters \
  '{names: [PILOT_STEER_TYPE, RCMAP_ROLL, RCMAP_THROTTLE, RC_OVERRIDE_TIME]}'
sha256sum ~/gazebo_blue_boat/artifacts/calibration/motor-interface-evidence.json
```

Save the actual mapping/takeover/failsafe report under the profile's relative `evidence_file` and set its SHA256. Flags `left_right_verified`, `neutral_verified`, `manual_takeover_tested`, `override_timeout_tested`, `gcs_failsafe_tested` and `paired_motor_test_passed` must reflect completed tests. Here the paired test establishes PWM-path separability; it does not establish force additivity. Also require `telemetry_conventions_verified` and `imu_placement_verified`, backed by observed frame/unit/gravity behavior and IMU-to-centre-of-mass geometry. Mark the **interface** `verified: true` only after those checks. Synthetic pytest profiles cannot substitute for that evidence.

Hardware additionally requires captured identity, its own interface evidence, `sitl_test_passed`, `restrained_motor_test_passed` and `sitl_calibration_test_passed`. Identity uses exact system/component, firmware, vehicle type and pinned parameters; UID is ignored. The calibration SITL tests below must be completed first. No node sets those flags for you. Required FC safeguards include tested positive `RC_OVERRIDE_TIME ≤0.5 s`, enabled GCS failsafe, MANUAL two-paddle mapping and pinned motor/RC settings. A separate receiver override-enable (`RCn_OPTION=46`) and receiver failsafe apply only to `rc_receiver`. Direct MAVROS requires a single command source, stopped QGC joystick output and tested takeover/loss behavior. The node re-reads identity/parameters every five seconds; checks expire after 15 seconds. It never writes parameters, changes mode or arms. The [ArduPilot RC-input documentation](https://ardupilot.org/dev/docs/mavlink-rcinput.html) describes source authorization and receiver fallback; release transfers authority and does not guarantee braking.

Output feedback is required by default. It must be fresh, neutral before ownership and during settled neutral periods, have a driven response of the expected sign after the grace interval, and keep the idle side neutral in single-motor tests. For an installation genuinely lacking RCOut, disabling it requires both `require_output_feedback: false` and `output_feedback_unavailable_accepted: true` in the evidence-backed profile, plus the same explicit analysis override. This reduces validation and must be documented.

## Dry-run, then live ArduPilot SITL acceptance

First stop telemetry and preview the finite plan against live telemetry, without any motor publisher:

```bash
ros2 launch blueboat_ppo_controller calibration.launch.py \
  config:=$HOME/gazebo_blue_boat/artifacts/calibration/run.yaml mode:=dry_run
```

Status/events show the left/right/symmetric planned values. Actual-command CSV fields stay empty, and analysis rejects dry-run trials. The default plan uses `[-40, -60, 40, 60]` PWM offsets, three repeats, a 2 s neutral baseline, a 0.5 s pulse and 3 s recovery. This supplies two magnitudes in each direction for fitting if the responses are resolved. For initial interface checks, reduce the plan to `[-40, 40]`; one level per direction is insufficient for a lookup. `delta_pwm` is signed propulsion direction: each side's independently measured input forward sign converts it to an RC PWM offset. Bounds are configurable, with a hard ±250-count maximum and ≤1 s pulses; the default maximum is ±60 counts. **PWM limits are not Newton limits.** A pulse may be inside the ESC deadband and produce no resolved estimate.

Before every pulse, fresh neutral-baseline samples must verify continuous rest for `rest_duration_s` (default 1 s): body speed ≤0.08 m/s, angular-rate norm ≤0.10 rad/s and gravity-corrected acceleration norm ≤0.6 m/s². Raw sample gaps larger than `rest_max_gap_s` (default 0.06 s), disturbances or missing synchronized attitude/velocity reset that interval. Powered samples also require commanded neutral and, by default, measured neutral output. A single quiet measurement cannot enable a pulse. If the interval is incomplete when the scheduled pulse begins, the procedure aborts and releases; it does not silently run a shortened baseline. Events record rest verification at each pulse transition.

The live pytest suite must use a dedicated, isolated **SITL-only ROS domain/network**, with no physical FC connected. Stop the calibrator and PPO actuator nodes. Start the established SITL stack; using your normal operator workflow, select MANUAL, arm SITL, keep simulated receiver motor inputs neutral, and enable its separate simulated override switch. The tests do not perform those actions. Set actual SITL mass and the verified SITL profile:

```bash
cd ~/gazebo_blue_boat
export BLUEBOAT_RUN_POWERED_SITL=1
export BLUEBOAT_SITL_INTERFACE="$HOME/gazebo_blue_boat/artifacts/calibration/sitl-interface.json"
export BLUEBOAT_SITL_MASS_KG='REPLACE_WITH_ACTUAL_SIMULATED_MASS'
export BLUEBOAT_SITL_CONFIG="$HOME/gazebo_blue_boat/artifacts/calibration/run.yaml"
python -m pytest -q -m sitl --basetemp=artifacts/calibration/sitl-acceptance
unset BLUEBOAT_RUN_POWERED_SITL
```

`--basetemp` is pytest-owned and removed on reuse; use a new directory for each acceptance run. These tests deliberately publish the calibration enable heartbeat and short pulses after preflight. They check no output before software enable, actual independent left/right and symmetric responses, completion/release, software takeover with a latched stop, and actual return to neutral after `SIGSTOP` stalls the companion process (testing FC override timeout). They preserve node logs and CSVs under that directory. Their short one-repeat runs validate transport/control, not a fitted calibration.

Also perform the existing manual SITL acceptance cases: physical/simulated switch takeover, flight-mode change, disarming, receiver loss, GCS/network/MAVROS loss, estimator invalidity, telemetry loss, conflicting command sources, battery/system failsafe and process termination. Do not disable failsafes to obtain a passing result. Record measured fallback time and actual outputs/applied Gazebo forces. These fault injections depend on Alberto's simulator/receiver setup and are not claimed as covered by the three automated live tests. Synthetic/mocked tests do not replace this step.

## Collect bounded calibration pulses

After SITL acceptance, configure `run.yaml` with the actual measured `mass_kg`, topics, bounds and log directory. For a fit, collect **at least two distinct resolved PWM magnitudes per direction and three clean repeats per magnitude/side**. For example, after the first bounds are confirmed safe, `delta_pwm: [-40, -60, 40, 60]` stays inside the default ±60 limit. Increase only with measured evidence and operator judgment. Configure `sides` to `[left]`, `[right]`, `[symmetric]` or all three. Recovery and neutral baseline must allow the boat to settle.

```bash
ros2 launch blueboat_ppo_controller calibration.launch.py \
  config:=$HOME/gazebo_blue_boat/artifacts/calibration/run.yaml \
  mode:=powered enable_powered:=true deployment_target:=sitl \
  interface_file:=$HOME/gazebo_blue_boat/artifacts/calibration/sitl-interface.json
```

Launch defaults override copied YAML to dry-run/disabled/SITL unless explicitly supplied. The operator must select MANUAL, arm externally, and enable the separate receiver switch. Only after preflight and with the operator ready:

```bash
ros2 topic pub --rate 10 /blueboat/calibration/enable std_msgs/msg/Bool '{data: true}'
```

This software heartbeat expires after 0.5 s. Before each pulse the node requires near-rest velocity/rate and a plausible gravity-corrected baseline. Speed, angular rate, tilt, source timing, graph, estimator, state, parameters and actual outputs are checked continuously. Faults latch until deliberate restart; the independent monotonic watchdog attempts neutral then release. Manual takeover/disarming/enable release hands authority back immediately. A completed plan releases and cannot start again. Stop the true heartbeat publisher before sending a one-shot false; then follow the tested receiver takeover/disarm workflow:

```bash
ros2 topic pub --once /blueboat/calibration/enable std_msgs/msg/Bool '{data: false}'
```

For the physical boat, repeat with its separately verified hardware profile and `deployment_target:=hardware` only after all SITL and restrained control checks. IMU response trials require a free hull in a controlled water area, an on-site operator with tested takeover, low speed relative to water, sufficient space and no tow-line load/contact. Do not infer a useful acceleration calibration from a tied-down boat.

## Estimate, fit and inspect uncertainty

Stop collection and copy `config/analysis.template.json` into the run directory. Set measured `imu_offset_flu_m` (COM → IMU, body FLU), uncertainty of total mass, and confirmed acceleration semantics. Default `specific_force` adds body-projected gravity; `linear_acceleration` is only for a verified upstream gravity-removed signal. Analysis inherits mass and acceleration semantics from the session sidecar; the analysis JSON may override semantics. Run with the CSV/sidecar and measured input neutrals:

```bash
calibration_analyze /absolute/path/calibration-RUN.csv \
  --metadata /absolute/path/calibration-RUN.json \
  --left-neutral ACTUAL_LEFT_NEUTRAL \
  --right-neutral ACTUAL_RIGHT_NEUTRAL \
  --settings /absolute/path/analysis.json --output /absolute/path/new-report
```

`mass_kg` is taken directly from the recorded ROS parameter. Without a sidecar, supply `--mass-kg ACTUAL_TOTAL_MASS_KG`; there is no fixed boat-mass fallback. An explicit `--mass-kg` overrides a recorded value, and lookup `mass_provenance` retains both values and identifies a mismatch for review. A missing, nonpositive or nonfinite mass is rejected.

The initial default window is 0.08–0.30 s after the later of command onset and first FC-output response. Check the real actuator/propeller delay and change the window deliberately; FC PWM response is not propeller spin-up measurement. Incomplete or insufficiently sampled windows are rejected. The estimator subtracts median neutral acceleration after gravity and rigid lever-arm correction:

```
a_COM_body = specific_force + R_body_from_ENU * [0,0,-g]
             - angular_acceleration × r - omega × (omega × r)
delta_a_forward = mean(initial a_COM_body.x) - median(neutral a_COM_body.x)
F_effective = mass_kg * delta_a_forward
```

The gyro derivative supplies angular acceleration; derivative noise and mounting uncertainty limit that correction. Rejections include invalid/stale/reordered/gapped telemetry, misaligned frames, incomplete trials, missing/unchanged actual output, coupled idle output, excessive actuator delay, moving baseline, speed/drag limits, tilt/yaw/angular acceleration, waves/vibration, transverse disturbance, IMU/velocity disagreement, wrong force sign and response below twice the scatter uncertainty. `trials.json` retains reasons for all rejected trials. Defaults can be overridden through the analysis JSON, but gate changes need experimental justification.

Per direction and side, repeated accepted trials are combined and fitted with weighted isotonic regression. Forward and reverse are separate; no polynomial extrapolation is used. Lookup records retain repeat count, raw means, fitted values, one-sigma trial scatter and supplied mass uncertainty. Scatter is **not divided by sample count**: adjacent IMU samples are correlated. Uncertainty does not include unknown attitude, drag, added-mass, placement, current, battery or propeller model bias. Isotonic plateaus are unresolved for inversion; missing directions, insufficient repeats, large PWM gaps, neutral-to-first-force gaps and forces above the largest measured value remain unmeasured. Plots compare commanded/FC-output PWM, raw and corrected initial acceleration, effective force, fitted curves and uncertainty bars.

**`F = ma` is approximate for a boat.** A simple surge balance includes `(mass + added_mass) * acceleration`, water-relative drag and other loads. The default `mass_kg * delta_a` is therefore an effective response estimate, not true static propeller thrust. Optional `added_mass_kg` and its uncertainty use `(mass_kg + added_mass_kg) * delta_a`; this is an explicitly supplied model correction. IMU surge data alone cannot identify thrust and added mass separately. Low GPS speed also does not establish low water-relative speed in a current. Drag, wind and waves may survive baseline subtraction. Bias cancellation is local, not a full accelerometer calibration.

Single-motor trials report an **isolated effective surge contribution** only under valid isolation, negligible background/rotation and consistent inertial response. A twin-thruster boat usually yaws during them; if yaw cannot be kept small the trials should be rejected and independent motor forces remain unidentifiable. Yaw-rate data alone does not supply yaw inertia, added yaw inertia, moment arms, or hydrodynamic damping. Symmetric trials measure total boat force response and are never automatically divided by two. A paired-additivity report compares repeated symmetric responses with the sum of isolated responses; this is necessary but insufficient for independent PPO force mapping. Review unequal and opposite-sign pairs and transient cross-effects too. Use additional instrumentation or a suitable load-cell experiment when those assumptions cannot be supported.

## Using the inverse mapping in PPO

Analysis emits a **draft**, never an automatically enabled controller calibration. With a recorded motor-interface profile it also constructs `ppo_calibration.draft.json`, referencing the actual CSV and lookup with their checksums and the measured motor mapping. Its global `verified`, `effective_mapping_accepted` and `independent_effective_mapping_verified` flags are false. The motor adapter refuses it until review; this is the actual PPO input format, not an example copied by hand. Synthetic demonstrations without a motor-interface profile produce only the lookup. Preview the inverse offline with explicit acknowledgement of unreviewed assumptions:

```python
from blueboat_ppo_controller.thrust_lookup import ThrustLookup
lookup = ThrustLookup('/absolute/path/report/lookup.json')
pwm, applied_effective_N = lookup.inverse(5.0, 'left', allow_unreviewed=True)
```

This example only works if 5 N is inside a resolved measured branch. Normal calls require `independent_mapping_reviewed: true`. The `out_of_range` default is `reject`, including gaps below the first measured force and absent directions. Explicit `out_of_range='clip'` clips a large force to the measured maximum and a force below the first measured point to neutral, returning the applied effective force. Commands inside the explicit `zero_force_band_N` map to measured neutral. Nonfinite commands and internal unmeasured PWM intervals are rejected. PWM is rounded and bounded to 1000–2000; lookup knots must also match the vehicle's narrower verified RC limits.

To use a reviewed effective lookup, preserve the CSV and original mapping evidence, and review the generated `ppo_calibration.draft.json`. Its 3 N request cap is an unverified starting limit and may lie outside resolved coverage; review the cap and measured domains together. Keep every identity/parameter/takeover/failsafe/paired/hardware gate. The generated paths are relative to the report directory; keep that structure intact or update paths and hashes when archiving. Review these fields:

| Field | Required evidence/value |
|---|---|
| `evidence_file`, `evidence_sha256` | Actual run CSV, relative path and hash, matching lookup's recorded CSV hash. Preserve original motor-interface evidence separately with the run. |
| `effective_lookup_file`, `effective_lookup_sha256` | Relative path to the reviewed lookup and hash of its final contents. |
| `effective_mapping_accepted` | Explicit acceptance of inertial-response vs. actual PPO force semantics and their limitations. |
| `independent_effective_mapping_verified` | Reviewed isolation/geometry/paired/transient evidence supporting two motor mappings. |
| `maximum_abs_force_N` | Deliberate deployment cap, ≤25 N; does not waive measured-region checks. |

Only a reviewer with supporting evidence may set acceptance flags and `verified: true` in the PPO JSON, and `verified: true` / `independent_mapping_reviewed: true` in the lookup. Recompute `effective_lookup_sha256` after any reviewed lookup edit. Its automatic `paired_validation.passed` must also be true (two matched magnitudes in each direction). Session metadata must identify a powered run of the same target, exact firmware/vehicle, motor mapping, expected parameters and neutrals. A missing/stale/synthetic/mismatched fit is refused. The normal PPO `calibration_file` parameter selects this JSON; no model changes are needed. The legacy `force_pwm_points` path remains supported with its original full −15..25 N coverage requirement.

PPO still clips requests to its original force envelope and deployment cap, then uses **strict rejection** for measured-region violations. A sparse low-pulse calibration will often be insufficient for PPO requests, especially near neutral. Remain in shadow until the required domain and effective-force interpretation are supported. Selecting a lookup does not prove that an MJX-trained Newton command is physically reproduced.

## Physical assumptions still unverified

No live boat topics/rates/time base, unit/gravity/frame conventions, motor/servo mapping, neutral/deadband, independent command path, firmware identity, mode/takeover/failsafe timing, IMU mounting/COM offset, actual mass, battery/payload dependence, water-relative speed/current, added mass, drag, actuator delay, yaw dynamics, repeatability or independent motor identifiability has been verified on the physical BlueBoat here. No measured hardware lookup is shipped. Preserve these as open acceptance items; a passing offline test is evidence about software behavior only.
