# Calibration and real-boat acceptance

**Alberto's QGroundControl/MAVProxy installation:** use [GCS_HANDOFF.md](GCS_HANDOFF.md) and [ALBERTO_BOAT.md](ALBERTO_BOAT.md). The receiver/switch requirements below describe the generic `rc_receiver` path. For `joystick_gcs`, a fresh authenticated ground grant replaces that switch, and PWM requests go through a local Unix socket to the exclusive MAVLink gateway. The gateway owns neutral/HOLD/release on faults; MAVROS is telemetry/read-only. All measurement, telemetry, evidence and calibration limits still apply.

The implemented powered path below assumes a physical RC receiver. Alberto’s joystick/GCS authority is explicitly blocked pending [a separate handoff integration](GCS_HANDOFF.md); do not add a receiver or change his receiver-related parameters merely to satisfy this path.

Physical actuation is disabled by default. The shipped calibration is intentionally incomplete and cannot enable actuator mode. This repository has no physical BlueBoat measurements and no completed ArduPilot SITL trial. ONNX/Gazebo agreement alone does not establish that the motor/estimator/network path works on hardware.

## Why RC override can carry PPO outputs

The investigated interface is ArduRover **MANUAL**, `PILOT_STEER_TYPE=1` (two paddles). The left paddle comes from `RCMAP_ROLL`, right from `RCMAP_THROTTLE`. ArduRover mixes these into steering/throttle, then into outputs assigned `SERVOx_FUNCTION=73` (left) and `74` (right). This retains the PPO's independent force requests; it does not use ArduPilot's velocity tracker.

This interpretation is based on the [Rover steering guide](https://ardupilot.org/rover/docs/rover-steering-input-type-and-reversing-behaviour.html), [motor connections](https://ardupilot.org/rover/docs/rover-motor-and-servo-connections.html), and version-pinned [Rover 4.5.7 input mixing](https://github.com/ArduPilot/ardupilot/blob/Rover-4.5.7/Rover/mode.cpp) and [channel mapping](https://github.com/ArduPilot/ardupilot/blob/Rover-4.5.7/Rover/radio.cpp). It has been **source-reviewed, not executed through SITL here**. Confirm the same behavior in the actual firmware, store its exact identity, and test both outputs together.

For separable measured curves, this adapter requires `MANUAL_STR_EXPO=0`, `MANUAL_OPTIONS=0` (no speed-dependent manual scaling), and `MOT_THST_ASYM=1`. It pins other motor settings including slew rate and thrust expo. The node never changes these settings. If the existing installation needs different mixing, do not loosen checks blindly: characterize its coupled mapping and extend/test the adapter. [Manual mode](https://github.com/ArduPilot/ardupilot/blob/Rover-4.5.7/Rover/mode_manual.cpp) and [motor mixing](https://github.com/ArduPilot/ardupilot/blob/Rover-4.5.7/libraries/AR_Motors/AP_MotorsUGV.cpp) are the primary references. An instantaneous scalar curve cannot remove actuator or throttle-slew dynamics; those require a measured closed-loop trial.

## Produce measured calibration, separately for SITL and hardware

For the separate IMU-based effective-response workflow, use [THRUST_CALIBRATION.md](THRUST_CALIBRATION.md). `motor_interface.template.json` verifies PWM mappings/neutral/takeover independently of a force curve, so calibration does not require an existing force calibration. The optional effective lookup path preserves these identity/failsafe gates, adds explicit assumptions/identifiability acceptance, and rejects unmeasured force regions. The full-force-range requirement below applies to the legacy `force_pwm_points` path. IMU mass-times-acceleration results are not measured static propeller thrust, and symmetric results cannot automatically supply independent motor curves.

Copy `config/calibration.template.json` into an ignored local `artifacts/calibration/` directory. Keep `verified: false` until all evidence is collected. The numbers 1/3/7 in the template are **candidate channel locations**, not verified wiring. No force/PWM samples or neutral values are supplied.

1. Save the vehicle's firmware version, `flight_sw_version`, UID when supplied, sysid/compid, complete parameter dump, MAVROS version and SITL/Gazebo plugin revisions. Alberto's current FC/MAVROS path reports `uid=0`; that recorded unavailable value is accepted only with exact system/component, firmware, vehicle type and pinned parameter checks. Use `/mavros/vehicle_info_get`; the human-readable version label alone is insufficient.
2. With propulsion safely isolated, establish port/starboard identity, propeller directions, RC input mapping, servo output assignments, reversals, receiver trims and a separate physical RC override-enable switch. Confirm neutral and manual control. Do not use a software Bool as the sole takeover mechanism.
3. On SITL, measure **RC-input PWM → actual Gazebo applied thrust**, through the complete MAVROS/ArduRover/plugin chain. Servo output alone is insufficient if the plugin maps it nonlinearly. On hardware, measure thrust on a suitable restrained, water-cooled test arrangement with the actual ESCs, propellers, battery voltage and payload. Never infer thrust from nominal PWM limits or copy the synthetic test fixture.
4. For each side record monotonic `[force_N, rc_input_pwm]` points covering −15 to +25 N, including an explicitly measured zero-force neutral. Include both directions, deadband, uncertainty and repeatability. The current adapter requires full-range evidence even if the deployment cap is low; if that evidence cannot be safely collected, remain in shadow. PWM-equivalent RC commands must lie within 1000–2000. Increasing or decreasing curves are supported, but the mapping and reversal must be measured.
5. Test simultaneous pairs (forward/forward, unequal positive forces, opposite signs, reverse/reverse) and transients. Independent scalar curves are only appropriate if paired tests show the mapping remains valid. Record slew, delay, saturation, deadband and left/right cross-effects. Set `paired_motor_test_passed` only after this test.
6. Hash the actual evidence file with `sha256sum`. Set its path relative to the calibration JSON and `evidence_sha256`. Populate all parameter values below from the tested vehicle; numerical comparison occurs before and during output.
7. Test physical switch takeover, receiver loss, companion process termination, MAVROS/network loss and command/telemetry loss in SITL first. Repeat appropriate restrained tests for hardware. Keep existing receiver/GCS failsafes enabled. Record actual time to neutral/fallback, not just parameter values.
8. Only then set the relevant verification flags and `verified: true`. For hardware, require `sitl_test_passed` and `restrained_motor_test_passed`, with `target: hardware` and a separate measured hardware curve. A SITL curve is not a hardware calibration.

Required `expected_parameters` include:

- `PILOT_STEER_TYPE`, `RCMAP_ROLL`, `RCMAP_THROTTLE`, separate enable `RCn_OPTION=46`.
- For each owned RC input: `RCn_MIN`, `MAX`, `TRIM`, `DZ`, `REVERSED`.
- For each output: `SERVOn_FUNCTION` (73/74), `MIN`, `MAX`, `TRIM`, `REVERSED`.
- `MOT_PWM_TYPE`, `MOT_THR_MIN`, `MOT_THR_MAX`, `MOT_SLEWRATE`, `MOT_THST_EXPO`, `MOT_STR_THR_MIX`, `MOT_THST_ASYM`, `MANUAL_STR_EXPO`, `MANUAL_OPTIONS`, `MODE_CH`.
- `RC_OVERRIDE_TIME`: tested positive timeout ≤0.5 s; `RC_OPTIONS` must allow MAVLink overrides and preserve receiver takeover/failsafe flags (bits 0, 1 and 2 clear); `FS_GCS_ENABLE` and `FS_THR_ENABLE` must remain nonzero. Preserve and add installation-specific failsafe parameters to the expected map.

Mode selection must not share either owned motor input or the enable switch. Test MAVLink source/GCS identity authorization too: the controlling GCS/sysid parameters vary by firmware. Do not blindly assign IDs or disable authentication/failsafes to get output. See [ArduPilot RC input/override documentation](https://ardupilot.org/dev/docs/mavlink-rcinput.html).

`maximum_abs_force_N` starts at 3 N in the template. This is a conservative software request cap, **not a claim that 3 N is safe for every setup** and not a verified physical bound without calibration. Increase only according to measured trial results. Existing motor slew/failsafe behavior remains active.

## Runtime stop behavior

- Inference test: no ROS and no motor path.
- Shadow: full telemetry and model; no RC override publisher.
- Actuator: explicit mode + enable flag + verified calibration + matching vehicle/parameters + healthy telemetry/estimator + fresh command + armed MANUAL + physical switch + fresh enable heartbeat.
- Before first valid output, failures simply block output. After output begins, faults latch until node restart; refreshing a heartbeat does not silently resume thrust.
- Telemetry/command/inference/parameter faults send measured neutral for approximately 0.1 s, then release only the two owned RC inputs. A 0.1 s control-loop watchdog runs in a separate thread; a hung inference call cannot publish a late action after the fault is latched.
- Physical takeover, enable release, mode change or disarming immediately releases owned RC inputs. Normal SIGINT/SIGTERM keeps ROS alive long enough to attempt neutral and release before shutdown. Neutral and release are best-effort messages, not acknowledgments of stopped motors.
- Process death, lost networking, interpreter/GIL starvation or a blocked middleware call can defeat the Python watchdog. The flight controller's independently tested RC override timeout and existing failsafes are essential. No hard real-time or safety-certification claim is made.
- Release returns authority to receiver/flight-controller behavior. A nonneutral receiver input can move the boat. Keep the operator ready and verify takeover rather than assuming release means braking.

## Before a pool trial

Complete real ROS serialization/build tests, shadow logs with measured telemetry age and rate, end-to-end SITL thrust/sign/latency tests, all fault injection and manual takeover checks, Jetson timing under load, restrained hardware calibration, and a controlled low-force trial with a responsible operator and recovery plan. Evaluate GPS/estimator validity in the actual pool environment: if required velocity/orientation estimates are unavailable, the controller must remain blocked. A visual Gazebo success does not resolve that sensor limitation.
