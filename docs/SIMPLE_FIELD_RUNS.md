# Three simple field commands

Branch: `alberto/simple-field-scripts`.

These launchers use distances relative to the first valid local odometry pose. They do not contain latitude/longitude waypoints. The first received pose becomes `(0, 0)`, and its yaw defines "forward."

## Before starting

The two movement launchers publish `/blueboat/cmd_vel`; a PPO controller must already be running and subscribed to that topic. They do not arm the boat, select a flight mode, start the gateway, grant motor ownership or start a PPO actuator node. They stop and publish zero if `/mavros/local_position/odom` is absent or stale.

Local position can come from GPS/EKF, motion capture or another validated odometry source. The default frame contract is `map` / `base_link`.

Only run one launcher at a time.

## 1. Unchecked calibration pulses

```bash
./scripts/01_calibrate_unchecked.sh
```

This intentionally bypasses the normal boat-readiness, evidence, GPS, odometry, IMU, estimator, identity, parameter, gateway and enable checks. It reads only RC channels, input neutral and direction signs from `~/.config/blueboat/alberto/motor_interface.json`, waits five seconds, and publishes the configured left/right/symmetric PWM sequence directly to `/mavros/rc/override`.

The sequence is `-40`, `-60`, `+40`, `+60` counts, left/right/symmetric, three repeats, with a 2 s baseline, 0.5 s pulse and 3 s recovery. It writes the commanded sequence under `~/.config/blueboat/alberto/runs/unchecked_calibration/`. The log is not a force calibration because no motion sensors are recorded.

This direct publisher will not pass through a gateway configured to discard MAVROS motor commands. Ctrl+C requests neutral and then releases RC1/RC3.

Optional example:

```bash
./scripts/01_calibrate_unchecked.sh --delta -20 20 --repeats 1
```

## 2. Forward 20 m, turn 180 degrees, return 18 m

```bash
./scripts/02_out_20m_turn_return.sh
```

The boat stores its starting position and heading, drives 20 m along that heading, rotates 180 degrees, returns 18 m along the same line, stops approximately 2 m before the original starting position and publishes zero for two seconds.

## 3. Forward 25 m, one 12 m-diameter circle, then return to 2 m

```bash
./scripts/03_forward_25m_circle_12m.sh
```

The boat drives 25 m along its starting heading. Its actual reached position becomes the circle center. Because a circle cannot begin at its center, it then moves 6 m to the left-hand rim, aligns with the original heading, makes one clockwise 12 m-diameter circle, and returns to a target 2 m ahead of the original starting position.

## Overrides

Both movement scripts default to 0.4 m/s, a 0.3 rad/s maximum yaw rate and a 0.6 m arrival radius. Extra arguments are forwarded to the mission executable:

```bash
./scripts/02_out_20m_turn_return.sh --speed 0.25
./scripts/03_forward_25m_circle_12m.sh --speed 0.3 --diameter 16
```

Use Ctrl+C at any point to publish a zero command and exit.

## Gazebo validation

Both relative-distance controllers were tested in calm-water Gazebo Harmonic using the bundled deterministic ONNX PPO and the upstream BlueBoat marine model. This was the direct-force `ONNX → Newton forces → Gazebo` test path; it validates policy/path behavior in Gazebo, not MAVROS, ArduRover, the gateway or hardware PWM conversion.

| Route | Target | Gazebo result |
|---|---:|---:|
| 20 m out / 18 m back cruise | 0.400 m/s | 0.380 m/s mean, 0.028 m/s RMSE, 0.404 m/s peak |
| Outbound distance | 20.0 m | 19.49 m at the configured 0.6 m arrival radius |
| Final position | 2.0 m ahead of start | 2.034 m ahead; 0.037 m target error |
| Circle-route cruise | 0.400 m/s | 0.377 m/s mean, 0.027 m/s RMSE, 0.409 m/s peak |
| Circle radius | 6.0 m | 6.086 m mean, 0.168 m RMSE |
| Circle progress | 2π rad | 6.284 rad |
| Circle-route final position | 2.0 m ahead of start | 2.038 m from start; 0.039 m target error |

The exact machine-readable record is [`validation/simple_field_missions.json`](validation/simple_field_missions.json).
