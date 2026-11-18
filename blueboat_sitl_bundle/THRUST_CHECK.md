# Simple thrust check

One run, about 5 minutes on the water. It checks whether the boat responds like the boat the u990 policy was trained on.

Each step holds a fixed thrust per motor. Newtons are converted to ESC PWM with Blue Robotics' published T200 data (`blueboat_ppo_controller/data/t200_thrust_N.csv`), at the battery voltage MAVROS reports. The tool then trims the RC input until `/mavros/rc/out` shows that PWM, so ArduPilot's mixing, trims and `MOT_THST_ASYM` are accounted for without being modelled. The expected speeds and turn rates are from the policy's README (its training model).

| Step | Left, right (N) | Expected | Boat travels |
|---|---|---|---|
| `spin10` | −10, +10 | 0.59 rad/s turning left | stays put |
| `spin15` | −15, +15 | 0.83 rad/s | stays put |
| `fwd15` | 15, 15 | ~1.79 m/s (interpolated) | ~16 m out |
| `rev15` | −15, −15 | −1.79 m/s | ~16 m back to the start |
| `fwd10` | 10, 10 | 1.41 m/s | ~13 m out |
| `rev10` | −10, −10 | −1.41 m/s (interpolated) | ~13 m back |
| `left10` | 10, 0 | 0.91 m/s, slow right curve | ~8 m out |
| `right10` | 0, 10 | 0.91 m/s, slow left curve | ~8 m further |

Each step lasts 8 s; the result is the mean over its last 3 s, after the RC trim has settled and the boat has reached speed. The tool flags a step if the boat is still speeding up (`--seconds 10` makes every step longer).

**Space: about 20 m of clear water in a straight line ahead of the boat.** The order is out and back, so the boat stays within about 16 m of where it starts, coasting included, if it is as fast as the training model (a slower boat travels less). Start with the boat at one end, pointed along the long side.

## Run

Before starting:
- the boat is in MANUAL and armed (you do this in QGroundControl);
- no PPO or calibration node is running;
- the GPS has a fix;
- `RC_OVERRIDE_TIME` is between 0 and 0.5 s.

```bash
scripts/10_thrust_check.sh list
ros2 topic pub --rate 10 /blueboat/thrust_check/enable std_msgs/msg/Bool '{data: true}'   # second terminal; Ctrl+C there = stop
scripts/10_thrust_check.sh run                     # all steps; or e.g.: run spin10 spin15
```

- Before each step the tool waits for the boat to stop, then for you to press Enter.
- **Stop at any time:** disarm or HOLD in QGC, stop the enable publisher, or Ctrl+C. Disarm, a mode change or losing enable releases immediately; anything else (Ctrl+C, a limit, the end of a step) sends neutral for 0.3 s, then releases.
- A step also ends at 2.6 m/s or 1.3 rad/s.
- In SITL, pass `--interface <SITL motor_interface.json>` and `--voltage 16`. SITL's simulated battery voltage is not the real one.

## Read the result

Example output:

```
fwd10    speed +1.32 vs +1.41 expected (-6%)  thrust x0.90
thrust scale: {"spin": 0.95, "forward": 0.90, "reverse": 0.88, "left_right_speed_ratio": 1.02}
GOOD: every step within 15% of the training model
```

- **Within 15%:** PPO's speed control absorbs the difference.
- **Off by more:** the thrust scale per direction is the one number to correct (for example forward ×0.85).
- **`left_right_speed_ratio`** compares `left10` and `right10`; 1.0 means the motors match.
- The CSV (`~/.config/blueboat/alberto/thrust_check/`) holds every sample: speed, turn rate, RC input, the ESC PWM reached and battery voltage.

## PPO force-to-PWM curve (no bollard pull)

On the boat, the PPO node needs a force-to-PWM curve per motor, covering −15 to +25 N. It is built from the thrust check result. No motors run while it is built.

```bash
scripts/10_thrust_check.sh curve build ~/.config/blueboat/alberto/thrust_check/thrust-check-<stamp>.json
#   prints the table: force N -> left RC, right RC; writes force-curve-<stamp>.json next to it
scripts/10_thrust_check.sh curve apply ~/.config/blueboat/alberto/thrust_check/force-curve-<stamp>.json
#   writes force_pwm_points and the thrust cap into ~/.config/blueboat/alberto/motor_interface.json (keeps a backup)
```

How each policy force F becomes an RC input, per motor:
1. **Thrust scale.** F is divided by the thrust check's scale for that direction (forward or reverse). The real boat then responds to F the way the training model did. `--no-scale` uses the T200 data alone.
2. **T200 data.** The force is converted to ESC PWM at `--voltage`, 15.0 V by default.
3. **Measured mapping.** The ESC PWM is converted to RC input with the mapping the thrust check measured: RC input sent against ESC PWM seen on `/mavros/rc/out`, settled at the end of each step. This mapping is a straight line per motor and direction.
   - Each motor gets its own line per direction from two thrust levels. If a step is missing, a motor with only one level borrows the other motor's slope (the RC and SERVO settings are identical on both sides).

**Thrust cap.** The cap defaults to ±15 N per motor (`--max-force`, `--min-force`). The policy can still ask for up to 25 N, but the PPO node clips to the cap. The curve still goes up to 25 N, because the node requires that; those rows are marked "above cap, not sent".

**Refused** if:
- a forward or reverse step didn't complete;
- a scale is outside ×0.5–1.5;
- the RC input hadn't settled;
- a capped force would need RC beyond `RCx_MIN`/`RCx_MAX`.

**Battery (4S Li-ion 18 Ah: 16.8 V full, 14.8 V nominal).** At a fixed PWM, T200 thrust drops with voltage. A curve built at 15 V gives about +10% thrust on a full battery and about −8% at 14 V. Stop powered runs at about 14 V. Build at another voltage with `--voltage`.

## In SITL

SITL's thrusters are Gazebo's, not T200s, so its numbers show how Gazebo compares with the T200 data, not how the real boat does. Use SITL to check that the tool runs, stops and releases correctly, and that the ESC PWM trim converges (`esc_offset_*` in the CSV, against `esc_offset_target_*`).

`curve build` can be tried on a SITL result. Don't `curve apply` to the SITL `motor_interface.json` unless you want to replace its SITL curve; apply to a copy with `--interface`.
