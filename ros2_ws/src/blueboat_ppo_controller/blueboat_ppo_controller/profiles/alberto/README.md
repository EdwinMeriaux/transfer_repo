# Alberto's hardware profile

All files in this directory ship with the ROS Python package. `alberto_boat init` copies them together into `~/.config/blueboat/alberto`, fills local run paths and refuses to overwrite existing work. The installed command works without the research repository or ROS for init/status/configure. ROS is needed for capture/run.

- `boat.json`: reported approximately 23 kg mass, operator report and original parameter-export SHA.
- `observed_parameters.json`: 66 relevant values extracted from the supplied 2026-11-09 Rover 4.6.2 export. These are observed settings, not a file to load onto the FC.
- `motor_interface.json`: RC1 → left → servo3; RC3 → right → servo1; reported RC neutral 1500 and forward-positive inputs. Hardware target, `control_authority: direct_mavros`, no force curve and no approved tests. Unknown actual servo neutrals/signs and live component/firmware identity stay null.
- `thrust_calibration.yaml`: dry-run default, 23 kg adjustable, neutral 1500, conservative original pulse plan and unchanged telemetry gates.
- `controller.yaml`: PPO shadow default, bundled policy, unchanged model/observations/actions.
- `analysis.json`: IMU-to-COM offset and mass uncertainty intentionally null until supplied. Added mass is left as an explicit zero-correction model assumption, not a measurement.

The export has servo1 trim=1510, servo3 trim=1510 and servo3 reversal=1. These are retained, not replaced with the reported RC neutral. Check actual output feedback and direction with the operator. A physical right motor on servo1 does not mean RC1 should be sent right-force requests: Rover's two-paddle RC input mapping is separate.

The Jetson MAVROS connection is the direct PPO/calibration command path to the Pixhawk. There is no custom gateway, ground key or grant client. Powered operation still requires reviewed routing, identity, measured mapping, command-loss behavior and manual-takeover evidence. The ignored receiver/failsafe bits (`RC_OPTIONS=37`), disabled receiver failsafe and missing physical switch are preserved for this receiver-less setup.

The export's 3 s override timeout, 1.6 mixer asymmetry and FS_ACTION=4 (SmartRTL or Hold) remain incompatible with this deployment adapter. Actual output neutrals/signs and calibration evidence are still missing. Configuration commands never write FC settings, arm or select a mode. Keep `verified` false until real acceptance review; synthetic tests are not boat evidence.
