# Alberto's hardware profile

All files in this directory ship with the ROS Python package. `alberto_boat init` copies them together into `~/.config/blueboat/alberto`, fills local run paths and refuses to overwrite existing work. The installed command works without the research repository or ROS for init/status/configure. ROS is needed for capture/run.

- `boat.json`: reported approximately 23 kg mass, operator report and original parameter-export SHA.
- `observed_parameters.json`: 66 relevant values extracted from the supplied 2026-11-09 Rover 4.6.2 export. These are observed settings, not a file to load onto the FC.
- `motor_interface.json`: RC1 → left → servo3; RC3 → right → servo1; reported RC neutral 1500 and forward-positive inputs. Hardware target, confirmed `control_authority: joystick_gcs`, no force curve, no approved tests. Unknown actual servo neutrals/signs and live identity stay null.
- `thrust_calibration.yaml`: dry-run default, 23 kg adjustable, neutral 1500, conservative original pulse plan and unchanged telemetry gates.
- `controller.yaml`: PPO shadow default, bundled policy, unchanged model/observations/actions.
- `analysis.json`: IMU-to-COM offset and mass uncertainty intentionally null until supplied. Added mass is left as an explicit zero-correction model assumption, not a measurement.

The export has servo1 trim=1510, servo3 trim=1510 and servo3 reversal=1. These are retained, not replaced with the reported RC neutral. Check actual output feedback and direction with the operator. A physical right motor on servo1 does not mean RC1 should be sent right-force requests: Rover's two-paddle RC input mapping is separate.

Joystick + GCS is confirmed. The ignored receiver/failsafe bits (`RC_OPTIONS=37`), disabled receiver failsafe and missing physical receiver-enable switch are retained, not described as instructions to add a receiver. **Joystick/GCS command arbitration and independent ground-link loss handling are not implemented; powered operation is explicitly blocked even if approval flags are set.** GCS software/router details are still needed. The 3 s override timeout, 1.6 mixer asymmetry, actual output measurements and calibration evidence remain additional acceptance items. No command here writes FC parameters, selects a mode or arms. See `docs/GCS_HANDOFF.md` for the source-reviewed shared-system-ID/heartbeat issue and required integration.

Use `alberto_boat --help`. `status` reports all known blockers and exits 2 while blocked; telemetry/dry_run/shadow remain available. `configure` and `capture` invalidate approvals. `refresh --params FILE` imports another NAME VALUE export, preserves a backup of the old interface, and resets identity/output measurements and approvals. No tool auto-marks tests passed.

Existing local bundles can record the confirmation with `alberto_boat configure --control-authority joystick_gcs`; new bundles already include it. This does not enable powered operation.

Full install, field commands, review procedure and source references are in the repository's `docs/ALBERTO_BOAT.md`. This bundle is hardware-specific and must not be reused as a SITL identity. Existing generic templates remain available for SITL.
