# Joystick/GCS takeover: confirmed setup, integration pending

The user confirmed Alberto uses **joystick + GCS**, not a physical RC receiver/switch. The Alberto profile now records `control_authority: joystick_gcs`. Its `RC_OPTIONS=37`, `FS_THR_ENABLE=0` and missing receiver-enable switch are preserved as properties of that architecture, not instructions to add a physical receiver.

**Powered joystick/GCS operation is not implemented in this package yet.** `MotorInterface` explicitly rejects this authority even if `verified` and every evidence flag are manually set true. Telemetry, dry-run and PPO shadow remain available. The existing `rc_receiver` implementation and its receiver checks remain intact for installations that actually use one; absent authority fields in older generic profiles retain that behavior.

## Why a new authority path is necessary

Reviewed [ArduPilot Rover-4.6.2 GCS_Common.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.6.2/libraries/GCS_MAVLink/GCS_Common.cpp):

- `handle_rc_channels_override` and `handle_manual_control` accept commands from `sysid_my_gcs()`; merely choosing a different companion sysid does not establish a second independent authorized control channel.
- `handle_heartbeat` updates GCS liveness for that system ID; manual-control messages also update it. If the companion uses the accepted GCS identity, its continued traffic can mask loss of the ground connection.
- The standard receiver option/failsafe bits have their documented meanings in [RC_Channel.h](https://github.com/ArduPilot/ardupilot/blob/Rover-4.6.2/libraries/RC_Channel/RC_Channel.h). Ignoring a nonexistent receiver is consistent with the confirmed setup, but does not create command arbitration.

A ROS Bool published on the companion proves only that a local process is publishing. It does not prove the joystick is disabled, the operator remains connected, or that no other MAVLink source is commanding the boat. ROS publisher discovery does not see a joystick's direct MAVLink traffic. Therefore the existing receiver interlock cannot simply be replaced by an always-true value or a second local heartbeat.

## Information still needed

Identify the GCS software/version (for example QGroundControl or BlueOS Cockpit), connection topology (BlueOS/mavlink-router/direct MAVROS), the joystick's actual message type and source IDs, and where all motor-command traffic passes. Document the operator's existing stop/disarm action and the actual effect of ground-link loss. The operator-confirmed phrase "joystick + GCS" resolves the receiver question, but does not identify these routing details.

## Required implementation and acceptance

The next implementation must establish one enforced motor-command authority at a point that observes both sources, not just a procedural promise to avoid moving the joystick. A GCS/router integration or a single command arbiter must:

1. Default to manual authority, with PPO/calibration output blocked.
2. Accept an explicit operator handoff, suppress joystick motor-command traffic while autonomy owns output, and reject simultaneous/unknown senders. Keep stop/mode/disarm commands available.
3. Revoke ownership immediately on the tested operator stop/mode action or loss of an independent ground-origin grant. A companion-generated heartbeat must not impersonate that ground-origin grant.
4. Stop/neutralize through the tested vehicle mechanism, latch the autonomous controller, and acknowledge release before joystick motor commands resume. Moving a stick while PPO is active is not, by itself, an implemented takeover.
5. Handle GCS loss while the companion remains alive, companion process failure while the GCS remains alive, router/arbiter failure, delayed/replayed grants, reconnects and rapid mode changes. Verify the FC's source authorization and heartbeat/failsafe behavior under each case.
6. Preserve bounded override expiry and measured fallback behavior. The export's 3 s override timeout and 1.6 mixer asymmetry remain separate compatibility issues; changing the authority mechanism does not resolve thrust mapping or calibration.

HOLD or disarming through the GCS may be part of the stop procedure, but the exact command and observed effect must be tested with the installed boat. Existing PPO/calibration nodes already release and latch when they observe disarming or leaving MANUAL; that alone does not filter competing MAVLink commands or independently detect ground-link loss.

Test the complete GCS → router → MAVROS → FC path in SITL, then the appropriate restrained hardware control tests, before collecting powered free-hull calibration data. Firmware source review, offline unit tests and a checked JSON flag are not evidence that this handoff has run. No FC parameter changes, GCS configuration changes or new motor commands were applied by this update.

## Existing installed profile

After updating the repository and reinstalling/rebuilding using the installer, record the confirmed authority in an existing local bundle with:

```bash
./scripts/alberto.sh configure --control-authority joystick_gcs
./scripts/alberto.sh status
./scripts/alberto.sh run telemetry
```

Configuration changes clear previous approvals, keep the exported parameters, and remove the unused local receiver-enable channel. Stop any running controller before editing local configuration. Newly initialized bundles already carry the confirmed authority. `status` exits 2 while the handoff implementation and remaining measurements/tests are pending. Do not change the authority back to `rc_receiver` merely to bypass this block.
