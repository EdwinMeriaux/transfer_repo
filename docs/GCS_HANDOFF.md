# QGroundControl / MAVProxy command handoff

Alberto confirmed QGroundControl with a joystick; both QGC and MAVROS currently connect through network MAVProxy endpoints. This package implements an exclusive gateway for that topology. **Local protocol/transport tests pass; the actual QGC installation, ArduRover SITL, Jetson and boat have not been tested with it.** The shipped hardware profile stays unverified. No routing, FC parameters or arming state on Alberto's boat were changed.

## Connection layout

```text
QGC computer                                      Boat / companion
QGroundControl :14550 <------ UDP ------> gateway :14601
blueboat_owner          ----- UDP ------> gateway :14604 (authenticated grant)
                                               |
MAVROS :14602 <----- loopback UDP -----> gateway :14603 (telemetry/read requests)
PPO OR calibration ---- local socket --> gateway gw.sock (bounded PWM requests)
                                               |
                                          gateway :14600
                                               |
                                    ONE MAVProxy --out endpoint
                                               |
                                        existing FC link
```

Run the gateway, MAVROS and the controller/calibrator on the **same Linux host**. Leave the FC connection with MAVProxy. Route **all** ground and companion commands through the gateway; remove previous direct QGC/MAVROS outputs, forwarding bridges, auto-connect paths, RC-input ports, other GCS clients and router control modules. A second route around the gateway defeats arbitration. Keep existing MAVLink signing/authentication and verify it end to end; the grant protocol does not replace FC link security. Use the trusted boat network and restrict these UDP listeners to the stated peers.

MAVROS is telemetry/read-only through this endpoint: its heartbeats, overrides, setpoints, parameter writes, arm and mode commands are dropped. Use QGC for deliberate manual actions. Gateway-generated RC overrides use the pinned GCS system ID; there is no velocity-setpoint fallback. PPO retains its original force output and measured PWM conversion.

## Prepare once

Update/install with `scripts/install_alberto.sh` as in [ALBERTO_BOAT.md](ALBERTO_BOAT.md). The installer includes the `handoff` extra (`pymavlink`). Supply the QGC computer's actual reachable IPv4 address, and the MAVProxy host's address as seen from the gateway:

```text
./scripts/alberto.sh gateway-configure --qgc-ip QGC_IP --mavproxy-ip MAVPROXY_IP
```

Use `127.0.0.1` for MAVProxy only when it runs on the same companion. QGC's receive port defaults to 14550; override with `--qgc-port PORT`. `--mavproxy-port 0` learns and pins the UDP source port from the first correctly identified FC heartbeat on that host; use a known fixed source port if available. `--vehicle-component` defaults to 1 and must match the captured FC identity. Configure creates `gateway.json` and a random private `ground.key` beside the other local files, records the exact configuration hash in `motor_interface.json`, and clears old approvals. It does not change any remote routing or parameters. Existing keys are retained. Stop running processes before editing/reconfiguring; changing the gateway file invalidates the pinned hash.

Configure these endpoints once:

| Client | Configuration |
|---|---|
| MAVProxy | Keep its actual FC `--master`; use one `--out=udp:GATEWAY_IP:14600`. Disable its own heartbeats, command-generating modules and interactive control. |
| QGC | One UDP connection, local receive port 14550, target `GATEWAY_IP:14601`. Disable old/direct/automatic links. Its source system ID must match the export's `SYSID_MYGCS=255`. |
| MAVROS | `fcu_url: udp://127.0.0.1:14602@127.0.0.1:14603`; source `system_id: 254`, target system/component 1/1 (or captured values). Remove any MAVROS GCS forwarding URL. |
| Ground grant client | Runs on the **QGC computer**, targets `GATEWAY_IP:14604`, using the private key copied securely from the bundle. |

A minimal relay command shape is below; substitute the existing FC transport and gateway IP. Check installed MAVProxy options/version first. Review `~/.mavinit.scr`, `~/.mavproxy/mavinit.scr` and aircraft startup scripts: they must not add bypass outputs, heartbeats, commands or incompatible modules. Source ID 253 is deliberately distinct from the authorized GCS ID.

```text
mavproxy.py --master=EXISTING_FC_LINK --out=udp:GATEWAY_IP:14600 \
  --source-system=253 --heartbeat-rate=0 --nowait \
  --default-modules=signing --non-interactive --streamrate=-1 --mav20
```

Keep the existing signing key/configuration in the signing module where used. Forwarded frames retain their bytes; gateway-generated frames need the relay/FC's tested signing arrangement. Do not disable FC signing to make them work. Our loopback relay test used an unsigned fake vehicle, not a signed boat connection. `--nowait` avoids MAVProxy's startup heartbeat-wait loop; `--heartbeat-rate=0` disables periodic heartbeats. Audit actual traffic rather than trusting options alone. Never operate a second command-capable MAVProxy console concurrently.

Start read-only monitor mode on the companion:

```bash
./scripts/alberto.sh gateway
```

Monitor forwards FC telemetry and read requests only; it cannot grant autonomy or pass manual motor/mode/arming commands. Use it to inspect wiring, capture identity and parameters, and record telemetry with propulsion isolated. The usual commands remain:

```bash
./scripts/alberto.sh capture
./scripts/alberto.sh status
./scripts/alberto.sh run telemetry
```

The ground grant tool uses only Python's standard library. On the QGC computer, install the package if desired, or copy **only** `ros2_ws/src/blueboat_ppo_controller/blueboat_ppo_controller/ground_owner.py` plus the private `ground.key` and run `python3 ground_owner.py ...`. Keep the key private; do not commit it, paste it into chat or place it in logs. A Linux/macOS copy should retain mode 600. No ROS installation is needed there. The tool requires numeric IPv4 addresses.

## Ownership and stopping

The explicitly enabled gateway starts in MANUAL ownership; its autonomous socket cannot command motors. After interface and acceptance evidence review, stop monitor and start:

```bash
./scripts/alberto.sh gateway --enable-forwarding
```

This requires a verified hardware motor interface, exact gateway hash, identity, compatible pinned parameters and all acceptance flags. It does not arm or select MANUAL. For an independently prepared **SITL** interface, use `blueboat_gateway --config PATH --interface PATH --enable-forwarding`; its `target: sitl` profile must pass the existing identity/mapping/interface checks. The five additional GCS evidence flags are required for hardware, allowing the initial GCS integration tests to be conducted in SITL. Keep SITL and hardware evidence/configuration separate; never label hardware as SITL to bypass checks.

Before a grant, select MANUAL and arm through the tested operator workflow, disable QGC joystick transmission (centering a joystick is insufficient), and wait at least one second after its last control packet. Keep QGC connected and sending its genuine GCS heartbeat. Start **one** controller or calibrator, with its normal fresh software-enable heartbeat and, for PPO, desired velocity command. The node waits without motor output until granted; it must be ready to submit within 0.2 s after the grant.

On the QGC computer:

```text
python3 ground_owner.py auto --gateway GATEWAY_IP:14604 --key-file /path/to/ground.key
```

Keep that process running. Each lease requires a single-use server challenge (0.25 s lifetime) and HMAC authentication, and lasts 0.5 s; the client renews every 0.1 s. The gateway separately requires a genuine QGC heartbeat within 2.5 s and a healthy FC heartbeat within 2 s, with matching recorded UID/firmware refreshed within 15 s. A recorded zero UID means unavailable, not a wildcard: live messages must continue reporting zero while system/component, firmware and parameter checks remain exact. Latency outside these limits causes refusal/stop; these are software deadlines, not measured physical stop times.

While AUTO owns output:

- Only the selected local controller session can send motor PWM; requests must be fresh within 0.1 s with increasing sequence numbers. No request for 0.2 s stops ownership. Restarted/replayed sessions and competing clients cannot resume it.
- QGC motor/write traffic revokes AUTO and is blocked. QGC HOLD and disarm commands remain passable. Read requests continue. The deliberately narrow read allowlist may stop AUTO for other QGC command traffic; validate with the installed QGC version before marking acceptance.
- Loss of ground grant, QGC heartbeat, FC identity/health/mode, node enable/telemetry, or controller requests stops autonomy. The node's fault latches once active.
- The gateway sends measured input neutral, requests Rover HOLD, releases only RC1/RC3 after 0.1 s, and repeats HOLD if unconfirmed. It suppresses ground heartbeats while STOPPED until a fresh healthy FC heartbeat reports HOLD/disarmed, allowing the FC failsafe to act if stop requests are not accepted. Graceful gateway shutdown attempts neutral/HOLD/release; process death relies on FC fallback.

Ctrl-C in the AUTO grant terminal requests stop; a killed/disconnected client cannot renew. A separate explicit stop command is also available:

```text
python3 ground_owner.py stop --gateway GATEWAY_IP:14604 --key-file /path/to/ground.key
```

Return to manual deliberately, **after** HOLD/disarmed feedback and release:

```text
python3 ground_owner.py manual --gateway GATEWAY_IP:14604 --key-file /path/to/ground.key
```

Then use QGC to choose the desired manual state and re-enable the joystick. A premature manual request stops AUTO and is refused until the stop state is confirmed; repeat only after inspection. The gateway never auto-arms, automatically returns to MANUAL mode, or automatically resumes AUTO after reconnect. Restart a faulted controller before a new grant. Heartbeat mode confirmation is not proof that propellers stopped; record servo/motion feedback in acceptance tests.

## FC fallback and outstanding boat settings

Rover 4.6.2 treats **HEARTBEAT, MANUAL_CONTROL and RC_CHANNELS_OVERRIDE** from its configured GCS system ID as GCS liveness. Therefore both removing companion/router heartbeats **and stopping autonomous override traffic on independent ground-grant loss** matter. The gateway only emits overrides while authorized, plus bounded neutral/release during stopping. Source: [GCS_Common.cpp](https://github.com/ArduPilot/ardupilot/blob/Rover-4.6.2/libraries/GCS_MAVLink/GCS_Common.cpp). MAVProxy normally forwards client packets without exclusive ownership: [mavproxy.py](https://github.com/ArduPilot/MAVProxy/blob/master/MAVProxy/mavproxy.py).

The export remains unchanged and still needs operator review:

- `RC_OVERRIDE_TIME=3` must be replaced by a **tested** positive value ≤0.5 s for this adapter. Expiry alone is not proof of neutral; measure the fallback input/output.
- `MOT_THST_ASYM=1.6` conflicts with this independent-curve mapping's requirement of 1; paired output tests are required.
- `FS_ACTION=4` means **SmartRTL or Hold**, which can command movement after gateway failure. This integration requires a tested Hold (`2`) or Terminate (`5`) fallback. Neither is silently written to the FC.
- `FS_GCS_TIMEOUT=5` plus `FS_TIMEOUT=5` can leave a long interval before FC fallback when the gateway/router itself dies. Record the actual delay and choose acceptable tested settings; the 0.5 s software lease is not a guarantee for a dead gateway. GCS liveness is refreshed by the last accepted override/neutral/release too. Source: [Rover failsafe logic](https://github.com/ArduPilot/ardupilot/blob/Rover-4.6.2/Rover/failsafe.cpp), [parameter definitions](https://github.com/ArduPilot/ardupilot/blob/Rover-4.6.2/Rover/Parameters.cpp).
- Receiver-related settings (`RC_OPTIONS=37`, `FS_THR_ENABLE=0`, no RC enable switch) are consistent with this confirmed GCS architecture and are not changed to pretend a receiver exists. Override-ignore bit 1 must remain clear.

Refresh the reviewed export, capture actual identity, measure output neutrals/signs, and reconfigure/review before approving the local interface. Original PPO weights, force limits, measured-curve requirements and sensor conventions are unchanged.

## Acceptance evidence

Use a separate SITL instance/profile first. Record software versions, routes/startup scripts, exact config hash, packet logs, RC output and mode/time traces. Then perform the appropriate restrained hardware checks before any free-hull calibration or pool trial. Use existing manual wiring/fallback tests to establish the base interface; do not set an evidence flag simply to get past a preflight error. New flags cover:

| Flag | Required evidence |
|---|---|
| `gcs_no_bypass_verified` | Every command route reaches the gateway; old/direct QGC, MAVROS, RC and router control paths are closed. |
| `gcs_heartbeat_isolation_tested` | Remove ground traffic while companion remains alive; verify FC-bound heartbeats/overrides cease as designed. Include router startup/restart and signing. |
| `gcs_handoff_tested` | Manual works; running joystick prevents grant; disable joystick and grant explicitly; joystick/control takeover stops AUTO; only HOLD/disarmed plus explicit reset returns manual; no reconnect auto-resume. |
| `gcs_ground_loss_tested` | Stop grant client, stop QGC alone, disconnect ground network, replay/delay grants; observe actual output and elapsed time. |
| `gcs_router_failure_tested` | Kill gateway and MAVProxy separately, stop MAVROS/controller, interrupt FC link; measure independent FC fallback and recovery without stale output. |

Also test stale/replayed motor requests, simultaneous PPO/calibrator starts, delayed telemetry, mismatched recorded UID/firmware/parameters, mode changes, and logging/CPU/network load. Retain existing interface/paired output/SITL calibration/telemetry/IMU evidence. Store evidence files and SHA in the local profile; hardware GCS flags require review of this installation's evidence, not the synthetic fixtures in `tests/`. Keep physical tests supervised and bounded.

The checked-in tests exercise protocol state, authentication/replay, real localhost UDP and Unix sockets, mocked ROS node paths, and (when installed) an actual MAVProxy relay against a **fake MAVLink vehicle**. They do not validate ArduPilot mixer physics, QGC UI behavior, ROS serialization, signed FC acceptance, propellers or stopping distance. Gateway transition logs and local status live beside `gateway.json`; preserve them with acceptance evidence. A stale `.sock.lock` after a hard kill is deliberately refused: verify the old process is gone and the FC has stopped before removing that socket/lock and restarting.
