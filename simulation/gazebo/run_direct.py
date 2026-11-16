import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
from blueboat_ppo_controller.policy_runtime import PolicyRuntime, thrust_action
from blueboat_ppo_controller.generalist_policy_runtime import GeneralistPolicyRuntime
from blueboat_ppo_controller.observation_adapter import ObservationAdapter
from bridge import GazeboBoat

parser = argparse.ArgumentParser(description='Standalone ONNX policy -> force -> Gazebo, without ArduPilot')
parser.add_argument('--world', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--seconds', type=float, default=24.)
parser.add_argument('--reference', choices=['constant_speed', 'changing_speed', 'positive_yaw', 'negative_yaw', 'sinusoidal_yaw', 'combined'], default='positive_yaw')
parser.add_argument('--policy-backend', choices=['e274_6_5b', 'legacy'], default='e274_6_5b')
parser.add_argument('--minimum-force-N', type=float, default=-15.)
parser.add_argument('--maximum-force-N', type=float, default=25.)
args = parser.parse_args()
if args.seconds <= 0 or not np.isfinite(args.seconds):
    parser.error('Duration must be finite and positive')
if (not np.isfinite([args.minimum_force_N, args.maximum_force_N]).all()
        or not -15 <= args.minimum_force_N <= 0
        or not 0 <= args.maximum_force_N <= 25
        or args.minimum_force_N >= args.maximum_force_N):
    parser.error('Force limits must be ordered within the policy -15..25 N envelope')
args.output.mkdir(parents=True, exist_ok=False)
runtime = GeneralistPolicyRuntime() if args.policy_backend == 'e274_6_5b' else PolicyRuntime()
adapter = None if args.policy_backend == 'e274_6_5b' else ObservationAdapter(runtime.contract)
boat = GazeboBoat(args.world)
for step in range(600):
    boat.step(np.zeros(2))
previous = np.zeros(2)
records = []
for step in range(round(args.seconds / .02)):
    t = step * .02
    command = {'constant_speed': [.6, 0.], 'changing_speed': [[.3, .8, -.4, 0.][min(int(t / 6), 3)], 0.], 'positive_yaw': [.4, .4], 'negative_yaw': [.4, -.4], 'sinusoidal_yaw': [.5, .4 * np.sin(2 * np.pi * t / 12)], 'combined': [.55 + .25 * np.sin(2 * np.pi * t / 12), .45 * np.sin(2 * np.pi * t / 16)]}[args.reference]
    state = boat.state
    if args.policy_backend == 'e274_6_5b':
        action = runtime.action_from_body(
            state['quaternion_xyzw'], state['body_velocity'],
            state['body_omega'], command, previous)
        requested_force = runtime.actuator_controls(action)
    else:
        observation = adapter.from_world(state['quaternion_xyzw'], state['velocity_world'], state['omega_world'], command, previous)
        action = runtime.action(observation)
        requested_force = thrust_action(action)
    force = np.clip(
        requested_force, args.minimum_force_N, args.maximum_force_N)
    state = boat.step(force)
    previous = action.copy()
    if np.max(np.abs(state['euler_rpy'][:2])) > 1.3 or abs(state['position'][2]) > 3.:
        raise RuntimeError('Boat exceeded stability limits')
    records.append([t + .02, *command, state['body_velocity'][0], state['body_omega'][2], *force, *requested_force, *action, *state['position'], *state['euler_rpy']])
values = np.asarray(records)
with (args.output / 'trajectory.csv').open('w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['time_s','target_u','target_r','actual_u','actual_r','left_N','right_N','requested_left_N','requested_right_N','raw_left','raw_right','x','y','z','roll','pitch','yaw'])
    writer.writerows(values)
provenance = (runtime.manifest['source_archive_sha256'] if args.policy_backend == 'e274_6_5b'
              else runtime.contract['checkpoint_sha256'])
requested = values[:, 7:9]
applied = values[:, 5:7]
saturated = np.any(np.abs(requested - applied) > 1e-6, axis=1)
report = dict(
    policy_backend=args.policy_backend,
    reference=args.reference,
    duration_s=len(values) * .02,
    controls=len(values),
    rmse=np.sqrt(np.mean(
        (values[:, 3:5] - values[:, 1:3]) ** 2, axis=0)).tolist(),
    stable=True,
    minimum_force_N=args.minimum_force_N,
    maximum_force_N=args.maximum_force_N,
    requested_force_range_N=[float(np.min(requested)),
                             float(np.max(requested))],
    applied_force_range_N=[float(np.min(applied)), float(np.max(applied))],
    saturated_records=int(np.sum(saturated)),
    saturation_percentage=float(100 * np.mean(saturated)),
    path='ONNX -> directional force clamp -> native Gazebo; no ROS/ArduPilot',
    policy_source_sha256=provenance)
(args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
