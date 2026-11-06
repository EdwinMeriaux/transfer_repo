import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
from blueboat_ppo_controller.policy_runtime import PolicyRuntime, thrust_action
from blueboat_ppo_controller.observation_adapter import ObservationAdapter
from bridge import GazeboBoat

parser = argparse.ArgumentParser(description='Standalone ONNX policy -> force -> Gazebo, without ArduPilot')
parser.add_argument('--world', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--seconds', type=float, default=24.)
parser.add_argument('--reference', choices=['constant_speed', 'changing_speed', 'positive_yaw', 'negative_yaw', 'sinusoidal_yaw', 'combined'], default='positive_yaw')
args = parser.parse_args()
if args.seconds <= 0 or not np.isfinite(args.seconds):
    parser.error('Duration must be finite and positive')
args.output.mkdir(parents=True, exist_ok=False)
runtime = PolicyRuntime()
adapter = ObservationAdapter(runtime.contract)
boat = GazeboBoat(args.world)
for step in range(600):
    boat.step(np.zeros(2))
previous = np.zeros(2)
records = []
for step in range(round(args.seconds / .02)):
    t = step * .02
    command = {'constant_speed': [.6, 0.], 'changing_speed': [[.3, .8, -.4, 0.][min(int(t / 6), 3)], 0.], 'positive_yaw': [.4, .4], 'negative_yaw': [.4, -.4], 'sinusoidal_yaw': [.5, .4 * np.sin(2 * np.pi * t / 12)], 'combined': [.55 + .25 * np.sin(2 * np.pi * t / 12), .45 * np.sin(2 * np.pi * t / 16)]}[args.reference]
    state = boat.state
    observation = adapter.from_world(state['quaternion_xyzw'], state['velocity_world'], state['omega_world'], command, previous)
    action = runtime.action(observation)
    force = thrust_action(action)
    state = boat.step(force)
    previous = action.copy()
    if np.max(np.abs(state['euler_rpy'][:2])) > 1.3 or abs(state['position'][2]) > 3.:
        raise RuntimeError('Boat exceeded stability limits')
    records.append([t + .02, *command, state['body_velocity'][0], state['body_omega'][2], *force, *action, *state['position'], *state['euler_rpy']])
values = np.asarray(records)
with (args.output / 'trajectory.csv').open('w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['time_s','target_u','target_r','actual_u','actual_r','left_N','right_N','raw_left','raw_right','x','y','z','roll','pitch','yaw'])
    writer.writerows(values)
report = dict(reference=args.reference, duration_s=len(values) * .02, controls=len(values), rmse=np.sqrt(np.mean((values[:, 3:5] - values[:, 1:3]) ** 2, axis=0)).tolist(), stable=True, path='ONNX -> Newton-valued forces -> native Gazebo; no ROS/ArduPilot', checkpoint_sha256=runtime.contract['checkpoint_sha256'])
(args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
