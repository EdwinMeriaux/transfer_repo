"""Run a simple relative-distance mission through the ONNX policy in native Gazebo."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from blueboat_ppo_controller.mission_control import MissionController
from blueboat_ppo_controller.observation_adapter import ObservationAdapter
from blueboat_ppo_controller.policy_runtime import PolicyRuntime, thrust_action
from bridge import GazeboBoat


parser = argparse.ArgumentParser(description='PPO relative-distance mission in direct-force Gazebo')
parser.add_argument('--world', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--mission', choices=('out-and-back', 'center-circle'), required=True)
parser.add_argument('--speed', type=float, default=.4)
parser.add_argument('--max-seconds', type=float, default=360.)
args = parser.parse_args()
if args.output.exists():
    raise FileExistsError(args.output)
if not 0 < args.speed <= .8 or not 10 <= args.max_seconds <= 1200:
    raise ValueError('Invalid mission speed or timeout')
args.output.mkdir(parents=True)

runtime = PolicyRuntime()
adapter = ObservationAdapter(runtime.contract)
boat = GazeboBoat(args.world)
for _ in range(600):
    boat.step(np.zeros(2))

controller = MissionController(
    args.mission, distance_m=20., center_distance_m=25., diameter_m=12.,
    speed_m_s=args.speed, max_yaw_rate_rad_s=.3, arrival_radius_m=.6)
previous = np.zeros(2)
records = []
stage_ids = {}
complete_at = None

for step in range(round(args.max_seconds / .02)):
    state = boat.state
    command = controller.update(state['position'][:2], state['euler_rpy'][2])
    target = np.array([command.surge_m_s, command.yaw_rate_rad_s])
    observation = adapter.from_world(
        state['quaternion_xyzw'], state['velocity_world'], state['omega_world'], target, previous)
    action = runtime.action(observation)
    force = thrust_action(action)
    if command.complete:
        force = np.zeros(2)
        action = previous.copy()
        if complete_at is None:
            complete_at = step
    state = boat.step(force)
    previous = action.copy()
    if np.max(np.abs(state['euler_rpy'][:2])) > 1.3 or abs(state['position'][2]) > 3.:
        raise RuntimeError('Boat exceeded stability limits')
    stage_ids.setdefault(command.stage, len(stage_ids))
    records.append([
        (step + 1) * .02, command.stage, *target,
        state['body_velocity'][0], state['body_omega'][2], *force, *action,
        *state['position'], *state['euler_rpy']])
    if complete_at is not None and step - complete_at >= 100:
        break
else:
    raise RuntimeError(f'Mission did not complete within {args.max_seconds} simulated seconds; stage={controller.stage}')

fields = ['time_s', 'stage', 'target_u', 'target_r', 'actual_u', 'actual_r',
          'left_N', 'right_N', 'raw_left', 'raw_right', 'x', 'y', 'z',
          'roll', 'pitch', 'yaw']
with (args.output / 'trajectory.csv').open('w', newline='') as stream:
    writer = csv.writer(stream)
    writer.writerow(fields)
    writer.writerows(records)

numeric = np.asarray([[float(value) for value in row[2:]] for row in records])
positions = numeric[:, 8:10]
origin = positions[0]
summary = dict(
    mission=args.mission,
    completed=controller.stage == 'complete',
    duration_s=records[-1][0],
    controls=len(records),
    final_position_m=positions[-1].tolist(),
    final_distance_from_start_m=float(np.linalg.norm(positions[-1] - origin)),
    max_roll_rad=float(np.max(np.abs(numeric[:, 11]))),
    max_pitch_rad=float(np.max(np.abs(numeric[:, 12]))),
    max_forward_speed_m_s=float(np.max(np.abs(numeric[:, 2]))),
    stages=stage_ids,
    path='relative mission -> deterministic ONNX PPO -> Newton forces -> Gazebo Harmonic marine physics',
    checkpoint_sha256=runtime.contract['checkpoint_sha256'])

if args.mission == 'out-and-back':
    summary.update(
        requested_outbound_m=20.,
        maximum_distance_from_start_m=float(np.max(np.linalg.norm(positions-origin, axis=1))))
else:
    circle_rows = np.array([row[1] == 'circle' for row in records])
    circle_positions = positions[circle_rows]
    radii = np.linalg.norm(circle_positions - controller.center, axis=1)
    summary.update(
        requested_center_distance_m=25., requested_diameter_m=12.,
        reached_center_m=controller.center.tolist(),
        mean_circle_radius_m=float(np.mean(radii)),
        circle_radius_rmse_m=float(np.sqrt(np.mean((radii-controller.radius)**2))),
        circle_progress_rad=float(controller.circle_progress))

(args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
