import argparse
import json
from pathlib import Path

import jax
import jax.numpy as jnp
import mujoco
from mujoco import mjx
import numpy as np
from scipy.spatial.transform import Rotation
from blueboat_ppo_controller.policy_runtime import PolicyRuntime, thrust_action
from blueboat_ppo_controller.observation_adapter import ObservationAdapter
from dynamics import mjx_step, planar_indices

parser = argparse.ArgumentParser()
parser.add_argument('--seconds', type=float, default=24.)
parser.add_argument('--speed', type=float, default=.4)
parser.add_argument('--yaw-rate', type=float, default=.4)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
if not np.isfinite([args.seconds, args.speed, args.yaw_rate]).all() or args.seconds <= 0:
    parser.error('Finite commands and positive finite duration required')
if args.output.exists():
    raise FileExistsError(args.output)
model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('plane.xml')))
qpos, dofs = map(jnp.asarray, planar_indices(model))
actuators = jnp.array([model.actuator(name).id for name in ['left_thruster', 'right_thruster']])
coefficients = jnp.asarray(model.numeric('blueboat_drag').data)
m = mjx.put_model(model)
native_data = mujoco.MjData(model)
mujoco.mj_forward(model, native_data)
data = mjx.put_data(model, native_data)
advance = jax.jit(lambda d, f: mjx_step(m, d, f, qpos, dofs, actuators, coefficients))
runtime = PolicyRuntime()
adapter = ObservationAdapter(runtime.contract)
previous = np.zeros(2)
rows = []
for step in range(round(args.seconds / .02)):
    position, velocity = np.asarray(data.qpos), np.asarray(data.qvel)
    q = Rotation.from_euler('z', position[2]).as_quat()
    obs = adapter.from_world(q, [*velocity[:2], 0.], [0., 0., velocity[2]], [args.speed, args.yaw_rate], previous)
    action = runtime.action(obs)
    force = thrust_action(action)
    data = advance(data, jnp.asarray(force, dtype=jnp.float32))
    previous = action.copy()
    position, velocity = np.asarray(data.qpos), np.asarray(data.qvel)
    u = np.cos(position[2]) * velocity[0] + np.sin(position[2]) * velocity[1]
    rows.append([u - args.speed, velocity[2] - args.yaw_rate])
report = dict(rmse=np.sqrt(np.mean(np.asarray(rows)**2, axis=0)).tolist(), duration_s=args.seconds, finite=bool(np.isfinite(rows).all()))
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
