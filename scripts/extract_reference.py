import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

parser = argparse.ArgumentParser(description='Extract exact trained weights, observation contract and JAX reference vectors')
parser.add_argument('--source', type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
source = args.source.resolve()
sys.path[:0] = [str(source), str(source / 'experiments'), str(source / 'experiments/gazebo_blueboat')]
import jax
import jax.numpy as jnp
import numpy as np
from scipy.spatial.transform import Rotation
from observation import FrozenPolicy

checkpoint = source / 'outputs/blueboat/coupled_training_20261026_230523/blueboat_coupled_4m_mac.model'
expected = '43213ea3bc2219d93acf991fdb74c02a559c1d2ed77a8af266138474eb1269e4'
assert hashlib.sha256(checkpoint.read_bytes()).hexdigest() == expected
policy = FrozenPolicy(checkpoint, expected, 'plane.xml')
env = policy.environment
weights = {}
for path, value in jax.tree_util.tree_flatten_with_path(policy.policy.params['params'])[0]:
    key = '/'.join(str(part.key) for part in path)
    weights[key] = np.asarray(value)
np.savez_compressed(root / 'policy/checkpoints/actor_weights.npz', **weights)
shutil.copy2(checkpoint, root / 'policy/checkpoints/blueboat_coupled_4m_mac.model')
groups = {}
for name in ['policy_linear_velocity_estimate_obs_idx', 'critic_imu_linear_vel_obs_idx', 'policy_imu_angular_vel_obs_idx', 'critic_imu_angular_vel_obs_idx', 'policy_gravity_vector_obs_idx', 'critic_gravity_vector_obs_idx', 'policy_base_frame_goal_velocity_obs_idx', 'critic_base_frame_goal_velocity_obs_idx', 'policy_joint_previous_actions_obs_idx', 'critic_joint_previous_actions_obs_idx']:
    groups[name] = np.asarray(getattr(env, name)).tolist()
contract = dict(checkpoint_sha256=expected, template=policy.template.tolist(), general_indices=np.asarray(env.policy_general_obs_idx).tolist(), description_size=env.joint_description_size, joint_size=policy.policy.joint_size, actuator_count=2, groups=groups, divisor=np.asarray(env.obs_norm_divisor).tolist(), offset=np.asarray(env.obs_norm_offset).tolist(), clip_low=np.asarray(env.obs_norm_clip_lo).tolist(), clip_high=np.asarray(env.obs_norm_clip_hi).tolist(), algorithm=policy.policy.algorithm_config, force_limits=policy.limits.tolist())
(root / 'policy/inference/observation_contract.json').write_text(json.dumps(contract, indent=2) + '\n')
rng = np.random.default_rng(7101)
observations, inputs = [], []
for index in range(1024):
    q = Rotation.from_euler('xyz', rng.uniform([-.4, -.4, -np.pi], [.4, .4, np.pi])).as_quat()
    v = rng.uniform([-1.2, -.5, -.3], [1.2, .5, .3])
    omega = rng.uniform([-.5, -.5, -1.], [.5, .5, 1.])
    command = rng.uniform([-.8, -.4], [.8, .4])
    previous = rng.uniform(-2., 2., 2)
    state = dict(quaternion_xyzw=q, velocity_world=v, omega_world=omega)
    observations.append(policy.observation(state, command, previous))
    inputs.append(np.r_[q, v, omega, command, previous])
obs = np.asarray(observations, dtype=np.float32)
actions = np.asarray(jax.jit(jax.vmap(policy.policy.action))(jnp.asarray(obs)))
np.savez_compressed(root / 'tests/reference_vectors.npz', inputs=np.asarray(inputs), observations=obs, actions=actions)
policy.environment.close()
print(json.dumps(dict(observation_size=len(policy.template), description_size=contract['description_size'], joint_size=contract['joint_size'], general_size=len(contract['general_indices']), weights={k:list(v.shape) for k,v in weights.items()}), indent=2))
