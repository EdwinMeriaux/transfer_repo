import hashlib
import json
from pathlib import Path
import time

import numpy as np
import onnxruntime as ort
from scipy.spatial.transform import Rotation


class GeneralistPolicyRuntime:
    """History-aware e274 BlueBoat actor with checksum and schema validation."""

    def __init__(self, directory=None, threads=1):
        """directory: a model folder, or the name of a packaged one under assets/
        (e274_6_5b, the default, or blueboat_u990)."""
        assets = Path(__file__).parent / 'assets'
        if not directory:
            directory = assets / 'e274_6_5b'
        elif '/' not in str(directory) and (assets / str(directory)).is_dir():
            directory = assets / str(directory)
        self.directory = Path(directory)
        manifest = json.loads((self.directory / 'manifest.json').read_text())
        self.manifest = manifest
        required = {'deploy_config.json', 'general_state_static.npy', 'joint_descriptions.npy'}
        if not required <= set(manifest['files']):
            raise ValueError('manifest.json must checksum ' + ', '.join(sorted(required)))
        for name, digest in manifest['files'].items():
            path = self.directory / name
            if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise ValueError('Policy asset checksum mismatch: ' + name)

        self.config = json.loads((self.directory / 'deploy_config.json').read_text())
        if (self.config.get('robot') != 'blueboat'
                or self.config.get('policy_version') != 'current'
                or self.config.get('nr_actuator_joints') != 2
                or self.config.get('actuator_names') != ['motor_port', 'motor_stbd']
                or self.config.get('actuator_control_types') != ['direct', 'direct']
                or self.config.get('control_dt') != .02):
            raise ValueError('Unexpected e274 BlueBoat deployment contract')

        self.nr_joints = 2
        self.nominal_joint_positions = np.asarray(
            self.config['nominal_joint_positions'], dtype=np.float32)
        self.scaling_factors = np.asarray(
            self.config['scaling_factors'], dtype=np.float32)
        self.control_limits = np.asarray(
            self.config['actuator_control_limits'], dtype=np.float32)
        self.command_axis_mask = np.asarray(
            self.config['command_axis_mask'], dtype=np.float32)
        self.max_command_velocities = np.asarray([
            self.config['max_command_velocity_xy'],
            self.config['max_command_velocity_xy'],
            self.config['max_command_velocity_z'],
            self.config['max_command_velocity_yaw'],
        ], dtype=np.float32)
        self.zero_clip_thresholds = (
            self.config['zero_clip_threshold_percentage']
            * self.max_command_velocities)

        norm = self.config['obs_norm']
        self.previous_action_divisor = norm['previous_action_divisor']
        self.linear_velocity_divisor = norm['linear_velocity_divisor']
        self.linear_velocity_clip = norm['linear_velocity_clip']
        self.angular_velocity_divisor = norm['angular_velocity_divisor']
        self.angular_velocity_clip = norm['angular_velocity_clip']
        self.observation_clip = norm['observation_clip']

        self.history_lags = np.asarray(self.config['history_lags'], dtype=np.int32)
        self.body_indices = np.asarray(
            self.config['body_history_general_indices'], dtype=np.int32)
        self.body_reflection = np.asarray(
            self.config['body_history_reflection'], dtype=np.float32)
        self.tracked_frame = np.asarray(
            self.config['tracked_frame'], dtype=np.float32)
        self.general_state_static = np.clip(
            np.load(self.directory / 'general_state_static.npy').astype(np.float32),
            -self.observation_clip, self.observation_clip)
        joint_descriptions = np.load(
            self.directory / 'joint_descriptions.npy').astype(np.float32)
        description_size = self.config.get('joint_description_size', 55)
        if joint_descriptions.shape != (2, description_size):
            raise ValueError('Joint descriptions do not match deploy_config joint_description_size')
        self.joint_descriptions = np.clip(
            joint_descriptions, -self.observation_clip,
            self.observation_clip)[None].astype(np.float16).astype(np.float32)

        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self.onnx_file = manifest.get('onnx', 'e274_6.5B.onnx')
        if self.onnx_file not in manifest['files']:
            raise ValueError('The ONNX actor must be listed and checksummed in manifest.json')
        self.session = ort.InferenceSession(
            str(self.directory / self.onnx_file), sess_options=options,
            providers=['CPUExecutionProvider'])
        expected = {
            'joint_description': [1, 2, description_size],
            'joint_state': [1, 2, 4],
            'general_state': [1, 32],
            'joint_history': [1, 2, 16],
            'body_history': [1, 74],
        }
        metadata = self.session.get_modelmeta().custom_metadata_map
        if (metadata.get('policy_version') != 'current'
                or metadata.get('robot') != 'blueboat'):
            raise ValueError('Unexpected ONNX policy metadata')
        actual = {value.name: value.shape for value in self.session.get_inputs()}
        if (actual != expected
                or [value.name for value in self.session.get_outputs()] != ['action']):
            raise ValueError('Unexpected ONNX input/output schema')

        self.backend = f'{self.directory.name}_onnxruntime_cpu_{threads}t'
        self.elapsed_s = 0.
        self.last_observation = np.empty(0, dtype=np.float32)
        self.reset_history()
        self.action_from_body(
            [0., 0., 0., 1.], np.zeros(3), np.zeros(3),
            np.zeros(2), np.zeros(2))
        self.reset_history()

    def reset_history(self):
        depth = int(self.history_lags.max())
        self.joint_history = np.zeros(
            (self.nr_joints, depth, 3), dtype=np.float16)
        self.body_history = np.zeros(
            (depth, len(self.body_indices)), dtype=np.float16)
        self.history_age = 0
        self.last_observation = np.empty(0, dtype=np.float32)

    def fix_command(self, command):
        values = np.asarray(command, dtype=np.float32)
        if values.shape == (2,):
            values = np.asarray([values[0], 0., 0., values[1]], dtype=np.float32)
        if values.shape != (4,) or not np.isfinite(values).all():
            raise ValueError('Expected finite surge/yaw or vx/vy/vz/yaw command')
        values = values * self.command_axis_mask
        values = np.where(
            np.abs(values) < self.zero_clip_thresholds, 0., values)
        return np.clip(values, -self.max_command_velocities,
                       self.max_command_velocities).astype(np.float32)

    def _joint_state(self, previous_action):
        previous = np.asarray(previous_action, dtype=np.float32)
        if previous.shape != (2,) or not np.isfinite(previous).all():
            raise ValueError('Expected two finite previous actions')
        # These are direct thrusters, not articulated joints. Only the raw
        # previous actor action is observable in the joint-state token.
        state = np.zeros((2, 4), dtype=np.float32)
        state[:, 2] = previous / self.previous_action_divisor
        return state.astype(np.float16).astype(np.float32)

    def _general_state(self, velocity, angular_velocity, gravity, command):
        velocity = np.asarray(velocity, dtype=np.float32)
        omega = np.asarray(angular_velocity, dtype=np.float32)
        gravity = np.asarray(gravity, dtype=np.float32)
        if any(value.shape != (3,) for value in (velocity, omega, gravity)):
            raise ValueError('Velocity, angular velocity and gravity must be 3-vectors')
        if not np.isfinite(np.r_[velocity, omega, gravity]).all():
            raise ValueError('Nonfinite policy telemetry')
        if abs(np.linalg.norm(gravity) - 1.) > .02:
            raise ValueError('Projected gravity is not a unit vector')
        general = np.concatenate([
            np.clip(velocity / self.linear_velocity_divisor,
                    -self.linear_velocity_clip, self.linear_velocity_clip),
            np.ones(1, dtype=np.float32),
            np.clip(omega / self.angular_velocity_divisor,
                    -self.angular_velocity_clip, self.angular_velocity_clip),
            command,
            np.zeros(4, dtype=np.float32),
            self.tracked_frame,
            gravity,
            self.general_state_static,
        ])
        if general.shape != (32,):
            raise RuntimeError('Unexpected general-state shape')
        return np.clip(
            general, -self.observation_clip,
            self.observation_clip).astype(np.float16).astype(np.float32)

    def action_from_body(self, quaternion_xyzw, velocity_flu,
                         angular_velocity_flu, command, previous_action):
        quaternion = np.asarray(quaternion_xyzw, dtype=float)
        if (quaternion.shape != (4,) or not np.isfinite(quaternion).all()
                or abs(np.linalg.norm(quaternion) - 1.) > .02):
            raise ValueError('Invalid FLU orientation quaternion')
        gravity = Rotation.from_quat(quaternion).inv().apply([0., 0., -1.])
        goal = self.fix_command(command)
        joint_state = self._joint_state(previous_action)
        general_state = self._general_state(
            velocity_flu, angular_velocity_flu, gravity, goal)

        lags = self.history_lags
        valid = (self.history_age >= lags).astype(np.float32)
        joint_features = np.concatenate([
            self.joint_history[:, lags - 1].reshape(2, 3 * len(lags)),
            np.broadcast_to(valid, (2, len(lags))),
        ], axis=-1).astype(np.float32)
        body = general_state[self.body_indices]
        body_features = np.concatenate([
            body, self.body_history[lags - 1].reshape(-1), valid,
        ]).astype(np.float32)
        inputs = {
            'joint_description': self.joint_descriptions,
            'joint_state': joint_state[None],
            'general_state': general_state[None],
            'joint_history': joint_features[None],
            'body_history': body_features[None],
        }
        self.last_observation = np.concatenate([
            inputs[name].reshape(-1) for name in (
                'joint_description', 'joint_state', 'general_state',
                'joint_history', 'body_history')
        ]).astype(np.float32)
        start = time.perf_counter()
        action = self.session.run(['action'], inputs)[0][0].astype(np.float32)
        self.elapsed_s = time.perf_counter() - start
        if action.shape != (2,) or not np.isfinite(action).all():
            raise RuntimeError('The e274 policy produced an invalid action')

        self.joint_history[:, 1:] = self.joint_history[:, :-1].copy()
        self.joint_history[:, 0] = np.column_stack([
            joint_state[:, :2], action / 10.])
        self.body_history[1:] = self.body_history[:-1].copy()
        self.body_history[0] = body
        self.history_age = min(self.history_age + 1, int(lags.max()))
        return action

    def actuator_controls(self, action):
        values = np.asarray(action, dtype=np.float32)
        if values.shape != (2,) or not np.isfinite(values).all():
            raise ValueError('Expected finite port/starboard actions')
        forces = self.scaling_factors * values
        return np.clip(
            forces, self.control_limits[:, 0],
            self.control_limits[:, 1]).astype(np.float32)
