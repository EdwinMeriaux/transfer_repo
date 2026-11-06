import numpy as np
from scipy.spatial.transform import Rotation


class ObservationAdapter:
    def __init__(self, contract):
        self.contract = contract
        self.template = np.asarray(contract['template'], dtype=np.float32)
        self.divisor = np.asarray(contract['divisor'])
        self.offset = np.asarray(contract['offset'])
        self.low = np.asarray(contract['clip_low'])
        self.high = np.asarray(contract['clip_high'])


    def from_body(self, quaternion_xyzw, velocity_flu, angular_velocity_flu, command, previous_action):
        q, velocity, omega, target, previous = [np.asarray(value, dtype=float) for value in (quaternion_xyzw, velocity_flu, angular_velocity_flu, command, previous_action)]
        if [value.shape for value in (q, velocity, omega, target, previous)] != [(4,), (3,), (3,), (2,), (2,)]:
            raise ValueError('Invalid observation input dimensions')
        if not np.isfinite(np.r_[q, velocity, omega, target, previous]).all():
            raise ValueError('Nonfinite observation input')
        if abs(np.linalg.norm(q) - 1.) > .02:
            raise ValueError('Orientation quaternion is not normalized')
        if np.max(np.abs(velocity)) > 5 or np.max(np.abs(omega)) > 5:
            raise ValueError('Measured velocity outside deployment validity limits')
        gravity = Rotation.from_quat(q).inv().apply([0., 0., -1.])
        values = dict(policy_linear_velocity_estimate_obs_idx=velocity, critic_imu_linear_vel_obs_idx=velocity,
            policy_imu_angular_vel_obs_idx=omega, critic_imu_angular_vel_obs_idx=omega,
            policy_gravity_vector_obs_idx=gravity, critic_gravity_vector_obs_idx=gravity,
            policy_base_frame_goal_velocity_obs_idx=np.array([target[0], 0., 0., target[1]]),
            critic_base_frame_goal_velocity_obs_idx=np.array([target[0], 0., 0., target[1]]),
            policy_joint_previous_actions_obs_idx=previous, critic_joint_previous_actions_obs_idx=previous)
        result = self.template.copy()
        for name, value in values.items():
            indices = np.asarray(self.contract['groups'][name])
            result[indices] = np.clip(value / self.divisor[indices] - self.offset[indices], self.low[indices], self.high[indices])
        return np.clip(result, -10., 10.).astype(np.float32)


    def from_world(self, quaternion_xyzw, velocity_enu, angular_velocity_enu, command, previous_action):
        q = np.asarray(quaternion_xyzw, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1.) > .02:
            raise ValueError('Invalid ENU orientation')
        inverse = Rotation.from_quat(q).inv()
        return self.from_body(q, inverse.apply(velocity_enu), inverse.apply(angular_velocity_enu), command, previous_action)
