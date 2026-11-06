import numpy as np


def rc_override(channels, pwm=None):
    if len(channels) != 2 or len(set(channels)) != 2 or any(type(c) is not int or not 1 <= c <= 8 for c in channels):
        raise ValueError('Only two distinct owned channels in 1..8 are supported')
    values = [65535] * 18
    if pwm is not None and (len(pwm) != 2 or any(not np.isfinite(v) or not 1000 <= v <= 2000 for v in pwm)):
        raise ValueError('Invalid PWM values')
    for i, channel in enumerate(channels):
        values[channel - 1] = 0 if pwm is None else int(pwm[i])
    return values


def telemetry_from_messages(odom, imu, world_frame, body_frame):
    if odom.header.frame_id != world_frame or odom.child_frame_id != body_frame or imu.header.frame_id != body_frame:
        raise ValueError('Telemetry frame IDs do not match configured ENU/FLU frames')
    if imu.orientation_covariance[0] < 0 or imu.angular_velocity_covariance[0] < 0:
        raise ValueError('IMU orientation or angular velocity unavailable')
    q = imu.orientation
    v = odom.twist.twist.linear
    w = imu.angular_velocity
    values = np.array([q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z])
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite telemetry')
    return values[:4], values[4:7], values[7:]
