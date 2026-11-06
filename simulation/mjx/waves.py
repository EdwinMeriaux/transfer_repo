import numpy as np


FREQUENCIES_HZ = (0.18, 0.29, 0.43, 0.61)
AMPLITUDE_WEIGHTS = (0.40, 0.28, 0.20, 0.12)
DEFAULT_FORCE_LIMITS = (4.0, 6.0, 0.8)


def wave_components(seed, max_height=0.05, direction=0.0, xp=np):
    index = xp.arange(1, 9, dtype=xp.uint32)
    bits = xp.asarray(seed, dtype=xp.uint32) ^ (index * xp.uint32(0x9E3779B9))
    bits = (bits ^ (bits >> xp.uint32(16))) * xp.uint32(0x85EBCA6B)
    bits = (bits ^ (bits >> xp.uint32(13))) * xp.uint32(0xC2B2AE35)
    bits = bits ^ (bits >> xp.uint32(16))
    uniform = (bits >> xp.uint32(8)).astype(xp.asarray(max_height).dtype) / 16777216.0
    angles = direction + (uniform[:4] * 2.0 - 1.0) * xp.pi
    omega = 2.0 * xp.pi * xp.asarray(FREQUENCIES_HZ)
    wavenumber = omega ** 2 / 9.81
    amplitudes = max_height * 0.5 * xp.asarray(AMPLITUDE_WEIGHTS)
    return xp.stack((amplitudes, wavenumber * xp.cos(angles), wavenumber * xp.sin(angles), omega, uniform[4:] * 2.0 * xp.pi), axis=-1)


def wave_field(position, time, components, intensity=1.0, xp=np):
    phase = xp.sum(xp.asarray(position)[..., None, :] * components[:, 1:3], axis=-1) - time * components[:, 3] + components[:, 4]
    height = intensity * xp.sum(components[:, 0] * xp.cos(phase), axis=-1)
    gradient = -intensity * xp.sum((components[:, 0] * xp.sin(phase))[..., :, None] * components[:, 1:3], axis=-2)
    return height, gradient


def wave_generalized(position, yaw, time, components, intensity, force_limits, xp=np):
    c, s = xp.cos(yaw), xp.sin(yaw)
    lateral = xp.stack((-s, c)) * 0.35
    positions = xp.stack((position, position + lateral, position - lateral))
    gradients = wave_field(positions, time, components, xp=xp)[1]
    forward = gradients[:, 0] * c + gradients[:, 1] * s
    sideways = -gradients[0, 0] * s + gradients[0, 1] * c
    k = (2.0 * xp.pi * xp.asarray(FREQUENCIES_HZ)) ** 2 / 9.81
    slope_scale = 0.025 * xp.sum(xp.asarray(AMPLITUDE_WEIGHTS) * k)
    yaw_scale = 0.025 * 0.70 * xp.sum(xp.asarray(AMPLITUDE_WEIGHTS) * k ** 2)
    fx = -intensity * force_limits[0] * xp.tanh(forward[0] / slope_scale)
    fy = -intensity * force_limits[1] * xp.tanh(sideways / slope_scale)
    nz = intensity * force_limits[2] * xp.tanh((forward[1] - forward[2]) / yaw_scale)
    return xp.stack((c * fx - s * fy, s * fx + c * fy, nz))


def make_waves(intensity=0.0, seed=1, max_height=0.05, direction=0.0, force_limits=DEFAULT_FORCE_LIMITS):
    values = np.array([intensity, max_height, direction, *force_limits], dtype=float)
    if not np.isfinite(values).all() or not 0.0 <= intensity <= 1.0 or max_height < 0.0 or min(force_limits) < 0.0:
        raise ValueError("Wave intensity must be 0..1, height and force limits nonnegative, and all values finite")
    if not isinstance(seed, (int, np.integer)) or not 0 <= seed <= 0xFFFFFFFF:
        raise ValueError("Wave seed must be an integer in 0..4294967295")
    return {"components": wave_components(seed, max_height, direction), "intensity": np.asarray(intensity), "force_limits": np.asarray(force_limits, dtype=float)}
