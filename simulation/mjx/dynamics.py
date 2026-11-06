import mujoco
import numpy as np

from waves import wave_generalized


DRAG_FIELDS = (
    "surge_linear_N_s_m", "surge_quadratic_N_s2_m2",
    "sway_linear_N_s_m", "sway_quadratic_N_s2_m2",
    "yaw_linear_Nm_s_rad", "yaw_quadratic_Nm_s2_rad2",
)
COUPLING_FIELDS = (
    "sway_offset_m", "yaw_surge_coefficient", "surge_exponent",
    "yaw_exponent", "yaw_smoothing_rad_s", "yaw_flow_length_m",
)


def planar_indices(model):
    joints = [model.joint(name).id for name in ("world_x", "world_y", "yaw")]
    if list(model.jnt_type[joints]) != [2, 2, 3] or len(set(model.jnt_bodyid[joints])) != 1:
        raise ValueError("BlueBoat requires ordered world-X/Y slides followed by yaw on one body")
    if joints != sorted(joints) or not np.allclose(model.jnt_axis[joints], np.eye(3)):
        raise ValueError("BlueBoat planar joints must have world X, Y, Z axes in that order")
    return np.array(model.jnt_qposadr[joints]), np.array(model.jnt_dofadr[joints])


def current_velocity(speed=0.0, direction=0.0):
    if not np.isfinite([speed, direction]).all() or speed < 0:
        raise ValueError("Current speed must be finite and nonnegative, direction finite radians")
    return np.asarray([speed * np.cos(direction), speed * np.sin(direction)])


def resistance_generalized(yaw, velocity, coefficients, xp=np, current=(0.0, 0.0)):
    if len(coefficients) not in (6, 12):
        raise ValueError("BlueBoat drag requires six legacy or twelve coupled coefficients")
    c, s = xp.cos(yaw), xp.sin(yaw)
    vx, vy = velocity[0] - current[0], velocity[1] - current[1]
    u = c * vx + s * vy
    v = -s * vx + c * vy
    r = velocity[2]
    fx = -coefficients[0] * u - coefficients[1] * xp.abs(u) * u
    fy = -coefficients[2] * v - coefficients[3] * xp.abs(v) * v
    nz = -coefficients[4] * r - coefficients[5] * xp.abs(r) * r
    if len(coefficients) == 12:
        offset, strength, surge_exponent, yaw_exponent, smoothing, flow_length = coefficients[6:]
        lateral = v - offset * r
        fy = -coefficients[2] * lateral - coefficients[3] * xp.abs(lateral) * lateral
        rotational_flow = (flow_length * r) ** 2 + 1e-6
        surge_factor = (u * u + rotational_flow) ** (surge_exponent / 2.) - rotational_flow ** (surge_exponent / 2.)
        yaw_factor = r * (r * r + smoothing * smoothing) ** ((yaw_exponent - 1.) / 2.)
        nz -= strength * surge_factor * yaw_factor + offset * fy
    return xp.stack((c * fx - s * fy, s * fx + c * fy, nz))


def native_step(model, data, thrust, substeps=4, waves=None, current=(0.0, 0.0)):
    qpos, dofs = planar_indices(model)
    actuators = [model.actuator(name).id for name in ("left_thruster", "right_thruster")]
    coefficients = model.numeric("blueboat_drag").data
    data.ctrl[actuators] = np.clip(thrust, model.actuator_ctrlrange[actuators, 0], model.actuator_ctrlrange[actuators, 1])
    external = data.qfrc_applied.copy()
    try:
        for step in range(substeps):
            data.qfrc_applied[dofs] = external[dofs] + resistance_generalized(data.qpos[qpos[2]], data.qvel[dofs], coefficients, current=current)
            if waves is not None:
                data.qfrc_applied[dofs] += wave_generalized(data.qpos[qpos[:2]], data.qpos[qpos[2]], data.time, **waves)
            mujoco.mj_step(model, data)
    finally:
        data.qfrc_applied[:] = external


def mjx_step(model, data, thrust, qpos, dofs, actuators, coefficients, substeps=4, waves=None, current=(0.0, 0.0)):
    import jax
    import jax.numpy as jnp
    from mujoco import mjx

    external = data.qfrc_applied
    water_velocity = jnp.asarray(current)
    data = data.replace(ctrl=data.ctrl.at[actuators].set(jnp.clip(thrust, model.actuator_ctrlrange[actuators, 0], model.actuator_ctrlrange[actuators, 1])))

    def physics_step(step, current):
        drag = resistance_generalized(current.qpos[qpos[2]], current.qvel[dofs], coefficients, xp=jnp, current=water_velocity)
        if waves is not None:
            drag += wave_generalized(current.qpos[qpos[:2]], current.qpos[qpos[2]], current.time, **waves, xp=jnp)
        return mjx.step(model, current.replace(qfrc_applied=external.at[dofs].add(drag))).replace(qfrc_applied=external)

    return jax.lax.fori_loop(0, substeps, physics_step, data)
