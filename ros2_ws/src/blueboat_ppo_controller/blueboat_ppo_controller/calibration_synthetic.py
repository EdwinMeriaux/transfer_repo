"""Synthetic evidence for exercising analysis; never usable to authorize actuation."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .thrust_calibration import PulsePlan, Recorder


def generate(path, mass_kg=42.5, added_mass_kg=0., seed=17):
    if not np.isfinite([mass_kg, added_mass_kg]).all() or mass_kg <= 0 or added_mass_kg < 0:
        raise ValueError('Invalid synthetic mass')
    rng = np.random.default_rng(seed)
    plan = PulsePlan(deltas=(-40, -60, 40, 60), repeats=3, baseline_s=1.2, recovery_s=1.2)
    recorder = Recorder(path)
    rotation = Rotation.from_euler('xyz', [.03, -.04, .2])
    q = rotation.as_quat()
    gravity = rotation.inv().apply([0, 0, -9.80665])
    velocity = 0.
    epoch, dt, neutral = 1800000000., .01, [1500, 1500]
    try:
        for step in range(int(len(plan.trials)*plan.period/dt)):
            now = step*dt
            trial, phase, pwm, pulse = plan.step(now, neutral)
            if trial < 0:
                break
            force = sum(np.sign(p-1500)*.002*(p-1500)**2 for p in pwm)
            acceleration = force/(mass_kg+added_mass_kg)-.05*velocity
            if phase == 'recovery':
                acceleration -= 5*velocity
            values = np.array([acceleration, 0., 0.])-gravity+np.array([.025, -.012, .018])+rng.normal(0, .002, 3)
            gyro = rng.normal(0, .0001, 3)
            row = dict(receipt_monotonic_s=now, ros_time_s=epoch+now, imu_stamp_s=epoch+now,
                attitude_stamp_s=epoch+now, velocity_stamp_s=epoch+now, output_stamp_s=epoch+now,
                command_monotonic_s=now, trial=trial, side=pulse.side, delta_pwm=pulse.delta_pwm,
                trial_end_monotonic_s=plan.started+(trial+1)*plan.period,
                phase=phase, mode='synthetic', valid=1, reason='', imu_frame='base_link',
                attitude_frame='base_link', velocity_frame='base_link', command_left_pwm=pwm[0], command_right_pwm=pwm[1],
                planned_left_pwm=pwm[0], planned_right_pwm=pwm[1], output_left_pwm=pwm[0], output_right_pwm=pwm[1],
                output_channels_json=json.dumps(pwm), vx=velocity, vy=0., vz=0.)
            row.update({'a'+axis: float(v) for axis,v in zip('xyz', values)})
            row.update({'w'+axis: float(v) for axis,v in zip('xyz', gyro)})
            row.update({'q'+axis: float(v) for axis,v in zip('xyzw', q)})
            recorder.write(row)
            velocity += acceleration*dt
    finally:
        recorder.close()
    return dict(settings=dict(mode='synthetic', mass_kg=mass_kg), synthetic=True, motor_interface=None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--mass-kg', type=float, default=42.5, help='SYNTHETIC test mass only')
    args = parser.parse_args()
    metadata = generate(args.csv, args.mass_kg)
    args.csv.with_suffix('.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print('Synthetic CSV only; cannot authorize powered operation: '+str(args.csv))
