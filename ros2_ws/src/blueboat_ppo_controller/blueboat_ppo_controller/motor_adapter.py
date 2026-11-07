import hashlib
from pathlib import Path

import numpy as np

from .motor_interface import MotorInterface
from .thrust_lookup import ThrustLookup


class MotorAdapter(MotorInterface):
    def __init__(self, path, target):
        super().__init__(path, target)
        config = self.config
        self.lookup = None
        if config.get('effective_lookup_file'):
            if not all(config.get(k) is True for k in ('effective_mapping_accepted', 'independent_effective_mapping_verified')):
                raise ValueError('Effective per-motor force mapping requires explicit model/identifiability acceptance')
            lookup_path = Path(path).parent / config['effective_lookup_file']
            if hashlib.sha256(lookup_path.read_bytes()).hexdigest() != config.get('effective_lookup_sha256'):
                raise ValueError('Effective lookup checksum mismatch')
            self.lookup = ThrustLookup(lookup_path)
            metadata = self.lookup.data.get('metadata', {})
            identity = metadata.get('motor_interface') or {}
            settings = metadata.get('settings', {})
            if (self.lookup.data.get('verified') is not True or self.lookup.data.get('independent_mapping_reviewed') is not True
                    or self.lookup.data.get('paired_validation',{}).get('passed') is not True
                    or settings.get('mode') != 'powered' or metadata.get('synthetic') is True
                    or identity.get('target') != target or any(identity.get(k) != config[k] for k in ('sysid','compid','flight_sw_version','uid'))
                    or identity.get('expected_parameters') != config['expected_parameters']
                    or self.lookup.data.get('evidence_sha256') != config['evidence_sha256']
                    or any(identity.get(side,{}).get(k) != config[side].get(k) for side in ('left','right') for k in ('rc_channel','servo_output','neutral_pwm','forward_pwm_sign'))
                    or any(self.lookup.neutral[side] != config[side]['neutral_pwm'] for side in ('left','right'))):
                raise ValueError('Lookup is unreviewed, synthetic, or from another vehicle/target/neutral')
            for side in ('left','right'):
                channel = config[side]['rc_channel']
                sign = config[side].get('forward_pwm_sign')
                if sign not in (-1,1):
                    raise ValueError('Effective lookup requires a verified forward PWM sign')
                for direction, polarity in (('forward',1),('reverse',-1)):
                    curve = self.lookup.curves.get(side+'_'+direction)
                    if curve:
                        pwm = np.array(curve['pwm'])
                        if (np.any(pwm < self.expected[f'RC{channel}_MIN']) or np.any(pwm > self.expected[f'RC{channel}_MAX'])
                                or np.any((pwm-config[side]['neutral_pwm'])*sign*polarity <= 0)):
                            raise ValueError('Lookup PWM disagrees with verified direction/RC limits')
        self.curves = []
        for name in ('left', 'right'):
            if self.lookup:
                continue
            values = np.asarray(config[name]['force_pwm_points'], dtype=float)
            if values.ndim != 2 or values.shape[1] != 2 or len(values) < 3 or not np.isfinite(values).all():
                raise ValueError('Invalid measured force/PWM curve')
            if not np.all(np.diff(values[:, 0]) > 0) or not (np.all(np.diff(values[:, 1]) >= 0) or np.all(np.diff(values[:, 1]) <= 0)):
                raise ValueError('Calibration curve must be monotonic')
            if values[0, 0] > -15 or values[-1, 0] < 25 or np.min(values[:, 1]) < 1000 or np.max(values[:, 1]) > 2000:
                raise ValueError('Curve must cover the full policy force range within 1000..2000 PWM')
            zero = values[values[:, 0] == 0]
            if len(zero) != 1 or zero[0, 1] != config[name]['neutral_pwm']:
                raise ValueError('An explicitly measured neutral point is required')
            self.curves.append(values)
        self.force_limit = float(config['maximum_abs_force_N'])
        if not np.isfinite(self.force_limit) or not 0 < self.force_limit <= 25:
            raise ValueError('Invalid deployment thrust cap')


    def convert(self, forces):
        values = np.asarray(forces, dtype=float)
        if values.shape != (2,) or not np.isfinite(values).all():
            raise ValueError('Invalid left/right forces')
        applied = np.clip(values, max(-15., -self.force_limit), min(25., self.force_limit))
        if self.lookup:
            pairs = [self.lookup.inverse(force, side) for force, side in zip(applied, ('left', 'right'))]
            return [p[0] for p in pairs], np.array([p[1] for p in pairs])
        pwm = [int(round(np.interp(force, curve[:, 0], curve[:, 1]))) for force, curve in zip(applied, self.curves)]
        return pwm, applied
