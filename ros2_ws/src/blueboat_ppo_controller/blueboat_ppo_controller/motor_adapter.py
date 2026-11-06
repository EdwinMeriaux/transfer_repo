import hashlib
import json
from pathlib import Path

import numpy as np


class MotorAdapter:
    def __init__(self, path, target):
        path = Path(path)
        config = json.loads(path.read_text())
        if config.get('schema') != 1 or config.get('verified') is not True or config.get('target') != target:
            raise ValueError('Actuation requires a verified calibration for this deployment target')
        if target not in ('sitl', 'hardware') or not config.get('vehicle_id') or not config.get('firmware_version'):
            raise ValueError('Missing vehicle/firmware calibration identity')
        evidence = path.parent / config['evidence_file']
        if hashlib.sha256(evidence.read_bytes()).hexdigest() != config['evidence_sha256']:
            raise ValueError('Calibration evidence checksum mismatch')
        checks = ['left_right_verified', 'neutral_verified', 'manual_takeover_tested', 'override_timeout_tested', 'gcs_failsafe_tested', 'paired_motor_test_passed']
        if target == 'hardware':
            checks += ['sitl_test_passed', 'restrained_motor_test_passed']
        if not all(config.get(key) is True for key in checks):
            raise ValueError('Required calibration and failsafe tests are unverified')
        for key in ('sysid', 'compid', 'flight_sw_version', 'uid'):
            if type(config.get(key)) is not int or config[key] < 0:
                raise ValueError('Missing verified vehicle identity: ' + key)
        if config['flight_sw_version'] == 0 or (target == 'hardware' and config['uid'] == 0):
            raise ValueError('Actual firmware version and hardware UID are required')
        self.config = config
        self.channels = [config[name]['rc_channel'] for name in ('left', 'right')]
        if any(type(channel) is not int or not 1 <= channel <= 8 for channel in self.channels) or len(set(self.channels)) != 2:
            raise ValueError('Use two distinct RC channels in 1..8')
        self.enable_channel = config['enable_rc_channel']
        if type(self.enable_channel) is not int or not 1 <= self.enable_channel <= 8 or self.enable_channel in self.channels:
            raise ValueError('Manual enable channel must be separate and in 1..8')
        self.curves = []
        for name in ('left', 'right'):
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
        self.expected = config['expected_parameters']
        fixed = {'MANUAL_STR_EXPO': 0, 'MANUAL_OPTIONS': 0, 'MOT_THST_ASYM': 1, 'PILOT_STEER_TYPE': 1, 'RCMAP_ROLL': self.channels[0], 'RCMAP_THROTTLE': self.channels[1], f'RC{self.enable_channel}_OPTION': 46}
        for key, value in fixed.items():
            if self.expected.get(key) != value:
                raise ValueError('Calibration must use documented Rover two-paddle mapping: ' + key)
        outputs = [config[name]['servo_output'] for name in ('left', 'right')]
        if len(set(outputs)) != 2 or any(type(v) is not int or not 1 <= v <= 16 for v in outputs):
            raise ValueError('Distinct left/right servo outputs required')
        for name, channel, output, function in zip(('left', 'right'), self.channels, outputs, (73, 74)):
            if self.expected.get(f'SERVO{output}_FUNCTION') != function:
                raise ValueError('Incorrect left/right motor function')
            for key in [f'RC{channel}_{suffix}' for suffix in ('MIN', 'MAX', 'TRIM', 'DZ', 'REVERSED')] + [f'SERVO{output}_{suffix}' for suffix in ('MIN', 'MAX', 'TRIM', 'REVERSED')]:
                if key not in self.expected:
                    raise ValueError('Calibration must pin ' + key)
        for key in ('MOT_PWM_TYPE', 'MOT_THR_MIN', 'MOT_THR_MAX', 'MOT_SLEWRATE', 'MOT_THST_EXPO', 'MOT_STR_THR_MIX', 'MOT_THST_ASYM', 'MANUAL_STR_EXPO', 'MANUAL_OPTIONS', 'MODE_CH'):
            if key not in self.expected:
                raise ValueError('Calibration must pin ' + key)
        if self.expected['MODE_CH'] in self.channels or self.expected['MODE_CH'] == self.enable_channel:
            raise ValueError('Mode selection must not share motor/enable channels')
        if not all(np.isfinite(value) for value in self.expected.values()):
            raise ValueError('Nonfinite expected parameters')
        if not 0 < self.expected.get('RC_OVERRIDE_TIME', -1) <= .5:
            raise ValueError('Require a tested positive RC override timeout <=0.5s')
        if self.expected.get('FS_GCS_ENABLE', 0) == 0 or self.expected.get('FS_THR_ENABLE', 0) == 0:
            raise ValueError('Existing GCS and receiver failsafes must remain enabled')
        if int(self.expected.get('RC_OPTIONS', 2)) & 2:
            raise ValueError('RC_OPTIONS is configured to ignore MAVLink overrides')
        self.force_limit = float(config['maximum_abs_force_N'])
        if not np.isfinite(self.force_limit) or not 0 < self.force_limit <= 25:
            raise ValueError('Invalid deployment thrust cap')


    def convert(self, forces):
        values = np.asarray(forces, dtype=float)
        if values.shape != (2,) or not np.isfinite(values).all():
            raise ValueError('Invalid left/right forces')
        applied = np.clip(values, max(-15., -self.force_limit), min(25., self.force_limit))
        pwm = [int(round(np.interp(force, curve[:, 0], curve[:, 1]))) for force, curve in zip(applied, self.curves)]
        return pwm, applied


    def verify_parameters(self, actual):
        return all(key in actual and np.isfinite(actual[key]) and np.isclose(actual[key], value, rtol=0, atol=1e-5) for key, value in self.expected.items())


    def neutral(self):
        return [int(self.config[name]['neutral_pwm']) for name in ('left', 'right')]


    def verify_vehicle(self, info):
        return bool(info.available_info & 2) and info.autopilot == 3 and info.type in (10, 11) and all(getattr(info, key) == self.config[key] for key in ('sysid', 'compid', 'flight_sw_version', 'uid'))
