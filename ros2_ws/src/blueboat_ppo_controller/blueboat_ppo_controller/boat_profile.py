import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import time

import yaml

from .motor_interface import MotorInterface

BUNDLE = Path(__file__).parent / 'profiles/alberto'
CHECKS = ('left_right_verified', 'neutral_verified', 'manual_takeover_tested', 'override_timeout_tested',
          'gcs_failsafe_tested', 'paired_motor_test_passed', 'sitl_test_passed', 'restrained_motor_test_passed',
          'telemetry_conventions_verified', 'imu_placement_verified', 'sitl_calibration_test_passed')


def read_parameters(path):
    values = {}
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        fields = line.replace(',', ' ').split()
        if len(fields) != 2 or not re.fullmatch(r'[A-Z][A-Z0-9_]*', fields[0]):
            raise ValueError(f'Expected NAME VALUE at line {number}')
        name, raw = fields
        value = float(raw)
        if not math.isfinite(value) or name in values:
            raise ValueError(f'Nonfinite or duplicate parameter: {name}')
        values[name] = int(value) if value.is_integer() else value
    if not values:
        raise ValueError('Empty parameter export')
    return values


def interface_from_parameters(values, template):
    profile = copy.deepcopy(template)
    required = ['PILOT_STEER_TYPE', 'RCMAP_ROLL', 'RCMAP_THROTTLE', 'RCMAP_PITCH', 'RCMAP_YAW',
                'MANUAL_STR_EXPO', 'MANUAL_OPTIONS', 'MODE_CH', 'RC_OPTIONS', 'RC_OVERRIDE_TIME',
                'MOT_PWM_TYPE', 'MOT_THR_MIN', 'MOT_THR_MAX', 'MOT_SLEWRATE', 'MOT_THST_EXPO',
                'MOT_STR_THR_MIX', 'MOT_THST_ASYM', 'FS_GCS_ENABLE', 'FS_THR_ENABLE',
                'SYSID_THISMAV', 'SYSID_MYGCS', 'SYSID_ENFORCE']
    for key in required:
        if key not in values:
            raise ValueError('Missing required parameter: ' + key)
    expected = {key:values[key] for key in required}
    for key, value in values.items():
        if key.startswith('FS_') or re.fullmatch(r'RC\d+_OPTION', key):
            expected[key] = value
    enable = [c for c in range(1, 9) if values.get(f'RC{c}_OPTION') == 46]
    profile['enable_rc_channel'] = enable[0] if len(enable) == 1 else None
    for side, rc_key, function in [('left', 'RCMAP_ROLL', 73), ('right', 'RCMAP_THROTTLE', 74)]:
        channel = values[rc_key]
        outputs = [n for n in range(1, 17) if values.get(f'SERVO{n}_FUNCTION') == function]
        if type(channel) is not int or not 1 <= channel <= 8 or len(outputs) != 1:
            raise ValueError('Expected one mapped servo and an RC input in 1..8 for ' + side)
        output = outputs[0]
        keys = [f'RC{channel}_{s}' for s in ('MIN','MAX','TRIM','DZ','REVERSED')]
        keys += [f'SERVO{output}_{s}' for s in ('FUNCTION','MIN','MAX','TRIM','REVERSED')]
        for key in keys:
            if key not in values:
                raise ValueError('Missing required parameter: ' + key)
            expected[key] = values[key]
        profile[side].update(rc_channel=channel, servo_output=output, servo_neutral_pwm=None, servo_forward_pwm_sign=None)
    if profile['left']['rc_channel'] == profile['right']['rc_channel']:
        raise ValueError('Left and right RC inputs must differ')
    profile.update(expected_parameters=expected, sysid=values['SYSID_THISMAV'], verified=False,
                   compid=None, flight_sw_version=None, uid=None, evidence_file='', evidence_sha256='')
    profile.update({key:False for key in CHECKS})
    return profile


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def initialize(directory, params=None, mass=None):
    profile = json.loads((BUNDLE / 'motor_interface.json').read_text())
    boat = json.loads((BUNDLE / 'boat.json').read_text())
    if params:
        profile = interface_from_parameters(read_parameters(params), profile)
        boat['parameter_export_sha256'] = hashlib.sha256(Path(params).read_bytes()).hexdigest()
        boat['parameter_export_name'] = Path(params).name
    if mass is not None:
        if not math.isfinite(mass) or mass <= 0:
            raise ValueError('Mass must be finite and positive')
        boat.update(mass_kg=mass, mass_source='operator CLI input; confirm total wet mass')
    directory.mkdir(parents=True, exist_ok=False)
    for source in BUNDLE.iterdir():
        if source.is_file():
            shutil.copy2(source, directory / source.name)
    write_json(directory / 'motor_interface.json', profile)
    write_json(directory / 'boat.json', boat)
    write_json(directory / 'observed_parameters.json', profile['expected_parameters'])
    for name, node in [('thrust_calibration.yaml', 'blueboat_thrust_calibration'), ('controller.yaml', 'blueboat_ppo_controller')]:
        config = yaml.safe_load((directory / name).read_text())
        settings = config[node]['ros__parameters']
        settings['log_directory'] = str(directory / 'runs' / ('calibration' if name.startswith('thrust') else 'ppo'))
        if name.startswith('thrust'):
            settings['mass_kg'] = boat['mass_kg']
            settings['interface_file'] = str(directory / 'motor_interface.json')
        (directory / name).write_text(yaml.safe_dump(config, sort_keys=False))


def report(directory):
    profile = json.loads((directory / 'motor_interface.json').read_text())
    boat = json.loads((directory / 'boat.json').read_text())
    expected = profile['expected_parameters']
    issues = []
    if profile.get('verified') is not True:
        issues.append('Motor interface is unverified; completed test evidence must be reviewed.')
    if expected.get('MOT_THST_ASYM') != 1:
        issues.append(f"MOT_THST_ASYM={expected.get('MOT_THST_ASYM')}: this independent-curve adapter requires 1; do not change without paired-output review.")
    if not 0 < expected.get('RC_OVERRIDE_TIME', -1) <= .5:
        issues.append(f"RC_OVERRIDE_TIME={expected.get('RC_OVERRIDE_TIME')}: a tested positive timeout <=0.5 s is required.")
    if expected.get('FS_THR_ENABLE', 0) == 0:
        issues.append('FS_THR_ENABLE=0: receiver failsafe is disabled; this package requires it enabled and tested.')
    if expected.get('FS_GCS_ENABLE', 0) == 0:
        issues.append('GCS failsafe is disabled; it must be enabled and tested.')
    if int(expected.get('RC_OPTIONS', 0)) & 5:
        issues.append(f"RC_OPTIONS={expected.get('RC_OPTIONS')}: physical receiver / receiver failsafe flags are ignored (bits 0/2); incompatible with receiver takeover.")
    channel = profile.get('enable_rc_channel')
    if type(channel) is not int or expected.get(f'RC{channel}_OPTION') != 46:
        issues.append('No verified separate RC override-enable switch (RCn_OPTION=46); channel must be chosen and tested on the vehicle.')
    for key in ('compid', 'flight_sw_version', 'uid'):
        if type(profile.get(key)) is not int or profile[key] <= 0:
            issues.append('Missing live hardware identity: ' + key + '; run capture after connecting MAVROS.')
    for side in ('left','right'):
        motor = profile[side]
        if motor.get('servo_neutral_pwm') is None or motor.get('servo_forward_pwm_sign') not in (-1,1):
            issues.append(f"{side}: measure actual servo neutral and forward sign; RC neutral {motor['neutral_pwm']} is separate from exported servo trim {expected.get('SERVO'+str(motor['servo_output'])+'_TRIM')}.")
    missing = [key for key in CHECKS if profile.get(key) is not True]
    if missing:
        issues.append('Tests/evidence still pending: ' + ', '.join(missing))
    try:
        MotorInterface(directory / 'motor_interface.json', profile['target'])
    except (ValueError, KeyError, TypeError, OSError) as error:
        issues.append('MotorInterface preflight: ' + str(error))
    return dict(vehicle=profile['vehicle_id'], mass_kg=boat['mass_kg'], mass_source=boat['mass_source'],
                mapping={s:profile[s] for s in ('left','right')}, powered_ready=not issues,
                blockers=issues, note='Static checks only; live telemetry, operator enable, mode, arm and output checks still apply. No FC settings were written.')


def command_for_run(directory, mode, enabled=False, calibration=None):
    if mode == 'shadow':
        return ['ros2','launch','blueboat_ppo_controller','controller.launch.py',
                'config:='+str(directory/'controller.yaml'),'mode:=shadow','deployment_target:=hardware']
    if mode == 'ppo':
        if not enabled or calibration is None:
            raise ValueError('PPO actuation needs --enable-actuators and --calibration with a reviewed measured mapping')
        from .motor_adapter import MotorAdapter
        MotorAdapter(calibration, 'hardware')
        return ['ros2','launch','blueboat_ppo_controller','controller.launch.py',
                'config:='+str(directory/'controller.yaml'),'mode:=actuator','enable_actuators:=true',
                'deployment_target:=hardware','calibration_file:='+str(calibration.resolve())]
    if mode == 'powered':
        if not enabled:
            raise ValueError('Powered calibration requires explicit --enable-actuators')
        result = report(directory)
        if not result['powered_ready']:
            raise ValueError('\n'.join(result['blockers']))
    return ['ros2','launch','blueboat_ppo_controller','calibration.launch.py',
            'config:='+str(directory/'thrust_calibration.yaml'),'mode:='+mode,
            'deployment_target:=hardware','interface_file:='+str(directory/'motor_interface.json'),
            'enable_powered:='+('true' if mode=='powered' else 'false')]


def main(args=None):
    parser = argparse.ArgumentParser(description='Alberto BlueBoat configuration and launch commands. Never sets FC parameters or arms.')
    parser.add_argument('--directory', type=Path, default=Path.home()/'.config/blueboat/alberto')
    subs = parser.add_subparsers(dest='operation', required=True)
    init = subs.add_parser('init', help='Copy complete prefilled bundle; refuses to overwrite an existing directory')
    init.add_argument('--params', type=Path)
    init.add_argument('--mass-kg', type=float)
    refresh = subs.add_parser('refresh', help='Re-import an operator-supplied export; clears approvals and unknown live measurements')
    refresh.add_argument('--params', type=Path, required=True)
    subs.add_parser('status')
    configure = subs.add_parser('configure', help='Save measurements/candidate channel; invalidates interface approval')
    configure.add_argument('--mass-kg', type=float)
    configure.add_argument('--mass-std-kg', type=float)
    configure.add_argument('--imu-offset', nargs=3, type=float, metavar=('X','Y','Z'))
    configure.add_argument('--enable-rc-channel', type=int)
    for side in ('left','right'):
        configure.add_argument('--'+side+'-servo-neutral', type=int)
        configure.add_argument('--'+side+'-servo-forward-sign', type=int, choices=(-1,1))
    subs.add_parser('capture', help='Read live identity and pinned parameters; no arming or parameter writes')
    run = subs.add_parser('run')
    run.add_argument('mode', choices=('telemetry','dry_run','shadow','powered','ppo'))
    run.add_argument('--enable-actuators', action='store_true')
    run.add_argument('--calibration', type=Path)
    analysis = subs.add_parser('analyze')
    analysis.add_argument('csv', type=Path)
    analysis.add_argument('--output', type=Path, required=True)
    parsed = parser.parse_args(args)
    directory = parsed.directory.expanduser().resolve()
    try:
        if parsed.operation == 'init':
            initialize(directory, parsed.params, parsed.mass_kg)
            print('Prepared ' + str(directory) + '. Run alberto_boat status; actuation remains disabled.')
            return
        if parsed.operation == 'status':
            result = report(directory)
            print(json.dumps(result, indent=2))
            return 0 if result['powered_ready'] else 2
        if parsed.operation == 'run':
            command = command_for_run(directory, parsed.mode, parsed.enable_actuators, parsed.calibration)
            os.execvp(command[0], command)
        profile = json.loads((directory/'motor_interface.json').read_text())
        if parsed.operation == 'refresh':
            updated = interface_from_parameters(read_parameters(parsed.params), profile)
            boat = json.loads((directory/'boat.json').read_text())
            boat.update(parameter_export_sha256=hashlib.sha256(parsed.params.read_bytes()).hexdigest(), parameter_export_name=parsed.params.name)
            shutil.copy2(directory/'motor_interface.json', directory/f'motor_interface.before-refresh-{time.time_ns()}.json')
            write_json(directory/'motor_interface.json', updated)
            write_json(directory/'observed_parameters.json', updated['expected_parameters'])
            write_json(directory/'boat.json', boat)
            print('Imported observed parameters only; approvals, live identity and servo measurements reset. No FC writes.')
        elif parsed.operation == 'configure':
            boat = json.loads((directory/'boat.json').read_text())
            analysis = json.loads((directory/'analysis.json').read_text())
            config = yaml.safe_load((directory/'thrust_calibration.yaml').read_text())
            if parsed.mass_kg is not None:
                if not math.isfinite(parsed.mass_kg) or parsed.mass_kg <= 0:
                    raise ValueError('Mass must be finite and positive')
                boat.update(mass_kg=parsed.mass_kg, mass_source='operator CLI input; confirm total wet mass')
                config['blueboat_thrust_calibration']['ros__parameters']['mass_kg'] = parsed.mass_kg
            if parsed.mass_std_kg is not None:
                if not math.isfinite(parsed.mass_std_kg) or parsed.mass_std_kg < 0:
                    raise ValueError('Mass uncertainty must be finite and nonnegative')
                analysis['mass_std_kg'] = parsed.mass_std_kg
            if parsed.imu_offset is not None:
                if not all(math.isfinite(v) for v in parsed.imu_offset):
                    raise ValueError('IMU offset must be finite')
                analysis['imu_offset_flu_m'] = parsed.imu_offset
            if parsed.enable_rc_channel is not None:
                if not 1 <= parsed.enable_rc_channel <= 8 or parsed.enable_rc_channel in (profile['left']['rc_channel'],profile['right']['rc_channel'],profile['expected_parameters']['MODE_CH']):
                    raise ValueError('Enable channel must be separate from motor/mode channels and in 1..8')
                profile['enable_rc_channel'] = parsed.enable_rc_channel
            for side in ('left','right'):
                neutral = getattr(parsed, side+'_servo_neutral')
                sign = getattr(parsed, side+'_servo_forward_sign')
                if neutral is not None:
                    if not 1000 <= neutral <= 2000:
                        raise ValueError('Servo neutral must be in 1000..2000')
                    profile[side]['servo_neutral_pwm'] = neutral
                if sign is not None:
                    profile[side]['servo_forward_pwm_sign'] = sign
            profile.update(verified=False, **{key:False for key in CHECKS})
            for name, value in [('motor_interface.json',profile),('boat.json',boat),('analysis.json',analysis)]:
                write_json(directory/name, value)
            (directory/'thrust_calibration.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
            print('Local measurements updated; interface approval/tests reset. No FC parameter changed.')
        elif parsed.operation == 'capture':
            from .boat_capture import capture
            capture(directory, profile)
        elif parsed.operation == 'analyze':
            options = json.loads((directory/'analysis.json').read_text())
            if options.get('mass_std_kg') is None or options.get('imu_offset_flu_m') is None:
                raise ValueError('First configure measured --mass-std-kg and --imu-offset X Y Z; these are not known from the parameter export')
            from .calibration_analysis import analyze, settings_from_session
            metadata = json.loads(parsed.csv.with_suffix('.json').read_text())
            recorded = metadata.get('motor_interface')
            if not isinstance(recorded, dict):
                raise ValueError('No recorded motor interface; use calibration_analyze for generic synthetic/telemetry analysis')
            if any(recorded.get(key) != profile.get(key) for key in ('target','sysid','compid','flight_sw_version','uid')):
                raise ValueError('Session vehicle identity differs from this local profile')
            if any(recorded.get(side, {}).get(key) != profile[side].get(key)
                   for side in ('left','right') for key in ('rc_channel','servo_output','neutral_pwm','forward_pwm_sign')):
                raise ValueError('Session motor mapping/neutrals differ from this local profile')
            settings = settings_from_session(metadata, None, options)
            trials, lookup = analyze(parsed.csv, parsed.output, settings,
                                     {side:profile[side]['neutral_pwm'] for side in ('left','right')}, metadata)
            print(json.dumps(dict(accepted=sum(t['accepted'] for t in trials), total=len(trials),
                                  mass_kg=settings.mass_kg, branches=list(lookup['curves']), verified=False), indent=2))
    except (ValueError, KeyError, TypeError, OSError, ImportError) as error:
        parser.exit(2, str(error)+'\n')


if __name__ == '__main__':
    sys.exit(main())
