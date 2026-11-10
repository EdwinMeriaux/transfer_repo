import hashlib
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/blueboat_ppo_controller'))


@pytest.fixture
def calibration(tmp_path):
    evidence = tmp_path / 'synthetic-test-only.csv'
    evidence.write_text('Synthetic unit-test fixture. Never use to actuate a vehicle.\n')
    expected = dict(PILOT_STEER_TYPE=1, RCMAP_ROLL=1, RCMAP_THROTTLE=3, RC7_OPTION=46, SERVO1_FUNCTION=73, SERVO3_FUNCTION=74, RC_OVERRIDE_TIME=.3, FS_GCS_ENABLE=1, FS_THR_ENABLE=1, RC_OPTIONS=0,
        MOT_PWM_TYPE=0, MOT_THR_MIN=0, MOT_THR_MAX=100, MOT_SLEWRATE=100, MOT_THST_EXPO=0, MOT_STR_THR_MIX=.5, MOT_THST_ASYM=1, MANUAL_STR_EXPO=0, MANUAL_OPTIONS=0, MODE_CH=8)
    for channel in (1, 3):
        expected.update({f'RC{channel}_{key}':value for key,value in dict(MIN=1000, MAX=2000, TRIM=1500, DZ=30, REVERSED=0).items()})
        expected.update({f'SERVO{channel}_{key}':value for key,value in dict(MIN=1000, MAX=2000, TRIM=1500, REVERSED=0).items()})
    c = dict(schema=1, verified=True, target='sitl', vehicle_id='SYNTHETIC_TEST_ONLY', firmware_version='TEST', sysid=1, compid=1, flight_sw_version=0x040507ff, uid=0,
        evidence_file=evidence.name, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(), left_right_verified=True, neutral_verified=True, manual_takeover_tested=True, override_timeout_tested=True, gcs_failsafe_tested=True, paired_motor_test_passed=True, maximum_abs_force_N=25., enable_rc_channel=7, expected_parameters=expected,
        left=dict(rc_channel=1, servo_output=1, neutral_pwm=1500, force_pwm_points=[[-15, 1100], [0, 1500], [25, 1900]]),
        right=dict(rc_channel=3, servo_output=3, neutral_pwm=1510, force_pwm_points=[[-15, 1120], [0, 1510], [25, 1920]]))
    path = tmp_path / 'calibration.json'
    path.write_text(json.dumps(c))
    return path


@pytest.fixture
def gateway_files(calibration):
    import tempfile
    from blueboat_ppo_controller.boat_profile import initialize, main
    from blueboat_ppo_controller.authority import gateway_config
    with tempfile.TemporaryDirectory(prefix='boat-',dir='/tmp') as temporary:
        directory = Path(temporary)
        initialize(directory/'profile')
        directory /= 'profile'
        main(['--directory',str(directory),'gateway-configure','--qgc-ip','127.0.0.1'])
        profile = json.loads(calibration.read_text())
        profile.update(control_authority='joystick_gcs', enable_rc_channel=None,
                       gcs_gateway_file=str(directory/'gateway.json'), gcs_gateway_sha256=gateway_config(directory/'gateway.json')['config_sha256'])
        profile['expected_parameters'].update(SYSID_MYGCS=255,SYSID_THISMAV=1,SYSID_ENFORCE=0,FS_GCS_TIMEOUT=5,FS_TIMEOUT=5,FS_ACTION=2,RC_OPTIONS=37,FS_THR_ENABLE=0)
        calibration.write_text(json.dumps(profile))
        yield directory, profile
