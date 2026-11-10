import copy
import hashlib
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace as S
import sys

import pytest
import yaml

from blueboat_ppo_controller.boat_profile import BUNDLE, CHECKS, initialize, interface_from_parameters, read_parameters, report, command_for_run, main
from blueboat_ppo_controller.motor_interface import MotorInterface
from blueboat_ppo_controller.mavros_interface import rc_override


@pytest.fixture
def bundle(tmp_path):
    path = tmp_path/'boat with spaces'
    initialize(path)
    return path


def test_export_mapping_input_output_and_no_assumed_servo_neutral(bundle):
    p = json.loads((bundle/'motor_interface.json').read_text())
    assert p['left']['rc_channel'] == 1 and p['left']['servo_output'] == 3
    assert p['right']['rc_channel'] == 3 and p['right']['servo_output'] == 1
    assert p['expected_parameters']['SERVO3_REVERSED'] == 1
    assert p['expected_parameters']['SERVO1_TRIM'] == p['expected_parameters']['SERVO3_TRIM'] == 1510
    for side in ('left','right'):
        assert p[side]['neutral_pwm'] == 1500
        assert p[side]['servo_neutral_pwm'] is None and p[side]['servo_forward_pwm_sign'] is None
    message = rc_override([p[s]['rc_channel'] for s in ('left','right')], [1540,1500])
    assert message[:3] == [1540,65535,1500]
    with pytest.raises(ValueError, match='verified motor interface'):
        MotorInterface(bundle/'motor_interface.json','hardware')


def test_bundle_preserves_settings_and_reports_every_known_conflict(bundle):
    p = json.loads((bundle/'motor_interface.json').read_text())
    assert p['expected_parameters'] == json.loads((BUNDLE/'observed_parameters.json').read_text())
    result = report(bundle)
    assert not result['powered_ready'] and result['mass_kg'] == 23
    for conflict in ('MOT_THST_ASYM=1.6','RC_OVERRIDE_TIME=3','FS_ACTION=4'):
        assert any(conflict in message for message in result['blockers'])
    assert p['enable_rc_channel'] is None
    assert p['control_authority'] == 'direct_mavros'
    assert not any('receiver failsafe is disabled' in message or 'RCn_OPTION=46' in message for message in result['blockers'])
    assert all(p[key] is False for key in CHECKS)


def test_initialization_does_not_overwrite_operator_files(bundle):
    before = (bundle/'motor_interface.json').read_bytes()
    with pytest.raises(FileExistsError):
        initialize(bundle, mass=24)
    assert (bundle/'motor_interface.json').read_bytes() == before


@pytest.mark.parametrize('content', ['A 1\nA 2\n','A nan\n','A inf\n','bad line 3\n','', 'A nope\n'])
def test_bad_export_rejected(tmp_path, content):
    file=tmp_path/'bad.params'; file.write_text(content)
    with pytest.raises(ValueError): read_parameters(file)


def test_real_parameter_subset_can_be_reimported_and_preserves_evidence(tmp_path, bundle):
    values=json.loads((BUNDLE/'observed_parameters.json').read_text())
    values.update(RC7_OPTION=46,RC_OVERRIDE_TIME=.3,RC_OPTIONS=32,FS_THR_ENABLE=1,MOT_THST_ASYM=1)
    file=tmp_path/'reviewed export.params'
    file.write_text('# operator test export\n'+'\n'.join(f'{k}\t{v}' for k,v in values.items()))
    old=(bundle/'motor_interface.json').read_text()
    main(['--directory',str(bundle),'refresh','--params',str(file)])
    p=json.loads((bundle/'motor_interface.json').read_text())
    assert p['expected_parameters']==values and p['enable_rc_channel'] is None
    assert p['control_authority']=='direct_mavros'
    assert not p['verified'] and 'uid' not in p and p['left']['servo_neutral_pwm'] is None
    assert next(bundle.glob('motor_interface.before-refresh-*')).read_text()==old
    assert json.loads((bundle/'boat.json').read_text())['parameter_export_sha256']==hashlib.sha256(file.read_bytes()).hexdigest()
    assert report(bundle)['powered_ready'] is False


def test_nonambiguous_servo_functions_required(bundle):
    p=json.loads((bundle/'motor_interface.json').read_text())
    values=copy.deepcopy(p['expected_parameters']); values['SERVO2_FUNCTION']=73
    with pytest.raises(ValueError,match='one mapped servo'):
        interface_from_parameters(values,p)


def test_mass_and_measurement_command_updates_only_local_config(bundle):
    main(['--directory',str(bundle),'configure','--mass-kg','24.5','--mass-std-kg','0.5',
          '--imu-offset','0.1','0','0.2','--left-servo-neutral','1510','--right-servo-neutral','1510',
          '--left-servo-forward-sign','-1','--right-servo-forward-sign','1'])
    p=json.loads((bundle/'motor_interface.json').read_text())
    assert p['left']['servo_neutral_pwm']==1510 and p['left']['servo_forward_pwm_sign']==-1
    assert p['expected_parameters']['RC7_OPTION']==0
    assert not p['verified'] and not any(p[k] for k in CHECKS)
    settings=yaml.safe_load((bundle/'thrust_calibration.yaml').read_text())['blueboat_thrust_calibration']['ros__parameters']
    assert settings['mass_kg']==24.5 and settings['mode']=='dry_run' and not settings['enable_powered']
    assert settings['require_estimator_status'] is False
    controller=yaml.safe_load((bundle/'controller.yaml').read_text())['blueboat_ppo_controller']['ros__parameters']
    assert controller['require_estimator_status'] is False
    assert json.loads((bundle/'boat.json').read_text())['mass_kg']==24.5
    assert json.loads((bundle/'analysis.json').read_text())['imu_offset_flu_m']==[.1,0,.2]


@pytest.mark.parametrize('args', [['--mass-kg','nan'],['--mass-kg','0'],['--mass-std-kg','-1'],['--enable-rc-channel','1'],['--enable-rc-channel','8'],['--left-servo-neutral','999'],['--imu-offset','nan','0','0']])
def test_invalid_field_measurements_do_not_change_any_config(bundle,args):
    before={f.name:f.read_bytes() for f in bundle.iterdir() if f.is_file()}
    with pytest.raises(SystemExit): main(['--directory',str(bundle),'configure',*args])
    assert before=={f.name:f.read_bytes() for f in bundle.iterdir() if f.is_file()}


def test_safe_launch_commands_and_powered_gates(bundle):
    for mode in ('telemetry','dry_run'):
        cmd=command_for_run(bundle,mode)
        assert 'enable_powered:=false' in cmd and 'deployment_target:=hardware' in cmd
    assert 'mode:=shadow' in command_for_run(bundle,'shadow')
    with pytest.raises(ValueError,match='explicit'): command_for_run(bundle,'powered')
    with pytest.raises(ValueError,match='MOT_THST_ASYM'): command_for_run(bundle,'powered',True)
    with pytest.raises(ValueError,match='reviewed'): command_for_run(bundle,'ppo',True)


@pytest.mark.parametrize('rc_options',[1,4,5,37])
def test_ignored_receiver_or_failsafe_flags_block_runtime(calibration,rc_options):
    p=json.loads(calibration.read_text()); p['expected_parameters']['RC_OPTIONS']=rc_options
    calibration.write_text(json.dumps(p))
    with pytest.raises(ValueError,match='physical receiver'):
        MotorInterface(calibration,'sitl')


def test_capture_reads_services_only_and_never_approves(bundle,monkeypatch):
    from blueboat_ppo_controller.boat_capture import capture
    profile=json.loads((bundle/'motor_interface.json').read_text())
    calls=[]
    info=S(available_info=3,autopilot=3,type=11,sysid=1,compid=1,flight_sw_version=0x040602ff)
    replies=[S(success=True,vehicles=[info]),S(success=True),S(values=[S(type=3,double_value=v) for v in profile['expected_parameters'].values()])]
    class Message:
        def __init__(self,**kwargs): self.__dict__.update(kwargs)
    class Node:
        def __init__(self,*args): pass
        def create_client(self,kind,name):
            calls.append(name)
            return S(wait_for_service=lambda **kw:True,call_async=lambda request:S(done=lambda:True,exception=lambda:None,result=lambda:replies[len(calls)-1]))
        def destroy_node(self): pass
    for name,attributes in {
        'rclpy':dict(init=lambda:None,shutdown=lambda:None,spin_until_future_complete=lambda *args,**kw:None),
        'rclpy.node':dict(Node=Node),
        'mavros_msgs.srv':dict(ParamPull=S(Request=Message),VehicleInfoGet=S(Request=Message)),
        'rcl_interfaces.srv':dict(GetParameters=S(Request=Message))}.items():
        module=ModuleType(name); module.__dict__.update(attributes); monkeypatch.setitem(sys.modules,name,module)
    capture(bundle,profile)
    assert calls==['/mavros/vehicle_info_get','/mavros/param/pull','/mavros/param/get_parameters']
    saved=json.loads((bundle/'motor_interface.json').read_text())
    assert 'uid' not in saved and saved['flight_sw_version']==0x040602ff
    assert saved['verified'] is False and all(saved[k] is False for k in CHECKS)
    assert saved['expected_parameters']==json.loads((BUNDLE/'observed_parameters.json').read_text())
    snapshot=json.loads(next(bundle.glob('live-capture-*.json')).read_text())
    assert snapshot['identity_basis']=='system_component_firmware_and_parameters'
    assert 'reported_uid' not in snapshot
    assert snapshot['parameter_differences']=={}


def test_analysis_uses_session_mass_and_checks_routing(bundle,tmp_path,monkeypatch):
    import blueboat_ppo_controller.calibration_analysis as module
    main(['--directory',str(bundle),'configure','--mass-kg','25','--mass-std-kg','.5','--imu-offset','0','0','0'])
    profile=json.loads((bundle/'motor_interface.json').read_text())
    csv=tmp_path/'actual.csv'; csv.write_text('unused by mocked estimator\n')
    meta=dict(settings=dict(mass_kg=23),motor_interface=copy.deepcopy(profile))
    csv.with_suffix('.json').write_text(json.dumps(meta))
    measured=[]
    def fake_analysis(path,output,settings,neutral,metadata):
        measured.append((settings.mass_kg,neutral))
        return [],dict(curves={})
    monkeypatch.setattr(module,'analyze',fake_analysis)
    main(['--directory',str(bundle),'analyze',str(csv),'--output',str(tmp_path/'report')])
    assert measured==[(23,dict(left=1500,right=1500))]
    meta['motor_interface']['left']['rc_channel']=3
    csv.with_suffix('.json').write_text(json.dumps(meta))
    with pytest.raises(SystemExit): main(['--directory',str(bundle),'analyze',str(csv),'--output',str(tmp_path/'report2')])
    assert len(measured)==1


def test_approval_flags_do_not_replace_missing_vehicle_evidence(bundle):
    profile=json.loads((bundle/'motor_interface.json').read_text())
    profile.update(verified=True, **{key:True for key in CHECKS})
    (bundle/'motor_interface.json').write_text(json.dumps(profile))
    with pytest.raises((ValueError, OSError)):
        MotorInterface(bundle/'motor_interface.json','hardware')
    assert not report(bundle)['powered_ready']
    with pytest.raises(ValueError,match='MOT_THST_ASYM'):
        command_for_run(bundle,'powered',True)


def test_existing_receiver_profile_can_select_direct_mavros(bundle):
    profile=json.loads((bundle/'motor_interface.json').read_text())
    profile.pop('control_authority')
    profile.update(enable_rc_channel=7,verified=True,uid=123,gcs_gateway_file='old.json',
                   gcs_gateway_sha256='deadbeef',gcs_handoff_tested=True,
                   gcs_ground_loss_tested=True,gcs_no_bypass_verified=True,
                   gcs_heartbeat_isolation_tested=True,gcs_router_failure_tested=True)
    (bundle/'motor_interface.json').write_text(json.dumps(profile))
    main(['--directory',str(bundle),'configure','--control-authority','direct_mavros'])
    updated=json.loads((bundle/'motor_interface.json').read_text())
    assert updated['control_authority']=='direct_mavros' and updated['enable_rc_channel'] is None
    assert not any(key in updated for key in ('uid','gcs_gateway_file','gcs_gateway_sha256','gcs_handoff_tested',
                                               'gcs_ground_loss_tested','gcs_no_bypass_verified',
                                               'gcs_heartbeat_isolation_tested','gcs_router_failure_tested'))
    assert not updated['verified']
    assert updated['expected_parameters']==profile['expected_parameters']
    assert json.loads((bundle/'boat.json').read_text())['control_authority']=='direct_mavros'
    with pytest.raises(SystemExit):
        main(['--directory',str(bundle),'configure','--enable-rc-channel','7'])


def test_unknown_control_authority_is_not_silently_treated_as_receiver(calibration):
    profile=json.loads(calibration.read_text()); profile['control_authority']='unknown'
    calibration.write_text(json.dumps(profile))
    with pytest.raises(ValueError,match='Unknown control authority'):
        MotorInterface(calibration,'sitl')
