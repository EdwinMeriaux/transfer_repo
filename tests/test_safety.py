import json
from types import SimpleNamespace as S
import numpy as np
import pytest
from blueboat_ppo_controller.motor_adapter import MotorAdapter
from blueboat_ppo_controller.mavros_interface import rc_override, telemetry_from_messages
from blueboat_ppo_controller.safety import SafetyGate


def test_motor_order_and_release(calibration):
    motor=MotorAdapter(calibration,'sitl')
    pwm,forces=motor.convert([25,-15])
    assert pwm == [1900,1120]
    msg=rc_override(motor.channels,pwm)
    assert len(msg)==18 and msg[0]==1900 and msg[2]==1120
    assert all(v==65535 for i,v in enumerate(msg) if i not in (0,2))
    release=rc_override(motor.channels)
    assert release[0]==release[2]==0 and release[8]==65535
    assert motor.neutral()==[1500,1510]
    assert motor.verify_parameters(motor.expected)
    actual=dict(motor.expected, PILOT_STEER_TYPE=0)
    assert not motor.verify_parameters(actual)


def test_calibration_limits_and_identity(calibration):
    c=json.loads(calibration.read_text())
    c['maximum_abs_force_N']=3.
    calibration.write_text(json.dumps(c))
    motor=MotorAdapter(calibration,'sitl')
    _,values=motor.convert([-100,100])
    np.testing.assert_array_equal(values,[-3,3])
    info=S(available_info=3,autopilot=3,type=11,sysid=1,compid=1,flight_sw_version=c['flight_sw_version'],uid=0)
    assert motor.verify_vehicle(info)
    info.flight_sw_version+=1
    assert not motor.verify_vehicle(info)
    with pytest.raises(ValueError): MotorAdapter(calibration,'hardware')


@pytest.mark.parametrize('field,value',[('verified',False),('neutral_verified',False),('manual_takeover_tested',False),('enable_rc_channel',1),('maximum_abs_force_N',float('nan'))])
def test_unverified_calibration_rejected(calibration,field,value):
    c=json.loads(calibration.read_text()); c[field]=value
    calibration.write_text(json.dumps(c))
    with pytest.raises(ValueError): MotorAdapter(calibration,'sitl')


def test_freshness_fault_latch_and_shutdown():
    gate=SafetyGate('actuator')
    receipts={name:1. for name in ['odometry','imu','command','state','rc','enable','parameters']}
    args=dict(receipts=receipts,connected=True,armed=True,vehicle_mode='MANUAL',enabled=True,rc_enabled=True,parameters_valid=True)
    assert gate.check(1.01,**args) is None
    assert gate.check(1.3,**args)=='odometry_timeout'
    assert gate.check(1.01,**dict(args,rc_enabled=False))=='manual_takeover'
    assert gate.check(1.01,**dict(args,vehicle_mode='HOLD'))=='manual_takeover_or_disarmed'
    gate.last_loop=1.
    assert gate.overdue(1.11)
    gate.trip('watchdog_timeout')
    assert gate.check(1.01,**args)=='watchdog_timeout'
    gate.shutdown()
    assert gate.check(1.01,**args)=='shutdown'


def test_shadow_does_not_require_actuation_enable():
    gate=SafetyGate('shadow')
    assert gate.check(1.,dict(odometry=1.,imu=1.,command=1.,state=1.),True,False,'HOLD',False,False,False) is None


def test_message_frame_and_validity_contract():
    vector=S(x=.1,y=.2,z=.3)
    odom=S(header=S(frame_id='map'),child_frame_id='base_link',twist=S(twist=S(linear=vector)))
    imu=S(header=S(frame_id='base_link'),orientation=S(x=0,y=0,z=0,w=1),angular_velocity=vector,orientation_covariance=[0]*9,angular_velocity_covariance=[0]*9)
    q,v,w=telemetry_from_messages(odom,imu,'map','base_link')
    np.testing.assert_array_equal(q,[0,0,0,1])
    np.testing.assert_array_equal(v,[.1,.2,.3])
    imu.orientation_covariance[0]=-1
    with pytest.raises(ValueError): telemetry_from_messages(odom,imu,'map','base_link')
    imu.orientation_covariance[0]=0
    odom.child_frame_id='base_link_frd'
    with pytest.raises(ValueError): telemetry_from_messages(odom,imu,'map','base_link')
