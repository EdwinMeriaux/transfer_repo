"""Executor/transport mocks, explicitly NOT a ROS or ArduPilot SITL test."""
import copy
import hashlib
import importlib
import json
import sys
import time
from collections import deque
from types import ModuleType, SimpleNamespace as S

import numpy as np
import pytest


class Message:
    def __init__(self, **values):
        self.__dict__.update(values)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def calibration_node(monkeypatch, tmp_path, calibration):
    profile = json.loads(calibration.read_text())
    profile.update(telemetry_conventions_verified=True, imu_placement_verified=True)
    for side in ('left','right'):
        profile[side].update(forward_pwm_sign=1, servo_forward_pwm_sign=1, servo_neutral_pwm=profile[side]['neutral_pwm'])
    calibration.write_text(json.dumps(profile))
    settings = dict(log_directory=str(tmp_path), interface_file=str(calibration), mass_kg=42.5)
    nodes = []

    class FakeNode:
        def __init__(self, name):
            self.name, self.values, self.publishers = name, {}, {}

        def get_name(self):
            return self.name

        def get_namespace(self):
            return '/'

        def declare_parameter(self, name, default):
            self.values[name] = settings.get(name, default)

        def get_parameter(self, name):
            return S(value=self.values[name])

        def create_publisher(self, message_type, topic, qos):
            self.publishers[topic] = Publisher()
            return self.publishers[topic]

        def create_subscription(self, *args):
            return None

        def create_client(self, *args):
            return S(service_is_ready=lambda:False)

        def create_timer(self, *args, **kwargs):
            return S(cancel=lambda:None)

        def get_clock(self):
            return S(now=lambda:S(nanoseconds=time.time_ns()))

        def get_logger(self):
            return S(info=lambda text:None, error=lambda text:None)

        def get_publishers_info_by_topic(self, topic):
            types = {'/mavros/imu/data_raw':'sensor_msgs/msg/Imu', '/mavros/imu/data':'sensor_msgs/msg/Imu',
                     '/mavros/local_position/odom':'nav_msgs/msg/Odometry', '/mavros/rc/out':'mavros_msgs/msg/RCOut',
                     '/mavros/rc/override':'mavros_msgs/msg/OverrideRCIn'}
            if topic == '/mavros/rc/override' and topic not in self.publishers:
                return []
            return [S(topic_type=types[topic], node_name=self.name if topic in self.publishers else 'mavros',
                      node_namespace='/', qos_profile='sensor_data')]

        def destroy_node(self):
            pass

    modules = {'rclpy':{}, 'rclpy.node':{'Node':FakeNode}, 'rclpy.clock':{'Clock':Message,'ClockType':S(STEADY_TIME=1)},
        'rclpy.qos':{'qos_profile_sensor_data':1}, 'rclpy.signals':{'SignalHandlerOptions':S(NO=0)},
        'sensor_msgs.msg':{'Imu':Message}, 'nav_msgs.msg':{'Odometry':Message}, 'geometry_msgs.msg':{'TwistStamped':Message},
        'std_msgs.msg':{'Bool':Message,'String':Message},
        'mavros_msgs.msg':{name:Message for name in ('State','RCIn','RCOut','OverrideRCIn','EstimatorStatus')},
        'mavros_msgs.srv':{name:S(Request=Message) for name in ('ParamPull','VehicleInfoGet')},
        'rcl_interfaces.srv':{'GetParameters':S(Request=Message)}}
    for name,attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules,name,module)
    sys.modules.pop('blueboat_ppo_controller.calibration_node',None)
    module = importlib.import_module('blueboat_ppo_controller.calibration_node')

    def factory(mode=None, **overrides):
        settings.pop('mode', None)
        settings['enable_powered'] = mode=='powered'
        if mode is not None:
            settings['mode'] = mode
        settings.update(overrides)
        node = module.CalibrationNode()
        nodes.append(node)
        timestamp = time.time_ns()-1000000
        header = S(frame_id='base_link',stamp=S(sec=timestamp//10**9,nanosec=timestamp%10**9))
        vector = S(x=0.,y=0.,z=0.)
        imu = S(header=header, orientation=S(x=0.,y=0.,z=0.,w=1.), angular_velocity=vector,
                linear_acceleration=S(x=0.,y=0.,z=9.80665), orientation_covariance=[0]*9,
                angular_velocity_covariance=[0]*9, linear_acceleration_covariance=[0]*9)
        node.attitude_callback(imu)
        node.velocity_callback(S(header=S(frame_id='map',stamp=header.stamp),child_frame_id='base_link',twist=S(twist=S(linear=vector))))
        node.output_callback(S(header=header,channels=[1500,1500,1510]+[1500]*5))
        node.raw_callback(imu)
        now = time.monotonic()
        for stream in node.streams.values():
            stream.receipts = deque([now-1.1+i*.01 for i in range(111)],maxlen=2000)
        node.vehicle = S(connected=True,armed=True,mode='MANUAL',system_status=4)
        node.enabled = node.rc_enabled = node.estimator_valid = node.vehicle_verified = node.parameters_valid = True
        node.receipts.update({k:now for k in ('state','enable','rc','estimator','parameters')})
        node.inspect_graph()
        return node

    yield factory
    for node in nodes:
        if not node.finished.is_set():
            node.close()
    sys.modules.pop('blueboat_ppo_controller.calibration_node',None)


@pytest.mark.parametrize('mode',[None,'telemetry','dry_run'])
def test_default_and_dry_run_never_create_output(calibration_node, mode):
    node = calibration_node(mode)
    node.report()
    node.control()
    assert '/mavros/rc/override' not in node.publishers
    assert node.fault is None
    assert node.settings['mode']==('dry_run' if mode is None else mode)
    assert node.phase == ('telemetry' if mode=='telemetry' else 'baseline')
    assert node.command is None
    assert node.diagnostics.messages


def test_powered_requires_explicit_startup_enable(calibration_node):
    with pytest.raises(ValueError, match='enable_powered'):
        calibration_node('powered', enable_powered=False)
    with pytest.raises(ValueError, match='mass_kg'):
        calibration_node('powered', enable_powered=True, mass_kg=0.)


def test_software_and_physical_enable_mode_and_arming(calibration_node):
    node = calibration_node('powered')
    now = time.monotonic()
    node.enabled = False
    assert node.permission(now)=='enable_released'
    node.enabled = True
    node.rc_enabled = False
    assert node.permission(now)=='manual_takeover'
    node.rc_enabled = True
    node.vehicle.armed = False
    assert node.permission(now)=='manual_takeover_or_disarmed'
    node.vehicle.armed = True
    node.vehicle.mode = 'HOLD'
    assert node.permission(now)=='manual_takeover_or_disarmed'
    assert node.output.messages == []


def test_neutral_first_pulse_release_and_latch(calibration_node):
    node = calibration_node('powered')
    node.control()
    assert node.owns_output and node.command == [1500,1510]
    node.rest.since, node.rest.last = time.monotonic()-1.1, time.monotonic()
    node.plan.started = time.monotonic()-2.01
    node.control()
    assert node.command == [1460,1510]
    node.enable_callback(S(data=False))
    assert not node.owns_output and node.fault == 'enable_released'
    assert node.output.messages[-1].channels == [0,65535,0]+[65535]*15
    count = len(node.output.messages)
    node.enable_callback(S(data=True))
    node.control()
    assert len(node.output.messages)==count


def test_actual_neutral_feedback_prevents_start(calibration_node):
    node = calibration_node('powered')
    node.latest['output'].channels[0]=1600
    assert node.permission(time.monotonic())=='actual_output_not_neutral_left'
    node.control()
    assert not node.owns_output and node.output.messages==[]


def test_actual_pulse_response_and_idle_outputs_are_checked(calibration_node):
    node=calibration_node('powered')
    node.control()
    node.rest.since, node.rest.last = time.monotonic()-1.1, time.monotonic()
    node.plan.started=time.monotonic()-2.01
    node.control()
    node.phase_started-=.16
    assert node.permission(time.monotonic())=='actual_output_wrong_or_absent_response_left'
    node.latest['output'].channels[0]=1460
    assert node.permission(time.monotonic()) is None
    node.latest['output'].channels[2]=1600
    assert node.permission(time.monotonic())=='actual_output_not_neutral_right'


@pytest.mark.parametrize('kind',['rate','skew','frame','clock','gap'])
def test_telemetry_faults_stop_powered_trial(calibration_node,kind):
    node = calibration_node('powered')
    node.control()
    if kind=='rate':
        node.streams['raw'].receipts=deque([time.monotonic()])
    elif kind=='skew':
        node.latest['velocity'].header.stamp=S(sec=node.latest['velocity'].header.stamp.sec-1,nanosec=0)
    elif kind=='gap':
        now=time.monotonic()
        node.streams['raw'].receipts=deque([now-1.5]+[now-.7+i*.01 for i in range(71)])
    elif kind=='clock':
        node.raw_callback(node.latest['raw'])
    else:
        node.latest['attitude'].header.frame_id='imu_link'
        node.attitude_callback(node.latest['attitude'])
    node.control()
    assert node.fault is not None
    assert node.output.messages[-1].channels[0]==1500


def test_watchdog_and_recording_failure_release(calibration_node,monkeypatch):
    node = calibration_node('powered')
    node.control()
    node.last_loop -= .2
    deadline=time.monotonic()+.4
    while node.owns_output and time.monotonic()<deadline:
        time.sleep(.01)
    assert node.fault=='watchdog_timeout' and not node.owns_output
    assert node.output.messages[-1].channels[0]==0
    another=calibration_node('powered')
    another.control()
    def failed(row):
        raise OSError('full disk')
    monkeypatch.setattr(another.recorder,'write',failed)
    # A new valid raw source sample reaches the recording-failure path.
    imu=another.latest['raw']
    imu.header.stamp.nanosec+=1000000
    another.raw_callback(imu)
    assert another.fault.startswith('recording_failed')


def test_competing_publisher_and_parameter_mismatch(calibration_node,monkeypatch):
    node=calibration_node('powered')
    original=node.get_publishers_info_by_topic
    monkeypatch.setattr(node,'get_publishers_info_by_topic',lambda topic: original(topic)*2 if topic=='/mavros/rc/override' else original(topic))
    node.inspect_graph()
    assert node.permission(time.monotonic())=='topic_graph_unverified_or_competing_publisher'
    monkeypatch.setattr(node,'get_publishers_info_by_topic',original)
    node.inspect_graph()
    node.control()
    node.parameters_received(S(result=lambda:S(values=[])))
    assert not node.parameters_valid and node.fault.startswith('vehicle_or_parameter')


def test_slow_event_log_cannot_block_the_independent_watchdog(calibration_node,monkeypatch):
    node=calibration_node('powered')
    node.control()
    node.rest.since, node.rest.last = time.monotonic()-1.1, time.monotonic()
    node.plan.started=time.monotonic()-2.01
    monkeypatch.setattr(node,'event',lambda *args,**kwargs:time.sleep(.16))
    node.control()
    assert node.fault=='watchdog_timeout'
    assert node.output.messages[-1].channels[0] in (0,1500)


def test_unavailable_raw_orientation_is_expected(calibration_node):
    node=calibration_node()
    raw=copy.deepcopy(node.latest['raw'])
    raw.orientation_covariance[0]=-1
    raw.header.stamp.nanosec+=1000000
    node.raw_callback(raw)
    assert node.streams['raw'].last_error is None


def test_a_single_quiet_sample_cannot_start_a_pulse(calibration_node):
    node=calibration_node('powered')
    node.control()
    node.observe_rest(node.latest['raw'])
    node.plan.started=time.monotonic()-node.plan.baseline_s-.01
    node.control()
    assert node.fault=='rest_baseline_not_verified'
    assert all(p.channels[0] in (0,1500) and p.channels[2] in (0,1510) for p in node.output.messages)


@pytest.mark.parametrize('disturbance',['speed','angular_rate','acceleration','missing_attitude'])
def test_a_disturbed_baseline_resets_the_rest_interval(calibration_node,disturbance):
    node=calibration_node('powered')
    node.control()
    node.rest.since, node.rest.last = time.monotonic()-1.1, time.monotonic()
    assert node.rest.ready(time.monotonic())
    if disturbance=='speed':
        node.latest['velocity'].twist.twist.linear=S(x=.15,y=0.,z=0.)
    elif disturbance=='angular_rate':
        node.latest['raw'].angular_velocity=S(x=0.,y=0.,z=.2)
    elif disturbance=='acceleration':
        node.latest['raw'].linear_acceleration.x=.8
    else:
        node.latest.pop('attitude')
    node.observe_rest(node.latest['raw'])
    assert not node.rest.ready(time.monotonic())


def test_hardware_refuses_without_completed_calibration_sitl_acceptance(calibration_node):
    from pathlib import Path
    node=calibration_node('telemetry')
    profile_path=Path(node.settings['interface_file'])
    profile=json.loads(profile_path.read_text())
    profile.update(target='hardware',uid=1234,sitl_test_passed=True,restrained_motor_test_passed=True,
                   sitl_calibration_test_passed=False)
    profile_path.write_text(json.dumps(profile))
    with pytest.raises(ValueError,match='sitl_calibration_test_passed'):
        calibration_node('powered',deployment_target='hardware')


def test_callbacks_csv_estimator_and_ppo_inverse_end_to_end(calibration_node,monkeypatch,tmp_path):
    """Synthetic dynamics through production callbacks, never a live ROS/SITL test."""
    from blueboat_ppo_controller.calibration_analysis import analyze,settings_from_session
    from blueboat_ppo_controller.motor_adapter import MotorAdapter
    from blueboat_ppo_controller.policy_runtime import thrust_action
    measured_mass=57.3
    node=calibration_node('powered',mass_kg=measured_mass,baseline_s=1.2,recovery_s=1.2)
    # Drive a deterministic source/receipt clock; watchdog behavior has separate wall-clock tests.
    node.finished.set()
    node.watchdog.join(timeout=1.)
    clock=S(mono=1000.,ros=1800000000.)
    module=sys.modules['blueboat_ppo_controller.calibration_node']
    monkeypatch.setattr(module,'time',S(monotonic=lambda:clock.mono))
    monkeypatch.setattr(node,'ros_now',lambda:clock.ros)
    for stream in node.streams.values():
        stream.last_stamp=None
        stream.receipts.clear()
        stream.last_error='no_samples'
    node.latest.clear()
    velocity=0.
    rng=np.random.default_rng(19)
    dt=.01
    try:
        for step in range(int((len(node.plan.trials)*node.plan.period+5)/dt)):
            elapsed=step*dt
            clock.mono,clock.ros=1000.+elapsed,1800000000.+elapsed+.001
            tick_ns=1800000000*10**9+step*10**7
            header=S(frame_id='base_link',stamp=S(sec=tick_ns//10**9,nanosec=tick_ns%10**9))
            command=node.command if node.command is not None else node.neutral
            offsets=np.array(command)-node.neutral
            force=float(np.sum(np.sign(offsets)*.002*offsets**2))
            acceleration=force/measured_mass-.05*velocity
            if node.phase=='recovery':
                acceleration-=5*velocity
            node.attitude_callback(S(header=header,orientation=S(x=0.,y=0.,z=0.,w=1.),orientation_covariance=[0.]*9))
            node.velocity_callback(S(header=S(frame_id='map',stamp=header.stamp),child_frame_id='base_link',
                                     twist=S(twist=S(linear=S(x=velocity,y=0.,z=0.)))))
            node.output_callback(S(header=header,channels=[command[0],0,command[1]]+[0]*5))
            if step%10==0:
                node.state_callback(S(connected=True,armed=True,mode='MANUAL',system_status=4))
                node.enable_callback(S(data=True))
                channels=[1500]*8
                channels[node.motor.enable_channel-1]=2000
                node.rc_callback(S(channels=channels))
                node.estimator_callback(S(attitude_status_flag=True,velocity_horiz_status_flag=True,
                    velocity_vert_status_flag=True,const_pos_mode_status_flag=False,gps_glitch_status_flag=False,accel_error_status_flag=False))
                # Identity/FC parameter query verification is supplied by the transport fixture.
                node.receipts['parameters']=clock.mono
            a=np.array([acceleration+.025,-.012,9.80665+.018])+rng.normal(0.,.002,3)
            node.raw_callback(S(header=header,linear_acceleration=S(x=float(a[0]),y=float(a[1]),z=float(a[2])),
                angular_velocity=S(x=0.,y=0.,z=0.),linear_acceleration_covariance=[0.]*9,angular_velocity_covariance=[0.]*9))
            if step%50==0:
                node.report()
            if step%2==0:
                node.control()
            velocity+=acceleration*dt
            assert node.fault in (None,'complete'), node.fault
            if node.fault=='complete':
                break
        assert node.fault=='complete'
    finally:
        node.close()
    from pathlib import Path
    metadata=json.loads(node.run_path.with_suffix('.json').read_text())
    settings=settings_from_session(metadata)
    assert settings.mass_kg==measured_mass
    report=tmp_path/'callback-analysis'
    trials,lookup=analyze(node.run_path.with_suffix('.csv'),report,settings,
        dict(zip(('left','right'),node.neutral)),metadata)
    assert len(trials)==36 and all(t['accepted'] for t in trials), [(t['trial'],t['reason']) for t in trials if not t['accepted']]
    assert len(lookup['curves'])==6 and lookup['paired_validation']['passed']
    for key,curve in lookup['curves'].items():
        factor=2 if key.startswith('symmetric') else 1
        np.testing.assert_allclose(curve['force_magnitude_N'],factor*np.array([3.2,7.2]),rtol=.04)
    candidate_path=report/'ppo_calibration.draft.json'
    candidate=json.loads(candidate_path.read_text())
    assert candidate['verified'] is False and candidate['effective_mapping_accepted'] is False
    assert candidate['independent_effective_mapping_verified'] is False
    with pytest.raises(ValueError,match='verified motor interface'):
        MotorAdapter(candidate_path,'sitl')
    # Emulate reviewer acceptance ONLY for this temporary synthetic unit-test artifact.
    lookup['verified']=lookup['independent_mapping_reviewed']=True
    lookup_path=report/'lookup.json'
    lookup_path.write_text(json.dumps(lookup))
    candidate.update(verified=True,effective_mapping_accepted=True,independent_effective_mapping_verified=True,
                     maximum_abs_force_N=6.,effective_lookup_sha256=hashlib.sha256(lookup_path.read_bytes()).hexdigest())
    candidate_path.write_text(json.dumps(candidate))
    motor=MotorAdapter(candidate_path,'sitl')
    requested=thrust_action([.2,-1./3.])
    before=requested.copy()
    pwm,applied=motor.convert(requested)
    np.testing.assert_allclose(requested,[5.,-5.])
    np.testing.assert_array_equal(requested,before)
    np.testing.assert_array_equal(applied,requested)
    assert 1540<=pwm[0]<=1560 and 1450<=pwm[1]<=1470
    with pytest.raises(ValueError,match='measured region'):
        motor.convert([.1,0.])


def test_gcs_calibration_uses_gateway_records_commands_and_stops_on_loss(calibration_node, gateway_files):
    import socket
    from pathlib import Path
    from blueboat_ppo_controller.authority import Authority, gateway_config
    directory, _ = gateway_files
    config = gateway_config(directory/'gateway.json')
    listener=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    listener.bind(config['socket_file']);listener.settimeout(.5)
    try:
        node=calibration_node('powered')
        assert node.gateway is not None and node.output is None and node.graph_valid
        assert '/mavros/rc/override' not in node.publishers
        node.rc_enabled=False;node.receipts.pop('rc')
        node.control()
        assert node.plan.started is None and node.fault is None
        a=Authority(True);now=time.monotonic();a.ground_seen=now;a.vehicle(now,True,0,True);assert a.request('auto',now)
        Path(config['status_file']).write_text(json.dumps(a.status(now,config['config_sha256'])))
        node.control()
        payload=json.loads(listener.recv(4096))
        assert payload['pwm']==node.neutral and node.command==node.neutral
        assert node.command_time is not None and node.owns_output
        Path(config['status_file']).unlink()
        node.control()
        assert node.fault=='gcs_authority_lost' and not node.owns_output
        node.close()
    finally:
        listener.close()
