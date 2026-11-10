import importlib
import sys
import time
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
def node_factory(monkeypatch, tmp_path, calibration):
    nodes = []
    settings = {'log_directory': str(tmp_path)}
    class FakeNode:
        def __init__(self, name):
            self.values = {}
            self.publishers = {}


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
            return S(service_is_ready=lambda: False)


        def create_timer(self, *args, **kwargs):
            return S(cancel=lambda: None)


        def get_clock(self):
            return S(now=lambda: S(nanoseconds=int(time.time() * 1e9)))


        def get_logger(self):
            return S(info=lambda text: None, error=lambda text: None)


        def destroy_node(self):
            pass
    modules = {'rclpy': {}, 'rclpy.node': {'Node': FakeNode}, 'rclpy.clock': {'Clock': Message, 'ClockType': S(STEADY_TIME=1)},
        'rclpy.signals': {'SignalHandlerOptions': S(NO=0)}, 'rclpy.qos': {'qos_profile_sensor_data': 1},
        'geometry_msgs.msg': {'Twist': Message}, 'nav_msgs.msg': {'Odometry': Message}, 'sensor_msgs.msg': {'Imu': Message},
        'std_msgs.msg': {name: Message for name in ('Bool','Float64MultiArray','String')},
        'mavros_msgs.msg': {name: Message for name in ('State','RCIn','OverrideRCIn','EstimatorStatus')},
        'mavros_msgs.srv': {name: S(Request=Message) for name in ('ParamPull','VehicleInfoGet')},
        'rcl_interfaces.srv': {'GetParameters': S(Request=Message)}}
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop('blueboat_ppo_controller.ppo_node', None)
    module = importlib.import_module('blueboat_ppo_controller.ppo_node')
    def factory(mode='shadow'):
        settings.update(mode=mode, enable_actuators=mode=='actuator', calibration_file=str(calibration))
        node = module.PPONode()
        nodes.append(node)
        timestamp = time.time_ns()
        header = S(frame_id='base_link', stamp=S(sec=timestamp // 10**9, nanosec=timestamp % 10**9))
        vector = S(x=0., y=0., z=0.)
        node.odometry_callback(S(header=S(frame_id='map',stamp=header.stamp),child_frame_id='base_link',twist=S(twist=S(linear=vector))))
        node.imu_callback(S(header=header,orientation=S(x=0.,y=0.,z=0.,w=1.),angular_velocity=vector,orientation_covariance=[0]*9,angular_velocity_covariance=[0]*9))
        node.command_callback(S(linear=S(x=.4,y=0.,z=0.),angular=S(x=0.,y=0.,z=.2)))
        node.state_callback(S(connected=True,armed=mode=='actuator',mode='MANUAL' if mode=='actuator' else 'HOLD',system_status=4))
        node.estimator_callback(S(attitude_status_flag=True,velocity_horiz_status_flag=True,velocity_vert_status_flag=True,const_pos_mode_status_flag=False,gps_glitch_status_flag=False,accel_error_status_flag=False))
        node.enabled = node.rc_enabled = node.parameters_valid = node.vehicle_verified = True
        node.receipts.update({key:time.monotonic() for key in ['enable','rc','parameters']})
        return node
    yield factory
    for node in nodes:
        if not node.gate.closed:
            node.close()
    sys.modules.pop('blueboat_ppo_controller.ppo_node', None)


def test_full_shadow_loop_never_creates_actuator_publisher(node_factory):
    node = node_factory()
    node.control()
    assert '/mavros/rc/override' not in node.publishers
    assert len(node.forces.messages) == len(node.observations.messages) == 1
    assert len(node.observations.messages[0].data) == 289
    previous = node.previous_action.copy()
    assert np.any(previous != 0)
    node.state_callback(S(connected=True,armed=False,mode='HOLD',system_status=3))
    np.testing.assert_array_equal(previous,node.previous_action)
    node.control()
    obs = node.observations.messages[-1].data
    np.testing.assert_allclose(np.array(obs)[[53,108]], previous / 10, atol=1e-7)
    node.close()
    assert node.gate.closed and not node.watchdog.is_alive()


def test_actuator_timeout_neutral_release_and_latch(node_factory):
    node = node_factory('actuator')
    node.control()
    assert node.gate.active and node.owns_output
    node.receipts['command'] -= 1
    node.control()
    assert node.gate.fault == 'command_timeout'
    assert node.output.messages[-1].channels[0] == 1500
    deadline = time.monotonic() + .4
    while node.owns_output and time.monotonic() < deadline:
        time.sleep(.01)
    assert node.output.messages[-1].channels[0] == node.output.messages[-1].channels[2] == 0
    assert node.output.messages[-1].channels[7] == 65535
    node.receipts['command'] = time.monotonic()
    node.control()
    assert not node.gate.active


@pytest.mark.parametrize('reason', ['manual_takeover', 'shutdown'])
def test_takeover_shutdown_release_only_owned_channels(node_factory, reason):
    node = node_factory('actuator')
    node.control()
    node.stop_outputs(reason)
    assert not node.owns_output
    assert node.output.messages[-1].channels == [0,65535,0] + [65535]*15


def test_watchdog_independent_of_control_callback(node_factory):
    node = node_factory('actuator')
    node.control()
    node.gate.last_loop -= .2
    deadline = time.monotonic() + .4
    while node.owns_output and time.monotonic() < deadline:
        time.sleep(.01)
    assert node.gate.fault == 'watchdog_timeout'
    assert not node.owns_output


def test_source_timestamp_and_frame_failure_stop_outputs(node_factory):
    node = node_factory('actuator')
    node.control()
    node.imu_callback(node.imu)
    assert 'repeated' in node.gate.fault
    assert 'imu' not in node.receipts


def test_logging_failure_stops_active_outputs(node_factory, monkeypatch):
    node = node_factory('actuator')
    node.control()
    def fail(*args, **kwargs):
        raise OSError('simulated full disk')
    monkeypatch.setattr(node, 'record', fail)
    node.control()
    assert node.gate.fault == 'control_fault'
    assert node.output.messages[-1].channels[0] == 1500


def test_late_inference_cannot_publish_after_watchdog(node_factory, monkeypatch):
    node = node_factory('actuator')
    node.control()
    original = node.runtime.action
    def delayed(observation):
        time.sleep(.16)
        return original(observation)
    monkeypatch.setattr(node.runtime, 'action', delayed)
    count = len(node.forces.messages)
    node.control()
    assert node.gate.fault in ('watchdog_timeout','control_deadline_missed')
    assert len(node.forces.messages) == count
    assert node.output.messages[-1].channels[0] in (0,1500)


def test_parameter_mismatch_stops_control(node_factory):
    node = node_factory('actuator')
    node.control()
    node.parameters_received(S(result=lambda: S(values=[])))
    assert node.gate.fault == 'parameter_mismatch'
    assert not node.parameters_valid


def test_gcs_node_waits_for_grant_uses_local_socket_and_latches_loss(node_factory, gateway_files):
    import json
    import socket
    from pathlib import Path
    from blueboat_ppo_controller.authority import Authority, gateway_config
    directory, _ = gateway_files
    config = gateway_config(directory/'gateway.json')
    listener = socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    listener.bind(config['socket_file']); listener.settimeout(.5)
    try:
        node = node_factory('actuator')
        assert node.gateway is not None and node.output is None
        assert '/mavros/rc/override' not in node.publishers
        node.rc_enabled = False; node.receipts.pop('rc')
        node.control()
        assert not node.gate.active and not node.gate.fault
        a = Authority(True); now = time.monotonic(); a.ground_seen=now; a.vehicle(now,True,0,True)
        assert a.request('auto',now)
        Path(config['status_file']).write_text(json.dumps(a.status(now,config['config_sha256'])))
        node.control()
        payload = json.loads(listener.recv(4096))
        assert payload['pwm'] is not None and payload['session']==a.session and node.gate.active
        Path(config['status_file']).unlink()
        node.control()
        assert node.gate.fault=='gcs_authority_lost' and not node.owns_output
        node.close()
    finally:
        listener.close()
