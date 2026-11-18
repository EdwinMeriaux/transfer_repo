"""ROS-free harness: runs the real node modules against stub rclpy/messages.

Everything is deterministic: a fake monotonic/ROS clock, timers fired by the
simulator, service calls answered synchronously, and the watchdog thread
replaced by explicit watch_once() calls. Works with or without a real ROS
installation because the stubs are installed with pytest's monkeypatch.
"""
import importlib.util
import sys
import threading
import time as real_time
import types
from pathlib import Path
from types import SimpleNamespace as NS

import blueboat_ppo_controller

PACKAGE = Path(blueboat_ppo_controller.__file__).resolve().parent
EPOCH = 1.8e9


class Clock:
    def __init__(self, start=1000.):
        self.mono = start

    def monotonic(self):
        return self.mono

    def ros(self):
        return EPOCH + self.mono


def message_type(name, typename=''):
    def init(self, **values):
        self.__dict__.update(values)
    return type(name, (), {'__init__': init, '_typename': typename})


def service_type(name):
    service = type(name, (), {})
    service.Request = message_type(name + 'Request')
    return service


class Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Timer:
    def __init__(self, period, callback):
        self.period, self.callback, self.cancelled = period, callback, False

    def cancel(self):
        self.cancelled = True


class Future:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        callback(self)


class Client:
    def __init__(self, name):
        self.name = name
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        if 'vehicle_info' in self.name:
            return Future(NS(success=True, vehicles=[NS()]))
        if 'pull' in self.name:
            return Future(NS(success=True))
        return Future(NS(values=[]))


class FakeNode:
    overrides = {}
    clock = None

    def __init__(self, name):
        self._name = name
        self._params = {}
        self.publishers, self.subscriptions, self.types, self.clients, self.timers = {}, {}, {}, {}, []

    def declare_parameter(self, name, value):
        self._params[name] = self.overrides.get(name, value)

    def get_parameter(self, name):
        return NS(value=self._params[name])

    def create_publisher(self, kind, topic, qos):
        self.publishers[topic] = Publisher(topic)
        return self.publishers[topic]

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions[topic] = callback
        self.types[topic] = kind._typename

    def create_timer(self, period, callback, clock=None):
        timer = Timer(period, callback)
        self.timers.append(timer)
        return timer

    def create_client(self, kind, name):
        self.clients[name] = Client(name)
        return self.clients[name]

    def get_clock(self):
        clock = self.clock
        return NS(now=lambda: NS(nanoseconds=int(round(clock.ros() * 1e9))))

    def get_logger(self):
        return NS(info=lambda *a: None, error=lambda *a: None, warning=lambda *a: None)

    def get_publishers_info_by_topic(self, topic):
        if topic in self.publishers:
            return [NS(node_name=self._name, node_namespace='/', topic_type='mavros_msgs/msg/OverrideRCIn', qos_profile='')]
        if topic in self.subscriptions:
            return [NS(node_name='mavros', node_namespace='/mavros', topic_type=self.types[topic], qos_profile='')]
        return []

    def get_name(self):
        return self._name

    def get_namespace(self):
        return '/'

    def destroy_node(self):
        pass


class DummyThread:
    def __init__(self, *args, **kwargs):
        pass

    def start(self):
        pass

    def join(self, timeout=None):
        pass


def _stub_modules():
    m = {}

    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        m[name] = mod
        return mod

    rclpy = module('rclpy', init=lambda **k: None, ok=lambda: True, shutdown=lambda: None,
                   spin_once=lambda *a, **k: None)
    rclpy.clock = module('rclpy.clock', Clock=lambda **k: None, ClockType=NS(STEADY_TIME=1))
    rclpy.executors = module('rclpy.executors', SingleThreadedExecutor=object)
    rclpy.node = module('rclpy.node', Node=FakeNode)
    rclpy.qos = module('rclpy.qos', qos_profile_sensor_data=None)
    rclpy.signals = module('rclpy.signals', SignalHandlerOptions=NS(NO=0))
    for package, names in {
            'sensor_msgs.msg': {'Imu': 'sensor_msgs/msg/Imu'},
            'nav_msgs.msg': {'Odometry': 'nav_msgs/msg/Odometry'},
            'geometry_msgs.msg': {'TwistStamped': 'geometry_msgs/msg/TwistStamped', 'Twist': 'geometry_msgs/msg/Twist'},
            'std_msgs.msg': {'Bool': 'std_msgs/msg/Bool', 'String': 'std_msgs/msg/String',
                             'Float64MultiArray': 'std_msgs/msg/Float64MultiArray'},
            'mavros_msgs.msg': {'State': 'mavros_msgs/msg/State', 'RCIn': 'mavros_msgs/msg/RCIn',
                                'RCOut': 'mavros_msgs/msg/RCOut', 'OverrideRCIn': 'mavros_msgs/msg/OverrideRCIn',
                                'EstimatorStatus': 'mavros_msgs/msg/EstimatorStatus'}}.items():
        root = package.split('.')[0]
        if root not in m:
            module(root)
        module(package, **{name: message_type(name, typename) for name, typename in names.items()})
    module('mavros_msgs.srv', ParamPull=service_type('ParamPull'), VehicleInfoGet=service_type('VehicleInfoGet'))
    module('rcl_interfaces')
    module('rcl_interfaces.srv', GetParameters=service_type('GetParameters'))
    try:
        import onnxruntime  # noqa: F401
    except ImportError:
        module('onnxruntime')
    return m


def load(monkeypatch, name, clock, overrides=None):
    """Import blueboat_ppo_controller.<name> fresh, against the stubs."""
    for key, value in _stub_modules().items():
        monkeypatch.setitem(sys.modules, key, value)
    monkeypatch.setattr(FakeNode, 'overrides', dict(overrides or {}))
    monkeypatch.setattr(FakeNode, 'clock', clock)
    spec = importlib.util.spec_from_file_location('blueboat_ppo_controller.' + name, PACKAGE / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    module.time = NS(monotonic=clock.monotonic, time_ns=real_time.time_ns)
    module.threading = NS(Thread=DummyThread, RLock=threading.RLock, Event=threading.Event)
    return module


# --- messages -----------------------------------------------------------------

def header(stamp_s, frame):
    sec = int(stamp_s)
    nanosec = min(int(round((stamp_s - sec) * 1e9)), 999999999)
    return NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame)


def vec(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


def imu(stamp_s, accel=(0., 0., 9.80665), gyro=(0., 0., 0.), q=(0., 0., 0., 1.)):
    return NS(header=header(stamp_s, 'base_link'), linear_acceleration=vec(*accel), angular_velocity=vec(*gyro),
              orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]), orientation_covariance=[0.] * 9,
              linear_acceleration_covariance=[0.] * 9, angular_velocity_covariance=[0.] * 9)


def odom(stamp_s, velocity=(0., 0., 0.)):
    return NS(header=header(stamp_s, 'map'), child_frame_id='base_link',
              twist=NS(twist=NS(linear=vec(*velocity), angular=vec())))


def rcout(stamp_s, channels):
    return NS(header=header(stamp_s, ''), channels=list(channels))


def state(connected=True, armed=True, mode='MANUAL', system_status=4):
    return NS(connected=connected, armed=armed, mode=mode, system_status=system_status)
