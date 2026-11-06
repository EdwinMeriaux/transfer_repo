import json
import signal
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64MultiArray, String
from mavros_msgs.msg import State, RCIn, OverrideRCIn, EstimatorStatus
from mavros_msgs.srv import ParamPull, VehicleInfoGet
from rcl_interfaces.srv import GetParameters

from .policy_runtime import PolicyRuntime, thrust_action
from .observation_adapter import ObservationAdapter
from .motor_adapter import MotorAdapter
from .mavros_interface import rc_override, telemetry_from_messages
from .safety import SafetyGate


class PPONode(Node):
    def __init__(self):
        super().__init__('blueboat_ppo_controller')
        defaults = dict(mode='shadow', enable_actuators=False, deployment_target='sitl', calibration_file='', model_directory='',
            command_topic='/blueboat/cmd_vel', enable_topic='/blueboat/enable', odometry_topic='/mavros/local_position/odom', imu_topic='/mavros/imu/data',
            state_topic='/mavros/state', rc_input_topic='/mavros/rc/in', estimator_topic='/mavros/estimator_status', rc_override_topic='/mavros/rc/override',
            vehicle_info_service='/mavros/vehicle_info_get', param_pull_service='/mavros/param/pull', param_get_service='/mavros/param/get_parameters',
            forces_topic='/blueboat/requested_forces', diagnostics_topic='/blueboat/diagnostics', observation_topic='/blueboat/observation',
            world_frame='map', body_frame='base_link', telemetry_timeout_s=.2, command_timeout_s=.5, watchdog_timeout_s=.1, log_directory='')
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        self.runtime = PolicyRuntime(self.settings['model_directory'] or None)
        self.adapter = ObservationAdapter(self.runtime.contract)
        self.gate = SafetyGate(self.settings['mode'], self.settings['telemetry_timeout_s'], self.settings['command_timeout_s'], self.settings['watchdog_timeout_s'])
        self.motor = None
        self.output = None
        if self.gate.mode == 'actuator':
            if not self.settings['enable_actuators']:
                raise ValueError('Actuator mode also requires enable_actuators:=true')
            self.motor = MotorAdapter(self.settings['calibration_file'], self.settings['deployment_target'])
            self.output = self.create_publisher(OverrideRCIn, self.settings['rc_override_topic'], 1)
        self.lock = threading.RLock()
        self.receipts, self.stamps = {}, {}
        self.odom = self.imu = self.vehicle = None
        self.command = np.zeros(2)
        self.previous_action = np.zeros(2)
        self.enabled = self.rc_enabled = self.parameters_valid = self.estimator_valid = False
        self.param_pending = False
        self.vehicle_verified = False
        self.owns_output = False
        self.release_at = None
        self.last_status = None
        self.finished = threading.Event()
        directory = Path(self.settings['log_directory']).expanduser() if self.settings['log_directory'] else Path.home() / '.ros/blueboat'
        directory.mkdir(parents=True, exist_ok=True)
        self.log = (directory / f'controller-{time.time_ns()}.jsonl').open('x', buffering=1)
        self.forces = self.create_publisher(Float64MultiArray, self.settings['forces_topic'], 10)
        self.diagnostics = self.create_publisher(String, self.settings['diagnostics_topic'], 10)
        self.observations = self.create_publisher(Float64MultiArray, self.settings['observation_topic'], 10)
        self.create_subscription(Twist, self.settings['command_topic'], self.command_callback, 1)
        self.create_subscription(Bool, self.settings['enable_topic'], self.enable_callback, 1)
        self.create_subscription(Odometry, self.settings['odometry_topic'], self.odometry_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, self.settings['imu_topic'], self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(State, self.settings['state_topic'], self.state_callback, 10)
        self.create_subscription(RCIn, self.settings['rc_input_topic'], self.rc_callback, qos_profile_sensor_data)
        self.create_subscription(EstimatorStatus, self.settings['estimator_topic'], self.estimator_callback, qos_profile_sensor_data)
        self.vehicle_client = self.create_client(VehicleInfoGet, self.settings['vehicle_info_service'])
        self.pull_client = self.create_client(ParamPull, self.settings['param_pull_service'])
        self.get_client = self.create_client(GetParameters, self.settings['param_get_service'])
        self.parameter_timer = self.create_timer(5., self.request_parameters, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.control_timer = self.create_timer(.02, self.control, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.watchdog = threading.Thread(target=self.watch, daemon=True)
        self.watchdog.start()
        self.get_logger().info(f'Controller mode={self.gate.mode}; no automatic arming or flight-mode changes')


    def record(self, status, **values):
        row = dict(monotonic_s=time.monotonic(), ros_time_s=self.get_clock().now().nanoseconds / 1e9, mode=self.gate.mode, status=status, **values)
        payload = json.dumps(row, allow_nan=False)
        self.log.write(payload + '\n')
        self.diagnostics.publish(String(data=payload))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status


    def accept_stamp(self, name, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if not 0 <= age <= self.gate.telemetry_timeout or stamp <= self.stamps.get(name, -1):
            self.receipts.pop(name, None)
            raise ValueError(name + ' has stale, repeated or future source timestamp')
        self.stamps[name] = stamp
        self.receipts[name] = time.monotonic()


    def odometry_callback(self, message):
        try:
            self.accept_stamp('odometry', message)
            self.odom = message
        except ValueError as error:
            self.stop_outputs(str(error))


    def imu_callback(self, message):
        try:
            self.accept_stamp('imu', message)
            self.imu = message
        except ValueError as error:
            self.stop_outputs(str(error))


    def command_callback(self, message):
        values = np.array([message.linear.x, message.linear.y, message.linear.z, message.angular.x, message.angular.y, message.angular.z])
        if not np.isfinite(values).all() or np.any(values[1:5] != 0) or abs(values[0]) > .8 or abs(values[5]) > .4:
            self.receipts.pop('command', None)
            self.stop_outputs('invalid_command')
            return
        self.command = values[[0, 5]]
        self.receipts['command'] = time.monotonic()


    def state_callback(self, message):
        self.vehicle = message
        self.receipts['state'] = time.monotonic()
        if self.motor and (not message.connected or not message.armed or message.mode != 'MANUAL'):
            self.stop_outputs('manual_takeover_or_disarmed')


    def estimator_callback(self, message):
        self.estimator_valid = bool(message.attitude_status_flag and message.velocity_horiz_status_flag and message.velocity_vert_status_flag and not message.const_pos_mode_status_flag and not message.gps_glitch_status_flag and not message.accel_error_status_flag)
        self.receipts['estimator'] = time.monotonic()
        if not self.estimator_valid:
            self.stop_outputs('estimator_invalid')


    def enable_callback(self, message):
        self.enabled = bool(message.data)
        self.receipts['enable'] = time.monotonic()
        if not self.enabled:
            self.stop_outputs('enable_released')


    def rc_callback(self, message):
        if self.motor:
            channel = self.motor.enable_channel
            self.rc_enabled = len(message.channels) >= channel and 1800 <= message.channels[channel - 1] <= 2200
            self.receipts['rc'] = time.monotonic()
            if not self.rc_enabled:
                self.stop_outputs('manual_takeover')


    def request_parameters(self):
        if not self.motor or self.param_pending or not all(client.service_is_ready() for client in (self.pull_client, self.get_client, self.vehicle_client)):
            return
        self.param_pending = True
        self.vehicle_client.call_async(VehicleInfoGet.Request(sysid=0, compid=0, get_all=False)).add_done_callback(self.vehicle_received)


    def vehicle_received(self, future):
        try:
            response = future.result()
            self.vehicle_verified = response.success and len(response.vehicles) == 1 and self.motor.verify_vehicle(response.vehicles[0])
            if not self.vehicle_verified:
                raise ValueError('Vehicle/firmware identity differs from measured calibration')
            self.pull_client.call_async(ParamPull.Request(force_pull=True)).add_done_callback(self.parameters_pulled)
        except Exception as error:
            self.param_pending = False
            self.parameters_valid = False
            self.stop_outputs('vehicle_identity_failed: ' + str(error))


    def parameters_pulled(self, future):
        try:
            if not future.result().success:
                raise ValueError('MAVROS parameter pull failed')
            request = GetParameters.Request(names=list(self.motor.expected))
            self.get_client.call_async(request).add_done_callback(self.parameters_received)
        except Exception as error:
            self.param_pending = False
            self.parameters_valid = False
            self.stop_outputs('parameter_pull_failed: ' + str(error))


    def parameters_received(self, future):
        try:
            values = future.result().values
            actual = {}
            for name, value in zip(self.motor.expected, values):
                if value.type == 2:
                    actual[name] = value.integer_value
                elif value.type == 3:
                    actual[name] = value.double_value
            self.parameters_valid = self.motor.verify_parameters(actual)
            self.receipts['parameters'] = time.monotonic()
            if not self.parameters_valid:
                self.stop_outputs('parameter_mismatch')
        except Exception as error:
            self.parameters_valid = False
            self.stop_outputs('parameter_query_failed: ' + str(error))
        finally:
            self.param_pending = False


    def permission(self, now):
        vehicle = self.vehicle
        reason = self.gate.check(now, self.receipts, bool(vehicle and vehicle.connected and vehicle.system_status in (3, 4)), bool(vehicle and vehicle.armed), vehicle.mode if vehicle else '', self.enabled, self.rc_enabled, self.parameters_valid)
        if reason:
            return reason
        if self.motor and not self.vehicle_verified:
            return 'vehicle_identity_unverified'
        ros_now = self.get_clock().now().nanoseconds / 1e9
        if any(not 0 <= ros_now - self.stamps[name] <= self.gate.telemetry_timeout for name in ('imu', 'odometry')):
            return 'source_telemetry_stale'
        if not self.estimator_valid or not 0 <= now - self.receipts.get('estimator', -np.inf) <= 2.:
            return 'estimator_invalid_or_stale'
        if abs(self.stamps['imu'] - self.stamps['odometry']) > .1:
            return 'telemetry_time_skew'
        return None


    def publish_pwm(self, pwm):
        if self.output is not None:
            self.output.publish(OverrideRCIn(channels=rc_override(self.motor.channels, pwm)))


    def stop_outputs(self, reason):
        with self.lock:
            if self.gate.mode == 'actuator' and (self.gate.active or self.owns_output):
                self.gate.trip(reason)
                if reason in ('manual_takeover', 'manual_takeover_or_disarmed', 'enable_released', 'shutdown'):
                    if reason == 'shutdown':
                        self.publish_pwm(self.motor.neutral())
                    self.publish_pwm(None)
                    self.owns_output = False
                    self.release_at = None
                elif self.release_at is None:
                    self.publish_pwm(self.motor.neutral())
                    self.release_at = time.monotonic() + .1
            self.previous_action = np.zeros(2)


    def control(self):
        started = time.monotonic()
        try:
            reason = self.permission(started)
            if reason:
                self.stop_outputs(reason)
                self.record(reason)
                return
            q, v, omega = telemetry_from_messages(self.odom, self.imu, self.settings['world_frame'], self.settings['body_frame'])
            observation = self.adapter.from_body(q, v, omega, self.command, self.previous_action)
            action = self.runtime.action(observation)
            forces = thrust_action(action)
            pwm, applied = self.motor.convert(forces) if self.motor else (None, forces)
            with self.lock:
                now = time.monotonic()
                reason = self.permission(now)
                if now - started > .02:
                    reason = 'control_deadline_missed'
                if reason:
                    self.stop_outputs(reason)
                else:
                    self.forces.publish(Float64MultiArray(data=forces.tolist()))
                    self.observations.publish(Float64MultiArray(data=observation.astype(float).tolist()))
                    if self.motor:
                        self.publish_pwm(pwm)
                        self.owns_output = True
                        self.gate.active = True
                    self.previous_action = action.copy()
                    self.gate.last_loop = now
            if reason:
                self.record(reason)
                return
            self.record('actuating' if self.motor else 'shadow', command=self.command.tolist(), quaternion_xyzw=q.tolist(), velocity_flu=v.tolist(), angular_velocity_flu=omega.tolist(), action=action.tolist(), requested_forces_N=forces.tolist(), capped_forces_N=applied.tolist(), rc_pwm=pwm, inference_s=self.runtime.elapsed_s, control_elapsed_s=now - started)
        except Exception as error:
            self.stop_outputs('control_fault')
            try:
                self.record('control_fault', error=str(error))
            except Exception:
                self.get_logger().error('Controller logging failed; actuator output stopped')


    def watch(self):
        while not self.finished.wait(.02):
            with self.lock:
                if self.gate.active and self.gate.overdue(time.monotonic()):
                    self.stop_outputs('watchdog_timeout')
                if self.release_at is not None:
                    if time.monotonic() >= self.release_at:
                        self.publish_pwm(None)
                        self.owns_output = False
                        self.release_at = None
                    else:
                        self.publish_pwm(self.motor.neutral())


    def close(self):
        self.control_timer.cancel()
        self.parameter_timer.cancel()
        self.stop_outputs('shutdown')
        self.gate.shutdown()
        self.finished.set()
        self.watchdog.join(timeout=1.)
        try:
            self.record('shutdown')
        except Exception:
            self.get_logger().error('Shutdown log could not be written')
        self.log.close()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = threading.Event()
    handlers = {sig: signal.signal(sig, lambda number, frame: stop_requested.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = PPONode()
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        if rclpy.ok():
            rclpy.shutdown()
