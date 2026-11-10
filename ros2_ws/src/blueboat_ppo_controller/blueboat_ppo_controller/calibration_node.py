"""Dry-run by default. No arming, mode-setting or parameter writes."""
import json
import signal
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State, RCIn, RCOut, OverrideRCIn, EstimatorStatus
from mavros_msgs.srv import ParamPull, VehicleInfoGet
from rcl_interfaces.srv import GetParameters

from .mavros_interface import rc_override
from .motor_interface import MotorInterface
from .thrust_calibration import PulsePlan, Recorder, RestCheck, StreamCheck


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec/1e9


def xyz(vector):
    return [vector.x, vector.y, vector.z]


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('blueboat_thrust_calibration')
        defaults = dict(mode='dry_run', enable_powered=False, deployment_target='sitl', interface_file='',
            mass_kg=0., log_directory='', body_frame='base_link', world_frame='map',
            raw_imu_topic='/mavros/imu/data_raw', attitude_topic='/mavros/imu/data',
            velocity_topic='/mavros/local_position/odom', velocity_type='odometry',
            output_topic='/mavros/rc/out', state_topic='/mavros/state', rc_input_topic='/mavros/rc/in',
            estimator_topic='/mavros/estimator_status', rc_override_topic='/mavros/rc/override',
            enable_topic='/blueboat/calibration/enable', diagnostics_topic='/blueboat/calibration/status',
            vehicle_info_service='/mavros/vehicle_info_get', param_pull_service='/mavros/param/pull',
            param_get_service='/mavros/param/get_parameters', telemetry_timeout_s=.2, max_skew_s=.05,
            max_gap_s=.15, min_raw_rate_hz=40., min_attitude_rate_hz=20., min_velocity_rate_hz=10.,
            min_output_rate_hz=5., require_output_feedback=True, watchdog_timeout_s=.1,
            baseline_s=2., rest_duration_s=1., rest_max_gap_s=.06,
            pulse_s=.5, recovery_s=3., repeats=3, delta_pwm=[-40, -60, 40, 60],
            sides=['left', 'right', 'symmetric'], max_delta_pwm=60, dry_run_neutral_pwm=[1500, 1500],
            output_neutral_tolerance_pwm=15, output_response_grace_s=.15,
            output_min_response_pwm=3, accelerometer_semantics='specific_force',
            max_live_speed_m_s=.3, max_live_angular_rate_rad_s=.25, max_live_tilt_rad=.2)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        s = self.settings
        if s['mode'] not in ('telemetry', 'dry_run', 'powered') or s['velocity_type'] not in ('odometry', 'body_twist', 'world_twist'):
            raise ValueError('Invalid calibration mode or velocity type')
        positive = ('telemetry_timeout_s', 'max_skew_s', 'max_gap_s', 'min_raw_rate_hz', 'min_attitude_rate_hz',
                    'min_velocity_rate_hz', 'min_output_rate_hz', 'watchdog_timeout_s', 'max_live_speed_m_s',
                    'max_live_angular_rate_rad_s', 'max_live_tilt_rad', 'output_response_grace_s')
        if any(not np.isfinite(s[k]) or s[k] <= 0 for k in positive) or not 0 < s['watchdog_timeout_s'] <= .2:
            raise ValueError('Invalid telemetry or watchdog thresholds')
        if not 0 < s['telemetry_timeout_s'] <= .5 or not 0 < s['max_skew_s'] <= .1:
            raise ValueError('Unsafe timestamp limits')
        if not np.isfinite(s['mass_kg']) or s['mass_kg'] < 0:
            raise ValueError('Mass must be finite and nonnegative; powered mode requires measured mass >0')
        if type(s['output_neutral_tolerance_pwm']) is not int or not 0 <= s['output_neutral_tolerance_pwm'] <= 50:
            raise ValueError('Invalid neutral output tolerance')
        if (s['accelerometer_semantics'] not in ('specific_force', 'linear_acceleration')
                or type(s['output_min_response_pwm']) is not int or not 1 <= s['output_min_response_pwm'] <= 50):
            raise ValueError('Invalid acceleration semantics or output response threshold')
        self.plan = PulsePlan(s['delta_pwm'], s['sides'], s['repeats'], s['baseline_s'], s['pulse_s'], s['recovery_s'], s['max_delta_pwm'])
        if not .5 <= s['rest_duration_s'] <= s['baseline_s']-.1 or not 0 < s['rest_max_gap_s'] <= .1:
            raise ValueError('Rest verification needs >=0.5 s, room in baseline, and <=0.1 s sample gaps')
        self.rest = RestCheck(s['rest_duration_s'], s['rest_max_gap_s'])
        self.motor = None
        if s['mode'] == 'powered':
            if not s['enable_powered'] or s['mass_kg'] <= 0:
                raise ValueError('Powered mode requires enable_powered:=true and measured mass_kg >0')
            if s['output_response_grace_s'] >= s['pulse_s']-.05:
                raise ValueError('Pulse must outlast the output feedback grace window')
            self.motor = MotorInterface(s['interface_file'], s['deployment_target'])
            required = ['telemetry_conventions_verified', 'imu_placement_verified']
            if s['deployment_target'] == 'hardware':
                required += ['sitl_calibration_test_passed']
            if not all(self.motor.config.get(key) is True for key in required):
                raise ValueError('Calibration-specific checks are unverified: '+', '.join(required))
            if not s['require_output_feedback'] and self.motor.config.get('output_feedback_unavailable_accepted') is not True:
                raise ValueError('Missing output feedback requires explicit evidence acceptance')
            for side in ('left', 'right'):
                profile = self.motor.config[side]
                if (profile.get('forward_pwm_sign') not in (-1, 1) or profile.get('servo_forward_pwm_sign') not in (-1, 1)
                        or type(profile.get('servo_neutral_pwm')) is not int
                        or not 1000 <= profile['servo_neutral_pwm'] <= 2000):
                    raise ValueError('Require measured forward input sign and actual servo neutral PWM')
                channel, output = profile['rc_channel'], profile['servo_output']
                if (not self.motor.expected[f'SERVO{output}_MIN'] <= profile['servo_neutral_pwm'] <= self.motor.expected[f'SERVO{output}_MAX']
                        or any(not self.motor.expected[f'RC{channel}_MIN'] <= profile['neutral_pwm']+d*profile['forward_pwm_sign'] <= self.motor.expected[f'RC{channel}_MAX'] for d in s['delta_pwm'])):
                    raise ValueError('Pulse/neutral crosses verified RC or servo limits')
        self.neutral = self.motor.neutral() if self.motor else list(s['dry_run_neutral_pwm'])
        if len(self.neutral) != 2 or any(type(v) is not int or not 1000 <= v <= 2000 for v in self.neutral):
            raise ValueError('Invalid dry-run neutral pair')
        self.forward_signs = [self.motor.config[side]['forward_pwm_sign'] for side in ('left', 'right')] if self.motor else [1, 1]
        # Validate every configured pulse before creating an actuator publisher.
        if any(not 1000 <= n + d*sign <= 2000 for n, sign in zip(self.neutral, self.forward_signs) for d in s['delta_pwm']):
            raise ValueError('Pulse plan exceeds absolute PWM bounds')
        self.lock = threading.RLock()
        self.streams = {key: StreamCheck(s['telemetry_timeout_s'], s['max_gap_s']) for key in ('raw', 'attitude', 'velocity', 'output')}
        self.latest, self.receipts = {}, {}
        self.vehicle = None
        self.enabled = self.rc_enabled = self.estimator_valid = False
        self.parameters_valid = self.vehicle_verified = self.param_pending = False
        self.graph_valid = False
        self.graph_report = {}
        self.fault = None
        self.owns_output = False
        self.release_at = None
        self.last_loop = None
        self.command = None
        self.command_time = None
        self.current_trial, self.phase, self.trial = -1, 'telemetry', None
        self.planned = list(self.neutral)
        self.phase_started = time.monotonic()
        self.finished = threading.Event()
        directory = Path(s['log_directory']).expanduser() if s['log_directory'] else Path.home()/'.ros/blueboat/calibration'
        directory.mkdir(parents=True, exist_ok=True)
        self.run_path = directory/f'calibration-{time.time_ns()}'
        self.recorder = Recorder(self.run_path.with_suffix('.csv'))
        self.events = self.run_path.with_suffix('.events.jsonl').open('x', buffering=1)
        self.run_path.with_suffix('.json').write_text(json.dumps(dict(settings=s, motor_interface=self.motor.config if self.motor else None,
            note='Effective inertial response only; no physical conventions or independent forces inferred automatically.'), indent=2, allow_nan=False)+'\n')
        self.output = self.create_publisher(OverrideRCIn, s['rc_override_topic'], 1) if self.motor else None
        self.diagnostics = self.create_publisher(String, s['diagnostics_topic'], 10)
        self.create_subscription(Imu, s['raw_imu_topic'], self.raw_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, s['attitude_topic'], self.attitude_callback, qos_profile_sensor_data)
        velocity_message = Odometry if s['velocity_type'] == 'odometry' else TwistStamped
        self.create_subscription(velocity_message, s['velocity_topic'], self.velocity_callback, qos_profile_sensor_data)
        self.create_subscription(RCOut, s['output_topic'], self.output_callback, qos_profile_sensor_data)
        self.create_subscription(State, s['state_topic'], self.state_callback, 10)
        self.create_subscription(RCIn, s['rc_input_topic'], self.rc_callback, qos_profile_sensor_data)
        self.create_subscription(EstimatorStatus, s['estimator_topic'], self.estimator_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, s['enable_topic'], self.enable_callback, 1)
        self.vehicle_client = self.create_client(VehicleInfoGet, s['vehicle_info_service'])
        self.pull_client = self.create_client(ParamPull, s['param_pull_service'])
        self.get_client = self.create_client(GetParameters, s['param_get_service'])
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.control_timer = self.create_timer(.02, self.control, clock=steady)
        self.report_timer = self.create_timer(1., self.report, clock=steady)
        self.parameter_timer = self.create_timer(5., self.request_parameters, clock=steady)
        self.watchdog = threading.Thread(target=self.watch, daemon=True)
        self.watchdog.start()
        self.get_logger().info(f'Calibration mode={s["mode"]}, CSV={self.run_path.with_suffix(".csv")}; operator retains arming/mode authority')

    def ros_now(self):
        return self.get_clock().now().nanoseconds/1e9

    def event(self, status, **values):
        payload = json.dumps(dict(status=status, monotonic_s=time.monotonic(), ros_time_s=self.ros_now(), **values), allow_nan=False)
        self.events.write(payload+'\n')

    def accept(self, name, message):
        self.streams[name].accept(stamp(message), self.ros_now(), time.monotonic())
        self.latest[name] = message

    def invalid(self, name, error):
        self.latest.pop(name, None)
        self.streams[name].last_error = str(error)
        self.rest.reset()
        if self.owns_output:
            self.stop(name+': '+str(error))

    def attitude_callback(self, message):
        try:
            q = [message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w]
            if message.header.frame_id != self.settings['body_frame'] or message.orientation_covariance[0] < 0 or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .01:
                raise ValueError('invalid_orientation_or_frame')
            self.accept('attitude', message)
        except Exception as error:
            self.invalid('attitude', error)

    def velocity_callback(self, message):
        try:
            kind = self.settings['velocity_type']
            expected = self.settings['world_frame'] if kind in ('odometry', 'world_twist') else self.settings['body_frame']
            if message.header.frame_id != expected or (kind == 'odometry' and message.child_frame_id != self.settings['body_frame']):
                raise ValueError('invalid_velocity_frame')
            linear = message.twist.twist.linear if kind == 'odometry' else message.twist.linear
            if not np.isfinite(xyz(linear)).all():
                raise ValueError('nonfinite_velocity')
            self.accept('velocity', message)
        except Exception as error:
            self.invalid('velocity', error)

    def body_velocity(self):
        message = self.latest['velocity']
        kind = self.settings['velocity_type']
        values = np.array(xyz(message.twist.twist.linear if kind == 'odometry' else message.twist.linear))
        if kind == 'world_twist':
            from scipy.spatial.transform import Rotation
            q = self.latest['attitude'].orientation
            values = Rotation.from_quat([q.x, q.y, q.z, q.w]).inv().apply(values)
        return values

    def output_callback(self, message):
        try:
            if any(x not in (0, 65535) and not 800 <= x <= 2200 for x in message.channels):
                raise ValueError('invalid_output_pwm')
            self.accept('output', message)
        except Exception as error:
            self.invalid('output', error)

    def raw_callback(self, message):
        error = None
        try:
            if (message.header.frame_id != self.settings['body_frame'] or message.linear_acceleration_covariance[0] < 0
                    or message.angular_velocity_covariance[0] < 0 or not np.isfinite(xyz(message.linear_acceleration)+xyz(message.angular_velocity)).all()):
                raise ValueError('invalid_raw_imu_or_frame')
            # Raw orientation is intentionally ignored: MAVROS declares it unavailable.
            self.accept('raw', message)
            self.observe_rest(message)
        except Exception as exception:
            error = str(exception)
            self.invalid('raw', exception)
        try:
            self.record_sample(message, error)
        except Exception as exception:
            self.stop('recording_failed: '+str(exception))
            self.get_logger().error('CSV recording failed; calibration stopped')

    def observe_rest(self, raw):
        stationary = False
        if self.phase == 'baseline' and self.plan.started is not None:
            attitude, velocity = (self.latest.get(k) for k in ('attitude', 'velocity'))
            if (attitude and velocity
                    and max(abs(stamp(raw)-stamp(attitude)), abs(stamp(raw)-stamp(velocity))) <= self.settings['max_skew_s']):
                from scipy.spatial.transform import Rotation
                q = attitude.orientation
                acceleration = np.array(xyz(raw.linear_acceleration))
                if self.settings['accelerometer_semantics'] == 'specific_force':
                    acceleration += Rotation.from_quat([q.x,q.y,q.z,q.w]).inv().apply([0,0,-9.80665])
                stationary = (np.linalg.norm(self.body_velocity()) <= .08
                    and np.linalg.norm(xyz(raw.angular_velocity)) <= .10 and np.linalg.norm(acceleration) <= .6)
                if self.motor:
                    stationary &= self.command == self.neutral
                    if self.settings['require_output_feedback']:
                        output = self.latest.get('output')
                        stationary &= bool(output and 0 <= self.ros_now()-stamp(output) <= self.settings['telemetry_timeout_s']
                            and all(len(output.channels) >= self.motor.config[side]['servo_output']
                                and abs(output.channels[self.motor.config[side]['servo_output']-1]-self.motor.config[side]['servo_neutral_pwm'])
                                    <= self.settings['output_neutral_tolerance_pwm'] for side in ('left','right')))
        self.rest.observe(time.monotonic(), stationary)

    def record_sample(self, raw, error):
        attitude, velocity, output = (self.latest.get(k) for k in ('attitude', 'velocity', 'output'))
        reason = error or self.fault
        if not attitude or not velocity:
            reason = reason or 'waiting_for_attitude_velocity'
        elif max(abs(stamp(raw)-stamp(attitude)), abs(stamp(raw)-stamp(velocity))) > self.settings['max_skew_s']:
            reason = reason or 'timestamp_skew'
        row = dict(receipt_monotonic_s=time.monotonic(), ros_time_s=self.ros_now(), imu_stamp_s=stamp(raw),
            imu_frame=raw.header.frame_id, trial=self.current_trial, side=self.trial.side if self.trial else '',
            delta_pwm=self.trial.delta_pwm if self.trial else 0, phase=self.phase, mode=self.settings['mode'],
            valid=int(reason is None), reason=reason or '', planned_left_pwm=self.planned[0], planned_right_pwm=self.planned[1])
        if self.current_trial >= 0 and self.plan.started is not None:
            row['trial_end_monotonic_s'] = self.plan.started+(self.current_trial+1)*self.plan.period
        for prefix, vector in (('a', raw.linear_acceleration), ('w', raw.angular_velocity)):
            row.update({prefix+axis: value for axis, value in zip('xyz', xyz(vector))})
        if attitude:
            row.update(attitude_stamp_s=stamp(attitude), attitude_frame=attitude.header.frame_id)
            row.update({'q'+axis: getattr(attitude.orientation, axis) for axis in 'xyzw'})
        if velocity:
            row.update(velocity_stamp_s=stamp(velocity), velocity_frame=self.settings['body_frame'])
            if self.settings['velocity_type'] != 'world_twist' or attitude:
                row.update({'v'+axis: float(value) for axis,value in zip('xyz', self.body_velocity())})
        if self.command is not None:
            row.update(command_left_pwm=self.command[0], command_right_pwm=self.command[1], command_monotonic_s=self.command_time)
        if output:
            row['output_stamp_s'] = stamp(output)
            row['output_channels_json'] = json.dumps(list(output.channels))
            if self.motor:
                for side in ('left', 'right'):
                    index = self.motor.config[side]['servo_output']-1
                    if len(output.channels) > index:
                        row['output_'+side+'_pwm'] = output.channels[index]
        self.recorder.write(row)

    def state_callback(self, message):
        self.vehicle = message
        self.receipts['state'] = time.monotonic()
        if self.owns_output and (not message.connected or not message.armed or message.mode != 'MANUAL' or message.system_status not in (3, 4)):
            self.stop('manual_takeover_or_disarmed', immediate=True)

    def rc_callback(self, message):
        if self.motor and self.motor.enable_channel is not None:
            channel = self.motor.enable_channel
            self.rc_enabled = len(message.channels) >= channel and 1800 <= message.channels[channel-1] <= 2200
            self.receipts['rc'] = time.monotonic()
            if self.owns_output and not self.rc_enabled:
                self.stop('manual_takeover', immediate=True)

    def enable_callback(self, message):
        self.enabled = bool(message.data)
        self.receipts['enable'] = time.monotonic()
        if self.owns_output and not self.enabled:
            self.stop('enable_released', immediate=True)

    def estimator_callback(self, message):
        self.estimator_valid = bool(message.attitude_status_flag and message.velocity_horiz_status_flag
            and message.velocity_vert_status_flag and not message.const_pos_mode_status_flag
            and not message.gps_glitch_status_flag and not message.accel_error_status_flag)
        self.receipts['estimator'] = time.monotonic()
        if self.owns_output and not self.estimator_valid:
            self.stop('estimator_invalid')

    def telemetry_reason(self, now):
        s = self.settings
        for name in ('raw', 'attitude', 'velocity'):
            check = self.streams[name].report(now, self.ros_now())
            minimum = s['min_'+name+'_rate_hz']
            if not check['fresh'] or check['span_s'] < 1. or check['rate_hz'] < minimum or check['max_gap_s'] > s['max_gap_s']:
                return name+'_timestamp_rate_or_gap'
        times = [stamp(self.latest[k]) for k in ('raw', 'attitude', 'velocity')]
        if max(times)-min(times) > s['max_skew_s']:
            return 'timestamp_skew'
        if np.linalg.norm(self.body_velocity()) > s['max_live_speed_m_s']:
            return 'live_speed_limit'
        if np.linalg.norm(xyz(self.latest['raw'].angular_velocity)) > s['max_live_angular_rate_rad_s']:
            return 'live_angular_rate_limit'
        from scipy.spatial.transform import Rotation
        q = self.latest['attitude'].orientation
        if max(abs(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[:2])) > s['max_live_tilt_rad']:
            return 'live_tilt_limit'
        return None

    def permission(self, now):
        if self.fault:
            return self.fault
        reason = self.telemetry_reason(now)
        if reason:
            return reason
        if not self.graph_valid or not 0 <= now-self.receipts.get('graph', -np.inf) <= 2.5:
            return 'topic_graph_unverified_or_competing_publisher'
        if not self.motor:
            return None
        vehicle = self.vehicle
        if not vehicle or not vehicle.connected or vehicle.system_status not in (3, 4) or not 0 <= now-self.receipts.get('state', -np.inf) <= 2.:
            return 'autopilot_disconnected_or_failsafe'
        if not vehicle.armed or vehicle.mode != 'MANUAL':
            return 'manual_takeover_or_disarmed'
        checks = [('enable', self.enabled)]
        if self.motor.authority == 'rc_receiver':
            checks.append(('rc', self.rc_enabled))
        for key, enabled in checks:
            if not enabled or not 0 <= now-self.receipts.get(key, -np.inf) <= .5:
                return 'enable_released' if key == 'enable' else 'manual_takeover'
        if not self.vehicle_verified or not self.parameters_valid or not 0 <= now-self.receipts.get('parameters', -np.inf) <= 15.:
            return 'vehicle_or_parameters_unverified'
        if not self.estimator_valid or not 0 <= now-self.receipts.get('estimator', -np.inf) <= 2.:
            return 'estimator_invalid_or_stale'
        if self.settings['require_output_feedback']:
            report = self.streams['output'].report(now, self.ros_now())
            if not report['fresh'] or report['rate_hz'] < self.settings['min_output_rate_hz']:
                return 'output_feedback_missing_or_stale'
            output = self.latest.get('output')
            for side in ('left', 'right'):
                profile = self.motor.config[side]
                index = profile['servo_output']-1
                if output is None or len(output.channels) <= index:
                    return 'motor_output_channel_missing'
                if not 800 <= output.channels[index] <= 2200:
                    return 'motor_output_invalid'
                # Check neutral before ownership, and after settling in baseline/recovery.
                neutral_expected = (not self.owns_output or (self.phase != 'pulse'
                    and now-self.phase_started > self.settings['output_response_grace_s']))
                if self.phase == 'pulse' and self.trial and self.trial.side not in (side, 'symmetric'):
                    neutral_expected = now-self.phase_started > self.settings['output_response_grace_s']
                if neutral_expected and abs(output.channels[index]-profile['servo_neutral_pwm']) > self.settings['output_neutral_tolerance_pwm']:
                    return 'actual_output_not_neutral_'+side
                if (self.phase == 'pulse' and self.trial and self.trial.side in (side, 'symmetric')
                        and now-self.phase_started > self.settings['output_response_grace_s']):
                    direction = 1 if self.trial.delta_pwm > 0 else -1
                    response = (output.channels[index]-profile['servo_neutral_pwm'])*profile['servo_forward_pwm_sign']*direction
                    if response < self.settings['output_min_response_pwm']:
                        return 'actual_output_wrong_or_absent_response_'+side
        return None

    def publish_pwm(self, pwm):
        if self.output is not None:
            self.output.publish(OverrideRCIn(channels=rc_override(self.motor.channels, pwm)))
        if self.output is not None:
            self.command = None if pwm is None else list(pwm)
            self.command_time = time.monotonic()

    def stop(self, reason, immediate=False):
        with self.lock:
            if self.fault is None:
                self.fault = reason
            if self.owns_output:
                if immediate:
                    self.publish_pwm(None)
                    self.owns_output = False
                    self.release_at = None
                elif self.release_at is None:
                    self.publish_pwm(self.neutral)
                    self.release_at = time.monotonic()+.1

    def control(self):
        if self.settings['mode'] == 'telemetry' or self.fault:
            return
        try:
            transition = None
            with self.lock:
                now = time.monotonic()
                reason = self.permission(now)
                if reason:
                    if self.plan.started is not None:
                        self.stop(reason, immediate=reason in ('manual_takeover', 'manual_takeover_or_disarmed', 'enable_released'))
                    return
                previous = self.current_trial, self.phase
                self.current_trial, self.phase, self.planned, self.trial = self.plan.step(now, self.neutral, self.forward_signs)
                if self.phase == 'pulse' and previous != (self.current_trial, self.phase):
                    if not self.rest.ready(now):
                        self.stop('rest_baseline_not_verified')
                        return
                if previous != (self.current_trial, self.phase):
                    self.phase_started = now
                    if self.phase == 'baseline':
                        self.rest.reset()
                    transition = dict(trial=self.current_trial, planned_pwm=self.planned,
                        rest_verified=self.phase=='pulse' and self.rest.ready(now))
                if self.phase == 'complete':
                    self.stop('complete', immediate=True)
                elif self.motor:
                    self.owns_output = True
                    self.publish_pwm(self.planned)
                self.last_loop = time.monotonic()
            # Disk I/O must not hold the watchdog/output lock.
            if transition is not None:
                self.event(self.phase, **transition)
        except Exception as error:
            self.stop('control_or_logging_fault: '+str(error))

    def watch(self):
        while not self.finished.wait(.02):
            with self.lock:
                now = time.monotonic()
                if self.owns_output and self.last_loop is not None and now-self.last_loop > self.settings['watchdog_timeout_s']:
                    self.stop('watchdog_timeout')
                if self.release_at is not None:
                    if now >= self.release_at:
                        self.publish_pwm(None)
                        self.owns_output = False
                        self.release_at = None
                    else:
                        self.publish_pwm(self.neutral)

    def inspect_graph(self):
        s = self.settings
        expected = dict(raw=(s['raw_imu_topic'], 'sensor_msgs/msg/Imu'), attitude=(s['attitude_topic'], 'sensor_msgs/msg/Imu'),
            velocity=(s['velocity_topic'], 'nav_msgs/msg/Odometry' if s['velocity_type'] == 'odometry' else 'geometry_msgs/msg/TwistStamped'),
            output=(s['output_topic'], 'mavros_msgs/msg/RCOut'))
        valid, report = True, {}
        for name, (topic, typename) in expected.items():
            endpoints = self.get_publishers_info_by_topic(topic)
            report[name] = dict(topic=topic, expected_type=typename, publishers=[dict(node=e.node_name,
                namespace=e.node_namespace, type=e.topic_type, qos=str(e.qos_profile)) for e in endpoints])
            if name != 'output' or (self.motor and s['require_output_feedback']):
                valid &= len(endpoints) == 1 and all(e.topic_type == typename for e in endpoints)
        override = self.get_publishers_info_by_topic(s['rc_override_topic'])
        report['override_publishers'] = [dict(node=e.node_name, namespace=e.node_namespace) for e in override]
        if self.motor:
            valid &= len(override) == 1 and override[0].node_name == self.get_name() and override[0].node_namespace == self.get_namespace()
        self.graph_valid, self.graph_report = bool(valid), report
        self.receipts['graph'] = time.monotonic()

    def report(self):
        try:
            self.inspect_graph()
            now = time.monotonic()
            reason = self.permission(now)
            if self.owns_output and reason:
                self.stop(reason)
            data = dict(mode=self.settings['mode'], status=reason or 'ready', trial=self.current_trial, phase=self.phase,
                graph=self.graph_report, streams={k:v.report(now, self.ros_now()) for k,v in self.streams.items()},
                frames={k:m.header.frame_id for k,m in self.latest.items()}, csv=str(self.run_path.with_suffix('.csv')))
            data['vehicle'] = {k:getattr(self.vehicle,k) for k in ('connected','armed','mode','system_status')} if self.vehicle else None
            data['gates'] = dict(software_enable=self.enabled, receiver_enable=self.rc_enabled,
                                 identity_verified=self.vehicle_verified, parameters_verified=self.parameters_valid,
                                 estimator_valid=self.estimator_valid, owns_output=self.owns_output)
            data['rest_elapsed_s'] = max(0., now-self.rest.since) if self.rest.since is not None else 0.
            data['rest_verified'] = self.rest.ready(now)
            times = [stamp(self.latest[k]) for k in ('raw', 'attitude', 'velocity') if k in self.latest]
            data['source_skew_s'] = max(times)-min(times) if len(times) == 3 else None
            self.diagnostics.publish(String(data=json.dumps(data, allow_nan=False)))
            self.event('diagnostics', diagnostics=data)
        except Exception as error:
            self.stop('diagnostics_or_logging_fault: '+str(error))

    def request_parameters(self):
        if not self.motor or self.param_pending or not all(c.service_is_ready() for c in (self.vehicle_client, self.pull_client, self.get_client)):
            return
        self.param_pending = True
        self.vehicle_client.call_async(VehicleInfoGet.Request(sysid=0, compid=0, get_all=False)).add_done_callback(self.vehicle_received)

    def verification_failed(self, error):
        self.param_pending = self.parameters_valid = self.vehicle_verified = False
        if self.owns_output:
            self.stop('vehicle_or_parameter_verification_failed: '+str(error))

    def vehicle_received(self, future):
        try:
            response = future.result()
            self.vehicle_verified = response.success and len(response.vehicles) == 1 and self.motor.verify_vehicle(response.vehicles[0])
            if not self.vehicle_verified:
                raise ValueError('identity_mismatch')
            self.pull_client.call_async(ParamPull.Request(force_pull=True)).add_done_callback(self.parameters_pulled)
        except Exception as error:
            self.verification_failed(error)

    def parameters_pulled(self, future):
        try:
            if not future.result().success:
                raise ValueError('parameter_pull_failed')
            self.get_client.call_async(GetParameters.Request(names=list(self.motor.expected))).add_done_callback(self.parameters_received)
        except Exception as error:
            self.verification_failed(error)

    def parameters_received(self, future):
        try:
            actual = {}
            for name,value in zip(self.motor.expected, future.result().values):
                if value.type in (2, 3):
                    actual[name] = value.integer_value if value.type == 2 else value.double_value
            if not self.motor.verify_parameters(actual):
                raise ValueError('parameter_mismatch')
            self.parameters_valid = True
            self.receipts['parameters'] = time.monotonic()
            self.param_pending = False
        except Exception as error:
            self.verification_failed(error)

    def close(self):
        self.control_timer.cancel()
        self.report_timer.cancel()
        self.parameter_timer.cancel()
        with self.lock:
            if self.owns_output:
                self.publish_pwm(self.neutral)
                self.stop('shutdown', immediate=True)
        self.finished.set()
        self.watchdog.join(timeout=1.)
        self.recorder.close()
        self.events.close()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    finished = threading.Event()
    handlers = {sig:signal.signal(sig, lambda number, frame: finished.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = CalibrationNode()
        while rclpy.ok() and not finished.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if node is not None:
            node.close()
        for sig,handler in handlers.items():
            signal.signal(sig, handler)
        if rclpy.ok():
            rclpy.shutdown()
