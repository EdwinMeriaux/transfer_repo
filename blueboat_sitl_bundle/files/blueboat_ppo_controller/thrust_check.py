#!/usr/bin/env python3
"""Simple thrust check: does the boat respond like the boat the u990 policy was trained on?

Each step holds a fixed left/right thrust (newtons, Blue Robotics T200 data) for a
few seconds and compares the boat's steady speed / turn rate with the policy's
training model (its README table). One figure per step, e.g. "speed 1.31 vs 1.41
expected (-7%)", plus an overall thrust scale for forward and reverse.

  thrust_check list                    show the steps and what is expected
  thrust_check run [STEP ...]          run them (default: all, in the listed order)

Thrust is set on the ESC's PWM, which RCOut reports: the tool starts from the RC
input that should give it and trims the input until RCOut shows the PWM the T200
table needs. ArduPilot's mixing (MOT_THST_ASYM, MOT_STR_THR_MIX, trims) is thereby
taken into account without modelling it.

Safety (same rules as the calibration tools): never arms or changes mode; needs
MANUAL + armed, one /mavros/rc/override publisher, an enable heartbeat on
/blueboat/thrust_check/enable (>=2 Hz) and fresh odometry/IMU. Disarm, mode change
or enable loss releases immediately; every other stop writes neutral for 0.3 s and
then releases. Speed and turn-rate caps end a step. RC_OVERRIDE_TIME must be
>0 and <=0.5 s.
"""
import argparse
import csv
import json
import math
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

SIDES = ('left', 'right')
DATA = Path(__file__).resolve().parent / 'data' / 't200_thrust_N.csv'
DEFAULT_CONFIG = Path('~/.config/blueboat/alberto').expanduser()
PACKAGE_PROFILE = Path(__file__).resolve().parent / 'profiles' / 'alberto' / 'motor_interface.json'
ESC_NEUTRAL_PWM = 1500
RELEASE_NEUTRAL_S = .3
PUBLISH_PERIOD_S = .05
ENABLE_TOPIC = '/blueboat/thrust_check/enable'
ENABLE_TIMEOUT_S = .5
TELEMETRY_TIMEOUT_S = .5
TRIM_GAIN = .6
MAX_TRIM_PWM = 150

# The u990 training model, open loop from rest (its README, "our model" column):
# (left N, right N) -> (surge m/s, yaw rate rad/s, positive = turning left).
TRAINING_MODEL = {
    (10., 10.): (1.406, 0.), (25., 25.): (2.409, 0.), (-15., -15.): (-1.793, 0.),
    (0., 10.): (.910, .048), (0., 25.): (1.607, .104), (-7.5, 15.): (.746, .200),
    (-15., 25.): (.874, .458), (-10., 10.): (-.021, .590), (-15., 15.): (-.035, .830),
    (-15., 0.): (-1.177, .066),
}


def fit_drag():
    """Total surge drag D(v) = a|v| + b v^2 through the model's straight-line rows."""
    rows = [(abs(v), abs(l + r)) for (l, r), (v, w) in TRAINING_MODEL.items() if l == r]
    a, b = np.linalg.lstsq(np.array([[v, v * v] for v, _ in rows]), np.array([f for _, f in rows]), rcond=None)[0]
    return float(a), float(b)


DRAG_A, DRAG_B = fit_drag()


def drag(v):
    return DRAG_A * abs(v) + DRAG_B * v * v


def speed_for_total_thrust(force):
    """Steady straight-line speed for a total thrust, from the fitted training-model drag."""
    root = (-DRAG_A + math.sqrt(DRAG_A ** 2 + 4 * DRAG_B * abs(force))) / (2 * DRAG_B)
    return math.copysign(root, force)


# Spin rows: yaw rate grows like force^k between the two table points.
SPIN_EXPONENT = math.log(TRAINING_MODEL[(-15., 15.)][1] / TRAINING_MODEL[(-10., 10.)][1]) / math.log(1.5)


def expected(left, right):
    """(surge, yaw, source) for a thrust pair; interpolated straight runs are labelled."""
    key = (float(left), float(right))
    if key in TRAINING_MODEL:
        return (*TRAINING_MODEL[key], 'training model table')
    if (float(right), float(left)) in TRAINING_MODEL:   # mirror image: same speed, opposite turn
        surge, yaw = TRAINING_MODEL[(float(right), float(left))]
        return surge, -yaw, 'training model table (mirrored)'
    if left == right:
        return speed_for_total_thrust(left + right), 0., 'interpolated from training-model drag'
    raise ValueError(f'no expectation for {left}/{right} N')


STEPS = {
    # name: (left N, right N, seconds, what it shows)
    'spin10': (-10., 10., 15., 'spin in place: both motors, opposite directions'),
    'spin15': (-15., 15., 15., 'spin in place at 15 N'),
    'left10': (10., 0., 15., 'left motor alone forward (gentle right curve)'),
    'right10': (0., 10., 15., 'right motor alone forward (gentle left curve)'),
    'fwd10': (10., 10., 15., 'straight ahead, 10 N per motor'),
    'rev15': (-15., -15., 15., 'straight back, 15 N per motor'),
    'fwd15': (15., 15., 15., 'straight ahead, 15 N per motor'),
}


# ----------------------------------------------------------------------------- T200 table
class T200:
    """Bollard thrust vs ESC PWM at a given voltage (linear between the published sheets)."""
    def __init__(self, voltage, path=DATA):
        lines = [line for line in Path(path).read_text().splitlines() if line and not line.startswith('#')]
        rows = list(csv.DictReader(lines))
        table = {name: np.array([float(r[name]) for r in rows]) for name in rows[0]}
        volts = sorted(int(n.split('_')[2][:-1]) for n in table if n.startswith('force_N_'))
        if not volts[0] <= voltage <= volts[-1]:
            raise ValueError(f'battery voltage {voltage} V outside the published {volts[0]}-{volts[-1]} V data')
        self.voltage = voltage
        self.pwm = table['pwm_us']
        lo = max(v for v in volts if v <= voltage)
        hi = min(v for v in volts if v >= voltage)
        w = 0. if hi == lo else (voltage - lo) / (hi - lo)
        self.force = (1 - w) * table[f'force_N_{lo}V'] + w * table[f'force_N_{hi}V']
        fwd, rev = self.pwm > ESC_NEUTRAL_PWM, self.pwm < ESC_NEUTRAL_PWM
        self._fwd = (self.force[fwd], self.pwm[fwd])
        self._rev = (self.force[rev], self.pwm[rev])

    def offset(self, force):
        """PWM offset from ESC neutral (always >=0) that gives |force| in its direction."""
        if force == 0:
            return 0.
        forces, pwms = self._fwd if force > 0 else self._rev
        if not forces.min() <= force <= forces.max():
            raise ValueError(f'{force} N is beyond the T200 table at {self.voltage} V')
        return abs(float(np.interp(force, forces, pwms)) - ESC_NEUTRAL_PWM)

    def force_at(self, pwm_offset, direction):
        """Thrust (N, signed) for a PWM offset from neutral in a thrust direction."""
        pwm = ESC_NEUTRAL_PWM + direction * abs(pwm_offset)
        return float(np.interp(pwm, self.pwm, self.force))


# ----------------------------------------------------------------------------- mapping
class Interface:
    """Channels, neutrals and signs from motor_interface.json."""
    def __init__(self, profile, source=''):
        self.source = str(source)
        self.channels = [profile[s]['rc_channel'] for s in SIDES]
        self.servos = [profile[s]['servo_output'] for s in SIDES]
        self.neutral = [profile[s]['neutral_pwm'] for s in SIDES]
        self.sign = [profile[s].get('forward_pwm_sign') or 1 for s in SIDES]
        # Measured ESC (servo) forward direction; learned live when the profile does not have it.
        self.servo_sign = [profile[s].get('servo_forward_pwm_sign') for s in SIDES]
        expected_parameters = profile.get('expected_parameters', {})
        self.limits = [(expected_parameters.get(f'RC{c}_MIN', 1100), expected_parameters.get(f'RC{c}_MAX', 1900))
                       for c in self.channels]
        if (len(set(self.channels)) != 2 or len(set(self.servos)) != 2
                or any(not 1000 <= n <= 2000 for n in self.neutral) or any(s not in (-1, 1) for s in self.sign)
                or any(s not in (None, -1, 1) for s in self.servo_sign)):
            raise ValueError('motor_interface.json needs two RC channels/servo outputs, neutrals and +1/-1 signs')

    def set_limits(self, parameters):
        self.limits = [(parameters.get(f'RC{c}_MIN', lo), parameters.get(f'RC{c}_MAX', hi))
                       for c, (lo, hi) in zip(self.channels, self.limits)]

    def clamp(self, i, pwm):
        lo, hi = self.limits[i]
        return int(round(min(max(pwm, max(lo, 1000)), min(hi, 2000))))


def load_interface(path=None):
    for candidate in ([Path(path).expanduser()] if path else [DEFAULT_CONFIG / 'motor_interface.json', PACKAGE_PROFILE]):
        if candidate.is_file():
            return Interface(json.loads(candidate.read_text()), candidate)
    raise FileNotFoundError('no motor_interface.json found')


def parameter_problem(parameters, channels):
    timeout = parameters.get('RC_OVERRIDE_TIME')
    if timeout is None or not math.isfinite(timeout):
        return 'RC_OVERRIDE_TIME not readable from MAVROS'
    if not 0 < timeout <= .5:
        return (f'RC_OVERRIDE_TIME={timeout:g}: if this tool dies, thrust keeps running that long. '
                'Set it to 0.3 in QGroundControl (>0 and <=0.5).')
    options = parameters.get('RC_OPTIONS')
    if options is None:
        return 'RC_OPTIONS not readable from MAVROS'
    if int(options) & 2:
        return 'RC_OPTIONS bit 1 set: the autopilot ignores MAVLink RC overrides'
    missing = [f'RC{c}_{k}' for c in channels for k in ('MIN', 'MAX') if f'RC{c}_{k}' not in parameters]
    return 'not readable from MAVROS: ' + ', '.join(missing) if missing else None


class ServoTrim:
    """Adjusts one side's RC input until RCOut shows the ESC PWM the T200 table needs."""
    def __init__(self, interface, i, force, offset, servo_neutral):
        self.interface, self.i, self.force, self.target = interface, i, force, offset
        self.direction = 0 if force == 0 else (1 if force > 0 else -1)
        self.servo_neutral = servo_neutral
        self.servo_sign = interface.servo_sign[i]
        self.adjust = 0.
        self.achieved = None
        self.problem = None

    def command(self):
        base = self.interface.neutral[self.i] + self.interface.sign[self.i] * self.direction * self.target
        return self.interface.clamp(self.i, base + self.interface.sign[self.i] * self.direction * self.adjust)

    def update(self, servo_pwm, elapsed):
        """Feed one new RCOut sample (PWM on this side's servo output)."""
        if self.direction == 0 or servo_pwm is None:
            return
        moved = servo_pwm - (self.servo_neutral if self.servo_neutral is not None else ESC_NEUTRAL_PWM)
        if self.servo_sign is None:
            if abs(moved) < 20:
                return
            self.servo_sign = 1 if moved * self.direction > 0 else -1     # learned: which way the ESC PWM moves
        self.achieved = self.servo_sign * self.direction * (servo_pwm - ESC_NEUTRAL_PWM)
        if self.achieved < -20 and elapsed > 1.:
            self.problem = f'{SIDES[self.i]} ESC PWM moved the wrong way ({servo_pwm})'
            return
        if elapsed > .4:
            self.adjust = float(np.clip(self.adjust + TRIM_GAIN * (self.target - self.achieved), -MAX_TRIM_PWM, MAX_TRIM_PWM))


# ----------------------------------------------------------------------------- results
def summarize(name, rows, left, right, settle_window_s=5.):
    """Mean surge/yaw over the last settle_window_s of a completed step, vs the training model."""
    surge_expected, yaw_expected, source = expected(left, right)
    t = np.array([r['t'] for r in rows])
    window = t >= t[-1] - settle_window_s
    surge = np.array([r['surge'] for r in rows])[window]
    yaw = np.array([r['yaw_rate'] for r in rows])[window]
    slope = float(np.polyfit(t[window] - t[window][0], surge, 1)[0]) if window.sum() > 3 else 0.
    out = dict(step=name, left_N=left, right_N=right, expected_source=source,
               surge_m_s=float(surge.mean()), surge_expected_m_s=surge_expected,
               yaw_rate_rad_s=float(yaw.mean()), yaw_rate_expected_rad_s=yaw_expected,
               settled=abs(slope) < .03, surge_trend_m_s2=slope)
    if left == -right:                       # spin: compare turn rate
        ratio = out['yaw_rate_rad_s'] / yaw_expected
        out['difference_pct'] = 100 * (ratio - 1)
        out['thrust_scale'] = max(ratio, 0.) ** (1 / SPIN_EXPONENT)
        out['direction'] = 'spin'
    else:                                     # straight or one motor: compare speed through the drag model
        ratio = out['surge_m_s'] / surge_expected
        out['difference_pct'] = 100 * (ratio - 1)
        out['thrust_scale'] = drag(out['surge_m_s']) / drag(surge_expected) if ratio > 0 else 0.
        out['direction'] = 'forward' if left + right > 0 else 'reverse'
    out['esc_pwm_offset_reached'] = [rows[-1]['esc_offset_left'], rows[-1]['esc_offset_right']]
    out['input_pwm'] = [rows[-1]['input_left'], rows[-1]['input_right']]
    return out


def verdict(results):
    """Overall scale per direction and a plain recommendation."""
    lines, scales = [], {}
    for r in results:
        if r.get('stopped'):
            lines.append(f"{r['step']:8s} STOPPED: {r['stopped']}")
            continue
        kind = 'turn rate' if r['direction'] == 'spin' else 'speed'
        measured = r['yaw_rate_rad_s'] if r['direction'] == 'spin' else r['surge_m_s']
        target = r['yaw_rate_expected_rad_s'] if r['direction'] == 'spin' else r['surge_expected_m_s']
        note = '' if r['settled'] else '  (still changing: lengthen the step)'
        lines.append(f"{r['step']:8s} {kind} {measured:+.2f} vs {target:+.2f} expected ({r['difference_pct']:+.0f}%)"
                     f"  thrust x{r['thrust_scale']:.2f}{note}")
        scales.setdefault(r['direction'], []).append(r['thrust_scale'])
    summary = {k: float(np.median(v)) for k, v in scales.items()}
    one_motor = {r['step']: r['surge_m_s'] for r in results if r['step'] in ('left10', 'right10') and not r.get('stopped')}
    if len(one_motor) == 2 and min(one_motor.values()) > 0:
        summary['left_right_speed_ratio'] = one_motor['left10'] / one_motor['right10']
    worst = max((abs(r['difference_pct']) for r in results if not r.get('stopped')), default=None)
    if worst is None:
        advice = 'no completed step'
    elif worst <= 15:
        advice = 'GOOD: every step within 15% of the training model'
    else:
        advice = 'ADJUST: some steps differ by more than 15%; use the thrust scale per direction'
    return lines, summary, advice


# ----------------------------------------------------------------------------- ROS side
class Stopped(Exception):
    pass


class Rig:
    def __init__(self, interface):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from mavros_msgs.msg import OverrideRCIn, RCOut, State
        from mavros_msgs.srv import ParamPull
        from nav_msgs.msg import Odometry
        from rcl_interfaces.srv import GetParameters
        from sensor_msgs.msg import BatteryState, Imu
        from std_msgs.msg import Bool
        from .mavros_interface import rc_override
        self.rclpy, self.OverrideRCIn, self.rc_override = rclpy, OverrideRCIn, rc_override
        self.GetParameters, self.ParamPull = GetParameters, ParamPull
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('blueboat_thrust_check')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.interface = interface
        self.state = self.rcout = self.battery = self.odom = self.imu = None
        self.t = dict(state=-math.inf, rcout=-math.inf, odom=-math.inf, imu=-math.inf, enable=-math.inf)
        self.enabled = False
        sensor = qos_profile_sensor_data
        self.node.create_subscription(State, '/mavros/state', lambda m: self._set('state', m), 10)
        self.node.create_subscription(RCOut, '/mavros/rc/out', lambda m: self._set('rcout', m), sensor)
        self.node.create_subscription(Odometry, '/mavros/local_position/odom', lambda m: self._set('odom', m), sensor)
        self.node.create_subscription(Imu, '/mavros/imu/data', lambda m: self._set('imu', m), sensor)
        self.node.create_subscription(BatteryState, '/mavros/battery', lambda m: setattr(self, 'battery', m), sensor)
        self.node.create_subscription(Bool, ENABLE_TOPIC, self._on_enable, 1)
        self.pub = self.node.create_publisher(OverrideRCIn, '/mavros/rc/override', 1)

    def _set(self, name, message):
        setattr(self, name, message)
        self.t[name] = time.monotonic()

    def _on_enable(self, message):
        self.enabled = bool(message.data)
        self.t['enable'] = time.monotonic()
        if not self.enabled:
            self.publish(None)              # immediate hand-back

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while True:
            self.executor.spin_once(timeout_sec=max(0., min(.01, end - time.monotonic())))
            if time.monotonic() >= end:
                return

    def voltage(self):
        v = getattr(self.battery, 'voltage', float('nan'))
        return float(v) if v is not None and math.isfinite(v) and v > 0 else None

    def servo_outputs(self):
        if self.rcout is None:
            return [None, None]
        channels = list(self.rcout.channels)
        return [channels[s - 1] if len(channels) >= s else None for s in self.interface.servos]

    def motion(self):
        """(surge m/s, yaw rate rad/s), both FLU; None if stale."""
        now = time.monotonic()
        if self.odom is None or self.imu is None or now - self.t['odom'] > TELEMETRY_TIMEOUT_S or now - self.t['imu'] > TELEMETRY_TIMEOUT_S:
            return None
        return self.odom.twist.twist.linear.x, self.imu.angular_velocity.z

    def read_parameters(self, timeout_s=20.):
        names = ['RC_OVERRIDE_TIME', 'RC_OPTIONS'] + [f'RC{c}_{k}' for c in self.interface.channels for k in ('MIN', 'MAX')]
        get = self.node.create_client(self.GetParameters, '/mavros/param/get_parameters')
        pull = self.node.create_client(self.ParamPull, '/mavros/param/pull')
        deadline, pulled = time.monotonic() + timeout_s, False
        while time.monotonic() < deadline:
            if get.wait_for_service(timeout_sec=1.):
                future = get.call_async(self.GetParameters.Request(names=names))
                end = time.monotonic() + 5.
                while not future.done() and time.monotonic() < end:
                    self.spin(.05)
                if future.done() and future.result() is not None:
                    values = {n: float(v.integer_value) if v.type == 2 else v.double_value
                              for n, v in zip(names, future.result().values) if v.type in (2, 3)}
                    if all(n in values for n in names):
                        return values
                    if not pulled and pull.wait_for_service(timeout_sec=1.):
                        pull.call_async(self.ParamPull.Request(force_pull=False))   # never a forced pull
                        pulled = True
            self.spin(1.)
        return {}

    def blocker(self):
        now = time.monotonic()
        if self.state is None or now - self.t['state'] > 3:
            return 'no /mavros/state for 3 s'
        if not self.state.connected:
            return 'MAVROS is not connected to the autopilot'
        if not self.state.armed:
            return 'boat is disarmed'
        if self.state.mode != 'MANUAL':
            return f'mode is {self.state.mode}, not MANUAL'
        if not self.enabled or now - self.t['enable'] > ENABLE_TIMEOUT_S:
            return f'no enable heartbeat on {ENABLE_TOPIC} (publish with --rate 10)'
        if self.node.count_publishers('/mavros/rc/override') > 1:
            return 'another node publishes /mavros/rc/override'
        if self.motion() is None:
            return 'odometry/IMU stale (is there a GPS fix?)'
        return None

    def publish(self, pwm):
        self.pub.publish(self.OverrideRCIn(channels=self.rc_override(self.interface.channels, pwm)))

    def release(self):
        end = time.monotonic() + RELEASE_NEUTRAL_S
        while time.monotonic() < end:
            self.publish(self.interface.neutral)
            self.spin(PUBLISH_PERIOD_S)
        for _ in range(3):
            self.publish(None)
            self.spin(PUBLISH_PERIOD_S)

    def close(self):
        try:
            if self.rclpy.ok():
                self.release()
        finally:
            self.executor.shutdown()
            self.node.destroy_node()
            self.rclpy.try_shutdown()


IMMEDIATE = ('boat is disarmed', 'mode is', 'no enable heartbeat')


def run_step(rig, table, name, left, right, seconds, max_speed, max_yaw, log):
    """Hold one thrust pair. Returns the step summary, or a dict with 'stopped'."""
    interface = rig.interface
    servo_neutral = rig.servo_outputs()
    trims = [ServoTrim(interface, i, f, table.offset(f), servo_neutral[i]) for i, f in enumerate((left, right))]
    start, last_rcout, rows, reason = time.monotonic(), rig.t['rcout'], [], None
    try:
        while (elapsed := time.monotonic() - start) < seconds:
            reason = rig.blocker()
            if reason:
                break
            surge, yaw = rig.motion()
            if abs(surge) > max_speed or abs(yaw) > max_yaw:
                reason = f'limit: surge {surge:+.2f} m/s, yaw {yaw:+.2f} rad/s'
                break
            if rig.t['rcout'] > last_rcout:
                last_rcout = rig.t['rcout']
                for trim, servo in zip(trims, rig.servo_outputs()):
                    trim.update(servo, elapsed)
            problem = next((t.problem for t in trims if t.problem), None)
            if problem:
                reason = problem
                break
            command = [t.command() for t in trims]
            rig.publish(command)
            rows.append(dict(t=elapsed, surge=surge, yaw_rate=yaw, input_left=command[0], input_right=command[1],
                             esc_offset_left=trims[0].achieved, esc_offset_right=trims[1].achieved,
                             esc_offset_target_left=trims[0].target, esc_offset_target_right=trims[1].target,
                             battery_v=rig.voltage()))
            log(name, left, right, rows[-1])
            rig.spin(PUBLISH_PERIOD_S)
    finally:
        if reason and reason.startswith(IMMEDIATE):
            rig.publish(None)
        else:
            rig.release()
    if reason:
        return dict(step=name, left_N=left, right_N=right, stopped=reason)
    result = summarize(name, rows, left, right)
    result['learned_servo_forward_sign'] = [t.servo_sign for t in trims]
    return result


def settle(rig, max_s=30.):
    """Wait (no output) until the boat has nearly stopped."""
    end = time.monotonic() + max_s
    while time.monotonic() < end:
        rig.spin(.2)
        motion = rig.motion()
        if motion and abs(motion[0]) < .15 and abs(motion[1]) < .1:
            return True
    return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=('list', 'run'))
    parser.add_argument('steps', nargs='*', help='steps to run (default: all): ' + ', '.join(STEPS))
    parser.add_argument('--voltage', type=float, help='battery voltage for the T200 table (default: MAVROS battery)')
    parser.add_argument('--interface', help='motor_interface.json (default ~/.config/blueboat/alberto, else packaged)')
    parser.add_argument('--output', default=str(DEFAULT_CONFIG / 'thrust_check'))
    parser.add_argument('--seconds', type=float, help='override every step length (8..40 s)')
    parser.add_argument('--max-speed', type=float, default=2.6, help='m/s cap that ends a step (default 2.6)')
    parser.add_argument('--max-yaw', type=float, default=1.3, help='rad/s cap that ends a step (default 1.3)')
    parser.add_argument('--yes', action='store_true', help='do not wait for Enter before each step')
    args = parser.parse_args(argv)
    unknown = [s for s in args.steps if s not in STEPS]
    if unknown:
        parser.error('unknown step ' + ', '.join(unknown))
    if args.seconds is not None and not 8 <= args.seconds <= 40:
        parser.error('--seconds must be 8..40')
    steps = args.steps or list(STEPS)
    if args.command == 'list':
        for name in steps:
            left, right, seconds, what = STEPS[name]
            surge, yaw, source = expected(left, right)
            print(f'{name:8s} L {left:+5.1f} N  R {right:+5.1f} N  {seconds:4.0f} s  expect surge {surge:+.2f} m/s, '
                  f'yaw {yaw:+.2f} rad/s  [{source}]  {what}')
        return 0

    interface = load_interface(args.interface)
    output = Path(args.output).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')
    csv_path, json_path = output / f'thrust-check-{stamp}.csv', output / f'thrust-check-{stamp}.json'

    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    rig = Rig(interface)
    results, writer, voltage = [], None, None
    with csv_path.open('w', newline='') as handle:
        def log(name, left, right, row):
            nonlocal writer
            row = dict(step=name, left_N=left, right_N=right, **row)
            if writer is None:
                writer = csv.DictWriter(handle, fieldnames=list(row))
                writer.writeheader()
            writer.writerow(row)
        try:
            parameters = rig.read_parameters()
            problem = parameter_problem(parameters, interface.channels)
            if problem:
                print('refused: ' + problem)
                return 2
            interface.set_limits(parameters)
            rig.spin(1.)
            voltage = args.voltage or rig.voltage()
            if voltage is None:
                print('refused: no battery voltage from MAVROS; pass --voltage')
                return 2
            table = T200(round(voltage, 1))
            print(f'T200 data at {table.voltage:.1f} V; interface {interface.source}; log {csv_path}')
            for name in steps:
                left, right, seconds, what = STEPS[name]
                seconds = args.seconds or seconds
                surge, yaw, _ = expected(left, right)
                print(f'\n== {name}: {what}. L {left:+.0f} N, R {right:+.0f} N for {seconds:.0f} s '
                      f'(expect {surge:+.2f} m/s, {yaw:+.2f} rad/s)')
                if not settle(rig):
                    print('   boat still moving after 30 s; continuing anyway')
                if not args.yes:
                    input('   clear water ahead? Enter to run, Ctrl+C to stop ')
                while (reason := rig.blocker()) is not None:
                    print('   waiting: ' + reason, flush=True)
                    rig.spin(1.)
                result = run_step(rig, table, name, left, right, seconds, args.max_speed, args.max_yaw, log)
                results.append(result)
                print('   ' + (f"STOPPED: {result['stopped']}" if result.get('stopped') else
                               f"surge {result['surge_m_s']:+.2f} (exp {result['surge_expected_m_s']:+.2f}) m/s, "
                               f"yaw {result['yaw_rate_rad_s']:+.2f} (exp {result['yaw_rate_expected_rad_s']:+.2f}) rad/s"))
                if result.get('stopped', '').startswith(IMMEDIATE):
                    break
        except (KeyboardInterrupt, EOFError):
            print('\naborted: neutral, then released')
        finally:
            rig.close()
    lines, summary, advice = verdict(results)
    print('\n' + '\n'.join(lines))
    print('thrust scale (1.00 = as the T200 data and training model say):', json.dumps({k: round(v, 2) for k, v in summary.items()}))
    print(advice)
    json_path.write_text(json.dumps(dict(results=results, summary=summary, advice=advice, voltage=voltage,
                                         interface=interface.source, csv=str(csv_path),
                                         drag_model=dict(a=DRAG_A, b=DRAG_B)), indent=2) + '\n')
    print('results: ' + str(json_path))
    return 0 if results and all(not r.get('stopped') for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
