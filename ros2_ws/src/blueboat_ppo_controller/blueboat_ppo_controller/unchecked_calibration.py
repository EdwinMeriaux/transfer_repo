"""Explicitly unchecked PWM pulse sequence for Alberto's requested lab branch.

This intentionally does not inspect GPS, odometry, IMU, estimator, vehicle identity,
flight mode, arming state, parameter values or evidence flags.
It is separate from the guarded calibration implementation.
"""
import argparse
import csv
import json
from pathlib import Path
import signal
import sys
import threading
import time

import rclpy
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .mavros_interface import rc_override
from .thrust_calibration import PulsePlan


class UncheckedCalibrationNode(Node):
    def __init__(self, arguments):
        super().__init__('blueboat_unchecked_calibration')
        profile = json.loads(arguments.interface.expanduser().read_text())
        motors = [profile[side] for side in ('left', 'right')]
        self.channels = [motor['rc_channel'] for motor in motors]
        self.neutral = [motor['neutral_pwm'] for motor in motors]
        self.signs = [motor.get('forward_pwm_sign', 1) for motor in motors]
        if (len(set(self.channels)) != 2 or any(type(value) is not int for value in self.channels + self.neutral + self.signs)
                or any(not 1 <= value <= 8 for value in self.channels)
                or any(not 1000 <= value <= 2000 for value in self.neutral)
                or any(value not in (-1, 1) for value in self.signs)):
            raise ValueError('Profile does not contain usable RC channels, neutral PWM and forward signs')
        self.plan = PulsePlan(
            deltas=tuple(arguments.delta), sides=tuple(arguments.sides), repeats=arguments.repeats,
            baseline_s=arguments.baseline, pulse_s=arguments.pulse,
            recovery_s=arguments.recovery, max_delta_pwm=max(abs(value) for value in arguments.delta))
        self.publisher = self.create_publisher(OverrideRCIn, arguments.output_topic, 1)
        self.timer = self.create_timer(0.02, self.control)
        self.countdown = arguments.countdown
        self.created = time.monotonic()
        self.started = None
        self.finished = False
        self.phase = self.trial = None
        output = arguments.log_directory.expanduser()
        output.mkdir(parents=True, exist_ok=True)
        self.log_path = output / f'unchecked-pwm-{time.time_ns()}.csv'
        self.log_file = self.log_path.open('x', newline='', buffering=1)
        self.log = csv.DictWriter(self.log_file, fieldnames=(
            'monotonic_s', 'trial', 'phase', 'side', 'delta_pwm', 'left_pwm', 'right_pwm'))
        self.log.writeheader()
        self.get_logger().warning(
            'UNCHECKED PWM sequence: no vehicle, telemetry, GPS, estimator, parameter or evidence checks')
        self.get_logger().warning(
            f'Publishing RC channels {self.channels} around neutral {self.neutral} in {self.countdown:.1f} seconds')

    def publish(self, pwm):
        self.publisher.publish(OverrideRCIn(channels=rc_override(self.channels, pwm)))

    def control(self):
        now = time.monotonic()
        if self.finished:
            return
        if now - self.created < self.countdown:
            self.publish(self.neutral)
            return
        if self.started is None:
            self.started = now
        index, phase, pwm, trial = self.plan.step(now, self.neutral, self.signs)
        if phase == 'complete':
            self.publish(self.neutral)
            self.finished = True
            self.get_logger().info(f'Unchecked pulse sequence complete; log={self.log_path}')
            return
        self.publish(pwm)
        if (index, phase) != (self.trial, self.phase):
            self.trial, self.phase = index, phase
            self.get_logger().info(
                f'trial={index} phase={phase} side={trial.side} delta={trial.delta_pwm} pwm={pwm}')
        self.log.writerow(dict(monotonic_s=now, trial=index, phase=phase,
                               side=trial.side, delta_pwm=trial.delta_pwm,
                               left_pwm=pwm[0], right_pwm=pwm[1]))

    def stop(self):
        # Briefly request neutral, then release only the two owned RC inputs.
        for _ in range(5):
            self.publish(self.neutral)
            rclpy.spin_once(self, timeout_sec=0.02)
        for _ in range(5):
            self.publish(None)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.log_file.close()


def arguments(argv=None):
    parser = argparse.ArgumentParser(description='Run the unchecked Alberto PWM calibration pulse sequence')
    parser.add_argument('--interface', type=Path,
                        default=Path.home()/'.config/blueboat/alberto/motor_interface.json')
    parser.add_argument('--output-topic', default='/mavros/rc/override')
    parser.add_argument('--log-directory', type=Path,
                        default=Path.home()/'.config/blueboat/alberto/runs/unchecked_calibration')
    parser.add_argument('--countdown', type=float, default=5.0)
    parser.add_argument('--delta', type=int, nargs='+', default=[-40, -60, 40, 60])
    parser.add_argument('--sides', nargs='+', choices=('left', 'right', 'symmetric'),
                        default=['left', 'right', 'symmetric'])
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--baseline', type=float, default=2.0)
    parser.add_argument('--pulse', type=float, default=0.5)
    parser.add_argument('--recovery', type=float, default=3.0)
    return parser.parse_args(argv)


def main(argv=None):
    parsed = arguments(argv)
    if not parsed.delta or any(value == 0 or abs(value) > 250 for value in parsed.delta):
        raise ValueError('PWM deltas must be nonzero and no greater than 250 counts')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = threading.Event()
    handlers = {sig: signal.signal(sig, lambda number, frame: stop_requested.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = UncheckedCalibrationNode(parsed)
        while rclpy.ok() and not node.finished and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and rclpy.ok():
            node.stop()
        if node is not None:
            node.destroy_node()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
