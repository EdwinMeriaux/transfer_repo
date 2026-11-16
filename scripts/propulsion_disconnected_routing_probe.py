#!/usr/bin/env python3
"""Bounded RC routing probe for a physically disconnected propulsion system."""

import argparse
import csv
import json
import math
from pathlib import Path
import signal
import statistics
import threading
import time

import rclpy
from mavros_msgs.msg import OverrideRCIn, RCOut, State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from blueboat_ppo_controller.mavros_interface import rc_override


class RoutingProbe(Node):
    def __init__(self, profile_path, output_directory, delta, stop_event):
        super().__init__('blueboat_propulsion_disconnected_routing_probe')
        profile = json.loads(profile_path.read_text())
        self.channels = [profile[side]['rc_channel'] for side in ('left', 'right')]
        self.outputs = [profile[side]['servo_output'] for side in ('left', 'right')]
        self.neutral = [profile[side]['neutral_pwm'] for side in ('left', 'right')]
        self.delta = delta
        self.stop_event = stop_event
        if self.channels != [1, 3] or self.outputs != [3, 1] or self.neutral != [1500, 1500]:
            raise ValueError(
                f'Unexpected routing profile: channels={self.channels}, '
                f'outputs={self.outputs}, neutral={self.neutral}'
            )

        self.state = None
        self.state_received = -math.inf
        self.rc_out_received = -math.inf
        self.stage = 'startup'
        self.requested = self.neutral.copy()
        self.rows = []
        self.publisher = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', 1
        )
        self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.create_subscription(
            RCOut, '/mavros/rc/out', self.on_rc_out, qos_profile_sensor_data
        )

        output_directory.mkdir(parents=True, exist_ok=False)
        self.csv_path = output_directory / 'routing-probe.csv'
        self.summary_path = output_directory / 'summary.json'
        self.csv_file = self.csv_path.open('x', newline='', buffering=1)
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=(
                'monotonic_s', 'stage', 'requested_left_pwm',
                'requested_right_pwm', 'servo1_pwm', 'servo3_pwm',
                'armed', 'mode',
            ),
        )
        self.writer.writeheader()

    def on_state(self, message):
        self.state = message
        self.state_received = time.monotonic()

    def on_rc_out(self, message):
        self.rc_out_received = time.monotonic()
        servo1 = message.channels[0] if len(message.channels) >= 1 else None
        servo3 = message.channels[2] if len(message.channels) >= 3 else None
        row = {
            'monotonic_s': self.rc_out_received,
            'stage': self.stage,
            'requested_left_pwm': self.requested[0],
            'requested_right_pwm': self.requested[1],
            'servo1_pwm': servo1,
            'servo3_pwm': servo3,
            'armed': None if self.state is None else self.state.armed,
            'mode': None if self.state is None else self.state.mode,
        }
        self.rows.append(row)
        self.writer.writerow(row)

    def safe(self):
        now = time.monotonic()
        return bool(
            self.state
            and self.state.connected
            and self.state.armed
            and self.state.mode == 'MANUAL'
            and self.state.system_status in (3, 4)
            and now - self.state_received <= 1.5
            and now - self.rc_out_received <= 0.5
            and self.count_publishers('/mavros/rc/override') == 1
            and not self.stop_event.is_set()
        )

    def publish(self, values):
        self.requested = list(values)
        self.publisher.publish(
            OverrideRCIn(channels=rc_override(self.channels, values))
        )

    def release(self):
        self.publisher.publish(
            OverrideRCIn(channels=rc_override(self.channels, None))
        )

    def run_stage(self, name, values, duration):
        self.stage = name
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if not self.safe():
                raise RuntimeError(
                    f'Safety state lost during {name}; releasing overrides'
                )
            self.publish(values)
            time.sleep(0.03)

    def execute(self):
        ready_by = time.monotonic() + 8.0
        while time.monotonic() < ready_by:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.safe():
                break
        if not self.safe():
            raise RuntimeError('Fresh armed MANUAL state and RCOut were not available')

        left_forward = [self.neutral[0] + self.delta, self.neutral[1]]
        left_reverse = [self.neutral[0] - self.delta, self.neutral[1]]
        right_forward = [self.neutral[0], self.neutral[1] + self.delta]
        right_reverse = [self.neutral[0], self.neutral[1] - self.delta]
        paired_forward = [value + self.delta for value in self.neutral]

        stages = [
            ('neutral_initial', self.neutral, 2.0),
            ('left_positive', left_forward, 1.0),
            ('neutral_after_left_positive', self.neutral, 1.5),
            ('left_negative', left_reverse, 1.0),
            ('neutral_after_left_negative', self.neutral, 1.5),
            ('right_positive', right_forward, 1.0),
            ('neutral_after_right_positive', self.neutral, 1.5),
            ('right_negative', right_reverse, 1.0),
            ('neutral_after_right_negative', self.neutral, 1.5),
            ('paired_positive', paired_forward, 1.0),
            ('neutral_final', self.neutral, 2.0),
        ]
        for name, values, duration in stages:
            self.get_logger().info(f'stage={name} requested={values}')
            self.run_stage(name, values, duration)

        self.stage = 'release'
        for _ in range(10):
            self.release()
            rclpy.spin_once(self, timeout_sec=0.03)

    def analyze(self):
        by_stage = {}
        for row in self.rows:
            if row['servo1_pwm'] is None or row['servo3_pwm'] is None:
                continue
            by_stage.setdefault(row['stage'], []).append(row)

        required = (
            'neutral_initial', 'left_positive', 'left_negative',
            'right_positive', 'right_negative', 'paired_positive',
            'neutral_final',
        )
        for name in required:
            if len(by_stage.get(name, ())) < 3:
                raise RuntimeError(f'Insufficient RCOut samples for {name}')

        def medians(name):
            rows = by_stage[name]
            return [
                statistics.median(row[key] for row in rows)
                for key in ('servo1_pwm', 'servo3_pwm')
            ]

        neutral_start = medians('neutral_initial')
        neutral_end = medians('neutral_final')
        stage_medians = {name: medians(name) for name in required}

        # left maps to servo output 3 (index 1); right maps to output 1 (index 0)
        left_pos = stage_medians['left_positive'][1] - neutral_start[1]
        left_neg = stage_medians['left_negative'][1] - neutral_start[1]
        right_pos = stage_medians['right_positive'][0] - neutral_start[0]
        right_neg = stage_medians['right_negative'][0] - neutral_start[0]
        left_cross = max(
            abs(stage_medians[name][0] - neutral_start[0])
            for name in ('left_positive', 'left_negative')
        )
        right_cross = max(
            abs(stage_medians[name][1] - neutral_start[1])
            for name in ('right_positive', 'right_negative')
        )
        neutral_return_error = max(
            abs(a - b) for a, b in zip(neutral_start, neutral_end)
        )
        paired_right = stage_medians['paired_positive'][0] - neutral_start[0]
        paired_left = stage_medians['paired_positive'][1] - neutral_start[1]

        passed = bool(
            abs(left_pos) >= 8
            and abs(left_neg) >= 8
            and left_pos * left_neg < 0
            and abs(right_pos) >= 8
            and abs(right_neg) >= 8
            and right_pos * right_neg < 0
            and left_cross <= 3
            and right_cross <= 3
            and neutral_return_error <= 3
            and abs(paired_left) >= 8
            and abs(paired_right) >= 8
            and paired_left * left_pos > 0
            and paired_right * right_pos > 0
        )
        summary = {
            'test': 'propulsion_disconnected_rc_routing',
            'propulsion_physically_disconnected': True,
            'physical_motor_direction_tested': False,
            'requested_delta_pwm': self.delta,
            'input_neutral_pwm': self.neutral,
            'servo_neutral_pwm': {
                'right_servo1': neutral_start[0],
                'left_servo3': neutral_start[1],
            },
            'servo_changes_pwm': {
                'left_positive': left_pos,
                'left_negative': left_neg,
                'right_positive': right_pos,
                'right_negative': right_neg,
                'paired_left': paired_left,
                'paired_right': paired_right,
            },
            'inferred_servo_forward_pwm_sign': {
                'left': 1 if left_pos > 0 else -1,
                'right': 1 if right_pos > 0 else -1,
            },
            'maximum_cross_output_change_pwm': {
                'left_test_changed_right': left_cross,
                'right_test_changed_left': right_cross,
            },
            'neutral_return_error_pwm': neutral_return_error,
            'stage_medians_servo1_servo3': stage_medians,
            'passed': passed,
        }
        self.summary_path.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
        if not passed:
            raise RuntimeError('Routing-probe acceptance gates failed')

    def close_probe(self):
        try:
            for _ in range(10):
                self.release()
                rclpy.spin_once(self, timeout_sec=0.03)
        finally:
            self.csv_file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--interface', type=Path, required=True)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--delta-pwm', type=int, default=20)
    args = parser.parse_args()
    if not 10 <= args.delta_pwm <= 25:
        raise SystemExit('delta-pwm must be in 10..25')

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    old_handlers = {
        sig: signal.signal(sig, lambda *_: stop.set())
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    node = RoutingProbe(
        args.interface.expanduser(), args.output_directory.expanduser(),
        args.delta_pwm, stop,
    )
    try:
        if stop.is_set():
            raise RuntimeError('Stop requested before routing probe')
        node.execute()
        node.analyze()
    finally:
        node.close_probe()
        node.destroy_node()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        rclpy.shutdown()


if __name__ == '__main__':
    main()

