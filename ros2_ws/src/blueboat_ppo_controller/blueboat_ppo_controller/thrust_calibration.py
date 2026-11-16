"""ROS-independent pulse scheduling, source-time checks and CSV recording."""
import csv
import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Pulse:
    side: str
    delta_pwm: int
    repeat: int


class PulsePlan:
    """A finite sequence; faults never resume. Time is monotonic, not ROS time."""
    def __init__(self, deltas=(-40, -60, 40, 60), sides=('left', 'right', 'symmetric'), repeats=3,
                 baseline_s=2., pulse_s=.5, recovery_s=3., max_delta_pwm=60):
        if (not sides or any(s not in ('left', 'right', 'symmetric') for s in sides)
                or not deltas or any(type(d) is not int or d == 0 or abs(d) > max_delta_pwm for d in deltas)
                or type(repeats) is not int or not 1 <= repeats <= 20
                or not 0 < max_delta_pwm <= 250
                or not all(math.isfinite(x) for x in (baseline_s, pulse_s, recovery_s))
                or baseline_s < 1 or not .1 <= pulse_s <= 1 or recovery_s < 1):
            raise ValueError('Invalid bounded pulse plan')
        self.trials = [Pulse(side, delta, repeat) for repeat in range(repeats)
                       for delta in deltas for side in sides]
        self.baseline_s, self.pulse_s, self.recovery_s = baseline_s, pulse_s, recovery_s
        self.period = baseline_s + pulse_s + recovery_s
        self.started = None

    def step(self, now, neutral, forward_signs=(1, 1)):
        if self.started is None:
            self.started = now
        elapsed = now - self.started
        if elapsed < 0:
            raise ValueError('Monotonic clock moved backwards')
        index = int(elapsed // self.period)
        if index >= len(self.trials):
            return -1, 'complete', list(neutral), None
        offset = elapsed - index * self.period
        phase = 'baseline' if offset < self.baseline_s else (
            'pulse' if offset < self.baseline_s + self.pulse_s else 'recovery')
        trial = self.trials[index]
        pwm = list(neutral)
        if phase == 'pulse':
            for i, side in enumerate(('left', 'right')):
                if trial.side in (side, 'symmetric'):
                    pwm[i] += trial.delta_pwm * forward_signs[i]
        if any(not 1000 <= v <= 2000 for v in pwm):
            raise ValueError('Pulse crosses PWM limits')
        return index, phase, pwm, trial


class RestCheck:
    """A continuous, fresh quiet interval; one stationary sample cannot enable a pulse."""
    def __init__(self, duration_s=1., max_gap_s=.06):
        if not all(math.isfinite(v) and v > 0 for v in (duration_s, max_gap_s)):
            raise ValueError('Invalid rest verification interval')
        self.duration, self.max_gap = duration_s, max_gap_s
        self.reset()

    def reset(self):
        self.since = self.last = None

    def observe(self, now, stationary):
        if not stationary or not math.isfinite(now):
            self.reset()
            return
        if self.last is None or not 0 < now-self.last <= self.max_gap:
            self.since = now
        self.last = now

    def ready(self, now):
        return (self.since is not None and self.last is not None
                and 0 <= now-self.last <= self.max_gap and now-self.since >= self.duration)


class StreamCheck:
    """Checks source AND receipt clocks; rates use a rolling receipt window."""
    def __init__(self, max_age_s=.2, max_gap_s=.15, window_s=2.):
        self.max_age, self.max_gap, self.window = max_age_s, max_gap_s, window_s
        self.last_stamp = None
        self.receipts = deque(maxlen=2000)
        self.last_error = 'no_samples'

    def accept(self, stamp, ros_now, receipt):
        if not all(math.isfinite(x) for x in (stamp, ros_now, receipt)):
            self.last_error = 'nonfinite_timestamp'
        elif not 0 <= ros_now - stamp <= self.max_age:
            self.last_error = 'stale_or_future_source_timestamp'
        elif self.last_stamp is not None and stamp <= self.last_stamp:
            self.last_error = 'repeated_or_backward_source_timestamp'
        else:
            self.last_stamp = stamp
            self.receipts.append(receipt)
            while len(self.receipts) > 1 and receipt - self.receipts[0] > self.window:
                self.receipts.popleft()
            self.last_error = None
            return
        raise ValueError(self.last_error)

    def report(self, now, ros_now):
        recent = [x for x in self.receipts if 0 <= now - x <= self.window]
        rate = (len(recent)-1) / (recent[-1]-recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.
        gap = max(np.diff(recent), default=0.)
        fresh = bool(recent and now - recent[-1] <= self.max_age and self.last_stamp is not None
                     and 0 <= ros_now - self.last_stamp <= self.max_age and self.last_error is None)
        return dict(rate_hz=rate, max_gap_s=float(gap), samples=len(recent),
                    span_s=recent[-1]-recent[0] if recent else 0., fresh=fresh,
                    source_stamp_s=self.last_stamp, error=self.last_error)


def post_command_feedback_ready(command_time, output_receipt, now, grace_s):
    """True only after the grace period and a feedback sample newer than the command."""
    values = (command_time, output_receipt, now, grace_s)
    if any(value is None or not math.isfinite(value) for value in values):
        return False
    return now-command_time >= grace_s and output_receipt > command_time


VECTORS = {'a': ('x', 'y', 'z'), 'w': ('x', 'y', 'z'), 'q': ('x', 'y', 'z', 'w'), 'v': ('x', 'y', 'z')}
CSV_FIELDS = ['receipt_monotonic_s', 'ros_time_s', 'imu_stamp_s', 'attitude_stamp_s', 'velocity_stamp_s',
              'output_stamp_s', 'command_monotonic_s', 'trial_end_monotonic_s', 'trial', 'side', 'delta_pwm', 'phase', 'mode',
              'valid', 'reason', 'imu_frame', 'attitude_frame', 'velocity_frame',
              'command_left_pwm', 'command_right_pwm', 'planned_left_pwm', 'planned_right_pwm',
              'output_left_pwm', 'output_right_pwm', 'output_channels_json'] + [f'{p}{a}' for p, axes in VECTORS.items() for a in axes]


class Recorder:
    def __init__(self, path):
        self.file = open(path, 'x', newline='', buffering=1)
        self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()

    def write(self, row):
        self.writer.writerow(row)

    def close(self):
        self.file.close()
