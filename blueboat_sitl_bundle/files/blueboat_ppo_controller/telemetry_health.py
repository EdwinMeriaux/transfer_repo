"""Bounded telemetry-gap handling for the PPO controller (ROS-free).

States, evaluated once per control cycle:

FRESH            every stream has a sample no older than ``fresh_s`` and the
                 streams agree in time. The policy steps normally.
TRANSIENT_GAP    a stream went quiet less than ``hold_s`` ago. The node keeps
                 the last command; the policy is not stepped on the reused
                 sample.
RECOVERING       quiet for longer than ``hold_s`` (or waiting for first data).
                 The node commands neutral. It returns to FRESH only after
                 ``resume_s`` of continuously fresh data with new samples on
                 every stream.
PERSISTENT_FAULT an engaged pause lasted longer than ``outage_s``, or pauses
                 repeated more than ``max_pauses`` times within
                 ``pause_window_s``. Latched; the node releases the output.

A reused sample keeps its original source and receipt timestamps. It never
counts as a new sample, never enters the rate history and never refreshes
freshness: age keeps growing until a genuinely new message arrives.
"""
import math
from collections import deque

FRESH = 'FRESH'
TRANSIENT_GAP = 'TRANSIENT_GAP'
RECOVERING = 'RECOVERING'
PERSISTENT_FAULT = 'PERSISTENT_FAULT'


class StreamHealth:
    def __init__(self, name, rate_window_s=2.):
        self.name = name
        self.window = rate_window_s
        self.message = None
        self.stamp = None
        self.receipt = None
        self.accepted = 0
        self.rejected = 0
        self.last_rejection = None
        self.receipts = deque(maxlen=2000)
        self._cycle_seen = None
        self.reused = False
        self.missed_cycles = 0

    def offer(self, message, stamp, ros_now, receipt, max_age_s, future_tolerance_s=0.):
        """Accept a genuinely new, fresh sample; otherwise count and drop it."""
        reason = None
        if not all(math.isfinite(x) for x in (stamp, ros_now, receipt)):
            reason = 'nonfinite_timestamp'
        elif ros_now - stamp > max_age_s:
            reason = 'stale_source_timestamp'
        elif ros_now - stamp < -future_tolerance_s:
            reason = 'future_source_timestamp'
        elif self.stamp is not None and stamp <= self.stamp:
            reason = 'repeated_or_backward_source_timestamp'
        if reason:
            self.rejected += 1
            self.last_rejection = reason
            return False
        self.message, self.stamp, self.receipt = message, stamp, receipt
        self.accepted += 1
        self.receipts.append(receipt)
        while self.receipts and receipt - self.receipts[0] > self.window:
            self.receipts.popleft()
        return True

    def mark_cycle(self):
        """Call once per control cycle: was the sample used this cycle new?"""
        if self.message is not None and self._cycle_seen == self.accepted:
            self.reused = True
            self.missed_cycles += 1
        else:
            self.reused = False
            self.missed_cycles = 0
        self._cycle_seen = self.accepted

    def rate(self, now):
        recent = [r for r in self.receipts if 0 <= now - r <= self.window]
        return (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.

    def report(self, now, ros_now):
        return dict(
            source_stamp_s=self.stamp, receipt_monotonic_s=self.receipt,
            source_age_s=None if self.stamp is None else ros_now - self.stamp,
            receipt_age_s=None if self.receipt is None else now - self.receipt,
            reused=self.reused, consecutive_reused_cycles=self.missed_cycles,
            accepted=self.accepted, rejected=self.rejected, last_rejection=self.last_rejection,
            rate_hz=self.rate(now))


class TelemetryMonitor:
    def __init__(self, names, fresh_s=.2, hold_s=.4, resume_s=.5, outage_s=5., max_skew_s=.1,
                 future_tolerance_s=0., max_pauses=5, pause_window_s=60.):
        values = (fresh_s, hold_s, resume_s, outage_s, max_skew_s, future_tolerance_s, pause_window_s)
        if (not all(math.isfinite(v) for v in values) or not 0 < fresh_s <= hold_s <= 1.5
                or not .1 <= resume_s <= 5. or not hold_s < outage_s <= 30. or max_skew_s <= 0
                or future_tolerance_s < 0 or type(max_pauses) is not int or not 1 <= max_pauses <= 100
                or not 10. <= pause_window_s <= 600.):
            raise ValueError('Invalid telemetry hold/resume/outage settings')
        self.fresh_s, self.hold_s, self.resume_s, self.outage_s = fresh_s, hold_s, resume_s, outage_s
        self.max_skew_s, self.future_tolerance_s = max_skew_s, future_tolerance_s
        self.max_pauses, self.pause_window_s = max_pauses, pause_window_s
        self.streams = {name: StreamHealth(name) for name in names}
        self.state = RECOVERING
        self.reason = 'waiting_for_telemetry'
        self.fault = None
        self.last_fresh = None
        self.fresh_since = None
        self._fresh_counts = None
        self.pause_started = None
        self.pauses = deque()
        self.resumes = 0

    def offer(self, name, message, stamp, ros_now, receipt):
        return self.streams[name].offer(message, stamp, ros_now, receipt, self.fresh_s, self.future_tolerance_s)

    def freshness(self, now, ros_now):
        """(True, None) when every stream is fresh and the streams agree in time."""
        for name, stream in self.streams.items():
            if stream.stamp is None:
                return False, name + '_no_samples'
            age = ros_now - stream.stamp
            if not -self.future_tolerance_s <= age <= self.fresh_s:
                return False, name + '_source_stale'
            if not 0 <= now - stream.receipt <= self.fresh_s:
                return False, name + '_receipt_stale'
        stamps = [s.stamp for s in self.streams.values()]
        if max(stamps) - min(stamps) > self.max_skew_s:
            return False, 'telemetry_time_skew'
        return True, None

    def update(self, now, ros_now, engaged):
        """Advance the state machine once per control cycle. ``engaged``: actuating."""
        for stream in self.streams.values():
            stream.mark_cycle()
        if self.fault:
            return self.state
        fresh, why = self.freshness(now, ros_now)
        if fresh:
            self.last_fresh = now
        if self.state in (FRESH, TRANSIENT_GAP):
            if fresh:
                self.state, self.reason = FRESH, None
            elif now - self.last_fresh <= self.hold_s:
                self.state, self.reason = TRANSIENT_GAP, why
            else:
                self._pause(now, why, engaged)
        if self.state == RECOVERING:
            if engaged and self.pause_started is None:
                self.pause_started = now
            if fresh:
                counts = {n: s.accepted for n, s in self.streams.items()}
                if self.fresh_since is None:
                    self.fresh_since, self._fresh_counts = now, counts
                elif (now - self.fresh_since >= self.resume_s
                      and all(counts[n] - self._fresh_counts[n] >= 2 for n in counts)):
                    self.state, self.reason = FRESH, None
                    self.fresh_since = self.pause_started = None
                    self.resumes += 1
            else:
                self.fresh_since, self.reason = None, why
            if (self.state == RECOVERING and engaged and self.pause_started is not None
                    and now - self.pause_started > self.outage_s):
                self._fail('telemetry_outage')
        return self.state

    def _pause(self, now, why, engaged):
        self.state, self.reason = RECOVERING, why
        self.fresh_since = None
        self.pause_started = now if engaged else None
        if engaged:
            self.pauses.append(now)
            while self.pauses and now - self.pauses[0] > self.pause_window_s:
                self.pauses.popleft()
            if len(self.pauses) > self.max_pauses:
                self._fail('telemetry_unhealthy_repeated_pauses')

    def _fail(self, reason):
        self.fault = reason
        self.state, self.reason = PERSISTENT_FAULT, reason

    def report(self, now, ros_now):
        return dict(
            state=self.state, reason=self.reason, fault=self.fault,
            since_last_fresh_s=None if self.last_fresh is None else now - self.last_fresh,
            pause_elapsed_s=None if self.pause_started is None else now - self.pause_started,
            resume_progress_s=None if self.fresh_since is None else now - self.fresh_since,
            pauses_in_window=len(self.pauses), resumes=self.resumes,
            limits=dict(fresh_s=self.fresh_s, hold_s=self.hold_s, resume_s=self.resume_s, outage_s=self.outage_s,
                        max_pauses=self.max_pauses, pause_window_s=self.pause_window_s),
            streams={n: s.report(now, ros_now) for n, s in self.streams.items()})
