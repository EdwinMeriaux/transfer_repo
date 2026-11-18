"""PPO telemetry hold / pause / resume (handoff tests 1, 2, 8, 10, 12)."""
import json

import numpy as np
import pytest
from types import SimpleNamespace as NS

from blueboat_ppo_controller.telemetry_health import (
    FRESH, PERSISTENT_FAULT, RECOVERING, TRANSIENT_GAP, TelemetryMonitor)
from ros_harness import Clock, imu, load, odom, state, vec

ODOM, IMU = '/mavros/local_position/odom', '/mavros/imu/data'


# --- TelemetryMonitor, no ROS ---------------------------------------------------

def feed(monitor, t, names=('odometry', 'imu')):
    for name in names:
        monitor.offer(name, ('msg', name, t), t, t + .002, t + .002)


def run(monitor, start, end, dt=.02, engaged=True, gap=None):
    t = start
    while t < end - 1e-9:
        t = round(t + dt, 6)
        if gap is None or not gap[0] <= t < gap[1]:
            feed(monitor, t)
        monitor.update(t + .002, t + .002, engaged)
    return t


def test_short_gap_reuses_previous_sample_with_its_original_timestamp():
    m = TelemetryMonitor(('odometry', 'imu'))
    t = run(m, 0., 1.)
    assert m.state == FRESH
    before = m.streams['imu']
    message, stamp, receipt = before.message, before.stamp, before.receipt
    for _ in range(12):                                     # 0.24 s with no messages
        t = round(t + .02, 6)
        m.update(t, t, True)
    assert m.state == TRANSIENT_GAP
    assert (before.message, before.stamp, before.receipt) == (message, stamp, receipt)
    report = m.report(t, t)['streams']['imu']
    assert report['source_stamp_s'] == stamp and report['reused'] and report['source_age_s'] > .2


def test_reused_samples_never_count_as_new_or_raise_the_rate():
    m = TelemetryMonitor(('odometry', 'imu'))
    t = run(m, 0., 1.)
    stream = m.streams['imu']
    accepted, history = stream.accepted, list(stream.receipts)
    for _ in range(10):
        t = round(t + .02, 6)
        m.update(t, t, True)
    assert stream.accepted == accepted and list(stream.receipts) == history
    assert stream.reused and stream.missed_cycles == 10
    # re-delivering the same stamp is rejected, not counted
    assert not m.offer('imu', 'dup', stream.stamp, t, t)
    assert stream.accepted == accepted and stream.last_rejection == 'repeated_or_backward_source_timestamp'
    assert stream.rate(t) <= 50. + 1e-6                     # only real receipts set the rate


def test_gap_longer_than_hold_pauses_and_resumes_after_fresh_window():
    m = TelemetryMonitor(('odometry', 'imu'), hold_s=.4, resume_s=.5)
    t = run(m, 0., 1.)
    resumes = m.resumes                                     # the initial start counts as one
    t = run(m, t, 2.2, gap=(t, t + 1.2))                    # 1.2 s gap, then data again
    assert m.state in (RECOVERING, FRESH)
    states = []
    for _ in range(40):
        t = run(m, t, t + .02)
        states.append(m.state)
    assert RECOVERING in states and states[-1] == FRESH and m.resumes == resumes + 1
    assert m.fault is None


def test_persistent_missing_telemetry_becomes_a_latched_fault():
    m = TelemetryMonitor(('odometry', 'imu'), outage_s=5.)
    t = run(m, 0., 1.)
    t = run(m, t, t + 5.6, gap=(t, t + 100))
    assert m.state == PERSISTENT_FAULT and m.fault == 'telemetry_outage'
    run(m, t, t + 2.)                                       # data returning does not clear it
    assert m.state == PERSISTENT_FAULT


def test_repeated_pauses_become_a_fault():
    m = TelemetryMonitor(('odometry', 'imu'), max_pauses=3, pause_window_s=60.)
    t = run(m, 0., 1.)
    for _ in range(4):
        t = run(m, t, t + 2., gap=(t, t + .8))
    assert m.fault == 'telemetry_unhealthy_repeated_pauses'


def test_waiting_before_engagement_never_faults():
    m = TelemetryMonitor(('odometry', 'imu'), outage_s=5.)
    for i in range(500):
        m.update(i * .02, i * .02, engaged=False)
    assert m.state == RECOVERING and m.fault is None


def test_monitor_diagnostics_distinguish_states():
    m = TelemetryMonitor(('odometry', 'imu'), outage_s=2.)
    t = run(m, 0., 1.)
    assert m.report(t, t)['state'] == FRESH
    t = run(m, t, t + .3, gap=(t, t + 100))
    r = m.report(t, t)
    assert r['state'] == TRANSIENT_GAP and r['streams']['imu']['reused'] and r['since_last_fresh_s'] > .05
    t = run(m, t, t + .5, gap=(0, 1e9))
    r = m.report(t, t)
    assert r['state'] == RECOVERING and r['pause_elapsed_s'] is not None
    t = run(m, t, t + 2.5, gap=(0, 1e9))
    assert m.report(t, t)['state'] == PERSISTENT_FAULT


def test_monitor_rejects_unbounded_settings():
    with pytest.raises(ValueError):
        TelemetryMonitor(('imu',), fresh_s=.2, hold_s=.1)
    with pytest.raises(ValueError):
        TelemetryMonitor(('imu',), hold_s=2.)
    with pytest.raises(ValueError):
        TelemetryMonitor(('imu',), outage_s=60.)


# --- PPO node, stub ROS ---------------------------------------------------------

class FakeRuntime:
    def __init__(self, directory=None):
        self.steps = self.resets = 0
        self.elapsed_s = 0.
        self.last_observation = np.zeros(3)

    def reset_history(self):
        self.resets += 1

    def action_from_body(self, q, v, w, command, previous):
        self.steps += 1
        return np.array([.1, .1])

    def actuator_controls(self, action):
        return np.array([5., 5.])


class FakeAdapter:
    def __init__(self, path, target):
        self.authority, self.enable_channel, self.channels, self.expected = 'direct_mavros', None, [1, 3], {}

    def convert(self, forces):
        return [1600, 1600], np.asarray(forces)

    def neutral(self):
        return [1500, 1500]

    def verify_vehicle(self, info):
        return True

    def verify_parameters(self, actual):
        return True


class PPOSim:
    PERIODS = {ODOM: 1/30, IMU: 1/50, '/mavros/state': .5, '/blueboat/enable': .1, '/blueboat/cmd_vel': .1}

    def __init__(self, monkeypatch, tmp_path, **overrides):
        self.clock = Clock()
        params = dict(mode='actuator', enable_actuators=True, calibration_file='x', require_estimator_status=False,
                      log_directory=str(tmp_path))
        params.update(overrides)
        self.module = load(monkeypatch, 'ppo_node', self.clock, params)
        monkeypatch.setattr(self.module, 'GeneralistPolicyRuntime', FakeRuntime)
        monkeypatch.setattr(self.module, 'MotorAdapter', FakeAdapter)
        self.node = self.module.PPONode()
        self.armed = True
        self.gaps = []
        self.next = {k: self.clock.mono for k in self.PERIODS}
        self.due = {id(t): self.clock.mono + t.period for t in self.node.timers}
        self.next_watch = self.clock.mono
        self.node.request_parameters()

    def gap(self, seconds, start=None):
        start = self.clock.mono if start is None else start
        self.gaps.append((start, start + seconds))

    def message(self, topic, stamp):
        if topic == ODOM:
            return odom(stamp)
        if topic == IMU:
            return imu(stamp)
        if topic == '/mavros/state':
            return state(armed=self.armed)
        if topic == '/blueboat/cmd_vel':
            return NS(linear=vec(.3), angular=vec())
        return NS(data=True)

    def step(self, dt=.005):
        self.clock.mono += dt
        t = self.clock.mono
        for topic, period in self.PERIODS.items():
            if t + 1e-9 >= self.next[topic]:
                self.next[topic] += period
                if topic in (ODOM, IMU) and any(a <= t < b for a, b in self.gaps):
                    continue
                self.node.subscriptions[topic](self.message(topic, self.clock.ros() - .002))
        for timer in self.node.timers:
            if not timer.cancelled and t + 1e-9 >= self.due[id(timer)]:
                self.due[id(timer)] += timer.period
                timer.callback()
        if t + 1e-9 >= self.next_watch:
            self.next_watch += .02
            self.node.watch_once()

    def run(self, seconds, until=None):
        end = self.clock.mono + seconds
        while self.clock.mono < end:
            self.step()
            if until is not None and until(self):
                return True
        return until is None

    def commands(self):
        return [[m.channels[0], m.channels[2]] for m in self.node.output.messages]

    def status(self):
        return json.loads(self.node.diagnostics.messages[-1].data)

    def statuses(self, last=None):
        messages = self.node.diagnostics.messages[-last:] if last else self.node.diagnostics.messages
        return [json.loads(m.data)['status'] for m in messages]


@pytest.fixture
def ppo(monkeypatch, tmp_path):
    sims = []

    def make(**kwargs):
        sims.append(PPOSim(monkeypatch, tmp_path, **kwargs))
        return sims[-1]
    yield make
    for s in sims:
        s.node.close()


def actuating(s):
    return s.node.gate.active and s.status()['status'] == 'actuating'


def test_ppo_short_gap_holds_last_command_without_stepping_policy(ppo):
    s = ppo()
    assert s.run(3, until=actuating)
    s.run(.5)
    s.gap(.3)
    s.run(.21)                                    # samples up to 0.2 s old are still fresh
    steps, sent = s.node.runtime.steps, len(s.commands())
    s.run(.08)
    assert s.status()['status'] == 'telemetry_hold'
    assert s.node.runtime.steps == steps                         # no policy step on reused samples
    assert all(c == [1600, 1600] for c in s.commands()[sent:])   # last command kept, never neutral
    assert s.status()['telemetry']['streams']['imu']['reused']
    assert s.run(1, until=actuating) and s.node.gate.fault is None


def test_ppo_long_gap_pauses_at_neutral_then_resumes_with_fresh_policy(ppo):
    s = ppo()
    assert s.run(3, until=actuating)
    resets = s.node.runtime.resets
    s.gap(1.2)
    assert s.run(.6, until=lambda s: s.status()['status'] == 'telemetry_paused')
    assert s.commands()[-1] == [1500, 1500] and s.node.owns_output and s.node.gate.fault is None
    assert s.status()['telemetry']['state'] == RECOVERING
    gap_end = s.gaps[-1][1]
    assert s.run(3, until=actuating)
    assert s.clock.mono - gap_end >= .5                          # waited for fresh, stable data
    assert s.node.runtime.resets == resets + 1                   # history reset on resume
    assert s.commands()[-1] == [1600, 1600] and s.node.gate.fault is None


def test_ppo_persistent_outage_neutral_then_release(ppo):
    s = ppo(telemetry_outage_s=2.)
    assert s.run(3, until=actuating)
    s.gap(100.)
    assert s.run(4, until=lambda s: s.node.gate.fault is not None)
    assert s.node.gate.fault == 'telemetry_outage'
    s.run(.5)
    commands = s.commands()
    assert commands[-1] == [0, 0] and [1500, 1500] in commands[-20:] and not s.node.owns_output


def test_ppo_disarm_is_immediate_during_a_pause(ppo):
    s = ppo()
    assert s.run(3, until=actuating)
    s.gap(5.)
    assert s.run(1, until=lambda s: s.status()['status'] == 'telemetry_paused')
    s.node.subscriptions['/mavros/state'](state(armed=False))
    assert s.commands()[-1] == [0, 0] and s.node.gate.fault == 'manual_takeover_or_disarmed'


def test_ppo_enable_loss_is_immediate_during_a_hold(ppo):
    s = ppo()
    assert s.run(3, until=actuating)
    s.gap(.3)
    s.run(.1)
    s.node.subscriptions['/blueboat/enable'](NS(data=False))
    assert s.commands()[-1] == [0, 0] and s.node.gate.fault == 'enable_released'


def test_ppo_watchdog_still_trips_when_the_loop_stalls(ppo):
    s = ppo()
    assert s.run(3, until=actuating)
    s.clock.mono += .15
    s.node.watch_once()
    assert s.node.gate.fault == 'watchdog_timeout' and s.commands()[-1] == [1500, 1500]


def test_ppo_late_message_is_dropped_not_fatal(ppo):
    s = ppo()
    assert s.run(3, until=actuating)
    s.node.subscriptions[ODOM](odom(s.clock.ros() - 1.))
    assert s.node.monitor.streams['odometry'].rejected == 1
    assert s.run(.2) and s.node.gate.fault is None and s.status()['status'] == 'actuating'


def test_ppo_parameters_force_pulled_once_per_connection(ppo):
    s = ppo()
    s.run(11)
    pulls = [r.force_pull for r in s.node.pull_client.requests]
    assert pulls[0] is True and len(pulls) >= 2 and not any(pulls[1:])


def test_ppo_shadow_mode_never_faults_on_gaps(ppo):
    s = ppo(mode='shadow', enable_actuators=False)
    s.run(2)
    s.gap(8.)
    s.run(10)
    assert s.node.gate.fault is None and s.status()['status'] == 'shadow'


def test_ppo_runs_on_the_packaged_alberto_controller_profile(ppo):
    import yaml
    from blueboat_ppo_controller import boat_profile
    settings = yaml.safe_load((boat_profile.BUNDLE / 'controller.yaml').read_text())['blueboat_ppo_controller']['ros__parameters']
    if 'telemetry_hold_s' not in settings:
        pytest.skip('this checkout has an older Alberto profile without gap settings')
    settings = {k: v for k, v in settings.items() if k not in ('mode', 'enable_actuators', 'deployment_target', 'calibration_file', 'model_directory')}
    s = ppo(**settings)
    assert s.run(3, until=actuating)
    s.gap(.23)                                   # the measured short gap: held, no pause
    s.run(.6)
    assert 'telemetry_paused' not in s.statuses(last=40) and s.node.gate.fault is None
    s.gap(1.1)                                   # the measured long pause: neutral, then resume
    assert s.run(4, until=lambda s: 'telemetry_paused' in s.statuses(last=5))
    assert s.run(4, until=actuating) and s.node.gate.fault is None


def test_old_live_controller_config_is_refused_with_instructions(ppo):
    with pytest.raises(ValueError, match='update-gap-settings'):
        ppo(telemetry_timeout_s=1.25)
