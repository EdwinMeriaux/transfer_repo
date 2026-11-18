"""Simple thrust check: T200 lookup, expectations, ESC-PWM trim and step safety, without ROS."""
import json
import math

import numpy as np
import pytest

from blueboat_ppo_controller import thrust_check as tc


def test_t200_lookup_at_16v_matches_the_published_sheet():
    table = tc.T200(16.)
    assert table.offset(0) == 0
    assert 135 <= table.offset(10) <= 145            # sheet: +10 N near 1639 us
    assert 175 <= table.offset(15) <= 185            # +15 N near 1679 us
    assert 205 <= table.offset(-15) <= 215           # -15 N near 1291 us: reverse is weaker
    assert table.force_at(table.offset(12), 1) == pytest.approx(12, abs=.2)
    with pytest.raises(ValueError):
        tc.T200(25.)


def test_voltage_between_sheets_interpolates():
    assert tc.T200(15.).offset(10) == pytest.approx((tc.T200(14.).offset(10) + tc.T200(16.).offset(10)) / 2, abs=3)


def test_expectations_come_from_the_training_model():
    assert tc.expected(10, 10)[:2] == (1.406, 0.)
    assert tc.expected(10, 0)[:2] == (.910, -.048)                    # mirror of (0, 10)
    surge, yaw, source = tc.expected(15, 15)
    assert 1.7 < surge < 1.9 and 'interpolated' in source
    # The fitted drag reproduces the table rows it was fitted to within 3%.
    for force, speed in ((20, 1.406), (50, 2.409), (-30, -1.793)):
        assert tc.speed_for_total_thrust(force) == pytest.approx(speed, rel=.03)


def profile():
    return json.loads(tc.PACKAGE_PROFILE.read_text())


class FakeAutopilot:
    """RC input -> ESC PWM like ArduPilot with trims 1510, a 30 us deadzone, reversed SERVO3
    and reverse output boosted (stand-in for MOT_THST_ASYM): unknown to the tool."""
    def __init__(self, interface):
        self.interface = interface

    def esc(self, i, rc):
        delta = rc - 1500
        throttle = 0. if abs(delta) <= 30 else (abs(delta) - 30) / 370 * math.copysign(1, delta)
        throttle *= 1.25 if throttle < 0 else 1.
        reversed_output = (-1, 1)[i]           # left = SERVO3 reversed, right = SERVO1
        return int(round(1510 + reversed_output * throttle * 390))


class FakeRig:
    def __init__(self, interface, thrust_scale=1., disarm_at=None):
        self.interface, self.scale, self.disarm_at = interface, thrust_scale, disarm_at
        self.autopilot = FakeAutopilot(interface)
        self.now, self.surge, self.yaw = 0., 0., 0.
        self.rc = list(interface.neutral)
        self.esc = [1510, 1510]
        self.t = dict(rcout=-1.)
        self.published = []
        self.table = tc.T200(16.)

    def spin(self, seconds):
        steps = max(1, int(round(seconds / .01)))
        for _ in range(steps):
            self.now += seconds / steps
            if self.now - self.t['rcout'] >= .2:          # 5 Hz RCOut
                self.esc = [self.autopilot.esc(i, self.rc[i]) for i in range(2)]
                self.t['rcout'] = self.now
            forces = [self.scale * self.table.force_at(abs(e - 1500), (-1, 1)[i] * np.sign(e - 1500))
                      if abs(e - 1500) > 28 else 0. for i, e in enumerate(self.esc)]
            total = forces[0] + forces[1]
            self.surge += (total - math.copysign(tc.drag(self.surge), self.surge)) / 25. * (seconds / steps)
            self.yaw += ((forces[1] - forces[0]) * .3 - 8. * self.yaw) / 4. * (seconds / steps)

    def blocker(self):
        return 'boat is disarmed' if self.disarm_at is not None and self.now >= self.disarm_at else None

    def motion(self):
        return self.surge, self.yaw

    def servo_outputs(self):
        return [self.esc[0], self.esc[1]]

    def voltage(self):
        return 16.

    def publish(self, pwm):
        self.published.append((round(self.now, 3), None if pwm is None else list(pwm)))
        if pwm is not None:
            self.rc = list(pwm)

    def release(self):
        for _ in range(6):
            self.publish(self.interface.neutral)
            self.spin(.05)
        self.publish(None)


@pytest.fixture
def clock(monkeypatch):
    rigs = []
    monkeypatch.setattr(tc.time, 'monotonic', lambda: rigs[0].now if rigs else 0.)
    return rigs


def test_trim_reaches_the_esc_pwm_the_table_needs(clock):
    interface = tc.Interface(profile())
    rig = FakeRig(interface)
    clock.append(rig)
    result = tc.run_step(rig, tc.T200(16.), 'fwd10', 10., 10., 15., 3., 2., lambda *a: None)
    assert 'stopped' not in result
    target = tc.T200(16.).offset(10)
    assert all(abs(reached - target) <= 6 for reached in result['esc_pwm_offset_reached'])
    assert result['learned_servo_forward_sign'] == [-1, 1]            # SERVO3 reversed, learned live
    assert result['input_pwm'][0] != 1500 + target                   # deadzone needed extra input


def test_a_boat_with_less_thrust_reports_the_scale(clock):
    interface = tc.Interface(profile())
    rig = FakeRig(interface, thrust_scale=.8)
    clock.append(rig)
    result = tc.run_step(rig, tc.T200(16.), 'fwd10', 10., 10., 15., 3., 2., lambda *a: None)
    assert result['settled'] and result['difference_pct'] < -5
    assert result['thrust_scale'] == pytest.approx(.8, abs=.06)
    lines, summary, advice = tc.verdict([result])
    assert summary['forward'] == pytest.approx(.8, abs=.06) and 'fwd10' in lines[0]


def test_disarm_releases_immediately_without_neutral(clock):
    interface = tc.Interface(profile())
    rig = FakeRig(interface, disarm_at=3.)
    clock.append(rig)
    result = tc.run_step(rig, tc.T200(16.), 'fwd10', 10., 10., 15., 3., 2., lambda *a: None)
    assert result['stopped'] == 'boat is disarmed'
    assert rig.published[-1][1] is None and rig.published[-2][1] != interface.neutral


def test_speed_cap_ends_the_step_through_neutral(clock):
    interface = tc.Interface(profile())
    rig = FakeRig(interface)
    clock.append(rig)
    result = tc.run_step(rig, tc.T200(16.), 'fwd15', 15., 15., 15., 1., 2., lambda *a: None)
    assert result['stopped'].startswith('limit')
    values = [p for _, p in rig.published]
    assert values[-1] is None and values[-2] == interface.neutral


def test_wrong_way_esc_is_caught(clock):
    data = profile()
    data['right']['servo_forward_pwm_sign'] = -1                     # wrong on purpose
    interface = tc.Interface(data)
    rig = FakeRig(interface)
    clock.append(rig)
    result = tc.run_step(rig, tc.T200(16.), 'right10', 0., 10., 15., 3., 2., lambda *a: None)
    assert 'wrong way' in result['stopped']


def test_refuses_long_override_timeout():
    good = {'RC_OVERRIDE_TIME': .3, 'RC_OPTIONS': 37., 'RC1_MIN': 1100., 'RC1_MAX': 1901., 'RC3_MIN': 1100., 'RC3_MAX': 1900.}
    assert tc.parameter_problem(good, [1, 3]) is None
    assert 'RC_OVERRIDE_TIME=3' in tc.parameter_problem(dict(good, RC_OVERRIDE_TIME=3.), [1, 3])


def test_list_runs_without_ros(capsys):
    assert tc.main(['list']) == 0
    out = capsys.readouterr().out
    assert 'spin10' in out and 'fwd15' in out and 'interpolated' in out
