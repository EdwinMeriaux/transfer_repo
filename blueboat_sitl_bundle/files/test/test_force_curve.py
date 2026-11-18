"""Force-to-PWM curve from a thrust check: end to end through a fake autopilot, without ROS."""
import csv
import json

import numpy as np
import pytest

from blueboat_ppo_controller import force_curve as fc
from blueboat_ppo_controller import motor_interface
from blueboat_ppo_controller import thrust_check as tc
from blueboat_ppo_controller.motor_adapter import MotorAdapter
from test_thrust_check import FakeAutopilot, FakeRig, profile


@pytest.fixture
def clock(monkeypatch):
    rigs = []
    monkeypatch.setattr(tc.time, 'monotonic', lambda: rigs[0].now if rigs else 0.)
    return rigs


def run_thrust_check(tmp_path, clock, scale=.9, steps=None, stop=None):
    """All thrust-check steps on the fake boat; writes the result JSON and CSV like the tool."""
    interface_path = tmp_path / 'motor_interface.json'
    interface_path.write_text(json.dumps(profile()))
    interface = tc.load_interface(interface_path)
    rig = FakeRig(interface, thrust_scale=scale)
    clock.append(rig)
    table = tc.T200(16.)
    rows, results = [], []

    def log(name, left, right, row):
        rows.append(dict(step=name, left_N=left, right_N=right, **row))
    for name in steps or tc.STEPS:
        left, right, seconds, _ = tc.STEPS[name]
        rig.surge = rig.yaw = 0.
        if name == stop:
            results.append(dict(step=name, left_N=left, right_N=right, stopped='limit: test'))
            continue
        results.append(tc.run_step(rig, table, name, left, right, seconds, 3., 2., log))
    csv_path, json_path = tmp_path / 'thrust-check-x.csv', tmp_path / 'thrust-check-x.json'
    with csv_path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    _, summary, advice = tc.verdict(results)
    json_path.write_text(json.dumps(dict(results=results, summary=summary, advice=advice, voltage=16.,
                                         interface=str(interface_path), csv=str(csv_path))))
    return json_path, interface_path


def adapter_for(interface_path, monkeypatch):
    """The real MotorAdapter curve checks and conversion, without the failsafe/identity checks."""
    def bare(self, path, target):
        self.config = json.loads(open(path).read())
        self.expected = self.config['expected_parameters']
    monkeypatch.setattr(motor_interface.MotorInterface, '__init__', bare)
    return MotorAdapter(interface_path, 'hardware')


def test_policy_newtons_become_the_thrust_the_training_model_expects(tmp_path, clock, monkeypatch):
    result, interface_path = run_thrust_check(tmp_path, clock, scale=.9)
    document, problems = fc.build(result, voltage=16., max_force=25.)
    assert problems == []
    curve_path = tmp_path / 'curve.json'
    curve_path.write_text(json.dumps(document))
    backup = fc.apply(curve_path, interface_path)
    assert backup.exists()
    adapter = adapter_for(interface_path, monkeypatch)
    autopilot, table = FakeAutopilot(None), tc.T200(16.)
    # The thrust check measured the boat at 0.9 of the T200 data, so the curve asks for more.
    assert document['thrust_scale']['forward'] == pytest.approx(.9, abs=.05)
    for force in (-15., -10., -6., -2.5, 2.5, 6., 10., 15., 20., 25.):
        pwm, applied = adapter.convert([force, force])
        for i in range(2):
            esc = autopilot.esc(i, pwm[i])
            direction = (-1, 1)[i] * np.sign(esc - 1500)
            real = .9 * table.force_at(abs(esc - 1500), direction)
            # ~1 us of PWM rounding on each side; scale error from the speed fit
            assert real == pytest.approx(force, abs=.5 + .05 * abs(force)), (force, i, pwm[i], esc)


def test_each_motor_gets_its_own_slope_in_both_directions(tmp_path, clock):
    result, _ = run_thrust_check(tmp_path, clock, scale=1.)
    document, problems = fc.build(result)
    assert problems == []
    maps = document['input_to_esc']
    assert all(maps[k]['slope_from'] == k.split('_')[0] for k in maps)   # rev10 + rev15: two reverse levels
    assert maps['left_forward']['slope_from'] == 'left'
    # Fake autopilot: forward 390/370 us per us, reverse x1.25 (asymmetry)
    assert maps['left_forward']['esc_us_per_input_us'] == pytest.approx(390 / 370, rel=.03)
    assert maps['left_reverse']['esc_us_per_input_us'] == pytest.approx(1.25 * 390 / 370, rel=.03)


def test_a_side_with_one_level_borrows_the_other_sides_slope(tmp_path, clock):
    result, _ = run_thrust_check(tmp_path, clock, scale=1.,
                                 steps=['spin10', 'spin15', 'fwd15', 'rev15', 'fwd10', 'left10', 'right10'])
    document, problems = fc.build(result)
    assert problems == []
    assert document['input_to_esc']['right_reverse']['slope_from'] == 'left'   # right reverse only in rev15


def test_default_cap_and_voltage(tmp_path, clock):
    result, _ = run_thrust_check(tmp_path, clock, scale=1.)
    at15, problems = fc.build(result)
    assert problems == [] and at15['voltage_V'] == 15.
    assert (at15['minimum_force_N'], at15['maximum_force_N']) == (-15., 15.)
    at16, _ = fc.build(result, voltage=16.)
    left15 = dict(map(tuple, at15['force_pwm_points']['left']))
    left16 = dict(map(tuple, at16['force_pwm_points']['left']))
    assert left15[15.] > left16[15.] and left15[-15.] < left16[-15.]     # lower voltage needs more PWM
    for side in fc.SIDES:
        assert fc.curve_problem(at15['force_pwm_points'][side], 1500) is None


def test_missing_reverse_step_is_refused(tmp_path, clock):
    result, _ = run_thrust_check(tmp_path, clock, steps=['left10', 'right10', 'fwd10', 'fwd15'])
    document, problems = fc.build(result)
    assert any('reverse' in p for p in problems)
    assert document['force_pwm_points'] == {}


def test_out_of_range_scale_is_refused(tmp_path, clock):
    result, _ = run_thrust_check(tmp_path, clock, scale=.4)
    _, problems = fc.build(result)
    assert any('thrust scale' in p for p in problems)


def test_unsettled_step_is_not_used(tmp_path, clock):
    result, _ = run_thrust_check(tmp_path, clock, scale=1.)
    csv_path = tmp_path / 'thrust-check-x.csv'
    rows = list(csv.DictReader(csv_path.open()))
    for row in rows:
        if row['step'] == 'fwd15':
            row['input_left'] = str(int(float(row['input_left'])) + (int(float(row['t']) * 10) % 2) * 20)
    with csv_path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    document, problems = fc.build(result)
    assert problems == []
    assert any('fwd15 left' in n and 'still moving' in n for n in document['notes'])
    assert all(p['step'] != 'fwd15' for p in document['input_to_esc']['left_forward']['points'])


def test_apply_refuses_another_interface(tmp_path, clock):
    result, interface_path = run_thrust_check(tmp_path, clock, scale=1.)
    document, _ = fc.build(result)
    curve_path = tmp_path / 'curve.json'
    curve_path.write_text(json.dumps(document))
    other = json.loads(interface_path.read_text())
    other['left']['neutral_pwm'] = 1510
    other_path = tmp_path / 'other.json'
    other_path.write_text(json.dumps(other))
    with pytest.raises(ValueError, match='neutral_pwm'):
        fc.apply(curve_path, other_path)
    other['left']['neutral_pwm'] = 1500
    other['effective_lookup_file'] = 'lookup.json'
    other_path.write_text(json.dumps(other))
    with pytest.raises(ValueError, match='effective_lookup_file'):
        fc.apply(curve_path, other_path)


def test_cli_build_and_apply(tmp_path, clock, capsys):
    result, interface_path = run_thrust_check(tmp_path, clock, scale=1.)
    output = tmp_path / 'curve.json'
    assert fc.main(['build', str(result), '--output', str(output)]) == 0
    out = capsys.readouterr().out
    assert 'above cap, not sent' in out and output.exists()
    assert fc.main(['apply', str(output), '--interface', str(interface_path)]) == 0
    applied = json.loads(interface_path.read_text())
    assert applied['maximum_force_N'] == 15. and len(applied['left']['force_pwm_points']) == len(fc.FORCES_N)
    assert applied['maximum_abs_force_N'] == 15.                      # for older MotorAdapter versions
    assert fc.main(['build', str(result), '--max-force', '30']) == 2
