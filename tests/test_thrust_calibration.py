import copy
import hashlib
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from blueboat_ppo_controller.calibration_analysis import (
    AnalysisSettings, analyze, corrected_acceleration, estimate_trial, fit_lookup, isotonic, read_csv, settings_from_session)
from blueboat_ppo_controller.calibration_synthetic import generate
from blueboat_ppo_controller.motor_adapter import MotorAdapter
from blueboat_ppo_controller.motor_interface import MotorInterface
from blueboat_ppo_controller.thrust_calibration import PulsePlan, RestCheck, StreamCheck
from blueboat_ppo_controller.thrust_lookup import ThrustLookup


@pytest.fixture(scope='module')
def synthetic(tmp_path_factory):
    path = tmp_path_factory.mktemp('synthetic')/'samples.csv'
    generate(path)
    rows = read_csv(path)
    groups = {}
    for row in rows:
        groups.setdefault(row['trial'], []).append(row)
    settings = AnalysisSettings(42.5)
    trials = [estimate_trial(group, settings) for group in groups.values()]
    return path, rows, list(groups.values()), trials


def test_plan_is_bounded_left_right_symmetric_and_reverse():
    plan = PulsePlan([40], repeats=1, baseline_s=1., pulse_s=.4, recovery_s=1.)
    assert plan.step(0., [1500, 1510])[1:3] == ('baseline', [1500, 1510])
    assert plan.step(1.01, [1500, 1510])[1:3] == ('pulse', [1540, 1510])
    assert plan.step(3.41, [1500, 1510])[2] == [1500, 1550]
    assert plan.step(5.81, [1500, 1510], [1, -1])[2] == [1540, 1470]
    assert plan.step(9., [1500, 1510])[1] == 'complete'
    reverse = PulsePlan([-40], sides=['left'], repeats=1)
    reverse.step(0., [1500, 1500])
    assert reverse.step(2.01, [1500, 1500])[2] == [1460, 1500]


@pytest.mark.parametrize('kwargs', [dict(deltas=[0]), dict(deltas=[61]), dict(repeats=0), dict(sides=['both']),
                                    dict(pulse_s=2.), dict(baseline_s=float('nan'))])
def test_invalid_plans(kwargs):
    with pytest.raises(ValueError):
        PulsePlan(**kwargs)


def test_source_and_receipt_clock_validation():
    check = StreamCheck()
    for i in range(110):
        check.accept(100+i*.01, 100+i*.01+.002, 5+i*.01)
    report = check.report(6.1, 101.1)
    assert report['fresh'] and report['rate_hz'] == pytest.approx(100)
    assert not check.report(6.5, 101.5)['fresh']
    with pytest.raises(ValueError, match='repeated'):
        check.accept(101.09, 101.10, 6.1)
    assert not check.report(6.1, 101.1)['fresh']
    with pytest.raises(ValueError, match='future'):
        StreamCheck().accept(102., 100., 1.)


def test_rest_requires_a_continuous_fresh_quiet_interval():
    rest=RestCheck(duration_s=1.,max_gap_s=.06)
    rest.observe(0.,True)
    assert not rest.ready(0.)
    for now in np.arange(.01,1.02,.01):
        rest.observe(float(now),True)
    assert rest.ready(1.02)
    assert not rest.ready(1.1)
    rest.observe(1.11,True)
    assert not rest.ready(1.11)
    rest.observe(1.12,False)
    assert rest.since is None


def test_synthetic_estimation_separate_directions_and_motors(synthetic):
    _, _, _, trials = synthetic
    assert all(t['accepted'] for t in trials), [(t['trial'], t['reason']) for t in trials if not t['accepted']]
    lookup = fit_lookup(trials, AnalysisSettings(42.5), dict(left=1500, right=1500))
    assert len(lookup['curves']) == 6
    assert not lookup['verified'] and not lookup['independent_mapping_reviewed']
    for key, curve in lookup['curves'].items():
        multiplier = 2 if key.startswith('symmetric') else 1
        np.testing.assert_allclose(curve['force_magnitude_N'], np.array([3.2, 7.2])*multiplier, rtol=.04)
        assert all(x > 0 for x in curve['sigma_force_N'])


def test_configurable_mass_and_added_mass(synthetic):
    group = synthetic[2][0]
    a = estimate_trial(group, AnalysisSettings(42.5))
    b = estimate_trial(group, AnalysisSettings(85., mass_std_kg=2.))
    c = estimate_trial(group, AnalysisSettings(42.5, added_mass_kg=10.))
    assert b['effective_force_N'] == pytest.approx(2*a['effective_force_N'])
    assert c['effective_force_N'] == pytest.approx(a['effective_force_N']*52.5/42.5)
    assert b['sigma_force_N'] > a['sigma_force_N']
    for mass in (0., -1., float('nan')):
        with pytest.raises(ValueError):
            AnalysisSettings(mass)


def test_analysis_uses_the_measured_ros_mass_without_a_fixed_default():
    metadata=dict(settings=dict(mass_kg=57.3,accelerometer_semantics='linear_acceleration'))
    settings=settings_from_session(metadata)
    assert settings.mass_kg==57.3 and settings.accelerometer_semantics=='linear_acceleration'
    assert settings_from_session(metadata,mass_kg=60.).mass_kg==60.
    with pytest.raises(ValueError,match='measured mass'):
        settings_from_session()
    with pytest.raises(ValueError):
        settings_from_session(dict(settings=dict(mass_kg=0.)))


@pytest.mark.parametrize('change,reason', [
    (lambda r: r[25].update(valid='0'), 'invalid_telemetry'),
    (lambda r: [x.update(mode='dry_run') for x in r], 'no_powered_trial'),
    (lambda r: r[25].update(attitude_stamp_s=str(float(r[25]['imu_stamp_s'])-.15)), 'timestamp_skew'),
    (lambda r: r[25].update(imu_stamp_s=r[24]['imu_stamp_s']), 'source_time_gap'),
    (lambda r: [x.update(wx='1.') for x in r], 'rotation_disturbance'),
    (lambda r: [x.update(vx='.5') for x in r], 'baseline_not_at_rest'),
    (lambda r: [x.update(vx='0.') for x in r], None),
    (lambda r: [x.update(qw='0.') for x in r], 'invalid_quaternion'),
    (lambda r: r[25].update(velocity_frame='map'), 'frame'),
    (lambda r: [x.update(az='0.') for x in r], 'gravity_semantics'),
    (lambda r: [x.update(command_right_pwm='1540') for x in r if x['phase']=='pulse'], 'inactive_motor'),
])
def test_invalid_trials_are_rejected(synthetic, change, reason):
    rows = copy.deepcopy(synthetic[2][0])
    change(rows)
    result = estimate_trial(rows, AnalysisSettings(42.5))
    if reason:
        assert not result['accepted'] and reason in result['reason']
    else:
        # With a configurable tighter disagreement tolerance, a frozen velocity fails.
        result = estimate_trial(rows, AnalysisSettings(42.5, max_velocity_disagreement_m_s2=.02))
        assert not result['accepted'] and result['reason'] == 'velocity_imu_disagreement'


def test_gravity_bias_and_rotating_lever_arm_correction():
    rows = []
    offset = np.array([.3, .2, .1])
    for t in np.arange(0., 1., .01):
        w, alpha = np.array([0., 0., .2*t]), np.array([0., 0., .2])
        rotation = Rotation.from_euler('xyz', [.05, -.07, .1*t*t])
        a = np.array([.15, 0., 0.])-rotation.inv().apply([0,0,-9.80665]) + np.cross(alpha, offset)+np.cross(w,np.cross(w,offset))
        row = dict(imu_stamp_s=t)
        for prefix, axes, values in [('a','xyz',a), ('w','xyz',w), ('q','xyzw',rotation.as_quat())]:
            row.update({prefix+axis:value for axis,value in zip(axes,values)})
        rows.append(row)
    acceleration, _, _, _ = corrected_acceleration(rows, AnalysisSettings(42.5, imu_offset_flu_m=tuple(offset)))
    np.testing.assert_allclose(acceleration, np.tile([.15,0.,0.],(len(rows),1)), atol=1e-10)


def test_monotonic_fit_and_unmeasured_directions(synthetic):
    np.testing.assert_array_equal(isotonic([1,3,2,4], [1]*4), [1,2.5,2.5,4])
    few = fit_lookup(synthetic[3][:1], AnalysisSettings(42.5), dict(left=1500,right=1500))
    assert not few['curves'] and len(few['unmeasured']) == 6


def test_inverse_deadzone_clipping_rejection_and_gaps(synthetic):
    document = fit_lookup(synthetic[3], AnalysisSettings(42.5), dict(left=1500,right=1500))
    lookup = ThrustLookup(document)
    with pytest.raises(ValueError, match='reviewed'):
        lookup.inverse(5., 'left')
    for side in ('left','right'):
        for force in (-5., 5.):
            pwm, applied = lookup.inverse(force, side, allow_unreviewed=True)
            assert 1440 <= pwm <= 1560 and applied == force
        assert lookup.inverse(.01, side, allow_unreviewed=True) == (1500, 0.)
        with pytest.raises(ValueError, match='measured region'):
            lookup.inverse(100.,side,allow_unreviewed=True)
        assert lookup.inverse(100.,side,'clip',True)[0] == 1560
        assert lookup.inverse(.1,side,'clip',True) == (1500,0.)
        with pytest.raises(ValueError):
            lookup.inverse(.1,side,allow_unreviewed=True)
    with pytest.raises(ValueError):
        lookup.inverse(float('nan'),'left',allow_unreviewed=True)
    document['curves']['left_forward']['unmeasured_pwm_gaps'] = [[1540,1560]]
    with pytest.raises(ValueError, match='unmeasured PWM'):
        ThrustLookup(document).inverse(5.,'left',allow_unreviewed=True)


def test_full_analysis_writes_plots_and_evidence_hash(synthetic, tmp_path):
    path, _, _, _ = synthetic
    trials, lookup = analyze(path, tmp_path/'report', AnalysisSettings(42.5), dict(left=1500,right=1500))
    assert sum(t['accepted'] for t in trials) == 36
    for name in ('timeseries.png','lookup.png','lookup.json','trials.json'):
        assert (tmp_path/'report'/name).stat().st_size > 100
    assert len(lookup['evidence_sha256']) == 64


def test_motor_interface_does_not_require_a_force_curve(calibration):
    config = json.loads(calibration.read_text())
    for side in ('left','right'):
        config[side].pop('force_pwm_points')
    calibration.write_text(json.dumps(config))
    assert MotorInterface(calibration,'sitl').neutral() == [1500,1510]
    with pytest.raises(KeyError):
        MotorAdapter(calibration,'sitl')


def test_effective_lookup_in_ppo_motor_adapter_is_opt_in_and_bound_to_evidence(synthetic, calibration):
    # Fake identities and data are confined to pytest's temporary directory.
    config=json.loads(calibration.read_text())
    for side in ('left','right'):
        config[side]['forward_pwm_sign']=1
    data=fit_lookup(synthetic[3],AnalysisSettings(42.5),dict(left=1500,right=1500))
    assert data['paired_validation']['passed']
    data['neutral_pwm']['right']=1510
    for direction in ('forward','reverse'):
        curve=data['curves']['right_'+direction]
        curve['pwm']=[p+10 for p in curve['pwm']]
    data['metadata']=dict(settings=dict(mode='powered'),motor_interface=copy.deepcopy(config))
    data['evidence_sha256']=hashlib.sha256(synthetic[0].read_bytes()).hexdigest()
    evidence=calibration.parent/'test-samples.csv'
    evidence.write_bytes(synthetic[0].read_bytes())
    config.update(evidence_file=evidence.name,evidence_sha256=data['evidence_sha256'],
                  effective_lookup_file='lookup.json',maximum_abs_force_N=6.)
    lookup_path=calibration.parent/'lookup.json'
    def save():
        lookup_path.write_text(json.dumps(data))
        config['effective_lookup_sha256']=hashlib.sha256(lookup_path.read_bytes()).hexdigest()
        calibration.write_text(json.dumps(config))
    save()
    with pytest.raises(ValueError,match='identifiability'):
        MotorAdapter(calibration,'sitl')
    config.update(effective_mapping_accepted=True,independent_effective_mapping_verified=True)
    save()
    with pytest.raises(ValueError,match='unreviewed'):
        MotorAdapter(calibration,'sitl')
    data['independent_mapping_reviewed']=True
    data['verified']=True
    save()
    adapter=MotorAdapter(calibration,'sitl')
    pwm,applied=adapter.convert([5.,-5.])
    assert 1540 <= pwm[0] <=1560 and 1450 <= pwm[1] <=1470
    np.testing.assert_array_equal(applied,[5.,-5.])
    with pytest.raises(ValueError,match='measured region'):
        adapter.convert([.1,0.])
    config['maximum_abs_force_N']=25.
    save()
    with pytest.raises(ValueError,match='measured region'):
        MotorAdapter(calibration,'sitl').convert([25.,0.])
    data['metadata']['synthetic']=True
    save()
    with pytest.raises(ValueError,match='synthetic'):
        MotorAdapter(calibration,'sitl')


@pytest.mark.parametrize('field,value,reason', [
    ('output_left_pwm','1500','did_not_respond'),
    ('output_right_pwm','1600','idle_motor'),
    ('output_stamp_s','0.','actual_output'),
])
def test_actual_output_failures_reject_trial(synthetic,field,value,reason):
    rows=copy.deepcopy(synthetic[2][0])
    for row in rows:
        if row['phase']=='pulse':
            row[field]=value
    trial=estimate_trial(rows,AnalysisSettings(42.5))
    assert not trial['accepted'] and reason in trial['reason']
