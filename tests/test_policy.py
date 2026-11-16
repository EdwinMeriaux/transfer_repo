import hashlib
import json
from pathlib import Path
import time
import zipfile

import numpy as np
import pytest
from blueboat_ppo_controller.policy_runtime import PolicyRuntime, thrust_action
from blueboat_ppo_controller.generalist_policy_runtime import GeneralistPolicyRuntime
from blueboat_ppo_controller.observation_adapter import ObservationAdapter

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def runtime():
    return PolicyRuntime()


@pytest.fixture(scope='module')
def generalist():
    return GeneralistPolicyRuntime()


def test_original_checkpoint_integrity():
    p = ROOT / 'policy/checkpoints/blueboat_coupled_4m_mac.model'
    assert hashlib.sha256(p.read_bytes()).hexdigest() == '43213ea3bc2219d93acf991fdb74c02a559c1d2ed77a8af266138474eb1269e4'
    with zipfile.ZipFile(p) as z:
        assert z.testzip() is None
        assert json.loads(z.read('training_progress.json'))


def test_export_matches_original_jax(runtime):
    reference = np.load(ROOT / 'tests/reference_vectors.npz')
    actions = np.array([runtime.action(row) for row in reference['observations']])
    np.testing.assert_allclose(actions, reference['actions'], atol=2e-5, rtol=2e-5)
    assert np.max(np.abs(thrust_action(actions[0]))) <= 25


def test_adapter_matches_original_gazebo_observations(runtime):
    reference = np.load(ROOT / 'tests/reference_vectors.npz')
    adapter = ObservationAdapter(runtime.contract)
    values = np.array([adapter.from_world(row[:4], row[4:7], row[7:10], row[10:12], row[12:14]) for row in reference['inputs']])
    np.testing.assert_allclose(values, reference['observations'], atol=2e-7)


def test_units_frames_and_previous_raw_action(runtime):
    adapter = ObservationAdapter(runtime.contract)
    q = [0, 0, np.sqrt(.5), np.sqrt(.5)]
    obs = adapter.from_world(q, [0, 1, 0], [0, 0, .5], [.4, -.2], [2., -3.])
    np.testing.assert_allclose(obs[[220, 221, 222]], [.1, 0., 0.], atol=1e-7)
    np.testing.assert_allclose(obs[[227, 228, 229]], [0., 0., .01], atol=1e-7)
    np.testing.assert_allclose(obs[[233, 234, 235, 236]], [.4, 0., 0., -.2], atol=1e-7)
    np.testing.assert_allclose(obs[[253, 254, 255]], [0., 0., -1.], atol=1e-7)
    np.testing.assert_allclose(obs[[53, 108]], [.2, -.3], atol=1e-7)
    assert len(runtime.contract['general_indices']) == 34


@pytest.mark.parametrize('q,v,w', [([0,0,0,0],[0,0,0],[0,0,0]), ([0,0,0,1],[np.nan,0,0],[0,0,0]), ([0,0,0,1],[0,0,0],[0,0,6])])
def test_invalid_observations_rejected(runtime, q, v, w):
    with pytest.raises(ValueError):
        ObservationAdapter(runtime.contract).from_body(q, v, w, [0,0], [0,0])


def test_asymmetric_force_scaling():
    np.testing.assert_array_equal(thrust_action([2,-2]), [25,-15])
    np.testing.assert_array_equal(thrust_action([.4,-.4]), [10,-6])
    with pytest.raises(ValueError):
        thrust_action([np.nan,0])


def test_inference_timing(runtime):
    values = np.asarray(runtime.contract['template'],dtype=np.float32)
    samples=[]
    for i in range(500):
        t=time.perf_counter()
        runtime.action(values)
        samples.append(time.perf_counter()-t)
    assert np.quantile(samples,.99) < .02


def test_generalist_assets_schema_and_known_first_action(generalist):
    action = generalist.action_from_body(
        [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], [.5, 0], [0, 0])
    np.testing.assert_allclose(
        action, [.31017452, .31017452], atol=2e-6, rtol=2e-6)
    np.testing.assert_allclose(
        generalist.actuator_controls(action),
        [2.326309, 2.326309], atol=2e-5, rtol=2e-5)
    assert generalist.last_observation.shape == (256,)


def test_generalist_history_reset_is_exact(generalist):
    def sequence():
        generalist.reset_history()
        previous = np.zeros(2)
        actions = []
        for _ in range(12):
            previous = generalist.action_from_body(
                [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], [.5, 0], previous)
            actions.append(previous.copy())
        return np.asarray(actions)
    np.testing.assert_array_equal(sequence(), sequence())


def test_generalist_command_force_and_yaw_contract(generalist):
    np.testing.assert_array_equal(generalist.fix_command([.09, 0]), [0, 0, 0, 0])
    np.testing.assert_allclose(
        generalist.fix_command([2, 2]), [1, 0, 0, .9836066], atol=1e-7)
    np.testing.assert_array_equal(
        generalist.actuator_controls([-10, 10]), [-15, 25])
    generalist.reset_history()
    left = generalist.action_from_body(
        [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], [0, .4], [0, 0])
    generalist.reset_history()
    right = generalist.action_from_body(
        [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], [0, -.4], [0, 0])
    np.testing.assert_allclose(left, right[::-1], atol=2e-6, rtol=2e-6)
    assert left[0] < left[1]


@pytest.mark.parametrize('q,v,w', [
    ([0,0,0,0], [0,0,0], [0,0,0]),
    ([0,0,0,1], [np.nan,0,0], [0,0,0]),
    ([0,0,0,1], [0,0,0], [0,np.inf,0]),
])
def test_generalist_rejects_invalid_telemetry(generalist, q, v, w):
    with pytest.raises(ValueError):
        generalist.action_from_body(q, v, w, [0, 0], [0, 0])


def test_generalist_inference_timing(generalist):
    samples = []
    previous = np.zeros(2)
    generalist.reset_history()
    for _ in range(500):
        started = time.perf_counter()
        previous = generalist.action_from_body(
            [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], [.5, 0], previous)
        samples.append(time.perf_counter() - started)
    assert np.quantile(samples, .99) < .02
