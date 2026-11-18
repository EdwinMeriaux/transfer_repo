"""Both packaged policies load, and u990 reproduces its own reference runtime."""
import shutil

import numpy as np
import pytest

pytest.importorskip('onnxruntime')
from blueboat_ppo_controller.generalist_policy_runtime import GeneralistPolicyRuntime  # noqa: E402

# From the model's reference runtime (blueboat_expert_u990/inference_onnx.py),
# which matched this runtime exactly over 2400 random steps with history.
U990_REFERENCE = {0: [1.8736885786056519, 3.3362700939178467], 5: [1.7104501724243164, 3.9658408164978027],
                  14: [0.8607968688011169, 3.6694469451904297], 15: [-2.9180586338043213, -3.5114548206329346],
                  29: [-3.590001106262207, -2.702822208404541]}


def test_default_model_is_unchanged():
    runtime = GeneralistPolicyRuntime()
    assert runtime.directory.name == 'e274_6_5b' and runtime.onnx_file == 'e274_6.5B.onnx'


def test_u990_matches_its_reference_runtime():
    runtime = GeneralistPolicyRuntime('blueboat_u990')
    assert runtime.onnx_file == 'blueboat_blueboat_u990.onnx' and runtime.joint_descriptions.shape == (1, 2, 57)
    previous, actions = np.zeros(2, np.float32), []
    for step in range(30):
        command = [.5, .3] if step < 15 else [-.4, -.5]
        velocity, omega = [.02 * step, .01, 0.], [0., 0., .01 * step]
        previous = runtime.action_from_body([0., 0., 0., 1.], velocity, omega, command, previous)
        actions.append(previous)
    for step, expected in U990_REFERENCE.items():
        np.testing.assert_allclose(actions[step], expected, atol=1e-5)
    np.testing.assert_allclose(runtime.actuator_controls(np.array([4., -3.])), [25., -15.])


def test_tampered_model_is_refused(tmp_path):
    source = GeneralistPolicyRuntime.__init__.__globals__['Path'](
        __import__('blueboat_ppo_controller').__file__).parent / 'assets' / 'blueboat_u990'
    copy = tmp_path / 'u990'
    shutil.copytree(source, copy)
    (copy / 'joint_descriptions.npy').write_bytes((copy / 'joint_descriptions.npy').read_bytes()[:-4] + b'\0\0\0\0')
    with pytest.raises(ValueError, match='checksum'):
        GeneralistPolicyRuntime(str(copy))
