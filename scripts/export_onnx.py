import hashlib
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import helper as h, numpy_helper as nh, TensorProto
import onnxruntime as ort

root = Path(__file__).resolve().parents[1]
weights = np.load(root / 'policy/checkpoints/actor_weights.npz')
contract = json.loads((root / 'policy/inference/observation_contract.json').read_text())
config = contract['algorithm']
assert not any(config[k] for k in ['policy_joint_history', 'policy_body_history', 'policy_symmetric_actor', 'policy_privileged_observations'])
nodes, constants = [], []


def constant(value, dtype=np.float32):
    name = 'constant_' + str(len(constants))
    constants.append(nh.from_array(np.asarray(value, dtype=dtype), name))
    return name


def op(kind, *inputs, **attributes):
    name = 'node_' + str(len(nodes))
    nodes.append(h.make_node(kind, list(inputs), [name], name=name, **attributes))
    return name


def dense(value, name, weightnorm=None):
    kernel = weights[name + '/kernel']
    if weightnorm is not None:
        scale = weights[f'core_weightnorm_{weightnorm}/{name}/kernel/scale']
        kernel = kernel * (1 / np.sqrt(np.sum(kernel * kernel, axis=0, keepdims=True) + 1e-12)) * scale
    return op('Add', op('MatMul', value, constant(kernel)), constant(weights[name + '/bias']))


def layernorm(value, name):
    mean = op('ReduceMean', value, axes=[-1], keepdims=1)
    variance = op('Sub', op('ReduceMean', op('Mul', value, value), axes=[-1], keepdims=1), op('Mul', mean, mean))
    variance = op('Max', variance, constant(0.))
    normalized = op('Div', op('Sub', value, mean), op('Sqrt', op('Add', variance, constant(1e-6))))
    return op('Add', op('Mul', normalized, constant(weights[name + '/scale'])), constant(weights[name + '/bias']))


def gelu(value):
    cube = op('Mul', op('Mul', value, value), value)
    inner = op('Mul', constant(np.sqrt(2 / np.pi)), op('Add', value, op('Mul', constant(.044715), cube)))
    return op('Mul', op('Mul', constant(.5), value), op('Add', constant(1.), op('Tanh', inner)))


joints = op('Reshape', op('Gather', 'observation', constant(np.arange(110), np.int64), axis=1), constant([-1, 2, 55], np.int64))
encoding_input = op('Gather', joints, constant(np.r_[0:50, 51:55], np.int64), axis=2)
state = op('Gather', joints, constant(np.arange(51, 55), np.int64), axis=2)
present = op('Gather', joints, constant([50], np.int64), axis=2)
latent = dense(gelu(layernorm(dense(encoding_input, 'encoder_dense_0'), 'encoder_layernorm')), 'encoder_dense_1')
routing = op('Clip', op('Tanh', latent), constant(-1 + config['stability_epsilon']), constant(1 - config['stability_epsilon']))
temperature = np.exp(weights['joint_log_softmax_temperature']) + config['softmax_temperature_min']
exponential = op('Exp', op('Div', routing, constant(temperature)))
mask = op('Div', exponential, op('Add', op('ReduceSum', exponential, constant([-1], np.int64), keepdims=1), constant(config['stability_epsilon'])))
state_latent = op('Mul', gelu(dense(state, 'encoder_latent_state')), present)
masked = op('Mul', op('Unsqueeze', mask, constant([-1], np.int64)), op('Unsqueeze', state_latent, constant([-2], np.int64)))
pooled = op('Reshape', op('ReduceSum', masked, constant([1], np.int64), keepdims=0), constant([-1, 1024], np.int64))
general = op('Gather', 'observation', constant(contract['general_indices'], np.int64), axis=1)
value = op('Concat', pooled, general, axis=1)
for index in range(5):
    value = dense(value, f'core_dense_{index}', weightnorm=index)
    if index == 0:
        value = layernorm(value, 'core_layernorm')
    if index < 4:
        value = gelu(value)
actions = op('ReduceSum', op('Mul', op('Unsqueeze', value, constant([1], np.int64)), mask), constant([-1], np.int64), keepdims=0)
actions = op('Clip', actions, constant(-config['policy_mean_abs_clip']), constant(config['policy_mean_abs_clip']))
nodes.append(h.make_node('Identity', [actions], ['actions']))
graph = h.make_graph(nodes, 'BlueBoat_URMA2_deterministic_actor', [h.make_tensor_value_info('observation', TensorProto.FLOAT, ['batch', 289])], [h.make_tensor_value_info('actions', TensorProto.FLOAT, ['batch', 2])], constants)
model = h.make_model(graph, opset_imports=[h.make_opsetid('', 17)], producer_name='blueboat verified export', ir_version=9)
onnx.checker.check_model(model)
path = root / 'policy/inference/blueboat_actor.onnx'
onnx.save(model, path)
options = ort.SessionOptions()
options.intra_op_num_threads = 1
session = ort.InferenceSession(str(path), sess_options=options, providers=['CPUExecutionProvider'])
reference = np.load(root / 'tests/reference_vectors.npz')
actual = session.run(None, {'observation': reference['observations']})[0]
error = np.abs(actual - reference['actions'])
np.testing.assert_allclose(actual, reference['actions'], atol=2e-5, rtol=2e-5)
report = dict(samples=len(actual), max_abs_raw_action_error=float(error.max()), mean_abs_raw_action_error=float(error.mean()), atol=2e-5, rtol=2e-5, source_checkpoint_sha256=contract['checkpoint_sha256'], onnx_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), onnx_bytes=path.stat().st_size, opset=17, ir_version=9, runtime=ort.__version__, passed=True)
(root / 'policy/inference/equivalence.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
