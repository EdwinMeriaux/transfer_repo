import hashlib
import json
from pathlib import Path
import time

import numpy as np
import onnxruntime as ort


class PolicyRuntime:
    def __init__(self, directory=None):
        self.directory = Path(directory) if directory else Path(__file__).parent / 'assets'
        manifest = json.loads((self.directory / 'manifest.json').read_text())
        for name, digest in manifest['files'].items():
            if hashlib.sha256((self.directory / name).read_bytes()).hexdigest() != digest:
                raise ValueError('Policy asset checksum mismatch: ' + name)
        self.contract = json.loads((self.directory / 'observation_contract.json').read_text())
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(self.directory / 'blueboat_actor.onnx'), sess_options=options, providers=['CPUExecutionProvider'])
        self.elapsed_s = 0.
        self.action(np.asarray(self.contract['template'], dtype=np.float32))


    def action(self, observation):
        values = np.asarray(observation, dtype=np.float32)
        if values.shape != (289,) or not np.isfinite(values).all():
            raise ValueError('Expected 289 finite normalized observation values')
        start = time.perf_counter()
        action = self.session.run(['actions'], {'observation': values[None]})[0][0]
        self.elapsed_s = time.perf_counter() - start
        if action.shape != (2,) or not np.isfinite(action).all():
            raise ValueError('Invalid actor output')
        return action


def thrust_action(action):
    values = np.asarray(action, dtype=np.float64)
    if values.shape != (2,) or not np.isfinite(values).all():
        raise ValueError('Expected two finite actions in left/right order')
    clipped = np.clip(values, -1., 1.)
    return clipped * np.where(clipped >= 0., 25., 15.)
