import json
import time
import numpy as np
from .policy_runtime import PolicyRuntime, thrust_action
from .observation_adapter import ObservationAdapter


def main():
    runtime = PolicyRuntime()
    observation = ObservationAdapter(runtime.contract).from_body([0, 0, 0, 1], [0, 0, 0], [0, 0, 0], [.4, .2], [0, 0])
    timings = []
    for index in range(1000):
        start = time.perf_counter()
        action = runtime.action(observation)
        timings.append(time.perf_counter() - start)
    print(json.dumps(dict(raw_action=action.tolist(), requested_left_right_N=thrust_action(action).tolist(), inference_median_ms=1000 * float(np.median(timings)), inference_p99_ms=1000 * float(np.quantile(timings, .99)), actuator_output=False), indent=2))


if __name__ == '__main__':
    main()
