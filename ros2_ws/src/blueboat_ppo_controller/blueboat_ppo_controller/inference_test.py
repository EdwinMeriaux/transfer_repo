import argparse
import json
import time
import numpy as np
from .policy_runtime import PolicyRuntime, thrust_action
from .generalist_policy_runtime import GeneralistPolicyRuntime
from .observation_adapter import ObservationAdapter


def main():
    parser = argparse.ArgumentParser(description='Offline BlueBoat policy check; publishes nothing')
    parser.add_argument('--policy-backend', choices=('e274_6_5b', 'legacy'), default='e274_6_5b')
    args = parser.parse_args()
    timings = []
    if args.policy_backend == 'legacy':
        runtime = PolicyRuntime()
        observation = ObservationAdapter(runtime.contract).from_body(
            [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], [.4, .2], [0, 0])
        for _ in range(1000):
            start = time.perf_counter()
            action = runtime.action(observation)
            timings.append(time.perf_counter() - start)
        forces = thrust_action(action)
    else:
        runtime = GeneralistPolicyRuntime()
        previous = np.zeros(2, dtype=np.float32)
        for _ in range(1000):
            start = time.perf_counter()
            action = runtime.action_from_body(
                [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], [.4, .2], previous)
            timings.append(time.perf_counter() - start)
            previous = action
        forces = runtime.actuator_controls(action)
    print(json.dumps(dict(policy_backend=args.policy_backend,
        raw_action=action.tolist(), requested_port_starboard_N=forces.tolist(),
        inference_median_ms=1000 * float(np.median(timings)),
        inference_p99_ms=1000 * float(np.quantile(timings, .99)),
        actuator_output=False), indent=2))


if __name__ == '__main__':
    main()
