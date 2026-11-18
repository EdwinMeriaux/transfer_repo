"""Make a SITL u990 shadow-test controller config from the container's own controller YAML.

    python3 sitl_config.py IN.yaml OUT.yaml [MODEL_DIR]

Keeps every existing SITL setting and only changes what the new ppo_node needs:
  - the u990 model (absolute path, so it works whether or not colcon installed the assets);
  - shadow mode, actuators off, deployment_target sitl;
  - the container's former local SITL edits, now settings:
      timestamp_future_tolerance_s and max_telemetry_skew_s >= telemetry_timeout_s, state_timeout_s >= 5;
  - telemetry_hold_s >= telemetry_timeout_s (the new gap handling refuses otherwise).
"""
import sys
from pathlib import Path

import yaml

DEFAULT_MODEL = Path('~/gz_ws/gazebo_blue_boat/ros2_ws/src/blueboat_ppo_controller/'
                     'blueboat_ppo_controller/assets/blueboat_u990').expanduser()


def main(source, destination, model=DEFAULT_MODEL):
    data = yaml.safe_load(Path(source).read_text())
    node = next(k for k, v in data.items() if isinstance(v, dict) and 'ros__parameters' in v)
    p = data[node]['ros__parameters']
    model = Path(model).expanduser().resolve()
    if not (model / 'manifest.json').is_file():
        sys.exit(f'no u990 model at {model} (did install.sh run?)')
    timeout = float(p.get('telemetry_timeout_s', .2))
    wanted = dict(
        mode='shadow', enable_actuators=False, deployment_target='sitl',
        policy_backend='e274_6_5b', model_directory=str(model),
        timestamp_future_tolerance_s=max(float(p.get('timestamp_future_tolerance_s', 0.)), timeout),
        max_telemetry_skew_s=max(float(p.get('max_telemetry_skew_s', .1)), timeout),
        state_timeout_s=max(float(p.get('state_timeout_s', 2.)), 5.),
        telemetry_hold_s=min(1.5, max(float(p.get('telemetry_hold_s', 0.)), 2 * timeout, .4)),
    )
    if wanted['telemetry_hold_s'] < timeout:
        sys.exit(f'telemetry_timeout_s={timeout} is above 1.5 s; lower it in {source}')
    for key, value in wanted.items():
        if p.get(key) != value:
            print(f'{key}: {p.get(key)!r} -> {value!r}')
        p[key] = value
    Path(destination).write_text(yaml.safe_dump(data, sort_keys=False))
    print('wrote ' + str(destination))


if __name__ == '__main__':
    if len(sys.argv) not in (3, 4):
        sys.exit(__doc__)
    main(*sys.argv[1:])
