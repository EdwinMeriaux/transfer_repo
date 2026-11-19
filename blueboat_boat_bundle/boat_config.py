"""Write a u990 controller config for the real boat next to the live one; the live file is not changed.

    python3 boat_config.py [LIVE.yaml] [OUT.yaml]
    default: ~/.config/blueboat/alberto/controller.yaml -> ~/.config/blueboat/alberto/controller.u990.yaml

Keeps every setting of the live file and changes only:
  - the u990 model (packaged name blueboat_u990, policy_backend e274_6_5b);
  - the new telemetry-gap settings (the old telemetry_timeout_s 1.25 is refused by the new gap handling).
Mode stays shadow with actuators off; actuation is chosen on the launch command line.
"""
import sys
from pathlib import Path

import yaml

GAP = dict(telemetry_timeout_s=.25, telemetry_hold_s=.5, telemetry_resume_s=.5, telemetry_outage_s=5.,
           max_telemetry_pauses=5, telemetry_pause_window_s=60.)


def main(source='~/.config/blueboat/alberto/controller.yaml', destination='~/.config/blueboat/alberto/controller.u990.yaml'):
    source, destination = Path(source).expanduser(), Path(destination).expanduser()
    if destination == source:
        sys.exit('refusing to overwrite the live file')
    data = yaml.safe_load(source.read_text())
    p = data['blueboat_ppo_controller']['ros__parameters']
    wanted = dict(GAP, mode='shadow', enable_actuators=False, deployment_target='hardware',
                  policy_backend='e274_6_5b', model_directory='blueboat_u990')
    for key, value in wanted.items():
        if p.get(key) != value:
            print(f'{key}: {p.get(key)!r} -> {value!r}')
        p[key] = value
    if p.get('timestamp_future_tolerance_s', 0) != 0:
        sys.exit('timestamp_future_tolerance_s must be 0 on hardware; fix ' + str(source))
    destination.write_text(yaml.safe_dump(data, sort_keys=False))
    print('wrote ' + str(destination) + ' (live file unchanged)')


if __name__ == '__main__':
    if len(sys.argv) > 3:
        sys.exit(__doc__)
    main(*sys.argv[1:])
