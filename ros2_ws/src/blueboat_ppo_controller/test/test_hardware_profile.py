from pathlib import Path

import yaml

from blueboat_ppo_controller import __file__ as package_file


def test_hardware_profile_has_bounded_transport_margins_and_fast_stop_gates():
    profile = Path(package_file).parent / 'profiles' / 'alberto' / 'controller.yaml'
    settings = yaml.safe_load(profile.read_text())['blueboat_ppo_controller']['ros__parameters']

    assert settings['telemetry_timeout_s'] == 1.25
    assert settings['state_timeout_s'] == 3.0
    assert settings['parameter_timeout_s'] == 30.0
    assert settings['max_telemetry_skew_s'] == 0.25
    assert settings['timestamp_future_tolerance_s'] == 0.0

    assert settings['command_timeout_s'] == 0.5
    assert settings['enable_timeout_s'] == 0.5
    assert settings['rc_timeout_s'] == 0.5
    assert settings['watchdog_timeout_s'] == 0.1
