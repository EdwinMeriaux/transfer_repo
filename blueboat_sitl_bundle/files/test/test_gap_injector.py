import yaml

from blueboat_ppo_controller import boat_profile
from blueboat_ppo_controller.gap_injector import gap_active, retarget_config


def test_schedule_cycles_through_gap_lengths():
    assert gap_active(5., 10., 8., [.25, 1.2]) is None
    assert gap_active(10.1, 10., 8., [.25, 1.2]) == 0
    assert gap_active(10.3, 10., 8., [.25, 1.2]) is None
    assert gap_active(19., 10., 8., [.25, 1.2]) == 1
    assert gap_active(19.3, 10., 8., [.25, 1.2]) is None


def test_configs_are_retargeted_and_logs_separated(tmp_path):
    for name, node, keys in (('thrust_calibration.yaml', 'blueboat_thrust_calibration',
                              ('raw_imu_topic', 'attitude_topic', 'velocity_topic', 'output_topic')),
                             ('controller.yaml', 'blueboat_ppo_controller', ('imu_topic', 'odometry_topic'))):
        source = tmp_path / name
        config = yaml.safe_load((boat_profile.BUNDLE / name).read_text())
        config[node]['ros__parameters']['log_directory'] = '/runs'
        source.write_text(yaml.safe_dump(config))
        retarget_config(source, tmp_path / ('gap-' + name))
        settings = yaml.safe_load((tmp_path / ('gap-' + name)).read_text())[node]['ros__parameters']
        assert all(settings[k].startswith('/blueboat/gap_test/') for k in keys)
        assert settings['log_directory'] == '/runs/gap-test'
