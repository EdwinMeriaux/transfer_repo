import math

import numpy as np
import pytest

from blueboat_ppo_controller.mission_control import MissionController, wrap_angle, yaw_from_quaternion


def test_yaw_and_angle_helpers():
    assert yaw_from_quaternion([0, 0, 0, 1]) == pytest.approx(0)
    assert yaw_from_quaternion([0, 0, math.sin(math.pi/4), math.cos(math.pi/4)]) == pytest.approx(math.pi/2)
    assert wrap_angle(3*math.pi) == pytest.approx(-math.pi)
    with pytest.raises(ValueError):
        yaw_from_quaternion([0, 0, 0, 0])


def test_out_and_back_relative_to_start_heading():
    mission = MissionController('out-and-back', distance_m=20)
    start = np.array([7.0, -4.0])
    command = mission.update(start, math.pi/2)
    assert mission.stage == 'outbound'
    assert mission.target == pytest.approx([7, 16])
    assert command.surge_m_s > 0

    mission.update(np.array([7.0, 16.0]), math.pi/2)
    assert mission.stage == 'turn_180'
    for _ in range(5):
        mission.update(np.array([7.0, 16.0]), -math.pi/2)
    assert mission.stage == 'return'
    command = mission.update(start, -math.pi/2)
    assert command.complete
    assert mission.stage == 'complete'


def test_circle_center_and_geometry_are_relative():
    mission = MissionController('center-circle', center_distance_m=25, diameter_m=12)
    start = np.array([10.0, 20.0])
    mission.update(start, 0.0)
    assert mission.target == pytest.approx([35, 20])
    mission.update(np.array([35.0, 20.0]), 0.0)
    assert mission.center == pytest.approx([35, 20])
    assert mission.rim == pytest.approx([35, 26])
    assert mission.stage == 'to_rim'


def test_command_limits_and_bad_configuration():
    with pytest.raises(ValueError):
        MissionController('bad')
    with pytest.raises(ValueError):
        MissionController('out-and-back', speed_m_s=.81)
    mission = MissionController('out-and-back')
    command = mission.update([0, 0], math.pi)
    assert abs(command.surge_m_s) <= .8
    assert abs(command.yaw_rate_rad_s) <= .4
