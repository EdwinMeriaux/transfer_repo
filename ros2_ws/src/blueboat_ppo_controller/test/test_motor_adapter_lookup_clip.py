import numpy as np
import pytest

from blueboat_ppo_controller.motor_adapter import MotorAdapter
from blueboat_ppo_controller.thrust_lookup import ThrustLookup


class RecordingLookup:
    def __init__(self):
        self.calls = []

    def inverse(self, force, side, out_of_range='reject'):
        self.calls.append((force, side, out_of_range))
        if out_of_range != 'clip':
            raise ValueError('neutral gap')
        return 1500, 0.0


def test_motor_adapter_safely_clips_lookup_gaps():
    adapter = MotorAdapter.__new__(MotorAdapter)
    adapter.minimum_force = -7.0
    adapter.maximum_force = 20.0
    adapter.lookup = RecordingLookup()
    adapter.curves = []

    pwm, applied = adapter.convert([0.5, -0.5])

    assert pwm == [1500, 1500]
    np.testing.assert_array_equal(applied, [0.0, 0.0])
    assert adapter.lookup.calls == [
        (0.5, 'left', 'clip'),
        (-0.5, 'right', 'clip'),
    ]


def test_lookup_clip_does_not_cross_flagged_unmeasured_pwm_intervals():
    lookup = ThrustLookup({
        'schema': 1,
        'kind': 'effective_surge_lookup',
        'independent_mapping_reviewed': True,
        'neutral_pwm': {'left': 1500, 'right': 1500},
        'zero_force_band_N': 0.05,
        'curves': {
            'left_forward': {
                'force_magnitude_N': [1.0, 2.0],
                'pwm': [1505, 1525],
                'unmeasured_pwm_gaps': [[1510, 1520]],
            },
        },
    })

    assert lookup.inverse(0.5, 'left', out_of_range='clip') == (1500, 0.0)
    assert lookup.inverse(3.0, 'left', out_of_range='clip') == (1525, 2.0)

    with pytest.raises(ValueError, match='unmeasured PWM interval'):
        lookup.inverse(1.5, 'left', out_of_range='clip')
