"""Inverse measured effective-force curves. No extrapolation or training imports."""
import json
from pathlib import Path

import numpy as np


class ThrustLookup:
    def __init__(self, document):
        self.data = json.loads(Path(document).read_text()) if isinstance(document, (str, Path)) else document
        if self.data.get('schema') != 1 or self.data.get('kind') != 'effective_surge_lookup':
            raise ValueError('Not an effective surge lookup')
        self.curves = self.data['curves']
        self.neutral = self.data['neutral_pwm']
        self.zero_band = float(self.data['zero_force_band_N'])
        if not np.isfinite(self.zero_band) or not 0 <= self.zero_band <= 1:
            raise ValueError('Invalid zero-force band')
        for side in ('left', 'right'):
            if type(self.neutral[side]) is not int or not 1000 <= self.neutral[side] <= 2000:
                raise ValueError('Invalid neutral PWM')
        for key, curve in self.curves.items():
            if key not in {f'{s}_{d}' for s in ('left', 'right', 'symmetric') for d in ('forward', 'reverse')}:
                raise ValueError('Unknown curve')
            f, pwm = np.asarray(curve['force_magnitude_N']), np.asarray(curve['pwm'])
            if (len(f) < 2 or f.ndim != 1 or pwm.shape != f.shape or not np.isfinite(f).all()
                    or not np.isfinite(pwm).all() or np.any(f <= self.zero_band)
                    or np.any(np.diff(f) <= 0) or not (np.all(np.diff(pwm) > 0) or np.all(np.diff(pwm) < 0))
                    or np.any((pwm < 1000) | (pwm > 2000))):
                raise ValueError('Invalid monotonic measured curve')

    def inverse(self, force_N, side, out_of_range='reject', allow_unreviewed=False):
        """Return (integer PWM, applied effective N). Clip only if explicitly selected.

        A gap between neutral and the first resolved force is also unmeasured.
        No default interpolation across it. Zero-band commands use measured neutral.
        """
        if side not in ('left', 'right') or out_of_range not in ('reject', 'clip') or not np.isfinite(force_N):
            raise ValueError('Invalid inverse command')
        if self.data.get('independent_mapping_reviewed') is not True and not allow_unreviewed:
            raise ValueError('Independent effective motor mapping has not been reviewed')
        magnitude = abs(float(force_N))
        if magnitude <= self.zero_band:
            return self.neutral[side], 0.
        key = side + ('_forward' if force_N > 0 else '_reverse')
        if key not in self.curves:
            raise ValueError('Unmeasured direction: ' + key)
        curve = self.curves[key]
        f, pwm = np.asarray(curve['force_magnitude_N']), np.asarray(curve['pwm'])
        if any(min(a, b) < float(np.interp(magnitude, f, pwm)) < max(a, b)
               for a,b in curve.get('unmeasured_pwm_gaps', [])):
            raise ValueError('Command crosses an unmeasured PWM interval: ' + key)
        if not f[0] <= magnitude <= f[-1]:
            if out_of_range == 'reject':
                raise ValueError('Force is outside measured region (including neutral gap): ' + key)
            # For an unresolved small force, neutral is safer than rounding UP to the first pulse.
            if magnitude < f[0]:
                return self.neutral[side], 0.
            magnitude = float(f[-1])
        value = int(round(float(np.interp(magnitude, f, pwm))))
        return int(np.clip(value, 1000, 2000)), float(np.copysign(magnitude, force_N))
