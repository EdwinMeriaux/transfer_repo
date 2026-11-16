"""Initial low-speed inertial response estimation, never static propeller thrust."""
import argparse
import copy
import csv
import hashlib
import json
import os
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class AnalysisSettings:
    mass_kg: float
    mass_std_kg: float = 0.
    added_mass_kg: float = 0.
    added_mass_std_kg: float = 0.
    initial_start_s: float = .08
    initial_end_s: float = .30
    min_baseline_s: float = .8
    min_recovery_s: float = .8
    max_source_age_s: float = .2
    max_skew_s: float = .05
    max_gap_s: float = .06
    max_baseline_speed_m_s: float = .08
    max_speed_m_s: float = .25
    max_angular_rate_rad_s: float = .15
    max_angular_accel_rad_s2: float = .6
    max_tilt_rad: float = .15
    max_baseline_std_m_s2: float = .12
    max_pulse_std_m_s2: float = .25
    max_transverse_accel_m_s2: float = .3
    max_velocity_disagreement_m_s2: float = .25
    max_bias_m_s2: float = .6
    require_output_feedback: bool = True
    output_response_threshold_pwm: float = 3.
    max_output_delay_s: float = .2
    zero_force_band_N: float = .05
    max_interpolation_gap_pwm: float = 30.
    accelerometer_semantics: str = 'specific_force'
    imu_offset_flu_m: tuple = (0., 0., 0.)

    def __post_init__(self):
        numeric = [v for k, v in asdict(self).items() if k not in ('accelerometer_semantics', 'imu_offset_flu_m')]
        if (not np.isfinite(numeric).all() or self.mass_kg <= 0
                or min(self.mass_std_kg, self.added_mass_kg, self.added_mass_std_kg) < 0
                or not 0 <= self.initial_start_s < self.initial_end_s
                or self.accelerometer_semantics not in ('specific_force', 'linear_acceleration')
                or np.asarray(self.imu_offset_flu_m).shape != (3,)
                or not np.isfinite(self.imu_offset_flu_m).all()
                or type(self.require_output_feedback) is not bool or not 0 <= self.zero_force_band_N <= 1
                or not 0 < self.output_response_threshold_pwm <= 50
                or any(v <= 0 for k, v in asdict(self).items() if k.startswith(('max_', 'min_')))):
            raise ValueError('Invalid analysis settings / mass / IMU semantics')


def settings_from_session(metadata=None, mass_kg=None, options=None):
    """Carry the ROS mass parameter into analysis unless explicitly overridden."""
    session = (metadata or {}).get('settings', {})
    mass = mass_kg if mass_kg is not None else session.get('mass_kg')
    if mass is None:
        raise ValueError('Supply measured mass_kg in the session metadata or --mass-kg')
    settings = dict(accelerometer_semantics=session.get('accelerometer_semantics', 'specific_force'))
    settings.update(options or {})
    settings['mass_kg'] = float(mass)
    return AnalysisSettings(**settings)


def read_csv(path):
    with open(path, newline='') as stream:
        return list(csv.DictReader(stream))


def vectors(rows, prefix, axes='xyz'):
    return np.array([[float(r[prefix+a]) for a in axes] for r in rows])


def corrected_acceleration(rows, settings):
    """Specific force + body gravity - tangential/centripetal lever acceleration."""
    t = np.array([float(r['imu_stamp_s']) for r in rows])
    q = vectors(rows, 'q', 'xyzw')
    if not np.isfinite(q).all() or np.any(abs(np.linalg.norm(q, axis=1)-1) > .01):
        raise ValueError('invalid_quaternion')
    rotation = Rotation.from_quat(q)
    a, w = vectors(rows, 'a'), vectors(rows, 'w')
    if len(t) < 3 or np.any(np.diff(t) <= 0):
        raise ValueError('nonmonotonic_source_time')
    alpha = np.gradient(w, t, axis=0)
    if settings.accelerometer_semantics == 'specific_force':
        a += rotation.inv().apply(np.tile([0., 0., -9.80665], (len(rows), 1)))
    offset = np.asarray(settings.imu_offset_flu_m)
    a -= np.cross(alpha, offset) + np.cross(w, np.cross(w, offset))
    return a, w, alpha, rotation


def estimate_trial(rows, settings):
    """Return an accepted/rejected trial with reasons; retain all rejected evidence."""
    result = dict(trial=int(rows[0]['trial']), side=rows[0]['side'], accepted=False)
    try:
        if any(r['valid'] not in ('1', 'True', True, 1) for r in rows):
            raise ValueError('invalid_telemetry_or_fault_in_trial')
        if any(r['mode'] not in ('powered', 'synthetic') for r in rows):
            raise ValueError('no_powered_trial_dry_run_or_telemetry_only')
        if len({(r['side'], r['delta_pwm']) for r in rows}) != 1:
            raise ValueError('inconsistent_trial_metadata')
        for field in ('imu_frame', 'attitude_frame', 'velocity_frame'):
            if len({r[field] for r in rows}) != 1 or not rows[0][field]:
                raise ValueError('invalid_or_changing_frame')
        if len({rows[0][f] for f in ('imu_frame', 'attitude_frame', 'velocity_frame')}) != 1:
            raise ValueError('frames_not_aligned_FLU')
        t = np.array([float(r['imu_stamp_s']) for r in rows])
        stamp_a = np.array([float(r['attitude_stamp_s']) for r in rows])
        stamp_v = np.array([float(r['velocity_stamp_s']) for r in rows])
        ros_now = np.array([float(r['ros_time_s']) for r in rows])
        numeric = np.column_stack([t, stamp_a, stamp_v, ros_now, vectors(rows, 'a'), vectors(rows, 'w'), vectors(rows, 'v')])
        if not np.isfinite(numeric).all():
            raise ValueError('nonfinite_measurement')
        ages = ros_now[:, None] - np.column_stack([t, stamp_a, stamp_v])
        if np.any((ages < 0) | (ages > settings.max_source_age_s)):
            raise ValueError('stale_or_future_source_timestamp')
        if max(np.max(abs(t-stamp_a)), np.max(abs(t-stamp_v))) > settings.max_skew_s:
            raise ValueError('timestamp_skew')
        if len(t) < 3 or np.any(np.diff(t) <= 0) or np.max(np.diff(t)) > settings.max_gap_s:
            raise ValueError('source_time_gap_or_reordering')
        baseline = np.array([r['phase'] == 'baseline' for r in rows])
        pulse = np.array([r['phase'] == 'pulse' for r in rows])
        recovery = np.array([r['phase'] == 'recovery' for r in rows])
        if (sum(baseline) < 20 or sum(pulse) < 6 or sum(recovery) < 2 or np.ptp(t[baseline]) < settings.min_baseline_s
                or np.ptp(t[recovery]) < settings.min_recovery_s
                or float(rows[-1]['receipt_monotonic_s'])+settings.max_gap_s < float(rows[-1]['trial_end_monotonic_s'])):
            raise ValueError('incomplete_trial_or_insufficient_samples')
        first = rows[int(np.flatnonzero(pulse)[0])]
        onset = float(first['ros_time_s']) - (float(first['receipt_monotonic_s']) - float(first['command_monotonic_s']))
        initial = pulse & (t-onset >= settings.initial_start_s) & (t-onset <= settings.initial_end_s)
        if sum(initial) < 6 or np.ptp(t[initial]) < .7*(settings.initial_end_s-settings.initial_start_s):
            raise ValueError('initial_window_not_covered')
        # Commands must actually have been issued; planned dry-run PWM cannot become a force curve.
        pwm = np.array([[float(r['command_left_pwm']), float(r['command_right_pwm'])] for r in rows])
        if not np.isfinite(pwm).all() or np.any((pwm < 1000) | (pwm > 2000)):
            raise ValueError('missing_or_invalid_command')
        neutral = np.median(pwm[baseline], axis=0)
        driven = np.median(pwm[pulse], axis=0)
        side = result['side']
        if side not in ('left', 'right', 'symmetric'):
            raise ValueError('unknown_trial_side')
        active = [0, 1] if side == 'symmetric' else [0 if side == 'left' else 1]
        for i in range(2):
            expected = driven[i] if i in active else neutral[i]
            if np.any(pwm[baseline, i] != neutral[i]) or np.any(pwm[pulse, i] != expected):
                raise ValueError('unstable_command_or_inactive_motor_not_neutral')
            if i in active and driven[i] == neutral[i]:
                raise ValueError('no_command_step')
        output_delay = None
        if settings.require_output_feedback:
            outputs = np.array([[float(r['output_left_pwm']), float(r['output_right_pwm'])] for r in rows])
            output_t = np.array([float(r['output_stamp_s']) for r in rows])
            if (not np.isfinite(outputs).all() or np.any((outputs < 800) | (outputs > 2200))
                    or not np.isfinite(output_t).all() or np.any((ros_now-output_t < 0) | (ros_now-output_t > settings.max_source_age_s))):
                raise ValueError('missing_stale_or_invalid_actual_output')
            servo_neutral = np.median(outputs[baseline], axis=0)
            for i in set(range(2))-set(active):
                if np.max(abs(outputs[pulse,i]-servo_neutral[i])) > 15:
                    raise ValueError('actual_idle_motor_not_neutral')
            responded = pulse & np.all(abs(outputs[:,active]-servo_neutral[active]) > settings.output_response_threshold_pwm, axis=1)
            if not np.any(responded):
                raise ValueError('actual_output_did_not_respond')
            output_onset = float(output_t[np.flatnonzero(responded)[0]])
            output_delay = max(0., output_onset-onset)
            if output_delay > settings.max_output_delay_s:
                raise ValueError('actual_output_delay_exceeded')
            onset = max(onset, output_onset)
            initial = pulse & (t-onset >= settings.initial_start_s) & (t-onset <= settings.initial_end_s)
            if sum(initial) < 6 or np.ptp(t[initial]) < .7*(settings.initial_end_s-settings.initial_start_s):
                raise ValueError('initial_window_after_output_not_covered')
        a, w, alpha, rotation = corrected_acceleration(rows, settings)
        v = vectors(rows, 'v')
        selected = baseline | initial
        if np.max(np.linalg.norm(v[baseline], axis=1)) > settings.max_baseline_speed_m_s:
            raise ValueError('baseline_not_at_rest')
        if np.max(np.linalg.norm(v[selected], axis=1)) > settings.max_speed_m_s:
            raise ValueError('speed_drag_limit')
        if np.max(np.linalg.norm(w[selected], axis=1)) > settings.max_angular_rate_rad_s or np.max(np.linalg.norm(alpha[selected], axis=1)) > settings.max_angular_accel_rad_s2:
            raise ValueError('yaw_or_rotation_disturbance')
        if np.max(abs(rotation.as_euler('xyz')[:, :2][selected])) > settings.max_tilt_rad:
            raise ValueError('roll_pitch_disturbance')
        bias = np.median(a[baseline], axis=0)
        if np.linalg.norm(bias) > settings.max_bias_m_s2:
            raise ValueError('gravity_semantics_or_bias_invalid')
        corrected = a-bias
        if np.max(np.std(a[baseline], axis=0)) > settings.max_baseline_std_m_s2:
            raise ValueError('disturbed_baseline')
        if np.std(corrected[initial, 0]) > settings.max_pulse_std_m_s2:
            raise ValueError('unsteady_initial_acceleration')
        if np.max(abs(np.mean(corrected[initial, 1:], axis=0))) > settings.max_transverse_accel_m_s2:
            raise ValueError('transverse_disturbance')
        # d(v_body)/dt + omega x v is inertial acceleration expressed in body axes.
        def velocity_acc(mask):
            indices = np.flatnonzero(mask)
            slope = np.polyfit(t[indices]-t[indices[0]], v[indices, 0], 1)[0]
            return slope + np.mean(np.cross(w[indices], v[indices])[:, 0])
        acceleration = float(np.mean(corrected[initial, 0]))
        velocity_delta = float(velocity_acc(initial)-velocity_acc(baseline))
        if abs(velocity_delta-acceleration) > settings.max_velocity_disagreement_m_s2:
            raise ValueError('velocity_imu_disagreement')
        direction = 1 if float(rows[0]['delta_pwm']) > 0 else -1
        if acceleration*direction <= 0:
            raise ValueError('wrong_sign_or_unresolved_force')
        # Use sample scatter, NOT standard error: adjacent IMU samples are correlated.
        sigma_a = float(np.hypot(np.std(a[baseline, 0], ddof=1), np.std(a[initial, 0], ddof=1)))
        inertial_mass = settings.mass_kg + settings.added_mass_kg
        force = inertial_mass * acceleration
        sigma_f = float(np.hypot(inertial_mass*sigma_a, acceleration*np.hypot(settings.mass_std_kg, settings.added_mass_std_kg)))
        if abs(force) <= max(settings.zero_force_band_N, 2*sigma_f):
            raise ValueError('force_not_resolved_above_noise')
        result.update(accepted=True, reason='', direction='forward' if direction > 0 else 'reverse',
                      delta_pwm=abs(int(rows[0]['delta_pwm'])), command_pwm=driven.astype(int).tolist(),
                      neutral_pwm=neutral.astype(int).tolist(), acceleration_m_s2=acceleration,
                      velocity_acceleration_m_s2=velocity_delta, effective_force_N=force,
                      sigma_force_N=sigma_f, baseline_bias_m_s2=bias.tolist(), samples=int(sum(initial)))
        result['actual_output_delay_s'] = output_delay
    except (ValueError, KeyError, IndexError, TypeError) as error:
        result['reason'] = str(error)
    return result


def isotonic(values, weights):
    """Weighted pool-adjacent-violators; monotonic without polynomial overshoot."""
    blocks = []
    for i, (value, weight) in enumerate(zip(values, weights)):
        blocks.append([i, i+1, float(value*weight), float(weight)])
        while len(blocks) > 1 and blocks[-2][2]/blocks[-2][3] > blocks[-1][2]/blocks[-1][3]:
            right, left = blocks.pop(), blocks.pop()
            blocks.append([left[0], right[1], left[2]+right[2], left[3]+right[3]])
    output = np.empty(len(values))
    for first, last, total, weight in blocks:
        output[first:last] = total/weight
    return output


def fit_lookup(trials, settings, neutral_pwm, min_repeats=3):
    curves, missing = {}, []
    for side in ('left', 'right', 'symmetric'):
        for direction in ('forward', 'reverse'):
            key = side+'_'+direction
            accepted = [r for r in trials if r['accepted'] and r['side'] == side and r['direction'] == direction]
            points = []
            for delta in sorted({r['delta_pwm'] for r in accepted}):
                group = [r for r in accepted if r['delta_pwm'] == delta]
                if len(group) < min_repeats:
                    continue
                forces = np.abs([r['effective_force_N'] for r in group])
                sigma = float(max(np.std(forces, ddof=1), np.sqrt(np.mean(np.square([r['sigma_force_N'] for r in group])))))
                index = 1 if side == 'right' else 0
                pwms = {r['command_pwm'][index] for r in group}
                if len(pwms) != 1 or any(r['neutral_pwm'] != [neutral_pwm[s] for s in ('left','right')] for r in group):
                    continue
                points.append((delta, float(np.mean(forces)), sigma, next(iter(pwms)), len(group)))
            if len(points) < 2:
                missing.append(key+': fewer than two PWM levels with sufficient repeat trials')
                continue
            delta, force, sigma, pwm, count = map(np.asarray, zip(*points))
            fitted = isotonic(force, 1/np.maximum(sigma, .01)**2)
            # A plateau is unresolved, and cannot be uniquely inverted. Preserve it in fit diagnostics.
            resolved = np.ones(len(fitted), dtype=bool)
            for i in range(len(fitted)-1):
                if fitted[i+1]-fitted[i] <= 1e-9:
                    resolved[i:i+2] = False
            resolved &= fitted > settings.zero_force_band_N
            unresolved = [{'pwm':int(p), 'reason':'isotonic_plateau_or_noise'} for p in pwm[~resolved]]
            if sum(resolved) < 2:
                missing.append(key+': force plateau / unresolved response')
                continue
            curves[key] = dict(pwm=pwm[resolved].astype(int).tolist(), force_magnitude_N=fitted[resolved].tolist(),
                sigma_force_N=np.hypot(sigma[resolved], force[resolved]-fitted[resolved]).tolist(),
                repeat_count=count[resolved].astype(int).tolist(), raw_mean_force_N=force[resolved].tolist(),
                unresolved_points=unresolved,
                unmeasured_pwm_gaps=[[int(a), int(b)] for a,b in zip(pwm[resolved][:-1], pwm[resolved][1:])
                    if abs(b-a) > settings.max_interpolation_gap_pwm or any(min(a,b) < p < max(a,b) for p in pwm[~resolved])],
                measured_force_domain_N=[float(fitted[resolved][0]), float(fitted[resolved][-1])],
                unmeasured_force_regions='Between zero-force band and first resolved point, above last point, and flagged PWM gaps',
                force_scope='total_boat_effective_surge' if side == 'symmetric' else 'isolated_motor_effective_surge',
                uncertainty='1 sigma trial scatter + supplied mass uncertainty; excludes unknown hydrodynamic/model bias')
    paired = []
    for direction in ('forward','reverse'):
        samples = {side:[r for r in trials if r['accepted'] and r['side']==side and r['direction']==direction]
                   for side in ('left','right','symmetric')}
        levels = set.intersection(*[{r['delta_pwm'] for r in group} for group in samples.values()])
        for delta in sorted(levels):
            groups = [[r for r in samples[side] if r['delta_pwm']==delta] for side in ('left','right','symmetric')]
            if min(map(len,groups)) < min_repeats:
                continue
            means = [float(np.mean([r['effective_force_N'] for r in group])) for group in groups]
            sigma = float(np.sqrt(sum(np.mean([r['sigma_force_N']**2 for r in group]) for group in groups)))
            residual = means[2]-means[0]-means[1]
            tolerance = max(2*sigma, .2*abs(means[0]+means[1]))
            paired.append(dict(direction=direction, delta_pwm=delta, residual_N=residual,
                               tolerance_N=tolerance, passed=abs(residual) <= tolerance))
    return dict(schema=1, kind='effective_surge_lookup', independent_mapping_reviewed=False,
                verified=False, neutral_pwm=neutral_pwm, zero_force_band_N=settings.zero_force_band_N,
                settings=asdict(settings), curves=curves, unmeasured=missing,
                paired_validation=dict(passed=len(paired)>=4 and all(p['passed'] for p in paired), comparisons=paired,
                    note='Necessary additivity check only; unequal/opposite-sign pairs and independent-motor identifiability still require review.'),
                assumptions=['Low speed relative to water; nearly level and negligible yaw.',
                    'Neutral has negligible thrust. Stable baseline removes local bias/background acceleration.',
                    'Mass times incremental acceleration is an effective surge response, not static propeller thrust.',
                    'Added mass, drag/current, mounting and actuator delay remain model uncertainty.',
                    'Single-motor results need isolation and paired-command review before independent PPO use.',
                    'Symmetric results are TOTAL force. They cannot be divided into independent motor forces.'])


def plot_results(rows, trials, lookup, directory, settings):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory = Path(directory)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    if rows:
        t = np.array([float(r['receipt_monotonic_s']) for r in rows])
        t -= t[0]
        for side in ('left', 'right'):
            pwm = [float(r['command_'+side+'_pwm']) if r['command_'+side+'_pwm'] else np.nan for r in rows]
            actual = [float(r['output_'+side+'_pwm']) if r['output_'+side+'_pwm'] else np.nan for r in rows]
            axes[0].plot(t, pwm, label=side+' command')
            axes[0].plot(t, actual, '--', alpha=.5, label=side+' output')
        axes[1].plot(t, [float(r['ax']) if r['ax'] else np.nan for r in rows], alpha=.4, label='raw body ax (specific force)')
        for trial in trials:
            if trial['accepted']:
                indices = [i for i,r in enumerate(rows) if int(r['trial']) == trial['trial'] and r['phase'] == 'pulse']
                axes[1].plot(t[indices], [trial['acceleration_m_s2']]*len(indices), label='initial baseline-corrected' if trial == next(x for x in trials if x['accepted']) else None)
                axes[2].plot(t[indices], [trial['effective_force_N']]*len(indices), label=trial['side'])
        axes[0].legend(loc='upper right', fontsize=8)
        axes[1].legend(fontsize=8)
    for ax, label in zip(axes, ('PWM (us)', 'Acceleration (m/s²)', 'Effective surge (N)')):
        ax.set_ylabel(label)
        ax.grid(alpha=.3)
    axes[-1].set_xlabel('Monotonic elapsed seconds')
    synthetic = bool(rows) and all(r['mode']=='synthetic' for r in rows)
    fig.suptitle(('SYNTHETIC TEST DATA — ' if synthetic else '')+'Initial inertial response; not static propeller thrust')
    fig.tight_layout()
    fig.savefig(directory/'timeseries.png', dpi=140)
    plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    for ax, side in zip(axes, ('left', 'right', 'symmetric')):
        for direction, sign in (('forward', 1), ('reverse', -1)):
            curve = lookup['curves'].get(side+'_'+direction)
            if curve:
                ax.errorbar(curve['pwm'], sign*np.array(curve['force_magnitude_N']),
                            yerr=curve['sigma_force_N'], fmt='o', label=direction)
                for i in range(len(curve['pwm'])-1):
                    a,b = curve['pwm'][i:i+2]
                    if [a,b] not in curve['unmeasured_pwm_gaps']:
                        ax.plot([a,b], sign*np.array(curve['force_magnitude_N'][i:i+2]), color='C0' if sign>0 else 'C1')
                for a,b in curve['unmeasured_pwm_gaps']:
                    ax.axvspan(min(a,b),max(a,b),color='grey',alpha=.15)
        ax.set_title(side+(' (total)' if side == 'symmetric' else ' (isolated)'))
        ax.set_xlabel('RC input PWM (us)')
        ax.set_ylabel('Effective surge (N)')
        ax.grid(alpha=.3)
        if ax.lines:
            ax.legend()
    fig.suptitle(('SYNTHETIC TEST DATA — ' if synthetic else '')+'Measured knots only; neutral gaps and outside coverage are unmeasured')
    fig.tight_layout()
    fig.savefig(directory/'lookup.png', dpi=140)
    plt.close(fig)


def analyze(path, directory, settings, neutral_pwm, metadata=None, make_plots=True):
    rows = read_csv(path)
    groups = {}
    for row in rows:
        if int(row['trial']) >= 0:
            groups.setdefault(row['trial'], []).append(row)
    trials = [estimate_trial(group, settings) for group in groups.values()]
    lookup = fit_lookup(trials, settings, neutral_pwm)
    lookup['evidence_sha256'] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    lookup['metadata'] = metadata or {}
    session_mass = lookup['metadata'].get('settings', {}).get('mass_kg')
    lookup['mass_provenance'] = dict(used_mass_kg=settings.mass_kg, session_mass_kg=session_mass,
        differs_from_session=session_mass is not None and not np.isclose(settings.mass_kg, session_mass))
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    (directory/'trials.json').write_text(json.dumps(trials, indent=2, allow_nan=False)+'\n')
    lookup_path = directory/'lookup.json'
    lookup_path.write_text(json.dumps(lookup, indent=2, allow_nan=False)+'\n')
    profile = lookup['metadata'].get('motor_interface')
    if profile:
        # Construct the actual PPO input artifact, but leave every acceptance gate closed.
        candidate = copy.deepcopy(profile)
        candidate.update(verified=False, effective_mapping_accepted=False, independent_effective_mapping_verified=False,
            evidence_file=os.path.relpath(Path(path).resolve(), directory.resolve()), evidence_sha256=lookup['evidence_sha256'],
            effective_lookup_file='lookup.json', effective_lookup_sha256=hashlib.sha256(lookup_path.read_bytes()).hexdigest(),
            maximum_abs_force_N=3.)
        for side in ('left','right'):
            candidate[side].pop('force_pwm_points', None)
        (directory/'ppo_calibration.draft.json').write_text(json.dumps(candidate, indent=2, allow_nan=False)+'\n')
    if make_plots:
        plot_results(rows, trials, lookup, directory, settings)
    return trials, lookup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mass-kg', type=float, help='Measured total mass; otherwise use the recorded ROS mass_kg parameter')
    parser.add_argument('--settings', type=Path, help='JSON analysis options; mass comes from the session or --mass-kg')
    parser.add_argument('--metadata', type=Path, help='Session JSON recorded by the calibration node')
    parser.add_argument('--left-neutral', type=int, required=True)
    parser.add_argument('--right-neutral', type=int, required=True)
    args = parser.parse_args()
    options = json.loads(args.settings.read_text()) if args.settings else {}
    metadata = json.loads(args.metadata.read_text()) if args.metadata else None
    try:
        settings = settings_from_session(metadata, args.mass_kg, options)
    except (ValueError, TypeError) as error:
        parser.error(str(error))
    trials, lookup = analyze(args.csv, args.output, settings,
                            dict(left=args.left_neutral, right=args.right_neutral), metadata)
    print(json.dumps(dict(accepted=sum(t['accepted'] for t in trials), rejected=sum(not t['accepted'] for t in trials),
                          fitted=list(lookup['curves']), missing_or_unresolved_branches=lookup['unmeasured'],
                          measured_force_magnitude_ranges_N={k:v['measured_force_domain_N'] for k,v in lookup['curves'].items()},
                          mass_provenance=lookup['mass_provenance'],
                          ppo_draft=str(args.output/'ppo_calibration.draft.json') if metadata and metadata.get('motor_interface') else None,
                          extrapolation='rejected; neutral gaps and flagged PWM intervals also rejected',
                          independent_mapping_reviewed=False), indent=2))
