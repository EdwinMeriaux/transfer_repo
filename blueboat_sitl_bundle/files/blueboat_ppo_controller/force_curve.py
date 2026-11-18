"""Force-to-PWM curve for the PPO controller, built from a thrust check (no bollard pull).

    python3 -m blueboat_ppo_controller.force_curve build THRUST_CHECK.json [--voltage 15] [--max-force 15]
    python3 -m blueboat_ppo_controller.force_curve apply FORCE_CURVE.json [--interface motor_interface.json]

How a policy force F (N) becomes an RC input PWM, per motor:
  1. F / thrust scale (forward or reverse, from the thrust check) = T200 force needed.
     The policy's newtons are the training model's newtons; the scale makes the real boat
     respond to F the way the training model did.
  2. T200 published data at --voltage gives the ESC PWM for that force.
  3. The RC input that produces that ESC PWM comes from the thrust check's own measurements
     (RC input sent vs ESC PWM seen on /mavros/rc/out, at the end of each step). That map is
     a straight line per motor and direction, past ArduPilot's deadzone; it accounts for the
     trims, MOT_THST_ASYM and the reversed servo without modelling them.

`build` only writes a file. `apply` copies the curve and the deployment thrust cap into
motor_interface.json (keeping a backup); it changes nothing on the autopilot.
"""
import argparse
import csv
import hashlib
import json
import math
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from . import thrust_check as tc

SIDES = tc.SIDES
FORCES_N = (-15., -12.5, -10., -7.5, -5., -2.5, -1., 0., 1., 2.5, 5., 7.5, 10., 12.5, 15., 17.5, 20., 22.5, 25.)
POLICY_MIN_N, POLICY_MAX_N = -15., 25.        # range MotorAdapter requires the curve to cover
SETTLED_WINDOW_S = 3.                         # end of each step used for the input -> ESC map
MAX_INPUT_SPREAD = 6                          # us: RC input must have stopped moving
MAX_TARGET_ERROR = 6                          # us: ESC PWM must have reached the T200 target
MIN_LEVEL_SEPARATION = 10                     # us between two input levels to fit a slope
SLOPE_RANGE = (.5, 3.)                        # ESC us per RC input us
SCALE_RANGE = (.5, 1.5)
SLOPE_MISMATCH = .15


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _number(value):
    if value in (None, '', 'None'):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def read_rows(path):
    with Path(path).open(newline='') as handle:
        return [{k: (v if k == 'step' else _number(v)) for k, v in row.items()} for row in csv.DictReader(handle)]


def csv_for(result_path, result):
    """The thrust check's CSV: as recorded, else the sibling file with the same stamp."""
    recorded = Path(result.get('csv') or '')
    sibling = Path(result_path).with_suffix('.csv')
    for candidate in (recorded, sibling):
        if str(candidate) not in ('', '.') and candidate.is_file():
            return candidate
    raise FileNotFoundError(f'thrust check CSV not found (looked for {recorded} and {sibling})')


# ----------------------------------------------------------------------------- measured input -> ESC map
def step_points(results, rows, interface):
    """(side, direction) -> list of measured points, plus notes on skipped steps.

    Each point: input = RC input offset from neutral in the thrust direction (us),
    esc = ESC PWM offset from 1500 in the thrust direction (us), both medians over the
    last SETTLED_WINDOW_S of a completed step.
    """
    points, notes = {}, []
    completed = {r['step']: r for r in results if not r.get('stopped')}
    for name, result in completed.items():
        step_rows = [r for r in rows if r['step'] == name]
        if not step_rows:
            notes.append(f'{name}: no CSV rows')
            continue
        end = max(r['t'] for r in step_rows)
        window = [r for r in step_rows if r['t'] >= end - SETTLED_WINDOW_S]
        for i, side in enumerate(SIDES):
            force = result['left_N'] if i == 0 else result['right_N']
            if not force:
                continue
            direction = 1 if force > 0 else -1
            inputs = [r[f'input_{side}'] for r in window if r[f'input_{side}'] is not None]
            reached = [r[f'esc_offset_{side}'] for r in window if r[f'esc_offset_{side}'] is not None]
            targets = [r[f'esc_offset_target_{side}'] for r in window if r[f'esc_offset_target_{side}'] is not None]
            if not inputs or not reached or not targets:
                notes.append(f'{name} {side}: no ESC feedback in the last {SETTLED_WINDOW_S:.0f} s')
                continue
            if max(inputs) - min(inputs) > MAX_INPUT_SPREAD:
                notes.append(f'{name} {side}: RC input still moving ({min(inputs):.0f}-{max(inputs):.0f}); not used')
                continue
            esc, target = float(np.median(reached)), float(np.median(targets))
            if abs(esc - target) > MAX_TARGET_ERROR:
                notes.append(f'{name} {side}: ESC PWM {esc:.0f} us did not reach {target:.0f} us; not used')
                continue
            sign = interface.sign[i]
            offset = sign * direction * (float(np.median(inputs)) - interface.neutral[i])
            points.setdefault((side, direction), []).append(dict(step=name, input=offset, esc=esc))
    return points, notes


def fit_maps(points):
    """Straight line esc = a + b * input per (side, direction).

    A side with only one thrust level in a direction borrows the slope measured on the other
    side (identical RC/SERVO settings) and keeps its own offset.
    """
    maps, problems, notes = {}, [], []
    for key, pts in points.items():
        levels = sorted({round(p['input']) for p in pts})
        if len(levels) >= 2 and levels[-1] - levels[0] >= MIN_LEVEL_SEPARATION:
            x = np.array([p['input'] for p in pts])
            y = np.array([p['esc'] for p in pts])
            b, a = np.polyfit(x, y, 1)
            residual = float(np.max(np.abs(a + b * x - y)))
            maps[key] = dict(a=float(a), b=float(b), slope_from=key[0], points=pts, max_residual_us=residual)
    for direction, name in ((1, 'forward'), (-1, 'reverse')):
        own = [maps[(s, direction)]['b'] for s in SIDES if (s, direction) in maps]
        if len(own) == 2 and abs(own[0] - own[1]) > SLOPE_MISMATCH * max(own):
            notes.append(f'{name}: left and right input->ESC slopes differ ({own[0]:.2f} vs {own[1]:.2f})')
        for side in SIDES:
            key = (side, direction)
            if key in maps:
                continue
            other = (SIDES[1 - SIDES.index(side)], direction)
            if key in points and other in maps:
                b = maps[other]['b']
                a = float(np.mean([p['esc'] - b * p['input'] for p in points[key]]))
                maps[key] = dict(a=a, b=b, slope_from=other[0], points=points[key], max_residual_us=None)
            else:
                problems.append(f'{side} {name}: needs a measured level (and two levels on one side) '
                                f'- rerun the thrust check steps that use it')
    for (side, direction), m in maps.items():
        if not SLOPE_RANGE[0] <= m['b'] <= SLOPE_RANGE[1]:
            problems.append(f'{side} {"forward" if direction > 0 else "reverse"}: input->ESC slope {m["b"]:.2f} '
                            f'outside {SLOPE_RANGE}; check the thrust check CSV')
    return maps, problems, notes


# ----------------------------------------------------------------------------- curve
def build_curve(maps, interface, table, scales, max_force, min_force):
    """{side: [[force_N, rc_pwm], ...]} covering POLICY_MIN_N..POLICY_MAX_N, plus problems/notes."""
    curves, problems, notes = {}, [], []
    for i, side in enumerate(SIDES):
        lo, hi = interface.limits[i]
        lo, hi = max(lo, 1000), min(hi, 2000)
        rows = []
        for force in FORCES_N:
            if force == 0:
                rows.append([0., int(interface.neutral[i])])
                continue
            direction = 1 if force > 0 else -1
            scale = scales['forward' if direction > 0 else 'reverse']
            m = maps[(side, direction)]
            try:
                esc = table.offset(force / scale)
            except ValueError as error:
                problems.append(f'{side} {force:+g} N: {error}')
                continue
            offset = max(1., (esc - m['a']) / m['b'])
            pwm = interface.neutral[i] + interface.sign[i] * direction * offset
            used = min_force <= force <= max_force
            if not lo <= pwm <= hi:
                if used:
                    problems.append(f'{side} {force:+g} N needs RC {pwm:.0f} us, outside {lo}-{hi}')
                    continue
                notes.append(f'{side} {force:+g} N clamped to the RC limit (above the cap, never sent)')
                pwm = min(max(pwm, lo), hi)
            rows.append([force, int(round(pwm))])
        curves[side] = rows
    return curves, problems, notes


def curve_problem(rows, neutral):
    """MotorAdapter's acceptance rules for one side, as text (None if accepted)."""
    values = np.asarray(rows, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2 or len(values) < 3 or not np.isfinite(values).all():
        return 'invalid curve'
    if not np.all(np.diff(values[:, 0]) > 0):
        return 'forces not increasing'
    if not (np.all(np.diff(values[:, 1]) >= 0) or np.all(np.diff(values[:, 1]) <= 0)):
        return 'PWM not monotonic'
    if values[0, 0] > POLICY_MIN_N or values[-1, 0] < POLICY_MAX_N:
        return f'does not cover {POLICY_MIN_N:g}..{POLICY_MAX_N:g} N'
    if values[:, 1].min() < 1000 or values[:, 1].max() > 2000:
        return 'PWM outside 1000..2000'
    zero = values[values[:, 0] == 0]
    if len(zero) != 1 or zero[0, 1] != neutral:
        return 'neutral point missing'
    return None


def build(result_path, voltage=15., max_force=15., min_force=-15., interface_path=None, use_scale=True):
    """Returns (curve document, problems). Nothing is written."""
    result_path = Path(result_path).expanduser()
    result = json.loads(result_path.read_text())
    csv_path = csv_for(result_path, result)
    rows = read_rows(csv_path)
    interface = tc.load_interface(interface_path or result.get('interface'))
    problems = []
    if not 0 < max_force <= POLICY_MAX_N or not POLICY_MIN_N <= min_force < 0:
        problems.append(f'thrust cap must be {POLICY_MIN_N:g} <= min < 0 < max <= {POLICY_MAX_N:g}')
    summary = result.get('summary', {})
    scales = dict(forward=1., reverse=1.)
    if use_scale:
        for direction in scales:
            value = summary.get(direction)
            if value is None:
                problems.append(f'no {direction} thrust scale in the thrust check (a {direction} step did not complete)')
            elif not SCALE_RANGE[0] <= value <= SCALE_RANGE[1]:
                problems.append(f'{direction} thrust scale x{value:.2f} outside {SCALE_RANGE}: check the boat before using it')
            else:
                scales[direction] = float(value)
    points, notes = step_points(result.get('results', []), rows, interface)
    maps, map_problems, map_notes = fit_maps(points)
    problems += map_problems
    notes += map_notes
    curves = {}
    if not problems:
        table = tc.T200(voltage)
        curves, curve_problems, curve_notes = build_curve(maps, interface, table, scales, max_force, min_force)
        problems += curve_problems
        notes += curve_notes
        for i, side in enumerate(SIDES):
            if side in curves and not curve_problems:
                problem = curve_problem(curves[side], interface.neutral[i])
                if problem:
                    problems.append(f'{side} curve: {problem}')
    ratio = summary.get('left_right_speed_ratio')
    if ratio is not None and not .85 <= ratio <= 1.15:
        notes.append(f'left/right one-motor speed ratio {ratio:.2f}: one motor is weaker; the curve does not correct it')
    battery = [r['battery_v'] for r in rows if r.get('battery_v')]
    document = dict(
        schema=1,
        kind='force_pwm_points_from_thrust_check',
        created=datetime.now(timezone.utc).isoformat(timespec='seconds'),
        thrust_check=str(result_path), thrust_check_sha256=sha256(result_path),
        thrust_check_csv=str(csv_path), thrust_check_csv_sha256=sha256(csv_path),
        voltage_V=voltage,
        thrust_check_battery_V=[min(battery), max(battery)] if battery else None,
        thrust_scale=scales, scale_applied=use_scale,
        minimum_force_N=min_force, maximum_force_N=max_force,
        interface=dict(source=interface.source,
                       **{side: dict(rc_channel=interface.channels[i], neutral_pwm=interface.neutral[i],
                                     forward_pwm_sign=interface.sign[i]) for i, side in enumerate(SIDES)}),
        input_to_esc={f'{side}_{"forward" if d > 0 else "reverse"}': dict(
            esc_us_per_input_us=round(m['b'], 4), esc_us_at_zero_input=round(m['a'], 2),
            slope_from=m['slope_from'], max_residual_us=m['max_residual_us'],
            points=[dict(step=p['step'], input_us=round(p['input'], 1), esc_us=round(p['esc'], 1)) for p in m['points']])
            for (side, d), m in sorted(maps.items())},
        force_pwm_points=curves,
        notes=notes,
    )
    return document, problems


def apply(curve_path, interface_path):
    """Copy the curve and cap into motor_interface.json, with a backup. Returns the backup path."""
    curve_path, interface_path = Path(curve_path).expanduser(), Path(interface_path).expanduser()
    document = json.loads(curve_path.read_text())
    profile = json.loads(interface_path.read_text())
    if document.get('kind') != 'force_pwm_points_from_thrust_check':
        raise ValueError(f'{curve_path} is not a force curve from this tool')
    if profile.get('effective_lookup_file'):
        raise ValueError('motor_interface.json uses effective_lookup_file, which would override force_pwm_points; '
                         'remove it first')
    for i, side in enumerate(SIDES):
        built = document['interface'][side]
        for key in ('rc_channel', 'neutral_pwm', 'forward_pwm_sign'):
            current = profile[side].get(key) or (1 if key == 'forward_pwm_sign' else None)
            if current != built[key]:
                raise ValueError(f'{side} {key} is {current} in {interface_path} but the curve was built with {built[key]}')
        problem = curve_problem(document['force_pwm_points'][side], profile[side]['neutral_pwm'])
        if problem:
            raise ValueError(f'{side} curve: {problem}')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')
    backup = interface_path.with_name(interface_path.name + '.before-force-curve-' + stamp)
    shutil.copy2(interface_path, backup)
    for side in SIDES:
        profile[side]['force_pwm_points'] = document['force_pwm_points'][side]
    profile['minimum_force_N'] = document['minimum_force_N']
    profile['maximum_force_N'] = document['maximum_force_N']
    profile['force_curve_file'] = str(curve_path)
    profile['force_curve_sha256'] = sha256(curve_path)
    interface_path.write_text(json.dumps(profile, indent=2) + '\n')
    return backup


def print_curve(document):
    curves = document['force_pwm_points']
    print(f"T200 at {document['voltage_V']:g} V, thrust scale forward x{document['thrust_scale']['forward']:.2f} "
          f"reverse x{document['thrust_scale']['reverse']:.2f}; PPO cap "
          f"{document['minimum_force_N']:+g}..{document['maximum_force_N']:+g} N")
    print('   force N   left RC   right RC')
    for (force, left), (_, right) in zip(curves['left'], curves['right']):
        mark = '' if document['minimum_force_N'] <= force <= document['maximum_force_N'] else '   (above cap, not sent)'
        print(f'   {force:+7.1f}   {left:7d}   {right:8d}{mark}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    b = sub.add_parser('build', help='build a curve file from a thrust check result (writes one file)')
    b.add_argument('result', help='thrust-check-*.json')
    b.add_argument('--voltage', type=float, default=15., help='battery voltage the curve is built for (default 15.0)')
    b.add_argument('--max-force', type=float, default=15., help='PPO forward cap per motor, N (default 15)')
    b.add_argument('--min-force', type=float, default=-15., help='PPO reverse cap per motor, N (default -15)')
    b.add_argument('--interface', help='motor_interface.json (default: the one the thrust check used)')
    b.add_argument('--no-scale', action='store_true', help='ignore the thrust scale (T200 data only)')
    b.add_argument('--output', help='output file (default: force-curve-<stamp>.json next to the result)')
    a = sub.add_parser('apply', help='copy a curve into motor_interface.json (keeps a backup)')
    a.add_argument('curve', help='force-curve-*.json')
    a.add_argument('--interface', default=str(tc.DEFAULT_CONFIG / 'motor_interface.json'))
    args = parser.parse_args(argv)

    if args.command == 'apply':
        try:
            backup = apply(args.curve, args.interface)
        except (ValueError, KeyError, FileNotFoundError) as error:
            print('refused: ' + str(error))
            return 2
        print(f'force_pwm_points and thrust cap written to {args.interface} (backup {backup})')
        return 0

    try:
        document, problems = build(args.result, args.voltage, args.max_force, args.min_force,
                                   args.interface, not args.no_scale)
    except (ValueError, KeyError, FileNotFoundError) as error:
        print('refused: ' + str(error))
        return 2
    for note in document['notes']:
        print('note: ' + note)
    if problems:
        print('refused:\n  ' + '\n  '.join(problems))
        return 2
    print_curve(document)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')
    output = Path(args.output).expanduser() if args.output else Path(args.result).expanduser().with_name(f'force-curve-{stamp}.json')
    output.write_text(json.dumps(document, indent=2) + '\n')
    print('curve: ' + str(output))
    print('next: force_curve apply ' + str(output))
    return 0


if __name__ == '__main__':
    sys.exit(main())
