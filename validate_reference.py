"""Reproduce RK4 vs matrix exponential comparisons without changing old results."""
import csv
import json
import math
import platform
from dataclasses import asdict
from pathlib import Path
import scipy
from motorlab import Motor
from reference import trajectory

CASES = (
    ('constant', Motor(), [(0, 12, 0)]),
    ('off_grid_load', Motor(), [(0, 12, 0), (0.137, 12, 0.03)]),
    ('hot_reversal', Motor(resistance=1.3), [(0, 12, 0), (0.333, -12, 0), (0.701, -6, -0.02)]),
    ('heavy_mixed', Motor(inertia=0.016), [(0, 24, 0), (0.137, 10, 0.025), (0.701, 0, 0)]),
)


def main():
    output = Path(__file__).parent / 'results/reference'
    output.mkdir(exist_ok=True)
    results = []
    for name, motor, events in CASES:
        for dt in (0.02, 0.01, 0.005):
            exact = trajectory(motor, events, dt=dt)
            numerical = trajectory(motor, events, dt=dt, method='rk4')
            current_error = max(abs(a[1]-b[1]) for a,b in zip(exact, numerical))
            speed_error = max(abs(a[2]-b[2]) for a,b in zip(exact, numerical))
            if not all(math.isfinite(x) for row in exact+numerical for x in row):
                raise RuntimeError('Nonfinite reference comparison')
            if dt == 0.005 and (current_error > 1e-6 or speed_error > 1e-7):
                raise RuntimeError('5 ms RK4 error exceeds acceptance tolerance')
            results.append(dict(case=name, dt_s=dt, max_current_error_A=current_error,
                                max_speed_error_rad_s=speed_error))
            with (output / f'{name}_{dt:g}s.csv').open('w', newline='') as stream:
                writer = csv.writer(stream, lineterminator='\n')
                writer.writerow(['time_s', 'reference_current_A', 'reference_speed_rad_s', 'rk4_current_A', 'rk4_speed_rad_s'])
                writer.writerows((*a, *b[1:]) for a,b in zip(exact, numerical))
    report = dict(python=platform.python_version(), scipy=scipy.__version__, duration_s=1,
                  cases=[dict(name=n, motor=asdict(m), events=e) for n,m,e in CASES], results=results)
    (output / 'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    lines = ['# RK4 versus independent matrix-exponential reference', '',
             '| Case | Step (ms) | Max current error (A) | Max speed error (rad/s) |',
             '|---|---:|---:|---:|']
    for r in results:
        lines.append(f"| {r['case']} | {r['dt_s']*1000:g} | {r['max_current_error_A']:.3g} | {r['max_speed_error_rad_s']:.3g} |")
    lines += ['', 'Both methods split at each input event before continuing integration.',
              'Errors are maxima on the output grid over one second, not global error bounds.',
              'All cases start from zero current and speed. Inputs are held between events.',
              'Parameters and schedules are in summary.json; paired trajectories are in CSVs.',
              'The reference builds an augmented linear system directly from motor parameters',
              'and uses scipy.linalg.expm, without calling the RK4 derivative implementation.',
              'The matrix-exponential solution is exact for held inputs in mathematical terms;',
              'the computed reference still has floating-point numerical error.', '',
              'The harness supports off-grid events; existing closed-loop simulators retain',
              'their documented timing behavior. This checks numerical implementation of the',
              'same idealized motor equations, not hardware validity or controller robustness.', '']
    (output / 'README.md').write_text('\n'.join(lines))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
