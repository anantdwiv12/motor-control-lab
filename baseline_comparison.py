"""Compare frozen controllers with an equation-derived PI baseline."""
import csv
import hashlib
import json
import math
import platform
import statistics
from pathlib import Path
from design import pole_cancellation_pi
from motorlab import TRAIN, HELD_OUT, simulate, simulate_sampled, metrics


def main():
    root = Path(__file__).parent
    source = root / 'results/summary.json'
    previous = json.loads(source.read_text())
    controllers = {'untuned': previous['baseline_gains'],
                   'derived': pole_cancellation_pi(damping_ratio=0.8),
                   'optimized': previous['tuned_gains']}
    output = root / 'results/derived-baseline'
    output.mkdir(exist_ok=True)
    runs = []
    for scenario in TRAIN + HELD_OUT:
        for label, gains in controllers.items():
            rows = simulate(gains, scenario)
            result = metrics(rows, scenario.target)
            if label != 'derived':
                old = previous['scenarios'][scenario.name]['baseline' if label == 'untuned' else 'tuned']
                if not all(math.isclose(result[k], old[k], abs_tol=1e-10, rel_tol=1e-10) for k in result):
                    raise RuntimeError('Previously published measurements no longer reproduce')
            runs.append(dict(scenario=scenario.name, mode='original_5ms', controller=label, seed=0, **result))
            with (output / f'{scenario.name}_{label}.csv').open('w', newline='') as stream:
                writer = csv.writer(stream, lineterminator='\n')
                writer.writerow(['time_s', 'current_A', 'speed_rad_s', 'voltage_V', 'load_Nm'])
                writer.writerows(rows)
    # One fixed stress configuration, chosen before evaluation: endpoint of the
    # previous sweep. Same random samples are used for all three controllers.
    for scenario in HELD_OUT:
        for label, gains in controllers.items():
            for seed in range(5):
                rows = simulate_sampled(gains, scenario, control_dt=0.05, plant_dt=0.001,
                                        noise_std=0.05, seed=seed)
                runs.append(dict(scenario=scenario.name, mode='50ms_noise_0.05', controller=label,
                                 seed=seed, **metrics(rows, scenario.target)))
    if not all(math.isfinite(v) for r in runs for v in r.values() if isinstance(v, float)):
        raise RuntimeError('Non-finite simulation metric')
    report = dict(python=platform.python_version(), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  damping_ratio=0.8, gains=controllers, duration_s=5, run_count=len(runs), runs=runs)
    (output / 'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    lines = ['# Derived baseline results', '',
             'Nominal-model PI with damping ratio 0.8; gains fixed before evaluation.',
             'All controllers use the same voltage saturation and anti-windup implementation.', '',
             '| Scenario | Untuned IAE (rad) | Derived IAE (rad) | Optimized IAE (rad) |',
             '|---|---:|---:|---:|']
    for scenario in TRAIN + HELD_OUT:
        vals = [next(r['iae_rad'] for r in runs if r['mode']=='original_5ms'
                     and r['scenario']==scenario.name and r['controller']==c) for c in controllers]
        lines.append(f'| {scenario.name} | {vals[0]:.5f} | {vals[1]:.5f} | {vals[2]:.5f} |')
    lines.extend(['', '## Reused evaluation scenarios: average error and effort', '',
                  '| Condition | Controller | Mean IAE (rad) | Mean voltage effort (V²s) |',
                  '|---|---|---:|---:|'])
    for mode in ('original_5ms', '50ms_noise_0.05'):
        for c in controllers:
            selected = [r for r in runs if r['mode']==mode and r['controller']==c
                        and r['scenario'] in {s.name for s in HELD_OUT}]
            lines.append(f"| {mode} | {c} | {statistics.mean(r['iae_rad'] for r in selected):.5f} | {statistics.mean(r['voltage_effort_V2s'] for r in selected):.2f} |")
    lines.extend(['', 'Original runs: 5 ms plant and controller steps, no measurement noise.',
                  'Stress runs: 1 ms plant steps, 50 ms controller updates, Gaussian speed-noise',
                  'SD 0.05 rad/s per sample, seeds 0–4. Values average equally over three scenarios',
                  'and (for stress) five seeds. The scenarios were reused from earlier evaluations.',
                  'Voltage effort is integral(V² dt), not energy. Individual metrics, including',
                  'overshoot and peak current, are in summary.json; nominal-condition trajectories',
                  'are in the CSV files. Stress trajectories can be regenerated from the recorded seeds.', '',
                  'This comparison adds one transparent engineering baseline, not a search over',
                  'all classical controllers. Continuous-time pole cancellation does not establish',
                  'robustness to parameter error, saturation, sampling, or hardware limitations.', ''])
    (output / 'README.md').write_text('\n'.join(lines))
    print(json.dumps(controllers, indent=2))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
