"""Evaluate frozen gains under controller-rate and measurement-noise changes."""
import csv
import hashlib
import json
import math
import platform
import statistics
from dataclasses import asdict
from pathlib import Path

from motorlab import HELD_OUT, metrics, simulate_sampled

PERIODS = (0.005, 0.01, 0.02, 0.05)
NOISE = (0.0, 0.01, 0.05)
SEEDS = tuple(range(5))


def main():
    root = Path(__file__).parent
    source = root / 'results/summary.json'
    original = json.loads(source.read_text())
    controllers = {'baseline': original['baseline_gains'], 'tuned': original['tuned_gains']}
    output = root / 'results/sensitivity'
    output.mkdir(exist_ok=True)
    runs, aggregates = [], []
    for scenario in HELD_OUT:
        for period in PERIODS:
            for noise in NOISE:
                for label, gains in controllers.items():
                    group = []
                    for seed in SEEDS if noise else (0,):
                        rows = simulate_sampled(gains, scenario, control_dt=period,
                                                noise_std=noise, seed=seed)
                        result = metrics(rows, scenario.target)
                        tail = [r[2] for r in rows if r[0] >= 4]
                        result['tail_speed_std_rad_s'] = statistics.pstdev(tail)
                        result.update(scenario=scenario.name, controller=label,
                                      control_dt_s=period, noise_std_rad_s=noise, seed=seed)
                        runs.append(result)
                        group.append(result['iae_rad'])
                    aggregates.append(dict(scenario=scenario.name, controller=label,
                                           control_dt_s=period, noise_std_rad_s=noise,
                                           n=len(group), iae_mean_rad=statistics.mean(group),
                                           iae_min_rad=min(group), iae_max_rad=max(group)))
    with (output / 'runs.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(runs[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(runs)
    # Same stochastic samples and controller timing; only plant integration is refined.
    refinement = []
    for scenario in HELD_OUT:
        for period in (PERIODS[0], PERIODS[-1]):
            for noise in (0.0, NOISE[-1]):
                coarse = simulate_sampled(controllers['tuned'], scenario, control_dt=period,
                                          noise_std=noise, seed=4)
                fine = simulate_sampled(controllers['tuned'], scenario, control_dt=period,
                                        plant_dt=0.0005, noise_std=noise, seed=4)
                refinement.append(dict(scenario=scenario.name, control_dt_s=period,
                                       noise_std_rad_s=noise,
                                       max_speed_difference_rad_s=max(abs(a[2]-b[2]) for a,b in zip(coarse, fine[::2]))))
    if not all(math.isfinite(v) for r in runs for v in r.values() if isinstance(v, float)):
        raise RuntimeError('Non-finite measurement in sweep')
    if max(r['max_speed_difference_rad_s'] for r in refinement) > 1e-7:
        raise RuntimeError('Plant integration refinement exceeds 1e-7 rad/s tolerance')
    report = dict(python=platform.python_version(), gain_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  gains=controllers, plant_dt_s=0.001, duration_s=5.0,
                  noise_model='Independent Gaussian speed error per control sample; fixed per-sample variance, not fixed spectral density',
                  seeds=SEEDS, scenarios=[asdict(s) for s in HELD_OUT],
                  run_count=len(runs), aggregates=aggregates, refinement=refinement)
    (output / 'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    table = ['# Controller sampling and sensor-noise sensitivity', '',
             'Frozen baseline and tuned gains from `../summary.json`; no retuning.', '',
             '| Control period (ms) | Noise SD (rad/s) | Baseline mean IAE (rad) | Tuned mean IAE (rad) |',
             '|---:|---:|---:|---:|']
    for period in PERIODS:
        for noise in NOISE:
            means = [statistics.mean(r['iae_mean_rad'] for r in aggregates
                                     if r['control_dt_s'] == period
                                     and r['noise_std_rad_s'] == noise and r['controller'] == c)
                     for c in controllers]
            table.append(f'| {period*1000:g} | {noise:g} | {means[0]:.5f} | {means[1]:.5f} |')
    table.extend(['', 'IAE is integrated absolute **true speed** error over five seconds.',
                  'Each cell averages equally over the three original held-out scenarios.',
                  'Noisy cells average five seeds per scenario; zero-noise cases run once.',
                  'The 264 per-run measurements are in `runs.csv`; scenario-level means',
                  'and observed min/max ranges are in `summary.json`. Ranges are not confidence intervals.', '',
                  'The plant uses 1 ms RK4 steps. Controller updates occur every 5, 10,',
                  '20 or 50 ms and hold voltage between updates. Independent Gaussian',
                  'measurement noise is applied only at control updates; it does not',
                  'directly change the physical state. Both controllers use the same',
                  'seed at each rate. Different rates sample different time sequences.', '',
                  'Noise variance is fixed per sample, not per unit bandwidth. This is',
                  'a synthetic sensor model, not a calibrated encoder/noise-spectrum model.',
                  'No filtering, quantization, sensor delay, current limits or hardware is modeled.', '',
                  'Twelve checks repeat the tuned controller with a 0.5 ms plant step:',
                  'three scenarios × two endpoint control periods × two endpoint noise levels',
                  '(seed 4 for noisy checks). Maximum speed difference across aligned samples:',
                  f"**{max(r['max_speed_difference_rad_s'] for r in refinement):.3g} rad/s**.",
                  'This checks numerical integration, not physical validity or formal stability.', '',
                  'Gains remain frozen from the earlier training experiment. The original',
                  'held-out scenarios are now a reused evaluation set; this sweep does not',
                  'constitute a new independent validation set. The baseline is still untuned.', ''])
    (output / 'README.md').write_text('\n'.join(table))
    print(f"Saved {len(runs)} runs and {len(refinement)} integration refinement checks")
    print(f"Largest refinement difference: {max(r['max_speed_difference_rad_s'] for r in refinement):.3g} rad/s")


if __name__ == '__main__':
    main()
