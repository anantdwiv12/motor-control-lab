"""Account for energy in all three frozen controllers, with grid refinement."""
import csv
import hashlib
import json
import math
import platform
from dataclasses import asdict
from pathlib import Path
from design import pole_cancellation_pi
from energy import energy_balance
from motorlab import TRAIN, HELD_OUT, simulate_sampled


def main():
    root = Path(__file__).parent
    source = root / 'results/summary.json'
    previous = json.loads(source.read_text())
    controllers = dict(untuned=previous['baseline_gains'], derived=pole_cancellation_pi(),
                       optimized=previous['tuned_gains'])
    output = root / 'results/energy'
    output.mkdir(exist_ok=True)
    runs = []
    for scenario in TRAIN+HELD_OUT:
        for label,gains in controllers.items():
            pair = []
            for dt in (0.001,0.0005):
                rows = simulate_sampled(gains,scenario,control_dt=0.005,plant_dt=dt)
                balance = energy_balance(scenario.motor,rows)
                if not all(math.isfinite(v) for v in balance.values()):
                    raise RuntimeError('Nonfinite energy result')
                if abs(balance['residual_J']) > 0.001:
                    raise RuntimeError('Energy residual exceeds 1 mJ')
                pair.append(balance)
                runs.append(dict(scenario=scenario.name,controller=label,plant_dt_s=dt,**balance))
            if abs(pair[1]['residual_J']) > 0.3*abs(pair[0]['residual_J'])+1e-10:
                raise RuntimeError('Residual did not decrease on refinement')
    with (output/'runs.csv').open('w',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(runs[0]),lineterminator='\n')
        writer.writeheader()
        writer.writerows(runs)
    report = dict(python=platform.python_version(),gain_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  gains=controllers,control_dt_s=0.005,duration_s=5,noise_std_rad_s=0,
                  scenarios=[asdict(s) for s in TRAIN+HELD_OUT],run_count=len(runs),runs=runs)
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    lines = ['# Motor energy accounting', '',
             'All energies in joules. Frozen gains; 5 ms controller updates; no noise.',
             'Table uses 0.5 ms plant steps, with five-second trajectories from rest.', '',
             '| Scenario | Controller | Net electrical | Copper loss | Friction loss | Load work | Stored change | Residual |',
             '|---|---|---:|---:|---:|---:|---:|---:|']
    for r in runs:
        if r['plant_dt_s']==0.0005:
            fields = [r[k] for k in ('electrical_net_J','copper_loss_J','friction_loss_J','load_work_J','stored_change_J')]
            lines.append('| '+r['scenario']+' | '+r['controller']+' | '+' | '.join(f'{v:.5f}' for v in fields)+f" | {r['residual_J']:.3g} |")
    lines += ['', f"Maximum absolute residual across all 36 runs: {max(abs(r['residual_J']) for r in runs):.3g} J.",
              'Every trajectory was repeated at 1 ms and 0.5 ms plant steps, keeping',
              'controller timing fixed. Each refined residual is at most 30% of its',
              'coarse counterpart (plus 1e-10 J roundoff tolerance).', '',
              'Residual = electrical input − copper loss − friction loss − load work − stored change.',
              'Input is signed integral(V i dt); negative values denote energy returned',
              'through the ideal motor terminals, not proven battery recovery.',
              'Stored energy is (L i² + J w²)/2. Losses are integrals of R i² and b w².',
              'Load work is signed integral(load w dt). Input values are held from the',
              'left endpoint while continuous states use trapezoidal quadrature.', '',
              'Residuals measure numerical consistency of the idealized model, not',
              'measurement accuracy. No inverter, battery, thermal or current-limit',
              'model is included. Voltage effort (V²s) in prior reports is not energy.',
              'All 36 per-run balances and parameters are reproducible from this script.', '']
    (output/'README.md').write_text('\n'.join(lines))
    print('\n'.join(lines))


if __name__=='__main__':
    main()
