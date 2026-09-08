"""Reproduce the baseline comparison, tuning, held-out evaluation and figure."""
import csv
import json
import platform
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import scipy
from scipy.optimize import differential_evolution
from motorlab import TRAIN, HELD_OUT, simulate, metrics, score


def main():
    output = Path(__file__).parent / "results"
    output.mkdir(exist_ok=True)
    baseline = (10.0, 10.0)  # deliberately simple, untuned reference, not best-in-class
    tuned = differential_evolution(score, [(0, 100), (0, 150)], seed=7,
                                   maxiter=20, popsize=6, polish=False, workers=1)
    gains = tuple(float(x) for x in tuned.x)
    report = {"python": platform.python_version(), "scipy": scipy.__version__,
              "seed": 7, "dt_s": 0.005, "duration_s": 5.0,
              "baseline_gains": baseline, "tuned_gains": gains,
              "optimizer_success": bool(tuned.success), "optimizer_message": str(tuned.message),
              "objective_evaluations": int(tuned.nfev),
              "training_score": {"baseline": score(baseline), "tuned": score(gains)},
              "held_out_score": {"baseline": score(baseline, HELD_OUT), "tuned": score(gains, HELD_OUT)},
              "scenarios": {}}
    fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True, constrained_layout=True)
    for scenario, ax in zip(TRAIN + HELD_OUT, axes.flat):
        report["scenarios"][scenario.name] = {"split": "train" if scenario in TRAIN else "held_out"}
        for label, params in (("baseline", baseline), ("tuned", gains)):
            rows = simulate(params, scenario)
            report["scenarios"][scenario.name][label] = metrics(rows, scenario.target)
            with (output / f"{scenario.name}_{label}.csv").open("w", newline="") as stream:
                writer = csv.writer(stream, lineterminator="\n")
                writer.writerow(["time_s", "current_A", "speed_rad_s", "voltage_V", "load_Nm"])
                writer.writerows(rows)
            ax.plot([r[0] for r in rows], [r[2] for r in rows], label=label, linewidth=1.8)
        ax.axhline(scenario.target, linestyle="--", color="gray", linewidth=0.8, label="target")
        ax.set_title(scenario.name.replace("_", " "))
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Speed (rad/s)")
        ax.grid(alpha=0.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("DC motor control · training scenarios (top), held-out scenarios (bottom)")
    fig.savefig(output / "comparison.png", dpi=170)
    plt.close(fig)
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k:v for k,v in report.items() if k != "scenarios"}, indent=2))


if __name__ == "__main__":
    main()
