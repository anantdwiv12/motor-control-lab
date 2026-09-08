# Controller sampling and sensor-noise sensitivity

Frozen baseline and tuned gains from `../summary.json`; no retuning.

| Control period (ms) | Noise SD (rad/s) | Baseline mean IAE (rad) | Tuned mean IAE (rad) |
|---:|---:|---:|---:|
| 5 | 0 | 1.30113 | 0.27885 |
| 5 | 0.01 | 1.30108 | 0.28200 |
| 5 | 0.05 | 1.30088 | 0.30130 |
| 10 | 0 | 1.29891 | 0.28114 |
| 10 | 0.01 | 1.29897 | 0.28527 |
| 10 | 0.05 | 1.29920 | 0.30760 |
| 20 | 0 | 1.29450 | 0.28382 |
| 20 | 0.01 | 1.29381 | 0.28783 |
| 20 | 0.05 | 1.29108 | 0.31778 |
| 50 | 0 | 1.28155 | 0.29609 |
| 50 | 0.01 | 1.27704 | 0.30191 |
| 50 | 0.05 | 1.25904 | 0.35566 |

IAE is integrated absolute **true speed** error over five seconds.
Each cell averages equally over the three original held-out scenarios.
Noisy cells average five seeds per scenario; zero-noise cases run once.
The 264 per-run measurements are in `runs.csv`; scenario-level means
and observed min/max ranges are in `summary.json`. Ranges are not confidence intervals.

The plant uses 1 ms RK4 steps. Controller updates occur every 5, 10,
20 or 50 ms and hold voltage between updates. Independent Gaussian
measurement noise is applied only at control updates; it does not
directly change the physical state. Both controllers use the same
seed at each rate. Different rates sample different time sequences.

Noise variance is fixed per sample, not per unit bandwidth. This is
a synthetic sensor model, not a calibrated encoder/noise-spectrum model.
No filtering, quantization, sensor delay, current limits or hardware is modeled.

Twelve checks repeat the tuned controller with a 0.5 ms plant step:
three scenarios × two endpoint control periods × two endpoint noise levels
(seed 4 for noisy checks). Maximum speed difference across aligned samples:
**2.78e-11 rad/s**.
This checks numerical integration, not physical validity or formal stability.

Gains remain frozen from the earlier training experiment. The original
held-out scenarios are now a reused evaluation set; this sweep does not
constitute a new independent validation set. The baseline is still untuned.
