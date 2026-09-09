# Derived baseline results

Nominal-model PI with damping ratio 0.8; gains fixed before evaluation.
All controllers use the same voltage saturation and anti-windup implementation.

| Scenario | Untuned IAE (rad) | Derived IAE (rad) | Optimized IAE (rad) |
|---|---:|---:|---:|
| nominal | 0.95844 | 0.26549 | 0.25540 |
| load | 1.17287 | 0.33143 | 0.28701 |
| heavy_rotor | 1.09100 | 0.36918 | 0.32008 |
| hot_winding | 1.56204 | 0.44624 | 0.33498 |
| higher_friction | 1.37526 | 0.38146 | 0.32149 |
| lower_speed | 0.96607 | 0.27809 | 0.18008 |

## Reused evaluation scenarios: average error and effort

| Condition | Controller | Mean IAE (rad) | Mean voltage effort (V²s) |
|---|---|---:|---:|
| original_5ms | untuned | 1.30112 | 673.17 |
| original_5ms | derived | 0.36860 | 981.51 |
| original_5ms | optimized | 0.27885 | 1051.17 |
| 50ms_noise_0.05 | untuned | 1.25904 | 686.25 |
| 50ms_noise_0.05 | derived | 0.37838 | 1007.80 |
| 50ms_noise_0.05 | optimized | 0.35566 | 1130.03 |

Original runs: 5 ms plant and controller steps, no measurement noise.
Stress runs: 1 ms plant steps, 50 ms controller updates, Gaussian speed-noise
SD 0.05 rad/s per sample, seeds 0–4. Values average equally over three scenarios
and (for stress) five seeds. The scenarios were reused from earlier evaluations.
Voltage effort is integral(V² dt), not energy. Individual metrics, including
overshoot and peak current, are in summary.json; nominal-condition trajectories
are in the CSV files. Stress trajectories can be regenerated from the recorded seeds.

This comparison adds one transparent engineering baseline, not a search over
all classical controllers. Continuous-time pole cancellation does not establish
robustness to parameter error, saturation, sampling, or hardware limitations.
