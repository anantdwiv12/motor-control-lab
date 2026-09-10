# RK4 versus independent matrix-exponential reference

| Case | Step (ms) | Max current error (A) | Max speed error (rad/s) |
|---|---:|---:|---:|
| constant | 20 | 9.74e-08 | 1.73e-06 |
| constant | 10 | 5.99e-09 | 9.96e-08 |
| constant | 5 | 3.71e-10 | 5.97e-09 |
| off_grid_load | 20 | 9.44e-08 | 2.73e-06 |
| off_grid_load | 10 | 5.84e-09 | 1.58e-07 |
| off_grid_load | 5 | 3.65e-10 | 9.59e-09 |
| hot_reversal | 20 | 2.53e-07 | 3.24e-06 |
| hot_reversal | 10 | 1.55e-08 | 1.89e-07 |
| hot_reversal | 5 | 9.68e-10 | 1.17e-08 |
| heavy_mixed | 20 | 9.97e-08 | 8.89e-07 |
| heavy_mixed | 10 | 6.31e-09 | 5.36e-08 |
| heavy_mixed | 5 | 4.03e-10 | 3.28e-09 |

Both methods split at each input event before continuing integration.
Errors are maxima on the output grid over one second, not global error bounds.
All cases start from zero current and speed. Inputs are held between events.
Parameters and schedules are in summary.json; paired trajectories are in CSVs.
The reference builds an augmented linear system directly from motor parameters
and uses scipy.linalg.expm, without calling the RK4 derivative implementation.
The matrix-exponential solution is exact for held inputs in mathematical terms;
the computed reference still has floating-point numerical error.

The harness supports off-grid events; existing closed-loop simulators retain
their documented timing behavior. This checks numerical implementation of the
same idealized motor equations, not hardware validity or controller robustness.
