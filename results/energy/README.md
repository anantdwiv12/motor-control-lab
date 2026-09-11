# Motor energy accounting

All energies in joules. Frozen gains; 5 ms controller updates; no noise.
Table uses 0.5 ms plant steps, with five-second trajectories from rest.

| Scenario | Controller | Net electrical | Copper loss | Friction loss | Load work | Stored change | Residual |
|---|---|---:|---:|---:|---:|---:|---:|
| nominal | untuned | 384.30825 | 360.07520 | 0.34985 | 0.00000 | 23.88320 | -3.51e-06 |
| nominal | derived | 504.41466 | 478.94272 | 0.46697 | 0.00000 | 25.00499 | -1.51e-05 |
| nominal | optimized | 502.48073 | 477.01149 | 0.46440 | 0.00000 | 25.00486 | -2.3e-05 |
| load | untuned | 508.32168 | 471.50862 | 0.31309 | 0.06453 | 36.43544 | -2.91e-06 |
| load | derived | 672.41852 | 632.82286 | 0.45498 | 0.07340 | 39.06729 | -1.47e-05 |
| load | optimized | 681.34554 | 641.74235 | 0.46102 | 0.07451 | 39.06769 | -2.44e-05 |
| heavy_rotor | untuned | 464.72015 | 433.08803 | 0.32879 | 0.04014 | 31.26319 | -3.49e-06 |
| heavy_rotor | derived | 613.02473 | 579.44724 | 0.46260 | 0.04442 | 33.07048 | -1.76e-05 |
| heavy_rotor | optimized | 610.15098 | 576.57872 | 0.45789 | 0.04481 | 33.06959 | -2.72e-05 |
| hot_winding | untuned | 641.17792 | 601.20912 | 0.25403 | 0.08077 | 39.63400 | -1.51e-06 |
| hot_winding | derived | 932.66397 | 886.60749 | 0.43140 | 0.10076 | 45.52435 | -1.51e-05 |
| hot_winding | optimized | 963.51301 | 917.39824 | 0.44980 | 0.10312 | 45.56187 | -2.68e-05 |
| higher_friction | untuned | 686.76415 | 635.91924 | 0.36940 | 0.05041 | 50.42511 | -2.57e-06 |
| higher_friction | derived | 988.00476 | 931.11355 | 0.57778 | 0.05897 | 56.25447 | -1.55e-05 |
| higher_friction | optimized | 1007.00319 | 950.09773 | 0.59064 | 0.05967 | 56.25518 | -2.29e-05 |
| lower_speed | untuned | 309.30676 | 283.98221 | 0.11304 | 0.05897 | 25.15254 | -1.11e-06 |
| lower_speed | derived | 415.55515 | 387.73085 | 0.18605 | 0.07391 | 27.56435 | -6.97e-06 |
| lower_speed | optimized | 433.52162 | 405.68248 | 0.19734 | 0.07680 | 27.56502 | -2.35e-05 |

Maximum absolute residual across all 36 runs: 0.000109 J.
Every trajectory was repeated at 1 ms and 0.5 ms plant steps, keeping
controller timing fixed. Each refined residual is at most 30% of its
coarse counterpart (plus 1e-10 J roundoff tolerance).

Residual = electrical input − copper loss − friction loss − load work − stored change.
Input is signed integral(V i dt); negative values denote energy returned
through the ideal motor terminals, not proven battery recovery.
Stored energy is (L i² + J w²)/2. Losses are integrals of R i² and b w².
Load work is signed integral(load w dt). Input values are held from the
left endpoint while continuous states use trapezoidal quadrature.

Residuals measure numerical consistency of the idealized model, not
measurement accuracy. No inverter, battery, thermal or current-limit
model is included. Voltage effort (V²s) in prior reports is not energy.
All 36 per-run balances and parameters are reproducible from this script.
