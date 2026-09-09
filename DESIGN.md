# A model-derived PI baseline

This baseline uses only the nominal motor parameters and a chosen damping ratio
of **0.8**. This is a fixed engineering choice for a well-damped response, not an
optimum found from the evaluation data. It was not adjusted after the comparison.

## Derivation

From the current and torque equations in the README, the voltage-to-speed plant is

```text
P(s) = K / [L J s² + (L b + R J) s + R b + K²]
     = g / [(s + a)(s + c)]

g = K / (L J)
a + c = R/L + b/J
a c = (R b + K²)/(L J),  0 < a <= c
```

The standard plant model is referenced to
[University of Michigan CTMS](https://ctms.engin.umich.edu/CTMS/?example=MotorSpeed&section=SystemModeling).
The following design and implementation are derived here; no upstream controller
code or tutorial gains are copied.

Choose `C(s) = Kp + Ki/s` with `Ki = a Kp`. The controller zero cancels the
nominal stable slow pole in the reference-to-speed transfer function:

```text
C(s) P(s) = g Kp / [s(s+c)]
T(s) = g Kp / [s² + c s + g Kp]

Compare denominator with s² + 2 zeta wn s + wn²:
wn = c / (2 zeta)
Kp = c² / (4 zeta² g)
Ki = a Kp
```

For the nominal motor this gives `Kp=19.5214825428`, `Ki=39.0917840527`.
The implementation uses `a = (a*c)/c` to avoid subtraction of nearly equal
numbers when computing a slow pole. It rejects complex plant poles and invalid
damping ratios instead of silently applying an inapplicable cancellation.

The full characteristic polynomial is `(s+a)(s²+c*s+g*Kp)`; the canceled mode
is stable but still matters internally and under load disturbances. Cancellation
is exact only for the nominal continuous-time linear model. The simulations retain
the original plant, voltage saturation, sampled PI and conditional anti-windup;
they do not simulate the reduced transfer function in place of the actual plant.

## Comparison

Run `python baseline_comparison.py`. The script preserves existing artifacts,
loads the frozen original gains, and checks that all original metrics reproduce.
It runs 18 original-condition trajectories and 45 seeded stress runs, recording
gains, source hash, metrics and a readable [results table](results/derived-baseline/README.md).

Across the three reused evaluation scenarios at 5 ms without noise, mean IAE is
1.30112 rad (untuned), 0.36860 rad (derived), and 0.27885 rad (optimized).
Under the selected 50 ms/0.05 rad/s noise stress condition, the respective means
are 1.25904, 0.37838 and 0.35566 rad. The optimized controller has about 6% lower
mean error than the derived baseline in that stress condition, with about 12%
more voltage effort. The comparison is less dramatic than against untuned gains.

The three new tests check full polynomial factorization for several plants,
the nominal small-signal response against an independent analytic second-order
step response, and rejection of unsupported design inputs. Tests do not certify
hardware safety, robustness, global optimality or superiority to all classical designs.
