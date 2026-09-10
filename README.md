# Motor Control Lab

A reproducible experiment in motor physics, sampled control, and numerical
optimization. Compare a simple PI controller with gains selected by SciPy's
differential evolution, then evaluate on scenarios excluded from tuning.

**Status: first simulation milestone. No hardware validation.**

![Baseline and optimized motor responses](results/comparison.png)

## Run

Python 3.9–3.12. The recorded experiment used Python 3.9.6 on macOS arm64.

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest -v
python experiment.py
```

The physics and controller tests require only the Python standard library.
The experiment needs SciPy and Matplotlib. `requirements-lock.txt` records the
full environment used for the checked-in results. Results can vary slightly by
platform; a random seed does not promise identical results across all versions.

## Model and implementation

The state consists of armature current `i` (A) and shaft speed `w` (rad/s):

```text
L di/dt = V - R i - K w
J dw/dt = K i - b w - load
```

The nominal educational motor parameters are `R=1 ohm`, `L=0.5 H`,
`J=0.01 kg m²`, `b=0.1 N m s/rad`, and `K=0.01` in consistent SI units.
Positive load opposes positive rotation. These are a textbook example, not a
calibrated model of a commercial motor. The equations and example parameters
are referenced to [University of Michigan CTMS](https://ctms.engin.umich.edu/CTMS/?example=MotorSpeed&section=SystemModeling).
No CTMS source code or images are copied.

An original RK4 integrator advances the plant with held voltage/load. A discrete
PI controller updates every 5 ms with a ±24 V actuator limit and conditional
integration anti-windup. Experiments start at rest and run for five seconds;
nonzero load disturbances begin at two seconds. CSV rows record the state at
each timestamp and the command for the following interval; the final row repeats
the last interval's command and load for convenience.

## Optimization and evaluation

- Baseline: `Kp=10`, `Ki=10`, a simple untuned reference.
- Search: `Kp` in `[0,100]`, `Ki` in `[0,150]`; seed 7, population multiplier 6,
  at most 20 generations, no polishing, one worker.
- Objective: mean across training scenarios of normalized integrated absolute
  speed error + `0.1 × voltage effort / (24² × 5)` + `2 × normalized overshoot`.
  Error and overshoot reference scales are 1 rad and 1 rad/s respectively.
- Training: nominal motor, added load, and increased rotor inertia.
- Held out: increased winding resistance, increased friction, and a lower target.
  These scenarios are evaluated only after gain selection.

Recorded gains: `Kp=60.73455`, `Ki=136.87662`. The optimizer reported convergence
after 24 objective evaluations. This is its numerical stopping condition, not a
proof of global optimality or robustness.

| Scenario | Baseline integrated error (rad) | Tuned integrated error (rad) |
|---|---:|---:|
| Nominal (train) | 0.9584 | 0.2554 |
| Load (train) | 1.1729 | 0.2870 |
| Heavy rotor (train) | 1.0910 | 0.3201 |
| Hot winding (held out) | 1.5620 | 0.3350 |
| Higher friction (held out) | 1.3753 | 0.3215 |
| Lower speed (held out) | 0.9661 | 0.1801 |

The aggregate held-out objective falls from **1.3245 to 0.3582**, about 73%,
relative to this untuned reference. This is not a comparison against state-of-the-art
control. Faster regulation uses more voltage effort: in the hot-winding case,
`integral(V² dt)` increases from 907.3 to 1414.7 V²s. That metric is a control-effort
proxy, not electrical energy. Current peaks and saturation fractions are also
reported in [summary.json](results/summary.json), alongside every raw trajectory.

## Verification

Seventeen tests cover equilibrium with opposing load, open-loop convergence,
electromechanical energy balance, RK4 convergence order on a known RL response,
anti-windup, load rejection, sample-rate refinement, and invalid parameters.
They also check command holding between updates, sensor-noise reproducibility,
isolation of measurement noise from the true plant state, grid alignment, and
integration refinement with the recorded tuned gains and identical sensor samples.
The tests also pass on Python 3.12 locally.
The model-derived baseline adds polynomial-factorization and independent
analytic-response checks, plus validation of unsupported design inputs.
[GitHub Actions passed on Python 3.9 and 3.12](https://github.com/anantdwiv12/motor-control-lab/actions/runs/34193151055)
for the initial published implementation.

## Sampling and noise experiment

```sh
python sensitivity.py
```

This standard-library-only experiment freezes the recorded gains and evaluates
four controller periods (5–50 ms) and three sensor-noise levels across the original
three held-out scenarios. It separates controller timing from 1 ms plant integration.
The [264-run report](results/sensitivity/README.md) includes per-run CSV measurements,
five noise seeds, scenario-level ranges, and twelve plant-integration refinement checks.

Mean tuned integrated error across scenarios rises from 0.27885 rad at 5 ms without
noise to 0.35566 rad at 50 ms with 0.05 rad/s noise. It remains below the untuned
baseline in every aggregate table cell. These finite simulations establish neither
formal stability nor hardware reliability; this is a reused evaluation set.

## Model-derived controller baseline

```sh
python baseline_comparison.py
```

A [documented pole-cancellation design](DESIGN.md) selects PI gains from the
nominal motor and a fixed damping ratio of 0.8. The [63-run comparison](results/derived-baseline/README.md)
preserves the original untuned and optimized results. In the selected slower/noisy
condition, optimized mean error is only about 6% below this derived baseline,
at about 12% higher voltage effort. This provides a more informative comparison
than the original untuned reference; it is still a limited simulation study.

## Independent numerical reference

With the declared dependencies installed:

```sh
python -m unittest discover -s reference_tests -v
python validate_reference.py
```

Six additional SciPy-dependent tests complement the 17 standard-library tests.
The [12-comparison report](results/reference/README.md) compares RK4 with
`scipy.linalg.expm` across four open-loop input schedules and three step sizes.
The reference independently constructs an augmented linear system from the motor
parameters; it does not call `Motor.derivative` or RK4. The system is

```text
z = [i, w, V, load]
dz/dt = [[-R/L, -K/L, 1/L, 0],
         [ K/J, -b/J,   0, -1/J],
         [   0,    0,   0, 0],
         [   0,    0,   0, 0]] z
z(t+h) = exp(M h) z(t)
```

Inputs are held between events. Both methods split at event times even when they
fall between output samples. Tests check event timing by explicit composition,
as well as steady state, held-input composition, and RK4 refinement. At 5 ms,
maximum speed error across the four one-second cases is **1.17e-8 rad/s**, and
maximum current error is **9.68e-10 A**. CI runs both test suites and the reference
comparison on Python 3.9 and 3.12, with acceptance thresholds of 1e-7 rad/s and
1e-6 A at 5 ms.

This reference is exact for the stated held-input linear equations in mathematical
terms; its computed matrix exponential still has floating-point error. The two
methods share the event harness, whose boundary behavior is separately tested.
Existing closed-loop simulators and prior result files are unchanged. This is an
open-loop numerical check, not independent experimental validation of the motor.

## Limits and next experiments

The model omits brush friction, PWM switching, encoder noise, thermal dynamics,
current limiting, delays, and drive electronics. The sensitivity experiment adds
synthetic Gaussian measurement noise, but no calibrated encoder model.
There is no formal closed-loop
stability proof. The three fixed held-out scenarios are a small generalization
check. Rate/noise sensitivity has now been evaluated with frozen tuned gains;
it is not an independent new validation set. No gains here are recommended for hardware.

Next: explicit current constraints and energy accounting, multi-seed
optimization, and randomized parameter evaluation with uncertainty intervals.

## Open source and authorship

This project was built with Codex assistance as part of an engineering portfolio
started September 2026. The model implementation, anti-windup controller, tests,
scenario definitions, and evaluation pipeline are original project code.
SciPy supplies the optimizer; Matplotlib supplies the charts. See
[ATTRIBUTION.md](ATTRIBUTION.md) for upstream references and licensing.

## Explain it in five minutes

1. Derive the current equation from Kirchhoff's voltage law and the speed equation
   from torque balance. Explain why torque and back-EMF constants coincide in SI.
2. Show that electrical input power equals stored-energy change plus mechanical
   output and resistive/friction losses; point to the energy-balance test.
3. Explain why voltage saturation can make an integral controller accumulate error
   and how conditional integration addresses it.
4. Explain why the optimizer never sees the evaluation scenarios and why three
   scenarios still do not establish real-world reliability.
5. Discuss the observed error/effort tradeoff and propose a current-limited follow-up.
